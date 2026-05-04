#include "Arduino.h"
#include "HardwareTimer.h"
#include "DShot.h"

/*
 * DShot for STM32F103 BluePill using Timer PWM + DMA.
 *
 * Mirrors the iNav / Betaflight approach:
 *   - A hardware timer runs at 12 MHz (DSHOT600), 6 MHz (DSHOT300),
 *     or 3 MHz (DSHOT150) with ARR = 19 (period = 20 ticks = 1 bit).
 *   - The timer's Update event fires a DMA request each bit period.
 *   - DMA writes a new CCR value from the frame buffer to the timer's
 *     capture/compare register, setting the duty cycle for that bit:
 *       bit-0 → CCR = 7  (35 % duty)
 *       bit-1 → CCR = 14 (70 % duty)
 *   - The last two buffer entries are 0 (output stays LOW = inter-frame gap).
 *   - A 500 Hz HardwareTimer ISR restarts the DMA every 2 ms.
 *
 * Supported pins / timers / DMA channels (STM32F103):
 *   PA0  TIM2_CH1  TIM2_UP → DMA1_Channel2
 *   PA6  TIM3_CH1  TIM3_UP → DMA1_Channel3
 *   PB6  TIM4_CH1  TIM4_UP → DMA1_Channel7
 */

/* ---- iNav DShot constants (timer ticks) ---- */
#define DSHOT_BIT_0       7
#define DSHOT_BIT_1       14
#define DSHOT_BITLENGTH   20   // ticks per bit period
#define DSHOT_DMA_SIZE    18   // 16 data bits + 2 reset ticks

#define DSHOT_MAX_MOTORS  3

/* ---- pin → timer → DMA mapping ---- */
struct DshotPinMap {
    uint8_t              arduinoPin;
    GPIO_TypeDef        *gpio;
    uint8_t              gpioPin;       // 0–15
    TIM_TypeDef         *tim;
    volatile uint32_t   *ccr;           // pointer to TIMx->CCR1
    DMA_Channel_TypeDef *dma;           // DMA channel for TIMx_UP
};

static const DshotPinMap DSHOT_PIN_MAP[] = {
    { PA0, GPIOA, 0, TIM2, &TIM2->CCR1, DMA1_Channel2 },  // TIM2_UP → CH2
    { PA6, GPIOA, 6, TIM3, &TIM3->CCR1, DMA1_Channel3 },  // TIM3_UP → CH3
    { PB6, GPIOB, 6, TIM4, &TIM4->CCR1, DMA1_Channel7 },  // TIM4_UP → CH7
};
#define DSHOT_PIN_MAP_SIZE (sizeof(DSHOT_PIN_MAP) / sizeof(DSHOT_PIN_MAP[0]))

/* ---- per-motor state ---- */
struct MotorState {
    const DshotPinMap *pin;
    // pendingBuffer is written by setThrottle() (main thread).
    // dmaBuffer is copied from pendingBuffer by the frame ISR just before DMA
    // starts, so DMA always reads a consistent, fully-written frame.
    uint32_t pendingBuffer[DSHOT_DMA_SIZE];
    uint32_t dmaBuffer[DSHOT_DMA_SIZE];
    bool     active;
};

static MotorState      motors[DSHOT_MAX_MOTORS];
static uint8_t         motorCount  = 0;
static HardwareTimer  *frameTimer  = nullptr;
static DShot::Mode     dshotMode   = DShot::Mode::DSHOT600;

/* ---- helpers ---- */

// Configure a GPIO pin as alternate-function push-pull output at 50 MHz.
// STM32F103 uses CRL (pins 0-7) / CRH (pins 8-15): MODE=11, CNF=10 → 0xB.
static void gpioConfigAF(GPIO_TypeDef *gpio, uint8_t pin) {
    if (gpio == GPIOA) RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    else if (gpio == GPIOB) RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    else if (gpio == GPIOC) RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    __IO uint32_t *cr = (pin < 8) ? &gpio->CRL : &gpio->CRH;
    uint8_t shift = (pin % 8) * 4;
    *cr = (*cr & ~(0xFUL << shift)) | (0xBUL << shift);
}

static void enableTimerClock(TIM_TypeDef *tim) {
    if      (tim == TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    else if (tim == TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    else if (tim == TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
}

// Copy pendingBuffer → dmaBuffer, then (re)start the DMA channel.
// Called from the frame ISR with DMA disabled, so the copy is safe.
static void startMotorDMA(MotorState *m) {
    DMA_Channel_TypeDef *dma = m->pin->dma;
    dma->CCR &= ~DMA_CCR_EN;                        // must disable before writing CNDTR
    memcpy(m->dmaBuffer, m->pendingBuffer, DSHOT_DMA_SIZE * sizeof(uint32_t));
    dma->CMAR = (uint32_t)m->dmaBuffer;
    dma->CNDTR = DSHOT_DMA_SIZE;
    dma->CCR  |= DMA_CCR_EN;                        // re-enable; first transfer on next Update
}

// 500 Hz ISR: send one DShot frame on every active motor.
static void frameISR() {
    for (int i = 0; i < motorCount; i++) {
        if (motors[i].active)
            startMotorDMA(&motors[i]);
    }
}

static void initFrameTimer() {
    frameTimer = new HardwareTimer(TIM1);
    frameTimer->setOverflow(500, HERTZ_FORMAT);
    frameTimer->attachInterrupt(frameISR);
    frameTimer->resume();
}

/* ---- packet helper (identical logic to original AVR library) ---- */
static uint16_t createPacket(uint16_t throttle) {
    uint8_t csum = 0;
    throttle <<= 1;
    if (throttle < 48 && throttle > 0)
        throttle |= 1;
    uint16_t csum_data = throttle;
    for (int i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0xf;
    return (throttle << 4) | csum;
}

/****************** DShot class ******************/

DShot::DShot(const enum Mode mode) {
    dshotMode = mode;
}

/*
 * attach() — configure a pin for DShot output.
 *
 * Sets up:
 *   1. GPIO in AF push-pull mode
 *   2. Timer for PWM CH1 output (12/6/3 MHz clock, period = 20 ticks)
 *      with Update DMA request enabled
 *   3. DMA channel: memory → CCR1, 32-bit transfers, one-shot
 *   4. 500 Hz frame timer (TIM1) if not already running
 */
void DShot::attach(uint8_t pin) {
    const DshotPinMap *p = nullptr;
    for (size_t i = 0; i < DSHOT_PIN_MAP_SIZE; i++) {
        if (DSHOT_PIN_MAP[i].arduinoPin == pin) {
            p = &DSHOT_PIN_MAP[i];
            break;
        }
    }
    if (!p || motorCount >= DSHOT_MAX_MOTORS) return;

    _motorIdx = motorCount++;
    MotorState *m = &motors[_motorIdx];
    m->pin    = p;
    m->active = true;
    memset(m->pendingBuffer, 0, sizeof(m->pendingBuffer));
    memset(m->dmaBuffer,     0, sizeof(m->dmaBuffer));

    /* 1. GPIO */
    gpioConfigAF(p->gpio, p->gpioPin);

    /* 2. Timer */
    enableTimerClock(p->tim);

    // Prescaler: divide SystemCoreClock down to the DShot base frequency.
    // DSHOT600 → 12 MHz, DSHOT300 → 6 MHz, DSHOT150 → 3 MHz.
    uint32_t baseHz;
    switch (dshotMode) {
        case DShot::Mode::DSHOT600: baseHz = 12000000UL; break;
        case DShot::Mode::DSHOT300: baseHz =  6000000UL; break;
        default:                    baseHz =  3000000UL; break;
    }
    uint32_t psc = (SystemCoreClock / baseHz) - 1;

    TIM_TypeDef *tim = p->tim;
    tim->CR1   = 0;
    tim->PSC   = psc;
    tim->ARR   = DSHOT_BITLENGTH - 1;       // 19 → period = 20 ticks
    tim->CCR1  = 0;
    // PWM mode 1 on CH1, preload DISABLED (DMA write takes effect immediately)
    tim->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos);
    tim->CCER  = TIM_CCER_CC1E;             // CH1 output, active high
    tim->DIER  = TIM_DIER_UDE;              // Update event → DMA request
    tim->EGR   = TIM_EGR_UG;               // load PSC/ARR now
    tim->SR    = 0;                         // clear the update flag set by UG
    tim->CR1   = TIM_CR1_CEN;              // start

    /* 3. DMA */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    DMA_Channel_TypeDef *dma = p->dma;
    dma->CCR = 0;                           // disable while configuring
    dma->CPAR  = (uint32_t)p->ccr;         // peripheral: TIMx->CCR1
    dma->CMAR  = (uint32_t)m->dmaBuffer;   // memory: frame buffer
    dma->CNDTR = DSHOT_DMA_SIZE;
    dma->CCR   = DMA_CCR_MSIZE_0   |       // 32-bit memory transfers
                 DMA_CCR_PSIZE_0   |       // 32-bit peripheral transfers
                 DMA_CCR_MINC      |       // increment memory address
                 DMA_CCR_DIR;              // memory → peripheral (one-shot, no circ)
    // DMA enable deferred to first frameISR call.

    /* 4. Frame timer */
    if (!frameTimer)
        initFrameTimer();
}

/*
 * setThrottle() — update the DShot frame for this motor.
 *
 * Writes to pendingBuffer under noInterrupts() so the frame ISR always
 * copies a complete, consistent frame into the DMA buffer.
 *
 * throttle: 0 = disarm, 48–2047 = throttle range.
 */
uint16_t DShot::setThrottle(uint16_t throttle) {
    _throttle = throttle;
    _packet   = createPacket(throttle);

    MotorState *m = &motors[_motorIdx];
    uint16_t mask = 0x8000;

    noInterrupts();
    for (int i = 0; i < 16; i++) {
        m->pendingBuffer[i] = (_packet & mask) ? DSHOT_BIT_1 : DSHOT_BIT_0;
        mask >>= 1;
    }
    m->pendingBuffer[16] = 0;   // inter-frame gap (output LOW)
    m->pendingBuffer[17] = 0;
    interrupts();

    return _packet;
}
