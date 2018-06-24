#include "ZPWM.h"
#include "CodalDmesg.h"
#include "dma.h"

#define LOG DMESG

static ZPWM *instances[4];
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

ZPWM::ZPWM(Pin &pin, DataSource &source, int sampleRate, uint16_t id) : upstream(source)
{
    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] == NULL)
        {
            instances[i] = this;
            break;
        }
    }

    pin.setDigitalValue(0);

    pwmout_t pwm;
    pwmout_init(&pwm, pin.name);

    memset(&tim, 0, sizeof(tim));
    tim.Instance = (TIM_TypeDef *)pwm.pwm;

    this->channel = pwm.channel;

    // initialise state
    this->id = id;
    this->dataReady = 0;
    this->active = false;

    // Ensure PWM is currently disabled.
    disable();

    // Configure hardware for requested sample rate.
    setSampleRate(sampleRate);

    dma_init((uint32_t)tim.Instance, this->channel + DMA_TIM_CH1 - 1, &hdma_left, DMA_FLAG_4BYTE);
    __HAL_LINKDMA(&tim, hdma[this->channel], hdma_left);

    // Enable the PWM module
    enable();

    // Register with our upstream component
    upstream.connect(*this);
}

/**
 * Determine the DAC playback sample rate to the given frequency.
 * @return the current sample rate.
 */
int ZPWM::getSampleRate()
{
    return sampleRate;
}

/**
 * Determine the maximum unsigned vlaue that can be loaded into the PWM data values, for the current
 * frequency configuration.
 */
int ZPWM::getSampleRange()
{
    return tim.Init.Period - 1;
}

static const uint32_t channels[] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4,
};

/**
 * Change the DAC playback sample rate to the given frequency.
 * @param frequency The new sample playback frequency.
 */
int ZPWM::setSampleRate(int frequency)
{
    int clock_frequency = 2 * HAL_RCC_GetPCLK1Freq();
    int cyclesPerSample = clock_frequency / frequency;

    int prescaler = cyclesPerSample / 256;
    int period_ticks = clock_frequency / (prescaler * frequency);

    CODAL_ASSERT(period_ticks >= 256);
    CODAL_ASSERT(period_ticks <= 512); // in reality it should be 260 or so

    // tim.Init.Prescaler = prescaler - 1;
    tim.Init.Period = period_ticks;
    // tim.Init.RepetitionCounter = prescaler - 1; // TIM1 or TIM8 only

    repCount = prescaler;

    // Update our internal record to reflect an accurate (probably rounded) samplerate.
    sampleRate = (clock_frequency / prescaler) / period_ticks;

    LOG("PWM presc=%d period=%d freq=%d->%d", prescaler, period_ticks, frequency, sampleRate);

    auto res = HAL_TIM_PWM_Init(&tim);
    CODAL_ASSERT(res == HAL_OK);

    TIM_OC_InitTypeDef sConfig;
    memset(&sConfig, 0, sizeof(sConfig));
    sConfig.OCMode = TIM_OCMODE_PWM1;
    res = HAL_TIM_PWM_ConfigChannel(&tim, &sConfig, channels[this->channel - 1]);
    CODAL_ASSERT(res == HAL_OK);

    LOG("PWM inited");

    return DEVICE_OK;
}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    // TODO use offsetof?
    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] && &instances[i]->tim == htim)
        {
            instances[i]->irq();
        }
    }
}

/**
 * Callback provided when data is ready.
 */
int ZPWM::pullRequest()
{
    dataReady++;

    if (!active)
        pull();

    return DEVICE_OK;
}

/**
 * Pull down a buffer from upstream, and schedule a DMA transfer from it.
 */
int ZPWM::pull()
{
    output = upstream.pull();
    dataReady--;

#if 1
    static uint32_t *buf;
    auto len = output.length() / 2;
    delete buf;
    buf = new uint32_t[len * repCount];
    auto tmp = (uint16_t *)&output[0];
    auto dst = 0;
    for (int i = 0; i < len; ++i)
    {
        auto n = repCount;
        while (n--)
            buf[dst++] = tmp[i] >> 2;
    }
    len *= repCount;
#endif

    auto res = HAL_TIM_PWM_Start_DMA(&tim, channels[this->channel - 1], buf, len);
    CODAL_ASSERT(res == HAL_OK);

    active = true;

    return DEVICE_OK;
}

/**
 * Base implementation of a DMA callback
 */
void ZPWM::irq()
{
    // once the sequence has finished playing, load up the next buffer.
    if (dataReady)
        pull();
    else
        active = false;
}

/**
 * Enable this component
 */
void ZPWM::enable()
{
    enabled = true;
    //    PWM.ENABLE = 1;
}

/**
 * Disable this component
 */
void ZPWM::disable()
{
    enabled = false;
    __HAL_TIM_DISABLE(&tim);
}
