#include "ZPWM.h"
#include "CodalDmesg.h"
#include "dma.h"

#define LOG DMESG

using namespace codal;

static ZPWM *instances[4];
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define HALF_BUFFER_SIZE 128

ZPWM::ZPWM(ZPin &pin, DataSource &source, int sampleRate, uint16_t id) : pin(pin), upstream(source)
{
    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] == NULL)
        {
            instances[i] = this;
            break;
        }
    }

    buf0 = new uint32_t[HALF_BUFFER_SIZE * 2];
    outptr = 0;

    // use pin api, so that the pin knows we're in PWM mode
    pin.setPWM(0, 20000);
    pwmout_t &pwm = *pin.pwmCfg;

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

    dma_init((uint32_t)tim.Instance, this->channel + DMA_TIM_CH1 - 1, &hdma_left,
             DMA_FLAG_CIRCULAR | DMA_FLAG_4BYTE | DMA_FLAG_PRI(1));
    __HAL_LINKDMA(&tim, hdma[this->channel], hdma_left);

    hdma_left.XferHalfCpltCallback = XferHalfCpltCallback;

    // Enable the PWM module
    enable();

    // Register with our upstream component
    upstream.connect(*this);
}

int ZPWM::setSleep(bool sleepMode)
{
    if (sleepMode)
    {
        __HAL_TIM_SET_COMPARE(&tim, channel, 0);
    }
    else
    {
        // DMA should resume it
    }

    return DEVICE_OK;
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

    CODAL_ASSERT(period_ticks >= 256, DEVICE_HARDWARE_CONFIGURATION_ERROR);
    CODAL_ASSERT(period_ticks <= 512,
                 DEVICE_HARDWARE_CONFIGURATION_ERROR); // in reality it should be 260 or so

    tim.Init.Period = period_ticks * prescaler;
    repCount = prescaler;

    // Update our internal record to reflect an accurate (probably rounded) samplerate.
    sampleRate = (clock_frequency / prescaler) / period_ticks;

    LOG("PWM presc=%d period=%d freq=%d->%d", prescaler, period_ticks, frequency, sampleRate);

    auto res = HAL_TIM_PWM_Init(&tim);
    CODAL_ASSERT(res == HAL_OK, DEVICE_HARDWARE_CONFIGURATION_ERROR);

    TIM_OC_InitTypeDef sConfig;
    memset(&sConfig, 0, sizeof(sConfig));
    sConfig.OCMode = TIM_OCMODE_PWM1;
    res = HAL_TIM_PWM_ConfigChannel(&tim, &sConfig, channels[this->channel - 1]);
    CODAL_ASSERT(res == HAL_OK, DEVICE_HARDWARE_CONFIGURATION_ERROR);

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
            instances[i]->irq(false);
        }
    }
}

void ZPWM::XferHalfCpltCallback(DMA_HandleTypeDef *hdma)
{
    // TODO use offsetof?
    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] && &instances[i]->hdma_left == hdma)
        {
            instances[i]->irq(true);
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
    {
        active = true;
        fillBuffer(buf0);
        fillBuffer(buf0 + HALF_BUFFER_SIZE);
        auto ch = channels[this->channel - 1];
        auto res = HAL_TIM_PWM_Start_DMA(&tim, ch, buf0, HALF_BUFFER_SIZE * 2);
        CODAL_ASSERT(res == HAL_OK, DEVICE_HARDWARE_CONFIGURATION_ERROR);
    }

    return DEVICE_OK;
}

void ZPWM::fillBuffer(uint32_t *buf)
{
    auto left = HALF_BUFFER_SIZE;

    while (left)
    {
        auto len = output.length() - outptr;

        if (len <= 0)
        {
            if (!active)
                break;
            dataReady--;
            output = upstream.pull();
            outptr = 0;
            len = output.length();
            if (len == 0)
                break;
        }

        auto src = (uint16_t *)&output[outptr];
        auto num = len >> 1;
        if (num > left)
            num = left;
        outptr += num << 1;

        for (uint32_t i = 0; i < num; ++i)
        {
            *buf++ = (src[i] * repCount) >> 2;
        }

        left -= num;
    }

    while (left)
    {
        *buf++ = 0;
        left--;
    }
}

/**
 * Base implementation of a DMA callback
 */
void ZPWM::irq(bool isHalf)
{
    // once the sequence has finished playing, load up the next buffer.
    if (isHalf)
        fillBuffer(buf0);
    else
        fillBuffer(buf0 + HALF_BUFFER_SIZE);
}

/**
 * Enable this component
 */
void ZPWM::enable()
{
    enabled = true;
}

/**
 * Disable this component
 */
void ZPWM::disable()
{
    enabled = false;
    __HAL_TIM_SET_COMPARE(&tim, channel, 0); // ???
    __HAL_TIM_DISABLE(&tim);
    // TODO disable DMA
}
