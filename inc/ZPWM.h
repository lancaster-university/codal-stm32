#ifndef ZPWM_H
#define ZPWM_H

#include "CodalConfig.h"
#include "codal-core/inc/types/Event.h"
#include "Timer.h"
#include "ZPin.h"
#include "DataStream.h"

#ifndef ZPWM_DEFAULT_FREQUENCY
#define ZPWM_DEFAULT_FREQUENCY 44100
#endif

namespace codal
{

class ZPWM : public CodalComponent, public DataSink
{
private:
    TIM_HandleTypeDef tim;
    DMA_HandleTypeDef hdma_left;

    bool enabled;
    bool active;
    int dataReady;
    int sampleRate;
    int channel;

    int repCount;
    uint32_t *buf0;
    uint32_t outptr;

    ZPin &pin;

    void fillBuffer(uint32_t *buf);
    static void XferHalfCpltCallback(DMA_HandleTypeDef *hdma);

public:
    // The stream component that is serving our data
    DataSource &upstream;
    ManagedBuffer output;

    /**
     * Constructor for an instance of a DAC.
     *
     * @param source The DataSource that will provide data.
     * @param sampleRate The frequency (in Hz) that data will be presented.
     * @param id The id to use for the message bus when transmitting events.
     */
    ZPWM(ZPin &pin, DataSource &source, int sampleRate = ZPWM_DEFAULT_FREQUENCY,
         uint16_t id = DEVICE_ID_SYSTEM_DAC);

    /**
     * Callback provided when data is ready.
     */
    virtual int pullRequest();

    /**
     * Determine the DAC playback sample rate to the given frequency.
     * @return the current sample rate.
     */
    int getSampleRate();

    /**
     * Determine the maximum unsigned value that can be loaded into the PWM data values, for the
     * current frequency configuration.
     */
    int getSampleRange();

    /**
     * Change the DAC playback sample rate to the given frequency.
     * @param frequency The new sample playback frequency.
     */
    int setSampleRate(int frequency);

    /**
     * Interrupt callback when playback of DMA buffer has completed
     */
    void irq(bool isHalf);

    /**
     * Enable this component
     */
    void enable();

    /**
     * Disable this component
     */
    void disable();

    friend void ::HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

    /**
     * Puts the DAC in (or out of) sleep mode.
     */
    virtual int setSleep(bool sleepMode);
};

} // namespace codal

#endif
