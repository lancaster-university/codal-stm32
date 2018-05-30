/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

/**
 * Class definition for ZPin.
 *
 * Commonly represents an I/O pin on the edge connector.
 */
#include "ZPin.h"
#include "Button.h"
#include "Timer.h"
#include "codal_target_hal.h"
#include "codal-core/inc/types/Event.h"
#include "pwm.h"
#include "board_pinmux.h"

#define IO_STATUS_CAN_READ                                                                         \
    (IO_STATUS_DIGITAL_IN | IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE)

#if CONFIG_SOC_FAMILY_STM32
#define PORTPINS 16
static const char *portNames[] = {
    "GPIOA", "GPIOB", "GPIOC", "GPIOD", "GPIOE", "GPIOF", "GPIOG", "GPIOH",
};
#define NUM_PORTS (sizeof(portNames) / sizeof(portNames[0]))
#else
#error "SOC family for pin not defined"
#endif

#define PINMASK (PORTPINS - 1)

namespace codal
{

struct ZPwmConfig
{
    struct device *pwm;
    u32_t period;
    u32_t pulse;
    uint8_t channel;
};

struct ZEventConfig
{
    struct gpio_callback callback;
    CODAL_TIMESTAMP prevPulse;
    ZPin *parent;
};

inline int map(codal::PullMode pinMode)
{
    switch (pinMode)
    {
    case PullMode::Up:
        return GPIO_PUD_PULL_UP;
    case PullMode::Down:
        return GPIO_PUD_PULL_DOWN;
    case PullMode::None:
        return GPIO_PUD_NORMAL;
    }

    return GPIO_PUD_NORMAL;
}

/**
 * Constructor.
 * Create a ZPin instance, generally used to represent a pin on the edge connector.
 *
 * @param id the unique EventModel id of this component.
 *
 * @param name the mbed PinName for this ZPin instance.
 *
 * @param capability the capabilities this ZPin instance should have.
 *                   (PIN_CAPABILITY_DIGITAL, PIN_CAPABILITY_ANALOG, PIN_CAPABILITY_AD,
 * PIN_CAPABILITY_ALL)
 *
 * @code
 * ZPin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_ALL);
 * @endcode
 */
ZPin::ZPin(int id, PinNumber name, PinCapability capability) : codal::Pin(id, name, capability)
{
    this->pullMode = DEVICE_DEFAULT_PULLMODE;

    // Power up in a disconnected, low power state.
    // If we're unused, this is how it will stay...
    this->status = 0x00;
    unsigned portNo = (unsigned)name / PORTPINS;
    CODAL_ASSERT(portNo < NUM_PORTS);
    this->port = device_get_binding(portNames[portNo]);
    CODAL_ASSERT(this->port);

    this->pwmCfg = NULL;
}

void ZPin::config(int status)
{
    int flags = 0;
    int pin = this->name & PINMASK;

    if (this->status & IO_STATUS_ANALOG_OUT)
    {
        delete this->pwmCfg;
        this->pwmCfg = NULL;
    }

    if (this->status & (IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE))
    {
        CODAL_ASSERT(this->evCfg->parent == this);
        gpio_pin_disable_callback(this->port, pin);
        gpio_remove_callback(this->port, &this->evCfg->callback);
        delete this->evCfg;
        this->evCfg = NULL;
    }

    if (this->status & IO_STATUS_TOUCH_IN)
    {
        delete this->btn;
        this->btn = NULL;
    }

    this->status = status;

    if (status & IO_STATUS_DIGITAL_OUT)
        flags |= GPIO_DIR_OUT;
    else if (status & IO_STATUS_DIGITAL_IN)
        flags |= GPIO_DIR_IN;

    if (status & IO_STATUS_CAN_READ)
        flags |= map(this->pullMode);

    gpio_pin_configure(this->port, pin, flags);
}

/**
 * Configures this IO pin as a digital output (if necessary) and sets the pin to 'value'.
 *
 * @param value 0 (LO) or 1 (HI)
 *
 * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if value is out of range, or
 * DEVICE_NOT_SUPPORTED if the given pin does not have digital capability.
 *
 * @code
 * ZPin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
 * P0.setDigitalValue(1); // P0 is now HI
 * @endcode
 */
int ZPin::setDigitalValue(int value)
{
    // Check if this pin has a digital mode...
    if (!(PIN_CAPABILITY_DIGITAL & capability))
        return DEVICE_NOT_SUPPORTED;

    // Ensure we have a valid value.
    if (value < 0 || value > 1)
        return DEVICE_INVALID_PARAMETER;

    // Move into a Digital input state if necessary.
    if (!(status & IO_STATUS_DIGITAL_OUT))
    {
        config(IO_STATUS_DIGITAL_OUT);
    }

    return gpio_pin_write(port, name & PINMASK, value);
}

/**
 * Configures this IO pin as a digital input (if necessary) and tests its current value.
 *
 *
 * @return 1 if this input is high, 0 if input is LO, or DEVICE_NOT_SUPPORTED
 *         if the given pin does not have digital capability.
 *
 * @code
 * ZPin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
 * P0.getDigitalValue(); // P0 is either 0 or 1;
 * @endcode
 */
int ZPin::getDigitalValue()
{
    // check if this pin has a digital mode...
    if (!(PIN_CAPABILITY_DIGITAL & capability))
        return DEVICE_NOT_SUPPORTED;

    // Move into a Digital input state if necessary.
    if (!(status & IO_STATUS_CAN_READ))
    {
        config(IO_STATUS_DIGITAL_IN);
    }

    u32_t v = 0;
    gpio_pin_read(port, name & PINMASK, &v);

    return v;
}

/**
 * Configures this IO pin as a digital input with the specified internal pull-up/pull-down
 * configuraiton (if necessary) and tests its current value.
 *
 * @param pull one of the mbed pull configurations: PullUp, PullDown, PullNone
 *
 * @return 1 if this input is high, 0 if input is LO, or DEVICE_NOT_SUPPORTED
 *         if the given pin does not have digital capability.
 *
 * @code
 * ZPin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
 * P0.getDigitalValue(PullUp); // P0 is either 0 or 1;
 * @endcode
 */
int ZPin::getDigitalValue(PullMode pull)
{
    setPull(pull);
    return getDigitalValue();
}

int ZPin::obtainAnalogChannel()
{
    // Move into an analogue output state if necessary
    if (!(status & IO_STATUS_ANALOG_OUT))
    {
        this->config(0);
        auto cfg = this->pwmCfg = new ZPwmConfig;
        cfg->period = DEVICE_DEFAULT_PWM_PERIOD; // 20ms
        cfg->pulse = 0;                          // 0%
        int chan;
        if (pinmux_setup_pwm(this->name, &cfg->pwm, &chan))
            return DEVICE_INVALID_PARAMETER;
        if (chan < 0)
        {
            chan = -chan;
            // TODO what to do about inverted channels?
        }
        cfg->channel = chan;
        status = IO_STATUS_ANALOG_OUT;
    }

    return DEVICE_OK;
}

int ZPin::setPWM(u32_t value, u32_t period)
{
    // check if this pin has an analogue mode...
    if (!(PIN_CAPABILITY_DIGITAL & capability))
        return DEVICE_NOT_SUPPORTED;

    // sanitise the level value
    if (value > period)
        value = period;

    if (obtainAnalogChannel())
        return DEVICE_INVALID_PARAMETER;

    auto cfg = this->pwmCfg;

    cfg->pulse = value;
    cfg->period = period;

    return pwm_pin_set_usec(cfg->pwm, cfg->channel, cfg->period, cfg->pulse);
}

/**
 * Configures this IO pin as an analog/pwm output, and change the output value to the given level.
 *
 * @param value the level to set on the output pin, in the range 0 - 1024
 *
 * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if value is out of range, or
 * DEVICE_NOT_SUPPORTED if the given pin does not have analog capability.
 */
int ZPin::setAnalogValue(int value)
{
    // sanitise the level value
    if (value < 0 || value > DEVICE_PIN_MAX_OUTPUT)
        return DEVICE_INVALID_PARAMETER;

    return setPWM((u64_t)value * this->pwmCfg->period / DEVICE_PIN_MAX_OUTPUT,
                  this->pwmCfg->period);
}

/**
 * Configures this IO pin as an analog/pwm output (if necessary) and configures the period to be
 * 20ms, with a duty cycle between 500 us and 2500 us.
 *
 * A value of 180 sets the duty cycle to be 2500us, and a value of 0 sets the duty cycle to be 500us
 * by default.
 *
 * This range can be modified to fine tune, and also tolerate different servos.
 *
 * @param value the level to set on the output pin, in the range 0 - 180.
 *
 * @param range which gives the span of possible values the i.e. the lower and upper bounds (center
 * +/- range/2). Defaults to DEVICE_PIN_DEFAULT_SERVO_RANGE.
 *
 * @param center the center point from which to calculate the lower and upper bounds. Defaults to
 * DEVICE_PIN_DEFAULT_SERVO_CENTER
 *
 * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if value is out of range, or
 * DEVICE_NOT_SUPPORTED if the given pin does not have analog capability.
 */
int ZPin::setServoValue(int value, int range, int center)
{
    // check if this pin has an analogue mode...
    if (!(PIN_CAPABILITY_ANALOG & capability))
        return DEVICE_NOT_SUPPORTED;

    // sanitise the servo level
    if (value < 0 || range < 1 || center < 1)
        return DEVICE_INVALID_PARAMETER;

    // clip - just in case
    if (value > DEVICE_PIN_MAX_SERVO_RANGE)
        value = DEVICE_PIN_MAX_SERVO_RANGE;

    // calculate the lower bound based on the midpoint
    int lower = (center - (range / 2)) * 1000;

    value = value * 1000;

    // add the percentage of the range based on the value between 0 and 180
    int scaled = lower + (range * (value / DEVICE_PIN_MAX_SERVO_RANGE));

    return setServoPulseUs(scaled / 1000);
}

/**
 * Configures this IO pin as an analogue input (if necessary), and samples the ZPin for its analog
 * value.
 *
 * @return the current analogue level on the pin, in the range 0 - 1024, or
 *         DEVICE_NOT_SUPPORTED if the given pin does not have analog capability.
 *
 * @code
 * ZPin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
 * P0.getAnalogValue(); // P0 is a value in the range of 0 - 1024
 * @endcode
 */
int ZPin::getAnalogValue()
{
    // check if this pin has an analogue mode...
    //    if (!(PIN_CAPABILITY_ANALOG & capability))
    return DEVICE_NOT_SUPPORTED;
}

/**
 * Determines if this IO pin is currently configured as an input.
 *
 * @return 1 if pin is an analog or digital input, 0 otherwise.
 */
int ZPin::isInput()
{
    return (status & (IO_STATUS_DIGITAL_IN | IO_STATUS_ANALOG_IN)) == 0 ? 0 : 1;
}

/**
 * Determines if this IO pin is currently configured as an output.
 *
 * @return 1 if pin is an analog or digital output, 0 otherwise.
 */
int ZPin::isOutput()
{
    return (status & (IO_STATUS_DIGITAL_OUT | IO_STATUS_ANALOG_OUT)) == 0 ? 0 : 1;
}

/**
 * Determines if this IO pin is currently configured for digital use.
 *
 * @return 1 if pin is digital, 0 otherwise.
 */
int ZPin::isDigital()
{
    return (status & (IO_STATUS_DIGITAL_IN | IO_STATUS_DIGITAL_OUT)) == 0 ? 0 : 1;
}

/**
 * Determines if this IO pin is currently configured for analog use.
 *
 * @return 1 if pin is analog, 0 otherwise.
 */
int ZPin::isAnalog()
{
    return (status & (IO_STATUS_ANALOG_IN | IO_STATUS_ANALOG_OUT)) == 0 ? 0 : 1;
}

/**
 * Configures this IO pin as a "makey makey" style touch sensor (if necessary)
 * and tests its current debounced state.
 *
 * Users can also subscribe to Button events generated from this pin.
 *
 * @return 1 if pin is touched, 0 if not, or DEVICE_NOT_SUPPORTED if this pin does not support touch
 * capability.
 *
 * @code
 * DeviceMessageBus bus;
 *
 * ZPin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_ALL);
 * if(P0.isTouched())
 * {
 *     //do something!
 * }
 *
 * // subscribe to events generated by this pin!
 * bus.listen(DEVICE_ID_IO_P0, DEVICE_BUTTON_EVT_CLICK, someFunction);
 * @endcode
 */
int ZPin::isTouched()
{
    // check if this pin has a touch mode...
    if (!(PIN_CAPABILITY_DIGITAL & capability))
        return DEVICE_NOT_SUPPORTED;

    // Move into a touch input state if necessary.
    if (!(status & IO_STATUS_TOUCH_IN))
    {
        config(0);
        this->btn = new Button(*this, id);
        status |= IO_STATUS_TOUCH_IN;
    }

    return this->btn->isPressed();
}

/**
 * Configures this IO pin as an analog/pwm output if it isn't already, configures the period to be
 * 20ms, and sets the pulse width, based on the value it is given.
 *
 * @param pulseWidth the desired pulse width in microseconds.
 *
 * @return DEVICE_OK on success, DEVICE_INVALID_PARAMETER if value is out of range, or
 * DEVICE_NOT_SUPPORTED if the given pin does not have analog capability.
 */
int ZPin::setServoPulseUs(int pulseWidth)
{
    // check if this pin has an analogue mode...
    if (!(PIN_CAPABILITY_ANALOG & capability))
        return DEVICE_NOT_SUPPORTED;

    // sanitise the pulse width
    if (pulseWidth < 0)
        return DEVICE_INVALID_PARAMETER;

    return setPWM(pulseWidth, DEVICE_DEFAULT_PWM_PERIOD);
}

/**
 * Configures the PWM period of the analog output to the given value.
 *
 * @param period The new period for the analog output in microseconds.
 *
 * @return DEVICE_OK on success.
 */
int ZPin::setAnalogPeriodUs(int period)
{
    // keep the % of duty cycle
    return setPWM((u64_t)this->pwmCfg->pulse * period / this->pwmCfg->period, period);
}

/**
 * Configures the PWM period of the analog output to the given value.
 *
 * @param period The new period for the analog output in milliseconds.
 *
 * @return DEVICE_OK on success, or DEVICE_NOT_SUPPORTED if the
 *         given pin is not configured as an analog output.
 */
int ZPin::setAnalogPeriod(int period)
{
    return setAnalogPeriodUs(period * 1000);
}

/**
 * Obtains the PWM period of the analog output in microseconds.
 *
 * @return the period on success, or DEVICE_NOT_SUPPORTED if the
 *         given pin is not configured as an analog output.
 */
uint32_t ZPin::getAnalogPeriodUs()
{
    if (!(status & IO_STATUS_ANALOG_OUT))
        return DEVICE_NOT_SUPPORTED;

    return this->pwmCfg->period;
}

/**
 * Obtains the PWM period of the analog output in milliseconds.
 *
 * @return the period on success, or DEVICE_NOT_SUPPORTED if the
 *         given pin is not configured as an analog output.
 */
int ZPin::getAnalogPeriod()
{
    return getAnalogPeriodUs() / 1000;
}

/**
 * Configures the pull of this pin.
 *
 * @param pull one of the mbed pull configurations: PullUp, PullDown, PullNone
 *
 * @return DEVICE_NOT_SUPPORTED if the current pin configuration is anything other
 *         than a digital input, otherwise DEVICE_OK.
 */
int ZPin::setPull(PullMode pull)
{
    pullMode = pull;

    if (status & IO_STATUS_CAN_READ)
        config(status);

    return DEVICE_NOT_SUPPORTED;
}

/**
 * This member function manages the calculation of the timestamp of a pulse detected
 * on a pin whilst in IO_STATUS_EVENT_PULSE_ON_EDGE or IO_STATUS_EVENT_ON_EDGE modes.
 *
 * @param eventValue the event value to distribute onto the message bus.
 */
void ZPin::pulseWidthEvent(int eventValue)
{
    Event evt(id, eventValue, CREATE_ONLY);
    auto now = evt.timestamp;
    auto previous = this->evCfg->prevPulse;

    if (previous != 0)
    {
        evt.timestamp -= previous;
        evt.fire();
    }

    this->evCfg->prevPulse = now;
}

/**
 * Interrupt handler for when an rise interrupt is triggered.
 */
void ZPin::onRise()
{
    if (status & IO_STATUS_EVENT_PULSE_ON_EDGE)
        pulseWidthEvent(DEVICE_PIN_EVT_PULSE_LO);

    if (status & IO_STATUS_EVENT_ON_EDGE)
        Event(id, DEVICE_PIN_EVT_RISE);
}

/**
 * Interrupt handler for when an fall interrupt is triggered.
 */
void ZPin::onFall()
{
    if (status & IO_STATUS_EVENT_PULSE_ON_EDGE)
        pulseWidthEvent(DEVICE_PIN_EVT_PULSE_HI);

    if (status & IO_STATUS_EVENT_ON_EDGE)
        Event(id, DEVICE_PIN_EVT_FALL);
}

void ZPin::eventCallback(struct device *port, struct gpio_callback *cb, u32_t pins)
{
    auto cfg = CONTAINER_OF(cb, ZEventConfig, callback);
    auto pin = cfg->parent;
    u32_t v = 0;
    // this is somewhat suboptimal, since the state of the pin might have changed
    // since the event was rised
    gpio_pin_read(pin->port, pin->name & PINMASK, &v);
    if (v)
        pin->onRise();
    else
        pin->onFall();
}

/**
 * This member function will construct an TimedInterruptIn instance, and configure
 * interrupts for rise and fall.
 *
 * @param eventType the specific mode used in interrupt context to determine how an
 *                  edge/rise is processed.
 *
 * @return DEVICE_OK on success
 */
int ZPin::enableRiseFallEvents(int eventType)
{
    // if we are in neither of the two modes, configure pin as a TimedInterruptIn.
    if (!(status & (IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE)))
    {
        config(0);
        auto cfg = this->evCfg = new ZEventConfig;
        cfg->parent = this;
        auto pin = this->name & PINMASK;
        gpio_init_callback(&cfg->callback, &ZPin::eventCallback, BIT(pin));
        gpio_add_callback(this->port, &cfg->callback);
        gpio_pin_enable_callback(this->port, pin);
    }

    status &= ~(IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE);

    // set our status bits accordingly.
    if (eventType == DEVICE_PIN_EVENT_ON_EDGE)
        status |= IO_STATUS_EVENT_ON_EDGE;
    else if (eventType == DEVICE_PIN_EVENT_ON_PULSE)
        status |= IO_STATUS_EVENT_PULSE_ON_EDGE;

    return DEVICE_OK;
}

/**
 * If this pin is in a mode where the pin is generating events, it will destruct
 * the current instance attached to this ZPin instance.
 *
 * @return DEVICE_OK on success.
 */
int ZPin::disableEvents()
{
    if (status & (IO_STATUS_EVENT_ON_EDGE | IO_STATUS_EVENT_PULSE_ON_EDGE | IO_STATUS_TOUCH_IN))
        config(0);

    return DEVICE_OK;
}

/**
 * Configures the events generated by this ZPin instance.
 *
 * DEVICE_PIN_EVENT_ON_EDGE - Configures this pin to a digital input, and generates events whenever
 * a rise/fall is detected on this pin. (DEVICE_PIN_EVT_RISE, DEVICE_PIN_EVT_FALL)
 * DEVICE_PIN_EVENT_ON_PULSE - Configures this pin to a digital input, and generates events where
 * the timestamp is the duration that this pin was either HI or LO. (DEVICE_PIN_EVT_PULSE_HI,
 * DEVICE_PIN_EVT_PULSE_LO) DEVICE_PIN_EVENT_ON_TOUCH - Configures this pin as a makey makey style
 * touch sensor, in the form of a Button. Normal button events will be generated using the ID of
 * this pin. DEVICE_PIN_EVENT_NONE - Disables events for this pin.
 *
 * @param eventType One of: DEVICE_PIN_EVENT_ON_EDGE, DEVICE_PIN_EVENT_ON_PULSE,
 * DEVICE_PIN_EVENT_ON_TOUCH, DEVICE_PIN_EVENT_NONE
 *
 * @code
 * DeviceMessageBus bus;
 *
 * ZPin P0(DEVICE_ID_IO_P0, DEVICE_PIN_P0, PIN_CAPABILITY_BOTH);
 * P0.eventOn(DEVICE_PIN_EVENT_ON_PULSE);
 *
 * void onPulse(Event evt)
 * {
 *     int duration = evt.timestamp;
 * }
 *
 * bus.listen(DEVICE_ID_IO_P0, DEVICE_PIN_EVT_PULSE_HI, onPulse, MESSAGE_BUS_LISTENER_IMMEDIATE)
 * @endcode
 *
 * @return DEVICE_OK on success, or DEVICE_INVALID_PARAMETER if the given eventype does not match
 *
 * @note In the DEVICE_PIN_EVENT_ON_PULSE mode, the smallest pulse that was reliably detected was
 * 85us, around 5khz. If more precision is required, please use the InterruptIn class supplied by
 * ARM mbed.
 */
int ZPin::eventOn(int eventType)
{
    switch (eventType)
    {
    case DEVICE_PIN_EVENT_ON_EDGE:
    case DEVICE_PIN_EVENT_ON_PULSE:
        enableRiseFallEvents(eventType);
        break;

    case DEVICE_PIN_EVENT_ON_TOUCH:
        isTouched();
        break;

    case DEVICE_PIN_EVENT_NONE:
        disableEvents();
        break;

    default:
        return DEVICE_INVALID_PARAMETER;
    }

    return DEVICE_OK;
}
} // namespace codal
