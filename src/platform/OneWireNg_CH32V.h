/*
 * Copyright (c) 2020-2022 Piotr Stolarz
 * OneWireNg: Arduino 1-wire service library
 *
 * Distributed under the 2-clause BSD License (the License)
 * see accompanying file LICENSE for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */

#ifndef __OWNG_CH32V__
#define __OWNG_CH32V__

#include "OneWireNg_BitBang.h"
#include "platform/Platform_TimeCritical.h"


struct CH32VPin {
    uint16_t gpioPin;
    GPIO_TypeDef *gpioDef;
    uint32_t busId;
};

/**
 * Arduino CH32V platform GPIO specific implementation.
 */
class OneWireNg_CH32V: public OneWireNg_BitBang
{
public:
    /**
     * OneWireNg 1-wire service for Arduino CH32V platform.
     *
     * Bus powering is supported via switching its GPIO to the high state.
     * In this case the GPIO servers as a voltage source for connected slaves
     * working in parasite powering configuration.
     *
     * @param pin Arduino GPIO pin definition used for bit-banging 1-wire bus.
     * @param pullUp If @c true configure internal pull-up resistor for the bus.
     */
    OneWireNg_CH32V(const CH32VPin &pinDef, bool pullUp)
    {
        initDtaGpio(pinDef, pullUp);
    }

#if CONFIG_PWR_CTRL_ENABLED
    /**
     * OneWireNg 1-wire service for Arduino STM32 platform.
     *
     * Bus powering is supported via a switching transistor providing
     * the power to the bus and controlled by a dedicated GPIO (@see
     * OneWireNg_BitBang::setupPwrCtrlGpio()). In this configuration the
     * service mimics the open-drain type of output. The approach may be
     * feasible if the GPIO is unable to provide sufficient power for
     * connected slaves working in parasite powering configuration.
     *
     * @param pin Arduino GPIO pin number used for bit-banging 1-wire bus.
     * @param pwrCtrlPin Arduino GPIO pin number controlling the switching
     *     transistor.
     * @param pullUp If @c true configure internal pull-up resistor for the bus.
     */
    OneWireNg_ArduinoSTM32(unsigned pin, unsigned pwrCtrlPin, bool pullUp)
    {
        initDtaGpio(pin, pullUp);
        initPwrCtrlGpio(pwrCtrlPin);
    }
#endif

protected:
    TIME_CRITICAL int readDtaGpioIn()
    {
        return GPIO_ReadInputDataBit(_dtaGpio.gpioDef, _dtaGpio.gpioPin);
    }

    TIME_CRITICAL void setDtaGpioAsInput()
    {
        setPinMode(_isPullUp ? GPIO_Mode_IPU : GPIO_Mode_IN_FLOATING);
    }

#if CONFIG_PWR_CTRL_ENABLED
    TIME_CRITICAL void writeGpioOut(int state, GpioType gpio)
    {
        if (gpio == GPIO_DTA) {
            digitalWriteFast(_dtaGpio.pinName, state);
        } else {
            digitalWriteFast(_pwrCtrlGpio.pinName, state);
        }
    }

    TIME_CRITICAL void setGpioAsOutput(int state, GpioType gpio)
    {
        if (gpio == GPIO_DTA) {
            digitalWriteFast(_dtaGpio.pinName, state);
            LL_GPIO_SetPinMode(
                _dtaGpio.gpio, _dtaGpio.ll_pin, LL_GPIO_MODE_OUTPUT);
        } else {
            digitalWriteFast(_pwrCtrlGpio.pinName, state);
            LL_GPIO_SetPinMode(
                _pwrCtrlGpio.gpio, _pwrCtrlGpio.ll_pin, LL_GPIO_MODE_OUTPUT);
        }
    }
#else
    TIME_CRITICAL void writeGpioOut(int state)
    {
        GPIO_WriteBit(_dtaGpio.gpioDef, _dtaGpio.gpioPin, state ? BitAction::Bit_SET : BitAction::Bit_RESET);
    }

    TIME_CRITICAL void setGpioAsOutput(int state)
    {
        writeGpioOut(state);
        setPinMode(GPIO_Mode_Out_PP);
        writeGpioOut(state);
    }
#endif /* CONFIG_PWR_CTRL_ENABLED */

#if CONFIG_OVERDRIVE_ENABLED
    TIME_CRITICAL int touch1Overdrive()
    {
        digitalWriteFast(_dtaGpio.pinName, 0);
        LL_GPIO_SetPinMode(_dtaGpio.gpio, _dtaGpio.ll_pin, LL_GPIO_MODE_OUTPUT);

        /* speed up low-to-high transition */
        digitalWriteFast(_dtaGpio.pinName, 1);
        LL_GPIO_SetPinMode(_dtaGpio.gpio, _dtaGpio.ll_pin, LL_GPIO_MODE_INPUT);

        return (digitalReadFast(_dtaGpio.pinName) == LOW ? 0 : 1);
    }
#endif

    void initDtaGpio(const CH32VPin &pinDef, bool pullUp)
    {
        _dtaGpio = pinDef;
        _isPullUp = pullUp;
        enableBusForPin();
        setDtaGpioAsInput();
        setupDtaGpio();
    }    

    void enableBusForPin() {
        RCC_APB2PeriphClockCmd(_dtaGpio.busId, ENABLE);
    }

    void setPinMode(GPIOMode_TypeDef gpioMode) {
        GPIO_InitTypeDef  GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = _dtaGpio.gpioPin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = gpioMode;
        GPIO_Init(_dtaGpio.gpioDef, &GPIO_InitStructure);    
    }

#if CONFIG_PWR_CTRL_ENABLED
    void initPwrCtrlGpio(unsigned pin)
    {
        _pwrCtrlGpio.pinName = digitalPinToPinName(pin);
        assert(_pwrCtrlGpio.pinName != NC);

        _pwrCtrlGpio.gpio = GPIOPort[STM_PORT(_pwrCtrlGpio.pinName)];
        _pwrCtrlGpio.ll_pin = STM_LL_GPIO_PIN(_pwrCtrlGpio.pinName);

        pinMode(pin, OUTPUT);
        setupPwrCtrlGpio(true);
    }

    struct {
        PinName pinName;
        GPIO_TypeDef *gpio;
        uint32_t ll_pin;
    } _pwrCtrlGpio;
#endif

    CH32VPin _dtaGpio;
    bool _isPullUp;
};

#endif /* __OWNG_ARDUINO_STM32__ */
