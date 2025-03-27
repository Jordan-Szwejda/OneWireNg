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
     * @param pinDef CH32V pin definition used for bit-banging 1-wire bus.
     * @param pullUp If @c true configure internal pull-up resistor for the bus.
     */
    OneWireNg_CH32V(const CH32VPin &pinDef, bool pullUp)
    {
        initDtaGpio(pinDef, pullUp);
    }

#if CONFIG_PWR_CTRL_ENABLED
    /**
     * OneWireNg 1-wire service for CH32V platform.
     *
     * Bus powering is supported via a switching transistor providing
     * the power to the bus and controlled by a dedicated GPIO (@see
     * OneWireNg_BitBang::setupPwrCtrlGpio()). In this configuration the
     * service mimics the open-drain type of output. The approach may be
     * feasible if the GPIO is unable to provide sufficient power for
     * connected slaves working in parasite powering configuration.
     *
     * @param pin CH32V pin definition used for bit-banging 1-wire bus.
     * @param pwrCtrlPin CH32V pin definition controlling the switching
     *     transistor.
     * @param pullUp If @c true configure internal pull-up resistor for the bus.
     */
    OneWireNg_CH32V(const CH32VPin &pinDef, const CH32VPin &pwrCtrlPinDef, bool pullUp)
    {
        initDtaGpio(pinDef, pullUp);
        initPwrCtrlGpio(pwrCtrlPinDef);
    }
#endif

protected:
    TIME_CRITICAL int readDtaGpioIn()
    {
        return GPIO_ReadInputDataBit(_dtaGpio.gpioDef, _dtaGpio.gpioPin);
    }

    TIME_CRITICAL void setDtaGpioAsInput()
    {
        setPinMode(_isPullUp ? GPIO_Mode_IPU : GPIO_Mode_IN_FLOATING, _dtaGpio);
    }

#if CONFIG_PWR_CTRL_ENABLED

    TIME_CRITICAL void writeGpioOut(int state, const CH32VPin &pinDef)
    {
        GPIO_WriteBit(pinDef.gpioDef, pinDef.gpioPin, state ? BitAction::Bit_SET : BitAction::Bit_RESET);
    }

    TIME_CRITICAL void setGpioAsOutput(int state, const CH32VPin &pinDef)
    {
        writeGpioOut(state, pinDef);
        setPinMode(GPIO_Mode_Out_PP, pinDef);
    }

    TIME_CRITICAL void writeGpioOut(int state, GpioType gpio)
    {
        if (gpio == GPIO_DTA) {
            writeGpioOut(state, _dtaGpio);
        } else {
            writeGpioOut(state, _pwrCtrlGpio);            
        }
    }

    TIME_CRITICAL void setGpioAsOutput(int state, GpioType gpio)
    {
        if (gpio == GPIO_DTA) {
            setGpioAsOutput(state, _dtaGpio);
        } else {
            setGpioAsOutput(state, _pwrCtrlGpio);
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
        setPinMode(GPIO_Mode_Out_PP, _dtaGpio);
    }
#endif /* CONFIG_PWR_CTRL_ENABLED */

#if CONFIG_OVERDRIVE_ENABLED
    TIME_CRITICAL int touch1Overdrive()
    {
        GPIO_WriteBit(_dtaGpio.gpioDef, _dtaGpio.gpioPin, BitAction::Bit_RESET);
        /* speed up low-to-high transition */
        GPIO_WriteBit(_dtaGpio.gpioDef, _dtaGpio.gpioPin, BitAction::Bit_SET);
        setPinMode(GPIO_Mode_IN_FLOATING, _dtaGpio);
        return readDtaGpioIn();
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

    void setPinMode(GPIOMode_TypeDef gpioMode, const CH32VPin &pinDef) {
        GPIO_InitTypeDef  GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = pinDef.gpioPin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = gpioMode;
        GPIO_Init(pinDef.gpioDef, &GPIO_InitStructure);    
    }

#if CONFIG_PWR_CTRL_ENABLED
    void initPwrCtrlGpio( const CH32VPin &pwrCtrlPinDef)
    {
        _pwrCtrlGpio = pwrCtrlPinDef;
        setPinMode(GPIO_Mode_Out_PP, _pwrCtrlGpio);
        setupPwrCtrlGpio(true);
    }

    CH32VPin _pwrCtrlGpio;

#endif

    CH32VPin _dtaGpio;
    bool _isPullUp;
};

#endif /* __OWNG_ARDUINO_STM32__ */
