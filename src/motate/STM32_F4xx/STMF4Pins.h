/*
 utility/SamPins.h - Library for the Motate system
 http://github.com/synthetos/motate/

 Copyright (c) 2015 - 2016 Robert Giseburt

	This file is part of the Motate Library.

	This file ("the software") is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License, version 2 as published by the
	Free Software Foundation. You should have received a copy of the GNU General Public
	License, version 2 along with the software. If not, see <http://www.gnu.org/licenses/>.

	As a special exception, you may use this file as part of a software library without
	restriction. Specifically, if other files instantiate templates or use macros or
	inline functions from this file, or you compile this file and link it with  other
	files to produce an executable, this file does not by itself cause the resulting
	executable to be covered by the GNU General Public License. This exception does not
	however invalidate any other reasons why the executable file might be covered by the
	GNU General Public License.

	THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
	WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
	SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
	OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */

#ifndef SAMPINS_H_ONCE
#define SAMPINS_H_ONCE

// #include <chip.h>
#include <stm32f4xx.h>
#include "stm32f4xx_hal.h"
#include "MotateTimers.h"

#include <functional>
#include <type_traits>

namespace Motate {
// Numbering is arbitrary:
enum PinMode : PinMode_t {
			kUnchanged      = 0,
			kOutput         = 1, // Output PP
			kInput          = 2,
			kOutputOD       = 3,	// Output OD
};

// Numbering is arbitrary, but bit unique for bitwise operations (unlike other architectures):
enum PinOptions : PinOptions_t {
	kNormal         = 0,
			kPullUp         = 1<<1,

			kDeglitch       = 1<<4,
			kDebounce       = 1<<5,

			// Set the intialized value of the pin
			kStartHigh      = 1<<6,
			kStartLow       = 1<<7,
			// For use on PWM pins only!
			kPWMPinInverted = 1<<8,
};

enum PinInterruptOptions : PinInterruptOptions_t {
	kPinInterruptsOff                = 0,

			kPinInterruptOnChange            = 1,

			kPinInterruptOnRisingEdge        = 1<<1,
			kPinInterruptOnFallingEdge       = 2<<1,

			kPinInterruptOnLowLevel          = 3<<1,
			kPinInterruptOnHighLevel         = 4<<1,

			kPinInterruptAdvancedMask        = ((1<<3)-1)<<1,

			/* This turns the IRQ on, but doesn't set the timer to ever trigger it. */
			kPinInterruptOnSoftwareTrigger   = 1<<4,

			kPinInterruptTypeMask            = (1<<5)-1,

			/* Set priority levels here as well: */
			kPinInterruptPriorityHighest     = 1<<5,
			kPinInterruptPriorityHigh        = 1<<6,
			kPinInterruptPriorityMedium      = 1<<7,
			kPinInterruptPriorityLow         = 1<<8,
			kPinInterruptPriorityLowest      = 1<<9,

			kPinInterruptPriorityMask        = ((1<<10) - (1<<5))
};

struct _pinChangeInterrupt {
	const uint32_t pc_mask; // Pin uses "mask" so we use a different name. "pc" for pinChange
	std::function<void(void)> interrupt_handler;
	_pinChangeInterrupt *next;

	_pinChangeInterrupt(const _pinChangeInterrupt &) = delete; // delete the copy constructor, we only allow moves
	_pinChangeInterrupt &operator=(const _pinChangeInterrupt &) = delete; // delete the assigment operator, we only allow moves
	_pinChangeInterrupt &operator=(const _pinChangeInterrupt &&) = delete; // delete the move assigment operator, we only allow moves


	_pinChangeInterrupt(const uint32_t _mask, std::function<void(void)> &&_interrupt, _pinChangeInterrupt *&_first)
	: pc_mask{_mask}, interrupt_handler{std::move(_interrupt)}, next{nullptr}
	{
		if (interrupt_handler) { // std::function returns false if the function isn't valid
			if (_first == nullptr) {
				_first = this;
				return;
			}

			_pinChangeInterrupt *i = _first;
			while (i->next != nullptr) {
				i = i->next;
			}
			i->next = this;
		}
	};

	void setInterrupt(std::function<void(void)> &&_interrupt)
	{
		interrupt_handler = std::move(_interrupt);
	};

	void setInterrupt(const std::function<void(void)> &_interrupt)
	{
		interrupt_handler = _interrupt;
	};
};

typedef uint32_t uintPort_t;

#pragma mark PortHardware
/**************************************************
 *
 * HARDWARE LAYER: PortHardware
 *
 **************************************************/

template <unsigned char portLetter>
struct PortHardware {
	static const uint8_t letter = portLetter;

	// The constexpr functions we can define here, and get really great optimization.
	// These switch statements are handled by the compiler, not at runtime.
	constexpr GPIO_TypeDef* const rawPort() const
	{
		switch (portLetter) {
		case 'A': return GPIOA;
#ifdef GPIOB
		case 'B': return GPIOB;
#endif
#ifdef GPIOC
		case 'C': return GPIOC;
#endif
#ifdef GPIOD
		case 'D': return GPIOD;
#endif
#ifdef GPIOF
		case 'F': return GPIOF;
#endif
		}
	};
	constexpr IRQn_Type _IRQn(const uintPort_t mask){
		switch (mask) {
			case GPIO_PIN_0:
				return EXTI0_IRQn;
			case GPIO_PIN_1:
				return EXTI1_IRQn;
			case GPIO_PIN_2:
				return EXTI2_IRQn;
			case GPIO_PIN_3:
				return EXTI3_IRQn;
			case GPIO_PIN_4:
				return EXTI4_IRQn;
			case GPIO_PIN_5:
			case GPIO_PIN_6:
			case GPIO_PIN_7:
			case GPIO_PIN_8:
			case GPIO_PIN_9:
				return EXTI9_5_IRQn;
			case GPIO_PIN_10:
			case GPIO_PIN_11:
			case GPIO_PIN_12:
			case GPIO_PIN_13:
			case GPIO_PIN_14:
			case GPIO_PIN_15:
				return EXTI15_10_IRQn;
			default:
				return EXTI0_IRQn;
		}

	}
	constexpr static const uint32_t peripheralId()
	{
		switch (portLetter) {
		default:
		case 'A': return 0;
#ifdef GPIOB
		case 'B': return 1;
#endif
#ifdef GPIOC
		case 'C': return 2;
#endif
#ifdef GPIOD
		case 'D': return 3;
#endif
#ifdef GPIOF
		case 'F': return 5;
#endif
		}
	};



	static _pinChangeInterrupt *_firstInterrupt;

	void enableClock();
	void setModes(const PinMode type, const uintPort_t mask) {
		// Configure GPIO
		GPIO_InitTypeDef GPIO_InitStructure;

		enableClock();


		GPIO_InitStructure.Pin       = mask;

		GPIO_InitStructure.Pull      = GPIO_NOPULL;
		GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
		GPIO_InitStructure.Alternate = 0;

		switch (type) {
		case kOutput:
			GPIO_InitStructure.Mode      = GPIO_MODE_OUTPUT_PP;
			HAL_GPIO_Init(rawPort(), &GPIO_InitStructure);
			break;
		case kOutputOD:
			GPIO_InitStructure.Mode      = GPIO_MODE_OUTPUT_OD;
			HAL_GPIO_Init(rawPort(), &GPIO_InitStructure);
			break;
		case kInput:
			GPIO_InitStructure.Mode      = GPIO_MODE_INPUT;
			HAL_GPIO_Init(rawPort(), &GPIO_InitStructure);
			break;
		default:
			break;
		}


	};
	// Returns the mode of ONE pin, and only Input or Output
	PinMode getMode(const uintPort_t mask) {
		uint32_t pin_index = 0;
		for (int i=1, iter=0; iter<32; i<<=1, iter++){
			if (mask & i){
				pin_index = iter;
				break;
			}
		}

		return (rawPort()->MODER & (GPIO_MODER_MODER0 << (pin_index * 2))) ? kOutput : kInput;
	};
	void setOptions(const PinOptions_t options, const uintPort_t mask) {
		uint32_t pin_index = 0;
		for (int i=1, iter=0; iter<32; i<<=1, iter++){
			if (mask & i){
				pin_index = iter;
				break;
			}
		}


		if (kStartHigh & options)
		{
			set(mask);
		} else if (kStartLow & options)
		{
			clear(mask);
		}
		if (kPullUp & options)
		{
			rawPort()->PUPDR &= (uint32_t)(~(GPIO_PUPDR_PUPDR0 << (pin_index * 2)));
			rawPort()->PUPDR |= (uint32_t)(GPIO_PULLUP << (pin_index * 2));
		}
		else
		{
			rawPort()->PUPDR &= (uint32_t)(~(GPIO_PUPDR_PUPDR0 << (pin_index * 2)));
		}
	};
	PinOptions_t getOptions(const uintPort_t mask) {
		/*            return ((//rawPort()->PIO_PUSR & mask) ? kPullUp : 0) |
            ((//rawPort()->PIO_MDSR & mask) ? kWiredAnd : 0) |
            ((//rawPort()->PIO_IFSR & mask) ?
             ((//rawPort()->PIO_IFDGSR & mask) ? kDebounce : kDeglitch) : 0);
		 */
		return 0;
	};
	void set(const uintPort_t mask) {
		rawPort()->BSRR = mask;
	};
	void clear(const uintPort_t mask) {
		rawPort()->BSRR = (uint32_t)mask << 16 ;
	};
	void toggle(const uintPort_t mask) {
		rawPort()->ODR ^= mask;
	};
	void write(const uintPort_t value) {
		rawPort()->ODR = value & 0xFFFF;
	};
	void write(const uintPort_t value, const uintPort_t mask) {
		rawPort()->ODR = value & mask;
	};
	uintPort_t getInputValues(const uintPort_t mask) {
		return rawPort()->IDR  & mask;
	};
	uintPort_t getOutputValues(const uintPort_t mask) {
		return rawPort()->ODR  & mask;
	};
	GPIO_TypeDef* portPtr() {
		return rawPort;
	};
	void setInterrupts(const uint32_t interrupts, const uintPort_t mask) {
		if (interrupts != kPinInterruptsOff) {
			GPIO_InitTypeDef GPIO_InitStructure;

			enableClock();


			GPIO_InitStructure.Pin       = mask;

			GPIO_InitStructure.Pull      = GPIO_NOPULL;
			GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
			GPIO_InitStructure.Alternate = 0;
			//rawPort()->PIO_IDR = mask;

			/*Is it an "advanced" interrupt?*/
			if (interrupts & kPinInterruptAdvancedMask) {
				/*Is it an edge interrupt?*/
				if ((interrupts & kPinInterruptTypeMask) == kPinInterruptOnRisingEdge ||
						(interrupts & kPinInterruptTypeMask) == kPinInterruptOnFallingEdge) {

					if((interrupts & kPinInterruptTypeMask) == kPinInterruptOnRisingEdge){
						GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
					}else{
						GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
					}
				}
			}
			else
			{
				// kPinInterruptOnChange
				GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;

			}
			HAL_GPIO_Init(rawPort(), &GPIO_InitStructure);

			/* Set interrupt priority */
			if (interrupts & kPinInterruptPriorityMask) {
				if (interrupts & kPinInterruptPriorityHighest) {
					NVIC_SetPriority(_IRQn(mask), 0);
				}
				else if (interrupts & kPinInterruptPriorityHigh) {
					NVIC_SetPriority(_IRQn(mask), 3);
				}
				else if (interrupts & kPinInterruptPriorityMedium) {
					NVIC_SetPriority(_IRQn(mask), 7);
				}
				else if (interrupts & kPinInterruptPriorityLow) {
					NVIC_SetPriority(_IRQn(mask), 11);
				}
				else if (interrupts & kPinInterruptPriorityLowest) {
					NVIC_SetPriority(_IRQn(mask), 15);
				}
			}
			/* Enable the IRQ */
			NVIC_EnableIRQ(_IRQn(mask));
			/* Enable the interrupt */
			//rawPort()->PIO_IER = mask;
		} else {
			//rawPort()->PIO_IDR = mask;
			//if (0)//rawPort()->PIO_ISR == 0)
			NVIC_DisableIRQ(_IRQn(mask));
		}
	};

	void addInterrupt(_pinChangeInterrupt *newInt) {
		_pinChangeInterrupt *i = _firstInterrupt;
		if (i == nullptr) {
			_firstInterrupt = newInt;
			return;
		}
		while (i->next != nullptr) {
			i = i->next;
		}
		i->next = newInt;
	}
};

/**************************************************
 *
 * BASIC PINS: _MAKE_MOTATE_PIN
 *
 **************************************************/

#define ___MAKE_MOTATE_PIN(pinNum, registerChar, registerPin)

#define _MAKE_MOTATE_PIN(pinNum, registerChar, registerPin) \
		template<> \
		struct Pin<pinNum> : RealPin<registerChar, registerPin> { \
	static const int16_t number = pinNum; \
	static const uint8_t portLetter = (uint8_t) registerChar; \
	Pin() : RealPin<registerChar, registerPin>() {}; \
	Pin(const PinMode type, const PinOptions_t options = kNormal) : RealPin<registerChar, registerPin>(type, options) {}; \
}; \
template<> \
struct ReversePinLookup<registerChar, registerPin> : Pin<pinNum> { \
	ReversePinLookup() {}; \
	ReversePinLookup(const PinMode type, const PinOptions_t options = kNormal) : Pin<pinNum>(type, options) {}; \
};




#pragma mark IRQPin support
/**************************************************
 *
 * PIN CHANGE INTERRUPT SUPPORT: IsIRQPin / MOTATE_PIN_INTERRUPT
 *
 **************************************************/

template<int16_t pinNum>
constexpr const bool IsIRQPin() { return !Pin<pinNum>::isNull(); }; // Basically return if we have a valid pin.

#define MOTATE_PIN_INTERRUPT(number) \
		template<> void Motate::IRQPin<number>::interrupt()





#pragma mark ADC_Module/ACD_Pin (Sam3x)
/**************************************************
 *
 * PIN CHANGE INTERRUPT SUPPORT: IsIRQPin / MOTATE_PIN_INTERRUPT
 *
 **************************************************/

template<pin_number pinNum>
struct ADCPinParent {
	static const uint32_t adcMask = 0;
	static const uint32_t adcNumber = 0;
	static const uint16_t getTop() { return 4095; };
};

// Some pins are ADC pins.
template<pin_number n>
struct ADCPin : Pin<-1> {
	ADCPin() : Pin<-1>() {};
	ADCPin(const PinOptions_t options) : Pin<-1>() {};

	uint32_t getRaw() {
		return 0;
	};
	uint32_t getValue() {
		return 0;
	};
	operator int16_t() {
		return getValue();
	};
	operator float() {
		return 0.0;
	};
	static const uint16_t getTop() { return 4095; };

	static const bool is_real = false;
	void setInterrupts(const uint32_t interrupts) {};
	static void interrupt() __attribute__ (( weak )); // Allow setting an interrupt on a invalid ADC pin -- will never be called
};

template<int16_t adcNum>
struct ReverseADCPin : ADCPin<-1> {
	ReverseADCPin() : ADCPin<-1>() {};
	ReverseADCPin(const PinOptions_t options) : ADCPin<-1>() {};
};

#define _MAKE_MOTATE_ADC_PIN(registerChar, registerPin, adcNum) \
		template<> \
		struct ADCPinParent< ReversePinLookup<registerChar, registerPin>::number > { \
	static const uint32_t adcMask = 1 << adcNum; \
	static const uint32_t adcNumber = adcNum; \
	static const uint16_t getTop() { return 4095; }; \
}; \
template<> \
struct ReverseADCPin<adcNum> : ADCPin<ReversePinLookup<registerChar, registerPin>::number> { \
	ReverseADCPin() : ADCPin<ReversePinLookup<registerChar, registerPin>::number>() {}; \
	ReverseADCPin(const PinOptions_t options) : ADCPin<ReversePinLookup<registerChar, registerPin>::number>(options) {}; \
};

template<int16_t pinNum>
constexpr const bool IsADCPin() { return ADCPin<pinNum>::is_real; };

template<uint8_t portChar, int16_t portPin>
using LookupADCPin = ADCPin< ReversePinLookup<portChar, portPin>::number >;

template<int16_t adcNum>
using LookupADCPinByADC = ADCPin< ReverseADCPin< adcNum >::number >;

#pragma mark PWMOutputPin support
/**************************************************
 *
 * PWM ("fake" analog) output pin support: _MAKE_MOTATE_PWM_PIN
 *
 **************************************************/

#define _MAKE_MOTATE_PWM_PIN(registerChar, registerPin, timerOrPWM, invertedByDefault) \
		template<> \
		struct AvailablePWMOutputPin< ReversePinLookup<registerChar, registerPin>::number > : RealPWMOutputPin< ReversePinLookup<registerChar, registerPin>::number, timerOrPWM > { \
	typedef timerOrPWM parentTimerType; \
	static const pin_number pinNum = ReversePinLookup<registerChar, registerPin>::number; \
	AvailablePWMOutputPin() : RealPWMOutputPin<pinNum, timerOrPWM>(kOutput) { pwmpin_init(invertedByDefault ? kPWMOnInverted : kPWMOn);}; \
	AvailablePWMOutputPin(const PinOptions_t options, const uint32_t freq) : RealPWMOutputPin<pinNum, timerOrPWM>(kOutput, options, freq) { \
		pwmpin_init((invertedByDefault ^ ((options & kPWMPinInverted)?true:false)) ? kPWMOnInverted : kPWMOn); \
	}; \
	using RealPWMOutputPin<pinNum, timerOrPWM>::operator=; \
	/* Signal to _GetAvailablePWMOrAlike that we're here, AND a real Pin<> exists. */ \
		static constexpr bool _isAvailable() { return !ReversePinLookup<registerChar, registerPin>::isNull(); };  \
};


#pragma mark SPI Pins support
/**************************************************
 *
 * SPI PIN METADATA and wiring: specializes SPIChipSelectPin / SPIMISOPin / SPIMOSIPin / SPISCKPin
 *
 * Provides: _MAKE_MOTATE_SPI_CS_PIN
 *           _MAKE_MOTATE_SPI_MISO_PIN
 *           _MAKE_MOTATE_SPI_MOSI_PIN
 *           _MAKE_MOTATE_SPI_SCK_PIN
 *
 **************************************************/


#define _MAKE_MOTATE_SPI_CS_PIN(registerChar, registerPin, spiNumber, peripheralAorB, csNum) \
		template<> \
		struct SPIChipSelectPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> { \
	SPIChipSelectPin() : ReversePinLookup<registerChar, registerPin>(kPeripheral ## peripheralAorB) {}; \
	static constexpr bool is_real = true; \
	static constexpr uint8_t spiNum = spiNumber; \
	static constexpr uint8_t csNumber =  csNum; \
	static constexpr uint8_t csValue  = ~csNum; \
	static constexpr bool usesDecoder = false; \
};

#define _MAKE_MOTATE_SPI_MISO_PIN(registerChar, registerPin, spiNumber, peripheralAorB)\
		template<>\
		struct SPIMISOPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
	SPIMISOPin() : ReversePinLookup<registerChar, registerPin>{kPeripheral ## peripheralAorB} {};\
	static constexpr bool is_real = true; \
	static constexpr uint8_t spiNum = spiNumber; \
};


#define _MAKE_MOTATE_SPI_MOSI_PIN(registerChar, registerPin, spiNumber, peripheralAorB)\
		template<>\
		struct SPIMOSIPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
	SPIMOSIPin() : ReversePinLookup<registerChar, registerPin>{kPeripheral ## peripheralAorB} {};\
	static constexpr bool is_real = true; \
	static constexpr uint8_t spiNum = spiNumber; \
};


#define _MAKE_MOTATE_SPI_SCK_PIN(registerChar, registerPin, spiNumber, peripheralAorB)\
		template<>\
		struct SPISCKPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
	SPISCKPin() : ReversePinLookup<registerChar, registerPin> { kPeripheral ## peripheralAorB } {};\
	static constexpr bool is_real = true; \
	static constexpr uint8_t spiNum = spiNumber; \
};


#pragma mark UART / USART Pin support
/**************************************************
 *
 * UART/USART PIN METADATA and wiring: specializes UARTTxPin / UARTRxPin / UARTRTSPin / UARTCTSPin
 *
 * Provides: _MAKE_MOTATE_UART_TX_PIN
 *           _MAKE_MOTATE_UART_RX_PIN
 *           _MAKE_MOTATE_UART_RTS_PIN
 *           _MAKE_MOTATE_UART_CTS_PIN
 *
 **************************************************/

#define _MAKE_MOTATE_UART_TX_PIN(registerChar, registerPin, uartNumVal, peripheralAorB)\
		template<>\
		struct UARTTxPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
	UARTTxPin() : ReversePinLookup<registerChar, registerPin>(kPeripheral ## peripheralAorB, kPullUp) {};\
	static const uint8_t uartNum = uartNumVal;\
	static const bool is_real = true;\
};

#define _MAKE_MOTATE_UART_RX_PIN(registerChar, registerPin, uartNumVal, peripheralAorB)\
		template<>\
		struct UARTRxPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
	UARTRxPin() : ReversePinLookup<registerChar, registerPin>(kPeripheral ## peripheralAorB) {};\
	static const uint8_t uartNum = uartNumVal;\
	static const bool is_real = true;\
};

#define _MAKE_MOTATE_UART_RTS_PIN(registerChar, registerPin, uartNumVal, peripheralAorB)\
		template<>\
		struct UARTRTSPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
	UARTRTSPin() : ReversePinLookup<registerChar, registerPin>(kPeripheral ## peripheralAorB) {};\
	static const uint8_t uartNum = uartNumVal;\
	static const bool is_real = true;\
	void operator=(const bool value); /*Will cause a failure if used.*/\
		};

#define _MAKE_MOTATE_UART_CTS_PIN(registerChar, registerPin, uartNumVal, peripheralAorB)\
		template<>\
		struct UARTCTSPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
	UARTCTSPin() : ReversePinLookup<registerChar, registerPin>(kPeripheral ## peripheralAorB, kPullUp) {};\
	UARTCTSPin(const PinOptions_t options, const std::function<void(void)> &&_interrupt, const uint32_t interrupt_settings = kPinInterruptOnChange|kPinInterruptPriorityMedium) : ReversePinLookup<registerChar, registerPin>(kPeripheral ## peripheralAorB, options) {};\
	void setInterrupts(const uint32_t interrupts); /*Will cause a failure if used.*/\
		static const uint8_t uartNum = uartNumVal;\
		static const bool is_real = true;\
};

#pragma mark ClockOutputPin
/**************************************************
 *
 * Clock Output PIN METADATA and wiring: CLKOutPin
 *
 * Provides: _MAKE_MOTATE_CLOCK_OUTPUT_PIN
 *
 **************************************************/

#define _MAKE_MOTATE_CLOCK_OUTPUT_PIN(registerChar, registerPin, clockNumber, peripheralAorB)\
		template<>\
		struct ClockOutputPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
	ClockOutputPin(const uint32_t target_freq) : ReversePinLookup<registerChar, registerPin>(kPeripheral ## peripheralAorB) {\
			uint32_t prescaler = PMC_PCK_PRES_CLK_1;\
			if ((SystemCoreClock >> 1) < target_freq) { prescaler = PMC_PCK_PRES_CLK_2; }\
			if ((SystemCoreClock >> 2) < target_freq) { prescaler = PMC_PCK_PRES_CLK_4; }\
			if ((SystemCoreClock >> 3) < target_freq) { prescaler = PMC_PCK_PRES_CLK_8; }\
			if ((SystemCoreClock >> 4) < target_freq) { prescaler = PMC_PCK_PRES_CLK_16; }\
			if ((SystemCoreClock >> 5) < target_freq) { prescaler = PMC_PCK_PRES_CLK_32; }\
			if ((SystemCoreClock >> 6) < target_freq) { prescaler = PMC_PCK_PRES_CLK_64; }\
			PMC->PMC_PCK[clockNumber] = PMC_PCK_CSS_MCK | prescaler;\
		};\
		static const bool is_real = true;\
		void operator=(const bool value); /*Will cause a failure if used.*/\
		};

} // end namespace Motate

#endif /* end of include guard: SAMPINS_H_ONCE */
