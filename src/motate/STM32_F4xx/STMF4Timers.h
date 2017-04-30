/*
 KL05ZTimers.h - Library for the Motate system
 http://github.com/synthetos/motate/

 Copyright (c) 2013 Robert Giseburt

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

#ifndef KL05ZTIMERS_H_ONCE
#define KL05ZTIMERS_H_ONCE

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
//#include "KL05ZCommon.h"
#include <functional>
#include <type_traits>

namespace Motate {
enum TimerMode {
	/* InputCapture mode (WAVE = 0) */
	kTimerInputCapture         = 0,

	/* Waveform select, Up to MAX */
	kTimerUp            = 1,
	/* Waveform select, Up to TOP */
	kTimerUpToTop     = 2,
	kTimerUpToMatch     = 2,
	/* For PWM, we'll alias kTimerUpToMatch as: */
	kPWMLeftAligned     = kTimerUpToMatch,

	/* Waveform select, Up to 0xFFFFFFFF, then Down */
	kTimerUpDown        = 3,
	/* Waveform select, Up to TOP, then Down */
	kTimerUpDownToTop = 4,
	kTimerUpDownToMatch = 4,

	/* For PWM, we'll alias kTimerUpDownToMatch as: */
	kPWMCenterAligned     = kTimerUpDownToMatch,
};

/* We're trading acronyms for verbose CamelCase. Dubious. */
enum TimerChannelOutputOptions {
	kOutputDisconnected = 0,

	kToggleOnMatch      = 1<<1,
	kClearOnMatch       = 1<<2,
	kSetOnMatch         = 1<<3,

	// Can't actually toggle on overflow...
	/*kToggleOnOverflow   = 1<<4,*/
	kClearOnOverflow    = 1<<5,
	kSetOnOverflow      = 1<<6,

	/* Aliases for use with PWM */
	kPWMOn              = kClearOnMatch | kSetOnOverflow,
	kPWMOnInverted      = kSetOnMatch | kClearOnOverflow,
};

enum TimerChannelInterruptOptions {
	kInterruptsOff              = 0,
	/* Alias for "off" to make more sense
         when returned from getInterruptCause(). */
	kInterruptUnknown           = 0,

	kInterruptOnMatch           = 1<<1,
	/* Note: Interrupt on overflow is actually in the timer, not the channel. */
	kInterruptOnOverflow        = 1<<2,

	/* This turns the IRQ on, but doesn't set the timer to ever trigger it. */
	kInterruptOnSoftwareTrigger = 1<<3,

	/* Set priority levels here as well: */
	kInterruptPriorityHighest   = 1<<5,
	kInterruptPriorityHigh      = 1<<6,
	kInterruptPriorityMedium    = 1<<7,
	kInterruptPriorityLow       = 1<<8,
	kInterruptPriorityLowest    = 1<<9,
};

enum TimerErrorCodes {
	kFrequencyUnattainable = -1,
	kInvalidMode = -2,
};

template <uint8_t timerNum>
struct Timer {

	// NOTE: Notice! The *pointers* are const, not the *values*.
	static TIM_TypeDef * const tc();
	TIM_HandleTypeDef TimHandle;
	static void _enablePeripheralClock();
	static void _disablePeripheralClock();
	static const IRQn_Type tcIRQ();

	static const bool has_channel_interrupts = false;

	Timer() { init(); };
	Timer(const TimerMode mode, const uint32_t freq) {
		init();
		setModeAndFrequency(mode, freq);
	};

	void init() const {
		/* Unlock this thing */
		unlock();
	};

	void unlock() const {
	};

	void lock() const {
		/* No locking this one...*/
	};

	// Set the mode and frequency.
	// Returns: The actual frequency that was used, or kFrequencyUnattainable
	// freq is not const since we may "change" it
	int32_t setModeAndFrequency(const TimerMode mode, uint32_t freq) {
		RCC_ClkInitTypeDef RCC_ClkInitStruct;
		uint32_t PclkFreq;
		uint32_t APBxCLKDivider;

		/* Enable the clock to this peripheral, or we may bus error chaning some of the config... */
		_enablePeripheralClock();

		/* Prepare to be able to make changes: */
		/*   Disable TC clock */
		/*   Disable interrupts */
		/*   Clear status register */

		TimHandle.Instance = tc();
		__HAL_TIM_DISABLE(&TimHandle);


		// Get clock configuration
		// Note: PclkFreq contains here the Latency (not used after)
		HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &PclkFreq);
		// Get the PCLK and APBCLK divider related to the timer
		switch ((uint32_t)tc()) {

		// APB1 clock
		case TIM2_BASE:
		case TIM3_BASE:
		case TIM4_BASE:
		case TIM5_BASE:
#if defined(TIM12_BASE)
		case TIM12_BASE:
#endif
#if defined(TIM13_BASE)
		case TIM13_BASE:
#endif
#if defined(TIM14_BASE)
		case TIM14_BASE:
#endif
			PclkFreq = HAL_RCC_GetPCLK1Freq();
			APBxCLKDivider = RCC_ClkInitStruct.APB1CLKDivider;
			break;

			// APB2 clock
		case TIM1_BASE:
#if defined(TIM8_BASE)
		case TIM8_BASE:
#endif
		case TIM9_BASE:
		case TIM10_BASE:
		case TIM11_BASE:
			// TIMxCLK = PCLKx when the APB prescaler = 1 else TIMxCLK = 2 * PCLKx
			PclkFreq = HAL_RCC_GetPCLK2Freq();
			APBxCLKDivider = RCC_ClkInitStruct.APB2CLKDivider;
			break;
		default:
			break;
		}
		if (APBxCLKDivider != RCC_HCLK_DIV1)
			PclkFreq *= 2;
		uint16_t divisor = 1;

		// Find prescaler value
		uint32_t test_value = PclkFreq / divisor;

		while ( (divisor < 65535) &&
				((freq > test_value) || (freq < (test_value / 0x10000)) || ((test_value / freq) >= 0x10000)) )
		{
			divisor++;
			test_value = (PclkFreq) / divisor;
		}

		TimHandle.Init.Period        = test_value / freq;
		TimHandle.Init.Prescaler   = divisor - 1;
		TimHandle.Init.ClockDivision = 0;
		TimHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;

		if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) {
			//error("Cannot initialize PWM\n");
		}

		__HAL_TIM_ENABLE(&TimHandle);

		// Determine and return the new frequency.
		return freq;
	};

	uint32_t getTopValue() const {
		return TimHandle.Init.Period;
	};

	// Return the current value of the counter. This is a fleeting thing...
	uint32_t getValue() const {
		return tc()->CNT;
	}

	void start() const {

		tc()->CR1|=(TIM_CR1_CEN);
	};
/* The counter of a timer instance is disabled only if all the CCx and CCxN
   channels have been disabled */
#define TIM_CCER_CCxE_MASK  ((uint32_t)(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E))
#define TIM_CCER_CCxNE_MASK ((uint32_t)(TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE))

	void stop() const {
        if ((tc()->CCER & TIM_CCER_CCxE_MASK) == 0U)
          {
            if((tc()->CCER & TIM_CCER_CCxNE_MASK) == 0U)
            {
            	tc()->CR1 &= ~(TIM_CR1_CEN);
            }
          }
	};

	void stopOnMatch() const {
		//tc()->CONF |= TPM_CONF_CSOO_MASK;
	};


	// The following functions are mirrored in Timer<x>::Channel<y>::


	// Specify the duty cycle as a value from 0.0 .. 1.0;
	void setDutyCycleForChannel(const uint8_t channel, const float ratio) {
		setExactDutyCycle(channel, getTopValue() * ratio);
	};

	// Specify channel duty cycle as a integer value from 0 .. TOP (a.k.a MOD).
	// WARNING: There are no checks on the bounds of channel!
	void setExactDutyCycle(const uint8_t channel, const uint32_t absolute) {
		//tc()->CONTROLS[channel].CnV = absolute;
	};

	void setOutputOptions(const uint8_t channel, const uint32_t options) {
		/*
            uint32_t bit_options = 00;

            if (options & kToggleOnMatch) {
                bit_options = TPM_CnSC_ELSA_MASK | TPM_CnSC_MSA_MASK;
            }
            else if ( (options & (kClearOnMatch | kSetOnOverflow)) == (kClearOnMatch | kSetOnOverflow) ) {
                bit_options = TPM_CnSC_ELSB_MASK | TPM_CnSC_MSB_MASK;
            }
            else if ( (options & (kSetOnMatch | kClearOnOverflow)) == (kSetOnMatch | kClearOnOverflow) ) {
                bit_options = TPM_CnSC_ELSA_MASK | TPM_CnSC_MSB_MASK;
            }
            else if (options & kClearOnMatch) {
                bit_options = TPM_CnSC_ELSB_MASK | TPM_CnSC_MSA_MASK;
            }
            else if (options & kSetOnMatch) {
                bit_options = TPM_CnSC_ELSA_MASK | TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK;
            }


            tc()->CONTROLS[channel].CnSC = (tc()->CONTROLS[channel].CnSC & ~(
                                                                             TPM_CnSC_ELSA_MASK |
                                                                             TPM_CnSC_ELSB_MASK |
                                                                             TPM_CnSC_MSA_MASK |
                                                                             TPM_CnSC_MSB_MASK
                                                                             )) | bit_options;
		 */
	};


	// These two start the waveform. We try to be as fast as we can.
	// ASSUMPTION: We stopped it with the corresponding function.
	// ASSUMPTION: The pin is not and was not in Toggle mode.
	void startPWMOutput(const uint8_t channel) {
		/*
            tc()->CONTROLS[channel].CnSC = (tc()->CONTROLS[channel].CnSC & ~(
                                                                             TPM_CnSC_ELSA_MASK |
                                                                             TPM_CnSC_ELSB_MASK
                                                                             )) | TPM_CnSC_ELSB_MASK;
		 */      }

	// These are special function for stopping output waveforms.
	// This disables the channel.
	// ASSUMPTION: The pin is not in Toggle mode.
	void stopPWMOutput(const uint8_t channel) {
		/*
            tc()->CONTROLS[channel].CnSC =
            (tc()->CONTROLS[channel].CnSC & ~(
                                              TPM_CnSC_ELSA_MASK |
                                              TPM_CnSC_ELSB_MASK
                                              ));
		 */
	};

	void setInterrupts(const uint32_t interrupts, const int16_t channel = -1) {

		if (interrupts != kInterruptsOff) {
			if (interrupts & kInterruptOnOverflow) {
				tc()->DIER |= TIM_IT_UPDATE;
			}
			// WARNING! No range checking or error reporting!
			if ((interrupts & kInterruptOnMatch) && (channel >= 0)) {
				/*
                    tc()->CONTROLS[channel].CnSC |= TPM_CnSC_CHIE_MASK;
                    if (!(tc()->CONTROLS[channel].CnSC & (TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK))) {
                        // The channel "output" must be turned on
                        tc()->CONTROLS[channel].CnSC |= TPM_CnSC_MSA_MASK;
                    }
				 */
			}

			/* Set interrupt priority */

			if (interrupts & kInterruptPriorityHighest) {
				HAL_NVIC_SetPriority(tcIRQ(), 2, 0);
			}
			else if (interrupts & kInterruptPriorityHigh) {
				HAL_NVIC_SetPriority(tcIRQ(), 5, 0);
			}
			else if (interrupts & kInterruptPriorityMedium) {
				HAL_NVIC_SetPriority(tcIRQ(), 8, 0);
			}
			else if (interrupts & kInterruptPriorityLow) {
				HAL_NVIC_SetPriority(tcIRQ(), 11, 0);
			}
			else if (interrupts & kInterruptPriorityLowest) {
				HAL_NVIC_SetPriority(tcIRQ(), 14, 0);
			}

			NVIC_EnableIRQ(tcIRQ());

		} else {
			tc()->DIER &= ~TIM_IT_UPDATE;
			if(channel != -1)
			{
				tc()->DIER &= ~(1 << channel);
			}
			NVIC_DisableIRQ(tcIRQ());
		}
	}

	void setInterruptPending() {
		NVIC_SetPendingIRQ(tcIRQ());
	}

	/* Here for reference (most we don't use):
         TC_SR_COVFS   (TC_SR) Counter Overflow Status
         TC_SR_LOVRS   (TC_SR) Load Overrun Status
         TC_SR_CPAS    (TC_SR) RA Compare Status
         TC_SR_CPBS    (TC_SR) RB Compare Status
         TC_SR_CPCS    (TC_SR) RC Compare Status
         TC_SR_LDRAS   (TC_SR) RA Loading Status
         TC_SR_LDRBS   (TC_SR) RB Loading Status
         TC_SR_ETRGS   (TC_SR) External Trigger Status
         TC_SR_CLKSTA  (TC_SR) Clock Enabling Status
         TC_SR_MTIOA   (TC_SR) TIOA Mirror
         TC_SR_MTIOB   (TC_SR) TIOB Mirror
	 */

	static TimerChannelInterruptOptions getInterruptCause() {
		uint32_t status = Timer<timerNum>::tc()->SR;
		if(status & TIM_FLAG_UPDATE)
		{
			Timer<timerNum>::tc()->SR &= ~TIM_FLAG_UPDATE;
			return kInterruptOnOverflow;
		}
		else if(status != 0)
		{
			Timer<timerNum>::tc()->SR &= ~(TIM_FLAG_CC1|TIM_FLAG_CC2|TIM_FLAG_CC3|TIM_FLAG_CC4);
			return kInterruptOnMatch;
		}
		return kInterruptUnknown;
	}

	// Placeholder for user code.
	static void interrupt() __attribute__ ((weak));
};


template<uint8_t timerNum, uint8_t channelNum>
struct TimerChannel : Timer<timerNum> {
	TimerChannel() : Timer<timerNum>{} {};
	TimerChannel(const TimerMode mode, const uint32_t freq) : Timer<timerNum>{mode, freq} {};

	void setDutyCycle(const float ratio) {
		Timer<timerNum>::setDutyCycleForChannel(channelNum, ratio);
	};

	void setExactDutyCycle(const uint32_t absolute) {
		Timer<timerNum>::setExactDutyCycle(channelNum, absolute);
	};

	void setOutputOptions(const uint32_t options) {
		Timer<timerNum>::setOutputOptions(channelNum, options);
	};

	void startPWMOutput() {
		Timer<timerNum>::startPWMOutput(channelNum);
	};

	void stopPWMOutput() {
		Timer<timerNum>::stopPWMOutput(channelNum);
	}

	void setInterrupts(const uint32_t interrupts) {
		Timer<timerNum>::setInterrupts(interrupts, channelNum);
	};

	static TimerChannelInterruptOptions getInterruptCause() {
		uint32_t status = Timer<timerNum>::tc()->SR;
		if(status & TIM_FLAG_UPDATE)
		{
			Timer<timerNum>::tc()->SR &= ~TIM_FLAG_UPDATE;
			return kInterruptOnOverflow;
		}
		else if(status != 0 && (status & (1 << channelNum)))
		{
			Timer<timerNum>::tc()->SR &= ~(1 << channelNum);
			return kInterruptOnMatch;
		}

		return kInterruptUnknown;
	}

	// Placeholder for user code.
	static void interrupt() __attribute__ ((weak));
};

/**************************************************
 *
 * SysTickTimer and related:
 *  Timer<SysTickTimerNum> is the special Timer for Systick.
 *  SysTickTimer is the global singleton to access it.
 *  SysTickEvent is the class to use to register a new event to occur every Tick.
 *
 **************************************************/
struct SysTickEvent {
    const std::function<void(void)> callback;
    SysTickEvent *next;
};

typedef const uint8_t timer_number;

static const timer_number SysTickTimerNum = 0xFF;
template <>
struct Timer<SysTickTimerNum> {
	static volatile uint32_t _motateTickCount;
	SysTickEvent *firstEvent = nullptr;

	Timer() { init(); };
	Timer(const TimerMode mode, const uint32_t freq) {
		init();
	};

	void init() {
		_motateTickCount = 0;

		// Set Systick to 1ms interval, common to all SAM3 variants
		if (SysTick_Config(SystemCoreClock / 1000))
		{
			// Capture error
			while (true);
		}
	};

	// Return the current value of the counter. This is a fleeting thing...
	uint32_t getValue() {
		return _motateTickCount;
	};

	void _increment() {
		_motateTickCount++;
	};
    void registerEvent(SysTickEvent *new_event) {
        if (firstEvent == nullptr) {
            firstEvent = new_event;
            return;
        }
        SysTickEvent *event = firstEvent;
        if (new_event == event) { return; }
        while (event->next != nullptr) {
            event = event->next;
            if (new_event == event) { return; }
        }
        event->next = new_event;
        new_event->next = nullptr;
    };

    void unregisterEvent(SysTickEvent *new_event) {
        if (firstEvent == new_event) {
            firstEvent = firstEvent->next;
            return;
        }
        SysTickEvent *event = firstEvent;
        while (event->next != nullptr) {
            if (event->next == new_event) {
                event->next = event->next->next;
                return;
            }
            event = event->next;
        }
    };

    void _handleEvents() {
        SysTickEvent *event = firstEvent;
        while (event != nullptr) {
            event->callback();
            event = event->next;
        }
    };
	// Placeholder for user code.
	static void interrupt() __attribute__ ((weak));
};
extern Timer<SysTickTimerNum> SysTickTimer;


static const timer_number WatchDogTimerNum = 0xFE;
template <>
struct Timer<WatchDogTimerNum> {

	Timer() { init(); };
	Timer(const TimerMode mode, const uint32_t freq) {
		init();
		//			setModeAndFrequency(mode, freq);
	};

	void init() {
	};

	void disable() {
		  /* Reset WWDG Control register */
		WWDG->CR  = (uint32_t)0x0000007F;

		  /* Reset WWDG Configuration register */
		WWDG->CFR = (uint32_t)0x0000007F;
	};

	void checkIn() {
	};

	// Placeholder for user code.
	static void interrupt() __attribute__ ((weak));
};
extern Timer<WatchDogTimerNum> WatchDogTimer;

// Provide a Arduino-compatible blocking-delay function
inline void delay( uint32_t microseconds )
{
	uint32_t doneTime = SysTickTimer.getValue() + microseconds;

	do
	{
		__NOP();
	} while ( SysTickTimer.getValue() < doneTime );
}

struct Timeout {
	uint32_t start_, delay_;
	Timeout() : start_ {0}, delay_ {0} {};

    bool isSet() {
        return (start_ > 0);
    }

	bool isPast() {
		return ((SysTickTimer.getValue() - start_) > delay_);
	};

	void set(uint32_t delay) {
		start_ = SysTickTimer.getValue();
		delay_ = delay;
	}
    void clear() {
        start_ = 0;
        delay_ = 0;
    }
};

#define MOTATE_TIMER_INTERRUPT(number) template<> void Motate::Timer<number>::interrupt()
#define MOTATE_TIMER_CHANNEL_INTERRUPT(t, ch) template<> void Motate::TimerChannel<t, ch>::interrupt()

} // namespace Motate


#endif /* end of include guard: KL05ZTIMERS_H_ONCE */
