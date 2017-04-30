/*
  KL05ZTimers.cpp - Library for the Motate system
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



#include "STM32_F4xx/STMF4Timers.h"
//#include "Reset.h"

namespace Motate {
	/* System-wide tick counter */
	/*  Inspired by code from Atmel and Arduino.
	 *  Some of which is:   Copyright (c) 2012 Arduino. All right reserved.
	 *  Some of which is:   Copyright (c) 2011-2012, Atmel Corporation. All rights reserved.
	 */

	Timer<SysTickTimerNum> SysTickTimer;
	Timer<WatchDogTimerNum> WatchDogTimer;

	volatile uint32_t Timer<SysTickTimerNum>::_motateTickCount = 0;

    template<> TIM_TypeDef * const  Timer<0>::tc()           { return TIM4; };
    template<> const IRQn_Type   Timer<0>::tcIRQ()        { return TIM4_IRQn; };
    template<> void Timer<0>::_enablePeripheralClock()    { __HAL_RCC_TIM4_CLK_ENABLE();};
    template<> void Timer<0>::interrupt();

    template<> TIM_TypeDef * const  Timer<1>::tc()           { return TIM2; };
    template<> const IRQn_Type   Timer<1>::tcIRQ()        { return TIM2_IRQn; };
    template<> void Timer<1>::_enablePeripheralClock()    { __HAL_RCC_TIM2_CLK_ENABLE();};
    template<> void Timer<1>::interrupt();

    template<> TIM_TypeDef * const  Timer<2>::tc()           { return TIM3; };
    template<> const IRQn_Type   Timer<2>::tcIRQ()        { return TIM3_IRQn; };
    template<> void Timer<2>::_enablePeripheralClock()    { __HAL_RCC_TIM3_CLK_ENABLE();};
    template<> void Timer<2>::interrupt();


} // namespace Motate

extern "C" void SysTick_Handler(void)
{
//	if (sysTickHook)
//		sysTickHook();

//	tickReset();

	Motate::SysTickTimer._increment();

	if (Motate::SysTickTimer.interrupt) {
		Motate::SysTickTimer.interrupt();
	}

    Motate::SysTickTimer._handleEvents();
}


extern "C" {

	void TIM4_IRQHandler(){
		Motate::Timer<0>::interrupt();
		__DSB();      // prevent erroneous recall of this handler due to delayed memory write
	}
	void TIM2_IRQHandler(){
		Motate::Timer<1>::interrupt();
		__DSB();      // prevent erroneous recall of this handler due to delayed memory write
		}
	void TIM3_IRQHandler(){
		Motate::Timer<2>::interrupt();
		__DSB();      // prevent erroneous recall of this handler due to delayed memory write
		}
}
#ifdef __NIJE__
  /* TIM Update event */
  if(__HAL_TIM_GET_FLAG(&hTIM5, TIM_FLAG_UPDATE) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(&hTIM5, TIM_IT_UPDATE) !=RESET)
    {
      __HAL_TIM_CLEAR_IT(&hTIM5, TIM_IT_UPDATE);
      //HAL_TIM_PeriodElapsedCallback(&hTIM5);

//			if(++t&1)
//				ddc.clear();
//			else
//				ddc.set();
			exec_timer_num_isr();
    }
  }
	__DSB();      // prevent erroneous recall of this handler due to delayed memory write

#endif

