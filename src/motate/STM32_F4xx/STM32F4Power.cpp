/*
 * SamPower.h - library for controlling the power on a Atmel sam3x8
 * This file is part of the Motate project, imported from the TinyG project
 *
 * Copyright (c) 2015 Robert Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


#include "STM32F4Power.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

namespace Motate {

    // This is dangerous, let's add another level of namespace in case "use Motate" is in effect.
    namespace System {

        void reset(bool bootloader)
        {
            // Disable all interrupts
        	__disable_irq();

            if (bootloader) {


                // From here flash memory is no longer available.

                // Memory swap needs some time to stabilize
//                for (uint32_t i=0; i<1000000; i++) {
//                    // force compiler to not optimize this -- NOPs don't work!
//                    __asm__ __volatile__("");
//                }
            }

            NVIC_SystemReset();
            while (true);
        }
    }
}
