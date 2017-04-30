/*
 utility/SamUSB.h - Library for the Motate system
 http://github.com/synthetos/motate/

 Copyright (c) 2015 - 2017 Robert Giseburt
 Copyright (c) 2017 Alden Hart

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

// This goes outside the guard! We need to ensure this happens first.
#include "MotateUSB.h"

#ifndef SAMUSB_ONCE
#define SAMUSB_ONCE

#include <functional> // for std::function<>

#include "STM32F4Common.h"
#include "MotateUSBHelpers.h"
#include "MotateUtilities.h"
#include "MotateUniqueID.h"
#include "MotateDebug.h"

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <cstring> // for memset
#include <algorithm> // for std::min, std::max
#include <functional>
#include <type_traits>

namespace Motate {
/*** ENDPOINT CONFIGURATION ***/


/*** STRINGS ***/

const char16_t *getUSBVendorString(int8_t &length) ATTR_WEAK;
const char16_t *getUSBProductString(int8_t &length) ATTR_WEAK;
const char16_t *getUSBSerialNumberString(int8_t &length) ATTR_WEAK;

// We break the rules here, sortof, by providing a macro shortcut that gets used in userland.
// I apologize, but this also opens it up to later optimization without changing user code.
#define MOTATE_SET_USB_VENDOR_STRING(...)\
		const char16_t MOTATE_USBVendorString[] = __VA_ARGS__;\
		const char16_t *Motate::getUSBVendorString(int8_t &length) {\
			length = sizeof(MOTATE_USBVendorString);\
			return MOTATE_USBVendorString;\
		}

#define MOTATE_SET_USB_PRODUCT_STRING(...)\
		const char16_t MOTATE_USBProductString[] = __VA_ARGS__;\
		const char16_t *Motate::getUSBProductString(int8_t &length) {\
			length = sizeof(MOTATE_USBProductString);\
			return MOTATE_USBProductString;\
		}

#define MOTATE_SET_USB_SERIAL_NUMBER_STRING(...)\
		const char16_t MOTATE_USBSerialNumberString[] = __VA_ARGS__;\
		const char16_t *Motate::getUSBSerialNumberString(int8_t &length) {\
			length = sizeof(MOTATE_USBSerialNumberString);\
			return MOTATE_USBSerialNumberString;\
		}

#define MOTATE_SET_USB_SERIAL_NUMBER_STRING_FROM_CHIPID() \
		char16_t MOTATE_USBSerialNumberString[Motate::UUID_t::length]; \
		const char16_t *Motate::getUSBSerialNumberString(int8_t &length) { \
			const char *uuid = Motate::UUID; \
			uint8_t i = Motate::UUID_t::length; \
			length = i * sizeof(uint16_t); \
			static bool inited = false; \
			if (inited == true) { return MOTATE_USBSerialNumberString; } \
			for (uint8_t j = 0; j < i; j++) { \
				MOTATE_USBSerialNumberString[j] = *uuid++; \
			} \
			return MOTATE_USBSerialNumberString; \
		}

// This needs to be provided in the hardware file
const char16_t *getUSBLanguageString(int8_t &length);

struct alignas(16) USB_DMA_Descriptor {
    enum _commands {  // This enum declaration takes up no space, but is in here for name scoping.
        stop_now        = 0,  // These match those of the SAM3X8n datasheet, but downcased.
        run_and_stop    = 1,
        load_next_desc  = 2,
        run_and_link    = 3
    };

    USB_DMA_Descriptor *next_descriptor;    // The address of the next Descriptor
    char *bufferAddress;                    // The address of the buffer to read/write
    union {
        struct {                                // controlData is a bit field with the settings of the descriptor
            // See SAMS70 datasheet for defintions.
            // Names used there are in the comment.
            _commands command : 2;

            bool end_transfer_enable : 1;                   // END_TR_EN
            bool end_buffer_enable : 1;                     // END_B_EN
            bool end_transfer_interrupt_enable : 1;         // END_TR_IT
            bool end_buffer_interrupt_enable : 1;           // END_BUFFIT
            bool descriptor_loaded_interrupt_enable : 1;    // DESC_LD_IT
            bool bust_lock_enable : 1;                      // BURST_LCK

            uint8_t _unused_1 : 8;

            uint16_t buffer_length : 16;                    // BUFF_LENGTH
        };
        uint32_t CONTROL;
    };


    // FUNCTIONS -- these take no space, and don't have a vtable since there's nothing 'virtual'.


    void setBuffer(char* data, uint16_t len) {
        bufferAddress = data;
        buffer_length = len;
    }

    void setNextDescriptor(USB_DMA_Descriptor *next) {
        next_descriptor = next;
    }
};

enum USBEndpointBufferSettingsFlags_t {
    // null endpoint is all zeros
    kEndpointBufferNull            = 0
};

// USBDeviceHardware actually talks to the hardware, and marshalls data to/from the interfaces.
struct USBDeviceHardware
{


	uint32_t _inited = 0;
	uint32_t config_number = 0;
	bool _address_available = false;
	uint32_t _dma_used_by_endpoint = 0;

	enum USBSetupState_t {
		SETUP                  = 0, // Waiting for a SETUP packet
		DATA_OUT               = 1, // Waiting for a OUT data packet
		DATA_IN                = 2, // Waiting for a IN data packet
		HANDSHAKE_WAIT_IN_ZLP  = 3, // Waiting for a IN ZLP packet
		HANDSHAKE_WAIT_OUT_ZLP = 4, // Waiting for a OUT ZLP packet
		STALL_REQ              = 5, // STALL enabled on IN & OUT packet
	};

	USBSetupState_t setup_state = SETUP;
	USBDevice_t * const proxy;
	static USBDeviceHardware *hw;

	static const uint8_t master_control_endpoint = 0;

	// Init
	USBDeviceHardware(USBDevice_t * const _proxy) : proxy{_proxy}
	{
		hw = this;
		_init();
	};

	// ensure we can't copy or move a USBDeviceHardware
	USBDeviceHardware(const USBDeviceHardware&) = delete;
	USBDeviceHardware(USBDeviceHardware&&) = delete;

	// hold the buffer pointers to setup responses.
	// we hold two pointers and lengths, they are to be sent in order
	// this allows us to store the header and the content seperately
	struct SetupBuffer_t {
		char *buf_addr_0;
		uint16_t length_0;
		char *buf_addr_1;
		uint16_t length_1;
	} _setup_buffer;

	// Utility functions to control various things

	static constexpr uint32_t _get_endpoint_max_nbr() { return (9); };
	static constexpr uint32_t MAX_PEP_NB() { return (_get_endpoint_max_nbr() + 1); };

	// callback for after a control read is done
	std::function<void(void)> _control_read_completed_callback;

	// ** Get 64-, 32-, 16- or 8-bit access to FIFO data register of selected endpoint.
	// ** @param ep Endpoint of which to access FIFO data register
	// ** @param scale Data scale in bits: 64, 32, 16 or 8
	// ** @return Volatile 64-, 32-, 16- or 8-bit data pointer to FIFO data register
	// ** @warning It is up to the user of this macro to make sure that all accesses
	// ** are aligned with their natural boundaries except 64-bit accesses which
	// ** require only 32-bit alignment.
	// ** @warning It is up to the user of this macro to make sure that used HSB
	// ** addresses are identical to the DPRAM internal pointer modulo 32 bits.
	// #define udd_get_endpoint_fifo_access(ep, scale) (((volatile TPASTE2(U, scale) (*)[0x8000 / ((scale) / 8)])UOTGHS_RAM_ADDR)[(ep)])

	// Now, what that crazt thing above does is make an array of volatile 32K buffers, starting at UOTGHS_RAM_ADDR.
	// We'll do that in a more readable format now. Note that we only support byte-level access. This is because we
	// only plan on using it that way.

	typedef volatile uint8_t UOTGHS_FIFO_t[0x8000];
	static constexpr uint32_t UOTGHS_RAM_ADDR_c = 0;
	// static constexpr UOTGHS_FIFO_t UOTGHS_FIFO[10] = (UOTGHS_FIFO_t *)UOTGHS_RAM_ADDR_c;

	static constexpr uint8_t *_dev_fifo(const uint32_t ep) {
		return (uint8_t *)(UOTGHS_RAM_ADDR_c + (ep * 0x8000));
	}

	void _init() {

		//_attach();
	};

	void _attach() {

	};

	bool attach() {
		if (_inited) {
			_attach();
			return true;
		}
		return false;
	};

	void _detach() {
	};
	bool detach() {
		if (_inited) {
			_detach();
			return true;
		}
		return false;
	};

	void initSetup() {
		// In case of abort of IN Data Phase:
		// No need to abort IN transfer (rise TXINI),
		// because it is automatically done by hardware when a Setup packet is received.
		// But the interrupt must be disabled to not generate interrupt TXINI after SETUP reception.


		setup_state = SETUP;

		_setup_buffer = {nullptr, 0, nullptr, 0};
	};

	// This function reads the data out immediately
	int16_t _readFromControl(char* buffer, int16_t length) {
		if (length < 0)
			return -1;

		int16_t to_read = length;
		volatile uint8_t *src = _dev_fifo(0);
		while (to_read-- > 0) {
			*buffer++ = *src++;
		}
		return length;
	};

	void readFromControlThen(char *buffer, uint16_t length, std::function<void(void)> &&callback) {
		_control_read_completed_callback = std::move(callback);
		_setup_buffer = {buffer, length, nullptr, 0};
		setup_state = DATA_OUT;
	}

	void readFromControlThen(char *buffer, uint16_t length, const std::function<void(void)> &callback) {
		_control_read_completed_callback = callback;
		_setup_buffer = {buffer, length, nullptr, 0};
		setup_state = DATA_OUT;
	}

	// This function sets the _setup_buffer to write to the control channel as IN packets come in.
	void writeToControl(char* buffer_0, uint16_t length_0, char* buffer_1 = nullptr, uint16_t length_1 = 0) {
		_setup_buffer = {buffer_0, length_0, buffer_1, length_1};
	};


	uint32_t get_byte_count(uint32_t ep) { return 0; };

	void readSetupPacket(Setup_t &setup) {

	}

	// Request the speed that the device is communicating at. It is unclear at what point this becomes valid,
	// but it's assumed that this would be decided *before* he configuration and descriptors are sent, and
	// the endpoints (other than 0) are configured.
	static const USBDeviceSpeed_t getDeviceSpeed() {
		return kUSBDeviceLowSpeed;

	}
    uint16_t getEndpointSizeFromHardware(const uint8_t &endpoint, const bool otherSpeed) {
        if (endpoint == 0) {
            if (getDeviceSpeed() == kUSBDeviceLowSpeed) {
                return 8;
            }
            return 64;
        }

        // Indicate that we didn't set one...
        return 0;
    };

    EndpointBufferSettings_t getEndpointConfigFromHardware(const uint8_t endpoint) {

        return kEndpointBufferNull;
    };
	 void setAddressAvailable() { _address_available = true; };
	bool checkAndHandleReset() {

		return true;
	};

	bool isConnected() { return 0; }

	bool checkAndHandleVbusChange() {


		return true;
	}

	bool checkAndHandleWakeupSuspend() {

		return false;
	}

	bool checkAndHandleSOF() {

		return false;
	};

	bool checkAndHandleControl() {
		return true;
	}

	bool checkAndHandleEndpoint() {
		return false;

	}
};
}

#endif
//SAMUSB_ONCE
