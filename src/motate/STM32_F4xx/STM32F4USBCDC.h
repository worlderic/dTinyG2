/*
 * STM32F4USBCDC.h
 *
 *  Created on: Jan 16, 2017
 *      Author: ddilber
 */

#ifndef STM32_F4XX_STM32F4USBCDC_H_
#define STM32_F4XX_STM32F4USBCDC_H_

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
#include "usbd_def.h"


namespace Motate {


/** Enum for the CDC class specific control requests that can be issued by the USB bus host. */
enum CDCClassRequests_t
{
	kSendEncapsulatedCommand = 0x00, /* CDC class-specific request to send an encapsulated command to the device. */
	kGetEncapsulatedResponse = 0x01, /* CDC class-specific request to retrieve an encapsulated command response from the device. */
	kSetLineEncoding         = 0x20, /* CDC class-specific request to set the current virtual serial port configuration settings. */
	kGetLineEncoding         = 0x21, /* CDC class-specific request to get the current virtual serial port configuration settings. */
	kSetControlLineState     = 0x22, /* CDC class-specific request to set the current virtual serial port handshake line states. */
	kSendBreak               = 0x23, /* CDC class-specific request to send a break to the receiver via the carrier channel. */
};

/** Enum for the possible line encoding formats of a virtual serial port. */
enum CDCLineEncodingFormats_t
{
	kCDCLineEncodingOneStopBit          = 0, /* Each frame contains one stop bit. */
	kCDCLineEncodingOneAndAHalfStopBits = 1, /* Each frame contains one and a half stop bits. */
	kCDCLineEncodingTwoStopBits         = 2, /* Each frame contains two stop bits. */
};

/** Enum for the possible line encoding parity settings of a virtual serial port. */
enum CDCLineEncodingParity_t
{
	kCDCParityNone  = 0, /* No parity bit mode on each frame. */
	kCDCParityOdd   = 1, /* Odd parity bit mode on each frame. */
	kCDCParityEven  = 2, /* Even parity bit mode on each frame. */
	kCDCParityMark  = 3, /* Mark parity bit mode on each frame. */
	kCDCParitySpace = 4, /* Space parity bit mode on each frame. */
};

/** Enum for the parameters of SetControlLineState */
enum CDCControlState_t
{
	kCDCControlState_DTR = 0x1, /* Data termianl ready */
	kCDCControlState_RTS = 0x2, /* Ready to send */
};


//Actual implementation of CDC
struct STM32F4USBCDC {
	const uint8_t control_endpoint;
	const uint8_t read_endpoint;
	const uint8_t write_endpoint;
	const uint8_t interface_number;

	std::function<void(bool)> connection_state_changed_callback;
	std::function<void(const size_t &length)> data_available_callback;
	std::function<void(void)> transfer_rx_done_callback;
	std::function<void(void)> transfer_tx_done_callback;

	struct _line_info_t
	{
		uint32_t dwDTERate;
		uint8_t  bCharFormat;
		uint8_t  bParityType;
		uint8_t  bDataBits;

		_line_info_t() :
			dwDTERate(57600),
			bCharFormat(0x00),
			bParityType(0x00),
			bDataBits(0x08)
		{};
	} ATTR_PACKED;

	volatile uint8_t _line_state;
	volatile _line_info_t _line_info;
	volatile bool _line_info_valid = false;
	volatile bool _rx_active;
	volatile char *_last_tx_position;
	volatile char *_last_rx_position;
	volatile uint16_t _rx_length;
	volatile char *_rx_buffer2;
	volatile uint16_t _rx_length2;
	//volatile uint32_t _cached_dwDTERate;

	STM32F4USBCDC(
			const uint8_t new_endpoint_offset,
			const uint8_t new_interface_number
	)
	: control_endpoint(new_endpoint_offset),
	  read_endpoint(new_endpoint_offset+1),
	  write_endpoint(new_endpoint_offset+2),
	  interface_number(new_interface_number),
	  _line_state(0x00),
	  _last_tx_position(0),
	  _last_rx_position(0),
	  _rx_length(0),
	  _rx_buffer2(0),
	  _rx_length2(0),
	  _rx_active(false)
	{};

	STM32F4USBCDC(const STM32F4USBCDC&) = delete;
	STM32F4USBCDC(STM32F4USBCDC&& other) = delete;

	bool TXTransferReady(uint16_t sent_);
	bool RXTransferReady(uint16_t sent_);
	//bool startRXTransfer(char *buffer, const uint16_t length);
	bool startRXTransfer(char *buffer, const uint16_t length, char *buffer2, const uint16_t length2);

	char* getRXTransferPosition() {
		return (char*)_last_rx_position;
	};

	void setRXTransferDoneCallback(const std::function<void()> &callback) {
		transfer_rx_done_callback = callback;
	}

	void setRXTransferDoneCallback(std::function<void()> &&callback) {
		transfer_rx_done_callback = std::move(callback);
	}



	bool startTXTransfer(char *buffer, const uint16_t length);
	char* getTXTransferPosition() {
		return (char*)_last_tx_position;
	};

	void setTXTransferDoneCallback(const std::function<void()> &callback) {
		transfer_tx_done_callback = callback;
	}

	void setTXTransferDoneCallback(std::function<void()> &&callback) {
		transfer_tx_done_callback = std::move(callback);
	}



	void flush() {
		//TODO usb.flush(write_endpoint);
	}

	void flushRead();

	bool isConnected() {
		return _line_state & (0x01 << kCDCControlState_DTR);
	}

	bool getDTR() {
		return _line_state & (0x01 << kCDCControlState_DTR);
	}

	bool getRTS() {
		return _line_state & (0x01 << kCDCControlState_RTS);
	}

	void setConnectionCallback(const std::function<void(bool)> &callback) {
		connection_state_changed_callback = callback;
		if (connection_state_changed_callback /*&& usb.isConnected()*/ && (_line_state & kCDCControlState_DTR)) {
			connection_state_changed_callback(_line_state & kCDCControlState_DTR);
		}
	}

	void setConnectionCallback(std::function<void(bool)> &&callback) {
		connection_state_changed_callback = std::move(callback);
		if (connection_state_changed_callback /*&& usb.isConnected()*/ && (_line_state & kCDCControlState_DTR)) {
			connection_state_changed_callback(_line_state & kCDCControlState_DTR);
		}
	}

	void setDataAvailableCallback(const std::function<void(const size_t &length)> &callback) {
		//TODO usb.enableRXInterrupt(read_endpoint);
		data_available_callback = callback;
	}

	void setDataAvailableCallback(std::function<void(const size_t &length)> &&callback) {
		//TODO usb.enableRXInterrupt(read_endpoint);
		data_available_callback = std::move(callback);
	}

	// This is to be called from USBDeviceHardware when new data is available.
	// It returns if the request was handled or not.
	bool handleDataAvailable(const uint8_t &endpointNum, const size_t &length) {
		if (data_available_callback && (endpointNum == read_endpoint)) {
			data_available_callback(length);
			return true;
		}
		return false;
	}

	// This is to be called from USBDeviceHardware when a transfer is done.
	// It returns if the request was handled or not.
	bool handleTransferDone(const uint8_t &endpointNum, uint8_t* Buf, uint32_t *Len) {
		if(endpointNum == read_endpoint)
		{
			if (transfer_rx_done_callback) {
				transfer_rx_done_callback();
				return true;
			}
		}
		if(endpointNum == write_endpoint)
		{
			_last_tx_position += *Len;
			if (transfer_tx_done_callback) {
				transfer_tx_done_callback();
				return true;
			}
		}
		return false;
	}
	bool handleNonstandardRequest(CDCClassRequests_t req, uint8_t* pbuf, uint16_t length) {

		if (req == kGetLineEncoding) {
			memcpy(pbuf, (void*)&_line_info, sizeof(_line_info_t));
			return true;
		}

		if (req == kSetLineEncoding)
		{
			// if (setup.length() < sizeof(_line_info_t)) { return false; }
			_line_info_valid = true;
			memcpy((char*)&_line_info, pbuf, sizeof(_line_info_t));
			return true;
		}

		if (req == kSetControlLineState)
		{
			USBD_SetupReqTypedef* req = (USBD_SetupReqTypedef*)pbuf;
			uint8_t _old_line_state = _line_state;
			_line_state = req->wValue & 0xFF;

			// If the DTR changed, call connectionStateChanged (if it's defined)
			if ((_old_line_state & kCDCControlState_DTR) != (_line_state & kCDCControlState_DTR)) {
				flush();

				if (connection_state_changed_callback) {
					connection_state_changed_callback(_line_state & kCDCControlState_DTR);
				}
			}
			// Auto-reset into the bootloader is triggered when the port, already open at 1200 bps, is closed.

			// Note that it may be reopened immediately at a different rate.
			// That will *NOT* cancel the reset.

			if (_line_info_valid && (1200 == _line_info.dwDTERate))
			{
				// We check DTR state to determine if host port is open (bit 0 of lineState).
				if (!getDTR()) {
					//TODO Motate::System::reset(1);
				}
				//                        else
				//                            cancelReset();
			}

			return true;
		}

		return false;
	}

	void handleConnectionStateChanged(const bool connected) {
		// We only use this to inform if DISconnects
		// We only show connection when the DTR changes, which is later
		if (connection_state_changed_callback && !connected) {
			connection_state_changed_callback(false);
		}
	}

	// Stub in begin() and end()
	void begin(uint32_t baud_count) {};
	void end(void){};

};
}  // namespace Motate

#endif /* STM32_F4XX_STM32F4USBCDC_H_ */


