/*
 * STM32F4USBCDC.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: ddilber
 */

#include "STM32F4USBCDC.h"
#include <string.h>

using Motate::STM32F4USBCDC;

STM32F4USBCDC SerialUSB(0,0);

extern "C"
{

#include "stm32f4xx_hal.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_CDC_LineCodingTypeDef LineCoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };


uint8_t UserRxBuffer[64];

/* UART handler declaration */
UART_HandleTypeDef UartHandle;
/* TIM handler declaration */
TIM_HandleTypeDef  TimHandle;
/* USB handler declaration */
extern USBD_HandleTypeDef  USBD_Device;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init(void);
static int8_t CDC_Itf_DeInit(void);
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_Itf_TxDone(uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_CDC_fops =
{
  CDC_Itf_Init,
  CDC_Itf_DeInit,
  CDC_Itf_Control,
  CDC_Itf_Receive,
  CDC_Itf_TxDone
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_Itf_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Init(void)
{

  /*##-5- Set Application Buffers ############################################*/
  //USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBuffer, 0);
  USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer);


  //USBD_CDC_ReceivePacket(&USBD_Device);

  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_DeInit(void)
{
  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Control (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

    /*******************************************************************************/
    /* Line Coding Structure                                                       */
    /*-----------------------------------------------------------------------------*/
    /* Offset | Field       | Size | Value  | Description                          */
    /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
    /* 4      | bCharFormat |   1  | Number | Stop bits                            */
    /*                                        0 - 1 Stop bit                       */
    /*                                        1 - 1.5 Stop bits                    */
    /*                                        2 - 2 Stop bits                      */
    /* 5      | bParityType |  1   | Number | Parity                               */
    /*                                        0 - None                             */
    /*                                        1 - Odd                              */
    /*                                        2 - Even                             */
    /*                                        3 - Mark                             */
    /*                                        4 - Space                            */
    /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
    /*******************************************************************************/
    case CDC_SET_LINE_CODING:
    case CDC_GET_LINE_CODING:
    case CDC_SET_CONTROL_LINE_STATE:
  	  SerialUSB.handleNonstandardRequest((Motate::CDCClassRequests_t)cmd, pbuf, length);
      break;

/*
  case CDC_SET_LINE_CODING:
    LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                            (pbuf[2] << 16) | (pbuf[3] << 24));
    LineCoding.format     = pbuf[4];
    LineCoding.paritytype = pbuf[5];
    LineCoding.datatype   = pbuf[6];

    //ComPort_Config();
    break;

  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(LineCoding.bitrate);
    pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
    pbuf[4] = LineCoding.format;
    pbuf[5] = LineCoding.paritytype;
    pbuf[6] = LineCoding.datatype;
    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;
*/
  case CDC_SEND_BREAK:
     /* Add your code here */
    break;

  default:
    break;
  }

  return (USBD_OK);
}


/**
  * @brief  CDC_Itf_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Receive(uint8_t* Buf, uint32_t *Len)
{
	uint32_t len = *Len;

#ifdef LOOPBACK
	//LOopback
	while(len--)
	{
			UserTxBuffer[UserTxBufPtrIn++] = *Buf++;
			/* To avoid buffer overflow */
			if(UserTxBufPtrIn == APP_RX_DATA_SIZE)
			{
				UserTxBufPtrIn = 0;
			}
	}
	USBD_CDC_ReceivePacket(&USBD_Device);
#else

	if(len > SerialUSB._rx_length)
	{
		memcpy((void*)SerialUSB._last_rx_position, Buf, SerialUSB._rx_length);
		memcpy((void*)SerialUSB._rx_buffer2, &Buf[SerialUSB._rx_length], len - SerialUSB._rx_length);
		SerialUSB._last_rx_position = SerialUSB._rx_buffer2 + (len - SerialUSB._rx_length);
	}
	else
	{
		memcpy((void*)SerialUSB._last_rx_position, Buf, len);
		SerialUSB._last_rx_position += len;
	}


	if (SerialUSB.transfer_rx_done_callback) {
		SerialUSB.transfer_rx_done_callback();
	}

#endif


  //HAL_UART_Transmit_DMA(&UartHandle, Buf, *Len);
  return (USBD_OK);
}



/**
  * @brief  CDC_Itf_TxDone
  *         Data send over USB IN endpoint
  * @param  Buf: transmitted Buffer
  * @param  Len: Number of data transmitted (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_TxDone(uint8_t* Buf, uint32_t *Len)
{
	uint32_t len = *Len;

	SerialUSB._last_tx_position += len;

	if (SerialUSB.transfer_tx_done_callback) {
		SerialUSB.transfer_tx_done_callback();
	}

  return (USBD_OK);
}
}

bool STM32F4USBCDC::startRXTransfer(char *buffer, const uint16_t length, char *buffer2, const uint16_t length2)
{
    //_rx_dma_descriptor.setBuffer(buffer, length);
    // // DON'T allow the DMA transfer to be stopped if the buffer runs out
    // // IOW, don't stop reading when a packet doesn't fill the buffer.
    // _rx_dma_descriptor.end_buffer_enable = false;

	_last_rx_position = buffer;
	_rx_length = length;
	_rx_buffer2 = buffer2;
	_rx_length2 = length2;

	USBD_CDC_ReceivePacket(&USBD_Device);

    return true;//usb.transfer(read_endpoint, _rx_dma_descriptor);
};

/*
bool STM32F4USBCDC::startRXTransfer(char *buffer, const uint16_t length)
{


 	int l=length;
	char *buffer_save = buffer;

	_last_rx_position = buffer;

	return true;//TODO usb.transfer(read_endpoint, _rx_dma_descriptor);
}
*/
bool STM32F4USBCDC::TXTransferReady(uint16_t sent_)
{
/*
	  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
	  if(hcdc->TxState == 0)
	  {
		  _last_tx_position += sent_;
		  return true;
	  }
	  else
*/
	  {
		  return false;
	  }
}
bool STM32F4USBCDC::RXTransferReady(uint16_t read_)
{
	return true;
}

bool STM32F4USBCDC::startTXTransfer(char *buffer, const uint16_t length)
{
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)USBD_Device.pClassData;
	while(hcdc->TxState)
	{

	}
	USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t*)buffer, length);

	if(USBD_CDC_TransmitPacket(&USBD_Device) == USBD_OK)
	{
		_last_tx_position = buffer;
	}

	SerialUSB._last_tx_position += length;

	if (SerialUSB.transfer_tx_done_callback) {
		SerialUSB.transfer_tx_done_callback();
	}

	return true;
};

void STM32F4USBCDC::flushRead()
{
	_last_rx_position = 0;
}
