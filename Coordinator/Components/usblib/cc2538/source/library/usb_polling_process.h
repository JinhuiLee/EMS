#ifndef __USB_POLLING_PROCESS_H__
#define __USB_POLLING_PROCESS_H__

#include "string.h"
#include "usb_firmware_library_headers.h"
#include "usb_cdc.h"
#include "usb_in_buffer.h"
#include "usb_out_buffer.h"

#include "OSAL_Memory.h"   
#include "OSAL.h"
#include "MT_UART.h"
#include "MT.h"
#include "OSAL_Clock.h"
#include "hal_lcd.h"
extern mtOSALSerialData_t  *pMsg;   
extern USB_EPIN_RINGBUFFER_DATA usbCdcInBufferData;
extern USB_EPOUT_RINGBUFFER_DATA usbCdcOutBufferData;
extern uint8_t pInBuffer[64];
extern uint8_t pOutBuffer[64];
extern uint8_t pAppBuffer[64];
extern uint8 usb_start_flag;
extern uint8 usb_end_flag;
extern uint8 USB_Msg_in[128];
extern uint8 dataLen;
void USB_Poll(void);

extern UTCTimeStruct TimeStruct;
extern uint32 sys_secold;
extern uint32 sys_secnew;
extern uint32 sys_timeold;
extern uint32 sys_timenew;

#endif