
#include "usb_polling_process.h"


uint8 usb_start_flag = 0;
uint8 usb_end_flag = 0;
char  Display_lcdString[10];
uint8* usb_alloc_buf;

/*************************************************************************************************
 * @fn      USB_Poll
 *
 * @brief   This routine simulate polling from USB port
 *
 * @param   void
 *
 * @return  void
 *************************************************************************************************/
void USB_Poll(void)	
{	
    usbCdcProcessEvents();
    //
    // Implement COM-port loopback
    //
    uint16_t count = 0;
    count = usbibufGetMaxPushCount(&usbCdcInBufferData);

    uint16_t maxPopCount = 0;
    maxPopCount = usbobufGetMaxPopCount(&usbCdcOutBufferData);

    if (count > maxPopCount)
    {
        count = maxPopCount;
    }

    if (count)
    {
        //////////////////////////////////////////////////////////////////////////////////
        if(usb_start_flag == 0) {
            usb_alloc_buf = (uint8*) osal_mem_alloc(sizeof(uint8)*200);
        }
        //////////////////////////////////////////////////////////////////////////////////
        usbobufPop(&usbCdcOutBufferData, pAppBuffer, count); // receive function
        
        uint16 index = 0;
        for(index = 0; index < count; index++)
            *(usb_alloc_buf + index + 64 * usb_start_flag) = *(pAppBuffer + index);
            //usb_alloc_buf[index + 64 * usb_start_flag] = *(pAppBuffer + index);
        usb_start_flag++;                        
        usb_end_flag = 0;
        
        uint8 data_len = *(usb_alloc_buf + 10);
        for(int i = 0; i < data_len + 13; i++)
            USB_Msg_in[i] = *(usb_alloc_buf + i);
        
        /*
        usbobufPop(&usbCdcOutBufferData, pAppBuffer, count); // receive function
        
        uint16 index = 0;
        for(index = 0; index < count; index++)
            USB_Msg_in[index + 64 * usb_start_flag] = *(pAppBuffer + index);
        usb_start_flag++;                        
        usb_end_flag = 0;
        */
    }
    else if (!count && usb_start_flag)
    {
        
        //osal_mem_free(usb_alloc_buf);
        ////////////////////////////////////////////////////////////////////////////////////////
        usb_start_flag = 0;
        usb_end_flag = 1;         
        dataLen = USB_Msg_in[10];
        //usbibufPush(&usbCdcInBufferData, USB_Msg_in, 128);
        pMsg = (mtOSALSerialData_t *)osal_msg_allocate( 50 );
        if (pMsg)
        {
            /* Fill up what we can */
            pMsg->hdr.event = CMD_USB_MSG;
            pMsg->msg = (uint8 *)(pMsg + 1);
            pMsg->msg[0] = 1 ;                                  
        }
        osal_msg_send(App_TaskID, (uint8 *)pMsg );
        osal_msg_deallocate((uint8*)pMsg );
    }
    else
    {
        sys_timenew = osal_GetSystemClock();
        sys_secnew = sys_secold + (uint32)((float)(sys_timenew - sys_timeold) / 1000);
        uint8 oldseconds = TimeStruct.seconds;
        osal_ConvertUTCTime(&TimeStruct , sys_secnew);
        
        uint8 newseconds = TimeStruct.seconds;
        
        if(oldseconds != newseconds)
        {
#ifdef LCD_SUPPORTED
            sprintf((char *)Display_lcdString, "%d %d %d %d %d %d", 
                    ((TimeStruct.month == 12)? (TimeStruct.year + 1) : TimeStruct.year), 
                    ((TimeStruct.month == 12)? 1: (TimeStruct.month + 1)),
                    TimeStruct.day + 1, TimeStruct.hour, TimeStruct.minutes, TimeStruct.seconds);
            HalLcdWriteString( Display_lcdString, HAL_LCD_LINE_7 );
#endif
        }
    }

}