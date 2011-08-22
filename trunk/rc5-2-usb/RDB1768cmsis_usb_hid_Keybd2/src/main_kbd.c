//

//*****************************************************************************
//   +--+       
//   | ++----+   
//   +-++    |  
//     |     |  
//   +-+--+  |   
//   | +--+--+  
//   +----+    Code Red Technologies Ltd. 
//
// 
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.

/*
	LPCUSB, an USB device driver for LPC microcontrollers	
	Copyright (C) 2006 Bertrik Sikken (bertrik@sikken.nl)

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright
	   notice, this list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright
	   notice, this list of conditions and the following disclaimer in the
	   documentation and/or other materials provided with the distribution.
	3. The name of the author may not be used to endorse or promote products
	   derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
	IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
	OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
	IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
	NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
	THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


/**********************************
 * CodeRed - extended version of LPCUSB HID example to provide PC mouse
 **********************************/

//CodeRed
//Added ref to stdio.h to pull in semihosted printf rather than using serial
#include <stdio.h>   		// Uses RedLib (Semihost) in Debug build


#include "stdint.h"			// included for uint8_t
#include "usbapi.h"
#include "usbdebug.h"

#include "LPC17xx.h"


//CRP = Code Read Protection
#include <cr_section_macros.h>
#include <NXP/crp.h>


#include "RC5.h"


// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;


#define INTR_IN_EP		0x81

#define MAX_PACKET_SIZE	64

#define LE_WORD(x)		((x)&0xFF),((x)>>8)

// CAS
#define REPORT_SIZE			8


// CAS
typedef struct {

    uint8_t Modifier;			/// ..
    uint8_t Reserved;			/// ..
    uint8_t Key_1;				/// ..
    uint8_t Key_2;				/// ..
    uint8_t Key_3;				/// ..
    uint8_t Key_4;				/// ..
    uint8_t Key_5;				/// ..
    uint8_t Key_6;				/// ..

} __attribute__ ((packed)) HIDDKbdInputReport; // GCC

HIDDKbdInputReport KbdInputReport;


static uint8_t	abClassReqData[4];
static int	_iIdleRate = 0;
static int	_iFrame = 0;

// CAS - Kbd Descriptor
static uint8_t abReportDesc[] = {
		0x05, 0x01,			// Usage_Page (Generic Desktop)
		0x09, 0x06,			// Usage (Keyboard)
		0xA1, 0x01,			// Collection (Application)
		0x05, 0x07,			//   Usage page (Key Codes)
		0x19, 0xE0,			//   Usage_Minimum (224)
		0x29, 0xE7,			//   Usage_Maximum (231)
		0x15, 0x00,			//   Logical_Minimum (0)
		0x25, 0x01,			//   Logical_Maximum (1)
		0x75, 0x01,			//   Report_Size (1)
		0x95, 0x08,			//   Report_Count (8)
		0x81, 0x02,			//   Input (Data,Var,Abs) = Modifier Byte
		0x95, 0x01,			//   Report Count (1)
		0x75, 0x08,			//   Report Size (8)
		0x81, 0x01,			//	 Input (Constant), ;Reserved byte
		0x95, 0x05,			//	 Report Count (5),
		0x75, 0x01,			//	 Report Size (1),
		0x05, 0x08,			//   Usage Page (LEDs)
		0x19, 0x01,			//	 Usage Minimum (1),
		0x29, 0x05,			// 	 Usage Maximum (5),
		0x91, 0x02,			//	 Output (Data, Variable, Absolute), ;LED report
		0x95, 0x01,			//	 Report Count (1),
		0x75, 0x03,			//	 Report Size (3),
		0x91, 0x01,			//	 Output (Constant), ;LED report padding
		0x95, 0x06,			//	 Report Count (6),
		0x75, 0x08,			//	 Report Size (8),
		0x15, 0x00,			//	 Logical Minimum (0),
		0x25, 0x65,			//	 Logical Maximum(101),
		0x05, 0x07,			//	 Usage Page (Key Codes),
		0x19, 0x00,			//	 Usage Minimum (0),
		0x29, 0x65,			//	 Usage Maximum (101),
		0x81, 0x00,			//   Input (Data, Array), ;Key arrays (6 bytes)
		0xC0				// End_Collection
};
		
static const uint8_t abDescriptors[] = {

/* Device descriptor */
		0x12,              		// Lenght = 0x12
		DESC_DEVICE,       		// Constant
		LE_WORD(0x0110),		// bcdUSB				// CAS MADOFICADO!
		0x00,              		// bDeviceClass 		// The interface descriptor specifies the class and the function doesn’t use an interface association descriptor.
		0x00,              		// bDeviceSubClass		// If bDeviceClass is 00h, bDeviceSubclass must be 00h.
		0x00,              		// bDeviceProtocol		// ¿?
		MAX_PACKET_SIZE0,  		// bMaxPacketSize		// For USB 2.0, the maximum packet size equals the field’s value and must be 8 for low speed; 8, 16, 32, or 64 for full speed; and 64 for high speed.
		LE_WORD(0xFDFF),		// idVendor				// Vendor ID
		LE_WORD(0x0021),		// idProduct			// Product ID
		LE_WORD(0x0100),		// bcdDevice			// Device’s release number in BCD format.
		0x01,              		// iManufacturer		// Index that points to a string ...
		0x02,              		// iProduct
		0x03,              		// iSerialNumber
		0x01,              		// bNumConfigurations	// Equals the number of configurations the device supports at the current operating speed.

// configuration
		0x09,					// Lenght = 0x09
		DESC_CONFIGURATION,		// Constant
		LE_WORD(0x22),  		// wTotalLength 		// The number of bytes in the configuration descriptor and all of its subordinate descriptors
														// 0x22 = 34. Len(ConfDesc=9) + Len(IntDesc=9) + Len(HID_Desc=9) + Len(EP_Desc=7) -> 9+9+9+7=34 ???
		0x01,  					// bNumInterfaces
		0x01,  					// bConfigurationValue
		0x00,  					// iConfiguration
		0x80,  					// bmAttributes			// Bus powered
		0x32,  					// bMaxPower

// interface
		0x09,   				// Lenght = 0x09
		DESC_INTERFACE,			// Constant
		0x00,  		 			// bInterfaceNumber		// The default interface is 00h.
		0x00,   				// bAlternateSetting	// Identifies the default interface setting or an alternate setting.
		0x01,   				// bNumEndPoints		// Number of endpoints the interface supports in addition to endpoint zero.
		0x03,   				// bInterfaceClass 		// HID = 0x03
		0x00,   				// bInterfaceSubClass	// Similar to bDeviceSubClass in the device descriptor, but for devices with a class defined by the interface.
		0x00,   				// bInterfaceProtocol	// Similar to bDeviceProtocol in the device descriptor, but for devices whose class is defined by the interface.
		0x00,   				// iInterface			// An index to a string that describes the interface (0 no string).

// HID descriptor
		0x09, 					//
		DESC_HID_HID, 			// bDescriptorType		// HID Class
		LE_WORD(0x0110),		// bcdHID
		0x00,   				// bCountryCode
		0x01,   				// bNumDescriptors = report
		DESC_HID_REPORT,   		// bDescriptorType
		LE_WORD(sizeof(abReportDesc)),

// EP descriptor
		0x07,   				// Lenght = 0x09
		DESC_ENDPOINT,   		// Constant
		INTR_IN_EP,				// bEndpointAddress		// 0x81 = 1000 0001 = Endpoint 1, Direction IN (Data from Endpoint to Host)
		0x03,   				// bmAttributes			// Type of transfer = Interrupt
		LE_WORD(MAX_PACKET_SIZE),// wMaxPacketSize
		10,						// bInterval

// string descriptors
	0x04,
	DESC_STRING,
	LE_WORD(0x0409),

	// manufacturer string
	0x0E,
	DESC_STRING,
	'L', 0, 'P', 0, 'C', 0, 'U', 0, 'S', 0, 'B', 0,

	// product string
	0x12,
	DESC_STRING,
	'P', 0, 'r', 0, 'o', 0, 'd', 0, 'u', 0, 'c', 0, 't', 0, 'X', 0,

	// serial number string
	0x12,
	DESC_STRING,
	'D', 0, 'E', 0, 'A', 0, 'D', 0, 'C', 0, '0', 0, 'D', 0, 'E', 0,
	
	// terminator
	0
};


/*************************************************************************
	HandleClassRequest
	==================
		HID class request handler
		
**************************************************************************/
static Bool HandleClassRequest(TSetupPacket *pSetup, int *piLen, uint8_t **ppbData)
{
	uint8_t	*pbData = *ppbData;

	switch (pSetup->bRequest) {
	
   	// get_idle
	case HID_GET_IDLE:
		DBG("GET IDLE, val=%X, idx=%X\n", pSetup->wValue, pSetup->wIndex);
		pbData[0] = (_iIdleRate / 4) & 0xFF;
		*piLen = 1;
		break;

	// set_idle:
	case HID_SET_IDLE:
		DBG("SET IDLE, val=%X, idx=%X\n", pSetup->wValue, pSetup->wIndex);
		_iIdleRate = ((pSetup->wValue >> 8) & 0xFF) * 4;
		break;

	default:
		DBG("Unhandled class %X\n", pSetup->bRequest);
		return FALSE;
	}
	return TRUE;
}



/*************************************************************************
	HIDHandleStdReq
	===============
		Standard request handler for HID devices.
		
	This function tries to service any HID specific requests.
		
**************************************************************************/
static Bool HIDHandleStdReq(TSetupPacket *pSetup, int *piLen, uint8_t **ppbData)
{
	uint8_t	bType, bIndex;

	if ((pSetup->bmRequestType == 0x81) &&			// standard IN request for interface
		(pSetup->bRequest == REQ_GET_DESCRIPTOR)) {	// get descriptor
		
		bType = GET_DESC_TYPE(pSetup->wValue);
		bIndex = GET_DESC_INDEX(pSetup->wValue);
		switch (bType) {

		case DESC_HID_REPORT:
			// report
			*ppbData = abReportDesc;
			*piLen = sizeof(abReportDesc);
			break;

		case DESC_HID_HID:
		case DESC_HID_PHYSICAL:
		default:
		    // search descriptor space
		    return USBGetDescriptor(pSetup->wValue, pSetup->wIndex, piLen, ppbData);
		}
		
		return TRUE;
	}
	return FALSE;
}

// CAS - rewritten to provide real mouse reports
/*
*/
static void HandleFrame(uint16_t wFrame)
{
	static int iCount;

	_iFrame++;
	if ((_iFrame > 100)) {
		iCount++;
		USBHwEPWrite(INTR_IN_EP, (void *)&KbdInputReport, REPORT_SIZE);
		_iFrame = 0;
	}
}

// CAS - initial kbd report
void HIDDKbdInputReport_Initialize(HIDDKbdInputReport *report)
{
    report->Modifier = 0;
    report->Reserved = 0;
    report->Key_1 = 0;
    report->Key_2 = 0;
    report->Key_3 = 0;
    report->Key_4 = 0;
    report->Key_5 = 0;
    report->Key_6 = 0;
}




/*************************************************************************
	main
	====
**************************************************************************/
int main(void)
{
	// USB: Init
	// CAS -> Initialization of KbdInputReport to initial values (0,0,0,0,..)
	HIDDKbdInputReport_Initialize (&KbdInputReport);

	// CAS -> Variable used to simulate the pressing of Letter A (0x04)
	int i;

	DBG("Initialising USB stack\n");
	
	// CAS -> Variable used to simulate the pressing of Letter A (0x04)
	// 	USBInit()
	//		USBHwInit()										HW Init
	//			-> Set Pins for USB, AHBClock, ACK generate interrupts
	//		USBHwRegisterDevIntHandler(HandleUsbReset)
	//			-> Define un handler (HandleUsbReset).
	//
	// initialize stack
	USBInit();
	
	// register device descriptors
	USBRegisterDescriptors(abDescriptors);

	// register HID standard request handler
	USBRegisterCustomReqHandler(HIDHandleStdReq);

	// register class request handler
	USBRegisterRequestHandler(REQTYPE_TYPE_CLASS, HandleClassRequest, abClassReqData);

	// register endpoint (2?). Interrupt In Endpoint
	USBHwRegisterEPIntHandler(INTR_IN_EP, NULL);

	// register frame handler
	USBHwRegisterFrameHandler(HandleFrame);
	
	DBG("Starting USB communication\n");


	// connect to bus
	USBHwConnect(TRUE);

	//RC5: Init
	GPIO_SetDir(0,1<<4,1);
	GPIO_SetDir(0,1<<5,1);

	GPIO_SetValue(0,1<<5);
	GPIO_SetValue(0,1<<4);


	RC5_Init();

	// main() Loop:


	// call USB interrupt handler continuously
	
	// CodeRed - either read mouse, or provide "fake mouse readings"
	while (1)
	{

		for (i=0; i<10;  i++) {


			// RC5
		    if (RC5_flag)                      // wait for RC5 code
		    	    {

		      if((RC5_System==0)&&(RC5_Command==1)){
		      GPIO_ClearValue(0,1<<4);
		      for(i=0;i<1000000;i++);
		      GPIO_SetValue(0,1<<4);
		      for(i=0;i<1000000;i++);
		      }

		      if((RC5_System==0)&&(RC5_Command==5)){
		      GPIO_ClearValue(0,1<<5);
		      for(i=0;i<1000000;i++);
		      GPIO_SetValue(0,1<<5);
		      for(i=0;i<1000000;i++);
		      }

		      RC5_flag = 0;
		      printf( "RC5 =  %d   %d\n", RC5_System, RC5_Command);
		    }
		    // RC5

		  USBHwISR();
		  if (i < 5) {
			  KbdInputReport.Key_1 = 0x04;
		  }
		  else {
			  KbdInputReport.Key_1 = 0x00;
		  }
		}
		
	}
	
	return 0;
}



