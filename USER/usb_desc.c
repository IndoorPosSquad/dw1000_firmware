/**
  ******************************************************************************
  * @file    usb_desc.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Descriptors for Virtual Com Port Demo
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"

/* USB Standard Device Descriptor */
const uint8_t Virtual_Com_Port_DeviceDescriptor[VIRTUAL_COM_PORT_SIZ_DEVICE_DESC] = {
	0x12,   /* bLength */
	USB_DEVICE_DESCRIPTOR_TYPE,     /* bDescriptorType */
	0x00,
	0x02,   /* bcdUSB = 2.00 */
	0xFF,   /* bDeviceClass: FF自定义*/
	0x00,   /* bDeviceSubClass */
	0x00,   /* bDeviceProtocol */
	0x40,   /* bMaxPacketSize0 */
	0x83,
	0x04,   /* idVendor = 0x0483 */
	0x41,
	0x57,   /* idProduct = 0x5741 */
	0x00,
	0x02,   /* bcdDevice = 2.00 */
	1,              /* Index of string descriptor describing manufacturer */
	2,              /* Index of string descriptor describing product */
	3,              /* Index of string descriptor describing the device's serial number */
	0x01    /* bNumConfigurations */
};
const uint8_t Virtual_Com_Port_ConfigDescriptor[VIRTUAL_COM_PORT_SIZ_CONFIG_DESC] = {
	/*Configuration Descriptor*/
	0x09,   /* bLength: Configuration Descriptor size */
	USB_CONFIGURATION_DESCRIPTOR_TYPE,      /* bDescriptorType: Configuration 9+9+7+7+7*/
	VIRTUAL_COM_PORT_SIZ_CONFIG_DESC,       /* wTotalLength:no of returned bytes */
	0x00,
	0x01,   /* bNumInterfaces: 1 interface */
	0x01,   /* bConfigurationValue: Configuration value */
	0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
	0xC0,   /* bmAttributes: self powered */
	0x32,   /* MaxPower 100 mA */
// 9
	/*interface descriptor*/
	0x09,   /* bLength: Endpoint Descriptor size */
	USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: */
	0x00,   /* bInterfaceNumber: Number of Interface */
	0x00,   /* bAlternateSetting: Alternate setting */
	0x03,   /* bNumEndpoints: 3 endpoints used */
	0x00,   /* bInterfaceClass: CDC */
	0x00,   /* bInterfaceSubClass: */
	0x00,   /* bInterfaceProtocol: */
	0x00,   /* iInterface: */
// 18
	/*Endpoint 1 IN Descriptor*/
	0x07,   /* bLength: Endpoint Descriptor size */
	USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
	0x81,   /* bEndpointAddress: (IN1) */
	0x02,   /* bmAttributes: Bulk */
	0x40,   /* wMaxPacketSize: */
	0x00,
	0x00,   /* bInterval */
// 25
	// /*Endpoint 2 IN Descriptor*/
	// 0x07,   /* bLength: Endpoint Descriptor size */
	// USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
	// 0x82,   /* bEndpointAddress: (IN2) */
	// 0x03,   /* bmAttributes: Interrupt */
	// 0x02,   /* wMaxPacketSize: */
	// 0x00,
	// 0x02,   /* bInterval: 2ms */

	/*Endpoint 2 OUT Descriptor*/
	0x07,   /* bLength: Endpoint Descriptor size */
	USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
	0x82,   /* bEndpointAddress: (IN2) */
	0x03,   /* bmAttributes: Bulk */
	0x02,   /* wMaxPacketSize: */
	0x00,
	0x02,   /* bInterval: 2ms */
// 32
	/*Endpoint 3 OUT Descriptor*/
	0x07,   /* bLength: Endpoint Descriptor size */
	USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
	0x03,   /* bEndpointAddress: (OUT3) */
	0x02,   /* bmAttributes: Bulk */
	0x40,   /* wMaxPacketSize: */
	0x00,
	0x00    /* bInterval */
//39
};
/* USB String Descriptors */
const uint8_t Virtual_Com_Port_StringLangID[VIRTUAL_COM_PORT_SIZ_STRING_LANGID] = {
	VIRTUAL_COM_PORT_SIZ_STRING_LANGID,
	USB_STRING_DESCRIPTOR_TYPE,
	0x09,
	0x04 /* LangID = 0x0409: U.S. English */
};

const uint8_t Virtual_Com_Port_StringVendor[VIRTUAL_COM_PORT_SIZ_STRING_VENDOR] = {
	VIRTUAL_COM_PORT_SIZ_STRING_VENDOR,     /* Size of Vendor string */
	USB_STRING_DESCRIPTOR_TYPE,             /* bDescriptorType*/
	/* Manufacturer: "STMicroelectronics" */
	'B', 0, 'U', 0, 'P', 0, 'T', 0, 'U', 0, 'W', 0, 'B', 0
};

const uint8_t Virtual_Com_Port_StringProduct[VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT] = {
	VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT,          /* bLength */
	USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
	/* Product name: "STM32 Virtual COM Port" */
	'D', 0, 'W', 0, 'M', 0, '1', 0, 'k', 0, ' ', 0, 'C', 0, 'o', 0,
	'n', 0, 't', 0, 'r', 0, 'o', 0, 'l', 0, 'l', 0, 'e', 0, 'r', 0
};

uint8_t Virtual_Com_Port_StringSerial[VIRTUAL_COM_PORT_SIZ_STRING_SERIAL] = {
	VIRTUAL_COM_PORT_SIZ_STRING_SERIAL,           /* bLength */
	USB_STRING_DESCRIPTOR_TYPE,                   /* bDescriptorType */
	'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0
};
// done
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
