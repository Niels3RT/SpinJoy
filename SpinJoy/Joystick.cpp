/*  Joystick.cpp
 *   
 *  Based on the advanced HID library for Arduino: 
 *  https://github.com/NicoHood/HID
 *  Copyright (c) 2014-2015 NicoHood
 * 
 *  Copyright (c) 2020 Mikael Norrgård <http://daemonbite.com>
 *
 *  GNU GENERAL PUBLIC LICENSE
 *  Version 3, 29 June 2007
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *  
 */

#include "Joystick.h"

static const uint8_t _hidReportDescriptor_joy[] PROGMEM = {
  0x05, 0x01,                       // USAGE_PAGE (Generic Desktop)
  0x09, 0x04,                       // USAGE (Joystick) (Maybe change to Joystick? I don't think so but...)
  0xa1, 0x01,                       // COLLECTION (Application)
    0xa1, 0x00,                     // COLLECTION (Physical)
    
      0x05, 0x09,                   // USAGE_PAGE (Button)
      0x19, 0x01,                   // USAGE_MINIMUM (Button 1)
      0x29, 0x0c,                   // USAGE_MAXIMUM (Button 12)
      0x15, 0x00,                   // LOGICAL_MINIMUM (0)
      0x25, 0x01,                   // LOGICAL_MAXIMUM (1)
      0x95, 0x10,                   // REPORT_COUNT (16)
      0x75, 0x01,                   // REPORT_SIZE (1)
      0x81, 0x02,                   // INPUT (Data,Var,Abs)
    
      0x05, 0x01,                   // USAGE_PAGE (Generic Desktop)

      0x09, 0x37,                   // USAGE (Dial)
      0x15, 0x80,                   // LOGICAL_MINIMUM (-128)
      0x25, 0x7F,                   // LOGICAL_MAXIMUM (127)
      0x95, 0x01,                   // REPORT_COUNT (1)
      0x75, 0x08,                   // REPORT_SIZE (8)
      0x81, 0x06,                   // INPUT (Data,Var,Rel)

      0x09, 0x38,                   // USAGE (Wheel)
      0x15, 0x00,                   // LOGICAL_MINIMUM (0)
      0x26, 0xFF, 0x00,             // LOGICAL_MAXIMUM (255)
      0x95, 0x01,                   // REPORT_COUNT (1)
      0x75, 0x08,                   // REPORT_SIZE (8)
      0x81, 0x02,                   // INPUT (Data,Var,Abs)

      0x05, 0x01,                       // USAGE_PAGE (Generic Desktop)
      0x09, 0x01,                       // USAGE (pointer)
      0xa1, 0x00,                       // COLLECTION (Physical) 
        0x09, 0x30,                       // USAGE (X)
        0x09, 0x31,                       // USAGE (Y)
        0x15, 0xff,                       // LOGICAL_MINIMUM (-1)
        0x25, 0x01,                       // LOGICAL_MAXIMUM (1)
        0x95, 0x02,                       // REPORT_COUNT (2)
        0x75, 0x08,                       // REPORT_SIZE (8)
        0x81, 0x02,                       // INPUT (Data,Var,Abs)
      0xc0,                             // END_COLLECTION

    0xc0,                           // END_COLLECTION
  0xc0,                             // END_COLLECTION 
};

Joystick_::Joystick_(void) : PluggableUSBModule(1, 1, epType), protocol(HID_REPORT_PROTOCOL), idle(1)
{
  epType[0] = EP_TYPE_INTERRUPT_IN;
  PluggableUSB().plug(this);
}

int Joystick_::getInterface(uint8_t* interfaceCount)
{
  *interfaceCount += 1; // uses 1
  HIDDescriptor hidInterface = {
    D_INTERFACE(pluggedInterface, 1, USB_DEVICE_CLASS_HUMAN_INTERFACE, HID_SUBCLASS_NONE, HID_PROTOCOL_NONE),
    D_HIDREPORT(sizeof(_hidReportDescriptor_joy)),
    D_ENDPOINT(USB_ENDPOINT_IN(pluggedEndpoint), USB_ENDPOINT_TYPE_INTERRUPT, USB_EP_SIZE, 0x01)
  };
  return USB_SendControl(0, &hidInterface, sizeof(hidInterface));
}

int Joystick_::getDescriptor(USBSetup& setup)
{
  // Check if this is a HID Class Descriptor request
  if (setup.bmRequestType != REQUEST_DEVICETOHOST_STANDARD_INTERFACE) { return 0; }
  if (setup.wValueH != HID_REPORT_DESCRIPTOR_TYPE) { return 0; }

  // In a HID Class Descriptor wIndex cointains the interface number
  if (setup.wIndex != pluggedInterface) { return 0; }

  // Reset the protocol on reenumeration. Normally the host should not assume the state of the protocol
  // due to the USB specs, but Windows and Linux just assumes its in report mode.
  protocol = HID_REPORT_PROTOCOL;

  return USB_SendControl(TRANSFER_PGM, _hidReportDescriptor_joy, sizeof(_hidReportDescriptor_joy));
}

bool Joystick_::setup(USBSetup& setup)
{
  if (pluggedInterface != setup.wIndex) {
    return false;
  }

  uint8_t request = setup.bRequest;
  uint8_t requestType = setup.bmRequestType;

  if (requestType == REQUEST_DEVICETOHOST_CLASS_INTERFACE)
  {
    if (request == HID_GET_REPORT) {
      // TODO: HID_GetReport();
      return true;
    }
    if (request == HID_GET_PROTOCOL) {
      // TODO: Send8(protocol);
      return true;
    }
  }

  if (requestType == REQUEST_HOSTTODEVICE_CLASS_INTERFACE)
  {
    if (request == HID_SET_PROTOCOL) {
      protocol = setup.wValueL;
      return true;
    }
    if (request == HID_SET_IDLE) {
      idle = setup.wValueL;
      return true;
    }
    if (request == HID_SET_REPORT)
    {
    }
  }

  return false;
}

void Joystick_::reset()
{
  _JoystickReport.paddle = 0;
  _JoystickReport.spinner = 0;
  _JoystickReport.buttons = 0;
  _JoystickReport.x = 0;
  _JoystickReport.y = 0;
  this->send();
}

void Joystick_::send() 
{
  USB_Send(pluggedEndpoint | TRANSFER_RELEASE, &_JoystickReport, sizeof(JoystickReport));
}

uint8_t Joystick_::getShortName(char *name)
{
  if(!next) 
  {
    strcpy(name, gp_serial);
    return strlen(name);
  }
  return 0;
}
