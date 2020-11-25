/*
    A2600 Paddles/Spinners USB Adapter
    (C) Alexey Melnikov

    Based on project by Mikael Norrgård <mick@daemonbite.com>

    GNU GENERAL PUBLIC LICENSE
    Version 3, 29 June 2007

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/

///////////////// Customizable settings /////////////////////////

// Spinner pulses per revolution
// For arduino shield spinner: 20
//#define SPINNER_PPR 20
#define SPINNER_PPR 600

// upper limit for debounce counter
// set to 1 to disable debouncing
// 16 seems ok, but sometimes still bounces on my setup
#define DEBOUNCE_TOP 32

// comment to disable autofire
#define USE_AUTOFIRE

///////////////// a shortcut ////////////////////////////////////

#define cbi(sfr, bit)     (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit)     (_SFR_BYTE(sfr) |= _BV(bit))

/////////////////////////////////////////////////////////////////

// pins map
const int8_t encpin[2] = {1, 0};        // rotary encoder

////////////////////////////////////////////////////////

#ifndef SPINNER_SENSITIVITY
#if SPINNER_PPR < 50
#define SPINNER_SENSITIVITY 1
#else
#define SPINNER_SENSITIVITY 2
#endif
#endif

// ID for special support in MiSTer
// ATT: 20 chars max (including NULL at the end) according to Arduino source code.
// Additionally serial number is used to differentiate arduino projects to have different button maps!
//const char *gp_serial_spin = "MiSTer-S1 Spinner";
const char *gp_serial = "SpinJoy V0.1";

#include <EEPROM.h>
#include "Joystick.h"
#include "Mouse.h"

Joystick_ Joystick;
SMouse_ SMouse;

uint8_t cnt_stick[4] = { DEBOUNCE_TOP, DEBOUNCE_TOP, DEBOUNCE_TOP, DEBOUNCE_TOP };
uint8_t cnt_btn[12]  = { DEBOUNCE_TOP, DEBOUNCE_TOP, DEBOUNCE_TOP, DEBOUNCE_TOP,
                         DEBOUNCE_TOP, DEBOUNCE_TOP, DEBOUNCE_TOP, DEBOUNCE_TOP,
                         DEBOUNCE_TOP, DEBOUNCE_TOP, DEBOUNCE_TOP, DEBOUNCE_TOP };

uint16_t drvpos;

#ifdef USE_AUTOFIRE
  uint16_t auto_cnt = 0;
  uint16_t auto_max = 1000;
  uint16_t auto_state = 0x001;
  bool auto_modify = false;
#endif

#define SP_MAX ((SPINNER_PPR*4*270UL)/360)
const uint16_t sp_max = SP_MAX;
int32_t sp_clamp = SP_MAX / 2;

void drv_proc(void)
{
  static int8_t prev;
  int8_t a = digitalRead(encpin[0]);
  int8_t b = digitalRead(encpin[1]);

  int8_t spval = (b << 1) | (b ^ a);
  int8_t diff = (prev - spval) & 3;

  if (diff == 1)
  {
    drvpos += 10;
    if (sp_clamp < sp_max) sp_clamp++;
  }
  if (diff == 3)
  {
    drvpos -= 10;
    if (sp_clamp > 0) sp_clamp--;
  }

  prev = spval;
}

void drv0_isr()
{
  drv_proc();
}

void drv1_isr()
{
  drv_proc();
}

void setup()
{
  float snap = .01;
  float thresh = 8.0;

  // set all usable pins to input with pullups
  DDRB  &= ~B01111110;
  PORTB |=  B01111110;
  DDRC  &= ~B01000000;
  PORTC |=  B01000000;
  DDRD  &= ~B10011111;
  PORTD |=  B10011111;
  DDRE  &= ~B01000000;
  PORTE |=  B01000000;
  DDRF  &= ~B11110000;
  PORTF |=  B11110000;

  Joystick.reset();
  SMouse.reset();

  drv_proc();
  drvpos = 0;
  attachInterrupt(digitalPinToInterrupt(encpin[0]), drv0_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encpin[1]), drv0_isr, CHANGE);

#ifdef USE_AUTOFIRE
  // read auto_max from eeprom
  ee_auto_max_read();
#endif
}

JoystickReport joy_rep;
SMouseReport mouse_rep;
const int16_t sp_step = (SPINNER_PPR * 10) / (20 * SPINNER_SENSITIVITY);
void loop()
{
  // LEDs off
  TXLED1; //RXLED1;

  // paddle emulation
  joy_rep.paddle = ((sp_clamp * 255) / sp_max);

  // spinner/wheel rotation
  static uint16_t prev = 0;
  int16_t val = ((int16_t)(drvpos - prev)) / sp_step;
  if (val > 127) val = 127; else if (val < -127) val = -127;
  joy_rep.spinner = val;
  prev += val * sp_step;

  ////// joystick //////
  joy_rep.buttons = 0x000;

  // joystick buttons
  if (!(PIND & (1 << PD7))) cnt_btn[0]  = 0; else if (cnt_btn[0]  < DEBOUNCE_TOP) cnt_btn[0]++;
  if (!(PINE & (1 << PE6))) cnt_btn[1]  = 0; else if (cnt_btn[1]  < DEBOUNCE_TOP) cnt_btn[1]++;
  if (!(PINB & (1 << PB4))) cnt_btn[2]  = 0; else if (cnt_btn[2]  < DEBOUNCE_TOP) cnt_btn[2]++;
  if (!(PINB & (1 << PB5))) cnt_btn[3]  = 0; else if (cnt_btn[3]  < DEBOUNCE_TOP) cnt_btn[3]++;
  if (!(PINF & (1 << PF4))) cnt_btn[4]  = 0; else if (cnt_btn[4]  < DEBOUNCE_TOP) cnt_btn[4]++;
  if (!(PINF & (1 << PF5))) cnt_btn[5]  = 0; else if (cnt_btn[5]  < DEBOUNCE_TOP) cnt_btn[5]++;
  if (!(PINF & (1 << PF6))) cnt_btn[6]  = 0; else if (cnt_btn[6]  < DEBOUNCE_TOP) cnt_btn[6]++;
  if (!(PINF & (1 << PF7))) cnt_btn[7]  = 0; else if (cnt_btn[7]  < DEBOUNCE_TOP) cnt_btn[7]++;
  if (!(PINB & (1 << PB1))) cnt_btn[8]  = 0; else if (cnt_btn[8]  < DEBOUNCE_TOP) cnt_btn[8]++;
  if (!(PINB & (1 << PB3))) cnt_btn[9]  = 0; else if (cnt_btn[9]  < DEBOUNCE_TOP) cnt_btn[9]++;
  if (!(PINB & (1 << PB2))) cnt_btn[10] = 0; else if (cnt_btn[10] < DEBOUNCE_TOP) cnt_btn[10]++;
  if (!(PINB & (1 << PB6))) cnt_btn[11] = 0; else if (cnt_btn[11] < DEBOUNCE_TOP) cnt_btn[11]++;
  // buttons 1 to 11
  for (uint8_t i=1;i<12;i++) {
    joy_rep.buttons >>= 1;
    if (cnt_btn[i] < DEBOUNCE_TOP) joy_rep.buttons |= 0x800;
  }
#ifndef USE_AUTOFIRE
  if (cnt_btn[0] < DEBOUNCE_TOP) joy_rep.buttons |= 0x001;
#else
  // button 0, autofire
  if (cnt_btn[0] < DEBOUNCE_TOP) {
    // autfire active if button 11 is pressed/switched
    if (cnt_btn[11] < DEBOUNCE_TOP) {
      // autofire
      if (auto_cnt < auto_max) {
        auto_cnt++;
        joy_rep.buttons |= auto_state;
      } else {
        auto_cnt = 0;
        auto_state ^= 0x001;
      }
    } else {
      // single fire
      joy_rep.buttons |= 0x001;
    }
  } else {
    auto_cnt = 0;
    auto_state = 0x001;
  }
  // modify autofire frequencey? must be active and button 10 be pressed
  if ((cnt_btn[10] < DEBOUNCE_TOP) && (cnt_btn[11] < DEBOUNCE_TOP)) {
    auto_modify = true;
    auto_max += (val * 4);
    if (auto_max < 400) auto_max = 400;
    if (auto_max > 60000) auto_max = 60000;
  } else {
    if (auto_modify) {
      auto_modify = false;
      ee_auto_max_update();
    }
  }
#endif

  // joystick directions
  if (!(PINC & (1 << PC6))) cnt_stick[0] = 0; else if (cnt_stick[0] < DEBOUNCE_TOP) cnt_stick[0]++;     // up
  if (!(PIND & (1 << PD4))) cnt_stick[1] = 0; else if (cnt_stick[1] < DEBOUNCE_TOP) cnt_stick[1]++;     // down
  if (!(PIND & (1 << PD0))) cnt_stick[2] = 0; else if (cnt_stick[2] < DEBOUNCE_TOP) cnt_stick[2]++;     // right
  if (!(PIND & (1 << PD1))) cnt_stick[3] = 0; else if (cnt_stick[3] < DEBOUNCE_TOP) cnt_stick[3]++;     // left

  if (cnt_stick[0] < DEBOUNCE_TOP) joy_rep.y = -1; else if (cnt_stick[1] < DEBOUNCE_TOP) joy_rep.y = 1; else joy_rep.y = 0;   // up/down
  if (cnt_stick[3] < DEBOUNCE_TOP) joy_rep.x = -1; else if (cnt_stick[2] < DEBOUNCE_TOP) joy_rep.x = 1; else joy_rep.x = 0;   // left/right

  // Only report controller state if it has changed
  if (memcmp(&Joystick._JoystickReport, &joy_rep, sizeof(JoystickReport)))
  {
    Joystick._JoystickReport = joy_rep;
    Joystick.send();
  }

  ////// mouse //////
  mouse_rep.x = val;
  mouse_rep.y = val;

  // Only report controller state if it has changed
  if (memcmp(&SMouse._SMouseReport, &mouse_rep, sizeof(SMouseReport)))
  {
    SMouse._SMouseReport = mouse_rep;
    SMouse.send();
  }
}
#ifdef USE_AUTOFIRE
// read autofire max from eeprom
void ee_auto_max_read() {
  uint16_t a = EEPROM.read(0x00);
  uint16_t b = EEPROM.read(0x02) ^ 0xffff;
  uint16_t c = EEPROM.read(0x04) ^ 0x4e4c;
  // compare
  if (a == b == c) {
    // read auto_max ok, use
    auto_max = a;
  } else {
    // read auto_max nok, use default and update eeprom
    auto_max = 1000;
    ee_auto_max_update();
  }
}

// write autofire max to eeprom
void ee_auto_max_update() {
  EEPROM.update(0x00, auto_max);
  EEPROM.update(0x02, auto_max ^ 0xffff);
  EEPROM.update(0x04, auto_max ^ 0x4e4c);
}
#endif
