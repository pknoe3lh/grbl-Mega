/*
  cpu_map.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The cpu_map.h files serve as a central pin mapping selection file for different 
   processor types or alternative pin layouts. This version of Grbl supports only the 
   Arduino Mega2560. */

#ifndef cpu_map_h
#define cpu_map_h


#ifdef CPU_MAP_DUE_INITIAL // (Arduino Mega 2560) Working @EliteEng

  #define X_STEP_BIT    26 // MEGA2560 Digital Pin 26
  #define Y1_STEP_BIT   24 // MEGA2560 Digital Pin 24
  #define Y2_STEP_BIT   25 // MEGA2560 Digital Pin 25
  #define Z_STEP_BIT    27 // MEGA2560 Digital Pin 27
  #define A_STEP_BIT    29 // MEGA2560 Digital Pin 29

  #define X_DIRECTION_BIT   32 // MEGA2560 Digital Pin 32
  #define Y1_DIRECTION_BIT  30 // MEGA2560 Digital Pin 30
  #define Y2_DIRECTION_BIT  31 // MEGA2560 Digital Pin 31
  #define Z_DIRECTION_BIT   33 // MEGA2560 Digital Pin 33

  #define STEPPERS_DISABLE_BIT   4 // MEGA2560 Digital Pin 4

  #define X_LIMIT_BIT     12 // MEGA2560 Digital Pin 12
  #define Y1_LIMIT_BIT    10 // MEGA2560 Digital Pin 10
  #define Y2_LIMIT_BIT    11 // MEGA2560 Digital Pin 11
  #define Z_LIMIT_BIT     13 // MEGA2560 Digital Pin 13

/*
  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR      DDRH
  #define SPINDLE_ENABLE_PORT     PORTH
  #define SPINDLE_ENABLE_BIT      3 // MEGA2560 Digital Pin 6
  #define SPINDLE_DIRECTION_DDR   DDRE
  #define SPINDLE_DIRECTION_PORT  PORTE
  #define SPINDLE_DIRECTION_BIT   3 // MEGA2560 Digital Pin 5

  // Define flood and mist coolant enable output pins.
  #define COOLANT_FLOOD_DDR   DDRH
  #define COOLANT_FLOOD_PORT  PORTH
  #define COOLANT_FLOOD_BIT   5 // MEGA2560 Digital Pin 8
  #define COOLANT_MIST_DDR    DDRH
  #define COOLANT_MIST_PORT   PORTH
  #define COOLANT_MIST_BIT    6 // MEGA2560 Digital Pin 9
*/
  // Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
  #define CONTROL_RESET_BIT         62  // MEGA2560 Analog Pin 8
  #define CONTROL_FEED_HOLD_BIT     63  // MEGA2560 Analog Pin 9
  #define CONTROL_CYCLE_START_BIT   64  // MEGA2560 Analog Pin 10
  #define CONTROL_SAFETY_DOOR_BIT   61  // 65 DISABELD maybe use later ;-)  // MEGA2560 Analog Pin 11

  // Define probe switch input pin.
  #define PROBE_BIT       69  // MEGA2560 Analog Pin 15

  /*
  // Advanced Configuration Below You should not need to touch these variables
  // Set Timer up to use TIMER4B which is attached to Digital Pin 7
  #define SPINDLE_PWM_MAX_VALUE     1024.0 // Translates to about 1.9 kHz PWM frequency at 1/8 prescaler
  #define SPINDLE_TCCRA_REGISTER		TCCR4A
  #define SPINDLE_TCCRB_REGISTER		TCCR4B
  #define SPINDLE_OCR_REGISTER	  	OCR4B
  #define SPINDLE_COMB_BIT			    COM4B1
  // 1/8 Prescaler, 16-bit Fast PWM mode
  #define SPINDLE_TCCRA_INIT_MASK ((1<<WGM40) | (1<<WGM41))
  #define SPINDLE_TCCRB_INIT_MASK ((1<<WGM42) | (1<<WGM43) | (1<<CS41)) 
  #define SPINDLE_OCRA_REGISTER   OCR4A // 16-bit Fast PWM mode requires top reset value stored here.
  #define SPINDLE_OCRA_TOP_VALUE  0x0400 // PWM counter reset value. Should be the same as PWM_MAX_VALUE in hex.

  // Define spindle output pins.
  #define SPINDLE_PWM_DDR		DDRH
  #define SPINDLE_PWM_PORT  PORTH
  #define SPINDLE_PWM_BIT		4 // MEGA2560 Digital Pin 7
*/
#endif

/* 
#ifdef CPU_MAP_CUSTOM_PROC
  // For a custom pin map or different processor, copy and edit one of the available cpu
  // map files and modify it to your needs. Make sure the defined name is also changed in
  // the config.h file.
#endif
*/

#endif
