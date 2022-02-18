//
//  BivalveBit_lib.h
//  
//
//  Created by Luke Miller on 2021-09-01
//
//

#ifndef BivalveBit_lib_H
#define BivalveBit_lib_H

#include <Arduino.h> // to get access to pinMode, digitalRead etc functions
#include "SdFat.h"	// https://github.com/greiman/SdFat
#include <SPI.h>
#include "MCP7940.h"  // https://github.com/Zanduino/MCP7940
//#include "RTClib.h" // https://github.com/millerlp/RTClib
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii


// Various additional libraries for access to sleep mode functions
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <wiring_private.h>
#include <avr/wdt.h>
#include <math.h>



//****************************************************************


//--------- Public functions

// Take a set of 4 Hall effect sensor readings and return the average integer value
unsigned int readHall(byte ANALOG_IN);
unsigned int readWakeHall(byte ANALOG_IN, byte HALL_SLEEP);

// Print formatted Date + Time to Serial monitor
void printTimeSerial(DateTime now);

// Print time to OLED display
void printTimeOLED(DateTime now, SSD1306AsciiWire& oled1);

// Print formatted Date + Time to SD card csv file. Notice that this passes the
// SdFile object by reference (SdFile& mylogfile) instead of making a copy and
// passing by value (which SdFile mylogfile would do).
void printTimeToSD(File& mylogfile, DateTime now);

// Initialize a new output csv file. Note that this writes a header row
// to the file, so you may want to tweak the column labels in this function.
void initHeartFileName(SdFat& sd, File& IRFile, DateTime time1, char *heartfilename, bool serialValid, char *serialNumber);

// Initialize a new output csv file. Note that this writes a header row
// to the file, so you may want to tweak the column labels in this function.
void initGapeFileName(SdFat& sd, File& GAPEFile, DateTime time1, char *gapefilename, bool serialValid, char *serialNumber);



/**
  Constructor

  @param BATT_MONITOR_EN - a pin number, used to enable the battery monitor circuit
  @param BATT_MONITOR - an analog input pin number used to read the battery voltage
  @param dividerRatio - the effective voltage divider value (i.e. (47k + 47k)/47k = 2)
  @param refVoltage - the voltage measured on the AREF pin that is used by the ADC
*/
// Function to read supply battery voltage
float readBatteryVoltage(byte BATT_MONITOR_EN, 
							byte BATT_MONITOR, 
							float dividerRatio, 
							float refVoltage);

							
// Set unused pins to INPUT_PULLUP to save power
void setUnusedPins(void);

// Disable some unused peripherals
void disableUnusedPeripherals(void);


// Restart ADC after it was shut down
void enableADC(void);
void disableADC(void);

// Func to set up the Periodic Interrupt Timer at 8Hz
// You must set up an external 32.768kHz clock signal on TOSC1
// such as from the MCP79400 real time clock
void PIT_init(void);			
							

// Function to set up sleep mode
// You must still call sleep_cpu() in the main program to enter sleep
void SLPCTRL_init(void);
							
#endif /* MusselBedHeaterlib_H */
