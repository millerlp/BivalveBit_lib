//
//  BivalveBit_lib.cpp
//  
//
//  Created by Luke Miller on 2021-09-01.
//
//

#include "BivalveBit_lib.h"




// Function to take a few readings from hall sensor and average them
unsigned int readHall(byte ANALOG_IN){
  unsigned int rawAnalog = 0;
  analogRead(ANALOG_IN); // throw away 1st reading
  for (byte i = 0; i<4; i++){
    rawAnalog = rawAnalog + analogRead(ANALOG_IN);
    delay(1);
  }
  // Do a 2-bit right shift to divide rawAnalog
  // by 4 to get the average of the 4 readings
  rawAnalog = rawAnalog >> 2;   
  return rawAnalog;
}


unsigned int readWakeHall(byte ANALOG_IN, byte HALL_SLEEP) {
  digitalWrite(HALL_SLEEP, HIGH); // turn on hall effect sensor
  delayMicroseconds(50);	
  unsigned int rawAnalog = 0;
  analogRead(ANALOG_IN); // throw away 1st reading
  for (byte i = 0; i<4; i++){
    rawAnalog = rawAnalog + analogRead(ANALOG_IN);
    delay(1);
  }
  digitalWrite(HALL_SLEEP, LOW); // put hall sensor to sleep
  // Do a 2-bit right shift to divide rawAnalog
  // by 4 to get the average of the 4 readings
  rawAnalog = rawAnalog >> 2;   
  return rawAnalog;
}



//------------------------------------------------------------
// Other public functions



void printTimeSerial(DateTime now){
    //------------------------------------------------
    // printTime function takes a DateTime object from
    // the real time clock and prints the date and time
    // to the serial monitor.
    Serial.print(now.year(), DEC);
    Serial.print('-');
    if (now.month() < 10) {
        Serial.print(F("0"));
    }
    Serial.print(now.month(), DEC);
    Serial.print('-');
    if (now.day() < 10) {
        Serial.print(F("0"));
    }
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    if (now.hour() < 10){
        Serial.print(F("0"));
    }
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    if (now.minute() < 10) {
        Serial.print("0");
    }
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    if (now.second() < 10) {
        Serial.print(F("0"));
    }
    Serial.print(now.second(), DEC);
    // You may want to print a newline character
    // after calling this function i.e. Serial.println();
    
}


void printTimeOLED(DateTime now, SSD1306AsciiWire& oled1){
    //------------------------------------------------
    // printTime function takes a DateTime object from
    // the real time clock and prints the date and time
    // to the OLED screen.
    oled1.print(now.year(), DEC);
    oled1.print('-');
    if (now.month() < 10) {
        oled1.print(F("0"));
    }
    oled1.print(now.month(), DEC);
    oled1.print('-');
    if (now.day() < 10) {
        oled1.print(F("0"));
    }
    oled1.print(now.day(), DEC);
    oled1.print(' ');
    if (now.hour() < 10){
        oled1.print(F("0"));
    }
    oled1.print(now.hour(), DEC);
    oled1.print(':');
    if (now.minute() < 10) {
        oled1.print("0");
    }
    oled1.print(now.minute(), DEC);
    oled1.print(':');
    if (now.second() < 10) {
        oled1.print(F("0"));
    }
    oled1.print(now.second(), DEC);
    // You may want to print a newline character
    // after calling this function i.e. Serial.println();
    
}


//---------------printTimeToSD----------------------------------------
// printTimeToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a
// comma-separated value format.

void printTimeToSD (SdFile& mylogfile, DateTime tempTime) {
    // Write the date and time in a human-readable format
    // to the file on the SD card.
    mylogfile.print(tempTime.year(), DEC);
    mylogfile.print(F("-"));
    if (tempTime.month() < 10) {
        mylogfile.print("0");
    }
    mylogfile.print(tempTime.month(), DEC);
    mylogfile.print(F("-"));
    if (tempTime.day() < 10) {
        mylogfile.print("0");
    }
    mylogfile.print(tempTime.day(), DEC);
    mylogfile.print(F(" "));
    if (tempTime.hour() < 10){
        mylogfile.print("0");
    }
    mylogfile.print(tempTime.hour(), DEC);
    mylogfile.print(F(":"));
    if (tempTime.minute() < 10) {
        mylogfile.print("0");
    }
    mylogfile.print(tempTime.minute(), DEC);
    mylogfile.print(F(":"));
    if (tempTime.second() < 10) {
        mylogfile.print("0");
    }
    mylogfile.print(tempTime.second(), DEC);
}


//-------------- initHeartFileName --------------------------------------------------
// initHeartFileName - a function to create a filename for the SD card based
// on the 4-digit year, month, day, hour, minutes and a serial number.
// The character array 'heartfilename' was defined as a global array
// at the top of the sketch in the form "YYYYMMDD_HHMM_00_SN000_IR.csv"
void initHeartFileName(SdFat& sd, SdFile& IRFile, DateTime time1, char *heartfilename, bool serialValid, char *serialNumber) {
    
    char buf[5];
    // integer to ascii function itoa(), supplied with numeric year value,
    // a buffer to hold output, and the base for the conversion (base 10 here)
    itoa(time1.year(), buf, 10);
    // copy the ascii year into the filename array
    for (byte i = 0; i < 4; i++){
        heartfilename[i] = buf[i];
    }
    // Insert the month value
    if (time1.month() < 10) {
        heartfilename[4] = '0';
        heartfilename[5] = time1.month() + '0';
    } else if (time1.month() >= 10) {
        heartfilename[4] = (time1.month() / 10) + '0';
        heartfilename[5] = (time1.month() % 10) + '0';
    }
    // Insert the day value
    if (time1.day() < 10) {
        heartfilename[6] = '0';
        heartfilename[7] = time1.day() + '0';
    } else if (time1.day() >= 10) {
        heartfilename[6] = (time1.day() / 10) + '0';
        heartfilename[7] = (time1.day() % 10) + '0';
    }
    // Insert an underscore between date and time
    heartfilename[8] = '_';
    // Insert the hour
    if (time1.hour() < 10) {
        heartfilename[9] = '0';
        heartfilename[10] = time1.hour() + '0';
    } else if (time1.hour() >= 10) {
        heartfilename[9] = (time1.hour() / 10) + '0';
        heartfilename[10] = (time1.hour() % 10) + '0';
    }
    // Insert minutes
    if (time1.minute() < 10) {
        heartfilename[11] = '0';
        heartfilename[12] = time1.minute() + '0';
    } else if (time1.minute() >= 10) {
        heartfilename[11] = (time1.minute() / 10) + '0';
        heartfilename[12] = (time1.minute() % 10) + '0';
    }
    // Insert another underscore after time
    heartfilename[13] = '_';
    // If there is a valid serialnumber SNxxx, insert it into
    // the file name in positions 17-21.
    if (serialValid) {
        byte serCount = 0;
        for (byte i = 17; i < 22; i++){
            heartfilename[i] = serialNumber[serCount];
            serCount++;
        }
    }
    // Next change the counter on the end of the filename
    // (digits 14+15) to increment count for files generated on
    // the same day. This shouldn't come into play
    // during a normal data run, but can be useful when
    // troubleshooting.
    for (uint8_t i = 0; i < 100; i++) {
        heartfilename[14] = i / 10 + '0';
        heartfilename[15] = i % 10 + '0';
        
        if (!sd.exists(heartfilename)) {
            // when sd.exists() returns false, this block
            // of code will be executed to open the file
            if (!IRFile.open(heartfilename, O_RDWR | O_CREAT | O_AT_END)) {
                // If there is an error opening the file, notify the
                // user. Otherwise, the file is open and ready for writing
                // Turn both indicator LEDs on to indicate a failure
                // to create the log file
                //				digitalWrite(ERRLED, !digitalRead(ERRLED)); // Toggle error led
                //				digitalWrite(GREENLED, !digitalRead(GREENLED)); // Toggle indicator led
                delay(5);
            }
            break; // Break out of the for loop when the
            // statement if(!IRFile.exists())
            // is finally false (i.e. you found a new file name to use).
        } // end of if(!sd.exists())
    } // end of file-naming for loop
    //------------------------------------------------------------
    // Write 1st header line
	// Header will consist of POSIXt,DateTime,Serial Number, IR value
    IRFile.print(F("POSIXt,DateTime,SN,IR"));
    IRFile.println();
    // Update the file's creation date, modify date, and access date.
    IRFile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    IRFile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    IRFile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    IRFile.close(); // force the data to be written to the file by closing it
} // end of initHeartFileName function


//-------------- initGapeFileName --------------------------------------------------
// initGapeFileName - a function to create a filename for the SD card based
// on the 4-digit year, month, day, hour, minutes and a serial number.
// The character array 'gapefilename' was defined as a global array
// at the top of the sketch in the form "YYYYMMDD_HHMM_00_SN000_GAPE.csv"
void initGapeFileName(SdFat& sd, SdFile& GAPEFile, DateTime time1, char *gapefilename, bool serialValid, char *serialNumber) {
    
    char buf[5];
    // integer to ascii function itoa(), supplied with numeric year value,
    // a buffer to hold output, and the base for the conversion (base 10 here)
    itoa(time1.year(), buf, 10);
    // copy the ascii year into the filename array
    for (byte i = 0; i < 4; i++){
        gapefilename[i] = buf[i];
    }
    // Insert the month value
    if (time1.month() < 10) {
        gapefilename[4] = '0';
        gapefilename[5] = time1.month() + '0';
    } else if (time1.month() >= 10) {
        gapefilename[4] = (time1.month() / 10) + '0';
        gapefilename[5] = (time1.month() % 10) + '0';
    }
    // Insert the day value
    if (time1.day() < 10) {
        gapefilename[6] = '0';
        gapefilename[7] = time1.day() + '0';
    } else if (time1.day() >= 10) {
        gapefilename[6] = (time1.day() / 10) + '0';
        gapefilename[7] = (time1.day() % 10) + '0';
    }
    // Insert an underscore between date and time
    gapefilename[8] = '_';
    // Insert the hour
    if (time1.hour() < 10) {
        gapefilename[9] = '0';
        gapefilename[10] = time1.hour() + '0';
    } else if (time1.hour() >= 10) {
        gapefilename[9] = (time1.hour() / 10) + '0';
        gapefilename[10] = (time1.hour() % 10) + '0';
    }
    // Insert minutes
    if (time1.minute() < 10) {
        gapefilename[11] = '0';
        gapefilename[12] = time1.minute() + '0';
    } else if (time1.minute() >= 10) {
        gapefilename[11] = (time1.minute() / 10) + '0';
        gapefilename[12] = (time1.minute() % 10) + '0';
    }
    // Insert another underscore after time
    gapefilename[13] = '_';
    // If there is a valid serialnumber SNxxx, insert it into
    // the file name in positions 17-21.
    if (serialValid) {
        byte serCount = 0;
        for (byte i = 17; i < 22; i++){
            gapefilename[i] = serialNumber[serCount];
            serCount++;
        }
    }
    // Next change the counter on the end of the filename
    // (digits 14+15) to increment count for files generated on
    // the same day. This shouldn't come into play
    // during a normal data run, but can be useful when
    // troubleshooting.
    for (uint8_t i = 0; i < 100; i++) {
        gapefilename[14] = i / 10 + '0';
        gapefilename[15] = i % 10 + '0';
        
        if (!sd.exists(gapefilename)) {
            // when sd.exists() returns false, this block
            // of code will be executed to open the file
            if (!GAPEFile.open(gapefilename, O_RDWR | O_CREAT | O_AT_END)) {
                // If there is an error opening the file, notify the
                // user. Otherwise, the file is open and ready for writing
                // Turn both indicator LEDs on to indicate a failure
                // to create the log file
                //				digitalWrite(ERRLED, !digitalRead(ERRLED)); // Toggle error led
                //				digitalWrite(GREENLED, !digitalRead(GREENLED)); // Toggle indicator led
                delay(5);
            }
            break; // Break out of the for loop when the
            // statement if(!GAPEFile.exists())
            // is finally false (i.e. you found a new file name to use).
        } // end of if(!sd.exists())
    } // end of file-naming for loop
    //------------------------------------------------------------
    // Write 1st header line
	// Header will be: POSIXt, DateTime, SN (serial number), Hall value, Temperature, Battery Voltage
    GAPEFile.print(F("POSIXt,DateTime,SN,Hall,Temp.C,Battery.V"));
    GAPEFile.println();
    // Update the file's creation date, modify date, and access date.
    GAPEFile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    GAPEFile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    GAPEFile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    GAPEFile.close(); // force the data to be written to the file by closing it
} // end of initGapeFileName function



//-------------- initGapeOnlyFileName --------------------------------------------------
// initGapeOnlyFileName - a function to create a filename for the SD card based
// on the 4-digit year, month, day, hour, minutes and a serial number.
// The character array 'gapefilename' was defined as a global array
// at the top of the sketch in the form "YYYYMMDD_HHMM_00_SN000_GAPE.csv"
// This version records gape data, but not temperature (assumes there is no temperature sensor available)
void initGapeOnlyFileName(SdFat& sd, SdFile& GAPEFile, DateTime time1, char *gapefilename, bool serialValid, char *serialNumber) {
    
    char buf[5];
    // integer to ascii function itoa(), supplied with numeric year value,
    // a buffer to hold output, and the base for the conversion (base 10 here)
    itoa(time1.year(), buf, 10);
    // copy the ascii year into the filename array
    for (byte i = 0; i < 4; i++){
        gapefilename[i] = buf[i];
    }
    // Insert the month value
    if (time1.month() < 10) {
        gapefilename[4] = '0';
        gapefilename[5] = time1.month() + '0';
    } else if (time1.month() >= 10) {
        gapefilename[4] = (time1.month() / 10) + '0';
        gapefilename[5] = (time1.month() % 10) + '0';
    }
    // Insert the day value
    if (time1.day() < 10) {
        gapefilename[6] = '0';
        gapefilename[7] = time1.day() + '0';
    } else if (time1.day() >= 10) {
        gapefilename[6] = (time1.day() / 10) + '0';
        gapefilename[7] = (time1.day() % 10) + '0';
    }
    // Insert an underscore between date and time
    gapefilename[8] = '_';
    // Insert the hour
    if (time1.hour() < 10) {
        gapefilename[9] = '0';
        gapefilename[10] = time1.hour() + '0';
    } else if (time1.hour() >= 10) {
        gapefilename[9] = (time1.hour() / 10) + '0';
        gapefilename[10] = (time1.hour() % 10) + '0';
    }
    // Insert minutes
    if (time1.minute() < 10) {
        gapefilename[11] = '0';
        gapefilename[12] = time1.minute() + '0';
    } else if (time1.minute() >= 10) {
        gapefilename[11] = (time1.minute() / 10) + '0';
        gapefilename[12] = (time1.minute() % 10) + '0';
    }
    // Insert another underscore after time
    gapefilename[13] = '_';
    // If there is a valid serialnumber SNxxx, insert it into
    // the file name in positions 17-21.
    if (serialValid) {
        byte serCount = 0;
        for (byte i = 17; i < 22; i++){
            gapefilename[i] = serialNumber[serCount];
            serCount++;
        }
    }
    // Next change the counter on the end of the filename
    // (digits 14+15) to increment count for files generated on
    // the same day. This shouldn't come into play
    // during a normal data run, but can be useful when
    // troubleshooting.
    for (uint8_t i = 0; i < 100; i++) {
        gapefilename[14] = i / 10 + '0';
        gapefilename[15] = i % 10 + '0';
        
        if (!sd.exists(gapefilename)) {
            // when sd.exists() returns false, this block
            // of code will be executed to open the file
            if (!GAPEFile.open(gapefilename, O_RDWR | O_CREAT | O_AT_END)) {
                // If there is an error opening the file, notify the
                // user. Otherwise, the file is open and ready for writing
                // Turn both indicator LEDs on to indicate a failure
                // to create the log file
                //				digitalWrite(ERRLED, !digitalRead(ERRLED)); // Toggle error led
                //				digitalWrite(GREENLED, !digitalRead(GREENLED)); // Toggle indicator led
                delay(5);
            }
            break; // Break out of the for loop when the
            // statement if(!GAPEFile.exists())
            // is finally false (i.e. you found a new file name to use).
        } // end of if(!sd.exists())
    } // end of file-naming for loop
    //------------------------------------------------------------
    // Write 1st header line
	// Header will be: POSIXt, DateTime, SN (serial number), Hall value, Battery Voltage
    GAPEFile.print(F("POSIXt,DateTime,SN,Hall,Battery.V"));
    GAPEFile.println();
    // Update the file's creation date, modify date, and access date.
    GAPEFile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    GAPEFile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    GAPEFile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
                      time1.hour(), time1.minute(), time1.second());
    GAPEFile.close(); // force the data to be written to the file by closing it
} // end of initGapeOnlyFileName function



//------------readBatteryVoltage-------------------
// readBatteryVoltage function. This will read the AD convertor
// and calculate the approximate battery voltage (before the
// voltage regulator). Returns a floating point value for
// voltage.
float readBatteryVoltage(byte BATT_MONITOR_EN, 
							byte BATT_MONITOR, 
							float dividerRatio, 
							float refVoltage){
    // Turn on the battery voltage monitor circuit
    digitalWrite(BATT_MONITOR_EN, HIGH);
    delay(1);
    // Read the analog input pin
    unsigned int rawAnalog = 0;
    analogRead(BATT_MONITOR); // This initial value is ignored
    delay(3); // Give the ADC time to stablize
    // Take 4 readings
    for (byte i = 0; i<4; i++){
        rawAnalog = rawAnalog + analogRead(BATT_MONITOR);
        delay(2);
    }
    // Do a 2-bit right shift to divide rawAnalog
    // by 4 to get the average of the 4 readings
    rawAnalog = rawAnalog >> 2;
    // Shut off the battery voltage sense circuit
    digitalWrite(BATT_MONITOR_EN, LOW);
    // Convert the rawAnalog count value (0-1023) into a voltage
    // Relies on variables dividerRatio and refVoltage
    float reading = (rawAnalog  * (refVoltage / 1023.0)) * dividerRatio;
    return reading; // return voltage result
}


// Set unused pins to INPUT_PULLUP to save power
void setUnusedPins(void){
  pinMode(14, INPUT_PULLUP); // PD2
  pinMode(15, INPUT_PULLUP); // PD3
  pinMode(16, INPUT_PULLUP); // PD4
  pinMode(17, INPUT_PULLUP); // PD5
  pinMode(18, INPUT_PULLUP); // PD6
  pinMode(21, INPUT_PULLUP); // PF1
  pinMode(22, INPUT_PULLUP); // PF2
  pinMode(23, INPUT_PULLUP); // PF3
  pinMode(25, INPUT_PULLUP); // PF5
}


// Disable unused peripherals during Active mode (and sleep modes)
void disableUnusedPeripherals(void){
	// Disable analog comparator 0
	uint8_t temp = AC0.CTRLA;
	temp &= ~0x01;
	AC0.CTRLA = temp;
	// Disable CCL peripheral
	temp = CCL.CTRLA;
	temp &= ~0x01;
	CCL.CTRLA = temp;
	// Disable CRCSCAN peripheral
	temp = CRCSCAN.CTRLA;
	temp &= ~0x01;
	CRCSCAN.CTRLA = temp;
}


void enableADC(void){
	uint8_t temp = ADC0.CTRLA;
	temp |= 0x01;
	ADC0.CTRLA = temp;
}

void disableADC(void){
	uint8_t temp = ADC0.CTRLA;
	temp &= ~0x01;
	ADC0.CTRLA = temp;
}


void PIT_init(void)
{
    
    uint8_t temp;
    
    /* Initialize 32.768kHz Oscillator: */
    /* Disable oscillator by writing 0 to the ENABLE bit in the XOSC32KCTRLA register: */
    temp = CLKCTRL.XOSC32KCTRLA; // read register contents
    temp &= ~CLKCTRL_ENABLE_bm; // modify register, write 0 to ENABLE bit to allow changing setting
    /* Writing to protected register */
    _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, temp);  // Arduino version of ccp_write_io
    
    while(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm)
    {
        ; /* Wait until XOSC32KS becomes 0 */
    }

    /* SEL = 0 (Use External Crystal): */
    // Now you can actually change the register values safely
    temp = CLKCTRL.XOSC32KCTRLA;
    temp |= CLKCTRL_SEL_bm; // Set up for external clock on TOSC1 pin only by writing a 1
    /* Writing to protected register */
     _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, temp);  // Arduino version of ccp_write_io
     // We're not yet set up to actually read a clock input on TOSC1

    /* Enable oscillator: */
    temp = CLKCTRL.XOSC32KCTRLA;
    temp |= CLKCTRL_ENABLE_bm; // This will update TOSC1's function
    /* Writing to protected register */
    _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, temp);  // Arduino version of ccp_write_io

    /* There was a quirk where the RTC status would not go to zero when you 
     *  re-entered this function after running it once. Disabling the RTC.CTRLA Enable
     *  bit appears to fix the problem
     */
    // Disable RTC (resets RTC and PIT if you write to RTC.CTRLA due to silicone error
    RTC.CTRLA &= ~RTC_RTCEN_bm; // disable RTC

    /* Initialize RTC: */
    while (RTC.STATUS > 0)
    {
        ; /* Wait for all register to be synchronized */
    }

    /* Set RTC peripheral to read a 32.768kHz external clock signal from TOSC1 */
    RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc; // external crystal on TOSC1

    
    RTC.PITINTCTRL = RTC_PI_bm; /* Periodic Interrupt: enabled */

    // Define the prescalar value for the periodic interrupt timer. This will divide the
    // 32.768kHz input by the chosen prescaler. A prescaler of 32768 will cause the 
    // interrupt to only fire once per second.
//    RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc /* RTC Clock Cycles 32768 */
//                 | RTC_PITEN_bm; /* Enable: enabled by writing 1*/

    // Prescaler 4096 gives 8 interrupts per second (period from high to high = 250ms)
    RTC.PITCTRLA = RTC_PERIOD_CYC4096_gc /* RTC Clock Cycles 4096 */
             | RTC_PITEN_bm; /* Enable: enabled by writing 1*/        
 
}


void SLPCTRL_init(void)
{
    SLPCTRL.CTRLA |= SLPCTRL_SMODE_PDOWN_gc; // Use this for lowest power POWERDOWN mode
//    SLPCTRL.CTRLA |= SLPCTRL_SMODE_STDBY_gc; // Use this for Standby mode
    SLPCTRL.CTRLA |= SLPCTRL_SEN_bm;	// Enable sleep mode
	// You must still call sleep_cpu() in the main program to enter sleep mode
}