/**********************************************************************
  Copyright (C) 2018 Maxim Integrated Products, Inc., All Rights Reserved.

  Permission is hereby granted, free of charge, to any person obtaining a
  copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation
  the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
  OTHER DEALINGS IN THE SOFTWARE.

  Except as contained in this notice, the name of Maxim Integrated
  Products, Inc. shall not be used except as stated in the Maxim Integrated
  Products, Inc. Branding Policy.

  The mere transfer of this software does not imply any licenses
  of trade secrets, proprietary technology, copyrights, patents,
  trademarks, maskwork rights, or any other form of intellectual
  property whatsoever. Maxim Integrated Products, Inc. retains all
  ownership rights.

  Authors:
  Ole Dreessen; ole.dreessen@maximintegrated.com
  Devin Alexander; devin.alexander@maximintegrated.com
  
  Revisions:
  0.1.0       13.03.2018      initial code
  1.0.0       14.03.2018      bugfixes and improvements; added interrupt functionality and interrupt register readouts
  1.0.1       14.08.2018      (Devin Alexander) Added functionality for the MAX17055 to this sketch for a basic battery charging with led charginh indicators.
                              This is meant to serve as a base sketch for charging and monitoring the MAX32620FTHR off of battery.
                              

  MAX77650 Details: 
  
  The MAX77650/MAX77651 provide highly-integrated battery charging and
  power supply solutions for low-power wearable applications where size and efficiency
  are critical. Both devices feature a SIMO buck-boost regulator that provides three
  independently programmable power rails from a single inductor to minimize total solution
  size. A 150mA LDO provides ripple rejection for audio and other noise-sensitive applications.
  A highly configurable linear charger supports a wide range of Li+ battery capacities and
  includes battery temperature monitoring for additional safety (JEITA).

  The devices include other features such as current sinks for driving LED indicators and
  an analog multiplexer that switches several internal voltage and current signals to an
  external node for monitoring with an external ADC. A bidirectional I2C interface allows
  for configuring and checking the status of the devices. An internal on/off controller
  provides a controlled startup sequence for the regulators and provides supervisory
  functionality when the devices are on. Numerous factory programmable options allow the
  device to be tailored for many applications, enabling faster time to market.

  Resources can be found at
  https://www.maximintegrated.com/en/products/power/battery-management/MAX77650.html
  https://www.maximintegrated.com/en/app-notes/index.mvp/id/6428


  MAX17055 Details:

  The MAX17055 is a low 7μA operating current fuel gauge
  that implements Maxim ModelGauge™ m5 EZ algorithm.
  ModelGauge m5 EZ makes fuel gauge implementation
  easy by eliminating battery characterization requirements
  and simplifying host software interaction. The ModelGauge
  m5 EZ robust algorithm provides tolerance against battery
  diversity for most lithium batteries and applications.
  
  ModelGauge m5 EZ algorithm combines the short-term
  accuracy and linearity of a coulomb counter with the longterm
  stability of a voltage-based fuel gauge, along with
  temperature compensation to provide industry-leading
  fuel gauge accuracy. The MAX17055 automatically compensates
  for cell aging, temperature, and discharge rate,
  and provides accurate state of charge (SOC in %) and
  remaining capacity in milliampere-hours (mAh). As the
  battery approaches the critical region near empty, the
  ModelGauge m5 algorithm invokes a special compensation
  that eliminates any error. It also provides three methods
  for reporting the age of the battery: reduction in capacity,
  increase in battery resistance, and cycle odometer.
  
  The MAX17055 provides precision measurements of current,
  voltage, and temperature. Temperature of the battery
  pack is measured using an internal temperature measurement
  or external thermistor. A 2-wire I2C interface
  provides access to data and control registers.
  
  The MAX17055 is available in a tiny, lead-free 0.4mm
  pitch 1.4mm x 1.5mm, 9-pin WLP package, and a 2mm x
  2.5mm, 10-pin TDFN package.
  
  Resources can be found at
  https://www.maximintegrated.com/en/products/power/battery-management/MAX17055.html

  
**********************************************************************/
// Includes
#include <Wire.h>
#include <MAX77650-Arduino-Library.h>
#include <max17055.h>


// Battery Setup struct instance
MAX17055::platform_data design_param; 

// MAX17055 Object
MAX17055 fuelGauge; //uses I2C2 Bus on MAX32620FTHR

// MAX77650 Definitions
#define MAX77650_debug true
#define MAX77650_PHLD P2_2   //Pin 18 -> connected to MAX77650 power hold input pin (A1)
#define MAX77650_IRQpin P2_3 //Pin 19 -> connected to MAX77650 IRQ output pin (C2)

// Globals
int i = 0;
boolean interrupt = false;
void chargerStatusChanged(void);

void setup(void)
{
  Serial.begin(115200);
  
  // Configure the Power-Hold pin on the MAX77650 to enable booting MCU from battery power
  pinMode(MAX77650_PHLD, OUTPUT);          //configure pin as output
  digitalWrite(MAX77650_PHLD, HIGH);       //set output to HIGH to hold the power-on state

  design_param.designcap = 400;  // (in mAh) Design Battery Capacity
  design_param.ichgterm  = 40;   // (in mA) Charge Termination Current for the Battery
  design_param.vempty    = 3100; // (in mV) Battery Empty Voltage
  design_param.vcharge   = 4200; // (in mV) Battery Charge Voltage can be obtained from MAX77650 configuration
  design_param.rsense    = 5;    // (in mOhms) The Rsense that is used in the MAX17055

  // init MAX17055 with a battery  
  int status_of_MAX17055 = fuelGauge.init(design_param);
  Serial.print("Init Function Status of MAX17055 = ");
  Serial.println(status_of_MAX17055, HEX);
  
  // init MAX77650 with a battery
  MAX77650_init();

  // Baseline Initialization following rules printed in MAX77650 Programmers Guide Chapter 4 Page 5
  if (MAX77650_debug) Serial.print("Set Main Bias to normal Mode: ");
  if (MAX77650_setSBIA_LPM(false)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");    //Set Main Bias to normal Mode
  if (MAX77650_debug) Serial.print("Set On/Off-Button to push-button-mode: ");
  if (MAX77650_setnEN_MODE(false)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");    //set on/off-button to push-button
  if (MAX77650_debug) Serial.print("Set nEN input debounce time to 30ms: ");
  if (MAX77650_setDBEN_nEN(true)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");    //set on/off-button to push-button
  if (MAX77650_debug) Serial.print("Comparing part-numbers: ");
  if (MAX77650_getDIDM() == PMIC_partnumber) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");     //checking partnumbers
  if (MAX77650_debug) Serial.print("Checking OTP options: ");
  if (MAX77650_getCID() != MAX77650_CID) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");     //checking OTP options
  // Values for NTC beta=3800K; Battery-values are for 1s 303759 with 600mAh
  if (MAX77650_debug) Serial.print("Set the VCOLD JEITA Temperature Threshold to 0°C: ");
  if (MAX77650_setTHM_COLD(2)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");   //0°C
  if (MAX77650_debug) Serial.print("Set the VCOOL JEITA Temperature Threshold to 15°C: ");
  if (MAX77650_setTHM_COOL(3)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");   //15°C
  if (MAX77650_debug) Serial.print("Set the VWARM JEITA Temperature Threshold to 45°C: ");
  if (MAX77650_setTHM_WARM(2)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");   //45°C
  if (MAX77650_debug) Serial.print("Set the VHOT JEITA Temperature Threshold to 60°C: ");
  if (MAX77650_setTHM_HOT(3)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");    //60°C
  if (MAX77650_debug) Serial.print("Set CHGIN regulation voltage to 4.00V: ");
  if (MAX77650_setVCHGIN_MIN(0)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed"); //
  if (MAX77650_debug) Serial.print("Set CHGIN Input Current Limit to 300mA: ");
  if (MAX77650_setICHGIN_LIM(0)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed"); //300mA
  if (MAX77650_debug) Serial.print("Set the prequalification charge current to 10%: ");
  if (MAX77650_setI_PQ(false)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");   //10%
  if (MAX77650_debug) Serial.print("Set Battery prequalification voltage threshold to 3.0V: ");
  if (MAX77650_setCHG_PQ(7)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");        //3.0V
  if (MAX77650_debug) Serial.print("Set Charger Termination Current to 15% of of fast charge current: ");
  if (MAX77650_setI_TERM(3)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");        //15%
  if (MAX77650_debug) Serial.print("Set Topoff timer value to 0 minutes: ");
  if (MAX77650_setT_TOPOFF(0)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");      //0 minutes
  if (MAX77650_debug) Serial.print("Set the die junction temperature regulation point to 60°C: ");
  if (MAX77650_setTJ_REG(0)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");        //60°C Tj
  if (MAX77650_debug) Serial.print("Set System voltage regulation to 4.50V: ");
  if (MAX77650_setVSYS_REG(0x10)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");   //Vsys=4.5V
  if (MAX77650_debug) Serial.print("Set the fast-charge constant current value to 300mA: ");
  if (MAX77650_setCHG_CC(0x3f)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");     //300mA
  if (MAX77650_debug) Serial.print("Set the fast-charge safety timer to 3h: ");
  if (MAX77650_setT_FAST_CHG(1)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");    //3h
  if (MAX77650_debug) Serial.print("Set IFAST-CHG_JEITA to 300mA: ");
  if (MAX77650_setCHG_CC_JEITA(0x3f)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed"); //300mA
  if (MAX77650_debug) Serial.print("Set Thermistor enable bit: ");
  if (MAX77650_setTHM_EN(true)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");     //enable the thermistor monitoring
  if (MAX77650_debug) Serial.print("Set fast-charge battery regulation voltage to 4.20V: ");
  if (MAX77650_setCHG_CV(0x18)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");     //4.20V
  if (MAX77650_debug) Serial.print("Set USB not in power down: ");
  if (MAX77650_setUSBS(false)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");      //USBS not suspended
  if (MAX77650_debug) Serial.print("Set the modified VFAST-CHG to 4.00V: ");
  if (MAX77650_setCHG_CV_JEITA(0x10)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed"); //4.0V
  if (MAX77650_debug) Serial.print("Selects the battery discharge current full-scale current value to 300mA: ");
  if (MAX77650_setIMON_DISCHG_SCALE(0x0A)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed"); //300mA
  if (MAX77650_debug) Serial.print("Disable the analog MUX output: ");
  if (MAX77650_setMUX_SEL(0)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");    //AMUX=off
  if (MAX77650_debug) Serial.print("Set the Charger to Enable: ");
  if (MAX77650_setCHG_EN(true)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");  //enable the Charger
  if (MAX77650_debug) Serial.print("Disable SIMO Buck-Boost Channel 0 Active-Discharge: ");
  if (MAX77650_setADE_SBB0(false)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Disable SIMO Buck-Boost Channel 1 Active-Discharge: ");
  if (MAX77650_setADE_SBB1(false)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Disable SIMO Buck-Boost Channel 2 Active-Discharge: ");
  if (MAX77650_setADE_SBB1(false)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Set SIMO Buck-Boost to maximum drive strength: ");
  if (MAX77650_setDRV_SBB(0b00)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Set SIMO Buck-Boost Channel 0 Peak Current Limit to 500mA: ");
  if (MAX77650_setIP_SBB0(0b00)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Set SIMO Buck-Boost Channel 1 Peak Current Limit to 500mA: ");
  if (MAX77650_setIP_SBB1(0b00)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Set SIMO Buck-Boost Channel 2 Peak Current Limit to 500mA: ");
  if (MAX77650_setIP_SBB2(0b00)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Set SIMO Buck-Boost Channel 2 to on while in stand-by-mode: ");
  if (MAX77650_setEN_SBB2(0b110)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Initialize Global Interrupt Mask Register: ");
  if (MAX77650_setINT_M_GLBL(0x0)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");
  if (MAX77650_debug) Serial.print("Initialize Charger Interrupt Mask Register: ");
  if (MAX77650_setINT_M_CHG(0x0)) if (MAX77650_debug) Serial.println("okay"); else if (MAX77650_debug) Serial.println("failed");

  // Initialization of PMIC-LEDs
  MAX77650_setLED_FS0(0b01);
  MAX77650_setINV_LED0(false);    //LED red: phase operation
  MAX77650_setBRT_LED0(0b00000);  //LED red: brightness
  MAX77650_setP_LED0(0b1111);     //LED red: LED period
  MAX77650_setD_LED0(0b1111);     //LED red: LED duty-cycle
  MAX77650_setLED_FS1(0b01);
  MAX77650_setINV_LED1(false);    //LED green: phase operation
  MAX77650_setBRT_LED1(0b00000);  //LED green: brightness
  MAX77650_setP_LED1(0b1111);     //LED green: LED period
  MAX77650_setD_LED1(0b1111);     //LED green: LED duty-cycle
  MAX77650_setLED_FS2(0b01);
  MAX77650_setINV_LED2(false);    //LED blue: phase operation
  MAX77650_setBRT_LED2(0b00000);  //LED blue: brightness
  MAX77650_setP_LED2(0b1111);     //LED blue: LED period
  MAX77650_setD_LED2(0b1111);     //LED blue: LED duty-cycle
  MAX77650_setEN_LED_MSTR(true);  //LEDs Master enable

  // MAX77650 Interrupt wiring
  pinMode(MAX77650_IRQpin, INPUT_PULLUP);
  attachInterrupt(MAX77650_IRQpin, MAX77650_IRQ, FALLING);

  // Read and clear Interrupt Registers
  MAX77650_getINT_GLBL();
  MAX77650_getINT_CHG();
  MAX77650_getERCFLAG();

  // Run the chargeStatusDetector function once at end of initialization
  chargerStatusChanged();
  if (MAX77650_debug) Serial.println("End Initialisation of MAX77650");
}

void check_IRQ_src(void){
  byte INT_GLBL = MAX77650_getINT_GLBL();
  byte INT_CHG = MAX77650_getINT_CHG();
  byte ERCFLAG = MAX77650_getINT_CHG();  
  Serial.println("MAX77650 Interrupts: ");
  // ERCFLAG related Interrupts 0x04
  if (((INT_GLBL) >> 6) & 0b00000001) Serial.println("LDO Dropout Detector Rising Interrupt");    //Returns whether Reset Source was PWR_HLD; Return Value: 0=no event; 1=RESET Source: PWR_HLD
  if (((INT_GLBL) >> 5) & 0b00000001) Serial.println("Thermal Alarm 2 Rising Interrupt");     //Returns whether Reset Source was CRST_F; Return Value: 0=no event; 1=RESET Source: Software Cold Reset 
  if (((INT_GLBL) >> 4) & 0b00000001) Serial.println("Thermal Alarm 1 Rising Interrupt");    //Returns whether Reset Source was OFF_F; Return Value: 0=no event; 1=RESET Source: Software Off Flag
  if (((INT_GLBL) >> 3) & 0b00000001) Serial.println("nEN Rising Interrupt");    //Returns whether Reset Source was MRST; Return Value: 0=no event; 1=RESET Source: Manual Reset Timer  
  if (((INT_GLBL) >> 2) & 0b00000001) Serial.println("nEN Falling Interrupt");   //Returns whether Reset Source was SYSUVLO; Return Value: 0=no event; 1=RESET Source: System Undervoltage Lockout   
  if (((INT_GLBL) >> 1) & 0b00000001) Serial.println("GPI Rising Interrupt");   //Returns whether Reset Source was SYSOVLO; Return Value: 0=no event; 1=RESET Source: System Overvoltage Lockout   
  if ((INT_GLBL)  & 0b00000001) Serial.println("GPI Falling Interrupt");      //Returns whether Reset Source was TOVLD; Return Value: 0=no event; 1=RESET Source: Thermal Overload
  // Charger related Interrupts 0x01
  if (((INT_CHG) >> 6) & 0b00000001) Serial.println("System voltage configuration error interrupt");    //Returns whether Reset Source was PWR_HLD; Return Value: 0=no event; 1=RESET Source: PWR_HLD
  if (((INT_CHG) >> 5) & 0b00000001) Serial.println("Minimum System Voltage Regulation-loop related interrup");     //Returns whether Reset Source was CRST_F; Return Value: 0=no event; 1=RESET Source: Software Cold Reset 
  if (((INT_CHG) >> 4) & 0b00000001) Serial.println("CHGIN control-loop related interrupt");    //Returns whether Reset Source was OFF_F; Return Value: 0=no event; 1=RESET Source: Software Off Flag
  if (((INT_CHG) >> 3) & 0b00000001) Serial.println("Die junction temperature regulation interrupt");    //Returns whether Reset Source was MRST; Return Value: 0=no event; 1=RESET Source: Manual Reset Timer  
  if (((INT_CHG) >> 2) & 0b00000001) {
    Serial.println("CHGIN related interrupt");   //Returns whether Reset Source was SYSUVLO; Return Value: 0=no event; 1=RESET Source: System Undervoltage Lockout  
    chargerStatusChanged();
  }
  if (((INT_CHG) >> 1) & 0b00000001) Serial.println("Charger related interrupt");   //Returns whether Reset Source was SYSOVLO; Return Value: 0=no event; 1=RESET Source: System Overvoltage Lockout   
  if ((INT_CHG)  & 0b00000001) Serial.println("Thermistor related interrupt");      //Returns whether Reset Source was TOVLD; Return Value: 0=no event; 1=RESET Source: Thermal Overload
  // ERCFLAG related Interrupts 0x04
  if (((ERCFLAG) >> 6) & 0b00000001) Serial.println("PWR_HLD Reset");    //Returns whether Reset Source was PWR_HLD; Return Value: 0=no event; 1=RESET Source: PWR_HLD
  if (((ERCFLAG) >> 5) & 0b00000001) Serial.println("Software Cold Reset");     //Returns whether Reset Source was CRST_F; Return Value: 0=no event; 1=RESET Source: Software Cold Reset 
  if (((ERCFLAG) >> 4) & 0b00000001) Serial.println("Software Off Flag");    //Returns whether Reset Source was OFF_F; Return Value: 0=no event; 1=RESET Source: Software Off Flag
  if (((ERCFLAG) >> 3) & 0b00000001) Serial.println("Manual Reset Timer");    //Returns whether Reset Source was MRST; Return Value: 0=no event; 1=RESET Source: Manual Reset Timer  
  if (((ERCFLAG) >> 2) & 0b00000001) Serial.println("SYS Domain Undervoltage Lockout");   //Returns whether Reset Source was SYSUVLO; Return Value: 0=no event; 1=RESET Source: System Undervoltage Lockout   
  if (((ERCFLAG) >> 1) & 0b00000001) Serial.println("SYS Domain Overvoltage Lockout");   //Returns whether Reset Source was SYSOVLO; Return Value: 0=no event; 1=RESET Source: System Overvoltage Lockout   
  if ((ERCFLAG)  & 0b00000001) Serial.println("Thermal Overload");      //Returns whether Reset Source was TOVLD; Return Value: 0=no event; 1=RESET Source: Thermal Overload  
  interrupt = false;
}

void MAX77650_IRQ(void) {
  interrupt = true;
}

void chargerStatusChanged(void) {
  byte charging_status = MAX77650_getCHGIN_DTLS();
  uint8_t fg_soc = fuelGauge.get_SOC();
  
  Serial.println();
  Serial.println("CHANGING THE STATUS OF THE LED LIGHTS");
  Serial.print("Value of CHARGING details: ");
  Serial.println(charging_status, HEX);
  Serial.println();

  // (LED GREEN - SOLID COLOR)
  // The CHGIN input is okay and debounced and battery is charged between 99% and 100%. Considering this a full charge.
  if (charging_status == 0x03 && fg_soc >= 99) {
    MAX77650_setLED_FS0(0b00);      // disable red led
    MAX77650_setBRT_LED0(0b00000);
    MAX77650_setINV_LED0(false);    // LED red: phase operation
    MAX77650_setLED_FS1(0b01);      // enable green led
    MAX77650_setBRT_LED1(0b11111);  
    MAX77650_setINV_LED1(false);    // LED green: phase operation
    MAX77650_setLED_FS2(0b00);      // disable blue led
    MAX77650_setBRT_LED2(0b00000);
    MAX77650_setINV_LED2(false);    // LED blue: phase operation
  } 
  // (LED RED  - SOLID COLOR)
  // The CHGIN input is okay and debounced and battery is less than 99%
  else if (charging_status == 0x03) {
    MAX77650_setLED_FS0(0b01);      // enable red led
    MAX77650_setBRT_LED0(0b11111);
    MAX77650_setINV_LED0(false);    // LED red: phase operation
    MAX77650_setLED_FS1(0b00);      // disable green led
    MAX77650_setBRT_LED1(0b00000);
    MAX77650_setINV_LED1(false);    // LED green: phase operation
    MAX77650_setLED_FS2(0b00);      // disable blue led
    MAX77650_setBRT_LED2(0b00000);
    MAX77650_setINV_LED2(false);    // LED blue: phase operation
  } 
  // (LED RED  - FLASHING)
  // The CHGIN input is being debounced (no power accepted from CHGIN during debounce)
  else if (charging_status == 0x02){
    MAX77650_setLED_FS0(0b01);      // enable red led
    MAX77650_setBRT_LED0(0b00000);
    MAX77650_setINV_LED0(false);    // LED red: phase operation
    MAX77650_setLED_FS1(0b00);      // disable green led
    MAX77650_setBRT_LED1(0b01111);  
    MAX77650_setINV_LED1(false);    // LED green: phase operation
    MAX77650_setLED_FS2(0b00);      // disable blue led
    MAX77650_setBRT_LED2(0b00000);
    MAX77650_setINV_LED2(false);    // LED blue: phase operation
  }
  // (LED BLUE - FLASHING)
  // The CHGIN input voltage is above the OVP threshold (VCHGIN > VOVP)
  else if (charging_status == 0x01){
    MAX77650_setLED_FS0(0b00);      // disable red led
    MAX77650_setBRT_LED0(0b00000);
    MAX77650_setINV_LED0(false);    // LED red: phase operation
    MAX77650_setLED_FS1(0b00);      // enable green led
    MAX77650_setBRT_LED1(0b00000);
    MAX77650_setINV_LED1(false);    // LED green: phase operation
    MAX77650_setLED_FS2(0b10);      // disable blue led
    MAX77650_setBRT_LED2(0b11111);
    MAX77650_setINV_LED2(true);     // LED blue: phase operation
  }
  // (LED OFF - Most likely not charging)
  // The CHGIN input voltage is below the UVLO threshold (VCHGIN < VUVLO)
  else if (charging_status == 0x00){
    MAX77650_setLED_FS0(0b00);      // disable red led
    MAX77650_setBRT_LED0(0b00000);  
    MAX77650_setINV_LED0(false);    // LED red: phase operation
    MAX77650_setLED_FS1(0b00);      // disable green led
    MAX77650_setBRT_LED1(0b00000);
    MAX77650_setINV_LED1(false);    // LED green: phase operation
    MAX77650_setLED_FS2(0b00);      // enable blue led
    MAX77650_setBRT_LED2(0b00000);
    MAX77650_setINV_LED2(false);    // LED blue: phase operation
  }
}


// init variables for loop below
int fg_soc = 0;
int fg_temp = 0;
float fg_vcell = 0;
float fg_current = 0;
float fg_tte = 0;

void loop(void)
{  
  // Read the State of charge of the battery using the MAX17055 Fuel Gauge
  fg_soc = fuelGauge.get_SOC();
  Serial.print("State of charge: ");
  Serial.println(fg_soc, DEC);

  // Read the temperature on board the MAX17055 Fuel Gauge
  fg_temp = fuelGauge.get_temperature();
  Serial.print("Temperature: ");
  Serial.println(fg_temp, DEC);
  
  // Read the voltage of the battery connected to the MAX17055 Fuel Gauge
  fg_vcell = (float)(fuelGauge.get_Vcell())/1000000;
  Serial.print("Voltage: ");
  Serial.println(fg_vcell);


  // Read the current consumed by the battery connected to the MAX17055 Fuel Gauge
  fg_current = (float)(fuelGauge.get_Current(design_param))/1000000;
  Serial.print("Current: ");
  Serial.println(fg_current);

  Serial.println();
  delay(1000);

  // Check if interupt happened
  if (interrupt) check_IRQ_src();
}


