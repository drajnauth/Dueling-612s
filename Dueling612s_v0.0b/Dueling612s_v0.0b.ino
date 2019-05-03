/*

  Program Written by Dave Rajnauth, VE3OOI to control the Dueling 612 Transceiver.
  
  Software is licensed (Non-Exclusive Licence) under a Creative Commons Attribution 4.0 International License.

*/
 
#include "Arduino.h"

#include <stdint.h>
#include <avr/eeprom.h>        // Needed for storeing calibration to Arduino EEPROM
#include <Wire.h>              // Needed to communitate I2C to Si5351
#include <SPI.h>               // Needed to communitate I2C to Si5351
#include <EEPROM.h>

#include "VE3OOI_Si5351_Signal_Generator.h"   // Defines for this program
#include "LCD.h"
#include "Encoder.h"
#include "Timer.h"
#include "VE3OOI_Si5351_v2.1.h"


#ifndef REMOVE_CLI
#include "UART.h"                             // VE3OOI Serial Interface Routines (TTY Commands)

// These variables are defined in UART.cpp and used for Serial interface
// rbuff is used to store all keystrokes which is parsed by Execute()
// commands[] and numbers[] store all characters or numbers entered provided they
// are separated by spaces.
// ctr is counter used to process entries
// command_entries contains the total number of charaters/numbers entered
char rbuff[RBUFF];
char commands[MAX_COMMAND_ENTRIES];
unsigned char command_entries;
unsigned long numbers[MAX_COMMAND_ENTRIES];
unsigned char ctr;
#endif // REMOVE_CLI


volatile unsigned long flags;


// LCD Menu
volatile unsigned char MenuSelection, RadioSelection;

char RootMenuOptions[MAXMENU_ITEMS][MAXMENU_LEN] = {
  {"RADIO     "},
  {"BFO CORREC"},
  {"LO CORRECT"},
  {"Si CORRECT"},
  {"Sm BASELIN"},
  {"Sm CORRECT"},
  {"Fw CORRECT"},
  {"Rv CORRECT"},
  {"FACT RESET"}
};



char header1[HEADER1] = {'D', 'U', 'E', 'L', 'I', 'N', 'G', ' ', '6', '1', '2', 's', ' ', ' ', '0', '.', '0', 'b', ' ', 0x0};
char header2[HEADER2] = {' ', '(', 'C', ')', 'V', 'E', '3', 'O', 'O', 'I', 0x0};
#define VERSION 0

char clkentry [CLKENTRYLEN];

char prompt[6] = {0xa, 0xd, ':', '>', ' ', 0x0};
char ovflmsg[9] = {'O', 'v', 'e', 'r', 'f', 'l', 'o', 'w', 0x0};
char errmsg[4] = {'E', 'r', 'r', 0x0};

char clearlcdline[LCD_CLEAR_LINE_LENGTH];
char clearlcderrmsg[LCD_ERROR_MSG_LENGTH];

char okmsg[LCD_ERROR_MSG_LENGTH];

// Specific radio parameters
D612_Struct sg;
Band_Struct mem;

unsigned char oldBandOffset;
unsigned char oldRxTx;
unsigned char oldMode;
unsigned char updateNeeded;
unsigned long memoryTime;
unsigned long sMeterTime;
unsigned int sMeter;
unsigned int fwdPower;
unsigned int revPower;
// Calibration and memory slot variable
volatile int rotaryNumber;
volatile int rotaryInc;
int origOffset;
unsigned long freq;
unsigned long pll_freq;
unsigned long origFreq;
unsigned long long workingUL;


// the setup function runs once when you press reset or power the board
void setup() {
  // define the baud rate for TTY communications. Note CR and LF must be sent by terminal program
  Serial.begin(9600);



  pinMode(SMETER_PIN, INPUT);
  pinMode(FWD_PIN, INPUT);
  pinMode(REV_PIN, INPUT);
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite (PTT_PIN, 0);  // Ensure PTT is off

  SetupLCD ();
  
#ifndef REMOVE_CLI
  ResetSerial ();
#endif // REMOVE_CLI

  LoadMemory ();

  if (sg.correction > 1000 || sg.correction < -1000) {
    sg.correction = 52;
  }
  
  setupSi5351(sg.correction);

  SetupEncoder();

  Reset ();

  randomSeed(analogRead(0));
}





// the loop function runs over and over again forever
void loop()
{

  
#ifndef REMOVE_CLI
  // Look for characters entered from the keyboard and process them
  // This function is part of the UART package.
  ProcessSerial ();
#endif // REMOVE_CLI

  if (flags & MASTER_RESET) {
    Reset();
  }

  if (flags & PTT_PUSHED) {
    flags &= ~PTT_PUSHED;
    if (sg.RxTx == TRANSMIT) {
      PTT(0);
    } else {
      PTT(1);
    }
    LCDDisplayRxTx();
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  if (flags & PBUTTON2_PUSHED) {
    if (flags & OPTION_CHANGED) {
      sg.BandOffset = oldBandOffset;
      sg.RxTx = oldRxTx;
      mem.Mode = oldMode;
      LCDDisplayBand();
      LCDDisplayRxTx();
      LCDDisplayMode();
      flags &= ~OPTION_CHANGED;
    }
    if (flags & CHANGE_BFO || flags & CHANGE_LO) {
      ResetMenuMode();
    }
    ChangeRadioSelection();
    flags &= ~PBUTTON2_PUSHED;
    flags &= ~UPDATE_MEMORY;
    digitalWrite(LED_BUILTIN, LOW);
  }
 
  if (flags & CHANGE_FREQ || flags & CHANGE_VFOB) {
    RadioUpdateFreq();
    
  } else if (flags & CHANGE_BAND) {
    RadioUpdateOther();
    
  } else if (flags & CHANGE_RXTX) {
    RadioUpdateOther();
    
  } else if (flags & CHANGE_MODE) {
    RadioUpdateOther();

  } else if (flags & CHANGE_MENU) {
    MenuDisplayMode();
    
  } else if (flags & CHANGE_SmC) {
     updateNeeded = GetRotaryNumber (-10, 10, 1);
     if (updateNeeded == 1) {
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
     } else if (updateNeeded == 0xFF) {
        sg.SmeterCorrection = rotaryNumber;
        ClearFlags();
        LCDClearLine(3);
        EEPROM.put(0, sg);
        flags |= CHANGE_MENU;
        LCDDisplayMenuOption (MenuSelection);
     }
    
  } else if (flags & CHANGE_SmB) {
     updateNeeded = GetRotaryNumber (1, 100, 10);
     if (updateNeeded == 1) {
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
     } else if (updateNeeded == 0xFF) {
        sg.SmeterBaseline = rotaryNumber;
        ClearFlags();
        LCDClearLine(3);
        EEPROM.put(0, sg);
        flags |= CHANGE_MENU;
        LCDDisplayMenuOption (MenuSelection);      
     }
    
  } else if (flags & CHANGE_Fw || flags & CHANGE_Rw) {
     updateNeeded = GetRotaryNumber (1, 900, 10);
     if (updateNeeded == 1) {
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
     } else if (updateNeeded == 0xFF) {
        if (flags & CHANGE_Fw) {
          sg.FwdCorrection = rotaryNumber;
        } else {
          sg.RevCorrection = rotaryNumber;
        }
        ClearFlags();
        LCDClearLine(3);
        EEPROM.put(0, sg);
        flags |= CHANGE_MENU;
        LCDDisplayMenuOption (MenuSelection);      
     }
    
  } else if (flags & CHANGE_Si) {
     updateNeeded = GetRotaryNumber (-500, 500, 10);
     if (updateNeeded == 1) {
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
     } else if (updateNeeded == 0xFF) {
        sg.correction = rotaryNumber;
        ClearFlags();
        LCDClearLine(3);
        EEPROM.put(0, sg);
        flags |= CHANGE_MENU;
        LCDDisplayMenuOption (MenuSelection);      
     }
   
  } else if (flags & CHANGE_BFO) {
     updateNeeded = GetRotaryNumber (-4000, 4000, 100);
     if (updateNeeded == 1) {
        LCDDisplayMenuLargeNumbner (rotaryNumber, rotaryInc);
        mem.BFOFreq = origFreq + rotaryNumber;
        UpdateBFOFrequency();
        mem.BFOFreq = origFreq;
     } else if (updateNeeded == 0xFF) {
        mem.BFOFreq = origFreq;
        if (mem.Mode == LSB_MODE) {
          mem.LSBBFOOffset = rotaryNumber;
        } else if (mem.Mode == USB_MODE) {
          mem.USBBFOOffset = rotaryNumber;
        } else if (mem.Mode == TUNE_MODE) {
          mem.CWBFOOffset = rotaryNumber;
        }
        SetBFOFrequency ();
        UpdateBFOFrequency();
        ClearFlags();
        LCDClearLine(3);
        putBandData (sg.BandOffset);
        flags |= CHANGE_MENU;
        LCDDisplayMenuOption (MenuSelection);      
     }
    
  } else if (flags & CHANGE_LO) {
     updateNeeded = GetRotaryNumber (-4000, 4000, 100);
     if (updateNeeded == 1) {
        if ((rotaryNumber+mem.Freq) > mem.FreqHigh) {
          rotaryNumber -= rotaryInc;
        } 
        if ((mem.Freq-rotaryNumber) < mem.FreqLow) {
          rotaryNumber += rotaryInc;
        } 
        mem.LOOffset = origOffset + rotaryNumber;
        LCDDisplayMenuLargeNumbner (rotaryNumber, rotaryInc);
        UpdateLOFrequency();
        mem.LOOffset = origOffset;
     } else if (updateNeeded == 0xFF) {
        mem.LOOffset = rotaryNumber;
        UpdateLOFrequency();
        ClearFlags();
        LCDClearLine(3);
        putBandData (sg.BandOffset);
        flags |= CHANGE_MENU;
        LCDDisplayMenuOption (MenuSelection);      
     }
    
  }
  

  if (flags & UPDATE_MEMORY) {
    memoryTime = millis();
  } else {
    if ((millis()-memoryTime) > UPDATE_MEMORY_THRESHOLD ) {
      flags &= ~UPDATE_MEMORY;
      putBandData (sg.BandOffset); 
      EEPROM.put(0, sg);   
      memoryTime = millis();  
      Serial.println ("WR"); 
    }
  }

  if ((millis()-sMeterTime) > UPDATE_SMETER_THRESHOLD ) {
    if (sg.RxTx == RECEIVE) {
      updateSmeter();
    } else {
      updatePmeter();
    }
    sMeterTime = millis();   
    RestoreLCDSelectLine();
  }


}

void ChangeRadioSelection (void)
{
  RadioSelection++;
  if (RadioSelection >= MAX_RADIO_SELECTION) RadioSelection = 0;
  ClearFlags ();
  switch (RadioSelection) {
    case RADIO_FREQ_MODE:
      flags |= CHANGE_FREQ;
      LCDDisplayFreq ();
      break;

    case RADIO_MODE:
      flags |= CHANGE_MODE;
      LCDDisplayMode();
      break;

    case RADIO_VFOB_MODE:
      flags |= CHANGE_VFOB;
      LCDDisplayVFOBFreq ();
      break;

    case RADIO_MENU_MODE:
      flags |= CHANGE_MENU;
      LCDDisplayMenuOption (MenuSelection);
      break;

    case RADIO_BAND_MODE:
      flags |= CHANGE_BAND;
      LCDDisplayBand ();
      break;

    case RADIO_RXTX_MODE:
      flags |= CHANGE_RXTX;
      LCDDisplayRxTx ();
      break;
  }
}

void MenuDisplayMode (void)
{

  if (flags & ROTARY_CW) {
    MenuSelection++;
    if (MenuSelection > MAXMENU_ITEMS) {
      MenuSelection = 0;
    }

  } else if (flags & ROTARY_CCW) {
    if (MenuSelection) MenuSelection--;
    else MenuSelection = MAXMENU_ITEMS-1;
  }

  if ( (flags & ROTARY_CW) || (flags & ROTARY_CCW) ) {
    LCDDisplayMenuItem(MenuSelection);
    flags &= ~ROTARY_CW;
    flags &= ~ROTARY_CCW;
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (flags & ROTARY_PUSH) {
    flags &= ~ROTARY_PUSH;
  }

  if (flags & PBUTTON1_PUSHED) {    // execute
    flags &= ~PBUTTON1_PUSHED;
    digitalWrite(LED_BUILTIN, LOW);
    doMenuSelection();  
  }
  
  if (flags & PBUTTON2_PUSHED) {
    ResetMenuMode();
  }

}

void doMenuSelection (void) 
{
    switch (MenuSelection) {
      case RADIO:
        putBandData (sg.BandOffset); 
        EEPROM.put(0, sg);   
        memoryTime = millis();  
        Serial.println ("WR"); 
        break;
        
      case BFO_CORRECT:
        ClearFlags();
        if (mem.Mode == LSB_MODE) {
          rotaryNumber = mem.LSBBFOOffset;
        } else if (mem.Mode == USB_MODE) {
          rotaryNumber = mem.USBBFOOffset;
        } else if (mem.Mode == TUNE_MODE) {
          rotaryNumber = mem.CWBFOOffset;
        }
        origFreq = mem.BFOFreq;
        rotaryInc = 100;        
        LCDDisplayMenuLargeNumbner (rotaryNumber, rotaryInc);
        flags |= CHANGE_BFO;
        break;
        
      case LO_CORRECT:
        ClearFlags();
        rotaryNumber = mem.LOOffset;
        origOffset = rotaryNumber;
        rotaryInc = 100;        
        LCDDisplayMenuLargeNumbner (rotaryNumber, rotaryInc);
        flags |= CHANGE_LO;
        break;
        
      case Si_CORRECT:
        ClearFlags();
        rotaryNumber = sg.correction;
        rotaryInc = 10;        
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
        flags |= CHANGE_Si;
        break;
        
      case Sm_BASELIN:
        ClearFlags();
        sMeter = getSmeterValue (SMETER_BASELINE_SAMPLES);
        if (sMeter == 0) sMeter = 1;
        sg.SmeterBaseline = sMeter;
        rotaryNumber = (int)sg.SmeterBaseline;
        rotaryInc = 1;        
        flags |= CHANGE_SmB;
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
        break;
        
      case Sm_CORRECT:
        ClearFlags();
        rotaryNumber = sg.SmeterCorrection;
        rotaryInc = 1;        
        flags |= CHANGE_SmC;
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
        break;
        
      case Fw_CORRECT:
        ClearFlags();
        if (sg.RxTx == TRANSMIT) {
          getPmeterValue(SMETER_BASELINE_SAMPLES);
          rotaryNumber = fwdPower;
        } else {
          rotaryNumber = sg.FwdCorrection;          
        }
        rotaryInc = 10;        
        flags |= CHANGE_Fw;
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
        break;
        
      case Rv_CORRECT:
        ClearFlags();
        if (sg.RxTx == TRANSMIT) {
          getPmeterValue(SMETER_BASELINE_SAMPLES);
          rotaryNumber = revPower;
        } else {
          rotaryNumber = sg.RevCorrection;          
        }
        rotaryInc = 1;        
        flags |= CHANGE_Rw;
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
        break;

      case FACT_RESET:
        resetFlash ();
        Reset();
        break;          
    }
}

void ResetMenuMode (void) 
{
    ClearFlags();
    LCDClearLine(3);
    flags |= CHANGE_MENU;
    LCDDisplayMenuOption (MenuSelection);      
    UpdateLOFrequency();
    UpdateBFOFrequency();     
  
}

unsigned char GetRotaryNumber (int lnum, int hnum, int maxinc)
{
  unsigned char change;
  change = 0;

  if (flags & ROTARY_CW) {
    rotaryNumber += rotaryInc;
    if (rotaryNumber > hnum) rotaryNumber = hnum; 

  } else if (flags & ROTARY_CCW) {
    rotaryNumber -= rotaryInc;
    if (rotaryNumber < lnum) rotaryNumber = lnum;     
  }
  

  if ( (flags & ROTARY_CW) || (flags & ROTARY_CCW) ) {
    flags &= ~ROTARY_CW;
    flags &= ~ROTARY_CCW;
    change = 1;
    digitalWrite(LED_BUILTIN, LOW);

  }

  if (flags & ROTARY_PUSH) {
    rotaryInc *= 10;
    if (rotaryInc > maxinc) rotaryInc = 1;
    flags &= ~ROTARY_PUSH;
  }

  if (flags & PBUTTON1_PUSHED) {
    change = 0xFF;
    flags &= ~PBUTTON1_PUSHED;
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  return change;

}


void RadioUpdateFreq (void)
{  
  freq = mem.Freq;
  if (flags & CHANGE_VFOB) {
    freq = mem.FreqB;
  }

  if (flags & ROTARY_CW) {
    freq += mem.FreqInc;
    if (freq > mem.FreqHigh) {
      freq = mem.FreqHigh;
    }

  } else if (flags & ROTARY_CCW) {
    freq -= mem.FreqInc;
    if (freq < mem.FreqLow) {
      freq = mem.FreqLow;
    } 
  }

  if ( (flags & ROTARY_CW) || (flags & ROTARY_CCW) ) {
    if (flags & CHANGE_FREQ) {
      mem.Freq = freq;
      mem.FreqA = freq;
      LCDDisplayFreq ();
      UpdateLOFrequency ();
      flags |= UPDATE_MEMORY;
    } else if (flags & CHANGE_VFOB) {
      mem.FreqB = freq;
      LCDDisplayVFOBFreq ();    
      flags |= UPDATE_MEMORY;
    }
    flags &= ~ROTARY_CW;
    flags &= ~ROTARY_CCW;

    digitalWrite(LED_BUILTIN, LOW);
  }

  if (flags & ROTARY_PUSH) {
    mem.FreqInc *= 10;
    if (mem.FreqInc > MAXIMUM_FREQUENCY_MULTIPLIER) mem.FreqInc = MINIMUM_FREQUENCY_MULTIPLIER;
    flags &= ~ROTARY_PUSH;
    if (flags & CHANGE_FREQ) {
      LCDDisplayFreq ();
    } else if (flags & CHANGE_VFOB) {
      LCDDisplayVFOBFreq ();    
    }
 }

  if (flags & PBUTTON1_PUSHED) {    // execute
    flags &= ~PBUTTON1_PUSHED;
    mem.FreqA = mem.FreqB;
    mem.FreqB = mem.Freq;
    mem.Freq = mem.FreqA;
    if (mem.VFO == 'A') {
      mem.VFO = 'B';
    } else {
      mem.VFO = 'A';
    }
    LCDDisplayVFOBFreq ();    
    LCDDisplayFreq ();
    UpdateLOFrequency ();
    RadioSelection = 0;
    flags |= UPDATE_MEMORY;
    digitalWrite(LED_BUILTIN, LOW);
  }
}


void RadioUpdateOther (void)
{

  if ( !(flags & OPTION_CHANGED) ) {
    oldBandOffset = sg.BandOffset;
    oldRxTx = sg.RxTx;
    oldMode = mem.Mode;
  }

  if (flags & ROTARY_CW) {
    if (flags & CHANGE_BAND) {
      sg.BandOffset++;
      if (sg.BandOffset >=MAX_BANDS) sg.BandOffset = 0;
            
    } else if (flags & CHANGE_MODE) {
      mem.Mode++;
      if (mem.Mode >= MAX_MODES) mem.Mode = 0;
    }

  } else if (flags & ROTARY_CCW) {
    if (flags & CHANGE_BAND) {
      if (sg.BandOffset) sg.BandOffset--;
      else sg.BandOffset = MAX_BANDS - 1;
     
    } else if (flags & CHANGE_MODE) {
      if (mem.Mode) mem.Mode--;
      else mem.Mode = MAX_MODES - 1;
    }
  }

  if ( (flags & ROTARY_CW) || (flags & ROTARY_CCW) ) {
    flags |= OPTION_CHANGED;
    if (flags & CHANGE_BAND) {
      LCDDisplayBand();
      
    } else if (flags & CHANGE_RXTX) {
      LCDDisplayRxTx();
      
    } else if (flags & CHANGE_MODE) {
      LCDDisplayMode();
    }
    flags &= ~ROTARY_CW;
    flags &= ~ROTARY_CCW;
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (flags & ROTARY_PUSH) {
    flags &= ~ROTARY_PUSH;
  }

  if (flags & PBUTTON1_PUSHED) {    // execute
    flags &= ~PBUTTON1_PUSHED;
    digitalWrite(LED_BUILTIN, LOW);

    if (flags & CHANGE_RXTX) {
      if (sg.RxTx == TRANSMIT) {
        PTT(0);
      } else {
        PTT(1);
      }
      LCDDisplayRxTx();
      return;
    
    } else if (flags & CHANGE_BAND) {
      getBandData (sg.BandOffset);
        
    } else if (flags & CHANGE_MODE) {
      SetBFOFrequency ();
    }
      
    ClearFlags();
    flags |= CHANGE_FREQ;
    RefreshLCD();
    SetupBandMSNDivers();
    UpdateBFOFrequency (); 
    UpdateLOFrequency ();
    RadioSelection = 0;
    flags |= UPDATE_MEMORY;
  }
}

void SetBFOFrequency (void) 
{
  if (mem.Mode == LSB_MODE) mem.BFOFreq = mem.LSBBFOFreq + mem.LSBBFOOffset;
  else if (mem.Mode == USB_MODE) mem.BFOFreq = mem.USBBFOFreq + mem.USBBFOOffset;  
  else if (mem.Mode == TUNE_MODE) mem.BFOFreq = mem.CWBFOFreq + mem.CWBFOOffset;
}

void PTT (unsigned char ptt) 
{
  if (ptt) {
    LCDClearSWR();
    sg.RxTx = TRANSMIT;    
    SetupBandMSNDivers();
    UpdateBFOFrequency();
    UpdateLOFrequency();
    drainSmeterCap(10);
    digitalWrite (PTT_PIN, HIGH);  // PTT is on
  } else {
    digitalWrite (PTT_PIN, LOW);  // PTT is off
    LCDClearSWR();
    sg.RxTx = RECEIVE;
    SetupBandMSNDivers();
    UpdateBFOFrequency();
    UpdateLOFrequency();
    drainSmeterCap(1);
  } 
}


void ClearFlags ()
{
  flags &= ~CHANGE_FREQ;
  flags &= ~CHANGE_BAND;
  flags &= ~CHANGE_RXTX;
  flags &= ~CHANGE_MODE;
  flags &= ~CHANGE_VFOB;
  flags &= ~CHANGE_MENU;
  flags &= ~CHANGE_SmB;
  flags &= ~CHANGE_SmC;
  flags &= ~CHANGE_Fw;
  flags &= ~CHANGE_Rw;
  flags &= ~CHANGE_Si;
  flags &= ~CHANGE_BFO;
  flags &= ~CHANGE_LO;
  flags &= ~OPTION_CHANGED;
  flags &= ~UPDATE_MEMORY;
}


unsigned char FrequencyDigitUpdate (long inc)
{
  switch (inc) {
    case 1:
      return 8;
      break;

    case 10:
      return 7;
      break;

    case 100:
      return 6;
      break;

    case 1000:
      return 5;
      break;

    case 10000:
      return 4;
      break;

    case 100000:
      return 3;
      break;

    case 1000000:
      return 2;
      break;

    case 10000000:
      return 1;
      break;

    default:
      return 1;
      break;

  }
}

void SetupBandMSNDivers (void) 
{
  freq = mem.Freq + mem.BFOFreq + mem.LOOffset;
  pll_freq = freq * mem.MSN_a;
  if (sg.RxTx == RECEIVE) {    
    ProgramSi5351MSN (RX_LO_CLK, SI_PLL_A, pll_freq, freq); 
    ProgramSi5351MSN (TX_LO_CLK, SI_PLL_B, pll_freq, mem.BFOFreq); 
  } else {
    ProgramSi5351MSN (TX_LO_CLK, SI_PLL_A, pll_freq, freq); 
    ProgramSi5351MSN (RX_LO_CLK, SI_PLL_B, pll_freq, mem.BFOFreq); 
  }  

  pll_freq = mem.BFOFreq * mem.PLL_a;
  if (sg.RxTx == RECEIVE) {    
    ProgramSi5351MSN (TX_LO_CLK, SI_PLL_B, pll_freq, mem.BFOFreq); 
  } else {
    ProgramSi5351MSN (RX_LO_CLK, SI_PLL_B, pll_freq, mem.BFOFreq); 
  }  

  drainSmeterCap(50);  
}


void UpdateLOFrequency (void)
{
  workingUL = (unsigned long long)mem.Freq + (unsigned long long)mem.BFOFreq + (unsigned long long)mem.LOOffset;
  workingUL *= (unsigned long long) mem.MSN_a;
  pll_freq = (unsigned long) workingUL; 
  ProgramSi5351PLL (SI_PLL_A, pll_freq);
}

void UpdateBFOFrequency (void)
{
  workingUL = (unsigned long long)mem.BFOFreq;
  workingUL *= (unsigned long long) mem.PLL_a;
  pll_freq = (unsigned long) workingUL; 
  ProgramSi5351PLL (SI_PLL_B, pll_freq); 
}


void RefreshLCD (void)
{
  LCDClearScreen(); 
  LCDDisplayBand();
  LCDDisplayRxTx();
  LCDDisplayMode();
  LCDDisplaySmeter(1);
  if (sg.RxTx == RECEIVE) {
    LCDDisplaySValue (1);
  } else {
    LCDDisplaySWR (0);
  }
  LCDDisplayMenuOption(0);
  LCDDisplayVFOBFreq ();
  LCDDisplayFreq();
}

void LoadMemory (void)
{
  // Read sg from EEPROM
  memset ((char *)&sg, 0, sizeof (sg));
  memset ((char *)&mem, 0, sizeof (mem));
  EEPROM.get(0, sg);

  if (sg.flags != D612_HEADER) {
    Serial.println ("D612 Reset");
    resetFlash();
  }

  if (sg.BandOffset >= MAX_BANDS) {
     Serial.println ("Offset Reset");
     resetFlash();
  }

  getBandData (sg.BandOffset);
  if (mem.flags != BAND_HEADER) {
     Serial.println ("BHEAD Reset");
     resetFlash();
     getBandData (sg.BandOffset);
  } 
  
#ifndef REMOVE_CLI
  printGlobalMem();
  printCurrentMem();
#endif

}

void updatePmeter (void)
{
  double swr;
  
  getPmeterValue(SWR_SAMPLES);
  
  if (!fwdPower || !revPower || fwdPower == revPower || revPower > fwdPower){
    swr = 0;
  } else {
    swr = (double)(fwdPower+revPower) / (double)(fwdPower-revPower);
  }
  LCDDisplaySWR(swr);

  swr = (double)fwdPower * (double)fwdPower;
  swr /= (double)sg.FwdCorrection;
  swr *= (double)FWD_POWER_REFERENCE;
  swr /= (double)sg.FwdCorrection;
  
  if (swr >= MAX_SMETER_BLOCKS) swr = MAX_SMETER_BLOCKS-1; 
  if (swr < 1) swr = 0; 
  LCDDisplaySmeter((unsigned char)swr);
}

void getPmeterValue (unsigned char samples)
{
  unsigned char i;
  unsigned long ftotal, rtotal;
  
  ftotal = rtotal = 0;
  for (i=0; i<samples; i++) {
    ftotal += (unsigned long)analogRead(FWD_PIN);
    rtotal += (unsigned long)analogRead(REV_PIN);
  }

  fwdPower = (unsigned int) (ftotal/(unsigned long)samples);
  revPower = (unsigned int) (rtotal/(unsigned long)samples);

}  


void updateSmeter (void)
{
  int value;
  
  sMeter = getSmeterValue(SMETER_SAMPLES);
  if (sMeter < sg.SmeterBaseline) sMeter = sg.SmeterBaseline;
  value = (int) (20 * log( (double)sMeter/(double)sg.SmeterBaseline));
  value /= 6;
  value += sg.SmeterCorrection;
  if (value <= 0) sMeter = 1;
  else sMeter = value;
  if (sMeter >= MAX_SMETER_BLOCKS) sMeter = MAX_SMETER_BLOCKS-1;
  LCDDisplaySmeter(sMeter);
  LCDDisplaySValue(sMeter);
}

unsigned int getSmeterValue (unsigned char samples)
{
  unsigned char i;
  unsigned long total;
  
  total = 0;
  for (i=0; i<samples; i++) {
    total += (unsigned long)analogRead(SMETER_PIN);
  }

  total /= (unsigned long)samples;

  Serial.println (total);
 
  return (unsigned int)total; 
}  

void drainSmeterCap (unsigned int dly) 
{
  pinMode(SMETER_PIN, OUTPUT);
  digitalWrite (SMETER_PIN, LOW);
  delay (dly);
  pinMode(SMETER_PIN, INPUT);

}


void Reset (void)
{
  char i;
  
#ifndef REMOVE_CLI
  ResetSerial();
  Serial.print (header1);
  Serial.println (header2);
//  Serial.print (prompt);
  Serial.flush();
#endif // REMOVE_CLI

  ResetSi5351();

  ResetEncoder();

  LoadMemory ();

  // LCD Menu
  LCDDisplayHeader();

  MenuSelection = 0;
  RadioSelection = 0;
  oldBandOffset = 0;
  oldRxTx = 0;
  oldMode = 0;
  memoryTime = millis();   
  sMeterTime = millis();   

  rotaryNumber = 0;
  rotaryInc = 1;

  flags = CHANGE_FREQ;
  sg.RxTx = RECEIVE;
  RefreshLCD();
  SetBFOFrequency ();
  SetupBandMSNDivers();
  UpdateLOFrequency ();
  UpdateBFOFrequency ();
  
  i = 0;
  while (i<=MAX_SMETER_BLOCKS) {
    LCDDisplaySmeter(i++);
    delay(100); 
  }
  i = MAX_SMETER_BLOCKS;
  while (i) {
    LCDDisplaySmeter(i--);
    delay(100); 
  }
  LCDDisplayFreq ();
}

long absl (long v)
{
  if (v < 0) return (-v);
  else return v;
}


void resetFlash (void)
{
    memset ((char *)&sg, 0, sizeof (sg));
    memset ((char *)&mem, 0, sizeof (mem));
    
    // Header
    sg.flags = D612_HEADER;
    sg.BandOffset = 2;
    sg.VFO = VFOA;
    sg.RxTx = RECEIVE;
    sg.SmeterBaseline = SMETER_BASELINE_VALUE;
    sg.SmeterCorrection = SMETER_CORRECTION_VALUE;
    sg.FwdCorrection = FWD_CORRECTION;
    sg.RevCorrection = REV_CORRECTION;
    sg.correction = 52;   // can be + or -
    EEPROM.put(0, sg);

    // 80m
    mem.flags = BAND_HEADER;
    mem.FreqLow = 3500000;
    mem.FreqHigh = 4000000;
    mem.FreqA = 3501000;
    mem.FreqB = 3950000;
    mem.Freq = 3501000;
    mem.BFOFreq = DEFAULT_LSBBFO;
    mem.USBBFOFreq = DEFAULT_USBBFO;
    mem.LSBBFOFreq = DEFAULT_LSBBFO;
    mem.CWBFOFreq = DEFAULT_CWBFO;
    mem.FreqInc = DEFAULT_FREQUENCY_MULTIPLIER;
    mem.LOOffset = 0;
    mem.USBBFOOffset = 0;
    mem.LSBBFOOffset = 0;
    mem.CWBFOOffset = 0;
    mem.Mode = LSB_MODE;
    mem.VFO = 'A';
    mem.MSN_a = 83;
    mem.PLL_a = 142;
    putBandData (BAND80M); 

    // 60m
    mem.flags = BAND_HEADER;
    mem.FreqLow = 5330500;
    mem.FreqHigh = 5403500;
    mem.FreqA = 5346500;
    mem.FreqB = 5371500;
    mem.Freq = 5346500;
    mem.BFOFreq = DEFAULT_LSBBFO;
    mem.USBBFOFreq = DEFAULT_USBBFO;
    mem.LSBBFOFreq = DEFAULT_LSBBFO;
    mem.CWBFOFreq = DEFAULT_CWBFO;
    mem.FreqInc = DEFAULT_FREQUENCY_MULTIPLIER;
    mem.LOOffset = 0;
    mem.USBBFOOffset = 0;
    mem.LSBBFOOffset = 0;
    mem.CWBFOOffset = 0;
    mem.Mode = LSB_MODE;
    mem.VFO = 'A';
    mem.MSN_a = 68;
    mem.PLL_a = 142;
    putBandData (BAND60M); 

    // 40m
    mem.flags = BAND_HEADER;
    mem.FreqLow = 7000000;
    mem.FreqHigh = 7300000;
    mem.FreqA = 7101000;
    mem.FreqB = 7175000;
    mem.Freq = 7101000;
    mem.FreqInc = DEFAULT_FREQUENCY_MULTIPLIER;
    mem.BFOFreq = DEFAULT_LSBBFO;
    mem.USBBFOFreq = DEFAULT_USBBFO;
    mem.LSBBFOFreq = DEFAULT_LSBBFO;
    mem.CWBFOFreq = DEFAULT_CWBFO;
    mem.LOOffset = 0;
    mem.USBBFOOffset = 0;
    mem.LSBBFOOffset = 0;
    mem.CWBFOOffset = 0;
    mem.Mode = LSB_MODE;
    mem.VFO = 'A';
    mem.MSN_a = 58;
    mem.PLL_a = 142;
    putBandData (BAND40M); 

    // 20m
    mem.flags = BAND_HEADER;
    mem.FreqLow = 14000000;
    mem.FreqHigh = 14350000;
    mem.FreqA = 14099000;
    mem.FreqB = 14343000;
    mem.Freq = 14099000;
    mem.FreqInc = DEFAULT_FREQUENCY_MULTIPLIER;
    mem.BFOFreq = DEFAULT_USBBFO;
    mem.USBBFOFreq = DEFAULT_USBBFO;
    mem.LSBBFOFreq = DEFAULT_LSBBFO;
    mem.CWBFOFreq = DEFAULT_CWBFO;
    mem.LOOffset = 0;
    mem.USBBFOOffset = 0;
    mem.LSBBFOOffset = 0;
    mem.CWBFOOffset = 0;
    mem.Mode = USB_MODE;
    mem.VFO = 'A';
    mem.MSN_a = 36;
    mem.PLL_a = 142;
    putBandData (BAND20M); 

    // 17m
    mem.flags = BAND_HEADER;
    mem.FreqLow = 18068000;
    mem.FreqHigh = 18168000;
    mem.FreqA = 18099000;
    mem.FreqB = 18150000;
    mem.Freq = 18099000;
    mem.FreqInc = DEFAULT_FREQUENCY_MULTIPLIER;
    mem.BFOFreq = DEFAULT_USBBFO;
    mem.USBBFOFreq = DEFAULT_USBBFO;
    mem.LSBBFOFreq = DEFAULT_LSBBFO;
    mem.CWBFOFreq = DEFAULT_CWBFO;
    mem.LOOffset = 0;
    mem.USBBFOOffset = 0;
    mem.LSBBFOOffset = 0;
    mem.CWBFOOffset = 0;
    mem.Mode = USB_MODE;
    mem.VFO = 'A';
    mem.MSN_a = 30;
    mem.PLL_a = 142;
    putBandData (BAND17M); 

    // 15m
    mem.flags = BAND_HEADER;
    mem.FreqLow = 21000000;
    mem.FreqHigh = 21450000;
    mem.FreqA = 21099000;
    mem.FreqB = 21343000;
    mem.Freq = 21099000;
    mem.FreqInc = DEFAULT_FREQUENCY_MULTIPLIER;
    mem.BFOFreq = DEFAULT_USBBFO;
    mem.USBBFOFreq = DEFAULT_USBBFO;
    mem.LSBBFOFreq = DEFAULT_LSBBFO;
    mem.CWBFOFreq = DEFAULT_CWBFO;
    mem.LOOffset = 0;
    mem.USBBFOOffset = 0;
    mem.LSBBFOOffset = 0;
    mem.CWBFOOffset = 0;
    mem.Mode = USB_MODE;
    mem.VFO = 'A';
    mem.MSN_a = 27;
    mem.PLL_a = 142;
    putBandData (BAND15M); 

}


//Debug these...there are not working correctly

void getBandData (unsigned char band)
{
  EEPROM.get ( (sizeof(sg)+sizeof(mem)*band), mem); 
}


void putBandData (unsigned char band)
{
  EEPROM.put ( (sizeof(sg)+sizeof(mem)*band), mem); 
  
}

#ifndef REMOVE_CLI

// Place program specific content here
void ExecuteSerial (char *str)
{
  // num defined the actual number of entries process from the serial buffer
  // i is a generic counter
  unsigned char num;
  unsigned char i;
  double expo;

  // This function called when serial input in present in the serial buffer
  // The serial buffer is parsed and characters and numbers are scraped and entered
  // in the commands[] and numbers[] variables.
  num = ParseSerial (str);

  // Process the commands
  // Note: Whenever a parameter is stated as [CLK] the square brackets are not entered. The square brackets means
  // that this is a command line parameter entered after the command.
  // E.g. F [CLK] [FREQ] would be mean "F 0 7000000" is entered (no square brackets entered)
  switch (commands[0]) {

    // Calibrate the Si5351.
    // Syntax: C [CAL] [FREQ], where CAL is the new Calibration value and FREQ is the frequency to output
    // Syntax: C , If no parameters specified, it will display current calibration value
    // Bascially you can set the initial CAL to 100 and check fequency accurate. Adjust up/down as needed
    // numbers[0] will contain the correction, numbers[1] will be the frequency in Hz
    case 'A':             // ADC
      sMeter = getSmeterValue (SMETER_BASELINE_SAMPLES);
      fwdPower = analogRead (FWD_PIN);
      revPower = analogRead (REV_PIN);
      Serial.print (sMeter);
      Serial.print (",");
      Serial.print (fwdPower);
      Serial.print (",");
      Serial.println (revPower);
      break;

    case 'B':             // ADC
      break;
      
      
    case 'C':             // Calibrate
      if (commands[1] == 'S') {
      }
      break;

    case 'F':             // Set Frequency
       break;

    case 'I':             // Memory setting
      resetFlash ();
      break;


    case 'M':             // Memory setting
      printGlobalMem();
      printCurrentMem();
      break;

    // Print Mem
    case 'P':
      printMem();
      break;

    case 'Q':             // 
      break;

    // This command reset the Si5351.  A reset zeros all parameters including the correction/calibration value
    // Therefore the calibration must be re-read from eeprom
    case 'R':             // Reset
      Reset();
      break;

    // If an undefined command is entered, display an error message
    default:
      ErrorOut ();
  }

}

void printGlobalMem (void) 
{
  Serial.print ("\r\nB: ");
  Serial.print (sg.BandOffset);
  Serial.print (" V: ");
  Serial.print (sg.VFO);
  Serial.print (" SmB: ");
  Serial.print (sg.SmeterBaseline);
  Serial.print (" Sm: ");
  Serial.print (sg.SmeterCorrection);
  Serial.print (" Fw: ");
  Serial.print (sg.FwdCorrection);
  Serial.print (" Rv: ");
  Serial.print (sg.RevCorrection);
  Serial.print (" C: ");
  Serial.println (sg.correction);
}

void printCurrentMem (void) 
{
    Serial.print (" => FL: ");
    Serial.print (mem.FreqLow);
    Serial.print (" FH: ");
    Serial.print (mem.FreqHigh);
    Serial.print (" FD: ");
    Serial.print (mem.Freq);
    Serial.print (" Fa: ");
    Serial.print (mem.FreqA);
    Serial.print (" Fb: ");
    Serial.print (mem.FreqB);
    Serial.print (" BF: ");
    Serial.print (mem.BFOFreq);
    Serial.print (" FI: ");
    Serial.print (mem.FreqInc);
    Serial.print (" LOOF: ");
    Serial.print (mem.LOOffset);
    Serial.print (" UOF: ");
    Serial.print (mem.USBBFOOffset);
    Serial.print (" LOF: ");
    Serial.print (mem.LSBBFOOffset);
    Serial.print (" COF: ");
    Serial.print (mem.CWBFOOffset);
    Serial.print (" M: ");
    Serial.print (mem.Mode);   
    Serial.print (" MSN: ");
    Serial.print (mem.MSN_a);   
    Serial.print (" PLL: ");
    Serial.print (mem.PLL_a);   
    Serial.print (" V: ");
    Serial.println (mem.VFO);     
}

void printMem (void) 
{
  unsigned char i;
  
  EEPROM.get (0, sg);
  printGlobalMem (); 

  for (i=0; i<MAX_BANDS; i++) {
    getBandData(i);
    Serial.print (i);
    printCurrentMem();   
  }
}

#endif // REMOVE_CLI
