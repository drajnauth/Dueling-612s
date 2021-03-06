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



volatile unsigned long bflags;
volatile unsigned long mflags;


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



char header1[HEADER1] = {'D', 'U', 'E', 'L', 'I', 'N', 'G', ' ', '6', '1', '2', 's', ' ', '0', '.', '1', 'C', 'T', '2', 0x0};
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
Band_Struct memB;

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
unsigned char rotaryChangeFlag;
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
  pinMode(PTT_OUT_PIN, OUTPUT);
  digitalWrite (PTT_OUT_PIN, 0);  // Ensure PTT is off

  SetupLCD ();
  
  LoadMemory ();

  if (sg.correction > 1000 || sg.correction < -1000) {
    sg.correction = 52;
  }

  setupSi5351(sg.correction);

  SetupEncoder();

  Reset ();

}





// the loop function runs over and over again forever
void loop()
{
  while (!checkFlag(BFLAGS, ANY_BUTTON)){   
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

  
  if (checkFlag (BFLAGS, MASTER_RESET) ) {
    Reset();
  }

  if (checkClearFlag (BFLAGS, PTT_PUSHED)) {
    PTT(1);
    LCDDisplayRxTx();
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  if (checkClearFlag (BFLAGS, PTT_CLEAR)) {
    PTT(0);
    LCDDisplayRxTx();
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  if (checkClearFlag (BFLAGS, PBUTTON2_PUSHED)) {
    if (checkFlag (MFLAGS, OPTION_CHANGED)) {
      sg.BandOffset = oldBandOffset;
      sg.RxTx = oldRxTx;
      mem.Mode = oldMode;
      LCDDisplayBand();
      LCDDisplayRxTx();
      LCDDisplayMode();
      clearFlag (MFLAGS, OPTION_CHANGED);
    }
    if (checkFlag (MFLAGS, ANY_MENU_OPTIONS)) {
      ResetMenuMode();
    } else {
      ChangeRadioSelection();
      clearFlag (MFLAGS, UPDATE_MEMORY);      
    }
    digitalWrite(LED_BUILTIN, LOW);
  }
 
  if (checkFlag (MFLAGS, CHANGE_FREQ)) {
    RadioUpdateFreq();
    
  } else if (checkFlag (MFLAGS, CHANGE_VFOB)) {
    RadioUpdateFreqB();
    
  } else if (checkFlag (MFLAGS, CHANGE_BAND)) {
    RadioUpdateOther();
    
  } else if (checkFlag (MFLAGS, CHANGE_RXTX)) {
    RadioUpdateOther();
    
  } else if (checkFlag (MFLAGS, CHANGE_MODE)) {
    RadioUpdateOther();

  } else if (checkFlag (MFLAGS, CHANGE_MENU)) {
    MenuDisplayMode();

  // Shift the sUnit reading to get to match an input reference level
  // e.g. feed S9 into the radio and adjust the Correction to shift to get S9 displayed 
  } else if (checkFlag (MFLAGS, CHANGE_SmC)) {
     updateNeeded = GetRotaryNumber (-10, 10, 1);
     if (updateNeeded == 1) {
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
     } else if (updateNeeded == 0xFF) {
        mem.SmeterCorrection = rotaryNumber;
        ClearFlags();
        LCDClearLine(3);
//        putConfigData ();
        putBandData (sg.BandOffset);
        setFlag (MFLAGS, CHANGE_MENU);
        LCDDisplayMenuOption (MenuSelection);
     }

  // Get the baseline to measure against.  The baseline will be the no signal present...just radio noise
  // e.g. disconnect antenna and use this feature to capture the reading.  Can manuall increase or decrease
  } else if (checkFlag (MFLAGS, CHANGE_SmB)) {
     updateNeeded = GetRotaryNumber (1, 300, 10);
     if (updateNeeded == 1) {
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
     } else if (updateNeeded == 0xFF) {
        mem.SmeterBaseline = rotaryNumber;
        ClearFlags();
        LCDClearLine(3);
//        putConfigData ();
        putBandData (sg.BandOffset);
        setFlag (MFLAGS, CHANGE_MENU);
        LCDDisplayMenuOption (MenuSelection);      
     }

  // Set the reading which give a reference voltate level (current reference is 5 Watts)
  // e.g. set to transmitt 5 Watt and use this to capture the FWR power reading.
  } else if (checkFlag (MFLAGS, CHANGE_Fw) || checkFlag (MFLAGS, CHANGE_Rw)) {
     updateNeeded = GetRotaryNumber (1, 900, 10);
     if (updateNeeded == 1) {
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
     } else if (updateNeeded == 0xFF) {
        if (checkFlag (MFLAGS, CHANGE_Fw)) {
          sg.FwdCorrection = rotaryNumber;
        } else {
          sg.RevCorrection = rotaryNumber;
        }
        ClearFlags();
        LCDClearLine(3);
        putConfigData ();
        setFlag (MFLAGS, CHANGE_MENU);
        LCDDisplayMenuOption (MenuSelection);      
     }

  // Set Si5351 Correction
  } else if (checkFlag (MFLAGS, CHANGE_Si)) {
     updateNeeded = GetRotaryNumber (-500, 500, 10);
     if (updateNeeded == 1) {
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
     } else if (updateNeeded == 0xFF) {
        sg.correction = rotaryNumber;
        ClearFlags();
        LCDClearLine(3);
        putConfigData ();
        setFlag (MFLAGS, CHANGE_MENU);
        LCDDisplayMenuOption (MenuSelection);      
     }
   
  } else if (checkFlag (MFLAGS, CHANGE_BFO)) {
     updateNeeded = GetRotaryNumber (-4000, 4000, 100);
     if (updateNeeded == 1) {
        LCDDisplayMenuLargeNumbner (rotaryNumber, rotaryInc);
        mem.BFOFreq = rotaryNumber;
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
        setFlag (MFLAGS, CHANGE_MENU);
        LCDDisplayMenuOption (MenuSelection);      
     }
    
  } else if (checkFlag (MFLAGS, CHANGE_LO)) {
     updateNeeded = GetRotaryNumber (-4000, 4000, 100);
     if (updateNeeded == 1) {
        if ((rotaryNumber+mem.Freq) > mem.FreqHigh) {
          rotaryNumber -= rotaryInc;
        } 
        if ((mem.Freq-rotaryNumber) < mem.FreqLow) {
          rotaryNumber += rotaryInc;
        } 
        mem.LOOffset = rotaryNumber;
        LCDDisplayMenuLargeNumbner (rotaryNumber, rotaryInc);
        UpdateLOFrequency();
        mem.LOOffset = origOffset;
     } else if (updateNeeded == 0xFF) {
        mem.LOOffset = rotaryNumber;
        UpdateLOFrequency();
        ClearFlags();
        LCDClearLine(3);
        putBandData (sg.BandOffset);
        setFlag (MFLAGS, CHANGE_MENU);
        LCDDisplayMenuOption (MenuSelection);      
     }
    
  }
  

  if (checkFlag (MFLAGS, UPDATE_MEMORY)) {
    memoryTime = millis();
  } else {
    if ((millis()-memoryTime) > UPDATE_MEMORY_THRESHOLD ) {
      clearFlag (MFLAGS, UPDATE_MEMORY);
      putBandData (sg.BandOffset); 
      putConfigData ();
      memoryTime = millis();  
      Serial.println ("WR"); 
    }
  }

}

void ChangeRadioSelection (void)
{
  RadioSelection++;
  if (RadioSelection >= MAX_RADIO_SELECTION) RadioSelection = 0;
  ClearFlags ();
  switch (RadioSelection) {
    case RADIO_FREQ_MODE:
      setFlag (MFLAGS, CHANGE_FREQ);
      LCDDisplayFreq ();
      break;

    case RADIO_MODE:
      setFlag (MFLAGS, CHANGE_MODE);
      LCDDisplayMode();
      break;

//////////////////////////////////
    case RADIO_VFOB_MODE:
      setFlag (MFLAGS, CHANGE_VFOB);
      LCDDisplayVFOBFreq ();
      break;

    case RADIO_MENU_MODE:
      setFlag (MFLAGS, CHANGE_MENU);
      LCDDisplayMenuOption (MenuSelection);
      break;

    case RADIO_BAND_MODE:
      setFlag (MFLAGS, CHANGE_BAND);
      LCDDisplayBand ();
      break;

  }
}

void MenuDisplayMode (void)
{
  rotaryChangeFlag = 0;

  if (checkClearFlag (BFLAGS, ROTARY_CW)) {
    rotaryChangeFlag = 1;
    MenuSelection++;
    if (MenuSelection >= MAXMENU_ITEMS) {
      MenuSelection = 0;
    }

  } else if (checkClearFlag (BFLAGS, ROTARY_CCW)) {
    rotaryChangeFlag = 1;
    if (MenuSelection) MenuSelection--;
    else MenuSelection = MAXMENU_ITEMS-1;
  }

  if ( rotaryChangeFlag ) {
    LCDDisplayMenuItem(MenuSelection);
    digitalWrite(LED_BUILTIN, LOW);
  }

  checkClearFlag (BFLAGS, ROTARY_PUSH);

  if (checkClearFlag (BFLAGS, PBUTTON1_PUSHED)) {    // execute
    digitalWrite(LED_BUILTIN, LOW);
    doMenuSelection();  
  }
  
  if (checkClearFlag (BFLAGS, PBUTTON2_PUSHED)) {
    ResetMenuMode();
  }

}

void doMenuSelection (void) 
{
    switch (MenuSelection) {
      case RADIO:
        putBandData (sg.BandOffset); 
        putConfigData ();
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
        setFlag (MFLAGS, CHANGE_BFO);
        break;
        
      case LO_CORRECT:
        ClearFlags();
        rotaryNumber = mem.LOOffset;
        origOffset = rotaryNumber;
        rotaryInc = 100;        
        LCDDisplayMenuLargeNumbner (rotaryNumber, rotaryInc);
        setFlag (MFLAGS, CHANGE_LO);
        break;
        
      case Si_CORRECT:
        ClearFlags();
        rotaryNumber = sg.correction;
        rotaryInc = 10;        
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
        setFlag (MFLAGS, CHANGE_Si);
        break;
        
      case Sm_BASELIN:
        ClearFlags();
        sMeter = getSmeterValue (SMETER_BASELINE_SAMPLES);
        if (sMeter == 0) sMeter = 1;
        rotaryNumber = sMeter;
        rotaryInc = 1;        
        setFlag (MFLAGS, CHANGE_SmB);
        LCDDisplayMenuSmallNumbner (rotaryNumber, rotaryInc);
        break;
        
      case Sm_CORRECT:
        ClearFlags();
        rotaryNumber = mem.SmeterCorrection;
        rotaryInc = 1;        
        setFlag (MFLAGS, CHANGE_SmC);
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
        setFlag (MFLAGS, CHANGE_Fw);
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
        setFlag (MFLAGS, CHANGE_Rw);
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
    setFlag (MFLAGS, CHANGE_MENU);
    LCDDisplayMenuOption (MenuSelection);      
    UpdateLOFrequency();
    UpdateBFOFrequency();     
  
}

unsigned char GetRotaryNumber (int lnum, int hnum, int maxinc)
{
  unsigned char change;
  change = 0;
  rotaryChangeFlag = 0;

  if (checkClearFlag (BFLAGS, ROTARY_CW)) {
    rotaryChangeFlag = 1;
    rotaryNumber += rotaryInc;
    if (rotaryNumber > hnum) rotaryNumber = hnum; 

  } else if (checkClearFlag (BFLAGS, ROTARY_CCW)) {
    rotaryChangeFlag = 1;
    rotaryNumber -= rotaryInc;
    if (rotaryNumber < lnum) rotaryNumber = lnum;     
  }
  

  if ( rotaryChangeFlag ) {
    change = 1;
    digitalWrite(LED_BUILTIN, LOW);

  }

  if (checkClearFlag (BFLAGS, ROTARY_PUSH)) {
    change = 1;
    rotaryInc *= 10;
    if (rotaryInc > maxinc) rotaryInc = 1;
  }

  if (checkClearFlag (BFLAGS, PBUTTON1_PUSHED)) {
    change = 0xFF;
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  return change;

}


void RadioUpdateFreq (void)
{  
  freq = mem.Freq;
  rotaryChangeFlag = 0;

  if (checkClearFlag (BFLAGS, ROTARY_CW)) {
    freq += mem.FreqInc;
    rotaryChangeFlag = 1;
    if (freq > mem.FreqHigh) {
      sg.BandOffset++;
      if (sg.BandOffset >=MAX_BANDS) sg.BandOffset = 0;
      getBandData (sg.BandOffset, 'A');
      RefreshLCD();
      SetupAllMSNDividers();
      UpdateBFOFrequency (); 
      UpdateLOFrequency ();
      freq = mem.FreqLow;            
    }

  } else if (checkClearFlag (BFLAGS, ROTARY_CCW)) {
    freq -= mem.FreqInc;
    rotaryChangeFlag = 1;
    if (freq < mem.FreqLow) {
      if (sg.BandOffset) sg.BandOffset--;
      else sg.BandOffset = MAX_BANDS - 1;
      getBandData (sg.BandOffset, 'A');
      RefreshLCD();
      SetupAllMSNDividers();
      UpdateBFOFrequency (); 
      UpdateLOFrequency ();
      freq = mem.FreqHigh;      
    } 
  }

  if ( rotaryChangeFlag ) {
    mem.Freq = freq;
    sg.FreqA = freq;
    LCDDisplayFreq ();
    UpdateLOFrequency ();
    setFlag (MFLAGS, UPDATE_MEMORY);
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (checkClearFlag (BFLAGS, ROTARY_PUSH)) {
    mem.FreqInc *= 10;
    if (mem.FreqInc > MAXIMUM_FREQUENCY_MULTIPLIER) mem.FreqInc = MINIMUM_FREQUENCY_MULTIPLIER;
    LCDDisplayFreq ();
 }

  if (checkClearFlag (BFLAGS, PBUTTON1_PUSHED)) {    // execute
    updateNeeded = sg.BandOffset;
    sg.BandOffset = sg.BandOffsetB;
    sg.BandOffsetB = updateNeeded;
    getBandData (sg.BandOffset, 'A');
    getBandData (sg.BandOffset, 'B');
    if (sg.VFO == 'A') {
      sg.VFO = 'B';
    } else {
      sg.VFO = 'A';
    }
    memB.Freq = sg.FreqA;
    mem.Freq = sg.FreqB;
    sg.FreqA = mem.Freq;
    sg.FreqB = memB.Freq;
    
    RefreshLCD();
    SetupAllMSNDividers();
    UpdateBFOFrequency (); 
    UpdateLOFrequency ();
    RadioSelection = RADIO_VFOB_MODE;
    ClearFlags();
    setFlag (MFLAGS, CHANGE_VFOB);
    setFlag (MFLAGS, UPDATE_MEMORY);
    LCDDisplayVFOBFreq ();    
    digitalWrite(LED_BUILTIN, LOW);
  }
}


void RadioUpdateFreqB (void)
{  
  freq = sg.FreqB;
  rotaryChangeFlag = 0;

  if (checkClearFlag (BFLAGS, ROTARY_CW)) {
    freq += memB.FreqInc;
    rotaryChangeFlag = 1;
    if (freq > memB.FreqHigh) {
      sg.BandOffsetB++;
      if (sg.BandOffsetB >=MAX_BANDS) sg.BandOffsetB = 0;
      getBandData (sg.BandOffsetB, 'B');
      freq = memB.FreqLow;    
      sg.FreqB = memB.FreqLow;        
      RefreshLCD();
    }

  } else if (checkClearFlag (BFLAGS, ROTARY_CCW)) {
    freq -= memB.FreqInc;
    rotaryChangeFlag = 1;
    if (freq < memB.FreqLow) {
      if (sg.BandOffsetB) sg.BandOffsetB--;
      else sg.BandOffsetB = MAX_BANDS - 1;
      getBandData (sg.BandOffsetB, 'B');
      freq = memB.FreqHigh;    
      sg.FreqB = memB.FreqHigh;        
      RefreshLCD();    
    } 
  }

  if ( rotaryChangeFlag ) {
    memB.Freq = freq;
    sg.FreqB = freq;
    LCDDisplayVFOBFreq ();    
    setFlag (MFLAGS, UPDATE_MEMORY);
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (checkClearFlag (BFLAGS, ROTARY_PUSH)) {
    memB.FreqInc *= 10;
    if (memB.FreqInc > MAXIMUM_FREQUENCY_MULTIPLIER) memB.FreqInc = MINIMUM_FREQUENCY_MULTIPLIER;
    LCDDisplayVFOBFreq ();    
 }

  if (checkClearFlag (BFLAGS, PBUTTON1_PUSHED)) {    // execute
    updateNeeded = sg.BandOffset;
    sg.BandOffset = sg.BandOffsetB;
    sg.BandOffsetB = updateNeeded;
    getBandData (sg.BandOffset, 'A');
    getBandData (sg.BandOffset, 'B');
    if (sg.VFO == 'A') {
      sg.VFO = 'B';
    } else {
      sg.VFO = 'A';
    }
    memB.Freq = sg.FreqA;
    mem.Freq = sg.FreqB;
    sg.FreqA = mem.Freq;
    sg.FreqB = memB.Freq;

    RefreshLCD();
    SetupAllMSNDividers();
    UpdateBFOFrequency (); 
    UpdateLOFrequency ();
    RadioSelection = RADIO_FREQ_MODE;
    ClearFlags();
    setFlag (MFLAGS, CHANGE_FREQ);
    setFlag (MFLAGS, UPDATE_MEMORY);
    LCDDisplayFreq ();
    digitalWrite(LED_BUILTIN, LOW);

  }
}



void RadioUpdateOther (void)
{
  rotaryChangeFlag = 0;

  if ( !checkFlag (MFLAGS, OPTION_CHANGED) ) {
    oldBandOffset = sg.BandOffset;
    oldRxTx = sg.RxTx;
    oldMode = mem.Mode;
  }

  if (checkClearFlag (BFLAGS, ROTARY_CW)) {
    rotaryChangeFlag = 1;
    if (checkFlag (MFLAGS, CHANGE_BAND)) {
      sg.BandOffset++;
      if (sg.BandOffset >=MAX_BANDS) sg.BandOffset = 0;
            
    } else if (checkFlag (MFLAGS, CHANGE_MODE)) {
      mem.Mode++;
      if (mem.Mode >= MAX_MODES) mem.Mode = 0;
    }

  } else if (checkClearFlag (BFLAGS, ROTARY_CCW)) {
    rotaryChangeFlag = 1;
    if (checkFlag (MFLAGS, CHANGE_BAND)) {
      if (sg.BandOffset) sg.BandOffset--;
      else sg.BandOffset = MAX_BANDS - 1;
     
    } else if (checkFlag (MFLAGS, CHANGE_MODE)) {
      if (mem.Mode) mem.Mode--;
      else mem.Mode = MAX_MODES - 1;
    }
  }

  if ( rotaryChangeFlag ) {
    setFlag (MFLAGS, OPTION_CHANGED);
    if (checkFlag (MFLAGS, CHANGE_BAND)) {
      LCDDisplayBand();
      
    } else if (checkFlag (MFLAGS, CHANGE_RXTX)) {
      LCDDisplayRxTx();
      
    } else if (checkFlag (MFLAGS, CHANGE_MODE)) {
      LCDDisplayMode();
    }
    digitalWrite(LED_BUILTIN, LOW);
  }

  checkClearFlag (BFLAGS, ROTARY_PUSH);

  if (checkClearFlag (BFLAGS, PBUTTON1_PUSHED)) {    // execute
    digitalWrite(LED_BUILTIN, LOW);

    if (checkFlag (MFLAGS, CHANGE_RXTX)) {
      if (sg.RxTx == TRANSMIT) {
        PTT(0);
      } else {
        PTT(1);
      }
      LCDDisplayRxTx();
      return;
    
    } else if (checkFlag (MFLAGS, CHANGE_BAND)) {
      getBandData (sg.BandOffset, 'A');
      setFlag (MFLAGS, UPDATE_MEMORY);
        
    } else if (checkFlag (MFLAGS, CHANGE_MODE)) {
      SetBFOFrequency ();
      setFlag (MFLAGS, UPDATE_MEMORY);
    }
      
    ClearFlags();
    setFlag (MFLAGS, CHANGE_FREQ);
    RefreshLCD();
    SetupAllMSNDividers();
    UpdateBFOFrequency (); 
    UpdateLOFrequency ();
    RadioSelection = 0;
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
    SetupAllMSNDividers();
    UpdateBFOFrequency();
    UpdateLOFrequency();
    drainSmeterCap(10);
    digitalWrite (PTT_OUT_PIN, HIGH);  // PTT is on
  } else {
    digitalWrite (PTT_OUT_PIN, LOW);  // PTT is off
    LCDClearSWR();
    sg.RxTx = RECEIVE;
    SetupAllMSNDividers();
    UpdateBFOFrequency();
    UpdateLOFrequency();
    drainSmeterCap(1);
  } 
}


void ClearFlags ()
{
  clearFlag (MFLAGS, CHANGE_FREQ);
  clearFlag (MFLAGS, CHANGE_BAND);
  clearFlag (MFLAGS, CHANGE_RXTX);
  clearFlag (MFLAGS, CHANGE_MODE);
  clearFlag (MFLAGS, CHANGE_VFOB);
  clearFlag (MFLAGS, CHANGE_MENU);
  clearFlag (MFLAGS, CHANGE_SmB);
  clearFlag (MFLAGS, CHANGE_SmC);
  clearFlag (MFLAGS, CHANGE_Fw);
  clearFlag (MFLAGS, CHANGE_Rw);
  clearFlag (MFLAGS, CHANGE_Si);
  clearFlag (MFLAGS, CHANGE_BFO);
  clearFlag (MFLAGS, CHANGE_LO);
  clearFlag (MFLAGS, OPTION_CHANGED);
  clearFlag (MFLAGS, UPDATE_MEMORY);
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

void SetupAllMSNDividers (void) 
{
  SetupLOMSNDividers();

  SetupBFOMSNDividers();  

  drainSmeterCap(50);  
}

void SetupLOMSNDividers (void) 
{
  freq = mem.Freq + mem.BFOFreq + mem.LOOffset;
  pll_freq = freq * mem.MSN_a;

  if (sg.RxTx == RECEIVE) {    
    ProgramSi5351MSN (RX_LO_CLK, SI_PLL_A, pll_freq, freq); 
  } else {
    ProgramSi5351MSN (TX_LO_CLK, SI_PLL_A, pll_freq, freq); 
  }  

}
 

void SetupBFOMSNDividers (void) 
{
  pll_freq = mem.BFOFreq * mem.PLL_a;
  
  if (sg.RxTx == RECEIVE) {    
    ProgramSi5351MSN (TX_LO_CLK, SI_PLL_B, pll_freq, mem.BFOFreq); 
  } else {
    ProgramSi5351MSN (RX_LO_CLK, SI_PLL_B, pll_freq, mem.BFOFreq); 
  }  
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
  getConfigData ();
  
  if (sg.flags != D612_HEADER) {
    Serial.println ("D612 Reset");
    resetFlash();
  }

  if (sg.BandOffset >= MAX_BANDS) {
     Serial.println ("Offset Reset");
     resetFlash();
  }

  getBandData (sg.BandOffset, 'A');
  if (mem.flags != BAND_HEADER) {
     Serial.println ("BHEAD Reset");
     resetFlash();
     getBandData (sg.BandOffset, 'A');
  } 
  
  getBandData (sg.BandOffset, 'B');
  if (memB.flags != BAND_HEADER) {
     Serial.println ("BHEAD Reset");
     resetFlash();
     getBandData (sg.BandOffset, 'B');
  } 
  
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

  // Fwd power is voltage and power is based on voltage squared
  swr = (double)fwdPower * (double)fwdPower;

  // FwdCorrection is the Voltage reading (i.e. ADC value) that gives the POWER_REFERENCE
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

  if (sMeter < mem.SmeterBaseline) sMeter = mem.SmeterBaseline;
  
  // Calculate dB relative to baseline. Baseline is no signal - radio noise only
  value = (int) (20 * log( (double)sMeter/(double)mem.SmeterBaseline));

  // Each sUnit is 6dB
  value /= 6;

  // Shift reading to match a reference level. i.e. feed S9 signal and adjust correction to get S9 reading
  value += mem.SmeterCorrection;

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

  mflags = CHANGE_FREQ;
  bflags = 0;
  
  sg.RxTx = RECEIVE;
  RefreshLCD();
  SetBFOFrequency ();
  SetupAllMSNDividers();
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
    sg.BandOffsetB = 0;
    sg.VFO = 'A';
    sg.RxTx = RECEIVE;
    sg.FreqA = 7101000;
    sg.FreqB = 3501000;
    sg.FwdCorrection = 478;
    sg.RevCorrection = 25;
    sg.correction = 40;   // can be + or -
    putConfigData ();
    
    // 80m
    mem.flags = BAND_HEADER;
    mem.FreqLow = 3500000;
    mem.FreqHigh = 4000000;
    mem.BFOFreq = DEFAULT_LSBBFO;
    mem.USBBFOFreq = DEFAULT_USBBFO;
    mem.LSBBFOFreq = DEFAULT_LSBBFO;
    mem.CWBFOFreq = DEFAULT_CWBFO;
    mem.FreqInc = DEFAULT_FREQUENCY_MULTIPLIER;
    mem.LOOffset = -18;
    mem.USBBFOOffset = 0;
    mem.LSBBFOOffset = 0;
    mem.CWBFOOffset = 0;
    mem.Mode = LSB_MODE;
    mem.SmeterBaseline = 2;
    mem.SmeterCorrection = (-3);
    mem.MSN_a = 47;
    mem.PLL_a = DEFAULT_BFO_PLL_MULT;
    putBandData (BAND80M); 

    // 60m
    mem.flags = BAND_HEADER;
    mem.FreqLow = 5330500;
    mem.FreqHigh = 5403500;
    mem.Freq = 5331500;
    mem.BFOFreq = DEFAULT_LSBBFO;
    mem.USBBFOFreq = DEFAULT_USBBFO;
    mem.LSBBFOFreq = DEFAULT_LSBBFO;
    mem.CWBFOFreq = DEFAULT_CWBFO;
    mem.FreqInc = DEFAULT_FREQUENCY_MULTIPLIER;
    mem.LOOffset = -11;
    mem.USBBFOOffset = 0;
    mem.LSBBFOOffset = 0;
    mem.CWBFOOffset = 0;
    mem.Mode = USB_MODE;
    mem.SmeterBaseline = 2;
    mem.SmeterCorrection = (-3);
    mem.MSN_a = 43;
    mem.PLL_a = DEFAULT_BFO_PLL_MULT;
    putBandData (BAND60M); 

    // 40m
    mem.flags = BAND_HEADER;
    mem.FreqLow = 7000000;
    mem.FreqHigh = 7300000;
    mem.Freq = 7101000;
    mem.FreqInc = DEFAULT_FREQUENCY_MULTIPLIER;
    mem.BFOFreq = DEFAULT_LSBBFO;
    mem.USBBFOFreq = DEFAULT_USBBFO;
    mem.LSBBFOFreq = DEFAULT_LSBBFO;
    mem.CWBFOFreq = DEFAULT_CWBFO;
    mem.LOOffset = -30;
    mem.USBBFOOffset = 0;
    mem.LSBBFOOffset = 0;
    mem.CWBFOOffset = 0;
    mem.Mode = LSB_MODE;
    mem.SmeterBaseline = 2;
    mem.SmeterCorrection = (-3);
    mem.MSN_a = 39;
    mem.PLL_a = DEFAULT_BFO_PLL_MULT;
    putBandData (BAND40M); 

    // 30m
    mem.flags = BAND_HEADER;
    mem.FreqLow = 10100000;
    mem.FreqHigh = 10150000;
    mem.Freq = 10100000;
    mem.FreqInc = DEFAULT_FREQUENCY_MULTIPLIER;
    mem.BFOFreq = DEFAULT_USBBFO;
    mem.USBBFOFreq = DEFAULT_USBBFO;
    mem.LSBBFOFreq = DEFAULT_LSBBFO;
    mem.CWBFOFreq = DEFAULT_CWBFO;
    mem.LOOffset = -36;
    mem.USBBFOOffset = 0;
    mem.LSBBFOOffset = 0;
    mem.CWBFOOffset = 0;
    mem.Mode = USB_MODE;
    mem.SmeterBaseline = 2;
    mem.SmeterCorrection = (-3);
    mem.MSN_a = 34;
    mem.PLL_a = DEFAULT_BFO_PLL_MULT;
    putBandData (BAND30M); 

    // 20m
    mem.flags = BAND_HEADER;
    mem.FreqLow = 14000000;
    mem.FreqHigh = 14350000;
    mem.Freq = 14099000;
    mem.FreqInc = DEFAULT_FREQUENCY_MULTIPLIER;
    mem.BFOFreq = DEFAULT_USBBFO;
    mem.USBBFOFreq = DEFAULT_USBBFO;
    mem.LSBBFOFreq = DEFAULT_LSBBFO;
    mem.CWBFOFreq = DEFAULT_CWBFO;
    mem.LOOffset = -47;
    mem.USBBFOOffset = 0;
    mem.LSBBFOOffset = 0;
    mem.CWBFOOffset = 0;
    mem.Mode = USB_MODE;
    mem.SmeterBaseline = 2;
    mem.SmeterCorrection = (-3);
    mem.MSN_a = 28;
    mem.PLL_a = DEFAULT_BFO_PLL_MULT;
    putBandData (BAND20M); 

}

unsigned char checkFlag (unsigned char type, unsigned long bitmask)
{
 unsigned char ret;

 if (type) {
  ret = 0;
  cli();
  if (bflags & bitmask) ret = 1;
  sei();
  return ret;
  
 } else {
  if ( mflags & bitmask) return 1;
  else return 0;
  
 }
}

unsigned char checkClearFlag (unsigned char type, unsigned long bitmask)
{
 unsigned char ret;

 if (type) {
  ret = 0;
  cli();
  if (bflags & bitmask) {
    ret = 1;
    bflags &= ~bitmask;
  }
  sei();
  return ret;
  
 } else {
  if ( mflags & bitmask) {
    mflags &= ~bitmask;
    return 1;
  }
  else return 0;
  
 }
}

void setFlag (unsigned char type, unsigned long bitmask)
{
 if (type) {
  cli();
  bflags |= bitmask;
  sei();
  
 } else {
  mflags |= bitmask;
  
 }
}

void clearFlag (unsigned char type, unsigned long bitmask)
{
 if (type) {
  cli();
  bflags &= ~bitmask;
  sei();
  
 } else {
  mflags &= ~bitmask;
 }
}




//Debug these...there are not working correctly

void getConfigData (void)
{
  EEPROM.get ( EEPROM_OFFSET, sg); 
}

void putConfigData (void)
{
  EEPROM.put ( EEPROM_OFFSET, sg); 
  
}


void getBandData (unsigned char band, unsigned char vfo)
{
  if (vfo == 'A') EEPROM.get ( (EEPROM_OFFSET+sizeof(sg)+sizeof(mem)*band), mem); 
  else EEPROM.get ( (EEPROM_OFFSET+sizeof(sg)+sizeof(mem)*band), memB); 
}


void putBandData (unsigned char band)
{
  EEPROM.put ( (EEPROM_OFFSET+sizeof(sg)+sizeof(mem)*band), mem); 
  
}
