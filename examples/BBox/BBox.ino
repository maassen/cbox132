/****************************************** -*- mode: c++ -*-
 * BBbox.ino
 * arduino implementation of Carrera BlackBox 2008
 * - announce race start over rails
 * - program speed/break w/ controller 1/2
 * - support wireless controller via Tower1 (not yet)
 * - measure times and rounds at rails w/ ir id
 *   on PD3(INT1) and PD6(ICP1) as ControlUnit and
 *   disableing Tower2 and Module Con
 * - communicate via Serial like ControlUnit and more
 *
 *  Part of CBox132 - https://github.com/maassen/cbox132
 *
 * Copyright (c) 2019 Michael Maassen
 ***********************************************************/
/* * *     FEATURES: (memory is limited!)              * * */
#define Carrera_CU		// CU protocol on PC-Unit
#define Carrera_Lap		// Signal on LacCount port
#define Car_IR			// IR sensor to detect cars
#define Remote_IR		// Remote Controllers on Tower1
#define Module_LCD		// I2C LCD on Module Port
#define Module_Lights		// start lights on Module Port
#define BBox_Cmd		// Listen for Commands on PC-Unit

#include <D132.h>

#if defined(Module_LCD) || defined(Module_Lights)
#include <Wire.h>
#endif
#ifdef Module_LCD
#include <LiquidCrystal_I2C.h>
#endif

D132 Rail;
#ifdef Module_LCD
LiquidCrystal_I2C lcd(0x20,0x40,0x10,0x20,0x80);  // LCD address:0x20 EN:0x40 RS:0x10 RW:0x20 BL:0x80
uint8_t CarDis = 0;				  // cars on display
#endif

enum BBoxModes { //0x0_
  PowerUpMode,			// after power up
  TrainMode,			// trainig: cars drive
  PreStartMode,			// first press of Start button
  Start1Mode,			// LED 1 on
  Start2Mode,			// LED 1-2 on
  Start3Mode,			// LED 1-3 on
  Start4Mode,			// LED 1-4 on
  Start5Mode,			// LED 1-5 on
  RaceMode,			// race: cars drive
  FailStart0Mode,		// fail start of a car 0
  FailStart1Mode,		// fail start of a car 1
  FailStart2Mode,		// fail start of a car 2
  FailStart3Mode,		// fail start of a car 3
  FailStart4Mode,		// fail start of a car 4
  FailStart5Mode,		// fail start of a car 5
  ProgSpeed0Mode,		// power on w/ controller 1 pressed
  // 0x1_
  ProgSpeed1Mode,		// power on w/ controller 1 pressed
  ProgSpeed2Mode,		// power on w/ controller 1 pressed
  ProgSpeed3Mode,		// power on w/ controller 1 pressed
  ProgSpeed4Mode,		// power on w/ controller 1 pressed
  ProgSpeed5Mode,		// power on w/ controller 1 pressed
  ProgSpeed6Mode,		// power on w/ controller 1 pressed
  ProgSpeed7Mode,		// power on w/ controller 1 pressed
  ProgSpeed8Mode,		// power on w/ controller 1 pressed
  ProgSpeed9Mode,		// power on w/ controller 1 pressed
  ProgBreak0Mode,		// power on w/ controller 2 pressed
  ProgBreak1Mode,		// power on w/ controller 2 pressed
  ProgBreak2Mode,		// power on w/ controller 2 pressed
  ProgBreak3Mode,		// power on w/ controller 2 pressed
  ProgBreak4Mode,		// power on w/ controller 2 pressed
  ProgBreak5Mode,		// power on w/ controller 2 pressed
  ProgBreak6Mode,		// power on w/ controller 2 pressed
  // 0x2_
  ProgBreak7Mode,		// power on w/ controller 2 pressed
  ProgBreak8Mode,		// power on w/ controller 2 pressed
  ProgBreak9Mode,		// power on w/ controller 2 pressed
  ProgFuel0Mode,		// power on w/ controller 3 pressed
  ProgFuel1Mode,		// power on w/ controller 3 pressed
  ProgFuel2Mode,		// power on w/ controller 3 pressed
  ProgFuel3Mode,		// power on w/ controller 3 pressed
  ProgFuel4Mode,		// power on w/ controller 3 pressed
  ProgFuel5Mode,		// power on w/ controller 3 pressed
  ProgFuel6Mode,		// power on w/ controller 3 pressed
  ProgFuel7Mode,		// power on w/ controller 3 pressed
  ProgFuel8Mode,		// power on w/ controller 3 pressed
  ProgFuel9Mode,		// power on w/ controller 3 pressed
  ErrorMode,			// error: rails off (i.e. short cut)
} BBMode = PowerUpMode;		// mode of BBox:

unsigned char Speed[D132::numCars]; // CIAXSSSS: C=Computer,I=IR,A=Analog,X=Cross,SSSS=Speed
#ifdef Carrera_CU
unsigned long Leap[D132::numCars];  // millis
#endif

const int CmdMax = 20;
char CmdLine[CmdMax];
unsigned char CmdLen = 0;	// Num char in line
unsigned char LedDly=16;	// delay to next LED update [~130ms]
unsigned char RaceStart = 0;	// ind. that race has started (Lap Count connector) [~130ms]
signed char   PaceAct = 0;      // >0 PaceCar run; <0 Pace car home  [~130ms]
unsigned char SerMsk=0x21;      // Bitmask for serial info:01=RailDat,2=Mode,3=PCar,4=Chk,5=CarID,7=DBG
unsigned long usec;

const char LedPins[] = {pins_LED1,pins_LED2,pins_LED3,pins_LED4,pins_LED5};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setup_i2c();
  pinMode(pins_LED1,OUTPUT);
  digitalWrite(pins_LED1,HIGH);	// LED1 on
  pinMode(pins_LED2,OUTPUT);
  pinMode(pins_LED3,OUTPUT);
  pinMode(pins_LED4,OUTPUT);
  pinMode(pins_LED5,OUTPUT);
  pinMode(pins_Uref,INPUT);
  pinMode(pins_Uout,INPUT);
  pinMode(pins_Speed1,INPUT);
  pinMode(pins_Speed2,INPUT);
  pinMode(pins_Speed3,INPUT);
  pinMode(pins_Speed4,INPUT);
  pinMode(pins_LaneX1,INPUT);
  pinMode(pins_LaneX2,INPUT);
  pinMode(pins_LaneX3,INPUT);
  pinMode(pins_LaneX4,INPUT);
  pinMode(pins_StartKey,INPUT);
  pinMode(pins_PaceKey,INPUT);
  pinMode(pins_Fuel_SW,INPUT);
#ifdef Carrera_Lap
  pinMode(pins_LapCount,OUTPUT);
  digitalWrite(pins_LapCount,LOW);
#endif
 //if (SerMsk & 0x80)
 //   Rail.print_debug();
  if (digitalRead(pins_LaneX1) == LOW)
    SwitchMode(ProgSpeed0Mode);
  else if (digitalRead(pins_LaneX2) == LOW)
    SwitchMode(ProgBreak0Mode);
  else if (digitalRead(pins_LaneX3) == LOW)
    SwitchMode(ProgFuel0Mode);
  else
    SwitchMode(TrainMode);
#ifdef Car_IR
  // Lane1 IR sensor: PD3(INT1)
  //attachInterrupt(2, Lane1IR, CHANGE);
  pinMode(pins_CarID1,INPUT);
  MCUCR = (MCUCR & ~(1 << ISC10)) | (1 << ISC11); // falling edge
  GICR |= (1 << INT1);
  //Lane2 IR sensor: PD6(ICP1)
  pinMode(pins_CarID2,INPUT);
  TIMSK |= 1 << TICIE1;
  TCCR1B &= ~(uint8_t)(1 << ICES1);
#endif
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  word cars;
  //Serial.println(Rail.updated,HEX);
  if ( Rail.changed(Rail.upd_telegram)) { // a telegram has been send
    cycleMode();
  } // endif ( Rail.changed(Rail.upd_telegram))
  if ((SerMsk&0x20)&&(cars= Rail.changed(0xff00))) { // car has been identified
    for (char c=0; c< Rail.numCars; c++) {
      if (cars & (0x100<<c)) {
	Serial.print((char)('a'+c));
	Serial.println(Rail.Score[c]);
	Serial.print((char)('A'+c));
	Serial.println(Rail.Rounds[c]-1);
#ifdef Module_LCD
	Lap_i2c(c);
#endif
      }
    }
  }
  if (Serial.available())
    newChar(Serial.read());
  usec = Rail.micros();
  if ((LedDly&1) != ((usec & 0x10000)?1:0))  // 2^17us ~ 130ms
    if(--LedDly == 0) {
      if((BBMode>=Start1Mode) && (BBMode<=Start5Mode))
	SwitchMode(BBMode+1);
      else
	UpdateLed();
    }
  if (RaceStart && ((RaceStart&1) != ((usec & 0x10000)?1:0))) { // 2^17us ~ 130ms
    RaceStart -= 1;
 #ifdef Carrera_Lap
    if (! RaceStart) {
      digitalWrite(pins_LapCount,LOW);
    }
 #endif
  }
  if (PaceAct && ((PaceAct&1) != ((usec & 0x80000)?1:0))) { // 2^20us ~ 1s
    if (PaceAct > 0) {
      PaceAct -= 1;
      if (PaceAct == 0) 
	PaceAct = -10;
    } else {
      PaceAct += 1;
    }
    if (SerMsk & 0x8) {
      Serial.print('P');
      Serial.println(PaceAct,HEX);
    }
  }
  if (!PaceAct && (digitalRead(pins_PaceKey) == LOW))
    PaceAct = 10;
  
}

///// tools ////////////////////////////////////////////////
void cycleMode()
{ // do a step dep. on mode
  static char last = LOW;
  if ((BBMode<=RaceMode)&&(BBMode>=TrainMode)) {
    char fail = prepTele(Rail._PackageIndex);
    if (fail > 0)
      SwitchMode(fail);
  }
  if ((SerMsk & 3)==3) {
    Serial.print('=');
    Serial.println(Rail.Packages[Rail._PackageIndex],HEX);
  }
  if (Rail._PackageIndex == 9)
    CheckRails();
  if ((BBMode==TrainMode)||(BBMode==PreStartMode)||(BBMode==RaceMode)) {
    if (digitalRead(pins_StartKey) != last) {
      if (last == HIGH) {
	SwitchMode((BBMode==PreStartMode)?BBMode+1:PreStartMode);
	last = LOW;
      } else
	last = HIGH;
    }
  }
  if ((BBMode>=ProgSpeed0Mode)&&(BBMode<=ProgSpeed9Mode)) {
    if (digitalRead(pins_LaneX1) != last) {
      if (last == HIGH) {
	SwitchMode((BBMode<ProgSpeed9Mode)?BBMode+1:ProgSpeed0Mode);
	last = LOW;
      } else
	last = HIGH;
    }
    if (Rail.ADval[Rail.carIdx[0]-1] > 15)
      SwitchMode(TrainMode);
  }
  if ((BBMode>=ProgBreak0Mode)&&(BBMode<=ProgBreak9Mode)) {
    if (digitalRead(pins_LaneX2) != last) {
      if (last == HIGH) {
	SwitchMode((BBMode<ProgBreak9Mode)?BBMode+1:ProgBreak0Mode);
	last = LOW;
      } else
	last = HIGH;
    }
    if (Rail.ADval[Rail.carIdx[1]-1] > 15)
      SwitchMode(TrainMode);
  }
  if ((BBMode>=ProgFuel0Mode)&&(BBMode<=ProgFuel9Mode)) {
    if (digitalRead(pins_LaneX3) != last) {
      if (last == HIGH) {
	SwitchMode((BBMode<ProgFuel9Mode)?BBMode+1:ProgFuel0Mode);
	last = LOW;
      } else
	last = HIGH;
    }
    if (Rail.ADval[Rail.carIdx[2]-1] > 15)
      SwitchMode(TrainMode);
  }
  if ((BBMode>=FailStart0Mode)&&(BBMode<=FailStart5Mode)) {
    if (digitalRead(pins_StartKey) != last) { 
      if (last == HIGH) {
	SwitchMode(Start1Mode);
	last = LOW;
      } else
	last = HIGH;
    }
  }
}  
  
char prepTele(char pkgIdx)
{ // prepare new data for next telegram
  char exSpd=0;
  switch (pkgIdx) { // what is next telegram?
  case 0:			// Program
    exSpd = 0;
    for(int i=0;i<D132::numCars;i++) {
      if (Speed[i] & 0x80)
	exSpd += 1;
      Speed[i] = 0;
    }
    if ((((SerMsk & 3)==1)&&((Rail.Packages[pkgIdx]&0x1FF0)!=0x1820))
	|| ((SerMsk & 3)==2)) {
      Serial.print('=');
      Serial.println(Rail.Packages[pkgIdx],HEX);
    }
    if (exSpd && (((SerMsk & 3)==0) || ((SerMsk & 3)==1)))
      Serial.println('*');
    break;
  case 1:			// Pace+Ghost Car
    PaceGhost();
    break;
  case 2:			// Active
    break;
  case 3:			// Car Id0
    CarCtrl(0);
    if ((BBMode>=Start1Mode)&&(BBMode<=Start5Mode)&&(Speed[0]&15))
      return FailStart0Mode;
    break;
  case 4:			// Car Id4
    CarCtrl(4);
    if ((BBMode>=Start1Mode)&&(BBMode<=Start5Mode)&&(Speed[4]&15))
      return FailStart4Mode;
    break;
  case 5:			// Car Id1
    CarCtrl(1);
    if ((BBMode>=Start1Mode)&&(BBMode<=Start5Mode)&&(Speed[1]&15))
      return FailStart1Mode;
    break;
  case 6:			// Car Id5
    CarCtrl(5);
    if ((BBMode>=Start1Mode)&&(BBMode<=Start5Mode)&&(Speed[5]&15))
      return FailStart5Mode;
    break;
  case 7:			// Car Id2
    CarCtrl(2);
    if ((BBMode>=Start1Mode)&&(BBMode<=Start5Mode)&&(Speed[2]&15))
      return FailStart2Mode;
    break;
  case 8:			// Active
    break;
  case 9:			// Car Id3
    CarCtrl(3);
    if ((BBMode>=Start1Mode)&&(BBMode<=Start5Mode)&&(Speed[3]&15))
      return FailStart3Mode;
    break;
  }
  return -1;
}

void SwitchMode(char to)
{ // switch to new mode
  if ((to < 0) || (to > ErrorMode))
    return;
  if (BBMode != to) {
    if (((BBMode==PowerUpMode)||(BBMode==ErrorMode))&&(to!=PowerUpMode)&&(to!=ErrorMode))
      Rail.begin(pins_Track,5000,Rail.ActionPackage|Rail.ActionStartADC|Rail.ActionIRcomm);
    switch (to) {
    case ErrorMode:
      Rail.stop();
      break;
    case TrainMode:
      if ((BBMode>=ProgSpeed0Mode) && (BBMode<=ProgFuel9Mode))
	SendProgram(BBMode);
      Rail.appQueue(Rail.CmdReset);
      break;
    case RaceMode:
      tone(pins_Buzzer,523,600);
      Rail.appQueue(Rail.CmdStart);
      Rail.start();
#ifdef Carrera_Lap
      digitalWrite(pins_LapCount,HIGH);
#endif
      RaceStart = 38;
      break;
    case Start5Mode:
      tone(pins_Buzzer,440,300);
    case PreStartMode:
      Rail.appQueue(Rail.CmdLeds5);
      break;
    case Start1Mode:
      Rail.appQueue(Rail.CmdLeds1);
      tone(pins_Buzzer,440,300);
      break;
    case Start2Mode:
      Rail.appQueue(Rail.CmdLeds2);
      tone(pins_Buzzer,440,300);
      break;
    case Start3Mode:
      Rail.appQueue(Rail.CmdLeds3);
      tone(pins_Buzzer,440,300);
      break;
    case Start4Mode:
      Rail.appQueue(Rail.CmdLeds4);
      tone(pins_Buzzer,440,300);
      break;
    default:
      for(char i=0;i<Rail.numCars;i++)
	Rail.setCar(i,0,0,0);
      Rail.setPGCar(0,0,1,0);
    }
    if (SerMsk & 4) {
      Serial.print('>');
      Serial.println(to,HEX);
    }
#ifdef Module_LCD
    mode_i2c(to); // before change of BBmode!
#endif
    BBMode = to;
    if ((BBMode>=Start1Mode)&&(BBMode<=RaceMode))
      UpdateLed();
  }
}

void UpdateLed()
{ // set LED acording to BBMode
  static unsigned char ti;
  ti = Rail.micros() >> 16;	// [0.065536s] ~ 1/16 s
  LedDly = ((BBMode>=Start1Mode)&&(BBMode<=Start5Mode))?0x10:0x08;		// 1:0.5 s
  switch (BBMode) {
  case TrainMode:
  case RaceMode: // midle led on
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],((i==2)||((i==1)&&(PaceAct>0))||((i==3)&&(PaceAct<0)))?HIGH:LOW);
    break;
  case PreStartMode:
  case ProgSpeed9Mode:		
  case ProgBreak9Mode:
  case ProgFuel9Mode:
  case Start5Mode:		// all leds on
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],HIGH);
    break;
  case ProgSpeed1Mode:		
  case ProgBreak1Mode:
  case ProgFuel1Mode:
  case Start1Mode:		// led 1 on
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],(i==0)?HIGH:LOW);
    break;
  case ProgSpeed3Mode:		
  case ProgBreak3Mode:
  case ProgFuel3Mode:
  case Start2Mode:		// led 1-2 on
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],(i<2)?HIGH:LOW);
    break;
  case ProgSpeed5Mode:		
  case ProgBreak5Mode:
  case ProgFuel5Mode:
  case Start3Mode:		// led 1-3 on
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],(i<3)?HIGH:LOW);
    break;
  case ProgSpeed7Mode:		
  case ProgBreak7Mode:
  case ProgFuel7Mode:
  case Start4Mode:		// led 1-4 on
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],(i<4)?HIGH:LOW);
    break;
  case FailStart0Mode:		// led 1 flash
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],((i==0)&&(ti&2))?HIGH:LOW);
    LedDly = 0x01;
    break;
  case FailStart1Mode:		// led 2 flash
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],((i==1)&&(ti&2))?HIGH:LOW);
    LedDly = 0x01;
    break;
  case FailStart2Mode:		// led 3 flash
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],((i==2)&&(ti&2))?HIGH:LOW);
    LedDly = 0x01;
    break;
  case FailStart3Mode:		// led 4 flash
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],((i==3)&&(ti&2))?HIGH:LOW);
    LedDly = 0x01;
    break;
  case FailStart4Mode:		// led 5 flash
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],((i==4)&&(ti&2))?HIGH:LOW);
    LedDly = 0x01;
    break;
  case FailStart5Mode:		// led 1+5 flash
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],(((i==0)||(i==4))&&(ti&2))?HIGH:LOW);
    LedDly = 0x01;
    break;
  case ProgSpeed0Mode:		// led 1 blink
  case ProgBreak0Mode:
  case ProgFuel0Mode:
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],((i==0)&&(ti&8))?HIGH:LOW);
    LedDly = 0x04;
    break;
  case ProgSpeed2Mode:		// led 1 on, 2 blink
  case ProgBreak2Mode:
  case ProgFuel2Mode:
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],((i==0)||((i==1)&&(ti&8)))?HIGH:LOW);
    LedDly = 0x04;
    break;
  case ProgSpeed4Mode:		// led 1-2 on, 3 blink
  case ProgBreak4Mode:
  case ProgFuel4Mode:
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],((i<2)||((i==2)&&(ti&8)))?HIGH:LOW);
    LedDly = 0x04;
    break;
  case ProgSpeed6Mode:		// led 1-3 on, 4 blink
  case ProgBreak6Mode:
  case ProgFuel6Mode:
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],((i<3)||((i==3)&&(ti&8)))?HIGH:LOW);
    LedDly = 0x04;
    break;
  case ProgSpeed8Mode:		// led 1-4 on, 5 blink
  case ProgBreak8Mode:
  case ProgFuel8Mode:
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],((i<4)||((i==4)&&(ti&8)))?HIGH:LOW);
    LedDly = 0x04;
    break;
  case ErrorMode:
    char a,b;
    ti >>= 2;
    ti &=  3;
    a = (ti==0)?2:((ti==1)?1:((ti==2)?0:-1));
    b = (ti==0)?2:((ti==1)?3:((ti==2)?4:-1));
    for (int i=0;i<5;i++) 
      digitalWrite(LedPins[i],((i==a)||(i==b))?HIGH:LOW);
    LedDly = 0x01;
    break;
  }
}

///// Cars /////////////////////////////////////////////////
void PaceGhost()
{ // check pace and ghost car
  if ((BBMode==TrainMode) || (BBMode==RaceMode)) {
    Rail.setPGCar(1, PaceAct!=0, PaceAct>0, digitalRead(pins_Fuel_SW) == LOW);
  } else {
    Rail.setPGCar(0,0,1,0);
  }
}

void CarCtrl(unsigned char car)
{ // set speed and crossing of car
  char fuel = 0;
  if (Speed[car] == 0) { // no extern
    switch (car) {
    case 0:
      Speed[car] = Rail.ADval[Rail.carIdx[0]-1] >> 3;
      if (digitalRead(pins_LaneX1)==LOW)
	Speed[car] |= 0x10;
      break;
    case 1:
      Speed[car] = Rail.ADval[Rail.carIdx[1]-1] >> 3;
      if (digitalRead(pins_LaneX2)==LOW)
	Speed[car] |= 0x10;
      break;
    case 2:
      Speed[car] = Rail.ADval[Rail.carIdx[2]-1] >> 3;
      if (digitalRead(pins_LaneX3)==LOW)
	Speed[car] |= 0x10;
      break;
    case 3:
      Speed[car] = Rail.ADval[Rail.carIdx[3]-1] >> 3;
      if (digitalRead(pins_LaneX4)==LOW)
	Speed[car] |= 0x10;
      break;
    }
    Speed[car] |= 0x20;
  }
  if (Speed[car] == 0)
    return;
  if ((BBMode==TrainMode)||(BBMode==RaceMode)) { 
    fuel = digitalRead(pins_Fuel_SW) == LOW;
    Rail.setCar(car,Speed[car]&0x0f,Speed[car]&0x10,fuel);
  } else {
    Rail.setCar(car,0,Speed[car]&0x10,fuel);
  }
}

void SendProgram(char Mode)
{ // put program commands in que
  if ((Mode>=ProgSpeed0Mode)&&(Mode<=ProgSpeed9Mode)) {
    for (char c=0;c<Rail.numCars;c++) {
      // Spd 1,2,3,4,5, 7,9,11,13,15
      Rail.appProg(c,Rail.CmdSpeed,
		   (Mode<ProgSpeed5Mode)?Mode-ProgSpeed0Mode+1:(Mode-ProgSpeed5Mode)*2+7);
      Rail.appProg(c,Rail.CmdSpeed,
		   (Mode<ProgSpeed5Mode)?Mode-ProgSpeed0Mode+1:(Mode-ProgSpeed5Mode)*2+7);
    }
    return;
  }
  if ((Mode>=ProgBreak0Mode)&&(Mode<=ProgBreak9Mode)) {
    for (char c=0;c<Rail.numCars;c++) { // Brk 6..15
      Rail.appProg(c,Rail.CmdBreak,Mode-ProgBreak0Mode+6);
      Rail.appProg(c,Rail.CmdBreak,Mode-ProgBreak0Mode+6);
    }
    return;
  }
  if ((Mode>=ProgFuel0Mode)&&(Mode<=ProgFuel9Mode)) {
    for (char c=0;c<Rail.numCars;c++) { // Fuel 1..10
      Rail.appProg(c,Rail.CmdFuel,Mode-ProgFuel0Mode+1);
      Rail.appProg(c,Rail.CmdFuel,Mode-ProgFuel0Mode+1);
    }
    return;
  }
}

void CheckRails()
{ // check rails for short cut
  const unsigned char PwrOK = 125;
  const unsigned char RailDiff = 10;
#if 0
  if (SerMsk & 0x10) {
    Serial.print(F("Uref: "));
    Serial.print(Rail.ADval[0]);
    Serial.print(' ');
    Serial.println(Rail.ADval[5]);
    Serial.print(F("Urail: "));
    Serial.print(Rail.ADval[1]);
    Serial.print(' ');
    Serial.print(Rail.ADval[3]);
    Serial.print(' ');
    Serial.print(Rail.ADval[7]);
    Serial.print(' ');
    Serial.println(Rail.ADval[9]);
  }
#endif
  if ((Rail.ADval[0] < PwrOK) && (Rail.ADval[5] < PwrOK))
    SwitchMode(ErrorMode);	// supply not ok
  return;
  if ((Rail.ADval[1] < (Rail.ADval[0]-RailDiff))
      && (Rail.ADval[9] < (Rail.ADval[0]-RailDiff))
      && (Rail.ADval[3] < (Rail.ADval[5]-RailDiff))
      && (Rail.ADval[7] < (Rail.ADval[5]-RailDiff)))
    SwitchMode(ErrorMode);	// rail not ok
}

///// RS232 ////////////////////////////////////////////////
void newChar(char c)
{ // received new char from PC
#ifdef Carrera_CU
  static unsigned char SMsave = 0;
  if (c == '"') {	        // start carrera cmd
    CmdLen = 0;			// discard old chars
    if (SerMsk) {
      SMsave = SerMsk;
      SerMsk = 0;
    }
  }
#endif
  if ((c >= ' ') && (c <= '~') && (CmdLen < CmdMax))
    CmdLine[CmdLen++] = c;
#ifdef BBox_Cmd
  if ((c=='\n') || (c=='\r')) {
    if (CmdLen)
      BBcmd();
    CmdLen = 0;
    CmdLine[0] = 0;
  }
#endif
#ifdef Carrera_CU
  if ((CmdLine[0]=='"')&&(CmdLen))
    CUcmd();
  else if (SMsave) {
    SerMsk = SMsave;
    SMsave = 0;
  }
#endif
}

char BBcmd()
{ // parse special commands
#ifdef BBox_Cmd
  int i;
  switch (CmdLine[0]) {
  case '>':			// switch mode
    CmdLine[min(CmdLen,3)] = 0;
    i = strtol(CmdLine+1,NULL,16);
    SwitchMode(i);
    break;
  case '!':			// set debug mask
    CmdLine[min(CmdLen,3)] = 0;
    i = strtol(CmdLine+1,NULL,16);
    SerMsk = i;
    break;
  case '=':			// que prog command
    CmdLine[min(CmdLen,5)] = 0;
    i = strtol(CmdLine+1,NULL,16);
    Rail.appQueue(i);
    break;
  case 'a':			// car0 no laneX
  case 'b':
  case 'c':
  case 'd':
  case 'e':
  case 'f':
    CmdLine[min(CmdLen,2)] = 0;
    i = strtol(CmdLine+1,NULL,16);
    if (i >= 0)
      Speed[CmdLine[0]-'a'] = 0x80 | (i & 0x0f);
    break;
  case 'A':			// car0 w/ laneX
  case 'B':
  case 'C':
  case 'D':
  case 'E':
  case 'F':
    CmdLine[min(CmdLen,2)] = 0;
    i = strtol(CmdLine+1,NULL,16);
    if (i >= 0)
      Speed[CmdLine[0]-'A'] = 0x90 | (i & 0x0f);
    break;
  case 'p':			// pace car active
    CmdLine[min(CmdLen,2)] = 0;
    i = strtol(CmdLine+1,NULL,16);
    PaceAct = i;
    break;
  case 'P':			// pace car home
    CmdLine[min(CmdLen,2)] = 0;
    i = strtol(CmdLine+1,NULL,16);
    PaceAct = -i;
    break;
  }
#endif
}

void sendInt(unsigned long val,char nibles,char &csum)
{ // send val Carrera coded via RS232
  if (nibles > 2)
    sendInt(val>>8,nibles-2,csum);
  Serial.print((char)('0'+(val&15)));
  csum += val&15;
  if (nibles > 1) {
    val >>= 4;
    Serial.print((char)('0'+(val&15)));
    csum += val&15;
  }
}

char CUcmd()
{ // parse Carrera CU commands
#ifdef Carrera_CU
  char car = -1;
  if (CmdLen == 2) {
    switch (CmdLine[1]) {
    case 'H':
      if (RaceStart)
	Serial.print(F("H00$"));
      else
	Serial.print(F("H11$"));
      break;
    case '0':
      Serial.print(F("011114$"));
      break;
    case '?':
      Serial.print(CmdLine[1]);
      for(char c=0;c<D132::numCars;c++) 
	if((Leap[c]!=0)&&((car==-1)||(Leap[c]<Leap[car])))
	  car = c;
      if (car < 0)
	Serial.print('#');
      else {
	char p=0;
	sendInt(car,1,p);
	sendInt(Leap[car],8,p);
	sendInt(p,1,p);
	Serial.print('$');
	Leap[car] = 0;
      }
      break;
    default:
      Serial.print('#');
    };
    CmdLen = 0;
  }
#endif
}

///// I2C //////////////////////////////////////////////////
char setup_i2c() {
#if defined(Module_LCD) || defined(Module_Lights)
  Wire.begin();
#endif
#ifdef Module_LCD
  lcd.begin(16,2); // Cols:16 Lines:2
  lcd.print(F("BackBox"));
  lcd.setCursor(0,1);
  lcd.print(F("Digital 132"));
#endif
}

char mode_i2c(char to) {
  char chg = 1;
  switch(to) {
  case ErrorMode:
    lcd.clear();
    lcd.print(F("*** ??? ***"));
    break;
  case PreStartMode:
    lcd.clear();
    lcd.print(F(" - - - - -"));
    break;
  case Start1Mode:
    lcd.setCursor(1,0);
    lcd.write((uint8_t) '*');
    break;
  case Start2Mode:
    lcd.setCursor(3,0);
    lcd.write((uint8_t) '*');
    break;
  case Start3Mode:
    lcd.setCursor(5,0);
    lcd.write((uint8_t) '*');
    break;
  case Start4Mode:
    lcd.setCursor(7,0);
    lcd.write((uint8_t) '*');
    break;
  case Start5Mode:
    lcd.setCursor(9,0);
    lcd.write((uint8_t) '*');
    break;
  case RaceMode:
    lcd.setCursor(0,0);
    lcd.print(F("GO!......."));
    break;
  case ProgSpeed0Mode:
    lcd.clear();
    lcd.print(F("Speed: 1"));
    break;
  case ProgBreak0Mode:
    lcd.clear();
    lcd.print(F("Break: 1"));
    break;
  case ProgFuel0Mode:
    lcd.clear();
    lcd.print(F("Tank:  1"));
    break;
  case ProgSpeed1Mode:
  case ProgBreak1Mode:
  case ProgFuel1Mode:
    lcd.setCursor(7,0);
    lcd.write((uint8_t) '2');
    break;
  case ProgSpeed2Mode:
  case ProgBreak2Mode:
  case ProgFuel2Mode:
    lcd.setCursor(7,0);
    lcd.write((uint8_t) '3');
    break;
  case ProgSpeed3Mode:
  case ProgBreak3Mode:
  case ProgFuel3Mode:
    lcd.setCursor(7,0);
    lcd.write((uint8_t) '4');
    break;
  case ProgSpeed4Mode:
  case ProgBreak4Mode:
  case ProgFuel4Mode:
    lcd.setCursor(7,0);
    lcd.write((uint8_t) '5');
    break;
  case ProgSpeed5Mode:
  case ProgBreak5Mode:
  case ProgFuel5Mode:
    lcd.setCursor(7,0);
    lcd.write((uint8_t) '6');
    break;
  case ProgSpeed6Mode:
  case ProgBreak6Mode:
  case ProgFuel6Mode:
    lcd.setCursor(7,0);
    lcd.write((uint8_t) '7');
    break;
  case ProgSpeed7Mode:
  case ProgBreak7Mode:
  case ProgFuel7Mode:
    lcd.setCursor(7,0);
    lcd.write((uint8_t) '8');
    break;
  case ProgSpeed8Mode:
  case ProgBreak8Mode:
  case ProgFuel8Mode:
    lcd.setCursor(7,0);
    lcd.write((uint8_t) '9');
    break;
  case ProgSpeed9Mode:
  case ProgBreak9Mode:
  case ProgFuel9Mode:
    lcd.setCursor(6,0);
    lcd.print(F("10"));
    break;
  default:
    chg = 0;
    break;
  };
  if (chg)
    CarDis = 0;
  return chg;
}

void Lap_i2c(char car) {
  long us = Rail.Score[car] * 8192L;
  lcd.setCursor(0,car);
  lcd.print((char)('A'+car));
  lcd.print(Rail.Rounds[car]-1);
  lcd.print((char)(' '));
  lcd.print(us/1000000L,DEC);
  lcd.print((char)('.'));
  lcd.print((us/10000L)%100,DEC);
  lcd.print((char)(' '));
  lcd.print((char)(' '));
  lcd.print((char)(' '));
}
///// interrupts ///////////////////////////////////////////
#ifdef Car_IR
ISR(INT1_vect)
{
  static unsigned long lane1_prev = 0;
  static unsigned char lane1_ccar = 0x40;;
  if ((ir_identify(lane1_prev, lane1_ccar)>=0)&&((lane1_ccar & 0x30) == 0)) {
#ifdef Carrera_CU
    Leap[lane1_ccar&7] = millis();
#endif
    PORTC |= 4;
  } 
}

ISR(TIMER1_CAPT_vect)
{
  static unsigned long lane2_prev = 0;
  static unsigned char lane2_ccar = 0x80;
  //ir_identify(lane2_prev, lane2_ccar);
  if ((ir_identify(lane2_prev, lane2_ccar)>=0)&&((lane2_ccar & 0x30) == 0)) {
#ifdef Carrera_CU
    Leap[lane2_ccar&7] = millis();
#endif
    PORTC |= 8;
  }
}
#endif /* Car_IR */
