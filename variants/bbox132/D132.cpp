#include <Arduino.h>

// at full speed two cars are seperated by at least 30ms
/*
 *  Part of CBox132 - https://github.com/maassen/cbox132
 *
 * Copyright (c) 2019 Michael Maassen
 */ 
#define RAIL_IR_NEXT    30000

#include <D132.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// variables accessed from class and isr
volatile uint16_t rail_out;
volatile uint8_t  rail_len;

#include "D132_HW.inc"
/*******  ^^^^^^^^^^^
 * must define:
 * isr_begin() to setup and start isr w/ puls generation
 * isr_stop()  to stop pulse generation
 * -----------------------------------------------------
 * isr gets data from rail_out and rail_len; clears rail_len when data taken
 * isr calls isr_next() at least once after each telegram
 ***********************************************************/

#ifndef mu_sec
#define mu_sec()        ::micros()
#endif

// local defines ////////////////////////////////////////////
#define ChUref  analogPinToChannel(6)
#define ChUrail analogPinToChannel(7)
#define ChSpd0  analogPinToChannel(0)
#define ChSpd1  analogPinToChannel(1)
#define ChSpd2  analogPinToChannel(2)
#define ChSpd3  analogPinToChannel(3)

// consts ///////////////////////////////////////////////////
const char D132::carIdx[numCars] = {3,5,7,9,4,6};

// vars /////////////////////////////////////////////////////
char volatile D132::_PackageIndex = -1;
volatile unsigned int D132::SequenceCount = 0;
unsigned char volatile D132::Action = 0;
char volatile D132::_NOpCnt = 0;
char volatile D132::_PrgCnt = 0;
static uint16_t volatile D132::updated = 0;
static long D132::race_start = 0;       /* start time in 128us */
static unsigned char D132::micro_count=0; /* micros overflow */
uint16_t volatile D132::prgQueue[D132::prgLen];
uint16_t volatile D132::Packages[numPackages];
uint8_t volatile D132::ADmux[numPackages];
uint8_t volatile D132::ADval[numPackages];
static word D132::Score[numCars];
static unsigned char D132::Rounds[numCars];

// class methodes ///////////////////////////////////////////
D132::D132()
{  
  _PackageIndex = -1;		// not yet started
  Packages[0] = 0x1820;
  Packages[1] = telePGCar(0,0,0,0);
  Packages[3] = teleCar(0,0,0,0);
  Packages[4] = teleCar(4,0,0,0);
  Packages[5] = teleCar(1,0,0,0);
  Packages[6] = teleCar(5,0,0,0);
  Packages[7] = teleCar(2,0,0,0);
  Packages[9] = teleCar(3,0,0,0);
  Packages[2] = Packages[8] = 0x80;

  ADmux[0]    = ADmuxDft | ChUref;
  ADmux[1]    = ADmuxDft | ChUrail;
  ADmux[2]    = ADmuxDft | ChSpd0;
  ADmux[3]    = ADmuxDft | ChUrail;
  ADmux[4]    = ADmuxDft | ChSpd1;
  ADmux[5]    = ADmuxDft | ChUref;
  ADmux[6]    = ADmuxDft | ChSpd2;
  ADmux[7]    = ADmuxDft | ChUrail;
  ADmux[8]    = ADmuxDft | ChSpd3;
  ADmux[9]    = ADmuxDft | ChUrail;
  
  for(int i = 0; i< prgLen; i++)
    prgQueue[i] = 0;
  start();
}

D132::~D132()
{
  stop();
}

void D132::begin(int rail, word ticks, const unsigned char action)
{
  Action = action;
  _PackageIndex = 0;
  isr_begin(rail,pins_TowerOut,ticks);
  if (Action)
    _transmited(0,0);
  start();
}

void D132::stop()
{
  isr_stop();
}

boolean D132::isEmpty()
{
  return (rail_len)? false : true;
}

char D132::pkgLen(word data)
{
  char l = 0;
  data &= ~TLGM_PIT_LANE;
  while (data) {
    data >>= 1;
    l++;
  }
  return l;
}

void D132::write(word data)
{
  char l;
  while( ! isEmpty());
  l = pkgLen(data);
  if (! l) return;
  rail_out = data;
  rail_len = l;
#ifdef DEBUG
  Serial.print("write");
  Serial.print(data);
  Serial.print(" ");
  Serial.println(l);
#endif /* DEBUG */
}

void D132::setCar(char car,char speed,char change,char fuel)
{ // update car telegram
  uint16_t old,tel,akt;
  uint8_t oldSREG;
  if (car < 0 || car >= numCars)
    return;
  old = Packages[carIdx[car]];
  tel = teleCar(car,speed,change,fuel,old);
  akt = Packages[8];
  if (speed > 0) {
    akt |= (64 >> car) | 1;
  } else if (speed == 0) {
    akt &= ~(64 >> car);
    if (! (akt & 126))
      akt = 128;		// no car drives
  }
  oldSREG = SREG;
  cli();
  Packages[carIdx[car]] = tel;
  Packages[8] = Packages[2] = akt;
  SREG = oldSREG;
}

uint16_t D132::teleCar(char car,char speed,char change,char fuel,uint16_t old)
{ // calc new tetegram
  old |= 512;			// startbit
  old &= ~(256|128|64);		// car address
  old |= (car&1?64:0)|(car&2?128:0)|(car&4?256:0);
  if (change > 0) 
    old &= ~32;
  else if (change == 0) 
    old |= 32;
  if (speed >= 0) {
    old &= ~(16|8|4|2);
    old |= (speed&1?2:0)|(speed&2?4:0)|(speed&4?8:0)|(speed&8?16:0);
  }
  if (fuel > 0) 
    old |= 1;
  else if (fuel == 0) 
    old &= ~1;
  return old;
}

void D132::setPGCar(char enable,char pace,char home,char fuel)
{
  uint16_t old,tel;
  old = Packages[1];
  tel = telePGCar(enable,pace,home,fuel,old);
  Packages[1] = tel;
}

uint16_t D132::telePGCar(char enable,char pace,char home,char fuel,uint16_t old)
{ // setup telegram from pace and ghost car
  old |= 512|256|128|64;			// startbit + adr7
  if (enable >= 0) {
    old |= (enable)?8:32;
    old &= ~((!enable)?8+16:32);
  }
  if (home > 0) 
    old |= 4;
  else if (home == 0)
    old &= ~4;
  if (pace > 0) 
    old |= 2;
  else if (pace == 0)
    old &= ~(2+4);
  if (fuel > 0) 
    old |= 1;
  else if (fuel == 0) 
    old &= ~1;
  return old;
}
  
uint16_t D132::telePGM(char car,char par,char val)
{ // compose program telegram
  uint16_t tel = 4096;		// start bit
  tel |= ((car&1)?4:0)|((car&2)?2:0)|((car&4)?1:0);
  tel |= ((par&1)?128:0)|((par&2)?64:0)|((par&4)?32:0)|
    ((par&8)?16:0)|((par&16)?8:0);
  tel |= ((val&1)?2048:0)|((val&2)?1024:0)|((val&4)?512:0)|((val&8)?256:0);
  return tel;
}

char D132::appQueue(uint16_t cmd)
{ // append command to queue
  
  for (char i=0;i<prgLen;i++) 
    if (!prgQueue[(_PrgCnt+i)%prgLen]) {
      uint8_t oldSREG = SREG;
      cli();
      prgQueue[(_PrgCnt+i)%prgLen] = cmd;
      SREG = oldSREG;
      return i;
    }
  return -1; 			// not queued!
}
char D132::appProg(char car,char par,char val) 
{
  return appQueue(telePGM(car, par, val));
}

uint16_t D132::changed(uint16_t what=0)
{ /* check for updates */
  if (what == 0)
    return updated;
  what &= updated;
  if (what == 0)
    return 0;
  updated &= ~what;
  return what;
}

unsigned long D132::micros() {
  unsigned long us = mu_sec();
  // MSB of mu_sec should be equal to LSB of micro_count
  if (((us & 1<31)==0) != ((micro_count & 1)==0))
    micro_count += 1;
  return us;
}

unsigned long D132::now(unsigned long us)
{ /* now in 128us */
  if (us == 0)
    us = micros();
  us >>= 7;
  us |= micro_count << 24;
  return us;
}

void D132::start()
{ /* race is started: reset counters */
  race_start = now();
  for(int i = 0; i< numCars; i++) {
    Score[i] = 0;
    Rounds[i] = 0;
  }
}
// ISRs of class ////////////////////////////////////////////
unsigned char D132::_transmited(uint16_t ticks_left, uint16_t telegram)
{ // prepare next package
  updated |= upd_transmited;
  if ( telegram ) { // first call after telegram
    _PackageIndex = (_PackageIndex+1)%numPackages;
    updated |= upd_telegram;
    if (_PackageIndex == 0) {
      SequenceCount++;
      updated |= upd_sequence; }
    if (Action & ActionInterrupt)
      // if SPMIE is set(SPMCR|=_BV(SPMIE)), SPM_RDY we be called
      sbi(SPMCR,SPMEN);
  }
  if ( ((ticks_left>>8) == 24) && (Action & ActionIRcomm) && ( _PackageIndex == 0)) {
    return ISR_NXT_IR30p;	// req. sending IR pulses
  }
  if ( ((ticks_left>>8) == 14) && (Action & ActionStartADC)) {
    ADMUX = ADmux[_PackageIndex];
    return ISR_NXT_256us;
  }
  if ( ((ticks_left>>8) == 12) && (Action & ActionStartADC)) {
    sbi(ADCSRA,  ADSC);		// ADC Start Conversion
    return ISR_NXT_256us;
  }
  if ( ((ticks_left>>8) == 10) && (Action & ActionStartADC)) {
    ADval[_PackageIndex] = ADCH;
    updated |= upd_adc;
    return ISR_NXT_256us;
  }
  if (((ticks_left>>8) == 2) && (Action & ActionPackage) &&
      (_PackageIndex >= 0) && (rail_len == 0)) {
    if (_PackageIndex == 0) { // Program
      if (prgQueue[_PrgCnt%prgLen]) {
	Packages[_PackageIndex] = prgQueue[_PrgCnt%prgLen];
	prgQueue[_PrgCnt%prgLen] = 0;
	_PrgCnt += 1;
      } else
	Packages[_PackageIndex] = telePGM((_NOpCnt++)%6);
    }
    if (_PackageIndex == 1) { // Pace & Ghost
      if (Packages[1] & 8) { // FR == 1
	Packages[1] ^= 16;   // toggle TK
      }
    }
    rail_out = Packages[_PackageIndex];
    rail_len = pkgLen(Packages[_PackageIndex]);
    return ISR_NXT_256us; //return ISR_NXT_NONE;
  }
  return ISR_NXT_256us;
}

void D132::_car_lane(unsigned char car, unsigned long mu_sec) // called by isr
{
  car &= 0x0f;
  if (car < numCars) {
    Score[car] = (now(mu_sec) - race_start)>>6; // ~ [8ms] = 64*128us
    Rounds[car] += 1;
    updated |= 1 << (car+8);
  }
}

void D132::print_debug()
{
#define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
  Serial.print(F("F_CPU: "));
  Serial.println(F_CPU);


#ifdef RAIL_CLK_SRC
  Serial.print(F("RAIL_CLK_SRC: "));
  Serial.println(RAIL_CLK_SRC);
#elif defined(  TIMER2_CLK_SRC ) 
  Serial.print(F("TIMER2_CLK_SRC: "));
  Serial.println(TIMER2_CLK_SRC);
#else
  Serial.print(F("TIMER0_CLK_SRC: "));
  Serial.println(TIMER0_CLK_SRC);
#endif /* TIMER2_CLK_SRC */

  Serial.print(F("CYCLES_PER_TICK: "));
  Serial.println(CYCLES_PER_TICK);

  Serial.print(F("TELEGRAM_PERIODE [us]: "));
  Serial.println(TELEGRAM_PERIODE);

  Serial.print(F("TELEGRAM_TICKS: "));
  Serial.println(TELEGRAM_TICKS);

  Serial.print(F("BIT_PERIODE[us]: "));
  Serial.println(BIT_PERIODE);

  Serial.print(F("BIT_TICKS: "));
  Serial.println(BIT_TICKS);

  Serial.print(F("BIT2_TICKS: "));
  Serial.println(BIT2_TICKS);

  Serial.print(F("rail_mask: "));
#ifdef RAIL_MASK
  Serial.println(RAIL_MASK);
#else
  Serial.println(rail_mask);
#endif
  
  Serial.print(F("rail_port: "));
#ifdef RAIL_PORT
  Serial.println(RAIL_PORT);
#else
  Serial.println((int)rail_port);
#endif
  
#ifdef RAIL_LOWACTIVE
  Serial.println(F("RAIL_LOWACTIVE: defined"));
#else
  Serial.println(F("RAIL_LOWACTIVE: not defined"));
#endif

  Serial.print(F("RailDat: "));
  Serial.println(RailDat);

  Serial.print(F("RailPos: "));
  Serial.println((int)RailPos);

  Serial.print(F("lastComp: "));
  Serial.println(lastComp);

  Serial.print(F("nextTelegram: "));
  Serial.println(nextTelegram);

  Serial.print(F("isrCount: "));
  Serial.println(isrCount);

  Serial.print(F("_PackageIndex: "));
  Serial.println((int)_PackageIndex);

  Serial.print(F("SequenceCount: "));
  Serial.println(SequenceCount);

#ifdef RAIL_TIMER_TICKS_PER_US  
  Serial.print(F("RAIL_TIMER_TICKS_PER_US: "));
  Serial.println(RAIL_TIMER_TICKS_PER_US);
#endif
#ifdef RAIL_TIMER_US_PER_TICKS
  Serial.print(F("RAIL_TIMER_US_PER_TICKS: "));
  Serial.println(RAIL_TIMER_US_PER_TICKS);
#endif
  
  Serial.print(F("Action: "));
  Serial.println(Action);

  Serial.print(F("Packages[0]: $"));
  Serial.println(Packages[0],HEX);

  Serial.print(F("Packages[1]: $"));
  Serial.println(Packages[1],HEX);

  Serial.print(F("Packages[2]: $"));
  Serial.println(Packages[2],HEX);

  Serial.print(F("Packages[3]: $"));
  Serial.println(Packages[3],HEX);

  Serial.print(F("Packages[4]: $"));
  Serial.println(Packages[4],HEX);

  Serial.print(F("Packages[5]: $"));
  Serial.println(Packages[5],HEX);

  Serial.print(F("Packages[6]: $"));
  Serial.println(Packages[6],HEX);

  Serial.print(F("Packages[7]: $"));
  Serial.println(Packages[7],HEX);

  Serial.print(F("Packages[8]: $"));
  Serial.println(Packages[8],HEX);

  Serial.print(F("Packages[9]: $"));
  Serial.println(Packages[9],HEX);
#endif /* def DEBUG_SERIAL */
}

////////////////////////////////////////////////////////////
// weak implementatins of ISR call backs for D132
////////////////////////////////////////////////////////////

// ISR of single telegram (7.5ms) 
uint8_t isr_next(uint16_t ticks_left, uint16_t telegram) __attribute__((weak));
uint8_t isr_next(uint16_t ticks_left, uint16_t telegram)
{
  return D132::_transmited(ticks_left, telegram);
}

// ISR of car identified by ir
void ir_car(unsigned char car, unsigned long mu_sec) __attribute__((weak));
void ir_car(unsigned char car, unsigned long mu_sec)
{
  return D132::_car_lane(car, mu_sec);
}

// ISR of car ir controler
void ir_ctrl(uint16_t &data, uint8_t pin) __attribute__((weak));
void ir_ctrl(uint16_t &data, uint8_t pin)
{
  return ;
}

////////////////////////////////////////////////////////////
// ISR work funktions, to be called from isr routine
////////////////////////////////////////////////////////////
char ir_identify(unsigned long &lane_prev, unsigned char &cnt_car)
{ // call on one or both changes of car ir sensor
  // calls ir_car() if car is detected
  // lane_prev, cnt_car carry info from call to call (not for user)
  unsigned long curr, delta;
  unsigned char car;
  curr = mu_sec();
  delta = curr - lane_prev; 
  lane_prev = curr;
  if (delta > RAIL_IR_NEXT) {
    cnt_car |= 0x3f;		// carid = 15 not existing
    return -1;
  }
  if ((cnt_car & 0x30) == 0)	// cnt_car: llcciiii == ll lane;cc count;iiii car id
    return -1;
  if ((delta > 48) && (delta < 600)) { // 48=64us+/-25%
    car = (delta+31)/64 - 1;
    if (car == (cnt_car & 0x0f)) {
      cnt_car -= 0x10;		// don't touch car id (low nibble)
      if ((cnt_car & 0x30) == 0) 
	ir_car(cnt_car, curr- 2*delta);
    } else {
      cnt_car &= 0xC0;
      cnt_car |= 0x10 | (car & 0x0f); // cnt = 1
    }
    return car;
  }
  return -1;
}

// decode manchester telegram from ir towers or rails
void carrera_decode(uint8_t pin, uint16_t snd_msk, uint16_t period,
		    unsigned long &previous, uint16_t &data){ 
  unsigned long currentMicros = mu_sec();                        
  unsigned long intervalMicros = currentMicros - previous;   
  if ((intervalMicros > (period - (period <<2))) &&
      (intervalMicros < (period + (period <<2)))) {  
    previous = currentMicros;
    data <<= 1;
    if ( (*portInputRegister(digitalPinToPort(pin))) & digitalPinToBitMask(pin))
      data |= 1;
  }                                        
  if ( (intervalMicros > (period << 2)) || (data & snd_msk)) {
    ir_ctrl(data,pin);
    data = 1;
    previous = currentMicros;                   // synchronise
  }
}                


/*
ISR(TIMER1_CAPT_vect)
{ // only used as ext. interrupt
  static unsigned long lane2_prev = 0;
  static unsigned char lane2_ccar;
  ir_identify(lane2_prev, lane2_ccar);
} 
*/

