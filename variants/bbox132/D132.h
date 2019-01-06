#include <Arduino.h>
/************************************************************
 * class D132 coordinates the timing for BlackBox D132
 * 1) rail out put
 * 2) adc measurement between rail packages
 * 3) send IR request to towers 
 *
 * timing (fall. edge of 1. bit):
 * [us]   (1 Bit= 100us):
 * 0      ProgWord [13B]
 * 7500   Pace/Ghost  [10B]
 * 8450    (Pitlane 150us)
 * 15000  Aktiv [8B] (Ack [9B]) 
 * 18150   (Receive)
 * 22500  Car0 [10B]
 * 23450   (Pitlane 150us)
 * 25900   (Receive)
 * 30000  Car4 [10B]
 * 30950   (Pitlane 150us)
 * 33400   (Receive)
 * 37500  Car1 [10B]
 * 38450   (Pitlane 150us)
 * 40900   (Receive)
 * 45000  Car5 [10B]
 * 45950   (Pitlane 150us)
 * 48400   (Receive)
 * 52500  Car2 [10B]
 * 53450   (Pitlane 150us)
 * 55900   (Receive)
 * 60000  Active [8B]
 * 63200   (Receive)
 * 67500  Car3 [10B]
 * 68450   (Pitlane 150us)
 * 68600  IR on 30 pules (775us)
 * 69375  IR off 30 pulses (775us)
 * 70150  IR in 20 pulses (520us)
 * 70670  IR off
 * 70900   (Receive)
 * 75000  ProgWord
 * ...
 ***********************************************************/

class D132 
{
  static unsigned char volatile Action;
public:
  D132();
  ~D132();
  void begin(int rail, word ticks = 1000,const unsigned char action = ActionPackage); // start isr and output
  void stop();			/* stop data and power on rail */
  boolean isEmpty();
  void write(word data);
  static char pkgLen(word data);

  void setCar(char car,char speed=-1,char change=-1,char fuel=0);
  static uint16_t teleCar(char car,char speed=-1,char change=-1,char fuel=0,uint16_t old=0);
  void setPGCar(char enable=-1,char pace=-1,char home=-1,char fuel=-1);
  static uint16_t telePGCar(char enable=-1,char pace=-1,char home=-1,char fuel=-1,uint16_t old=0);
  static uint16_t telePGM(char car=0,char par=4,char val=1);
  char appQueue(uint16_t cmd);
  char appProg(char car=0,char par=4,char val=1);
  static const char prgLen = 16;
  static uint16_t volatile prgQueue[prgLen];
  enum railCommands { // command of BlackBox 2008
    CmdSpeed = 0,
    CmdBreak = 1,
    CmdFuel  = 2,
    CmdNOp   = 4,
    CmdReset = 0x10C3,
    CmdLeds0 = 0x1003,		/* all leds off */
    CmdLeds1 = 0x1803,		/* led  1 on */
    CmdLeds2 = 0x1403,		/* leds 1-2 on */
    CmdLeds3 = 0x1C03,		/* leds 1-3 on */
    CmdLeds4 = 0x1203,		/* leds 1-4 on */
    CmdLeds5 = 0x1A03,		/* leds 1-5 on */
    CmdStart = 0x1023,
  };

  static const char numCars = 6;
  static const char carIdx[numCars];
  static const int ActionPackage = 1; // start next package
  static const int ActionStartADC = 2; // start adc convertion
  static const int ActionInterrupt = 4; // fire SPM_RDY isr
  static const int ActionIRcomm = 8; // send IR pulses from towers after sequence
  static const int numPackages = 10;
  static uint16_t volatile Packages[numPackages];
  //static uint8_t volatile PackageLen[numPackages];
  static const uint8_t ADmuxDft = _BV(ADLAR); 
  static uint8_t volatile ADmux[numPackages];
  static uint8_t volatile ADval[numPackages];
  static char volatile _PackageIndex,_NOpCnt,_PrgCnt;
  static unsigned int volatile SequenceCount;
  // in updated the isr's set bits, so user prg can check for updates
  static uint16_t volatile updated;
  static uint16_t changed(uint16_t what=0); /* check for updates */
  enum updateBits {
    upd_transmited = 1,
    upd_telegram   = 2,
    upd_sequence   = 4,
    upd_adc        = 8,
    upd_ir_ctrl    = 16,
  };
  static unsigned char _transmited(uint16_t ticks_left, uint16_t telegram);	// will be called from isr only
  //static const long Score_us = 1000; /* factor for Score[] times */
  static word Score[numCars];
  static unsigned char Rounds[numCars];
  static void _car_lane(unsigned char car, unsigned long mu_sec); // called by isr
  static void print_debug();

  static long race_start;	/* start time in [128us] */
  static unsigned char micro_count; /* micros overflow */
  static unsigned long micros();    /* best micro secs */
  static unsigned long now(unsigned long us=0);	    /* now in [128us] */
  void start();			    /* race is started: reset counters */
};

//const char D132::carIdx[numCars] = {3,5,7,9,4,6};

/************************************************************
 * isr of Timer0  Compare match
 ************************************************************
 * the ocr is changed to setup the different timings
 * for rail telegrams and the ir pulses
 * after a telegram or the ir pulse pattern isr_next() is
 * called. D132.cpp provides default function but can
 * be overwritten by user.
 *
 * isr_next() is called w/ ticks left to next telegram and
 * the telegram data. The telegram is <> 0 only once a 
 * has been send telegram, and =0 otherwise. 
 * isr_next() must return before next irq occurs.
 * return value should be one of the folowing defines to say
 * the isr what to do next:
 ***********************************************************/
uint8_t isr_next(uint16_t ticks_left, uint16_t telegramm);

// ISR of car identified by ir
void ir_car(unsigned char car, unsigned long mu_sec);

// ir_identify() can be called from isr to detect cars by ir sensor
// on successfill detection ir_car() will be called
char ir_identify(unsigned long &lane_prev, unsigned char &cnt_car);

// ISR of car ir controler
void ir_ctrl(uint16_t &data, uint8_t pin);

// decode manchester telegram from ir towers or rails
void carrera_decode(uint8_t pin, uint16_t snd_msk, uint16_t period,
		    unsigned long &pervious, uint16_t &data);

#define ISR_NXT_256us   0
// call isr_next() w/ next irq in 256us
#define ISR_NXT_100us   1
// call isr_next() w/ next irq in 100us
#define ISR_NXT_50us    2
// call isr_next() w/ next irq in 50us
#define ISR_NXT_DONE    ISR_NXT_256us
// state to switch when done w/ telegram or ir
#define ISR_NXT_NONE    4
// don't call isr_next() until next telegram
#define ISR_NXT_ETGM    5
// end telegram: rails on and next irq after 50us
#define ISR_NXT_IR30p   30
// send 30 ir pulses on next irq, than...
#define ISR_NXT_PS30p   31
// pause for 30 ir pulses 
#define ISR_NXT_PS20p   32
// pause for 20 ir pulses 
#define ISR_NXT_PS10p   33
// pause for 10 ir pulses, than ... 
#define ISR_NXT_IR20p   34
// send 20 ir pulses on next irq, than...
#define ISR_NXT_IRoff   35
// power off ir led
#define ISR_NXT_IRclr   36

#define TLGM_PIT_LANE   0x8000
// highest bit for pit lane low pulse 
