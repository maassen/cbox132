/******************************************* -*- mode:C++ -*-
 * to be included from D132.cpp
 *
 * hardware (timer1) implementation of rail and ir puls output
 * so only OCR1x pins can be used for rail out.
 * can use timer0 (arduino standart system timer) for ir output
 *
 *  Part of CBox132 - https://github.com/maassen/cbox132
 *
 * Copyright (c) 2019 Michael Maassen
 * 
 ***********************************************************/
/////////////////////////////////////////////////////////////
// configure your hardware here
/////////////////////////////////////////////////////////////
#define RAIL_TIMER_NR   1
#define RAIL_OCR_SUF    B

#define RAIL_PORT_SUF   D
#define RAIL_PORT_BIT   4

// timer clock source settings
#define RAIL_CLK_SRC    2

// undefine for software support of IR out
#define IRout_TIMER_NR  0
//#define IRout_OCR_SUF   A
#define IRout_CLK_SRC    2
#define IRout_PORT_SUF  B
#define IRout_PORT_BIT  3
/////////////////////////////////////////////////////////////
// configure your hardware here (if needed)
/////////////////////////////////////////////////////////////
#define __cat(a,b)       a##b
#define _cat(a,b)        __cat(a,b)
#ifdef RAIL_OCR_SUF
#define _tmr(v)         _cat(v,_cat(RAIL_TIMER_NR,RAIL_OCR_SUF))
#else
#define _tmr(v)         _cat(v,RAIL_TIMER_NR)
#endif /* def RAIL_OCR_SUF */

// arduino pin number
#define RAIL_PIN_NR     _tmr(pins_OC)

// timer configuration registers (set to same if only one TCCR)
//#define RAIL_TCCRA      TCCR1A
#define RAIL_TCCRA      _cat(_cat(TCCR,RAIL_TIMER_NR),A)
//#define RAIL_TCCRB      TCCR1B
#define RAIL_TCCRB      _cat(_cat(TCCR,RAIL_TIMER_NR),B)


// compare match action on pin
#define RAIL_COM_HLD    0
#define RAIL_COM_CLR    (_BV(_cat(_tmr(COM),1)))
#define RAIL_COM_TGL    (_BV(_cat(_tmr(COM),0)))
#define RAIL_COM_SET    (RAIL_COM_CLR|RAIL_COM_TGL)
#define RAIL_COM_MSK    RAIL_COM_SET

// the compare register
#define RAIL_OCR        _tmr(OCR)
// max. value of timer bevor overflow (2^n-1)
#define RAIL_TIMER_TOP  0xFFFF

// interrupt
//#define RAIL_TIMER_TMSK _cat(TIMSK,RAIL_TIMER_NR)
#define RAIL_TIMER_TMSK TIMSK
#define RAIL_TIMER_OCIE _tmr(OCIE)
#ifdef RAIL_OCR_SUF
#define RAIL_COMP_vect  _cat(_cat(_cat(TIMER,RAIL_TIMER_NR),_COMP),_cat(RAIL_OCR_SUF,_vect))
#else
#define RAIL_COMP_vect  _cat(_cat(_cat(TIMER,RAIL_TIMER_NR),_COMP),_vect)
#endif /* def RAIL_OCR_SUF */

// these are for mu_sec()
#define RAIL_uSEC_vect  _cat(_cat(_cat(TIMER,RAIL_TIMER_NR),_OVF),_vect)
#define RAIL_TIMER_TCNT _cat(TCNT,RAIL_TIMER_NR)
#define RAIL_uSEC_TOIE  _cat(TOIE,RAIL_TIMER_NR)
/////////////////////////////////////////////////////////////
// end of hardware config
/////////////////////////////////////////////////////////////
#define RAIL_PORT       _cat(PORT,RAIL_PORT_SUF)
#define RAIL_MASK       _BV(RAIL_PORT_BIT)
  
#define IRout_PORT       _cat(PORT,IRout_PORT_SUF)
#define IRout_MASK       _BV(IRout_PORT_BIT)
#ifdef IRout_TIMER_NR
#define IRout_TCCR       _cat(TCCR,IRout_TIMER_NR)
#define IRout_TCNT       _cat(TCNT,IRout_TIMER_NR)
#define IRout_TIMSK      RAIL_TIMER_TMSK
#define IRout_TOIE       _cat(TOIE,IRout_TIMER_NR)
#ifdef IRout_OCR_SUF
#define IRout_COMP_vect  _cat(_cat(_cat(TIMER,IRout_TIMER_NR),_COMP),_cat(IRout_OCR_SUF,_vect))
#define IRout_OCR        _cat(_cat(OCR,IRout_TIMER_NR),IRout_OCR_SUF)
#define IRout_COM0       _cat(_cat(_cat(COM,IRout_TIMER_NR),0),IRout_OCR_SUF)
#define IRout_OCIE       _cat(_cat(OCIE,IRout_TIMER_NR),IRout_OCR_SUF)
#else
#define IRout_COMP_vect  _cat(_cat(_cat(TIMER,IRout_TIMER_NR),_COMP),_vect)
#define IRout_OCR        _cat(OCR,IRout_TIMER_NR)
#define IRout_COM0       _cat(_cat(COM,IRout_TIMER_NR),0)
#define IRout_COM1       _cat(_cat(COM,IRout_TIMER_NR),1)
#define IRout_WGM0       _cat(_cat(WGM,IRout_TIMER_NR),0)
#define IRout_WGM1       _cat(_cat(WGM,IRout_TIMER_NR),1)
#define IRout_OCIE       _cat(OCIE,IRout_TIMER_NR)
#endif /* def IRout_OCR_SUF */
#endif /* IRout_TIMER_NR */
  
#define TIMER1_CLK_SRC  2	// use timer1 (/8)
// the following macros are identical to wiring.c
#ifndef TIMER1_CLK_SRC
#if F_CPU < 250000L
#define TIMER1_CLK_SRC 1	/* CS0 :1 */
#elif F_CPU < 2000000L
#define TIMER1_CLK_SRC 2	/* CS0 | CS1 :8*/
#else
#define TIMER1_CLK_SRC 3	/* CS0 | CS1 :64*/
#endif
#endif

#ifndef CYCLES_PER_TICK
#if TIMER1_CLK_SRC == 1
#define CYCLES_PER_TICK 1
#elif TIMER1_CLK_SRC == 2
#define CYCLES_PER_TICK 8
#elif TIMER1_CLK_SRC == 3
#define CYCLES_PER_TICK 64
#elif TIMER1_CLK_SRC == 4
#define CYCLES_PER_TICK 256
#elif TIMER1_CLK_SRC == 5
#define CYCLES_PER_TICK 1024
#else 
#error "TIMER1_CLK_SRC has illigal value"
#endif
#endif /* CYCLES_PER_TICK */


// time from telegram to telegram in us
#define TELEGRAM_PERIODE 7500L
// timer ticks from telegram to telegram
#define TELEGRAM_TICKS (F_CPU/CYCLES_PER_TICK/1000000L*TELEGRAM_PERIODE)
// time between two bit starting edges in us
#define BIT_PERIODE 100L
// timer ticks for a bit periode
#define BIT_TICKS (F_CPU/CYCLES_PER_TICK/1000000L*BIT_PERIODE)
// timer ticks for intermediate edge
#define BIT2_TICKS (BIT_TICKS/2)
// IR sends w/ 38kHz and 1:1 ratio (= 13us / half wave)
#define IR_PULSE_PERIODE 13
// and in ticks
#define IRpulse_TICKS (F_CPU/CYCLES_PER_TICK/1000000L*IR_PULSE_PERIODE)

#if ((F_CPU/CYCLES_PER_TICK) > 1000000L)
#define RAIL_TIMER_TICKS_PER_US (F_CPU/CYCLES_PER_TICK/1000000L)
#else
#define RAIL_TIMER_US_PER_TICKS (1000000L*CYCLES_PER_TICK/F_CPU)
#endif
// variabes accessed from isr and begin() end()
volatile int8_t irPin=-1;

#ifdef RAIL_LOWACTIVE
#define rail_off()      sbi(RAIL_PORT,RAIL_PORT_BIT)
#define rail_on()       cbi(RAIL_PORT,RAIL_PORT_BIT)
#define rail_set()      (RAIL_TCCRA=(RAIL_TCCRA & ~RAIL_COM_MSK)|RAIL_COM_CLR)
#define rail_clr()      (RAIL_TCCRA=(RAIL_TCCRA & ~RAIL_COM_MSK)|RAIL_COM_SET)
#else
#define rail_on()       sbi(RAIL_PORT,RAIL_PORT_BIT)
#define rail_off()      cbi(RAIL_PORT,RAIL_PORT_BIT)
#define rail_clr()      (RAIL_TCCRA=(RAIL_TCCRA & ~RAIL_COM_MSK)|RAIL_COM_CLR)
#define rail_set()      (RAIL_TCCRA=(RAIL_TCCRA & ~RAIL_COM_MSK)|RAIL_COM_SET)
#endif /* RAIL_LOWACTIVE */
#define rail_tgl()      (RAIL_TCCRA=(RAIL_TCCRA & ~RAIL_COM_MSK)|RAIL_COM_TGL)
#define rail_hld()      (RAIL_TCCRA=(RAIL_TCCRA & ~RAIL_COM_MSK)|RAIL_COM_HLD)


// variables only accesed from isr:
static volatile uint16_t RailDat;	// actual data to shift
static volatile uint16_t RailPos = 0;	// mask of actual bit
static volatile uint16_t lastComp = 0;	// value of last OCR val
static volatile uint16_t nextTelegram = 0;   // ticks left for next telegram
static volatile uint8_t rail_toggle = 0; // delay after next toggle
static volatile uint8_t ir_pulse = 0;   // toggle ir if >0
static volatile uint8_t state_m = ISR_NXT_NONE; // isr state machine
#ifdef RAIL_uSEC_vect
static volatile uint16_t rail_timer_cnt = 0;
#endif
#ifndef IRout_TIMER_NR
static volatile uint8_t *ir_port, ir_mask;
#define IRout_PORT      (*ir_port)
#define IRout_MASK      (ir_mask)
#endif
static volatile uint16_t isrCount = 0;   // just count isr calls DEBUG

static int isr_begin(int rail, int ir_out, word ticks)
{
  RailPos = 0;
  nextTelegram = ticks;
  rail_toggle = 0;
  ir_pulse = 0;
  state_m = ISR_NXT_NONE;
  
  // set timer to normal mode
  RAIL_TCCRA = 0;			// WGM=0 COM=0
  RAIL_TCCRB = RAIL_CLK_SRC;

#ifdef RAIL_uSEC_vect
  rail_timer_cnt = 0;
  RAIL_TIMER_TCNT = 0;
  sbi(RAIL_TIMER_TMSK, RAIL_uSEC_TOIE);
#endif
  
  if ((rail != RAIL_PIN_NR))
    return -1;

  pinMode(rail,OUTPUT);
  rail_on();
  rail_set();			// set rail on at next irq
  RAIL_OCR =  RAIL_TIMER_TCNT + 10;
  sbi(RAIL_TIMER_TMSK, RAIL_TIMER_OCIE);

  if(ir_out >= 0) {
    pinMode(ir_out,OUTPUT);
#ifdef IRout_TIMER_NR
    //IRout_TCCR &= ~(_BV(IRout_WGM0)|_BV(IRout_WGM1)|_BV(IRout_COM0)|_BV(IRout_COM1));
    //sbi(IRout_TIMSK, IRout_OCIE);
#else
    ir_port = portOutputRegister(digitalPinToPort(ir_out));
    ir_mask = digitalPinToBitMask(ir_out);
    *ir_port &= ~ir_mask;
#endif
  }
}

static void isr_stop()
{
  // disable timer capture interrupt
  cbi(RAIL_TIMER_TMSK, RAIL_TIMER_OCIE);
  rail_hld();
  rail_off();
}

ISR(RAIL_COMP_vect)
{
  static word last;
  //digitalWrite(pins_LED3,HIGH);
  isrCount++;
#ifndef IRout_TIMER_NR
  if (ir_pulse) {
    // fast! no update of nextTelegram
    IRout_PORT ^= IRout_MASK;
    ir_pulse--;
    lastComp += IRpulse_TICKS;
    RAIL_OCR = lastComp;
    return;
  }
#endif
  if (rail_toggle) {
    // fast! no update of nextTelegram
    rail_tgl();
    lastComp += rail_toggle;
    rail_toggle = 0;
    RAIL_OCR = lastComp;
    return;
  }
  if (RailPos) {
    // start a new bit, by toggling
    //char last = (RailDat & RailPos) ? 1 : 0;
    last = (RailDat & RailPos) ? 1 : 0;
    RailPos >>= 1;
    //rail_tgl();
    if (RailPos) { // at least one bit follows
      if (last)
	rail_set();		  // set rail on next ocr match
      else
	rail_clr();		  // clear rail on next ocr match 
      if (last == ((RailDat & RailPos)?1:0)) {
	// next bit is same as current bit
	// we have to toggle after 50 us
	rail_toggle = BIT2_TICKS; // 50us
	lastComp += BIT2_TICKS;
	RAIL_OCR = lastComp;
	nextTelegram -= BIT_TICKS; // 100us
      } else { // current and next bit differ
	lastComp += BIT_TICKS;	// 100us
	RAIL_OCR = lastComp;
	nextTelegram -= BIT_TICKS;
      }
      return;
    } else { // last bit of telegram
      //state_m = ISR_NXT_ETGM; // set rail on at next irq
      if (RailDat & TLGM_PIT_LANE) { // append pit lane pulse
	if (last) { // we are low right now
	  rail_set();
	  lastComp += 2*BIT_TICKS;	// 200us
	  RAIL_OCR = lastComp;
	  nextTelegram -= 2*BIT_TICKS;
	} else { // set low after 50us for 150us
	  rail_clr();
	  rail_toggle = 2*BIT_TICKS-BIT2_TICKS; // 150us
	  lastComp += BIT2_TICKS;	// 50us
	  RAIL_OCR = lastComp;
	}
      } else { // no pit lane pulse
	// we have to set rails on after 50 us
	rail_set();
	lastComp += BIT2_TICKS;
	RAIL_OCR = lastComp;
	nextTelegram -= BIT2_TICKS;
      }
    }
    return;
  }
  if (nextTelegram < 256) {
    // prepare next rail telegram on next isr
    lastComp += nextTelegram;	// nextTelegram must < 256
    RAIL_OCR = lastComp;
    nextTelegram = TELEGRAM_TICKS;
    if (rail_len) {
      rail_clr();
      RailPos = 1L << (rail_len-1);
      RailDat = rail_out | RailPos;
      rail_len = 0;		// say user data is taken
    }
    state_m = ISR_NXT_DONE;
    return;
  }
  rail_set() ; //rail_hld();
  // no communication (rail,ir), see what to do now
  switch(state_m) {
  case ISR_NXT_ETGM:
    rail_on();
    lastComp += BIT2_TICKS;    // 50us
    RAIL_OCR = lastComp;
    nextTelegram -= BIT2_TICKS; // 50us
    state_m = ISR_NXT_DONE;
    break;
  case ISR_NXT_IR30p:
#ifdef IRout_TIMER_NR
    cbi(IRout_TIMSK, IRout_TOIE); // stop system timer isr
    IRout_TCNT = 1; // OCR match in xx ticks
    IRout_OCR = IRpulse_TICKS-1;
    // setup timer0 for 38kHz in CTC mode, toggle on match 
    IRout_TCCR = (IRout_TCCR | _BV(IRout_WGM1) | _BV(IRout_COM0)) & ~((uint8_t)(_BV(IRout_WGM0)|_BV(IRout_COM1)));
    // wait 30pulses
    last = (2 * 30) * IRpulse_TICKS - (IRpulse_TICKS>>1);
    lastComp += last;
    RAIL_OCR = lastComp;
    nextTelegram -= last;	// next interrupt
    state_m = ISR_NXT_PS30p;
    break;
  case ISR_NXT_PS30p:
    // set output on match
    IRout_TCCR = (IRout_TCCR | _BV(IRout_COM1)) & ~((uint8_t)_BV(IRout_COM0));
    // fire system timer isr every 100us = 8 timers for correct missing ones 
    IRout_OCR = 81;
    IRout_TCNT = IRout_OCR - 10;
    sbi(IRout_TIMSK, IRout_TOIE);
    // wait 30pulses
    last = 2 * 30 * IRpulse_TICKS;
    lastComp += last;
    RAIL_OCR = lastComp;
    nextTelegram -= last;	// next interrupt
    state_m = ISR_NXT_IR20p;
    break;
  case ISR_NXT_IR20p:
    cbi(IRout_TIMSK, IRout_TOIE); // stop system timer isr
    IRout_TCNT = 1; // OCR match in xx ticks
    IRout_OCR = IRpulse_TICKS-1;
    // setup timer0 for 38kHz in CTC mode, toggle on match 
    IRout_TCCR = (IRout_TCCR | _BV(IRout_WGM1) | _BV(IRout_COM0)) & ~(
		 (uint8_t)(_BV(IRout_WGM0)|_BV(IRout_COM1)));
    // wait 20pulses
    last = (2 * 20) * IRpulse_TICKS;
    lastComp += last;
    RAIL_OCR = lastComp;
    nextTelegram -= last;
    state_m = ISR_NXT_IRoff;
    break;
  case ISR_NXT_IRclr:
    // set output on match
    IRout_TCCR = (IRout_TCCR | _BV(IRout_COM1)) & ~((uint8_t)_BV(IRout_COM0));
    lastComp += 50;
    RAIL_OCR = lastComp;
    nextTelegram -= 50;
    state_m = ISR_NXT_IRoff;
    break;
  case ISR_NXT_IRoff:
    // set timer to normal mode, output disconnected
    IRout_TCCR = IRout_TCCR & ~((uint8_t)(_BV(IRout_COM0)|_BV(IRout_COM1)
					  |_BV(IRout_WGM0)|_BV(IRout_WGM1)));
    IRout_TCNT = 256-46;	// next overflow isr in 46us
    sbi(IRout_TIMSK, IRout_TOIE);
    lastComp += 50;
    RAIL_OCR = lastComp;
    nextTelegram -= 50;
    state_m = ISR_NXT_DONE;
    break;
#else
    ir_pulse = 30*2;
    state_m = ISR_NXT_PS30p;
    lastComp += 50;		// isr_next() must finish before 50 ticks
    RAIL_OCR = lastComp;
    nextTelegram -= 50+30*2*IRpulse_TICKS;
    break;
  case ISR_NXT_PS30p:
    IRout_PORT &= ~IRout_MASK;
    lastComp += 20*IRpulse_TICKS;
    RAIL_OCR = lastComp;
    nextTelegram -= 20*IRpulse_TICKS;
    state_m = ISR_NXT_PS20p;
    break;
  case ISR_NXT_IR20p:
    ir_pulse = 20*2;
    state_m = ISR_NXT_IRoff;
    lastComp += 50;
    RAIL_OCR = lastComp;
    nextTelegram -= 50+40*IRpulse_TICKS;	// next interrupt
    break;
  case ISR_NXT_IRoff:
    IRout_PORT &= ~IRout_MASK;
    lastComp += 50;
    RAIL_OCR = lastComp;
    nextTelegram -= 50;
    state_m = ISR_NXT_DONE;
    break;
#endif /* IRout_TIMER_NR */
  case ISR_NXT_PS20p:
    lastComp += 20*IRpulse_TICKS;
    RAIL_OCR = lastComp;
    nextTelegram -= 20*IRpulse_TICKS;
    state_m = ISR_NXT_PS10p;
    break;
  case ISR_NXT_PS10p:
    lastComp += 20*IRpulse_TICKS;
    RAIL_OCR = lastComp;
    nextTelegram -= 20*IRpulse_TICKS;
    state_m = ISR_NXT_IR20p;
    break;
  case ISR_NXT_100us:
    lastComp += 100;		// isr_next() must finish before 100 ticks
    RAIL_OCR = lastComp;
    nextTelegram -= 100;	// next interrupt
    state_m = isr_next(nextTelegram, RailDat);
    RailDat = 0;
    break;
  case ISR_NXT_50us:
    lastComp += 50;		// isr_next() must finish before 50 ticks
    RAIL_OCR = lastComp;
    nextTelegram -= 50;	// next interrupt
    state_m = isr_next(nextTelegram, RailDat);
    RailDat = 0;
    break;
  case ISR_NXT_NONE:		
    // wait for next telegram
    if (nextTelegram > 500) {
      lastComp += 256;
      RAIL_OCR = lastComp;
      nextTelegram -= 256;	// next interrupt
    } else {
      // 1..2 timer cycle
      lastComp += nextTelegram >> 1;
      RAIL_OCR = lastComp;
      nextTelegram -= nextTelegram >> 1;	// next interrupt
    }
    break;
  default: //case ISR_NXT_256us:
    if (nextTelegram > 300) {
      lastComp += 256;
      RAIL_OCR = lastComp;
      nextTelegram -= 256;	// next interrupt
      state_m = isr_next(nextTelegram, RailDat);
      RailDat = 0;
    } else {
      lastComp += 200;
      RAIL_OCR = lastComp;
      nextTelegram -= 200;	// next interrupt will prepare telegram
    }
    break;
  }
}

#ifdef RAIL_uSEC_vect
ISR(RAIL_uSEC_vect)
{
  rail_timer_cnt += 1;
}

unsigned long rail_micros()
{
  unsigned long mu;
  uint16_t ti; 
  uint8_t oldSREG = SREG;

  cli();
  ti = RAIL_TIMER_TCNT;
  mu = rail_timer_cnt;
  SREG = oldSREG;
  mu <<= 16;
  mu |= ti;
#ifdef RAIL_TIMER_US_PER_TICKS
  return mu * RAIL_TIMER_US_PER_TICKS;
#else
  return mu / RAIL_TIMER_TICKS_PER_US;
#endif
}

#define mu_sec()        rail_micros()

#endif /* RAIL_uSEC_vect */
