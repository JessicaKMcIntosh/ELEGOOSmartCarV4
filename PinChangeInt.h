#define PCINT_VERSION 2402
#define	detachPinChangeInterrupt(pin)				PCintPort::detachInterrupt(pin)
#define	attachPinChangeInterrupt(pin,userFunc,mode)	PCintPort::attachInterrupt(pin, &userFunc,mode)
#define getInterruptedPin()							PCintPort::getArduinoPin()
#ifndef PinChangeInt_h
#define	PinChangeInt_h
#include "stddef.h"
#include <Arduino.h>
#include <new.h>
#include <wiring_private.h>
#undef DEBUG
#undef	INLINE_PCINT
#define INLINE_PCINT
#if defined __AVR_ATmega2560__ || defined __AVR_ATmega1280__ || defined __AVR_ATmega1281__ || defined __AVR_ATmega2561__ || defined __AVR_ATmega640__
	#define __USE_PORT_JK
	#define NO_PORTA_PINCHANGES
	#define NO_PORTC_PINCHANGES
	#define NO_PORTD_PINCHANGES
	#if ((defined(NO_PORTB_PINCHANGES) && defined(NO_PORTJ_PINCHANGES)) || \
			(defined(NO_PORTJ_PINCHANGES) && defined(NO_PORTK_PINCHANGES)) || \
			(defined(NO_PORTK_PINCHANGES) && defined(NO_PORTB_PINCHANGES)))
		#define	INLINE_PCINT inline
	#endif
#else
	#define NO_PORTJ_PINCHANGES
	#define NO_PORTK_PINCHANGES
	#if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
		#ifndef NO_PORTA_PINCHANGES
			#define __USE_PORT_A
		#endif
	#else
		#define NO_PORTA_PINCHANGES
	#endif
	#if (   (defined(NO_PORTA_PINCHANGES) && defined(NO_PORTB_PINCHANGES) && defined(NO_PORTC_PINCHANGES)) || \
			(defined(NO_PORTA_PINCHANGES) && defined(NO_PORTB_PINCHANGES) && defined(NO_PORTD_PINCHANGES)) || \
			(defined(NO_PORTA_PINCHANGES) && defined(NO_PORTC_PINCHANGES) && defined(NO_PORTD_PINCHANGES)) || \
			(defined(NO_PORTB_PINCHANGES) && defined(NO_PORTC_PINCHANGES) && defined(NO_PORTD_PINCHANGES)) )
		#define	INLINE_PCINT inline
	#endif
#endif
#define	PCdetachInterrupt(pin)	PCintPort::detachInterrupt(pin)
#define	PCattachInterrupt(pin,userFunc,mode) PCintPort::attachInterrupt(pin, userFunc,mode)
#define PCgetArduinoPin() PCintPort::getArduinoPin()
typedef void (*PCIntvoidFuncPtr)(void);
class PCintPort {
public:
	PCintPort(int index,int pcindex, volatile uint8_t& maskReg) :
	portInputReg(*portInputRegister(index)),
	portPCMask(maskReg),
	PCICRbit(1 << pcindex),
	portRisingPins(0),
	portFallingPins(0),
	firstPin(NULL)
#ifdef PINMODE
	,intrCount(0)
#endif
	{
		#ifdef FLASH
		ledsetup();
		#endif
	}
	volatile	uint8_t&		portInputReg;
	static		int8_t attachInterrupt(uint8_t pin, PCIntvoidFuncPtr userFunc, int mode);
	static		void detachInterrupt(uint8_t pin);
	INLINE_PCINT void PCint();
	static volatile uint8_t curr;
	#ifndef NO_PIN_NUMBER
	static	volatile uint8_t	arduinoPin;
	#endif
	#ifndef NO_PIN_STATE
	static volatile	uint8_t	pinState;
	#endif
	#ifdef PINMODE
	static volatile uint8_t pinmode;
	static volatile uint8_t s_portRisingPins;
	static volatile uint8_t s_portFallingPins;
	static volatile uint8_t s_lastPinView;
	static volatile uint8_t s_pmask;
	static volatile char s_PORT;
	static volatile uint8_t s_changedPins;
	static volatile uint8_t s_portRisingPins_nCurr;
	static volatile uint8_t s_portFallingPins_nNCurr;
	static volatile uint8_t s_currXORlastPinView;
	volatile uint8_t intrCount;
	static volatile uint8_t s_count;
	static volatile uint8_t pcint_multi;
	static volatile uint8_t PCIFRbug;
	#endif
	#ifdef FLASH
	static void ledsetup(void);
	#endif
protected:
	class PCintPin {
	public:
		PCintPin() :
		PCintFunc((PCIntvoidFuncPtr)NULL),
		mode(0) {}
		PCIntvoidFuncPtr PCintFunc;
		uint8_t 	mode;
		uint8_t		mask;
		uint8_t arduinoPin;
		PCintPin* next;
	};
	void 		enable(PCintPin* pin, PCIntvoidFuncPtr userFunc, uint8_t mode);
	int8_t		addPin(uint8_t arduinoPin,PCIntvoidFuncPtr userFunc, uint8_t mode);
	volatile	uint8_t&		portPCMask;
	const		uint8_t			PCICRbit;
	volatile	uint8_t			portRisingPins;
	volatile	uint8_t			portFallingPins;
	volatile uint8_t		lastPinView;
	PCintPin*	firstPin;
};
#ifndef LIBCALL_PINCHANGEINT
volatile uint8_t PCintPort::curr=0;
#ifndef NO_PIN_NUMBER
volatile uint8_t PCintPort::arduinoPin=0;
#endif
#ifndef NO_PIN_STATE
volatile uint8_t PCintPort::pinState=0;
#endif
#ifdef PINMODE
volatile uint8_t PCintPort::pinmode=0;
volatile uint8_t PCintPort::s_portRisingPins=0;
volatile uint8_t PCintPort::s_portFallingPins=0;
volatile uint8_t PCintPort::s_lastPinView=0;
volatile uint8_t PCintPort::s_pmask=0;
volatile char	 PCintPort::s_PORT='x';
volatile uint8_t PCintPort::s_changedPins=0;
volatile uint8_t PCintPort::s_portRisingPins_nCurr=0;
volatile uint8_t PCintPort::s_portFallingPins_nNCurr=0;
volatile uint8_t PCintPort::s_currXORlastPinView=0;
volatile uint8_t PCintPort::s_count=0;
volatile uint8_t PCintPort::pcint_multi=0;
volatile uint8_t PCintPort::PCIFRbug=0;
#endif
#ifdef FLASH
#define PINLED 13
volatile uint8_t *led_port;
uint8_t led_mask;
uint8_t not_led_mask;
boolean ledsetup_run=false;
void PCintPort::ledsetup(void) {
	if (! ledsetup_run) {
		led_port=portOutputRegister(digitalPinToPort(PINLED));
		led_mask=digitalPinToBitMask(PINLED);
		not_led_mask=led_mask^0xFF;
		pinMode(PINLED, OUTPUT); digitalWrite(PINLED, LOW);
		ledsetup_run=true;
	}
};
#endif
#if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) 
#ifndef NO_PORTA_PINCHANGES
PCintPort portA=PCintPort(1, 0,PCMSK0);
#endif
#ifndef NO_PORTB_PINCHANGES
PCintPort portB=PCintPort(2, 1,PCMSK1);
#endif
#ifndef NO_PORTC_PINCHANGES
PCintPort portC=PCintPort(3, 2,PCMSK2);
#endif
#ifndef NO_PORTD_PINCHANGES
PCintPort portD=PCintPort(4, 3,PCMSK3);
#endif
#else
#ifndef NO_PORTB_PINCHANGES
PCintPort portB=PCintPort(2, 0,PCMSK0);
#endif
#ifndef NO_PORTC_PINCHANGES
PCintPort portC=PCintPort(3, 1,PCMSK1);
#endif
#ifndef NO_PORTD_PINCHANGES
PCintPort portD=PCintPort(4, 2,PCMSK2);
#endif
#endif
#ifdef __USE_PORT_JK
#ifndef NO_PORTJ_PINCHANGES
PCintPort portJ=PCintPort(10,1,PCMSK1);
#endif
#ifndef NO_PORTK_PINCHANGES
PCintPort portK=PCintPort(11,2,PCMSK2);
#endif
#endif
static PCintPort *lookupPortNumToPort( int portNum ) {
    PCintPort *port = NULL;
	switch (portNum) {
#ifndef NO_PORTA_PINCHANGES
	case 1:
		port=&portA;
		break;
#endif
#ifndef NO_PORTB_PINCHANGES
	case 2:
		port=&portB;
		break;
#endif
#ifndef NO_PORTC_PINCHANGES
	case 3:
		port=&portC;
		break;
#endif
#ifndef NO_PORTD_PINCHANGES
	case 4:
		port=&portD;
		break;
#endif
#ifdef __USE_PORT_JK
#ifndef NO_PORTJ_PINCHANGES
	case 10:
		port=&portJ;
		break;
#endif
#ifndef NO_PORTK_PINCHANGES
	case 11:
		port=&portK;
		break;
#endif
#endif
    }
    return port;
}
void PCintPort::enable(PCintPin* p, PCIntvoidFuncPtr userFunc, uint8_t mode) {
	p->mode=mode;
	p->PCintFunc=userFunc;
#ifndef NO_PORTJ_PINCHANGES
	if ((p->arduinoPin == 14) || (p->arduinoPin == 15)) {
		portPCMask |= (p->mask << 1);
	}
	else {
		portPCMask |= p->mask;
	}
#else
    portPCMask |= p->mask;
#endif
	if ((p->mode == RISING) || (p->mode == CHANGE)) portRisingPins |= p->mask;
	if ((p->mode == FALLING) || (p->mode == CHANGE)) portFallingPins |= p->mask;
	PCICR |= PCICRbit;
}
int8_t PCintPort::addPin(uint8_t arduinoPin, PCIntvoidFuncPtr userFunc, uint8_t mode)
{
	PCintPin* tmp;
	tmp=firstPin;
	if (firstPin != NULL) {
		do {
			if (tmp->arduinoPin == arduinoPin) { enable(tmp, userFunc, mode); return(0); }
			if (tmp->next == NULL) break;
			tmp=tmp->next;
		} while (true);
	}
	PCintPin* p=new PCintPin;
	if (p == NULL) return(-1);
	p->arduinoPin=arduinoPin;
	p->mode = mode;
	p->next=NULL;
	p->mask = digitalPinToBitMask(arduinoPin);
	if (firstPin == NULL) firstPin=p;
	else tmp->next=p;
#ifdef DEBUG
	Serial.print("addPin. pin given: "); Serial.print(arduinoPin, DEC);
	int addr = (int) p;
	Serial.print(" instance addr: "); Serial.println(addr, HEX);
	Serial.print("userFunc addr: "); Serial.println((int)p->PCintFunc, HEX);
#endif
	enable(p, userFunc, mode);
#ifdef DEBUG
	Serial.print("addPin. pin given: "); Serial.print(arduinoPin, DEC), Serial.print (" pin stored: ");
	int addr = (int) p;
	Serial.print(" instance addr: "); Serial.println(addr, HEX);
#endif
	return(1);
}
int8_t PCintPort::attachInterrupt(uint8_t arduinoPin, PCIntvoidFuncPtr userFunc, int mode)
{
	PCintPort *port;
	uint8_t portNum = digitalPinToPort(arduinoPin);
	if ((portNum == NOT_A_PORT) || (userFunc == NULL)) return(-1);
	port=lookupPortNumToPort(portNum);
	port->lastPinView=port->portInputReg;
#ifdef DEBUG
	Serial.print("attachInterrupt- pin: "); Serial.println(arduinoPin, DEC);
#endif
	return(port->addPin(arduinoPin,userFunc,mode));
}
void PCintPort::detachInterrupt(uint8_t arduinoPin)
{
	PCintPort *port;
	PCintPin* current;
	uint8_t mask;
	uint8_t portNum = digitalPinToPort(arduinoPin);
	if (portNum == NOT_A_PORT) return;
	port=lookupPortNumToPort(portNum);
	mask=digitalPinToBitMask(arduinoPin);
	current=port->firstPin;
	while (current) {
		if (current->mask == mask) {
			uint8_t oldSREG = SREG;
			cli();
#ifndef NO_PORTJ_PINCHANGES
			if ((arduinoPin == 14) || (arduinoPin == 15)) {
				port->portPCMask &= ~(mask << 1);
			}
			else {
				port->portPCMask &= ~mask;
			}
#else
			port->portPCMask &= ~mask;
#endif
			if (port->portPCMask == 0) PCICR &= ~(port->PCICRbit);
			port->portRisingPins &= ~current->mask; port->portFallingPins &= ~current->mask;
			SREG = oldSREG; 
			return;
		}
		current=current->next;
	}
}
void PCintPort::PCint() {
	#ifdef FLASH
	if (*led_port & led_mask) *led_port&=not_led_mask;
	else *led_port|=led_mask;
    #endif
	#ifndef DISABLE_PCINT_MULTI_SERVICE
	uint8_t pcifr;
	while (true) {
	#endif
		#ifdef PINMODE
		PCintPort::s_lastPinView=lastPinView;
		intrCount++;
		PCintPort::s_count=intrCount;
		#endif
		uint8_t changedPins = (PCintPort::curr ^ lastPinView) &
							  ((portRisingPins & PCintPort::curr ) | ( portFallingPins & ~PCintPort::curr ));
		#ifdef PINMODE
		PCintPort::s_currXORlastPinView=PCintPort::curr ^ lastPinView;
		PCintPort::s_portRisingPins_nCurr=portRisingPins & PCintPort::curr;
		PCintPort::s_portFallingPins_nNCurr=portFallingPins & ~PCintPort::curr;
		#endif
		lastPinView = PCintPort::curr;
		PCintPin* p = firstPin;
		while (p) {
			if (p->mask & changedPins) {
				#ifndef NO_PIN_STATE
				PCintPort::pinState=PCintPort::curr & p->mask ? HIGH : LOW;
				#endif
				#ifndef NO_PIN_NUMBER
				PCintPort::arduinoPin=p->arduinoPin;
				#endif
				#ifdef PINMODE
				PCintPort::pinmode=p->mode;
				PCintPort::s_portRisingPins=portRisingPins;
				PCintPort::s_portFallingPins=portFallingPins;
				PCintPort::s_pmask=p->mask;
				PCintPort::s_changedPins=changedPins;
				#endif
				p->PCintFunc();
			}
			p=p->next;
		}
	#ifndef DISABLE_PCINT_MULTI_SERVICE
		pcifr = PCIFR & PCICRbit;
		if (pcifr == 0) break;
		PCIFR |= PCICRbit;
		#ifdef PINMODE
		PCintPort::pcint_multi++;
		if (PCIFR & PCICRbit) PCintPort::PCIFRbug=1;
		#endif
		PCintPort::curr=portInputReg;
	}
	#endif
}
#ifndef NO_PORTA_PINCHANGES
ISR(PCINT0_vect) {
	#ifdef PINMODE
	PCintPort::s_PORT='A';
	#endif
	PCintPort::curr = portA.portInputReg;
	portA.PCint();
}
#define PORTBVECT PCINT1_vect
#define PORTCVECT PCINT2_vect
#define PORTDVECT PCINT3_vect
#else
#define PORTBVECT PCINT0_vect
#define PORTCVECT PCINT1_vect
#define PORTDVECT PCINT2_vect
#endif
#ifndef NO_PORTB_PINCHANGES
ISR(PORTBVECT) {
	#ifdef PINMODE
	PCintPort::s_PORT='B';
	#endif
	PCintPort::curr = portB.portInputReg;
	portB.PCint();
}
#endif
#ifndef NO_PORTC_PINCHANGES
ISR(PORTCVECT) {
	#ifdef PINMODE
	PCintPort::s_PORT='C';
	#endif
	PCintPort::curr = portC.portInputReg;
	portC.PCint();
}
#endif
#ifndef NO_PORTD_PINCHANGES
ISR(PORTDVECT){
	#ifdef PINMODE
	PCintPort::s_PORT='D';
	#endif
	PCintPort::curr = portD.portInputReg;
	portD.PCint();
}
#endif
#ifdef __USE_PORT_JK
#ifndef NO_PORTJ_PINCHANGES
ISR(PCINT1_vect) {
	#ifdef PINMODE
	PCintPort::s_PORT='J';
	#endif
	PCintPort::curr = portJ.portInputReg;
	portJ.PCint();
}
#endif
#ifndef NO_PORTK_PINCHANGES
ISR(PCINT2_vect){
	#ifdef PINMODE
	PCintPort::s_PORT='K';
	#endif
	PCintPort::curr = portK.portInputReg;
	portK.PCint();
}
#endif
#endif
#ifdef GET_PCINT_VERSION
uint16_t getPCIntVersion () {
	return ((uint16_t) PCINT_VERSION);
}
#endif
#endif
#endif
