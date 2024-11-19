// RH_ASK.cpp
//
// Copyright (C) 2014 Mike McCauley
// $Id: RH_ASK.cpp,v 1.32 2020/08/04 09:02:14 mikem Exp $

#include "RH_ASK.h"
#include "RHCRC.h"

#define RH_PLATFORM_RAK4630 46

#ifndef __SAMD51__

#if (RH_PLATFORM == RH_PLATFORM_STM32)
    // Maple etc
  HardwareTimer timer(MAPLE_TIMER);
#elif defined(ARDUINO_ARCH_RP2040)

#elif defined(BOARD_NAME)
  // ST's Arduino Core STM32, https://github.com/stm32duino/Arduino_Core_STM32
 #if defined(RH_HW_TIMER)
 // Can define your own timer name based on macros defs passed to compiler eg in platformio.ini
  HardwareTimer timer(RH_HW_TIMER);
 #else
  HardwareTimer timer(TIM1);
 #endif
    
#elif defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_STM32F1) || defined(ARDUINO_ARCH_STM32F3) || defined(ARDUINO_ARCH_STM32F4)
  // Roger Clark Arduino STM32, https://github.com/rogerclarkmelbourne/Arduino_STM32
  // And stm32duino    
  HardwareTimer timer(1);
#endif

#if (RH_PLATFORM == RH_PLATFORM_ESP32)
  // Michael Cain
  DRAM_ATTR hw_timer_t * timer;
  //jPerotto Non-constant static data from ESP32 https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/general-notes.html#dram-data-ram
   #define RH_DRAM_ATTR DRAM_ATTR
#else
   #define RH_DRAM_ATTR 
#endif

// RH_ASK on Arduino uses Timer 1 to generate interrupts 8 times per bit interval
// Define RH_ASK_ARDUINO_USE_TIMER2 if you want to use Timer 2 instead of Timer 1 on Arduino
// You may need this to work around other libraries that insist on using timer 1
// Should be moved to header file
//#define RH_ASK_ARDUINO_USE_TIMER2

// RH_ASK on ATtiny8x uses Timer 0 to generate interrupts 8 times per bit interval. 
// Timer 0 is used by Arduino platform for millis()/micros() which is used by delay()
// Uncomment the define RH_ASK_ATTINY_USE_TIMER1 bellow, if you want to use Timer 1 instead of Timer 0 on ATtiny
// Timer 1 is also used by some other libraries, e.g. Servo. Alway check usage of Timer 1 before enabling this.
//  Should be moved to header file
//#define RH_ASK_ATTINY_USE_TIMER1

// Interrupt handler uses this to find the most recently initialised instance of this driver
static RH_ASK* thisASKDriver;

// 4 bit to 6 bit symbol converter table
// Used to convert the high and low nybbles of the transmitted data
// into 6 bit symbols for transmission. Each 6-bit symbol has 3 1s and 3 0s 
// with at most 3 consecutive identical bits
RH_DRAM_ATTR static uint8_t symbols[] =
{
    0xd,  0xe,  0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c, 
    0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

// This is the value of the start symbol after 6-bit conversion and nybble swapping
#define RH_ASK_START_SYMBOL 0xb38

RH_ASK::RH_ASK(uint16_t speed, uint8_t rxPin, uint8_t txPin, uint8_t pttPin, bool pttInverted)
    :
    _speed(speed),
    _rxPin(rxPin),
    _txPin(txPin),
    _pttPin(pttPin),
    _rxInverted(false),
    _pttInverted(pttInverted)
{
    // Initialise the first 8 nibbles of the tx buffer to be the standard
    // preamble. We will append messages after that. 0x38, 0x2c is the start symbol before
    // 6-bit conversion to RH_ASK_START_SYMBOL
    uint8_t preamble[RH_ASK_PREAMBLE_LEN] = {0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x38, 0x2c};
    memcpy(_txBuf, preamble, sizeof(preamble));
}

bool RH_ASK::init()
{
    if (!RHGenericDriver::init())
	return false;
    thisASKDriver = this;

#if (RH_PLATFORM == RH_PLATFORM_GENERIC_AVR8)
 #ifdef RH_ASK_PTT_PIN 				
    RH_ASK_PTT_DDR  |=  (1<<RH_ASK_PTT_PIN); 
    RH_ASK_TX_DDR   |=  (1<<RH_ASK_TX_PIN);
    RH_ASK_RX_DDR   &= ~(1<<RH_ASK_RX_PIN);
 #else
    RH_ASK_TX_DDR   |=  (1<<RH_ASK_TX_PIN);
    RH_ASK_RX_DDR   &= ~(1<<RH_ASK_RX_PIN);
 #endif
#else
    // Set up digital IO pins for arduino
    pinMode(_txPin, OUTPUT);
    pinMode(_rxPin, INPUT);
    pinMode(_pttPin, OUTPUT);
#endif

    // Ready to go
    setModeIdle();
    timerSetup();

    return true;
}


// The idea here is to get 8 timer interrupts per bit period
void RH_ASK::timerSetup()
{
    #if (RH_PLATFORM == RH_PLATFORM_RAK4630)
        // Set up Timer 1 for nRF52840 on RAK4631
        NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Timer Mode
        NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit; // 16-bit mode

        // Calculate the required timer frequency
        uint32_t timer_freq = (16000000 / 8) / _speed; // Assuming 16 MHz HFCLK

        NRF_TIMER2->PRESCALER = 0;  // Prescaler, 0 for 1:1
        NRF_TIMER2->CC[0] = timer_freq;

        // Enable interrupt for COMPARE[0] event
        NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;

        // Clear event
        NRF_TIMER2->EVENTS_COMPARE[0] = 0;

        // Enable IRQ in the NVIC
        NVIC_SetPriority(TIMER1_IRQn, 3);
        NVIC_EnableIRQ(TIMER1_IRQn);

        // Start Timer
        NRF_TIMER2->TASKS_START = 1;
    #else
        exit(0); //Undefined!

  #endif

}

void RH_INTERRUPT_ATTR RH_ASK::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	// Disable the transmitter hardware
	writePtt(LOW);
	writeTx(LOW);
	_mode = RHModeIdle;
    }
}

void RH_INTERRUPT_ATTR RH_ASK::setModeRx()
{
    if (_mode != RHModeRx)
    {
	// Disable the transmitter hardware
	writePtt(LOW);
	writeTx(LOW);
	_mode = RHModeRx;
    }
}

void RH_ASK::setModeTx()
{
    if (_mode != RHModeTx)
    {
	// PRepare state varibles for a new transmission
	_txIndex = 0;
	_txBit = 0;
	_txSample = 0;

	// Enable the transmitter hardware
	writePtt(HIGH);

	_mode = RHModeTx;
    }
}

// Call this often
bool RH_ASK::available()
{
    if (_mode == RHModeTx)
	return false;
    setModeRx();
    if (_rxBufFull)
    {
	validateRxBuf();
	_rxBufFull= false;
    }
    return _rxBufValid;
}

bool RH_INTERRUPT_ATTR RH_ASK::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;

    if (buf && len)
    {
	// Skip the length and 4 headers that are at the beginning of the rxBuf
	// and drop the trailing 2 bytes of FCS
	uint8_t message_len = _rxBufLen-RH_ASK_HEADER_LEN - 3;
	if (*len > message_len)
	    *len = message_len;
	memcpy(buf, _rxBuf+RH_ASK_HEADER_LEN+1, *len);
    }
    _rxBufValid = false; // Got the most recent message, delete it
//    printBuffer("recv:", buf, *len);
    return true;
}

// Caution: this may block
bool RH_ASK::send(const uint8_t* data, uint8_t len)
{
    uint8_t i;
    uint16_t index = 0;
    uint16_t crc = 0xffff;
    uint8_t *p = _txBuf + RH_ASK_PREAMBLE_LEN; // start of the message area
    uint8_t count = len + 3 + RH_ASK_HEADER_LEN; // Added byte count and FCS and headers to get total number of bytes

    if (len > RH_ASK_MAX_MESSAGE_LEN)
	return false;

    // Wait for transmitter to become available
    waitPacketSent();

    if (!waitCAD()) 
	return false;  // Check channel activity

    // Encode the message length
    crc = RHcrc_ccitt_update(crc, count);
    p[index++] = symbols[count >> 4];
    p[index++] = symbols[count & 0xf];

    // Encode the headers
    crc = RHcrc_ccitt_update(crc, _txHeaderTo);
    p[index++] = symbols[_txHeaderTo >> 4];
    p[index++] = symbols[_txHeaderTo & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderFrom);
    p[index++] = symbols[_txHeaderFrom >> 4];
    p[index++] = symbols[_txHeaderFrom & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderId);
    p[index++] = symbols[_txHeaderId >> 4];
    p[index++] = symbols[_txHeaderId & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderFlags);
    p[index++] = symbols[_txHeaderFlags >> 4];
    p[index++] = symbols[_txHeaderFlags & 0xf];

    // Encode the message into 6 bit symbols. Each byte is converted into 
    // 2 6-bit symbols, high nybble first, low nybble second
    for (i = 0; i < len; i++)
    {
	crc = RHcrc_ccitt_update(crc, data[i]);
	p[index++] = symbols[data[i] >> 4];
	p[index++] = symbols[data[i] & 0xf];
    }

    // Append the fcs, 16 bits before encoding (4 6-bit symbols after encoding)
    // Caution: VW expects the _ones_complement_ of the CCITT CRC-16 as the FCS
    // VW sends FCS as low byte then hi byte
    crc = ~crc;
    p[index++] = symbols[(crc >> 4)  & 0xf];
    p[index++] = symbols[crc & 0xf];
    p[index++] = symbols[(crc >> 12) & 0xf];
    p[index++] = symbols[(crc >> 8)  & 0xf];

    // Total number of 6-bit symbols to send
    _txBufLen = index + RH_ASK_PREAMBLE_LEN;

    // Start the low level interrupt handler sending symbols
    setModeTx();

    return true;
}

// Read the RX data input pin, taking into account platform type and inversion.
bool RH_INTERRUPT_ATTR RH_ASK::readRx()
{
    bool value;
#if (RH_PLATFORM == RH_PLATFORM_GENERIC_AVR8)
    value = ((RH_ASK_RX_PORT & (1<<RH_ASK_RX_PIN)) ? 1 : 0);
#else
    value = digitalRead(_rxPin);
#endif
    return value ^ _rxInverted;
}

// Write the TX output pin, taking into account platform type.
void RH_INTERRUPT_ATTR RH_ASK::writeTx(bool value)
{
    digitalWrite(_txPin, value);
}

// Write the PTT output pin, taking into account platform type and inversion.
void RH_INTERRUPT_ATTR RH_ASK::writePtt(bool value)
{
    digitalWrite(_pttPin, value ^ _pttInverted);
}

uint8_t RH_ASK::maxMessageLength()
{
    return RH_ASK_MAX_MESSAGE_LEN;
}

#if (RH_PLATFORM == RH_PLATFORM_RAK4630)
extern "C" void TIMER1_IRQHandler(void) 
{
    // Check if the timer compare event occurred
    if (NRF_TIMER2->EVENTS_COMPARE[0])
    {
        // Clear the event
        NRF_TIMER2->EVENTS_COMPARE[0] = 0;
        NRF_TIMER2->TASKS_CLEAR = 1;

        // Call the interrupt handling function in the driver
        thisASKDriver->handleTimerInterrupt();
    }
}

#elif (RH_PLATFORM == RH_PLATFORM_ESP32)
void IRAM_ATTR esp32_timer_interrupt_handler()
{
    thisASKDriver->handleTimerInterrupt();
}

#endif

// Convert a 6 bit encoded symbol into its 4 bit decoded equivalent
uint8_t RH_INTERRUPT_ATTR RH_ASK::symbol_6to4(uint8_t symbol)
{
    uint8_t i;
    uint8_t count;
    
    // Linear search :-( Could have a 64 byte reverse lookup table?
    // There is a little speedup here courtesy Ralph Doncaster:
    // The shortcut works because bit 5 of the symbol is 1 for the last 8
    // symbols, and it is 0 for the first 8.
    // So we only have to search half the table
    for (i = (symbol>>2) & 8, count=8; count-- ; i++)
	if (symbol == symbols[i]) return i;

    return 0; // Not found
}

// Check whether the latest received message is complete and uncorrupted
// We should always check the FCS at user level, not interrupt level
// since it is slow
void RH_ASK::validateRxBuf()
{
    uint16_t crc = 0xffff;
    // The CRC covers the byte count, headers and user data
    for (uint8_t i = 0; i < _rxBufLen; i++)
	crc = RHcrc_ccitt_update(crc, _rxBuf[i]);
    if (crc != 0xf0b8) // CRC when buffer and expected CRC are CRC'd
    {
	// Reject and drop the message
	_rxBad++;
	_rxBufValid = false;
	return;
    }

    // Extract the 4 headers that follow the message length
    _rxHeaderTo    = _rxBuf[1];
    _rxHeaderFrom  = _rxBuf[2];
    _rxHeaderId    = _rxBuf[3];
    _rxHeaderFlags = _rxBuf[4];
    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
	_rxGood++;
	_rxBufValid = true;
    }
}

void RH_INTERRUPT_ATTR RH_ASK::receiveTimer()
{
    bool rxSample = readRx();

    // Integrate each sample
    if (rxSample)
	_rxIntegrator++;

    if (rxSample != _rxLastSample)
    {
	// Transition, advance if ramp > 80, retard if < 80
	_rxPllRamp += ((_rxPllRamp < RH_ASK_RAMP_TRANSITION) 
			   ? RH_ASK_RAMP_INC_RETARD 
			   : RH_ASK_RAMP_INC_ADVANCE);
	_rxLastSample = rxSample;
    }
    else
    {
	// No transition
	// Advance ramp by standard 20 (== 160/8 samples)
	_rxPllRamp += RH_ASK_RAMP_INC;
    }
    if (_rxPllRamp >= RH_ASK_RX_RAMP_LEN)
    {
	// Add this to the 12th bit of _rxBits, LSB first
	// The last 12 bits are kept
	_rxBits >>= 1;

	// Check the integrator to see how many samples in this cycle were high.
	// If < 5 out of 8, then its declared a 0 bit, else a 1;
	if (_rxIntegrator >= 5)
	    _rxBits |= 0x800;

	_rxPllRamp -= RH_ASK_RX_RAMP_LEN;
	_rxIntegrator = 0; // Clear the integral for the next cycle

	if (_rxActive)
	{
	    // We have the start symbol and now we are collecting message bits,
	    // 6 per symbol, each which has to be decoded to 4 bits
	    if (++_rxBitCount >= 12)
	    {
		// Have 12 bits of encoded message == 1 byte encoded
		// Decode as 2 lots of 6 bits into 2 lots of 4 bits
		// The 6 lsbits are the high nybble
		uint8_t this_byte = 
		    (symbol_6to4(_rxBits & 0x3f)) << 4 
		    | symbol_6to4(_rxBits >> 6);

		// The first decoded byte is the byte count of the following message
		// the count includes the byte count and the 2 trailing FCS bytes
		// REVISIT: may also include the ACK flag at 0x40
		if (_rxBufLen == 0)
		{
		    // The first byte is the byte count
		    // Check it for sensibility. It cant be less than 7, since it
		    // includes the byte count itself, the 4 byte header and the 2 byte FCS
		    _rxCount = this_byte;
		    if (_rxCount < 7 || _rxCount > RH_ASK_MAX_PAYLOAD_LEN)
		    {
			// Stupid message length, drop the whole thing
			_rxActive = false;
			_rxBad++;
                        return;
		    }
		}
		_rxBuf[_rxBufLen++] = this_byte;

		if (_rxBufLen >= _rxCount)
		{
		    // Got all the bytes now
		    _rxActive = false;
		    _rxBufFull = true;
		    setModeIdle();
		}
		_rxBitCount = 0;
	    }
	}
	// Not in a message, see if we have a start symbol
	else if (_rxBits == RH_ASK_START_SYMBOL)
	{
	    // Have start symbol, start collecting message
	    _rxActive = true;
	    _rxBitCount = 0;
	    _rxBufLen = 0;
	}
    }
}

void RH_INTERRUPT_ATTR RH_ASK::transmitTimer()
{
    if (_txSample++ == 0)
    {
	// Send next bit
	// Symbols are sent LSB first
	// Finished sending the whole message? (after waiting one bit period 
	// since the last bit)
	if (_txIndex >= _txBufLen)
	{
	    setModeIdle();
	    _txGood++;
	}
	else
	{
	    writeTx(_txBuf[_txIndex] & (1 << _txBit++));
	    if (_txBit >= 6)
	    {
		_txBit = 0;
		_txIndex++;
	    }
	}
    }
	
    if (_txSample > 7)
	_txSample = 0;
}

void RH_INTERRUPT_ATTR RH_ASK::handleTimerInterrupt()
{
    if (_mode == RHModeRx)
	receiveTimer(); // Receiving
    else if (_mode == RHModeTx)
        transmitTimer(); // Transmitting
}

#endif //_SAMD51__
