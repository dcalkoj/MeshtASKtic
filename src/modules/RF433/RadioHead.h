// Fork of RadioHead.h to support NRF52 for ASK
// 2DO: Make this into a seperate library.

/*! \mainpage RadioHead Packet Radio library for embedded microprocessors

This is the RadioHead Packet Radio library for embedded microprocessors.
It provides a complete object-oriented library for sending and receiving packetized messages
via a variety of common data radios and other transports on a range of embedded microprocessors.

The version of the package that this documentation refers to can be downloaded 
from http://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.122.zip
You can find the latest version of the documentation at http://www.airspayce.com/mikem/arduino/RadioHead

You can also find online help and discussion at 
http://groups.google.com/group/radiohead-arduino
Please use that group for all questions and discussions on this topic. 
Do not contact the author directly, unless it is to discuss commercial licensing.
Before asking a question or reporting a bug, please read 
- http://en.wikipedia.org/wiki/Wikipedia:Reference_desk/How_to_ask_a_software_question
- http://www.catb.org/esr/faqs/smart-questions.html
- http://www.chiark.greenend.org.uk/~shgtatham/bugs.html
*/

#ifndef RadioHead_h
#define RadioHead_h

// Official version numbers are maintained automatically by Makefile:
#define RH_VERSION_MAJOR 1
#define RH_VERSION_MINOR 122

// Symbolic names for currently supported platform types
#define RH_PLATFORM_ARDUINO          1
#define RH_PLATFORM_MSP430           2
#define RH_PLATFORM_STM32            3
#define RH_PLATFORM_GENERIC_AVR8     4
#define RH_PLATFORM_UNO32            5
#define RH_PLATFORM_UNIX             6
#define RH_PLATFORM_STM32STD         7
#define RH_PLATFORM_STM32F4_HAL      8 
#define RH_PLATFORM_RASPI            9
#define RH_PLATFORM_NRF51            10
#define RH_PLATFORM_ESP8266          11
#define RH_PLATFORM_STM32F2          12
#define RH_PLATFORM_CHIPKIT_CORE     13
#define RH_PLATFORM_ESP32            14						   
#define RH_PLATFORM_NRF52            15
#define RH_PLATFORM_MONGOOSE_OS      16
#define RH_PLATFORM_ATTINY           17
// Spencer Kondes megaTinyCore:						   
#define RH_PLATFORM_ATTINY_MEGA      18
#define RH_PLATFORM_STM32L0          19
#define RH_PLATFORM_RASPI_PICO       20
#define RH_PLATFORM_RAK4631          46

//2DO - Autoselect between platforms here.
#define RH_PLATFORM RH_PLATFORM_RAK4631

						   
////////////////////////////////////////////////////
// Select platform automatically, if possible
#ifndef RH_PLATFORM
 #if (defined(MPIDE) && MPIDE>=150 && defined(ARDUINO))
  // Using ChipKIT Core on Arduino IDE
  #define RH_PLATFORM RH_PLATFORM_CHIPKIT_CORE
 #elif defined(RAK4631)
  #define RH_PLATFORM RH_PLATFORM_RAK4631
 #elif defined(MPIDE)
  // Uno32 under old MPIDE, which has been discontinued:
  #define RH_PLATFORM RH_PLATFORM_UNO32
 #elif defined(NRF51) || defined(NRF52)
  #define RH_PLATFORM RH_PLATFORM_NRF51
 #elif defined(NRF52)
  #define RH_PLATFORM RH_PLATFORM_NRF52
 #elif defined(ESP8266)
  #define RH_PLATFORM RH_PLATFORM_ESP8266
 #elif defined(ESP32)
  #define RH_PLATFORM RH_PLATFORM_ESP32
 #elif defined(STM32L0) || defined(ARDUINO_ARCH_STM32L0)
  #define RH_PLATFORM RH_PLATFORM_STM32L0
 #elif defined(MGOS)
  #define RH_PLATFORM RH_PLATFORM_MONGOOSE_OS
#elif defined(MEGATINYCORE) || defined(ARDUINO_attinyxy2) || defined(ARDUINO_attinyxy4) || defined(ARDUINO_attinyxy6) || defined(ARDUINO_attinyxy7)
  #define RH_PLATFORM RH_PLATFORM_ATTINY_MEGA
 #elif defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtinyX4__) || defined(__AVR_ATtinyX5__) || defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny4313__) || defined(__AVR_ATtinyX313__) || defined(ARDUINO_attiny)
  #define RH_PLATFORM RH_PLATFORM_ATTINY
 #elif defined(ARDUINO)
  #define RH_PLATFORM RH_PLATFORM_ARDUINO
 #elif defined(__MSP430G2452__) || defined(__MSP430G2553__)
  #define RH_PLATFORM RH_PLATFORM_MSP430
 #elif defined(MCU_STM32F103RE)
  #define RH_PLATFORM RH_PLATFORM_STM32
 #elif defined(STM32F2XX)
  #define RH_PLATFORM RH_PLATFORM_STM32F2
 #elif defined(USE_STDPERIPH_DRIVER)
  #define RH_PLATFORM RH_PLATFORM_STM32STD
 #elif defined(RASPBERRY_PI)
  #define RH_PLATFORM RH_PLATFORM_RASPI
 #elif defined(__unix__) // Linux
  #define RH_PLATFORM RH_PLATFORM_UNIX
 #elif defined(__APPLE__) // OSX
  #define RH_PLATFORM RH_PLATFORM_UNIX
						   
 #else
  #error Platform not defined! 	
 #endif
#endif
						   
////////////////////////////////////////////////////
// Platform specific headers:
#if (RH_PLATFORM == RH_PLATFORM_ESP32)   // ESP32 processor on Arduino IDE
 #include <Arduino.h>
 #include <SPI.h>
 #define RH_HAVE_HARDWARE_SPI
 #define RH_HAVE_SERIAL
 #define RH_MISSING_SPIUSINGINTERRUPT
 // ESP32 has 2 user SPI buses: VSPI and HSPI. They are essentially identical, but use different pins.
 // The usual, default bus VSPI (available as SPI object in Arduino) uses pins:
 // SCLK:      18
 // MISO:      19
 // MOSI:      23
 // SS:	       5
 // The other HSPI bus uses pins
 // SCLK:      14
 // MISO:      12
 // MOSI:      12
 // SS:	       15
 // By default RadioHead uses VSPI, but you can make it use HSPI by defining this:
 //#define RH_ESP32_USE_HSPI
						   

#elif (RH_PLATFORM == RH_PLATFORM_NRF52 || RH_PLATFORM == RH_PLATFORM_RAK4631)
 #include <SPI.h>
 #define RH_HAVE_HARDWARE_SPI
 #define RH_HAVE_SERIAL
 #define PROGMEM
  #include <Arduino.h>

#else
 #error Platform unknown!
#endif

////////////////////////////////////////////////////
// Try to be compatible with systems that support yield() and multitasking
// instead of spin-loops
#if (RH_PLATFORM == RH_PLATFORM_ESP32)
 // ESP32 also has it
 #define YIELD yield();
#else
 #define YIELD
#endif

////////////////////////////////////////////////////
// digitalPinToInterrupt is not available prior to Arduino 1.5.6 and 1.0.6
// See http://arduino.cc/en/Reference/attachInterrupt
#ifndef NOT_AN_INTERRUPT
 #define NOT_AN_INTERRUPT -1
#endif
#ifndef digitalPinToInterrupt

 #if (RH_PLATFORM == RH_PLATFORM_ESP32)
  #define digitalPinToInterrupt(p) (((p) < 40) ? (p) : -1)

 #else
  // Everything else (including Due and Teensy) interrupt number the same as the interrupt pin number
  #define digitalPinToInterrupt(p) (p)
 #endif
#endif

// Slave select pin, some platforms such as ATTiny do not define it.
// ESP32 pins_arduino.h uses static const uint8_t SS = <UINT>; instead
// of a #define to declare the SS constant.
#if (RH_PLATFORM != RH_PLATFORM_ESP32)
  #ifndef SS
    #define SS 10
  #endif
#endif

#if (RH_PLATFORM == RH_PLATFORM_ESP32)
    #define RH_INTERRUPT_ATTR IRAM_ATTR
#else
    #define RH_INTERRUPT_ATTR
#endif

// These defs cause trouble on some versions of Arduino
#undef abs
#undef round
#undef double

// Sigh: there is no widespread adoption of htons and friends in the base code, only in some WiFi headers etc
// that have a lot of excess baggage
#if RH_PLATFORM != RH_PLATFORM_UNIX && !defined(htons)
// #ifndef htons
// These predefined macros available on modern GCC compilers
 #if   __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  // Atmel processors
  #define htons(x) ( ((x)<<8) | (((x)>>8)&0xFF) )
  #define ntohs(x) htons(x)
  #define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
  #define ntohl(x) htonl(x)

 #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  // Others
  #define htons(x) (x)
  #define ntohs(x) (x)
  #define htonl(x) (x)
  #define ntohl(x) (x)

 #else
  #error "RadioHead.h: Dont know how to define htons and friends for this processor" 
 #endif
#endif

// Some platforms need a mutex for multihreaded case
#ifdef RH_USE_MUTEX
 #include <pthread.h>
 #define RH_DECLARE_MUTEX(X) pthread_mutex_t X;						   
 #define RH_MUTEX_INIT(X) pthread_mutex_init(&X, NULL)
 #define RH_MUTEX_LOCK(X) pthread_mutex_lock(&X)
 #define RH_MUTEX_UNLOCK(X) pthread_mutex_unlock(&X)						   
#else
 #define RH_DECLARE_MUTEX(X)
 #define RH_MUTEX_INIT(X)
 #define RH_MUTEX_LOCK(X)
 #define RH_MUTEX_UNLOCK(X)
#endif

// This is the address that indicates a broadcast
#define RH_BROADCAST_ADDRESS 0xff

// Specifies an invalid IO pin selection
#define RH_INVALID_PIN       0xff

// Here we have some system wide macroses you can define to alter the baviour of RadioHead
// in various ways. The Ardiono IDE has no way to configure such things at compile time so
// on that pltform you are forced to edit these macros here.
// On platformio you can add them to platformio.ini like, say:
// -D RH_ACK_DELAY=10`

// Uncomment this to add a delay before acknowledgement in RHReliableDatagram.
// In some networks with mixed processor speeds, may need this delay to prevent a
// fast processor sending an ack before the receiver is ready for it.
// The time is in milliseconds
// #define  RH_ACK_DELAY 10

// RH_Uncomment this to control which timer used by RH_ASK in some platforms like
// STM32:
// #define  RH_HW_TIMER TIM21`

// Uncomment this is to enable Encryption (see RHEncryptedDriver):
// But ensure you have installed the Crypto directory from arduinolibs first:
// http://rweather.github.io/arduinolibs/index.html
//#define RH_ENABLE_ENCRYPTION_MODULE

#endif