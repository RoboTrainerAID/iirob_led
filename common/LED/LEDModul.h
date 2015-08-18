#ifndef HARDWARE_LED_LEDMODUL_H_
#define HARDWARE_LED_LEDMODUL_H_

#include "../SERIAL/SerialPortFactory.h"
#include "../SERIAL/I2C.h"


namespace iirob_hardware {

	class IROB_EXPORT LEDModul {
	public:
		static const unsigned char LED_BASE_ADDR = 0xC0;
        static const unsigned char LED_READ  = LED_BASE_ADDR | I2C::I2C_READ;
		static const unsigned char LED_WRITE = LED_BASE_ADDR | I2C::I2C_WRITE;
		
		/* registers */
		
		static const unsigned char INPUT0 = 0x00;  //input register 1, ro
		static const unsigned char INPUT1 = 0x01;  //input register 2, ro
		
		static const unsigned char PSC0   = 0x02;  //frequency prescaler 0, r/w
		static const unsigned char PWM0   = 0x03;  //PWM register 0, r/w
		static const unsigned char PSC1   = 0x04;  //frequency prescaler 1, r/w
		static const unsigned char PWM1   = 0x05;  //PWM register 1, r/w
		static const unsigned char LS0    = 0x06;  //LED0 to LED3 selector, r/w
		static const unsigned char LS1    = 0x07;  //LED4 to LED7 selector, r/w
		static const unsigned char LS2    = 0x08;  //LED8 to LED11 selector, r/w
		static const unsigned char LS3    = 0x09;  //LED12 to LED15 selector, r/w
		static const unsigned char AINC   = 0x10;  //automatic increment flag
		
		/** Create a connection to the specified serial port and open the device.
		 *  Valid port strings are e.g. "COM7" or "/dev/ttyS7", "/dev/ttyUSB0". */
		LEDModul(const std::string& port, bool printDebug=false);
		/** Closes the serial port connection. */
		~LEDModul();
	
		/** Returns true if the serial port was connected successfully. */
		bool ready();
		
		/** Send n bytes of the specified buffer to the i2c controller or the sensor. */
		bool send(unsigned char* buf, int n);
		/** Receive n bytes from the i2c/sensor and store them in the specified buffer. */
		bool receive(unsigned char* buf, int n);
		
		bool init(char module);
		/** Write value to register in specified module. */
		bool set(unsigned char module, unsigned char reg, unsigned char value);
		/** Write n values to registers starting with startReg in specified module. */
		bool setN(unsigned char module, unsigned char startReg, 
				unsigned char* values, int n);
		/** Read value from register in specified module. */
		int  get(unsigned char module, unsigned char reg);
		/** Set RGB value of led on module, binary format XXRRGGBB. */
		bool setRGB(unsigned char module, unsigned char led, unsigned char value);
		/** As above, explicit variables */
		bool setRGB(unsigned char module, unsigned char led, 
				unsigned char red, unsigned char green, unsigned char blue);
		/** Set hue of led on module, 24 steps/360 degrees -> angle/15 */
		bool setHue(unsigned char module, unsigned char led, unsigned char hue);
		/** Set RGB value of all leds on module, binary format XXRRGGBB. */
		bool setAllRGB(unsigned char module, unsigned char value);
		/** As above, explicit variables */
		bool setAllRGB(unsigned char module, 
				unsigned char red, unsigned char green, unsigned char blue);
		/** Set hue of all leds on module, 24 steps/360 degrees -> angle/15 */
		bool setAllHue(unsigned char module, unsigned char hue);
		
		
	private:
		LEDModul() {}
		
		SerialPort* com;
		bool printDebug;
		int nn;
		unsigned char buf[36];
	};

}

#endif
