#ifndef HARDWARE_LED_LEDSTRIP_H_
#define HARDWARE_LED_LEDSTRIP_H_

#include "../SERIAL/SerialPortFactory.h"


namespace irob_hardware {

	class IROB_EXPORT LEDStrip {
	public:
		static const unsigned char START = 0xAA;
		static const unsigned char END  = 0xBB;
		static const unsigned char OK  = 0xCC;
		static const unsigned char led_lut[256];

		/** Create a connection to the specified serial port and open the device.
		 *  Valid port strings are e.g. "COM7" or "/dev/ttyS7", "/dev/ttyUSB0". */
		LEDStrip(const std::string& port, bool printDebug=false);
		/** Closes the serial port connection. */
		~LEDStrip();

		/** Returns logarithmic value for perception based scaling **/
		unsigned int ledLog16(unsigned char col);
		float ledLogf(float col);
	
		/** Returns true if the serial port was connected successfully. */
		bool ready();
		
		/** Send n bytes of the specified buffer */
		bool send(unsigned char* buf, int n);
		/** Receive n bytes */
		bool receive(unsigned char* buf, int n);
		

		/** Set RGB value of n RGB triples (3 floats per led)*/
		bool setRGB(unsigned char* rgb, int n, bool log=false);
		/** As above, float [0.1], opt. logarithmic scaling */
		bool setRGBf(float* rgb, int n, bool log=true);
		/** Set hue value of n RGB triples (1 float per led) */
		bool setHue(float* hue, int n, bool log=true);


		/** Set RGB value of all leds */
		bool setAllRGB(unsigned char red, unsigned char green, unsigned char blue, int n, bool log=true);
		/** As above, float [0.1], opt. logarithmic scaling */
		bool setAllRGBf(float red, float green, float blue, int n, bool log=true);
		/** Set hue of all leds [0,1] */
		bool setAllHue(float hue, int n, bool log=true);

		/** Set RGB value of single color string */
		bool setUniRGB(unsigned int red, unsigned int green, unsigned int blue);
		/** As above, float [0.1] */
		bool setUniRGBf(float red, float green, float blue);
		/** As above, hue */
		bool setUniHue(float hue);

		/** Returns RGB for hue value [0,1] (S,V = 1.0) */
		std::vector<float> hueToRGB(float hue);

	/**Hinzugefügt um einzelne Bereiche des Streifens anzusteuern*/
	bool setMyRGB(unsigned char* rgb, int n, int b, int e, bool log=false);
	/** Set RGB value of all leds */
	bool setMyAllRGB(unsigned char red, unsigned char green, unsigned char blue, int n, int b, int e, bool log=true);
	/** As above, float [0.1], opt. logarithmic scaling */
	bool setMyAllRGBf(float red, float green, float blue, int n, int b, int e, bool log=true);
		
		
	private:
		LEDStrip() {}
		
		SerialPort* com;
		bool printDebug;
		int nn;
		unsigned char buf[512];
		unsigned char rgbtemp[512];
	};

}

#endif
