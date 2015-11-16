#ifndef HARDWARE_LED_LEDSTRIP_H_
#define HARDWARE_LED_LEDSTRIP_H_

#include "../SERIAL/SerialPortFactory.h"

#define MAX_LED 512

namespace iirob_hardware {

    class IIROB_EXPORT LEDStrip {
	public:
		static const unsigned char START = 0xAA;
		static const unsigned char END  = 0xBB;
		static const unsigned char OK  = 0xCC;
        static const unsigned char led_lut[256];    ///< Lookup table for the intensity

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


        /** Set RGB value of a given range of leds */
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

        /////////////////////////////////////// NEW FEATURES ////////////////////////////////////////
        /**
         * @brief withinRange Check if start and end are withing the allowed limits for the LED array
         * @param totNumLeds Total number of LEDs
         * @param start_led Index of the starting LED of the given range
         * @param end_led Index of the ending LED of the given range
         * @return Result of the check whether @start_led and @end_led are within the given range (neither can be less than 0 or greater than the value of @totNumLeds)
         */
        bool withinRange(int totNumLeds, int start_led, int end_led);

        // All methods below ofer the optional logarithmic scaling (param: log)
        /** Set RGB value of a given range of leds - final stage that triggers the actualy transfer of the buffer in its current state; if sendTrigger=false, the buffer will not be sent and will be overwritten (useful for doing multiple changes to the leds and then sending all in one push */
        /**
         * @brief setXRangeRGB Set RGB value of a given range of leds with color values (array of unsigned chars). Generates @buf from @rgbtemp
         * @param rgb Array of RGB colour values in the interval [0..255]
         * @param totNumLeds Total number of LEDs
         * @param start_led Index of the starting LED of the given range
         * @param end_led Index of the ending LED of the given range
         * @param log Logarithmic scaling
         * @param checkLimits Check whether @start_led and @end_led are within the limits of the range. Useful only if method is not called indirectly through setRangeRGB(...)
         * @param sendTrigger  If true (default) sending of the buffer @buf in its current state will be triggered.
         * @return True if sendTrigger==false or if sendTrigger==true and both send and receive were successful
         */
        bool setXRangeRGB(unsigned char* rgb, int totNumLeds, int start_led, int end_led, bool log=false, bool checkLimits=true, bool sendTrigger=false);
        /**
         * @brief setRangeRGB Set RGB value of a given range of leds with color values (float). Also handles the different cases based on the values of @start_led and @end_led. This method is usually called indirectly upon calling setRangeRGBf(...)
         * @param red RGB red color valuee within the limits of the float interval [0..1]
         * @param green RGB green color valuee within the limits of the float interval [0..1]
         * @param blue RGB blue color valuee within the limits of the float interval [0..1]
         * @param totNumLeds Total number of LEDs
         * @param start_led Index of the starting LED of the given range
         * @param end_led Index of the ending LED of the given range
         * @param log Logarithmic scaling
         * @param checkLimits Check whether @start_led and @end_led are within the limits of the range. Useful only if method is not called indirectly through setRangeRGBf(...)
         * @param sendTrigger If true (default) sending of the buffer @buf in its current state will be triggered. Note that this has no effect if setXRangeRGB(...) is not called through this method
         * @return Return value of setXRangeRGB(...)
         */
        bool setRangeRGB(unsigned char red, unsigned char green, unsigned char blue, int totNumLeds, int start_led, int end_led, bool log=true, bool checkLimits=false, bool sendTrigger=false);
        /**
         * @brief setRangeRGBf Set RGB value of a given range of leds with color values (unsigned char). Generates @rgbtemp
         * @param red RGB red color valuee within the limits of the interval [0..255]
         * @param green RGB green color valuee within the limits of the interval [0..255]
         * @param blue RGB blue color valuee within the limits of the finterval [0..255]
         * @param totNumLeds Total number of LEDs
         * @param start_led Index of the starting LED of the given range
         * @param end_led Index of the ending LED of the given range
         * @param log Logarithmic scaling
         * @param checkLimits Check whether @start_led and @end_led are within the limits of the range
         * @param sendTrigger If true (default) sending of the buffer @buf in its current state will be triggered. Note that this has no effect if setRangeRGB(...) or setXRangeRGB(...) are not called through this method
         * @return Return value of setRangeRGB(...)
         */
        bool setRangeRGBf(float red, float green, float blue, int totNumLeds, int start_led, int end_led, bool log=true, bool checkLimits=true, bool sendTrigger=true);

	private:
		LEDStrip() {}

		SerialPort* com;
		bool printDebug;
		int nn;
        unsigned char buf[MAX_LED*3+4];     ///< A buffer which contains the raw data that is used to light up the selected LEDs
        unsigned char rgbtemp[MAX_LED*3];   ///< A buffer which contains intermediary data of the colours that the LEDs will display. It is used for the generation of @buf
	};

}

#endif
