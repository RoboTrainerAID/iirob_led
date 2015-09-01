#include "LEDStrip.h"
#include "../SERIAL/SerialPortFactory.h"

#include <cstdio>
#include <cmath>

#define THREEWIRE

using namespace iirob_hardware;

const unsigned char LEDStrip::led_lut[256] = {
		0,   0,   0,   0,   0,   0,   0,   0,
		0,   0,   0,   0,   0,   0,   0,   0,
		0,   0,   1,   1,   1,   1,   1,   1,
		1,   1,   1,   1,   1,   1,   1,   1,
		1,   1,   1,   1,   1,   1,   1,   1,
		1,   1,   2,   2,   2,   2,   2,   2,
		2,   2,   2,   2,   2,   2,   2,   2,
		2,   3,   3,   3,   3,   3,   3,   3,
		3,   3,   3,   3,   3,   4,   4,   4,
		4,   4,   4,   4,   4,   4,   5,   5,
		5,   5,   5,   5,   5,   5,   6,   6,
		6,   6,   6,   6,   6,   7,   7,   7,
		7,   7,   8,   8,   8,   8,   8,   9,
		9,   9,   9,   9,  10,  10,  10,  10,
	 11,  11,  11,  11,  12,  12,  12,  12,
	 13,  13,  13,  14,  14,  14,  15,  15,
	 15,  16,  16,  16,  17,  17,  18,  18,
	 18,  19,  19,  20,  20,  21,  21,  22,
	 22,  23,  23,  24,  24,  25,  25,  26,
	 26,  27,  28,  28,  29,  30,  30,  31,
	 32,  32,  33,  34,  35,  35,  36,  37,
	 38,  39,  40,  40,  41,  42,  43,  44,
	 45,  46,  47,  48,  49,  51,  52,  53,
	 54,  55,  56,  58,  59,  60,  62,  63,
	 64,  66,  67,  69,  70,  72,  73,  75,
	 77,  78,  80,  82,  84,  86,  88,  90,
	 91,  94,  96,  98, 100, 102, 104, 107,
	109, 111, 114, 116, 119, 122, 124, 127,
	130, 133, 136, 139, 142, 145, 148, 151,
	155, 158, 161, 165, 169, 172, 176, 180,
	184, 188, 192, 196, 201, 205, 210, 214,
	219, 224, 229, 234, 239, 244, 250, 255
};

unsigned int LEDStrip::ledLog16(unsigned char col) {
	return 65535*pow(col/255.0f, 2.5f);
}

float LEDStrip::ledLogf(float col) {
	return pow(col, 2.5f);
}

LEDStrip::LEDStrip(const std::string& port, bool printDebug)
: printDebug(printDebug) {

	printf("Creating  serial port on %s... ", port.c_str());
	com = CSerialPortFactory::CreateSerialPort(port);

	SerialPort::Attributes attr;
		attr.baudRate = SerialPort::BaudRate_921600;
		//attr.flowControl = SerialPort::FlowControl_off;
		//attr.dataBits = SerialPort::DataBits_8;
		attr.stopBits = SerialPort::StopBits_1;
		//attr.timeout = 500;
  com->setAttributes(attr);

	printf("%s\nOpening   serial port... ", (com!=NULL)?"OK":"FAILED");
	bool ok = com->open();
	printf("%s\n", (ok)?"OK":"FAILED");
	//ok &= setMaxRange(600);
	if (!ok) {
		printf("FAILED to initialize!\n");
		com->close();
		com = NULL;
	}
}

LEDStrip::~LEDStrip() {
	if (com) {
		printf("Closing serial port...\n");
		com->close();
	}
}

bool LEDStrip::ready() {
	return ((com != NULL) && com->isOpen());
}

bool LEDStrip::send(unsigned char* buf, int n) {
	if (!com)
		return false;
	if (printDebug)
		printf("Sending   %d bytes... ", n);
	bool ok = com->write((char*)buf, n);
	if (printDebug)
		printf("%s\n", (ok)?"OK":"FAILED");
	return ok;
}

bool LEDStrip::receive(unsigned char* buf, int n) {
	if (!com)
		return false;
	if (printDebug)
		printf("Receiving %d bytes... ", n);
	nn = com->read((char*)buf, n);
	if (printDebug && (n == nn)) {
		printf("OK");
		for (int i=0; i<nn; ++i)
			printf("%4d", buf[i]);
		printf("\n");
	} else if (printDebug && (n != nn))
		printf("FAILED (%d bytes received) [%s]\n", nn, com->getError().c_str());
	return (n == nn);
}

bool LEDStrip::setRGB(unsigned char* rgb, int n, bool log) {
	if ((n*3+4)>sizeof(buf))
		return false;
	bool ok = true;
	buf[0] = LEDStrip::START;
	buf[1] = (char)(n>>8);
	buf[2] = (char)n;
	for (int i = 3; i<(n*3+3); i++)
		buf[i] = (log?led_lut[*rgb++]:*rgb++);
	buf[n*3+3] = LEDStrip::END;
	ok &= send(buf, n*3+4);
	ok &= receive(buf, 1);
	ok &= (buf[0]==LEDStrip::OK);
	return ok;
}

bool LEDStrip::setUniRGB(unsigned int red, unsigned int green, unsigned int blue) {
	bool ok = true;
	buf[0] = LEDStrip::START;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = red;
	buf[4] = red >> 8;
	buf[5] = green;
	buf[6] = green >> 8;
	buf[7] = blue;
	buf[8] = blue >> 8;
	buf[9] = LEDStrip::END;
	ok &= send(buf, 10);
	ok &= receive(buf, 1);
	ok &= (buf[0]==LEDStrip::OK);
	return ok;
}

bool LEDStrip::setUniRGBf(float red, float green, float blue) {
	return LEDStrip::setUniRGB((unsigned int)(red*65535.0), (unsigned int)(green*65535.0),
			(unsigned int)(blue*65535.0));
}

bool LEDStrip::setUniHue(float hue) {
	std::vector<float> rgb = hueToRGB(hue);
	return setUniRGBf(ledLogf(rgb[0]), ledLogf(rgb[1]), ledLogf(rgb[2]));
}

bool LEDStrip::setRGBf(float* rgb, int n, bool log) {
	if (n*3>sizeof(rgbtemp))
		return false;
	for (int i = 0; i<(n*3); i++)
		rgbtemp[i] = (unsigned char) ((*rgb++)*255.0);
	return LEDStrip::setRGB(rgbtemp, n, log);
}

bool LEDStrip::setHue(float* hue, int n, bool log) {
	float red, green, blue;
	for (int i = 1; i<(n*3); i+=3) {
		*hue -= (unsigned char)(*hue);
		unsigned char sect = (unsigned char)(*hue*6);
		*hue = *hue*6-sect;
		switch (sect) {
			case 0: red = 1.0;      green = *hue;     blue = 0.0;      break;
			case 1: red = 1.0-*hue; green = 1.0;      blue = 0.0;      break;
			case 2: red = 0.0;      green = 1.0;      blue = *hue;     break;
			case 3: red = 0.0;      green = 1.0-*hue; blue = 1.0;      break;
			case 4: red = *hue;     green = 0.0;      blue = 1.0;      break;
			case 5: red = 1.0;      green = 0.0;      blue = 1.0-*hue; break;
			default:red = 0.0;      green = 0.0;      blue = 0.0;      break;
		}
#ifdef THREEWIRE
		rgbtemp[i]   = (unsigned char)(green*255.0);
		rgbtemp[i+1] = (unsigned char)(red*255.0);
		rgbtemp[i+2] = (unsigned char)(blue*255.0);
#else
		rgbtemp[i]   = (unsigned char)(blue*255.0);
		rgbtemp[i+1] = (unsigned char)(green*255.0);
		rgbtemp[i+2] = (unsigned char)(red*255.0);
#endif
		hue++;
	}
	return LEDStrip::setRGB(rgbtemp, n, log);
}

bool LEDStrip::setAllRGB(unsigned char red, unsigned char green, unsigned char blue, int n, bool log){
	if (n*3>sizeof(rgbtemp))
		return false;
	for (int i = 0; i<(n*3); i+=3) {
#ifdef THREEWIRE
		rgbtemp[i]   = green;
		rgbtemp[i+1] = red;
		rgbtemp[i+2] = blue;
#else
		rgbtemp[i]   = blue;
		rgbtemp[i+1] = green;
		rgbtemp[i+2] = red;
#endif
	}
	return setRGB(rgbtemp, n, log);
}

bool LEDStrip::setAllRGBf(float red, float green, float blue, int n, bool log) {
	return LEDStrip::setAllRGB((unsigned char)(red*255.0), (unsigned char)(green*255.0),
			(unsigned char)(blue*255.0), n, log);
}

bool LEDStrip::setAllHue(float hue, int n, bool log) {
	float red, green, blue;
	hue -= (unsigned char)hue;
	unsigned char sect = (unsigned char)(hue*6);
	hue = hue*6-sect;
	switch (sect) {
		case 0: red = 1.0;     green = hue;     blue = 0.0;     break;
		case 1: red = 1.0-hue; green = 1.0;     blue = 0.0;     break;
		case 2: red = 0.0;     green = 1.0;     blue = hue;     break;
		case 3: red = 0.0;     green = 1.0-hue; blue = 1.0;     break;
		case 4: red = hue;     green = 0.0;     blue = 1.0;     break;
		case 5: red = 1.0;     green = 0.0;     blue = 1.0-hue; break;
		default:red = 0.0;     green = 0.0;     blue = 0.0;     break;
	}
	return LEDStrip::setAllRGBf(red, green, blue, n, log);
}

std::vector<float> LEDStrip::hueToRGB(float hue) {
	float red, green, blue;
	std::vector<float> rgb;
	hue -= (unsigned char)hue;
	unsigned char sect = (unsigned char)(hue*6);
	hue = hue*6-sect;
	switch (sect) {
		case 0: red = 1.0;     green = hue;     blue = 0.0;     break;
		case 1: red = 1.0-hue; green = 1.0;     blue = 0.0;     break;
		case 2: red = 0.0;     green = 1.0;     blue = hue;     break;
		case 3: red = 0.0;     green = 1.0-hue; blue = 1.0;     break;
		case 4: red = hue;     green = 0.0;     blue = 1.0;     break;
		case 5: red = 1.0;     green = 0.0;     blue = 1.0-hue; break;
		default:red = 0.0;     green = 0.0;     blue = 0.0;     break;
	}
	rgb.push_back(red);
	rgb.push_back(green);
	rgb.push_back(blue);
	return rgb;
}

//Methods for modifying individual parts of the strip
bool LEDStrip::setXRangeRGB(unsigned char* rgb, int n, int b, int e, bool log) {
	if ((n*3+4)>sizeof(buf))
		return false;
	bool ok = true;
	buf[0] = LEDStrip::START;
	buf[1] = (char)(n>>8);
	buf[2] = (char)n;
	for (int i = 2+(b*3)+1; i<(e*3+2)+1; i++) //+1
        buf[i] = log ? led_lut[*rgb++] : *rgb++;
	buf[n*3+3] = LEDStrip::END;
	ok &= send(buf, n*3+4);
	ok &= receive(buf, 1);
	ok &= (buf[0]==LEDStrip::OK);
	return ok;
}

bool LEDStrip::setRangeRGB(unsigned char red, unsigned char green, unsigned char blue, int n, int b, int e, bool log){
	if (n*3>sizeof(rgbtemp))
		return false;
	for (int i = 0; i<(n*3); i+=3) {
#ifdef THREEWIRE
		rgbtemp[i]   = green;
		rgbtemp[i+1] = red;
		rgbtemp[i+2] = blue;
#else
		rgbtemp[i]   = blue;
		rgbtemp[i+1] = green;
		rgbtemp[i+2] = red;
#endif
	}
	return setXRangeRGB(rgbtemp, n, b, e, log);
}

bool LEDStrip::setRangeRGBf(float red, float green, float blue, int n, int b, int e, bool log) {
	return LEDStrip::setRangeRGB((unsigned char)(red*255.0), (unsigned char)(green*255.0),
			(unsigned char)(blue*255.0), n, b, e, log);
}
