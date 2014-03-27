#include "LEDModul.h"
#include "../SERIAL/SerialPortFactory.h"

#include <cstdio>

using namespace irob_hardware;

LEDModul::LEDModul(const std::string& port, bool printDebug)
: printDebug(printDebug) {
	
	printf("Creating  serial port on %s... ", port.c_str());
	com = CSerialPortFactory::CreateSerialPort(port);
	
	SerialPort::Attributes attr;
		attr.baudRate = SerialPort::BaudRate_19200;
		//attr.flowControl = SerialPort::FlowControl_off;
		//attr.dataBits = SerialPort::DataBits_8;
		attr.stopBits = SerialPort::StopBits_2;
		//attr.timeout = 500;
  com->setAttributes(attr);
	
	printf("%s\nOpening   serial port... ", (com!=NULL)?"OK":"FAILED");
	bool ok = com->open();
	printf("%s\n", (ok)?"OK":"FAILED");
	//ok &= setMaxRange(600);
	if (!ok) {
		printf("FAILED to initialize sensor!\n");
		com->close();
		com = NULL;
	}
}

LEDModul::~LEDModul() {
	if (com) {
		printf("Closing   serial port...\n");
		com->close();
	}
}

bool LEDModul::ready() {
	return ((com != NULL) && com->isOpen());
}

bool LEDModul::send(unsigned char* buf, int n) {
	if (!com)
		return false;
	if (printDebug)
		printf("Sending   %d bytes... ", n);
	bool ok = com->write((char*)buf, n);
	if (printDebug)
		printf("%s\n", (ok)?"OK":"FAILED");
	return ok;
}

bool LEDModul::receive(unsigned char* buf, int n) {
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

bool LEDModul::init(char module) {
	bool ok = true;
	ok &= LEDModul::set(module, LEDModul::PWM0, 128);
	ok &= LEDModul::set(module, LEDModul::PWM1, 64);
	return ok;
}

bool LEDModul::set(unsigned char module, unsigned char reg, unsigned char value) {
	bool ok = true;
	buf[0] = I2C::I2C_AD1;
	buf[1] = LED_WRITE | module << 1;
	buf[2] = reg;
	buf[3] = 1;
	buf[4] = value;
	ok &= send(buf, 5);
	ok &= receive(buf, 1);
	return ok;
}

bool LEDModul::setN(unsigned char module, unsigned char startReg, 
		unsigned char* values, int n) {
	bool ok = true;
	buf[0] = I2C::I2C_AD1;
	buf[1] = LED_WRITE | module << 1;
	buf[2] = startReg | AINC;
	buf[3] = (unsigned char)n;
	for (int i=0; i<n; i++)
		buf[4+i] = values[i];
	ok &= send(buf, 4+n);
	ok &= receive(buf, 1);
	return ok;
}

int LEDModul::get(unsigned char module, unsigned char reg) {
	bool ok = true;
	buf[0] = I2C::I2C_AD1;
	buf[1] = LED_READ | module << 1;
	buf[2] = reg;
	buf[3] = 1;
	ok &= send(buf, 4);
	ok &= receive(buf, 1);
	return ok ? (int)(buf[0]) : -1;
}

bool LEDModul::setRGB(unsigned char module, unsigned char led, unsigned char color) {
	bool ok = true;
	int reg = ((3*led)>>2)&0x3;
	int part = (3*led)&0x3;
	color &= 0x3F;
	color = color^(color&0x15)<<1;
	int data = LEDModul::get(module, LEDModul::LS0+reg);
	if (data > -1) {
		if (part)
			data &=	0xFF>>(8-(part<<1));
		else
			data &= 0xC0;
		data |= color<<(part<<1);
		ok &= LEDModul::set(module, LEDModul::LS0+reg, (unsigned char)data);
	}	else
		ok = false;
	if (part&0x2) { 
		int data = LEDModul::get(module, LEDModul::LS1+reg);
		if (data > -1) {
			data &=	0xFF<<(2+((part&0x1)<<1));
			data |= color>>(4-((part&0x1)<<1));
			ok &= LEDModul::set(module, LEDModul::LS1+reg, (unsigned char)data);
		}
	}	else
		ok = false;
	return ok;
}

bool LEDModul::setRGB(unsigned char module, unsigned char led,
		unsigned char red, unsigned char green, unsigned char blue) {
	return LEDModul::setRGB(module, led, (red&0x3)<<4 | (green&0x3)<<2 | (blue&0x3));
}
	
bool LEDModul::setHue(unsigned char module, unsigned char led, unsigned char hue) {
		char red, green, blue;
		hue %= 24;
		switch (hue/4) {
			case 0: red = 3; green = hue%4; blue = 0; break;
			case 1: red = 3-hue%4; green = 3; blue = 0; break;
			case 2: red = 0; green= 3; blue = hue%4; break;
			case 3: red = 0; green = 3-hue%4; blue = 3; break;
			case 4: red = hue%4; green = 0; blue = 3; break;
			case 5: red = 3; green = 0; blue = 3-hue%4; break;
			default:red = 0; green = 0; blue = 0; break;
		}
	return LEDModul::setRGB(module, led, (red&0x3)<<4 | (green&0x3)<<2 | (blue&0x3));
}

bool LEDModul::setAllRGB(unsigned char module, unsigned char color) {
	bool ok = true;
	unsigned char bufC[4];
	color &= 0x3F;
	color = color^(color&0x15)<<1;
	long data = LEDModul::get(module, LEDModul::LS3);
	if (data > -1) {
		data = color | (data&0xC0)<<24;
		for (int i=0; i<4; i++) {
			data |= color << ((i+1)*6);
			bufC[i] = data >> (i*8);
		}
		ok &= LEDModul::setN(module, LEDModul::LS0, &bufC[0], 4);
	}	else
		ok = false;
	return ok;
}

bool LEDModul::setAllRGB(unsigned char module,
		unsigned char red, unsigned char green, unsigned char blue) {
	return LEDModul::setAllRGB(module, (red&0x3)<<4 | (green&0x3)<<2 | (blue&0x3));
}

bool LEDModul::setAllHue(unsigned char module, unsigned char hue) {
		char red, green, blue;
		hue %= 24;
		switch (hue/4) {
			case 0: red = 3; green = hue%4; blue = 0; break;
			case 1: red = 3-hue%4; green = 3; blue = 0; break;
			case 2: red = 0; green= 3; blue = hue%4; break;
			case 3: red = 0; green = 3-hue%4; blue = 3; break;
			case 4: red = hue%4; green = 0; blue = 3; break;
			case 5: red = 3; green = 0; blue = 3-hue%4; break;
			default:red = 0; green = 0; blue = 0; break;
		}
	return LEDModul::setAllRGB(module, (red&0x3)<<4 | (green&0x3)<<2 | (blue&0x3));
}
