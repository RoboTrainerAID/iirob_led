/*
 * I2C.h
 *
 *  Created on: 18.05.2009
 *      Author: notheis
 */

#ifndef HARDWARE_RANGER_I2C_H_
#define HARDWARE_RANGER_I2C_H_


namespace irob_hardware {

	class IROB_EXPORT I2C {
	public:
		static const unsigned char I2C_SGL = 0x53;
		static const unsigned char I2C_MUL = 0x54;
		static const unsigned char I2C_AD1 = 0x55;
		static const unsigned char I2C_AD2 = 0x56;
		static const unsigned char I2C_USB = 0x5A;
		
		static const unsigned char I2C_WRITE = 0x00;
		static const unsigned char I2C_READ  = 0x01;
		
		/* i2c usb commands */
		
		static const unsigned char I2C_USB_SCAN1 = 0x04;
	};

}

#endif
