#include "SerialPort.h"

#ifdef _WIN32
	#include "HARDWARE/TRACKER/COMMON/src/WinSerialPort.h"
#else
	#include "PosixSerialPort.h"
#endif


/* --------------------------------------------------------------
 *
 * class CSerialPortFactory
 *
 * -------------------------------------------------------------- */

#ifndef _SERIALPORTFACTORY_H_
#define _SERIALPORTFACTORY_H_


/**
   Factory class for creating a os-dependent serialport class

   @author Peter Heinze
   @date   15.12.2003
 */
class  IROB_EXPORT CSerialPortFactory
{
public:

	/**
	Static function for creation an os-dependent serialport class

  Constant         Used By         Naming Convention
----------       -------------   ------------------------
_TTY_WIN_        Windows         COM1, COM2
_TTY_IRIX_       SGI/IRIX        /dev/ttyf1, /dev/ttyf2
_TTY_HPUX_       HP-UX           /dev/tty1p0, /dev/tty2p0
_TTY_SUN_        SunOS/Solaris   /dev/ttya, /dev/ttyb
_TTY_DIGITAL_    Digital UNIX    /dev/tty01, /dev/tty02
_TTY_LINUX_      Linux           /dev/ttyS0, /dev/ttyS1
<none>           Linux           /dev/ttyS0, /dev/ttyS1

	@param string	constant for describing the device
	*/
	static SerialPort*	CreateSerialPort(std::string sDevice="")
	  {

	    if(sDevice.empty()){
                #if defined(IROB_OS_LINUX) || defined(IROB_OS_IRIX)
   	          sDevice = "/dev/ttyS0";
                #else
	          sDevice = "COM1";
               #endif
	    }
		#ifdef _WIN32
			return new WinSerialPort(sDevice);
		#else
			return new PosixSerialPort(sDevice);
		#endif
	}
};

#endif

