#include "WinSerialPort.h"


WinSerialPort::WinSerialPort(std::string device) : SerialPort(device) {
	m_hIDComDev = NULL;
}


WinSerialPort::~WinSerialPort() {
  close();
}


void WinSerialPort::setAttributes(const Attributes& a) {
  attributes = a;
  if (isOpen()) setAttributes();
}


void WinSerialPort::setTimeout(int ms) {
  attributes.timeout = ms;
  if (isOpen()) setAttributes();
}


bool WinSerialPort::open() 
{
	if (isOpen()) return true;
	error = "";

	m_hIDComDev = CreateFile( serialDevice.c_str(), GENERIC_READ | GENERIC_WRITE, NULL, NULL, OPEN_EXISTING, 0  , NULL );

	if (m_hIDComDev == INVALID_HANDLE_VALUE)
	{
		error = "Cannot open device '" + serialDevice + "'";
		return false ;
	}

	flagOpen = true;

	setAttributes();

	return isOpen();
}



void WinSerialPort::close() 
{
	if (isOpen()) 
	{
		CloseHandle( m_hIDComDev );
		flagOpen	= false;
		m_hIDComDev	= NULL;
	}
}



bool WinSerialPort::write(const char* buffer, int size) 
{
	if (!isOpen()) 
	{
		error = "Port not open";
		return false;
	}
  
	unsigned long n = 0;
	int nResult = ::WriteFile(m_hIDComDev, buffer, size, &n, NULL);

	if ((nResult==0) || (n!=size))
	{
		close();
		error = "Write error";
		return false;
	}

	return true;
}


int WinSerialPort::read(char* buffer, int size) 
{
	if (!isOpen()) 
	{
		error = "Port not open";
		return 0;
	}

	unsigned long	n = 0;

	int nResult	= ::ReadFile(m_hIDComDev, buffer, size, &n, NULL) ; 
	if ((nResult==0)) 
	{
		if (GetLastError() != ERROR_MORE_DATA) 
		{
			close();
			error = "Read error";
			return 0;
		}
		return 0;
	}
	return n;
}


bool WinSerialPort::sendBreak() 
{
	if (!isOpen()) 
	{
		error = "Port not open";
		return false;
	}

	if (SetCommBreak( m_hIDComDev ) == 0)
	{
		error = "Error sending break";
		return false;		
	}
		
	Sleep( 1000 );

	if (ClearCommBreak( m_hIDComDev ) == 0)
	{
		error = "Error sending break";
		return false;
	}

	return true;
}


void WinSerialPort::setAttributes() 
{
	int  nBaud;
	switch(attributes.baudRate) 
	{
	case BaudRate_9600:
		nBaud = 9600;
		break;
	case BaudRate_19200:
	    nBaud = 19200;
		break;
	case BaudRate_38400:
		nBaud = 38400;
		break;
	case BaudRate_57600:
		nBaud = 57600;
		break;
	case BaudRate_115200:
		nBaud = 115200;
		break;
	case BaudRate_230400:
		nBaud = 230400;
		break;
	case BaudRate_460800:
		nBaud = 460800;
		break;
	case BaudRate_921600:
		nBaud = 921600;
		break;
	default:
		close();
		error = "Invalid baud rate";
		return;
	}


	COMMTIMEOUTS	CommTimeOuts;
	COMMCONFIG		CommConfig;
	unsigned long	nConfSize;
	
	if (GetCommConfig(m_hIDComDev, &CommConfig, &nConfSize) == 0)
		error = "GetCommConfig error";

    if (GetCommState(m_hIDComDev, &(CommConfig.dcb)) == 0)
		error = "GetCommState error";

	if (GetCommTimeouts( m_hIDComDev, &CommTimeOuts ) == 0)
		error = "GetCommTimeouts error";

	CommTimeOuts.ReadIntervalTimeout			= attributes.timeout;
	CommTimeOuts.ReadTotalTimeoutMultiplier		= attributes.timeout;
	CommTimeOuts.ReadTotalTimeoutConstant		= 0;
	CommTimeOuts.WriteTotalTimeoutMultiplier	= attributes.timeout;
	CommTimeOuts.WriteTotalTimeoutConstant		= 0;

	if (SetCommTimeouts( m_hIDComDev, &CommTimeOuts ) == 0)
		error = "SetCommTimeouts error";

	CommConfig.dcb.BaudRate		= nBaud;
	CommConfig.dcb.ByteSize		= 8;
	CommConfig.dcb.fBinary		= TRUE;
    CommConfig.dcb.fInX			= FALSE;
    CommConfig.dcb.fOutX		= FALSE;
	CommConfig.dcb.fAbortOnError= FALSE;
    CommConfig.dcb.fNull		= FALSE;
	CommConfig.dcb.fParity		= FALSE;
	CommConfig.dcb.fOutxCtsFlow	= FALSE;
    CommConfig.dcb.fRtsControl	= RTS_CONTROL_DISABLE;
    CommConfig.dcb.fInX			= FALSE;
    CommConfig.dcb.fOutX		= FALSE;

	if (SetCommConfig(m_hIDComDev, &CommConfig, sizeof(COMMCONFIG)) == 0)
		error = "GetCommConfig error";
}

