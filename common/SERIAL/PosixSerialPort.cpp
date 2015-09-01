

#include "PosixSerialPort.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>



PosixSerialPort::PosixSerialPort(std::string device) : SerialPort(device) {
  deviceTermios = new struct termios;
}


PosixSerialPort::~PosixSerialPort() {
  close();
  delete deviceTermios;
}


void PosixSerialPort::setAttributes(const Attributes& a) {
  attributes = a;
  if (isOpen()) setAttributes();
}


void PosixSerialPort::setTimeout(int ms) {
  attributes.timeout = ms;
  if (isOpen()) setAttributes();
}


bool PosixSerialPort::open() {
  if (isOpen()) return true;
  error = "";

  //file = ::open(serialDevice.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); 
  file = ::open(serialDevice.c_str(), O_RDWR | O_NOCTTY); 
  if (file < 0) {
    error = "Cannot open device '" + serialDevice + "'";
    return false;
  }

  flagOpen = true;

  if (tcgetattr(file, deviceTermios) != 0) {
    close();
    error = "Cannot get device attributes";
    return false;
  }

  // info about the flags can be found @ http://www.unixguide.net/unix/programming/3.6.2.shtml

  //cfmakeraw(deviceTermios);  // does not exist on IRIX
  deviceTermios->c_iflag = 0;
  deviceTermios->c_oflag = 0;
  deviceTermios->c_cflag = CREAD | CLOCAL;
  deviceTermios->c_lflag &= ~( ICANON | ECHO | ISIG );
  for(int i=0; i<NCCS; i++) deviceTermios->c_cc[i] = _POSIX_VDISABLE;

  setAttributes();

  return isOpen();
}


void PosixSerialPort::close() {
  if (isOpen()) {
    ::close(file);
    flagOpen = false;
  }
}


bool PosixSerialPort::write(const char* buffer, int size) {
  if (!isOpen()) {
    error = "Port not open";
    return false;
  }
  while(size > 0) {
    int n = ::write(file, buffer, size);
    if (n < 0) {
      if (errno != EAGAIN) {
	close();
	error = "Write error";
	return false;
      }
      n = 0;
      usleep(20*1000);
    }
    buffer += n;
    size -= n;
  }
  return true;
}


int PosixSerialPort::read(char* buffer, int size) {
  if (!isOpen()) {
    error = "Port not open";
    return 0;
  }
  int n = ::read(file, buffer, size);
  if (n < 0) {
    if (errno != EAGAIN) {
      close();
      error = "Read error";
      return 0;
    }
    return 0;
  }
  return n;
}


bool PosixSerialPort::sendBreak() {
  if (!isOpen()) {
    error = "Port not open";
    return false;
  }
  if (tcsendbreak(file, 3) != 0) {
    error = "Error sending break";
    return false;
  }
  return true;
}


void PosixSerialPort::setAttributes() {
  speed_t speed;
  switch(attributes.baudRate) {
  case BaudRate_9600:
    speed = B9600;
    break;
  case BaudRate_19200:
    speed = B19200;
    break;
  case BaudRate_38400:
    speed = B38400;
    break;
  case BaudRate_57600:
    speed = B57600;
    break;
  case BaudRate_115200:
    speed = B115200;
    break;
  case BaudRate_230400:
    speed = B230400;
    break;
  case BaudRate_460800:
    speed = B460800;
    break;
  case BaudRate_921600:
    speed = B921600;
    break;
  default:
    close();
    error = "Invalid baud rate";
    return;
  }
  cfsetispeed(deviceTermios, speed);
  cfsetospeed(deviceTermios, speed);

  tcflag_t hwFlag;
#ifdef CNEW_RTSCTS
  hwFlag = CNEW_RTSCTS;
#else
  hwFlag = CRTSCTS;
#endif
  deviceTermios->c_cflag &= ~hwFlag;
  deviceTermios->c_iflag &= ~(IXON | IXOFF);
  switch(attributes.flowControl) {
  case FlowControl_hardware:
    deviceTermios->c_cflag |= hwFlag;
    break;
  case FlowControl_XONXOFF:
    deviceTermios->c_iflag |= (IXON | IXOFF);
    break;
  default:
    break;
  }

  deviceTermios->c_cflag &= ~CSIZE;
  deviceTermios->c_cflag |= CS8;

  if (attributes.stopBits == StopBits_1) {
    deviceTermios->c_cflag &= ~CSTOPB;
  } else {
    deviceTermios->c_cflag |= CSTOPB;
  }

  deviceTermios->c_cc[VMIN] = 0;
  int timeout = attributes.timeout / 100;
  if (attributes.timeout && !timeout) timeout = 1;
  deviceTermios->c_cc[VTIME] = timeout;

  if (tcsetattr(file, TCSADRAIN, deviceTermios) != 0) {
    close();
    error = "Cannot set attributes";
    return;
  }
}



