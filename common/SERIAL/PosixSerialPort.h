

#ifndef _POSIXSERIALPORT_H_
#define _POSIXSERIALPORT_H_


#include "SerialPort.h"


class IIROB_EXPORT PosixSerialPort : public SerialPort {
public:

  PosixSerialPort(std::string device);
  virtual ~PosixSerialPort();

  virtual void setAttributes(const Attributes& a);
  virtual void setTimeout(int ms);
  virtual bool open();
  virtual void close();

  virtual bool write(const char* buffer, int size);

  virtual bool write(std::string data) { return SerialPort::write(data); }

  virtual int read(char* buffer, int size);

  virtual int read(char* buffer, int size, char stopChar) { return SerialPort::read(buffer, size, stopChar); }
  virtual std::string read() { return SerialPort::read(); }
  virtual std::string read(char stopChar) { return SerialPort::read(stopChar); }


  virtual bool sendBreak();

private:
  int file;
  struct termios* deviceTermios;

  void setAttributes();
};


#endif

