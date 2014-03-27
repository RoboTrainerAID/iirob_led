#ifndef _WINSERIALPORT_H_
#define _WINSERIALPORT_H_

#include "SerialPort.h"
#include <windows.h>


#pragma warning( disable : 4786 )  // Disable warning messages 4786

class IROB_EXPORT WinSerialPort : public SerialPort {
public:

  WinSerialPort(std::string device);
  virtual ~WinSerialPort();

  virtual void setAttributes(const Attributes& a);
  virtual void setTimeout(int ms);
  virtual bool open();
  virtual void close();

  virtual bool write(const char* buffer, int size);
  virtual int read(char* buffer, int size);

  virtual bool sendBreak();

private:
  void		setAttributes();
  bool		WriteCommByte( unsigned char );

  HANDLE		m_hIDComDev;
};


#endif




