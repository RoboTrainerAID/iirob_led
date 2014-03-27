#ifndef _SERIALPORT_H_
#define _SERIALPORT_H_

#include <string>
#include <vector>


class IROB_EXPORT SerialPort {
public:

  enum BaudRate {
    BaudRate_9600,
    BaudRate_19200,
    BaudRate_38400,
    BaudRate_57600,
    BaudRate_115200,
    BaudRate_230400,
    BaudRate_460800,
    BaudRate_921600,
  };

  enum FlowControl {
    FlowControl_off,
    FlowControl_hardware,
    FlowControl_XONXOFF
  };

  enum DataBits {
    DataBits_8
  };

  enum StopBits {
    StopBits_1,
    StopBits_2
  };


  class IROB_EXPORT Attributes {
  public:
    BaudRate baudRate;
    FlowControl flowControl;
    DataBits dataBits;
    StopBits stopBits;
    int timeout;

    Attributes();
  };


  SerialPort(std::string device);
  virtual ~SerialPort();

  Attributes getAttributes() const;
  void setBaudRate(SerialPort::BaudRate newBaudrate);
  virtual void setAttributes(const Attributes& a) = 0;
  virtual void setTimeout(int ms) = 0;
  virtual bool open() = 0;
  virtual void close() = 0;

  bool isOpen() const;
  bool isError() const;
  std::string getError() const;

  virtual bool write(const char* buffer, int size) = 0;
  virtual bool write(std::string data);
  virtual bool write(std::vector<char> data);

  // return: number of bytes read, 0: error or timeout
  // always check isError() after read
  virtual int read(char* buffer, int size) = 0;
  virtual int read(char* buffer, int size, char stopChar);

  // always check isError() after read
  virtual std::string read();
  virtual std::string read(char stopChar);

  virtual bool sendBreak() = 0;

protected:
  std::string serialDevice;
  Attributes attributes;
  bool flagOpen;
  std::string error;

private:
  SerialPort(const SerialPort&);
  SerialPort& operator=(const SerialPort&);
};


#endif

