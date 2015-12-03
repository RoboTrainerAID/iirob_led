

#include "SerialPort.h"



SerialPort::Attributes::Attributes() {
  baudRate = BaudRate_9600;
  flowControl = FlowControl_off;
  dataBits = DataBits_8;
  stopBits = StopBits_1;
  timeout = 1000;
}


SerialPort::SerialPort(std::string device) {
  serialDevice = device;
  flagOpen = false;
}




SerialPort::~SerialPort() {
}


void SerialPort::setBaudRate(SerialPort::BaudRate newBaudrate)
{
	attributes.baudRate = newBaudrate;
	if (isOpen()) setAttributes(attributes);
}

SerialPort::Attributes SerialPort::getAttributes() const {
  return attributes;
}


bool SerialPort::isOpen() const {
  return flagOpen;
}


bool SerialPort::isError() const {
  return !error.empty();
}


std::string SerialPort::getError() const {
  return error;
}


bool SerialPort::write(std::string data) {
  return write(data.c_str(), data.length());
}

bool SerialPort::write(std::vector<char> data) {
  std::string s(data.begin(),data.end());
  return write(s);
}

int SerialPort::read(char* buffer, int size, char stopChar) {
  int i = 0;
  while(i < size) {
    int n = read(&buffer[i], 1);
    if (n <= 0) {
      if (isError()) return 0;
      return i;
    }
    i++;
    if (buffer[i - 1] == stopChar) return i;
  }
  return i;
}


#define STRING_READ_BUFFER_SIZE 1024


std::string SerialPort::read() {
  char buffer[STRING_READ_BUFFER_SIZE];
  std::string s;
  while(true) {
    int n = read(buffer, STRING_READ_BUFFER_SIZE);
    if (n <= 0) {
      if (isError()) return "";
    } else {
		s += std::string(buffer, n);
    }
    if (n != STRING_READ_BUFFER_SIZE) break;  // this is just a wild guess that there is more data available
  }
  return s;
}

std::string SerialPort::read(char stopChar) {
  char c;
  std::string s;
  while(true) {
    int n = read(&c, 1);
    if (n <= 0) {
      if (isError()) return "";
      return s;
    }
    s += std::string(1, c);
    if (c == stopChar) return s;
  }
}
