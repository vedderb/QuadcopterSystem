
#include "serialport.h"


SerialPort::SerialPort(QObject *parent) :
    QThread(parent)
{

}

SerialPort::~SerialPort()
{
}

int SerialPort::openPort(
        const QString& port,
        int baudrate,
        SerialDataBits dataBits,
        SerialStopBits stopBits,
        SerialParity parity)
{

    return 0;
}

bool SerialPort::isOpen()
{
    return mIsOpen;
}

void SerialPort::closePort()
{

}

bool SerialPort::setBaudrate(int baudrate)
{
      return true;
}

bool SerialPort::setDataBits(SerialDataBits dataBits)
{

    return true;
}

bool SerialPort::setStopBits(SerialStopBits stopBits)
{

    return true;
}

bool SerialPort::setParity(SerialParity parity)
{

    return true;
}

bool SerialPort::readByte(char &byte)
{
        return true;
}

int SerialPort::readBytes(char* buffer, int bytes)
{

    return bytes;
}

int SerialPort::readString(QString& string, int length)
{


    return length;
}

QByteArray SerialPort::readAll()
{

    return 0;
}

int SerialPort::writeData(const char *data, int length, bool block)
{
    return 0;
}

bool SerialPort::writeByte(char byte, bool block)
{

    return false;
}

int SerialPort::bytesAvailable()
{
    return 0;
}

bool SerialPort::writeString(const QString& string, bool block)
{
    return true;
}

int SerialPort::captureBytes(char *buffer, int num, int timeoutMs, const QString& preTransmit)
{
  return 0;
}

int SerialPort::captureBytes(char *buffer, int num, int timeoutMs, const char *preTransmit, int preTransLen)
{
    return 0;
}

void SerialPort::run()
{

}

