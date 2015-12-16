#include "joystick.h"

Joystick::Joystick(QObject *parent) :
    QThread(parent)
{

    mConnected = false;
    mAxis_count = 0;
    mButton_count = 0;
    mName[0] = '\0';

    qRegisterMetaType<JoystickErrorType>("JoystickErrorType");

}

Joystick::~Joystick()
{
    stop();
}

Joystick::Joystick( QString& joydev, QObject *parent ) :
    QThread(parent)
{
    init( joydev );
}

void Joystick::errorSlot(int error, JoystickErrorType errorType)
{
    Q_UNUSED(error);
    Q_UNUSED(errorType);
    stop();
}

int Joystick::init( QString& joydev )
{
    return 0;
}

void Joystick::stop()
{
}

char Joystick::getButton( int button )
{
    if(button < (mButton_count))
    {
        QMutexLocker locker(&mMutex);
        return (mButtons)[button];
    }
    return -1;
}

int Joystick::getAxis( int axis )
{
    if(axis < mAxis_count)
    {
        QMutexLocker locker(&mMutex);
        return mAxes[axis];
    }
    return -65535;
}

QString Joystick::getName()
{
    return QString(mName);
}

QString Joystick::getDevice()
{
    return mDevice;
}

int Joystick::numButtons()
{
    return mButton_count;
}

int Joystick::numAxes()
{
    return mAxis_count;
}

bool Joystick::isConnected()
{
    return mConnected;
}

void Joystick::run()
{

}

