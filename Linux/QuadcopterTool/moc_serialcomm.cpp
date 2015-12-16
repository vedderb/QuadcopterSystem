/****************************************************************************
** Meta object code from reading C++ file 'serialcomm.h'
**
** Created: Fri Jan 31 16:00:33 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "serialcomm.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'serialcomm.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SerialComm[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: signature, parameters, type, tag, flags
      23,   12,   11,   11, 0x05,
      81,   71,   11,   11, 0x05,
     125,  114,   11,   11, 0x05,
     155,  151,   11,   11, 0x05,
     178,   71,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
     208,   11,   11,   11, 0x08,
     235,  230,   11,   11, 0x08,
     265,  230,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_SerialComm[] = {
    "SerialComm\0\0ctrl_param\0"
    "controlParametersReceived(CONTROL_PARAMETERS_t)\0"
    "pos_state\0orientationReceived(POS_STATE_t)\0"
    "imu_values\0rawIMUReceived(RAW_IMU_t)\0"
    "str\0printReceived(QString)\0"
    "positionReceived(POS_STATE_t)\0"
    "serialDataAvailable()\0data\0"
    "packetDataToSend(QByteArray&)\0"
    "packetReceived(QByteArray)\0"
};

void SerialComm::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SerialComm *_t = static_cast<SerialComm *>(_o);
        switch (_id) {
        case 0: _t->controlParametersReceived((*reinterpret_cast< CONTROL_PARAMETERS_t(*)>(_a[1]))); break;
        case 1: _t->orientationReceived((*reinterpret_cast< POS_STATE_t(*)>(_a[1]))); break;
        case 2: _t->rawIMUReceived((*reinterpret_cast< RAW_IMU_t(*)>(_a[1]))); break;
        case 3: _t->printReceived((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->positionReceived((*reinterpret_cast< POS_STATE_t(*)>(_a[1]))); break;
        case 5: _t->serialDataAvailable(); break;
        case 6: _t->packetDataToSend((*reinterpret_cast< QByteArray(*)>(_a[1]))); break;
        case 7: _t->packetReceived((*reinterpret_cast< QByteArray(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SerialComm::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SerialComm::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_SerialComm,
      qt_meta_data_SerialComm, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SerialComm::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SerialComm::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SerialComm::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SerialComm))
        return static_cast<void*>(const_cast< SerialComm*>(this));
    return QObject::qt_metacast(_clname);
}

int SerialComm::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void SerialComm::controlParametersReceived(CONTROL_PARAMETERS_t _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SerialComm::orientationReceived(POS_STATE_t _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void SerialComm::rawIMUReceived(RAW_IMU_t _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void SerialComm::printReceived(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void SerialComm::positionReceived(POS_STATE_t _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_END_MOC_NAMESPACE
