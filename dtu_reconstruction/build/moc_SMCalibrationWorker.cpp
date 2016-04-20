/****************************************************************************
** Meta object code from reading C++ file 'SMCalibrationWorker.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/SMCalibrationWorker.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SMCalibrationWorker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SMCalibrationWorker[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      51,   21,   20,   20, 0x05,
      92,   88,   20,   20, 0x05,
     113,   20,   20,   20, 0x05,

 // slots: signature, parameters, type, tag, flags
     146,  120,   20,   20, 0x0a,
     224,  208,   20,   20, 0x2a,

       0        // eod
};

static const char qt_meta_stringdata_SMCalibrationWorker[] = {
    "SMCalibrationWorker\0\0idx,camID,success,frameResult\0"
    "newFrameResult(int,int,bool,cv::Mat)\0"
    "idx\0newSetProcessed(int)\0done()\0"
    "calibrationData,file_path\0"
    "performCalibration(std::vector<SMCalibrationSet>,std::string)\0"
    "calibrationData\0"
    "performCalibration(std::vector<SMCalibrationSet>)\0"
};

void SMCalibrationWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SMCalibrationWorker *_t = static_cast<SMCalibrationWorker *>(_o);
        switch (_id) {
        case 0: _t->newFrameResult((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< cv::Mat(*)>(_a[4]))); break;
        case 1: _t->newSetProcessed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->done(); break;
        case 3: _t->performCalibration((*reinterpret_cast< std::vector<SMCalibrationSet>(*)>(_a[1])),(*reinterpret_cast< std::string(*)>(_a[2]))); break;
        case 4: _t->performCalibration((*reinterpret_cast< std::vector<SMCalibrationSet>(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SMCalibrationWorker::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SMCalibrationWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_SMCalibrationWorker,
      qt_meta_data_SMCalibrationWorker, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SMCalibrationWorker::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SMCalibrationWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SMCalibrationWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SMCalibrationWorker))
        return static_cast<void*>(const_cast< SMCalibrationWorker*>(this));
    return QObject::qt_metacast(_clname);
}

int SMCalibrationWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void SMCalibrationWorker::newFrameResult(int _t1, int _t2, bool _t3, cv::Mat _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SMCalibrationWorker::newSetProcessed(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void SMCalibrationWorker::done()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}
QT_END_MOC_NAMESPACE
