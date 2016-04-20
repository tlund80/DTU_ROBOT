/****************************************************************************
** Meta object code from reading C++ file 'SMColorCalibrationWorker.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/SMColorCalibrationWorker.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SMColorCalibrationWorker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SMColorCalibrationWorker[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      56,   26,   25,   25, 0x05,
      97,   93,   25,   25, 0x05,
     118,   25,   25,   25, 0x05,

 // slots: signature, parameters, type, tag, flags
     141,  125,   25,   25, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SMColorCalibrationWorker[] = {
    "SMColorCalibrationWorker\0\0"
    "idx,camID,success,frameResult\0"
    "newFrameResult(int,int,bool,cv::Mat)\0"
    "idx\0newSetProcessed(int)\0done()\0"
    "calibrationData\0"
    "performCalibration(std::vector<SMCalibrationSet>)\0"
};

void SMColorCalibrationWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SMColorCalibrationWorker *_t = static_cast<SMColorCalibrationWorker *>(_o);
        switch (_id) {
        case 0: _t->newFrameResult((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< cv::Mat(*)>(_a[4]))); break;
        case 1: _t->newSetProcessed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->done(); break;
        case 3: _t->performCalibration((*reinterpret_cast< std::vector<SMCalibrationSet>(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SMColorCalibrationWorker::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SMColorCalibrationWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_SMColorCalibrationWorker,
      qt_meta_data_SMColorCalibrationWorker, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SMColorCalibrationWorker::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SMColorCalibrationWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SMColorCalibrationWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SMColorCalibrationWorker))
        return static_cast<void*>(const_cast< SMColorCalibrationWorker*>(this));
    return QObject::qt_metacast(_clname);
}

int SMColorCalibrationWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void SMColorCalibrationWorker::newFrameResult(int _t1, int _t2, bool _t3, cv::Mat _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SMColorCalibrationWorker::newSetProcessed(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void SMColorCalibrationWorker::done()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}
QT_END_MOC_NAMESPACE
