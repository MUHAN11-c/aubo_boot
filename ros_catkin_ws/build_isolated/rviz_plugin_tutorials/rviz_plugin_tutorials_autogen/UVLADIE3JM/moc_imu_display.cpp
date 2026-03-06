/****************************************************************************
** Meta object code from reading C++ file 'imu_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/visualization_tutorials/rviz_plugin_tutorials/src/imu_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'imu_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz_plugin_tutorials__ImuDisplay_t {
    QByteArrayData data[4];
    char stringdata0[75];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz_plugin_tutorials__ImuDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz_plugin_tutorials__ImuDisplay_t qt_meta_stringdata_rviz_plugin_tutorials__ImuDisplay = {
    {
QT_MOC_LITERAL(0, 0, 33), // "rviz_plugin_tutorials::ImuDis..."
QT_MOC_LITERAL(1, 34, 19), // "updateColorAndAlpha"
QT_MOC_LITERAL(2, 54, 0), // ""
QT_MOC_LITERAL(3, 55, 19) // "updateHistoryLength"

    },
    "rviz_plugin_tutorials::ImuDisplay\0"
    "updateColorAndAlpha\0\0updateHistoryLength"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz_plugin_tutorials__ImuDisplay[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x08 /* Private */,
       3,    0,   25,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz_plugin_tutorials::ImuDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ImuDisplay *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->updateColorAndAlpha(); break;
        case 1: _t->updateHistoryLength(); break;
        default: ;
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject rviz_plugin_tutorials::ImuDisplay::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz::MessageFilterDisplay<sensor_msgs::Imu>::staticMetaObject>(),
    qt_meta_stringdata_rviz_plugin_tutorials__ImuDisplay.data,
    qt_meta_data_rviz_plugin_tutorials__ImuDisplay,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz_plugin_tutorials::ImuDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz_plugin_tutorials::ImuDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz_plugin_tutorials__ImuDisplay.stringdata0))
        return static_cast<void*>(this);
    return rviz::MessageFilterDisplay<sensor_msgs::Imu>::qt_metacast(_clname);
}

int rviz_plugin_tutorials::ImuDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::MessageFilterDisplay<sensor_msgs::Imu>::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
