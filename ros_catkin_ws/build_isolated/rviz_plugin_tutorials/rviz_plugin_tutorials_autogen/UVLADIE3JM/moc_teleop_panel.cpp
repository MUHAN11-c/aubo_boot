/****************************************************************************
** Meta object code from reading C++ file 'teleop_panel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/visualization_tutorials/rviz_plugin_tutorials/src/teleop_panel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'teleop_panel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz_plugin_tutorials__TeleopPanel_t {
    QByteArrayData data[9];
    char stringdata0[113];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz_plugin_tutorials__TeleopPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz_plugin_tutorials__TeleopPanel_t qt_meta_stringdata_rviz_plugin_tutorials__TeleopPanel = {
    {
QT_MOC_LITERAL(0, 0, 34), // "rviz_plugin_tutorials::Teleop..."
QT_MOC_LITERAL(1, 35, 6), // "setVel"
QT_MOC_LITERAL(2, 42, 0), // ""
QT_MOC_LITERAL(3, 43, 16), // "linear_velocity_"
QT_MOC_LITERAL(4, 60, 17), // "angular_velocity_"
QT_MOC_LITERAL(5, 78, 8), // "setTopic"
QT_MOC_LITERAL(6, 87, 5), // "topic"
QT_MOC_LITERAL(7, 93, 7), // "sendVel"
QT_MOC_LITERAL(8, 101, 11) // "updateTopic"

    },
    "rviz_plugin_tutorials::TeleopPanel\0"
    "setVel\0\0linear_velocity_\0angular_velocity_\0"
    "setTopic\0topic\0sendVel\0updateTopic"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz_plugin_tutorials__TeleopPanel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    2,   34,    2, 0x0a /* Public */,
       5,    1,   39,    2, 0x0a /* Public */,
       7,    0,   42,    2, 0x09 /* Protected */,
       8,    0,   43,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void, QMetaType::Float, QMetaType::Float,    3,    4,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz_plugin_tutorials::TeleopPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<TeleopPanel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->setVel((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 1: _t->setTopic((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->sendVel(); break;
        case 3: _t->updateTopic(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rviz_plugin_tutorials::TeleopPanel::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz::Panel::staticMetaObject>(),
    qt_meta_stringdata_rviz_plugin_tutorials__TeleopPanel.data,
    qt_meta_data_rviz_plugin_tutorials__TeleopPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz_plugin_tutorials::TeleopPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz_plugin_tutorials::TeleopPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz_plugin_tutorials__TeleopPanel.stringdata0))
        return static_cast<void*>(this);
    return rviz::Panel::qt_metacast(_clname);
}

int rviz_plugin_tutorials::TeleopPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
