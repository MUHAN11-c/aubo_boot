/****************************************************************************
** Meta object code from reading C++ file 'time_panel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../src/rviz/src/rviz/time_panel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'time_panel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz__TimePanel_t {
    QByteArrayData data[19];
    char stringdata0[194];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__TimePanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__TimePanel_t qt_meta_stringdata_rviz__TimePanel = {
    {
QT_MOC_LITERAL(0, 0, 15), // "rviz::TimePanel"
QT_MOC_LITERAL(1, 16, 12), // "pauseToggled"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 7), // "checked"
QT_MOC_LITERAL(4, 38, 16), // "syncModeSelected"
QT_MOC_LITERAL(5, 55, 5), // "index"
QT_MOC_LITERAL(6, 61, 18), // "syncSourceSelected"
QT_MOC_LITERAL(7, 80, 6), // "update"
QT_MOC_LITERAL(8, 87, 14), // "onDisplayAdded"
QT_MOC_LITERAL(9, 102, 14), // "rviz::Display*"
QT_MOC_LITERAL(10, 117, 7), // "display"
QT_MOC_LITERAL(11, 125, 16), // "onDisplayRemoved"
QT_MOC_LITERAL(12, 142, 12), // "onTimeSignal"
QT_MOC_LITERAL(13, 155, 9), // "ros::Time"
QT_MOC_LITERAL(14, 165, 4), // "time"
QT_MOC_LITERAL(15, 170, 4), // "load"
QT_MOC_LITERAL(16, 175, 6), // "Config"
QT_MOC_LITERAL(17, 182, 6), // "config"
QT_MOC_LITERAL(18, 189, 4) // "save"

    },
    "rviz::TimePanel\0pauseToggled\0\0checked\0"
    "syncModeSelected\0index\0syncSourceSelected\0"
    "update\0onDisplayAdded\0rviz::Display*\0"
    "display\0onDisplayRemoved\0onTimeSignal\0"
    "ros::Time\0time\0load\0Config\0config\0"
    "save"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__TimePanel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   59,    2, 0x09 /* Protected */,
       4,    1,   62,    2, 0x09 /* Protected */,
       6,    1,   65,    2, 0x09 /* Protected */,
       7,    0,   68,    2, 0x09 /* Protected */,
       8,    1,   69,    2, 0x09 /* Protected */,
      11,    1,   72,    2, 0x09 /* Protected */,
      12,    1,   75,    2, 0x09 /* Protected */,
      15,    1,   78,    2, 0x09 /* Protected */,
      18,    1,   81,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, 0x80000000 | 13,   14,
    QMetaType::Void, 0x80000000 | 16,   17,
    QMetaType::Void, 0x80000000 | 16,   17,

       0        // eod
};

void rviz::TimePanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<TimePanel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->pauseToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->syncModeSelected((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->syncSourceSelected((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->update(); break;
        case 4: _t->onDisplayAdded((*reinterpret_cast< rviz::Display*(*)>(_a[1]))); break;
        case 5: _t->onDisplayRemoved((*reinterpret_cast< rviz::Display*(*)>(_a[1]))); break;
        case 6: _t->onTimeSignal((*reinterpret_cast< ros::Time(*)>(_a[1]))); break;
        case 7: _t->load((*reinterpret_cast< const Config(*)>(_a[1]))); break;
        case 8: _t->save((*reinterpret_cast< Config(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rviz::TimePanel::staticMetaObject = { {
    QMetaObject::SuperData::link<Panel::staticMetaObject>(),
    qt_meta_stringdata_rviz__TimePanel.data,
    qt_meta_data_rviz__TimePanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz::TimePanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::TimePanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__TimePanel.stringdata0))
        return static_cast<void*>(this);
    return Panel::qt_metacast(_clname);
}

int rviz::TimePanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
