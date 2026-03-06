/****************************************************************************
** Meta object code from reading C++ file 'map_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../../src/rviz/src/rviz/default_plugin/map_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'map_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz__MapDisplay_t {
    QByteArrayData data[8];
    char stringdata0[95];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__MapDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__MapDisplay_t qt_meta_stringdata_rviz__MapDisplay = {
    {
QT_MOC_LITERAL(0, 0, 16), // "rviz::MapDisplay"
QT_MOC_LITERAL(1, 17, 11), // "updateAlpha"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 11), // "updateTopic"
QT_MOC_LITERAL(4, 42, 15), // "updateDrawUnder"
QT_MOC_LITERAL(5, 58, 13), // "updatePalette"
QT_MOC_LITERAL(6, 72, 9), // "updateMap"
QT_MOC_LITERAL(7, 82, 12) // "transformMap"

    },
    "rviz::MapDisplay\0updateAlpha\0\0updateTopic\0"
    "updateDrawUnder\0updatePalette\0updateMap\0"
    "transformMap"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__MapDisplay[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x09 /* Protected */,
       3,    0,   45,    2, 0x09 /* Protected */,
       4,    0,   46,    2, 0x09 /* Protected */,
       5,    0,   47,    2, 0x09 /* Protected */,
       6,    0,   48,    2, 0x09 /* Protected */,
       7,    0,   49,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz::MapDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MapDisplay *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->updateAlpha(); break;
        case 1: _t->updateTopic(); break;
        case 2: _t->updateDrawUnder(); break;
        case 3: _t->updatePalette(); break;
        case 4: _t->updateMap(); break;
        case 5: _t->transformMap(); break;
        default: ;
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject rviz::MapDisplay::staticMetaObject = { {
    QMetaObject::SuperData::link<Display::staticMetaObject>(),
    qt_meta_stringdata_rviz__MapDisplay.data,
    qt_meta_data_rviz__MapDisplay,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz::MapDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::MapDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__MapDisplay.stringdata0))
        return static_cast<void*>(this);
    return Display::qt_metacast(_clname);
}

int rviz::MapDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Display::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
