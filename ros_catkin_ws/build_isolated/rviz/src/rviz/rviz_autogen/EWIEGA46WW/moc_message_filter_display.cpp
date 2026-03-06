/****************************************************************************
** Meta object code from reading C++ file 'message_filter_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../src/rviz/src/rviz/message_filter_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'message_filter_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz___RosTopicDisplay_t {
    QByteArrayData data[7];
    char stringdata0[123];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz___RosTopicDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz___RosTopicDisplay_t qt_meta_stringdata_rviz___RosTopicDisplay = {
    {
QT_MOC_LITERAL(0, 0, 22), // "rviz::_RosTopicDisplay"
QT_MOC_LITERAL(1, 23, 11), // "updateTopic"
QT_MOC_LITERAL(2, 35, 0), // ""
QT_MOC_LITERAL(3, 36, 15), // "updateQueueSize"
QT_MOC_LITERAL(4, 52, 24), // "processTypeErasedMessage"
QT_MOC_LITERAL(5, 77, 29), // "boost::shared_ptr<const void>"
QT_MOC_LITERAL(6, 107, 15) // "type_erased_msg"

    },
    "rviz::_RosTopicDisplay\0updateTopic\0\0"
    "updateQueueSize\0processTypeErasedMessage\0"
    "boost::shared_ptr<const void>\0"
    "type_erased_msg"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz___RosTopicDisplay[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   29,    2, 0x09 /* Protected */,
       3,    0,   30,    2, 0x09 /* Protected */,
       4,    1,   31,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5,    6,

       0        // eod
};

void rviz::_RosTopicDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<_RosTopicDisplay *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->updateTopic(); break;
        case 1: _t->updateQueueSize(); break;
        case 2: _t->processTypeErasedMessage((*reinterpret_cast< boost::shared_ptr<const void>(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 2:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< boost::shared_ptr<const void> >(); break;
            }
            break;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rviz::_RosTopicDisplay::staticMetaObject = { {
    QMetaObject::SuperData::link<Display::staticMetaObject>(),
    qt_meta_stringdata_rviz___RosTopicDisplay.data,
    qt_meta_data_rviz___RosTopicDisplay,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz::_RosTopicDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::_RosTopicDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz___RosTopicDisplay.stringdata0))
        return static_cast<void*>(this);
    return Display::qt_metacast(_clname);
}

int rviz::_RosTopicDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Display::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
