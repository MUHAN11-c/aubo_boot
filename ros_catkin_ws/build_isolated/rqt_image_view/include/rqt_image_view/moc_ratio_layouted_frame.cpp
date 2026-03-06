/****************************************************************************
** Meta object code from reading C++ file 'ratio_layouted_frame.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/rqt_image_view/include/rqt_image_view/ratio_layouted_frame.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ratio_layouted_frame.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rqt_image_view__RatioLayoutedFrame_t {
    QByteArrayData data[8];
    char stringdata0[94];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rqt_image_view__RatioLayoutedFrame_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rqt_image_view__RatioLayoutedFrame_t qt_meta_stringdata_rqt_image_view__RatioLayoutedFrame = {
    {
QT_MOC_LITERAL(0, 0, 34), // "rqt_image_view::RatioLayouted..."
QT_MOC_LITERAL(1, 35, 14), // "delayed_update"
QT_MOC_LITERAL(2, 50, 0), // ""
QT_MOC_LITERAL(3, 51, 9), // "mouseLeft"
QT_MOC_LITERAL(4, 61, 1), // "x"
QT_MOC_LITERAL(5, 63, 1), // "y"
QT_MOC_LITERAL(6, 65, 20), // "onSmoothImageChanged"
QT_MOC_LITERAL(7, 86, 7) // "checked"

    },
    "rqt_image_view::RatioLayoutedFrame\0"
    "delayed_update\0\0mouseLeft\0x\0y\0"
    "onSmoothImageChanged\0checked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rqt_image_view__RatioLayoutedFrame[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   29,    2, 0x06 /* Public */,
       3,    2,   30,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   35,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    4,    5,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    7,

       0        // eod
};

void rqt_image_view::RatioLayoutedFrame::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<RatioLayoutedFrame *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->delayed_update(); break;
        case 1: _t->mouseLeft((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 2: _t->onSmoothImageChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (RatioLayoutedFrame::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&RatioLayoutedFrame::delayed_update)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (RatioLayoutedFrame::*)(int , int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&RatioLayoutedFrame::mouseLeft)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rqt_image_view::RatioLayoutedFrame::staticMetaObject = { {
    QMetaObject::SuperData::link<QFrame::staticMetaObject>(),
    qt_meta_stringdata_rqt_image_view__RatioLayoutedFrame.data,
    qt_meta_data_rqt_image_view__RatioLayoutedFrame,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rqt_image_view::RatioLayoutedFrame::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rqt_image_view::RatioLayoutedFrame::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rqt_image_view__RatioLayoutedFrame.stringdata0))
        return static_cast<void*>(this);
    return QFrame::qt_metacast(_clname);
}

int rqt_image_view::RatioLayoutedFrame::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void rqt_image_view::RatioLayoutedFrame::delayed_update()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void rqt_image_view::RatioLayoutedFrame::mouseLeft(int _t1, int _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
