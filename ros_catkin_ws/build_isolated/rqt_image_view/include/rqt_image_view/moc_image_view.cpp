/****************************************************************************
** Meta object code from reading C++ file 'image_view.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/rqt_image_view/include/rqt_image_view/image_view.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'image_view.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rqt_image_view__ImageView_t {
    QByteArrayData data[20];
    char stringdata0[245];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rqt_image_view__ImageView_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rqt_image_view__ImageView_t qt_meta_stringdata_rqt_image_view__ImageView = {
    {
QT_MOC_LITERAL(0, 0, 25), // "rqt_image_view::ImageView"
QT_MOC_LITERAL(1, 26, 18), // "setColorSchemeList"
QT_MOC_LITERAL(2, 45, 0), // ""
QT_MOC_LITERAL(3, 46, 15), // "updateTopicList"
QT_MOC_LITERAL(4, 62, 14), // "onTopicChanged"
QT_MOC_LITERAL(5, 77, 5), // "index"
QT_MOC_LITERAL(6, 83, 7), // "onZoom1"
QT_MOC_LITERAL(7, 91, 7), // "checked"
QT_MOC_LITERAL(8, 99, 14), // "onDynamicRange"
QT_MOC_LITERAL(9, 114, 9), // "saveImage"
QT_MOC_LITERAL(10, 124, 18), // "updateNumGridlines"
QT_MOC_LITERAL(11, 143, 14), // "onMousePublish"
QT_MOC_LITERAL(12, 158, 11), // "onMouseLeft"
QT_MOC_LITERAL(13, 170, 1), // "x"
QT_MOC_LITERAL(14, 172, 1), // "y"
QT_MOC_LITERAL(15, 174, 17), // "onPubTopicChanged"
QT_MOC_LITERAL(16, 192, 20), // "onHideToolbarChanged"
QT_MOC_LITERAL(17, 213, 4), // "hide"
QT_MOC_LITERAL(18, 218, 12), // "onRotateLeft"
QT_MOC_LITERAL(19, 231, 13) // "onRotateRight"

    },
    "rqt_image_view::ImageView\0setColorSchemeList\0"
    "\0updateTopicList\0onTopicChanged\0index\0"
    "onZoom1\0checked\0onDynamicRange\0saveImage\0"
    "updateNumGridlines\0onMousePublish\0"
    "onMouseLeft\0x\0y\0onPubTopicChanged\0"
    "onHideToolbarChanged\0hide\0onRotateLeft\0"
    "onRotateRight"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rqt_image_view__ImageView[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   79,    2, 0x09 /* Protected */,
       3,    0,   80,    2, 0x09 /* Protected */,
       4,    1,   81,    2, 0x09 /* Protected */,
       6,    1,   84,    2, 0x09 /* Protected */,
       8,    1,   87,    2, 0x09 /* Protected */,
       9,    0,   90,    2, 0x09 /* Protected */,
      10,    0,   91,    2, 0x09 /* Protected */,
      11,    1,   92,    2, 0x09 /* Protected */,
      12,    2,   95,    2, 0x09 /* Protected */,
      15,    0,  100,    2, 0x09 /* Protected */,
      16,    1,  101,    2, 0x09 /* Protected */,
      18,    0,  104,    2, 0x09 /* Protected */,
      19,    0,  105,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Bool,    7,
    QMetaType::Void, QMetaType::Bool,    7,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    7,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,   13,   14,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   17,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rqt_image_view::ImageView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ImageView *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->setColorSchemeList(); break;
        case 1: _t->updateTopicList(); break;
        case 2: _t->onTopicChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->onZoom1((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->onDynamicRange((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->saveImage(); break;
        case 6: _t->updateNumGridlines(); break;
        case 7: _t->onMousePublish((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->onMouseLeft((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 9: _t->onPubTopicChanged(); break;
        case 10: _t->onHideToolbarChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 11: _t->onRotateLeft(); break;
        case 12: _t->onRotateRight(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rqt_image_view::ImageView::staticMetaObject = { {
    QMetaObject::SuperData::link<rqt_gui_cpp::Plugin::staticMetaObject>(),
    qt_meta_stringdata_rqt_image_view__ImageView.data,
    qt_meta_data_rqt_image_view__ImageView,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rqt_image_view::ImageView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rqt_image_view::ImageView::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rqt_image_view__ImageView.stringdata0))
        return static_cast<void*>(this);
    return rqt_gui_cpp::Plugin::qt_metacast(_clname);
}

int rqt_image_view::ImageView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rqt_gui_cpp::Plugin::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
