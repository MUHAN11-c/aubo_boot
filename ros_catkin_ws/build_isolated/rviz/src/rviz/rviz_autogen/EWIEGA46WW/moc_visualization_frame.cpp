/****************************************************************************
** Meta object code from reading C++ file 'visualization_frame.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../src/rviz/src/rviz/visualization_frame.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'visualization_frame.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz__VisualizationFrame_t {
    QByteArrayData data[55];
    char stringdata0[768];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__VisualizationFrame_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__VisualizationFrame_t qt_meta_stringdata_rviz__VisualizationFrame = {
    {
QT_MOC_LITERAL(0, 0, 24), // "rviz::VisualizationFrame"
QT_MOC_LITERAL(1, 25, 12), // "statusUpdate"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 7), // "message"
QT_MOC_LITERAL(4, 47, 16), // "fullScreenChange"
QT_MOC_LITERAL(5, 64, 6), // "hidden"
QT_MOC_LITERAL(6, 71, 24), // "displayConfigFileChanged"
QT_MOC_LITERAL(7, 96, 8), // "fullpath"
QT_MOC_LITERAL(8, 105, 24), // "setDisplayConfigModified"
QT_MOC_LITERAL(9, 130, 9), // "setStatus"
QT_MOC_LITERAL(10, 140, 13), // "setFullScreen"
QT_MOC_LITERAL(11, 154, 11), // "full_screen"
QT_MOC_LITERAL(12, 166, 14), // "exitFullScreen"
QT_MOC_LITERAL(13, 181, 6), // "onOpen"
QT_MOC_LITERAL(14, 188, 6), // "onSave"
QT_MOC_LITERAL(15, 195, 8), // "onSaveAs"
QT_MOC_LITERAL(16, 204, 11), // "onSaveImage"
QT_MOC_LITERAL(17, 216, 22), // "onRecentConfigSelected"
QT_MOC_LITERAL(18, 239, 10), // "onHelpWiki"
QT_MOC_LITERAL(19, 250, 11), // "onHelpAbout"
QT_MOC_LITERAL(20, 262, 18), // "openNewPanelDialog"
QT_MOC_LITERAL(21, 281, 17), // "openNewToolDialog"
QT_MOC_LITERAL(22, 299, 21), // "openPreferencesDialog"
QT_MOC_LITERAL(23, 321, 13), // "showHelpPanel"
QT_MOC_LITERAL(24, 335, 17), // "onDockPanelChange"
QT_MOC_LITERAL(25, 353, 19), // "onToolbarRemoveTool"
QT_MOC_LITERAL(26, 373, 8), // "QAction*"
QT_MOC_LITERAL(27, 382, 23), // "remove_tool_menu_action"
QT_MOC_LITERAL(28, 406, 17), // "onButtonStyleTool"
QT_MOC_LITERAL(29, 424, 29), // "button_style_tool_menu_action"
QT_MOC_LITERAL(30, 454, 24), // "onToolbarActionTriggered"
QT_MOC_LITERAL(31, 479, 6), // "action"
QT_MOC_LITERAL(32, 486, 7), // "addTool"
QT_MOC_LITERAL(33, 494, 5), // "Tool*"
QT_MOC_LITERAL(34, 500, 4), // "tool"
QT_MOC_LITERAL(35, 505, 17), // "onToolNameChanged"
QT_MOC_LITERAL(36, 523, 4), // "name"
QT_MOC_LITERAL(37, 528, 10), // "removeTool"
QT_MOC_LITERAL(38, 539, 11), // "refreshTool"
QT_MOC_LITERAL(39, 551, 21), // "indicateToolIsCurrent"
QT_MOC_LITERAL(40, 573, 12), // "changeMaster"
QT_MOC_LITERAL(41, 586, 13), // "onDeletePanel"
QT_MOC_LITERAL(42, 600, 15), // "markLoadingDone"
QT_MOC_LITERAL(43, 616, 21), // "setImageSaveDirectory"
QT_MOC_LITERAL(44, 638, 9), // "directory"
QT_MOC_LITERAL(45, 648, 5), // "reset"
QT_MOC_LITERAL(46, 654, 14), // "onPanelDeleted"
QT_MOC_LITERAL(47, 669, 4), // "dock"
QT_MOC_LITERAL(48, 674, 15), // "onHelpDestroyed"
QT_MOC_LITERAL(49, 690, 12), // "hideLeftDock"
QT_MOC_LITERAL(50, 703, 4), // "hide"
QT_MOC_LITERAL(51, 708, 13), // "hideRightDock"
QT_MOC_LITERAL(52, 722, 27), // "onDockPanelVisibilityChange"
QT_MOC_LITERAL(53, 750, 7), // "visible"
QT_MOC_LITERAL(54, 758, 9) // "updateFps"

    },
    "rviz::VisualizationFrame\0statusUpdate\0"
    "\0message\0fullScreenChange\0hidden\0"
    "displayConfigFileChanged\0fullpath\0"
    "setDisplayConfigModified\0setStatus\0"
    "setFullScreen\0full_screen\0exitFullScreen\0"
    "onOpen\0onSave\0onSaveAs\0onSaveImage\0"
    "onRecentConfigSelected\0onHelpWiki\0"
    "onHelpAbout\0openNewPanelDialog\0"
    "openNewToolDialog\0openPreferencesDialog\0"
    "showHelpPanel\0onDockPanelChange\0"
    "onToolbarRemoveTool\0QAction*\0"
    "remove_tool_menu_action\0onButtonStyleTool\0"
    "button_style_tool_menu_action\0"
    "onToolbarActionTriggered\0action\0addTool\0"
    "Tool*\0tool\0onToolNameChanged\0name\0"
    "removeTool\0refreshTool\0indicateToolIsCurrent\0"
    "changeMaster\0onDeletePanel\0markLoadingDone\0"
    "setImageSaveDirectory\0directory\0reset\0"
    "onPanelDeleted\0dock\0onHelpDestroyed\0"
    "hideLeftDock\0hide\0hideRightDock\0"
    "onDockPanelVisibilityChange\0visible\0"
    "updateFps"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__VisualizationFrame[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      38,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  204,    2, 0x06 /* Public */,
       4,    1,  207,    2, 0x06 /* Public */,
       6,    1,  210,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       8,    0,  213,    2, 0x0a /* Public */,
       9,    1,  214,    2, 0x0a /* Public */,
      10,    1,  217,    2, 0x0a /* Public */,
      12,    0,  220,    2, 0x0a /* Public */,
      13,    0,  221,    2, 0x09 /* Protected */,
      14,    0,  222,    2, 0x09 /* Protected */,
      15,    0,  223,    2, 0x09 /* Protected */,
      16,    0,  224,    2, 0x09 /* Protected */,
      17,    0,  225,    2, 0x09 /* Protected */,
      18,    0,  226,    2, 0x09 /* Protected */,
      19,    0,  227,    2, 0x09 /* Protected */,
      20,    0,  228,    2, 0x09 /* Protected */,
      21,    0,  229,    2, 0x09 /* Protected */,
      22,    0,  230,    2, 0x09 /* Protected */,
      23,    0,  231,    2, 0x09 /* Protected */,
      24,    0,  232,    2, 0x09 /* Protected */,
      25,    1,  233,    2, 0x09 /* Protected */,
      28,    1,  236,    2, 0x09 /* Protected */,
      30,    1,  239,    2, 0x09 /* Protected */,
      32,    1,  242,    2, 0x09 /* Protected */,
      35,    1,  245,    2, 0x09 /* Protected */,
      37,    1,  248,    2, 0x09 /* Protected */,
      38,    1,  251,    2, 0x09 /* Protected */,
      39,    1,  254,    2, 0x09 /* Protected */,
      40,    0,  257,    2, 0x09 /* Protected */,
      41,    0,  258,    2, 0x09 /* Protected */,
      42,    0,  259,    2, 0x09 /* Protected */,
      43,    1,  260,    2, 0x09 /* Protected */,
      45,    0,  263,    2, 0x09 /* Protected */,
      46,    1,  264,    2, 0x09 /* Protected */,
      48,    0,  267,    2, 0x09 /* Protected */,
      49,    1,  268,    2, 0x09 /* Protected */,
      51,    1,  271,    2, 0x09 /* Protected */,
      52,    1,  274,    2, 0x09 /* Protected */,
      54,    0,  277,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::Bool,    5,
    QMetaType::Void, QMetaType::QString,    7,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::Bool,   11,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 26,   27,
    QMetaType::Void, 0x80000000 | 26,   29,
    QMetaType::Void, 0x80000000 | 26,   31,
    QMetaType::Void, 0x80000000 | 33,   34,
    QMetaType::Void, QMetaType::QString,   36,
    QMetaType::Void, 0x80000000 | 33,   34,
    QMetaType::Void, 0x80000000 | 33,   34,
    QMetaType::Void, 0x80000000 | 33,   34,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   44,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QObjectStar,   47,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   50,
    QMetaType::Void, QMetaType::Bool,   50,
    QMetaType::Void, QMetaType::Bool,   53,
    QMetaType::Void,

       0        // eod
};

void rviz::VisualizationFrame::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<VisualizationFrame *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->statusUpdate((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->fullScreenChange((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->displayConfigFileChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->setDisplayConfigModified(); break;
        case 4: _t->setStatus((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 5: _t->setFullScreen((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->exitFullScreen(); break;
        case 7: _t->onOpen(); break;
        case 8: _t->onSave(); break;
        case 9: _t->onSaveAs(); break;
        case 10: _t->onSaveImage(); break;
        case 11: _t->onRecentConfigSelected(); break;
        case 12: _t->onHelpWiki(); break;
        case 13: _t->onHelpAbout(); break;
        case 14: _t->openNewPanelDialog(); break;
        case 15: _t->openNewToolDialog(); break;
        case 16: _t->openPreferencesDialog(); break;
        case 17: _t->showHelpPanel(); break;
        case 18: _t->onDockPanelChange(); break;
        case 19: _t->onToolbarRemoveTool((*reinterpret_cast< QAction*(*)>(_a[1]))); break;
        case 20: _t->onButtonStyleTool((*reinterpret_cast< QAction*(*)>(_a[1]))); break;
        case 21: _t->onToolbarActionTriggered((*reinterpret_cast< QAction*(*)>(_a[1]))); break;
        case 22: _t->addTool((*reinterpret_cast< Tool*(*)>(_a[1]))); break;
        case 23: _t->onToolNameChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 24: _t->removeTool((*reinterpret_cast< Tool*(*)>(_a[1]))); break;
        case 25: _t->refreshTool((*reinterpret_cast< Tool*(*)>(_a[1]))); break;
        case 26: _t->indicateToolIsCurrent((*reinterpret_cast< Tool*(*)>(_a[1]))); break;
        case 27: _t->changeMaster(); break;
        case 28: _t->onDeletePanel(); break;
        case 29: _t->markLoadingDone(); break;
        case 30: _t->setImageSaveDirectory((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 31: _t->reset(); break;
        case 32: _t->onPanelDeleted((*reinterpret_cast< QObject*(*)>(_a[1]))); break;
        case 33: _t->onHelpDestroyed(); break;
        case 34: _t->hideLeftDock((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 35: _t->hideRightDock((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 36: _t->onDockPanelVisibilityChange((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 37: _t->updateFps(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (VisualizationFrame::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&VisualizationFrame::statusUpdate)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (VisualizationFrame::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&VisualizationFrame::fullScreenChange)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (VisualizationFrame::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&VisualizationFrame::displayConfigFileChanged)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rviz::VisualizationFrame::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_rviz__VisualizationFrame.data,
    qt_meta_data_rviz__VisualizationFrame,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz::VisualizationFrame::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::VisualizationFrame::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__VisualizationFrame.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "WindowManagerInterface"))
        return static_cast< WindowManagerInterface*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int rviz::VisualizationFrame::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 38)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 38;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 38)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 38;
    }
    return _id;
}

// SIGNAL 0
void rviz::VisualizationFrame::statusUpdate(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void rviz::VisualizationFrame::fullScreenChange(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void rviz::VisualizationFrame::displayConfigFileChanged(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
