
// default includes
#include <shiboken.h>
#ifndef QT_NO_VERSION_TAGGING
#  define QT_NO_VERSION_TAGGING
#endif
#include <QDebug>
#include <pysidesignal.h>
#include <pysideproperty.h>
#include <pyside.h>
#include <pysideqenum.h>
#include <feature_select.h>
#include <qapp_macro.h>

QT_WARNING_DISABLE_DEPRECATED

#include <typeinfo>
#include <iterator>

// module include
#include "libqt_gui_cpp_shiboken_python.h"

// main header
#include "qt_gui_cpp_wrapper.h"

// inner classes
#include "qt_gui_cpp_pluginprovider_wrapper.h"
#include "qt_gui_cpp_rospluginlibpluginprovider_forplugins_wrapper.h"
#include "qt_gui_cpp_rospluginlibpluginprovider_forpluginproviders_wrapper.h"
#include "qt_gui_cpp_compositepluginprovider_wrapper.h"
#include "qt_gui_cpp_recursivepluginprovider_wrapper.h"
#include "qt_gui_cpp_plugin_wrapper.h"
#include "qt_gui_cpp_genericproxy_wrapper.h"
#include "qt_gui_cpp_settings_wrapper.h"
#include "qt_gui_cpp_plugincontext_wrapper.h"
#include "qt_gui_cpp_pluginbridge_wrapper.h"
#include "qt_gui_cpp_plugindescriptor_wrapper.h"

#include <cctype>
#include <cstring>



template <class T>
static const char *typeNameOf(const T &t)
{
    const char *typeName =  typeid(t).name();
    auto size = std::strlen(typeName);
#if defined(Q_CC_MSVC) // MSVC: "class QPaintDevice * __ptr64"
    if (auto lastStar = strchr(typeName, '*')) {
        // MSVC: "class QPaintDevice * __ptr64"
        while (*--lastStar == ' ') {
        }
        size = lastStar - typeName + 1;
    }
#else // g++, Clang: "QPaintDevice *" -> "P12QPaintDevice"
    if (size > 2 && typeName[0] == 'P' && std::isdigit(typeName[1])) {
        ++typeName;
        --size;
    }
#endif
    char *result = new char[size + 1];
    result[size] = '\0';
    memcpy(result, typeName, size);
    return result;
}


// Target ---------------------------------------------------------

extern "C" {

static const char *Sbk_qt_gui_cpp_PropertyStrings[] = {
    nullptr // Sentinel
};

static PyMethodDef Sbk_qt_gui_cpp_methods[] = {

    {nullptr, nullptr} // Sentinel
};

} // extern "C"

static int Sbk_qt_gui_cpp_traverse(PyObject *self, visitproc visit, void *arg)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_traverse(self, visit, arg);
}
static int Sbk_qt_gui_cpp_clear(PyObject *self)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_clear(self);
}
// Class Definition -----------------------------------------------
extern "C" {
static SbkObjectType *_Sbk_qt_gui_cpp_Type = nullptr;
static SbkObjectType *Sbk_qt_gui_cpp_TypeF(void)
{
    return _Sbk_qt_gui_cpp_Type;
}

static PyType_Slot Sbk_qt_gui_cpp_slots[] = {
    {Py_tp_base,        nullptr}, // inserted by introduceWrapperType
    {Py_tp_dealloc,     reinterpret_cast<void *>(Sbk_object_dealloc /* PYSIDE-832: Prevent replacement of "0" with subtype_dealloc. */)},
    {Py_tp_repr,        nullptr},
    {Py_tp_hash,        nullptr},
    {Py_tp_call,        nullptr},
    {Py_tp_str,         nullptr},
    {Py_tp_getattro,    nullptr},
    {Py_tp_setattro,    nullptr},
    {Py_tp_traverse,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_traverse)},
    {Py_tp_clear,       reinterpret_cast<void *>(Sbk_qt_gui_cpp_clear)},
    {Py_tp_richcompare, nullptr},
    {Py_tp_iter,        nullptr},
    {Py_tp_iternext,    nullptr},
    {Py_tp_methods,     reinterpret_cast<void *>(Sbk_qt_gui_cpp_methods)},
    {Py_tp_getset,      nullptr},
    {Py_tp_init,        nullptr},
    {Py_tp_new,         reinterpret_cast<void *>(SbkDummyNew /* PYSIDE-595: Prevent replacement of "0" with base->tp_new. */)},
    {0, nullptr}
};
static PyType_Spec Sbk_qt_gui_cpp_spec = {
    "1:libqt_gui_cpp_shiboken.qt_gui_cpp",
    sizeof(SbkObject),
    0,
    Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_CHECKTYPES|Py_TPFLAGS_HAVE_GC,
    Sbk_qt_gui_cpp_slots
};

} //extern "C"


// Type conversion functions.

// The signatures string for the functions.
// Multiple signatures have their index "n:" in front.
static const char *libqt_gui_cpp_shibokenqt_gui_cpp_SignatureStrings[] = {
    nullptr}; // Sentinel

void init_libqt_gui_cpp_shibokenqt_gui_cpp(PyObject *module)
{
    _Sbk_qt_gui_cpp_Type = Shiboken::ObjectType::introduceWrapperType(
        module,
        "qt_gui_cpp",
        "qt_gui_cpp",
        &Sbk_qt_gui_cpp_spec,
        0,
        0,
        0,
        0    );
    
    auto pyType = reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_Type);
    InitSignatureStrings(pyType, libqt_gui_cpp_shibokenqt_gui_cpp_SignatureStrings);
    SbkObjectType_SetPropertyStrings(reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_Type), Sbk_qt_gui_cpp_PropertyStrings);
    Sbklibqt_gui_cpp_shibokenTypes[SBK_libqt_gui_cpp_shibokenQT_GUI_CPP_IDX]
        = reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_TypeF());



}
