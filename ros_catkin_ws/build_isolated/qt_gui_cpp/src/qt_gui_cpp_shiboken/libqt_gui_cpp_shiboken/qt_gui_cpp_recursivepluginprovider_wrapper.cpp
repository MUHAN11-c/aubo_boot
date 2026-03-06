
//workaround to access protected functions
#define protected public

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
#include "qt_gui_cpp_recursivepluginprovider_wrapper.h"

// inner classes

#include <cctype>
#include <cstring>

using namespace qt_gui_cpp;



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

// Native ---------------------------------------------------------

void RecursivePluginProviderWrapper::resetPyMethodCache()
{
    std::fill_n(m_PyMethodCache, sizeof(m_PyMethodCache) / sizeof(m_PyMethodCache[0]), false);
}

RecursivePluginProviderWrapper::RecursivePluginProviderWrapper(qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders * plugin_provider) : qt_gui_cpp::RecursivePluginProvider(plugin_provider)
{
    resetPyMethodCache();
    // ... middle
}

QMap<QString,QString > RecursivePluginProviderWrapper::discover(QObject * discovery_data)
{
    if (m_PyMethodCache[0])
        return this->::qt_gui_cpp::RecursivePluginProvider::discover(discovery_data);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return {};
    static PyObject *nameCache[2] = {};
    static const char *funcName = "discover";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[0] = true;
        return this->::qt_gui_cpp::RecursivePluginProvider::discover(discovery_data);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(N)",
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), discovery_data)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return {};
    }
    // Check return type
    PythonToCppFunc pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], pyResult);
    if (!pythonToCpp) {
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "RecursivePluginProvider.discover", "map", Py_TYPE(pyResult)->tp_name);
        return {};
    }
    ::QMap<QString,QString > cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

QList<qt_gui_cpp::PluginDescriptor* > RecursivePluginProviderWrapper::discover_descriptors(QObject * discovery_data)
{
    if (m_PyMethodCache[1])
        return this->::qt_gui_cpp::CompositePluginProvider::discover_descriptors(discovery_data);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return {};
    static PyObject *nameCache[2] = {};
    static const char *funcName = "discover_descriptors";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[1] = true;
        return this->::qt_gui_cpp::CompositePluginProvider::discover_descriptors(discovery_data);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(N)",
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), discovery_data)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return {};
    }
    // Check return type
    PythonToCppFunc pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QT_GUI_CPP_PLUGINDESCRIPTORPTR_IDX], pyResult);
    if (!pythonToCpp) {
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "RecursivePluginProvider.discover_descriptors", "list", Py_TYPE(pyResult)->tp_name);
        return {};
    }
    ::QList<qt_gui_cpp::PluginDescriptor* > cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

void * RecursivePluginProviderWrapper::load(const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context)
{
    if (m_PyMethodCache[2])
        return this->::qt_gui_cpp::CompositePluginProvider::load(plugin_id, plugin_context);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return nullptr;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "load";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[2] = true;
        return this->::qt_gui_cpp::CompositePluginProvider::load(plugin_id, plugin_context);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(NN)",
    Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], &plugin_id),
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]), plugin_context)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return nullptr;
    }
    // Check return type
    PythonToCppFunc pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(Shiboken::Conversions::PrimitiveTypeConverter<void *>(), pyResult);
    if (!pythonToCpp) {
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "RecursivePluginProvider.load", reinterpret_cast<PyTypeObject *>(Shiboken::SbkType< void >())->tp_name, Py_TYPE(pyResult)->tp_name);
        return nullptr;
    }
    void* cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

qt_gui_cpp::Plugin * RecursivePluginProviderWrapper::load_plugin(const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context)
{
    if (m_PyMethodCache[3])
        return this->::qt_gui_cpp::CompositePluginProvider::load_plugin(plugin_id, plugin_context);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return nullptr;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "load_plugin";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[3] = true;
        return this->::qt_gui_cpp::CompositePluginProvider::load_plugin(plugin_id, plugin_context);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(NN)",
    Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], &plugin_id),
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]), plugin_context)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return nullptr;
    }
    // Check return type
    PythonToCppFunc pythonToCpp = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX]), pyResult);
    if (!pythonToCpp) {
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "RecursivePluginProvider.load_plugin", reinterpret_cast<PyTypeObject *>(Shiboken::SbkType< qt_gui_cpp::Plugin >())->tp_name, Py_TYPE(pyResult)->tp_name);
        return nullptr;
    }
    ::qt_gui_cpp::Plugin *cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

void RecursivePluginProviderWrapper::set_plugin_providers(const QList<qt_gui_cpp::PluginProvider* > & plugin_providers)
{
    if (m_PyMethodCache[4]) {
        return this->::qt_gui_cpp::CompositePluginProvider::set_plugin_providers(plugin_providers);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "set_plugin_providers";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[4] = true;
        return this->::qt_gui_cpp::CompositePluginProvider::set_plugin_providers(plugin_providers);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(N)",
    Shiboken::Conversions::copyToPython(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QT_GUI_CPP_PLUGINPROVIDERPTR_IDX], &plugin_providers)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

void RecursivePluginProviderWrapper::shutdown()
{
    if (m_PyMethodCache[5]) {
        return this->::qt_gui_cpp::CompositePluginProvider::shutdown();
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "shutdown";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[5] = true;
        return this->::qt_gui_cpp::CompositePluginProvider::shutdown();
    }

    Shiboken::AutoDecRef pyArgs(PyTuple_New(0));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

void RecursivePluginProviderWrapper::unload(void * plugin_instance)
{
    if (m_PyMethodCache[6]) {
        return this->::qt_gui_cpp::CompositePluginProvider::unload(plugin_instance);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "unload";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[6] = true;
        return this->::qt_gui_cpp::CompositePluginProvider::unload(plugin_instance);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(N)",
    Shiboken::Conversions::copyToPython(Shiboken::Conversions::PrimitiveTypeConverter<void *>(), plugin_instance)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

void RecursivePluginProviderWrapper::unload_plugin(qt_gui_cpp::Plugin * plugin_instance)
{
    if (m_PyMethodCache[7]) {
        return this->::qt_gui_cpp::PluginProvider::unload_plugin(plugin_instance);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "unload_plugin";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[7] = true;
        return this->::qt_gui_cpp::PluginProvider::unload_plugin(plugin_instance);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(N)",
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX]), plugin_instance)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

RecursivePluginProviderWrapper::~RecursivePluginProviderWrapper()
{
    SbkObject *wrapper = Shiboken::BindingManager::instance().retrieveWrapper(this);
    Shiboken::Object::destroy(wrapper, this);
}

// Target ---------------------------------------------------------

extern "C" {
static int
Sbk_qt_gui_cpp_RecursivePluginProvider_Init(PyObject *self, PyObject *args, PyObject *kwds)
{
    SbkObject *sbkSelf = reinterpret_cast<SbkObject *>(self);
    if (Shiboken::Object::isUserType(self) && !Shiboken::ObjectType::canCallConstructor(self->ob_type, Shiboken::SbkType< ::qt_gui_cpp::RecursivePluginProvider >()))
        return -1;

    ::RecursivePluginProviderWrapper *cptr{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0};

    // invalid argument lengths


    if (!PyArg_UnpackTuple(args, "RecursivePluginProvider", 1, 1, &(pyArgs[0])))
        return -1;


    // Overloaded function decisor
    // 0: RecursivePluginProvider::RecursivePluginProvider(qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders*)
    if (numArgs == 1
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINPROVIDERS_IDX]), (pyArgs[0])))) {
        overloadId = 0; // RecursivePluginProvider(qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_RecursivePluginProvider_Init_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArgs[0]))
            return -1;
        ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders *cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);

        if (!PyErr_Occurred()) {
            // RecursivePluginProvider(qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders*)
            cptr = new ::RecursivePluginProviderWrapper(cppArg0);
        }
    }

    if (PyErr_Occurred() || !Shiboken::Object::setCppPointer(sbkSelf, Shiboken::SbkType< ::qt_gui_cpp::RecursivePluginProvider >(), cptr)) {
        delete cptr;
        return -1;
    }
    if (!cptr) goto Sbk_qt_gui_cpp_RecursivePluginProvider_Init_TypeError;

    Shiboken::Object::setValidCpp(sbkSelf, true);
    Shiboken::Object::setHasCppWrapper(sbkSelf, true);
    if (Shiboken::BindingManager::instance().hasWrapper(cptr)) {
        Shiboken::BindingManager::instance().releaseWrapper(Shiboken::BindingManager::instance().retrieveWrapper(cptr));
    }
    Shiboken::BindingManager::instance().registerWrapper(sbkSelf, cptr);


    return 1;

    Sbk_qt_gui_cpp_RecursivePluginProvider_Init_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.RecursivePluginProvider");
        return -1;
}

static PyObject *Sbk_qt_gui_cpp_RecursivePluginProviderFunc_discover(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::RecursivePluginProvider *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_RECURSIVEPLUGINPROVIDER_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: PluginProvider::discover(QObject*)
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), (pyArg)))) {
        overloadId = 0; // discover(QObject*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_RecursivePluginProviderFunc_discover_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArg))
            return {};
        ::QObject *cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // discover(QObject*)
            QMap<QString,QString > cppResult = Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::RecursivePluginProvider::discover(cppArg0)
                : cppSelf->discover(cppArg0);
            pyResult = Shiboken::Conversions::copyToPython(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_RecursivePluginProviderFunc_discover_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.RecursivePluginProvider.discover");
        return {};
}


static const char *Sbk_qt_gui_cpp_RecursivePluginProvider_PropertyStrings[] = {
    nullptr // Sentinel
};

static PyMethodDef Sbk_qt_gui_cpp_RecursivePluginProvider_methods[] = {
    {"discover", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_RecursivePluginProviderFunc_discover), METH_O},

    {nullptr, nullptr} // Sentinel
};

static int Sbk_qt_gui_cpp_RecursivePluginProvider_setattro(PyObject *self, PyObject *name, PyObject *value)
{
    PySide::Feature::Select(self);
    if (value && PyCallable_Check(value)) {
        auto plain_inst = reinterpret_cast< ::qt_gui_cpp::RecursivePluginProvider *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_RECURSIVEPLUGINPROVIDER_IDX], reinterpret_cast<SbkObject *>(self)));
        auto inst = dynamic_cast<RecursivePluginProviderWrapper *>(plain_inst);
        if (inst)
            inst->resetPyMethodCache();
    }
    return PyObject_GenericSetAttr(self, name, value);
}

} // extern "C"

static int Sbk_qt_gui_cpp_RecursivePluginProvider_traverse(PyObject *self, visitproc visit, void *arg)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_traverse(self, visit, arg);
}
static int Sbk_qt_gui_cpp_RecursivePluginProvider_clear(PyObject *self)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_clear(self);
}
// Class Definition -----------------------------------------------
extern "C" {
static SbkObjectType *_Sbk_qt_gui_cpp_RecursivePluginProvider_Type = nullptr;
static SbkObjectType *Sbk_qt_gui_cpp_RecursivePluginProvider_TypeF(void)
{
    return _Sbk_qt_gui_cpp_RecursivePluginProvider_Type;
}

static PyType_Slot Sbk_qt_gui_cpp_RecursivePluginProvider_slots[] = {
    {Py_tp_base,        nullptr}, // inserted by introduceWrapperType
    {Py_tp_dealloc,     reinterpret_cast<void *>(&SbkDeallocWrapper)},
    {Py_tp_repr,        nullptr},
    {Py_tp_hash,        nullptr},
    {Py_tp_call,        nullptr},
    {Py_tp_str,         nullptr},
    {Py_tp_getattro,    nullptr},
    {Py_tp_setattro,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_RecursivePluginProvider_setattro)},
    {Py_tp_traverse,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_RecursivePluginProvider_traverse)},
    {Py_tp_clear,       reinterpret_cast<void *>(Sbk_qt_gui_cpp_RecursivePluginProvider_clear)},
    {Py_tp_richcompare, nullptr},
    {Py_tp_iter,        nullptr},
    {Py_tp_iternext,    nullptr},
    {Py_tp_methods,     reinterpret_cast<void *>(Sbk_qt_gui_cpp_RecursivePluginProvider_methods)},
    {Py_tp_getset,      nullptr},
    {Py_tp_init,        reinterpret_cast<void *>(Sbk_qt_gui_cpp_RecursivePluginProvider_Init)},
    {Py_tp_new,         reinterpret_cast<void *>(SbkObjectTpNew)},
    {0, nullptr}
};
static PyType_Spec Sbk_qt_gui_cpp_RecursivePluginProvider_spec = {
    "1:libqt_gui_cpp_shiboken.qt_gui_cpp.RecursivePluginProvider",
    sizeof(SbkObject),
    0,
    Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_CHECKTYPES|Py_TPFLAGS_HAVE_GC,
    Sbk_qt_gui_cpp_RecursivePluginProvider_slots
};

} //extern "C"

static void *Sbk_qt_gui_cpp_RecursivePluginProvider_typeDiscovery(void *cptr, SbkObjectType *instanceType)
{
    if (instanceType == reinterpret_cast<SbkObjectType *>(Shiboken::SbkType< ::qt_gui_cpp::PluginProvider >()))
        return dynamic_cast< ::qt_gui_cpp::RecursivePluginProvider *>(reinterpret_cast< ::qt_gui_cpp::PluginProvider *>(cptr));
    return {};
}


// Type conversion functions.

// Python to C++ pointer conversion - returns the C++ object of the Python wrapper (keeps object identity).
static void RecursivePluginProvider_PythonToCpp_RecursivePluginProvider_PTR(PyObject *pyIn, void *cppOut) {
    Shiboken::Conversions::pythonToCppPointer(Sbk_qt_gui_cpp_RecursivePluginProvider_TypeF(), pyIn, cppOut);
}
static PythonToCppFunc is_RecursivePluginProvider_PythonToCpp_RecursivePluginProvider_PTR_Convertible(PyObject *pyIn) {
    if (pyIn == Py_None)
        return Shiboken::Conversions::nonePythonToCppNullPtr;
    if (PyObject_TypeCheck(pyIn, reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_RecursivePluginProvider_TypeF())))
        return RecursivePluginProvider_PythonToCpp_RecursivePluginProvider_PTR;
    return {};
}

// C++ to Python pointer conversion - tries to find the Python wrapper for the C++ object (keeps object identity).
static PyObject *RecursivePluginProvider_PTR_CppToPython_RecursivePluginProvider(const void *cppIn) {
    auto pyOut = reinterpret_cast<PyObject *>(Shiboken::BindingManager::instance().retrieveWrapper(cppIn));
    if (pyOut) {
        Py_INCREF(pyOut);
        return pyOut;
    }
    bool changedTypeName = false;
    auto tCppIn = reinterpret_cast<const ::qt_gui_cpp::RecursivePluginProvider *>(cppIn);
    const char *typeName = typeid(*tCppIn).name();
    auto sbkType = Shiboken::ObjectType::typeForTypeName(typeName);
    if (sbkType && Shiboken::ObjectType::hasSpecialCastFunction(sbkType)) {
        typeName = typeNameOf(tCppIn);
        changedTypeName = true;
    }
    PyObject *result = Shiboken::Object::newObject(Sbk_qt_gui_cpp_RecursivePluginProvider_TypeF(), const_cast<void *>(cppIn), false, /* exactType */ changedTypeName, typeName);
    if (changedTypeName)
        delete [] typeName;
    return result;
}

// The signatures string for the functions.
// Multiple signatures have their index "n:" in front.
static const char *qt_gui_cpp_RecursivePluginProvider_SignatureStrings[] = {
    "libqt_gui_cpp_shiboken.qt_gui_cpp.RecursivePluginProvider(self,plugin_provider:libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.RecursivePluginProvider.discover(self,discovery_data:PySide2.QtCore.QObject)->QMap[QString, QString]",
    nullptr}; // Sentinel

void init_qt_gui_cpp_RecursivePluginProvider(PyObject *enclosingClass)
{
    _Sbk_qt_gui_cpp_RecursivePluginProvider_Type = Shiboken::ObjectType::introduceWrapperType(
        enclosingClass,
        "RecursivePluginProvider",
        "qt_gui_cpp::RecursivePluginProvider*",
        &Sbk_qt_gui_cpp_RecursivePluginProvider_spec,
        &Shiboken::callCppDestructor< ::qt_gui_cpp::RecursivePluginProvider >,
        reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_COMPOSITEPLUGINPROVIDER_IDX]),
        0,
        Shiboken::ObjectType::WrapperFlags::InnerClass    );
    
    auto pyType = reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_RecursivePluginProvider_Type);
    InitSignatureStrings(pyType, qt_gui_cpp_RecursivePluginProvider_SignatureStrings);
    SbkObjectType_SetPropertyStrings(reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_RecursivePluginProvider_Type), Sbk_qt_gui_cpp_RecursivePluginProvider_PropertyStrings);
    Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_RECURSIVEPLUGINPROVIDER_IDX]
        = reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_RecursivePluginProvider_TypeF());

    // Register Converter
    SbkConverter *converter = Shiboken::Conversions::createConverter(Sbk_qt_gui_cpp_RecursivePluginProvider_TypeF(),
        RecursivePluginProvider_PythonToCpp_RecursivePluginProvider_PTR,
        is_RecursivePluginProvider_PythonToCpp_RecursivePluginProvider_PTR_Convertible,
        RecursivePluginProvider_PTR_CppToPython_RecursivePluginProvider);

    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::RecursivePluginProvider");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::RecursivePluginProvider*");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::RecursivePluginProvider&");
    Shiboken::Conversions::registerConverterName(converter, "RecursivePluginProvider");
    Shiboken::Conversions::registerConverterName(converter, "RecursivePluginProvider*");
    Shiboken::Conversions::registerConverterName(converter, "RecursivePluginProvider&");
    Shiboken::Conversions::registerConverterName(converter, typeid(::qt_gui_cpp::RecursivePluginProvider).name());
    Shiboken::Conversions::registerConverterName(converter, typeid(::RecursivePluginProviderWrapper).name());


    Shiboken::ObjectType::setTypeDiscoveryFunctionV2(Sbk_qt_gui_cpp_RecursivePluginProvider_TypeF(), &Sbk_qt_gui_cpp_RecursivePluginProvider_typeDiscovery);


}
