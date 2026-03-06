
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
#include <signalmanager.h>
#include <pysidemetafunction.h>
#include <iterator>

// module include
#include "libqt_gui_cpp_shiboken_python.h"

// main header
#include "qt_gui_cpp_rospluginlibpluginprovider_forpluginproviders_wrapper.h"

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

void RosPluginlibPluginProvider_ForPluginProvidersWrapper::resetPyMethodCache()
{
    std::fill_n(m_PyMethodCache, sizeof(m_PyMethodCache) / sizeof(m_PyMethodCache[0]), false);
}

RosPluginlibPluginProvider_ForPluginProvidersWrapper::RosPluginlibPluginProvider_ForPluginProvidersWrapper(const QString & export_tag, const QString & base_class_type) : qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders(export_tag, base_class_type)
{
    resetPyMethodCache();
    // ... middle
}

void RosPluginlibPluginProvider_ForPluginProvidersWrapper::childEvent(QChildEvent * event)
{
    if (m_PyMethodCache[0]) {
        return this->::QObject::childEvent(event);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "childEvent";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[0] = true;
        return this->::QObject::childEvent(event);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(N)",
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QCHILDEVENT_IDX]), event)
    ));
    bool invalidateArg1 = PyTuple_GET_ITEM(pyArgs, 0)->ob_refcnt == 1;

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
    if (invalidateArg1)
        Shiboken::Object::invalidate(PyTuple_GET_ITEM(pyArgs, 0));
}

void RosPluginlibPluginProvider_ForPluginProvidersWrapper::connectNotify(const QMetaMethod & signal)
{
    if (m_PyMethodCache[1]) {
        return this->::QObject::connectNotify(signal);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "connectNotify";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[1] = true;
        return this->::QObject::connectNotify(signal);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(N)",
    Shiboken::Conversions::copyToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QMETAMETHOD_IDX]), &signal)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

void RosPluginlibPluginProvider_ForPluginProvidersWrapper::customEvent(QEvent * event)
{
    if (m_PyMethodCache[2]) {
        return this->::QObject::customEvent(event);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "customEvent";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[2] = true;
        return this->::QObject::customEvent(event);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(N)",
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QEVENT_IDX]), event)
    ));
    bool invalidateArg1 = PyTuple_GET_ITEM(pyArgs, 0)->ob_refcnt == 1;

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
    if (invalidateArg1)
        Shiboken::Object::invalidate(PyTuple_GET_ITEM(pyArgs, 0));
}

void RosPluginlibPluginProvider_ForPluginProvidersWrapper::disconnectNotify(const QMetaMethod & signal)
{
    if (m_PyMethodCache[3]) {
        return this->::QObject::disconnectNotify(signal);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "disconnectNotify";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[3] = true;
        return this->::QObject::disconnectNotify(signal);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(N)",
    Shiboken::Conversions::copyToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QMETAMETHOD_IDX]), &signal)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

QMap<QString,QString > RosPluginlibPluginProvider_ForPluginProvidersWrapper::discover(QObject * discovery_data)
{
    if (m_PyMethodCache[4])
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::discover(discovery_data);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return {};
    static PyObject *nameCache[2] = {};
    static const char *funcName = "discover";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[4] = true;
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::discover(discovery_data);
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "RosPluginlibPluginProvider_ForPluginProviders.discover", "map", Py_TYPE(pyResult)->tp_name);
        return {};
    }
    ::QMap<QString,QString > cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

QList<qt_gui_cpp::PluginDescriptor* > RosPluginlibPluginProvider_ForPluginProvidersWrapper::discover_descriptors(QObject * discovery_data)
{
    if (m_PyMethodCache[5])
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::discover_descriptors(discovery_data);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return {};
    static PyObject *nameCache[2] = {};
    static const char *funcName = "discover_descriptors";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[5] = true;
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::discover_descriptors(discovery_data);
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "RosPluginlibPluginProvider_ForPluginProviders.discover_descriptors", "list", Py_TYPE(pyResult)->tp_name);
        return {};
    }
    ::QList<qt_gui_cpp::PluginDescriptor* > cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

bool RosPluginlibPluginProvider_ForPluginProvidersWrapper::event(QEvent * e)
{
    if (m_PyMethodCache[6])
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::event(e);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return false;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "event";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[6] = true;
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::event(e);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(N)",
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QEVENT_IDX]), e)
    ));
    bool invalidateArg1 = PyTuple_GET_ITEM(pyArgs, 0)->ob_refcnt == 1;

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return false;
    }
    // Check return type
    PythonToCppFunc pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(Shiboken::Conversions::PrimitiveTypeConverter<bool>(), pyResult);
    if (!pythonToCpp) {
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "RosPluginlibPluginProvider_ForPluginProviders.event", "bool", Py_TYPE(pyResult)->tp_name);
        return false;
    }
    bool cppResult;
    pythonToCpp(pyResult, &cppResult);
    if (invalidateArg1)
        Shiboken::Object::invalidate(PyTuple_GET_ITEM(pyArgs, 0));
    return cppResult;
}

bool RosPluginlibPluginProvider_ForPluginProvidersWrapper::eventFilter(QObject * watched, QEvent * event)
{
    if (m_PyMethodCache[7])
        return this->::QObject::eventFilter(watched, event);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return false;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "eventFilter";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[7] = true;
        return this->::QObject::eventFilter(watched, event);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(NN)",
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), watched),
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QEVENT_IDX]), event)
    ));
    bool invalidateArg2 = PyTuple_GET_ITEM(pyArgs, 1)->ob_refcnt == 1;

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return false;
    }
    // Check return type
    PythonToCppFunc pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(Shiboken::Conversions::PrimitiveTypeConverter<bool>(), pyResult);
    if (!pythonToCpp) {
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "RosPluginlibPluginProvider_ForPluginProviders.eventFilter", "bool", Py_TYPE(pyResult)->tp_name);
        return false;
    }
    bool cppResult;
    pythonToCpp(pyResult, &cppResult);
    if (invalidateArg2)
        Shiboken::Object::invalidate(PyTuple_GET_ITEM(pyArgs, 1));
    return cppResult;
}

void RosPluginlibPluginProvider_ForPluginProvidersWrapper::init_plugin(const QString & arg__1, qt_gui_cpp::PluginContext * plugin_context, qt_gui_cpp::Plugin * plugin)
{
    if (m_PyMethodCache[8]) {
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::init_plugin(arg__1, plugin_context, plugin);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "init_plugin";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[8] = true;
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::init_plugin(arg__1, plugin_context, plugin);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(NNN)",
    Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], &arg__1),
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]), plugin_context),
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX]), plugin)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

void * RosPluginlibPluginProvider_ForPluginProvidersWrapper::load(const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context)
{
    if (m_PyMethodCache[9])
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::load(plugin_id, plugin_context);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return nullptr;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "load";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[9] = true;
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::load(plugin_id, plugin_context);
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "RosPluginlibPluginProvider_ForPluginProviders.load", reinterpret_cast<PyTypeObject *>(Shiboken::SbkType< void >())->tp_name, Py_TYPE(pyResult)->tp_name);
        return nullptr;
    }
    void* cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

qt_gui_cpp::PluginProvider * RosPluginlibPluginProvider_ForPluginProvidersWrapper::load_explicit_type(const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context)
{
    if (m_PyMethodCache[10])
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::load_explicit_type(plugin_id, plugin_context);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return nullptr;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "load_explicit_type";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[10] = true;
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::load_explicit_type(plugin_id, plugin_context);
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
    PythonToCppFunc pythonToCpp = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX]), pyResult);
    if (!pythonToCpp) {
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "RosPluginlibPluginProvider_ForPluginProviders.load_explicit_type", reinterpret_cast<PyTypeObject *>(Shiboken::SbkType< qt_gui_cpp::PluginProvider >())->tp_name, Py_TYPE(pyResult)->tp_name);
        return nullptr;
    }
    ::qt_gui_cpp::PluginProvider *cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

qt_gui_cpp::Plugin * RosPluginlibPluginProvider_ForPluginProvidersWrapper::load_plugin(const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context)
{
    if (m_PyMethodCache[11])
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::load_plugin(plugin_id, plugin_context);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return nullptr;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "load_plugin";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[11] = true;
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::load_plugin(plugin_id, plugin_context);
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "RosPluginlibPluginProvider_ForPluginProviders.load_plugin", reinterpret_cast<PyTypeObject *>(Shiboken::SbkType< qt_gui_cpp::Plugin >())->tp_name, Py_TYPE(pyResult)->tp_name);
        return nullptr;
    }
    ::qt_gui_cpp::Plugin *cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

void RosPluginlibPluginProvider_ForPluginProvidersWrapper::timerEvent(QTimerEvent * event)
{
    if (m_PyMethodCache[13]) {
        return this->::QObject::timerEvent(event);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "timerEvent";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[13] = true;
        return this->::QObject::timerEvent(event);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(N)",
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QTIMEREVENT_IDX]), event)
    ));
    bool invalidateArg1 = PyTuple_GET_ITEM(pyArgs, 0)->ob_refcnt == 1;

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
    if (invalidateArg1)
        Shiboken::Object::invalidate(PyTuple_GET_ITEM(pyArgs, 0));
}

void RosPluginlibPluginProvider_ForPluginProvidersWrapper::unload(void * instance)
{
    if (m_PyMethodCache[14]) {
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::unload(instance);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "unload";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[14] = true;
        return this->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::unload(instance);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(N)",
    Shiboken::Conversions::copyToPython(Shiboken::Conversions::PrimitiveTypeConverter<void *>(), instance)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

const QMetaObject *RosPluginlibPluginProvider_ForPluginProvidersWrapper::metaObject() const
{
    if (QObject::d_ptr->metaObject)
        return QObject::d_ptr->dynamicMetaObject();
    SbkObject *pySelf = Shiboken::BindingManager::instance().retrieveWrapper(this);
    if (pySelf == nullptr)
        return qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::metaObject();
    return PySide::SignalManager::retrieveMetaObject(reinterpret_cast<PyObject *>(pySelf));
}

int RosPluginlibPluginProvider_ForPluginProvidersWrapper::qt_metacall(QMetaObject::Call call, int id, void **args)
{
    int result = qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::qt_metacall(call, id, args);
    return result < 0 ? result : PySide::SignalManager::qt_metacall(this, call, id, args);
}

void *RosPluginlibPluginProvider_ForPluginProvidersWrapper::qt_metacast(const char *_clname)
{
        if (!_clname) return {};
        SbkObject *pySelf = Shiboken::BindingManager::instance().retrieveWrapper(this);
        if (pySelf && PySide::inherits(Py_TYPE(pySelf), _clname))
                return static_cast<void *>(const_cast< RosPluginlibPluginProvider_ForPluginProvidersWrapper *>(this));
        return qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::qt_metacast(_clname);
}

RosPluginlibPluginProvider_ForPluginProvidersWrapper::~RosPluginlibPluginProvider_ForPluginProvidersWrapper()
{
    SbkObject *wrapper = Shiboken::BindingManager::instance().retrieveWrapper(this);
    Shiboken::Object::destroy(wrapper, this);
}

// Target ---------------------------------------------------------

extern "C" {
static int
Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_Init(PyObject *self, PyObject *args, PyObject *kwds)
{
    const char **argNames{};
    const QMetaObject *metaObject;
    SbkObject *sbkSelf = reinterpret_cast<SbkObject *>(self);
    if (Shiboken::Object::isUserType(self) && !Shiboken::ObjectType::canCallConstructor(self->ob_type, Shiboken::SbkType< ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders >()))
        return -1;

    ::RosPluginlibPluginProvider_ForPluginProvidersWrapper *cptr{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0};

    // invalid argument lengths


    if (!PyArg_UnpackTuple(args, "RosPluginlibPluginProvider_ForPluginProviders", 2, 2, &(pyArgs[0]), &(pyArgs[1])))
        return -1;


    // Overloaded function decisor
    // 0: RosPluginlibPluginProvider_ForPluginProviders::RosPluginlibPluginProvider_ForPluginProviders(QString,QString)
    if (numArgs == 2
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[1])))) {
        overloadId = 0; // RosPluginlibPluginProvider_ForPluginProviders(QString,QString)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_Init_TypeError;

    // Call function/method
    {
        ::QString cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        ::QString cppArg1;
        pythonToCpp[1](pyArgs[1], &cppArg1);

        if (!PyErr_Occurred()) {
            // RosPluginlibPluginProvider_ForPluginProviders(QString,QString)
            void *addr = PySide::nextQObjectMemoryAddr();
            if (addr) {
                cptr = new (addr) ::RosPluginlibPluginProvider_ForPluginProvidersWrapper(cppArg0, cppArg1);
                PySide::setNextQObjectMemoryAddr(0);
            } else {
                cptr = new ::RosPluginlibPluginProvider_ForPluginProvidersWrapper(cppArg0, cppArg1);
            }

        }
    }

    if (PyErr_Occurred() || !Shiboken::Object::setCppPointer(sbkSelf, Shiboken::SbkType< ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders >(), cptr)) {
        delete cptr;
        return -1;
    }
    if (!cptr) goto Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_Init_TypeError;

    Shiboken::Object::setValidCpp(sbkSelf, true);
    Shiboken::Object::setHasCppWrapper(sbkSelf, true);
    if (Shiboken::BindingManager::instance().hasWrapper(cptr)) {
        Shiboken::BindingManager::instance().releaseWrapper(Shiboken::BindingManager::instance().retrieveWrapper(cptr));
    }
    Shiboken::BindingManager::instance().registerWrapper(sbkSelf, cptr);

    // QObject setup
    PySide::Signal::updateSourceObject(self);
    metaObject = cptr->metaObject(); // <- init python qt properties
    if (kwds && !PySide::fillQtProperties(self, metaObject, kwds, argNames, 0))
        return -1;


    return 1;

    Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_Init_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders");
        return -1;
}

static PyObject *Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_create_instance(PyObject *self, PyObject *args)
{
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0};

    // invalid argument lengths


    if (!PyArg_UnpackTuple(args, "create_instance", 2, 2, &(pyArgs[0]), &(pyArgs[1])))
        return {};


    // Overloaded function decisor
    // 0: static RosPluginlibPluginProvider_ForPluginProviders::create_instance(QString,QString)
    if (numArgs == 2
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[1])))) {
        overloadId = 0; // create_instance(QString,QString)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_create_instance_TypeError;

    // Call function/method
    {
        ::QString cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        ::QString cppArg1;
        pythonToCpp[1](pyArgs[1], &cppArg1);

        if (!PyErr_Occurred()) {
            // create_instance(QString,QString)
            qt_gui_cpp::RosPluginlibPluginProvider<qt_gui_cpp::PluginProvider > * cppResult = ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::create_instance(cppArg0, cppArg1);
            pyResult = Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_QT_GUI_CPP_PLUGINPROVIDER_IDX]), cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_create_instance_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.create_instance");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_discover(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINPROVIDERS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: RosPluginlibPluginProvider_ForPluginProviders::discover(QObject*)
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), (pyArg)))) {
        overloadId = 0; // discover(QObject*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_discover_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArg))
            return {};
        ::QObject *cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // discover(QObject*)
            QMap<QString,QString > cppResult = Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::discover(cppArg0)
                : cppSelf->discover(cppArg0);
            pyResult = Shiboken::Conversions::copyToPython(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_discover_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.discover");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_discover_descriptors(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINPROVIDERS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: RosPluginlibPluginProvider_ForPluginProviders::discover_descriptors(QObject*)
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), (pyArg)))) {
        overloadId = 0; // discover_descriptors(QObject*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_discover_descriptors_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArg))
            return {};
        ::QObject *cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // discover_descriptors(QObject*)
            QList<qt_gui_cpp::PluginDescriptor* > cppResult = Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::discover_descriptors(cppArg0)
                : cppSelf->discover_descriptors(cppArg0);
            pyResult = Shiboken::Conversions::copyToPython(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QT_GUI_CPP_PLUGINDESCRIPTORPTR_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_discover_descriptors_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.discover_descriptors");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_event(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINPROVIDERS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: QObject::event(QEvent*)
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QEVENT_IDX]), (pyArg)))) {
        overloadId = 0; // event(QEvent*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_event_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArg))
            return {};
        ::QEvent *cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // event(QEvent*)
            bool cppResult = Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::event(cppArg0)
                : cppSelf->event(cppArg0);
            pyResult = Shiboken::Conversions::copyToPython(Shiboken::Conversions::PrimitiveTypeConverter<bool>(), &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_event_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.event");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_init_plugin(PyObject *self, PyObject *args)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINPROVIDERS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0, 0};

    // invalid argument lengths


    if (!PyArg_UnpackTuple(args, "init_plugin", 3, 3, &(pyArgs[0]), &(pyArgs[1]), &(pyArgs[2])))
        return {};


    // Overloaded function decisor
    // 0: RosPluginlibPluginProvider_ForPluginProviders::init_plugin(QString,qt_gui_cpp::PluginContext*,qt_gui_cpp::Plugin*)
    if (numArgs == 3
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]), (pyArgs[1])))
        && (pythonToCpp[2] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX]), (pyArgs[2])))) {
        overloadId = 0; // init_plugin(QString,qt_gui_cpp::PluginContext*,qt_gui_cpp::Plugin*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_init_plugin_TypeError;

    // Call function/method
    {
        ::QString cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        if (!Shiboken::Object::isValid(pyArgs[1]))
            return {};
        ::qt_gui_cpp::PluginContext *cppArg1;
        pythonToCpp[1](pyArgs[1], &cppArg1);
        if (!Shiboken::Object::isValid(pyArgs[2]))
            return {};
        ::qt_gui_cpp::Plugin *cppArg2;
        pythonToCpp[2](pyArgs[2], &cppArg2);

        if (!PyErr_Occurred()) {
            // init_plugin(QString,qt_gui_cpp::PluginContext*,qt_gui_cpp::Plugin*)
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::init_plugin(cppArg0, cppArg1, cppArg2)
                : cppSelf->init_plugin(cppArg0, cppArg1, cppArg2);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_init_plugin_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.init_plugin");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_load(PyObject *self, PyObject *args)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINPROVIDERS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0};

    // invalid argument lengths


    if (!PyArg_UnpackTuple(args, "load", 2, 2, &(pyArgs[0]), &(pyArgs[1])))
        return {};


    // Overloaded function decisor
    // 0: RosPluginlibPluginProvider_ForPluginProviders::load(QString,qt_gui_cpp::PluginContext*)
    if (numArgs == 2
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]), (pyArgs[1])))) {
        overloadId = 0; // load(QString,qt_gui_cpp::PluginContext*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_load_TypeError;

    // Call function/method
    {
        ::QString cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        if (!Shiboken::Object::isValid(pyArgs[1]))
            return {};
        ::qt_gui_cpp::PluginContext *cppArg1;
        pythonToCpp[1](pyArgs[1], &cppArg1);

        if (!PyErr_Occurred()) {
            // load(QString,qt_gui_cpp::PluginContext*)
            void * cppResult = Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::load(cppArg0, cppArg1)
                : cppSelf->load(cppArg0, cppArg1);
            pyResult = Shiboken::Conversions::copyToPython(Shiboken::Conversions::PrimitiveTypeConverter<void *>(), cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_load_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.load");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_load_explicit_type(PyObject *self, PyObject *args)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINPROVIDERS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0};

    // invalid argument lengths


    if (!PyArg_UnpackTuple(args, "load_explicit_type", 2, 2, &(pyArgs[0]), &(pyArgs[1])))
        return {};


    // Overloaded function decisor
    // 0: RosPluginlibPluginProvider_ForPluginProviders::load_explicit_type(QString,qt_gui_cpp::PluginContext*)
    if (numArgs == 2
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]), (pyArgs[1])))) {
        overloadId = 0; // load_explicit_type(QString,qt_gui_cpp::PluginContext*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_load_explicit_type_TypeError;

    // Call function/method
    {
        ::QString cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        if (!Shiboken::Object::isValid(pyArgs[1]))
            return {};
        ::qt_gui_cpp::PluginContext *cppArg1;
        pythonToCpp[1](pyArgs[1], &cppArg1);

        if (!PyErr_Occurred()) {
            // load_explicit_type(QString,qt_gui_cpp::PluginContext*)
            qt_gui_cpp::PluginProvider * cppResult = Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::load_explicit_type(cppArg0, cppArg1)
                : cppSelf->load_explicit_type(cppArg0, cppArg1);
            pyResult = Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX]), cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_load_explicit_type_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.load_explicit_type");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_load_plugin(PyObject *self, PyObject *args)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINPROVIDERS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0};

    // invalid argument lengths


    if (!PyArg_UnpackTuple(args, "load_plugin", 2, 2, &(pyArgs[0]), &(pyArgs[1])))
        return {};


    // Overloaded function decisor
    // 0: RosPluginlibPluginProvider_ForPluginProviders::load_plugin(QString,qt_gui_cpp::PluginContext*)
    if (numArgs == 2
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]), (pyArgs[1])))) {
        overloadId = 0; // load_plugin(QString,qt_gui_cpp::PluginContext*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_load_plugin_TypeError;

    // Call function/method
    {
        ::QString cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        if (!Shiboken::Object::isValid(pyArgs[1]))
            return {};
        ::qt_gui_cpp::PluginContext *cppArg1;
        pythonToCpp[1](pyArgs[1], &cppArg1);

        if (!PyErr_Occurred()) {
            // load_plugin(QString,qt_gui_cpp::PluginContext*)
            qt_gui_cpp::Plugin * cppResult = Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::load_plugin(cppArg0, cppArg1)
                : cppSelf->load_plugin(cppArg0, cppArg1);
            pyResult = Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX]), cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_load_plugin_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.load_plugin");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_unload(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINPROVIDERS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: RosPluginlibPluginProvider_ForPluginProviders::unload(void*)
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(Shiboken::Conversions::PrimitiveTypeConverter<void *>(), (pyArg)))) {
        overloadId = 0; // unload(void*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_unload_TypeError;

    // Call function/method
    {
        void* cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // unload(void*)
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::unload(cppArg0)
                : cppSelf->unload(cppArg0);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_unload_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.unload");
        return {};
}


static const char *Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_PropertyStrings[] = {
    nullptr // Sentinel
};

static PyMethodDef Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_methods[] = {
    {"create_instance", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_create_instance), METH_VARARGS|METH_STATIC},
    {"discover", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_discover), METH_O},
    {"discover_descriptors", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_discover_descriptors), METH_O},
    {"event", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_event), METH_O},
    {"init_plugin", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_init_plugin), METH_VARARGS},
    {"load", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_load), METH_VARARGS},
    {"load_explicit_type", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_load_explicit_type), METH_VARARGS},
    {"load_plugin", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_load_plugin), METH_VARARGS},
    {"unload", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProvidersFunc_unload), METH_O},

    {nullptr, nullptr} // Sentinel
};

static int Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_setattro(PyObject *self, PyObject *name, PyObject *value)
{
    PySide::Feature::Select(self);
    if (value && PyCallable_Check(value)) {
        auto plain_inst = reinterpret_cast< ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINPROVIDERS_IDX], reinterpret_cast<SbkObject *>(self)));
        auto inst = dynamic_cast<RosPluginlibPluginProvider_ForPluginProvidersWrapper *>(plain_inst);
        if (inst)
            inst->resetPyMethodCache();
    }
    Shiboken::AutoDecRef pp(reinterpret_cast<PyObject *>(PySide::Property::getObject(self, name)));
    if (!pp.isNull())
        return PySide::Property::setValue(reinterpret_cast<PySideProperty *>(pp.object()), self, value);
    return PyObject_GenericSetAttr(self, name, value);
}

} // extern "C"

static int Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_traverse(PyObject *self, visitproc visit, void *arg)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_traverse(self, visit, arg);
}
static int Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_clear(PyObject *self)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_clear(self);
}
// Class Definition -----------------------------------------------
extern "C" {
static SbkObjectType *_Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_Type = nullptr;
static SbkObjectType *Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_TypeF(void)
{
    return _Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_Type;
}

static PyType_Slot Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_slots[] = {
    {Py_tp_base,        nullptr}, // inserted by introduceWrapperType
    {Py_tp_dealloc,     reinterpret_cast<void *>(&SbkDeallocWrapper)},
    {Py_tp_repr,        nullptr},
    {Py_tp_hash,        nullptr},
    {Py_tp_call,        nullptr},
    {Py_tp_str,         nullptr},
    {Py_tp_getattro,    nullptr},
    {Py_tp_setattro,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_setattro)},
    {Py_tp_traverse,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_traverse)},
    {Py_tp_clear,       reinterpret_cast<void *>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_clear)},
    {Py_tp_richcompare, nullptr},
    {Py_tp_iter,        nullptr},
    {Py_tp_iternext,    nullptr},
    {Py_tp_methods,     reinterpret_cast<void *>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_methods)},
    {Py_tp_getset,      nullptr},
    {Py_tp_init,        reinterpret_cast<void *>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_Init)},
    {Py_tp_new,         reinterpret_cast<void *>(SbkObjectTpNew)},
    {0, nullptr}
};
static PyType_Spec Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_spec = {
    "1:libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders",
    sizeof(SbkObject),
    0,
    Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_CHECKTYPES|Py_TPFLAGS_HAVE_GC,
    Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_slots
};

} //extern "C"

static void *Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_typeDiscovery(void *cptr, SbkObjectType *instanceType)
{
    return {};
}


// Type conversion functions.

// Python to C++ pointer conversion - returns the C++ object of the Python wrapper (keeps object identity).
static void RosPluginlibPluginProvider_ForPluginProviders_PythonToCpp_RosPluginlibPluginProvider_ForPluginProviders_PTR(PyObject *pyIn, void *cppOut) {
    Shiboken::Conversions::pythonToCppPointer(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_TypeF(), pyIn, cppOut);
}
static PythonToCppFunc is_RosPluginlibPluginProvider_ForPluginProviders_PythonToCpp_RosPluginlibPluginProvider_ForPluginProviders_PTR_Convertible(PyObject *pyIn) {
    if (pyIn == Py_None)
        return Shiboken::Conversions::nonePythonToCppNullPtr;
    if (PyObject_TypeCheck(pyIn, reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_TypeF())))
        return RosPluginlibPluginProvider_ForPluginProviders_PythonToCpp_RosPluginlibPluginProvider_ForPluginProviders_PTR;
    return {};
}

// C++ to Python pointer conversion - tries to find the Python wrapper for the C++ object (keeps object identity).
static PyObject *RosPluginlibPluginProvider_ForPluginProviders_PTR_CppToPython_RosPluginlibPluginProvider_ForPluginProviders(const void *cppIn) {
    return PySide::getWrapperForQObject(reinterpret_cast<::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders *>(const_cast<void *>(cppIn)), Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_TypeF());

}

// The signatures string for the functions.
// Multiple signatures have their index "n:" in front.
static const char *qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_SignatureStrings[] = {
    "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders(self,export_tag:QString,base_class_type:QString)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.create_instance(export_tag:QString,base_class_type:QString)->libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider[libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider]",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.discover(self,discovery_data:PySide2.QtCore.QObject)->QMap[QString, QString]",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.discover_descriptors(self,discovery_data:PySide2.QtCore.QObject)->QList[libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor]",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.event(self,e:PySide2.QtCore.QEvent)->bool",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.init_plugin(self,arg__1:QString,plugin_context:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext,plugin:libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.load(self,plugin_id:QString,plugin_context:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext)->void",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.load_explicit_type(self,plugin_id:QString,plugin_context:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext)->libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.load_plugin(self,plugin_id:QString,plugin_context:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext)->libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.RosPluginlibPluginProvider_ForPluginProviders.unload(self,instance:void)",
    nullptr}; // Sentinel

void init_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders(PyObject *enclosingClass)
{
    _Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_Type = Shiboken::ObjectType::introduceWrapperType(
        enclosingClass,
        "RosPluginlibPluginProvider_ForPluginProviders",
        "qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders*",
        &Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_spec,
        &Shiboken::callCppDestructor< ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders >,
        reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]),
        0,
        Shiboken::ObjectType::WrapperFlags::InnerClass    );
    
    auto pyType = reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_Type);
    InitSignatureStrings(pyType, qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_SignatureStrings);
    SbkObjectType_SetPropertyStrings(reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_Type), Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_PropertyStrings);
    Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINPROVIDERS_IDX]
        = reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_TypeF());

    // Register Converter
    SbkConverter *converter = Shiboken::Conversions::createConverter(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_TypeF(),
        RosPluginlibPluginProvider_ForPluginProviders_PythonToCpp_RosPluginlibPluginProvider_ForPluginProviders_PTR,
        is_RosPluginlibPluginProvider_ForPluginProviders_PythonToCpp_RosPluginlibPluginProvider_ForPluginProviders_PTR_Convertible,
        RosPluginlibPluginProvider_ForPluginProviders_PTR_CppToPython_RosPluginlibPluginProvider_ForPluginProviders);

    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders*");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders&");
    Shiboken::Conversions::registerConverterName(converter, "RosPluginlibPluginProvider_ForPluginProviders");
    Shiboken::Conversions::registerConverterName(converter, "RosPluginlibPluginProvider_ForPluginProviders*");
    Shiboken::Conversions::registerConverterName(converter, "RosPluginlibPluginProvider_ForPluginProviders&");
    Shiboken::Conversions::registerConverterName(converter, typeid(::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders).name());
    Shiboken::Conversions::registerConverterName(converter, typeid(::RosPluginlibPluginProvider_ForPluginProvidersWrapper).name());


    Shiboken::ObjectType::setTypeDiscoveryFunctionV2(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_TypeF(), &Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_typeDiscovery);

    PySide::Signal::registerSignals(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_TypeF(), &::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::staticMetaObject);

    Shiboken::ObjectType::setSubTypeInitHook(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_TypeF(), &PySide::initQObjectSubType);
    PySide::initDynamicMetaObject(Sbk_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders_TypeF(), &::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders::staticMetaObject, sizeof(RosPluginlibPluginProvider_ForPluginProvidersWrapper));
}
