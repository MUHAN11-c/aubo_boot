
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
#include "qt_gui_cpp_pluginbridge_wrapper.h"

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

void PluginBridgeWrapper::resetPyMethodCache()
{
    std::fill_n(m_PyMethodCache, sizeof(m_PyMethodCache) / sizeof(m_PyMethodCache[0]), false);
}

PluginBridgeWrapper::PluginBridgeWrapper() : qt_gui_cpp::PluginBridge()
{
    resetPyMethodCache();
    // ... middle
}

void PluginBridgeWrapper::childEvent(QChildEvent * event)
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

void PluginBridgeWrapper::connectNotify(const QMetaMethod & signal)
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

void PluginBridgeWrapper::customEvent(QEvent * event)
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

void PluginBridgeWrapper::disconnectNotify(const QMetaMethod & signal)
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

bool PluginBridgeWrapper::event(QEvent * event)
{
    if (m_PyMethodCache[4])
        return this->::QObject::event(event);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return false;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "event";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[4] = true;
        return this->::QObject::event(event);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(N)",
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QEVENT_IDX]), event)
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "PluginBridge.event", "bool", Py_TYPE(pyResult)->tp_name);
        return false;
    }
    bool cppResult;
    pythonToCpp(pyResult, &cppResult);
    if (invalidateArg1)
        Shiboken::Object::invalidate(PyTuple_GET_ITEM(pyArgs, 0));
    return cppResult;
}

bool PluginBridgeWrapper::eventFilter(QObject * watched, QEvent * event)
{
    if (m_PyMethodCache[5])
        return this->::QObject::eventFilter(watched, event);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return false;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "eventFilter";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[5] = true;
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "PluginBridge.eventFilter", "bool", Py_TYPE(pyResult)->tp_name);
        return false;
    }
    bool cppResult;
    pythonToCpp(pyResult, &cppResult);
    if (invalidateArg2)
        Shiboken::Object::invalidate(PyTuple_GET_ITEM(pyArgs, 1));
    return cppResult;
}

bool PluginBridgeWrapper::has_configuration() const
{
    if (m_PyMethodCache[6])
        return this->::qt_gui_cpp::PluginBridge::has_configuration();
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return false;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "has_configuration";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[6] = true;
        return this->::qt_gui_cpp::PluginBridge::has_configuration();
    }

    Shiboken::AutoDecRef pyArgs(PyTuple_New(0));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return false;
    }
    // Check return type
    PythonToCppFunc pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(Shiboken::Conversions::PrimitiveTypeConverter<bool>(), pyResult);
    if (!pythonToCpp) {
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "PluginBridge.has_configuration", "bool", Py_TYPE(pyResult)->tp_name);
        return false;
    }
    bool cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

bool PluginBridgeWrapper::load_plugin(qt_gui_cpp::PluginProvider * provider, const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context)
{
    if (m_PyMethodCache[7])
        return this->::qt_gui_cpp::PluginBridge::load_plugin(provider, plugin_id, plugin_context);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return false;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "load_plugin";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[7] = true;
        return this->::qt_gui_cpp::PluginBridge::load_plugin(provider, plugin_id, plugin_context);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(NNN)",
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX]), provider),
    Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], &plugin_id),
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]), plugin_context)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return false;
    }
    // Check return type
    PythonToCppFunc pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(Shiboken::Conversions::PrimitiveTypeConverter<bool>(), pyResult);
    if (!pythonToCpp) {
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "PluginBridge.load_plugin", "bool", Py_TYPE(pyResult)->tp_name);
        return false;
    }
    bool cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

void PluginBridgeWrapper::restore_settings(QObject * plugin_settings, QObject * instance_settings)
{
    if (m_PyMethodCache[9]) {
        return this->::qt_gui_cpp::PluginBridge::restore_settings(plugin_settings, instance_settings);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "restore_settings";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[9] = true;
        return this->::qt_gui_cpp::PluginBridge::restore_settings(plugin_settings, instance_settings);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(NN)",
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), plugin_settings),
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), instance_settings)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

void PluginBridgeWrapper::save_settings(QObject * plugin_settings, QObject * instance_settings)
{
    if (m_PyMethodCache[10]) {
        return this->::qt_gui_cpp::PluginBridge::save_settings(plugin_settings, instance_settings);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "save_settings";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[10] = true;
        return this->::qt_gui_cpp::PluginBridge::save_settings(plugin_settings, instance_settings);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(NN)",
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), plugin_settings),
    Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), instance_settings)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

void PluginBridgeWrapper::shutdown_plugin()
{
    if (m_PyMethodCache[11]) {
        return this->::qt_gui_cpp::PluginBridge::shutdown_plugin();
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "shutdown_plugin";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[11] = true;
        return this->::qt_gui_cpp::PluginBridge::shutdown_plugin();
    }

    Shiboken::AutoDecRef pyArgs(PyTuple_New(0));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

void PluginBridgeWrapper::timerEvent(QTimerEvent * event)
{
    if (m_PyMethodCache[12]) {
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
        m_PyMethodCache[12] = true;
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

void PluginBridgeWrapper::trigger_configuration()
{
    if (m_PyMethodCache[13]) {
        return this->::qt_gui_cpp::PluginBridge::trigger_configuration();
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "trigger_configuration";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[13] = true;
        return this->::qt_gui_cpp::PluginBridge::trigger_configuration();
    }

    Shiboken::AutoDecRef pyArgs(PyTuple_New(0));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

void PluginBridgeWrapper::unload_plugin()
{
    if (m_PyMethodCache[14]) {
        return this->::qt_gui_cpp::PluginBridge::unload_plugin();
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "unload_plugin";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[14] = true;
        return this->::qt_gui_cpp::PluginBridge::unload_plugin();
    }

    Shiboken::AutoDecRef pyArgs(PyTuple_New(0));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

const QMetaObject *PluginBridgeWrapper::metaObject() const
{
    if (QObject::d_ptr->metaObject)
        return QObject::d_ptr->dynamicMetaObject();
    SbkObject *pySelf = Shiboken::BindingManager::instance().retrieveWrapper(this);
    if (pySelf == nullptr)
        return qt_gui_cpp::PluginBridge::metaObject();
    return PySide::SignalManager::retrieveMetaObject(reinterpret_cast<PyObject *>(pySelf));
}

int PluginBridgeWrapper::qt_metacall(QMetaObject::Call call, int id, void **args)
{
    int result = qt_gui_cpp::PluginBridge::qt_metacall(call, id, args);
    return result < 0 ? result : PySide::SignalManager::qt_metacall(this, call, id, args);
}

void *PluginBridgeWrapper::qt_metacast(const char *_clname)
{
        if (!_clname) return {};
        SbkObject *pySelf = Shiboken::BindingManager::instance().retrieveWrapper(this);
        if (pySelf && PySide::inherits(Py_TYPE(pySelf), _clname))
                return static_cast<void *>(const_cast< PluginBridgeWrapper *>(this));
        return qt_gui_cpp::PluginBridge::qt_metacast(_clname);
}

PluginBridgeWrapper::~PluginBridgeWrapper()
{
    SbkObject *wrapper = Shiboken::BindingManager::instance().retrieveWrapper(this);
    Shiboken::Object::destroy(wrapper, this);
}

// Target ---------------------------------------------------------

extern "C" {
static int
Sbk_qt_gui_cpp_PluginBridge_Init(PyObject *self, PyObject *args, PyObject *kwds)
{
    const char **argNames{};
    const QMetaObject *metaObject;
    SbkObject *sbkSelf = reinterpret_cast<SbkObject *>(self);
    if (Shiboken::Object::isUserType(self) && !Shiboken::ObjectType::canCallConstructor(self->ob_type, Shiboken::SbkType< ::qt_gui_cpp::PluginBridge >()))
        return -1;

    ::PluginBridgeWrapper *cptr{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // PluginBridge()
            void *addr = PySide::nextQObjectMemoryAddr();
            if (addr) {
                cptr = new (addr) ::PluginBridgeWrapper();
                PySide::setNextQObjectMemoryAddr(0);
            } else {
                cptr = new ::PluginBridgeWrapper();
            }

        }
    }

    if (PyErr_Occurred() || !Shiboken::Object::setCppPointer(sbkSelf, Shiboken::SbkType< ::qt_gui_cpp::PluginBridge >(), cptr)) {
        delete cptr;
        return -1;
    }
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
}

static PyObject *Sbk_qt_gui_cpp_PluginBridgeFunc_has_configuration(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginBridge *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINBRIDGE_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // has_configuration()const
            bool cppResult = Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? const_cast<const ::qt_gui_cpp::PluginBridge *>(cppSelf)->::qt_gui_cpp::PluginBridge::has_configuration()
                : const_cast<const ::qt_gui_cpp::PluginBridge *>(cppSelf)->has_configuration();
            pyResult = Shiboken::Conversions::copyToPython(Shiboken::Conversions::PrimitiveTypeConverter<bool>(), &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;
}

static PyObject *Sbk_qt_gui_cpp_PluginBridgeFunc_load_plugin(PyObject *self, PyObject *args)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginBridge *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINBRIDGE_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0, 0};

    // invalid argument lengths


    if (!PyArg_UnpackTuple(args, "load_plugin", 3, 3, &(pyArgs[0]), &(pyArgs[1]), &(pyArgs[2])))
        return {};


    // Overloaded function decisor
    // 0: PluginBridge::load_plugin(qt_gui_cpp::PluginProvider*,QString,qt_gui_cpp::PluginContext*)
    if (numArgs == 3
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX]), (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[1])))
        && (pythonToCpp[2] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]), (pyArgs[2])))) {
        overloadId = 0; // load_plugin(qt_gui_cpp::PluginProvider*,QString,qt_gui_cpp::PluginContext*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginBridgeFunc_load_plugin_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArgs[0]))
            return {};
        ::qt_gui_cpp::PluginProvider *cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        ::QString cppArg1;
        pythonToCpp[1](pyArgs[1], &cppArg1);
        if (!Shiboken::Object::isValid(pyArgs[2]))
            return {};
        ::qt_gui_cpp::PluginContext *cppArg2;
        pythonToCpp[2](pyArgs[2], &cppArg2);

        if (!PyErr_Occurred()) {
            // load_plugin(qt_gui_cpp::PluginProvider*,QString,qt_gui_cpp::PluginContext*)
            bool cppResult = Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::PluginBridge::load_plugin(cppArg0, cppArg1, cppArg2)
                : cppSelf->load_plugin(cppArg0, cppArg1, cppArg2);
            pyResult = Shiboken::Conversions::copyToPython(Shiboken::Conversions::PrimitiveTypeConverter<bool>(), &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_PluginBridgeFunc_load_plugin_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginBridge.load_plugin");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginBridgeFunc_restore_settings(PyObject *self, PyObject *args)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginBridge *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINBRIDGE_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0};

    // invalid argument lengths


    if (!PyArg_UnpackTuple(args, "restore_settings", 2, 2, &(pyArgs[0]), &(pyArgs[1])))
        return {};


    // Overloaded function decisor
    // 0: PluginBridge::restore_settings(QObject*,QObject*)
    if (numArgs == 2
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), (pyArgs[1])))) {
        overloadId = 0; // restore_settings(QObject*,QObject*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginBridgeFunc_restore_settings_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArgs[0]))
            return {};
        ::QObject *cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        if (!Shiboken::Object::isValid(pyArgs[1]))
            return {};
        ::QObject *cppArg1;
        pythonToCpp[1](pyArgs[1], &cppArg1);

        if (!PyErr_Occurred()) {
            // restore_settings(QObject*,QObject*)
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::PluginBridge::restore_settings(cppArg0, cppArg1)
                : cppSelf->restore_settings(cppArg0, cppArg1);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_PluginBridgeFunc_restore_settings_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginBridge.restore_settings");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginBridgeFunc_save_settings(PyObject *self, PyObject *args)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginBridge *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINBRIDGE_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0};

    // invalid argument lengths


    if (!PyArg_UnpackTuple(args, "save_settings", 2, 2, &(pyArgs[0]), &(pyArgs[1])))
        return {};


    // Overloaded function decisor
    // 0: PluginBridge::save_settings(QObject*,QObject*)
    if (numArgs == 2
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), (pyArgs[1])))) {
        overloadId = 0; // save_settings(QObject*,QObject*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginBridgeFunc_save_settings_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArgs[0]))
            return {};
        ::QObject *cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        if (!Shiboken::Object::isValid(pyArgs[1]))
            return {};
        ::QObject *cppArg1;
        pythonToCpp[1](pyArgs[1], &cppArg1);

        if (!PyErr_Occurred()) {
            // save_settings(QObject*,QObject*)
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::PluginBridge::save_settings(cppArg0, cppArg1)
                : cppSelf->save_settings(cppArg0, cppArg1);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_PluginBridgeFunc_save_settings_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginBridge.save_settings");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginBridgeFunc_shutdown_plugin(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginBridge *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINBRIDGE_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // shutdown_plugin()
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::PluginBridge::shutdown_plugin()
                : cppSelf->shutdown_plugin();
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;
}

static PyObject *Sbk_qt_gui_cpp_PluginBridgeFunc_trigger_configuration(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginBridge *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINBRIDGE_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // trigger_configuration()
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::PluginBridge::trigger_configuration()
                : cppSelf->trigger_configuration();
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;
}

static PyObject *Sbk_qt_gui_cpp_PluginBridgeFunc_unload_plugin(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginBridge *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINBRIDGE_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // unload_plugin()
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::PluginBridge::unload_plugin()
                : cppSelf->unload_plugin();
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;
}


static const char *Sbk_qt_gui_cpp_PluginBridge_PropertyStrings[] = {
    nullptr // Sentinel
};

static PyMethodDef Sbk_qt_gui_cpp_PluginBridge_methods[] = {
    {"has_configuration", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginBridgeFunc_has_configuration), METH_NOARGS},
    {"load_plugin", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginBridgeFunc_load_plugin), METH_VARARGS},
    {"restore_settings", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginBridgeFunc_restore_settings), METH_VARARGS},
    {"save_settings", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginBridgeFunc_save_settings), METH_VARARGS},
    {"shutdown_plugin", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginBridgeFunc_shutdown_plugin), METH_NOARGS},
    {"trigger_configuration", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginBridgeFunc_trigger_configuration), METH_NOARGS},
    {"unload_plugin", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginBridgeFunc_unload_plugin), METH_NOARGS},

    {nullptr, nullptr} // Sentinel
};

static int Sbk_qt_gui_cpp_PluginBridge_setattro(PyObject *self, PyObject *name, PyObject *value)
{
    PySide::Feature::Select(self);
    if (value && PyCallable_Check(value)) {
        auto plain_inst = reinterpret_cast< ::qt_gui_cpp::PluginBridge *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINBRIDGE_IDX], reinterpret_cast<SbkObject *>(self)));
        auto inst = dynamic_cast<PluginBridgeWrapper *>(plain_inst);
        if (inst)
            inst->resetPyMethodCache();
    }
    Shiboken::AutoDecRef pp(reinterpret_cast<PyObject *>(PySide::Property::getObject(self, name)));
    if (!pp.isNull())
        return PySide::Property::setValue(reinterpret_cast<PySideProperty *>(pp.object()), self, value);
    return PyObject_GenericSetAttr(self, name, value);
}

} // extern "C"

static int Sbk_qt_gui_cpp_PluginBridge_traverse(PyObject *self, visitproc visit, void *arg)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_traverse(self, visit, arg);
}
static int Sbk_qt_gui_cpp_PluginBridge_clear(PyObject *self)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_clear(self);
}
// Class Definition -----------------------------------------------
extern "C" {
static SbkObjectType *_Sbk_qt_gui_cpp_PluginBridge_Type = nullptr;
static SbkObjectType *Sbk_qt_gui_cpp_PluginBridge_TypeF(void)
{
    return _Sbk_qt_gui_cpp_PluginBridge_Type;
}

static PyType_Slot Sbk_qt_gui_cpp_PluginBridge_slots[] = {
    {Py_tp_base,        nullptr}, // inserted by introduceWrapperType
    {Py_tp_dealloc,     reinterpret_cast<void *>(&SbkDeallocWrapper)},
    {Py_tp_repr,        nullptr},
    {Py_tp_hash,        nullptr},
    {Py_tp_call,        nullptr},
    {Py_tp_str,         nullptr},
    {Py_tp_getattro,    nullptr},
    {Py_tp_setattro,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginBridge_setattro)},
    {Py_tp_traverse,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginBridge_traverse)},
    {Py_tp_clear,       reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginBridge_clear)},
    {Py_tp_richcompare, nullptr},
    {Py_tp_iter,        nullptr},
    {Py_tp_iternext,    nullptr},
    {Py_tp_methods,     reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginBridge_methods)},
    {Py_tp_getset,      nullptr},
    {Py_tp_init,        reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginBridge_Init)},
    {Py_tp_new,         reinterpret_cast<void *>(SbkObjectTpNew)},
    {0, nullptr}
};
static PyType_Spec Sbk_qt_gui_cpp_PluginBridge_spec = {
    "1:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginBridge",
    sizeof(SbkObject),
    0,
    Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_CHECKTYPES|Py_TPFLAGS_HAVE_GC,
    Sbk_qt_gui_cpp_PluginBridge_slots
};

} //extern "C"

static void *Sbk_qt_gui_cpp_PluginBridge_typeDiscovery(void *cptr, SbkObjectType *instanceType)
{
    if (instanceType == reinterpret_cast<SbkObjectType *>(Shiboken::SbkType< ::QObject >()))
        return dynamic_cast< ::qt_gui_cpp::PluginBridge *>(reinterpret_cast< ::QObject *>(cptr));
    return {};
}


// Type conversion functions.

// Python to C++ pointer conversion - returns the C++ object of the Python wrapper (keeps object identity).
static void PluginBridge_PythonToCpp_PluginBridge_PTR(PyObject *pyIn, void *cppOut) {
    Shiboken::Conversions::pythonToCppPointer(Sbk_qt_gui_cpp_PluginBridge_TypeF(), pyIn, cppOut);
}
static PythonToCppFunc is_PluginBridge_PythonToCpp_PluginBridge_PTR_Convertible(PyObject *pyIn) {
    if (pyIn == Py_None)
        return Shiboken::Conversions::nonePythonToCppNullPtr;
    if (PyObject_TypeCheck(pyIn, reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_PluginBridge_TypeF())))
        return PluginBridge_PythonToCpp_PluginBridge_PTR;
    return {};
}

// C++ to Python pointer conversion - tries to find the Python wrapper for the C++ object (keeps object identity).
static PyObject *PluginBridge_PTR_CppToPython_PluginBridge(const void *cppIn) {
    return PySide::getWrapperForQObject(reinterpret_cast<::qt_gui_cpp::PluginBridge *>(const_cast<void *>(cppIn)), Sbk_qt_gui_cpp_PluginBridge_TypeF());

}

// The signatures string for the functions.
// Multiple signatures have their index "n:" in front.
static const char *qt_gui_cpp_PluginBridge_SignatureStrings[] = {
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginBridge(self)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginBridge.has_configuration(self)->bool",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginBridge.load_plugin(self,provider:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider,plugin_id:QString,plugin_context:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext)->bool",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginBridge.restore_settings(self,plugin_settings:PySide2.QtCore.QObject,instance_settings:PySide2.QtCore.QObject)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginBridge.save_settings(self,plugin_settings:PySide2.QtCore.QObject,instance_settings:PySide2.QtCore.QObject)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginBridge.shutdown_plugin(self)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginBridge.trigger_configuration(self)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginBridge.unload_plugin(self)",
    nullptr}; // Sentinel

void init_qt_gui_cpp_PluginBridge(PyObject *enclosingClass)
{
    _Sbk_qt_gui_cpp_PluginBridge_Type = Shiboken::ObjectType::introduceWrapperType(
        enclosingClass,
        "PluginBridge",
        "qt_gui_cpp::PluginBridge*",
        &Sbk_qt_gui_cpp_PluginBridge_spec,
        &Shiboken::callCppDestructor< ::qt_gui_cpp::PluginBridge >,
        reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]),
        0,
        Shiboken::ObjectType::WrapperFlags::InnerClass    );
    
    auto pyType = reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_PluginBridge_Type);
    InitSignatureStrings(pyType, qt_gui_cpp_PluginBridge_SignatureStrings);
    SbkObjectType_SetPropertyStrings(reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_PluginBridge_Type), Sbk_qt_gui_cpp_PluginBridge_PropertyStrings);
    Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINBRIDGE_IDX]
        = reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_PluginBridge_TypeF());

    // Register Converter
    SbkConverter *converter = Shiboken::Conversions::createConverter(Sbk_qt_gui_cpp_PluginBridge_TypeF(),
        PluginBridge_PythonToCpp_PluginBridge_PTR,
        is_PluginBridge_PythonToCpp_PluginBridge_PTR_Convertible,
        PluginBridge_PTR_CppToPython_PluginBridge);

    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::PluginBridge");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::PluginBridge*");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::PluginBridge&");
    Shiboken::Conversions::registerConverterName(converter, "PluginBridge");
    Shiboken::Conversions::registerConverterName(converter, "PluginBridge*");
    Shiboken::Conversions::registerConverterName(converter, "PluginBridge&");
    Shiboken::Conversions::registerConverterName(converter, typeid(::qt_gui_cpp::PluginBridge).name());
    Shiboken::Conversions::registerConverterName(converter, typeid(::PluginBridgeWrapper).name());


    Shiboken::ObjectType::setTypeDiscoveryFunctionV2(Sbk_qt_gui_cpp_PluginBridge_TypeF(), &Sbk_qt_gui_cpp_PluginBridge_typeDiscovery);

    PySide::Signal::registerSignals(Sbk_qt_gui_cpp_PluginBridge_TypeF(), &::qt_gui_cpp::PluginBridge::staticMetaObject);

    Shiboken::ObjectType::setSubTypeInitHook(Sbk_qt_gui_cpp_PluginBridge_TypeF(), &PySide::initQObjectSubType);
    PySide::initDynamicMetaObject(Sbk_qt_gui_cpp_PluginBridge_TypeF(), &::qt_gui_cpp::PluginBridge::staticMetaObject, sizeof(PluginBridgeWrapper));
}
