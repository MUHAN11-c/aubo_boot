
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
#include "qt_gui_cpp_plugin_wrapper.h"

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

void PluginWrapper::resetPyMethodCache()
{
    std::fill_n(m_PyMethodCache, sizeof(m_PyMethodCache) / sizeof(m_PyMethodCache[0]), false);
}

PluginWrapper::PluginWrapper() : qt_gui_cpp::Plugin()
{
    resetPyMethodCache();
    // ... middle
}

void PluginWrapper::childEvent(QChildEvent * event)
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

void PluginWrapper::connectNotify(const QMetaMethod & signal)
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

void PluginWrapper::customEvent(QEvent * event)
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

void PluginWrapper::disconnectNotify(const QMetaMethod & signal)
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

bool PluginWrapper::event(QEvent * event)
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "Plugin.event", "bool", Py_TYPE(pyResult)->tp_name);
        return false;
    }
    bool cppResult;
    pythonToCpp(pyResult, &cppResult);
    if (invalidateArg1)
        Shiboken::Object::invalidate(PyTuple_GET_ITEM(pyArgs, 0));
    return cppResult;
}

bool PluginWrapper::eventFilter(QObject * watched, QEvent * event)
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "Plugin.eventFilter", "bool", Py_TYPE(pyResult)->tp_name);
        return false;
    }
    bool cppResult;
    pythonToCpp(pyResult, &cppResult);
    if (invalidateArg2)
        Shiboken::Object::invalidate(PyTuple_GET_ITEM(pyArgs, 1));
    return cppResult;
}

bool PluginWrapper::hasConfiguration() const
{
    if (m_PyMethodCache[6])
        return this->::qt_gui_cpp::Plugin::hasConfiguration();
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return false;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "hasConfiguration";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[6] = true;
        return this->::qt_gui_cpp::Plugin::hasConfiguration();
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "Plugin.hasConfiguration", "bool", Py_TYPE(pyResult)->tp_name);
        return false;
    }
    bool cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

void PluginWrapper::initPlugin(qt_gui_cpp::PluginContext & arg__1)
{
    if (m_PyMethodCache[7]) {
        return this->::qt_gui_cpp::Plugin::initPlugin(arg__1);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "initPlugin";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[7] = true;
        return this->::qt_gui_cpp::Plugin::initPlugin(arg__1);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(N)",
    Shiboken::Conversions::referenceToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]), &arg__1)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

void PluginWrapper::restoreSettings(const qt_gui_cpp::Settings & arg__1, const qt_gui_cpp::Settings & arg__2)
{
    if (m_PyMethodCache[9]) {
        return this->::qt_gui_cpp::Plugin::restoreSettings(arg__1, arg__2);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "restoreSettings";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[9] = true;
        return this->::qt_gui_cpp::Plugin::restoreSettings(arg__1, arg__2);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(NN)",
    Shiboken::Conversions::referenceToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX]), &arg__1),
    Shiboken::Conversions::referenceToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX]), &arg__2)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

void PluginWrapper::saveSettings(qt_gui_cpp::Settings & arg__1, qt_gui_cpp::Settings & arg__2) const
{
    if (m_PyMethodCache[10]) {
        return this->::qt_gui_cpp::Plugin::saveSettings(arg__1, arg__2);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "saveSettings";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[10] = true;
        return this->::qt_gui_cpp::Plugin::saveSettings(arg__1, arg__2);
    }

    Shiboken::AutoDecRef pyArgs(Py_BuildValue("(NN)",
    Shiboken::Conversions::referenceToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX]), &arg__1),
    Shiboken::Conversions::referenceToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX]), &arg__2)
    ));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

void PluginWrapper::shutdownPlugin()
{
    if (m_PyMethodCache[11]) {
        return this->::qt_gui_cpp::Plugin::shutdownPlugin();
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "shutdownPlugin";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[11] = true;
        return this->::qt_gui_cpp::Plugin::shutdownPlugin();
    }

    Shiboken::AutoDecRef pyArgs(PyTuple_New(0));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

void PluginWrapper::timerEvent(QTimerEvent * event)
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

void PluginWrapper::triggerConfiguration()
{
    if (m_PyMethodCache[13]) {
        return this->::qt_gui_cpp::Plugin::triggerConfiguration();
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "triggerConfiguration";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[13] = true;
        return this->::qt_gui_cpp::Plugin::triggerConfiguration();
    }

    Shiboken::AutoDecRef pyArgs(PyTuple_New(0));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

const QMetaObject *PluginWrapper::metaObject() const
{
    if (QObject::d_ptr->metaObject)
        return QObject::d_ptr->dynamicMetaObject();
    SbkObject *pySelf = Shiboken::BindingManager::instance().retrieveWrapper(this);
    if (pySelf == nullptr)
        return qt_gui_cpp::Plugin::metaObject();
    return PySide::SignalManager::retrieveMetaObject(reinterpret_cast<PyObject *>(pySelf));
}

int PluginWrapper::qt_metacall(QMetaObject::Call call, int id, void **args)
{
    int result = qt_gui_cpp::Plugin::qt_metacall(call, id, args);
    return result < 0 ? result : PySide::SignalManager::qt_metacall(this, call, id, args);
}

void *PluginWrapper::qt_metacast(const char *_clname)
{
        if (!_clname) return {};
        SbkObject *pySelf = Shiboken::BindingManager::instance().retrieveWrapper(this);
        if (pySelf && PySide::inherits(Py_TYPE(pySelf), _clname))
                return static_cast<void *>(const_cast< PluginWrapper *>(this));
        return qt_gui_cpp::Plugin::qt_metacast(_clname);
}

PluginWrapper::~PluginWrapper()
{
    SbkObject *wrapper = Shiboken::BindingManager::instance().retrieveWrapper(this);
    Shiboken::Object::destroy(wrapper, this);
}

// Target ---------------------------------------------------------

extern "C" {
static int
Sbk_qt_gui_cpp_Plugin_Init(PyObject *self, PyObject *args, PyObject *kwds)
{
    const char **argNames{};
    const QMetaObject *metaObject;
    SbkObject *sbkSelf = reinterpret_cast<SbkObject *>(self);
    if (Shiboken::Object::isUserType(self) && !Shiboken::ObjectType::canCallConstructor(self->ob_type, Shiboken::SbkType< ::qt_gui_cpp::Plugin >()))
        return -1;

    ::PluginWrapper *cptr{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // Plugin()
            void *addr = PySide::nextQObjectMemoryAddr();
            if (addr) {
                cptr = new (addr) ::PluginWrapper();
                PySide::setNextQObjectMemoryAddr(0);
            } else {
                cptr = new ::PluginWrapper();
            }

        }
    }

    if (PyErr_Occurred() || !Shiboken::Object::setCppPointer(sbkSelf, Shiboken::SbkType< ::qt_gui_cpp::Plugin >(), cptr)) {
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

static PyObject *Sbk_qt_gui_cpp_PluginFunc_hasConfiguration(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Plugin *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // hasConfiguration()const
            bool cppResult = Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? const_cast<const ::qt_gui_cpp::Plugin *>(cppSelf)->::qt_gui_cpp::Plugin::hasConfiguration()
                : const_cast<const ::qt_gui_cpp::Plugin *>(cppSelf)->hasConfiguration();
            pyResult = Shiboken::Conversions::copyToPython(Shiboken::Conversions::PrimitiveTypeConverter<bool>(), &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;
}

static PyObject *Sbk_qt_gui_cpp_PluginFunc_initPlugin(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Plugin *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: Plugin::initPlugin(qt_gui_cpp::PluginContext&)
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppReferenceConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]), (pyArg)))) {
        overloadId = 0; // initPlugin(qt_gui_cpp::PluginContext&)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginFunc_initPlugin_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArg))
            return {};
        ::qt_gui_cpp::PluginContext *cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // initPlugin(qt_gui_cpp::PluginContext&)
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::Plugin::initPlugin(*cppArg0)
                : cppSelf->initPlugin(*cppArg0);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_PluginFunc_initPlugin_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin.initPlugin");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginFunc_restoreSettings(PyObject *self, PyObject *args)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Plugin *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0};

    // invalid argument lengths


    if (!PyArg_UnpackTuple(args, "restoreSettings", 2, 2, &(pyArgs[0]), &(pyArgs[1])))
        return {};


    // Overloaded function decisor
    // 0: Plugin::restoreSettings(qt_gui_cpp::Settings,qt_gui_cpp::Settings)
    if (numArgs == 2
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppReferenceConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX]), (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppReferenceConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX]), (pyArgs[1])))) {
        overloadId = 0; // restoreSettings(qt_gui_cpp::Settings,qt_gui_cpp::Settings)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginFunc_restoreSettings_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArgs[0]))
            return {};
        ::qt_gui_cpp::Settings *cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        if (!Shiboken::Object::isValid(pyArgs[1]))
            return {};
        ::qt_gui_cpp::Settings *cppArg1;
        pythonToCpp[1](pyArgs[1], &cppArg1);

        if (!PyErr_Occurred()) {
            // restoreSettings(qt_gui_cpp::Settings,qt_gui_cpp::Settings)
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::Plugin::restoreSettings(*cppArg0, *cppArg1)
                : cppSelf->restoreSettings(*cppArg0, *cppArg1);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_PluginFunc_restoreSettings_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin.restoreSettings");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginFunc_saveSettings(PyObject *self, PyObject *args)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Plugin *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0};

    // invalid argument lengths


    if (!PyArg_UnpackTuple(args, "saveSettings", 2, 2, &(pyArgs[0]), &(pyArgs[1])))
        return {};


    // Overloaded function decisor
    // 0: Plugin::saveSettings(qt_gui_cpp::Settings&,qt_gui_cpp::Settings&)const
    if (numArgs == 2
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppReferenceConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX]), (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppReferenceConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX]), (pyArgs[1])))) {
        overloadId = 0; // saveSettings(qt_gui_cpp::Settings&,qt_gui_cpp::Settings&)const
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginFunc_saveSettings_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArgs[0]))
            return {};
        ::qt_gui_cpp::Settings *cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        if (!Shiboken::Object::isValid(pyArgs[1]))
            return {};
        ::qt_gui_cpp::Settings *cppArg1;
        pythonToCpp[1](pyArgs[1], &cppArg1);

        if (!PyErr_Occurred()) {
            // saveSettings(qt_gui_cpp::Settings&,qt_gui_cpp::Settings&)const
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? const_cast<const ::qt_gui_cpp::Plugin *>(cppSelf)->::qt_gui_cpp::Plugin::saveSettings(*cppArg0, *cppArg1)
                : const_cast<const ::qt_gui_cpp::Plugin *>(cppSelf)->saveSettings(*cppArg0, *cppArg1);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_PluginFunc_saveSettings_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin.saveSettings");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginFunc_shutdownPlugin(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Plugin *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // shutdownPlugin()
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::Plugin::shutdownPlugin()
                : cppSelf->shutdownPlugin();
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;
}

static PyObject *Sbk_qt_gui_cpp_PluginFunc_triggerConfiguration(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Plugin *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // triggerConfiguration()
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::Plugin::triggerConfiguration()
                : cppSelf->triggerConfiguration();
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;
}


static const char *Sbk_qt_gui_cpp_Plugin_PropertyStrings[] = {
    nullptr // Sentinel
};

static PyMethodDef Sbk_qt_gui_cpp_Plugin_methods[] = {
    {"hasConfiguration", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginFunc_hasConfiguration), METH_NOARGS},
    {"initPlugin", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginFunc_initPlugin), METH_O},
    {"restoreSettings", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginFunc_restoreSettings), METH_VARARGS},
    {"saveSettings", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginFunc_saveSettings), METH_VARARGS},
    {"shutdownPlugin", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginFunc_shutdownPlugin), METH_NOARGS},
    {"triggerConfiguration", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginFunc_triggerConfiguration), METH_NOARGS},

    {nullptr, nullptr} // Sentinel
};

static int Sbk_qt_gui_cpp_Plugin_setattro(PyObject *self, PyObject *name, PyObject *value)
{
    PySide::Feature::Select(self);
    if (value && PyCallable_Check(value)) {
        auto plain_inst = reinterpret_cast< ::qt_gui_cpp::Plugin *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX], reinterpret_cast<SbkObject *>(self)));
        auto inst = dynamic_cast<PluginWrapper *>(plain_inst);
        if (inst)
            inst->resetPyMethodCache();
    }
    Shiboken::AutoDecRef pp(reinterpret_cast<PyObject *>(PySide::Property::getObject(self, name)));
    if (!pp.isNull())
        return PySide::Property::setValue(reinterpret_cast<PySideProperty *>(pp.object()), self, value);
    return PyObject_GenericSetAttr(self, name, value);
}

} // extern "C"

static int Sbk_qt_gui_cpp_Plugin_traverse(PyObject *self, visitproc visit, void *arg)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_traverse(self, visit, arg);
}
static int Sbk_qt_gui_cpp_Plugin_clear(PyObject *self)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_clear(self);
}
// Class Definition -----------------------------------------------
extern "C" {
static SbkObjectType *_Sbk_qt_gui_cpp_Plugin_Type = nullptr;
static SbkObjectType *Sbk_qt_gui_cpp_Plugin_TypeF(void)
{
    return _Sbk_qt_gui_cpp_Plugin_Type;
}

static PyType_Slot Sbk_qt_gui_cpp_Plugin_slots[] = {
    {Py_tp_base,        nullptr}, // inserted by introduceWrapperType
    {Py_tp_dealloc,     reinterpret_cast<void *>(&SbkDeallocWrapper)},
    {Py_tp_repr,        nullptr},
    {Py_tp_hash,        nullptr},
    {Py_tp_call,        nullptr},
    {Py_tp_str,         nullptr},
    {Py_tp_getattro,    nullptr},
    {Py_tp_setattro,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_Plugin_setattro)},
    {Py_tp_traverse,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_Plugin_traverse)},
    {Py_tp_clear,       reinterpret_cast<void *>(Sbk_qt_gui_cpp_Plugin_clear)},
    {Py_tp_richcompare, nullptr},
    {Py_tp_iter,        nullptr},
    {Py_tp_iternext,    nullptr},
    {Py_tp_methods,     reinterpret_cast<void *>(Sbk_qt_gui_cpp_Plugin_methods)},
    {Py_tp_getset,      nullptr},
    {Py_tp_init,        reinterpret_cast<void *>(Sbk_qt_gui_cpp_Plugin_Init)},
    {Py_tp_new,         reinterpret_cast<void *>(SbkObjectTpNew)},
    {0, nullptr}
};
static PyType_Spec Sbk_qt_gui_cpp_Plugin_spec = {
    "1:libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin",
    sizeof(SbkObject),
    0,
    Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_CHECKTYPES|Py_TPFLAGS_HAVE_GC,
    Sbk_qt_gui_cpp_Plugin_slots
};

} //extern "C"

static void *Sbk_qt_gui_cpp_Plugin_typeDiscovery(void *cptr, SbkObjectType *instanceType)
{
    if (instanceType == reinterpret_cast<SbkObjectType *>(Shiboken::SbkType< ::QObject >()))
        return dynamic_cast< ::qt_gui_cpp::Plugin *>(reinterpret_cast< ::QObject *>(cptr));
    return {};
}


// Type conversion functions.

// Python to C++ pointer conversion - returns the C++ object of the Python wrapper (keeps object identity).
static void Plugin_PythonToCpp_Plugin_PTR(PyObject *pyIn, void *cppOut) {
    Shiboken::Conversions::pythonToCppPointer(Sbk_qt_gui_cpp_Plugin_TypeF(), pyIn, cppOut);
}
static PythonToCppFunc is_Plugin_PythonToCpp_Plugin_PTR_Convertible(PyObject *pyIn) {
    if (pyIn == Py_None)
        return Shiboken::Conversions::nonePythonToCppNullPtr;
    if (PyObject_TypeCheck(pyIn, reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_Plugin_TypeF())))
        return Plugin_PythonToCpp_Plugin_PTR;
    return {};
}

// C++ to Python pointer conversion - tries to find the Python wrapper for the C++ object (keeps object identity).
static PyObject *Plugin_PTR_CppToPython_Plugin(const void *cppIn) {
    return PySide::getWrapperForQObject(reinterpret_cast<::qt_gui_cpp::Plugin *>(const_cast<void *>(cppIn)), Sbk_qt_gui_cpp_Plugin_TypeF());

}

// The signatures string for the functions.
// Multiple signatures have their index "n:" in front.
static const char *qt_gui_cpp_Plugin_SignatureStrings[] = {
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin(self)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin.hasConfiguration(self)->bool",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin.initPlugin(self,arg__1:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin.restoreSettings(self,arg__1:libqt_gui_cpp_shiboken.qt_gui_cpp.Settings,arg__2:libqt_gui_cpp_shiboken.qt_gui_cpp.Settings)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin.saveSettings(self,arg__1:libqt_gui_cpp_shiboken.qt_gui_cpp.Settings,arg__2:libqt_gui_cpp_shiboken.qt_gui_cpp.Settings)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin.shutdownPlugin(self)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin.triggerConfiguration(self)",
    nullptr}; // Sentinel

void init_qt_gui_cpp_Plugin(PyObject *enclosingClass)
{
    _Sbk_qt_gui_cpp_Plugin_Type = Shiboken::ObjectType::introduceWrapperType(
        enclosingClass,
        "Plugin",
        "qt_gui_cpp::Plugin*",
        &Sbk_qt_gui_cpp_Plugin_spec,
        &Shiboken::callCppDestructor< ::qt_gui_cpp::Plugin >,
        reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]),
        0,
        Shiboken::ObjectType::WrapperFlags::InnerClass    );
    
    auto pyType = reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_Plugin_Type);
    InitSignatureStrings(pyType, qt_gui_cpp_Plugin_SignatureStrings);
    SbkObjectType_SetPropertyStrings(reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_Plugin_Type), Sbk_qt_gui_cpp_Plugin_PropertyStrings);
    Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX]
        = reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_Plugin_TypeF());

    // Register Converter
    SbkConverter *converter = Shiboken::Conversions::createConverter(Sbk_qt_gui_cpp_Plugin_TypeF(),
        Plugin_PythonToCpp_Plugin_PTR,
        is_Plugin_PythonToCpp_Plugin_PTR_Convertible,
        Plugin_PTR_CppToPython_Plugin);

    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::Plugin");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::Plugin*");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::Plugin&");
    Shiboken::Conversions::registerConverterName(converter, "Plugin");
    Shiboken::Conversions::registerConverterName(converter, "Plugin*");
    Shiboken::Conversions::registerConverterName(converter, "Plugin&");
    Shiboken::Conversions::registerConverterName(converter, typeid(::qt_gui_cpp::Plugin).name());
    Shiboken::Conversions::registerConverterName(converter, typeid(::PluginWrapper).name());


    Shiboken::ObjectType::setTypeDiscoveryFunctionV2(Sbk_qt_gui_cpp_Plugin_TypeF(), &Sbk_qt_gui_cpp_Plugin_typeDiscovery);

    PySide::Signal::registerSignals(Sbk_qt_gui_cpp_Plugin_TypeF(), &::qt_gui_cpp::Plugin::staticMetaObject);

    Shiboken::ObjectType::setSubTypeInitHook(Sbk_qt_gui_cpp_Plugin_TypeF(), &PySide::initQObjectSubType);
    PySide::initDynamicMetaObject(Sbk_qt_gui_cpp_Plugin_TypeF(), &::qt_gui_cpp::Plugin::staticMetaObject, sizeof(PluginWrapper));
}
