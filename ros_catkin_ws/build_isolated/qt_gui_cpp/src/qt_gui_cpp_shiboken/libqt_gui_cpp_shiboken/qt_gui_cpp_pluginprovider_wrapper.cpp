
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
#include "qt_gui_cpp_pluginprovider_wrapper.h"

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

void PluginProviderWrapper::resetPyMethodCache()
{
    std::fill_n(m_PyMethodCache, sizeof(m_PyMethodCache) / sizeof(m_PyMethodCache[0]), false);
}

PluginProviderWrapper::PluginProviderWrapper() : qt_gui_cpp::PluginProvider()
{
    resetPyMethodCache();
    // ... middle
}

QMap<QString,QString > PluginProviderWrapper::discover(QObject * discovery_data)
{
    if (m_PyMethodCache[0])
        return this->::qt_gui_cpp::PluginProvider::discover(discovery_data);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return {};
    static PyObject *nameCache[2] = {};
    static const char *funcName = "discover";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[0] = true;
        return this->::qt_gui_cpp::PluginProvider::discover(discovery_data);
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "PluginProvider.discover", "map", Py_TYPE(pyResult)->tp_name);
        return {};
    }
    ::QMap<QString,QString > cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

QList<qt_gui_cpp::PluginDescriptor* > PluginProviderWrapper::discover_descriptors(QObject * discovery_data)
{
    if (m_PyMethodCache[1])
        return this->::qt_gui_cpp::PluginProvider::discover_descriptors(discovery_data);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return {};
    static PyObject *nameCache[2] = {};
    static const char *funcName = "discover_descriptors";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[1] = true;
        return this->::qt_gui_cpp::PluginProvider::discover_descriptors(discovery_data);
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "PluginProvider.discover_descriptors", "list", Py_TYPE(pyResult)->tp_name);
        return {};
    }
    ::QList<qt_gui_cpp::PluginDescriptor* > cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

void * PluginProviderWrapper::load(const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context)
{
    if (m_PyMethodCache[2])
        return this->::qt_gui_cpp::PluginProvider::load(plugin_id, plugin_context);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return nullptr;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "load";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[2] = true;
        return this->::qt_gui_cpp::PluginProvider::load(plugin_id, plugin_context);
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "PluginProvider.load", reinterpret_cast<PyTypeObject *>(Shiboken::SbkType< void >())->tp_name, Py_TYPE(pyResult)->tp_name);
        return nullptr;
    }
    void* cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

qt_gui_cpp::Plugin * PluginProviderWrapper::load_plugin(const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context)
{
    if (m_PyMethodCache[3])
        return this->::qt_gui_cpp::PluginProvider::load_plugin(plugin_id, plugin_context);
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return nullptr;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "load_plugin";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[3] = true;
        return this->::qt_gui_cpp::PluginProvider::load_plugin(plugin_id, plugin_context);
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "PluginProvider.load_plugin", reinterpret_cast<PyTypeObject *>(Shiboken::SbkType< qt_gui_cpp::Plugin >())->tp_name, Py_TYPE(pyResult)->tp_name);
        return nullptr;
    }
    ::qt_gui_cpp::Plugin *cppResult;
    pythonToCpp(pyResult, &cppResult);
    return cppResult;
}

void PluginProviderWrapper::shutdown()
{
    if (m_PyMethodCache[4]) {
        return this->::qt_gui_cpp::PluginProvider::shutdown();
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "shutdown";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[4] = true;
        return this->::qt_gui_cpp::PluginProvider::shutdown();
    }

    Shiboken::AutoDecRef pyArgs(PyTuple_New(0));

    Shiboken::AutoDecRef pyResult(PyObject_Call(pyOverride, pyArgs, nullptr));
    // An error happened in python code!
    if (pyResult.isNull()) {
        PyErr_Print();
        return;
    }
}

void PluginProviderWrapper::unload(void * plugin_instance)
{
    if (m_PyMethodCache[5]) {
        return this->::qt_gui_cpp::PluginProvider::unload(plugin_instance);
    }
    Shiboken::GilState gil;
    if (PyErr_Occurred())
        return;
    static PyObject *nameCache[2] = {};
    static const char *funcName = "unload";
    Shiboken::AutoDecRef pyOverride(Shiboken::BindingManager::instance().getOverride(this, nameCache, funcName));
    if (pyOverride.isNull()) {
        gil.release();
        m_PyMethodCache[5] = true;
        return this->::qt_gui_cpp::PluginProvider::unload(plugin_instance);
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

void PluginProviderWrapper::unload_plugin(qt_gui_cpp::Plugin * plugin_instance)
{
    if (m_PyMethodCache[6]) {
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
        m_PyMethodCache[6] = true;
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

PluginProviderWrapper::~PluginProviderWrapper()
{
    SbkObject *wrapper = Shiboken::BindingManager::instance().retrieveWrapper(this);
    Shiboken::Object::destroy(wrapper, this);
}

// Target ---------------------------------------------------------

extern "C" {
static int
Sbk_qt_gui_cpp_PluginProvider_Init(PyObject *self, PyObject *args, PyObject *kwds)
{
    SbkObject *sbkSelf = reinterpret_cast<SbkObject *>(self);
    if (Shiboken::Object::isUserType(self) && !Shiboken::ObjectType::canCallConstructor(self->ob_type, Shiboken::SbkType< ::qt_gui_cpp::PluginProvider >()))
        return -1;

    ::PluginProviderWrapper *cptr{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // PluginProvider()
            cptr = new ::PluginProviderWrapper();
        }
    }

    if (PyErr_Occurred() || !Shiboken::Object::setCppPointer(sbkSelf, Shiboken::SbkType< ::qt_gui_cpp::PluginProvider >(), cptr)) {
        delete cptr;
        return -1;
    }
    Shiboken::Object::setValidCpp(sbkSelf, true);
    Shiboken::Object::setHasCppWrapper(sbkSelf, true);
    if (Shiboken::BindingManager::instance().hasWrapper(cptr)) {
        Shiboken::BindingManager::instance().releaseWrapper(Shiboken::BindingManager::instance().retrieveWrapper(cptr));
    }
    Shiboken::BindingManager::instance().registerWrapper(sbkSelf, cptr);


    return 1;
}

static PyObject *Sbk_qt_gui_cpp_PluginProviderFunc_discover(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginProvider *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX], reinterpret_cast<SbkObject *>(self)));
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
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginProviderFunc_discover_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArg))
            return {};
        ::QObject *cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // discover(QObject*)
            QMap<QString,QString > cppResult = Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::PluginProvider::discover(cppArg0)
                : cppSelf->discover(cppArg0);
            pyResult = Shiboken::Conversions::copyToPython(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_PluginProviderFunc_discover_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider.discover");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginProviderFunc_discover_descriptors(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginProvider *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: PluginProvider::discover_descriptors(QObject*)
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), (pyArg)))) {
        overloadId = 0; // discover_descriptors(QObject*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginProviderFunc_discover_descriptors_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArg))
            return {};
        ::QObject *cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // discover_descriptors(QObject*)
            QList<qt_gui_cpp::PluginDescriptor* > cppResult = Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::PluginProvider::discover_descriptors(cppArg0)
                : cppSelf->discover_descriptors(cppArg0);
            pyResult = Shiboken::Conversions::copyToPython(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QT_GUI_CPP_PLUGINDESCRIPTORPTR_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_PluginProviderFunc_discover_descriptors_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider.discover_descriptors");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginProviderFunc_load(PyObject *self, PyObject *args)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginProvider *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX], reinterpret_cast<SbkObject *>(self)));
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
    // 0: PluginProvider::load(QString,qt_gui_cpp::PluginContext*)
    if (numArgs == 2
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]), (pyArgs[1])))) {
        overloadId = 0; // load(QString,qt_gui_cpp::PluginContext*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginProviderFunc_load_TypeError;

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
                ? cppSelf->::qt_gui_cpp::PluginProvider::load(cppArg0, cppArg1)
                : cppSelf->load(cppArg0, cppArg1);
            pyResult = Shiboken::Conversions::copyToPython(Shiboken::Conversions::PrimitiveTypeConverter<void *>(), cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_PluginProviderFunc_load_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider.load");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginProviderFunc_load_plugin(PyObject *self, PyObject *args)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginProvider *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX], reinterpret_cast<SbkObject *>(self)));
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
    // 0: PluginProvider::load_plugin(QString,qt_gui_cpp::PluginContext*)
    if (numArgs == 2
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]), (pyArgs[1])))) {
        overloadId = 0; // load_plugin(QString,qt_gui_cpp::PluginContext*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginProviderFunc_load_plugin_TypeError;

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
                ? cppSelf->::qt_gui_cpp::PluginProvider::load_plugin(cppArg0, cppArg1)
                : cppSelf->load_plugin(cppArg0, cppArg1);
            pyResult = Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX]), cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_PluginProviderFunc_load_plugin_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider.load_plugin");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginProviderFunc_shutdown(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginProvider *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // shutdown()
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::PluginProvider::shutdown()
                : cppSelf->shutdown();
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;
}

static PyObject *Sbk_qt_gui_cpp_PluginProviderFunc_unload(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginProvider *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: PluginProvider::unload(void*)
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(Shiboken::Conversions::PrimitiveTypeConverter<void *>(), (pyArg)))) {
        overloadId = 0; // unload(void*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginProviderFunc_unload_TypeError;

    // Call function/method
    {
        void* cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // unload(void*)
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::PluginProvider::unload(cppArg0)
                : cppSelf->unload(cppArg0);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_PluginProviderFunc_unload_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider.unload");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginProviderFunc_unload_plugin(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginProvider *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: PluginProvider::unload_plugin(qt_gui_cpp::Plugin*)
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX]), (pyArg)))) {
        overloadId = 0; // unload_plugin(qt_gui_cpp::Plugin*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginProviderFunc_unload_plugin_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArg))
            return {};
        ::qt_gui_cpp::Plugin *cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // unload_plugin(qt_gui_cpp::Plugin*)
            Shiboken::Object::hasCppWrapper(reinterpret_cast<SbkObject *>(self))
                ? cppSelf->::qt_gui_cpp::PluginProvider::unload_plugin(cppArg0)
                : cppSelf->unload_plugin(cppArg0);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_PluginProviderFunc_unload_plugin_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider.unload_plugin");
        return {};
}


static const char *Sbk_qt_gui_cpp_PluginProvider_PropertyStrings[] = {
    nullptr // Sentinel
};

static PyMethodDef Sbk_qt_gui_cpp_PluginProvider_methods[] = {
    {"discover", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginProviderFunc_discover), METH_O},
    {"discover_descriptors", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginProviderFunc_discover_descriptors), METH_O},
    {"load", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginProviderFunc_load), METH_VARARGS},
    {"load_plugin", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginProviderFunc_load_plugin), METH_VARARGS},
    {"shutdown", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginProviderFunc_shutdown), METH_NOARGS},
    {"unload", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginProviderFunc_unload), METH_O},
    {"unload_plugin", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginProviderFunc_unload_plugin), METH_O},

    {nullptr, nullptr} // Sentinel
};

static int Sbk_qt_gui_cpp_PluginProvider_setattro(PyObject *self, PyObject *name, PyObject *value)
{
    PySide::Feature::Select(self);
    if (value && PyCallable_Check(value)) {
        auto plain_inst = reinterpret_cast< ::qt_gui_cpp::PluginProvider *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX], reinterpret_cast<SbkObject *>(self)));
        auto inst = dynamic_cast<PluginProviderWrapper *>(plain_inst);
        if (inst)
            inst->resetPyMethodCache();
    }
    return PyObject_GenericSetAttr(self, name, value);
}

} // extern "C"

static int Sbk_qt_gui_cpp_PluginProvider_traverse(PyObject *self, visitproc visit, void *arg)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_traverse(self, visit, arg);
}
static int Sbk_qt_gui_cpp_PluginProvider_clear(PyObject *self)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_clear(self);
}
// Class Definition -----------------------------------------------
extern "C" {
static SbkObjectType *_Sbk_qt_gui_cpp_PluginProvider_Type = nullptr;
static SbkObjectType *Sbk_qt_gui_cpp_PluginProvider_TypeF(void)
{
    return _Sbk_qt_gui_cpp_PluginProvider_Type;
}

static PyType_Slot Sbk_qt_gui_cpp_PluginProvider_slots[] = {
    {Py_tp_base,        nullptr}, // inserted by introduceWrapperType
    {Py_tp_dealloc,     reinterpret_cast<void *>(&SbkDeallocWrapper)},
    {Py_tp_repr,        nullptr},
    {Py_tp_hash,        nullptr},
    {Py_tp_call,        nullptr},
    {Py_tp_str,         nullptr},
    {Py_tp_getattro,    nullptr},
    {Py_tp_setattro,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginProvider_setattro)},
    {Py_tp_traverse,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginProvider_traverse)},
    {Py_tp_clear,       reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginProvider_clear)},
    {Py_tp_richcompare, nullptr},
    {Py_tp_iter,        nullptr},
    {Py_tp_iternext,    nullptr},
    {Py_tp_methods,     reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginProvider_methods)},
    {Py_tp_getset,      nullptr},
    {Py_tp_init,        reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginProvider_Init)},
    {Py_tp_new,         reinterpret_cast<void *>(SbkObjectTpNew)},
    {0, nullptr}
};
static PyType_Spec Sbk_qt_gui_cpp_PluginProvider_spec = {
    "1:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider",
    sizeof(SbkObject),
    0,
    Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_CHECKTYPES|Py_TPFLAGS_HAVE_GC,
    Sbk_qt_gui_cpp_PluginProvider_slots
};

} //extern "C"


// Type conversion functions.

// Python to C++ pointer conversion - returns the C++ object of the Python wrapper (keeps object identity).
static void PluginProvider_PythonToCpp_PluginProvider_PTR(PyObject *pyIn, void *cppOut) {
    Shiboken::Conversions::pythonToCppPointer(Sbk_qt_gui_cpp_PluginProvider_TypeF(), pyIn, cppOut);
}
static PythonToCppFunc is_PluginProvider_PythonToCpp_PluginProvider_PTR_Convertible(PyObject *pyIn) {
    if (pyIn == Py_None)
        return Shiboken::Conversions::nonePythonToCppNullPtr;
    if (PyObject_TypeCheck(pyIn, reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_PluginProvider_TypeF())))
        return PluginProvider_PythonToCpp_PluginProvider_PTR;
    return {};
}

// C++ to Python pointer conversion - tries to find the Python wrapper for the C++ object (keeps object identity).
static PyObject *PluginProvider_PTR_CppToPython_PluginProvider(const void *cppIn) {
    auto pyOut = reinterpret_cast<PyObject *>(Shiboken::BindingManager::instance().retrieveWrapper(cppIn));
    if (pyOut) {
        Py_INCREF(pyOut);
        return pyOut;
    }
    bool changedTypeName = false;
    auto tCppIn = reinterpret_cast<const ::qt_gui_cpp::PluginProvider *>(cppIn);
    const char *typeName = typeid(*tCppIn).name();
    auto sbkType = Shiboken::ObjectType::typeForTypeName(typeName);
    if (sbkType && Shiboken::ObjectType::hasSpecialCastFunction(sbkType)) {
        typeName = typeNameOf(tCppIn);
        changedTypeName = true;
    }
    PyObject *result = Shiboken::Object::newObject(Sbk_qt_gui_cpp_PluginProvider_TypeF(), const_cast<void *>(cppIn), false, /* exactType */ changedTypeName, typeName);
    if (changedTypeName)
        delete [] typeName;
    return result;
}

// The signatures string for the functions.
// Multiple signatures have their index "n:" in front.
static const char *qt_gui_cpp_PluginProvider_SignatureStrings[] = {
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider(self)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider.discover(self,discovery_data:PySide2.QtCore.QObject)->QMap[QString, QString]",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider.discover_descriptors(self,discovery_data:PySide2.QtCore.QObject)->QList[libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor]",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider.load(self,plugin_id:QString,plugin_context:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext)->void",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider.load_plugin(self,plugin_id:QString,plugin_context:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext)->libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider.shutdown(self)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider.unload(self,plugin_instance:void)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginProvider.unload_plugin(self,plugin_instance:libqt_gui_cpp_shiboken.qt_gui_cpp.Plugin)",
    nullptr}; // Sentinel

void init_qt_gui_cpp_PluginProvider(PyObject *enclosingClass)
{
    _Sbk_qt_gui_cpp_PluginProvider_Type = Shiboken::ObjectType::introduceWrapperType(
        enclosingClass,
        "PluginProvider",
        "qt_gui_cpp::PluginProvider*",
        &Sbk_qt_gui_cpp_PluginProvider_spec,
        &Shiboken::callCppDestructor< ::qt_gui_cpp::PluginProvider >,
        0,
        0,
        Shiboken::ObjectType::WrapperFlags::InnerClass    );
    
    auto pyType = reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_PluginProvider_Type);
    InitSignatureStrings(pyType, qt_gui_cpp_PluginProvider_SignatureStrings);
    SbkObjectType_SetPropertyStrings(reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_PluginProvider_Type), Sbk_qt_gui_cpp_PluginProvider_PropertyStrings);
    Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX]
        = reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_PluginProvider_TypeF());

    // Register Converter
    SbkConverter *converter = Shiboken::Conversions::createConverter(Sbk_qt_gui_cpp_PluginProvider_TypeF(),
        PluginProvider_PythonToCpp_PluginProvider_PTR,
        is_PluginProvider_PythonToCpp_PluginProvider_PTR_Convertible,
        PluginProvider_PTR_CppToPython_PluginProvider);

    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::PluginProvider");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::PluginProvider*");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::PluginProvider&");
    Shiboken::Conversions::registerConverterName(converter, "PluginProvider");
    Shiboken::Conversions::registerConverterName(converter, "PluginProvider*");
    Shiboken::Conversions::registerConverterName(converter, "PluginProvider&");
    Shiboken::Conversions::registerConverterName(converter, typeid(::qt_gui_cpp::PluginProvider).name());
    Shiboken::Conversions::registerConverterName(converter, typeid(::PluginProviderWrapper).name());



}
