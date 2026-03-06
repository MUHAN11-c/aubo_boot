
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
#include "qt_gui_cpp_plugincontext_wrapper.h"

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

void PluginContextWrapper::resetPyMethodCache()
{
    std::fill_n(m_PyMethodCache, sizeof(m_PyMethodCache) / sizeof(m_PyMethodCache[0]), false);
}

PluginContextWrapper::PluginContextWrapper(QObject * obj, int serial_number, const QStringList & argv) : qt_gui_cpp::PluginContext(obj, serial_number, argv)
{
    resetPyMethodCache();
    // ... middle
}

void PluginContextWrapper::childEvent(QChildEvent * event)
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

void PluginContextWrapper::connectNotify(const QMetaMethod & signal)
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

void PluginContextWrapper::customEvent(QEvent * event)
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

void PluginContextWrapper::disconnectNotify(const QMetaMethod & signal)
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

bool PluginContextWrapper::event(QEvent * event)
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "PluginContext.event", "bool", Py_TYPE(pyResult)->tp_name);
        return false;
    }
    bool cppResult;
    pythonToCpp(pyResult, &cppResult);
    if (invalidateArg1)
        Shiboken::Object::invalidate(PyTuple_GET_ITEM(pyArgs, 0));
    return cppResult;
}

bool PluginContextWrapper::eventFilter(QObject * watched, QEvent * event)
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
        Shiboken::warning(PyExc_RuntimeWarning, 2, "Invalid return value in function %s, expected %s, got %s.", "PluginContext.eventFilter", "bool", Py_TYPE(pyResult)->tp_name);
        return false;
    }
    bool cppResult;
    pythonToCpp(pyResult, &cppResult);
    if (invalidateArg2)
        Shiboken::Object::invalidate(PyTuple_GET_ITEM(pyArgs, 1));
    return cppResult;
}

void PluginContextWrapper::timerEvent(QTimerEvent * event)
{
    if (m_PyMethodCache[7]) {
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
        m_PyMethodCache[7] = true;
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

const QMetaObject *PluginContextWrapper::metaObject() const
{
    if (QObject::d_ptr->metaObject)
        return QObject::d_ptr->dynamicMetaObject();
    SbkObject *pySelf = Shiboken::BindingManager::instance().retrieveWrapper(this);
    if (pySelf == nullptr)
        return qt_gui_cpp::PluginContext::metaObject();
    return PySide::SignalManager::retrieveMetaObject(reinterpret_cast<PyObject *>(pySelf));
}

int PluginContextWrapper::qt_metacall(QMetaObject::Call call, int id, void **args)
{
    int result = qt_gui_cpp::PluginContext::qt_metacall(call, id, args);
    return result < 0 ? result : PySide::SignalManager::qt_metacall(this, call, id, args);
}

void *PluginContextWrapper::qt_metacast(const char *_clname)
{
        if (!_clname) return {};
        SbkObject *pySelf = Shiboken::BindingManager::instance().retrieveWrapper(this);
        if (pySelf && PySide::inherits(Py_TYPE(pySelf), _clname))
                return static_cast<void *>(const_cast< PluginContextWrapper *>(this));
        return qt_gui_cpp::PluginContext::qt_metacast(_clname);
}

PluginContextWrapper::~PluginContextWrapper()
{
    SbkObject *wrapper = Shiboken::BindingManager::instance().retrieveWrapper(this);
    Shiboken::Object::destroy(wrapper, this);
}

// Target ---------------------------------------------------------

extern "C" {
static int
Sbk_qt_gui_cpp_PluginContext_Init(PyObject *self, PyObject *args, PyObject *kwds)
{
    const char **argNames{};
    const QMetaObject *metaObject;
    SbkObject *sbkSelf = reinterpret_cast<SbkObject *>(self);
    if (Shiboken::Object::isUserType(self) && !Shiboken::ObjectType::canCallConstructor(self->ob_type, Shiboken::SbkType< ::qt_gui_cpp::PluginContext >()))
        return -1;

    ::PluginContextWrapper *cptr{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0, 0};

    // invalid argument lengths
    if (numArgs == 2)
        goto Sbk_qt_gui_cpp_PluginContext_Init_TypeError;

    if (!PyArg_UnpackTuple(args, "PluginContext", 1, 3, &(pyArgs[0]), &(pyArgs[1]), &(pyArgs[2])))
        return -1;


    // Overloaded function decisor
    // 0: PluginContext::PluginContext(QObject*,int,QStringList)
    // 1: PluginContext::PluginContext(qt_gui_cpp::PluginContext)
    if (numArgs == 1
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppReferenceConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]), (pyArgs[0])))) {
        overloadId = 1; // PluginContext(qt_gui_cpp::PluginContext)
    } else if (numArgs == 3
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppConvertible(Shiboken::Conversions::PrimitiveTypeConverter<int>(), (pyArgs[1])))
        && (pythonToCpp[2] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRINGLIST_IDX], (pyArgs[2])))) {
        overloadId = 0; // PluginContext(QObject*,int,QStringList)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginContext_Init_TypeError;

    // Call function/method
    switch (overloadId) {
        case 0: // PluginContext(QObject * obj, int serial_number, const QStringList & argv)
        {
            if (!Shiboken::Object::isValid(pyArgs[0]))
                return -1;
            ::QObject *cppArg0;
            pythonToCpp[0](pyArgs[0], &cppArg0);
            int cppArg1;
            pythonToCpp[1](pyArgs[1], &cppArg1);
            ::QStringList cppArg2;
            pythonToCpp[2](pyArgs[2], &cppArg2);

            if (!PyErr_Occurred()) {
                // PluginContext(QObject*,int,QStringList)
                void *addr = PySide::nextQObjectMemoryAddr();
                if (addr) {
                    cptr = new (addr) ::PluginContextWrapper(cppArg0, cppArg1, cppArg2);
                    PySide::setNextQObjectMemoryAddr(0);
                } else {
                    cptr = new ::PluginContextWrapper(cppArg0, cppArg1, cppArg2);
                }

            }
            break;
        }
        case 1: // PluginContext(const qt_gui_cpp::PluginContext & other)
        {
            if (!Shiboken::Object::isValid(pyArgs[0]))
                return -1;
            ::qt_gui_cpp::PluginContext *cppArg0;
            pythonToCpp[0](pyArgs[0], &cppArg0);

            if (!PyErr_Occurred()) {
                // PluginContext(qt_gui_cpp::PluginContext)
                cptr = new ::PluginContextWrapper(*cppArg0);
            }
            break;
        }
    }

    if (PyErr_Occurred() || !Shiboken::Object::setCppPointer(sbkSelf, Shiboken::SbkType< ::qt_gui_cpp::PluginContext >(), cptr)) {
        delete cptr;
        return -1;
    }
    if (!cptr) goto Sbk_qt_gui_cpp_PluginContext_Init_TypeError;

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

    Sbk_qt_gui_cpp_PluginContext_Init_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext");
        return -1;
}

static PyObject *Sbk_qt_gui_cpp_PluginContextFunc_addWidget(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginContext *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: PluginContext::addWidget(QWidget*)
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtWidgetsTypes[SBK_QWIDGET_IDX]), (pyArg)))) {
        overloadId = 0; // addWidget(QWidget*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginContextFunc_addWidget_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArg))
            return {};
        ::QWidget *cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // addWidget(QWidget*)
            cppSelf->addWidget(cppArg0);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_PluginContextFunc_addWidget_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext.addWidget");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginContextFunc_argv(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginContext *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // argv()const
            const QStringList & cppResult = const_cast<const ::qt_gui_cpp::PluginContext *>(cppSelf)->argv();
            pyResult = Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRINGLIST_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;
}

static PyObject *Sbk_qt_gui_cpp_PluginContextFunc_closePlugin(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginContext *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // closePlugin()
            cppSelf->closePlugin();
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;
}

static PyObject *Sbk_qt_gui_cpp_PluginContextFunc_reloadPlugin(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginContext *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // reloadPlugin()
            cppSelf->reloadPlugin();
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;
}

static PyObject *Sbk_qt_gui_cpp_PluginContextFunc_removeWidget(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginContext *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: PluginContext::removeWidget(QWidget*)
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtWidgetsTypes[SBK_QWIDGET_IDX]), (pyArg)))) {
        overloadId = 0; // removeWidget(QWidget*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginContextFunc_removeWidget_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArg))
            return {};
        ::QWidget *cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // removeWidget(QWidget*)
            cppSelf->removeWidget(cppArg0);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_PluginContextFunc_removeWidget_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext.removeWidget");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginContextFunc_serialNumber(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginContext *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // serialNumber()const
            int cppResult = const_cast<const ::qt_gui_cpp::PluginContext *>(cppSelf)->serialNumber();
            pyResult = Shiboken::Conversions::copyToPython(Shiboken::Conversions::PrimitiveTypeConverter<int>(), &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;
}


static const char *Sbk_qt_gui_cpp_PluginContext_PropertyStrings[] = {
    nullptr // Sentinel
};

static PyMethodDef Sbk_qt_gui_cpp_PluginContext_methods[] = {
    {"addWidget", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginContextFunc_addWidget), METH_O},
    {"argv", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginContextFunc_argv), METH_NOARGS},
    {"closePlugin", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginContextFunc_closePlugin), METH_NOARGS},
    {"reloadPlugin", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginContextFunc_reloadPlugin), METH_NOARGS},
    {"removeWidget", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginContextFunc_removeWidget), METH_O},
    {"serialNumber", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginContextFunc_serialNumber), METH_NOARGS},

    {nullptr, nullptr} // Sentinel
};

static int Sbk_qt_gui_cpp_PluginContext_setattro(PyObject *self, PyObject *name, PyObject *value)
{
    PySide::Feature::Select(self);
    if (value && PyCallable_Check(value)) {
        auto plain_inst = reinterpret_cast< ::qt_gui_cpp::PluginContext *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX], reinterpret_cast<SbkObject *>(self)));
        auto inst = dynamic_cast<PluginContextWrapper *>(plain_inst);
        if (inst)
            inst->resetPyMethodCache();
    }
    Shiboken::AutoDecRef pp(reinterpret_cast<PyObject *>(PySide::Property::getObject(self, name)));
    if (!pp.isNull())
        return PySide::Property::setValue(reinterpret_cast<PySideProperty *>(pp.object()), self, value);
    return PyObject_GenericSetAttr(self, name, value);
}

static PyObject *Sbk_qt_gui_cpp_PluginContext_get_proxy_(PyObject *self, void *)
{
    if (!Shiboken::Object::isValid(self))
        return nullptr;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginContext *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyOut = {};
    if (reinterpret_cast<void *>(&(cppSelf->proxy_)) == reinterpret_cast<void *>(cppSelf)) {
        pyOut = reinterpret_cast<PyObject *>(Shiboken::Object::findColocatedChild(reinterpret_cast<SbkObject *>(self), reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_GENERICPROXY_IDX])));
        if (pyOut) {
            Py_IncRef(pyOut);
            return pyOut;
        }
    } else if (Shiboken::BindingManager::instance().hasWrapper(&(cppSelf->proxy_))) {
        pyOut = reinterpret_cast<PyObject *>(Shiboken::BindingManager::instance().retrieveWrapper(&(cppSelf->proxy_)));
        Py_IncRef(pyOut);
        return pyOut;
    }
    pyOut = Shiboken::Object::newObject(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_GENERICPROXY_IDX]), &(cppSelf->proxy_), false, true);
    Shiboken::Object::setParent(self, pyOut);
    return pyOut;
}
static int Sbk_qt_gui_cpp_PluginContext_set_proxy_(PyObject *self, PyObject *pyIn, void *)
{
    if (!Shiboken::Object::isValid(self))
        return 0;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginContext *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    if (pyIn == nullptr) {
        PyErr_SetString(PyExc_TypeError, "'proxy_' may not be deleted");
        return -1;
    }
    PythonToCppFunc pythonToCpp{nullptr};
    if (!(pythonToCpp = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_GENERICPROXY_IDX]), (pyIn)))) {
        PyErr_SetString(PyExc_TypeError, "wrong type attributed to 'proxy_', 'GenericProxy' or convertible type expected");
        return -1;
    }

    ::qt_gui_cpp::GenericProxy& cppOut_ptr = cppSelf->proxy_;
    pythonToCpp(pyIn, &cppOut_ptr);

    return 0;
}

static PyObject *Sbk_qt_gui_cpp_PluginContext_get_serial_number_(PyObject *self, void *)
{
    if (!Shiboken::Object::isValid(self))
        return nullptr;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginContext *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int cppOut_local = cppSelf->serial_number_;
    PyObject *pyOut = {};
    pyOut = Shiboken::Conversions::copyToPython(Shiboken::Conversions::PrimitiveTypeConverter<int>(), &cppOut_local);
    return pyOut;
}
static int Sbk_qt_gui_cpp_PluginContext_set_serial_number_(PyObject *self, PyObject *pyIn, void *)
{
    if (!Shiboken::Object::isValid(self))
        return 0;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginContext *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    if (pyIn == nullptr) {
        PyErr_SetString(PyExc_TypeError, "'serial_number_' may not be deleted");
        return -1;
    }
    PythonToCppFunc pythonToCpp{nullptr};
    if (!(pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(Shiboken::Conversions::PrimitiveTypeConverter<int>(), (pyIn)))) {
        PyErr_SetString(PyExc_TypeError, "wrong type attributed to 'serial_number_', 'int' or convertible type expected");
        return -1;
    }

    int cppOut_local = cppSelf->serial_number_;
    pythonToCpp(pyIn, &cppOut_local);
    cppSelf->serial_number_ = cppOut_local;

    return 0;
}

static PyObject *Sbk_qt_gui_cpp_PluginContext_get_argv_(PyObject *self, void *)
{
    if (!Shiboken::Object::isValid(self))
        return nullptr;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginContext *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyOut = {};
    pyOut = Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRINGLIST_IDX], &cppSelf->argv_);
    return pyOut;
}
static int Sbk_qt_gui_cpp_PluginContext_set_argv_(PyObject *self, PyObject *pyIn, void *)
{
    if (!Shiboken::Object::isValid(self))
        return 0;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginContext *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    if (pyIn == nullptr) {
        PyErr_SetString(PyExc_TypeError, "'argv_' may not be deleted");
        return -1;
    }
    PythonToCppFunc pythonToCpp{nullptr};
    if (!(pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRINGLIST_IDX], (pyIn)))) {
        PyErr_SetString(PyExc_TypeError, "wrong type attributed to 'argv_', 'QStringList' or convertible type expected");
        return -1;
    }

    ::QStringList& cppOut_ptr = cppSelf->argv_;
    pythonToCpp(pyIn, &cppOut_ptr);

    return 0;
}

// Getters and Setters for PluginContext
static PyGetSetDef Sbk_qt_gui_cpp_PluginContext_getsetlist[] = {
    {const_cast<char *>("proxy_"), Sbk_qt_gui_cpp_PluginContext_get_proxy_, Sbk_qt_gui_cpp_PluginContext_set_proxy_},
    {const_cast<char *>("serial_number_"), Sbk_qt_gui_cpp_PluginContext_get_serial_number_, Sbk_qt_gui_cpp_PluginContext_set_serial_number_},
    {const_cast<char *>("argv_"), Sbk_qt_gui_cpp_PluginContext_get_argv_, Sbk_qt_gui_cpp_PluginContext_set_argv_},
    {nullptr} // Sentinel
};

} // extern "C"

static int Sbk_qt_gui_cpp_PluginContext_traverse(PyObject *self, visitproc visit, void *arg)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_traverse(self, visit, arg);
}
static int Sbk_qt_gui_cpp_PluginContext_clear(PyObject *self)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_clear(self);
}
// Class Definition -----------------------------------------------
extern "C" {
static SbkObjectType *_Sbk_qt_gui_cpp_PluginContext_Type = nullptr;
static SbkObjectType *Sbk_qt_gui_cpp_PluginContext_TypeF(void)
{
    return _Sbk_qt_gui_cpp_PluginContext_Type;
}

static PyType_Slot Sbk_qt_gui_cpp_PluginContext_slots[] = {
    {Py_tp_base,        nullptr}, // inserted by introduceWrapperType
    {Py_tp_dealloc,     reinterpret_cast<void *>(&SbkDeallocWrapper)},
    {Py_tp_repr,        nullptr},
    {Py_tp_hash,        nullptr},
    {Py_tp_call,        nullptr},
    {Py_tp_str,         nullptr},
    {Py_tp_getattro,    nullptr},
    {Py_tp_setattro,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginContext_setattro)},
    {Py_tp_traverse,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginContext_traverse)},
    {Py_tp_clear,       reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginContext_clear)},
    {Py_tp_richcompare, nullptr},
    {Py_tp_iter,        nullptr},
    {Py_tp_iternext,    nullptr},
    {Py_tp_methods,     reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginContext_methods)},
    {Py_tp_getset,      reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginContext_getsetlist)},
    {Py_tp_init,        reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginContext_Init)},
    {Py_tp_new,         reinterpret_cast<void *>(SbkObjectTpNew)},
    {0, nullptr}
};
static PyType_Spec Sbk_qt_gui_cpp_PluginContext_spec = {
    "1:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext",
    sizeof(SbkObject),
    0,
    Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_CHECKTYPES|Py_TPFLAGS_HAVE_GC,
    Sbk_qt_gui_cpp_PluginContext_slots
};

} //extern "C"

static void *Sbk_qt_gui_cpp_PluginContext_typeDiscovery(void *cptr, SbkObjectType *instanceType)
{
    if (instanceType == reinterpret_cast<SbkObjectType *>(Shiboken::SbkType< ::QObject >()))
        return dynamic_cast< ::qt_gui_cpp::PluginContext *>(reinterpret_cast< ::QObject *>(cptr));
    return {};
}


// Type conversion functions.

// Python to C++ pointer conversion - returns the C++ object of the Python wrapper (keeps object identity).
static void PluginContext_PythonToCpp_PluginContext_PTR(PyObject *pyIn, void *cppOut) {
    Shiboken::Conversions::pythonToCppPointer(Sbk_qt_gui_cpp_PluginContext_TypeF(), pyIn, cppOut);
}
static PythonToCppFunc is_PluginContext_PythonToCpp_PluginContext_PTR_Convertible(PyObject *pyIn) {
    if (pyIn == Py_None)
        return Shiboken::Conversions::nonePythonToCppNullPtr;
    if (PyObject_TypeCheck(pyIn, reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_PluginContext_TypeF())))
        return PluginContext_PythonToCpp_PluginContext_PTR;
    return {};
}

// C++ to Python pointer conversion - tries to find the Python wrapper for the C++ object (keeps object identity).
static PyObject *PluginContext_PTR_CppToPython_PluginContext(const void *cppIn) {
    return PySide::getWrapperForQObject(reinterpret_cast<::qt_gui_cpp::PluginContext *>(const_cast<void *>(cppIn)), Sbk_qt_gui_cpp_PluginContext_TypeF());

}

// The signatures string for the functions.
// Multiple signatures have their index "n:" in front.
static const char *qt_gui_cpp_PluginContext_SignatureStrings[] = {
    "1:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext(self,obj:PySide2.QtCore.QObject,serial_number:int,argv:QStringList)",
    "0:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext(self,other:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext.addWidget(self,widget:PySide2.QtWidgets.QWidget)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext.argv(self)->QStringList",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext.closePlugin(self)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext.reloadPlugin(self)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext.removeWidget(self,widget:PySide2.QtWidgets.QWidget)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginContext.serialNumber(self)->int",
    nullptr}; // Sentinel

void init_qt_gui_cpp_PluginContext(PyObject *enclosingClass)
{
    _Sbk_qt_gui_cpp_PluginContext_Type = Shiboken::ObjectType::introduceWrapperType(
        enclosingClass,
        "PluginContext",
        "qt_gui_cpp::PluginContext*",
        &Sbk_qt_gui_cpp_PluginContext_spec,
        &Shiboken::callCppDestructor< ::qt_gui_cpp::PluginContext >,
        reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]),
        0,
        Shiboken::ObjectType::WrapperFlags::InnerClass    );
    
    auto pyType = reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_PluginContext_Type);
    InitSignatureStrings(pyType, qt_gui_cpp_PluginContext_SignatureStrings);
    SbkObjectType_SetPropertyStrings(reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_PluginContext_Type), Sbk_qt_gui_cpp_PluginContext_PropertyStrings);
    Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]
        = reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_PluginContext_TypeF());

    // Register Converter
    SbkConverter *converter = Shiboken::Conversions::createConverter(Sbk_qt_gui_cpp_PluginContext_TypeF(),
        PluginContext_PythonToCpp_PluginContext_PTR,
        is_PluginContext_PythonToCpp_PluginContext_PTR_Convertible,
        PluginContext_PTR_CppToPython_PluginContext);

    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::PluginContext");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::PluginContext*");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::PluginContext&");
    Shiboken::Conversions::registerConverterName(converter, "PluginContext");
    Shiboken::Conversions::registerConverterName(converter, "PluginContext*");
    Shiboken::Conversions::registerConverterName(converter, "PluginContext&");
    Shiboken::Conversions::registerConverterName(converter, typeid(::qt_gui_cpp::PluginContext).name());
    Shiboken::Conversions::registerConverterName(converter, typeid(::PluginContextWrapper).name());


    Shiboken::ObjectType::setTypeDiscoveryFunctionV2(Sbk_qt_gui_cpp_PluginContext_TypeF(), &Sbk_qt_gui_cpp_PluginContext_typeDiscovery);

    PySide::Signal::registerSignals(Sbk_qt_gui_cpp_PluginContext_TypeF(), &::qt_gui_cpp::PluginContext::staticMetaObject);

    Shiboken::ObjectType::setSubTypeInitHook(Sbk_qt_gui_cpp_PluginContext_TypeF(), &PySide::initQObjectSubType);
    PySide::initDynamicMetaObject(Sbk_qt_gui_cpp_PluginContext_TypeF(), &::qt_gui_cpp::PluginContext::staticMetaObject, sizeof(PluginContextWrapper));
}
