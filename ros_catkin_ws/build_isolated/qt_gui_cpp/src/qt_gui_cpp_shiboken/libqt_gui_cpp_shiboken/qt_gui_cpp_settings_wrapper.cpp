
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
#include "qt_gui_cpp_settings_wrapper.h"

// inner classes

// Extra includes
#include <qobject.h>
#include <qt_gui_cpp/settings.h>


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


// Target ---------------------------------------------------------

extern "C" {
static int
Sbk_qt_gui_cpp_Settings_Init(PyObject *self, PyObject *args, PyObject *kwds)
{
    SbkObject *sbkSelf = reinterpret_cast<SbkObject *>(self);
    if (Shiboken::Object::isUserType(self) && !Shiboken::ObjectType::canCallConstructor(self->ob_type, Shiboken::SbkType< ::qt_gui_cpp::Settings >()))
        return -1;

    ::qt_gui_cpp::Settings *cptr{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0};

    // invalid argument lengths


    if (!PyArg_UnpackTuple(args, "Settings", 1, 1, &(pyArgs[0])))
        return -1;


    // Overloaded function decisor
    // 0: Settings::Settings(QObject*)
    if (numArgs == 1
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), (pyArgs[0])))) {
        overloadId = 0; // Settings(QObject*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_Settings_Init_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArgs[0]))
            return -1;
        ::QObject *cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);

        if (!PyErr_Occurred()) {
            // Settings(QObject*)
            cptr = new ::qt_gui_cpp::Settings(cppArg0);
        }
    }

    if (PyErr_Occurred() || !Shiboken::Object::setCppPointer(sbkSelf, Shiboken::SbkType< ::qt_gui_cpp::Settings >(), cptr)) {
        delete cptr;
        return -1;
    }
    if (!cptr) goto Sbk_qt_gui_cpp_Settings_Init_TypeError;

    Shiboken::Object::setValidCpp(sbkSelf, true);
    if (Shiboken::BindingManager::instance().hasWrapper(cptr)) {
        Shiboken::BindingManager::instance().releaseWrapper(Shiboken::BindingManager::instance().retrieveWrapper(cptr));
    }
    Shiboken::BindingManager::instance().registerWrapper(sbkSelf, cptr);


    return 1;

    Sbk_qt_gui_cpp_Settings_Init_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings");
        return -1;
}

static PyObject *Sbk_qt_gui_cpp_SettingsFunc_allKeys(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Settings *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // allKeys()const
            QStringList cppResult = const_cast<const ::qt_gui_cpp::Settings *>(cppSelf)->allKeys();
            pyResult = Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRINGLIST_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;
}

static PyObject *Sbk_qt_gui_cpp_SettingsFunc_childGroups(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Settings *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // childGroups()const
            QStringList cppResult = const_cast<const ::qt_gui_cpp::Settings *>(cppSelf)->childGroups();
            pyResult = Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRINGLIST_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;
}

static PyObject *Sbk_qt_gui_cpp_SettingsFunc_childKeys(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Settings *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // childKeys()const
            QStringList cppResult = const_cast<const ::qt_gui_cpp::Settings *>(cppSelf)->childKeys();
            pyResult = Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRINGLIST_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;
}

static PyObject *Sbk_qt_gui_cpp_SettingsFunc_contains(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Settings *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: Settings::contains(QString)const
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArg)))) {
        overloadId = 0; // contains(QString)const
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_SettingsFunc_contains_TypeError;

    // Call function/method
    {
        ::QString cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // contains(QString)const
            bool cppResult = const_cast<const ::qt_gui_cpp::Settings *>(cppSelf)->contains(cppArg0);
            pyResult = Shiboken::Conversions::copyToPython(Shiboken::Conversions::PrimitiveTypeConverter<bool>(), &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_SettingsFunc_contains_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.contains");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_SettingsFunc_getSettings(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Settings *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: Settings::getSettings(QString)
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArg)))) {
        overloadId = 0; // getSettings(QString)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_SettingsFunc_getSettings_TypeError;

    // Call function/method
    {
        ::QString cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // getSettings(QString)
            qt_gui_cpp::Settings* cppResult = new qt_gui_cpp::Settings(cppSelf->getSettings(cppArg0));
            pyResult = Shiboken::Object::newObject(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX]), cppResult, true, true);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_SettingsFunc_getSettings_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.getSettings");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_SettingsFunc_remove(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Settings *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: Settings::remove(QString)
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArg)))) {
        overloadId = 0; // remove(QString)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_SettingsFunc_remove_TypeError;

    // Call function/method
    {
        ::QString cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // remove(QString)
            cppSelf->remove(cppArg0);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_SettingsFunc_remove_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.remove");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_SettingsFunc_setValue(PyObject *self, PyObject *args)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Settings *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0};

    // invalid argument lengths


    if (!PyArg_UnpackTuple(args, "setValue", 2, 2, &(pyArgs[0]), &(pyArgs[1])))
        return {};


    // Overloaded function decisor
    // 0: Settings::setValue(QString,QVariant)
    if (numArgs == 2
        && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[0])))
        && (pythonToCpp[1] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QVARIANT_IDX], (pyArgs[1])))) {
        overloadId = 0; // setValue(QString,QVariant)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_SettingsFunc_setValue_TypeError;

    // Call function/method
    {
        ::QString cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        ::QVariant cppArg1;
        pythonToCpp[1](pyArgs[1], &cppArg1);

        if (!PyErr_Occurred()) {
            // setValue(QString,QVariant)
            cppSelf->setValue(cppArg0, cppArg1);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_SettingsFunc_setValue_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.setValue");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_SettingsFunc_value(PyObject *self, PyObject *args, PyObject *kwds)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Settings *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numNamedArgs = (kwds ? PyDict_Size(kwds) : 0);
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0};

    // invalid argument lengths
    if (numArgs + numNamedArgs > 2) {
        PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.value(): too many arguments");
        return {};
    } else if (numArgs < 1) {
        PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.value(): not enough arguments");
        return {};
    }

    if (!PyArg_ParseTuple(args, "|OO:value", &(pyArgs[0]), &(pyArgs[1])))
        return {};


    // Overloaded function decisor
    // 0: Settings::value(QString,QVariant)const
    if ((pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[0])))) {
        if (numArgs == 1) {
            overloadId = 0; // value(QString,QVariant)const
        } else if ((pythonToCpp[1] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QVARIANT_IDX], (pyArgs[1])))) {
            overloadId = 0; // value(QString,QVariant)const
        }
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_SettingsFunc_value_TypeError;

    // Call function/method
    {
        if (kwds) {
            PyObject *keyName = nullptr;
            PyObject *value = nullptr;
            keyName = Py_BuildValue("s","defaultValue");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[1]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.value(): got multiple values for keyword argument 'defaultValue'.");
                    return {};
                }
                if (value) {
                    pyArgs[1] = value;
                    if (!(pythonToCpp[1] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QVARIANT_IDX], (pyArgs[1]))))
                        goto Sbk_qt_gui_cpp_SettingsFunc_value_TypeError;
                }
            }
        }
        ::QString cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        ::QVariant cppArg1 = QVariant();
        if (pythonToCpp[1]) pythonToCpp[1](pyArgs[1], &cppArg1);

        if (!PyErr_Occurred()) {
            // value(QString,QVariant)const
            QVariant cppResult = const_cast<const ::qt_gui_cpp::Settings *>(cppSelf)->value(cppArg0, cppArg1);
            pyResult = Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QVARIANT_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_SettingsFunc_value_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.value");
        return {};
}


static const char *Sbk_qt_gui_cpp_Settings_PropertyStrings[] = {
    nullptr // Sentinel
};

static PyMethodDef Sbk_qt_gui_cpp_Settings_methods[] = {
    {"allKeys", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_SettingsFunc_allKeys), METH_NOARGS},
    {"childGroups", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_SettingsFunc_childGroups), METH_NOARGS},
    {"childKeys", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_SettingsFunc_childKeys), METH_NOARGS},
    {"contains", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_SettingsFunc_contains), METH_O},
    {"getSettings", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_SettingsFunc_getSettings), METH_O},
    {"remove", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_SettingsFunc_remove), METH_O},
    {"setValue", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_SettingsFunc_setValue), METH_VARARGS},
    {"value", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_SettingsFunc_value), METH_VARARGS|METH_KEYWORDS},

    {nullptr, nullptr} // Sentinel
};

static PyObject *Sbk_qt_gui_cpp_Settings_get_proxy_(PyObject *self, void *)
{
    if (!Shiboken::Object::isValid(self))
        return nullptr;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Settings *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX], reinterpret_cast<SbkObject *>(self)));
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
static int Sbk_qt_gui_cpp_Settings_set_proxy_(PyObject *self, PyObject *pyIn, void *)
{
    if (!Shiboken::Object::isValid(self))
        return 0;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::Settings *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX], reinterpret_cast<SbkObject *>(self)));
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

// Getters and Setters for Settings
static PyGetSetDef Sbk_qt_gui_cpp_Settings_getsetlist[] = {
    {const_cast<char *>("proxy_"), Sbk_qt_gui_cpp_Settings_get_proxy_, Sbk_qt_gui_cpp_Settings_set_proxy_},
    {nullptr} // Sentinel
};

} // extern "C"

static int Sbk_qt_gui_cpp_Settings_traverse(PyObject *self, visitproc visit, void *arg)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_traverse(self, visit, arg);
}
static int Sbk_qt_gui_cpp_Settings_clear(PyObject *self)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_clear(self);
}
// Class Definition -----------------------------------------------
extern "C" {
static SbkObjectType *_Sbk_qt_gui_cpp_Settings_Type = nullptr;
static SbkObjectType *Sbk_qt_gui_cpp_Settings_TypeF(void)
{
    return _Sbk_qt_gui_cpp_Settings_Type;
}

static PyType_Slot Sbk_qt_gui_cpp_Settings_slots[] = {
    {Py_tp_base,        nullptr}, // inserted by introduceWrapperType
    {Py_tp_dealloc,     reinterpret_cast<void *>(&SbkDeallocWrapper)},
    {Py_tp_repr,        nullptr},
    {Py_tp_hash,        nullptr},
    {Py_tp_call,        nullptr},
    {Py_tp_str,         nullptr},
    {Py_tp_getattro,    nullptr},
    {Py_tp_setattro,    nullptr},
    {Py_tp_traverse,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_Settings_traverse)},
    {Py_tp_clear,       reinterpret_cast<void *>(Sbk_qt_gui_cpp_Settings_clear)},
    {Py_tp_richcompare, nullptr},
    {Py_tp_iter,        nullptr},
    {Py_tp_iternext,    nullptr},
    {Py_tp_methods,     reinterpret_cast<void *>(Sbk_qt_gui_cpp_Settings_methods)},
    {Py_tp_getset,      reinterpret_cast<void *>(Sbk_qt_gui_cpp_Settings_getsetlist)},
    {Py_tp_init,        reinterpret_cast<void *>(Sbk_qt_gui_cpp_Settings_Init)},
    {Py_tp_new,         reinterpret_cast<void *>(SbkObjectTpNew)},
    {0, nullptr}
};
static PyType_Spec Sbk_qt_gui_cpp_Settings_spec = {
    "1:libqt_gui_cpp_shiboken.qt_gui_cpp.Settings",
    sizeof(SbkObject),
    0,
    Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_CHECKTYPES|Py_TPFLAGS_HAVE_GC,
    Sbk_qt_gui_cpp_Settings_slots
};

} //extern "C"


// Type conversion functions.

// Python to C++ pointer conversion - returns the C++ object of the Python wrapper (keeps object identity).
static void Settings_PythonToCpp_Settings_PTR(PyObject *pyIn, void *cppOut) {
    Shiboken::Conversions::pythonToCppPointer(Sbk_qt_gui_cpp_Settings_TypeF(), pyIn, cppOut);
}
static PythonToCppFunc is_Settings_PythonToCpp_Settings_PTR_Convertible(PyObject *pyIn) {
    if (pyIn == Py_None)
        return Shiboken::Conversions::nonePythonToCppNullPtr;
    if (PyObject_TypeCheck(pyIn, reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_Settings_TypeF())))
        return Settings_PythonToCpp_Settings_PTR;
    return {};
}

// C++ to Python pointer conversion - tries to find the Python wrapper for the C++ object (keeps object identity).
static PyObject *Settings_PTR_CppToPython_Settings(const void *cppIn) {
    auto pyOut = reinterpret_cast<PyObject *>(Shiboken::BindingManager::instance().retrieveWrapper(cppIn));
    if (pyOut) {
        Py_INCREF(pyOut);
        return pyOut;
    }
    bool changedTypeName = false;
    auto tCppIn = reinterpret_cast<const ::qt_gui_cpp::Settings *>(cppIn);
    const char *typeName = typeid(*tCppIn).name();
    auto sbkType = Shiboken::ObjectType::typeForTypeName(typeName);
    if (sbkType && Shiboken::ObjectType::hasSpecialCastFunction(sbkType)) {
        typeName = typeNameOf(tCppIn);
        changedTypeName = true;
    }
    PyObject *result = Shiboken::Object::newObject(Sbk_qt_gui_cpp_Settings_TypeF(), const_cast<void *>(cppIn), false, /* exactType */ changedTypeName, typeName);
    if (changedTypeName)
        delete [] typeName;
    return result;
}

// The signatures string for the functions.
// Multiple signatures have their index "n:" in front.
static const char *qt_gui_cpp_Settings_SignatureStrings[] = {
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings(self,obj:PySide2.QtCore.QObject)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.allKeys(self)->QStringList",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.childGroups(self)->QStringList",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.childKeys(self)->QStringList",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.contains(self,key:QString)->bool",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.getSettings(self,prefix:QString)->libqt_gui_cpp_shiboken.qt_gui_cpp.Settings",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.remove(self,key:QString)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.setValue(self,key:QString,value:QVariant)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.Settings.value(self,key:QString,defaultValue:QVariant=QVariant())->QVariant",
    nullptr}; // Sentinel

void init_qt_gui_cpp_Settings(PyObject *enclosingClass)
{
    _Sbk_qt_gui_cpp_Settings_Type = Shiboken::ObjectType::introduceWrapperType(
        enclosingClass,
        "Settings",
        "qt_gui_cpp::Settings*",
        &Sbk_qt_gui_cpp_Settings_spec,
        &Shiboken::callCppDestructor< ::qt_gui_cpp::Settings >,
        0,
        0,
        Shiboken::ObjectType::WrapperFlags::InnerClass    );
    
    auto pyType = reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_Settings_Type);
    InitSignatureStrings(pyType, qt_gui_cpp_Settings_SignatureStrings);
    SbkObjectType_SetPropertyStrings(reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_Settings_Type), Sbk_qt_gui_cpp_Settings_PropertyStrings);
    Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX]
        = reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_Settings_TypeF());

    // Register Converter
    SbkConverter *converter = Shiboken::Conversions::createConverter(Sbk_qt_gui_cpp_Settings_TypeF(),
        Settings_PythonToCpp_Settings_PTR,
        is_Settings_PythonToCpp_Settings_PTR_Convertible,
        Settings_PTR_CppToPython_Settings);

    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::Settings");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::Settings*");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::Settings&");
    Shiboken::Conversions::registerConverterName(converter, "Settings");
    Shiboken::Conversions::registerConverterName(converter, "Settings*");
    Shiboken::Conversions::registerConverterName(converter, "Settings&");
    Shiboken::Conversions::registerConverterName(converter, typeid(::qt_gui_cpp::Settings).name());



}
