
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
#include "qt_gui_cpp_genericproxy_wrapper.h"

// inner classes

// Extra includes
#include <qobject.h>
#include <qobjectdefs.h>


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
Sbk_qt_gui_cpp_GenericProxy_Init(PyObject *self, PyObject *args, PyObject *kwds)
{
    SbkObject *sbkSelf = reinterpret_cast<SbkObject *>(self);
    if (Shiboken::Object::isUserType(self) && !Shiboken::ObjectType::canCallConstructor(self->ob_type, Shiboken::SbkType< ::qt_gui_cpp::GenericProxy >()))
        return -1;

    ::qt_gui_cpp::GenericProxy *cptr{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numNamedArgs = (kwds ? PyDict_Size(kwds) : 0);
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0};

    // invalid argument lengths
    if (numArgs + numNamedArgs > 1) {
        PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy(): too many arguments");
        return -1;
    }

    if (!PyArg_ParseTuple(args, "|O:GenericProxy", &(pyArgs[0])))
        return -1;


    // Overloaded function decisor
    // 0: GenericProxy::GenericProxy(QObject*)
    if (numArgs == 0) {
        overloadId = 0; // GenericProxy(QObject*)
    } else if ((pythonToCpp[0] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), (pyArgs[0])))) {
        overloadId = 0; // GenericProxy(QObject*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_GenericProxy_Init_TypeError;

    // Call function/method
    {
        if (kwds) {
            PyObject *keyName = nullptr;
            PyObject *value = nullptr;
            keyName = Py_BuildValue("s","obj");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[0]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy(): got multiple values for keyword argument 'obj'.");
                    return -1;
                }
                if (value) {
                    pyArgs[0] = value;
                    if (!(pythonToCpp[0] = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), (pyArgs[0]))))
                        goto Sbk_qt_gui_cpp_GenericProxy_Init_TypeError;
                }
            }
        }
        if (!Shiboken::Object::isValid(pyArgs[0]))
            return -1;
        ::QObject *cppArg0 = 0;
        if (pythonToCpp[0]) pythonToCpp[0](pyArgs[0], &cppArg0);

        if (!PyErr_Occurred()) {
            // GenericProxy(QObject*)
            cptr = new ::qt_gui_cpp::GenericProxy(cppArg0);
        }
    }

    if (PyErr_Occurred() || !Shiboken::Object::setCppPointer(sbkSelf, Shiboken::SbkType< ::qt_gui_cpp::GenericProxy >(), cptr)) {
        delete cptr;
        return -1;
    }
    if (!cptr) goto Sbk_qt_gui_cpp_GenericProxy_Init_TypeError;

    Shiboken::Object::setValidCpp(sbkSelf, true);
    if (Shiboken::BindingManager::instance().hasWrapper(cptr)) {
        Shiboken::BindingManager::instance().releaseWrapper(Shiboken::BindingManager::instance().retrieveWrapper(cptr));
    }
    Shiboken::BindingManager::instance().registerWrapper(sbkSelf, cptr);


    return 1;

    Sbk_qt_gui_cpp_GenericProxy_Init_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy");
        return -1;
}

static PyObject *Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethod(PyObject *self, PyObject *args, PyObject *kwds)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::GenericProxy *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_GENERICPROXY_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numNamedArgs = (kwds ? PyDict_Size(kwds) : 0);
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // invalid argument lengths
    if (numArgs + numNamedArgs > 11) {
        PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethod(): too many arguments");
        return {};
    } else if (numArgs < 1) {
        PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethod(): not enough arguments");
        return {};
    }

    if (!PyArg_ParseTuple(args, "|OOOOOOOOOOO:invokeMethod", &(pyArgs[0]), &(pyArgs[1]), &(pyArgs[2]), &(pyArgs[3]), &(pyArgs[4]), &(pyArgs[5]), &(pyArgs[6]), &(pyArgs[7]), &(pyArgs[8]), &(pyArgs[9]), &(pyArgs[10])))
        return {};


    // Overloaded function decisor
    // 0: GenericProxy::invokeMethod(const char*,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
    if (Shiboken::String::check(pyArgs[0]) && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(Shiboken::Conversions::PrimitiveTypeConverter<const char *>(), (pyArgs[0])))) {
        if (numArgs == 1) {
            overloadId = 0; // invokeMethod(const char*,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
        } else if ((pythonToCpp[1] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[1])))) {
            if (numArgs == 2) {
                overloadId = 0; // invokeMethod(const char*,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
            } else if ((pythonToCpp[2] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[2])))) {
                if (numArgs == 3) {
                    overloadId = 0; // invokeMethod(const char*,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                } else if ((pythonToCpp[3] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[3])))) {
                    if (numArgs == 4) {
                        overloadId = 0; // invokeMethod(const char*,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                    } else if ((pythonToCpp[4] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[4])))) {
                        if (numArgs == 5) {
                            overloadId = 0; // invokeMethod(const char*,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                        } else if ((pythonToCpp[5] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[5])))) {
                            if (numArgs == 6) {
                                overloadId = 0; // invokeMethod(const char*,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                            } else if ((pythonToCpp[6] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[6])))) {
                                if (numArgs == 7) {
                                    overloadId = 0; // invokeMethod(const char*,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                                } else if ((pythonToCpp[7] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[7])))) {
                                    if (numArgs == 8) {
                                        overloadId = 0; // invokeMethod(const char*,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                                    } else if ((pythonToCpp[8] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[8])))) {
                                        if (numArgs == 9) {
                                            overloadId = 0; // invokeMethod(const char*,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                                        } else if ((pythonToCpp[9] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[9])))) {
                                            if (numArgs == 10) {
                                                overloadId = 0; // invokeMethod(const char*,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                                            } else if ((pythonToCpp[10] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[10])))) {
                                                overloadId = 0; // invokeMethod(const char*,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethod_TypeError;

    // Call function/method
    {
        if (kwds) {
            PyObject *keyName = nullptr;
            PyObject *value = nullptr;
            keyName = Py_BuildValue("s","val0");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[1]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethod(): got multiple values for keyword argument 'val0'.");
                    return {};
                }
                if (value) {
                    pyArgs[1] = value;
                    if (!(pythonToCpp[1] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[1]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethod_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val1");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[2]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethod(): got multiple values for keyword argument 'val1'.");
                    return {};
                }
                if (value) {
                    pyArgs[2] = value;
                    if (!(pythonToCpp[2] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[2]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethod_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val2");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[3]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethod(): got multiple values for keyword argument 'val2'.");
                    return {};
                }
                if (value) {
                    pyArgs[3] = value;
                    if (!(pythonToCpp[3] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[3]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethod_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val3");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[4]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethod(): got multiple values for keyword argument 'val3'.");
                    return {};
                }
                if (value) {
                    pyArgs[4] = value;
                    if (!(pythonToCpp[4] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[4]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethod_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val4");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[5]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethod(): got multiple values for keyword argument 'val4'.");
                    return {};
                }
                if (value) {
                    pyArgs[5] = value;
                    if (!(pythonToCpp[5] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[5]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethod_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val5");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[6]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethod(): got multiple values for keyword argument 'val5'.");
                    return {};
                }
                if (value) {
                    pyArgs[6] = value;
                    if (!(pythonToCpp[6] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[6]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethod_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val6");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[7]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethod(): got multiple values for keyword argument 'val6'.");
                    return {};
                }
                if (value) {
                    pyArgs[7] = value;
                    if (!(pythonToCpp[7] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[7]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethod_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val7");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[8]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethod(): got multiple values for keyword argument 'val7'.");
                    return {};
                }
                if (value) {
                    pyArgs[8] = value;
                    if (!(pythonToCpp[8] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[8]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethod_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val8");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[9]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethod(): got multiple values for keyword argument 'val8'.");
                    return {};
                }
                if (value) {
                    pyArgs[9] = value;
                    if (!(pythonToCpp[9] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[9]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethod_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val9");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[10]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethod(): got multiple values for keyword argument 'val9'.");
                    return {};
                }
                if (value) {
                    pyArgs[10] = value;
                    if (!(pythonToCpp[10] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[10]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethod_TypeError;
                }
            }
        }
        const char* cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        if (!Shiboken::Object::isValid(pyArgs[1]))
            return {};
        ::QGenericArgument cppArg1 = QGenericArgument();
        if (pythonToCpp[1]) pythonToCpp[1](pyArgs[1], &cppArg1);
        if (!Shiboken::Object::isValid(pyArgs[2]))
            return {};
        ::QGenericArgument cppArg2 = QGenericArgument();
        if (pythonToCpp[2]) pythonToCpp[2](pyArgs[2], &cppArg2);
        if (!Shiboken::Object::isValid(pyArgs[3]))
            return {};
        ::QGenericArgument cppArg3 = QGenericArgument();
        if (pythonToCpp[3]) pythonToCpp[3](pyArgs[3], &cppArg3);
        if (!Shiboken::Object::isValid(pyArgs[4]))
            return {};
        ::QGenericArgument cppArg4 = QGenericArgument();
        if (pythonToCpp[4]) pythonToCpp[4](pyArgs[4], &cppArg4);
        if (!Shiboken::Object::isValid(pyArgs[5]))
            return {};
        ::QGenericArgument cppArg5 = QGenericArgument();
        if (pythonToCpp[5]) pythonToCpp[5](pyArgs[5], &cppArg5);
        if (!Shiboken::Object::isValid(pyArgs[6]))
            return {};
        ::QGenericArgument cppArg6 = QGenericArgument();
        if (pythonToCpp[6]) pythonToCpp[6](pyArgs[6], &cppArg6);
        if (!Shiboken::Object::isValid(pyArgs[7]))
            return {};
        ::QGenericArgument cppArg7 = QGenericArgument();
        if (pythonToCpp[7]) pythonToCpp[7](pyArgs[7], &cppArg7);
        if (!Shiboken::Object::isValid(pyArgs[8]))
            return {};
        ::QGenericArgument cppArg8 = QGenericArgument();
        if (pythonToCpp[8]) pythonToCpp[8](pyArgs[8], &cppArg8);
        if (!Shiboken::Object::isValid(pyArgs[9]))
            return {};
        ::QGenericArgument cppArg9 = QGenericArgument();
        if (pythonToCpp[9]) pythonToCpp[9](pyArgs[9], &cppArg9);
        if (!Shiboken::Object::isValid(pyArgs[10]))
            return {};
        ::QGenericArgument cppArg10 = QGenericArgument();
        if (pythonToCpp[10]) pythonToCpp[10](pyArgs[10], &cppArg10);

        if (!PyErr_Occurred()) {
            // invokeMethod(const char*,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
            bool cppResult = cppSelf->invokeMethod(cppArg0, cppArg1, cppArg2, cppArg3, cppArg4, cppArg5, cppArg6, cppArg7, cppArg8, cppArg9, cppArg10);
            pyResult = Shiboken::Conversions::copyToPython(Shiboken::Conversions::PrimitiveTypeConverter<bool>(), &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethod_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethod");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn(PyObject *self, PyObject *args, PyObject *kwds)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::GenericProxy *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_GENERICPROXY_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numNamedArgs = (kwds ? PyDict_Size(kwds) : 0);
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // invalid argument lengths
    if (numArgs + numNamedArgs > 12) {
        PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn(): too many arguments");
        return {};
    } else if (numArgs < 1) {
        PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn(): not enough arguments");
        return {};
    }

    if (!PyArg_ParseTuple(args, "|OOOOOOOOOOOO:invokeMethodWithReturn", &(pyArgs[0]), &(pyArgs[1]), &(pyArgs[2]), &(pyArgs[3]), &(pyArgs[4]), &(pyArgs[5]), &(pyArgs[6]), &(pyArgs[7]), &(pyArgs[8]), &(pyArgs[9]), &(pyArgs[10]), &(pyArgs[11])))
        return {};


    // Overloaded function decisor
    // 0: GenericProxy::invokeMethodWithReturn(const char*,QGenericReturnArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
    if (Shiboken::String::check(pyArgs[0]) && (pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(Shiboken::Conversions::PrimitiveTypeConverter<const char *>(), (pyArgs[0])))) {
        if (numArgs == 1) {
            overloadId = 0; // invokeMethodWithReturn(const char*,QGenericReturnArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
        } else if ((pythonToCpp[1] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICRETURNARGUMENT_IDX]), (pyArgs[1])))) {
            if (numArgs == 2) {
                overloadId = 0; // invokeMethodWithReturn(const char*,QGenericReturnArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
            } else if ((pythonToCpp[2] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[2])))) {
                if (numArgs == 3) {
                    overloadId = 0; // invokeMethodWithReturn(const char*,QGenericReturnArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                } else if ((pythonToCpp[3] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[3])))) {
                    if (numArgs == 4) {
                        overloadId = 0; // invokeMethodWithReturn(const char*,QGenericReturnArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                    } else if ((pythonToCpp[4] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[4])))) {
                        if (numArgs == 5) {
                            overloadId = 0; // invokeMethodWithReturn(const char*,QGenericReturnArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                        } else if ((pythonToCpp[5] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[5])))) {
                            if (numArgs == 6) {
                                overloadId = 0; // invokeMethodWithReturn(const char*,QGenericReturnArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                            } else if ((pythonToCpp[6] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[6])))) {
                                if (numArgs == 7) {
                                    overloadId = 0; // invokeMethodWithReturn(const char*,QGenericReturnArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                                } else if ((pythonToCpp[7] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[7])))) {
                                    if (numArgs == 8) {
                                        overloadId = 0; // invokeMethodWithReturn(const char*,QGenericReturnArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                                    } else if ((pythonToCpp[8] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[8])))) {
                                        if (numArgs == 9) {
                                            overloadId = 0; // invokeMethodWithReturn(const char*,QGenericReturnArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                                        } else if ((pythonToCpp[9] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[9])))) {
                                            if (numArgs == 10) {
                                                overloadId = 0; // invokeMethodWithReturn(const char*,QGenericReturnArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                                            } else if ((pythonToCpp[10] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[10])))) {
                                                if (numArgs == 11) {
                                                    overloadId = 0; // invokeMethodWithReturn(const char*,QGenericReturnArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                                                } else if ((pythonToCpp[11] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[11])))) {
                                                    overloadId = 0; // invokeMethodWithReturn(const char*,QGenericReturnArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn_TypeError;

    // Call function/method
    {
        if (kwds) {
            PyObject *keyName = nullptr;
            PyObject *value = nullptr;
            keyName = Py_BuildValue("s","ret");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[1]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn(): got multiple values for keyword argument 'ret'.");
                    return {};
                }
                if (value) {
                    pyArgs[1] = value;
                    if (!(pythonToCpp[1] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICRETURNARGUMENT_IDX]), (pyArgs[1]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val0");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[2]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn(): got multiple values for keyword argument 'val0'.");
                    return {};
                }
                if (value) {
                    pyArgs[2] = value;
                    if (!(pythonToCpp[2] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[2]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val1");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[3]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn(): got multiple values for keyword argument 'val1'.");
                    return {};
                }
                if (value) {
                    pyArgs[3] = value;
                    if (!(pythonToCpp[3] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[3]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val2");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[4]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn(): got multiple values for keyword argument 'val2'.");
                    return {};
                }
                if (value) {
                    pyArgs[4] = value;
                    if (!(pythonToCpp[4] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[4]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val3");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[5]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn(): got multiple values for keyword argument 'val3'.");
                    return {};
                }
                if (value) {
                    pyArgs[5] = value;
                    if (!(pythonToCpp[5] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[5]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val4");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[6]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn(): got multiple values for keyword argument 'val4'.");
                    return {};
                }
                if (value) {
                    pyArgs[6] = value;
                    if (!(pythonToCpp[6] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[6]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val5");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[7]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn(): got multiple values for keyword argument 'val5'.");
                    return {};
                }
                if (value) {
                    pyArgs[7] = value;
                    if (!(pythonToCpp[7] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[7]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val6");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[8]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn(): got multiple values for keyword argument 'val6'.");
                    return {};
                }
                if (value) {
                    pyArgs[8] = value;
                    if (!(pythonToCpp[8] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[8]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val7");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[9]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn(): got multiple values for keyword argument 'val7'.");
                    return {};
                }
                if (value) {
                    pyArgs[9] = value;
                    if (!(pythonToCpp[9] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[9]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val8");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[10]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn(): got multiple values for keyword argument 'val8'.");
                    return {};
                }
                if (value) {
                    pyArgs[10] = value;
                    if (!(pythonToCpp[10] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[10]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn_TypeError;
                }
            }
            keyName = Py_BuildValue("s","val9");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[11]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn(): got multiple values for keyword argument 'val9'.");
                    return {};
                }
                if (value) {
                    pyArgs[11] = value;
                    if (!(pythonToCpp[11] = Shiboken::Conversions::isPythonToCppValueConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QGENERICARGUMENT_IDX]), (pyArgs[11]))))
                        goto Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn_TypeError;
                }
            }
        }
        const char* cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        if (!Shiboken::Object::isValid(pyArgs[1]))
            return {};
        ::QGenericReturnArgument cppArg1 = QGenericReturnArgument(0, 0);
        if (pythonToCpp[1]) pythonToCpp[1](pyArgs[1], &cppArg1);
        if (!Shiboken::Object::isValid(pyArgs[2]))
            return {};
        ::QGenericArgument cppArg2 = QGenericArgument();
        if (pythonToCpp[2]) pythonToCpp[2](pyArgs[2], &cppArg2);
        if (!Shiboken::Object::isValid(pyArgs[3]))
            return {};
        ::QGenericArgument cppArg3 = QGenericArgument();
        if (pythonToCpp[3]) pythonToCpp[3](pyArgs[3], &cppArg3);
        if (!Shiboken::Object::isValid(pyArgs[4]))
            return {};
        ::QGenericArgument cppArg4 = QGenericArgument();
        if (pythonToCpp[4]) pythonToCpp[4](pyArgs[4], &cppArg4);
        if (!Shiboken::Object::isValid(pyArgs[5]))
            return {};
        ::QGenericArgument cppArg5 = QGenericArgument();
        if (pythonToCpp[5]) pythonToCpp[5](pyArgs[5], &cppArg5);
        if (!Shiboken::Object::isValid(pyArgs[6]))
            return {};
        ::QGenericArgument cppArg6 = QGenericArgument();
        if (pythonToCpp[6]) pythonToCpp[6](pyArgs[6], &cppArg6);
        if (!Shiboken::Object::isValid(pyArgs[7]))
            return {};
        ::QGenericArgument cppArg7 = QGenericArgument();
        if (pythonToCpp[7]) pythonToCpp[7](pyArgs[7], &cppArg7);
        if (!Shiboken::Object::isValid(pyArgs[8]))
            return {};
        ::QGenericArgument cppArg8 = QGenericArgument();
        if (pythonToCpp[8]) pythonToCpp[8](pyArgs[8], &cppArg8);
        if (!Shiboken::Object::isValid(pyArgs[9]))
            return {};
        ::QGenericArgument cppArg9 = QGenericArgument();
        if (pythonToCpp[9]) pythonToCpp[9](pyArgs[9], &cppArg9);
        if (!Shiboken::Object::isValid(pyArgs[10]))
            return {};
        ::QGenericArgument cppArg10 = QGenericArgument();
        if (pythonToCpp[10]) pythonToCpp[10](pyArgs[10], &cppArg10);
        if (!Shiboken::Object::isValid(pyArgs[11]))
            return {};
        ::QGenericArgument cppArg11 = QGenericArgument();
        if (pythonToCpp[11]) pythonToCpp[11](pyArgs[11], &cppArg11);

        if (!PyErr_Occurred()) {
            // invokeMethodWithReturn(const char*,QGenericReturnArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument,QGenericArgument)
            bool cppResult = cppSelf->invokeMethodWithReturn(cppArg0, cppArg1, cppArg2, cppArg3, cppArg4, cppArg5, cppArg6, cppArg7, cppArg8, cppArg9, cppArg10, cppArg11);
            pyResult = Shiboken::Conversions::copyToPython(Shiboken::Conversions::PrimitiveTypeConverter<bool>(), &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_GenericProxyFunc_proxiedObject(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::GenericProxy *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_GENERICPROXY_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // proxiedObject()
            QObject * cppResult = cppSelf->proxiedObject();
            pyResult = Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;
}

static PyObject *Sbk_qt_gui_cpp_GenericProxyFunc_setProxiedObject(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::GenericProxy *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_GENERICPROXY_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: GenericProxy::setProxiedObject(QObject*)
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppPointerConvertible(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), (pyArg)))) {
        overloadId = 0; // setProxiedObject(QObject*)
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_GenericProxyFunc_setProxiedObject_TypeError;

    // Call function/method
    {
        if (!Shiboken::Object::isValid(pyArg))
            return {};
        ::QObject *cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // setProxiedObject(QObject*)
            cppSelf->setProxiedObject(cppArg0);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_GenericProxyFunc_setProxiedObject_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.setProxiedObject");
        return {};
}


static const char *Sbk_qt_gui_cpp_GenericProxy_PropertyStrings[] = {
    nullptr // Sentinel
};

static PyMethodDef Sbk_qt_gui_cpp_GenericProxy_methods[] = {
    {"invokeMethod", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethod), METH_VARARGS|METH_KEYWORDS},
    {"invokeMethodWithReturn", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_GenericProxyFunc_invokeMethodWithReturn), METH_VARARGS|METH_KEYWORDS},
    {"proxiedObject", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_GenericProxyFunc_proxiedObject), METH_NOARGS},
    {"setProxiedObject", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_GenericProxyFunc_setProxiedObject), METH_O},

    {nullptr, nullptr} // Sentinel
};

} // extern "C"

static int Sbk_qt_gui_cpp_GenericProxy_traverse(PyObject *self, visitproc visit, void *arg)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_traverse(self, visit, arg);
}
static int Sbk_qt_gui_cpp_GenericProxy_clear(PyObject *self)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_clear(self);
}
// Class Definition -----------------------------------------------
extern "C" {
static SbkObjectType *_Sbk_qt_gui_cpp_GenericProxy_Type = nullptr;
static SbkObjectType *Sbk_qt_gui_cpp_GenericProxy_TypeF(void)
{
    return _Sbk_qt_gui_cpp_GenericProxy_Type;
}

static PyType_Slot Sbk_qt_gui_cpp_GenericProxy_slots[] = {
    {Py_tp_base,        nullptr}, // inserted by introduceWrapperType
    {Py_tp_dealloc,     reinterpret_cast<void *>(&SbkDeallocWrapper)},
    {Py_tp_repr,        nullptr},
    {Py_tp_hash,        nullptr},
    {Py_tp_call,        nullptr},
    {Py_tp_str,         nullptr},
    {Py_tp_getattro,    nullptr},
    {Py_tp_setattro,    nullptr},
    {Py_tp_traverse,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_GenericProxy_traverse)},
    {Py_tp_clear,       reinterpret_cast<void *>(Sbk_qt_gui_cpp_GenericProxy_clear)},
    {Py_tp_richcompare, nullptr},
    {Py_tp_iter,        nullptr},
    {Py_tp_iternext,    nullptr},
    {Py_tp_methods,     reinterpret_cast<void *>(Sbk_qt_gui_cpp_GenericProxy_methods)},
    {Py_tp_getset,      nullptr},
    {Py_tp_init,        reinterpret_cast<void *>(Sbk_qt_gui_cpp_GenericProxy_Init)},
    {Py_tp_new,         reinterpret_cast<void *>(SbkObjectTpNew)},
    {0, nullptr}
};
static PyType_Spec Sbk_qt_gui_cpp_GenericProxy_spec = {
    "1:libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy",
    sizeof(SbkObject),
    0,
    Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_CHECKTYPES|Py_TPFLAGS_HAVE_GC,
    Sbk_qt_gui_cpp_GenericProxy_slots
};

} //extern "C"


// Type conversion functions.

// Python to C++ pointer conversion - returns the C++ object of the Python wrapper (keeps object identity).
static void GenericProxy_PythonToCpp_GenericProxy_PTR(PyObject *pyIn, void *cppOut) {
    Shiboken::Conversions::pythonToCppPointer(Sbk_qt_gui_cpp_GenericProxy_TypeF(), pyIn, cppOut);
}
static PythonToCppFunc is_GenericProxy_PythonToCpp_GenericProxy_PTR_Convertible(PyObject *pyIn) {
    if (pyIn == Py_None)
        return Shiboken::Conversions::nonePythonToCppNullPtr;
    if (PyObject_TypeCheck(pyIn, reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_GenericProxy_TypeF())))
        return GenericProxy_PythonToCpp_GenericProxy_PTR;
    return {};
}

// C++ to Python pointer conversion - tries to find the Python wrapper for the C++ object (keeps object identity).
static PyObject *GenericProxy_PTR_CppToPython_GenericProxy(const void *cppIn) {
    auto pyOut = reinterpret_cast<PyObject *>(Shiboken::BindingManager::instance().retrieveWrapper(cppIn));
    if (pyOut) {
        Py_INCREF(pyOut);
        return pyOut;
    }
    bool changedTypeName = false;
    auto tCppIn = reinterpret_cast<const ::qt_gui_cpp::GenericProxy *>(cppIn);
    const char *typeName = typeid(*tCppIn).name();
    auto sbkType = Shiboken::ObjectType::typeForTypeName(typeName);
    if (sbkType && Shiboken::ObjectType::hasSpecialCastFunction(sbkType)) {
        typeName = typeNameOf(tCppIn);
        changedTypeName = true;
    }
    PyObject *result = Shiboken::Object::newObject(Sbk_qt_gui_cpp_GenericProxy_TypeF(), const_cast<void *>(cppIn), false, /* exactType */ changedTypeName, typeName);
    if (changedTypeName)
        delete [] typeName;
    return result;
}

// The signatures string for the functions.
// Multiple signatures have their index "n:" in front.
static const char *qt_gui_cpp_GenericProxy_SignatureStrings[] = {
    "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy(self,obj:PySide2.QtCore.QObject=0)",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethod(self,member:char*,val0:PySide2.QtCore.QGenericArgument=QGenericArgument(),val1:PySide2.QtCore.QGenericArgument=QGenericArgument(),val2:PySide2.QtCore.QGenericArgument=QGenericArgument(),val3:PySide2.QtCore.QGenericArgument=QGenericArgument(),val4:PySide2.QtCore.QGenericArgument=QGenericArgument(),val5:PySide2.QtCore.QGenericArgument=QGenericArgument(),val6:PySide2.QtCore.QGenericArgument=QGenericArgument(),val7:PySide2.QtCore.QGenericArgument=QGenericArgument(),val8:PySide2.QtCore.QGenericArgument=QGenericArgument(),val9:PySide2.QtCore.QGenericArgument=QGenericArgument())->bool",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.invokeMethodWithReturn(self,member:char*,ret:PySide2.QtCore.QGenericReturnArgument=QGenericReturnArgument(0, 0),val0:PySide2.QtCore.QGenericArgument=QGenericArgument(),val1:PySide2.QtCore.QGenericArgument=QGenericArgument(),val2:PySide2.QtCore.QGenericArgument=QGenericArgument(),val3:PySide2.QtCore.QGenericArgument=QGenericArgument(),val4:PySide2.QtCore.QGenericArgument=QGenericArgument(),val5:PySide2.QtCore.QGenericArgument=QGenericArgument(),val6:PySide2.QtCore.QGenericArgument=QGenericArgument(),val7:PySide2.QtCore.QGenericArgument=QGenericArgument(),val8:PySide2.QtCore.QGenericArgument=QGenericArgument(),val9:PySide2.QtCore.QGenericArgument=QGenericArgument())->bool",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.proxiedObject(self)->PySide2.QtCore.QObject",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.GenericProxy.setProxiedObject(self,obj:PySide2.QtCore.QObject)",
    nullptr}; // Sentinel

void init_qt_gui_cpp_GenericProxy(PyObject *enclosingClass)
{
    _Sbk_qt_gui_cpp_GenericProxy_Type = Shiboken::ObjectType::introduceWrapperType(
        enclosingClass,
        "GenericProxy",
        "qt_gui_cpp::GenericProxy*",
        &Sbk_qt_gui_cpp_GenericProxy_spec,
        &Shiboken::callCppDestructor< ::qt_gui_cpp::GenericProxy >,
        0,
        0,
        Shiboken::ObjectType::WrapperFlags::InnerClass    );
    
    auto pyType = reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_GenericProxy_Type);
    InitSignatureStrings(pyType, qt_gui_cpp_GenericProxy_SignatureStrings);
    SbkObjectType_SetPropertyStrings(reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_GenericProxy_Type), Sbk_qt_gui_cpp_GenericProxy_PropertyStrings);
    Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_GENERICPROXY_IDX]
        = reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_GenericProxy_TypeF());

    // Register Converter
    SbkConverter *converter = Shiboken::Conversions::createConverter(Sbk_qt_gui_cpp_GenericProxy_TypeF(),
        GenericProxy_PythonToCpp_GenericProxy_PTR,
        is_GenericProxy_PythonToCpp_GenericProxy_PTR_Convertible,
        GenericProxy_PTR_CppToPython_GenericProxy);

    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::GenericProxy");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::GenericProxy*");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::GenericProxy&");
    Shiboken::Conversions::registerConverterName(converter, "GenericProxy");
    Shiboken::Conversions::registerConverterName(converter, "GenericProxy*");
    Shiboken::Conversions::registerConverterName(converter, "GenericProxy&");
    Shiboken::Conversions::registerConverterName(converter, typeid(::qt_gui_cpp::GenericProxy).name());



}
