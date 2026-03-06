
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
#include "qt_gui_cpp_plugindescriptor_wrapper.h"

// inner classes

// Extra includes
#include <QMap>


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
Sbk_qt_gui_cpp_PluginDescriptor_Init(PyObject *self, PyObject *args, PyObject *kwds)
{
    SbkObject *sbkSelf = reinterpret_cast<SbkObject *>(self);
    if (Shiboken::Object::isUserType(self) && !Shiboken::ObjectType::canCallConstructor(self->ob_type, Shiboken::SbkType< ::qt_gui_cpp::PluginDescriptor >()))
        return -1;

    ::qt_gui_cpp::PluginDescriptor *cptr{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numNamedArgs = (kwds ? PyDict_Size(kwds) : 0);
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0};

    // invalid argument lengths
    if (numArgs + numNamedArgs > 2) {
        PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor(): too many arguments");
        return -1;
    } else if (numArgs < 1) {
        PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor(): not enough arguments");
        return -1;
    }

    if (!PyArg_ParseTuple(args, "|OO:PluginDescriptor", &(pyArgs[0]), &(pyArgs[1])))
        return -1;


    // Overloaded function decisor
    // 0: PluginDescriptor::PluginDescriptor(QString,QMap<QString,QString>)
    if ((pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[0])))) {
        if (numArgs == 1) {
            overloadId = 0; // PluginDescriptor(QString,QMap<QString,QString>)
        } else if ((pythonToCpp[1] = Shiboken::Conversions::isPythonToCppConvertible(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], (pyArgs[1])))) {
            overloadId = 0; // PluginDescriptor(QString,QMap<QString,QString>)
        }
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginDescriptor_Init_TypeError;

    // Call function/method
    {
        if (kwds) {
            PyObject *keyName = nullptr;
            PyObject *value = nullptr;
            keyName = Py_BuildValue("s","attributes");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[1]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor(): got multiple values for keyword argument 'attributes'.");
                    return -1;
                }
                if (value) {
                    pyArgs[1] = value;
                    if (!(pythonToCpp[1] = Shiboken::Conversions::isPythonToCppConvertible(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], (pyArgs[1]))))
                        goto Sbk_qt_gui_cpp_PluginDescriptor_Init_TypeError;
                }
            }
        }
        ::QString cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        ::QMap<QString,QString > cppArg1;
        if (pythonToCpp[1]) pythonToCpp[1](pyArgs[1], &cppArg1);

        if (!PyErr_Occurred()) {
            // PluginDescriptor(QString,QMap<QString,QString>)
            cptr = new ::qt_gui_cpp::PluginDescriptor(cppArg0, cppArg1);
        }
    }

    if (PyErr_Occurred() || !Shiboken::Object::setCppPointer(sbkSelf, Shiboken::SbkType< ::qt_gui_cpp::PluginDescriptor >(), cptr)) {
        delete cptr;
        return -1;
    }
    if (!cptr) goto Sbk_qt_gui_cpp_PluginDescriptor_Init_TypeError;

    Shiboken::Object::setValidCpp(sbkSelf, true);
    if (Shiboken::BindingManager::instance().hasWrapper(cptr)) {
        Shiboken::BindingManager::instance().releaseWrapper(Shiboken::BindingManager::instance().retrieveWrapper(cptr));
    }
    Shiboken::BindingManager::instance().registerWrapper(sbkSelf, cptr);


    return 1;

    Sbk_qt_gui_cpp_PluginDescriptor_Init_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor");
        return -1;
}

static PyObject *Sbk_qt_gui_cpp_PluginDescriptorFunc_actionAttributes(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // actionAttributes()const
            const QMap<QString,QString > & cppResult = const_cast<const ::qt_gui_cpp::PluginDescriptor *>(cppSelf)->actionAttributes();
            pyResult = Shiboken::Conversions::copyToPython(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;
}

static PyObject *Sbk_qt_gui_cpp_PluginDescriptorFunc_addGroupAttributes(PyObject *self, PyObject *args, PyObject *kwds)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr, nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numNamedArgs = (kwds ? PyDict_Size(kwds) : 0);
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0, 0, 0};

    // invalid argument lengths
    if (numArgs + numNamedArgs > 4) {
        PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.addGroupAttributes(): too many arguments");
        return {};
    } else if (numArgs < 1) {
        PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.addGroupAttributes(): not enough arguments");
        return {};
    }

    if (!PyArg_ParseTuple(args, "|OOOO:addGroupAttributes", &(pyArgs[0]), &(pyArgs[1]), &(pyArgs[2]), &(pyArgs[3])))
        return {};


    // Overloaded function decisor
    // 0: PluginDescriptor::addGroupAttributes(QString,QString,QString,QString)
    if ((pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[0])))) {
        if (numArgs == 1) {
            overloadId = 0; // addGroupAttributes(QString,QString,QString,QString)
        } else if ((pythonToCpp[1] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[1])))) {
            if (numArgs == 2) {
                overloadId = 0; // addGroupAttributes(QString,QString,QString,QString)
            } else if ((pythonToCpp[2] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[2])))) {
                if (numArgs == 3) {
                    overloadId = 0; // addGroupAttributes(QString,QString,QString,QString)
                } else if ((pythonToCpp[3] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[3])))) {
                    overloadId = 0; // addGroupAttributes(QString,QString,QString,QString)
                }
            }
        }
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginDescriptorFunc_addGroupAttributes_TypeError;

    // Call function/method
    {
        if (kwds) {
            PyObject *keyName = nullptr;
            PyObject *value = nullptr;
            keyName = Py_BuildValue("s","statustip");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[1]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.addGroupAttributes(): got multiple values for keyword argument 'statustip'.");
                    return {};
                }
                if (value) {
                    pyArgs[1] = value;
                    if (!(pythonToCpp[1] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[1]))))
                        goto Sbk_qt_gui_cpp_PluginDescriptorFunc_addGroupAttributes_TypeError;
                }
            }
            keyName = Py_BuildValue("s","icon");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[2]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.addGroupAttributes(): got multiple values for keyword argument 'icon'.");
                    return {};
                }
                if (value) {
                    pyArgs[2] = value;
                    if (!(pythonToCpp[2] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[2]))))
                        goto Sbk_qt_gui_cpp_PluginDescriptorFunc_addGroupAttributes_TypeError;
                }
            }
            keyName = Py_BuildValue("s","icontype");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[3]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.addGroupAttributes(): got multiple values for keyword argument 'icontype'.");
                    return {};
                }
                if (value) {
                    pyArgs[3] = value;
                    if (!(pythonToCpp[3] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[3]))))
                        goto Sbk_qt_gui_cpp_PluginDescriptorFunc_addGroupAttributes_TypeError;
                }
            }
        }
        ::QString cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        ::QString cppArg1 = QString();
        if (pythonToCpp[1]) pythonToCpp[1](pyArgs[1], &cppArg1);
        ::QString cppArg2 = QString();
        if (pythonToCpp[2]) pythonToCpp[2](pyArgs[2], &cppArg2);
        ::QString cppArg3 = QString();
        if (pythonToCpp[3]) pythonToCpp[3](pyArgs[3], &cppArg3);

        if (!PyErr_Occurred()) {
            // addGroupAttributes(QString,QString,QString,QString)
            cppSelf->addGroupAttributes(cppArg0, cppArg1, cppArg2, cppArg3);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_PluginDescriptorFunc_addGroupAttributes_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.addGroupAttributes");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginDescriptorFunc_attributes(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // attributes()
            QMap<QString,QString > & cppResult = cppSelf->attributes();
            pyResult = Shiboken::Conversions::copyToPython(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;
}

static PyObject *Sbk_qt_gui_cpp_PluginDescriptorFunc_countGroups(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // countGroups()const
            int cppResult = const_cast<const ::qt_gui_cpp::PluginDescriptor *>(cppSelf)->countGroups();
            pyResult = Shiboken::Conversions::copyToPython(Shiboken::Conversions::PrimitiveTypeConverter<int>(), &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;
}

static PyObject *Sbk_qt_gui_cpp_PluginDescriptorFunc_group(PyObject *self, PyObject *pyArg)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};
    int overloadId = -1;
    PythonToCppFunc pythonToCpp{};
    SBK_UNUSED(pythonToCpp)

    // Overloaded function decisor
    // 0: PluginDescriptor::group(int)const
    if ((pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(Shiboken::Conversions::PrimitiveTypeConverter<int>(), (pyArg)))) {
        overloadId = 0; // group(int)const
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginDescriptorFunc_group_TypeError;

    // Call function/method
    {
        int cppArg0;
        pythonToCpp(pyArg, &cppArg0);

        if (!PyErr_Occurred()) {
            // group(int)const
            QMap<QString,QString > cppResult = const_cast<const ::qt_gui_cpp::PluginDescriptor *>(cppSelf)->group(cppArg0);
            pyResult = Shiboken::Conversions::copyToPython(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;

    Sbk_qt_gui_cpp_PluginDescriptorFunc_group_TypeError:
        Shiboken::setErrorAboutWrongArguments(pyArg, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.group");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginDescriptorFunc_pluginId(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // pluginId()const
            const QString & cppResult = const_cast<const ::qt_gui_cpp::PluginDescriptor *>(cppSelf)->pluginId();
            pyResult = Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;
}

static PyObject *Sbk_qt_gui_cpp_PluginDescriptorFunc_setActionAttributes(PyObject *self, PyObject *args, PyObject *kwds)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    int overloadId = -1;
    PythonToCppFunc pythonToCpp[] = { nullptr, nullptr, nullptr, nullptr };
    SBK_UNUSED(pythonToCpp)
    const Py_ssize_t numNamedArgs = (kwds ? PyDict_Size(kwds) : 0);
    const Py_ssize_t numArgs = PyTuple_GET_SIZE(args);
    SBK_UNUSED(numArgs)
    PyObject *pyArgs[] = {0, 0, 0, 0};

    // invalid argument lengths
    if (numArgs + numNamedArgs > 4) {
        PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.setActionAttributes(): too many arguments");
        return {};
    } else if (numArgs < 1) {
        PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.setActionAttributes(): not enough arguments");
        return {};
    }

    if (!PyArg_ParseTuple(args, "|OOOO:setActionAttributes", &(pyArgs[0]), &(pyArgs[1]), &(pyArgs[2]), &(pyArgs[3])))
        return {};


    // Overloaded function decisor
    // 0: PluginDescriptor::setActionAttributes(QString,QString,QString,QString)
    if ((pythonToCpp[0] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[0])))) {
        if (numArgs == 1) {
            overloadId = 0; // setActionAttributes(QString,QString,QString,QString)
        } else if ((pythonToCpp[1] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[1])))) {
            if (numArgs == 2) {
                overloadId = 0; // setActionAttributes(QString,QString,QString,QString)
            } else if ((pythonToCpp[2] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[2])))) {
                if (numArgs == 3) {
                    overloadId = 0; // setActionAttributes(QString,QString,QString,QString)
                } else if ((pythonToCpp[3] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[3])))) {
                    overloadId = 0; // setActionAttributes(QString,QString,QString,QString)
                }
            }
        }
    }

    // Function signature not found.
    if (overloadId == -1) goto Sbk_qt_gui_cpp_PluginDescriptorFunc_setActionAttributes_TypeError;

    // Call function/method
    {
        if (kwds) {
            PyObject *keyName = nullptr;
            PyObject *value = nullptr;
            keyName = Py_BuildValue("s","statustip");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[1]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.setActionAttributes(): got multiple values for keyword argument 'statustip'.");
                    return {};
                }
                if (value) {
                    pyArgs[1] = value;
                    if (!(pythonToCpp[1] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[1]))))
                        goto Sbk_qt_gui_cpp_PluginDescriptorFunc_setActionAttributes_TypeError;
                }
            }
            keyName = Py_BuildValue("s","icon");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[2]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.setActionAttributes(): got multiple values for keyword argument 'icon'.");
                    return {};
                }
                if (value) {
                    pyArgs[2] = value;
                    if (!(pythonToCpp[2] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[2]))))
                        goto Sbk_qt_gui_cpp_PluginDescriptorFunc_setActionAttributes_TypeError;
                }
            }
            keyName = Py_BuildValue("s","icontype");
            if (PyDict_Contains(kwds, keyName)) {
                value = PyDict_GetItem(kwds, keyName);
                if (value && pyArgs[3]) {
                    PyErr_SetString(PyExc_TypeError, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.setActionAttributes(): got multiple values for keyword argument 'icontype'.");
                    return {};
                }
                if (value) {
                    pyArgs[3] = value;
                    if (!(pythonToCpp[3] = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyArgs[3]))))
                        goto Sbk_qt_gui_cpp_PluginDescriptorFunc_setActionAttributes_TypeError;
                }
            }
        }
        ::QString cppArg0;
        pythonToCpp[0](pyArgs[0], &cppArg0);
        ::QString cppArg1 = QString();
        if (pythonToCpp[1]) pythonToCpp[1](pyArgs[1], &cppArg1);
        ::QString cppArg2 = QString();
        if (pythonToCpp[2]) pythonToCpp[2](pyArgs[2], &cppArg2);
        ::QString cppArg3 = QString();
        if (pythonToCpp[3]) pythonToCpp[3](pyArgs[3], &cppArg3);

        if (!PyErr_Occurred()) {
            // setActionAttributes(QString,QString,QString,QString)
            cppSelf->setActionAttributes(cppArg0, cppArg1, cppArg2, cppArg3);
        }
    }

    if (PyErr_Occurred()) {
        return {};
    }
    Py_RETURN_NONE;

    Sbk_qt_gui_cpp_PluginDescriptorFunc_setActionAttributes_TypeError:
        Shiboken::setErrorAboutWrongArguments(args, "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.setActionAttributes");
        return {};
}

static PyObject *Sbk_qt_gui_cpp_PluginDescriptorFunc_toDictionary(PyObject *self)
{
    if (!Shiboken::Object::isValid(self))
        return {};
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyResult{};

    // Call function/method
    {

        if (!PyErr_Occurred()) {
            // toDictionary()const
            QMap<QString,QString > cppResult = const_cast<const ::qt_gui_cpp::PluginDescriptor *>(cppSelf)->toDictionary();
            pyResult = Shiboken::Conversions::copyToPython(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], &cppResult);
        }
    }

    if (PyErr_Occurred() || !pyResult) {
        Py_XDECREF(pyResult);
        return {};
    }
    return pyResult;
}


static const char *Sbk_qt_gui_cpp_PluginDescriptor_PropertyStrings[] = {
    nullptr // Sentinel
};

static PyMethodDef Sbk_qt_gui_cpp_PluginDescriptor_methods[] = {
    {"actionAttributes", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginDescriptorFunc_actionAttributes), METH_NOARGS},
    {"addGroupAttributes", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginDescriptorFunc_addGroupAttributes), METH_VARARGS|METH_KEYWORDS},
    {"attributes", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginDescriptorFunc_attributes), METH_NOARGS},
    {"countGroups", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginDescriptorFunc_countGroups), METH_NOARGS},
    {"group", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginDescriptorFunc_group), METH_O},
    {"pluginId", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginDescriptorFunc_pluginId), METH_NOARGS},
    {"setActionAttributes", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginDescriptorFunc_setActionAttributes), METH_VARARGS|METH_KEYWORDS},
    {"toDictionary", reinterpret_cast<PyCFunction>(Sbk_qt_gui_cpp_PluginDescriptorFunc_toDictionary), METH_NOARGS},

    {nullptr, nullptr} // Sentinel
};

static PyObject *Sbk_qt_gui_cpp_PluginDescriptor_get_plugin_id_(PyObject *self, void *)
{
    if (!Shiboken::Object::isValid(self))
        return nullptr;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyOut = {};
    pyOut = Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], &cppSelf->plugin_id_);
    return pyOut;
}
static int Sbk_qt_gui_cpp_PluginDescriptor_set_plugin_id_(PyObject *self, PyObject *pyIn, void *)
{
    if (!Shiboken::Object::isValid(self))
        return 0;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    if (pyIn == nullptr) {
        PyErr_SetString(PyExc_TypeError, "'plugin_id_' may not be deleted");
        return -1;
    }
    PythonToCppFunc pythonToCpp{nullptr};
    if (!(pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], (pyIn)))) {
        PyErr_SetString(PyExc_TypeError, "wrong type attributed to 'plugin_id_', 'QString' or convertible type expected");
        return -1;
    }

    ::QString& cppOut_ptr = cppSelf->plugin_id_;
    pythonToCpp(pyIn, &cppOut_ptr);

    return 0;
}

static PyObject *Sbk_qt_gui_cpp_PluginDescriptor_get_attributes_(PyObject *self, void *)
{
    if (!Shiboken::Object::isValid(self))
        return nullptr;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyOut = {};
    pyOut = Shiboken::Conversions::copyToPython(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], &cppSelf->attributes_);
    return pyOut;
}
static int Sbk_qt_gui_cpp_PluginDescriptor_set_attributes_(PyObject *self, PyObject *pyIn, void *)
{
    if (!Shiboken::Object::isValid(self))
        return 0;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    if (pyIn == nullptr) {
        PyErr_SetString(PyExc_TypeError, "'attributes_' may not be deleted");
        return -1;
    }
    PythonToCppFunc pythonToCpp{nullptr};
    if (!(pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], (pyIn)))) {
        PyErr_SetString(PyExc_TypeError, "wrong type attributed to 'attributes_', 'QMap' or convertible type expected");
        return -1;
    }

    ::QMap<QString,QString >& cppOut_ptr = cppSelf->attributes_;
    pythonToCpp(pyIn, &cppOut_ptr);

    return 0;
}

static PyObject *Sbk_qt_gui_cpp_PluginDescriptor_get_action_attributes_(PyObject *self, void *)
{
    if (!Shiboken::Object::isValid(self))
        return nullptr;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyOut = {};
    pyOut = Shiboken::Conversions::copyToPython(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], &cppSelf->action_attributes_);
    return pyOut;
}
static int Sbk_qt_gui_cpp_PluginDescriptor_set_action_attributes_(PyObject *self, PyObject *pyIn, void *)
{
    if (!Shiboken::Object::isValid(self))
        return 0;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    if (pyIn == nullptr) {
        PyErr_SetString(PyExc_TypeError, "'action_attributes_' may not be deleted");
        return -1;
    }
    PythonToCppFunc pythonToCpp{nullptr};
    if (!(pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], (pyIn)))) {
        PyErr_SetString(PyExc_TypeError, "wrong type attributed to 'action_attributes_', 'QMap' or convertible type expected");
        return -1;
    }

    ::QMap<QString,QString >& cppOut_ptr = cppSelf->action_attributes_;
    pythonToCpp(pyIn, &cppOut_ptr);

    return 0;
}

static PyObject *Sbk_qt_gui_cpp_PluginDescriptor_get_groups_(PyObject *self, void *)
{
    if (!Shiboken::Object::isValid(self))
        return nullptr;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    PyObject *pyOut = {};
    pyOut = Shiboken::Conversions::copyToPython(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QVECTOR_QMAP_QSTRING_QSTRING_IDX], &cppSelf->groups_);
    return pyOut;
}
static int Sbk_qt_gui_cpp_PluginDescriptor_set_groups_(PyObject *self, PyObject *pyIn, void *)
{
    if (!Shiboken::Object::isValid(self))
        return 0;
    auto cppSelf = reinterpret_cast< ::qt_gui_cpp::PluginDescriptor *>(Shiboken::Conversions::cppPointer(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], reinterpret_cast<SbkObject *>(self)));
    SBK_UNUSED(cppSelf)
    if (pyIn == nullptr) {
        PyErr_SetString(PyExc_TypeError, "'groups_' may not be deleted");
        return -1;
    }
    PythonToCppFunc pythonToCpp{nullptr};
    if (!(pythonToCpp = Shiboken::Conversions::isPythonToCppConvertible(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QVECTOR_QMAP_QSTRING_QSTRING_IDX], (pyIn)))) {
        PyErr_SetString(PyExc_TypeError, "wrong type attributed to 'groups_', 'QVector' or convertible type expected");
        return -1;
    }

    ::QVector<QMap< QString,QString > >& cppOut_ptr = cppSelf->groups_;
    pythonToCpp(pyIn, &cppOut_ptr);

    return 0;
}

// Getters and Setters for PluginDescriptor
static PyGetSetDef Sbk_qt_gui_cpp_PluginDescriptor_getsetlist[] = {
    {const_cast<char *>("plugin_id_"), Sbk_qt_gui_cpp_PluginDescriptor_get_plugin_id_, Sbk_qt_gui_cpp_PluginDescriptor_set_plugin_id_},
    {const_cast<char *>("attributes_"), Sbk_qt_gui_cpp_PluginDescriptor_get_attributes_, Sbk_qt_gui_cpp_PluginDescriptor_set_attributes_},
    {const_cast<char *>("action_attributes_"), Sbk_qt_gui_cpp_PluginDescriptor_get_action_attributes_, Sbk_qt_gui_cpp_PluginDescriptor_set_action_attributes_},
    {const_cast<char *>("groups_"), Sbk_qt_gui_cpp_PluginDescriptor_get_groups_, Sbk_qt_gui_cpp_PluginDescriptor_set_groups_},
    {nullptr} // Sentinel
};

} // extern "C"

static int Sbk_qt_gui_cpp_PluginDescriptor_traverse(PyObject *self, visitproc visit, void *arg)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_traverse(self, visit, arg);
}
static int Sbk_qt_gui_cpp_PluginDescriptor_clear(PyObject *self)
{
    return reinterpret_cast<PyTypeObject *>(SbkObject_TypeF())->tp_clear(self);
}
// Class Definition -----------------------------------------------
extern "C" {
static SbkObjectType *_Sbk_qt_gui_cpp_PluginDescriptor_Type = nullptr;
static SbkObjectType *Sbk_qt_gui_cpp_PluginDescriptor_TypeF(void)
{
    return _Sbk_qt_gui_cpp_PluginDescriptor_Type;
}

static PyType_Slot Sbk_qt_gui_cpp_PluginDescriptor_slots[] = {
    {Py_tp_base,        nullptr}, // inserted by introduceWrapperType
    {Py_tp_dealloc,     reinterpret_cast<void *>(&SbkDeallocWrapper)},
    {Py_tp_repr,        nullptr},
    {Py_tp_hash,        nullptr},
    {Py_tp_call,        nullptr},
    {Py_tp_str,         nullptr},
    {Py_tp_getattro,    nullptr},
    {Py_tp_setattro,    nullptr},
    {Py_tp_traverse,    reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginDescriptor_traverse)},
    {Py_tp_clear,       reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginDescriptor_clear)},
    {Py_tp_richcompare, nullptr},
    {Py_tp_iter,        nullptr},
    {Py_tp_iternext,    nullptr},
    {Py_tp_methods,     reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginDescriptor_methods)},
    {Py_tp_getset,      reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginDescriptor_getsetlist)},
    {Py_tp_init,        reinterpret_cast<void *>(Sbk_qt_gui_cpp_PluginDescriptor_Init)},
    {Py_tp_new,         reinterpret_cast<void *>(SbkObjectTpNew)},
    {0, nullptr}
};
static PyType_Spec Sbk_qt_gui_cpp_PluginDescriptor_spec = {
    "1:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor",
    sizeof(SbkObject),
    0,
    Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_CHECKTYPES|Py_TPFLAGS_HAVE_GC,
    Sbk_qt_gui_cpp_PluginDescriptor_slots
};

} //extern "C"


// Type conversion functions.

// Python to C++ pointer conversion - returns the C++ object of the Python wrapper (keeps object identity).
static void PluginDescriptor_PythonToCpp_PluginDescriptor_PTR(PyObject *pyIn, void *cppOut) {
    Shiboken::Conversions::pythonToCppPointer(Sbk_qt_gui_cpp_PluginDescriptor_TypeF(), pyIn, cppOut);
}
static PythonToCppFunc is_PluginDescriptor_PythonToCpp_PluginDescriptor_PTR_Convertible(PyObject *pyIn) {
    if (pyIn == Py_None)
        return Shiboken::Conversions::nonePythonToCppNullPtr;
    if (PyObject_TypeCheck(pyIn, reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_PluginDescriptor_TypeF())))
        return PluginDescriptor_PythonToCpp_PluginDescriptor_PTR;
    return {};
}

// C++ to Python pointer conversion - tries to find the Python wrapper for the C++ object (keeps object identity).
static PyObject *PluginDescriptor_PTR_CppToPython_PluginDescriptor(const void *cppIn) {
    auto pyOut = reinterpret_cast<PyObject *>(Shiboken::BindingManager::instance().retrieveWrapper(cppIn));
    if (pyOut) {
        Py_INCREF(pyOut);
        return pyOut;
    }
    bool changedTypeName = false;
    auto tCppIn = reinterpret_cast<const ::qt_gui_cpp::PluginDescriptor *>(cppIn);
    const char *typeName = typeid(*tCppIn).name();
    auto sbkType = Shiboken::ObjectType::typeForTypeName(typeName);
    if (sbkType && Shiboken::ObjectType::hasSpecialCastFunction(sbkType)) {
        typeName = typeNameOf(tCppIn);
        changedTypeName = true;
    }
    PyObject *result = Shiboken::Object::newObject(Sbk_qt_gui_cpp_PluginDescriptor_TypeF(), const_cast<void *>(cppIn), false, /* exactType */ changedTypeName, typeName);
    if (changedTypeName)
        delete [] typeName;
    return result;
}

// The signatures string for the functions.
// Multiple signatures have their index "n:" in front.
static const char *qt_gui_cpp_PluginDescriptor_SignatureStrings[] = {
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor(self,plugin_id:QString,attributes:QMap[QString, QString]=(QMap< QString,QString >()))",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.actionAttributes(self)->QMap[QString, QString]",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.addGroupAttributes(self,label:QString,statustip:QString=QString(),icon:QString=QString(),icontype:QString=QString())",
    "1:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.attributes(self)->QMap[QString, QString]",
    "0:libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.attributes(self)->QMap[QString, QString]",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.countGroups(self)->int",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.group(self,index:int)->QMap[QString, QString]",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.pluginId(self)->QString",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.setActionAttributes(self,label:QString,statustip:QString=QString(),icon:QString=QString(),icontype:QString=QString())",
    "libqt_gui_cpp_shiboken.qt_gui_cpp.PluginDescriptor.toDictionary(self)->QMap[QString, QString]",
    nullptr}; // Sentinel

void init_qt_gui_cpp_PluginDescriptor(PyObject *enclosingClass)
{
    _Sbk_qt_gui_cpp_PluginDescriptor_Type = Shiboken::ObjectType::introduceWrapperType(
        enclosingClass,
        "PluginDescriptor",
        "qt_gui_cpp::PluginDescriptor*",
        &Sbk_qt_gui_cpp_PluginDescriptor_spec,
        &Shiboken::callCppDestructor< ::qt_gui_cpp::PluginDescriptor >,
        0,
        0,
        Shiboken::ObjectType::WrapperFlags::InnerClass    );
    
    auto pyType = reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_PluginDescriptor_Type);
    InitSignatureStrings(pyType, qt_gui_cpp_PluginDescriptor_SignatureStrings);
    SbkObjectType_SetPropertyStrings(reinterpret_cast<PyTypeObject *>(_Sbk_qt_gui_cpp_PluginDescriptor_Type), Sbk_qt_gui_cpp_PluginDescriptor_PropertyStrings);
    Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX]
        = reinterpret_cast<PyTypeObject *>(Sbk_qt_gui_cpp_PluginDescriptor_TypeF());

    // Register Converter
    SbkConverter *converter = Shiboken::Conversions::createConverter(Sbk_qt_gui_cpp_PluginDescriptor_TypeF(),
        PluginDescriptor_PythonToCpp_PluginDescriptor_PTR,
        is_PluginDescriptor_PythonToCpp_PluginDescriptor_PTR_Convertible,
        PluginDescriptor_PTR_CppToPython_PluginDescriptor);

    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::PluginDescriptor");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::PluginDescriptor*");
    Shiboken::Conversions::registerConverterName(converter, "qt_gui_cpp::PluginDescriptor&");
    Shiboken::Conversions::registerConverterName(converter, "PluginDescriptor");
    Shiboken::Conversions::registerConverterName(converter, "PluginDescriptor*");
    Shiboken::Conversions::registerConverterName(converter, "PluginDescriptor&");
    Shiboken::Conversions::registerConverterName(converter, typeid(::qt_gui_cpp::PluginDescriptor).name());



}
