
#include <sbkpython.h>
#include <shiboken.h>
#include <algorithm>
#include <signature.h>
#ifndef QT_NO_VERSION_TAGGING
#  define QT_NO_VERSION_TAGGING
#endif
#include <QDebug>
#include <pyside.h>
#include <pysideqenum.h>
#include <feature_select.h>
#include <qapp_macro.h>
#include "libqt_gui_cpp_shiboken_python.h"



// Extra includes

// Current module's type array.
PyTypeObject **Sbklibqt_gui_cpp_shibokenTypes = nullptr;
// Current module's PyObject pointer.
PyObject *Sbklibqt_gui_cpp_shibokenModuleObject = nullptr;
// Current module's converter array.
SbkConverter **Sbklibqt_gui_cpp_shibokenTypeConverters = nullptr;
void cleanTypesAttributes(void) {
    if (PY_VERSION_HEX >= 0x03000000 && PY_VERSION_HEX < 0x03060000)
        return; // PYSIDE-953: testbinding crashes in Python 3.5 when hasattr touches types!
    for (int i = 0, imax = SBK_libqt_gui_cpp_shiboken_IDX_COUNT; i < imax; i++) {
        PyObject *pyType = reinterpret_cast<PyObject *>(Sbklibqt_gui_cpp_shibokenTypes[i]);
        Shiboken::AutoDecRef attrName(Py_BuildValue("s", "staticMetaObject"));
        if (pyType && PyObject_HasAttr(pyType, attrName))
            PyObject_SetAttr(pyType, attrName, Py_None);
    }
}
// Global functions ------------------------------------------------------------

static PyMethodDef libqt_gui_cpp_shiboken_methods[] = {
    {0} // Sentinel
};

// Classes initialization functions ------------------------------------------------------------
void init_libqt_gui_cpp_shibokenqt_gui_cpp(PyObject *module);
void init_qt_gui_cpp_PluginProvider(PyObject *enclosingClass);
void init_qt_gui_cpp_PluginDescriptor(PyObject *enclosingClass);
void init_qt_gui_cpp_CompositePluginProvider(PyObject *enclosingClass);
void init_qt_gui_cpp_RecursivePluginProvider(PyObject *enclosingClass);
void init_qt_gui_cpp_GenericProxy(PyObject *enclosingClass);
void init_qt_gui_cpp_Settings(PyObject *enclosingClass);
void init_qt_gui_cpp_RosPluginlibPluginProvider_ForPlugins(PyObject *enclosingClass);
void init_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders(PyObject *enclosingClass);
void init_qt_gui_cpp_PluginContext(PyObject *enclosingClass);
void init_qt_gui_cpp_PluginBridge(PyObject *enclosingClass);
void init_qt_gui_cpp_Plugin(PyObject *enclosingClass);

// Required modules' type and converter arrays.
PyTypeObject **SbkPySide2_QtCoreTypes;
SbkConverter **SbkPySide2_QtCoreTypeConverters;
PyTypeObject **SbkPySide2_QtGuiTypes;
SbkConverter **SbkPySide2_QtGuiTypeConverters;
PyTypeObject **SbkPySide2_QtWidgetsTypes;
SbkConverter **SbkPySide2_QtWidgetsTypeConverters;

// Module initialization ------------------------------------------------------------
// Container Type converters.

// C++ to Python conversion for type 'const QList<qt_gui_cpp::PluginProvider* > &'.
static PyObject *_constQList_qt_gui_cpp_PluginProviderPTR_REF_CppToPython__constQList_qt_gui_cpp_PluginProviderPTR_REF(const void *cppIn) {
    auto &cppInRef = *reinterpret_cast<::QList<qt_gui_cpp::PluginProvider* > *>(const_cast<void *>(cppIn));
    // TEMPLATE - cpplist_to_pylist_conversion - START
    PyObject* pyOut = PyList_New((int) cppInRef.size());
    ::QList<qt_gui_cpp::PluginProvider* >::const_iterator it = cppInRef.begin();
    for (int idx = 0; it != cppInRef.end(); ++it, ++idx) {
        ::qt_gui_cpp::PluginProvider* cppItem(*it);
        PyList_SET_ITEM(pyOut, idx, Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX]), cppItem));
    }
    return pyOut;
    // TEMPLATE - cpplist_to_pylist_conversion - END

}
static void _constQList_qt_gui_cpp_PluginProviderPTR_REF_PythonToCpp__constQList_qt_gui_cpp_PluginProviderPTR_REF(PyObject *pyIn, void *cppOut) {
    auto &cppOutRef = *reinterpret_cast<::QList<qt_gui_cpp::PluginProvider* > *>(cppOut);
    // TEMPLATE - pyseq_to_cpplist_conversion - START
    // PYSIDE-795: Turn all sequences into iterables.
    Shiboken::AutoDecRef it(PyObject_GetIter(pyIn));
    PyObject *(*iternext)(PyObject *) = *Py_TYPE(it)->tp_iternext;
    for (;;) {
        Shiboken::AutoDecRef pyItem(iternext(it));
        if (pyItem.isNull()) {
            if (PyErr_Occurred() && PyErr_ExceptionMatches(PyExc_StopIteration))
                PyErr_Clear();
            break;
        }
        ::qt_gui_cpp::PluginProvider* cppItem{nullptr};
        Shiboken::Conversions::pythonToCppPointer(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX]), pyItem, &(cppItem));
        cppOutRef << cppItem;
    }
    // TEMPLATE - pyseq_to_cpplist_conversion - END

}
static PythonToCppFunc is__constQList_qt_gui_cpp_PluginProviderPTR_REF_PythonToCpp__constQList_qt_gui_cpp_PluginProviderPTR_REF_Convertible(PyObject *pyIn) {
    if (Shiboken::Conversions::checkSequenceTypes(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX], pyIn))
        return _constQList_qt_gui_cpp_PluginProviderPTR_REF_PythonToCpp__constQList_qt_gui_cpp_PluginProviderPTR_REF;
    return {};
}

// C++ to Python conversion for type 'QMap<QString,QString >'.
static PyObject *_QMap_QString_QString__CppToPython__QMap_QString_QString_(const void *cppIn) {
    auto &cppInRef = *reinterpret_cast<::QMap<QString,QString > *>(const_cast<void *>(cppIn));
    // TEMPLATE - cppmap_to_pymap_conversion - START
    PyObject *pyOut = PyDict_New();
    for (::QMap<QString,QString >::const_iterator it = cppInRef.begin(), end = cppInRef.end(); it != end; ++it) {
        ::QString key = it.key();
        ::QString value = it.value();
        PyObject *pyKey = Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], &key);
        PyObject *pyValue = Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], &value);
        PyDict_SetItem(pyOut, pyKey, pyValue);
        Py_DECREF(pyKey);
        Py_DECREF(pyValue);
    }
    return pyOut;
    // TEMPLATE - cppmap_to_pymap_conversion - END

}
static void _QMap_QString_QString__PythonToCpp__QMap_QString_QString_(PyObject *pyIn, void *cppOut) {
    auto &cppOutRef = *reinterpret_cast<::QMap<QString,QString > *>(cppOut);
    // TEMPLATE - pydict_to_cppmap_conversion - START
    PyObject *key;
    PyObject *value;
    Py_ssize_t pos = 0;
    while (PyDict_Next(pyIn, &pos, &key, &value)) {
        ::QString cppKey;
        Shiboken::Conversions::pythonToCppCopy(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], key, &(cppKey));
        ::QString cppValue;
        Shiboken::Conversions::pythonToCppCopy(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], value, &(cppValue));
        cppOutRef.insert(cppKey, cppValue);
    }
    // TEMPLATE - pydict_to_cppmap_conversion - END

}
static PythonToCppFunc is__QMap_QString_QString__PythonToCpp__QMap_QString_QString__Convertible(PyObject *pyIn) {
    if (Shiboken::Conversions::convertibleDictTypes(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], false, SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], false, pyIn))
        return _QMap_QString_QString__PythonToCpp__QMap_QString_QString_;
    return {};
}

// C++ to Python conversion for type 'QList<qt_gui_cpp::PluginDescriptor* >'.
static PyObject *_QList_qt_gui_cpp_PluginDescriptorPTR__CppToPython__QList_qt_gui_cpp_PluginDescriptorPTR_(const void *cppIn) {
    auto &cppInRef = *reinterpret_cast<::QList<qt_gui_cpp::PluginDescriptor* > *>(const_cast<void *>(cppIn));
    // TEMPLATE - cpplist_to_pylist_conversion - START
    PyObject* pyOut = PyList_New((int) cppInRef.size());
    ::QList<qt_gui_cpp::PluginDescriptor* >::const_iterator it = cppInRef.begin();
    for (int idx = 0; it != cppInRef.end(); ++it, ++idx) {
        ::qt_gui_cpp::PluginDescriptor* cppItem(*it);
        PyList_SET_ITEM(pyOut, idx, Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX]), cppItem));
    }
    return pyOut;
    // TEMPLATE - cpplist_to_pylist_conversion - END

}
static void _QList_qt_gui_cpp_PluginDescriptorPTR__PythonToCpp__QList_qt_gui_cpp_PluginDescriptorPTR_(PyObject *pyIn, void *cppOut) {
    auto &cppOutRef = *reinterpret_cast<::QList<qt_gui_cpp::PluginDescriptor* > *>(cppOut);
    // TEMPLATE - pyseq_to_cpplist_conversion - START
    // PYSIDE-795: Turn all sequences into iterables.
    Shiboken::AutoDecRef it(PyObject_GetIter(pyIn));
    PyObject *(*iternext)(PyObject *) = *Py_TYPE(it)->tp_iternext;
    for (;;) {
        Shiboken::AutoDecRef pyItem(iternext(it));
        if (pyItem.isNull()) {
            if (PyErr_Occurred() && PyErr_ExceptionMatches(PyExc_StopIteration))
                PyErr_Clear();
            break;
        }
        ::qt_gui_cpp::PluginDescriptor* cppItem{nullptr};
        Shiboken::Conversions::pythonToCppPointer(reinterpret_cast<SbkObjectType *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX]), pyItem, &(cppItem));
        cppOutRef << cppItem;
    }
    // TEMPLATE - pyseq_to_cpplist_conversion - END

}
static PythonToCppFunc is__QList_qt_gui_cpp_PluginDescriptorPTR__PythonToCpp__QList_qt_gui_cpp_PluginDescriptorPTR__Convertible(PyObject *pyIn) {
    if (Shiboken::Conversions::checkSequenceTypes(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX], pyIn))
        return _QList_qt_gui_cpp_PluginDescriptorPTR__PythonToCpp__QList_qt_gui_cpp_PluginDescriptorPTR_;
    return {};
}

// C++ to Python conversion for type 'const QList<QObject* > &'.
static PyObject *_constQList_QObjectPTR_REF_CppToPython__constQList_QObjectPTR_REF(const void *cppIn) {
    auto &cppInRef = *reinterpret_cast<::QList<QObject* > *>(const_cast<void *>(cppIn));
    // TEMPLATE - cpplist_to_pylist_conversion - START
    PyObject* pyOut = PyList_New((int) cppInRef.size());
    ::QList<QObject* >::const_iterator it = cppInRef.begin();
    for (int idx = 0; it != cppInRef.end(); ++it, ++idx) {
        ::QObject* cppItem(*it);
        PyList_SET_ITEM(pyOut, idx, Shiboken::Conversions::pointerToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), cppItem));
    }
    return pyOut;
    // TEMPLATE - cpplist_to_pylist_conversion - END

}
static void _constQList_QObjectPTR_REF_PythonToCpp__constQList_QObjectPTR_REF(PyObject *pyIn, void *cppOut) {
    auto &cppOutRef = *reinterpret_cast<::QList<QObject* > *>(cppOut);
    // TEMPLATE - pyseq_to_cpplist_conversion - START
    // PYSIDE-795: Turn all sequences into iterables.
    Shiboken::AutoDecRef it(PyObject_GetIter(pyIn));
    PyObject *(*iternext)(PyObject *) = *Py_TYPE(it)->tp_iternext;
    for (;;) {
        Shiboken::AutoDecRef pyItem(iternext(it));
        if (pyItem.isNull()) {
            if (PyErr_Occurred() && PyErr_ExceptionMatches(PyExc_StopIteration))
                PyErr_Clear();
            break;
        }
        ::QObject* cppItem{nullptr};
        Shiboken::Conversions::pythonToCppPointer(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX]), pyItem, &(cppItem));
        cppOutRef << cppItem;
    }
    // TEMPLATE - pyseq_to_cpplist_conversion - END

}
static PythonToCppFunc is__constQList_QObjectPTR_REF_PythonToCpp__constQList_QObjectPTR_REF_Convertible(PyObject *pyIn) {
    if (Shiboken::Conversions::checkSequenceTypes(SbkPySide2_QtCoreTypes[SBK_QOBJECT_IDX], pyIn))
        return _constQList_QObjectPTR_REF_PythonToCpp__constQList_QObjectPTR_REF;
    return {};
}

// C++ to Python conversion for type 'QList<QByteArray >'.
static PyObject *_QList_QByteArray__CppToPython__QList_QByteArray_(const void *cppIn) {
    auto &cppInRef = *reinterpret_cast<::QList<QByteArray > *>(const_cast<void *>(cppIn));
    // TEMPLATE - cpplist_to_pylist_conversion - START
    PyObject* pyOut = PyList_New((int) cppInRef.size());
    ::QList<QByteArray >::const_iterator it = cppInRef.begin();
    for (int idx = 0; it != cppInRef.end(); ++it, ++idx) {
        ::QByteArray cppItem(*it);
        PyList_SET_ITEM(pyOut, idx, Shiboken::Conversions::copyToPython(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QBYTEARRAY_IDX]), &cppItem));
    }
    return pyOut;
    // TEMPLATE - cpplist_to_pylist_conversion - END

}
static void _QList_QByteArray__PythonToCpp__QList_QByteArray_(PyObject *pyIn, void *cppOut) {
    auto &cppOutRef = *reinterpret_cast<::QList<QByteArray > *>(cppOut);
    // TEMPLATE - pyseq_to_cpplist_conversion - START
    // PYSIDE-795: Turn all sequences into iterables.
    Shiboken::AutoDecRef it(PyObject_GetIter(pyIn));
    PyObject *(*iternext)(PyObject *) = *Py_TYPE(it)->tp_iternext;
    for (;;) {
        Shiboken::AutoDecRef pyItem(iternext(it));
        if (pyItem.isNull()) {
            if (PyErr_Occurred() && PyErr_ExceptionMatches(PyExc_StopIteration))
                PyErr_Clear();
            break;
        }
        ::QByteArray cppItem;
        Shiboken::Conversions::pythonToCppCopy(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QBYTEARRAY_IDX]), pyItem, &(cppItem));
        cppOutRef << cppItem;
    }
    // TEMPLATE - pyseq_to_cpplist_conversion - END

}
static PythonToCppFunc is__QList_QByteArray__PythonToCpp__QList_QByteArray__Convertible(PyObject *pyIn) {
    if (Shiboken::Conversions::convertibleSequenceTypes(reinterpret_cast<SbkObjectType *>(SbkPySide2_QtCoreTypes[SBK_QBYTEARRAY_IDX]), pyIn))
        return _QList_QByteArray__PythonToCpp__QList_QByteArray_;
    return {};
}

// C++ to Python conversion for type 'QVector<QMap< QString,QString > >'.
static PyObject *_QVector_QMap_QString_QString___CppToPython__QVector_QMap_QString_QString__(const void *cppIn) {
    auto &cppInRef = *reinterpret_cast<::QVector<QMap< QString,QString > > *>(const_cast<void *>(cppIn));
    // TEMPLATE - cppvector_to_pylist_conversion - START
    ::QVector<QMap< QString,QString > >::size_type vectorSize = cppInRef.size();
    PyObject* pyOut = PyList_New((int) vectorSize);
    for (::QVector<QMap< QString,QString > >::size_type idx = 0; idx < vectorSize; ++idx) {
        ::QMap<QString,QString > cppItem(cppInRef[idx]);
        PyList_SET_ITEM(pyOut, idx, Shiboken::Conversions::copyToPython(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], &cppItem));
    }
    return pyOut;
    // TEMPLATE - cppvector_to_pylist_conversion - END

}
static void _QVector_QMap_QString_QString___PythonToCpp__QVector_QMap_QString_QString__(PyObject *pyIn, void *cppOut) {
    auto &cppOutRef = *reinterpret_cast<::QVector<QMap< QString,QString > > *>(cppOut);
    // TEMPLATE - pyseq_to_cppvector_conversion - START
    // PYSIDE-795: Turn all sequences into iterables.
    if (PySequence_Check(pyIn)) {
        int vectorSize = PySequence_Size(pyIn);
        cppOutRef.reserve(vectorSize);
    }
    Shiboken::AutoDecRef it(PyObject_GetIter(pyIn));
    PyObject *(*iternext)(PyObject *) = *Py_TYPE(it)->tp_iternext;
    for (;;) {
        Shiboken::AutoDecRef pyItem(iternext(it));
        if (pyItem.isNull()) {
            if (PyErr_Occurred() && PyErr_ExceptionMatches(PyExc_StopIteration))
                PyErr_Clear();
            break;
        }
        ::QMap<QString,QString > cppItem;
        Shiboken::Conversions::pythonToCppCopy(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], pyItem, &(cppItem));
        cppOutRef.push_back(cppItem);
    }
    // TEMPLATE - pyseq_to_cppvector_conversion - END

}
static PythonToCppFunc is__QVector_QMap_QString_QString___PythonToCpp__QVector_QMap_QString_QString___Convertible(PyObject *pyIn) {
    if (Shiboken::Conversions::convertibleSequenceTypes(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], pyIn))
        return _QVector_QMap_QString_QString___PythonToCpp__QVector_QMap_QString_QString__;
    return {};
}

// C++ to Python conversion for type 'QList<QVariant >'.
static PyObject *_QList_QVariant__CppToPython__QList_QVariant_(const void *cppIn) {
    auto &cppInRef = *reinterpret_cast<::QList<QVariant > *>(const_cast<void *>(cppIn));
    // TEMPLATE - cpplist_to_pylist_conversion - START
    PyObject* pyOut = PyList_New((int) cppInRef.size());
    ::QList<QVariant >::const_iterator it = cppInRef.begin();
    for (int idx = 0; it != cppInRef.end(); ++it, ++idx) {
        ::QVariant cppItem(*it);
        PyList_SET_ITEM(pyOut, idx, Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QVARIANT_IDX], &cppItem));
    }
    return pyOut;
    // TEMPLATE - cpplist_to_pylist_conversion - END

}
static void _QList_QVariant__PythonToCpp__QList_QVariant_(PyObject *pyIn, void *cppOut) {
    auto &cppOutRef = *reinterpret_cast<::QList<QVariant > *>(cppOut);
    // TEMPLATE - pyseq_to_cpplist_conversion - START
    // PYSIDE-795: Turn all sequences into iterables.
    Shiboken::AutoDecRef it(PyObject_GetIter(pyIn));
    PyObject *(*iternext)(PyObject *) = *Py_TYPE(it)->tp_iternext;
    for (;;) {
        Shiboken::AutoDecRef pyItem(iternext(it));
        if (pyItem.isNull()) {
            if (PyErr_Occurred() && PyErr_ExceptionMatches(PyExc_StopIteration))
                PyErr_Clear();
            break;
        }
        ::QVariant cppItem;
        Shiboken::Conversions::pythonToCppCopy(SbkPySide2_QtCoreTypeConverters[SBK_QVARIANT_IDX], pyItem, &(cppItem));
        cppOutRef << cppItem;
    }
    // TEMPLATE - pyseq_to_cpplist_conversion - END

}
static PythonToCppFunc is__QList_QVariant__PythonToCpp__QList_QVariant__Convertible(PyObject *pyIn) {
    if (Shiboken::Conversions::convertibleSequenceTypes(SbkPySide2_QtCoreTypeConverters[SBK_QVARIANT_IDX], pyIn))
        return _QList_QVariant__PythonToCpp__QList_QVariant_;
    return {};
}

// C++ to Python conversion for type 'QList<QString >'.
static PyObject *_QList_QString__CppToPython__QList_QString_(const void *cppIn) {
    auto &cppInRef = *reinterpret_cast<::QList<QString > *>(const_cast<void *>(cppIn));
    // TEMPLATE - cpplist_to_pylist_conversion - START
    PyObject* pyOut = PyList_New((int) cppInRef.size());
    ::QList<QString >::const_iterator it = cppInRef.begin();
    for (int idx = 0; it != cppInRef.end(); ++it, ++idx) {
        ::QString cppItem(*it);
        PyList_SET_ITEM(pyOut, idx, Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], &cppItem));
    }
    return pyOut;
    // TEMPLATE - cpplist_to_pylist_conversion - END

}
static void _QList_QString__PythonToCpp__QList_QString_(PyObject *pyIn, void *cppOut) {
    auto &cppOutRef = *reinterpret_cast<::QList<QString > *>(cppOut);
    // TEMPLATE - pyseq_to_cpplist_conversion - START
    // PYSIDE-795: Turn all sequences into iterables.
    Shiboken::AutoDecRef it(PyObject_GetIter(pyIn));
    PyObject *(*iternext)(PyObject *) = *Py_TYPE(it)->tp_iternext;
    for (;;) {
        Shiboken::AutoDecRef pyItem(iternext(it));
        if (pyItem.isNull()) {
            if (PyErr_Occurred() && PyErr_ExceptionMatches(PyExc_StopIteration))
                PyErr_Clear();
            break;
        }
        ::QString cppItem;
        Shiboken::Conversions::pythonToCppCopy(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], pyItem, &(cppItem));
        cppOutRef << cppItem;
    }
    // TEMPLATE - pyseq_to_cpplist_conversion - END

}
static PythonToCppFunc is__QList_QString__PythonToCpp__QList_QString__Convertible(PyObject *pyIn) {
    if (Shiboken::Conversions::convertibleSequenceTypes(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], pyIn))
        return _QList_QString__PythonToCpp__QList_QString_;
    return {};
}

// C++ to Python conversion for type 'QMap<QString,QVariant >'.
static PyObject *_QMap_QString_QVariant__CppToPython__QMap_QString_QVariant_(const void *cppIn) {
    auto &cppInRef = *reinterpret_cast<::QMap<QString,QVariant > *>(const_cast<void *>(cppIn));
    // TEMPLATE - cppmap_to_pymap_conversion - START
    PyObject *pyOut = PyDict_New();
    for (::QMap<QString,QVariant >::const_iterator it = cppInRef.begin(), end = cppInRef.end(); it != end; ++it) {
        ::QString key = it.key();
        ::QVariant value = it.value();
        PyObject *pyKey = Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], &key);
        PyObject *pyValue = Shiboken::Conversions::copyToPython(SbkPySide2_QtCoreTypeConverters[SBK_QVARIANT_IDX], &value);
        PyDict_SetItem(pyOut, pyKey, pyValue);
        Py_DECREF(pyKey);
        Py_DECREF(pyValue);
    }
    return pyOut;
    // TEMPLATE - cppmap_to_pymap_conversion - END

}
static void _QMap_QString_QVariant__PythonToCpp__QMap_QString_QVariant_(PyObject *pyIn, void *cppOut) {
    auto &cppOutRef = *reinterpret_cast<::QMap<QString,QVariant > *>(cppOut);
    // TEMPLATE - pydict_to_cppmap_conversion - START
    PyObject *key;
    PyObject *value;
    Py_ssize_t pos = 0;
    while (PyDict_Next(pyIn, &pos, &key, &value)) {
        ::QString cppKey;
        Shiboken::Conversions::pythonToCppCopy(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], key, &(cppKey));
        ::QVariant cppValue;
        Shiboken::Conversions::pythonToCppCopy(SbkPySide2_QtCoreTypeConverters[SBK_QVARIANT_IDX], value, &(cppValue));
        cppOutRef.insert(cppKey, cppValue);
    }
    // TEMPLATE - pydict_to_cppmap_conversion - END

}
static PythonToCppFunc is__QMap_QString_QVariant__PythonToCpp__QMap_QString_QVariant__Convertible(PyObject *pyIn) {
    if (Shiboken::Conversions::convertibleDictTypes(SbkPySide2_QtCoreTypeConverters[SBK_QSTRING_IDX], false, SbkPySide2_QtCoreTypeConverters[SBK_QVARIANT_IDX], false, pyIn))
        return _QMap_QString_QVariant__PythonToCpp__QMap_QString_QVariant_;
    return {};
}


#ifdef IS_PY3K
static struct PyModuleDef moduledef = {
    /* m_base     */ PyModuleDef_HEAD_INIT,
    /* m_name     */ "libqt_gui_cpp_shiboken",
    /* m_doc      */ nullptr,
    /* m_size     */ -1,
    /* m_methods  */ libqt_gui_cpp_shiboken_methods,
    /* m_reload   */ nullptr,
    /* m_traverse */ nullptr,
    /* m_clear    */ nullptr,
    /* m_free     */ nullptr
};

#endif

// The signatures string for the global functions.
// Multiple signatures have their index "n:" in front.
static const char *libqt_gui_cpp_shiboken_SignatureStrings[] = {
    nullptr}; // Sentinel

SBK_MODULE_INIT_FUNCTION_BEGIN(libqt_gui_cpp_shiboken)
    {
        Shiboken::AutoDecRef requiredModule(Shiboken::Module::import("PySide2.QtCore"));
        if (requiredModule.isNull())
            return SBK_MODULE_INIT_ERROR;
        SbkPySide2_QtCoreTypes = Shiboken::Module::getTypes(requiredModule);
        SbkPySide2_QtCoreTypeConverters = Shiboken::Module::getTypeConverters(requiredModule);
    }

    {
        Shiboken::AutoDecRef requiredModule(Shiboken::Module::import("PySide2.QtGui"));
        if (requiredModule.isNull())
            return SBK_MODULE_INIT_ERROR;
        SbkPySide2_QtGuiTypes = Shiboken::Module::getTypes(requiredModule);
        SbkPySide2_QtGuiTypeConverters = Shiboken::Module::getTypeConverters(requiredModule);
    }

    {
        Shiboken::AutoDecRef requiredModule(Shiboken::Module::import("PySide2.QtWidgets"));
        if (requiredModule.isNull())
            return SBK_MODULE_INIT_ERROR;
        SbkPySide2_QtWidgetsTypes = Shiboken::Module::getTypes(requiredModule);
        SbkPySide2_QtWidgetsTypeConverters = Shiboken::Module::getTypeConverters(requiredModule);
    }

    // Create an array of wrapper types for the current module.
    static PyTypeObject *cppApi[SBK_libqt_gui_cpp_shiboken_IDX_COUNT];
    Sbklibqt_gui_cpp_shibokenTypes = cppApi;

    // Create an array of primitive type converters for the current module.
    static SbkConverter *sbkConverters[SBK_libqt_gui_cpp_shiboken_CONVERTERS_IDX_COUNT];
    Sbklibqt_gui_cpp_shibokenTypeConverters = sbkConverters;

#ifdef IS_PY3K
    PyObject *module = Shiboken::Module::create("libqt_gui_cpp_shiboken", &moduledef);
#else
    PyObject *module = Shiboken::Module::create("libqt_gui_cpp_shiboken", libqt_gui_cpp_shiboken_methods);
#endif

    // Make module available from global scope
    Sbklibqt_gui_cpp_shibokenModuleObject = module;

    // Initialize classes in the type system
    init_libqt_gui_cpp_shibokenqt_gui_cpp(module);
    init_qt_gui_cpp_PluginProvider(reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_libqt_gui_cpp_shibokenQT_GUI_CPP_IDX])->tp_dict);
    init_qt_gui_cpp_PluginDescriptor(reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_libqt_gui_cpp_shibokenQT_GUI_CPP_IDX])->tp_dict);
    init_qt_gui_cpp_CompositePluginProvider(reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_libqt_gui_cpp_shibokenQT_GUI_CPP_IDX])->tp_dict);
    init_qt_gui_cpp_RecursivePluginProvider(reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_libqt_gui_cpp_shibokenQT_GUI_CPP_IDX])->tp_dict);
    init_qt_gui_cpp_GenericProxy(reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_libqt_gui_cpp_shibokenQT_GUI_CPP_IDX])->tp_dict);
    init_qt_gui_cpp_Settings(reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_libqt_gui_cpp_shibokenQT_GUI_CPP_IDX])->tp_dict);
    init_qt_gui_cpp_RosPluginlibPluginProvider_ForPlugins(reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_libqt_gui_cpp_shibokenQT_GUI_CPP_IDX])->tp_dict);
    init_qt_gui_cpp_RosPluginlibPluginProvider_ForPluginProviders(reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_libqt_gui_cpp_shibokenQT_GUI_CPP_IDX])->tp_dict);
    init_qt_gui_cpp_PluginContext(reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_libqt_gui_cpp_shibokenQT_GUI_CPP_IDX])->tp_dict);
    init_qt_gui_cpp_PluginBridge(reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_libqt_gui_cpp_shibokenQT_GUI_CPP_IDX])->tp_dict);
    init_qt_gui_cpp_Plugin(reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_libqt_gui_cpp_shibokenQT_GUI_CPP_IDX])->tp_dict);

    // Register converter for type 'const QList<qt_gui_cpp::PluginProvider*>&'.
    Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QT_GUI_CPP_PLUGINPROVIDERPTR_IDX] = Shiboken::Conversions::createConverter(&PyList_Type, _constQList_qt_gui_cpp_PluginProviderPTR_REF_CppToPython__constQList_qt_gui_cpp_PluginProviderPTR_REF);
    Shiboken::Conversions::registerConverterName(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QT_GUI_CPP_PLUGINPROVIDERPTR_IDX], "const QList<qt_gui_cpp::PluginProvider*>&");
    Shiboken::Conversions::registerConverterName(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QT_GUI_CPP_PLUGINPROVIDERPTR_IDX], "QList<qt_gui_cpp::PluginProvider*>");
    Shiboken::Conversions::addPythonToCppValueConversion(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QT_GUI_CPP_PLUGINPROVIDERPTR_IDX],
        _constQList_qt_gui_cpp_PluginProviderPTR_REF_PythonToCpp__constQList_qt_gui_cpp_PluginProviderPTR_REF,
        is__constQList_qt_gui_cpp_PluginProviderPTR_REF_PythonToCpp__constQList_qt_gui_cpp_PluginProviderPTR_REF_Convertible);

    // Register converter for type 'QMap<QString,QString>'.
    Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX] = Shiboken::Conversions::createConverter(&PyDict_Type, _QMap_QString_QString__CppToPython__QMap_QString_QString_);
    Shiboken::Conversions::registerConverterName(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX], "QMap<QString,QString>");
    Shiboken::Conversions::addPythonToCppValueConversion(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX],
        _QMap_QString_QString__PythonToCpp__QMap_QString_QString_,
        is__QMap_QString_QString__PythonToCpp__QMap_QString_QString__Convertible);

    // Register converter for type 'QList<qt_gui_cpp::PluginDescriptor*>'.
    Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QT_GUI_CPP_PLUGINDESCRIPTORPTR_IDX] = Shiboken::Conversions::createConverter(&PyList_Type, _QList_qt_gui_cpp_PluginDescriptorPTR__CppToPython__QList_qt_gui_cpp_PluginDescriptorPTR_);
    Shiboken::Conversions::registerConverterName(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QT_GUI_CPP_PLUGINDESCRIPTORPTR_IDX], "QList<qt_gui_cpp::PluginDescriptor*>");
    Shiboken::Conversions::addPythonToCppValueConversion(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QT_GUI_CPP_PLUGINDESCRIPTORPTR_IDX],
        _QList_qt_gui_cpp_PluginDescriptorPTR__PythonToCpp__QList_qt_gui_cpp_PluginDescriptorPTR_,
        is__QList_qt_gui_cpp_PluginDescriptorPTR__PythonToCpp__QList_qt_gui_cpp_PluginDescriptorPTR__Convertible);

    // Register converter for type 'const QList<QObject*>&'.
    Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QOBJECTPTR_IDX] = Shiboken::Conversions::createConverter(&PyList_Type, _constQList_QObjectPTR_REF_CppToPython__constQList_QObjectPTR_REF);
    Shiboken::Conversions::registerConverterName(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QOBJECTPTR_IDX], "const QList<QObject*>&");
    Shiboken::Conversions::registerConverterName(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QOBJECTPTR_IDX], "QList<QObject*>");
    Shiboken::Conversions::addPythonToCppValueConversion(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QOBJECTPTR_IDX],
        _constQList_QObjectPTR_REF_PythonToCpp__constQList_QObjectPTR_REF,
        is__constQList_QObjectPTR_REF_PythonToCpp__constQList_QObjectPTR_REF_Convertible);

    // Register converter for type 'QList<QByteArray>'.
    Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QBYTEARRAY_IDX] = Shiboken::Conversions::createConverter(&PyList_Type, _QList_QByteArray__CppToPython__QList_QByteArray_);
    Shiboken::Conversions::registerConverterName(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QBYTEARRAY_IDX], "QList<QByteArray>");
    Shiboken::Conversions::addPythonToCppValueConversion(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QBYTEARRAY_IDX],
        _QList_QByteArray__PythonToCpp__QList_QByteArray_,
        is__QList_QByteArray__PythonToCpp__QList_QByteArray__Convertible);

    // Register converter for type 'QVector<QMap<QString,QString>>'.
    Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QVECTOR_QMAP_QSTRING_QSTRING_IDX] = Shiboken::Conversions::createConverter(&PyList_Type, _QVector_QMap_QString_QString___CppToPython__QVector_QMap_QString_QString__);
    Shiboken::Conversions::registerConverterName(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QVECTOR_QMAP_QSTRING_QSTRING_IDX], "QVector<QMap<QString,QString>>");
    Shiboken::Conversions::addPythonToCppValueConversion(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QVECTOR_QMAP_QSTRING_QSTRING_IDX],
        _QVector_QMap_QString_QString___PythonToCpp__QVector_QMap_QString_QString__,
        is__QVector_QMap_QString_QString___PythonToCpp__QVector_QMap_QString_QString___Convertible);

    // Register converter for type 'QList<QVariant>'.
    Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QVARIANT_IDX] = Shiboken::Conversions::createConverter(&PyList_Type, _QList_QVariant__CppToPython__QList_QVariant_);
    Shiboken::Conversions::registerConverterName(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QVARIANT_IDX], "QList<QVariant>");
    Shiboken::Conversions::addPythonToCppValueConversion(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QVARIANT_IDX],
        _QList_QVariant__PythonToCpp__QList_QVariant_,
        is__QList_QVariant__PythonToCpp__QList_QVariant__Convertible);

    // Register converter for type 'QList<QString>'.
    Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QSTRING_IDX] = Shiboken::Conversions::createConverter(&PyList_Type, _QList_QString__CppToPython__QList_QString_);
    Shiboken::Conversions::registerConverterName(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QSTRING_IDX], "QList<QString>");
    Shiboken::Conversions::addPythonToCppValueConversion(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QSTRING_IDX],
        _QList_QString__PythonToCpp__QList_QString_,
        is__QList_QString__PythonToCpp__QList_QString__Convertible);

    // Register converter for type 'QMap<QString,QVariant>'.
    Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QVARIANT_IDX] = Shiboken::Conversions::createConverter(&PyDict_Type, _QMap_QString_QVariant__CppToPython__QMap_QString_QVariant_);
    Shiboken::Conversions::registerConverterName(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QVARIANT_IDX], "QMap<QString,QVariant>");
    Shiboken::Conversions::addPythonToCppValueConversion(Sbklibqt_gui_cpp_shibokenTypeConverters[SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QVARIANT_IDX],
        _QMap_QString_QVariant__PythonToCpp__QMap_QString_QVariant_,
        is__QMap_QString_QVariant__PythonToCpp__QMap_QString_QVariant__Convertible);

    // Register primitive types converters.

    Shiboken::Module::registerTypes(module, Sbklibqt_gui_cpp_shibokenTypes);
    Shiboken::Module::registerTypeConverters(module, Sbklibqt_gui_cpp_shibokenTypeConverters);

    if (PyErr_Occurred()) {
        PyErr_Print();
        Py_FatalError("can't initialize module libqt_gui_cpp_shiboken");
    }
    PySide::registerCleanupFunction(cleanTypesAttributes);

    FinishSignatureInitialization(module, libqt_gui_cpp_shiboken_SignatureStrings);

SBK_MODULE_INIT_FUNCTION_END
