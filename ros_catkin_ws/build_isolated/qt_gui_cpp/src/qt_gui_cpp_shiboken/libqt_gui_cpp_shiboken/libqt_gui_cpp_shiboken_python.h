

#ifndef SBK_LIBQT_GUI_CPP_SHIBOKEN_PYTHON_H
#define SBK_LIBQT_GUI_CPP_SHIBOKEN_PYTHON_H

//workaround to access protected functions
#define protected public

#include <sbkpython.h>
#include <sbkconverter.h>
// Module Includes
#include <pyside2_qtcore_python.h>
#include <pyside2_qtgui_python.h>
#include <pyside2_qtwidgets_python.h>

// Bound library includes
#include <qt_gui_cpp/composite_plugin_provider.h>
#include <qt_gui_cpp/settings.h>
#include <qt_gui_cpp/plugin.h>
#include <qt_gui_cpp/recursive_plugin_provider.h>
#include <qt_gui_cpp/plugin_bridge.h>
#include <qt_gui_cpp/generic_proxy.h>
#include <qt_gui_cpp/plugin_descriptor.h>
#include <qt_gui_cpp/plugin_context.h>
#include <qt_gui_cpp/plugin_provider.h>
#include <qt_gui_cpp/ros_pluginlib_plugin_provider_for_plugins.h>
#include <qt_gui_cpp/ros_pluginlib_plugin_provider_for_plugin_providers.h>
// Conversion Includes - Primitive Types
#include <qabstractitemmodel.h>
#include <QString>
#include <QStringList>
#include <signalmanager.h>

// Conversion Includes - Container Types
#include <pysideqflags.h>
#include <QLinkedList>
#include <QList>
#include <QMap>
#include <QMultiMap>
#include <QPair>
#include <QQueue>
#include <QSet>
#include <QStack>
#include <QVector>

// Type indices
enum : int {
    SBK_libqt_gui_cpp_shibokenQT_GUI_CPP_IDX                 = 0,
    SBK_QT_GUI_CPP_COMPOSITEPLUGINPROVIDER_IDX               = 1,
    SBK_QT_GUI_CPP_GENERICPROXY_IDX                          = 2,
    SBK_QT_GUI_CPP_PLUGIN_IDX                                = 3,
    SBK_QT_GUI_CPP_PLUGINBRIDGE_IDX                          = 4,
    SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX                         = 5,
    SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX                      = 6,
    SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX                        = 7,
    SBK_QT_GUI_CPP_RECURSIVEPLUGINPROVIDER_IDX               = 8,
    SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINPROVIDERS_IDX = 9,
    SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_QT_GUI_CPP_PLUGINPROVIDER_IDX = 9,
    SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINS_IDX = 10,
    SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_QT_GUI_CPP_PLUGIN_IDX = 10,
    SBK_QT_GUI_CPP_SETTINGS_IDX                              = 11,
    SBK_libqt_gui_cpp_shiboken_IDX_COUNT                     = 12
};
// This variable stores all Python types exported by this module.
extern PyTypeObject **Sbklibqt_gui_cpp_shibokenTypes;

// This variable stores the Python module object exported by this module.
extern PyObject *Sbklibqt_gui_cpp_shibokenModuleObject;

// This variable stores all type converters exported by this module.
extern SbkConverter **Sbklibqt_gui_cpp_shibokenTypeConverters;

// Converter indices
enum : int {
    SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QT_GUI_CPP_PLUGINPROVIDERPTR_IDX = 0, // const QList<qt_gui_cpp::PluginProvider* > &
    SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QSTRING_IDX      = 1, // QMap<QString,QString >
    SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QT_GUI_CPP_PLUGINDESCRIPTORPTR_IDX = 2, // QList<qt_gui_cpp::PluginDescriptor* >
    SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QOBJECTPTR_IDX          = 3, // const QList<QObject* > &
    SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QBYTEARRAY_IDX          = 4, // QList<QByteArray >
    SBK_LIBQT_GUI_CPP_SHIBOKEN_QVECTOR_QMAP_QSTRING_QSTRING_IDX = 5, // QVector<QMap< QString,QString > >
    SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QVARIANT_IDX            = 6, // QList<QVariant >
    SBK_LIBQT_GUI_CPP_SHIBOKEN_QLIST_QSTRING_IDX             = 7, // QList<QString >
    SBK_LIBQT_GUI_CPP_SHIBOKEN_QMAP_QSTRING_QVARIANT_IDX     = 8, // QMap<QString,QVariant >
    SBK_libqt_gui_cpp_shiboken_CONVERTERS_IDX_COUNT          = 9
};
// Macros for type check

namespace Shiboken
{

// PyType functions, to get the PyObjectType for a type T
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
template<> inline PyTypeObject *SbkType< ::qt_gui_cpp::CompositePluginProvider >() { return reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_COMPOSITEPLUGINPROVIDER_IDX]); }
template<> inline PyTypeObject *SbkType< ::qt_gui_cpp::GenericProxy >() { return reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_GENERICPROXY_IDX]); }
template<> inline PyTypeObject *SbkType< ::qt_gui_cpp::Plugin >() { return reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGIN_IDX]); }
template<> inline PyTypeObject *SbkType< ::qt_gui_cpp::PluginBridge >() { return reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINBRIDGE_IDX]); }
template<> inline PyTypeObject *SbkType< ::qt_gui_cpp::PluginContext >() { return reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINCONTEXT_IDX]); }
template<> inline PyTypeObject *SbkType< ::qt_gui_cpp::PluginDescriptor >() { return reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINDESCRIPTOR_IDX]); }
template<> inline PyTypeObject *SbkType< ::qt_gui_cpp::PluginProvider >() { return reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_PLUGINPROVIDER_IDX]); }
template<> inline PyTypeObject *SbkType< ::qt_gui_cpp::RecursivePluginProvider >() { return reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_RECURSIVEPLUGINPROVIDER_IDX]); }
template<> inline PyTypeObject *SbkType< ::qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders >() { return reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINPROVIDERS_IDX]); }
template<> inline PyTypeObject *SbkType< ::qt_gui_cpp::RosPluginlibPluginProvider_ForPlugins >() { return reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_ROSPLUGINLIBPLUGINPROVIDER_FORPLUGINS_IDX]); }
template<> inline PyTypeObject *SbkType< ::qt_gui_cpp::Settings >() { return reinterpret_cast<PyTypeObject *>(Sbklibqt_gui_cpp_shibokenTypes[SBK_QT_GUI_CPP_SETTINGS_IDX]); }
QT_WARNING_POP

} // namespace Shiboken

#endif // SBK_LIBQT_GUI_CPP_SHIBOKEN_PYTHON_H

