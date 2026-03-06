#ifndef SBK_RECURSIVEPLUGINPROVIDERWRAPPER_H
#define SBK_RECURSIVEPLUGINPROVIDERWRAPPER_H

#define protected public

#include <qt_gui_cpp/recursive_plugin_provider.h>


// Extra includes
#include <qt_gui_cpp/ros_pluginlib_plugin_provider_for_plugin_providers.h>
#include <QMap>
#include <qobject.h>
#include <QList>
#include <qt_gui_cpp/plugin_provider.h>
#include <qt_gui_cpp/plugin_descriptor.h>
#include <qt_gui_cpp/plugin_context.h>
#include <qt_gui_cpp/plugin.h>
class RecursivePluginProviderWrapper : public qt_gui_cpp::RecursivePluginProvider
{
public:
    RecursivePluginProviderWrapper(qt_gui_cpp::RosPluginlibPluginProvider_ForPluginProviders * plugin_provider);
    QMap<QString,QString > discover(QObject * discovery_data) override;
    QList<qt_gui_cpp::PluginDescriptor* > discover_descriptors(QObject * discovery_data) override;
    void * load(const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context) override;
    qt_gui_cpp::Plugin * load_plugin(const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context) override;
    void set_plugin_providers(const QList<qt_gui_cpp::PluginProvider* > & plugin_providers) override;
    void shutdown() override;
    void unload(void * plugin_instance) override;
    void unload_plugin(qt_gui_cpp::Plugin * plugin_instance) override;
    ~RecursivePluginProviderWrapper();
    static void pysideInitQtMetaTypes();
    void resetPyMethodCache();
private:
    mutable bool m_PyMethodCache[8];
};

#endif // SBK_RECURSIVEPLUGINPROVIDERWRAPPER_H

