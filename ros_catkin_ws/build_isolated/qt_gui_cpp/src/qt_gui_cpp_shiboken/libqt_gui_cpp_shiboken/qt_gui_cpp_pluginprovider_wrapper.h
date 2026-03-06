#ifndef SBK_PLUGINPROVIDERWRAPPER_H
#define SBK_PLUGINPROVIDERWRAPPER_H

#define protected public

#include <qt_gui_cpp/plugin_provider.h>


// Extra includes
#include <QMap>
#include <qobject.h>
#include <QList>
#include <qt_gui_cpp/plugin_descriptor.h>
#include <qt_gui_cpp/plugin_context.h>
#include <qt_gui_cpp/plugin.h>
class PluginProviderWrapper : public qt_gui_cpp::PluginProvider
{
public:
    PluginProviderWrapper();
    QMap<QString,QString > discover(QObject * discovery_data) override;
    QList<qt_gui_cpp::PluginDescriptor* > discover_descriptors(QObject * discovery_data) override;
    void * load(const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context) override;
    qt_gui_cpp::Plugin * load_plugin(const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context) override;
    void shutdown() override;
    void unload(void * plugin_instance) override;
    void unload_plugin(qt_gui_cpp::Plugin * plugin_instance) override;
    ~PluginProviderWrapper();
    static void pysideInitQtMetaTypes();
    void resetPyMethodCache();
private:
    mutable bool m_PyMethodCache[7];
};

#endif // SBK_PLUGINPROVIDERWRAPPER_H

