#ifndef SBK_COMPOSITEPLUGINPROVIDERWRAPPER_H
#define SBK_COMPOSITEPLUGINPROVIDERWRAPPER_H

#define protected public

#include <qt_gui_cpp/composite_plugin_provider.h>


// Extra includes
#include <QList>
#include <qt_gui_cpp/plugin_provider.h>
#include <qt_gui_cpp/plugin_descriptor.h>
#include <qobject.h>
#include <qt_gui_cpp/plugin_context.h>
#include <qt_gui_cpp/plugin.h>
#include <QMap>
class CompositePluginProviderWrapper : public qt_gui_cpp::CompositePluginProvider
{
public:
    CompositePluginProviderWrapper(const QList<qt_gui_cpp::PluginProvider* > & plugin_providers = QList< qt_gui_cpp::PluginProvider* >());
    QMap<QString,QString > discover(QObject * discovery_data) override;
    QList<qt_gui_cpp::PluginDescriptor* > discover_descriptors(QObject * discovery_data) override;
    void * load(const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context) override;
    qt_gui_cpp::Plugin * load_plugin(const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context) override;
    void set_plugin_providers(const QList<qt_gui_cpp::PluginProvider* > & plugin_providers) override;
    void shutdown() override;
    void unload(void * plugin_instance) override;
    void unload_plugin(qt_gui_cpp::Plugin * plugin_instance) override;
    ~CompositePluginProviderWrapper();
    static void pysideInitQtMetaTypes();
    void resetPyMethodCache();
private:
    mutable bool m_PyMethodCache[8];
};

#endif // SBK_COMPOSITEPLUGINPROVIDERWRAPPER_H

