#ifndef SBK_PLUGINBRIDGEWRAPPER_H
#define SBK_PLUGINBRIDGEWRAPPER_H

#define protected public

#include <qt_gui_cpp/plugin_bridge.h>


// Extra includes
#include <qobject.h>
#include <qt_gui_cpp/plugin_provider.h>
#include <qt_gui_cpp/plugin_context.h>
#include <qobjectdefs.h>
#include <qmetaobject.h>
#include <qthread.h>
#include <qcoreevent.h>
#include <QList>
#include <qbytearray.h>
namespace PySide { class DynamicQMetaObject; }

class PluginBridgeWrapper : public qt_gui_cpp::PluginBridge
{
public:
    PluginBridgeWrapper();
    void childEvent(QChildEvent * event) override;
    void connectNotify(const QMetaMethod & signal) override;
    void customEvent(QEvent * event) override;
    void disconnectNotify(const QMetaMethod & signal) override;
    bool event(QEvent * event) override;
    bool eventFilter(QObject * watched, QEvent * event) override;
    bool has_configuration() const override;
    bool load_plugin(qt_gui_cpp::PluginProvider * provider, const QString & plugin_id, qt_gui_cpp::PluginContext * plugin_context) override;
    const QMetaObject * metaObject() const override;
    void restore_settings(QObject * plugin_settings, QObject * instance_settings) override;
    void save_settings(QObject * plugin_settings, QObject * instance_settings) override;
    void shutdown_plugin() override;
    void timerEvent(QTimerEvent * event) override;
    void trigger_configuration() override;
    void unload_plugin() override;
    ~PluginBridgeWrapper();
public:
    int qt_metacall(QMetaObject::Call call, int id, void **args) override;
    void *qt_metacast(const char *_clname) override;
    static void pysideInitQtMetaTypes();
    void resetPyMethodCache();
private:
    mutable bool m_PyMethodCache[15];
};

#endif // SBK_PLUGINBRIDGEWRAPPER_H

