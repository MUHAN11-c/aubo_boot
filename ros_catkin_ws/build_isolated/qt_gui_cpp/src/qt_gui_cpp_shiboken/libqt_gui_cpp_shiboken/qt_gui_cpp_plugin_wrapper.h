#ifndef SBK_PLUGINWRAPPER_H
#define SBK_PLUGINWRAPPER_H

#define protected public

#include <qt_gui_cpp/plugin.h>


// Extra includes
#include <qobject.h>
#include <qt_gui_cpp/plugin_context.h>
#include <qt_gui_cpp/settings.h>
#include <qobjectdefs.h>
#include <qmetaobject.h>
#include <qthread.h>
#include <qcoreevent.h>
#include <QList>
#include <qbytearray.h>
namespace PySide { class DynamicQMetaObject; }

class PluginWrapper : public qt_gui_cpp::Plugin
{
public:
    PluginWrapper();
    void childEvent(QChildEvent * event) override;
    void connectNotify(const QMetaMethod & signal) override;
    void customEvent(QEvent * event) override;
    void disconnectNotify(const QMetaMethod & signal) override;
    bool event(QEvent * event) override;
    bool eventFilter(QObject * watched, QEvent * event) override;
    bool hasConfiguration() const override;
    void initPlugin(qt_gui_cpp::PluginContext & arg__1) override;
    const QMetaObject * metaObject() const override;
    void restoreSettings(const qt_gui_cpp::Settings & arg__1, const qt_gui_cpp::Settings & arg__2) override;
    void saveSettings(qt_gui_cpp::Settings & arg__1, qt_gui_cpp::Settings & arg__2) const override;
    void shutdownPlugin() override;
    void timerEvent(QTimerEvent * event) override;
    void triggerConfiguration() override;
    ~PluginWrapper();
public:
    int qt_metacall(QMetaObject::Call call, int id, void **args) override;
    void *qt_metacast(const char *_clname) override;
    static void pysideInitQtMetaTypes();
    void resetPyMethodCache();
private:
    mutable bool m_PyMethodCache[14];
};

#endif // SBK_PLUGINWRAPPER_H

