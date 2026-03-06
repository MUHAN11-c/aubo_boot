#ifndef SBK_PLUGINCONTEXTWRAPPER_H
#define SBK_PLUGINCONTEXTWRAPPER_H

#define protected public

#include <qt_gui_cpp/plugin_context.h>


// Extra includes
#include <qobject.h>
#include <qt_gui_cpp/plugin_context.h>
#include <qwidget.h>
#include <qobjectdefs.h>
#include <qmetaobject.h>
#include <qthread.h>
#include <qcoreevent.h>
#include <QList>
#include <qbytearray.h>
namespace PySide { class DynamicQMetaObject; }

class PluginContextWrapper : public qt_gui_cpp::PluginContext
{
public:
    PluginContextWrapper(QObject * obj, int serial_number, const QStringList & argv);
    PluginContextWrapper(const qt_gui_cpp::PluginContext& self) : qt_gui_cpp::PluginContext(self)
    {
    }

    void childEvent(QChildEvent * event) override;
    void connectNotify(const QMetaMethod & signal) override;
    void customEvent(QEvent * event) override;
    void disconnectNotify(const QMetaMethod & signal) override;
    bool event(QEvent * event) override;
    bool eventFilter(QObject * watched, QEvent * event) override;
    const QMetaObject * metaObject() const override;
    void timerEvent(QTimerEvent * event) override;
    ~PluginContextWrapper();
public:
    int qt_metacall(QMetaObject::Call call, int id, void **args) override;
    void *qt_metacast(const char *_clname) override;
    static void pysideInitQtMetaTypes();
    void resetPyMethodCache();
private:
    mutable bool m_PyMethodCache[8];
};

#endif // SBK_PLUGINCONTEXTWRAPPER_H

