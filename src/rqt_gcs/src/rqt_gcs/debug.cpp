#include "util/debug.h"
#include <QDebug>

namespace rqt_gcs
{
 
Q_LOGGING_CATEGORY(lcar_bot, "lcar_bot");
    
namespace dbg
{
    
    bool initialized = false;
    
    void InitDbg()
    {
        if(initialized)
            return;
        
        initialized = true;
        
        QLoggingCategory::setFilterRules("*.debug=false\n"
                                         "*.warning=false\n"
                                         "lcar_bot.debug=true\n"
                                         "lcar_bot.warning=true\n"
                                         "lcar_bot.critical=true\n");
        qInstallMessageHandler(MsgHandler
                            //,0
                               );
    }
    
    void MsgHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg)
    {
        QByteArray localMsg = msg.toLocal8Bit();
        QByteArray file = FileFromPath(QString(context.file)).toLocal8Bit(); // file contains complete path
        switch (type) 
        {
            case QtInfoMsg:
                fprintf(stdout, "qInfo: %s\n", localMsg.constData());
                break;
            case QtDebugMsg:    // cyan
                fprintf(stderr, "\033[1;36mqDebug: %s (%s:%u, %s)\033[0m\n", localMsg.constData(), file.constData(), context.line, context.function);
                break;
            case QtWarningMsg:  // yellow
                fprintf(stderr, "\033[1;33mqWarning: %s (%s:%u, %s)\033[0m\n", localMsg.constData(), file.constData(), context.line, context.function);
                break;
            case QtCriticalMsg: // red
                fprintf(stderr, "\033[1;31mqCritical: %s (%s:%u, %s)\033[0m\n", localMsg.constData(), file.constData(), context.line, context.function);
                break;
            case QtFatalMsg:    // magenta
                fprintf(stderr, "\033[1;35mqFatal: %s (%s:%u, %s)\033[0m\n", localMsg.constData(), file.constData(), context.line, context.function);
                abort();
        }
    }
    
    QString FileFromPath(const QString& path)
    {
        QString s(path);
        return s.mid(s.lastIndexOf('/') + 1);
    }

}
    
}
