#ifndef MASTER_THREAD_H
#define MASTER_THREAD_H

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QVector>
#include <QMap>
#include <QSerialPort>
#include "ComPort.h"

class MasterThread : public QThread
{
    Q_OBJECT

public:

    explicit MasterThread(QObject *parent = nullptr);
    ~MasterThread();

    void transaction(const QString &portName, int waitTimeout,
                     const QString &request, const QString &fus_name);
    void end_transaction();
    void run() Q_DECL_OVERRIDE;

signals:
    void response(const QString &s);
    void error(const QString &s);
    void timeout(const QString &s);

private:
    QString portName;
    QString request;
    QString fus_name;
    int waitTimeout;
    QMutex mutex;
    QWaitCondition cond;
    bool quit;

    void set_initial_position(CCommPort& serial);
    void motion_romb(CCommPort& serial ,const QString& fus_name);
};

#endif // MASTER_THREAD_H
