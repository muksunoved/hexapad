#include "master_thread.h"

#include <QtSerialPort/QSerialPort>

#include <QTime>
#include <QDebug>

QT_USE_NAMESPACE

struct Fus {
    int horiz;
    int vert;
    int zax;
};

static const QMap<QString, Fus> fuses  = {
    {"RightRear",   {0,1,2} },
    {"RightCentre", {4,5,6} },
    {"RightFront",  {8,9,10} },
    {"LeftRear",    {16,17,18} },
    {"LeftCentre",  {20,21,22} },
    {"LeftFront",   {24,25,26} },
    };
static const QVector<Fus>romb = {
    {1000, 1500, 1500},
    {1500, 2000, 2000},
    {2000, 1500, 1500},
    {1500, 1000, 1000},
    {1000, 1500, 1500},
};

MasterThread::MasterThread(QObject *parent)
    : QThread(parent), waitTimeout(0), quit(false)
{
}

MasterThread::~MasterThread()
{
    end_transaction();
}

void MasterThread::end_transaction()
{
    mutex.lock();
    quit = true;
    cond.wakeOne();
    mutex.unlock();
    wait();
}

void MasterThread::transaction(const QString &portName, int waitTimeout,
                               const QString &request, const QString &fus_name)
{
    QMutexLocker locker(&mutex);
    this->portName = portName;
    this->waitTimeout = waitTimeout;
    this->request = request;
    this->fus_name = fus_name;

    if (!isRunning())
        start();
    else
        cond.wakeOne();
}

void MasterThread::set_initial_position(CCommPort& serial)
{
    qDebug() << "Set initial position";
    QString cmd = "";
    for (auto& fus : fuses) {
        cmd += QString("#%1 P%2 #%3 P%4 #%5 P%6 ").arg(fus.horiz).arg(1500)
                .arg(fus.vert).arg(1500)
                .arg(fus.zax).arg(1500) ;
    }
    cmd += "T1000\r";
    qDebug() << "Send to bot " + cmd;
    QByteArray requestData = cmd.toLocal8Bit();
    qDebug() << "WriteBlock";
    for(int i = 0; i<requestData.size(); i++) {
        printf("%x ", int(*(requestData.data() + i)));
    }
    serial.WriteBlock(requestData.data(), requestData.size());
    qDebug() << "Sleep";
    this->msleep(1000);
    qDebug() << "Success";
}

void MasterThread::motion_romb(CCommPort &serial, const QString& fus_name)
{
    emit error("Motion romb");
    if (fuses.find(fus_name) == fuses.end()) {
        return;
    }
    auto fus = fuses[fus_name];
    for (auto pnt : romb) {
        QString cmd = QString("#%1 P%2 #%3 P%4 #%5 P%6 ")
                .arg(fus.horiz).arg(pnt.horiz)
                .arg(fus.vert).arg(pnt.vert)
                .arg(fus.zax).arg(pnt.zax);
        cmd += "T1000\r";
        qDebug() << "Send to bot " + cmd;
        QByteArray requestData = cmd.toLocal8Bit();
        serial.WriteBlock(requestData.data(), requestData.size());
        this->msleep(1000);
    }


}

void MasterThread::run()
{
    bool currentPortNameChanged = false;

    mutex.lock();
    QString currentPortName;
    if (currentPortName != portName) {
        currentPortName = portName;
        currentPortNameChanged = true;
    }

    int currentWaitTimeout = waitTimeout;
    QString currentRequest = request;
    QString currentFus = fus_name;
    mutex.unlock();
    CCommPort serial;

    qDebug() << currentRequest;

    if (currentPortName.isEmpty()) {
        emit error(tr("No port name specified"));
        return;
    }

    while (!quit) {
        if (currentPortNameChanged) {
            serial.Close();

            if (!serial.Open(currentPortName.toStdString(), 115200, CCommPort::kFlagsDefault, 'N', 8, 1 )) {
                emit error(tr("Can't open %1")
                           .arg(portName));
                return;
            }
        }
        set_initial_position(serial);
        if (currentRequest.contains("Romb")) {
            qDebug() << "Contain Romb";
            while(!quit) {
                motion_romb(serial, currentFus);
            }
        }
        set_initial_position(serial);
        mutex.lock();
        //cond.wait(&mutex);
        if (currentPortName != portName) {
            currentPortName = portName;
            currentPortNameChanged = true;
        } else {
            currentPortNameChanged = false;
        }
        currentWaitTimeout = waitTimeout;
        currentRequest = request;
        mutex.unlock();
    }
    serial.Close();
}
