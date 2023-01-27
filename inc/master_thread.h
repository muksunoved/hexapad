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
    struct FusPoint {
        float x;
        float y;
        float z;
    };

    struct FusDegrees {
        int coxa;
        int femur;
        int tibia;
        int time = 0;
    };

    enum kInitials : size_t {
        kFront  = 0,
        kCentre = 1,
        kRear   = 2
    };

    enum Servos : size_t {
        kRightRear	 	= 0,
        kRightCentre	= 1,
        kRightFront		= 2,
        kLeftRear		= 3,
        kLeftCentre		= 4,
        kLeftFront		= 5

    };

    const float kCoxa = 35;
    const float kFemur = 100;
    const float kTibia = 150;

    const float kMaxForwardStep = 200.0;
    const float kFusYPosition = 100.0;

    const float kStepsInDegrees = 9.5;

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
    FusDegrees prepare_motion_from_to( FusPoint to, Servos servos);
    void motion_romb(CCommPort& serial ,const QString& fus_name);
    void motion_step(CCommPort& serial ,const QString& fus_name);
    int get_max_time(FusDegrees a, FusDegrees b);
    void calibrate(CCommPort& serial ,const QString& fus_name);

    void forward(CCommPort &serial);
    void Rotate(CCommPort &serial, bool left);

    QMap<size_t,FusPoint> last_step;
    QMap<size_t,FusDegrees> last_degrees;
};

#endif // MASTER_THREAD_H
