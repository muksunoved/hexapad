#include "master_thread.h"

#include <QtSerialPort/QSerialPort>

#include <QTime>
#include <QDebug>
#include <math.h>

QT_USE_NAMESPACE

static const QMap<size_t, MasterThread::FusDegrees> fuses  = {
    {MasterThread::Servos::kRightRear,   {0,1,2} },
    {MasterThread::Servos::kRightCentre, {4,5,6} },
    {MasterThread::Servos::kRightFront,  {8,9,10} },
    {MasterThread::Servos::kLeftRear,    {16,17,18} },
    {MasterThread::Servos::kLeftCentre,  {20,21,22} },
    {MasterThread::Servos::kLeftFront,   {24,25,26} },
    };

static const QMap<size_t, std::tuple<float,float>> zero_angles_coxa  = {
    {MasterThread::Servos::kRightRear,   {60.0,		1.0} },
    {MasterThread::Servos::kRightCentre, {0, 		1.0} },
    {MasterThread::Servos::kRightFront,  {-60.0,	1.0} },
    {MasterThread::Servos::kLeftRear,    {-60.0,   	-1.0} },
    {MasterThread::Servos::kLeftCentre,  {0, 		-1.0} },
    {MasterThread::Servos::kLeftFront,   {60.0, 	-1.0} },
    };

static const QMap<size_t, float> zero_angles_femur  = {
    {MasterThread::Servos::kRightRear,   -90.0 },
    {MasterThread::Servos::kRightCentre, -90.0 },
    {MasterThread::Servos::kRightFront,  -90.0 },
    {MasterThread::Servos::kLeftRear,    -90.0 },
    {MasterThread::Servos::kLeftCentre,  -90.0 },
    {MasterThread::Servos::kLeftFront,   -90.0 },
    };

static const QMap<size_t, float> zero_angles_tibia  = {
    {MasterThread::Servos::kRightRear,   -90.0 },
    {MasterThread::Servos::kRightCentre, -90.0 },
    {MasterThread::Servos::kRightFront,  -90.0 },
    {MasterThread::Servos::kLeftRear,    -90.0 },
    {MasterThread::Servos::kLeftCentre,  -90.0 },
    {MasterThread::Servos::kLeftFront,   -90.0 },
    };


static const QMap<QString, MasterThread::Servos> fuses_names  = {
    {"RightRear",   		MasterThread::Servos::kRightRear  	},
    {"RightCentre", 		MasterThread::Servos::kRightCentre	},
    {"RightFront",  		MasterThread::Servos::kRightFront 	},
    {"LeftRear",    		MasterThread::Servos::kLeftRear   	},
    {"LeftCentre",  		MasterThread::Servos::kLeftCentre 	},
    {"LeftFront",   		MasterThread::Servos::kLeftFront  	},
    };

static const QVector<MasterThread::FusDegrees>romb = {
    {1000, 1500, 1500},
    {1500, 2000, 2000},
    {2000, 1500, 1500},
    {1500, 1000, 1000},
    {1000, 1500, 1500},
};

static const QMap<size_t,MasterThread::FusPoint>kInitialPositions = {
    {MasterThread::Servos::kRightRear,  {-116.9, 67.5 , 150.0}},
    {MasterThread::Servos::kRightCentre,{ 0    , 135.0, 150.0}},
    {MasterThread::Servos::kRightFront, { 116.9, 67.5 , 150.0}},
    {MasterThread::Servos::kLeftRear,   {-116.9, 67.5 , 150.0}},
    {MasterThread::Servos::kLeftCentre, { 0    , 135.0, 150.0}},
    {MasterThread::Servos::kLeftFront,  { 116.9, 67.5 , 150.0}},
};

static const QMap<size_t,MasterThread::FusDegrees>kInitialDegrees = {
    {MasterThread::Servos::kRightRear,  { 1500, 1500, 1500}},
    {MasterThread::Servos::kRightCentre,{ 1500, 1500, 1500}},
    {MasterThread::Servos::kRightFront, { 1500, 1500, 1500}},
    {MasterThread::Servos::kLeftRear,   { 1500, 1500, 1500}},
    {MasterThread::Servos::kLeftCentre, { 1500, 1500, 1500}},
    {MasterThread::Servos::kLeftFront,  { 1500, 1500, 1500}},
};

static const QVector<MasterThread::FusPoint>kStepsInitial = {
    { 116.9, 67.5 , 150.0 },
    { 116.9, 67.5 ,  80.0 },
    { 200.0, 100.0 , 80.0 },
};
static const QVector<MasterThread::FusPoint>kStepsMotion = {
    { 150.0, 100.0 , 80.0},
    { 100.0, 100.0 , 80.0},
    { 50.0,  100.0 , 80.0},
    { 0.0,   100.0 , 80.0},
    { 50.0,  100.0 , 60.0},
    { 100.0, 100.0 , 60.0},
    { 150.0, 100.0 , 60.0},
    { 200.0, 100.0 , 60.0},
};

static const QVector<MasterThread::FusPoint>kTripodPos1 = {
    {  0.0, 120.0 , 120.0},
    {-75.0, 120.0 , 120.0},
    {-75.0, 120.0 , 120.0},
    {-75.0, 120.0 , 70.0},
    {  0.0, 120.0 , 60.0},
    { 75.0, 120.0 , 70.0},
    { 75.0, 120.0 , 120.0},
    { 75.0, 120.0 , 120.0},
};
static const QVector<MasterThread::FusPoint>kTripodPos2 = {
    {  0.0, 120.0 , 60.0},
    { 75.0, 120.0 , 70.0},
    { 75.0, 120.0 , 120.0},
    { 75.0, 120.0 , 120.0},
    {  0.0, 120.0 , 120.0},
    {-75.0, 120.0 , 120.0},
    {-75.0, 120.0 , 120.0},
    {-75.0, 120.0 , 70.0},
};

static const QMap<size_t,MasterThread::FusPoint>kStepOne = {
    {MasterThread::Servos::kRightRear,  {-200.0, 100.0,  80.0}},
    {MasterThread::Servos::kRightCentre,{-100.0, 100.0,  80.0}},
    {MasterThread::Servos::kRightFront, {   0.0, 100.0 , 80.0}},
    {MasterThread::Servos::kLeftRear,   {   0.0, 100.0 , 80.0}},
    {MasterThread::Servos::kLeftCentre, { 100.0, 100.0,  80.0}},
    {MasterThread::Servos::kLeftFront,  { 200.0, 100.0 , 80.0}},
};


static const QVector<MasterThread::FusPoint>kStepsCalibrate = {
    { 0.0, 180.0 , 0.0 },
};

MasterThread::MasterThread(QObject *parent)
    : QThread(parent), waitTimeout(0), quit(false), last_step(kInitialPositions)
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
    qDebug() << "End transaction";
}

void MasterThread::transaction(const QString &portName, int waitTimeout,
                               const QString &request, const QString &fus_name)
{
    mutex.lock();
    this->portName = portName;
    this->waitTimeout = waitTimeout;
    this->request = request;
    this->fus_name = fus_name;
    quit = false;
    mutex.unlock();

    if (!isRunning()) {
        start();
    } else {
     //   cond.wakeOne();
    }
}

int MasterThread::get_max_time(FusDegrees a, FusDegrees b)
{
    int max = std::max(abs((a.coxa - b.coxa)),
    abs((a.tibia - b.tibia)));
    max = std::max(max, abs(a.femur - b.femur));
    float time = float(max) / 1500.0 * 1500.0;
    return int(time);
}

void MasterThread::motion_step(CCommPort &serial, const QString& fus_name)
{
    qDebug() << "Motion step";

    if (fuses_names.find(fus_name) == fuses_names.end()) {
        return;
    }
    auto fus_num = fuses_names[fus_name];
    auto fus = fuses[fus_num];
    QVector<FusDegrees> initial_degrees;
    QVector<FusDegrees> step_degrees;

    qDebug() << "Prepare initial steps";

    for (auto step : kStepsInitial) {
        auto next = prepare_motion_from_to(step, fus_num);
        next.time = get_max_time(last_degrees[fus_num], next);
        last_step[fus_num] = step;
        last_degrees[fus_num] = next;
        initial_degrees.push_back(next);
        qDebug() << "[" << next.coxa << ";" << next.femur << ";" << next.tibia << " : " << next.time << "]";
    }

    qDebug() << "Prepare motion steps";

    for (auto step : kStepsMotion) {
        auto next = prepare_motion_from_to(step, fus_num);
        next.time = get_max_time(last_degrees[fus_num], next);
        last_step[fus_num] = step;
        last_degrees[fus_num] = next;
        step_degrees.push_back(next);
        qDebug() << "[" << next.coxa << ";" << next.femur << ";" << next.tibia << " : " << next.time << "]";
    }

    for (auto& pnt : initial_degrees) {
        QString cmd = QString("#%1 P%2 #%3 P%4 #%5 P%6 T%7\r")
                .arg(fus.coxa).arg(pnt.coxa)
                .arg(fus.femur).arg(pnt.femur)
                .arg(fus.tibia).arg(pnt.tibia)
                .arg(pnt.time);
        QByteArray requestData = cmd.toLocal8Bit();
        serial.WriteBlock(requestData.data(), requestData.size());
        this->msleep(std::min(1000, pnt.time));
    }
    while (!quit) {
        for (auto& pnt : step_degrees) {
        QString cmd = QString("#%1 P%2 #%3 P%4 #%5 P%6 T%7\r")
            .arg(fus.coxa).arg(pnt.coxa)
            .arg(fus.femur).arg(pnt.femur)
            .arg(fus.tibia).arg(pnt.tibia)
            .arg(pnt.time);
        QByteArray requestData = cmd.toLocal8Bit();
        serial.WriteBlock(requestData.data(), requestData.size());
        this->msleep(std::min(1000, pnt.time));
        }

    }
}

void MasterThread::Rotate(CCommPort &serial, bool left)
{
    QMap<Servos, QVector<FusPoint>>initial_step;
    QMap<Servos, QVector<FusDegrees>>degrees;

    auto last_degrees_saved = last_degrees;
    auto last_point_saved = last_step;

    qDebug() << "Prepare pos";

    if (!left) {
        for (int i=0; i<kTripodPos1.size(); i++) {
            auto tripod_pos = kTripodPos1[i];
            initial_step[Servos::kRightRear		].push_back(tripod_pos);
            initial_step[Servos::kLeftCentre	].push_back(tripod_pos);
            initial_step[Servos::kRightFront	].push_back(tripod_pos);

            tripod_pos = kTripodPos2[i];
            initial_step[Servos::kLeftRear		].push_back(tripod_pos);
            initial_step[Servos::kRightCentre	].push_back(tripod_pos);
            initial_step[Servos::kLeftFront		].push_back(tripod_pos);
        }
    } else {

        for (int i=kTripodPos1.size()-1; i>=0; i--) {
            auto tripod_pos = kTripodPos1[i];
            initial_step[Servos::kRightRear		].push_back(tripod_pos);
            initial_step[Servos::kLeftCentre	].push_back(tripod_pos);
            initial_step[Servos::kRightFront	].push_back(tripod_pos);

            tripod_pos = kTripodPos2[i];
            initial_step[Servos::kLeftRear		].push_back(tripod_pos);
            initial_step[Servos::kRightCentre	].push_back(tripod_pos);
            initial_step[Servos::kLeftFront		].push_back(tripod_pos);
        }
    }

    qDebug() << "Prepare degrees";

    for(auto pos = initial_step.begin(); pos != initial_step.end(); pos++) {
        auto size = pos.value().size();
        auto values = pos.value();
        auto servos = pos.key();
        for (int i = 0; i<size; i++) {
            auto next = prepare_motion_from_to(values[i],Servos::kLeftCentre);
            next.time = get_max_time(last_degrees[servos], next);
            last_degrees[servos] = next;
            degrees[servos].push_back(next);
            QString d = QString("[ %1 %2 %3 %4]")
                    .arg(next.coxa)
                    .arg(next.femur)
                    .arg(next.tibia)
                    .arg(next.time);
            qDebug() << d;
        }
    }

    last_degrees = last_degrees_saved;
    last_step = last_point_saved;

    qDebug() << "Motion";

    while(!quit) {
        for (int i=0; i<kTripodPos1.size(); i++) {
            int time = 0;
            QString cmd;
            for(auto deg = degrees.begin(); deg != degrees.end(); deg++) {
                auto servos = deg.key();
                auto fus = fuses[servos];
                auto pnt = deg.value()[i];
                pnt.time = get_max_time(last_degrees[servos], pnt);
                time = std::max(time, pnt.time);

                cmd += QString("#%1 P%2 #%3 P%4 #%5 P%6 ")
                    .arg(fus.coxa).arg(pnt.coxa)
                    .arg(fus.femur).arg(pnt.femur)
                    .arg(fus.tibia).arg(pnt.tibia);
                last_degrees[servos]  = pnt;
            }
            time = std::min(2000, time);

            qDebug() << "Time: " << time;

            cmd += QString("T%1\r").arg(time);
            QByteArray requestData = cmd.toLocal8Bit();
            serial.WriteBlock(requestData.data(), requestData.size());
            this->msleep(time);
        }

    }

}
void MasterThread::forward(CCommPort &serial)
{
    QMap<Servos, QVector<FusPoint>>initial_step;
    QMap<Servos, QVector<FusDegrees>>degrees;

    auto last_degrees_saved = last_degrees;
    auto last_point_saved = last_step;

    qDebug() << "Prepare pos";

    for (int i=0; i<kTripodPos1.size(); i++) {
        auto tripod_pos = kTripodPos1[i];
        tripod_pos.x -= 75.0;
        initial_step[Servos::kRightRear		].push_back(tripod_pos);
        tripod_pos = kTripodPos1[i];
        initial_step[Servos::kLeftCentre	].push_back(tripod_pos);
        tripod_pos = kTripodPos1[i];
        tripod_pos.x += 75.0;
        initial_step[Servos::kRightFront	].push_back(tripod_pos);

        tripod_pos = kTripodPos2[i];
        tripod_pos.x -= 75.0;
        initial_step[Servos::kLeftRear		].push_back(tripod_pos);
        tripod_pos = kTripodPos2[i];
        initial_step[Servos::kRightCentre	].push_back(tripod_pos);
        tripod_pos = kTripodPos2[i];
        tripod_pos.x += 75.0;
        initial_step[Servos::kLeftFront		].push_back(tripod_pos);
    }
    qDebug() << "Prepare degrees";

    for(auto pos = initial_step.begin(); pos != initial_step.end(); pos++) {
        auto size = pos.value().size();
        auto values = pos.value();
        auto servos = pos.key();
        for (int i = 0; i<size; i++) {
            auto next = prepare_motion_from_to(values[i], servos);
            next.time = get_max_time(last_degrees[servos], next);
            last_degrees[servos] = next;
            degrees[servos].push_back(next);
            QString d = QString("[ %1 %2 %3 %4]")
                    .arg(next.coxa)
                    .arg(next.femur)
                    .arg(next.tibia)
                    .arg(next.time);
            qDebug() << d;
        }
    }

    last_degrees = last_degrees_saved;
    last_step = last_point_saved;

    qDebug() << "Motion";

    while(!quit) {
        for (int i=0; i<kTripodPos1.size(); i++) {
            int time = 0;
            QString cmd;
            for(auto deg = degrees.begin(); deg != degrees.end(); deg++) {
                auto servos = deg.key();
                auto fus = fuses[servos];
                auto pnt = deg.value()[i];
                pnt.time = get_max_time(last_degrees[servos], pnt);
                time = std::max(time, pnt.time);

                cmd += QString("#%1 P%2 #%3 P%4 #%5 P%6 ")
                    .arg(fus.coxa).arg(pnt.coxa)
                    .arg(fus.femur).arg(pnt.femur)
                    .arg(fus.tibia).arg(pnt.tibia);
                last_degrees[servos]  = pnt;
            }
            time = std::min(2000, time);

            qDebug() << "Time: " << time;

            cmd += QString("T%1\r").arg(time);
            QByteArray requestData = cmd.toLocal8Bit();
            serial.WriteBlock(requestData.data(), requestData.size());
            this->msleep(time);
        }

    }

}

MasterThread::FusDegrees MasterThread::prepare_motion_from_to(FusPoint to, Servos servos)
{
    float gamma_deg = atan(to.x/to.y) / M_PI * 180.0 ;
    float correction_deg;
    float correctin_sign;

    std::tie(correction_deg, correctin_sign) = zero_angles_coxa[servos];
    gamma_deg = correction_deg + correctin_sign*gamma_deg;
    gamma_deg *= kStepsInDegrees;

    float L1 = sqrt(to.x*to.x + to.y*to.y);
    float L = sqrt(to.z*to.z + (L1-kCoxa)*(L1-kCoxa));
    float alpha1 = acos(to.z / L);
    float alpha2 = acos((kTibia*kTibia - kFemur*kFemur - L*L) / (-2*kFemur*L));
    float alpha_deg = (alpha1 + alpha2) / M_PI * 180 + zero_angles_femur[servos];
    alpha_deg *= kStepsInDegrees;

    float beta_deg = acos((L*L - kTibia*kTibia - kFemur*kFemur) / (-2*kTibia*kFemur)) / M_PI * 180  + zero_angles_tibia[servos];
    beta_deg *= kStepsInDegrees;
    MasterThread::FusDegrees result = {kInitialDegrees[servos].coxa + gamma_deg,
                kInitialDegrees[servos].femur + alpha_deg,
                kInitialDegrees[servos].tibia + beta_deg};
    return result;

}

void MasterThread::set_initial_position(CCommPort& serial)
{
    qDebug() << "Set initial position";
    QString cmd = "";
    for (auto& fus : fuses) {
        cmd += QString("#%1 P%2 #%3 P%4 #%5 P%6 T1000\r ").arg(fus.coxa).arg(1500)
                .arg(fus.femur).arg(1500)
                .arg(fus.tibia).arg(1500) ;
    }
    qDebug() << "Send to bot " + cmd;
    QByteArray requestData = cmd.toLocal8Bit();

    serial.WriteBlock(requestData.data(), requestData.size());

    qDebug() << "Sleep";
    this->msleep(1000);
    qDebug() << "Success";
}

void MasterThread::calibrate(CCommPort &serial, const QString& fus_name)
{
    qDebug() << "Calibrate step";

    if (fuses_names.find(fus_name) == fuses_names.end()) {
        return;
    }
    auto fus_num = fuses_names[fus_name];
    auto fus = fuses[fus_num];
    QVector<FusDegrees> initial_degrees;

    qDebug() << "Prepare initial steps";

    for (auto step : kStepsCalibrate) {
        auto next = prepare_motion_from_to(step, fus_num);
        next.time = get_max_time(last_degrees[fus_num], next);
        last_step[fus_num] = step;
        last_degrees[fus_num] = next;
        initial_degrees.push_back(next);
        qDebug() << "[" << next.coxa << ";" << next.femur << ";" << next.tibia << " : " << next.time << "]";
    }


    for (auto& pnt : initial_degrees) {
        QString cmd = QString("#%1 P%2 #%3 P%4 #%5 P%6 T%7\r")
                .arg(fus.coxa).arg(pnt.coxa)
                .arg(fus.femur).arg(pnt.femur)
                .arg(fus.tibia).arg(pnt.tibia)
                .arg(pnt.time);
        qDebug() << "Send to bot " + cmd;
        QByteArray requestData = cmd.toLocal8Bit();
        serial.WriteBlock(requestData.data(), requestData.size());
        this->msleep(std::min(1000, pnt.time));
    }
    while (!quit) {
        this->msleep(1000);
    }



}
void MasterThread::motion_romb(CCommPort &serial, const QString& fus_name)
{
    emit error("Motion romb");
    if (fuses_names.find(fus_name) == fuses_names.end()) {
        return;
    }
    auto fus = fuses[fuses_names[fus_name]];
    for (auto pnt : romb) {
        QString cmd = QString("#%1 P%2 #%3 P%4 #%5 P%6 ")
                .arg(fus.coxa).arg(pnt.coxa)
                .arg(fus.femur).arg(pnt.femur)
                .arg(fus.tibia).arg(pnt.tibia);
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
    if (currentPortNameChanged) {
        serial.Close();
    }

    if (!serial.Open(currentPortName.toStdString(), 115200, CCommPort::kFlagsDefault, 'N', 8, 1 )) {
        emit error(tr("Can't open %1")
           .arg(portName));
        return;
    }

    while (!quit) {
        set_initial_position(serial);
        if (currentRequest.contains("Romb")) {
            qDebug() << "Contain Romb";
            while(!quit) {
                motion_romb(serial, currentFus);
            }
        }
        if (currentRequest.contains("Step")) {
            qDebug() << "Contain Step";
            while(!quit) {
                motion_step(serial, currentFus);
            }
        }
        if (currentRequest.contains("Calibre")) {
            qDebug() << "Contain Step";
            while(!quit) {
                calibrate(serial, currentFus);
            }
        }
        if (currentRequest.contains("Forward")) {
            qDebug() << "Contain Forward";
            while(!quit) {
                forward(serial);
            }
        }
        if (currentRequest.contains("RotateLeft")) {
            qDebug() << "Contain Forward";
            while(!quit) {
                Rotate(serial, true);
            }
        }
        if (currentRequest.contains("RotateRight")) {
            qDebug() << "Contain Forward";
            while(!quit) {
                Rotate(serial, false);
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
