#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtSerialPort/QSerialPortInfo>
#include <iostream>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    const auto infos = QSerialPortInfo::availablePorts() ;
    for (const QSerialPortInfo& info : infos ) {
        ui->serialPortComboBox->addItem(info.portName());

    }
    connect(&master_thread, &MasterThread::error, this, &MainWindow::processError);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionExit_triggered()
{
    this->close();
}

void MainWindow::on_pushButton_pressed()
{

}

void MainWindow::processError(const QString &s)
  {
    qDebug() << s ;
  }
void MainWindow::on_pushButton_released()
{
}

void MainWindow::on_pushButton_toggled(bool checked)
{
    if (checked) {
        master_thread.transaction(
            "/dev/" + ui->serialPortComboBox->currentText(),
            200, ui->motionComboBox->currentText(),
            ui->fusComboBox->currentText());
        qDebug() << "Pressed";

    } else {
        qDebug() << "Released";
        master_thread.end_transaction();
    }

}
