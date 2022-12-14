#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include "master_thread.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_actionExit_triggered();

    void on_pushButton_pressed();

    void on_pushButton_released();
    void processError(const QString &s);

    void on_pushButton_toggled(bool checked);

private:
    Ui::MainWindow *ui;
    QSerialPort serialPort;
    MasterThread master_thread;
};

#endif // MAINWINDOW_H
