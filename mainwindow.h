#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "pca9685.h"
#include "conveyorcontrol.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
private slots:

    void on_pushButton_3_clicked();
    void on_pushButton_2_clicked();


    void on_pushButton_4_clicked();

    void on_pushButton_clicked();

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    float len1 = 17.0;
    float len2 = 24.0;
    float arm_x = 30.0;
    float arm_y = 17.0;
    bool C_open = true;
    int servo_control_1 = 150;
    int servo_control_2 = 400;
    int servo_control_3 = 400;
    int servo_control_4 = 400;

    PCA9685 pca9685;
    ConveyorControl* conveyorDialog = nullptr;


};
#endif // MAINWINDOW_H

