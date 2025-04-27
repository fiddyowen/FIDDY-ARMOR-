
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "pca9685.h"
#include <opencv2/opencv.hpp>
#include "conveyorcontrol.h"
#include "settingsdialog.h"
#include <QDebug>
#include "rundiagnosis.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->showFullScreen();
    //setup
    ui->pushButton->setText("");
    ui->pushButton_2->setText("");
    ui->pushButton_3->setText("");
    ui->pushButton_4->setText("");
    pca9685.setPWMFreq(50);  // ? THIS IS ESSENTIAL FOR SERVOS!

    on_pushButton_2_clicked();



}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    for (int i = 0; i < 6; i++) {
        pca9685.setPWM(i, 0, 0); // Turns off signal for each channel
    }
    QApplication::quit();  // Ends the whole app

}


void MainWindow::on_pushButton_2_clicked()
{
    if (!conveyorDialog) {
        conveyorDialog = new ConveyorControl(this);
        conveyorDialog->setPCA9685(&pca9685);
        conveyorDialog->showFullScreen();

        connect(conveyorDialog, &ConveyorControl::dialogClosed, this, [=]() {
            conveyorDialog = nullptr;
        });
    }

    // ? Add connection HERE after creating conveyorDialog
    // We use a static_cast just in case you open Settings before Conveyor
    SettingsDialog* settings = findChild<SettingsDialog*>();
    if (settings) {
        connect(settings, &SettingsDialog::armValueChanged, conveyorDialog, &ConveyorControl::setArmValue);
    }

    conveyorDialog->showFullScreen();
}

void MainWindow::on_pushButton_3_clicked()
{
    RunDiagnosis *diagnosisWindow = new RunDiagnosis(this);
    diagnosisWindow->setAttribute(Qt::WA_DeleteOnClose);
    diagnosisWindow->show();


    diagnosisWindow->setPCA9685(&pca9685);


}


void MainWindow::on_pushButton_4_clicked()
{
    // If Conveyor hasn't been created yet, create it first
    if (!conveyorDialog) {
        on_pushButton_2_clicked();  // ? Ensures conveyorDialog exists
    }

    SettingsDialog* settings = new SettingsDialog(this);
    settings->setAttribute(Qt::WA_DeleteOnClose);
    settings->setPCA9685(&pca9685);
    settings->showFullScreen();

    connect(settings, &SettingsDialog::motorSpeedChanged, this, [=](int newSpeed){
        this->servo_control_1 = newSpeed;
        if (conveyorDialog) {
            conveyorDialog->applyMotorSpeed(newSpeed);
        }
    });

    connect(settings, &SettingsDialog::armValueChanged, this, [=](int newArmValue){
        if (conveyorDialog) {
            conveyorDialog->setArmValue(newArmValue);
        }
    });
}













