#include "settingsdialog.h"
#include "ui_settingsdialog.h"
#include <QDebug>
#include <QSettings>
#include <QShowEvent>


int SettingsDialog::lastSpeedValue = 150;  // default starting value

SettingsDialog::SettingsDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::SettingsDialog)
{
    ui->setupUi(this);
    connect(ui->arm_slider, &QSlider::valueChanged, this, &SettingsDialog::on_arm_slider_valueChanged);

    QSettings settings("OwenCorp", "ARMOR");  // You can change this name if you want
    int savedSpeed = settings.value("sliderValue", 150).toInt();  // default to 150
    ui->speedSlider->setValue(savedSpeed);


    this->setWindowFlags(Qt::Window);  // ? Required
    this->showFullScreen();            // ? Ensures it fills the screen

    ui->emergancystop->setText("");
}
void SettingsDialog::on_arm_slider_valueChanged(int value)
{
    emit armValueChanged(value);  // Emit the new value
}


SettingsDialog::~SettingsDialog()
{
    delete ui;
}

void SettingsDialog::on_speedSlider_valueChanged(int value)
{
    // Save slider value persistently
    QSettings settings("OwenCorp", "ARMOR");
    settings.setValue("sliderValue", value);

    emit motorSpeedChanged(value);
}

void SettingsDialog::on_emergancystop_clicked()
{
    for (int i = 0; i < 6; i++) {
        pca->setPWM(i, 0, 0);  // turn off all channels
    }
    QApplication::quit();  // Ends the whole app
}

void SettingsDialog::setPCA9685(PCA9685* controller)
{
    pca = controller;
}



