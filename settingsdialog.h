#ifndef SETTINGSDIALOG_H
#define SETTINGSDIALOG_H

#include <QDialog>
#include "pca9685.h"

namespace Ui {
class SettingsDialog;
}

class SettingsDialog : public QDialog
{
    Q_OBJECT


private slots:
    void on_speedSlider_valueChanged(int value);
    void on_emergancystop_clicked();
    void on_arm_slider_valueChanged(int value);



public:
    explicit SettingsDialog(QWidget *parent = nullptr);
    ~SettingsDialog();
    void setPCA9685(PCA9685* controller);


private:
    Ui::SettingsDialog *ui;
    static int lastSpeedValue;
     PCA9685* pca;


signals:
     void motorSpeedChanged(int newSpeed);
     void armValueChanged(int newValue);  // New signal


};





#endif // SETTINGSDIALOG_H

