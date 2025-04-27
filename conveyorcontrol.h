#ifndef CONVEYORCONTROL_H
#define CONVEYORCONTROL_H

#include <QDialog>
#include "pca9685.h"
#include <QTimer>
#include <opencv2/opencv.hpp>

namespace Ui {
class ConveyorControl;
}

class ConveyorControl : public QDialog
{
    Q_OBJECT

public:
    explicit ConveyorControl(QWidget *parent = nullptr);
    ~ConveyorControl();

   //static bool isConveyorOn;  // shared between all ConveyorControl instances

    void setPCA9685(PCA9685* controller);
    void setMotorSpeed(int value);
    void applyMotorSpeed(int value);
    bool getConveyorStatus() const;
    void setArmValue(int value);





private slots:
    void on_pushButton_on_clicked();
    void on_pushButton_off_clicked();
    void on_pushButton_return_clicked();
    void updateCameraFrame();
    void updateCameraFrame2();  // second camera's update function
    void loadTemplate();
    float convertRange(float value, float in_min, float in_max, float out_min, float out_max);
        float deg(float radian);
        void updateServos();

private:
    Ui::ConveyorControl *ui;
    PCA9685* pca;
    QTimer* timer;
    cv::VideoCapture cap;
    cv::VideoCapture cap2;
    int motorSpeedValue;
    int patternW = 0;
    int patternH = 0;
    bool hasTemplate = false;
    bool patternMatched1 = false;
    bool patternMatched2 = false;
    cv::Rect lastMatchBox1, lastMatchBox2;
    float realWidthMM, realHeightMM, realWidthMM_rounded2, claww;

    int armValue = 1;


    float len1 = 17.0;             // length of arm 1
    float len2 = 24.0;             // length of arm 2
    char c;                        // initialise char
    float arm_x = 20.0;            // initialise x starting position
    float arm_y = 17.0;            // initialise y starting position
    float targetX = 0.0;
    float targetY = 0.0;
    bool C_open = true;            // initialise C_open as true to show the claw is open
    bool arrived = false;
    bool drop = false;
    int movementStage = 0;


    int servo_control_1 = 400;     // initialise servo_control_1 starting position at mid point
    int servo_control_2 = 400;     // initialise servo_control_2 starting position at mid point
    int servo_control_3 = 400;     // initialise servo_control_3 starting position at mid point
    int servo_control_4 = 600;     // initialise servo_control_4 starting position at mid point
    int servo_control_5 = 300;     // initialise servo_control_5 starting position at mid point 300 - 550
    int turn_table = 400;

    int convaya_servo = 150;       // initialise servo_control starting position at mid point


signals:
    void dialogClosed();



};

#endif // CONVEYORCONTROL_H
