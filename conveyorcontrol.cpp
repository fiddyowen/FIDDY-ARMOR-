
#include "conveyorcontrol.h"
#include "ui_conveyorcontrol.h"
#include <QDebug>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include <QThread>
#include <QSettings>

 cv::Mat templateImage;  // To store the template image
 bool isConveyorOn = true;


 void ConveyorControl::loadTemplate()
 {
     // Load as color
     templateImage = cv::imread("/home/ofiddy/GUI/captured_pattern.jpg", cv::IMREAD_COLOR);
     hasTemplate = !templateImage.empty();

     if (!hasTemplate) {
         qDebug() << "Template image could not be loaded!";
     } else {
         qDebug() << "Template image loaded successfully!";
     }
 }


 float ConveyorControl::convertRange(float value, float in_min, float in_max, float out_min, float out_max) {
     return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 }

 // Helper function to convert radians to degrees
 float ConveyorControl::deg(float radian) {
     return radian * 180.0 / M_PI;
 }

 void ConveyorControl::updateServos() {

     float len1 = this->len1;
     float len2 = this->len2;

     // Gradually update the arm's position toward the target
     float smoothingFactor = armValue/10.0f;  // Smaller = slower, smoother
     qDebug() << "armValue:" << armValue;
     arm_x += smoothingFactor * (targetX - arm_x);
     arm_y += smoothingFactor * (targetY - arm_y);


     float x = arm_x;
     float y = arm_y;


     float angle1 = acos(((len2 * len2) - ((x * x) + (y * y)) - (len1 * len1)) / (-2 * (sqrt((x * x) + (y * y))) * len1)) + atan2(y, x);
     float angle2 = acos(((x * x) + (y * y) - (len2 * len2) - (len1 * len1)) / (-2 * len2 * len1));

     float angle1_deg = deg(angle1);
     float angle2_deg = deg(angle2);

     float convertedValue2 = convertRange(angle1_deg, 1, 180, 150, 650);  // servo 1
     float convertedValue1 = convertRange(angle1_deg, 1, 180, 650, 150);  // if reversed
     float convertedValue4 = convertRange(angle2_deg -90 , 1, 180, 650, 150);  // elbow reversed

     // Calculate the maximum change in position
     int maxChange = std::max(std::abs(convertedValue1 - servo_control_1), std::abs(convertedValue2 - servo_control_4));

     // Control PWM signals
     pca->setPWM(2, 0, servo_control_1);
     pca->setPWM(3, 0, servo_control_2);
     pca->setPWM(4, 0, servo_control_3);
     pca->setPWM(5, 0, servo_control_4);

     // Control convaya servo
     pca->setPWM(1, 0, servo_control_5);

     // Update servo positions smoothly if within range
     float hypotonose = sqrt((arm_x * arm_x) + (arm_y * arm_y)); // Calculate the distance from origin
     servo_control_3 += smoothingFactor * (turn_table - servo_control_3);


     if (hypotonose > 29.3 && hypotonose <= 41) { // Only allow change if in the possible range
         if (maxChange > 0) {
             servo_control_1 += smoothingFactor * (convertedValue1 - servo_control_1);
             servo_control_2 += smoothingFactor * (convertedValue2 - servo_control_2);
             servo_control_4 += smoothingFactor * (convertedValue4 - servo_control_4);


             bool positionCloseEnough = std::abs(arm_x - targetX) < 0.5 && std::abs(arm_y - targetY) < 0.5;



             // Check if we?ve arrived at the target position
             if (std::abs(convertedValue1 - servo_control_1) < 10 &&
                 std::abs(convertedValue2 - servo_control_2) < 10 &&
                 std::abs(convertedValue4 - servo_control_4) < 10 &&
                 std::abs(turn_table - servo_control_3) < 10 &&
                     positionCloseEnough) {

                if (!arrived) {
                    arrived = true;

                        if (movementStage == 0) {
                            // Move to second position
                            targetX = 18.0;
                            targetY = 35.0;


                            servo_control_5 = 550-(1.5*claww);
                            movementStage = 1;
                            arrived = false;
                        }
                        else if (movementStage == 1) {
                            // Move to second position
                            targetX = 18.0;
                            targetY = 35.0;

                            turn_table = 150;

                            movementStage = 2;
                            arrived = false;
                        }
                        else if (movementStage == 2) {
                            // Move to final home position
                            targetX = 35.0;
                            targetY = 2.0;


                            movementStage = 3;
                            arrived = false;

                        }
                        else if (movementStage == 3) {
                            // Move to final home position
                            targetX = 18.0;
                            targetY = 35.0;


                            movementStage = 4;
                            arrived = false;
                            servo_control_5 = 300;

                        }
                        else if (movementStage == 4) {
                            // Move to final home position
                            targetX = 20.0;
                            targetY = 17.0;

                            turn_table = 400;
                            movementStage = 0;
                            arrived = false;
                            drop = false;
                        }

                 }

             }
         }

     }
     qDebug() << "X:" << arm_x << " Y:" << arm_y;
     qDebug() << "Angle1 (deg):" << deg(angle1);
     qDebug() << "Angle2 (deg):" << deg(angle2);
     qDebug() << "Servo 1 PWM:" << convertedValue1;
     qDebug() << "Servo 4 PWM:" << convertedValue4;

 }


ConveyorControl::ConveyorControl(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ConveyorControl),
    pca(nullptr)
{
    ui->setupUi(this);
    this->showFullScreen();

    loadTemplate();

    QSettings settings("OwenCorp", "ARMOR");
    motorSpeedValue = settings.value("sliderValue", 250).toInt();


    ui->pushButton_on->setText("");
    ui->pushButton_off->setText("");
    ui->pushButton_return->setText("");

    connect(ui->pushButton_on, &QPushButton::clicked, this, &ConveyorControl::on_pushButton_on_clicked);
    connect(ui->pushButton_off, &QPushButton::clicked, this, &ConveyorControl::on_pushButton_off_clicked);
    connect(ui->pushButton_return, &QPushButton::clicked, this, &ConveyorControl::on_pushButton_return_clicked);




    cap.open("/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index0"); // Top view
    cap2.open("/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"); // Bottom view

    cap.set(cv::CAP_PROP_FPS, 30);  // Try 30 or 60 depending on your camera
    cap2.set(cv::CAP_PROP_FPS, 30);  // Try 30 or 60 depending on your camera


    if (!cap.isOpened()) {
        qDebug() << "Camera failed to open!";
    } else {
        timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, &ConveyorControl::updateCameraFrame);
        timer->start(30); // roughly 30 FPS
    }




    if (!cap2.isOpened()) {
        qDebug() << "Second camera failed to open!";
    } else {
        connect(timer, &QTimer::timeout, this, &ConveyorControl::updateCameraFrame2);
        cap2.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap2.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap2.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap2.set(cv::CAP_PROP_FPS, 30);
    }






}

ConveyorControl::~ConveyorControl()
{
    if (timer) timer->stop();         // Stop the camera refresh timer
    if (cap.isOpened()) cap.release(); // Release the camera so it’s free to use again
    if (cap2.isOpened()) cap2.release();
    delete ui;                        // Clean up the UI
}


void ConveyorControl::on_pushButton_on_clicked()
{

}






void ConveyorControl::on_pushButton_off_clicked()
{
    motorSpeedValue = 0;

    for (int i = 0; i < 6; i++) {
       pca->setPWM(i, 0, 0);  // turn off all channels
    }
    QApplication::quit();  // Ends the whole app
}

void ConveyorControl::on_pushButton_return_clicked()
{
    this->hide(); // Closes the dialog and returns to main window
}

void ConveyorControl::setPCA9685(PCA9685* controller)
{
    pca = controller;
}

void ConveyorControl::updateCameraFrame()
{
    cv::Mat frame;
    cap >> frame;

    if (frame.empty()) {
        qDebug() << "Camera 2 frame is empty!";
        return;
    }

    if (!frame.empty()) {
        // Convert to HSV
        cv::Mat hsv;
        if (hasTemplate && !patternMatched2) {
            cv::Mat grayTemplate, grayFrame, smallTemplate, smallFrame;
            cv::cvtColor(templateImage, grayTemplate, cv::COLOR_BGR2GRAY);
            cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
            cv::resize(grayTemplate, smallTemplate, cv::Size(), 0.5, 0.5);
            cv::resize(grayFrame, smallFrame, cv::Size(), 0.5, 0.5);
            cv::GaussianBlur(smallTemplate, smallTemplate, cv::Size(3, 3), 0);

            cv::Mat result;
            cv::matchTemplate(smallFrame, smallTemplate, result, cv::TM_SQDIFF_NORMED);
            double minVal;
            cv::Point minLoc;
            cv::minMaxLoc(result, &minVal, nullptr, &minLoc, nullptr);



            if (minVal < 0.2) {
                int matchWidth = smallTemplate.cols * 2;
                int matchHeight = smallTemplate.rows * 2;
                lastMatchBox2 = cv::Rect(minLoc.x * 2, minLoc.y * 2, matchWidth, matchHeight);
                patternMatched2 = true;
            }
        }
        if (patternMatched2) {
            cv::rectangle(frame, lastMatchBox2, cv::Scalar(255, 0, 255), 2);
            cv::putText(frame, "stop point",
                        cv::Point(lastMatchBox2.x, lastMatchBox2.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
        }



        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Define the range of red color in HSV space
        // Red HSV range
        cv::Mat maskRed1, maskRed2;
        cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), maskRed1);
        cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), maskRed2);
        cv::Mat redMask = maskRed1 | maskRed2;

        // Green HSV range
        cv::Mat greenMask;
        cv::inRange(hsv, cv::Scalar(35, 100, 70), cv::Scalar(85, 255, 255), greenMask);

        // Blue HSV range
        cv::Mat blueMask;
        cv::inRange(hsv, cv::Scalar(100, 150, 70), cv::Scalar(140, 255, 255), blueMask);

        // Combine all masks
        cv::Mat combinedMask = redMask | greenMask | blueMask;


        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(combinedMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);



        // Find the largest contour (based on your logic)
        if (!contours.empty()) {
            double maxArea = 0;
            std::vector<cv::Point> largestContour;
            for (const auto& contour : contours) {
                double area = cv::contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            if (maxArea > 1500) {  // Check if the contour is sufficiently large
                cv::Rect boundingBox = cv::boundingRect(largestContour);
                cv::rectangle(frame, boundingBox, cv::Scalar(0, 255, 255), 2);  // Draw bounding box

                // Calculate the center of the bounding box
                int objectCenterX = boundingBox.x + boundingBox.width / 2;

                // Get the center of the camera frame
                int frameCenterX = frame.cols / 2;

                // Display width and height of the bounding box
                int boxWidth = boundingBox.width;
                int boxHeight = boundingBox.height;

                int objectRightEdge = boundingBox.x + boundingBox.width;

                if (objectRightEdge > frameCenterX + 130) {
                    pca->setPWM(0, 0, 0);


                if(!drop){
                    targetX = 30.0;
                    targetY = 17.0;

                   drop = true;

                   claww = realWidthMM_rounded2;
                }


                   float scaleValue = 25.0f / patternW;  // width scale from green box



                   int realHeightMM_rounded2 = int(realWidthMM);

                   float realWidthMM2 = (realWidthMM / boxHeight) * boxWidth;
                   realWidthMM_rounded2 = int(realWidthMM2);



                // Display real-world width in mm below the pixel width
                cv::putText(frame, "Width: " + std::to_string(realWidthMM_rounded2) + " mm",
                        cv::Point(boundingBox.x, boundingBox.y + 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);  // Yellow text
                // Display real-world width in mm below the pixel width
                cv::putText(frame, "hight: " + std::to_string(realHeightMM_rounded2) + " mm",
                        cv::Point(boundingBox.x + boundingBox.width + 5, (boundingBox.y + boundingBox.height / 2)+20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);  // Yellow text


                }



                // Display width on top of the box
                cv::putText(frame, "W: " + std::to_string(boxWidth) + " px",
                            cv::Point(boundingBox.x, boundingBox.y - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

                // Display height on the side of the box
                cv::putText(frame, "H: " + std::to_string(boxHeight) + " px",
                            cv::Point(boundingBox.x + boundingBox.width + 5, boundingBox.y + boundingBox.height / 2),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

            }
        } else  {
            // If object not detected, check if the conveyor is ON and then set motor speed
                pca->setPWM(0, 0, motorSpeedValue);


        }




        // Convert the frame back to RGB for displaying in Qt
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);


        // Resize and display the frame
        cv::resize(frame, frame, cv::Size(ui->cameraLabel->width(), ui->cameraLabel->height()));
        QImage qimg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
        ui->cameraLabel->setPixmap(QPixmap::fromImage(qimg));
        updateServos();
    }


}


void ConveyorControl::updateCameraFrame2()
{
    cv::Mat frame2;
    cap2 >> frame2;
    if (frame2.empty()) {
        qDebug() << "Camera 2 frame is empty!";
        return;
    }

    cv::Mat hsv;
    cv::Rect expandedBox;
    bool drawCalibrationBox = false;

    // STEP 1: PATTERN MATCHING FOR CALIBRATION (green box)
    if (hasTemplate && !patternMatched1) {
        cv::Mat grayTemplate, grayFrame, smallTemplate, smallFrame;
        cv::cvtColor(templateImage, grayTemplate, cv::COLOR_BGR2GRAY);
        cv::cvtColor(frame2, grayFrame, cv::COLOR_BGR2GRAY);
        cv::resize(grayTemplate, smallTemplate, cv::Size(), 0.5, 0.5);
        cv::resize(grayFrame, smallFrame, cv::Size(), 0.5, 0.5);
        cv::GaussianBlur(smallTemplate, smallTemplate, cv::Size(3, 3), 0);

        cv::Mat result;
        cv::matchTemplate(smallFrame, smallTemplate, result, cv::TM_SQDIFF_NORMED);
        double minVal;
        cv::Point minLoc;
        cv::minMaxLoc(result, &minVal, nullptr, &minLoc, nullptr);

        if (minVal < 0.2) {
            int matchWidth = smallTemplate.cols * 2;
            int matchHeight = smallTemplate.rows * 2;
            lastMatchBox1 = cv::Rect(minLoc.x * 2, minLoc.y * 2, matchWidth, matchHeight);
            patternMatched1 = true;
        }
    }

    // STEP 2: HSV COLOUR DETECTION ? DO THIS BEFORE DRAWING ANYTHING
    cv::cvtColor(frame2, hsv, cv::COLOR_BGR2HSV);

    // Red HSV
    cv::Mat maskRed1, maskRed2;
    cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), maskRed1);
    cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), maskRed2);
    cv::Mat redMask = maskRed1 | maskRed2;

    // Green HSV
    cv::Mat greenMask;
    cv::inRange(hsv, cv::Scalar(35, 100, 70), cv::Scalar(85, 255, 255), greenMask);

    // Blue HSV
    cv::Mat blueMask;
    cv::inRange(hsv, cv::Scalar(100, 150, 70), cv::Scalar(140, 255, 255), blueMask);

    // Combine all masks
    cv::Mat combinedMask = redMask | greenMask | blueMask;

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(combinedMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxArea2 = 0;
    std::vector<cv::Point> largestContour2;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > maxArea2) {
            maxArea2 = area;
            largestContour2 = contour;
        }
    }

    // STEP 3: DETECTION RESULT HANDLING ? NO DRAWING YET
    bool drawObjectBox = false;
    cv::Rect boundingBox2;

    if (maxArea2 > 1000 && !largestContour2.empty()) {
        boundingBox2 = cv::boundingRect(largestContour2);
        drawObjectBox = true;
    }

    // STEP 4: NOW WE DRAW EVERYTHING (AFTER DETECTION IS DONE)

    // Calibration box (green)
    if (patternMatched1) {
        expandedBox = lastMatchBox1;
        expandedBox.x = std::max(0, expandedBox.x - 10);
        expandedBox.y = std::max(0, expandedBox.y - 10);
        expandedBox.width = std::min(frame2.cols - expandedBox.x, expandedBox.width + 18);
        expandedBox.height = std::min(frame2.rows - expandedBox.y, expandedBox.height + 18);

        drawCalibrationBox = true;

        patternW = expandedBox.width;
        patternH = expandedBox.height;

        float scaleValue = 25.0f / patternW;

        cv::rectangle(frame2, expandedBox, cv::Scalar(0, 255, 0), 2); // Green box
        cv::putText(frame2, "W: " + std::to_string(patternW) + " px",
                    cv::Point(expandedBox.x, expandedBox.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame2, "H: " + std::to_string(patternH) + " px",
                    cv::Point(expandedBox.x + expandedBox.width + 5, expandedBox.y + expandedBox.height / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        std::string scaleText = "25mm / width (" + std::to_string(patternW) + ") = " + std::to_string(scaleValue);
        cv::putText(frame2, scaleText, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }

    // Object box (yellow)
    if (drawObjectBox) {
        cv::rectangle(frame2, boundingBox2, cv::Scalar(0, 255, 255), 1);  // Yellow box

        int boxWidth2 = boundingBox2.width;
        int boxHeight2 = boundingBox2.height;

        cv::putText(frame2, "W: " + std::to_string(boxWidth2) + " px",
                    cv::Point(boundingBox2.x, boundingBox2.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

        cv::putText(frame2, "H: " + std::to_string(boxHeight2) + " px",
                    cv::Point(boundingBox2.x + boundingBox2.width + 5, boundingBox2.y + boundingBox2.height / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

        float scaleValueW = 25.0f / patternW;
        float scaleValueH = 25.0f / patternH;

        realWidthMM = boxWidth2 * scaleValueW;
        realHeightMM = boxHeight2 * scaleValueH;

        int realWidthMM_rounded = static_cast<int>(realWidthMM);
        int realHeightMM_rounded = static_cast<int>(realHeightMM);

        cv::putText(frame2, "Width: " + std::to_string(realWidthMM_rounded) + " mm",
                    cv::Point(boundingBox2.x, boundingBox2.y + 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        cv::putText(frame2, "Height: " + std::to_string(realHeightMM_rounded) + " mm",
                    cv::Point(boundingBox2.x + boundingBox2.width + 5, boundingBox2.y + boundingBox2.height / 2 + 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    }

    // Final conversion and display
    cv::cvtColor(frame2, frame2, cv::COLOR_BGR2RGB);
    cv::resize(frame2, frame2, cv::Size(ui->cameraLabel_2->width(), ui->cameraLabel_2->height()));
    QImage qimg2(frame2.data, frame2.cols, frame2.rows, frame2.step, QImage::Format_RGB888);
    ui->cameraLabel_2->setPixmap(QPixmap::fromImage(qimg2));
}



void ConveyorControl::setMotorSpeed(int value)
{
    motorSpeedValue = value;

}
void ConveyorControl::setArmValue(int value)
{
    armValue = value;
    qDebug() << "Arm slider value updated to:" << armValue;

    // You can apply it to a motor like this if needed:
    // if (pca) pca->setPWM(6, 0, armValue);  // example channel
}


void ConveyorControl::applyMotorSpeed(int value)
{
    motorSpeedValue = value;


    if (pca) {
        pca->setPWM(0, 0, motorSpeedValue);
    }
}
