#ifndef RUNDIAGNOSIS_H
#define RUNDIAGNOSIS_H

#include <QDialog>
#include <QTimer>
#include <QLabel>
#include <QPushButton>
#include "pca9685.h"

class RunDiagnosis : public QDialog
{
    Q_OBJECT

public:
    ~RunDiagnosis();
    explicit RunDiagnosis(QWidget *parent = nullptr);
    void setPCA9685(PCA9685* controller);  // ? Add this

private slots:
    void showNextImage();
    void returnToMainMenu();

private:
    QLabel *imageLabel;
    QPushButton *emergencyStopButton;
    QPushButton *diagnosticsButton;
    QPushButton *returnButton;
    QStringList imagePaths;
    QTimer *imageTimer;
    int currentImageIndex;
    PCA9685* pca;

    void emergencyStop();


};

#endif // RUNDIAGNOSIS_H
