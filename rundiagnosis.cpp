#include "rundiagnosis.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPixmap>
#include <QTimer>
#include <QDebug>
#include <QScreen>
#include <QApplication>


RunDiagnosis::RunDiagnosis(QWidget *parent)
    : QDialog(parent), currentImageIndex(0)
{
    setWindowFlags(Qt::Window);
    showFullScreen();

    // === Main image label ===
    imageLabel = new QLabel(this);
    imageLabel->setAlignment(Qt::AlignCenter);
    imageLabel->setScaledContents(true);
    imageLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // === Buttons ===
    emergencyStopButton = new QPushButton("");
    diagnosticsButton = new QPushButton("Run Diagnostics");
    returnButton = new QPushButton("");

    emergencyStopButton->setFixedSize(300, 300);
    emergencyStopButton->setIcon(QIcon(":/images/stop_resized.jpg"));
    emergencyStopButton->setIconSize(QSize(300, 300));



    diagnosticsButton->setFixedSize(300, 300);

    returnButton->setFixedSize(300, 300);
    returnButton->setIcon(QIcon(":/images/exit_resized-removebg-preview.png"));
    returnButton->setIconSize(QSize(300, 300));

    returnButton->setVisible(true);

    QString buttonStyle = "QPushButton { background: none; border: none; }";
    QString buttonStyle2 = "QPushButton { background: none; border: 2px solid rgba(0, 0, 0, 0.4); border-radius: 12px;  }";
    emergencyStopButton->setStyleSheet(buttonStyle);
    diagnosticsButton->setStyleSheet(buttonStyle2);
    returnButton->setStyleSheet(buttonStyle2);


    // === Button layout ===
    QVBoxLayout *buttonLayout = new QVBoxLayout();
    buttonLayout->addWidget(emergencyStopButton);
    buttonLayout->addWidget(diagnosticsButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(returnButton);
    buttonLayout->setContentsMargins(30, 30, 30, 30);

    QWidget *buttonContainer = new QWidget(this);
    buttonContainer->setLayout(buttonLayout);
    buttonContainer->setStyleSheet("background-color: rgba(0, 0, 0, 100);");
    buttonContainer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);

    // === Layout image and buttons ===
    QHBoxLayout *mainLayout = new QHBoxLayout(this);
    mainLayout->addWidget(buttonContainer);
    mainLayout->addWidget(imageLabel);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    setLayout(mainLayout);

    // === Load images ===
    imagePaths << ":/images/0.png";
    for (int i = 1; i <= 14; ++i) {
        imagePaths << QString(":/images/%1.jpg").arg(i);
    }

    // === Button connections ===
    connect(emergencyStopButton, &QPushButton::clicked, this, &RunDiagnosis::emergencyStop);
    connect(diagnosticsButton, &QPushButton::clicked, this, [=]() {
        currentImageIndex = 1;
        returnButton->setVisible(false);
        imageTimer->start(2000);
    });
    connect(returnButton, &QPushButton::clicked, this, &RunDiagnosis::returnToMainMenu);

    // === Image slideshow timer ===
    imageTimer = new QTimer(this);
    connect(imageTimer, &QTimer::timeout, this, &RunDiagnosis::showNextImage);
    QTimer::singleShot(100, this, [=]() {
        QPixmap pix(imagePaths[0]);
        imageLabel->setPixmap(pix.scaled(imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    });

}

RunDiagnosis::~RunDiagnosis() {}

void RunDiagnosis::showNextImage()
{
    if (currentImageIndex >= imagePaths.size()) {
        imageTimer->stop();
        returnButton->setVisible(true);
        return;
    }

    QPixmap pix(imagePaths[currentImageIndex]);
    imageLabel->setPixmap(pix.scaled(imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    currentImageIndex++;
}

void RunDiagnosis::returnToMainMenu()
{
    close();
}

void RunDiagnosis::emergencyStop()
{
    for (int i = 0; i < 6; i++) {
        pca->setPWM(i, 0, 0);  // turn off all channels
    }
    QApplication::quit();  // Ends the whole app
}

void RunDiagnosis::setPCA9685(PCA9685* controller)
{
    pca = controller;
}
