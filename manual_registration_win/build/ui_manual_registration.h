/********************************************************************************
** Form generated from reading UI file 'manual_registration.ui'
**
** Created by: Qt User Interface Compiler version 5.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MANUAL_REGISTRATION_H
#define UI_MANUAL_REGISTRATION_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionClose;
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QHBoxLayout *horizontalLayout;
    QLineEdit *textModelPath;
    QToolButton *btnModel;
    QVTKWidget *qvtk_widget_src;
    QVBoxLayout *verticalLayout_3;
    QPushButton *calculateButton;
    QCheckBox *robustBox;
    QCheckBox *refineBox;
    QCheckBox *cbDownsample;
    QPushButton *clearButton;
    QPushButton *btnApplyTrfm;
    QSpacerItem *verticalSpacer;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_2;
    QHBoxLayout *horizontalLayout_2;
    QLineEdit *textScenePath;
    QToolButton *btnScene;
    QVTKWidget *qvtk_widget_dst;
    QStatusBar *statusBar;
    QMenuBar *menuBar;
    QMenu *menuFile;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1188, 966);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(1);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setIconSize(QSize(22, 22));
        actionClose = new QAction(MainWindow);
        actionClose->setObjectName(QStringLiteral("actionClose"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        centralwidget->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(255);
        sizePolicy1.setVerticalStretch(255);
        sizePolicy1.setHeightForWidth(centralwidget->sizePolicy().hasHeightForWidth());
        centralwidget->setSizePolicy(sizePolicy1);
        horizontalLayout_3 = new QHBoxLayout(centralwidget);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        label = new QLabel(centralwidget);
        label->setObjectName(QStringLiteral("label"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy2);
        QFont font;
        font.setBold(true);
        font.setUnderline(false);
        font.setWeight(75);
        label->setFont(font);

        verticalLayout->addWidget(label);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        textModelPath = new QLineEdit(centralwidget);
        textModelPath->setObjectName(QStringLiteral("textModelPath"));
        textModelPath->setMinimumSize(QSize(500, 20));

        horizontalLayout->addWidget(textModelPath);

        btnModel = new QToolButton(centralwidget);
        btnModel->setObjectName(QStringLiteral("btnModel"));
        QSizePolicy sizePolicy3(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(btnModel->sizePolicy().hasHeightForWidth());
        btnModel->setSizePolicy(sizePolicy3);

        horizontalLayout->addWidget(btnModel);


        verticalLayout->addLayout(horizontalLayout);

        qvtk_widget_src = new QVTKWidget(centralwidget);
        qvtk_widget_src->setObjectName(QStringLiteral("qvtk_widget_src"));
        sizePolicy1.setHeightForWidth(qvtk_widget_src->sizePolicy().hasHeightForWidth());
        qvtk_widget_src->setSizePolicy(sizePolicy1);
        qvtk_widget_src->setMinimumSize(QSize(530, 860));
        qvtk_widget_src->setStyleSheet(QStringLiteral("background-color: rgb(0, 0, 0);"));

        verticalLayout->addWidget(qvtk_widget_src);


        horizontalLayout_3->addLayout(verticalLayout);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        calculateButton = new QPushButton(centralwidget);
        calculateButton->setObjectName(QStringLiteral("calculateButton"));

        verticalLayout_3->addWidget(calculateButton);

        robustBox = new QCheckBox(centralwidget);
        robustBox->setObjectName(QStringLiteral("robustBox"));
        robustBox->setChecked(true);

        verticalLayout_3->addWidget(robustBox);

        refineBox = new QCheckBox(centralwidget);
        refineBox->setObjectName(QStringLiteral("refineBox"));
        refineBox->setChecked(true);

        verticalLayout_3->addWidget(refineBox);

        cbDownsample = new QCheckBox(centralwidget);
        cbDownsample->setObjectName(QStringLiteral("cbDownsample"));
        cbDownsample->setChecked(true);

        verticalLayout_3->addWidget(cbDownsample);

        clearButton = new QPushButton(centralwidget);
        clearButton->setObjectName(QStringLiteral("clearButton"));

        verticalLayout_3->addWidget(clearButton);

        btnApplyTrfm = new QPushButton(centralwidget);
        btnApplyTrfm->setObjectName(QStringLiteral("btnApplyTrfm"));

        verticalLayout_3->addWidget(btnApplyTrfm);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);


        horizontalLayout_3->addLayout(verticalLayout_3);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        sizePolicy2.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy2);
        QFont font1;
        font1.setBold(true);
        font1.setWeight(75);
        label_2->setFont(font1);

        verticalLayout_2->addWidget(label_2);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        textScenePath = new QLineEdit(centralwidget);
        textScenePath->setObjectName(QStringLiteral("textScenePath"));
        textScenePath->setMinimumSize(QSize(500, 20));

        horizontalLayout_2->addWidget(textScenePath);

        btnScene = new QToolButton(centralwidget);
        btnScene->setObjectName(QStringLiteral("btnScene"));

        horizontalLayout_2->addWidget(btnScene);


        verticalLayout_2->addLayout(horizontalLayout_2);

        qvtk_widget_dst = new QVTKWidget(centralwidget);
        qvtk_widget_dst->setObjectName(QStringLiteral("qvtk_widget_dst"));
        sizePolicy1.setHeightForWidth(qvtk_widget_dst->sizePolicy().hasHeightForWidth());
        qvtk_widget_dst->setSizePolicy(sizePolicy1);
        qvtk_widget_dst->setMinimumSize(QSize(530, 860));
        qvtk_widget_dst->setStyleSheet(QStringLiteral("background-color: rgb(0, 0, 0);"));

        verticalLayout_2->addWidget(qvtk_widget_dst);


        horizontalLayout_3->addLayout(verticalLayout_2);

        MainWindow->setCentralWidget(centralwidget);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        QSizePolicy sizePolicy4(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(statusBar->sizePolicy().hasHeightForWidth());
        statusBar->setSizePolicy(sizePolicy4);
        MainWindow->setStatusBar(statusBar);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1188, 21));
        QSizePolicy sizePolicy5(QSizePolicy::Expanding, QSizePolicy::Minimum);
        sizePolicy5.setHorizontalStretch(0);
        sizePolicy5.setVerticalStretch(0);
        sizePolicy5.setHeightForWidth(menuBar->sizePolicy().hasHeightForWidth());
        menuBar->setSizePolicy(sizePolicy5);
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        MainWindow->setMenuBar(menuBar);

        menuBar->addAction(menuFile->menuAction());
        menuFile->addAction(actionClose);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QString());
        actionClose->setText(QApplication::translate("MainWindow", "Close", 0));
        label->setText(QApplication::translate("MainWindow", "Model", 0));
        btnModel->setText(QApplication::translate("MainWindow", "...", 0));
        calculateButton->setText(QApplication::translate("MainWindow", "Calculate pose", 0));
        robustBox->setText(QApplication::translate("MainWindow", "Robust", 0));
        refineBox->setText(QApplication::translate("MainWindow", "Refine", 0));
        cbDownsample->setText(QApplication::translate("MainWindow", "Downsample", 0));
        clearButton->setText(QApplication::translate("MainWindow", "Clear", 0));
        btnApplyTrfm->setText(QApplication::translate("MainWindow", "Apply Transform", 0));
        label_2->setText(QApplication::translate("MainWindow", "Scene", 0));
        btnScene->setText(QApplication::translate("MainWindow", "...", 0));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MANUAL_REGISTRATION_H
