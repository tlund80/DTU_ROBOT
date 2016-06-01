/********************************************************************************
** Form generated from reading UI file 'AnnotateDialog.ui'
**
** Created: Mon Mar 11 20:19:55 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ANNOTATEDIALOG_H
#define UI_ANNOTATEDIALOG_H

#include <QtCore/QVariant>

#if QT_VERSION >= 0x050000
    #include <QAction>
    #include <QApplication>
    #include <QButtonGroup>
    #include <QDialog>
    #include <QHBoxLayout>
    #include <QHeaderView>
    #include <QLabel>
    #include <QPushButton>
    #include <QVBoxLayout>
#else
    #include <QtGui/QAction>
    #include <QtGui/QApplication>
    #include <QtGui/QButtonGroup>
    #include <QtGui/QDialog>
    #include <QtGui/QHBoxLayout>
    #include <QtGui/QHeaderView>
    #include <QtGui/QLabel>
    #include <QtGui/QPushButton>
    #include <QtGui/QVBoxLayout>
#endif

#include <QDesktopWidget>
#include <iostream>

#define DIALOG_SIZE_PERCENT 0.95
QT_BEGIN_NAMESPACE

class Ui_annotationDialog
{
public:
    QVBoxLayout *verticalLayout;
    QLabel *_imgLabel;
    QHBoxLayout *horizontalLayout;
    QPushButton *btnOk;
    QPushButton *btnSkip;

    QDialog* _annotationDialog;
    QRect res;


    void setupUi(QDialog *annotationDialog)
    {
         // Get primary screen resolution
         res = QApplication::desktop()->availableGeometry( QApplication::desktop()->primaryScreen());


        if (annotationDialog->objectName().isEmpty())
            annotationDialog->setObjectName(QString::fromUtf8("annotationDialog"));
        annotationDialog->setMaximumSize(res.width()*DIALOG_SIZE_PERCENT,res.height()*DIALOG_SIZE_PERCENT);
        annotationDialog->setModal(false);

        _imgLabel = new QLabel(annotationDialog);
        _imgLabel->setObjectName(QString::fromUtf8("_imgLabel"));

        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(_imgLabel->sizePolicy().hasHeightForWidth());

        _imgLabel->setSizePolicy(sizePolicy);

        _imgLabel->setContextMenuPolicy(Qt::NoContextMenu);
        _imgLabel->setFrameShape(QFrame::NoFrame);
        _imgLabel->setMaximumSize(res.width()*DIALOG_SIZE_PERCENT,res.height()*DIALOG_SIZE_PERCENT);
        _imgLabel->setMinimumSize(400,300);

      //  _imgLabel->setMinimumSize(res.width()*0.4,res.width()*0.4);

        _imgLabel->setBackgroundRole( QPalette::Base );
        _imgLabel->setAlignment( Qt::AlignTop | Qt::AlignLeft );

   //     annotationDialog->setSizePolicy(sizePolicy);
     //   annotationDialog->setFixedSize(r.width()/2, r.height()/2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));

        //Setup OK Button
        btnOk = new QPushButton(annotationDialog);
        btnOk->setObjectName(QString::fromUtf8("btnOk"));
        horizontalLayout->addWidget(btnOk);

        //Setup Skip Button
        btnSkip = new QPushButton(annotationDialog);
        btnSkip->setObjectName(QString::fromUtf8("btnSkip"));
        horizontalLayout->addWidget(btnSkip);

        verticalLayout = new QVBoxLayout(annotationDialog);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->addWidget(_imgLabel);
        verticalLayout->addLayout(horizontalLayout);

        retranslateUi(annotationDialog);
        QObject::connect(btnSkip, SIGNAL(released()), annotationDialog, SLOT(close()));

        QMetaObject::connectSlotsByName(annotationDialog);

        _annotationDialog = annotationDialog;
    } // setupUi

    void retranslateUi(QDialog *annotationDialog)
    {
        #if QT_VERSION >= 0x050000
        annotationDialog->setWindowTitle(QApplication::translate("annotationDialog", "Dialog", 0));
        _imgLabel->setText(QString());
        btnOk->setText(QApplication::translate("annotationDialog", "Apply Roi", 0));
        btnSkip->setText(QApplication::translate("annotationDialog", "Skip image", 0));
        #else
        annotationDialog->setWindowTitle(QApplication::translate("annotationDialog", "Dialog", 0, QApplication::UnicodeUTF8));
        _imgLabel->setText(QString());
        btnOk->setText(QApplication::translate("annotationDialog", "Apply Roi", 0, QApplication::UnicodeUTF8));
        btnSkip->setText(QApplication::translate("annotationDialog", "Skip image", 0, QApplication::UnicodeUTF8));

        #endif
    } // retranslateUi

    void resize(){
        _imgLabel->setFixedSize(res.width()*DIALOG_SIZE_PERCENT, res.height()*DIALOG_SIZE_PERCENT);
        _imgLabel->adjustSize();
    }
};

namespace Ui {
    class annotationDialog: public Ui_annotationDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ANNOTATEDIALOG_H
