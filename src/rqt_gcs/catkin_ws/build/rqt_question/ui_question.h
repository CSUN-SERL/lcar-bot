/********************************************************************************
** Form generated from reading UI file 'question.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QUESTION_H
#define UI_QUESTION_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGraphicsView>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_widget
{
public:
    QGraphicsView *uavPhoto;
    QLabel *photoLabel;
    QLabel *label_2;
    QLabel *label_3;
    QRadioButton *yesRadioButton;
    QRadioButton *noRadioButton;
    QLineEdit *objectName;
    QPushButton *questionSubmit;

    void setupUi(QWidget *widget)
    {
        if (widget->objectName().isEmpty())
            widget->setObjectName(QString::fromUtf8("widget"));
        widget->resize(422, 596);
        widget->setMinimumSize(QSize(422, 596));
        uavPhoto = new QGraphicsView(widget);
        uavPhoto->setObjectName(QString::fromUtf8("uavPhoto"));
        uavPhoto->setGeometry(QRect(10, 60, 401, 301));
        photoLabel = new QLabel(widget);
        photoLabel->setObjectName(QString::fromUtf8("photoLabel"));
        photoLabel->setGeometry(QRect(30, 30, 67, 17));
        label_2 = new QLabel(widget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(30, 390, 271, 16));
        label_3 = new QLabel(widget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(30, 460, 131, 17));
        yesRadioButton = new QRadioButton(widget);
        yesRadioButton->setObjectName(QString::fromUtf8("yesRadioButton"));
        yesRadioButton->setGeometry(QRect(70, 420, 117, 22));
        noRadioButton = new QRadioButton(widget);
        noRadioButton->setObjectName(QString::fromUtf8("noRadioButton"));
        noRadioButton->setGeometry(QRect(260, 420, 117, 22));
        objectName = new QLineEdit(widget);
        objectName->setObjectName(QString::fromUtf8("objectName"));
        objectName->setGeometry(QRect(30, 490, 331, 27));
        questionSubmit = new QPushButton(widget);
        questionSubmit->setObjectName(QString::fromUtf8("questionSubmit"));
        questionSubmit->setGeometry(QRect(250, 550, 99, 27));

        retranslateUi(widget);

        QMetaObject::connectSlotsByName(widget);
    } // setupUi

    void retranslateUi(QWidget *widget)
    {
        widget->setWindowTitle(QApplication::translate("widget", "Form", 0, QApplication::UnicodeUTF8));
        photoLabel->setText(QApplication::translate("widget", "Photo", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("widget", "Question", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("widget", "If No, What is it?", 0, QApplication::UnicodeUTF8));
        yesRadioButton->setText(QApplication::translate("widget", "Yes", 0, QApplication::UnicodeUTF8));
        noRadioButton->setText(QApplication::translate("widget", "No", 0, QApplication::UnicodeUTF8));
        questionSubmit->setText(QApplication::translate("widget", "Submit", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class widget: public Ui_widget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QUESTION_H
