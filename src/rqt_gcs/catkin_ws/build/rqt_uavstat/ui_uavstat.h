/********************************************************************************
** Form generated from reading UI file 'uavstat.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_UAVSTAT_H
#define UI_UAVSTAT_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QProgressBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_widget
{
public:
    QLabel *yawLabel;
    QLabel *rollLabel;
    QLabel *pitchLabel;
    QLabel *altitudeLabel;
    QLabel *horizontalSpaceLabel;
    QLabel *verticalSpaceLabel;
    QLabel *headingLabel;
    QLabel *distanceToWayPointLabel;
    QLabel *batteryLabel;
    QProgressBar *batteryProgressBar;
    QFrame *line;
    QFrame *line_2;
    QLabel *yawDisplay;
    QLabel *rollDisplay;
    QLabel *pitchDisplay;
    QLabel *altitudeDisplay;
    QLabel *verticalSpaceDisplay;
    QLabel *horizontalSpaceDisplay;
    QLabel *headingLabel_2;
    QLabel *waypointDisplay;
    QFrame *line_3;

    void setupUi(QWidget *widget)
    {
        if (widget->objectName().isEmpty())
            widget->setObjectName(QString::fromUtf8("widget"));
        widget->resize(400, 300);
        widget->setMinimumSize(QSize(400, 300));
        yawLabel = new QLabel(widget);
        yawLabel->setObjectName(QString::fromUtf8("yawLabel"));
        yawLabel->setGeometry(QRect(40, 20, 41, 16));
        rollLabel = new QLabel(widget);
        rollLabel->setObjectName(QString::fromUtf8("rollLabel"));
        rollLabel->setGeometry(QRect(180, 20, 41, 17));
        pitchLabel = new QLabel(widget);
        pitchLabel->setObjectName(QString::fromUtf8("pitchLabel"));
        pitchLabel->setGeometry(QRect(310, 20, 51, 17));
        altitudeLabel = new QLabel(widget);
        altitudeLabel->setObjectName(QString::fromUtf8("altitudeLabel"));
        altitudeLabel->setGeometry(QRect(20, 80, 71, 20));
        horizontalSpaceLabel = new QLabel(widget);
        horizontalSpaceLabel->setObjectName(QString::fromUtf8("horizontalSpaceLabel"));
        horizontalSpaceLabel->setGeometry(QRect(300, 80, 81, 20));
        verticalSpaceLabel = new QLabel(widget);
        verticalSpaceLabel->setObjectName(QString::fromUtf8("verticalSpaceLabel"));
        verticalSpaceLabel->setGeometry(QRect(160, 80, 81, 20));
        headingLabel = new QLabel(widget);
        headingLabel->setObjectName(QString::fromUtf8("headingLabel"));
        headingLabel->setGeometry(QRect(20, 160, 81, 20));
        distanceToWayPointLabel = new QLabel(widget);
        distanceToWayPointLabel->setObjectName(QString::fromUtf8("distanceToWayPointLabel"));
        distanceToWayPointLabel->setGeometry(QRect(190, 160, 191, 20));
        batteryLabel = new QLabel(widget);
        batteryLabel->setObjectName(QString::fromUtf8("batteryLabel"));
        batteryLabel->setGeometry(QRect(130, 240, 121, 20));
        batteryProgressBar = new QProgressBar(widget);
        batteryProgressBar->setObjectName(QString::fromUtf8("batteryProgressBar"));
        batteryProgressBar->setGeometry(QRect(30, 270, 331, 23));
        batteryProgressBar->setValue(24);
        line = new QFrame(widget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(0, 70, 391, 16));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        line_2 = new QFrame(widget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(0, 150, 391, 16));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        yawDisplay = new QLabel(widget);
        yawDisplay->setObjectName(QString::fromUtf8("yawDisplay"));
        yawDisplay->setGeometry(QRect(20, 40, 67, 17));
        yawDisplay->setFrameShape(QFrame::StyledPanel);
        rollDisplay = new QLabel(widget);
        rollDisplay->setObjectName(QString::fromUtf8("rollDisplay"));
        rollDisplay->setGeometry(QRect(160, 40, 67, 17));
        rollDisplay->setFrameShape(QFrame::StyledPanel);
        pitchDisplay = new QLabel(widget);
        pitchDisplay->setObjectName(QString::fromUtf8("pitchDisplay"));
        pitchDisplay->setGeometry(QRect(300, 40, 67, 17));
        pitchDisplay->setFrameShape(QFrame::StyledPanel);
        altitudeDisplay = new QLabel(widget);
        altitudeDisplay->setObjectName(QString::fromUtf8("altitudeDisplay"));
        altitudeDisplay->setGeometry(QRect(20, 110, 67, 17));
        altitudeDisplay->setFrameShape(QFrame::StyledPanel);
        verticalSpaceDisplay = new QLabel(widget);
        verticalSpaceDisplay->setObjectName(QString::fromUtf8("verticalSpaceDisplay"));
        verticalSpaceDisplay->setGeometry(QRect(160, 110, 67, 17));
        verticalSpaceDisplay->setFrameShape(QFrame::StyledPanel);
        horizontalSpaceDisplay = new QLabel(widget);
        horizontalSpaceDisplay->setObjectName(QString::fromUtf8("horizontalSpaceDisplay"));
        horizontalSpaceDisplay->setGeometry(QRect(300, 110, 67, 17));
        horizontalSpaceDisplay->setFrameShape(QFrame::StyledPanel);
        headingLabel_2 = new QLabel(widget);
        headingLabel_2->setObjectName(QString::fromUtf8("headingLabel_2"));
        headingLabel_2->setGeometry(QRect(20, 190, 67, 17));
        headingLabel_2->setFrameShape(QFrame::StyledPanel);
        waypointDisplay = new QLabel(widget);
        waypointDisplay->setObjectName(QString::fromUtf8("waypointDisplay"));
        waypointDisplay->setGeometry(QRect(240, 190, 67, 17));
        waypointDisplay->setFrameShape(QFrame::StyledPanel);
        line_3 = new QFrame(widget);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(0, 210, 391, 16));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);

        retranslateUi(widget);

        QMetaObject::connectSlotsByName(widget);
    } // setupUi

    void retranslateUi(QWidget *widget)
    {
        widget->setWindowTitle(QApplication::translate("widget", "Form", 0, QApplication::UnicodeUTF8));
        yawLabel->setText(QApplication::translate("widget", "YAW", 0, QApplication::UnicodeUTF8));
        rollLabel->setText(QApplication::translate("widget", "ROLL", 0, QApplication::UnicodeUTF8));
        pitchLabel->setText(QApplication::translate("widget", "PITCH", 0, QApplication::UnicodeUTF8));
        altitudeLabel->setText(QApplication::translate("widget", "ALTITUDE", 0, QApplication::UnicodeUTF8));
        horizontalSpaceLabel->setText(QApplication::translate("widget", "HORZ. SP.", 0, QApplication::UnicodeUTF8));
        verticalSpaceLabel->setText(QApplication::translate("widget", "VERT. SP.", 0, QApplication::UnicodeUTF8));
        headingLabel->setText(QApplication::translate("widget", "HEADING", 0, QApplication::UnicodeUTF8));
        distanceToWayPointLabel->setText(QApplication::translate("widget", "DISTANCE TO WAYPOINT", 0, QApplication::UnicodeUTF8));
        batteryLabel->setText(QApplication::translate("widget", "BATTERY LEVEL", 0, QApplication::UnicodeUTF8));
        yawDisplay->setText(QApplication::translate("widget", "0", 0, QApplication::UnicodeUTF8));
        rollDisplay->setText(QApplication::translate("widget", "0", 0, QApplication::UnicodeUTF8));
        pitchDisplay->setText(QApplication::translate("widget", "0", 0, QApplication::UnicodeUTF8));
        altitudeDisplay->setText(QApplication::translate("widget", "0", 0, QApplication::UnicodeUTF8));
        verticalSpaceDisplay->setText(QApplication::translate("widget", "0", 0, QApplication::UnicodeUTF8));
        horizontalSpaceDisplay->setText(QApplication::translate("widget", "0", 0, QApplication::UnicodeUTF8));
        headingLabel_2->setText(QApplication::translate("widget", "0", 0, QApplication::UnicodeUTF8));
        waypointDisplay->setText(QApplication::translate("widget", "0", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class widget: public Ui_widget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_UAVSTAT_H
