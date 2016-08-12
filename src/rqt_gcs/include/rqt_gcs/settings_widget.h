/*
 * File:   settings_widget.h
 * Author: serl
 *
 * Created on June 7, 2016, 3:40 PM
 */

#ifndef _SETTINGSWIDGET_H
#define _SETTINGSWIDGET_H

#include<rqt_gcs/simple_gcs.h>
#include "ui_SettingsWidget.h"
#include <QtCore/QSettings>


namespace rqt_gcs
{

    class SimpleGCS;
    
    class SettingsWidget : public QWidget
    {
        Q_OBJECT
    public:
        SettingsWidget(SimpleGCS *);
        virtual ~SettingsWidget();

    private:
        Ui::SettingsWidget widget_;
        //QSettings *settings_;
        QString ml_state_;
        SimpleGCS * gcs;
        
        struct ObjectDetectionParams // object detection parameters
        {
            double hit_thresh; // displayed as a decimal
            int step_size;
            int padding;
            double scale_factor; // displayed as a decimal
            bool mean_shift;
        } obj_params;
        
        void setToolTips();
        
        void readGeneralSettings();
        //void readObjectDetectionSettings();
        
        void writeGeneralSettings();
        //void writeObjectDetectionSettings();
        
        bool validateGeneralSettings();
        //bool validateObjectDetectionSettings();

        void updateHitThreshold(double);
        
    signals:
        void dismissMe();
        void machineLearningModeToggled(bool);

    private slots:
        void applyClicked();
        void cancelClicked();
        void toggleFrequencyGroup();
        void toggleIntervalTextBox();
        void toggleLengthTextBox();
        
        //object detection tab sliders
        void onHitThresholdSliderChange(int); // convert to double
        void onHitThresholdLineChange(QString);
        
        void onStepSizeSliderChange(int);
        void onStepSizeLineChange(QString);
        
        void updatePadding(int);
        void updateScaleFactor(int); // convert to double
        void updateMeanShiftGrouping(bool);

    };

} // end name space
#endif /* _SETTINGSWIDGET_H */
