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
#include <QtCore/qmath.h>


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
            //defaults
            double hit_thresh = 0.45; // displayed as a decimal
            int step_size = 8;
            int padding = 0;
            double scale_factor = 1.15; // displayed as a decimal
            bool mean_shift = true;
        } od_params;
        
        void setToolTips();
        
        void readGeneralSettings();
        void writeGeneralSettings();
        bool validateGeneralSettings();
        
        void readObjectDetectionSettings();
        void writeObjectDetectionSettings();
        
        
    signals:
        void dismissMe();
        void machineLearningModeToggled(bool);

    private slots:
        bool onApplyClicked();
        void onOkClicked();
        void onCancelClicked();
        void onToggleFrequencyGroup();
        void onToggleIntervalLine();
        void onToggleLengthLine();
        
        //object detection tab sliders and line_edits
        void onHitThresholdSliderChange(int);
        void onHitThresholdLineChange();
        
        void onStepSizeSliderChange(int);
        void onStepSizeLineChange();
        
        void onPaddingSliderChange(int);
        void onPaddingLineChange();
        
        void onScaleFactorSliderChange(int);
        void onScaleFactorLineChange();
        
        void onMeanShiftRadioChange();

    };

} // end name space
#endif /* _SETTINGSWIDGET_H */
