/*
 * File:   settings_widget.h
 * Author: serl
 *
 * Created on June 7, 2016, 3:40 PM
 */

#ifndef _SETTINGSWIDGET_H
#define _SETTINGSWIDGET_H

#include "rqt_gcs/simple_gcs.h"
#include "rqt_gcs/my_q_widget.h"
#include "ui_SettingsWidget.h"

namespace rqt_gcs
{

    class SimpleGCS;
    
    class SettingsWidget : public MyQWidget
    {
        Q_OBJECT
    public:
        SettingsWidget(SimpleGCS *);
        virtual ~SettingsWidget();
        
    private:
        Ui::SettingsWidget widget_;
        QString ml_state_;
        SimpleGCS * gcs;
        
        void setToolTips();
        
        void readGeneralSettings();
        void writeGeneralSettings();
        bool validateGeneralSettings();
        
        void readObjectDetectionSettings();
        void writeObjectDetectionSettings();
        
        
    signals:
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
