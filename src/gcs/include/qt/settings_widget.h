/*
 * File:   settings_widget.h
 * Author: serl
 *
 * Created on June 7, 2016, 3:40 PM
 */

#ifndef _SETTINGSWIDGET_H
#define _SETTINGSWIDGET_H

#include "ui_SettingsWidget.h"

#include "util/settings.h"
#include "util/object_detection_parameters.h"

namespace gcs
{

    class SettingsWidget : public QWidget
    {
        Q_OBJECT
    public:
        SettingsWidget();
        virtual ~SettingsWidget();
        
    private:
        Ui::SettingsWidget widget;
        QString ml_state;
        QString coordinate_system;
        QString image_root_dir;
        Settings settings; 
        
        ObjectDetectionParameters od_params;

        void setToolTips();
        
        void readGeneralSettings();
        void writeGeneralSettings();
        bool validateGeneralSettings();
        
        void readObjectDetectionSettings();
        void writeObjectDetectionSettings();

    private slots:
        bool onApplyClicked();
        void onOkClicked();
        void onCancelClicked();
        void onToggleFrequencyGroup();
        void onToggleIntervalLine();
        void onToggleDurationLine();
        void OnCoordinateSystemChange();
        
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
