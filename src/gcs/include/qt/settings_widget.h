/*
 * File:   settings_widget.h
 * Author: serl
 *
 * Created on June 7, 2016, 3:40 PM
 */

#ifndef _SETTINGSWIDGET_H
#define _SETTINGSWIDGET_H

#include "qt/vehicle_manager.h"
#include "ui_SettingsWidget.h"
#include <QSettings>

namespace gcs
{

    class GCSMainWindow;
    
    class SettingsWidget : public QWidget
    {
        Q_OBJECT
    public:
        SettingsWidget(VehicleManager * vm);
        virtual ~SettingsWidget();
        
    private:
        Ui::SettingsWidget widget;
        QString ml_state;
        QString coordinate_system;
        VehicleManager *vm;
        QSettings *settings; 
        
        ObjectDetectionParameters* od_params;

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
        void onToggleLengthLine();
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
