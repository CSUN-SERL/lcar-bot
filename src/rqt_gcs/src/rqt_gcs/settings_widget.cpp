/*
 * File:   settings_widget.cpp
 * Author: serl
 *
 * Created on June 7, 2016, 3:40 PM
 */

#include "rqt_gcs/settings_widget.h"
#include <iostream>
#include <assert.h>

namespace rqt_gcs
{

    SettingsWidget::SettingsWidget(SimpleGCS * sgcs) :
    gcs(sgcs)
    {
        this->setAttribute(Qt::WA_DeleteOnClose);
        
        widget_.setupUi(this);

        //apply and cancel buttons
        connect(widget_.apply_btn, &QPushButton::clicked,
                this, &SettingsWidget::onApplyClicked);
        
        connect(widget_.ok_btn, &QPushButton::clicked,
                this, &SettingsWidget::onOkClicked);

        connect(widget_.cancel_btn, &QPushButton::clicked,
                this, &SettingsWidget::onCancelClicked);

        //nominal, marginal, poor radio buttons enable or disable frequency group
        connect(widget_.nominal_btn, &QRadioButton::clicked,
                this, &SettingsWidget::onToggleFrequencyGroup);

        connect(widget_.marginal_btn, &QRadioButton::clicked,
                this,&SettingsWidget::onToggleFrequencyGroup);

        connect(widget_.poor_btn, &QRadioButton::clicked,
                this, &SettingsWidget::onToggleFrequencyGroup);


        //interval, random radio buttons enable or disable interval text edit field
        connect(widget_.interval_btn, &QRadioButton::clicked,
                this, &SettingsWidget::onToggleIntervalLine);

        connect(widget_.random_btn, &QRadioButton::clicked,
                this, &SettingsWidget::onToggleIntervalLine);


        //frequency/length check box enables the length text edit field
        connect(widget_.length_check_box, &QRadioButton::clicked,
                this, &SettingsWidget::onToggleLengthLine);

        
        //object detection parameter inits and slider/line_edit connections
        //hit threshold
        widget_.sl_hit_thresh->setMinimum(0); // divided by 100 to get actual value
        widget_.sl_hit_thresh->setMaximum(90);
        
        connect(widget_.sl_hit_thresh, &QSlider::valueChanged, 
                this, &SettingsWidget::onHitThresholdSliderChange);
        
        connect(widget_.line_edit_hit_thresh, &QLineEdit::returnPressed,
                this, &SettingsWidget::onHitThresholdLineChange);
        
        //step size
        widget_.sl_step_size->setMinimum(1); // multipied by 4 to get actual value
        widget_.sl_step_size->setMaximum(4);
        
        connect(widget_.sl_step_size, &QSlider::valueChanged,
                this, &SettingsWidget::onStepSizeSliderChange);
        
        connect(widget_.line_edit_step_size, &QLineEdit::returnPressed,
                this, &SettingsWidget::onStepSizeLineChange);
        
        //padding
        widget_.sl_padding->setMinimum(0); // multiplied by 8 to get actual value
        widget_.sl_padding->setMaximum(4);
        
        connect(widget_.sl_padding, &QSlider::valueChanged,
                this, &SettingsWidget::onPaddingSliderChange);
        
        connect(widget_.line_edit_padding, &QLineEdit::returnPressed,
                this, &SettingsWidget::onPaddingLineChange);
        
        //scale factor
        widget_.sl_scale_factor->setMinimum(105); // divided by 100 to get actual value
        widget_.sl_scale_factor->setMaximum(130);
        
        connect(widget_.sl_scale_factor, &QSlider::valueChanged,
                this, &SettingsWidget::onScaleFactorSliderChange);
        
        connect(widget_.line_edit_scale_factor, &QLineEdit::returnPressed,
                this, &SettingsWidget::onScaleFactorLineChange);
        
        //mean shift grouping
        connect(widget_.radio_on_mean_shift, &QRadioButton::clicked,
                this, &SettingsWidget::onMeanShiftRadioChange);
        
        connect(widget_.radio_off_mean_shift, &QRadioButton::clicked,
                this, &SettingsWidget::onMeanShiftRadioChange);
        
        setToolTips();
        readGeneralSettings();
        readObjectDetectionSettings();
    }
    
    void SettingsWidget::closeEvent(QCloseEvent* event)
    {
        gcs->widgets_.settings_ = nullptr;
        event->accept();
    }

    SettingsWidget::~SettingsWidget()
    {
       gcs = nullptr;
    }

    void SettingsWidget::setToolTips()
    {
        widget_.group_obj_dtct_params->setToolTip("These parameters affect the behavior of the object detection node associated with each UAV.\n"
                                                  "Mouse over each one for a description.");
        widget_.group_obj_dtct_params->setToolTipDuration(10000);
        
        widget_.lblHitThreshold->setToolTip("Controls the maximum Euclidian distance between the input HOG\n"
                                            "features and the classifying plane of the Support Vector Machine.\n"
                                            "Range: 0.0 - 0.9");
        widget_.lblHitThreshold->setToolTipDuration(10000);
        
        widget_.lblStepSize->setToolTip("Controls the number of pixels that the detection window jumps forward.\n"
                                        "Range: 4, 8, 12 or 16");
        widget_.lblStepSize->setToolTipDuration(10000);
        
        widget_.lblPadding->setToolTip("Controls the amount of padding around the region of interest that will\n"
                                       "be added before feature extraction.\n"
                                       "Range: 0, 8, 16 or 32");
        widget_.lblPadding->setToolTipDuration(10000);
        
        widget_.lblScaleFactor->setToolTip("Controls the percentage by which the image is shrunk on subsequent object detection passes.\n"
                                           "Range: 1.05 - 1.3");
        widget_.lblScaleFactor->setToolTipDuration(10000);
        
        widget_.lblMeanShiftGrouping->setToolTip("Controls overlapping bounding boxes to decide if the\n"
                                                 "common space between boxes is an object of interest.");
        widget_.lblMeanShiftGrouping->setToolTipDuration(10000);
        
        widget_.line_edit_hit_thresh->setToolTip("Press Enter to apply changes");
        widget_.line_edit_hit_thresh->setToolTipDuration(5000);
        
        widget_.line_edit_step_size->setToolTip("Press Enter to apply changes");
        widget_.line_edit_step_size->setToolTipDuration(5000);
        
        widget_.line_edit_padding->setToolTip("Press Enter to apply changes");
        widget_.line_edit_padding->setToolTipDuration(5000);
        
        widget_.line_edit_scale_factor->setToolTip("Press Enter to apply changes");
        widget_.line_edit_scale_factor->setToolTipDuration(5000);
    }
    
    void SettingsWidget::readGeneralSettings()
    {
        gcs->settings_->beginGroup("general_tab");

        QString ml = gcs->settings_->value("machine_learning", "online").toString();
        if(ml == "online")
            widget_.online_btn->setChecked(true);
        else
            widget_.offline_btn->setChecked(true); 
        
        ml_state_ = ml;
        
        QString vehicle_link = gcs->settings_->value("connection_drop/vehicle_gcs_link",
                                                "marginal").toString();
        if(vehicle_link == "nominal")
            widget_.nominal_btn->setChecked(true);
        else if(vehicle_link == "marginal")
            widget_.marginal_btn->setChecked(true);
        else //poor
            widget_.poor_btn->setChecked(true);

         //dont forget to disable the frequency box if nominal button is checked.
        if(widget_.nominal_btn->isChecked())
            widget_.frequency_groupbox->setEnabled(false);


        QString conn_freq = "connection_drop/frequency";

        QString frequency = gcs->settings_->value(conn_freq, "random").toString();
        if(frequency == "interval")
        {
            widget_.interval_btn->setChecked(true);
            QVariant interval = gcs->settings_->value(conn_freq + "/interval_text");
            if(!interval.isNull())
                widget_.interval_text_box->setText(interval.toString());

            widget_.interval_text_box->setEnabled(true);
        }
        else
        {
            widget_.random_btn->setChecked(true);
            widget_.interval_text_box->setEnabled(false);
        }

        bool length_specified = gcs->settings_->value(conn_freq + "/length_box_checked",
                                                 false).toBool();
        if(length_specified)
        {
            QVariant length = gcs->settings_->value(conn_freq + "/length_text");
            if(!length.isNull())
                widget_.length_text_box->setText(length.toString());

             widget_.length_check_box->setChecked(true);
        }
        else
        {
            widget_.length_check_box->setChecked(false);
            widget_.length_text_box->setEnabled(false);
        }
        
        QString img_dir = gcs->settings_->value("images_root_directory", gcs->image_root_dir_).toString();
        widget_.line_edit_images_dir->setText(img_dir);
        
        gcs->settings_->endGroup();
    }

    void SettingsWidget::writeGeneralSettings()
    {
        gcs->settings_->beginGroup("general_tab");
        
        QString ml;
        if(widget_.online_btn->isChecked())
            ml = "online";  
        else
            ml = "offline";
        
        ml_state_ = ml;
        gcs->settings_->setValue("machine_learning", ml);

        
        QString conn = "connection_drop";

        QString vehicle_link;
        if(widget_.nominal_btn->isChecked())
            vehicle_link = "nominal";
        else if(widget_.marginal_btn->isChecked())
            vehicle_link = "marginal";
        else
            vehicle_link = "poor";
        
        gcs->settings_->setValue(conn % "/vehicle_gcs_link", vehicle_link);
        
        QString conn_freq = "connection_drop/frequency";
        if(vehicle_link != "nominal") // write frequency settings to config
        {   
            QString frequency;
            if(widget_.interval_btn->isChecked())
                frequency = "interval";
            else
                frequency = "random";
            gcs->settings_->setValue(conn_freq, frequency);
            
            if(frequency == "interval")
            {   //already guaranteed text is valid in validateGeneralSettings()
                QString interval_text = widget_.interval_text_box->text();
                gcs->settings_->setValue(conn_freq % "/interval_text", interval_text);
            }
            else
                gcs->settings_->remove(conn_freq % "/interval_text");
               
            bool length_specified = widget_.length_check_box->isChecked();
            gcs->settings_->setValue(conn_freq % "/length_box_checked", length_specified);

            if(length_specified)
            {   //already guaranteed text is valid in validateGeneralSettings()
                QString length_text = widget_.length_text_box->text();
                gcs->settings_->setValue(conn_freq % "/length_text", length_text);
            }
            else
                gcs->settings_->remove(conn_freq % "/length_text");
        }
        else // remove all frequency related settings from config
            gcs->settings_->remove(conn_freq);
        
        QString img_dir = widget_.line_edit_images_dir->text();
        if(!img_dir.isNull())
            gcs->settings_->setValue("images_root_directory", img_dir);
        
        gcs->settings_->endGroup(); //general_tab
    }
    
        bool SettingsWidget::validateGeneralSettings()
    {
        //assure that text inputs are either enabled and not empty, or not enabled.
        //write settins only if the are disabled or are enabled and have valid input.
        //the settings dialogue stay open if these checks fail.
        if(widget_.frequency_groupbox->isEnabled())
        {
            QString interval_text = widget_.interval_text_box->text();
            QString length_text = widget_.length_text_box->text();
        
            if(widget_.interval_text_box->isEnabled() && interval_text.isEmpty())
            {
                std::cout << "tried to apply settings with interval radio button checked but no interval specified\n";
                return false;
            }
            
            bool ok;
            if(!interval_text.isEmpty())
            {
                float interval = interval_text.toFloat(&ok);
                if(!ok || interval <= 0)
                {
                    std::cout << "entered invalid interval: " << interval_text.toStdString() 
                        << std::endl;
                    return false;
                }
            }
            
            if(widget_.length_text_box->isEnabled() && length_text.isEmpty())
            {
                std::cout << "tried to apply settings with length checkbox checked but no length specified\n";
                return false;
            }

            if(!length_text.isEmpty())
            {   
                float length = length_text.toFloat(&ok);
                if(!ok || length <= 0)
                {
                    std::cout << "entered invalid length: " << length_text.toStdString() 
                        << std::endl;
                    return false;
                }
            }
        }
        
        QString img_dir = widget_.line_edit_images_dir->text();
        if(!img_dir.isEmpty())
        {
            if(!img_dir.startsWith('/'))
            {
                std::cout << "image root directory MUST be an absolute path\n";
                return false;
            }   
        }
        
        return true;
    }

    void SettingsWidget::readObjectDetectionSettings()
    {
        gcs->settings_->beginGroup("object_detection_tab");
        
        QString node_loc = gcs->settings_->value("node_location", "gcs").toString();
        if(node_loc == "gcs")
            widget_.gcs_btn->setChecked(true);
        else
            widget_.uav_btn->setChecked(true);
        // todo apply logic for determining where node will be run
        
        QString params = "tuning_paramaters";
        od_params.hit_thresh = gcs->settings_->value(params % "/hit_threshold", 0.45).toDouble();    
        od_params.step_size = gcs->settings_->value(params % "/step_size", 8).toInt();
        od_params.padding = gcs->settings_->value(params % "/padding", 4).toInt();
        od_params.scale_factor = gcs->settings_->value(params % "/scale_factor", 1.15).toDouble();
        od_params.mean_shift = gcs->settings_->value(params % "/mean_shift_grouping", true).toBool();
        
        widget_.line_edit_hit_thresh->setText(QString::number(od_params.hit_thresh, 'f', 2));
        widget_.sl_hit_thresh->setValue(od_params.hit_thresh * 100);
        
        widget_.line_edit_step_size->setText(QString::number(od_params.step_size));
        widget_.sl_step_size->setValue(od_params.step_size / 4);
        
        widget_.line_edit_padding->setText(QString::number(od_params.padding));
        widget_.sl_padding->setValue(od_params.padding / 8);
        
        widget_.line_edit_scale_factor->setText(QString::number(od_params.scale_factor, 'f', 2));
        widget_.sl_scale_factor->setValue(od_params.scale_factor * 100);
        
        if(od_params.mean_shift == true)
            widget_.radio_on_mean_shift->setChecked(true);
        else 
            widget_.radio_off_mean_shift->setChecked(true);
        onMeanShiftRadioChange();
        
        gcs->settings_->endGroup();
    }
    
    void SettingsWidget::writeObjectDetectionSettings()
    {
        gcs->settings_->beginGroup("object_detection_tab");
        
        if(widget_.uav_btn->isChecked())
            gcs->settings_->setValue("node_location", "uav");
        else if(widget_.gcs_btn->isChecked())
            gcs->settings_->setValue("node_location", "gcs");
        
        QString params = "tuning_paramaters";
        QString thresh = QString::number(od_params.hit_thresh,'f', 2);
        gcs->settings_->setValue(params % "/hit_threshold", thresh);
        gcs->settings_->setValue(params % "/step_size", od_params.step_size);
        gcs->settings_->setValue(params % "/padding", od_params.padding);
        QString scale = QString::number(od_params.scale_factor, 'f', 2);
        gcs->settings_->setValue(params % "/scale_factor", scale);
        gcs->settings_->setValue(params % "/mean_shift_grouping", od_params.mean_shift);
        
        gcs->settings_->endGroup();
    }
        
    bool SettingsWidget::onApplyClicked()
    {   
        QString ml_state_previous = ml_state_;
        
        writeObjectDetectionSettings();
        
        if(!validateGeneralSettings()) 
            return false;
        
        writeGeneralSettings();
        
        if(ml_state_ != ml_state_previous)
            emit machineLearningModeToggled(widget_.online_btn->isChecked());
        
        return true;
    }
    
    void SettingsWidget::onOkClicked()
    {
        if(onApplyClicked())
            this->close();
    }

    void SettingsWidget::onCancelClicked()
    {
        this->close();
    }

    void SettingsWidget::onToggleFrequencyGroup()
    {
        widget_.frequency_groupbox->setEnabled(!widget_.nominal_btn->isChecked());

        std::cout << "frequency group "
                << (widget_.frequency_groupbox->isEnabled() ? "enabled" : "disabled")
                << std::endl;
    }

    void SettingsWidget::onToggleIntervalLine()
    {
        widget_.interval_text_box->setEnabled(widget_.interval_btn->isChecked()); 

        std::cout << "interval text box "
                << (widget_.interval_btn->isEnabled() ? "enabled" : "disabled")
                << std::endl;
    }

    void SettingsWidget::onToggleLengthLine()
    {
        widget_.length_text_box->setEnabled(widget_.length_check_box->isChecked());

        std::cout << "length text box "
                << (widget_.length_check_box->isEnabled() ? "enabled" : "disabled")
                << std::endl;
    }
    
    
    // object detection parameter adjustments
    
    void SettingsWidget::onHitThresholdSliderChange(int new_thresh)
    {
        double thresh = ( ((double)new_thresh) / 100 );
        if(od_params.hit_thresh != thresh)
        {
            od_params.hit_thresh = thresh;
            widget_.line_edit_hit_thresh->setText(QString::number(thresh, 'f', 2));
            gcs->publishHitThreshold(thresh);
        }
    }
    
    void SettingsWidget::onHitThresholdLineChange()
    {
        QString s = widget_.line_edit_hit_thresh->text();
        if(s.startsWith('.'))
            s = "0" % s;
        
        bool ok;
        double thresh = s.toDouble(&ok);
        if(!ok)
        {
            std::cout << "tried to enter invalid threshold value: " 
                      << s.toStdString() << "\n";
            widget_.line_edit_hit_thresh->setText("");
            return;
        }
        
        int thresh_old = thresh;
        if(thresh < 0)
            thresh = 0;
        else if(thresh > 0.9)
            thresh = 0.9;
        
        if(thresh != thresh_old)
             widget_.line_edit_hit_thresh->setText(QString::number(thresh, 'f', 2));
        
        od_params.hit_thresh = thresh;
        widget_.sl_hit_thresh->setValue(thresh * 100);
    }
    
    void SettingsWidget::onStepSizeSliderChange(int new_step)
    {
        new_step *= 4; // map to 4, 8, 12 or 16
        if(od_params.step_size != new_step)
        {
            od_params.step_size = new_step;
            widget_.line_edit_step_size->setText(QString::number(new_step));
            gcs->publishStepSize(new_step);
        }
    }
    
    void SettingsWidget::onStepSizeLineChange()
    {
        QString s = widget_.line_edit_step_size->text();
        
        bool ok;
        int step = s.toInt(&ok);
        if(!ok || step % 4 != 0)
        {
            std::cout << "tried to enter invalid step size value: "
                      << s.toStdString() << "\n";
            widget_.line_edit_step_size->setText("");
            return;
        }
        
        int step_old = step;
        if(step < 4)
            step = 4;
        else if(step > 16)
            step = 16;
        
        if(step != step_old)
             widget_.line_edit_step_size->setText(QString::number(step));
        
        od_params.step_size = step;
        widget_.sl_step_size->setValue(step / 4);
    }
    
    void SettingsWidget::onPaddingSliderChange(int new_padding)
    {
        new_padding *= 8; // map to 0, 8, 16, 24 or 32
        if(od_params.padding != new_padding)
        {
            od_params.padding = new_padding;
            widget_.line_edit_padding->setText(QString::number(new_padding));
            gcs->publishPadding(new_padding);
        }
    }
    
    void SettingsWidget::onPaddingLineChange()
    {
        QString s = widget_.line_edit_padding->text();
        
        bool ok;
        int padding = s.toInt(&ok);
        if(!ok || padding % 8 != 0)
        {
            std::cout << "tried to enter invalid padding value: " 
                      << s.toStdString() << "\n";
            widget_.line_edit_padding->setText("");
            return;
        }
        
        int padding_old = padding;
        if(padding < 0)
            padding = 0;
        else if(padding > 32)
            padding = 32;
        
        if(padding != padding_old)
            widget_.line_edit_padding->setText(QString::number(padding));
        
        od_params.padding = padding;
        widget_.sl_padding->setValue(padding / 8);
    }
    
    void SettingsWidget::onScaleFactorSliderChange(int new_scale)
    {
        double scale = ( ((double)new_scale) / 100 );
        if(od_params.scale_factor != scale)
        {
            od_params.scale_factor = scale;
            widget_.line_edit_scale_factor->setText(QString::number(scale, 'f', 2));
            gcs->publishScaleFactor(scale);
        }
    }
    
    void SettingsWidget::onScaleFactorLineChange()
    {   
        QString s = widget_.line_edit_scale_factor->text();
        if(s.startsWith('.'))
            s = "1" % s;
        
        bool ok;
        double scale = s.toDouble(&ok);
        if(!ok)
        {
            std::cout << "tried to enter invalid scale value: " 
                      << s.toStdString() << "\n";
            widget_.line_edit_scale_factor->setText("");
            return;
        }
        
        int scale_old = scale;
        if(scale < 1.05)
            scale = 1.05;
        else if(scale > 1.3)
            scale = 1.3;
       
        if(scale != scale_old)
            widget_.line_edit_scale_factor->setText(QString::number(scale, 'f', 2));
        
        od_params.scale_factor = scale;
        widget_.sl_scale_factor->setValue(scale * 100);
    }
    
    void SettingsWidget::onMeanShiftRadioChange()
    {
        bool on = widget_.radio_on_mean_shift->isChecked();
        if(od_params.mean_shift != on)
        {
            od_params.mean_shift = on;
            gcs->publishMeanShift(on);
        }
    }
    
}// end name space