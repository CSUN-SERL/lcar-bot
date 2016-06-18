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

    SettingsWidget::SettingsWidget(QSettings *settings) :
    settings_(settings)
    {
        assert(settings != NULL && settings != nullptr);

        widget_.setupUi(this);

        //apply and cancel buttons
        connect(widget_.apply_btn, SIGNAL(clicked()),
                this, SLOT(applyClicked()));

        connect(widget_.cancel_btn, SIGNAL(clicked()),
                this, SLOT(cancelClicked()));


        //nominal, marginal, poor radio buttons enable or disable frequency group
        connect(widget_.nominal_btn, SIGNAL(clicked()),
                this, SLOT(toggleFrequencyGroup()));

        connect(widget_.marginal_btn, SIGNAL(clicked()),
                this, SLOT(toggleFrequencyGroup()));

        connect(widget_.poor_btn, SIGNAL(clicked()),
                this, SLOT(toggleFrequencyGroup()));


        //interval, random radio buttons enable or disable interval text edit field
        connect(widget_.interval_btn, SIGNAL(clicked()),
                this, SLOT(toggleIntervalTextBox()));

        connect(widget_.random_btn, SIGNAL(clicked()),
                this, SLOT(toggleIntervalTextBox()));


        //frequency/length check box enables the length text edit field
        connect(widget_.length_check_box, SIGNAL(clicked()),
                this, SLOT(toggleLengthTextBox()));


        readGeneralSettings();

        //TODO
        //setObjectDetectionTabDefaults();
    }

    SettingsWidget::~SettingsWidget()
    {
        // don't delete this because SimpleGCS needs it. just derefence it.
        settings_ = nullptr;
    }

    void SettingsWidget::readGeneralSettings()
    {
        settings_->beginGroup("general_tab");

        QString ml = settings_->value("machine_learning", "online").toString();
        if(ml == "online")
            widget_.online_btn->setChecked(true);
        else
            widget_.offline_btn->setChecked(true); 
        
        ml_state_ = ml;
        
        QString vehicle_link = settings_->value("connection_drop/vehicle_gcs_link",
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

        QString frequency = settings_->value(conn_freq, "random").toString();
        if(frequency == "interval")
        {
            widget_.interval_btn->setChecked(true);
            QVariant interval = settings_->value(conn_freq + "/interval_text");
            if(!interval.isNull())
                widget_.interval_text_box->setText(interval.toString());

            widget_.interval_text_box->setEnabled(true);
        }
        else
        {
            widget_.random_btn->setChecked(true);
            widget_.interval_text_box->setEnabled(false);
        }

        bool length_specified = settings_->value(conn_freq + "/length_box_checked",
                                                 false).toBool();
        if(length_specified)
        {
            QVariant length = settings_->value(conn_freq + "/length_text");
            if(!length.isNull())
                widget_.length_text_box->setText(length.toString());

             widget_.length_check_box->setChecked(true);
        }
        else
        {
            widget_.length_check_box->setChecked(false);
            widget_.length_text_box->setEnabled(false);
        }
        
        settings_->endGroup();
    }

    void SettingsWidget::writeGeneralSettings()
    {
        settings_->beginGroup("general_tab");
        
        QString ml;
        if(widget_.online_btn->isChecked())
            ml = "online";  
        else
            ml = "offline";
        
        ml_state_ = ml;
        settings_->setValue("machine_learning", ml);

        
        QString conn = "connection_drop";

        QString vehicle_link;
        if(widget_.nominal_btn->isChecked())
            vehicle_link = "nominal";
        else if(widget_.marginal_btn->isChecked())
            vehicle_link = "marginal";
        else
            vehicle_link = "poor";
        
        settings_->setValue(conn + "/vehicle_gcs_link", vehicle_link);
        
        QString conn_freq = "connection_drop/frequency";
        if(vehicle_link != "nominal") // write frequency settings to config
        {   
            QString frequency;
            if(widget_.interval_btn->isChecked())
                frequency = "interval";
            else
                frequency = "random";
            settings_->setValue(conn_freq, frequency);
            
            if(frequency == "interval")
            {   //already guaranteed text is valid in validateGeneralSettings()
                QString interval_text = widget_.interval_text_box->text();
                settings_->setValue(conn_freq + "/interval_text", interval_text);
            }
            else
                settings_->remove(conn_freq + "/interval_text");
               
            bool length_specified = widget_.length_check_box->isChecked();
            settings_->setValue(conn_freq + "/length_box_checked", length_specified);

            if(length_specified)
            {   //already guaranteed text is valid in validateGeneralSettings()
                QString length_text = widget_.length_text_box->text();
                settings_->setValue(conn_freq + "/length_text", length_text);
            }
            else
                settings_->remove(conn_freq + "/length_text");
        }
        else // remove all frequency related settings from config
            settings_->remove(conn_freq);
        
        settings_->endGroup(); //general_tab
    }

//    void SettingsWidget::writeObjectDetectionSettings()
//    {
//        settings_->beginGroup("object_detection_tab");
//        //TODO
//        settings_->endGroup();
//        return false;
//    }

//    void SettingsWidget::readObjectDetectionSettings()
//    {
//        settings_->beginGroup("object_detection_tab");
//        //TODO
//        settings_->endGroup();
//    }

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
        } // end if nominal button not checked
        
        return true;
    }

    //bool SettingsWidget::validateObjectDetectionSettings() { }
    
    void SettingsWidget::applyClicked()
    {
        if(settings_ == nullptr)
        {
            std::cout << "SettingsWidget contains a nullptr settings object"
                    << std::endl;
            return;
        }
        
        QString ml_state_previous = ml_state_;
        
        if(!validateGeneralSettings()
         /*&& !validateObjectDetectionTabSettings()*/) // TODO
            return;
        
        writeGeneralSettings();
        
        //TODO
        //writeObjectDetectionTabSettings();
        
        if(ml_state_ != ml_state_previous)
            emit toggleMachineLearningMode(widget_.online_btn->isChecked());
    
        emit dismissMe();
    }

    void SettingsWidget::cancelClicked()
    {
       emit dismissMe();
    }

    void SettingsWidget::toggleFrequencyGroup()
    {
        widget_.frequency_groupbox->setEnabled(!widget_.nominal_btn->isChecked());

        std::cout << "frequency group "
                << (widget_.frequency_groupbox->isEnabled() ? "enabled" : "disabled")
                << std::endl;
    }

    void SettingsWidget::toggleIntervalTextBox()
    {
        widget_.interval_text_box->setEnabled(widget_.interval_btn->isChecked()); 

        std::cout << "interval text box "
                << (widget_.interval_btn->isEnabled() ? "enabled" : "disabled")
                << std::endl;
    }

    void SettingsWidget::toggleLengthTextBox()
    {
        widget_.length_text_box->setEnabled(widget_.length_check_box->isChecked());

        std::cout << "length text box "
                << (widget_.length_check_box->isEnabled() ? "enabled" : "disabled")
                << std::endl;
    }
    
}// end name space