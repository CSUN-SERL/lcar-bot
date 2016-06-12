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


        setGeneralTabDefaults();

        //TODO
        //setObjectDetectionTabDefaults();
    }

    SettingsWidget::~SettingsWidget()
    {
        // don't delete this because main GUI needs it.
        settings_ = nullptr;
    }

    void SettingsWidget::setGeneralTabDefaults()
    {
        settings_->beginGroup("general_tab");

        QString ml = settings_->value("machine_learning", "online").toString();
        if(ml == "online")
            widget_.online_btn->setChecked(true);
        else
            widget_.offline_btn->setChecked(true);


        QString vehicle_link = settings_->value("connection_drop/vehicle_gcs_link",
                                                "nominal").toString();
        if(vehicle_link == "nominal")
            widget_.nominal_btn->setChecked(true);
        else if(vehicle_link == "marginal")
            widget_.marginal_btn->setChecked(true);
        else //poor
            widget_.poor_btn->setChecked(true);

         //dont forget to disable the frequency box if nominal button is checked.
        if(widget_.nominal_btn->isChecked())
            widget_.frequency_box->setEnabled(false);

        
        QString conn_freq = "connection_drop/frequency";

        QString frequency = settings_->value(conn_freq, "random").toString();
        if(frequency == "interval")
        {
            widget_.interval_btn->setChecked(true);
            int interval_text = settings_->value(conn_freq + "/interval_text", -1).toInt();
            if(interval_text != -1)
                widget_.interval_text_box->setText(QString::number(interval_text));

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
            int length_text = settings_->value(conn_freq + "/length_text", -1).toInt();
            if(length_text != -1)
                widget_.length_text_box->setText(QString::number(length_text));

             widget_.length_check_box->setChecked(true);
        }
        else
        {
            widget_.length_check_box->setChecked(false);
            widget_.length_text_box->setEnabled(false);
        }
        
        settings_->endGroup();
    }

    bool SettingsWidget::applyGeneralTabSettings()
    {
        //assure that text inputs arent enabled and empty
        QString interval_text = widget_.interval_text_box->text();
        QString length_text = widget_.length_text_box->text();
        if(widget_.interval_text_box->isEnabled() && interval_text.isEmpty())
        {
            std::cout << "tried to apply settings with interval radio button checked but no interval specified\n";
            return false;
        }
        if(widget_.length_text_box->isEnabled() && length_text.isEmpty())
        {
            std::cout << "tried to apply settings with length checkbox checked but no length specified\n";
            return false;
        }
        
        bool ok;
        int interval, length;
        if(!interval_text.isEmpty())
        {
            interval = interval_text.toInt(&ok);
            if(!ok)
            {
                std::cout << "entered invalid interval" << std::endl;
                return false;
            }
        }
        if(!length_text.isEmpty())
        {   
            length = length_text.toInt(&ok);
            if(!ok)
            {
                std::cout << "entered invalid length" << std::endl;
                return false;
            }
        }

        
        settings_->beginGroup("general_tab");

        QString ml;
        if(widget_.online_btn->isChecked())
            ml = "online";
        else if(widget_.offline_btn->isChecked())
            ml = "offline";
        if(!ml.isNull())
            settings_->setValue("machine_learning", ml);

        QString conn = "connection_drop";

        QString vehicle_link;
        if(widget_.nominal_btn->isChecked())
            vehicle_link = "nominal";
        else if(widget_.marginal_btn->isChecked())
            vehicle_link = "marginal";
        else if(widget_.poor_btn->isChecked())
            vehicle_link = "poor";
        
        if(!vehicle_link.isNull())
            settings_->setValue(conn + "/vehicle_gcs_link", vehicle_link);
        
        if(vehicle_link != "nominal")
        {
            QString conn_freq = "connection_drop/frequency";
            QString frequency;
            if(widget_.interval_btn->isChecked())
                frequency = "interval";
            else if(widget_.random_btn->isChecked())
                frequency = "random";
            if(!frequency.isNull())
                settings_->setValue(conn_freq, frequency);

            //already guaranteed that the input is valid above
            if(!interval_text.isEmpty())
                settings_->setValue(conn_freq + "/interval_text", interval);
            else
               settings_->remove(conn_freq + "/interval_text");
            
            bool length_specified = widget_.length_check_box->isChecked();
            settings_->setValue(conn_freq + "/length_box_checked", length_specified);

            //already guaranteed that the input is valid above
            if(!length_text.isEmpty())
                settings_->setValue(conn_freq + "/length_text", length);
            else
                settings_->remove(conn_freq + "/length_text");
        }
        
        settings_->endGroup(); //general_tab
        return true;
    }

    bool SettingsWidget::applyObjectDetectionSettings()
    {
        settings_->beginGroup("object_detection_tab");
        //TODO
        settings_->endGroup();
        return false;
    }

    void SettingsWidget::setObjectDetectionTabDefaults()
    {
        settings_->beginGroup("object_detection_tab");
        //TODO
        settings_->endGroup();
    }

    void SettingsWidget::applyClicked()
    {
        if(settings_ == nullptr)
        {
            std::cout << "SettingsWidget contains a nullptr settings object"
                    << std::endl;
            return;
        }

        if(applyGeneralTabSettings()
            /*&& applyObjectDetectionSettings() */) // TODO
            emit dismissMe();
    }

    void SettingsWidget::cancelClicked()
    {
       emit dismissMe();
    }

    void SettingsWidget::toggleFrequencyGroup()
    {
        if(widget_.nominal_btn->isChecked())
        {
            widget_.interval_text_box->setText("");
            widget_.length_text_box->setText("");
            widget_.frequency_box->setEnabled(false);
        }
        else
            widget_.frequency_box->setEnabled(true);

        std::cout << "frequency group "
                << (widget_.nominal_btn->isChecked() ? "disabled" : "enabled")
                << std::endl;
    }

    void SettingsWidget::toggleIntervalTextBox()
    {
        if(widget_.interval_btn->isChecked())
            widget_.interval_text_box->setEnabled(true);
        else
        {
            widget_.interval_text_box->setText("");
            widget_.interval_text_box->setEnabled(false);
        }

        std::cout << "interval text box "
                << (widget_.interval_btn->isChecked() ? "enabled" : "disabled")
                << std::endl;
    }

    void SettingsWidget::toggleLengthTextBox()
    {
        if(widget_.length_check_box->isChecked())
            widget_.length_text_box->setEnabled(true);
        else
        {
            widget_.length_text_box->setText("");
            widget_.length_text_box->setEnabled(false);
        }

        std::cout << "length text box "
                << (widget_.length_check_box->isChecked() ? "enabled" : "disabled")
                << std::endl;
    }

}// end name space