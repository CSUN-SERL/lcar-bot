
/*
 * File:   settings_widget.cpp
 * Author: serl
 *
 * Created on June 7, 2016, 3:40 PM
 */

#include "rqt_gcs/settings_widget.h"
#include <iostream>

namespace rqt_gcs
{

    SettingsWidget::SettingsWidget(QSettings *settings) :
    settings_(settings)
    {
        widget_.setupUi(this);

        //apply and cancel buttons
        connect(widget_.apply_btn, SIGNAL(clicked()),
                this, SLOT(applySettings()));

        connect(widget_.cancel_btn, SIGNAL(clicked()),
                this, SLOT(cancel()));


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
    }

    SettingsWidget::~SettingsWidget(){ }

    void SettingsWidget::applySettings()
    {
        std::cout << "clicked apply or cancel\n";
        if(settings_ == nullptr)
        {
            std::cout << "SettingsWidget contains a nullptr settings object"
                    << std::endl;
            return;
        }
        
        this->applyGeneralSettings();

        //TODO
        //this->applyObjectDetectionSettings();
        
        dismissMe();

    }

    void SettingsWidget::applyGeneralSettings()
    {
        settings_->beginGroup("general_tab");

        QString ml = "";
        if(widget_.online_btn->isChecked())
            ml = "online";
        else if(widget_.offline_btn->isChecked())
            ml = "offline";
        if(!ml.isNull())
            settings_->setValue("machine_learning", ml);

        settings_->beginGroup("connection_drop");

        QString vehicle_link;
        if(widget_.nominal_btn->isChecked())
            vehicle_link = "nominal";
        else if(widget_.marginal_btn->isChecked())
            vehicle_link = "marginal";
        else if(widget_.poor_btn->isChecked())
            vehicle_link = "poor";
        if(!vehicle_link.isNull())
            settings_->setValue("vehicle_gcs_link", vehicle_link);

        QString frequency;
        bool interval_checked = widget_.interval_btn->isChecked();
        if(interval_checked)
            frequency = "interval";
        else if(widget_.random_btn->isChecked())
            frequency = "random";
        if(!frequency.isNull())
            settings_->setValue("frequency", frequency);

        QString interval_text = widget_.interval_text_box->toPlainText();
        if(interval_checked && !interval_text.isNull())
            settings_->setValue("frequency/interval", interval_text.toInt());

        bool length_specified = widget_.length_check_box->isChecked();
        settings_->setValue("frequency/length_checked", length_specified);

        QString length_text = widget_.length_text_edit->toPlainText();
        if(length_specified && !length_text.isNull())
            settings_->setValue("frequency/length", length_text.toInt());

        settings_->endGroup(); //connection_drop
        settings_->endGroup(); //general_tab
    }

    void SettingsWidget::applyObjectDetectionSettings(){ }

    void SettingsWidget::cancel()
    { 
        dismissMe();
    }

    void SettingsWidget::toggleFrequencyGroup()
    {
        if(widget_.nominal_btn->isChecked())
            widget_.frequency_box->setEnabled(false);
        else
            widget_.frequency_box->setEnabled(true);

        std::cout << "frequency group "
                << (widget_.nominal_btn->isChecked() ? "disabled" : "enabled")
                << std::endl;
    }

    void SettingsWidget::toggleIntervalTextBox()
    {
        if(widget_.interval_btn->isChecked())
            widget_.interval_text_box->setReadOnly(false);
        else
            widget_.interval_text_box->setReadOnly(true);

        std::cout << "interval text box "
                << (widget_.interval_btn->isChecked() ? "enabled" : "disabled")
                << std::endl;
    }

    void SettingsWidget::toggleLengthTextBox()
    {
        if(widget_.length_check_box->isChecked())
            widget_.length_text_edit->setReadOnly(false);
        else
            widget_.length_text_edit->setReadOnly(true);

        std::cout << "length text box "
                << (widget_.length_check_box->isChecked() ? "enabled" : "disabled")
                << std::endl;
    }
    
    void SettingsWidget::dismissMe()
    {
        this->setVisible(false);
        
        settings_ = nullptr;
        delete this;
    }

} // end name space
