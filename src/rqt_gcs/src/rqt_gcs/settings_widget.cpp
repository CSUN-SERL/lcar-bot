
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
            this,              SLOT(applySettings()));
    
    connect(widget_.cancel_btn, SIGNAL(clicked()), 
            this,               SLOT(cancel()));
    
    
    //nominal, marginal, poor radio buttons enable or disable frequency group
    connect(widget_.nominal_btn, SIGNAL(clicked()), 
            this,                SLOT(toggleFrequencyGroup()));
    
    connect(widget_.marginal_btn, SIGNAL(clicked()),
            this,                 SLOT(toggleFrequencyGroup()));
    
    connect(widget_.poor_btn, SIGNAL(clicked()),
            this,             SLOT(toggleFrequencyGroup()));
    
    
    //interval, random radio buttons enable or disable interval text edit field
    connect(widget_.interval_btn, SIGNAL(clicked()),
            this,                 SLOT(toggleIntervalTextBox()));
    
    connect(widget_.random_btn, SIGNAL(clicked()),
            this,               SLOT(toggleIntervalTextBox()));
    
     
    //frequency/length check box enables the length text edit field
    connect(widget_.length_check_box, SIGNAL(clicked()),
            this,                     SLOT(toggleLengthTextBox()));
    
}

SettingsWidget::~SettingsWidget() 
{
    settings_ = nullptr;
    delete settings_;
    
    emit destroyed();
    delete this;
}

void SettingsWidget::applySettings()
{
    
}

void SettingsWidget::cancel()
{
    
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
        widget_.length_text_edit->setEnabled(true);
    else
        widget_.length_text_edit->setReadOnly(true);
    
    std::cout << "length text box " 
              << (widget_.length_check_box->isChecked() ? "enabled" : "disabled")
              << std::endl;
}

} // end name space