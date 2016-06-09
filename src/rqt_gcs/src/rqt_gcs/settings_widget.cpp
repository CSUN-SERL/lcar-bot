
/*
 * File:   settings_widget.cpp
 * Author: serl
 *
 * Created on June 7, 2016, 3:40 PM
 */

#include "rqt_gcs/settings_widget.h"

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
    
    //nominal, marginal, poor radio buttons
    connect(widget_.nominal_btn, SIGNAL(clicked()), 
            this,                SLOT(toggleFrequencyGroup()));
    connect(widget_.marginal_btn, SIGNAL(clicked()),
            this,                 SLOT(toggleFrequencyGroup()));
    connect(widget_.poor_btn, SIGNAL(clicked()),
            this,                 SLOT(toggleFrequencyGroup()));
    
    connect(widget_.interval_btn, SIGNAL(clicked()),
            this,                     SLOT(toggleIntervalTextBox()));
    
    // freqency length check box enables the text edit field
    connect(widget_.length_check_box, SIGNAL(clicked()),
            this,                     SLOT(toggleLengthTextBox()));
    
}

SettingsWidget::~SettingsWidget() 
{
    settings_ = nullptr;
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
}

void SettingsWidget::toggleLengthTextBox()
{
    if(widget_.length_check_box->isChecked())
        widget_.length_text_edit->setEnabled(true);
    else
        widget_.length_text_edit->setDisabled(true);
}

void SettingsWidget::toggleIntervalTextBox()
{
    if(widget_.interval_btn->isChecked())
        widget_.interval_text_box->setEnabled(true);
    else
        widget_.interval_text_box->setDisabled(true);
}

} // end name space