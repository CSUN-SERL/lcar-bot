/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SettingsWidget.h
 * Author: serl
 *
 * Created on June 7, 2016, 3:40 PM
 */

#ifndef _SETTINGSWIDGET_H
#define _SETTINGSWIDGET_H

#include "ui_SettingsWidget.h"

class SettingsWidget : public QWidget {
    Q_OBJECT
public:
    SettingsWidget();
    virtual ~SettingsWidget();
private:
    Ui::SettingsWidget widget;
};

#endif /* _SETTINGSWIDGET_H */
