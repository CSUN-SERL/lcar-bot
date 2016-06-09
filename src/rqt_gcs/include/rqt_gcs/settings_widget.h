/* 
 * File:   settings_widget.h
 * Author: serl
 *
 * Created on June 7, 2016, 3:40 PM
 */

#ifndef _SETTINGSWIDGET_H
#define _SETTINGSWIDGET_H

#include "ui_SettingsWidget.h"

namespace rqt_gcs
{

class SettingsWidget : public QWidget {
    Q_OBJECT
public:
    SettingsWidget();
    virtual ~SettingsWidget();
private:
    Ui::SettingsWidget widget;
};

} // end name space
#endif /* _SETTINGSWIDGET_H */
