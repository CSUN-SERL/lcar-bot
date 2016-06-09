/* 
 * File:   settings_widget.h
 * Author: serl
 *
 * Created on June 7, 2016, 3:40 PM
 */

#ifndef _SETTINGSWIDGET_H
#define _SETTINGSWIDGET_H

#include "ui_SettingsWidget.h"
#include <QtCore/QSettings>

namespace rqt_gcs
{

class SettingsWidget : public QWidget {
    Q_OBJECT
public:
    SettingsWidget(QSettings* settings);
    virtual ~SettingsWidget();
private:
    Ui::SettingsWidget widget_;
    QSettings *settings_;
    
private slots:
    void applySettings();
    void cancel();
    void toggleFrequencyGroup();
    void toggleIntervalTextBox();
    void toggleLengthTextBox();
    
};

} // end name space
#endif /* _SETTINGSWIDGET_H */
