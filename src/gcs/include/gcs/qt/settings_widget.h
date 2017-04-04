/*
 * File:   settings_widget.h
 * Author: serl
 *
 * Created on June 7, 2016, 3:40 PM
 */

#ifndef _SETTINGSWIDGET_H
#define _SETTINGSWIDGET_H

#include <QItemDelegate>

#include "ui_SettingsWidget.h"

#include <gcs/qt/settings_manager.h>

#include <gcs/util/settings.h>
#include <gcs/util/object_detection_parameters.h>


class QAction;
class QMenu;

namespace gcs
{
    
    class SettingsManager;
    
class SettingsWidget : public QWidget
{
    Q_OBJECT
public:
    SettingsWidget(SettingsManager * sm);
    virtual ~SettingsWidget();

private:
    Ui::SettingsWidget widget;
    QString ml_state;
    QString coordinate_system;
    QString image_root_dir;
    Settings settings; 
    SettingsManager * sm;
    
    QMenu* menu;
    
    ObjectDetectionParameters od_params;

    void setToolTips();

    void readGeneralSettings();
    void writeGeneralSettings();
    bool validateGeneralSettings();

    void readObjectDetectionSettings();
    void writeObjectDetectionSettings();
    
    void readCoordinateSystemSettings();
    void WriteCoordinateSystemSettings();

private slots:
    bool onApplyClicked();
    void onOkClicked();
    void onCancelClicked();
    void onToggleFrequencyGroup();
    void onToggleIntervalLine();
    void onToggleDurationLine();
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
    
    void onTableViewContextMenuRequested(const QPoint& p);
    void onAddRowClicked();
    void onDeleteRowClicked();
    void addOrDeleteSelectedRows(bool add, int row_offset = 0);
};

class Delegate : public QItemDelegate
{
public:
    Delegate(QObject* parent = nullptr) :
    QItemDelegate(parent)
    { }
    
    QWidget* createEditor(QWidget *parent, const QStyleOptionViewItem & option,
                      const QModelIndex & index) const
    {
        QLineEdit *lineEdit = new QLineEdit(parent);
        // Set validator
        QDoubleValidator *validator = new QDoubleValidator(-1000.0, 1000.0, 2, lineEdit);
        lineEdit->setValidator(validator);
        return lineEdit;
    }
};

} // end name space
#endif /* _SETTINGSWIDGET_H */
