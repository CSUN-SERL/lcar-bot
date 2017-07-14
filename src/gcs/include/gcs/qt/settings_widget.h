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

#include <gcs/util/settings.h>
#include <vehicle/position.h> 
#include <gcs/util/object_detection_parameters.h>


class QStandardItemModel;
class QAction;
class QMenu;

namespace gcs
{
    
class SettingsWidget : public QWidget
{
    Q_OBJECT
public:
    SettingsWidget();
    virtual ~SettingsWidget();
    
    void getCoordinates(QVector<Point>& vector);

    
signals:
    void localCoordinatesUpdated(const QVector<Point>& vector);
    
private:
    Ui::SettingsWidget widget;
    QString ml_state;
    QString coordinate_system;
    QString image_root_dir;
    Settings settings; 
    
    struct CoordinateColumns
    {
        int x;
        int y;
        int z;
    } columns;
    
    QStandardItemModel * mdl_coords;
    QMenu * menu;
    
    ObjectDetectionParameters od_params;

    void setToolTips();

    void readGeneralSettings();
    void writeGeneralSettings();
    bool validateGeneralSettings();

    void readObjectDetectionSettings();
    void writeObjectDetectionSettings();
    
    bool validateCoordinateSystemSettings();
    void readCoordinateSystemSettings();
    void writeCoordinateSystemSettings();
    
    
    void initCoordinatesModel();
    void emitLocalCoordinatesUpdated();
    
private slots:
    bool onApplyClicked();
    void onOkClicked();
    void onCancelClicked();
    void onToggleFrequencyGroup();
    void onToggleIntervalLine();
    void onToggleDurationLine();
    void onToggleCoordinateSystem();
    void onCoordinateSystemChange();

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
    
    QWidget * createEditor(QWidget * parent, 
                          const QStyleOptionViewItem & option,
                          const QModelIndex & index) const
    {
        QLineEdit * lineEdit = new QLineEdit(parent);
        QDoubleValidator * validator = new QDoubleValidator(-1000.0, 1000.0, 2, lineEdit);
        lineEdit->setValidator(validator);
        return lineEdit;
    }
    
    virtual void setModelData(QWidget * editor, 
                              QAbstractItemModel * model,
                              const QModelIndex & index) const override
    {
        QLineEdit * line_edit = static_cast<QLineEdit*>(editor);
        model->setData(index, line_edit->text().toDouble(), Qt::DisplayRole);
    }
};

} // end name space
#endif /* _SETTINGSWIDGET_H */
