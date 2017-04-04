/*
 * File:   settings_widget.cpp
 * Author: serl
 *
 * Created on June 7, 2016, 3:40 PM
 */

#include <QStandardItemModel>
#include <QItemSelectionModel>
#include <QStringBuilder>
#include <QMessageBox>
#include <QAction>
#include <QMenu>

#include <iostream>

#include <gcs/qt/ui_adapter.h>
#include <gcs/qt/settings_widget.h>
#include <gcs/qt/settings_manager.h>
#include <gcs/util/image_conversions.h>
#include <gcs/util/flight_modes.h>
#include <gcs/util/settings.h>

namespace gcs
{

SettingsWidget::SettingsWidget(SettingsManager * sm) :
sm(sm),
menu(nullptr)
{
    this->setAttribute(Qt::WA_DeleteOnClose);

    widget.setupUi(this);
    widget.tablev_coordinates->setModel(sm->mdl_cs);
    widget.tablev_coordinates->setItemDelegate(new Delegate);
    widget.tablev_coordinates->setContextMenuPolicy(Qt::CustomContextMenu);
    
    QObject::connect(widget.btn_add_row, &QPushButton::clicked,
                    this, &SettingsWidget::onAddRowClicked);
    
    QObject::connect(widget.btn_delete_row, &QPushButton::clicked,
                    this, &SettingsWidget::onDeleteRowClicked);
    
    QObject::connect(widget.tablev_coordinates, &QTableView::customContextMenuRequested,
                    this, &SettingsWidget::onTableViewContextMenuRequested);
    
    //apply and cancel buttons
    connect(widget.apply_btn, &QPushButton::clicked,
            this, &SettingsWidget::onApplyClicked);

    connect(widget.ok_btn, &QPushButton::clicked,
            this, &SettingsWidget::onOkClicked);

    connect(widget.cancel_btn, &QPushButton::clicked,
            this, &SettingsWidget::onCancelClicked);

    //nominal, marginal, poor radio buttons enable or disable frequency group
    connect(widget.nominal_btn, &QRadioButton::clicked,
            this, &SettingsWidget::onToggleFrequencyGroup);

    connect(widget.marginal_btn, &QRadioButton::clicked,
            this,&SettingsWidget::onToggleFrequencyGroup);

    connect(widget.poor_btn, &QRadioButton::clicked,
            this, &SettingsWidget::onToggleFrequencyGroup);


    //interval, random radio buttons enable or disable interval text edit field
    connect(widget.interval_btn, &QRadioButton::clicked,
            this, &SettingsWidget::onToggleIntervalLine);

    connect(widget.random_btn, &QRadioButton::clicked,
            this, &SettingsWidget::onToggleIntervalLine);

    //frequency/duration check box enables the duration text edit field
    connect(widget.duration_check_box, &QRadioButton::clicked,
            this, &SettingsWidget::onToggleDurationLine);

    //object detection parameter inits and slider/line_edit connections
    //hit threshold
    widget.sl_hit_thresh->setMinimum(0); // divided by 100 to get actual value
    widget.sl_hit_thresh->setMaximum(90);

    connect(widget.sl_hit_thresh, &QSlider::valueChanged, 
            this, &SettingsWidget::onHitThresholdSliderChange);

    connect(widget.line_edit_hit_thresh, &QLineEdit::returnPressed,
            this, &SettingsWidget::onHitThresholdLineChange);

    //step size
    widget.sl_step_size->setMinimum(1); // multipied by 4 to get actual value
    widget.sl_step_size->setMaximum(4);

    connect(widget.sl_step_size, &QSlider::valueChanged,
            this, &SettingsWidget::onStepSizeSliderChange);

    connect(widget.line_edit_step_size, &QLineEdit::returnPressed,
            this, &SettingsWidget::onStepSizeLineChange);

    //padding
    widget.sl_padding->setMinimum(0); // multiplied by 8 to get actual value
    widget.sl_padding->setMaximum(4);

    connect(widget.sl_padding, &QSlider::valueChanged,
            this, &SettingsWidget::onPaddingSliderChange);

    connect(widget.line_edit_padding, &QLineEdit::returnPressed,
            this, &SettingsWidget::onPaddingLineChange);

    //scale factor
    widget.sl_scale_factor->setMinimum(105); // divided by 100 to get actual value
    widget.sl_scale_factor->setMaximum(130);

    connect(widget.sl_scale_factor, &QSlider::valueChanged,
            this, &SettingsWidget::onScaleFactorSliderChange);

    connect(widget.line_edit_scale_factor, &QLineEdit::returnPressed,
            this, &SettingsWidget::onScaleFactorLineChange);

    //mean shift grouping
    connect(widget.radio_on_mean_shift, &QRadioButton::clicked,
            this, &SettingsWidget::onMeanShiftRadioChange);

    connect(widget.radio_off_mean_shift, &QRadioButton::clicked,
            this, &SettingsWidget::onMeanShiftRadioChange);

    widget.line_edit_interval->setValidator(new QIntValidator(1, 10, this));
    widget.line_edit_duration->setValidator(new QIntValidator(1, 10, this));
    
    widget.line_edit_hit_thresh->setValidator(new QDoubleValidator(0.0, 0.9, 2, this));
    widget.line_edit_step_size->setValidator(new QIntValidator(4, 16, this));
    widget.line_edit_padding->setValidator(new QIntValidator(0, 32, this));
    widget.line_edit_scale_factor->setValidator(new QDoubleValidator(1.05, 1.3, 2, this));
    
    this->setToolTips();
    this->readGeneralSettings();
    this->readObjectDetectionSettings();
    this->readCoordinateSystemSettings();
}

SettingsWidget::~SettingsWidget()
{
}

void SettingsWidget::setToolTips()
{
    widget.group_obj_dtct_params->setToolTip("These parameters affect the behavior of the object detection node associated with each UAV.\n"
                                              "Mouse over each one for a description.");
    widget.group_obj_dtct_params->setToolTipDuration(10000);

    widget.lblHitThreshold->setToolTip("Controls the maximum Euclidian distance between the input HOG\n"
                                        "features and the classifying plane of the Support Vector Machine.\n"
                                        "Range: 0.0 - 0.9");
    widget.lblHitThreshold->setToolTipDuration(10000);

    widget.lblStepSize->setToolTip("Controls the number of pixels that the detection window jumps forward.\n"
                                    "Range: 4, 8, 12 or 16");
    widget.lblStepSize->setToolTipDuration(10000);

    widget.lblPadding->setToolTip("Controls the amount of padding around the region of interest that will\n"
                                   "be added before feature extraction.\n"
                                   "Range: 0, 8, 16 or 32");
    widget.lblPadding->setToolTipDuration(10000);

    widget.lblScaleFactor->setToolTip("Controls the percentage by which the image is shrunk on subsequent object detection passes.\n"
                                       "Range: 1.05 - 1.3");
    widget.lblScaleFactor->setToolTipDuration(10000);

    widget.lblMeanShiftGrouping->setToolTip("Controls overlapping bounding boxes to decide if the\n"
                                             "common space between boxes is an object of interest.");
    widget.lblMeanShiftGrouping->setToolTipDuration(10000);

    widget.line_edit_hit_thresh->setToolTip("Press Enter to apply changes");
    widget.line_edit_hit_thresh->setToolTipDuration(5000);

    widget.line_edit_step_size->setToolTip("Press Enter to apply changes");
    widget.line_edit_step_size->setToolTipDuration(5000);

    widget.line_edit_padding->setToolTip("Press Enter to apply changes");
    widget.line_edit_padding->setToolTipDuration(5000);

    widget.line_edit_scale_factor->setToolTip("Press Enter to apply changes");
    widget.line_edit_scale_factor->setToolTipDuration(5000);
}

void SettingsWidget::readGeneralSettings()
{
    QString ml = settings.GetMachineLearningType();
    if(ml == settings.val_machine_learning_online)
        widget.online_btn->setChecked(true);
    else
        widget.offline_btn->setChecked(true);
    ml_state = ml;

    QString vehicle_link = settings.GetVehicleLink();
    if(vehicle_link == settings.val_vehicle_link_nominal)
        widget.nominal_btn->setChecked(true);
    else if(vehicle_link == settings.val_vehicle_link_marginal)
        widget.marginal_btn->setChecked(true);
    else //poor
        widget.poor_btn->setChecked(true);

     //dont forget to disable the frequency box if nominal button is checked.
    if(widget.nominal_btn->isChecked())
        widget.frequency_groupbox->setEnabled(false);

    int interval = settings.GetInterval();
    if(interval != settings.val_interval_unspecified)
    {
        widget.interval_btn->setChecked(true);
        widget.line_edit_interval->setText(QString::number(interval));
        widget.line_edit_interval->setEnabled(true);
    }
    else
    {
        widget.random_btn->setChecked(true);
        widget.line_edit_interval->setEnabled(false);
    }

    int duration = settings.GetDuration();
    if(duration != settings.val_duration_unspecified)
    {
        widget.line_edit_duration->setText(QString::number(duration));
        widget.duration_check_box->setChecked(true);
    }
    else
    {
        widget.duration_check_box->setChecked(false);
        widget.line_edit_duration->setEnabled(false);
    }

    image_root_dir = settings.GetImagesRootDir();
    
    widget.line_edit_images_dir->setText(image_root_dir);
}

void SettingsWidget::writeGeneralSettings()
{
    QString ml = (widget.online_btn->isChecked() 
                    ? settings.val_machine_learning_online 
                    : settings.val_machine_learning_offline);
    ml_state = ml;
    settings.SetMachineLearningType(ml);
    
    QString vehicle_link;
    if(widget.nominal_btn->isChecked())
        vehicle_link = settings.val_vehicle_link_nominal;
    else if(widget.marginal_btn->isChecked())
        vehicle_link = settings.val_vehicle_link_marginal;
    else
        vehicle_link = settings.val_vehicle_link_poor;
    settings.SetVehicleLink(vehicle_link);

    if(vehicle_link != settings.val_vehicle_link_nominal) // write frequency settings to config
    {
        QString frequency;
        int interval;
        if(widget.interval_btn->isChecked())
        {
            frequency = settings.val_frequency_interval;
            interval = widget.line_edit_interval->text().toInt();
        }
        else
        {
            frequency = settings.val_frequency_random;
            interval = settings.val_interval_unspecified;
        }
        settings.SetFrequency(frequency);
        settings.SetInterval(interval);
        
        int duration = (widget.duration_check_box->isChecked()
                            ? widget.line_edit_duration->text().toInt()
                            : settings.val_duration_unspecified);
        settings.SetDuration(duration);
    }
    else // remove all frequency related settings from config
        settings.ClearFrequencyGroup();

    QString img_dir = widget.line_edit_images_dir->text();
    if(!img_dir.isNull())
    {
        settings.SetImagesRootDir(img_dir);
        image_root_dir = img_dir;
    }
}

bool SettingsWidget::validateGeneralSettings()
{
    //assure that text inputs are either enabled and not empty, or not enabled.
    //write settins only if the are disabled or are enabled and have valid input.
    //the settings dialogue stay open if these checks fail.
    if(widget.frequency_groupbox->isEnabled())
    {
        QString interval_text = widget.line_edit_interval->text();
        QString duration_text = widget.line_edit_duration->text();

        if(widget.line_edit_interval->isEnabled() && interval_text.isEmpty())
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

        if(widget.line_edit_duration->isEnabled() && duration_text.isEmpty())
        {
            std::cout << "tried to apply settings with duration checkbox checked but no duration specified\n";
            return false;
        }

        if(!duration_text.isEmpty())
        {   
            float duration = duration_text.toFloat(&ok);
            if(!ok || duration <= 0)
            {
                std::cout << "entered invalid duration: " << duration_text.toStdString() 
                    << std::endl;
                return false;
            }
        }
    }

    QString img_dir = widget.line_edit_images_dir->text();
    if(!img_dir.isEmpty())
    {
        if(!img_dir.startsWith('/'))
        {
            std::cout << "image root directory MUST be an absolute path\n";
            return false;
        }   
    }

    return true;
}

void SettingsWidget::readObjectDetectionSettings()
{
    QString node_loc = settings.GetNodeLocation();
    if(node_loc == settings.val_node_location_gcs)
        widget.gcs_btn->setChecked(true);
    else // settings.val_node_location_uav
        widget.uav_btn->setChecked(true);
    // todo apply logic for determining where node will be run

    od_params = settings.GetObjectDetectionParameters();
    widget.line_edit_hit_thresh->setText(QString::number(od_params.hit_thresh, 'f', 2));
    widget.sl_hit_thresh->setValue(od_params.hit_thresh * 100);

    widget.line_edit_step_size->setText(QString::number(od_params.step_size));
    widget.sl_step_size->setValue(od_params.step_size / 4);

    widget.line_edit_padding->setText(QString::number(od_params.padding));
    widget.sl_padding->setValue(od_params.padding / 8);

    widget.line_edit_scale_factor->setText(QString::number(od_params.scale_factor, 'f', 2));
    widget.sl_scale_factor->setValue(od_params.scale_factor * 100);

    if(od_params.mean_shift == true)
        widget.radio_on_mean_shift->setChecked(true);
    else 
        widget.radio_off_mean_shift->setChecked(true);
    this->onMeanShiftRadioChange();
}

void SettingsWidget::writeObjectDetectionSettings()
{
    QString node_location = (widget.uav_btn->isChecked()
                            ? settings.val_node_location_gcs
                            : settings.val_node_location_uav);
    settings.SetNodeLocation(node_location);
    
    settings.SetObjectDetectionParameters(od_params);
}

bool SettingsWidget::onApplyClicked()
{   
    if(!validateGeneralSettings()) 
        return false;

    QString ml_state_previous = ml_state;
    QString coordinate_previous = coordinate_system;
    QString image_dir_previous = image_root_dir;
    
    this->writeGeneralSettings(); // may update ml_state, coordinate_system, image_root_dir

    if(ml_state != ml_state_previous)
        emit UIAdapter::Instance()->SetMachineLearningMode(widget.online_btn->isChecked());

    if(coordinate_system != coordinate_previous)
        emit UIAdapter::Instance()->SetCoordinateSystem(coordinate_system);
    
    if(image_root_dir != image_dir_previous)
        emit UIAdapter::Instance()->SetImageRootDir(image_root_dir);
    
    this->writeObjectDetectionSettings();
    this->WriteCoordinateSystemSettings();
    emit sm->coordinatesReady();
    
    return true;
}

void SettingsWidget::WriteCoordinateSystemSettings()
{
    coordinate_system = (widget.global_btn->isChecked() 
                                ? settings.val_coordinate_system_global
                                : settings.val_coordinate_system_local);
    settings.SetCoordinateSystem(coordinate_system);

    if(coordinate_system == settings.val_coordinate_system_local)
    {
        QVector<Point> coordinates;
        sm->getCoordinates(coordinates);
        settings.SetCoordinateSystemArray(coordinates);
    }
    else
        settings.SetCoordinateSystemArray(QVector<Point>());
}

void SettingsWidget::readCoordinateSystemSettings()
{
    QString cs = settings.GetCoordinateSystem();
    if(cs == settings.val_coordinate_system_global)
        widget.global_btn->setChecked(true);
    else
        widget.local_btn->setChecked(true);
    coordinate_system = cs;
    
    QVector<Point> coordinates = settings.GetCoordinateSystemArray();
    for(const Point& p : coordinates)
    {
        QList<QStandardItem*> new_row;
        new_row.append(new QStandardItem(QString::number(p.x, 'f', 2)));
        new_row.append(new QStandardItem(QString::number(p.y, 'f', 2)));
        new_row.append(new QStandardItem(QString::number(p.z, 'f', 2)));
        sm->mdl_cs->appendRow(new_row);
    }
    
    widget.group_coordinates->setEnabled(widget.local_btn->isChecked());
}

void SettingsWidget::onOkClicked()
{
    if(onApplyClicked()) // see if the settings are valid and applied
        this->close();
}

void SettingsWidget::onCancelClicked()
{
    this->close();
}

void SettingsWidget::onToggleFrequencyGroup()
{
    widget.frequency_groupbox->setEnabled(!widget.nominal_btn->isChecked());

    std::cout << "frequency group "
            << (widget.frequency_groupbox->isEnabled() ? "enabled" : "disabled")
            << std::endl;
}

void SettingsWidget::onToggleIntervalLine()
{
    widget.line_edit_interval->setEnabled(widget.interval_btn->isChecked()); 

    std::cout << "interval text box "
            << (widget.interval_btn->isEnabled() ? "enabled" : "disabled")
            << std::endl;
}

void SettingsWidget::onToggleDurationLine()
{
    widget.line_edit_duration->setEnabled(widget.duration_check_box->isChecked());
    
    std::cout << "duration text box "
            << (widget.duration_check_box->isEnabled() ? "enabled" : "disabled")
            << std::endl;
}

void SettingsWidget::OnCoordinateSystemChange()
{
    bool global = widget.global_btn->isChecked();
    coordinate_system = global ? settings.val_coordinate_system_global:
                                 settings.val_coordinate_system_local;
    
    widget.group_coordinates->setEnabled(!global);

    
    std::cout << "coordinate system set to " << ((global) ? "global" : "local")
            << std::endl;
}

// object detection parameter adjustments

void SettingsWidget::onHitThresholdSliderChange(int new_thresh)
{
    double thresh = ( ((double)new_thresh) / 100 );
    if(od_params.hit_thresh != thresh)
    {
        od_params.hit_thresh = thresh;
        widget.line_edit_hit_thresh->setText(QString::number(thresh, 'f', 2));
        emit UIAdapter::Instance()->PublishHitThreshold(thresh);
    }
}

void SettingsWidget::onHitThresholdLineChange()
{
    QString s = widget.line_edit_hit_thresh->text();
    if(s.startsWith('.'))
        s = "0" % s;

    bool ok;
    double thresh = s.toDouble(&ok);
    if(!ok)
    {
        std::cout << "tried to enter invalid threshold value: " 
                  << s.toStdString() << std::endl;
        widget.line_edit_hit_thresh->clear();
        return;
    }

    int thresh_old = thresh;
    if(thresh < settings.val_hit_threshold_low)
        thresh = settings.val_hit_threshold_low;
    else if(thresh > settings.val_hit_threshold_high)
        thresh = settings.val_hit_threshold_high;

    if(thresh != thresh_old)
         widget.line_edit_hit_thresh->setText(QString::number(thresh, 'f', 2));

    od_params.hit_thresh = thresh;
    widget.sl_hit_thresh->setValue(thresh * 100);
}

void SettingsWidget::onStepSizeSliderChange(int new_step)
{
    new_step *= 4; // map to 4, 8, 12 or 16
    if(od_params.step_size != new_step)
    {
        od_params.step_size = new_step;
        widget.line_edit_step_size->setText(QString::number(new_step));
        emit UIAdapter::Instance()->PublishStepSize(new_step);
    }
}

void SettingsWidget::onStepSizeLineChange()
{
    QString s = widget.line_edit_step_size->text();

    bool ok;
    int step = s.toInt(&ok);
    if(!ok || step % 4 != 0)
    {
        std::cout << "tried to enter invalid step size value: "
                  << s.toStdString() << std::endl;
        widget.line_edit_step_size->clear();
        return;
    }

    int step_old = step;
    if(step < settings.val_step_size_low)
        step = settings.val_step_size_low;
    else if(step > settings.val_step_size_high)
        step = settings.val_step_size_high;

    if(step != step_old)
         widget.line_edit_step_size->setText(QString::number(step));

    od_params.step_size = step;
    widget.sl_step_size->setValue(step / 4);
}

void SettingsWidget::onPaddingSliderChange(int new_padding)
{
    new_padding *= 8; // map to 0, 8, 16, 24 or 32
    if(od_params.padding != new_padding)
    {
        od_params.padding = new_padding;
        widget.line_edit_padding->setText(QString::number(new_padding));
        emit UIAdapter::Instance()->PublishPadding(new_padding);
    }
}

void SettingsWidget::onPaddingLineChange()
{
    QString s = widget.line_edit_padding->text();

    bool ok;
    int padding = s.toInt(&ok);
    if(!ok || padding % 8 != 0)
    {
        std::cout << "tried to enter invalid padding value: " 
                  << s.toStdString() << std::endl;
        widget.line_edit_padding->clear();
        return;
    }

    int padding_old = padding;
    if(padding < settings.val_padding_low)
        padding = settings.val_padding_low;
    else if(padding > settings.val_padding_high)
        padding = settings.val_padding_high;

    if(padding != padding_old)
        widget.line_edit_padding->setText(QString::number(padding));

    od_params.padding = padding;
    widget.sl_padding->setValue(padding / 8);
}

void SettingsWidget::onScaleFactorSliderChange(int new_scale)
{
    double scale = ( ((double)new_scale) / 100 );
    if(od_params.scale_factor != scale)
    {
        od_params.scale_factor = scale;
        widget.line_edit_scale_factor->setText(QString::number(scale, 'f', 2));
        emit UIAdapter::Instance()->PublishScaleFactor(scale);
    }
}

void SettingsWidget::onScaleFactorLineChange()
{   
    QString s = widget.line_edit_scale_factor->text();
    if(s.startsWith('.'))
        s = "1" % s;

    bool ok;
    double scale = s.toDouble(&ok);
    if(!ok)
    {
        std::cout << "tried to enter invalid scale value: " 
                  << s.toStdString() << std::endl;
        widget.line_edit_scale_factor->clear();
        return;
    }

    int scale_old = scale;
    if(scale < settings.val_scale_factor_low)
        scale = settings.val_scale_factor_low;
    else if(scale > settings.val_scale_factor_high)
        scale = settings.val_scale_factor_high;

    if(scale != scale_old)
        widget.line_edit_scale_factor->setText(QString::number(scale, 'f', 2));

    od_params.scale_factor = scale;
    widget.sl_scale_factor->setValue(scale * 100);
}

void SettingsWidget::onMeanShiftRadioChange()
{
    bool on = widget.radio_on_mean_shift->isChecked();
    if(od_params.mean_shift != on)
    {
        od_params.mean_shift = on;
        emit UIAdapter::Instance()->PublishMeanShift(od_params.mean_shift);
    }
}

void SettingsWidget::onTableViewContextMenuRequested(const QPoint& p)
{
    if(!menu)
    {
        menu = new QMenu(this);
        QAction * insert_before = menu->addAction("Insert row before here");
        QAction * insert_after = menu->addAction("Insert row after here");
        QAction* delete_rows = menu->addAction("Delete selected rows");
        
        QObject::connect(insert_before, &QAction::triggered,
        this, 
        [=]()
        {
            addOrDeleteSelectedRows(true);
        });
                    
    QObject::connect(insert_after, &QAction::triggered,
        this,
        [=]()
        {
            addOrDeleteSelectedRows(true, 1);
        });
        
    QObject::connect(delete_rows, &QAction::triggered,
        this, &SettingsWidget::onDeleteRowClicked);
    }
    
    menu->popup(widget.tablev_coordinates->viewport()->mapToGlobal(p));
}

void SettingsWidget::onAddRowClicked()
{
    sm->mdl_cs->appendRow(new QStandardItem());
}

void SettingsWidget::onDeleteRowClicked()
{
    if(sm->mdl_cs->rowCount() == 0)
        return;
    
    addOrDeleteSelectedRows(false);
}

void SettingsWidget::addOrDeleteSelectedRows(bool add, int row_offset)
{
    QItemSelectionModel * selection = widget.tablev_coordinates->selectionModel();
    QModelIndexList indexes = selection->selectedIndexes();
    QMap<int, int> row_map;
    for(const QModelIndex& index : indexes)
    {
        int row = index.row();
        if(!row_map.contains(row))
            row_map.insert(row, row);
    }
    
    if(row_map.isEmpty())
    {
        int last_row = (sm->mdl_cs->rowCount() - 1);
        row_map.insert(last_row, last_row);
    }
    
    QMap<int, int>::const_iterator it = row_map.constEnd();
    if(add)
    {
        for(; it != row_map.constBegin();)
        {
            sm->mdl_cs->insertRow((--it).value() + row_offset);
        }
    }
    else
    {
        for(; it != row_map.constBegin();)
        {
            sm->mdl_cs->removeRow((--it).value());
        }
    }
}

}// end name space