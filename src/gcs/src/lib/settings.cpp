
/* 
 * File:   settings.cpp
 * Author: n8
 * 
 * Created on November 18, 2016, 11:49 AM
 */

#include <QSettings>
#include <QStringBuilder>
#include <QCoreApplication>
#include <QProcessEnvironment>

#include <gcs/util/settings.h>
#include <gcs/util/object_detection_parameters.h>

namespace gcs
{
    
//public:///////////////////////////////////////////////////////////////////////
Settings::Settings()
{
}

Settings::~Settings()
{  
}

void Settings::SetMachineLearningType(const QString ml_type)
{
    QString ml = ml_type.toLower();
    Q_ASSERT(ml == Settings::val_machine_learning_offline 
                || ml == Settings::val_machine_learning_online);
    
    QStringList groups = QStringList() << Settings::group_general;
    this->Write(Settings::key_machine_learning, ml, groups);
}

QString Settings::GetMachineLearningType()
{
    QStringList groups = QStringList() << Settings::group_general;
    return this->Read(Settings::key_machine_learning, 
                        Settings::val_machine_learning_online,
                        groups).toString();
}

void Settings::SetCoordinateSystem(const QString system)
{
    QString new_system = system.toLower();
    Q_ASSERT(new_system == Settings::val_coordinate_system_global
                || new_system == Settings::val_coordinate_system_local);
    
    QStringList groups = QStringList() << Settings::group_general;
    this->Write(Settings::key_coordinate_system, new_system, groups);
}

QString Settings::GetCoordinateSystem()
{
    QStringList groups = QStringList() << Settings::group_general;
    return this->Read(Settings::key_coordinate_system, 
                      Settings::val_coordinate_system_local,
                      groups).toString();
}

void Settings::SetVehicleLink(const QString vehicle_link)
{
    QString link = vehicle_link.toLower();
    Q_ASSERT(link == Settings::val_vehicle_link_nominal
                || link == Settings::val_vehicle_link_marginal
                || link == Settings::val_vehicle_link_poor);
    
    QStringList groups = QStringList() << Settings::group_general << Settings::group_connection;
    this->Write(Settings::key_vehicle_link, link, groups);
}

QString Settings::GetVehicleLink()
{
    QStringList groups = QStringList() << Settings::group_general << Settings::group_connection;
    return this->Read(Settings::key_vehicle_link, 
                        Settings::val_vehicle_link_nominal,
                        groups).toString();
}

void Settings::SetFrequency(const QString frequency)
{
    QString freq = frequency.toLower();
    Q_ASSERT(freq == Settings::val_frequency_interval
                || freq == Settings::val_frequency_random);
    QStringList groups = QStringList() << Settings::group_general << Settings::group_connection;
    this->Write(Settings::key_frequency, freq, groups);
}

QString Settings::GetFrequency()
{
    QStringList groups = QStringList() << Settings::group_general << Settings::group_connection;
    return this->Read(Settings::key_frequency, 
                        Settings::val_frequency_random,
                        groups).toString();
}

void Settings::SetInterval(const int minutes)
{
    Q_ASSERT(minutes >= Settings::val_interval_unspecified);
    QStringList groups = QStringList() << Settings::group_general << Settings::group_connection;
    this->Write(Settings::key_interval, QString::number(minutes), groups);
}

int Settings::GetInterval()
{
    QStringList groups = QStringList() << Settings::group_general << Settings::group_connection;
    return this->Read(Settings::key_interval, 
                        QString::number(Settings::val_interval_unspecified),
                        groups).toInt();
}

void Settings::SetDuration(const int minutes)
{
    Q_ASSERT(minutes >= Settings::val_duration_unspecified);
    QStringList groups = QStringList() << Settings::group_general << Settings::group_connection;
    this->Write(Settings::key_duration, QString::number(minutes), groups);
}

int Settings::GetDuration()
{
    QStringList groups = QStringList() << Settings::group_general << Settings::group_connection;
    return this->Read(Settings::key_duration,
                        QString::number(Settings::val_duration_unspecified),
                        groups).toInt();
}

void Settings::ClearFrequencyGroup()
{
    settings.beginGroup(Settings::group_general);
    settings.beginGroup(Settings::group_connection);
    
    settings.remove(Settings::key_frequency);
    settings.remove(Settings::key_interval);
    settings.remove(Settings::key_duration);
    
    settings.endGroup();
    settings.endGroup();
}

void Settings::SetImagesRootDir(const QString dir)
{
    Q_ASSERT(dir.startsWith("/"));
    QStringList groups = QStringList() << Settings::group_general;
    this->Write(Settings::key_image_root_dir, dir, groups);
}

QString Settings::GetImagesRootDir()
{
    QStringList groups = QStringList() << Settings::group_general;
    QString default_path = QProcessEnvironment::systemEnvironment().value("HOME") 
                            % "/Pictures/" % QCoreApplication::applicationName();
    return this->Read(Settings::key_image_root_dir,
                        default_path, 
                        groups).toString();
}

void Settings::SetNodeLocation(const QString location)
{
    QString temp_loc = location.toLower();
    Q_ASSERT(temp_loc == Settings::val_node_location_gcs
                || temp_loc == Settings::val_node_location_uav);
    QStringList groups = QStringList() << Settings::group_object_detection;
    this->Write(Settings::key_node_location, temp_loc, groups);
}

QString Settings::GetNodeLocation()
{
    QStringList groups = QStringList() << Settings::group_object_detection;
    return this->Read(Settings::key_node_location,
                        Settings::val_node_location_gcs,
                        groups).toString();
}

void Settings::SetObjectDetectionParameters(ObjectDetectionParameters& od_params)
{
    settings.beginGroup(Settings::group_object_detection);
    settings.beginGroup(Settings::group_tuning_params);
    
    settings.setValue(Settings::key_hit_threshold, QString::number(od_params.hit_thresh, 'f', 2));
    settings.setValue(Settings::key_step_size, od_params.step_size);
    settings.setValue(Settings::key_padding, od_params.padding);
    settings.setValue(Settings::key_scale_factor, QString::number(od_params.scale_factor, 'f', 2));
    settings.setValue(Settings::key_mean_shift, od_params.mean_shift);
    
    settings.endGroup();
    settings.endGroup();
}

ObjectDetectionParameters Settings::GetObjectDetectionParameters()
{
    ObjectDetectionParameters od_params;
    settings.beginGroup(Settings::group_object_detection);
    settings.beginGroup(Settings::group_tuning_params);
    
    od_params.hit_thresh = settings.value(Settings::key_hit_threshold, 
                                          QString::number(Settings::val_hit_threshold_low)).toDouble();
    od_params.step_size = settings.value(Settings::key_step_size,
                                         QString::number(Settings::val_step_size_low)).toInt();
    od_params.padding = settings.value(Settings::key_padding, 
                                       QString::number(Settings::val_padding_low)).toInt();
    od_params.scale_factor = settings.value(Settings::key_scale_factor,
                                            QString::number(Settings::val_scale_factor_low)).toDouble();
    od_params.mean_shift = settings.value(Settings::key_mean_shift,
                                          Settings::val_mean_shift_on).toBool();
    
    settings.endGroup();
    settings.endGroup();
    
    return od_params;
}

void Settings::Write(const QString key, const QVariant value, const QStringList groups, int index)
{
    if(index == groups.size())
    {
        settings.setValue(key, value);
        return;
    }
    
    settings.beginGroup(groups[index]);
    this->Write(key, value, groups, index+1);
    settings.endGroup();  
}

QVariant Settings::Read(const QString key, const QVariant default_value, const QStringList groups, int index)
{
    if(index == groups.size())
        return settings.value(key, default_value);

    settings.beginGroup(groups[index]);
    QVariant var = this->Read(key, default_value, groups, index + 1);
    settings.endGroup();
    
    return var;
}

//variables
const QString Settings::group_general = "general";
const QString Settings::group_connection = "connection_drop";

const QString Settings::group_object_detection = "object_detection";
const QString Settings::group_tuning_params = "tuning_parameters";

const QString Settings::key_machine_learning = "machine_learning";
const QString Settings::val_machine_learning_online = "online";
const QString Settings::val_machine_learning_offline = "offline";

const QString Settings::key_coordinate_system = "coordinate_system";
const QString Settings::val_coordinate_system_global = "global";
const QString Settings::val_coordinate_system_local = "local";

const QString Settings::key_vehicle_link = "vehicle_gcs_link";
const QString Settings::val_vehicle_link_nominal = "nominal";
const QString Settings::val_vehicle_link_marginal = "marginal";
const QString Settings::val_vehicle_link_poor = "poor";

const QString Settings::key_frequency = "frequency";
const QString Settings::val_frequency_random = "random";
const QString Settings::val_frequency_interval = "interval";

const QString Settings::key_interval = "interval";
const int Settings::val_interval_unspecified = -1;

const QString Settings::key_duration = "duration";
const int Settings::val_duration_unspecified = -1;

const QString Settings::key_image_root_dir = "images_root_directory";

const QString Settings::key_node_location = "node_location";
const QString Settings::val_node_location_uav = "uav";
const QString Settings::val_node_location_gcs = "gcs";

const QString Settings::key_hit_threshold = "hit_threshold";
const double Settings::val_hit_threshold_high = 0.9;
const double Settings::val_hit_threshold_low = 0.0;

const QString Settings::key_step_size = "step_size";
const int Settings::val_step_size_high = 16;
const int Settings::val_step_size_low = 0;

const QString Settings::key_padding = "padding";
const int Settings::val_padding_high = 32;
const int Settings::val_padding_low = 0;

const QString Settings::key_scale_factor = "scale_factor";
const double Settings::val_scale_factor_high = 1.3;
const double Settings::val_scale_factor_low = 1.0;

const QString Settings::key_mean_shift = "mean_shift_grouping";
const bool Settings::val_mean_shift_on = true;
const bool Settings::val_mean_shift_off = false;

}//namespace gcs