
/* 
 * File:   settings.h
 * Author: n8
 *
 * Created on November 18, 2016, 11:49 AM
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include <QString>
#include <QSettings>
#include <QStringList>

#include "util/object_detection_parameters.h"

namespace gcs
{

class Settings {
public:
    Settings();
    virtual ~Settings();

    void SetMachineLearningType(const QString ml_type);
    QString GetMachineLearningType();
    
    void SetCoordinateSystem(const QString system);
    QString GetCoordinateSystem();
    
    void SetVehicleLink(const QString vehicle_link);
    QString GetVehicleLink();
    
    void SetFrequency(const QString frequency);
    QString GetFrequency();
    
    void SetInterval(const int minutes);
    int GetInterval();
    
    void SetDuration(const int minutes);
    int GetDuration();
    
    void SetImagesRootDir(const QString dir);
    QString GetImagesRootDir();
    
    void SetNodeLocation(const QString location);
    QString GetNodeLocation();
    
    //object detection tab
    ObjectDetectionParameters GetObjectDetectionParams();

    static const QString val_machine_learning_online,
                         val_machine_learning_offline;
    
    static const QString val_coordinate_system_global,
                         val_coordinate_system_local;
    
    static const QString val_vehicle_link_nominal,
                         val_vehicle_link_marginal,
                         val_vehicle_link_poor;

    static const QString val_frequency_random,
                         val_frequency_interval;
    
    static const int val_interval_unspecified;
    static const int val_duration_unspecified;
    
    static const QString val_node_location_uav,
                         val_node_location_gcs;
    
    static const double val_hit_threshold_high,
                        val_hit_threshold_low;
    
    static const int val_step_size_high,
                     val_step_size_low;
    
    static const int val_padding_high,
                     val_padding_low;
    
    static const double val_scale_factor_high,
                        val_scale_factor_low;
        
    static const bool val_mean_shift_on,
                      val_mean_shift_off;
    
    
private:
    
    void Write(const QString key, const QVariant value, QStringList groups = QStringList(), int index = 0);
    QVariant Read(QString key, const QVariant default_value, QStringList groups = QStringList(), int index = 0);
    
    QSettings settings;
    
    const static QString group_general;
    const static QString group_connection;
    
    const static QString group_object_detection;
    const static QString group_tuning_params;
    
    const static QString key_machine_learning;
        
    const static QString key_coordinate_system;
        
    const static QString key_vehicle_link;
    const static QString key_frequency;
    const static QString key_interval;
    const static QString key_duration;
            
    const static QString key_image_root_dir;
    
    const static QString key_node_location;

    const static QString key_hit_threshold;
    const static QString key_step_size;
    const static QString key_padding;
    const static QString key_scale_factor;
    const static QString key_mean_shift;
    
};

}
#endif /* SETTINGS_H */

