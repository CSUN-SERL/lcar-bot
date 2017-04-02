
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

#include "gcs/util/object_detection_parameters.h"

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
    
    void ClearFrequencyGroup();
    
    void SetImagesRootDir(const QString dir);
    QString GetImagesRootDir();
    
    void SetNodeLocation(const QString location);
    QString GetNodeLocation();
    
    //object detection tab
    void SetObjectDetectionParameters(ObjectDetectionParameters& od_params);
    ObjectDetectionParameters GetObjectDetectionParameters();
    
    void Write(const QString key, const QVariant value, QStringList groups = QStringList(), int index = 0);
    QVariant Read(QString key, const QVariant default_value, QStringList groups = QStringList(), int index = 0);

    //variables
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
    
    QSettings settings;
    
    static const QString group_general;
    static const QString group_connection;
    
    static const QString group_object_detection;
    static const QString group_tuning_params;
    
    static const QString key_machine_learning;
        
    static const QString key_coordinate_system;
        
    static const QString key_vehicle_link;
    static const QString key_frequency;
    static const QString key_interval;
    static const QString key_duration;
            
    static const QString key_image_root_dir;
    
    static const QString key_node_location;

    static const QString key_hit_threshold;
    static const QString key_step_size;
    static const QString key_padding;
    static const QString key_scale_factor;
    static const QString key_mean_shift;
    
};

}
#endif /* SETTINGS_H */

