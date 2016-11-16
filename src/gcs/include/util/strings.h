
/* 
 * File:   strings.h
 * Author: serl
 *
 * Created on November 4, 2016, 12:03 PM
 */

#ifndef STRINGS_H
#define STRINGS_H

#include <QString>

namespace gcs
{
    extern QString company_; // the name of the company for QSettings
    extern QString application_; // the name of the application for QSettings

    namespace img
    {
        extern QString image_root_dir_; // the root directory of for saving images by GCS
    }
    
    typedef struct FlightModes_ // flight modes for use by different User Interfaces
    {
        QString stabilized;// = "STABILIZED";
        QString loiter;// = "AUTO.LOITER";
        QString land;// = "AUTO.LAND";
        QString altitude_control;// = "ALTCTL";
        QString position_control;// = "POSCTL";
        QString mode_auto;// = "AUTO";
        QString offboard;// = "OFFBOARD";
    } FlightModes;
    
    extern FlightModes flight_modes_;
    
    void InitStrings();
}

#endif /* STRINGS_H */

