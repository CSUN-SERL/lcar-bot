
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
    extern const QString company_; // the name of the company for QSettings
    extern const QString application_; // the name of the application for QSettings

    extern QString image_root_dir_; // the root directory for saving images by GCS

    namespace FlightModes // flight modes for use by different User Interfaces
    {
        const QString stabilized = "STABILIZED";
        const QString loiter = "AUTO.LOITER";
        const QString land = "AUTO.LAND";
        const QString altitude_control = "ALTCTL";
        const QString position_control = "POSCTL";
        const QString mode_auto = "AUTO";
        const QString offboard = "OFFBOARD";
    };
}

#endif /* STRINGS_H */

