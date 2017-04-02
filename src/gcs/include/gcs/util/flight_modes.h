
/* 
 * File:   flight_modes.h
 * Author: n8
 *
 * Created on November 4, 2016, 12:03 PM
 */

#ifndef STRINGS_H
#define STRINGS_H

#include <QString>

namespace gcs
{
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

