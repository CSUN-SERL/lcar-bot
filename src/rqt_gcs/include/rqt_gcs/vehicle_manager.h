
/* 
 * File:   vehicle_manager.h
 * Author: n8
 *
 * Created on September 8, 2016, 11:41 AM
 */

#ifndef VEHICLEMANAGER_H
#define VEHICLEMANAGER_H

#include <QMap>
#include <QVector>
#include <QObject>

#include "rqt_gcs/vehicle_control.h"
#include "rqt_gcs/uav_control.h"

namespace rqt_gcs
{

enum VehicleType
{
    ugv=1000,        //corresponds to the vehicle id range for this vehicle type
    quad_rotor=2000,
    octo_rotor=3000,
    vtol=4000
};

class VehicleManager : public QObject
{
    Q_OBJECT
public:
    VehicleManager();
    virtual ~VehicleManager();
    void AddUGV(int id);
    void AddQuadRotor(int id);
    void AddOctoRotor(int id);
    void AddVTOL(int id);
private:
    QMap<VehicleType, QVector<VehicleControl*>> db; //the database
    void AddVehicleByType(VehicleType type, int id);
    int NUM_UGV, 
        NUM_QUAD,
        NUM_OCTO,
        NUM_VTOL;
};

}

#endif /* VEHICLEMANAGER_H */

