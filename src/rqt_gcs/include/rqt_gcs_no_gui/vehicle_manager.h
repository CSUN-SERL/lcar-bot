
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

#include "rqt_gcs_no_gui/uav_control.h"
#include "rqt_gcs_no_gui/data_types.h"

namespace rqt_gcs
{

class VehicleManager
{
public:
    VehicleManager(QObject *parent=0);
    virtual ~VehicleManager();
    void AddUGV(int id);
    void AddQuadRotor(int id);
    void AddOctoRotor(int id);
    void AddVTOL(int id);
    
    
private:
    QMap<VehicleType, QVector<VehicleControl*>> db; //the database
    
    void AddVehicleByType(VehicleType type, VehicleControl* vehicle);
    int NUM_UGV, 
        NUM_QUAD,
        NUM_OCTO,
        NUM_VTOL,
        NUM_VEHICLES;
};

}

#endif /* VEHICLEMANAGER_H */

