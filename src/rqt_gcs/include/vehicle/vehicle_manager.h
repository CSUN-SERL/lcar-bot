
/* 
 * File:   vehicle_manager.h
 * Author: n8
 *
 * Created on September 8, 2016, 11:41 AM
 */

#ifndef VEHICLEMANAGER_H
#define VEHICLEMANAGER_H

#include <QMap>
#include <QObject>

#include "vehicle/vehicle_control.h"
#include "util/data_types.h"

namespace rqt_gcs
{
    
class VehicleManager : QObject
{
    Q_OBJECT
public:
    VehicleManager(QObject *parent=0);
    virtual ~VehicleManager();
    void AddUGV(int id);
    void AddQuadRotor(int id);
    void AddOctoRotor(int id);
    void AddVTOL(int id);
    
    void DeleteUGV(int id);
    void DeleteQuadRotor(int id);
    void DeleteOctoRotor(int id);
    void DeleteVTOL(int id);
    
    QString VehicleString(int id);
    
private:
    QMap<int, VehicleControl*> db; //the database
    
    VehicleControl* EraseVehicleFromDB(int id);
    
    int NUM_UGV, 
        NUM_QUAD,
        NUM_OCTO,
        NUM_VTOL;
};

}

#endif /* VEHICLEMANAGER_H */

