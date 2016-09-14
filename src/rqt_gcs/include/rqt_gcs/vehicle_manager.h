
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

#include "rqt_gcs/uav_control.h"
#include "rqt_gcs/data_types.h"

namespace rqt_gcs
{

class VehicleManager : public QObject
{
    Q_OBJECT
public:
    VehicleManager(QObject *parent=0);
    virtual ~VehicleManager();
    void AddUGV(int id);
    void AddQuadRotor(int id);
    void AddOctoRotor(int id);
    void AddVTOL(int id);
    
    
private:
    QMap<rqt_gcs::VehicleType, QVector<VehicleControl*>> db; //the database
    
    void AddVehicleByType(VehicleType type, VehicleControl* vehicle);
    int NUM_UGV, 
        NUM_QUAD,
        NUM_OCTO,
        NUM_VTOL,
        NUM_VEHICLES;
};

}

#endif /* VEHICLEMANAGER_H */

