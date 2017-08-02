
/* 
 * File:   TrialLoader.h
 * Author: n8
 *
 * Created on July 29, 2017, 8:18 PM
 */

#ifndef TRIALLOADER_H
#define TRIALLOADER_H

#include <memory>
#include <QMap>
#include <gcs/util/building.h>

namespace gcs
{
    
class Building;    

struct WaypointInfo
{
    double building_id;
    double x;
    double y;
    double z;
    int yaw;
};

class TrialLoader 
{
public:
    
    enum Condition
    {
        Null,
        Predictable,
        UnPredictable
    };
    
public:
    bool load(Condition c, int trial);
    bool loadBuildings(Condition c, int trial);
    bool loadWaypoints(Condition c, int trial);
    
    const QMap<int, std::shared_ptr<Building> >& getBuildings() const;
    const QList< std::shared_ptr<WaypointInfo> >& getWaypointInfoList() const;
    void reset();
    
    bool isValid()
    {
        return !_buildings.isEmpty() && !_waypoints.isEmpty();
    }
    
private:
    QMap<int, std::shared_ptr<Building> > _buildings;
    QList< std::shared_ptr<WaypointInfo> > _waypoints;
};

}

#endif /* TRIALLOADER_H */

