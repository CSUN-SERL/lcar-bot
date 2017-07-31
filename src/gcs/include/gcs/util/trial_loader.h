
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
};

class TrialLoader 
{
public:
    
    enum Condition
    {
        Predictable,
        UnPredictable
    };
    
public:
    void load(Condition c, int trial);
    void loadBuildings(Condition c, int trial);
    void loadWaypoints(Condition c, int trial);
    
    const QList< std::shared_ptr<Building> >& getBuildings() const;
    const QList< std::shared_ptr<WaypointInfo> >& getWaypointInfoList() const;
    void reset();
    
    bool isValid()
    {
        return !_buildings.isEmpty() && !_waypoints.isEmpty();
    }
    
private:
    QList< std::shared_ptr<Building> > _buildings;
    QList< std::shared_ptr<WaypointInfo> > _waypoints;
};

}

#endif /* TRIALLOADER_H */

