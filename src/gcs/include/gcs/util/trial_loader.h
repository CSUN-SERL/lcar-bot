
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
#include <gcs/qt/building.h>

namespace gcs
{
    
class Building;    

struct WaypointInfo
{
    int building_id;
    float x;
    float y;
    float z;
    int yaw;
    
    bool isBuildingWP()
    {
        return building_id != -1;
    }

    double distanceTo(const std::shared_ptr<WaypointInfo>& other)
    {
        if(!other)
            return 0;

        float dist_x = x - other->x;
        float dist_y = y - other->y;
        float dist_z = z - other->z;

        return std::sqrt((dist_x * dist_x) + (dist_y * dist_y) + (dist_z * dist_z));
    }
    
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
    bool loadWaypoints(Condition c, int trial);
    bool loadBuildings(Condition c, int trial);
    
    const QMap<int, std::shared_ptr<Building> >& getBuildings() const;
    const QList< std::shared_ptr<WaypointInfo> >& getWaypoints() const;
    void reset();
    
    bool isValid()
    {
        return !_buildings.isEmpty() && !_waypoints.isEmpty();
    }
    
private:
    void setWallContent(const std::shared_ptr<Building> b, int& i, const QStringList& list);
    void setPromptInfo(const std::shared_ptr<Building> b, int& i, const QStringList& list);
    
private:
    QString trimEOLComment(const QString& entry);
    
private:
    QMap<int, std::shared_ptr<Building> > _buildings;
    QList< std::shared_ptr<WaypointInfo> > _waypoints;
};

}

#endif /* TRIALLOADER_H */

