
/* 
 * File:   Building.h
 * Author: serl
 *
 * Created on July 28, 2017, 2:19 PM
 */

#ifndef BUILDING_H
#define BUILDING_H

namespace gcs 
{
    
class Building 
{
    enum BuildingType
    {
        bNULL = 0,
        bPurple,
        bWhite
    };
    
    enum FoundBy
    {
        fNULL = 0,
        fOperator,
        fVehicle
    };
    
public:
    Building();
    Building(BuildingType t);
    
    void spaceDown();
    void spaceUp();
    int spaceCount();
    
    FoundBy foundBy();
    void setFoundBy(FoundBy f);
    
    BuildingType buildingType();
    void setBuldingType(BuildingType t);
    
    void setDoorWall(int wall);
    int doorWall();
    
private:
    
    bool _space_down = false;
    int _space_count = 0;
    int _door_wall = -1;
    
    BuildingType _type = bNULL;
    FoundBy _found_by = fNULL;
    
};

}

#endif /* BUILDING_H */

