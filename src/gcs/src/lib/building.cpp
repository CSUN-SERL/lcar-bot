
/* 
 * File:   Building.cpp
 * Author: serl
 * 
 * Created on July 28, 2017, 2:19 PM
 */

#include <gcs/util/building.h>

namespace gcs
{

Building::Building() :
_type(bNULL)
{
    
}
    
Building::Building(BuildingType t) :
_type(t)
{ }

void Building::spaceDown()
{
    _space_count++;
    _space_down = true;
}

void Building::spaceUp()
{
    _space_down = false;
}

int Building::spaceCount()
{
    return _space_count;
}

Building::FoundBy Building::foundBy()
{
    return _found_by;
}

void Building::setFoundBy(FoundBy f)
{
    _found_by = f;
}

Building::BuildingType Building::buildingType()
{
    return _type;   
}

void Building::setBuldingType(BuildingType t)
{
    _type = t;
}

void Building::setDoorWall(int wall)
{
    _door_wall = wall;
}

int Building::doorWall()
{
    return _door_wall;
}

}

