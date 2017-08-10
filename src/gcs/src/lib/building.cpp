
/* 
 * File:   Building.cpp
 * Author: serl
 * 
 * Created on July 28, 2017, 2:19 PM
 */

#include <gcs/util/building.h>

namespace gcs
{

Building::Building()
{
    for(int i = 0; i < 3; i++)
    {
        _space_count_by_wall[i] = 0;
    }
}

void Building::setLocation(float x, float y)
{
    _x = x;
    _y = y;
}

float Building::xPos()
{
    return _x;
}

float Building::yPos()
{
    return _y;
}

void Building::setID(int id)
{
    _id = id;
}

int Building::getID()
{
    return _id;
}

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

Building::Type Building::buildingType()
{
    return _type;   
}

void Building::setBuldingType(Type t)
{
    _type = t;
}

void Building::setDoorLocation(int wall)
{
    _door_location = wall;
}

int Building::doorLocation()
{
    return _door_location;
}

void Building::setDoors(const QMap<int, int>& doors)
{
    _doors = doors;
}

const QMap<int, int>& Building::doors()
{
    return _doors;
}
    
void Building::setWindows(const QMap<int, int>& windows)
{
    _windows = windows;
}

const QMap<int, int>& Building::windows()
{
    return _windows;
}

void Building::setDoorPrompt(int wall)
{
    _door_prompt = wall;
}

int Building::doorPrompt()
{
    return _door_prompt;
}

void Building::setDoorMissing(int wall)
{
    _door_missing = wall;
}

int Building::doorMissing()
{
    return _door_missing;
}

void Building::setFalsePrompt(int wall)
{
    _false_prompt = wall;
}

int Building::falsePrompt()
{
    return _false_prompt;
}

}

