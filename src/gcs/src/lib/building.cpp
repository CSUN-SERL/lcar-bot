
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
{}

void Building::setLocation(double x, double y)
{
    _x = x;
    _y = y;
}

int Building::xPos()
{
    return _x;
}
int Building::yPos()
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

