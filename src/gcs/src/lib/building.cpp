
/* 
 * File:   Building.cpp
 * Author: serl
 * 
 * Created on July 28, 2017, 2:19 PM
 */

#include <gcs/util/building.h>

namespace gcs
{

int Building::MAX_PROMPTS_PER_WALL = 2;
    
int Building::targetYawToWall(int yaw)
{
    switch(yaw)
    {
        case 0: 
            return 2;
        case 90:
            return 3;
        case 180:
            return 0;
        case 270:
            return 1;
        default:
            break;
    }
    return -1;
}
    
Building::Building()
{
    for(int i = 0; i < 4; i++)
    {
        _space_count_by_wall[i] = 0;
        _prompt_count[i] = 0;
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

//void Building::setDoorLocation(int wall)
//{
//    _door_location = wall;
//}
//
//int Building::doorLocation()
//{
//    return _door_location;
//}

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

void Building::setDoorPrompts(const QMap<int, int>& door_prompts)
{
    _door_prompts = door_prompts;
}

const QMap<int, int>& Building::doorPrompts()
{
    return _door_prompts;
}

void Building::setWindowPrompts(const QMap<int, int>& window_prompts)
{
    _window_prompts = window_prompts;
}

const QMap<int, int>& Building::windowPrompts()
{
    return _window_prompts;
}

//void Building::setPromptAnswer(PromptAnswer answer)
//{
//    _answer = answer;
//}
//
//PromptAnswer Building::promptAnswer()
//{
//    return _answer;
//}

void Building::incrementSpaceCountForWall(int i)
{
    int val = _space_count_by_wall[i];
    _space_count_by_wall[i] = val + 1;
}

const QMap<int, int>& Building::spaceCountPerWall()
{
    return _space_count_by_wall;
}

void Building::wallQueried(int wall)
{
    int count = _prompt_count[wall];
    if(count < MAX_PROMPTS_PER_WALL)
        _prompt_count[wall] = count + 1;
}

int Building::queryCountForWall(int wall)
{
    return _prompt_count.value(wall, -1);
}

int Building::maxQueriesPerWall()
{
    return MAX_PROMPTS_PER_WALL;
}


//void Building::setDoorPrompt(int wall)
//{
//    _door_prompt = wall;
//}
//
//int Building::doorPrompt()
//{
//    return _door_prompt;
//}
//
//void Building::setDoorMissing(int wall)
//{
//    _door_missing = wall;
//}
//
//int Building::doorMissing()
//{
//    return _door_missing;
//}
//
//void Building::setFalsePrompt(int wall)
//{
//    _false_prompt = wall;
//}
//
//int Building::falsePrompt()
//{
//    return _false_prompt;
//}

}

