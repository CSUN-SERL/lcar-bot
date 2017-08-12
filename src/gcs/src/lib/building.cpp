
/* 
 * File:   Building.cpp
 * Author: serl
 * 
 * Created on July 28, 2017, 2:19 PM
 */

#include <gcs/qt/building.h>

namespace gcs
{

int Building::MAX_PROMPTS_PER_WALL_DOOR = 1;
int Building::MAX_PROMPTS_PER_WALL_WINDOW = 1;
    
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
        _prompt_count_doors[i] = 0;
        _prompt_count_windows[i] = 0;
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

void Building::setID(BuildingID id)
{
    _id = id;
}

int Building::getID()
{
    return _id;
}

void Building::spaceDown(int wall)
{
    _space_count++;
    _space_down = true;
    incrementSpaceCountForWall(wall);
}

void Building::spaceUp()
{
    _space_down = false;
}

int Building::spaceCount()
{
    return _space_count;
}

FoundBy Building::foundBy()
{
    return _found_by;
}

void Building::setFoundBy(FoundBy f)
{
    if(_found_by != f)
    {
        _found_by = f;
        emit foundByChanged(_id, _found_by);
    }
}


void Building::setFoundByTentative(FoundBy f)
{
    _found_by_tentative = f;
}

FoundBy Building::foundByTentative()
{
    return _found_by_tentative;
}

Building::Type Building::buildingType()
{
    return _type;   
}

void Building::setBuldingType(Type t)
{
    _type = t;
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

void Building::setPromptAnswer(int wall, PromptAnswer answer)
{
    if(wall == -1)
        return;
    
    _answer_for_wall[wall] = answer;
}

PromptAnswer Building::promptAnswer(int wall)
{
    return (PromptAnswer) _answer_for_wall.value(wall, -1);
}

void Building::incrementSpaceCountForWall(int i)
{
    Q_ASSERT(i >= 0);
    if(i == -1)
        return;
    int val = _space_count_by_wall[i];
    _space_count_by_wall[i] = val + 1;
}

const QMap<int, int>& Building::spaceCountPerWall()
{
    return _space_count_by_wall;
}

void Building::wallQueried(int wall, int query_type)
{
    if(query_type == qDoor)
    {
        int count = _prompt_count_doors[wall];
        //if(count < MAX_PROMPTS_PER_WALL_DOOR)
            _prompt_count_doors[wall] = count + 1;
    }
    else if(query_type == qWindow)
    {
        int count = _prompt_count_windows[wall];
        //if(count < MAX_PROMPTS_PER_WALL_WINDOW)
            _prompt_count_windows[wall] = count + 1;
    }
    else
    {
        Q_ASSERT(false);
    }
}

int Building::queryCountForWall(int wall, int query_type)
{
    switch(query_type)
    {
        case qDoor:
            return _prompt_count_doors.value(wall, -1);
        case qWindow:
            return _prompt_count_windows.value(wall, -1);
        default:
            break;
    }
    
    return -1;
}

int Building::maxQueriesPerWall(int query_type)
{
    switch(query_type)
    {
        case qDoor:
            return MAX_PROMPTS_PER_WALL_DOOR;
        case qWindow:
            return MAX_PROMPTS_PER_WALL_WINDOW;
        default:
            break;
    }
    
    return -1;
}

bool Building::hasQueries()
{
    for(auto it = _door_prompts.constBegin(); it != _door_prompts.constEnd(); ++it)
    {
        if(it.value() == 1)
            return true;
    }
    
    for(auto it = _window_prompts.constBegin(); it != _window_prompts.constEnd(); ++it)
    {
        if(it.value() == 1)
            return true;
    }
    
    return false;
}

bool Building::wallHasDoor(int wall)
{
    Q_ASSERT(wall >= 0);
    if(wall == -1)
        return false;
    
    return _doors.value(wall, -1) == 1;
}

}

