
/* 
 * File:   Building.h
 * Author: serl
 *
 * Created on July 28, 2017, 2:19 PM
 */

#ifndef BUILDING_H
#define BUILDING_H

#include <memory>

#include <QMap>

namespace gcs 
{
    
class Building 
{
public:
    enum Type
    {
        tNull = 0,
        tPurple,
        tWhite
    };
    
    enum FoundBy
    {
        fNull = 0,
        fOperator,
        fVehicle
    };
    
    enum PromptAnswer
    {
        aNull,
        aYes,
        aNo
    };
    
public:
    Building();
    
    void setLocation(float x, float y);
    
    float xPos();
    float yPos();
    
    void setID(int id);
    int getID();
    
    void spaceDown();
    void spaceUp();
    int spaceCount();
    
    FoundBy foundBy();
    void setFoundBy(FoundBy f);
    
    Type buildingType();
    void setBuldingType(Type t);
    
    void setDoorLocation(int wall);
    int doorLocation();
    
    void setDoors(const QMap<int, int>& doors);
    const QMap<int, int>& doors();
    
    void setWindows(const QMap<int, int>& windows);
    const QMap<int, int>& windows();
    
    void setDoorPrompt(int wall);
    int doorPrompt();
    
    void setDoorMissing(int wall);
    int doorMissing();
    
    void setFalsePrompt(int wall);
    int falsePrompt();
    
    void setPromptAnswer(PromptAnswer answer)
    {
        _answer = answer;
    }
    
    PromptAnswer promptAnswer()
    {
        return _answer;
    }
    
    void incrementSpaceCountForWall(int i)
    {
        int val = _space_count_by_wall[i];
        _space_count_by_wall[i] = val + 1;
    }
    
    const QMap<int, int> spaceCountPerWall()
    {
        return _space_count_by_wall;
    }
    
private:
    int _id;
    
    bool _space_down = false;
    int _space_count = 0;
    
    int _door_location = -1;
    int _door_prompt = -1;
    int _door_missing = -1;
    int _false_prompt = -1;
    
    Type _type = tNull;
    FoundBy _found_by = fNull;
    PromptAnswer _answer = aNull;
    
    float _x;
    float _y;
    
    QMap<int, int> _doors;
    QMap<int, int> _windows;
    
    QMap<int, int> _space_count_by_wall;
};

}

#endif /* BUILDING_H */

