
/* 
 * File:   Building.h
 * Author: serl
 *
 * Created on July 28, 2017, 2:19 PM
 */

#ifndef BUILDING_H
#define BUILDING_H

#include <memory>

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
    
public:
    Building();
    
    void setLocation(double x, double y);
    
    int xPos();
    int yPos();
    
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
    
    void setDoorPrompt(int wall);
    int doorPrompt();
    
    void setDoorMissing(int wall);
    int doorMissing();
    
    void setFalsePrompt(int wall);
    int falsePrompt();
    
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
    
    double _x;
    double _y;
};

}

#endif /* BUILDING_H */

