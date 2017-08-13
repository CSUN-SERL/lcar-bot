
/* 
 * File:   Building.h
 * Author: n8
 *
 * Created on July 28, 2017, 2:19 PM
 */

#ifndef BUILDING_H
#define BUILDING_H

#include <memory>

#include <QObject>
#include <QMap>

#define F2M 0.3048  // feet to meters
#define B_SIZE ((float) (3.0 * F2M)) // building size

#define CAMERA_FOV 53.2

namespace gcs
{

typedef int BuildingID;
typedef int Wall;
typedef int FoundBy;
typedef int PromptAnswer;
typedef int QueryType;
    
class Building : public QObject
{   
    Q_OBJECT
public:
    typedef int Type;
    enum TypeEnum
    {
        tNull = 0,
        tPurple,
        tWhite
    };
    
    enum FoundByEnum
    {
        fNull = 0,
        fOperator,
        fVehicle
    };
    
    enum PromptAnswerEnum
    {
        aNull,
        aYes,
        aNo
    };
    
    enum QueryTypeEnum
    {
        qNull,
        qDoor,
        qWindow
    };
    
    static int targetYawToWall(int yaw);
    static int actualYawToWall(float yaw);
    
public:
    Building();
    
    void setLocation(float x, float y);
    
    float xPos();
    float yPos();
    
    void setID(BuildingID id);
    int getID();
    
    void spaceDown(int wall);
    void spaceUp();
    int spaceCount();
    
    FoundBy foundBy(Wall wall);
    void setFoundBy(Wall wall, FoundBy f);
    const QMap<Wall, FoundBy>& foundByAll();
    
    void setFoundByTentative(Wall wall, FoundBy f);
    FoundBy foundByTentative(Wall wall);
    const QMap<Wall, FoundBy> foundByTentativeAll();
    
    Type buildingType();
    void setBuldingType(Type t);
    
    void setDoors(const QMap<Wall, int>& doors);
    const QMap<int, int>& doors();
    
    void setWindows(const QMap<Wall, int>& windows);
    const QMap<int, int>& windows();
    
    void setDoorPrompts(const QMap<Wall, int>& door_prompts);
    const QMap<int, int>& doorPrompts();
    
    void setWindowPrompts(const QMap<Wall, int>& window_prompts);
    const QMap<int, int>& windowPrompts();
    
    void setPromptAnswer(Wall wall, PromptAnswer answer);
    PromptAnswer promptAnswer(Wall wall);
    
    void incrementSpaceCountForWall(Wall i);
    
    const QMap<Wall, int>& spaceCountPerWall();
    
    void wallQueried(Wall wall, QueryType query_type);
    int queryCountForWall(Wall wall, QueryType query_type);
    
    int maxQueriesPerWall(QueryType query_type);
    
    bool hasQueries();
    bool wallHasDoor(int wall);
    
signals:
    void foundByChanged(BuildingID building_id, FoundBy found_by);
    
private:
    BuildingID _id;
    
    bool _space_down = false;
    int _space_count = 0;
    
    Type _type = tNull;
    FoundBy _found_by = fNull;
    FoundBy _found_by_tentative = fNull;


    QMap<Wall, FoundBy> _found_by_for_wall;
    QMap<Wall, FoundBy> _found_by_for_wall_tentative;
    QMap<Wall, int> _answer_for_wall;
    
    float _x;
    float _y;
    
    QMap<Wall, int> _doors;
    QMap<Wall, int> _windows;
    
    QMap<Wall, int> _door_prompts;
    QMap<Wall, int> _window_prompts;
    
    QMap<Wall, int> _prompt_count_doors;
    QMap<Wall, int> _prompt_count_windows;
    
    static int MAX_PROMPTS_PER_WALL_DOOR;
    static int MAX_PROMPTS_PER_WALL_WINDOW;
    
    QMap<Wall, int> _space_count_by_wall;
};

}

#endif /* BUILDING_H */

