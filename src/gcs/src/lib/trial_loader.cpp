/* 
 * File:   TrialLoader.cpp
 * Author: n8
 * 
 * Created on July 29, 2017, 8:18 PM
 */

#include <QTextStream>
#include <QFile>

#include <gcs/util/trial_loader.h>
#include <gcs/qt/building.h>
#include <gcs/util/debug.h>
#include <QtCore/qurl.h>

#define COMMENT '#'
#define DELIMIT ", "

namespace gcs
{

const QMap<int, std::shared_ptr<Building> >& TrialLoader::getBuildings() const
{
    return _buildings;
}

const QList< std::shared_ptr<WaypointInfo> >& TrialLoader::getWaypointInfoList() const
{
    return _waypoints;
}

void TrialLoader::reset()
{
    _buildings.clear();
    _waypoints.clear();
}
    
bool TrialLoader::load(Condition c, int trial)
{
    bool b = loadBuildings(c, trial);
    bool w = loadWaypoints(c, trial);
    
    return b && w;
}

bool TrialLoader::loadWaypoints(Condition c, int trial)
{
    _waypoints.clear();
    
    QString file_name = c == Predictable ?
        ":/waypoints/wp_predictable.txt" :
        ":/waypoints/wp_unpredictable.txt";
    
    qCDebug(lcar_bot) << file_name;
    
    QFile file(file_name);
    
    bool open = file.open(QIODevice::ReadOnly | QIODevice::Text);
    Q_ASSERT(open);
    if(!open)
        return false;
        
    QTextStream ts(&file);

    // get the number of fields each line should have at the top of the file
    QString s = ts.readLine();
    s = trimEOLComment(s);
    
    bool ok;
    int num_fields = s.toInt(&ok);
    Q_ASSERT(ok);
    
    int line_num = 0;
    
    while(!ts.atEnd())
    {
        QString line = ts.readLine();
        if(line.isEmpty() || line.startsWith(COMMENT))
            continue;
        
        line = line.trimmed().simplified();
        
        QStringList list = line.split(DELIMIT);
        
        if(list.length() != num_fields)
        {
            qCDebug(lcar_bot) << "line" << line_num << "is malformed. Expected" << num_fields << "fields, got" << list.length();
            qCDebug(lcar_bot) << line;
            Q_ASSERT(false);
            continue;   
        }
        
        int i = 0;
        auto wp = std::make_shared<WaypointInfo>();
        
        wp->x = list[i++].toDouble();
        wp->y = list[i++].toDouble();
        wp->z = list[i++].toDouble();
        wp->yaw = list[i++].toInt();
        wp->building_id = trimEOLComment(list[i++]).toInt();
        
        _waypoints.append(wp);
    }
    
     qCDebug(lcar_bot) << "Waypoint list size:" << _waypoints.length();
//     for(const auto& wp : _waypoints)
//     {
//        qCDebug(lcar_bot) << "x:" << wp->x;
//        qCDebug(lcar_bot) << "y:" << wp->y;
//        qCDebug(lcar_bot) << "z:" << wp->z;
//        qCDebug(lcar_bot) << "yaw:" << wp->yaw;   
//     }
    
    return true;
}

bool TrialLoader::loadBuildings(Condition c, int trial)
{
    _buildings.clear();
    
    QString trial_s = QString::number(trial);
    
    QString file_name = c == Predictable ?
        QString(":/buildings/b_predictable%1.txt").arg(trial_s) :
        QString(":/buildings/b_unpredictable%1.txt").arg(trial_s);
    
    qCDebug(lcar_bot) << file_name;
    
    QFile file(file_name);
    
    bool open = file.open(QIODevice::ReadOnly | QIODevice::Text);
    Q_ASSERT(open);
    if(!open)
        return false;
        
    QTextStream ts(&file);
    
    // get the number of fields each line should have at the top of the file
    QString s = ts.readLine();
    s = trimEOLComment(s);
    
    bool ok;
    int num_fields = s.toInt(&ok);
    Q_ASSERT(ok);
    
    int line_num = 0;
    
    while(!ts.atEnd())
    {
        QString line = ts.readLine();
        line_num++;
        if(line.isEmpty() || line.startsWith(COMMENT))
            continue;
        
        line = line.trimmed().simplified();
        
        QStringList list = line.split(DELIMIT);
        
        if(list.length() != num_fields)
        {
            qCDebug(lcar_bot) << "line" << line_num << "is malformed. Expected" << num_fields << "fields, got" << list.length();
            qCDebug(lcar_bot) << line;
            Q_ASSERT(false);
            continue;
        }
        
        int i = 0;
        
        auto b = std::make_shared<Building>();

        b->setID(list[i++].toInt());    // 0
        
        Building::Type t;
        if(list[i].contains('p'))
            t = Building::tPurple;
        else if(list[i].contains('w'))
            t = Building::tWhite;
        else
        {
            Q_ASSERT(false);
            continue;
        }
        
        b->setBuldingType(t);
        i++;                             // 1
        
        double x = list[i++].toDouble(); // 2
        double y = list[i++].toDouble(); // 3
        b->setLocation(x, y);
        
        setWallContent(b, i, list);      // 4,5,6,7
        setPromptInfo(b, i, list);       // 8,9,10,11
        
        _buildings.insert(b->getID(), b);
    }
    
    return true;
}

void TrialLoader::setWallContent(const std::shared_ptr<Building> b, int& i, const QStringList& list)
{
    qCDebug(lcar_bot) << "Wall Info";
    QMap<int, int> doors;
    QMap<int, int> windows;
    for(int j = 0; j < 4; j++)
    {
        QString s = list[i++];
        qCDebug(lcar_bot) << s;
        if(s.contains('d'))
            doors[j] = 1;
        
        if(s.contains('w'))
            windows[j] = 1;
    }
    b->setDoors(doors);
    b->setWindows(windows);
}

void TrialLoader::setPromptInfo(const std::shared_ptr<Building> b, int& i, const QStringList& list)
{
    qCDebug(lcar_bot) << "Prompt Info";
    QMap<int, int> doors;
    QMap<int, int> windows;
    int j= 0;
    for(; j < 3; j++)
    {
        QString s = list[i++];
        qCDebug(lcar_bot) << s;
        doors[j] = s.contains('d') ? 1 : -1;    
        windows[j] = s.contains('w') ? 1 : -1;
    }
    
    QString s = trimEOLComment(list[i++]);
    qCDebug(lcar_bot) << s;
    doors[j] = s.contains('d') ? 1 : -1;
    windows[j] = s.contains('w') ? 1 : -1;
    
    b->setDoorPrompts(doors);
    b->setWindowPrompts(windows);
}

QString TrialLoader::trimEOLComment(const QString& entry)
{
    int index = entry.indexOf(COMMENT);
    if(index == -1)
        return entry.trimmed();

    return entry.left(index).trimmed();
}


}