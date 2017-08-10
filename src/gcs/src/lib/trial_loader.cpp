/* 
 * File:   TrialLoader.cpp
 * Author: n8
 * 
 * Created on July 29, 2017, 8:18 PM
 */

#include <QTextStream>
#include <QFile>

#include <gcs/util/trial_loader.h>
#include <gcs/util/building.h>
#include <gcs/util/debug.h>
#include <QtCore/qurl.h>

#define COMMENT '#'
#define DELIMIT ", "

namespace gcs
{

bool TrialLoader::load(Condition c, int trial)
{
    bool b = loadBuildings(c, trial);
    bool w = loadWaypoints(c, trial);
    
    return b && w;
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

        b->setID(list[i++].toInt()); // 0
        
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
        
        QMap<int, int> doors;
        for(int j = 0; j < 4; j++)
        {
            doors[j] = list[i++].toInt(); // 4, 5, 6, 7
        }
        b->setDoors(doors);
        
        QMap<int, int> windows;
        for(int j = 0; j < 4; j++)
        {
            windows[j] = list[i++].toInt(); // 8, 9, 10, 11
        }
        b->setWindows(windows);
        
        b->setDoorPrompt(list[i++].toInt());  // 12
        b->setDoorMissing(list[i++].toInt()); // 13
        b->setFalsePrompt(trimEOLComment(list[i++]).toInt()); // 14
        
        _buildings.insert(b->getID(), b);
    }
    
    return true;
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

    while(!ts.atEnd())
    {
        QString line = ts.readLine();
        if(line.isEmpty() || line.startsWith(COMMENT))
            continue;
        
        line = line.trimmed().simplified();
        
        QStringList list = line.split(DELIMIT);
        
        Q_ASSERT(list.length() == 5);
        if(list.length() != 5)
            continue;
        
        int i = 0;
        auto wp = std::make_shared<WaypointInfo>();
        
        wp->x = list[i++].toDouble();
        wp->y = list[i++].toDouble();
        wp->z = list[i++].toDouble();
        wp->yaw = list[i++].toInt();
        wp->building_id = trimEOLComment(list[i++]).toDouble();
        
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

QString TrialLoader::trimEOLComment(const QString& entry)
{
    int index = entry.indexOf(COMMENT);
    if(index == -1)
        return entry.trimmed();

    return entry.left(index).trimmed();
}


}