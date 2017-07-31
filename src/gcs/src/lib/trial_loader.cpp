
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

namespace gcs
{

void TrialLoader::load(Condition c, int trial)
{
    loadBuildings(c, trial);   
    loadWaypoints(c, trial);
}

void TrialLoader::loadBuildings(Condition c, int trial)
{
    _buildings.clear();
    
    QString trial_s = QString::number(trial);
    
    QString file_name = c == Predictable ?
        QString(":/buildings/predictable%1.txt").arg(trial_s) :
        QString(":/buildings/unpredictable%1.txt").arg(trial_s);
    
    QFile file(file_name);
    
    bool open = file.open(QIODevice::ReadOnly);
    Q_ASSERT(open);
    if(!open)
        return;
        
    QTextStream ts(&file);

    while(!ts.atEnd())
    {
        QString line = ts.readLine();
        if(line.isEmpty() || line.startsWith('#'))
            continue;
        
        QStringList list = line.split(", ");
        
        Q_ASSERT(list.length() == 8);
        if(list.length() != 8)
            continue;
        
        int i = 0;
        
        auto b = std::make_shared<Building>();

        b->setID(list[i++].toInt());
        
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
        i++;
        
        double x = list[i++].toDouble();
        double y = list[i++].toDouble();
        
        b->setLocation(x, y);
        b->setDoorLocation(list[i++].toInt());
        b->setDoorPrompt(list[i++].toInt());
        b->setDoorMissing(list[i++].toInt());
        b->setFalsePrompt(list[i++].toInt());
        
        _buildings.append(b);
    }
}
    
void TrialLoader::loadWaypoints(Condition c, int trial)
{
    _waypoints.clear();
    
    QString file_name = c == Predictable ?
        ":/waypoints/predictable.txt" :
        ":/waypoints/unpredictable.txt";
    
    QFile file(file_name);
    
    bool open = file.open(QIODevice::ReadOnly);
    Q_ASSERT(open);
    if(!open)
        return;
        
    QTextStream ts(&file);

    while(!ts.atEnd())
    {
        QString line = ts.readLine();
        if(line.isEmpty() || line.startsWith('#'))
            continue;
        
        QStringList list = line.split(", ");
        
        Q_ASSERT(list.length() == 4);
        if(list.length() != 4)
            continue;
        
        auto wp = std::make_shared<WaypointInfo>();
        wp->building_id = list[0].toDouble();
        wp->x = list[1].toDouble();
        wp->y = list[2].toDouble();
        wp->z = list[3].toDouble();
        
        _waypoints.append(wp);
    }
        
}

const QList< std::shared_ptr<Building> >& TrialLoader::getBuildings() const
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


}