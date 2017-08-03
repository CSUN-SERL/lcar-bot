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
        if(line.isEmpty() || line.startsWith('#'))
            continue;
        
        QStringList list = line.split(", ");
        
        Q_ASSERT(list.length() == 5);
        if(list.length() != 5)
            continue;
        
        int i = 0;
        auto wp = std::make_shared<WaypointInfo>();
        
        wp->x = list[i++].toDouble();
        wp->y = list[i++].toDouble();
        wp->z = list[i++].toDouble();
        wp->yaw = list[i++].toInt();
        wp->building_id = list[i++].toDouble();
        
        _waypoints.append(wp);
    }
        
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


}