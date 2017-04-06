
/* 
 * File:   CoordinateSystemHelper.cpp
 * Author: n8
 * 
 * Created on April 2, 2017, 5:24 PM
 */

#include<QStandardItemModel>
#include<QStandardItem>

#include <gcs/qt/settings_manager.h>

#include <lcar_msgs/WorldMap.h>

namespace gcs
{

SettingsManager::SettingsManager(QObject * parent) :
QObject(parent),
mdl_cs(new QStandardItemModel(this))
{
    mdl_cs->setHorizontalHeaderLabels({"X", "Y", "Z"});
}

SettingsManager::~SettingsManager() 
{
}

void SettingsManager::getCoordinates(QVector<Point>& vector)
{
    int rows = mdl_cs->rowCount();
    vector.reserve(rows);
    for(int i = 0; i < rows; i++)
    {   
        vector.insert(i, Point(mdl_cs->item(i, COLUMN_X)->data(Qt::DisplayRole).toDouble(),
                               mdl_cs->item(i, COLUMN_Y)->data(Qt::DisplayRole).toDouble(),
                               mdl_cs->item(i, COLUMN_Z)->data(Qt::DisplayRole).toDouble()));
    }
}

void SettingsManager::onAddCoordinateSystemSelection(const QModelIndex& top_left, const QModelIndex& bottom_right)
{
    
}

void SettingsManager::onDeleteCoordinateSystemSelection(const QModelIndex& top_left, const QModelIndex& bottom_right)
{
    
}

}