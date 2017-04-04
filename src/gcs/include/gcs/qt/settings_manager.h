
/* 
 * File:   CoordinateSystemHelper.h
 * Author: n8
 *
 * Created on April 2, 2017, 5:24 PM
 */

#ifndef SETTINGSMANAGER_H
#define SETTINGSMANAGER_H

#include <QObject>
#include <QVector>

#include <gcs/util/point.h>

#define COLUMN_X 0
#define COLUMN_Y 1
#define COLUMN_Z 2

class QStandardItemModel;

namespace gcs
{

class VehicleManager;
class SettingsWidget;

class SettingsManager : public QObject {
    Q_OBJECT
    
    friend class VehicleManager;
    friend class SettingsWidget;
    
public:

    SettingsManager(QObject * parent = nullptr);
    virtual ~SettingsManager();
    
    void getCoordinates(QVector<Point>& vector);
    
signals: 
    void coordinatesReady();

public slots:
    void onAddCoordinateSystemSelection(const QModelIndex& top_left, const QModelIndex& bottom_right);
    void onDeleteCoordinateSystemSelection(const QModelIndex& top_left, const QModelIndex& bottom_right);
    
private:
    QStandardItemModel * mdl_cs; // coordinate system model
    
    Q_DISABLE_COPY(SettingsManager);  
};

}
#endif /* SETTINGSMANAGER_H */

