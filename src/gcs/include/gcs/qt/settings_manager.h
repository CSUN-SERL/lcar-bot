
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

#define COLUMN_X 0
#define COLUMN_Y 1
#define COLUMN_Z 2

class QStandardItemModel;

namespace gcs
{

class VehicleManager;
class SettingsWidget;

struct Point 
{
public:
    Point()
    {}

    Point(int x, int y, int z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Point(const Point& other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    inline Point& operator=(const Point& other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    int x;
    int y;
    int z;
};

class SettingsManager : public QObject {
    Q_OBJECT
    
    friend class VehicleManager;
    friend class SettingsWidget;
    
public:

    SettingsManager(QObject * parent = nullptr);
    virtual ~SettingsManager();
    
    void getData(QVector<Point>& vector);
    
public slots:
    void onAddCoordinateSystemSelection(const QModelIndex& top_left, const QModelIndex& bottom_right);
    void onDeleteCoordinateSystemSelection(const QModelIndex& top_left, const QModelIndex& bottom_right);
    
private:
    QStandardItemModel * mdl_cs; // coordinate system model
    
    Q_DISABLE_COPY(SettingsManager);  
};

}
#endif /* SETTINGSMANAGER_H */

