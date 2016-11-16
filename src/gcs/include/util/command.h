
/* 
 * File:   Command.h
 * Author: serl
 *
 * Created on November 11, 2016, 11:36 AM
 */

#ifndef VOCE_H
#define VOCE_H

#include "command.h"
#include <voce/voce.h>

#include <QObject>
namespace gcs
{

class Command 
{
public:
    Command();
    virtual ~Command();
    
//    void initiate();
//    void update();
    void getCommand();
    
    int IdFromVehicleString(QString v_type);
    
private:
    
};

}
#endif /* VOCE_H */

