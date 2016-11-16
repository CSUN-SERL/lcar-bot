
#include "util/strings.h"

namespace gcs
{
    QString company_; 
    QString application_;
    
    FlightModes flight_modes_;
    
    QString img::image_root_dir_;
    
    void InitStrings()
    {
        company_ = "SERL";
        application_ = "ISLURP";
        
        flight_modes_.stabilized = "STABILIZED";
        flight_modes_.loiter = "AUTO.LOITER";
        flight_modes_.land = "AUTO.LAND";
        flight_modes_.altitude_control = "ALTCTL";
        flight_modes_.position_control = "POSCTL";
        flight_modes_.mode_auto = "AUTO";
        flight_modes_.offboard = "OFFBOARD";
    }
}