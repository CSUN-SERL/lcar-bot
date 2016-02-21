#include <rqt_gcs/simple_control.h>
#include <rqt_gcs/simple_control.cpp>

//define and manage access point
SimpleControl::AccessPoint()
{
	SimpleControl quad1{1};
	quad1.getLocation();
}
