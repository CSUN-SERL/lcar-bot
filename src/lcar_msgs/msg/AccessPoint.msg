# access point message type for lcar-bot
int8 BLOB=0
int8 DOOR=1	
int8 WINDOW=2
int8 ap_type
Query query
bool is_query
sensor_msgs/NavSatFix location
float64 compass_heading
float64 altitude