#  _    _                _           
# | |  | |              | |          
# | |__| | ___  __ _  __| | ___ _ __ 
# |  __  |/ _ \/ _` |/ _` |/ _ | '__|
# | |  | |  __| (_| | (_| |  __| |   
# |_|  |_|\___|\__,_|\__,_|\___|_|   
#
# Subtype:  std_msgs/Header (https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html)                      

std_msgs/Header header

## * header.seq      --> uint32 seq :  consecutively increasing ID 
## * header.stamp    --> time stamp :  timestamp
## * header.frame_id --> string frame_id : frame id



#  _____            _                 _ 
# |  __ \          | |               | |
# | |__) __ _ _   _| | ___   __ _  __| |
# |  ___/ _` | | | | |/ _ \ / _` |/ _` |
# | |  | (_| | |_| | | (_) | (_| | (_| |
# |_|   \__,_|\__, |_|\___/ \__,_|\__,_|
#              __/ |                    
#             |___/                     

int32 width       # original image width
int32 height      # original image height
string encoding	  # encoding used
uint64 pts        # packet pts
uint8  flags      # packet flags
bool is_bigendian # true if machine stores in big endian format
uint8[] data      # ffmpeg compressed payload