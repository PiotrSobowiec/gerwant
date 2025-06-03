# VisionResult.msg (ROS 2)
std_msgs/Header header
int32 frame_number
float32 fps
string status
string person_name
float32 confidence
geometry_msgs/Point face_rect      # x=left, y=top, z=width
geometry_msgs/Point marker_center  # x,y center of ArUco marker
