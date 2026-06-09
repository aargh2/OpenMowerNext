string id # id of the docking station
string name # name of the docking station
geometry_msgs/PoseStamped approach_pose # Start of the docking vector. Robot navigates here before docking.
geometry_msgs/PoseStamped pose # End of the docking vector, located at the dock edge. Orientation follows approach_pose -> pose.
