std_msgs/Header header
# Deprecated compatibility field; observations is the tracker new interface.
geometry_msgs/Pose[] armor_poses

uint8 id
# Tracker observations are appended after the legacy fields.
aim_msgs/ArmorPoseObservation[] observations
