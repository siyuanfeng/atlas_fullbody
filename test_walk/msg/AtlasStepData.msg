#This message provides the data for the foot step data structure.

int32 step_index                #Step index
int32 foot_index                #Foot id
float32 duration                #Step duration
geometry_msgs/Vector3 position  #Foot position, in Atlas world frame
float32 yaw                     #Foot orientation yaw component, in Atlas world frame
geometry_msgs/Vector3 normal    #Foot ground normal, in Atlas world frame
float32 swing_height            #Step apex swing height as measured from the midpoint between the feet

#Foot id
int32 FOOT_LEFT=0
int32 FOOT_RIGHT=1
int32 NUM_FEET=2
