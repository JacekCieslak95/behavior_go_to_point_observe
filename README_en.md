# Behavior GoToPointObserve

Read in [Polish]

This package creates behavior used by [Aerostack] (framework developed by [Vision4UAV]).
This behavior allow the drone to move to a given point while observing (with the camera) a given point or a drone of a given ID. It does not utilize path planning.

### Instalation ###
1. Move the files of this repository into the
    `~/workspace/ros/aerostack_catkin_ws/src/`
   folder so that the catalogue tree looks as follows:
    
        ~/workspace/ros/aerostack_catkin_ws/src/
            -behavior_go_to_point_observe
    		    -CMakeLists.txt
                -package.xml
                -launch
                    -behavior_go_to_point_observe.launch
    			-src
                    -include
                        -behavior_go_to_point_observe.h
                    -source
                        -behavior_go_to_point_observe.cpp
                        -behavior_go_to_point_observe_main.cpp

2. Compile using catkin_make `~/workspace/ros/aerostack_catkin_ws/$ catkin_make`
3. Edit `simulated_quadrotor_basic.sh` file - in the startup script paste the following lines at the end:
    
	    `#----------------------------------------------` \
            `# Behavior GoToPointObserve                                   ` \
	    `#----------------------------------------------` \
            --tab --title "Behavior GoToPointObserve" --command "bash -c \"
            roslaunch behavior_go_to_point_observe behavior_go_to_point_observe.launch --wait \
    		drone_id_namespace:=drone$NUMID_DRONE \
    		drone_id:=$NUMID_DRONE \
    		my_stack_directory:=${AEROSTACK_STACK};
    	exec bash\"" \
    
4. Edit `behavior_catalog.yaml` file located in `~/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/configs/droneX` 
    Go to the `behavior_descriptors` section and paste the following lines:
#### NOTICE! It should be pasted in the `configs/droneX` folder of every drone that is going to utilize this behavior
		
          - behavior: GO_TO_POINT_OBSERVE
            timeout: 30
            incompatible_lists: [motion_behaviors]
            capabilities: [SETPOINT_BASED_FLIGHT_CONTROL, PATH_PLANNING]
            arguments:
              - argument: COORDINATES
                allowed_values: [-100,100]
                dimensions: 3
              - argument: RELATIVE_COORDINATES
                allowed_values: [-100,100]
                dimensions: 3
              - argument: SPEED
                allowed_values: [0,30]
              - argument: AVOID_DRONE_ID
                allowed_values: [0,10]
              - argument: OBSERVE_DRONE_ID
                allowed_values: [0,10]
              - argument: OBSERVED_POINT
                allowed_values: [-100,100]
                dimensions: 3

##### NOTICE! Make the indentation using the space bar, not the tab key!

### Arguments taken: ###
Behavior takes following arguments:
    
    coordinates=[x,y,z]
    
lub
    
    relative_coordinates=[x,y,z]
    
This sets the destination point towards which the drone will move.
    
    speed=x
    
This it the speed of the drone. If it is not filled, the default speed=5.

    avoid_drone_ID=x

This is the ID number of the drone that our drone will not collide with. Notice! If it is not given, the drone will not avoid collisions.

    observe_drone_id=x

This is the ID number of the drone that our drone will observe during movement.

    observed_point=[x,y,z]

This sets the point which will be obverved by drone during movement.

Example of a call::
`result = api.executeBehavior('GO_TO_POINT_ANGLE', coordinates=[6.5, 6.5, 3], speed=5, angle=30, avoid_drone_id=2)`


[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)
   [Polish]: <https://github.com/JacekCieslak95/behavior_go_to_point_observe/blob/master/README.md>
   [English]: <https://github.com/JacekCieslak95/behavior_go_to_point_observe/blob/master/README_en.md>
   [Aerostack]: <https://github.com/Vision4UAV/Aerostack>
   [Vision4UAV]: <https://github.com/Vision4UAV>
