# Behavior GoToPointObserve
Paczka tworzy behavior używany przez Aerostack (oprogramowanie grupy Vision4UAV: https://github.com/Vision4UAV/Aerostack)
Zachowanie umożliwia dronowi poruszanie się do zadanego punktu prowadząc jednocześnie obserwację (z użyciem kamery) zadanego punktu, lub drona o podanym ID
### Instalacja ###
1. Pliki niniejszego repozytorium należy umieścić w folderze 
    `~/workspace/ros/aerostack_catkin_ws/src/`
    tak, aby tworzyły poniższe drzewo:
    
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

2. Przeprowadzić kompilację catkin `~/workspace/ros/aerostack_catkin_ws/$ catkin_make`
3. Edytować plik `simulated_quadrotor_basic.sh` - W skrypcie uruchamiającym należy dokleić na końcu poniższe linie:
    
	    `#----------------------------------------------` \
            `# Behavior GoToPointObserve                                   ` \
	    `#----------------------------------------------` \
            --tab --title "Behavior GoToPointObserve" --command "bash -c \"
            roslaunch behavior_go_to_point_observe behavior_go_to_point_observe.launch --wait \
    		drone_id_namespace:=drone$NUMID_DRONE \
    		drone_id:=$NUMID_DRONE \
    		my_stack_directory:=${AEROSTACK_STACK};
    	exec bash\"" \
    
4. Edytować plik `behavior_catalog.yaml`. Plik znajduje się w lokalizacji: `~/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/configs/droneX` 
    W sekcji `behavior_descriptors` należy dokleić poniższe linie:
#### UWAGA! Należy to wkleić do folderu `configs/droneX` każdego drona, którego chcemy uruchamiać z danym zachowaniem.
#### Np. Używając tego w dronach 1 i 2 poniższy fragment należy dokleić do `behavior_catalog.yaml` w folderach `configs/drone1` oraz `configs/drone2`
		
          - behavior: GO_TO_POINT_OBSERVE
            timeout: 30
            incompatible_lists: [motion_behaviors]
            capabilities: [SETPOINT_BASED_FLIGHT_CONTROL, PATH_PLANNING]
            arguments:
              - argument: POSITION
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
				
##### UWAGA! Wcięcia powinny być realizowane przez spacje, nie tabulatory!

### Przyjmowane argumenty ###
Behavior przyjmuje argumenty:
    
    coordinates=[x,y,z]
    
lub
    
    relative_coordinates=[x,y,z]
    
Jest to kąt (w stopniach) pod jakim dron będzie skierowany względem punktu. Domyślnie 0 stopni.
    
    speed=x
    
Jest to prędkość osiągana przez drona. Nie podając argumentu domyślnie speed=5

    avoid_drone_ID=x

Jest to numer drona, z którym kolizjii dron będzie unikał. Uwaga, przy niepodaniu numeru - dron nie unika kolizji.

Przykład wywołania:
`result = api.executeBehavior('GO_TO_POINT_OBSERVE', coordinates=[6.5, 6.5, 3], speed=5, avoid_drone_id=2, observe_drone_id=2)`


