
cmake_minimum_required(VERSION 2.8)
project(EMC_project)

find_package(OpenCV REQUIRED)
find_package(assimp REQUIRED)

add_compile_options(-std=c++11)
SET(TARGET_H

#        src/definitions.h
        src/task_manager/tasks.h
        src/config.h
        src/objects.h
        src/vector2.h
        src/instances.h

	src/world_model/world_model.h
	src/drive_control/drive_control.h
	src/perception/perception.h
	src/visualisation/visualisation.h
	src/task_manager/task_manager.h
        src/path_planner/path_planner.h
        src/graph/graph.h
        src/pid_control/pid_control.h
)

SET(TARGET_SRC
    	src/main.cpp
	src/world_model/world_model.cpp
	src/drive_control/drive_control.cpp
	src/perception/perception.cpp
	src/visualisation/visualisation.cpp
	src/task_manager/task_manager.cpp
        src/path_planner/path_planner.cpp
        src/graph/graph.cpp
        src/pid_control/pid_control.cpp
)


add_executable(main ${TARGET_SRC} ${TARGET_H}) # ${GLOBFILES})
target_link_libraries(main ${OpenCV_LIBS})
target_link_libraries(main emc-framework)
target_link_libraries(main assimp)
#target_link_libraries(main geolib)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
