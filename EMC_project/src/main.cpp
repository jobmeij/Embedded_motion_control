#include <emc/io.h>
#include <emc/rate.h>
#include <emc/odom.h>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "./world_model/world_model.h"
#include "./perception/perception.h"
#include "./task_manager/task_manager.h"
#include "./drive_control/drive_control.h"
#include "./visualisation/visualisation.h"
#include "./path_planner/path_planner.h"
#include "./pid_control/pid_control.h"
#include "./graph/graph.h"
#include "config.h"
//#include "definitions.h"
#include "instances.h"

int main(){

    emc::Rate r(EXECUTION_RATE);
    emc::IO io;
    int c=0;
    Graph graph(40.0,0.0,500,400); //scale offset width height
    Visualisation visualisation(800,800); //width height

    WorldModel worldmodel;
    TaskManager taskmanager(&worldmodel, &io);
    PathPlanner pathplanner(&worldmodel);
    DriveControl driveControl(&worldmodel, &io);
    Perception perception(&worldmodel, &io, &visualisation, &graph);

    taskmanager.init();

    Map& localmap = worldmodel.getlocalmap();
    Map& currentmap = worldmodel.getcurrentmap();
    Map& globalmap = worldmodel.getglobalmap();
    Position& dest = worldmodel.getdesiredposition();
    Position& curpos = worldmodel.getcurrentposition();
    Position zero(2,0,0);
    Position global(3,3.5,0);

    Object robot = robotobj();
    Object origin = originobj();
    Object arrow = arrowobj();

    while(io.ok()) {
        visualisation.setorigin(curpos); // CHOOSE curpos / zero

        //std::cout<<"crashed in perceptor"<<std::endl;
        perception.evaluate();
        //std::cout<<"crashed in pathplanner"<<std::endl;
        pathplanner.pathPlanner();
        //std::cout<<"crashed in drive controller"<<std::endl;
        driveControl.driveToPosition();
        //std::cout<<"crashed in task manager"<<std::endl;
        taskmanager.execute();

        visualisation.addObject(origin);
        Object robotcopy = robot;
        robotcopy.transform(curpos.x, curpos.y, curpos.a);
        visualisation.addObject(robotcopy);
        Object destination = arrow;
        destination.transform(dest.x, dest.y, dest.a);
        visualisation.addObject(destination);
//        visualisation.addMap(currentmap);
        visualisation.addMap(localmap);
        visualisation.addMap(globalmap);
//        visualisation.addSafeDis(worldmodel.getToClose(),curpos);
        visualisation.visualize();
        visualisation.emptycanvas();

        r.sleep();
    }

    return 0;
}
