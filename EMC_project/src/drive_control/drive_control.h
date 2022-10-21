#include <emc/io.h>
#include "../config.h"
#include "../world_model/world_model.h"
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <list>
#include "../visualisation/visualisation.h"
#include "../graph/graph.h"
#include "../pid_control/pid_control.h"

#ifndef driveControl_H
#define driveControl_H

class DriveControl {
private:
    WorldModel* worldmodel;
    emc::IO* robotIO;
    emc::OdometryData odometrydata;
    PID_control pidx;
    PID_control pidy;
    PID_control pida;
    int previousDriveMode;
    int Drivemode;
    bool infeasable_direction;

public:
    DriveControl(WorldModel* worldmodel_, emc::IO* robotIO_){
        worldmodel = worldmodel_;
        robotIO = robotIO_;


    }

    void driveToPosition();

    void actuateDriveSystem(Position DesiredVelocity_);
    void saturate(float &value_, float saturate_);
    float sign(float in_);

    void status();
    int closest(std::vector<int> const& vec, int value);
    void ColisionPrevention(double& Direction_to_goal, Position& distancetogoRF, close_prx& safeDir, bool& CloseToObject, bool& infeasable_direction);


};

#endif //driveControl_H
