#include "drive_control.h"
#include "../objects.h"


// TODO: add arrows for the driving direction
// copy from main

void DriveControl::driveToPosition() {
    // Prepare variables needed for the function call
    Position distancetogoWF; //distance to go in world frame
    Position distancetogoRF; //distance to go in robot frame (forward/rotate)
    Position desiredspeed; //?
    double Direction_to_goal;
    double Distance_to_goal;
    double Distance_to_goal_WF;
//    int Drivemode;
    float signx;
    double signa;
    close_prx safeDir;
//    double indexSafeDir;

    bool CloseToObject;
    float speedfactor = 1;

    Block::Modes DcBlockMode = worldmodel->getmode(Block::DriveController);

    if (DcBlockMode == Block::MODE_EXECUTE){
        // Execute driving towards a goal.
        //std::cout << "drivecontroller execute" << std::endl;


        // Get current and desired position from world model
        Position currentpostemp = worldmodel->getcurrentposition();
        Position desiredpostemp = worldmodel->getdesiredposition();
        //std::cout<<"From Drive Controller : "<<"desired position : "<< desiredpostemp.x <<" "<< desiredpostemp.y<<" "<< desiredpostemp.a<<std::endl;
        //std::cout<<"From Drive Controller : "<<"current position : "<< currentpostemp.x <<" "<< currentpostemp.y<<" "<< currentpostemp.a<<std::endl;


        // Added yet another transformation
        Position currentpos;
        currentpos.x = -currentpostemp.y;
        currentpos.y = currentpostemp.x;
        currentpos.a = currentpostemp.a;
        Position desiredpos;
        desiredpos.x = -desiredpostemp.y;
        desiredpos.y = desiredpostemp.x;
        desiredpos.a = desiredpostemp.a;

        // Calculate distance to go in world frame
        distancetogoWF.x = desiredpos.x - currentpos.x;
        distancetogoWF.y = desiredpos.y - currentpos.y;
        distancetogoWF.a = desiredpos.a - currentpos.a;

        // normalize angle in worldframe between -pi and pi
        signa = sign(distancetogoWF.a);
        distancetogoWF.a += signa*M_PI;
        distancetogoWF.a = fmod(distancetogoWF.a, 2*M_PI);
        distancetogoWF.a -= signa*M_PI;

        // Calculate distance to go in robotframe
        distancetogoRF.x = distancetogoWF.x*cos(currentpos.a) + distancetogoWF.y*sin(currentpos.a);
        distancetogoRF.y = distancetogoWF.y*cos(currentpos.a) - distancetogoWF.x*sin(currentpos.a);

        // Calculate distance to go in worldframe
        Distance_to_goal_WF = sqrt(distancetogoWF.y*distancetogoWF.y + distancetogoWF.x*distancetogoWF.x);


        // Calculate straight line distance to goal
        Distance_to_goal = sqrt(distancetogoRF.y*distancetogoRF.y + distancetogoRF.x*distancetogoRF.x);
        //cout << "Distance to goal:" << Distance_to_goal << endl;
        // Calculate angle to desired position in robotframe
        if (distancetogoWF.y == 0 && Distance_to_goal >= DriveTransDeadzone){
            // no y distance, unable to calculate angle with arctan, fixed value by def.
            signx = sign(distancetogoWF.x);
            Direction_to_goal = signx*(M_PI/2);
        }
        else if (distancetogoWF.y == 0 && distancetogoWF.x == 0){
            // robot is at desired location, not needed to calculate angle in robotframe.
            Direction_to_goal = 0;
        }
        else{
            // use arctan funtion to calculate angle towards goal.
            Direction_to_goal = atan2(-distancetogoRF.x, distancetogoRF.y);
        };

       ////////////////////////////////////////Bump into walls detection./////////////////////////////////////////////////////

        if (worldmodel->getToClose().valid){
            safeDir = worldmodel->getToClose();
        }

        //float Old_dir = Direction_to_goal;

    //    std::cout << "Ideal direction angle " << Direction_to_goal*180/M_PI << std::endl;


        //infeasable_direction = ColisionPrevention_old(Direction_to_goal, distancetogoRF, safeDir, CloseToObject);




        //float angle_diff = Direction_to_goal - Old_dir;
        //Direction_to_goal -= Old_dir;
        //distancetogoRF.x = distancetogoRF.x*cos(angle_diff) - distancetogoRF.y*sin(angle_diff);
        //distancetogoRF.y = distancetogoRF.y*cos(angle_diff) + distancetogoRF.x*sin(angle_diff);
    //    std::cout << "new best direction angle " << Direction_to_goal*180/M_PI << std::endl;
        Distance_to_goal = sqrt(distancetogoRF.y*distancetogoRF.y + distancetogoRF.x*distancetogoRF.x);

       /////////////////////////////////////////////////////////////////////////////////////////////

        std::cout << "DC drivemode previous: " << previousDriveMode << std::endl;
        std::cout << " 1: " << Distance_to_goal_WF << std::endl;
        // determine in which state the drivecontroller must operate
        if (infeasable_direction == true){
            Drivemode = -1;
        }
        //else if (CloseToObject == true){
            // robot is close to wall, both drive and rotate to prevent chatter
         //   Drivemode = 2;

        //}
        //else if ((Distance_to_goal_WF >= DriveTransDeadzone) && (abs(Direction_to_goal) >= DriveRotDeadzone)&& (CloseToObject==false)){
        if (((previousDriveMode == 4) ||(previousDriveMode == 1))){
        // First rotate front of robot towards goal before start driving

            if ((abs(Direction_to_goal) >= DriveRotDeadzone)&&(Distance_to_goal_WF >= DriveTransDeadzone)){
                // robot is not facing the goal, first rotate
                Drivemode = 1;
                std::cout << "1" << std::endl;
            }
            else{

                // Robot is facing the goal, start driving
                std::cout << "2" << std::endl;
                Drivemode = 2;
            }
        }
        if((previousDriveMode == 2)|| (Drivemode == 2)){
            if (Distance_to_goal_WF >= DriveTransDeadzone){
                ColisionPrevention(Direction_to_goal, distancetogoRF, safeDir, CloseToObject, infeasable_direction);
                if (CloseToObject == true){
                    speedfactor = 0.5;
                }
                if (infeasable_direction == true){
                    Drivemode = -1;
                }
                else{
                    Drivemode = 2;
                }

            }
            else{
                std::cout << "2" << std::endl;
                Drivemode = 3;
            }


        }

        if((previousDriveMode == 3)|| (Drivemode == 3)){
            if(abs(distancetogoWF.a) >= DriveRotDeadzone){
                Drivemode = 3;
            }
            else{
                std::cout << "4" << std::endl;
                Drivemode = 4;
            }
        }
        if((Drivemode == 4)){
            Drivemode = 4;
        }
        std::cout << "DC drivemode after: " << Drivemode << std::endl;

        /*if (Distance_to_goal_WF >= DriveTransDeadzone) {
            // Robot is in correct orientation towards goal, start driving and make small corrections in the angle
            Drivemode = 2;





        }
        else if (abs(distancetogoWF.a) >= DriveRotDeadzone){
            // Robot is in correct location, start rotating to the desired orientation
            Drivemode = 3;
        }
        else{
            // Robot is on the correct location and orientation, driving done
            Drivemode = 4;
        }
        */
    //        std::cout << "Drive mode: " << Drivemode << "  Distance to goal: "<< Distance_to_goal << "  Direction to goal: " << Direction_to_goal << " Direction to go in WF: " << distancetogoWF.a << std::endl;

        // perform the needed calculations for the current operation mode
        switch (Drivemode){
        if (DC_DEBUG){std::cout<<"DRIVE MODE : "<<Drivemode<<std::endl;};
                 case 1: // rotate robot to correct drive orientation
                        pidx.disable_latch(); // Set output of x PID for one call to zero
                        pidy.disable_latch(); // Set output of y PID for one call to zero

                        pidx.integrator_reset();
                        pidy.integrator_reset();

                        distancetogoRF.a = Direction_to_goal;
                        break;


                 case 2: // Drive and make small angle corrections
                        distancetogoRF.a = Direction_to_goal;
                        if (previousDriveMode == 1){
                            pida.integrator_reset();
                        }
                        break;

                 case 3: // Rotate to desired final orientation
                        // Desired angle is equal to desired angle in world frame
                        distancetogoRF.a = distancetogoWF.a;

                        // Disable x and y PID controllers this iteration
                        pidx.disable_latch();
                        pidy.disable_latch();
                        // Reset x and y PID integrator values to prevent weird behaviour
                        pidx.integrator_reset();
                        pidy.integrator_reset();
                        break;

                  case 4: // Robot is at desired position/orientation. stop all outputs
                        // disable all PID controllers this iteration
                        pidx.disable_latch();
                        pidy.disable_latch();
                        pida.disable_latch();

                        // reset all PID integrator values to prevent weird behaviour
                        pidx.integrator_reset();
                        pidy.integrator_reset();
                        pida.integrator_reset();

                        // Set values to all PID controllers to zero
                        distancetogoRF.x = 0;
                        distancetogoRF.y = 0;
                        distancetogoRF.a = 0;
                        break;
                  case -1:
                        // disable all PID controllers this iteration
                        pidx.disable_latch();
                        pidy.disable_latch();
                        pida.disable_latch();

                        // reset all PID integrator values to prevent weird behaviour
                        pidx.integrator_reset();
                        pidy.integrator_reset();
                        pida.integrator_reset();

                        // Set values to all PID controllers to zero
                        distancetogoRF.x = 0;
                        distancetogoRF.y = 0;
                        distancetogoRF.a = 0;
         }

        // Determine the control values for all three axis
        desiredspeed.x = pidx.evaluate(distancetogoRF.x);
        desiredspeed.y = pidy.evaluate(distancetogoRF.y);
        desiredspeed.a = pida.evaluate(distancetogoRF.a);

        // Saturate output according max allowable speeds
        saturate(desiredspeed.x, speedfactor*MaxTransVelocity);
        saturate(desiredspeed.y, speedfactor*MaxTransVelocity);
        saturate(desiredspeed.a, speedfactor*MaxRotVelocity);

        // Send desired values to motors
        actuateDriveSystem(desiredspeed);

        // store last drive mode
        previousDriveMode = Drivemode;

        // Update block status in world model
        status();

    }
    else if(DcBlockMode == Block::MODE_IDLE){
        // Drive controller is disabled from Task planner, do nothing
        desiredspeed.x = 0;
        desiredspeed.y = 0;
        desiredspeed.a = 0;


        // Send desired values to motors
        actuateDriveSystem(desiredspeed);

        Drivemode = 4;
        previousDriveMode = Drivemode;

        infeasable_direction = false;
        // Update block status in world model
        status();

    }
    else if(DcBlockMode == Block::MODE_INIT){
        pidx.setparams(1,0,0);
        pidy.setparams(1,0,0);
        pida.setparams(1,0,0);

        Drivemode = 4;
        previousDriveMode = Drivemode;
        infeasable_direction = false;
        // Update block status in world model
        status();
    }
    else{
        // unknown blockmode
        std::cout << "DriveController error: Unknown blockmode" << std::endl;
        Drivemode = -1;
        previousDriveMode = Drivemode;
        // Update block status in world model
        status();
    }
}


// Convert own xya frame to framework of given software
void DriveControl::actuateDriveSystem(Position DesiredVelocity_){
    //cout << "Velocity:" <<DesiredVelocity_.y << "  "<< -DesiredVelocity_.x << "  " << DesiredVelocity_.a << endl;
    robotIO->sendBaseReference(DesiredVelocity_.y, -DesiredVelocity_.x, DesiredVelocity_.a);
}

// Saturate function
void DriveControl::saturate(float &value_, float saturate_){
    if(value_ >= saturate_){
        value_ = saturate_;
    };
    if(value_ <= -saturate_){
        value_ = -saturate_;
    };

}

// send status of drivecontroller to world model
void DriveControl::status(){
    if (DC_DEBUG){std::cout << "Current Drivemode: " << Drivemode << std::endl;}
    switch (Drivemode){
        case 1:
            worldmodel->setstatus(Block::DriveController,Block::STATE_BUSY);
            break;
        case 2:
            worldmodel->setstatus(Block::DriveController,Block::STATE_BUSY);
            break;
        case 3:
            worldmodel->setstatus(Block::DriveController,Block::STATE_BUSY);
            break;
        case 4:
            worldmodel->setstatus(Block::DriveController,Block::STATE_DONE);
            break;
        case -1:
            worldmodel->setstatus(Block::DriveController,Block::STATE_ERROR);
            break;
        default :
            break;
        }
}



// Determine sign of value. standard sign funtion is not suitable for desired application
float DriveControl::sign(float in_){
    float sign;
    if (in_ == 0){
        sign = 1;
    }
    else{
        sign = abs(in_)/in_;
    };
    return sign;
}

int DriveControl::closest(std::vector<int> const& vec, int value) {
    std::vector<int> diff_vec;
    for (int i=0; i<vec.size(); i++){
        diff_vec.push_back(value - vec[i] );
    }
    std::sort(diff_vec.begin(), diff_vec.end());
    return (diff_vec[0]+value);

}

// Function to check is the desired direction is free of obstacals and if not, calculate a safe direction if possible
void DriveControl::ColisionPrevention(double& Direction_to_goal, Position& distancetogoRF, close_prx& safeDir, bool& CloseToObject, bool& infeasable_direction){


    double idealDir;
    double Distance_to_goal;
    double minPiIndex;
    double plusPiIndex;
    double minFreeIndeces;
    bool dirIsSafe;
    int smallestDifference;
    int bestDirectionIndex;
    double angleDiff;
    double indexSafeDir;
    std::vector<int> freeDir;
    idealDir = Direction_to_goal;
    Distance_to_goal = sqrt(distancetogoRF.x*distancetogoRF.x+distancetogoRF.y*distancetogoRF.y);


    infeasable_direction = false;

    // Function only needed if robot is not on desired location.
    if (Distance_to_goal >= DriveTransDeadzone){
        // Only determine if the direction is free if the robot is approx. facing the goal.
        // otherwise the robot does not have a full view of the area
        if (true){ // new colision prevention function, if set to false, use old version
            //safeDir = worldmodel->getToClose();
            float R_att;
            float R_rep;
            float F_good = 1;
            float F_bad = 0.00001;

            Vector2 F_att;
            Vector2 F_rep = Vector2(0,0);
            Vector2 F_tot;
            Vector2 F_rep_i = Vector2(0,0);
            float angle;
            float force;
            //bool CloseToObject = false;

            float pushrange = 0.6;


            R_att = F_good*Distance_to_goal;
            //a_att = idealDir;

            F_att = Vector2(distancetogoRF.x, distancetogoRF.y);
            std::cout << "Start: F_att: X: " << F_att.x << "Y: " << F_att.y << std::endl;


            for (int i = 1; i<(safeDir.distances.size()-1); i++){
                if ((safeDir.distances[i] < pushrange)&&(safeDir.toClose[i]==false)){
                    angle = safeDir.start_angle + (safeDir.dt_angle*i);
                    //force = sqrt(pow(((safeDir.distances[i]-pushrange)*cos(angle)),2)+pow(((safeDir.distances[i]-pushrange)*sin(angle)),2));
                    force = safeDir.distances[i] - CLOSEPROXIMITY;
                    R_rep = -F_bad/(pow(force,3));
                    //std::cout << "for loop: Angle: " << angle << " force: " << force << " R_rep: " << R_rep << std::endl;
                    F_rep_i.x =  -R_rep*sin(angle);
                    F_rep_i.y =  R_rep*cos(angle);
                    F_rep = F_rep+F_rep_i;
                    //std::cout << "for loop: F_rep_i: X: " << F_rep_i.x << " Y: " << F_rep_i.y << std::endl;
                    CloseToObject = true;
                }

            }
            //saturate(F_rep.y, 0.5*F_att.y);
            F_tot.x = F_att.x+0.1*F_rep.x;
            F_tot.y = F_att.y+0.1*F_rep.y;

            distancetogoRF.x = F_tot.x;
            distancetogoRF.y = F_tot.y;
            //distancetogoRF.a = atan2(-F_tot.x, F_tot.y);
            //saturate(distancetogoRF.a, M_PI/2);
            //Direction_to_goal = distancetogoRF.a;

            std::cout << "for loop: F_rep sum: X: " << F_rep.x << " Y: " << F_rep.y << std::endl;
            std::cout << "for loop: F_tot: X: " << F_tot.x << " Y: " << F_tot.y << std::endl;
            std::cout << "Distance to go: X: " << distancetogoRF.x << " Y: " << distancetogoRF.y << " Angle: " << distancetogoRF.a << std::endl;

            //Direction_to_goal =
            if ((abs(F_att.x)<0.01)&&(abs(F_tot.y) < 0.01)&& abs(F_tot.x) < 0.05){
                infeasable_direction = true;
                std::cout << "Infeasable direction" << std::endl;
            }



            //return infeasable_direction;


            /*
            Vector2 cumm_direction = Vector2(-sin(Direction_to_goal), cos(Direction_to_goal));
            std::cout << "Cumm sum for vector at start: " << cumm_direction.angle() << " with dirtogoal: " << Direction_to_goal << std::endl;


            for (int i = 0; i<safeDir.distances.size(); i++){
                if (safeDir.distances[i] < pushrange){

                    float angle = safeDir.start_angle + (safeDir.dt_angle*i);
                    float magnitude = -0.1*(pushrange-safeDir.distances[i]);
                    //std::cout << "Add angle: " << angle << " distance: " << safeDir.distances[i] <<" to weigthing with magnitude: "<< magnitude << std::endl;
                    Vector2 addvect = Vector2((cos(angle)*magnitude), -(sin(angle)*magnitude));
                    cumm_direction = cumm_direction + addvect;

                    //std::cout << "Cumm sum for vector: " << cumm_direction.angle() << " Last added vector: "<< addvect.x << std::endl;
                }
            }
            float angle_diff = atan2(-cumm_direction.x, cumm_direction.y) - Direction_to_goal;
            Direction_to_goal += angle_diff;
            std::cout << "Cumm sum for vector after: " << cumm_direction.angle() << " Dir to goal: " << Direction_to_goal << std::endl;
            */
        }
        else{


        if (abs(idealDir) <= M_PI/2){
            safeDir = worldmodel->getToClose();
            if (safeDir.valid){
                // determine if the range around the desired direction is free
                minFreeIndeces = round((minFreedeg*M_PI/180)/safeDir.dt_angle); // amount of samples in data corresponding to the desired angle
                indexSafeDir = round((idealDir-safeDir.start_angle)/safeDir.dt_angle);
                minPiIndex = minFreeIndeces+1;
                plusPiIndex = safeDir.toClose.size()-minFreeIndeces-1;
                if (minPiIndex <minFreeIndeces+1){minPiIndex =minFreeIndeces+1;}
                if (plusPiIndex > safeDir.toClose.size()-minFreeIndeces-1){minPiIndex =safeDir.toClose.size()-minFreeIndeces-1;}

                // Find directions where there is no wall
                for (int i = minPiIndex; i<plusPiIndex; i++){
                    dirIsSafe = true;
                    for (int j=-minFreeIndeces; j<(minFreeIndeces+1); j++){
                        if (safeDir.toClose[i+j]){
                            dirIsSafe = false;
                        }
                    }
                    if (dirIsSafe == true){
                        freeDir.push_back(i);
                    }

                }

                if (freeDir.size()>0){
                    // Get the closest direction towards the goal that is free
                    smallestDifference=100;
                    for (int i=0; i<freeDir.size(); i++){
                        if (abs(freeDir[i]-indexSafeDir)<smallestDifference){
                            smallestDifference = abs(freeDir[i]-indexSafeDir);
                            bestDirectionIndex = freeDir[i];
                        }
                    }
                    if (bestDirectionIndex == indexSafeDir){
                        if (DC_DEBUG){std::cout << "Safe direction is equal to desired direction, do nothing " << std::endl;}
                    }
                    else{
                        // found new safe direction, performing a shift and rotation of the desired location (setpoint)
                        Direction_to_goal = ((bestDirectionIndex*safeDir.dt_angle)+safeDir.start_angle);
                        angleDiff = Direction_to_goal-idealDir;

                        // check if the new angle is not to different from original, if so the robot moves away from goal
                        if (abs(angleDiff) > maxAngleShift*M_PI/180){
                            infeasable_direction = true;
                            if (DC_DEBUG){std::cout << "Infeasable" << std::endl;}
                        }
                        else{
                            // New direction satisfies the requirements, set new direction as goal
                            distancetogoRF.y = Distance_to_goal*cos(angleDiff);
                            distancetogoRF.x = -Distance_to_goal*sin(angleDiff);
                            if (angleDiff>0){
                                distancetogoRF.x -= collisionSpeedOffset;
                                Direction_to_goal += collisionAngleOffset;
                            }
                            else{
                                distancetogoRF.x += collisionSpeedOffset;
                                Direction_to_goal -= collisionAngleOffset;
                            }


                            if (DC_DEBUG){
                                std::cout << "Ideal direction index " << indexSafeDir << std::endl;
                                std::cout << "Best direction index " << bestDirectionIndex << std::endl;
                                std::cout << "Ideal direction angle " << idealDir*180/M_PI << std::endl;
                                std::cout << "Best direction angle " << Direction_to_goal*180/M_PI << std::endl;
                                std::cout << "angle shift " << angleDiff*180/M_PI << std::endl;
//                                std::cout << "Original distance to goal " << Distance_to_goal << std::endl;
//                                std::cout << "Distance to new goal " << distancetogoRF.y << std::endl;
                                Distance_to_goal = sqrt(distancetogoRF.x*distancetogoRF.x+distancetogoRF.y*distancetogoRF.y);
                            }
                        }
                    }
                }
                else {
                    // No obstacle free directions found
                    infeasable_direction = true;
                    if (DC_DEBUG){std::cout << "Infeasable" << std::endl;}
                }
            }
        }
        //return infeasable_direction;
        }

    }

}
