#define EXECUTION_RATE 20 // [Hz]

#define RESOLUTION 0.02 //visualisation

////////////////////////////////////////TASK MANAGER/////////////////////////////////////////////////////////////////
#define TM_DEBUG false // Debug mode for the task manager

////////////////////////////////////////PERCEPTOR/////////////////////////////////////////////////////////////////
#define LASERANGLE 3.5 //[angle rad] angle to determine which laser data is taken into account
#define MINRANGE 0.05 //[m] min distance to include laserdata

#define FITDIFFERENCE 0.3 //

#define CABINETNUMBERS 4 // number of cabinets in the map

#define AVGLINEFITERROR 0.02 //[m] maximum line fit error
#define MAXLINEFITERROR 0.1 //[m] maximum line fit error
#define CLUSTERSIZE 4 //[minimum number of datapoints] perception
#define SAMPLEDISTANCE 0.01 //[m] minimum distance to resample
#define DISTANCEDIFFERENCE 0.2 //[m] max distance deviation for a cluster

#define MAXMAPSIZE 500 //[max number of objects in map] to ensure memory doesnt blow up > perceptor

#define MINDOORLENGTH 0.3 //[m] minimum length of door
#define MAXDOORLENGTH 1.7 //[m] maximum length of door

#define POINTCOMBINEDISTANCE 0.1 //[m] max distance to combine corner points > perception
#define ANGLEMERGEDIFFERENCE 0.8 //[radians] maximum angle deviation for a merge > perception
#define DISTMERGEDIFFERENCE 0.1 //[m] maximum perpundicular dist deviation for a merge > perception

#define PC_DEBUG false  // Debug mode for the Perceptor

///////////////////////////////////DRIVE CONTROLLER///////////////////////////////////////////////////////////////

#define MaxTransVelocity 0.5//0.5            // 0.5 //Maximum allowed translational velocity [m/sec]
#define MaxRotVelocity 1//1.2              // 1.2 // Maximum allowed rotational velocity [rad/sec]
#define DriveTransDeadzone 0.15          // Distance when the drive controller stops driving
#define DriveRotDeadzone 0.1            // Distance when the drive controller stops rotating
#define DC_DEBUG false                  // Debug mode for the path planner, enables outputting status

/////////////////////////////////////PATH PLANNER////////////////////////////////////////////////////////////////

// Definitions for path planner
#define PP_DEBUG false                   // Debug mode for the path planner, enables outputting status
#define PP_DRVFWDDIST 5                 // Distance that the robot drives forward if no finish line is found
#define ROBOTROTATEINTERVAL  0.8        //[rad] rotation angle of leftrotate function
#define PP_NOLINKS_PENALTY 1000         // Cost penalty for nodes with no links TBD needed?
#define PP_MAX_PLANNING_STAGES 10000     // The maximum stages (iterations) for planning a path between the nodes

/////////////////////////////////////COLISION PREVENTION/////////////////////////////////////////////////////////////////////////////

#define minFreedeg 45                   // [deg] +/- this angle towards the goal should be free of obstacles
#define maxAngleShift 65                // [deg] max angle shift before the solution is to be assumed infeasable
#define collisionSpeedOffset 0.4        // [m] offset given to the x direction if robot is close to a wall. Ensures that the robot moves away from the wall
#define collisionAngleOffset 0.2        // [rad] offset given to the desired angle if robot is close to a wall. Ensures that the robot moves away from the wall

#define CLOSEPROXIMITY 0.3             // [m] Radius around the robot to prevent collisions

/////////////////////////////////////BEHAVIOUR SETTINGS/////////////////////////////////////////////////////////////////////////////
#define max_fit_cycles 5
#define max_rotate_cycles 5
