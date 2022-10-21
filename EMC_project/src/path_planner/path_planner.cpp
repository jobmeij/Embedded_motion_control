#include "path_planner.h"
#include "../config.h"
#include "../world_model/world_model.h"
#include "../vector2.h"
#include <iostream>

/* BLOCK DESCRIPTION: New pathplanner for hospital challenge:

               __
             <(o )___
              ( ._> /
               `---'   Rubber ducking FTW

 Block Block:
    - Init
    - Idle
    - Rotate
    - Plan
    - Follow
    - Middle_room
    - Break_links
    - Next_node

 Block states:
    - Busy
    - Done
    - Error

 Functions:
    - createPath: creating a possible path from all available nodes, placing extra nodes if a path is blocked by e.g. a wall (L-shaped room)
    - resetBrokenLinks: resetting all blocked links between nodes. If no blocked links are resetted, go to error state
    - storePath: storing a list of all nodes that the robot has to drive towards in the world model

 Definitions:
    - Node: a point that the robot can drive towards
    - Link: a connection between nodes

 Path planner is active in the 'Relocate phase', which consists of the following cases:
    - Case 1: Path planner plans a path for the robot to drive
    - Case 2: Path planner sets next node for drive controller
    - Case 3: Drive controller is moving towards the set node, path planner waits until drive controller is done
    - Case 4: In case the drive controller gives an error, case 4 is activated: this resets all broken links between nodes and resets to case 1.
    - Init/idle: Waiting for input
    - Rotate: rotating the robot a given angle

*/

//
//
// Path planner executes the following functions (MAIN)
void PathPlanner::pathPlanner(){
    if (PP_DEBUG) {cout << "Pp: Starting path planner (Pp)" << endl;} // DEBUG

    // Get PP MODE and STATE

    PpBlockMode = worldmodel->getmode(Block::PathPlanner);
    Block::Status PpBlock = worldmodel->getstatus(Block::PathPlanner);

    // Output PP MODE and STATE for DEBUG
    if (PP_DEBUG) {cout << "Pp: Blockmode: " << PpBlockMode << ", Block status: " << PpBlock << endl;} // DEBUG

    vector<node>& nodeList = worldmodel->getnodelist(); // Create pointer towards memory location of nodeList in worldmodel
    vector<link>& linkList = worldmodel->getlinklist(); // Create pointer towards memory location of linklist in worldmodel
    vector<node>& path = worldmodel->getpath(); // Create pointer towards memory location of path in worldmodel

    Map& globalmap = worldmodel->getglobalmap();
    Map& currentmap = worldmodel->getcurrentmap();
    Position& curpos = worldmodel->getcurrentposition();
    Position& desiredpos = worldmodel->getdesiredposition();

    // DEBUG: set PP block mode to plan for testing the path planning algorithm
//    cout << "PP MODE: " << PpBlockMode << " , expected: " << Block::PP_MODE_PLAN << endl;
//    PpBlockMode = Block::PP_MODE_PLAN;
    //


    //
    // Execute functions for given relocate case
    switch (PpBlockMode){
    case Block::MODE_INIT:

        // DEBUGGING:
        if (debugPlaceNodes == true){

            // Create nodes
            nodeList.clear(); // Remove old insertion
            nodeList.push_back(node(Vector2(0,0),0,node::start));
            nodeList.push_back(node(Vector2(1,0),0,node::normal));
            nodeList.push_back(node(Vector2(0,1),0,node::normal));
            nodeList.push_back(node(Vector2(1,2),0,node::normal));
            nodeList.push_back(node(Vector2(3,1),0,node::destination));
            nodeList.push_back(node(Vector2(2,0),0,node::normal));

            // Create links
            linkList.clear(); // Remove old insertions
            linkList.push_back(link(&nodeList[0],&nodeList[1],false));
            linkList.push_back(link(&nodeList[1],&nodeList[2],false));
            linkList.push_back(link(&nodeList[2],&nodeList[3],false));
            linkList.push_back(link(&nodeList[3],&nodeList[4],false));
            linkList.push_back(link(&nodeList[1],&nodeList[5],false));
            linkList.push_back(link(&nodeList[4],&nodeList[5],true));

            debugPlaceNodes = false;
        }

        worldmodel->setstatus(Block::PathPlanner, Block::STATE_DONE);
        break;
    case Block::MODE_IDLE:

        worldmodel->setstatus(Block::PathPlanner, Block::STATE_DONE);
        break;
    case Block::PP_MODE_PLAN:
        // Create path
        pathPossible = createPath(nodeList, linkList, path);
        // Check if there is a possible path
        if (!pathPossible) // No path possible
        {
            brokenLinksResetted = resetBrokenLinks(linkList); // Reset all broken links and try again

            if (!brokenLinksResetted) // If there were no broken links to reset: go to error state
            {
                worldmodel->setstatus(Block::PathPlanner, Block::STATE_ERROR);
                break;
            }
            else // Path possible, plan path again
            {
                pathPossible = createPath(nodeList, linkList, path);

                if (!pathPossible)  // Still no path possible: go to error state
                {
                    worldmodel->setstatus(Block::PathPlanner, Block::STATE_ERROR);
                    break;
                }
            }
        }

        // Set block state to done
        worldmodel->setstatus(Block::PathPlanner, Block::STATE_DONE);

        break;
    case Block::PP_MODE_NEXTNODE:
        // Proceed path by outputting the next node to the drive controller. If there is no next node, set status done
        if(!setnextnode(path, desiredpos)){
            worldmodel->setstatus(Block::PathPlanner, Block::STATE_BUSY);
        }else{
            worldmodel->setstatus(Block::PathPlanner, Block::STATE_DONE);
        }

        break;
    case Block::PP_MODE_FOLLOW:
        // Drivecontroller is driving to node, check if path is clear from obstructions

        // TODO

        // Set block state to busy
        worldmodel->setstatus(Block::PathPlanner, Block::STATE_BUSY);

        break;
    case Block::PP_MODE_BREAKLINKS:
        // Reset all broken links
        brokenLinksResetted = resetBrokenLinks(linkList);

        // Set block state to done
        worldmodel->setstatus(Block::PathPlanner, Block::STATE_DONE);

        break;
    case Block::PP_MODE_ROTATE:
        // Rotate the robot
        rotateleft(desiredpos, ROBOTROTATEINTERVAL);

        // Set block state to done
        worldmodel->setstatus(Block::PathPlanner, Block::STATE_DONE);

        break;
    default:
        break;
    }

    // Add nodes to global map
    addnodestoglobalmap(nodeList, linkList, globalmap, path);

    if (PP_DEBUG) {cout << "Pp: Path planner iteration finished" << endl;} // DEBUG
}

//
// Creating a path based on available nodes, returning true if a path is available
bool PathPlanner::createPath(vector<node>& nodeList, vector<link>& linkList, vector<node>& path){

    cout << "PP: Starting planning a path!" << endl;
    bool pathFound = false;

    pathposition = -1;
    path.clear();

    bool debugCreatePath = false;

    if (debugCreatePath){  // Insert a given path of nodes in the world model
        path.push_back(nodeList[17]);
        path.push_back(nodeList[13]);
        path.push_back(nodeList[12]);
        path.push_back(nodeList[18]);
        path.push_back(nodeList[11]);
        path.push_back(nodeList[10]);
        path.push_back(nodeList[16]);
        path.push_back(nodeList[2]);
        pathFound = true;
    }
    else{ // THIS IF/ELSE CASE HAS TO BE REMOVED AFTER DEBUGGING, ONLY KEEPING THE ELSE STATEMENT


//    if (PP_DEBUG) {cout << "Pp: Creating a path" << endl;} // DEBUG
        // Determine the path using the algorithm

        node tempNode;
        for (int i=0; i<nodeList.size(); i++){
            tempNode = nodeList.at(i);
            if (PP_DEBUG){cout << "PP: Node " << i << " has " << tempNode.nodelinks.size() << " connected nodes " << endl;}
        }

        // Identify start and destination node, TBD if this remains in the code like this (how are start and destination node set?)
        node startNode;
        bool startNodeSet = false;
        node destinationNode;
        bool destinationNodeSet = false;

        for (int i=0; i<nodeList.size(); i++){
            node tempNode = nodeList.at(i);

            if (tempNode.property == node::start && !startNodeSet){
                startNode = tempNode;
                startNodeSet = true;
            }
            else if (tempNode.property == node::destination && !destinationNodeSet){
                destinationNode = tempNode;
                destinationNodeSet = true;
            }
        }


        //
        // DEBUG: SETTING START AND DESTINATION NODE MYSELF
        startNode = nodeList.at(17);            // was 11 for happy flow, 3 for worst case
        startNodeSet = true;
        destinationNode = nodeList.at(0);      // was 18 for happy flow, 1 for worst case
        destinationNodeSet = true;

        if (PP_DEBUG){cout << "PP: Start node location: x= " << startNode.location.x << " , y= " << startNode.location.y << endl;}
        if (PP_DEBUG){cout << "PP: Destination node location: x= " << destinationNode.location.x << " , y= " << destinationNode.location.y << endl;}

        // Check if start and destination node are directly linked
        if (PP_DEBUG){cout << "PP: Link list size is " << linkList.size() << endl;}
        for (int k=0; k<linkList.size(); k++){
            // Copying nodes from link
            link tempLink = linkList.at(k);
            node tempNode1 = *tempLink.node1;
            node tempNode2 = *tempLink.node2;

            if ((tempNode1.location == startNode.location && tempNode2.location == destinationNode.location) || (tempNode1.location == destinationNode.location && tempNode2.location == startNode.location))
                if (PP_DEBUG){cout << "PP: Start and destination node have a direct link!" << endl;}
        }
        // DEBUG END
        //


        if (PP_DEBUG){cout << "PP: Start node found: " << startNodeSet << " , destination node found: " << destinationNodeSet << endl;}

        // Security check to see if there are a start and destination node
        if (!startNodeSet && !destinationNodeSet)
            return false;

        //
        // Start planning paths
        vector<vector<node> > plannedPaths;                                     // Vector containing all planned paths, which are vectors of nodes
        vector<float> plannedPathCost;                                          // Cost for each planned path of plannedPaths map

        // For loop used instead of while loop to protect the program from getting stuck, tune the pathPlanMaxStages accordingly!
        for (int i=0; i<PP_MAX_PLANNING_STAGES; i++){

            // Check if i is its max value, if it is change block status to error
            if (i == PP_MAX_PLANNING_STAGES-1)
            {
                if (PP_DEBUG){cout << "PP: NO PATH FOUND! Try increasing the iteration limit." << endl;}
                worldmodel->setstatus(Block::PathPlanner,Block::STATE_ERROR);
            }

            // Stop if a path is found
            if (pathFound)
                break;

            // Initial node is slightly different
            if (i==0){

                // Get destination node
                node currentNode = destinationNode;                             // Destination node
                vector<node> initialPath;                                       // Vector used to create initial paths
                initialPath.push_back(currentNode);                             // Insert destination node

                // Place all paths originating from the destination node in the plannedPaths vector including the cost in plannedPathCost vector
                for (int k=0; k<linkList.size(); k++){                          // Scanning link list for all connected nodes
                    // Copying nodes from link because fuck pointers, will fix later
                    link tempLink = linkList.at(k);
                    node tempNode1 = *tempLink.node1;
                    node tempNode2 = *tempLink.node2;

                    // Create a temporary path and cost
                    vector<node> tempPath = initialPath;

                    // Determine if one of the temporary nodes is the destination node
                    if (tempNode1.location == destinationNode.location){
                        // Add tempNode2 to path and store it in plannedPaths
                        tempPath.push_back(tempNode2);
                        plannedPaths.push_back(tempPath);
                        plannedPathCost.push_back(linkList.at(k).distance);
                    }
                    else if (tempNode2.location == destinationNode.location){
                        // Add tempNode1 to path and store it in plannedPaths
                        tempPath.push_back(tempNode1);
                        plannedPaths.push_back(tempPath);
                        plannedPathCost.push_back(linkList.at(k).distance);
                    }
                }

                // Check if start node is already available by scanning the final nodes of all the planned paths
                pathFound = searchStartNode(startNode, plannedPaths, plannedPathCost, path, i);

            }
            else{

                // Now:
                // 1. Select path with lowest cost and select the latest node
                // 2. Determine all links of this node
                // 3. Replace the initial path and add more paths if needed, also adding the cost of the newly added paths
                // 4. Check if the start node is now at the end of one of the planned paths. If this planned path also has the lowest cost of all paths, this is the returned path.

                // Take path with lowest cost until now
                int iteratorLowestCostPath;
                float lowestCost;
                for (int i=0; i<plannedPathCost.size(); i++){
                    if (i==0 || plannedPathCost.at(i) < lowestCost){
                        iteratorLowestCostPath = i;
                        lowestCost = plannedPathCost.at(i);
                    }
                }

                // Pick the path with the lowest cost
                vector<node> lowestCostPath = plannedPaths.at(iteratorLowestCostPath);

                //
                // DEBUG
//                cout << "PP: i= " << i << endl;
//                pathToConsole(lowestCostPath, lowestCost);
                // DEBUG END
                //

                // Take the last node from lowestCostPath and determine the node that is reachable with the lowest link cost
                node currentNode = lowestCostPath.at(lowestCostPath.size()-1);  // Picking latest node

                // Determine all connected nodes, add them to the path with lowest cost and push them back to the plannedPaths vector
                int countConnectedNodes = 0;
                for (int k=0; k<linkList.size(); k++){                          // Scanning link list for all connected nodes
                    // Copying nodes from link because fuck pointers, will fix later
                    link tempLink = linkList.at(k);
                    node tempNode1 = *tempLink.node1;
                    node tempNode2 = *tempLink.node2;

                    // Create a temporary path and cost and add the selected lowest cost path to it
                    vector<node> tempPath = lowestCostPath;
                    node tempNode;
                    bool nodeAlreadyVisited = false;

                    // Determine if one of the temporary nodes is the destination node
                    if (tempNode1.location == currentNode.location || tempNode2.location == currentNode.location){

                        // Determine the correct tempNode what to add to the tempPath vector
                        if (tempNode1.location == currentNode.location)
                            tempNode = tempNode2;
                        else if (tempNode2.location == currentNode.location)
                            tempNode = tempNode1;

                        // Check if tempNode is already visited
                        nodeAlreadyVisited = checkNodeAlreadyVisited(tempPath,tempNode);

                        // if nodeAlreadyVisited = true, skip this path
                        if (nodeAlreadyVisited){
//                            cout << "PP: Node was already visited (x=" << tempNode.location.x << ",y=" << tempNode.location.y << "), skipping this path." << endl;
                            continue;
                        }
                        else
                            tempPath.push_back(tempNode);

                        // Determine if the initially selected path is overwritten or a new planned path is created
                        if (countConnectedNodes == 0){
                            plannedPaths[iteratorLowestCostPath] = tempPath;
                            plannedPathCost[iteratorLowestCostPath] = (lowestCost + linkList.at(k).distance);
                            countConnectedNodes++;
                        }
                        else{
                            plannedPaths.push_back(tempPath);
                            plannedPathCost.push_back(lowestCost + linkList.at(k).distance);
                            countConnectedNodes++;
                        }
                    }
                }

                if (PP_DEBUG){cout << "PP: i=" << i << " planned paths size is " << plannedPaths.size() << endl;}

                // Now the new paths are inserted: checking if the start node is present in one of the planned paths
                // If the start node is present and it has the lowest cost of all planned paths, this is the found path
                pathFound = searchStartNode(startNode, plannedPaths, plannedPathCost, path, i);
            }
        }
    }

    for(unsigned long p = 0; p < path.size()-1; p++){
        Vector2 diff;
        diff = path[p+1].location - path[p].location;
        float angle = atan2(diff.y, diff.x);
        path[p].angle = angle;
    }

    // Printing found path nodes in console if DEBUG mode is enabled
    pathToConsole(path,0);

    cout << "PP: Path planned!" << endl;

    return pathFound;
}

//
// Check the path if the given node is already in it, if this is the case return true
bool PathPlanner::checkNodeAlreadyVisited(vector<node> path, node Node)
{
    node tempNode = path.at(path.size()-2);    // minus 2 because size is already iterator+1

    if (tempNode.location == Node.location)
        return true;
    else
        return false;
}

//
//
// Function to print a path in the console
void PathPlanner::pathToConsole(vector<node> path, float cost)
{
    cout << "PP: Printing path nodes:" << endl;

    for (int i=0; i<path.size(); i++){
        node tempNode = path.at(i);
        cout << "PP: planned path node " << i << " location: x= " << tempNode.location.x << " , y= " << tempNode.location.y << endl;
    }

    cout << "PP: planned path has cost " << cost << endl;
}

//
//
// Function of createPath() to check if the start node is in one of the paths, returning true if this path also has the lowest cost
bool PathPlanner::searchStartNode(node startNode, vector<vector<node>> plannedPaths, vector<float> plannedPathCost, vector<node>& path, int i)
{
    for (int h=0; h<plannedPaths.size(); h++){
        vector<node> tempPlannedPath = plannedPaths.at(h);
        node tempFinalNode = tempPlannedPath.at(tempPlannedPath.size()-1);

        if (PP_DEBUG){cout << "PP: Location temporary node: x= " << tempFinalNode.location.x << " , y= " << tempFinalNode.location.y << endl;}

        if (tempFinalNode.location == startNode.location){
            // Start node found!

            // Determine iteration of path
            int foundPathIteration = h;
            float foundPathCost = plannedPathCost.at(foundPathIteration);
            bool foundLowestCost = true;

            // Check if it is the path with the lowest cost, then store it in the world model
            for (int g=0; g<plannedPaths.size(); g++){
//                if (PP_DEBUG) {cout << "PP: planned path cost at g= " << g << " is " << plannedPathCost.at(g) << " , found path cost is " << foundPathCost << endl;}
                if (foundPathCost > plannedPathCost.at(g)){
                    foundLowestCost = false;
                }
            }

            if (PP_DEBUG){cout << "PP: Start node found after " << i << " iterations! Lowest cost path towards start node (true/false): " << foundLowestCost << endl;}

            // If cost is the lowest, store the path in the world model
            if (foundLowestCost){
                if (PP_DEBUG){cout << "PP: PATH FOUND! Cost of path was the lowest, now storing the found path in the world model." << endl;}


                // Reverse planned path vector because it is backwards
                reverse(tempPlannedPath.begin(), tempPlannedPath.end());

                // Store planned path in worldmodel
                path = tempPlannedPath;

                return true;                   // Set path found to true
            }
            else
                return false;
        }
    }
}

//
// UNTESTED FUNCTION
// Take all nodes from obstructedNodes and move them to connectedNodes
bool PathPlanner::resetBrokenLinks(vector<link>& linkList){
//    if (PP_DEBUG) {cout << "Pp: Resetting broken links between all nodes" << endl;} // DEBUG

//    vector<node>& nodeList = worldmodel->getnodelist();   // Update node list
//    node tempNode;                          // Temporary node for storage
//    int resettedObstructions = 0;

//    for (int i=0; i<nodeList.size() ; i++){
//        tempNode = nodeList.at(i);

//        for (int k=0; k<tempNode.obstructednodes.size(); k++){
//            if (tempNode.connectednodes.size() > 0){
//                tempNode.connectednodes.insert(tempNode.connectednodes.end(),tempNode.obstructednodes.at(k));
//            }
//            else{
//                tempNode.connectednodes.insert(tempNode.connectednodes.begin(),tempNode.obstructednodes.at(k));
//            }

//            resettedObstructions++;
//        }

//        tempNode.obstructednodes.clear();   // Delete all obstructed nodes
//        nodeList.at(i) = tempNode;          // Store node list again
//    }

//    // Determine if at least one obstructed node is resetted
//    if (resettedObstructions > 0)
//        return true;
//    else
//        return false;
}

//
// Determine if there are obstructions between the robot and the set node, return true if there are obstructions
bool PathPlanner::obsturctionsToSetNode(){
    return false;
}

bool PathPlanner::setnextnode(vector<node> &path, Position &desiredpos){
    bool reachedend = false;
    pathposition++;
    std::cout << "Pathpos: " << pathposition << " Length of path: "<< path.size() << std::endl;
    if(pathposition < path.size()){
        node& nextnode = path[pathposition];
        desiredpos = Position(nextnode.location.x, nextnode.location.y, nextnode.angle);
    }else{
        reachedend = true;
    }
    //std::cout<<"From Path planner : "<<"desired position : "<< desiredpos.x <<" "<< desiredpos.y<<" "<< desiredpos.a<<std::endl;
    return reachedend;
}

void PathPlanner::addnodestoglobalmap(vector<node>& nodelist, vector<link>& linklist, Map& globalmap, vector<node>& path){
    //Nodes
    globalmap.removeobjects(); //removes objects with the remove flag set
    for(unsigned long n = 0; n < nodelist.size(); n++){
        node &curnode = nodelist[n];

        //Node objects
        char name[15] = "";
        Color nodecolor = {255,255,255};
        switch (curnode.property){
            case node::normal:
                nodecolor[0] = 255;
                nodecolor[1] = 255;
                nodecolor[2] = 255;
            break;
            case node::start:
                nodecolor[0] = 0;
                nodecolor[1] = 255;
                nodecolor[2] = 0;
            break;
            case node::destination:
                nodecolor[0] = 255;
                nodecolor[1] = 0;
                nodecolor[2] = 0;
            break;
        }
        sprintf(name,"N:%lu",n);
        Object nodeobj(Object::node,name,nodecolor,Objectoptions::points,Objectoptions::open);
        Vector2 pos(curnode.location.x, curnode.location.y);
        nodeobj.points.push_back(Point(pos,0,Point::floating));
        nodeobj.pointradius = 3;
        nodeobj.remove = true;
        globalmap.objects.push_back(nodeobj);

    }
    //Links
    for(unsigned long l = 0; l < linklist.size(); l++){
        link &curlink = linklist[l];
        //link objects
        char name[15] = "";
        Color linkcolor = {255,255,255};
        if (curlink.obstructed){
            linkcolor[0] = 255;
            linkcolor[1] = 0;
            linkcolor[2] = 0;
        }else{
            linkcolor[0] = 255;
            linkcolor[1] = 255;
            linkcolor[2] = 255;
        }
        sprintf(name,"L:%.1f",curlink.distance);
        Object linkobj(Object::node,name,linkcolor,Objectoptions::lines,Objectoptions::open);
        Vector2 pos1 = curlink.node1->location;
        Vector2 pos2 = curlink.node2->location;
        linkobj.points.push_back(Point(pos1,0,Point::floating));
        linkobj.points.push_back(Point(pos2,0,Point::floating));
        linkobj.pointradius = 3;
        linkobj.remove = true;
        globalmap.objects.push_back(linkobj);
    }
    //Path

    //path object
    char pathname[15] = "";
    Color pathcolor = {0,255,0};
    Object pathobj(Object::path,pathname,pathcolor,Objectoptions::lines,Objectoptions::open);
    pathobj.pointradius = 3;
    pathobj.remove = true;
    for(unsigned long p = 0; p < path.size(); p++){
        node &curnode = path[p];
        Vector2 pos(curnode.location.x, curnode.location.y);
        pathobj.points.push_back(Point(pos,0,Point::floating));
    }
    globalmap.objects.push_back(pathobj);

    //Next path node
    if(pathposition < path.size()){
        char pathnodename[15] = "";
        Color pathnodecolor = {0,0,255};
        Object pathnodeobj(Object::path,pathnodename,pathnodecolor,Objectoptions::points,Objectoptions::open);
        pathnodeobj.pointradius = 3;
        pathnodeobj.remove = true;
        node &curnode = path[pathposition];
        Vector2 pos(curnode.location.x, curnode.location.y);
        pathnodeobj.points.push_back(Point(pos,0,Point::floating));
        globalmap.objects.push_back(pathnodeobj);
    }
}

void PathPlanner::rotateleft(Position &desiredpos, float amount){
    desiredpos.a += amount;
}
