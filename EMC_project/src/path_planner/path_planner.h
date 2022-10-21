
#include <emc/io.h>
#include <cmath>

#include "../config.h"
#include "../objects.h"
#include "../vector2.h"
#include "../world_model/world_model.h"

#ifndef pathplanner_H
#define pathplanner_H

class PathPlanner
{
private:
    WorldModel* worldmodel;

    // Functions
    bool createPath                             (vector<node>& nodeList, vector<link>& linkList, vector<node>& path);
    bool checkNodeAlreadyVisited                (vector<node> path, node Node);
    void pathToConsole                          (vector<node> path, float cost);
    bool searchStartNode                        (node startNode, vector<vector<node>> plannedPaths, vector<float> plannedPathCost, vector<node>& path, int i);
    bool resetBrokenLinks                       (vector<link>& linkList);
    void breakAllLinks                          (vector<link>& linkList);
    bool setnextnode                            (vector<node>& path, Position& desiredpos);
    bool obsturctionsToSetNode                  ();
    void rotateleft                             (Position& desiredpos, float amount);
    void addnodestoglobalmap                    (vector<node>& nodelist, vector<link>& linklist, Map& globalmap, vector<node>& path);

    // Variables
    bool brokenLinksResetted;
    bool pathPossible;
    bool obstructionDetected;
    int relocateCase;
    long pathposition;
    Block::Modes PpBlockMode;

    bool debugPlaceNodes = false;   // DEBUG


public:
    PathPlanner(WorldModel* worldmodel_){ worldmodel = worldmodel_; }
    void pathPlanner();
};

#endif //pathplanner_H

