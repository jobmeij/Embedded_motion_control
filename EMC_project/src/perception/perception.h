#include <emc/io.h>
#include "../config.h"
#include "../world_model/world_model.h"
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <list>
#include "../visualisation/visualisation.h"
#include "../graph/graph.h"
#include <vector>
#include "../json.hpp"
#include <fstream>


#include <emc/io.h>
#include "../config.h"
#include "../world_model/world_model.h"
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <list>
#include "../visualisation/visualisation.h"
#include "../graph/graph.h"
#include <vector>

#ifndef perception_H
#define perception_H

using namespace std;

class Perception{
private:
    emc::IO* robotIO;
    emc::LaserData laserdatainst;
    emc::OdometryData odometrydata;
    WorldModel* worldmodel;
    Visualisation* visualisation;
    Graph* graph;

public:
    Perception(WorldModel* worldmodel_, emc::IO* robotIO_, Visualisation* visualisation_, Graph* graph_){
        worldmodel = worldmodel_;
        robotIO = robotIO_;
        visualisation = visualisation_;
        graph = graph_;
    }

    void init();
    void creategm(Map &globalmap, vector<node>& nodelist, vector<link>& linklist);
    void evaluate();
    void closeproximity(emc::LaserData laserdata);
    void scan(Map &currentmap, Position& curpos, emc::LaserData laserdata);
    void locate();
    void merge(Map &currentmap, Map &localmap, Position &curpos);
    void identifydoors(Map &map);
    void identifyrooms(Map &map);
    int fitmap(Map &localmap,Map &globalmap, vector<node>& nodelist, vector<node>& path, float& curbestfitscore);
    void align(Map& scanmap, Map& globalmap, Map& localmap, vector<node>& nodelist, vector<node>& path);

    void combinepoints(Map &map);
    void polar2cartesian(std::vector<Vector2> &cartesiandata, std::vector<float> &radii, float angle0, float angleincrement);
    void resample(std::vector<Vector2> &cartesiandata, float mindist);
    void fitlines(std::vector<Vector2> &cartesiandata, Map &map);
    void projectpoints(Map &map);
    void pointproperties(Map &map, Position &curpos);
    void removepointproperties(Map &map, Object::types type);
    void transform(Map& map, vector<node>& nodelist, vector<node>& path, float x, float y, float a, bool direction);
};

#endif //detection_H
