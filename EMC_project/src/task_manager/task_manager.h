#include <emc/io.h>
#include "../config.h"
#include "../world_model/world_model.h"
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <list>
#include "../visualisation/visualisation.h"
#include "../graph/graph.h"
//#include "./tasks.h"

#ifndef taskmanager_H
#define taskmanager_H

class TaskManager{

private:

    WorldModel* worldmodel;
    emc::IO* robotIO;
    int fit_cycle_counter=0;
    int nr_rotate=0;
    int objectivecnt=0;
    //int rotatecnt=0;


    std::vector<int>cabinets;
    int cabinet_to_visit;
    //int ncurrhlt=0;
    int nr_cabinets_to_visit;
    //int nr_cabinets;
    int hlt=0;
    std::string current_phase;
    int current_case;


    struct Blockstatus
        {
            Block::Status PCstatus;
            Block::Status PPstatus;
            Block::Status DCstatus;
        };

    struct Blockmodes
    {
        Block::Modes PCmode;
        Block::Modes PPmode;
        Block::Modes DCmode;
    };

public:


        Blockstatus status;
        Blockmodes modes;

        TaskManager(WorldModel* worldmodel_,emc::IO* robotIO_){
        worldmodel = worldmodel_;
        robotIO = robotIO_;
    }
    void init();
    void execute();
    void check_exitconditions(std::string &current_phas, int &current_cas, Blockstatus &statu, Blockmodes &mod );
    void change_blockmodes(std::string &current_phas, int &current_cas, Blockmodes &mod);
    void gotoerror(std::string &current_phas, int &current_cas, Blockmodes &mod, Blockstatus &statu);
};

#endif //taskmanager_H
