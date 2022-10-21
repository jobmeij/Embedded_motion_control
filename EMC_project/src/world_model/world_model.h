#include "../objects.h"
#include <list>
#include <iostream>
#include <string>
#include "./json.hpp"
#ifndef world_model_H
#define world_model_H

using json = nlohmann::json;

//struct PHASECASE
//{

//};

/*struct highlevel{

    json k;

    /*string phases;
    string cases[10];
    string excond[10];
    string transto[10];
    string transact[10];
}*/




struct Block{
    enum Names { DriveController, Perceptor, PathPlanner } name;
    enum Status{ STATE_BUSY, STATE_DONE, STATE_ERROR } sts;
    enum Modes { MODE_EXECUTE, MODE_INIT, MODE_IDLE, MODE_DISABLE,
                 PP_MODE_ROTATE, PP_MODE_RESETLINKS, PP_MODE_PLAN, PP_MODE_FOLLOW, PP_MODE_MIDDLEROOM, PP_MODE_BREAKLINKS, PP_MODE_NEXTNODE,
                 PC_MODE_FIT} mode;
};

//  look for function to get an equal string for a enum
struct close_prx{
    double start_angle;
    double dt_angle;
    bool valid;
    vector<bool> toClose;
    vector<float> distances;
    close_prx(){valid = false;}
};

class WorldModel{
private:
    Map localmap;
    Map currentmap;
    Map globalmap;
    std::vector<int> list_of_cabinets;
    Position zeropos;
    Position currentpos;
    Position desiredpos;
    float curbestfit;

    vector<node> nodelist;
    vector<node> path;
    vector<link> linklist;

    int current_cabinet;

    std::vector<Block> blocks;
    int currenttask;
    //int numberofcabinets;
    //std::string current_phase="INIT";
    //int current_case="1";
    close_prx safeDir;
    std::string current_phase1;
    int current_case1;

public:

    WorldModel(){currenttask = 0;current_phase1="INIT";current_case1=1;}

    Map& getlocalmap(){return localmap;}
    Map& getcurrentmap(){return currentmap;}
    Map& getglobalmap(){return globalmap;}

    //void addlocalmapObject(Object obj){localmap.objects.push_back(obj);}
    void setlocalmap(Map localmap_){localmap = localmap_;}
    void setcurrentmap(Map currentmap_){currentmap = currentmap_;}
    void setglobalmap(Map globalmap_){globalmap = globalmap_;}
    //void clearlocalmap(){localmap.objects.clear();}

    std::string getcurrentphase(){return current_phase1;}
    int getcurrentcase(){return current_case1;}
    int getnumberofcabinets(){return list_of_cabinets.size();}
    int get_cabinet_to_visit(){return current_cabinet;}
    std::vector<int> getcabinetlist(){return list_of_cabinets;}

    int getcurrenttask(){return currenttask;}

    Position& getzeroposition(){return zeropos;}
    Position& getcurrentposition(){return currentpos;}
    Position& getdesiredposition(){return desiredpos;}

    float& getfitscore(){return curbestfit;}

    vector<node>& getnodelist(){return nodelist;}
    vector<node>& getpath(){return path;}
    vector<link>& getlinklist(){return linklist;}

    Block::Modes getmode(Block::Names name_){
        Block::Modes blockmode;
        for(unsigned int i=0; i < blocks.size();i++){
            Block currentblock = blocks[i];
            if(name_ == currentblock.name){
                blockmode = currentblock.mode;
            }
        }
        return blockmode;
    }

    Block::Status getstatus(Block::Names name_){
        Block::Status blockstatus;
        for(unsigned int i=0; i < blocks.size();i++){
            Block currentblock = blocks[i];
            if(name_ == currentblock.name){
                blockstatus = currentblock.sts;
            }
        }
        return blockstatus;
    }

    close_prx& getToClose(){return safeDir;}

    // set functions

//<<<<<<< HEAD
//    void setcurrentphase(std::string p){current_phase1=p;}
//    void setcurrentcase(int c){current_case1=c;}
//    void addlocalmapObject(Object obj){localmap.objects.push_back(obj);}
//    void setlocalmap(Map localmap_){localmap = localmap_;}
//    void setcurrentmap(Map currentmap_){currentmap = currentmap_;}
//    void setglobalmap(Map globalmap_){globalmap = globalmap_;}
//    void clearlocalmap(){localmap.objects.clear();}
//=======
    void setcurrentphase(std::string pc){current_phase1=pc;}
    void setcurrentcase(int pc){current_case1=pc;}
//>>>>>>> origin/master
    //void setnumberofcabinets(int num){numberofcabinets = num;}
    void setcabinetlist(std::vector<int> cablist){list_of_cabinets=cablist;}
    void set_cabinet_to_visit(int c){current_cabinet;}
    void setcurrenttask(int currenttask_){currenttask = currenttask_;}

    void setstatus(Block::Names name_, Block::Status status_){
        for(unsigned int i=0; i < blocks.size();i++){
            if(name_ == blocks[i].name){
                blocks[i].sts = status_;
            }
        }
    }

    void setmode(Block::Names name_, Block::Modes mode_){
        for(unsigned int i =0; i < blocks.size();i++){
            if(name_ == blocks[i].name){
                blocks[i].mode = mode_;
            }
        }
    }

    void addblock(Block::Names name_, Block::Modes mode_, Block::Status status_){
        Block newblock;
        newblock.name = name_;
        newblock.mode = mode_;
        newblock.sts = status_;
        blocks.push_back(newblock);
    }

    void setzeroposition(float x_, float y_, float a_){zeropos.x = x_;zeropos.y = y_;zeropos.a = a_;}
    void setcurrentposition(float x_, float y_, float a_){currentpos.x = x_;currentpos.y = y_;currentpos.a = a_;}
    void setdesiredposition(float x_, float y_, float a_){desiredpos.x = x_;desiredpos.y = y_;desiredpos.a = a_;}
    void setClosePrx(close_prx cl){safeDir = cl;}

};

#endif //world_model_H
