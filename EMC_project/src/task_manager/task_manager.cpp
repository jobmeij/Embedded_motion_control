#include "task_manager.h"
#include "../config.h"
#include <fstream>
#include <cassert>
#include <iostream>
#include <string.h>
#include "./json.hpp"
#include "./tasks.h"
#include <emc/io.h>


//function to initialize the task manger

void TaskManager::init()
{

    //input list of cabinets from user
    std::vector<int>cabinetlist = {1, 2, 3, 5};

    //send number of cabinets to visit and the cabinet list to world mode
    worldmodel->addblock(Block::PathPlanner, Block::MODE_INIT, Block::STATE_DONE);
    worldmodel->addblock(Block::Perceptor, Block::MODE_INIT, Block::STATE_DONE);
    worldmodel->addblock(Block::DriveController, Block::MODE_INIT, Block::STATE_DONE);

    worldmodel->setcabinetlist(cabinetlist);
}


//function to execute the task manger
void TaskManager::execute()
{
    //get block statuses

    //Blockstatus state;
    status.PCstatus = worldmodel->getstatus(Block::Perceptor);
    status.DCstatus = worldmodel->getstatus(Block::DriveController);
    status.PPstatus = worldmodel->getstatus(Block::PathPlanner);

    modes.PCmode = worldmodel->getmode(Block::Perceptor);
    modes.DCmode = worldmodel->getmode(Block::DriveController);
    modes.PPmode = worldmodel->getmode(Block::PathPlanner);

    //get current phase and case


    current_phase = worldmodel->getcurrentphase();
    current_case = worldmodel->getcurrentcase();
    //std::cout<<"lol"<<std::endl;

    //functions
    //std::cout<<"DCmode : "<<modes.DCmode<<" "<<"PCmode :"<<modes.PCmode<<" "<<"PPmode :"<<modes.PPmode<<" "<<"DCstatus :"<<status.DCstatus<<" "<<"PCstatus :"<<status.PCstatus<<" "<<"PPstatus :"<<status.PPstatus<<" "<<current_phase<<" "<<current_case<<std::endl;
    check_exitconditions(current_phase, current_case, status, modes);
    change_blockmodes(current_phase, current_case, modes);
    //std::cout<<"DCmode : "<<modes.DCmode<<" "<<"PCmode :"<<modes.PCmode<<" "<<"PPmode :"<<modes.PPmode<<std::endl;
    //setmodes to world model

    worldmodel->setmode(Block::Perceptor,modes.PCmode);
    worldmodel->setmode(Block::PathPlanner,modes.PPmode);
    worldmodel->setmode(Block::DriveController,modes.DCmode);

    //send current phase and case to world model

    worldmodel->setcurrentphase(current_phase);
    worldmodel->setcurrentcase(current_case);

}

//function to set modes of the blocks

void TaskManager::change_blockmodes(std::string &curr_phase, int &curr_case, Blockmodes &mode)
{

//CHANGED BLOCK MODE OF PERCEPTOR FROM IDLE TO EXECUTE!!

    if(curr_phase=="INIT")
    {

        switch(curr_case)
        {

        case 1: {
                    mode.PCmode=Block::MODE_INIT;
                    mode.PPmode=Block::MODE_INIT;
                    mode.DCmode=Block::MODE_INIT;
                    std::cout<<"case1init"<<std::endl;
                    break;
                }
        default:    std::cout << "default\n";
                break;
    }

                            
        }



    else if(curr_phase=="FITTING")
    {
        switch(curr_case)
        {

        case 1: {
            mode.PCmode=Block::PC_MODE_FIT;
            mode.PPmode=Block::MODE_IDLE;
            mode.DCmode=Block::MODE_IDLE;
                    break;
                }


        case 2: {
            mode.PCmode=Block::MODE_EXECUTE;
            mode.PPmode=Block::PP_MODE_ROTATE;
            mode.DCmode=Block::MODE_EXECUTE;
                     break;
                }

        case 3: {
            mode.PCmode=Block::PC_MODE_FIT;
            mode.PPmode=Block::MODE_IDLE;
            mode.DCmode=Block::MODE_EXECUTE;
                    break;
                }

        case 4: {
            mode.PCmode=Block::MODE_EXECUTE;
            mode.PPmode=Block::PP_MODE_MIDDLEROOM;
            mode.DCmode=Block::MODE_IDLE;
                    break;
                }
        default:    std::cout << "default\n";
                    break;
        }


    }




    else if(curr_phase=="RELOCATE")
    {
        switch(curr_case)
        {

        case 1: {
            mode.PCmode=Block::MODE_EXECUTE;
            mode.PPmode=Block::PP_MODE_PLAN;
            mode.DCmode=Block::MODE_IDLE;
                    break;
                }

        case 2: {
            mode.PCmode=Block::MODE_EXECUTE;
            mode.PPmode=Block::PP_MODE_NEXTNODE;
            mode.DCmode=Block::MODE_IDLE;
                    break;
                }

        case 3: {
            mode.PCmode=Block::MODE_EXECUTE;
            mode.PPmode=Block::MODE_IDLE;
            mode.DCmode=Block::MODE_EXECUTE;
                    break;
                }

        case 4: {
            mode.PCmode=Block::MODE_EXECUTE;
            mode.PPmode=Block::PP_MODE_BREAKLINKS;
            mode.DCmode=Block::MODE_IDLE;
                   break;
                }

        case 5: {
            mode.PCmode=Block::MODE_EXECUTE;
            mode.PPmode=Block::PP_MODE_RESETLINKS;
            mode.DCmode=Block::MODE_IDLE;
                    break;
                }
        default:    std::cout << "default\n";
                    break;
        }

    }



    else if(curr_phase=="ACTION")
    {
        switch(curr_case)
                {

        case 1: {
            mode.PCmode=Block::MODE_EXECUTE;
            mode.PPmode=Block::MODE_IDLE;
            mode.DCmode=Block::MODE_IDLE;
                    break;
                }

        case 2: {
            mode.PCmode=Block::MODE_EXECUTE;
            mode.PPmode=Block::MODE_IDLE;
            mode.DCmode=Block::MODE_IDLE;
                    break;
                }
        default:    std::cout << "default\n";
                    break;

                }

    }



}


//function to check exit conditions and make transitions with actions.

void TaskManager::check_exitconditions(std::string &curr_phase, int &curr_case, Blockstatus &s, Blockmodes &mode)
{

    if(curr_phase=="INIT")
    {
        switch(curr_case)
        {

        case 1: {
                    if((s.PCstatus==Block::STATE_DONE)
                            &&(s.PPstatus==Block::STATE_DONE)
                            &&(s.DCstatus==Block::STATE_DONE))
                    {
                        curr_phase="FITTING";
                        curr_case=1;
                        fit_cycle_counter=0;
                        nr_rotate=0;
                        if (TM_DEBUG){std::cout << "Task Manager debuging enabled " << std::endl;}
                    }

                    else
                    {
                       gotoerror(curr_phase, curr_case, mode, s);
                    }
                            break;
                }
        default:    std::cout << "default\n";
                    break;
        }

    }



    else if(curr_phase=="FITTING"){

        switch(curr_case)
        {

        case 1: {

          if((s.PCstatus==Block::STATE_BUSY)
                  &&(fit_cycle_counter < max_fit_cycles))

          {

              fit_cycle_counter++;
              curr_phase="FITTING";
              curr_case=1;
          }
          else if((s.PCstatus==Block::STATE_BUSY)
                   &&(fit_cycle_counter == max_fit_cycles))
          {
              curr_phase="FITTING";
              curr_case=2;
              if (TM_DEBUG){std::cout << "Task Manager debuging enabled from FITTING case1 to FITTING case2" << std::endl;}
          }
          else if((s.PCstatus==Block::STATE_BUSY)
                   &&(fit_cycle_counter == max_fit_cycles)
                   &&(nr_rotate == max_rotate_cycles))
          {
              curr_phase="FITTING";
              curr_case=4;
              if (TM_DEBUG){std::cout << "Task Manager debuging enabled from FITTING case1 to FITTING case4 " << std::endl;}
            }
          else if((s.PCstatus==Block::STATE_DONE))
          {
                //getting list of cabinets
                cabinets = worldmodel->getcabinetlist();
                //update cabinet to visit
                cabinet_to_visit=cabinets.front();
                worldmodel->set_cabinet_to_visit(cabinet_to_visit);
                curr_phase="RELOCATE";
                curr_case=1;
                if (TM_DEBUG){std::cout << "Task Manager debuging enabled from FITTING case1 to RELOCATE case1  " << std::endl;}
          }
          else
          {
                gotoerror(curr_phase, curr_case, mode, s);

          }
          break;
                }


        case 2: {
            if((s.PPstatus==Block::STATE_DONE))
            {
                nr_rotate++;
                curr_phase="FITTING";
                curr_case=3;
                if (TM_DEBUG){std::cout << "Task Manager debuging enabled from FITTING case2 to FITTING case3 " << std::endl;}
            }
            else
            {
                gotoerror(curr_phase, curr_case, mode, s);

            }

                break;
                }

        case 3: {
            if((s.DCstatus==Block::STATE_DONE))
            {

                fit_cycle_counter=0;
                curr_phase="FITTING";
                curr_case=1;
                if (TM_DEBUG){std::cout << "Task Manager debuging enabled from FITTING case3 to FITTING case1 " << std::endl;}
            }
            else if((s.DCstatus==Block::STATE_BUSY))
            {
                curr_phase="FITTING";
                curr_case=3;

            }
            else
            {
               gotoerror(curr_phase, curr_case, mode, s);

            }
                break;
                }

        case 4: {
            if((s.PPstatus==Block::STATE_DONE))
            {
                nr_rotate=0;
                curr_phase="FITTING";
                curr_case=3;
                if (TM_DEBUG){std::cout << "Task Manager debuging enabled from FITTING case4 to case3" << std::endl;}
            }
            else
            {
                gotoerror(curr_phase, curr_case, mode, s);
            }

                break;
                }
        default:    std::cout << "default\n";
                    break;
            }
    }



    else if(curr_phase=="RELOCATE"){

        switch(curr_case)
        {

        case 1: {

                if((s.PPstatus==Block::STATE_DONE))
                {

                    curr_phase="RELOCATE";
                    curr_case=2;
                    if (TM_DEBUG){std::cout << "Task Manager debuging enabled from RELOCATE case1 to RELOCATE case 2 " << std::endl;}
                }
                else if(s.PPstatus==Block::STATE_ERROR)
                {
                    curr_phase="RELOCATE";
                    curr_case=5;
                    if (TM_DEBUG){std::cout << "Task Manager debuging enabled from RELOCATE case1 to RELOCATE case5" << std::endl;}
                }
                else
                {
                   gotoerror(curr_phase, curr_case, mode, s);

                }break;
                }

        case 2: {
                if((s.PPstatus==Block::STATE_DONE))
                {
                    curr_phase="ACTION";
                    curr_case=1;

                if (TM_DEBUG){std::cout << "Task Manager debuging enabled from RELOCATE case2 to ACTION case1" << std::endl;}

                }
                else if(s.PPstatus==Block::STATE_BUSY)
                {
                    curr_phase="RELOCATE";
                    curr_case=3;
                    if (TM_DEBUG){std::cout << "Task Manager debuging enabled from RELOCATE case2 to RELOCATE case3" << std::endl;}
                }
                else
                {
                   gotoerror(curr_phase, curr_case, mode, s);
    
                }break;
                }

        case 3: {
                if((s.DCstatus==Block::STATE_DONE))
                {

                    curr_phase="RELOCATE";
                    curr_case=2;
                    if (TM_DEBUG){std::cout << "Task Manager debuging enabled from RELOCATE case3 to RELOCATE case2" << std::endl;}
                }
                else if(s.DCstatus==Block::STATE_ERROR)
                {
                    curr_phase="RELOCATE";
                    curr_case=4;
                    if (TM_DEBUG){std::cout << "Task Manager debuging enabled from RELOCATE case3 to RELOCATe case4" << std::endl;}
                }
                else if(s.DCstatus==Block::STATE_BUSY)
                {
                    curr_phase="RELOCATE";
                    curr_case=3;
                }
                else
                {
                  gotoerror(curr_phase, curr_case, mode, s);
                }break;
                }

        case 4: {
                if((s.PPstatus==Block::STATE_DONE))
                {

                    curr_phase="RELOCATE";
                    curr_case=2;
                    if (TM_DEBUG){std::cout << "Task Manager debuging enabled from RELOCATE case4 to RELOCATE case2" << std::endl;}
                }

                else
                {
                   gotoerror(curr_phase, curr_case, mode, s);
    
                }break;
                }

        case 5: {
                if((s.DCstatus==Block::STATE_DONE))
                {

                    curr_phase="RELOCATE";
                    curr_case=2;
                    if (TM_DEBUG){std::cout << "Task Manager debuging enabled from RELOCATE case5 to case2" << std::endl;}
                }
                else if(s.DCstatus==Block::STATE_ERROR)
                {
                    curr_phase="RELOCATE";
                    curr_case=4;
                    if (TM_DEBUG){std::cout << "Task Manager debuging enabled from RELOCATE case5 to case4" << std::endl;}
                }
                else
                {
                   gotoerror(curr_phase, curr_case, mode, s);

                }break;
                }
                default:    std::cout << "default\n";
                break;
        }
    }

    else if(curr_phase=="ACTION")
    {
            //getting the number of cabinets to visit from worldmodel
            nr_cabinets_to_visit = worldmodel->getnumberofcabinets();


            switch(curr_case)
                    {

            case 1: {

                // add a store lazer data function

                if((s.PPstatus==Block::STATE_DONE))
                {
                    objectivecnt++;
                    curr_phase="ACTION";
                    curr_case=2;

                    //remove the visited cabinet from cabinet list
                    cabinets.erase(cabinets.begin());
                    //set new list of cabinets to world model
                    worldmodel->setcabinetlist(cabinets);
                    if (TM_DEBUG){std::cout << "Task Manager debuging enabled from ACTION case1 to ACTION case2" << std::endl;}

                }

                else
                {
                    gotoerror(curr_phase, curr_case, mode, s);
                }break;
                    }

            case 2: {


                if( nr_cabinets_to_visit>0)
                {
                    //getting list of cabinets
                    cabinets = worldmodel->getcabinetlist();
                    //update cabinet to visit
                    cabinet_to_visit=cabinets.front();
                    worldmodel->set_cabinet_to_visit(cabinet_to_visit);
                    curr_phase="RELOCATE";
                    curr_case=1;
                    if (TM_DEBUG){std::cout << "Task Manager debuging enabled from ACTION case1 to RELOCATE case1" << std::endl;}
                }
                else if( nr_cabinets_to_visit==0)
                {

                    std::cout<<"Task accomplished"<<std::endl;
                    curr_phase="ERROR";
                    curr_case=1;
                }
                else
                {
                    gotoerror(curr_phase, curr_case, mode, s);
                }break;
                    }
            default:    std::cout << "default\n";
                        break;

                    }



                }
            

            //case "ERROR"

    else if(curr_phase=="ERROR")
    {
        switch(curr_case)
                {

        case 1: {


                    break;
                }
        default:    std::cout << "default\n";
                    break;
        }
    }
            

}

void TaskManager::gotoerror(std::string &curr_phase, int &curr_case,  Blockmodes &mode, Blockstatus &state)
{
    worldmodel->setmode(Block::Perceptor,Block::MODE_IDLE);
    worldmodel->setmode(Block::Perceptor,Block::MODE_IDLE);
    worldmodel->setmode(Block::Perceptor,Block::MODE_IDLE);
    std::cout<<"Pathplanner mode"<<mode.PPmode<<std::endl;
    std::cout<<"Perceptor mode"<<mode.PCmode<<std::endl;
    std::cout<<"Drivecontroller mode"<<mode.DCmode<<std::endl;
    std::cout<<"Pathplanner status"<<state.PPstatus<<std::endl;
    std::cout<<"Perceptor status"<<state.PCstatus<<std::endl;
    std::cout<<"Drivecontroller status"<<state.DCstatus<<std::endl;
    std::cout<<"Current Phase"<<curr_phase<<" "<<"Current Case"<<curr_case<<std::endl;
    std::cout<<"Exit the program"<<std::endl;
    curr_phase="ERROR";
    curr_case=1;

}
