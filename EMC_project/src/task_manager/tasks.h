#pragma once
#include <iostream>
#include <string>

struct Phase{
    string Phasename_;
    string currentphase_;
    string nextphase_;
    Phase(string Phasename, string currentphase, string nextphase) : Phasename_(Phasename), currentphase_(currentphase), nextphase_(nextphase)
    {
    }
};

/*struct tasks
{

    std::string phases_;
    std::string cases_;
    std::string excond_;
    std::string transto_;
    std::string transact_;


  tasks(string phases, string cases, string excond, string transto, string transact) : phases_(phases), cases_(cases), excond_(excond), transto_(transto), transact_(transact)
  {
  }
};
*/
