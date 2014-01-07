/*
 * =====================================================================================
 *
 *       Filename:  WalkingCon.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/02/2013 05:19:23 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#include "WalkingCon.h"

const std::string WalkingCon::TerrainModeNames[NUM_TERRAIN_MODE] = 
{"Flat", 
  "Up block", 
  "Down block", 
  "Up slope", 
  "Down slope",
  "TiltUp block",
  "TiltDown block",
  "TiltLeft block",
  "TiltRight block"}; 

void WalkingCon::splitCoP(const double CoP[2], double wl, const double Pl[2], const double Pr[2], double CoPl[2], double CoPr[2]) 
{
  //wl*(Pl[X]+CoPl[X]) + (1-wl)*(Pr[X]+CoPr[X]) = CoP[X]
  //CoPl[X] = CoPr[X] = px
  //px = CoP[X] - wl*Pl[X] - (1-wl)*Pr[X]
    
  CoPl[XX] = CoPr[XX] = CoP[XX] - wl*Pl[XX] - (1-wl)*Pr[XX];
  CoPl[YY] = CoPr[YY] = CoP[YY] - wl*Pl[YY] - (1-wl)*Pr[YY];
}
 
bool WalkingCon::readParams(std::ifstream &in)
{
  std::map<const std::string, double *>::iterator res;
  std::string name;
  double val;
  bool ret = true;

  while (true) 
  {
    in >> name;
    if (in.eof())
      break;

    res = _lookup.find(name);
    // can't find item
    if (res == _lookup.end()) {
      std::cerr << _conName << " unknown param: " << name << " aborting." << std::endl;
      ret = false;
      break;
    }

    in >> val; 
    *(res->second) = val;
    std::cout << _conName << " read " << name << " = " << val << std::endl;
  }

  return ret; 
}

void WalkingCon::printParams()
{
  std::cout << "===================================\n" 
    << _conName << " params:\n";
  for (std::map<const std::string, double *>::iterator it = _lookup.begin(); it != _lookup.end(); it++)
    std::cout << it->first << ": " << *(it->second) << std::endl;
  std::cout << "===================================\n";   
}
