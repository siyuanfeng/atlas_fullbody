/*
 * =====================================================================================
 *
 *       Filename:  WalkingCon.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/01/2013 11:23:02 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __WALKING_CON_H
#define __WALKING_CON_H

#include "RobotState.h"
#include "IKcmd.h"
#include "IDcmd.h"
#include "Command.h"
#include "id_controller.hpp"
#include "ik_controller.hpp"
#include "Logger.h"
#include "spline.h"
#include <map>
#include <string>

class WalkingCon {
  protected:
    enum WalkingConState {
      UnInited = -1,
      Initial,
      Idle,
      Walking,
      Stopping,
      NUM_WC_STATE
    };
    
    knot_t _joints0_d[N_JOINTS];
    knot_t _joints1_d[N_JOINTS];
    double _neck_ay_d;
    
    std::string _conName;
    std::map <const std::string, double *> _lookup;

    // everything in world coordinate
    void splitCoP(const double CoP[2], double wl, const double Pl[2], const double Pr[2], double CoPl[2], double CoPr[2]);
    virtual void buildLookup()=0;

  public:
    enum FootStage {
      Planted = 0,
      LiftOff,
      Swing,
      TouchDown
    };
 
    enum WalkingConTerrainMode {
      Flat = 0,
      UpBlock,
      DownBlock,
      UpSlope,
      DownSlope,
      TiltUpBlock,
      TiltDownBlock,
      TiltLeftBlock,    // "\"
      TiltRightBlock,   // "/"
      NUM_TERRAIN_MODE
    };

    static const std::string TerrainModeNames[NUM_TERRAIN_MODE];
    
    // qp stuff
    IdController *idCon;
    IkController *ikCon;
    IKcmd ikcmd;
    IDcmd idcmd;

    WalkingCon() 
    { 
      idCon = NULL; 
      ikCon = NULL;
    }

    ~WalkingCon() 
    { 
      if (idCon) 
        delete idCon; 
      if (ikCon)
        delete ikCon;
    }

    void allocCon(RobotState::ModelType t)
    {
      if (t == RobotState::TYPE_PELVIS) {
        if (!idCon)
          idCon = new SFPelvIdCon();
        if (!ikCon)
          ikCon = new SFPelvIkCon();
      }
      else if (t == RobotState::TYPE_HAT) {
        if (!idCon)
          idCon = new HatIdCon();
        if (!ikCon)
          ikCon = new HatIkCon();
      }
      else if (t == RobotState::TYPE_HAT_NS) {
        if (!idCon)
          idCon = new HatNSIdCon();
        if (!ikCon)
          ikCon = new HatNSIkCon();
      }
    }

    void setNeckAy(double a) 
    { 
      _neck_ay_d = std::min(a, RobotState::jointsLimit[1][A_NECK_AY]); 
      _neck_ay_d = std::max(_neck_ay_d, RobotState::jointsLimit[0][A_NECK_AY]); 
    }

    void printParams();
    bool readParams(std::ifstream &in);

    virtual void init(const RobotState &rs)=0;
    virtual int control(RobotState &rs, Command &cmd)=0;
    virtual bool hasInited() const =0;
    virtual int getPlannedContactState(double time) const=0;
    virtual bool isIdle() =0;
    virtual const double *getPlannedCurRoot(const RobotState &rs) const=0;
    virtual const double *getPlannedCurLeftFoot(const RobotState &rs) const=0;
    virtual const double *getPlannedCurRightFoot(const RobotState &rs) const=0;

    virtual FootStage getPlannedFootState(const RobotState &rs, int side) const =0;
    virtual void addToLog(BatchLogger &logger) =0;
};

#endif
