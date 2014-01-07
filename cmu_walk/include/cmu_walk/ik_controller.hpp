/*
 * =====================================================================================
 *
 *       Filename:  ik_controller.hpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  10/02/2013 04:19:56 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __IK_CONTROLLER_H
#define __IK_CONTROLLER_H

#include "RobotState.h"
#include "IKcmd.h"
#include "Command.h"
#include <eigen3/Eigen/Core>
#include "Logger.h"

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <algorithm> 
#include <boost/thread.hpp>

class IkController 
{
  public:
    RobotState *ikrs;

    RobotState::ModelType _type;
    static const int MAX_ROWS = 400;
    static const int MAX_VARS = MAX_N_SDFAST_U;
    Eigen::Matrix<double,MAX_ROWS,MAX_VARS> _A;
    Eigen::Matrix<double,MAX_ROWS,1> _b;
    
    Eigen::Matrix<double,MAX_N_SDFAST_U,1> _qdOld;

    bool forceMask[28];
    double forceQ[28];

    //adjustable weights
    double foot_q_weight;
    double hand_weight_p;
    double hand_weight_r_axial;
    double hand_weight_r_yawish;
    double hand_weight_r_pitchish;
    double torsoWeight;

    double hardenAxialThresh;
    double hardenAxialWeight;

    //-1 means all is well - otherwise, returns the first joint to be out of bounds
    int outOfLimits;

    //keeps track of iterations in iterative constraint method
    int computeCount;
    
    std::map <const std::string, double *> lookup;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    double IK_FOOT_WEIGHT;
    double IK_HAND_WEIGHT_P;
    double IK_HAND_WEIGHT_R;
    double IK_COM_WEIGHT;
    double IK_COMxyz_WEIGHT[3];
    double IK_PELVIS_WEIGHT;
    double IK_TORSO_WEIGHT;
    double IK_REG_WEIGHT;
    double IK_REF_WEIGHT;
    double IK_DQDOT_WEIGHT;

    double IK_REF_GAIN;
    
    double IK_COM_RATE;
    double IK_POS_RATE;
    double posRatePelv, posRateTorso, posRateCOM;
    double QPval;
    double torsoPitchLimit[2];
    bool hardenTorso;
    
    double elbowThresh;
    double elbowWeight;

    virtual ~IkController() {;}
    
    virtual bool IK(const IKcmd &cmd, double theta_d[N_JOINTS], double thetad_d[N_JOINTS]) =0;
    virtual bool IK(const IKcmd &cmd, Command &output) =0;
    virtual bool setToRobotState(const RobotState &rs) =0;
    virtual bool setToRobotStateLeg(const RobotState &rs, int side) =0;
    virtual void addToLog(BatchLogger &logger) =0;
    
    IkController() : 
      hardenAxialThresh(100),
      hardenAxialWeight(0),
      IK_FOOT_WEIGHT(1.0),
      IK_HAND_WEIGHT_P(1.0),
      IK_HAND_WEIGHT_R(1.0),
      IK_COM_WEIGHT(1.0),
      IK_PELVIS_WEIGHT(1.0),
      IK_TORSO_WEIGHT(1.0),
      IK_REG_WEIGHT(1.0),
      IK_REF_WEIGHT(0.01),
      IK_DQDOT_WEIGHT(0),

      IK_REF_GAIN(10.0),

      IK_COM_RATE(10.0),
      IK_POS_RATE(10.0),
      posRatePelv(-1),
      posRateTorso(-1),
      posRateCOM(-1),
      QPval(0.0),
      elbowThresh(0.0),
      elbowWeight(0.0)
    { 
      restoreDefaults();
      torsoPitchLimit[0] = -4;
      torsoPitchLimit[1] = 4;
      hardenTorso = false;

      for(int i = 0; i < 3; i++)	IK_COMxyz_WEIGHT[i] = 1.0;
 
      lookup["IK_FOOT_WEIGHT"] = &IK_FOOT_WEIGHT;
      lookup["IK_HAND_WEIGHT_P"] = &IK_HAND_WEIGHT_P;
      lookup["IK_HAND_WEIGHT_R"] = &IK_HAND_WEIGHT_R;
      lookup["IK_COM_WEIGHT"] = &IK_COM_WEIGHT;
      lookup["IK_COMX_WEIGHT"] = &(IK_COMxyz_WEIGHT[XX]);
      lookup["IK_COMY_WEIGHT"] = &(IK_COMxyz_WEIGHT[YY]);
      lookup["IK_COMZ_WEIGHT"] = &(IK_COMxyz_WEIGHT[ZZ]);
      lookup["IK_PELVIS_WEIGHT"] = &IK_PELVIS_WEIGHT;
      lookup["IK_TORSO_WEIGHT"] = &IK_TORSO_WEIGHT;
      lookup["IK_REG_WEIGHT"] = &IK_REG_WEIGHT;
      lookup["IK_REF_WEIGHT"] = &IK_REF_WEIGHT;
      lookup["IK_DQDOT_WEIGHT"] = &IK_DQDOT_WEIGHT;
      lookup["IK_REF_GAIN"] = &IK_REF_GAIN;
      lookup["IK_POS_RATE"] = &IK_POS_RATE;
      lookup["IK_POS_RATE_PELVIS"] = &posRatePelv;
      lookup["IK_POS_RATE_TORSO"] = &posRateTorso;
      lookup["IK_POS_RATE_COM"] = &posRateCOM;
      lookup["IK_COM_RATE"] = &IK_COM_RATE;
      lookup["IK_TORSO_PITCH_MIN"] = &(torsoPitchLimit[0]);
      lookup["IK_TORSO_PITCH_MAX"] = &(torsoPitchLimit[1]);

      _qdOld.setZero();
    }

    void restoreDefaults() {
    	hand_weight_p = IK_HAND_WEIGHT_P;
    	for(int i = 0; i < 28; i++) {
    		forceMask[i] = false;
    	}
    	hand_weight_r_axial = IK_HAND_WEIGHT_R;
    	hand_weight_r_yawish = IK_HAND_WEIGHT_R;
    	hand_weight_r_pitchish = IK_HAND_WEIGHT_R;
    	torsoWeight = IK_TORSO_WEIGHT;
    }

    bool readParams(std::ifstream &in)
    {
      if (!in.good())
        return false;

      bool ret = true;
      std::string name;
      std::map<const std::string, double *>::iterator res;
      double val;
      while (true)
      {
        in >> name;
        if (in.eof())
          break;

        res = lookup.find(name);
        // can't find item
        if (res == lookup.end()) {
          std::cerr << "cant find param: " << name << std::endl;
          ret = false;
          break;
        }

        in >> val; 
        *(res->second) = val;
        //std::cerr << "read " << name << " = " << val << std::endl;
      }

      printParams();
      foot_q_weight = IK_FOOT_WEIGHT;
   
      return ret;
    }

    void printParams() 
    {
      std::cout << "===================================\n" << "ikCon params\n";
      for (std::map<const std::string, double *>::iterator it = lookup.begin(); it != lookup.end(); it++)
        std::cout << it->first << ": " << *(it->second) << std::endl;
      std::cout << "===================================\n"; 
    }
};

/////////////////////////////////////////////////////////////
// eric land
class PelvIkCon : public IkController 
{
  public:
    PelvIkCon() 
    { 
      _type = RobotState::TYPE_PELVIS; 
      ikrs = new PelvRobotState();
    }
    ~PelvIkCon() { delete ikrs; }

    bool IK(const IKcmd &cmd, double *theta_d=NULL, double *thetad_d=NULL);
    bool IK(const IKcmd &cmd, Command &output) { fprintf(stderr, "IK unimplemented.\n"); return false; }
    bool setToRobotState(const RobotState &rs);
    bool setToRobotStateLeg(const RobotState &rs, int side) { fprintf(stderr, "IK unimplemented.\n"); return false; }
    void setToRefPose();
    void setPose(const double *theta, const double *thetad);
    void setRoot(const double *root, Eigen::Quaterniond rootQ, const double *rootd=NULL, const double *rootW=NULL);
    void matchRoot(const RobotState &rs);
    void getCommand(double *theta_d, double *thetad_d);
    void addToLog(BatchLogger &logger);
};

/////////////////////////////////////////////////////////////
// siyuan land
class SFIkCon : public IkController 
{
  public:
    ~SFIkCon() { delete ikrs; }

    bool IK(const IKcmd &cmd, double *theta_d=NULL, double *thetad_d=NULL) { fprintf(stderr, "IK unimplemented.\n"); return false; }

    bool setToRobotState(const RobotState &rs_real)
    {
      ikrs->time = rs_real.time;
      ikrs->timeStep = rs_real.timeStep;
      ikrs->computeSDFvars(rs_real.root, rs_real.rootq.coeffs().data(), rs_real.rootd, rs_real.root_b_w, rs_real.joints, rs_real.jointsd); 
      return true;   
    }

    bool setToRobotStateLeg(const RobotState &rs_real, int side)
    {
      if (side == LEFT) {
        for (int i = A_L_LEG_UHZ; i <= A_L_LEG_LAX; i++) {
          ikrs->joints[i] = rs_real.joints[i];
          ikrs->jointsd[i] = 0;
        }
        ikrs->computeSDFvars();
        return true; 
      }
      else if (side == RIGHT) {
        for (int i = A_R_LEG_UHZ; i <= A_R_LEG_LAX; i++) {
          ikrs->joints[i] = rs_real.joints[i];
          ikrs->jointsd[i] = 0;
        }
        ikrs->computeSDFvars();
        return true; 
      }
      else 
        return false; 
    }

    virtual void addToLog(BatchLogger &logger) =0;
};

class SFPelvIkCon : public SFIkCon
{
  public:
    SFPelvIkCon() 
    { 
      _type = RobotState::TYPE_PELVIS; 
      ikrs = new PelvRobotState();
    }

    bool IK(const IKcmd &cmd, Command &output);
    void addToLog(BatchLogger &logger);
};

class HatIkCon : public SFIkCon
{
  public:
    HatIkCon() 
    { 
      _type = RobotState::TYPE_HAT; 
      ikrs = new HatRobotState();
    }

    bool IK(const IKcmd &cmd, Command &output);
    void addToLog(BatchLogger &logger);
};

class HatNSIkCon : public SFIkCon
{
  public:
    HatNSIkCon() 
    { 
      _type = RobotState::TYPE_HAT_NS; 
      ikrs = new HatNSRobotState();
    }

    bool IK(const IKcmd &cmd, Command &output);
    void addToLog(BatchLogger &logger);
};

#endif
