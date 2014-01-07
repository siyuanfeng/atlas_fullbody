#ifndef __ROBOT_STATE_DEFINED
#define __ROBOT_STATE_DEFINED

// used to make M, J, nonLin terms for QP inverse dynamics
#include "drc_common_defines.h"
#include "SDModel.h"

#include "Foot.h"
#include "Logger.h"
#include <eigen3/Eigen/Geometry>

class RobotState {
  public:
    enum ModelType {
      TYPE_PELVIS = 0,
      TYPE_HAT,
      TYPE_HAT_NS
    };

    enum Contact {
      UP = 0,
      DOWN
    };

  protected:
    double *_sdFastState;
    ModelType _type;

  public:

    static const std::string joint_names[N_JOINTS];
    static const std::string body_names[N_BODIES];
#ifndef ATLAS_ONLINE
    static const double jointsCFMDamping[N_JOINTS];
#endif
    //static const double torqueLimit[2][N_JOINTS];     // see Transmission.cpp
    static const double jointsLimit[2][N_JOINTS];
    static const double root2tfRoot[3];

    double time;
    double ctrl_dt;
    double timeStep;

    Eigen::Quaterniond rootq;
    double root[6];   // last 3 are euler angles
    double rootd[6];  
    double rootdd[6]; 
    double root_b_w[3];    // body angular velocity
    double root_b_wd[3];   // body angular acceleration

    double tfRoot[3];   // for all the ros people

    double joints[N_JOINTS];
    double jointsd[N_JOINTS];
    double jointsdd[N_JOINTS];
    double joints_torque[N_JOINTS];

    double locations[N_LOCATIONS][XYZ];

    Foot feet[LR];
    bool footContact[LR];
    int contactCount[LR];

    double hand[LR][6], elbow[LR][6];
    Eigen::Quaterniond handQ[LR];
    Eigen::Vector3d handAxis[LR];
    Eigen::Vector3d drillAxis[LR];
    double handd[LR][6];
    double handForces[LR][6];
    double handForcesRaw[LR][6];

    // for controlling orientation
    Eigen::Quaterniond utorsoq, pelvisq;
    double utorsow[3], pelvisw[3];

    // last 3, just copy root's 
    double com[3];
    double comd[3];
    double comdd[3];

    SDModel *model;

    double comOffset[3];

    // EIGEN uses col major by default, need to explicitly say rowmajor!!!
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> J[LR];    // foot J transpose
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jd[LR];   // foot Jd transpose
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jtoe[LR];    // foot J transpose
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jtoed[LR];   // foot Jd transpose
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jheel[LR];    // foot J transpose
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jheeld[LR];   // foot Jd transpose
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jmid[LR];    // foot J transpose
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jmidd[LR];   // foot Jd transpose

    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jhand[LR];    // hand J transpose
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jhandd[LR];   // hand Jd transpose

    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jelbow[LR];    // elbow J transpose

    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jc;       // com J transpose 
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jcd;      // com Jd transpose

    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jtorso;   
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jtorsod;

    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jpelvis;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Jpelvisd;

    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> M;    // Mass
    Eigen::Matrix<double,3,3,Eigen::RowMajor> I, Itorso;                        // tot and torso moment of inertia 

    Eigen::Matrix<double,Eigen::Dynamic,1> nonLin; 

    double m;							//mass
    double mom[3]; 
    double angMom[3];			//momentum and angular momentum
    double KE;							//kinetic energy
    double cop[2];						//center of pressure - absolute coordinates

    //extra state variables
    int contactState;




    RobotState();
    virtual ~RobotState() {;}
    virtual void computeSDFvars() =0;
    virtual void addToLog(BatchLogger &logger) =0;
    virtual void makeSDFastState(const double pos[3], const Eigen::Quaterniond &q, const double vel[3], const double w[3], const double j[N_JOINTS], const double jd[N_JOINTS], double *sdfast_state) =0;
    //RobotState &operator=(const RobotState &other);

    void rotateToFootForward(int side);
    void rotateAroundFoot(int side, double yaw);
    void rotateAroundFoot(double wl, double yaw);
    void reset();
    void fillZeros();
    Eigen::Vector3d getFootCenter() const;
    Eigen::Vector3d getFootCenterLower() const;
    ModelType getType() const { return _type; }

    void checkContact();

    static double hipZlim(double hipY) {
      if(hipY > -1.4)	return -0.17;
      else if(hipY < -1.5)	return 0.1;
      else return (-hipY-1.4)*.27/0.1-0.17;
    }

    void computeSDFvars(const double pos[3], const Eigen::Quaterniond q,
        const double vel[3], const double w[3],     
        const double j[N_JOINTS], const double jd[N_JOINTS]) { computeSDFvars(pos, q.coeffs().data(), vel, w, j, jd); }

    void computeSDFvars(const double pos[3], const double q[4],   
        const double vel[3], const double w[3],     
        const double j[N_JOINTS], const double jd[N_JOINTS]);

    void computeSDFvarsFilt(const double pos[3], const Eigen::Quaterniond q,
        const double vel[3], const double w[3],
        const double j[N_JOINTS], const double jd[N_JOINTS], double filtA) { computeSDFvarsFilt(pos, q.coeffs().data(), vel, w, j, jd, filtA); }

    void computeSDFvarsFilt(const double pos[3], const double q[4],
        const double vel[3], const double w[3],
        const double j[N_JOINTS], const double jd[N_JOINTS], double filtA);

    inline const double *getSDFState() const { return _sdFastState; }

    static bool inBounds(double ang, int ind) {
      if(ang < RobotState::jointsLimit[0][ind] )		return false;
      if(ang > RobotState::jointsLimit[1][ind] )		return false;
      return true;
    }
    static double putInBounds(double ang, int ind) {
      if(ang < RobotState::jointsLimit[0][ind] )		ang = RobotState::jointsLimit[0][ind];
      if(ang > RobotState::jointsLimit[1][ind] )		ang = RobotState::jointsLimit[1][ind];
      return ang;
    }
    static double putInBoundsMargin(double ang, int ind) {
      if(ang < RobotState::jointsLimit[0][ind] )		ang = RobotState::jointsLimit[0][ind]+1e-6;
      if(ang > RobotState::jointsLimit[1][ind] )		ang = RobotState::jointsLimit[1][ind]-1e-6;
      return ang;
    }
    bool inBounds(int ind) {
      return inBounds(joints[ind],ind);
    }

};

class PelvRobotState : public RobotState 
{
  public:
    PelvRobotState();
    ~PelvRobotState();

    static void setHandDist(double d);
    static void setHandDist(double d, int side);
    void addToLog(BatchLogger &logger);
    void computeSDFvars();
    void makeSDFastState(const double pos[3], const Eigen::Quaterniond &q, const double vel[3], const double w[3], const double j[N_JOINTS], const double jd[N_JOINTS], double *sdfast_state);
};

class HatRobotState : public RobotState 
{
  public:
    HatRobotState();
    ~HatRobotState();

    // walking with arms raised
    enum UpperBodyMode {
      SF,
      EW,
      HM
    };
    void setUpperBodyMode(UpperBodyMode m);

    static void pelv2hat(const double *pelv, double *hat);
    static void hat2pelv(const double *hat, double *pelv);
    static int pelvIdx2hatIdx(int i);
    static int hatIdx2pelvIdx(int i);

    void addToLog(BatchLogger &logger);
    void computeSDFvars();
    void makeSDFastState(const double pos[3], const Eigen::Quaterniond &q, const double vel[3], const double w[3], const double j[N_JOINTS], const double jd[N_JOINTS], double *sdfast_state);  
};

class HatNSRobotState : public RobotState 
{
  public:
    HatNSRobotState();
    ~HatNSRobotState();

    static void pelv2hatNS(const double *pelv, double *hat);
    static void hatNS2pelv(const double *hat, double *pelv);
    static int pelvIdx2hatNSIdx(int i);
    static int hatNSIdx2pelvIdx(int i);

    void addToLog(BatchLogger &logger);
    void computeSDFvars();
    void makeSDFastState(const double pos[3], const Eigen::Quaterniond &q, const double vel[3], const double w[3], const double j[N_JOINTS], const double jd[N_JOINTS], double *sdfast_state);  
};

#endif
