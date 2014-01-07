#include <math.h>
#include <stdio.h>
#include <string>

#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <sensors/sensors.hh>
#include <math/Vector3.hh>
#include <math/Pose.hh>
#include <math/Matrix4.hh>
#include <iostream>
#include <fstream> 
#include <gsl/gsl_blas.h>
 
#include "drc_pelvis_sim_defines.h"
#include "RobotState.h"
#include "LLcon.h"
#include "traj.h"
#include "FootStep.h"
#include "spline.h"
#include "quaternion.h"
#include "Logger.h"
#include "gsl_utils.h"

#include "WalkingCon.h"
#include "DummyFSPlanner.h"
//#include "kalmanFilter.h"
#include "filters.h"
#include "data.h"
#define MRDPLOT_SAVE_TIME  30
#define TIME_STEP     0.001
#define FLOOR_HEIGHT  0

namespace gazebo
{   

inline void VectorToArray(const gazebo::math::Vector3 &V, double *v) { v[0] = V.x; v[1] = V.y; v[2] = V.z; }

static const double joint_damping[N_JOINTS] = 
{
  10.0, 10.0, 10.0, 0.1,
  10.0, 10.0, 10.0, 0.1, 0.1, 0.1,
  10.0, 10.0, 10.0, 0.1, 0.1, 0.1,
  0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
  0.1, 0.1, 0.1, 0.1, 0.1, 0.1
};
 
static const std::string gz_joint_names[N_JOINTS] =
{ 
  "back_lbz", "back_mby", "back_ubx", "neck_ay", 
  "l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax", 
  "r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax", 
  "l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx", 
  "r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx" 
};

static const std::string gz_link_names[N_BODIES] = 
{
  "pelvis", "ltorso", "mtorso", "utorso", "head", 
  "l_uglut", "l_lglut", "l_uleg", "l_lleg", "l_talus", "l_foot", 
  "r_uglut", "r_lglut", "r_uleg", "r_lleg", "r_talus", "r_foot", 
  "l_clav", "l_scap", "l_uarm", "l_larm", "l_farm", "l_hand", 
  "r_clav", "r_scap", "r_uarm", "r_larm", "r_farm", "r_hand"
};

/****************************************************************************/

class atlas_terrain : public ModelPlugin
{
  // ModelPlugin
  public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void initWeights()
    {
      wc.idCon.qddWeight = 1e-1;
      wc.idCon.comddWeight = 6e-1;
      wc.idCon.comFWeight = 1e-3;
      wc.idCon.comTauWeight = 1e-5;
      wc.idCon.footddWeight = 1e0;
      wc.idCon.contactFWeight = 1e-6;
      wc.idCon.copWeight = 2e-3;
      wc.idCon.wWeight = 1e-3;
      wc.idCon.minTauWeight = 1e-6;
      wc.idCon.DTauWeight = 1e-6;
    }

    // Initialize KF
    void init_KF(){
     // Initialize ImuKF
      {
        imuekf.timestep = rs.timeStep;
        Eigen::Matrix<double,10,1> x0;
       
        x0.setZero();
        x0.block(6,0,4,1) = Eigen::Map<const Eigen::Matrix<double, 4, 1> >(root_q);
        Eigen::Matrix<double,9,9> P0;
        Eigen::Matrix<double,9,9> Q;
        Eigen::Matrix<double,6,6> R;

        P0.setIdentity();
        std::ifstream in ("Quat_Q");
        if(in.is_open())
          in >> Q;    
        in.close(); 
        in.open ("Quat_R"); 
        if (in.is_open())
          in >> R;    
        in.close();

        imuekf.setX(x0);   
        imuekf.setP(P0);
        imuekf.setQ(Q);
        imuekf.setR(R);

      }

      // Initialize BiasKF
      {
        Eigen::Matrix<double,6,1> x0;
        x0.setZero();
        Eigen::Matrix<double,6,6> P0;
        Eigen::Matrix<double,6,6> Q;
        Eigen::Matrix<double,6,6> R;

        P0.setIdentity();
        std::ifstream in ("Bias_Q");
        if(in.is_open())
          in >> Q;    
        in.close(); 
        in.open ("Bias_R"); 
        if (in.is_open())
          in >> R;    
        in.close();

        biasekf.setX(x0);   
        biasekf.setP(P0);
        biasekf.setQ(Q);
        biasekf.setR(R);
        // Compute steady state kalman gain
        biasekf.setKss();
      } 


      // Initialize JointvFilter
      {
        jvekf.timestep = rs.timeStep; 
        Eigen::Matrix<double,N_JOINTS,1> x0;
        x0.setZero();
        Eigen::Matrix<double,N_JOINTS,N_JOINTS> P0;
        Eigen::Matrix<double,N_JOINTS,N_JOINTS> Q;
        Eigen::Matrix<double,N_JOINTS,N_JOINTS> R;

        P0.setIdentity();
        Q.setIdentity();
        R.setIdentity();
        Q *= 0.1; 

        jvekf.setX(x0);   
        jvekf.setP(P0);
        jvekf.setQ(Q);
        jvekf.setR(R);
        // Compute steady state kalman gain
        jvekf.setKss();
        //std::cout << jvekf.getK() << std::endl;
      } 
      
      // Initialize Kinematic Constraint KF
      /*{
        kcekf.timestep = rs.timeStep; 
        Eigen::Matrix<double,16,1> x0;  // [x, v, pl, pr, q]
        x0.block(0,0,3,1) = Eigen::Map<const Eigen::Matrix<double, 3, 1> >(root_pos);
        x0.block(3,0,3,1) = Eigen::Map<const Eigen::Matrix<double, 3, 1> >(root_vel);
        x0.block(6,0,3,1) = Eigen::Map<const Eigen::Matrix<double, 3, 1> >(rs.feet[LEFT].w_pos);
        x0.block(9,0,3,1) = Eigen::Map<const Eigen::Matrix<double, 3, 1> >(rs.feet[RIGHT].w_pos);
        x0.block(12,0,4,1) = Eigen::Map<const Eigen::Matrix<double, 4, 1> >(root_q);
        Eigen::Matrix<double,15,15> P0;
        P0.setIdentity(); 
        kcekf.setX(x0);   
        kcekf.setP(P0);

        std::ifstream in ("DSc_Q");
        if(in.is_open())
          in >> kcekf.DSc_Q;    
        in.close(); 
        in.open ("SSL_Q"); 
        if (in.is_open())
          in >> kcekf.SSL_Q;    
        in.close();                  
        in.open ("SSR_Q"); 
        if (in.is_open())
          in >> kcekf.SSR_Q;    
        in.close();     
        in.open ("DSc_R"); 
        if (in.is_open())
          in >> kcekf.DSc_R;    
        in.close();     
        in.open ("SS_R"); 
        if (in.is_open())
          in >> kcekf.SS_R;    
        in.close();     

        printf("Kinematic KF initialized\n\n\n\n\n\n");
        // Compute steady state kalman gain
        // kcekf.setKss();
        //std::cout << jvekf.getK() << std::endl;
      }*/

      // Initialize Kinematic Constraint KF  (Velocity only)
      {
        kcekf.timestep = rs.timeStep; 
        Eigen::Matrix<double,3,1> x0;  // [x, v, pl, pr, q]
        kcekf._pos = Eigen::Map<const Eigen::Matrix<double, 3, 1> >(root_pos); // Position
        x0 = Eigen::Map<const Eigen::Matrix<double, 3, 1> >(root_vel); // Velocity
        
        Eigen::Matrix<double,3,3> P0;
        P0.setIdentity(); 
        kcekf.setX(x0);   
        kcekf.setP(P0);

        std::ifstream in ("DSc_Q");
        if(in.is_open())
          in >> kcekf.DSc_Q;    
        in.close(); 
        in.open ("SSL_Q"); 
        if (in.is_open())
          in >> kcekf.SSL_Q;    
        in.close();                  
        in.open ("SSR_Q"); 
        if (in.is_open())
          in >> kcekf.SSR_Q;    
        in.close();     
        in.open ("DSc_R"); 
        if (in.is_open())
          in >> kcekf.DSc_R;    
        in.close();     
        in.open ("SS_R"); 
        if (in.is_open())
          in >> kcekf.SS_R;    
        in.close(); 
        printf("Kinematic KF initialized\n\n\n\n\n\n\n\n\n\n\n\n\n");
        // Parameter used to scale innovation
        in.open ("alpha"); 
        if (in.is_open())
          in >> kcekf._alpha;    
        in.close();
        printf("Alpha = %g\n", kcekf._alpha);
         

        printf("Kinematic KF initialized\n\n\n\n\n\n");
        // Compute steady state kalman gain
        // kcekf.setKss();
        //std::cout << jvekf.getK() << std::endl;
      } 
    }


    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      int i;
      
      step_ctr = 0;
      ctr = 0;
      phase = 0;

      //BiasFilter bf;
      
      //Eigen::Matrix<double,6,1> x;
      //x = bf.getX();
      //std::cout << x << std::endl;
      //getchar();
      // Store the pointer to the model
      this->model = _parent;
      this->state = 0;

      // this->startupTime = common::Time::GetWallTime();
      this->startupTime = this->model->GetWorld()->GetSimTime();
      this->last_action_time = 0;

      // set up joint ptr
      physics::JointPtr jtmp;
      for ( i = 0; i < N_JOINTS; i++ )
      {
        jtmp = this->model->GetJoint(gz_joint_names[i]);
        if (jtmp)
          jointsPtr.push_back(jtmp);
        cmd.joints_d[i] = 0.0;
        cmd.jointsd_d[i] = 0.0;
      }

      // set up link ptr
      physics::LinkPtr ltmp;
      for ( i = 0; i < N_BODIES; i++ ) {
        ltmp = this->model->GetLink(gz_link_names[i]);
        if (ltmp)
          linksPtr.push_back(ltmp);
        // std::cout << gz_link_names[i] << " " << ltmp->GetInertial()->GetMass() << std::endl;
      }
      
      // lower arm 
      cmd.joints_d[ PELV_A_L_ARM_SHX ] = -1.3; // 1.39 joint limit
      cmd.joints_d[ PELV_A_R_ARM_SHX ] = 1.3;
      jointsPtr[PELV_A_L_ARM_SHX]->SetAngle(0, math::Angle(-1.3));
      jointsPtr[PELV_A_R_ARM_SHX]->SetAngle(0, math::Angle(1.3));
      jointsPtr[PELV_A_L_ARM_SHX]->Update();
      jointsPtr[PELV_A_R_ARM_SHX]->Update();
      
      forwardKinematics();
      updateRobotState(0);
      
      // Load parameters for Sensors
      LoadSensorParams(); 
      // Data collection
      this->mrdptr = &(this->mrd_data); 
      init_data(this->mrdptr);
      // Initialize KF
      init_KF();    

      wc.start(rs);
      initWeights();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart( boost::bind(&atlas_terrain::OnUpdate, this ));  
    }
    /****************************************************************************/
    /****************************************************************************/
    void LoadSensorParams() 
    {
      // Right Foot Contact
      this->rAnkleJoint = this->model->GetJoint("r_leg_lax");
      if (!this->rAnkleJoint)
        gzerr << "right ankle joint (r_leg_lax) not found\n";

      // Left Foot Contact
      this->lAnkleJoint = this->model->GetJoint("l_leg_lax");
      if (!this->lAnkleJoint)
        gzerr << "left ankle joint (l_leg_lax) not found\n";

      // IMU
      // the parent link of the imu_sensor ends up being pelvis after fixed joint reduction.  Offset of the imu_link is lumped into
      // the <pose> tag in the imu_senosr block.
      this->imuLinkName = "pelvis";
      this->imuLink = this->model->GetLink(this->imuLinkName);
      if (!this->imuLink)
        gzerr << this->imuLinkName << " not found\n";
      this->lastImuTime = startupTime.Double();
      this->imuSensor = boost::shared_dynamic_cast<sensors::ImuSensor> (sensors::SensorManager::Instance()->GetSensor(
            this->model->GetWorld()->GetName() + "::" + this->model->GetScopedName()+"::pelvis::" "imu_sensor"));
      if (!this->imuSensor)
        gzerr << "imu_sensor not found\n" << "\n";
    }



    /****************************************************************************/
    /****************************************************************************/

    void ReadSensors()
    {
      int i;
      // Read Joint Position Velocity
      for (int i = 0; i < N_JOINTS; i++)
      {
        joints[i] = this->jointsPtr[i]->GetAngle(0).Radian();
        jointsd[i] = this->jointsPtr[i]->GetVelocity(0);
      }
               
      ReadImu2(imu_angular_velocity, imu_linear_acceleration, imu_orientation);          // Read Left Foot Contact information
      ReadContactLeft(lcontact_force, lcontact_torque);
      // Read Right Foot Contact information
      ReadContactRight(rcontact_force, rcontact_torque);
      // printf ("a = [%g, %g, %g]\n", imu_linear_acceleration[0],imu_linear_acceleration[1], imu_linear_acceleration[2]);

    }
    /****************************************************************************/
    // Called by the world update start event
    void OnUpdate()
    {
      //static int state = 0;
      state = 0;
      static bool planFlag = true;

      if (ctr == 2)
        getchar();

      double qp_acc[PELV_N_SDFAST_U];
      double qp_grf[12];
      double x;

      // update RobotState
      ReadSensors();
      //getchar();
      copy_mrdplot_data(); 
      save_data();

      if (rs.root[ZZ] < 0.8+FLOOR_HEIGHT || rs.time == MRDPLOT_SAVE_TIME){  
        //logger.writeToMRDPLOT();
        printf("Log saved\n\n\n\n");
        if(!write_the_mrdplot_file(mrdptr))
          gzerr<< "Can not write mrdplot file\n"; 
        wc.stop();
        exit(0);
      }       

      forwardKinematics();
      //EstimateState();
      updateRobotState(ctr);
      
      // printf("rs.time %g ctr %lu phase %d\n", rs.time, ctr, phase);
      
      if (planFlag) {
        std::vector<SFootStep> fsplan;
        double pos0[6], pos1[6];

        dvec_copy(pos0, rs.root, 6);
        dvec_copy(pos1, rs.root, 6);
        wc.lipm_z_d = 0.86;
        wc.lipm_step_duration = 0.8;
        wc.lipm_step_height = 0.25;
        wc.lipm_step_ds_duration = 0.06;
        
        switch (state) {
          case -2:
            pos0[ZZ] = 0;
            pos1[ZZ] = 0;
            pos1[5] = 10*M_PI;
            
            make_turn_fs_plan(pos0, pos1, fsplan);
            wc.updateFootSteps(rs, fsplan);
            planFlag = false;
            break;
           
          case 0:
            // go up ramp
            pos0[ZZ] = 0;
            pos1[ZZ] = 0;

            ////////////////////////////////////////////////////////////////////
            // change here to for longer foot steps / distance 
            pos1[XX] = 50;
            make_straight_fs_plan(pos0, pos1, 0.4, fsplan); 
            ////////////////////////////////////////////////////////////////////

            wc.updateFootSteps(rs, fsplan);
            planFlag = false;
            break;

          case 1:
            //logger.writeToMRDPLOT();
            wc.stop();
            exit(0);
            break;
        }
      }
      ///////////////////////////////////////////////////////////////////////
      if (rs.time > 1) {
        wc.idCon.copWeight = 8e-3;
        wc.idCon.kneeHack = false;
      }

      wc.control(rs, cmd);
      // fb
      for (int i = 0; i < N_JOINTS; i++) {
        //cmd.controls[i] = cmd.controls_ff[i] + 1*rs.jointsd[i]*joint_damping[i];
        cmd.controls[i] = cmd.controls_ff[i] + 0*this->jointsPtr[i]->GetVelocity(0)*joint_damping[i];
      }

      //logger.saveData(rs, cmd);
      
      // apply torque
      for (int i = 0; i < N_JOINTS; i++) {
        this->jointsPtr[i]->SetForce( 0, cmd.controls[i] );
        this->jointsPtr[i]->Update();
      }
      
      // fall quit
      if (rs.root[ZZ] < 0.8+FLOOR_HEIGHT || rs.time > MRDPLOT_SAVE_TIME) {
        //logger.writeToMRDPLOT();
        wc.stop();
        exit(0);
      }

      ctr++;
    }
    void updateRobotState(size_t t)
    {
      rs.storePrevs();
      rs.time = current_time;
      rs.timeStep = TIME_STEP;

      dvec_copy(rs.root, root_pos, 3);
      dvec_copy(rs.root+3, root_euler, 3);
      dvec_copy(rs.rootd, root_vel, 3);
      dvec_copy(rs.rootd+3, root_w, 3);
      dvec_copy(rs.root_b_w, root_b_w, 3);
      dvec_copy(rs.rootq, root_q, 4);
      
      dvec_copy(rs.joints, joints, N_JOINTS);
      dvec_copy(rs.jointsd, jointsd, N_JOINTS);

      rs.computeSDFvars(rs.root, rs.rootq, rs.rootd, rs.root_b_w, rs.joints, rs.jointsd);
      //rs.computeSDFvars(root_pos, root_q, root_vel, root_b_w, joints, jointsd);

      // HACK
      // foot pos / force stuff, 
      for(int i = 0; i < XYZ; i++) {
        for (int side = 0; side < LR; side++) {
          rs.react[side*6+i] = 0;
          rs.react[side*6+3+i] = 0;
        }
      }      

      // contact state
      if (wc.hasInited()) 
        rs.contactState = wc.getPlannedContactState(rs.time);
      else
        rs.contactState = DSc;
      
      // returns all zeros?!
      // acc part
      VectorToArray(linksPtr[PELV_S_BODY_PELVIS]->GetWorldLinearAccel(), rs.rootdd);
      VectorToArray(linksPtr[PELV_S_BODY_PELVIS]->GetRelativeAngularAccel(), rs.rootdd+3);

      VectorToArray(linksPtr[PELV_S_BODY_L_FOOT]->GetWorldLinearAccel(), rs.feet[LEFT].w_acc);
      VectorToArray(linksPtr[PELV_S_BODY_L_FOOT]->GetRelativeAngularAccel(), rs.feet[LEFT].w_acc+3);
      VectorToArray(linksPtr[PELV_S_BODY_R_FOOT]->GetWorldLinearAccel(), rs.feet[RIGHT].w_acc);
      VectorToArray(linksPtr[PELV_S_BODY_R_FOOT]->GetRelativeAngularAccel(), rs.feet[RIGHT].w_acc+3);
      
      /*
      // use the actuall foot stuff instead of sdfast computed kinematics
      math::Pose fl_pose = linksPtr[PELV_S_BODY_L_FOOT]->GetWorldCoGPose();
      math::Vector3 fl = fl_pose.pos;
      math::Vector3 fl_v = linksPtr[PELV_S_BODY_L_FOOT]->GetWorldCoGLinearVel();
      math::Vector3 fl_w = linksPtr[PELV_S_BODY_L_FOOT]->GetRelativeAngularVel();
      VectorToArray(fl, rs.feet[LEFT].w_pos);
      VectorToArray(fl_v, rs.feet[LEFT].w_vel);
      VectorToArray(fl_w, rs.feet[LEFT].w_vel+3);
      rs.feet[LEFT].w_q[0] = fl_pose.rot.x;
      rs.feet[LEFT].w_q[1] = fl_pose.rot.y;
      rs.feet[LEFT].w_q[2] = fl_pose.rot.z;
      rs.feet[LEFT].w_q[3] = fl_pose.rot.w;
      normalize_q(rs.feet[LEFT].w_q);
      math::Vector3 euler = fl_pose.rot.GetAsEuler();
      rs.feet[LEFT].w_pos[3] = euler.x;
      rs.feet[LEFT].w_pos[4] = euler.y;
      rs.feet[LEFT].w_pos[5] = euler.z;
      
      math::Pose fr_pose = linksPtr[PELV_S_BODY_R_FOOT]->GetWorldCoGPose();
      math::Vector3 fr = fr_pose.pos;
      math::Vector3 fr_v = linksPtr[PELV_S_BODY_R_FOOT]->GetWorldCoGLinearVel();
      math::Vector3 fr_w = linksPtr[PELV_S_BODY_R_FOOT]->GetRelativeAngularVel();
      VectorToArray(fr, rs.feet[RIGHT].w_pos);
      VectorToArray(fr_v, rs.feet[RIGHT].w_vel);
      VectorToArray(fr_w, rs.feet[RIGHT].w_vel+3);
      rs.feet[RIGHT].w_q[0] = fr_pose.rot.x;
      rs.feet[RIGHT].w_q[1] = fr_pose.rot.y;
      rs.feet[RIGHT].w_q[2] = fr_pose.rot.z;
      rs.feet[RIGHT].w_q[3] = fr_pose.rot.w;
      normalize_q(rs.feet[RIGHT].w_q);
      euler = fr_pose.rot.GetAsEuler();
      rs.feet[RIGHT].w_pos[3] = euler.x;
      rs.feet[RIGHT].w_pos[4] = euler.y;
      rs.feet[RIGHT].w_pos[5] = euler.z; 
      */
    }

  private: 
    int phase;
    // my stuff
    double current_time;
    double root_pos[XYZ];
    double root_euler[RPY];
    double root_q[QQ];
    double root_vel[XYZ];
    double root_b_w[3]; 
    double root_w[3]; 

    double joints[ N_JOINTS ];
    double jointsd[ N_JOINTS ];
    size_t ctr;
 
    RobotState rs;
    Command cmd;
    WalkingCon wc;
    Logger logger;
    
    // Pointer to the model
    physics::ModelPtr model;

    // Pointers to joints
    std::vector<physics::JointPtr> jointsPtr;
    std::vector<physics::LinkPtr> linksPtr;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    common::Time startupTime;

    double last_action_time;
    int state;
    int step_ctr;


        // Force sensor pointers (V2.2)
    physics::JointPtr rAnkleJoint;
    physics::JointPtr lAnkleJoint;

    // IMU sensor (V2.2)
    boost::shared_ptr<sensors::ImuSensor> imuSensor;
    std::string imuLinkName;
    physics::LinkPtr imuLink;
    // ros::Publisher pubImu;
    common::Time lastImuTime;   

    double imu_angular_velocity[3];
    double imu_linear_acceleration[3];
    double imu_orientation[4];
    double lcontact_force[3]; // Left foot contact force
    double lcontact_torque[3]; // left foot contact torque
    double rcontact_force[3]; // right foot contact force
    double rcontact_torque[3]; //right foot contact torque

    // KF stuff
    IMUFilter imuekf;
    BiasFilter biasekf;        
    JointvFilter jvekf;
    //KinematicFilter kcekf;
    KinematicFilter1 kcekf;
    //IMUFilter *imuekfptr; 
    // Data for mrdplot, defined in data.h 
    MRDDATA mrd_data;
    MRDDATA *mrdptr;        

    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    void EstimateState()
    {
      int i;
      Eigen::Matrix<double,6,1> imusensed;         // Actual sensor 
      Eigen::Matrix<double,6,1> imubias_filtered;  // From bias filter
      Eigen::Matrix<double,6,1> imuwa_filtered;    // From imu filter

      imusensed.block(0,0,3,1) = Eigen::Map<const Eigen::Matrix<double,3, 1> >(imu_angular_velocity);
      imusensed.block(3,0,3,1) = Eigen::Map<const Eigen::Matrix<double,3, 1> >(imu_linear_acceleration);
      
      imubias_filtered = biasekf.getX();
      double w_imuekf[3], q_imuekf[4], a_imuekf[3]; // These lines can be deleted once the filter is tested working 

      imuekf.getAngularV(w_imuekf);
      imuekf.getQuaternion(q_imuekf);
      imuekf.getLinearA(a_imuekf);
      
      double jointsd_jvekf[N_JOINTS];
      jvekf.getX(jointsd_jvekf, N_JOINTS); 
      
      imuwa_filtered.block(0,0,3,1) = Eigen::Map<const Eigen::Matrix<double,3, 1> >(w_imuekf);
      imuwa_filtered.block(3,0,3,1) = imuekf.getLinearA_GLocal();  


      jvekf.makePelDynState(root_pos, q_imuekf , root_vel, w_imuekf, joints);      // This has to be changed with kinematic filter
      jvekf.getContactState(rs.contactState);
      jvekf.makeControl(cmd.controls);
      
      jvekf.makeMeasurement(this->jointsd);
      
      //kcekf.makeInputs(w_imuekf, a_imuekf, this->joints, jointsd_jvekf);
      // Use LPF joint velocity other than the filter
      kcekf.makeInputs(q_imuekf, w_imuekf, a_imuekf, this->joints, this->jointsd); 
      //kcekf.makeInputs(q_imuekf, w_imuekf, a_imuekf, this->joints, jointsd_jvekf);
      kcekf.getContactState(rs.contactState);
      
      //kcekf. makeMeasurement(this->joints);
      kcekf. makeMeasurement(); 
      biasekf.makeMeasurement(imusensed, imuwa_filtered);

      imuekf.makeMeasurement(imusensed, imubias_filtered) ;
      
      // Filter step
      biasekf.filterOneTimeStep_ss(); // Steady state filter
      imuekf.filterOneTimeStep();  //
      jvekf.filterOneTimeStep_ss(); // Steady state filter, the dynamic is nonlinear
      kcekf.filterOneTimeStep(); 
      //std::cout << (imuekf.getInnov()).transpose() <<std::endl;
      //std::cout << imubias_filtered.transpose() <<std::endl;

      // Reset states
      imuekf.getQuaternion(root_q);    // Reset quaternion
      imuekf.getAngularV(root_b_w);    // Reset angular velocity
      //jvekf.getX(jointsd, N_JOINTS);   // Reset jointsd
      // Z position is not set
      //double root_pos_temp[3];
      //kcekf.getPosition(root_pos_temp);
      //root_pos[XX] = root_pos_temp[XX];
      //root_pos[YY] = root_pos_temp[YY];
      kcekf.getPosition(root_pos);
      kcekf.getLinearV(root_vel);
      //kcekf.getQuaternion(); // This has to be merged with root_q
    }

/****************************************************************************/  
    void copy_mrdplot_data()
    {
      mrdptr->time_step = rs.timeStep; 
      mrdptr->current_time = rs.time; 
      
      // Actual from simulator
      for (int i = 0; i<N_JOINTS; i++){   
        mrdptr->joints[i] = this->jointsPtr[i]->GetAngle(0).Radian();
        mrdptr->jointsd[i] = this->jointsPtr[i]->GetVelocity(0);  
        mrdptr->torques[i] = cmd.controls[i];
      }
      // These are actual states directly from Gazebo
      math::Pose pose = linksPtr[PELV_S_BODY_PELVIS]->GetWorldPose();
      mrdptr->root_q[3] = pose.rot.w;
      mrdptr->root_q[0] = pose.rot.x;
      mrdptr->root_q[1] = pose.rot.y;
      mrdptr->root_q[2] = pose.rot.z;   

      // update pelvis pos
      math::Pose pelv_pose = linksPtr[PELV_S_BODY_PELVIS]->GetWorldCoGPose();
      math::Vector3 pelv_com = pelv_pose.pos;
      math::Vector3 pelv_com_v = linksPtr[PELV_S_BODY_PELVIS]->GetWorldCoGLinearVel();

      VectorToArray(pelv_com,  mrdptr->root_pos);
      VectorToArray(pelv_com_v,  mrdptr->root_vel);   

      // update pelvis ang vel
      //math::Vector3 angVel = linksPtr[PELV_S_BODY_PELVIS]->GetWorldAngularVel();
//      math::Vector3 angVel = linksPtr[PELV_S_BODY_PELVIS]->GetRelativeAngularVel();
//      VectorToArray(angVel, root_b_w);
//      angVel = linksPtr[PELV_S_BODY_PELVIS]->GetWorldAngularVel();
 //     VectorToArray(angVel, root_w);   


      //math::Vector3 angVel = linksPtr[PELV_S_BODY_PELVIS]->GetWorldAngularVel();
      math::Vector3 angVel = linksPtr[PELV_S_BODY_PELVIS]->GetRelativeAngularVel();
      VectorToArray(angVel,  mrdptr->root_w);

      // Sensors
      for (int i = 0; i<QQ; i++)
        mrdptr->imu_orientation[i] = this->imu_orientation[i];

      for (int i = 0; i < 3; i++) {
        mrdptr->imu_angular_velocity[i] = this->imu_angular_velocity[i];
        mrdptr->imu_linear_acceleration[i] = this->imu_linear_acceleration[i];
      }
      for (int i = 0; i < 3; i++) {
        mrdptr->lcontact_force[i] = this->lcontact_force[i];
        mrdptr->rcontact_force[i] = this->rcontact_force[i];
        mrdptr->lcontact_torque[i] = this->lcontact_torque[i];
        mrdptr->rcontact_torque[i] = this->rcontact_torque[i];
      }

      this->imuekf.getQuaternion(mrdptr->KFroot_q);
      this->imuekf.getAngularV(mrdptr->KFroot_w);
      this->imuekf.getLinearA(mrdptr->KFroot_acc);
      this->biasekf.getWb(mrdptr->KFw_b);
      this->biasekf.getAb(mrdptr->KFa_b);
      this->jvekf.getX(mrdptr->KFjointsd, N_JOINTS);
      
      this->kcekf.getPosition(mrdptr->KFroot_pos);
      this->kcekf.getLinearV(mrdptr->KFroot_vel);
      mrdptr->mag1 = this->kcekf._mag1;
      mrdptr->mag2 = this->kcekf._mag2;
      //this->kcekf.getLeftFootPos(mrdptr->KFfoot_pos[LEFT]);
      //this->kcekf.getRightFootPos(mrdptr->KFfoot_pos[RIGHT]);
      //this->kcekf.getQuaternion(mrdptr->KFroot_q_from_kc);

      for (int i = 0; i < N_JOINTS; i++) {
        mrdptr->KFjointsd[i]+= 0.3; // Offset to see clearly in mrdplot  
      }
      // JointEKF
      //for (int i = 0; i<N_JOINTS; i++)  
      //  mrdptr->KFjointsd[i] = this->jointdekfptr->xk->data[i];

      // IMU KF
      /*
      for (int i = 0; i < 3; i++) {
        mrdptr->KFroot_pos[i] = this->imukfptr->root_pos[i];
        mrdptr->KFroot_vel[i] = this->imukfptr->root_vel[i];
        mrdptr->KFroot_w[i] = this->imukfptr->root_w[i];
        mrdptr->KFroot_acc[i] = this->imukfptr->root_acc[i];
        mrdptr->KFroot_acc_pelv[i] = this->imukfptr->root_acc_pelv[i];
        mrdptr->KFw_b[i] = this->imukfptr->w_b[i];
        mrdptr->KFa_b[i] = this->imukfptr->a_b[i];
        mrdptr->KFfoot_pos[LEFT][i] = this->imukfptr->foot_pos[LEFT][i];
        mrdptr->KFfoot_pos[RIGHT][i] = this->imukfptr->foot_pos[RIGHT][i];
        mrdptr->KFroot_to_foot[LEFT][i] = this->imukfptr->root_to_foot[LEFT][i];
        mrdptr->KFroot_to_foot[RIGHT][i] = this->imukfptr->root_to_foot[RIGHT][i];
      }

     */
     
      //for (int i = 0; i<QQ; i++)
      //  mrdptr->KFroot_q[i] = this->ekfimuptr->root_q[i];  
      
      /*
      for (int i = 0; i<PELV_N_SDFAST_STATE; i++)
      {
        mrdptr->KFStateQuat[i] = this->quatukfptr->KFStateQuat_RECORD[i];
      } 
      */
      mrdptr->state = (double) rs.contactState;
     // mrdptr->cs_from_fz = (double) this->quatukfptr->CS; // Contact state from Fz
      
      //std::cout << mrdptr->current_time << std::endl;
    }        
    /****************************************************************************/
    /****************************************************************************/
    // This is another implementation of IMU sensor (DRCsim V2.2)
    void ReadImu2( double *angular_velocity, double *acceleration, double *orientation)
    { 

      common::Time curTime = this->model->GetWorld()->GetSimTime();
      // get imu data from imu link
      if (this->imuSensor && curTime > this->lastImuTime)
      {
        // sensor_msgs::Imu* imuMsg = &this->atlasState.imu;
        // imuMsg->header.frame_id = this->imuLinkName;
        // imuMsg->header.stamp = ros::Time(curTime.Double());

        // compute angular rates
        {
          math::Vector3 wLocal = this->imuSensor->GetAngularVelocity();
          angular_velocity[0] = wLocal.x;
          angular_velocity[1] = wLocal.y;
          angular_velocity[2] = wLocal.z;
        }

        // compute acceleration
        {
          math::Vector3 accel = this->imuSensor->GetLinearAcceleration();
          acceleration[0] = accel.x;
          acceleration[1] = accel.y;
          acceleration[2] = accel.z;
        }

        // compute orientation
        {
          math::Quaternion imuRot = this->imuSensor->GetOrientation();
          orientation[1] = imuRot.x;
          orientation[2] = imuRot.y;
          orientation[3] = imuRot.z;
          orientation[0] = imuRot.w;
        }
        // Update time
        this->lastImuTime = curTime.Double(); 
      }
    }



    /****************************************************************************/
    /****************************************************************************/ 
    void ReadContactLeft(double *force, double *torque)
    {
      physics::JointWrench wrench = this->lAnkleJoint->GetForceTorque(0);

      // Copy the force to double array
      force[0] = wrench.body2Force.x;
      force[1] = wrench.body2Force.y;
      force[2] = wrench.body2Force.z;
      torque[0] = wrench.body2Torque.x;
      torque[1] = wrench.body2Torque.y;  
      torque[2] = wrench.body2Torque.z;   
    }

    /****************************************************************************/
    /****************************************************************************/
    void ReadContactRight(double *force, double *torque)
    {
      physics::JointWrench wrench = this->rAnkleJoint->GetForceTorque(0);

      // Copy the force to double array
      force[0] = wrench.body2Force.x;
      force[1] = wrench.body2Force.y;
      force[2] = wrench.body2Force.z;
      torque[0] = wrench.body2Torque.x;
      torque[1] = wrench.body2Torque.y;  
      torque[2] = wrench.body2Torque.z;  
    }                                         

    /****************************************************************************/
    /****************************************************************************/  
    
    void forwardKinematics()
    {  
      // update pelvis pos
      math::Pose pelv_pose = linksPtr[PELV_S_BODY_PELVIS]->GetWorldCoGPose();
      math::Vector3 pelv_com = pelv_pose.pos;
      math::Vector3 pelv_com_v = linksPtr[PELV_S_BODY_PELVIS]->GetWorldCoGLinearVel();

      VectorToArray(pelv_com, root_pos);
      VectorToArray(pelv_com_v, root_vel);

      // update pelvis ang vel
      //math::Vector3 angVel = linksPtr[PELV_S_BODY_PELVIS]->GetWorldAngularVel();
      math::Vector3 angVel = linksPtr[PELV_S_BODY_PELVIS]->GetRelativeAngularVel();
      VectorToArray(angVel, root_b_w);
      angVel = linksPtr[PELV_S_BODY_PELVIS]->GetWorldAngularVel();
      VectorToArray(angVel, root_w);

      root_q[0] = pelv_pose.rot.x;
      root_q[1] = pelv_pose.rot.y;
      root_q[2] = pelv_pose.rot.z;
      root_q[3] = pelv_pose.rot.w;
      normalize_q(root_q);
 
      math::Vector3 euler = pelv_pose.rot.GetAsEuler();
      root_euler[0] = euler.x;
      root_euler[1] = euler.y;
      root_euler[2] = euler.z;
 
      // update joint status
      static double joints_h[N_JOINTS] = {0};
      static double jointsd_h[N_JOINTS] = {0};
      double e = 0.9; //9;
      //double e = 0.7;
      
      for (int i = 0; i < N_JOINTS; i++)
      {
        joints[i] = this->jointsPtr[i]->GetAngle(0).Radian();
        jointsd[i] = this->jointsPtr[i]->GetVelocity(0);

        //joints_h[i] = joints_h[i]*e + joints[i]*(1-e);
        jointsd_h[i] = jointsd_h[i]*e + jointsd[i]*(1-e);

        //joints[i] = joints_h[i];
        jointsd[i] = jointsd_h[i];
      }
      /*
      static double v_h[3] = {0};
      static double w_h[3] = {0};
      static double b_w_h[3] = {0};
      for (int i = 0; i < 3; i++) {
        v_h[i] = v_h[i]*e + root_vel[i]*(1-e);
        root_vel[i] = v_h[i];

        w_h[i] = w_h[i]*e + root_w[i]*(1-e);
        root_w[i] = w_h[i];
        
        b_w_h[i] = b_w_h[i]*e + root_b_w[i]*(1-e);
        root_b_w[i] = b_w_h[i];
      }
      */
      /*
      */

      // update time
      common::Time tt = this->model->GetWorld()->GetSimTime()-this->startupTime;
      current_time = tt.Double(); 
    }

    void print_statics() 
    {
      printf("%15s, %15s %15s %15s %15s %15s %15s %15s\n", "names", "pos_d", "pos", "vel_d", "vel", "acc_d", "trq_ff", "trq");
      for (int i = 0; i < N_JOINTS; i++) {
        printf("%15s, %15g %15g %15g %15g %15g %15g %15g\n", Simulator::joint_names[i].c_str(), cmd.joints_d[i], rs.joints[i], cmd.jointsd_d[i], rs.jointsd[i], cmd.jointsdd_d[i], cmd.controls_ff[i], cmd.controls[i]);
      }
    }

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN( atlas_terrain )

}

