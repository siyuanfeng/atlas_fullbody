#include <test_walk/control_loop_sf_walk.h>
#include <cmu_walk/LipmWalkingCon.h>
#include <test_walk/cmu_ctrl_utils.h>

#include <cmu_walk/DummyFSPlanner.h>
 
static int cState;
static CMUCtrlUtils utils;           // handles talking to the simulator
static PelvRobotState rs;            // contains all info about robot
static KinematicFilter3 kcekf;       // state estimator
static LipmWalkingCon lwc;           // walking controller
static Command cmd;                  // contains outputs from controller
static BatchLogger logger;           // logging

static double init_time = 0;
extern std::string pkg_name;

void control_loop_sf_walk(const atlas_msgs::AtlasState &data_from_robot,
    boost::mutex &data_from_robot_lock,
    atlas_msgs::AtlasCommand &data_to_robot,
    test_walk::field_param &params,
    test_walk::AtlasWalkParams &input_steps,
    bool firstTime)
{
  static int state = 0;

  {
    boost::mutex::scoped_lock lock(data_from_robot_lock);
    utils.UnpackDataFromRobot(data_from_robot);
  }

  if (firstTime) {
    for (int i = 0; i < N_JOINTS; i++)
      utils.f_mask[i] = CMUCtrlUtils::FF;
    utils.init_KF(kcekf, rs.getType(), 0.9545);
    utils.updateRobotState(DSc, rs);
    lwc.init(rs);
    init_time = utils.time;
    rs.ctrl_dt = 0;  
  }
  else {
    cState = lwc.getPlannedContactState(utils.time);
    utils.estimateState(cState, kcekf, utils.foot_forces[LEFT][ZZ], utils.foot_forces[RIGHT][ZZ]);
    utils.updateRobotState(cState, rs);  
  }
  
  if (rs.time - init_time > 5 && state == 0) {
    state++;
    std::vector<SFootStep> fsplan;
    double pos0[6], pos1[6];
    dvec_copy(pos0, rs.root, 6); 
    pos0[ZZ] = (rs.feet[LEFT].w_mid_pos[ZZ]+rs.feet[RIGHT].w_mid_pos[ZZ]) / 2.; 
    dvec_copy(pos1, pos0, 6); 
    pos1[XX] = 3.5; 
    make_straight_fs_plan(pos0, pos1, 0.3, fsplan);
    lwc.updateFootSteps(rs, fsplan);
  }

  // run walking controller
  lwc.control(rs, cmd);

  utils.PackDataToRobot(cmd, rs.time, data_to_robot);
  logger.saveData();
}

void initialize_loop_sf_walk()
{
  // initialize controller
  load_KFParams(pkg_name, kcekf);
  lwc.allocCon(rs.getType());
  // load control params
  load_sf_params(
      pkg_name,
      std::string("/config/con_param/atlas_static_idCon.conf"),
      std::string("/config/con_param/atlas_static_ikCon.conf"),
      std::string("/config/con_param/atlas_wc.conf"),
      lwc); 
   
  // init logger
  logger.init(0.001);
  rs.addToLog(logger);
  cmd.addToLog(logger);
  lwc.idcmd.addToLog(logger);
  lwc.idCon->addToLog(logger);
  lwc.ikcmd.addToLog(logger);
  lwc.ikCon->addToLog(logger);
  utils.addToLog(logger);
  lwc.addToLog(logger); 
}

void quit_sf_walk()
{
  logger.writeToMRDPLOT();
} 
