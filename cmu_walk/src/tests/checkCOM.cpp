#include <cmu_walk/RobotState.h>
#include <cmu_walk/Utils.hpp>
#include <cmu_walk/id_controller.hpp>
#include <cmu_walk/Logger.h>

double joints[N_JOINTS] = {0};
double jointsd[N_JOINTS] = {0};
double root[6] = {0};
double rootd[6] = {0};
double rootq[4] = {0,0,0,1};
double root_b_w[3] = {0};

int main ()
{
  PelvRobotState rs;
  //HatRobotState rs;
  
  dvec_copy(rs.root, root, 6);
  dvec_copy(rs.rootd, rootd, 6);
  rs.rootq = Eigen::Quaterniond(rootq[3], rootq[0], rootq[1], rootq[2]);
  dvec_copy(rs.root_b_w, root_b_w, 3); 
  
  dvec_copy(rs.joints, joints, N_JOINTS); 
  dvec_copy(rs.jointsd, jointsd, N_JOINTS); 

  rs.computeSDFvars();

  printf("%g com %.5g, %.5g, %.5g | %g %g %g\n", rs.m, rs.com[0], rs.com[1], rs.com[2], rs.root[0], rs.root[1], rs.root[2]);

  return 0;
}
