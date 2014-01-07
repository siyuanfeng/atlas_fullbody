#include <cmu_walk/RobotState.h>
#include <cmu_walk/Command.h>
#include <cmu_walk/Utils.hpp>
#include <iostream>
#include <cmu_walk/mrdplot.h>
#include <cmu_walk/drc_pelvis_defines.h>
#include <cmu_walk/drc_hat_defines.h>

#include <eigen3/Eigen/SVD>

double joints[HAT_N_JOINTS] = {0};
double jointsd[HAT_N_JOINTS] = {0};
double root[6] = {0};
double rootd[6] = {0};
double rootq[4] = {0,0,0,1};
double root_b_w[3] = {0};

MRDPLOT_DATA *data;

double lookup_value( MRDPLOT_DATA *d, int point, int channel );
void read_data(int t, double q[4], double w_b[3], double j[HAT_N_JOINTS], double jd[HAT_N_JOINTS], double trq[HAT_N_JOINTS], double F[12]);
void avg_data(int start, int end, double acc_q[4], double acc_w_b[3], double acc_j[HAT_N_JOINTS], double acc_jd[HAT_N_JOINTS], double acc_trq[HAT_N_JOINTS], double acc_F[12]);
void get_cop(int idx, double cop[LR][2]);

void check_with_pelv()
{
  PelvRobotState rs;
  rs.feet[LEFT].setRefPoint(Foot::RefAtlas);
  rs.feet[RIGHT].setRefPoint(Foot::RefAtlas);

  rs.rootq = Eigen::Quaterniond::Identity();
  rs.computeSDFvars();

  double pos[3];
  double zero[3] = {0};
  rs.model->sdpos(PELV_S_BODY_L_FOOT, zero, pos);
  printf("lf com %.5g %.5g %.5g\n", pos[0], pos[1], pos[2]);
  
  printf("lf center %.5g %.5g %.5g\n", rs.feet[LEFT].w_pos[0], rs.feet[LEFT].w_pos[1], rs.feet[LEFT].w_pos[2]);
  
  rs.model->sdgetbtj(PELV_S_JOINT_L_LEG_LAX, zero);
  rs.model->sdpos(PELV_S_BODY_L_FOOT, zero, pos);
  printf("lax %.5g %.5g %.5g\n", pos[0], pos[1], pos[2]);
  getchar();

  Command cmd;

  //read_data((int)(140./0.003), rs.rootq.coeffs().data(), rs.root_b_w, rs.joints, rs.jointsd, cmd.controls, rs.react);
  avg_data((int)(140./0.003), (int)(160/0.003), rs.rootq.coeffs().data(), rs.root_b_w, rs.joints, rs.jointsd, cmd.controls, rs.react);

  rs.rootq.normalize();
  printf("\n\n\n\n\n");
  
  // compute M, J
  rs.computeSDFvars();

  Eigen::Matrix<double,PELV_N_SDFAST_U,1> qdd(Eigen::Matrix<double,PELV_N_SDFAST_U,1>::Zero());
  Eigen::Matrix<double,PELV_N_JOINTS,1> tau_m = Eigen::Map<Eigen::Matrix<double,PELV_N_JOINTS,1> >(cmd.controls);
  Eigen::Matrix<double,PELV_N_JOINTS,1> tau_d = Eigen::Map<Eigen::Matrix<double,PELV_N_JOINTS,1> >(cmd.id_controls);

  // need to transform
  Eigen::Matrix<double,12,1> F_m = Eigen::Map<Eigen::Matrix<double,12,1> >(rs.react);
  Eigen::Matrix<double,12,1> F_d = Eigen::Map<Eigen::Matrix<double,12,1> >(cmd.id_grf);

  Eigen::Matrix<double,PELV_N_SDFAST_U,12,Eigen::RowMajor> Jt;
  Jt << rs.J[LEFT], rs.J[RIGHT];
  
  Eigen::Matrix<double,PELV_N_SDFAST_U,1> h = rs.nonLin;
  Eigen::Matrix<double,PELV_N_SDFAST_U,PELV_N_JOINTS,Eigen::RowMajor> S(Eigen::Matrix<double,PELV_N_SDFAST_U,PELV_N_JOINTS>::Zero());
  S.block<PELV_N_JOINTS,PELV_N_JOINTS>(6,0) = Eigen::Matrix<double,PELV_N_JOINTS,PELV_N_JOINTS>::Identity();
 
  // Forward dynamics
  // M*qdd = S*tau + Jt*F + h
  Eigen::Matrix<double,PELV_N_SDFAST_U,1> tmp = S * tau_m + Jt * F_m + h;
  qdd = rs.M.colPivHouseholderQr().solve(tmp);
  for (int i = 0; i < PELV_N_SDFAST_U; i++) {
    if (i < 6)
      printf("%10s %.5g\n", "pelvis", qdd[i]);
    else
      printf("%10s %.5g\n", RobotState::joint_names[i-6].c_str(), qdd[i]);
  }

  //std::cout << "qdd_diff\n" << qdd - Eigen::Map<Eigen::Matrix<double,PELV_N_SDFAST_U,1> >(cmd.qp_qdd) << std::endl;

  // check grf
  // Jt*F = M*qdd - S*tau - h, where qdd = 0
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jt, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix<double,12,1> F_pred = svd.solve(-S*tau_m - h);

  std::cout << "predict F: " << F_pred << std::endl;
  std::cout << "real F: " << F_m << std::endl;
  
  double offset[3] = {0}; 
  rs.model->sdgetbtj(PELV_S_JOINT_L_LEG_LAX, offset);
  printf("%.5g %.5g %.5g\n", offset[0], offset[1], offset[2]);

} 


void check_with_hat()
{
  HatRobotState rs;
  //rs.feet[LEFT].setRefPoint(Foot::RefCenter);
  //rs.feet[RIGHT].setRefPoint(Foot::RefCenter);
  rs.feet[LEFT].setRefPoint(Foot::RefAtlas);
  rs.feet[RIGHT].setRefPoint(Foot::RefAtlas);
  
  Command cmd;
  
  avg_data((int)(140./0.003), (int)(145./0.003), rs.rootq.coeffs().data(), rs.root_b_w, rs.joints, rs.jointsd, cmd.controls, rs.react);
  //avg_data((int)(203./0.003), (int)(206./0.003), rs.rootq.coeffs().data(), rs.root_b_w, rs.joints, rs.jointsd, cmd.controls, rs.react);

  rs.rootq.normalize();
  printf("\n\n\n\n\n");
  
  // compute M, J
  rs.computeSDFvars();
  printf("mass %g\n", rs.m);

  Eigen::Matrix<double,HAT_N_SDFAST_U,1> qdd(Eigen::Matrix<double,HAT_N_SDFAST_U,1>::Zero());
  Eigen::Matrix<double,HAT_N_JOINTS,1> tau_m;
  HatRobotState::pelv2hat(cmd.controls, tau_m.data());
  Eigen::Matrix<double,HAT_N_JOINTS,1> tau_d;
  HatRobotState::pelv2hat(cmd.controls_ff, tau_d.data());

  // need to transform
  Eigen::Matrix<double,12,1> F_m = Eigen::Map<Eigen::Matrix<double,12,1> >(rs.react);
  Eigen::Matrix<double,12,1> F_d = Eigen::Map<Eigen::Matrix<double,12,1> >(cmd.id_grf);

  Eigen::Matrix<double,HAT_N_SDFAST_U,12,Eigen::RowMajor> Jt;
  Jt << rs.J[LEFT], rs.J[RIGHT];
  
  Eigen::Matrix<double,HAT_N_SDFAST_U,1> h = rs.nonLin;
  Eigen::Matrix<double,HAT_N_SDFAST_U,HAT_N_JOINTS,Eigen::RowMajor> S(Eigen::Matrix<double,HAT_N_SDFAST_U,HAT_N_JOINTS>::Zero());
  S.block<HAT_N_JOINTS,HAT_N_JOINTS>(6,0) = Eigen::Matrix<double,HAT_N_JOINTS,HAT_N_JOINTS>::Identity();
 
  // Forward dynamics
  // M*qdd = S*tau + Jt*F + h
  Eigen::Matrix<double,HAT_N_SDFAST_U,1> tmp = S * tau_m + Jt * F_m + h;
  qdd = rs.M.colPivHouseholderQr().solve(tmp);
  for (int i = 0; i < HAT_N_SDFAST_U; i++) {
    if (i < 6)
      printf("%10s %.5g\n", "pelvis", qdd[i]);
    else
      printf("%10s %.5g\n", RobotState::joint_names[HatRobotState::hatIdx2pelvIdx(i-6)].c_str(), qdd[i]);
  }

  //std::cout << "qdd_diff\n" << qdd - Eigen::Map<Eigen::Matrix<double,HAT_N_SDFAST_U,1> >(cmd.qp_qdd) << std::endl;

  // check grf
  // Jt*F = M*qdd - S*tau - h, where qdd = 0
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jt, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix<double,12,1> F_pred = svd.solve(-S*tau_m - h);

  std::cout << "predict F: " << F_pred << std::endl;
  std::cout << "real F: " << F_m << std::endl;
  
  double offset[3] = {0}; 
  rs.model->sdgetbtj(HAT_S_JOINT_L_LEG_LAX, offset);
  printf("%.5g %.5g %.5g\n", offset[0], offset[1], offset[2]);

}


int main()
{
  data = read_mrdplot("/home/sfeng/ATLAS_DATA/922/field_rosbag_2013-09-22-12-45-28.mrd");
  printf("chanels %d %d\n", data->n_channels, data->n_points);

  double cop[LR][2];
  FILE *out = fopen("/home/sfeng/ATLAS_DATA/922/field_rosbag_2013-09-22-12-45-28_cop", "w");
  for(int i = 0; i < data->n_points; i++) {
    get_cop(i, cop);
    fprintf(out, "%.5g %.5g %.5g %.5g\n", cop[LEFT][XX], cop[LEFT][XX], cop[RIGHT][XX], cop[RIGHT][YY]);
  }
  fclose(out);

  check_with_pelv();
  check_with_hat();
  return 0;  
}

























double lookup_value( MRDPLOT_DATA *d, int point, int channel )
{
  if ( channel < 0 || channel >= d->n_channels )
  {
    fprintf( stderr, "Bad channel: %d\n", channel );
    exit( -1 );
  }
  return d->data[point*d->n_channels + channel];
}

void read_data(int t, double q[4], double w_b[3], double j[HAT_N_JOINTS], double jd[HAT_N_JOINTS], double trq[HAT_N_JOINTS], double F[12])
{
  int idx = find_channel("ftime", data);
  //printf("idx %d time %.5g\n", t, lookup_value(data, t, idx));

  q[0] = lookup_value(data, t, find_channel("imu_qx", data));
  q[1] = lookup_value(data, t, find_channel("imu_qy", data));
  q[2] = lookup_value(data, t, find_channel("imu_qz", data));
  q[3] = lookup_value(data, t, find_channel("imu_qw", data));
  
  //printf("quat %.5g %.5g %.5g %.5g\n", q[0], q[1], q[2], q[3]);

  //printf("==============\nb_w\n");
  // root_b_w
  idx = find_channel("imu_wx", data);
  for (int i = 0; i < 3; i++) {
    w_b[i] = lookup_value(data, t, idx+i);
    //printf("%.5g\n", w_b[i]);
  }

  int stride = 14;
  //printf("==============\njoints\n");
  // joints
  idx = find_channel("LBZ", data);
  for (int i = 0; i < HAT_N_JOINTS; i++) {
    j[i] = lookup_value(data, t, idx+stride*i);
    //printf("%10s, %.5g\n", RobotState::joint_names[i].c_str(), j[i]);
  }
  
  //printf("==============\njointsd\n");
  // jointsd
  idx = find_channel("LBZd", data);
  for (int i = 0; i < HAT_N_JOINTS; i++) {
    jd[i] = lookup_value(data, t, idx+stride*i);
    //printf("%10s %.5g\n", RobotState::joint_names[i].c_str(), jd[i]);
  }
  
  //printf("==============\nm_trq\n");
  // trq
  idx = find_channel("LBZ_trq", data);
  for (int i = 0; i < HAT_N_JOINTS; i++) {
    trq[i] = lookup_value(data, t, idx+stride*i);
    //printf("%10s %.5g\n", RobotState::joint_names[i].c_str(), trq[i]);
  }

  //printf("==============\nm_F"); 
  for (int i = 0; i < 12; i++) {
    F[i] = 0;
  }
  F[2] = lookup_value(data, t, find_channel("lf_fz", data));
  F[3] = lookup_value(data, t, find_channel("lf_mx", data));
  F[4] = lookup_value(data, t, find_channel("lf_my", data));
  F[6+2] = lookup_value(data, t, find_channel("rf_fz", data));
  F[6+3] = lookup_value(data, t, find_channel("rf_mx", data));
  F[6+4] = lookup_value(data, t, find_channel("rf_my", data));
  for (int i = 0; i < 12; i++) {
    //printf("%.5g\n", F[i]);
  }
}

void get_cop(int t, double cop[LR][2])
{
  cop[LEFT][XX] = -lookup_value(data, t, find_channel("lf_my", data)) / 
    lookup_value(data, t, find_channel("lf_fz", data));
  cop[LEFT][YY] = lookup_value(data, t, find_channel("lf_mx", data)) / 
    lookup_value(data, t, find_channel("lf_fz", data));
  cop[RIGHT][XX] = -lookup_value(data, t, find_channel("rf_my", data)) / 
    lookup_value(data, t, find_channel("rf_fz", data));
  cop[RIGHT][YY] = lookup_value(data, t, find_channel("rf_mx", data)) / 
    lookup_value(data, t, find_channel("rf_fz", data));
}

void avg_data(int start, int end, double acc_q[4], double acc_w_b[3], double acc_j[HAT_N_JOINTS], double acc_jd[HAT_N_JOINTS], double acc_trq[HAT_N_JOINTS], double acc_F[12])
{
  double q[4] = {0};
  double w_b[3] = {0};
  double j[HAT_N_JOINTS] = {0};
  double jd[HAT_N_JOINTS] = {0};
  double trq[HAT_N_JOINTS] = {0};
  double F[12] = {0};

  dvec_set(acc_q, 0, 4);
  dvec_set(acc_w_b, 0, 3);
  dvec_set(acc_j, 0, HAT_N_JOINTS);
  dvec_set(acc_jd, 0, HAT_N_JOINTS);
  dvec_set(acc_trq, 0, HAT_N_JOINTS);
  dvec_set(acc_F, 0, 12);

  for (int t = start; t < end; t++) {
    read_data(t, q, w_b, j, jd, trq, F);
    
    for (int i = 0; i < 4; i++)
      acc_q[i] += q[i];
    for (int i = 0; i < 3; i++)
      acc_w_b[i] += w_b[i];
    for (int i = 0; i < HAT_N_JOINTS; i++) {
      acc_j[i] += j[i];
      acc_jd[i] += jd[i];
      acc_trq[i] += trq[i];
    }
    for (int i = 0; i < 12; i++)
      acc_F[i] += F[i];
  }

  for (int i = 0; i < 4; i++)
    acc_q[i] /= (double)(end-start);
  for (int i = 0; i < 3; i++)
    acc_w_b[i] /= (double)(end-start);
  for (int i = 0; i < HAT_N_JOINTS; i++) {
    acc_j[i] /= (double)(end-start); 
    acc_jd[i] /= (double)(end-start);
    acc_trq[i] /= (double)(end-start);
  }
  for (int i = 0; i < 12; i++)
    acc_F[i] /= (double)(end-start);
}
   
