/////////////////////////////////////////////

/*****************************************************************************/
/*
  data.c: save data from simulation
*/

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>
#include "mrdplot.h"
#include "KFdata.h"

/*****************************************************************************/

/*****************************************************************************/

#define MAX_N_POINTS 100000
#define MAX_CHANNELS 500
#define  MAX_CHARS 30

int n_points = 0;
float data[MAX_N_POINTS*MAX_CHANNELS];


static const char * joint_names[N_JOINTS] =
{ 
  "back_lbz", "back_mby", "back_ubx", "neck_ay", 
  "l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax", 
  "r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax", 
  "l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx", 
  "r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx" 
}; 

typedef struct{
	char units[MAX_CHARS];
	char name[MAX_CHARS];
	double *ptr;
} DataPoint_t;

DataPoint_t datapoints[MAX_CHANNELS];
static int n_channels = 0;



void add_datapoint(char *name, char *units, double *ptr){
	if (n_channels<MAX_CHANNELS){
		DataPoint_t *dptr = &datapoints[n_channels];

		  strncpy(dptr->name,name,MAX_CHARS-1);
		  dptr->name[MAX_CHARS-1] = '\0';
		  strncpy(dptr->units,units,MAX_CHARS-1);
		  dptr->units[MAX_CHARS-1] = '\0';

		datapoints[n_channels].ptr = ptr;
		n_channels++;
	}
}

void init_data(KFdata* s)
{
  int i;
  char buff[MAX_CHARS];

  add_datapoint("time", "s", &s->current_time);
  
  for(i=0;i<N_JOINTS;i++){
    // Position
    sprintf(buff,"%s_th",joint_names[i]);
    add_datapoint(buff, "rad", &s->joints[i]);
    // Velocity
    sprintf(buff,"%s_thd",joint_names[i]);
    add_datapoint(buff, "rad/s", &s->jointsd[i]);

        // KF Velocity
    sprintf(buff,"KF.%s_thd",joint_names[i]);
    add_datapoint(buff, "rad/s", &s->KFjointsd[i]);
    // Torque
    sprintf(buff,"%s_trq",joint_names[i]);
    add_datapoint(buff, "Nm", &s->torques[i]);
    //// KF Position
    //sprintf(buff,"KF.%s_th",joint_names[i]);
    //add_datapoint(buff, "rad", &s->KFStateQuat[i+6]);

    //// KF Velocity
    //sprintf(buff,"KF.%s_thd",joint_names[i]);
    //add_datapoint(buff, "rad/s", &s->KFStateQuat[i+6+PELV_N_SDFAST_Q]);
  }


  //add_datapoint("l_foot_fx_LPF", "N", &s->lcontact_force_LPF[0]);
  //add_datapoint("l_foot_fy_LPF", "N", &s->lcontact_force_LPF[1]);
  //add_datapoint("l_foot_fz_LPF", "N", &s->lcontact_force_LPF[2]);

  //add_datapoint("r_foot_fx_LPF", "N", &s->rcontact_force_LPF[0]);
  //add_datapoint("r_foot_fy_LPF", "N", &s->rcontact_force_LPF[1]);
  //add_datapoint("r_foot_fz_LPF", "N", &s->rcontact_force_LPF[2]);

  //add_datapoint("l_foot_tx_LPF","Nm", &s->lcontact_torque_LPF[0]);
  //add_datapoint("l_foot_ty_LPF","Nm", &s->lcontact_torque_LPF[1]);
  //add_datapoint("l_foot_tz_LPF","Nm", &s->lcontact_torque_LPF[2]);

  //add_datapoint("r_foot_tx_LPF","Nm", &s->rcontact_torque_LPF[0]);
  //add_datapoint("r_foot_ty_LPF","Nm", &s->rcontact_torque_LPF[1]);
  //add_datapoint("r_foot_tz_LPF","Nm", &s->rcontact_torque_LPF[2]);  

  //add_datapoint("l_foot_fx_ave", "N", &s->lcontact_force_ave[0]);
  //add_datapoint("l_foot_fy_ave", "N", &s->lcontact_force_ave[1]);
  //add_datapoint("l_foot_fz_ave", "N", &s->lcontact_force_ave[2]);

  //add_datapoint("r_foot_fx_ave", "N", &s->rcontact_force_ave[0]);
  //add_datapoint("r_foot_fy_ave", "N", &s->rcontact_force_ave[1]);
  //add_datapoint("r_foot_fz_ave", "N", &s->rcontact_force_ave[2]);

  //add_datapoint("l_foot_tx_ave","Nm", &s->lcontact_torque_ave[0]);
  //add_datapoint("l_foot_ty_ave","Nm", &s->lcontact_torque_ave[1]);
  //add_datapoint("l_foot_tz_ave","Nm", &s->lcontact_torque_ave[2]);

  //add_datapoint("r_foot_tx_ave","Nm", &s->rcontact_torque_ave[0]);
  //add_datapoint("r_foot_ty_ave","Nm", &s->rcontact_torque_ave[1]);
  //add_datapoint("r_foot_tz_ave","Nm", &s->rcontact_torque_ave[2]);     
  
  // Actual pelvis states from Gazebo 
  add_datapoint("x", "m", &s->root_pos[0]);
  add_datapoint("y", "m", &s->root_pos[1]);
  add_datapoint("z", "m", &s->root_pos[2]);

  add_datapoint("qw", "xxx", &s->root_q[3]);
  add_datapoint("qx", "xxx", &s->root_q[0]);
  add_datapoint("qy", "xxx", &s->root_q[1]);
  add_datapoint("qz", "xxx", &s->root_q[2]);
  
  add_datapoint("vx", "m/s", &s->root_vel[0]);
  add_datapoint("vy", "m/s", &s->root_vel[1]);
  add_datapoint("vz", "m/s", &s->root_vel[2]);

  add_datapoint("wx", "rad/s", &s->root_w[0]);
  add_datapoint("wy", "rad/s", &s->root_w[1]);
  add_datapoint("wz", "rad/s", &s->root_w[2]);    


  // IMU readings
  add_datapoint("imu_wx", "rad/s", &s->imu_angular_velocity[0]);
  add_datapoint("imu_wy", "rad/s", &s->imu_angular_velocity[1]);
  add_datapoint("imu_wz", "rad/s", &s->imu_angular_velocity[2]);

  add_datapoint("imu_ax", "m/s/s", &s->imu_linear_acceleration[0]);
  add_datapoint("imu_ay", "m/s/s", &s->imu_linear_acceleration[1]);
  add_datapoint("imu_az", "m/s/s", &s->imu_linear_acceleration[2]);

  add_datapoint("imu_ax_world", "m/s/s", &s->imu_linear_acc_world[0]);
  add_datapoint("imu_ay_world", "m/s/s", &s->imu_linear_acc_world[1]);
  add_datapoint("imu_az_world", "m/s/s", &s->imu_linear_acc_world[2]); 
                                     
  add_datapoint("imu_quaternion_scalar", "-", &s->imu_orientation[0]);
  add_datapoint("imu_quaternion_vector_x", "-", &s->imu_orientation[1]);
  add_datapoint("imu_quaternion_vector_y", "-", &s->imu_orientation[2]);
  add_datapoint("imu_quaternion_vector_z", "-", &s->imu_orientation[3]);  

  // KF filtered states
  add_datapoint("KF.x", "m", &s->KFroot_pos[0]);   
  add_datapoint("KF.y", "m", &s->KFroot_pos[1]);   
  add_datapoint("KF.z", "m", &s->KFroot_pos[2]); 

  add_datapoint("KF.qw", "xxx", &s->KFroot_q[3]);
  add_datapoint("KF.qx", "xxx", &s->KFroot_q[0]);
  add_datapoint("KF.qy", "xxx", &s->KFroot_q[1]);
  add_datapoint("KF.qz", "xxx", &s->KFroot_q[2]);

  add_datapoint("KF.vx", "m/s", &s->KFroot_vel[0]);
  add_datapoint("KF.vy", "m/s", &s->KFroot_vel[1]);
  add_datapoint("KF.vz", "m/s", &s->KFroot_vel[2]);
  
  add_datapoint("KF.wx", "rad/s", &s->KFroot_w[0]);
  add_datapoint("KF.wy", "rad/s", &s->KFroot_w[1]);
  add_datapoint("KF.wz", "rad/s", &s->KFroot_w[2]);

  add_datapoint("KF.ax", "m/s/s", &s->KFroot_acc[0]);
  add_datapoint("KF.ay", "m/s/s", &s->KFroot_acc[1]);
  add_datapoint("KF.az", "m/s/s", &s->KFroot_acc[2]);     
 
  add_datapoint("KF.ax_pelv", "m/s/s", &s->KFroot_acc_pelv[0]);
  add_datapoint("KF.ay_pelv", "m/s/s", &s->KFroot_acc_pelv[1]);
  add_datapoint("KF.az_pelv", "m/s/s", &s->KFroot_acc_pelv[2]); 

  add_datapoint("KF.wx_bias", "rad/s", &s->KFw_b[0]);
  add_datapoint("KF.wy_bias", "rad/s", &s->KFw_b[1]);
  add_datapoint("KF.wz_bias", "rad/s", &s->KFw_b[2]); 

  add_datapoint("KF.ax_bias", "m/s/s", &s->KFa_b[0]);
  add_datapoint("KF.ay_bias", "m/s/s", &s->KFa_b[1]);
  add_datapoint("KF.az_bias", "m/s/s", &s->KFa_b[2]);

  add_datapoint("Lfootx", "m", &s->foot_pos[0][0]);   
  add_datapoint("Lfooty", "m", &s->foot_pos[0][1]);   
  add_datapoint("Lfootz", "m", &s->foot_pos[0][2]); 

  add_datapoint("Rfootx", "m", &s->foot_pos[1][0]);   
  add_datapoint("Rfooty", "m", &s->foot_pos[1][1]);   
  add_datapoint("Rfootz", "m", &s->foot_pos[1][2]);

  add_datapoint("Lfoot_vx", "m/s", &s->foot_vel[0][0]);   
  add_datapoint("Lfoot_vy", "m/s", &s->foot_vel[0][1]);   
  add_datapoint("Lfoot_vz", "m/s", &s->foot_vel[0][2]); 

  add_datapoint("Rfoot_vx", "m/s", &s->foot_vel[1][0]);   
  add_datapoint("Rfoot_vy", "m/s", &s->foot_vel[1][1]);   
  add_datapoint("Rfoot_vz", "m/s", &s->foot_vel[1][2]); 

  add_datapoint("Lroot_to_footx", "m", &s->root_to_foot[0][0]);   
  add_datapoint("Lroot_to_footy", "m", &s->root_to_foot[0][1]);   
  add_datapoint("Lroot_to_footz", "m", &s->root_to_foot[0][2]); 

  add_datapoint("Rroot_to_footx", "m", &s->root_to_foot[1][0]);   
  add_datapoint("Rroot_to_footy", "m", &s->root_to_foot[1][1]);   
  add_datapoint("Rroot_to_footz", "m", &s->root_to_foot[1][2]);   

  add_datapoint("Lfootx", "m", &s->foot_pos[0][0]);   
  add_datapoint("KF.Lfooty", "m", &s->KFfoot_pos[0][1]);   
  add_datapoint("KF.Lfootz", "m", &s->KFfoot_pos[0][2]); 

  add_datapoint("KF.Rfootx", "m", &s->KFfoot_pos[1][0]);   
  add_datapoint("KF.Rfooty", "m", &s->KFfoot_pos[1][1]);   
  add_datapoint("KF.Rfootz", "m", &s->KFfoot_pos[1][2]);

  add_datapoint("KF.Lfoot_vx", "m/s", &s->KFfoot_vel[0][0]);   
  add_datapoint("KF.Lfoot_vy", "m/s", &s->KFfoot_vel[0][1]);   
  add_datapoint("KF.Lfoot_vz", "m/s", &s->KFfoot_vel[0][2]); 

  add_datapoint("KF.Rfoot_vx", "m/s", &s->KFfoot_vel[1][0]);   
  add_datapoint("KF.Rfoot_vy", "m/s", &s->KFfoot_vel[1][1]);   
  add_datapoint("KF.Rfoot_vz", "m/s", &s->KFfoot_vel[1][2]);  

  add_datapoint("KF.Lroot_to_footx", "m", &s->KFroot_to_foot[0][0]);   
  add_datapoint("KF.Lroot_to_footy", "m", &s->KFroot_to_foot[0][1]);   
  add_datapoint("KF.Lroot_to_footz", "m", &s->KFroot_to_foot[0][2]); 

  add_datapoint("KF.Rroot_to_footx", "m", &s->KFroot_to_foot[1][0]);   
  add_datapoint("KF.Rroot_to_footy", "m", &s->KFroot_to_foot[1][1]);   
  add_datapoint("KF.Rroot_to_footz", "m", &s->KFroot_to_foot[1][2]);  


  add_datapoint("l_foot_fx", "N", &s->lcontact_force[0]);
  add_datapoint("l_foot_fy", "N", &s->lcontact_force[1]);
  add_datapoint("l_foot_fz", "N", &s->lcontact_force[2]);

  add_datapoint("r_foot_fx", "N", &s->rcontact_force[0]);
  add_datapoint("r_foot_fy", "N", &s->rcontact_force[1]);
  add_datapoint("r_foot_fz", "N", &s->rcontact_force[2]);

  add_datapoint("l_foot_tx","Nm", &s->lcontact_torque[0]);
  add_datapoint("l_foot_ty","Nm", &s->lcontact_torque[1]);
  add_datapoint("l_foot_tz","Nm", &s->lcontact_torque[2]);

  add_datapoint("r_foot_tx","Nm", &s->rcontact_torque[0]);
  add_datapoint("r_foot_ty","Nm", &s->rcontact_torque[1]);
  add_datapoint("r_foot_tz","Nm", &s->rcontact_torque[2]);    

  add_datapoint("contact_state","xxx", &s->state);  
  add_datapoint("contact_state_fz","xxx", &s-> cs_from_fz);
  //add_datapoint("imu_theta", "-", &s->imu_theta);  
  //add_datapoint("KF_theta", "-", &s->kf_theta);  

}

/*****************************************************************************/

bool save_data()
{
  static int myindex = 0;
  int i;
  double *ptr;

  if ( n_points >= MAX_N_POINTS )
    return true;

  for(i=0;i<n_channels;i++){
	  ptr = (double*)datapoints[i].ptr;
	  data[myindex + i] = (float)*ptr;
  }
  myindex+=n_channels;
  n_points++;

  return true;
}

/*****************************************************************************/

bool write_matlab_file( char *filename )
{
  FILE *stream;
  int i;
  static int myindex = 0;

  stream = fopen( filename, "w" );
  if ( stream == NULL )
    {
      fprintf( stderr, "Can't open %s for write.\n", filename );
      return false;
    }
  for ( i = 0; i < n_points; i++ )
    {
      fprintf( stream, "%g ", data[myindex] );
      fprintf( stream, "\n" );
      myindex += n_channels;
    }
  fclose( stream );

  return true;
}

/*****************************************************************************/

bool write_the_mrdplot_file(KFdata *s)
{
  MRDPLOT_DATA *d; 
  int i;
  char *names[MAX_CHANNELS];
  char *units[MAX_CHANNELS];

  d = malloc_mrdplot_data( 0, 0 );
  d->filename = generate_file_name();
  d->n_channels = n_channels;
  d->n_points = n_points;
  d->total_n_numbers = d->n_channels*d->n_points;
  d->frequency = 1.0/s->time_step;
  d->data = data;

  for(i=0;i<n_channels;i++){
	names[i] = datapoints[i].name;
	units[i] = datapoints[i].units;
	//sprintf(&d->units[i*MAX_CHARS],"%s ",datapoints[i].units);
  }
  d->names = names;
  d->units = units;

  write_mrdplot_file( d );

  // write_matlab_file( "dd" );

  return true;
}

/*****************************************************************************/


