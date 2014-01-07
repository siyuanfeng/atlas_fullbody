#ifndef KINEMATICFILTER_H
#define KINEMATICFILTER_H 

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include "RobotState.h"

/************************************************************************************************/ 
// Set a matrix from istream using >> operator
/************************************************************************************************/ 
// ============================================================================
/// @brief overload >> operator to set a matrix from istream
///
/// @tparam Derived
/// @param is istream input
/// @param rhs matrix to be filled
///
/// @return istream
// ============================================================================
template <typename Derived>
inline std::istream& operator>>(std::istream& is, Eigen::MatrixBase<Derived> const & rhs)
{
  typedef typename Derived::Scalar Scalar;   
  Scalar data[rhs.size()];  
  for(int i = 0; i < rhs.size(); ++i)
    is >> data[i];

  const_cast< Eigen::MatrixBase<Derived>& >(rhs) = 
    Eigen::Map< Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > (data, rhs.rows(), rhs.cols());

  return is;
};  


class KinematicFilter3 
{
  public:
    // ============================================================================
    /// @brief Constructor by copying stuff from other
    ///
    /// @param other
    // ===========================================================================
    KinematicFilter3 (const KinematicFilter3 & other);
    // ============================================================================
    /// @brief Copy operator
    ///
    /// @param other
    ///
    /// @return Class pointer
    // ============================================================================
    KinematicFilter3 & operator=(const KinematicFilter3 & other);

    // ============================================================================
    /// @brief Constructor, set P,Q,R to Identity
    // ============================================================================
    KinematicFilter3();
    // ============================================================================
    /// @brief Default destructor
    // ============================================================================
    ~KinematicFilter3();        

    // Takes angular velocity and linear acc as input
    void predictX();
    void updateZ(int zDim);
    void updateX(int zDim);
    // ============================================================================
    /// @brief compute innovation vector \f$ \Deta y = y - \hat{y}_{k+1}\f$
    ///
    /// @param z measurement 
    // ============================================================================
    void computeInnovation(const Eigen::Matrix<double, 6, 1> & z);

    void makeProcessJacobian ();

    void makeObservationJacobian ();
    // Ankle Related Stuff                        
    void computeAnkleRelated();
    // This function computes Kss once
    void setKss();

    // Steady state filter, call setKss first
    void filterOneTimeStep_ss();
    void getLinearV(double *v);
    void getPosition(double *x);
    void getFootPose(int side, double *x); //Measurement
    void makeInputs(double *q, double *w, double *a, double *joints, double *jointsd);
    void getContactState (int cs);
    // ROS
    void readParams(std::string &nameQ, std::string &name);
    /*void initKF(int type, double dt, const double root_pos[3], const double root_vel[3], Eigen::Matrix<double,6,6> &P0, Eigen::Matrix<double,6,6> &Q, Eigen::Matrix<double,6,6> &D_R, std::ifstream &in);*/
    void initKF(int type, double dt, const double root_pos[3], const double root_vel[3]);
    // ============================================================================
    /// @brief fill the measurement buffer.
    // ============================================================================
    void fill_measurement_buffer();
    // ============================================================================
    /// @brief compute foot position and velocity (used before makeMeasurement)
    // ============================================================================
    void computeFootPV();
    // ============================================================================
    /// @brief compute measurement. Measurement is the weighted sum of both feet (p,v) in previous steps.
    ///
    /// @param left_fz
    /// @param right_fz
    // ============================================================================
    void makeMeasurement(double left_fz, double right_fz);

    void set_hack_footpos();
    // ============================================================================
    /// @brief set internal state by double array x with length len
    ///
    /// @param x double array
    /// @param len int, array length
    // ============================================================================
    void setX (const double * x, int len);
    // ============================================================================
    /// @brief set internal state by Eigen matrix 
    ///
    /// @param x Eigen Column Matrix
    // ============================================================================
    void setX (const Eigen::Matrix<double, 6, 1> & x);
    // ============================================================================
    /// @brief set covariance
    ///
    /// @param P Eigen Matrix
    // ============================================================================
    void setP (const Eigen::Matrix<double, 6, 6> & P);
    // ============================================================================
    /// @brief set process covariance
    ///
    /// @param Q Eigen Matrix
    // ============================================================================
    void setQ (const Eigen::Matrix<double, 6, 6> & Q);
    // ============================================================================
    /// @brief set observation covariance
    ///
    /// @param R Eigen Matrix
    // ============================================================================
    void setR (const Eigen::Matrix<double, 6, 6> & R);
    // ============================================================================
    /// @brief get state and save it to x[len]
    ///
    /// @param x double array
    /// @param len length of x
    // ============================================================================
    void getX(double * x, int len);
   
    // ============================================================================
    /// @brief get state in Eigen
    ///
    /// @return Eigen Matrix
    // ============================================================================
    const Eigen::Matrix<double,6,1> & getX() const;             
    // ============================================================================
    /// @brief get predicted observation in Eigen
    ///
    /// @return Eigen Matrix
    // ============================================================================
    const Eigen::Matrix<double,6,1> & getZ() const;
    // ============================================================================
    /// @brief get innovation vector
    ///
    /// @return Eigen Matrix
    // ============================================================================
    const Eigen::Matrix<double,6,1> & getInnov() const;
    // ============================================================================
    /// @brief get process covariance
    ///
    /// @return Eigen Matrix
    // ============================================================================
    const Eigen::Matrix<double,6,6> & getQ() const; 
    // ============================================================================
    /// @brief get observation covariance
    ///
    /// @return Eigen Matrix
    // ============================================================================
    const Eigen::Matrix<double,6,6> & getR() const;  

    void computeKss(const Eigen::Matrix<double,6,6> &A, const Eigen::Matrix<double,6,6> &C, int zDim);
    // ============================================================================
    /// @brief Rodrigues formula, compute the exponential of a cross product matrix of a vector 
    ///  \f$ \exp{u^{\times}} = \frac{\sin(\alpha)}{\alpha} u^{\times} + \frac{\cos(\alpha)-1}{\alpha^2} u^{\times}^2 \f$  
    ///
    /// @param u input vector
    /// @param EA exponential of skew symmetric matrix (effectively a rotation matrix)
    // ============================================================================
    void RodriguesFormula (const Eigen::Matrix<double,3,1> & u, Eigen::Matrix<double,3,3> & EA); 

    double timestep;
    // These matrices are set from outside at first call
    /*Eigen::Matrix<double,6,6> DSc_Q; */
    /*Eigen::Matrix<double,6,6> DSc_R; */
    /*Eigen::Matrix<double,6,6> DSc_K; // Steady state kalman gain */


    //Eigen::Matrix<double,3,1> _pos; //  Position 
    double _mag1[3]; // Magnitude of innovation of fixed foot velocity
    double _mag2[3]; // Magnitude of innovation of fixed foot velocity  
    double _alpha_R; // Value used to scale mag1 and mag2

    double _alpha_x;// Modulate position innovation
    double _alpha_v;// Modulate velocity innovation
    Eigen::Matrix<double,6,1> _y;   
    Eigen::Matrix<double,12,1> _ytemp;   

  protected:
    bool buffer_first_time;
    bool contact_change_flag;

    Eigen::Matrix<double,6,1> _x;
    Eigen::Matrix<double,6,1> _z;
    Eigen::Matrix<double,6,1> _innov; 

    Eigen::Matrix<double,6,6> _P;
    Eigen::Matrix<double,6,6> _Q;
    Eigen::Matrix<double,6,6> _R;
    Eigen::Matrix<double,6,6> _K;
    Eigen::Matrix<double,6,6> _S;   

    Eigen::Matrix<double,6,6> _A;
    Eigen::Matrix<double,6,6> _C;
    Eigen::Matrix<double,6,1> _dx;
    static Eigen::Matrix<double,3,1> _gravity;          // g
    static Eigen::Matrix<double,3,1> _imu_lin_offset;   // IMU linear offset
    static Eigen::Quaterniond _imu_quat_offset;   // IMU quaternion
    //Eigen::Matrix<double,3,1> _a_proper_world;   // a - g 
    //Eigen::Matrix<double,3,1> _ra_g; // R*(a-g);  
    Eigen::Matrix<double,6,1> _innomag; 
    Eigen::Quaternion<double> _q;
    double _a[3]; // Acceleration as input from IMU ekf or raw data
    double _w[3]; // Angular velocity as input IMU ekf or raw data 
    Eigen::Matrix<double,3,1> _vimu;
    double _joints[N_JOINTS];
    double _jointsd[N_JOINTS];

    int _contact_state;
    int _contact_state_full;
    int _previous_contact_state; // Previous contact state 
    int _contact_timer;
    double _weight[LR]; // A weight stores contact point weight from back of foot to front of foot (0 is back of foot, 1 is front of foot)
    
    // TODO
    //RobotState rs;      
    //double sdFastState[PELV_N_SDFAST_STATE];
    RobotState *rs;
    double *sdFastState;
    int modelType;
    
    // index for each component
    static const int X0 = 0;
    static const int V0 = 3;
    //static const int PL = 6;
    //static const int PR = 9;
    //static const int Quat0 = 12;
    // Flag of contact state to avoid multiple set operations
    bool Dsc_flag;
    bool SSL_flag;
    bool SSR_flag;
    bool DSl_flag;
    bool DSr_flag;
    /*bool Lhs_flag;*/
    /*bool Rhs_flag;*/
    /*int _lhs_timer;*/
    /*int _rhs_timer;*/
    int zDim;
    // ComputeFootstuff
    Eigen::Matrix<double,3,1> foot_pos[LR]; // Contact point of foot
    Eigen::Matrix<double,3,1> foot_vel[LR]; // Velocity of contact point
    //double foot_heel_offset[XYZ]; // Heel offset
    double foot_contact_point_offset[LR][XYZ]; // Contact point offset
    //double toe_heel_offset[3];
    // Controller parameters
    double _ss_step_duration; // From wc planner
    double _ds_step_duration;
    // The following stuff is for foot buffer
    static const int buffer_size = 10; // Buffer size for measured foot position
    Eigen::Matrix<double,12,1> foot_pv; // Currently prediced foot position and velocity [pl,pr,vl,vr];
    Eigen::Matrix<double,12,buffer_size> foot_buffer; // foot buffer stores previous buffer_size predicted foot position and velocity
    Eigen::Matrix<double,12,1> foot_registered; // registered foot position
    int buffer_index;
    double _beta; // Weight distribution between left and right leg  

  private:
    
    void copy(const KinematicFilter3 & other); 
    // ============================================================================
    /// @brief DARE using Hamiltonian formulation, see http://web.ics.purdue.edu/~zheng33/p_Psoftware.html
    ///
    /// @param A state matrix
    /// @param B input matrix
    /// @param P covariance matrix ??
    /// @param zDim
    // ============================================================================
    void dare(const Eigen::Matrix<double,6,6> &A, const Eigen::Matrix<double,6,6> &B, Eigen::Matrix<double,6,6> &P, int zDim) ;
};

/****************************************************************************/
/*                   ERIC LADDER FILTER                                     */
/****************************************************************************/
class KinematicFilterEric 
{
  public:
    // ============================================================================
    /// @brief Constructor by copying stuff from other
    ///
    /// @param other
    // ===========================================================================
    KinematicFilterEric (const KinematicFilterEric & other);
    // ============================================================================
    /// @brief Copy operator
    ///
    /// @param other
    ///
    /// @return Class pointer
    // ============================================================================
    KinematicFilterEric & operator=(const KinematicFilterEric & other);

    // ============================================================================
    /// @brief Constructor, set P,Q,R to Identity
    // ============================================================================
    KinematicFilterEric();
    // ============================================================================
    /// @brief Default destructor
    // ============================================================================
    ~KinematicFilterEric();        

    // Takes angular velocity and linear acc as input
    void predictX();
    void updateZ(int zDim);
    void updateX(int zDim);
    // ============================================================================
    /// @brief compute innovation vector \f$ \Deta y = y - \hat{y}_{k+1}\f$
    ///
    /// @param z measurement 
    // ============================================================================
    void computeInnovation(const Eigen::Matrix<double, 6, 1> & z);

    void makeProcessJacobian ();

    void makeObservationJacobian ();
    // Ankle Related Stuff                        
    void computeAnkleRelated();
    // This function computes Kss once
    void setKss();

    // Steady state filter, call setKss first
    void filterOneTimeStep_ss();
    void getLinearV(double *v);
    void getPosition(double *x);
    void getFootPose(int side, double *x); //Measurement
    void makeInputs(double *q, double *w, double *a, double *joints, double *jointsd);
    void getContactState (int cs);
    // ROS
    void readParams(std::string &nameQ, std::string &name);
    void initKF(int type, double dt, const double root_pos[3], const double root_vel[3]);
    // ============================================================================
    /// @brief fill the measurement buffer.
    // ============================================================================
    void fill_measurement_buffer();
    // ============================================================================
    /// @brief compute foot position and velocity (used before makeMeasurement)
    // ============================================================================
    void computeFootPV();
    // ============================================================================
    /// @brief compute measurement. Measurement is the weighted sum of both feet (p,v) in previous steps.
    ///
    /// @param left_fz
    /// @param right_fz
    // ============================================================================
    void makeMeasurement(double left_fz, double right_fz);

    void set_hack_footpos();
    // ============================================================================
    /// @brief set internal state by double array x with length len
    ///
    /// @param x double array
    /// @param len int, array length
    // ============================================================================
    void setX (const double * x, int len);
    // ============================================================================
    /// @brief set internal state by Eigen matrix 
    ///
    /// @param x Eigen Column Matrix
    // ============================================================================
    void setX (const Eigen::Matrix<double, 6, 1> & x);
    // ============================================================================
    /// @brief set covariance
    ///
    /// @param P Eigen Matrix
    // ============================================================================
    void setP (const Eigen::Matrix<double, 6, 6> & P);
    // ============================================================================
    /// @brief set process covariance
    ///
    /// @param Q Eigen Matrix
    // ============================================================================
    void setQ (const Eigen::Matrix<double, 6, 6> & Q);
    // ============================================================================
    /// @brief set observation covariance
    ///
    /// @param R Eigen Matrix
    // ============================================================================
    void setR (const Eigen::Matrix<double, 6, 6> & R);
    // ============================================================================
    /// @brief get state and save it to x[len]
    ///
    /// @param x double array
    /// @param len length of x
    // ============================================================================
    void getX(double * x, int len);
   
    // ============================================================================
    /// @brief get state in Eigen
    ///
    /// @return Eigen Matrix
    // ============================================================================
    const Eigen::Matrix<double,6,1> & getX() const;             
    // ============================================================================
    /// @brief get predicted observation in Eigen
    ///
    /// @return Eigen Matrix
    // ============================================================================
    const Eigen::Matrix<double,6,1> & getZ() const;
    // ============================================================================
    /// @brief get innovation vector
    ///
    /// @return Eigen Matrix
    // ============================================================================
    const Eigen::Matrix<double,6,1> & getInnov() const;
    // ============================================================================
    /// @brief get process covariance
    ///
    /// @return Eigen Matrix
    // ============================================================================
    const Eigen::Matrix<double,6,6> & getQ() const; 
    // ============================================================================
    /// @brief get observation covariance
    ///
    /// @return Eigen Matrix
    // ============================================================================
    const Eigen::Matrix<double,6,6> & getR() const;  

    void computeKss(const Eigen::Matrix<double,6,6> &A, const Eigen::Matrix<double,6,6> &C, int zDim);
    // ============================================================================
    /// @brief Rodrigues formula, compute the exponential of a cross product matrix of a vector 
    ///  \f$ \exp{u^{\times}} = \frac{\sin(\alpha)}{\alpha} u^{\times} + \frac{\cos(\alpha)-1}{\alpha^2} u^{\times}^2 \f$  
    ///
    /// @param u input vector
    /// @param EA exponential of skew symmetric matrix (effectively a rotation matrix)
    // ============================================================================
    void RodriguesFormula (const Eigen::Matrix<double,3,1> & u, Eigen::Matrix<double,3,3> & EA); 

    double timestep;
    // These matrices are set from outside at first call
    Eigen::Matrix<double,6,6> DSc_Q; 
    Eigen::Matrix<double,6,6> DSc_R; 
    Eigen::Matrix<double,6,6> DSc_K; // Steady state kalman gain 


    //Eigen::Matrix<double,3,1> _pos; //  Position 
    double _mag1[3]; // Magnitude of innovation of fixed foot velocity
    double _mag2[3]; // Magnitude of innovation of fixed foot velocity  
    double _alpha_R; // Value used to scale mag1 and mag2

    double _alpha_x;// Modulate position innovation
    double _alpha_v;// Modulate velocity innovation
    Eigen::Matrix<double,6,1> _y;   
    Eigen::Matrix<double,12,1> _ytemp;   

  protected:
    bool buffer_first_time;
    bool contact_change_flag;

    Eigen::Matrix<double,6,1> _x;
    Eigen::Matrix<double,6,1> _z;
    Eigen::Matrix<double,6,1> _innov; 

    Eigen::Matrix<double,6,6> _P;
    Eigen::Matrix<double,6,6> _Q;
    Eigen::Matrix<double,6,6> _R;
    Eigen::Matrix<double,6,6> _K;
    Eigen::Matrix<double,6,6> _S;   

    Eigen::Matrix<double,6,6> _A;
    Eigen::Matrix<double,6,6> _C;
    Eigen::Matrix<double,6,1> _dx;
    static Eigen::Matrix<double,3,1> _gravity;          // g
    static Eigen::Matrix<double,3,1> _imu_lin_offset;   // IMU linear offset
    static Eigen::Quaterniond _imu_quat_offset;   // IMU quaternion
    //Eigen::Matrix<double,3,1> _a_proper_world;   // a - g 
    //Eigen::Matrix<double,3,1> _ra_g; // R*(a-g);  
    Eigen::Matrix<double,6,1> _innomag; 
    Eigen::Quaternion<double> _q;
    double _a[3]; // Acceleration as input from IMU ekf or raw data
    double _w[3]; // Angular velocity as input IMU ekf or raw data 
    Eigen::Matrix<double,3,1> _vimu;
    double _joints[N_JOINTS];
    double _jointsd[N_JOINTS];

    int _contact_state;
    int _contact_state_full;
    int _previous_contact_state; // Previous contact state 
    int _contact_timer;
    double _weight[LR]; // A weight stores contact point weight from back of foot to front of foot (0 is back of foot, 1 is front of foot)
    
    // TODO
    //RobotState rs;      
    //double sdFastState[PELV_N_SDFAST_STATE];
    RobotState *rs;
    double *sdFastState;
    int modelType;
    
    // index for each component
    static const int X0 = 0;
    static const int V0 = 3;
    //static const int PL = 6;
    //static const int PR = 9;
    //static const int Quat0 = 12;
    // Flag of contact state to avoid multiple set operations
    bool Dsc_flag;
    bool SSL_flag;
    bool SSR_flag;
    bool DSl_flag;
    bool DSr_flag;
    /*bool Lhs_flag;*/
    /*bool Rhs_flag;*/
    /*int _lhs_timer;*/
    /*int _rhs_timer;*/
    int zDim;
    // ComputeFootstuff
    Eigen::Matrix<double,3,1> foot_pos[LR]; // Contact point of foot
    Eigen::Matrix<double,3,1> foot_vel[LR]; // Velocity of contact point
    //double foot_heel_offset[XYZ]; // Heel offset
    double foot_contact_point_offset[LR][XYZ]; // Contact point offset
    //double toe_heel_offset[3];
    // Controller parameters
    double _ss_step_duration; // From wc planner
    double _ds_step_duration;
    // The following stuff is for foot buffer
    static const int buffer_size = 10; // Buffer size for measured foot position
    Eigen::Matrix<double,12,1> foot_pv; // Currently prediced foot position and velocity [pl,pr,vl,vr];
    Eigen::Matrix<double,12,buffer_size> foot_buffer; // foot buffer stores previous buffer_size predicted foot position and velocity
    Eigen::Matrix<double,12,1> foot_registered; // registered foot position
    int buffer_index;
    double _beta; // Weight distribution between left and right leg  

  private:
    
    void copy(const KinematicFilterEric & other); 
    // ============================================================================
    /// @brief DARE using Hamiltonian formulation, see http://web.ics.purdue.edu/~zheng33/p_Psoftware.html
    ///
    /// @param A state matrix
    /// @param B input matrix
    /// @param P covariance matrix ??
    /// @param zDim
    // ============================================================================
    void dare(const Eigen::Matrix<double,6,6> &A, const Eigen::Matrix<double,6,6> &B, Eigen::Matrix<double,6,6> &P, int zDim) ;
};
#endif /* KINEMATICFILTER_H */
