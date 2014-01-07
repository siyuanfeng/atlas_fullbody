#ifndef ID_CONTROLLER_H
#define ID_CONTROLLER_H

#include <eigen3/Eigen/Core>

#include "IDcmd.h"
#include "RobotState.h"
#include "Command.h"

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <algorithm>

#define ID_MAGIC_INT_OFF      (0)
#define ID_MAGIC_INT_TORQUE   (1)
#define ID_MAGIC_INT_FORCE    (1 << 1)
#define ID_MAGIC_INT_COP      (1 << 2)
#define ID_MAGIC_INT_T_TORQUE (1 << 3)
#define ID_MAGIC_INT_T_FORCE  (1 << 4)

#define ID_IS_MAGIC_INT_TORQUE_ON(x)     ((ID_MAGIC_INT_TORQUE) & (x))
#define ID_IS_MAGIC_INT_FORCE_ON(x)      (((ID_MAGIC_INT_FORCE) & (x)) >> 1)
#define ID_IS_MAGIC_INT_COP_ON(x)        (((ID_MAGIC_INT_COP) & (x)) >> 2)
#define ID_IS_MAGIC_INT_T_TORQUE_ON(x)   (((ID_MAGIC_INT_T_TORQUE) & (x)) >> 3)
#define ID_IS_MAGIC_INT_T_FORCE_ON(x)    (((ID_MAGIC_INT_T_FORCE) & (x)) >> 4)

class IdController {
protected:  
	Eigen::Matrix3d makeCrossMatrix(const Eigen::Vector3d & v)
	{
		return (Eigen::Matrix3d() << 0, -v(2), v(1),
				v(2), 0, -v(0),
				-v(1), v(0), 0).finished();
	}

	// idx is in pelv order.
	double getJointLimitAcc(int idx, double ang)
	{
		double range = RobotState::jointsLimit[1][idx]-RobotState::jointsLimit[0][idx];
		//double offset = (RobotState::jointsLimit[1][idx]+RobotState::jointsLimit[0][idx])/2.;

		// ((x-l)/(u-l) - 0.5)*pi
		double x = ((ang - RobotState::jointsLimit[0][idx]) / range - 0.5) * M_PI;
		x = std::max(x, -M_PI/2.+0.1);
		x = std::min(x, M_PI/2.-0.1);
		return -tan(x) * 10;
		//return -pow(tan(x), 3) / 2;
	}

public:
	RobotState::ModelType _type;

	//for recording
	double comdd[3], pelvdd[3], torsodd[3], footdd[2][6], handdd[2][6], Qdd[34];
	double QPval;
	double handForce[2][3];
	double footForce[2][6];
	double cop[2], foot_b_cop[LR][2];
	double magicFT[6], magicCop[2];

	int handFricMode;

	//fix QDD
	bool fixQDDmask[MAX_N_SDFAST_U];
	double fixQdd[MAX_N_SDFAST_U];

	static const int MAX_FORCES = 12;

	// increase when adding more terms or variables to objective function
	static const int MAX_ROWS = 400;
	static const int MAX_VARS = MAX_N_SDFAST_U + N_JOINTS + MAX_FORCES+6;

	Eigen::Matrix<double,MAX_FORCES,1> _forceOld;
	Eigen::Matrix<double,N_JOINTS,1> _torqueOld;

	Eigen::Matrix<double,MAX_ROWS,MAX_VARS> _A;
	Eigen::Matrix<double,MAX_ROWS,1> _b;
	Eigen::Matrix<double,MAX_VARS,1> _X;

	Eigen::Matrix<double,MAX_N_SDFAST_U,MAX_VARS> _AD;

	Eigen::Matrix<double,6,1> _magicFT;
	Eigen::Matrix<double,2,1> _magicCop;
	Eigen::Matrix<double,6,1> _magicFTtorso;

	// foot rotation matrix
	Eigen::Matrix<double,12,12> _Rot;

	// very conservative
	Eigen::Matrix<double,MAX_ROWS,MAX_FORCES> _AF;


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	int integrateMagicMask;
	std::map <const std::string, double *> lookup;
  double offseted_com[3];

	//////////////////////////////////////////////////////
	// parameters
	double MAGIC_TORSO_TRQ_GAIN;
	double MAGIC_TRQ_GAIN;
	double MAGIC_F_GAIN;
	double MAGIC_COP_GAIN;

	double ROOTDD_REG_WEIGHT;
	double FOOT_COP_WEIGHT;
	double COM_F_WEIGHT;
	double COM_TAU_WEIGHT;

	double QDD_REG_WEIGHT;
	double TAU_REG_WEIGHT;
	double DTAU_REG_WEIGHT;
	double F_REG_WEIGHT;
	double FH_REG_WEIGHT;
	double DF_REG_WEIGHT;
	double COMDD_XY_WEIGHT;
	double COMDD_Z_WEIGHT;
	double ANGDD_T_WEIGHT;
	double ANGDD_P_WEIGHT;
	double FOOTDD_WEIGHT;
	double HANDDD_WEIGHT;
	double WL_WEIGHT;
	double COP_WEIGHT;

	double SWING_MULT_WEIGHT;
	double FIX_QDD_WEIGHT;
	double QD_JD_USE_FRAC;

	bool COM_BY_ACCEL;
	double FRICTION_COEF;
	double HAND_MU;

	double cba;
	//////////////////////////////////////////////////////

	double handWeightR, handWeightP;
	double copWeight;

	IdController() :
		MAGIC_TORSO_TRQ_GAIN(0),
		MAGIC_TRQ_GAIN(0),
		MAGIC_F_GAIN(0),
		MAGIC_COP_GAIN(0),

		ROOTDD_REG_WEIGHT(0),
		FOOT_COP_WEIGHT(0),
		COM_F_WEIGHT(0),
		COM_TAU_WEIGHT(0),

		QDD_REG_WEIGHT(1e-4),
		TAU_REG_WEIGHT(1e-4),
		DTAU_REG_WEIGHT(0.0),
		F_REG_WEIGHT(1e-6),
		FH_REG_WEIGHT(1e-6),
		DF_REG_WEIGHT(0.0),
		COMDD_XY_WEIGHT(3e-3),
		COMDD_Z_WEIGHT(3e-2),
		ANGDD_T_WEIGHT(1e-1),
		ANGDD_P_WEIGHT(1e-1),
		FOOTDD_WEIGHT(1e0),
		HANDDD_WEIGHT(1e0),
		WL_WEIGHT(1e-1),
		COP_WEIGHT(1e-3),

		SWING_MULT_WEIGHT(2.0),
		FIX_QDD_WEIGHT(3.0),
		QD_JD_USE_FRAC(0.0),

		COM_BY_ACCEL(false),
		FRICTION_COEF(2.0),
		HAND_MU(0.5),
		cba(0)
	{
    offseted_com[0] = 0;
    offseted_com[1] = 0;
    offseted_com[2] = 0;

		_torqueOld.setZero();
		_forceOld.setZero();
		_magicFT.setZero();
		_magicCop.setZero();
		_magicFTtorso.setZero();

		handFricMode = 0;
		QPval = 0;
		restoreDefaults();

		lookup["MAGIC_TORSO_TRQ_GAIN"] = &MAGIC_TORSO_TRQ_GAIN;
		lookup["MAGIC_TRQ_GAIN"] = &MAGIC_TRQ_GAIN;
		lookup["MAGIC_F_GAIN"] = &MAGIC_F_GAIN;
		lookup["MAGIC_COP_GAIN"] = &MAGIC_COP_GAIN;
		lookup["cba"] = &cba;

		lookup["ROOTDD_REG_WEIGHT"] = &ROOTDD_REG_WEIGHT;
		lookup["FOOT_COP_WEIGHT"] = &FOOT_COP_WEIGHT;
		lookup["COM_F_WEIGHT"] = &COM_F_WEIGHT;
		lookup["COM_TAU_WEIGHT"] = &COM_TAU_WEIGHT;

		lookup["QDD_REG_WEIGHT"] = &QDD_REG_WEIGHT;
		lookup["TAU_REG_WEIGHT"] = &TAU_REG_WEIGHT;
		lookup["DTAU_REG_WEIGHT"] = &DTAU_REG_WEIGHT;
		lookup["F_REG_WEIGHT"] = &F_REG_WEIGHT;
		lookup["FH_REG_WEIGHT"] = &FH_REG_WEIGHT;
		lookup["DF_REG_WEIGHT"] = &DF_REG_WEIGHT;
		lookup["COMDD_XY_WEIGHT"] = &COMDD_XY_WEIGHT;
		lookup["COMDD_Z_WEIGHT"] = &COMDD_Z_WEIGHT;
		lookup["ANGDD_T_WEIGHT"] = &ANGDD_T_WEIGHT;
		lookup["ANGDD_P_WEIGHT"] = &ANGDD_P_WEIGHT;
		lookup["FOOTDD_WEIGHT"] = &FOOTDD_WEIGHT;
		lookup["HANDDD_WEIGHT"] = &HANDDD_WEIGHT;
		lookup["WL_WEIGHT"] = &WL_WEIGHT;
		lookup["COP_WEIGHT"] = &COP_WEIGHT;

		lookup["SWING_MULT_WEIGHT"] = &SWING_MULT_WEIGHT;
		lookup["FIX_QDD_WEIGHT"] = &FIX_QDD_WEIGHT;
		lookup["QD_JD_USE_FRAC"] = &QD_JD_USE_FRAC;

		lookup["COM_BY_ACCEL"] = &cba;
		lookup["FRICTION_COEF"] = &FRICTION_COEF;
		lookup["HAND_MU"] = &HAND_MU;
	}

	~IdController() {;}
	RobotState::ModelType getType() const { return _type; }
	virtual bool ID(const RobotState &rs, const IDcmd &cmd, double torque[N_JOINTS]) =0;
	virtual bool ID(const RobotState &rs, const IDcmd &cmd, Command &out) =0;
	virtual void addToLog(BatchLogger &logger) =0;

	void restoreDefaults() {
		handWeightR = HANDDD_WEIGHT;
		handWeightP = HANDDD_WEIGHT;
		copWeight = COP_WEIGHT;
		for(int i = 0; i < MAX_N_SDFAST_U; i++)		fixQDDmask[i] = false;

		//fix neck (it is NOT a flywheel)
		fixQDDmask[3+6] = true;
		fixQdd[3+6] = 0.0;
	}

	void printParams()
	{
		std::cout << "===================================\n" << "idCon params\n";
		for (std::map<const std::string, double *>::iterator it = lookup.begin(); it != lookup.end(); it++)
			std::cout << it->first << ": " << *(it->second) << std::endl;
		std::cout << "===================================\n";
	}

	bool readParams(std::ifstream &in)
	{
		if (!in.good())
			return false;

		std::string name;
		std::map<const std::string, double *>::iterator res;
		double val;
		bool ret = true;
		while (true)
		{
			in >> name;
			if (in.eof())
				break;

			res = lookup.find(name);
			// can't find item
			if (res == lookup.end()) {
				std::cerr << "unknown param: " << name << " aborting." << std::endl;
				ret = false;
				break;
			}

			in >> val;
			*(res->second) = val;
			//std::cerr << "read " << name << " = " << val << std::endl;
		}

		if(cba > 0.5)		COM_BY_ACCEL = true;
		else			    COM_BY_ACCEL = false;

		printParams();

		return ret;
	}
};

/////////////////////////////////////////////////////////////
// eric land
class PelvIdCon : public IdController 
{
private:

public:
	PelvIdCon() { _type = RobotState::TYPE_PELVIS; }
	~PelvIdCon() {;}
	bool ID(const RobotState &rs, const IDcmd &cmd, double *torque = NULL);
	bool ID(const RobotState &rs, const IDcmd &cmd, Command &out) { fprintf(stderr, "ID unimplemented.\n"); return false; }
	void getCommand(double *torque);
	void setCommand(const double *torque, const double *F = NULL);
	void addToLog(BatchLogger &logger);
	double getWl();
	double getCoPx(int side);
	double getFz(int side);
};

/////////////////////////////////////////////////////////////
// siyuan land
class SFPelvIdCon : public IdController 
{
protected:

public:
	SFPelvIdCon() { _type = RobotState::TYPE_PELVIS; }
	~SFPelvIdCon() {;}
	bool ID(const RobotState &rs, const IDcmd &cmd, Command &out);
	bool ID(const RobotState &rs, const IDcmd &cmd, double *torque = NULL) { fprintf(stderr, "ID unimplemented.\n"); return false; }
	void addToLog(BatchLogger &logger);
};

class HatIdCon : public IdController 
{
private:

public:
	HatIdCon() { _type = RobotState::TYPE_HAT; }
	~HatIdCon() {;}
	void addToLog(BatchLogger &logger);
	bool ID(const RobotState &rs, const IDcmd &cmd, Command &out);
	bool ID(const RobotState &rs, const IDcmd &cmd, double *torque = NULL) { fprintf(stderr, "ID unimplemented.\n"); return false; }
};

class HatNSIdCon : public IdController 
{
private:

public:
	HatNSIdCon() { _type = RobotState::TYPE_HAT_NS; }
	~HatNSIdCon() {;}
	void addToLog(BatchLogger &logger);
	bool ID(const RobotState &rs, const IDcmd &cmd, Command &out);
	bool ID(const RobotState &rs, const IDcmd &cmd, double *torque = NULL) { fprintf(stderr, "ID unimplemented.\n"); return false; }
};

#endif
