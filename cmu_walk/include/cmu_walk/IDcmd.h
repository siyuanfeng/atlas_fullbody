#ifndef IDCMD_INCLUDED
#define IDCMD_INCLUDED

#include "Foot.h"
#include "drc_common_defines.h"
#include "Logger.h"

enum HandContact {
	NO_HANDS = 0,
	LEFT_HAND,
	RIGHT_HAND,
	BOTH_HANDS
};

class IDcmd {
public:
	Foot::FootRefType footRef[LR];
	// sfeng
	double rootdd[3];
	//double root_b_wd[3];
	double jointsdd[N_JOINTS];
	double b_cop[LR][2];
	double w_cop[LR][2];
	bool rootMode[3];

	double heelSupport[2];
	double comdd[3], torsoWd[3], pelvWd[3];
	double footdd[2][6], handdd[2][6];
	double cop[2];
	double wl;
	int contactState;
	int handContact;
	double totMom[3];	//probably unused
	double magicTorqueGain, magicCopGain;
	bool skipHand[2];
	bool skipFoot[2];

	IDcmd() {
		footRef[LEFT] = footRef[RIGHT] = Foot::RefSensor;
		setZeros();
		heelSupport[LEFT] = heelSupport[RIGHT] = -1.0;
		skipHand[LEFT] = skipHand[RIGHT] = false;
	}

	void setZeros() {
		for(int i = 0; i < 3; i++) {
			comdd[i] = 0.0;
			torsoWd[i] = 0.0;
			pelvWd[i] = 0.0;
			totMom[i] = 0.0;
      rootMode[i] = false;
		}
    skipHand[LEFT] = skipHand[RIGHT] = false;
    skipFoot[LEFT] = skipFoot[RIGHT] = false;
		cop[0] = cop[1] = 0.0;
		wl = 0.5;
		for(int i = 0; i < 6; i++) {
			for(int j = 0; j < 2; j++) {
				footdd[j][i] = 0.0;
				handdd[j][i] = 0.0;
			}
		}
		contactState = DSc;
		handContact = NO_HANDS;
		magicTorqueGain = magicCopGain = 0.0;
	}

	void addToLog(BatchLogger &logger) {
		logger.add_datapoint("ID_d.footRef[L]","m", (const int *)&(footRef[LEFT]));
		logger.add_datapoint("ID_d.footRef[R]","m", (const int *)&(footRef[RIGHT]));
		
		logger.add_datapoint("ID_d.comdd[X]","a", &(comdd[XX]));
		logger.add_datapoint("ID_d.comdd[Y]","a", &(comdd[YY]));
		logger.add_datapoint("ID_d.comdd[Z]","a", &(comdd[ZZ]));

		logger.add_datapoint("ID_d.cop[X]","m", &(cop[XX]));
		logger.add_datapoint("ID_d.cop[Y]","m", &(cop[YY]));

		logger.add_datapoint("ID_d.b_cop[L][X]","m", &(b_cop[LEFT][XX]));
		logger.add_datapoint("ID_d.b_cop[L][Y]","m", &(b_cop[LEFT][YY]));
		logger.add_datapoint("ID_d.b_cop[R][X]","m", &(b_cop[RIGHT][XX]));
		logger.add_datapoint("ID_d.b_cop[R][Y]","m", &(b_cop[RIGHT][YY]));

		logger.add_datapoint("ID_d.wl","-", &wl);
		logger.add_datapoint("ID_d.contactState","-", &contactState);
		logger.add_datapoint("ID_d.handContact","-", &handContact);

		logger.add_datapoint("ID_d.pelvWd[X]","ra", &(pelvWd[XX]));
		logger.add_datapoint("ID_d.pelvWd[Y]","ra", &(pelvWd[YY]));
		logger.add_datapoint("ID_d.pelvWd[Z]","ra", &(pelvWd[ZZ]));

		logger.add_datapoint("ID_d.torsoWd[X]","ra", &(torsoWd[XX]));
		logger.add_datapoint("ID_d.torsoWd[Y]","ra", &(torsoWd[YY]));
		logger.add_datapoint("ID_d.torsoWd[Z]","ra", &(torsoWd[ZZ]));

		logger.add_datapoint("ID_d.ftdd[L][X]","a", &(footdd[LEFT][XX]));
		logger.add_datapoint("ID_d.ftdd[L][Y]","a", &(footdd[LEFT][YY]));
		logger.add_datapoint("ID_d.ftdd[L][Z]","a", &(footdd[LEFT][ZZ]));

		logger.add_datapoint("ID_d.ftdd[R][X]","a", &(footdd[RIGHT][XX]));
		logger.add_datapoint("ID_d.ftdd[R][Y]","a", &(footdd[RIGHT][YY]));
		logger.add_datapoint("ID_d.ftdd[R][Z]","a", &(footdd[RIGHT][ZZ]));

		logger.add_datapoint("ID_d.ftdd[L][AX]","ra", &(footdd[LEFT][3+XX]));
		logger.add_datapoint("ID_d.ftdd[L][AY]","ra", &(footdd[LEFT][3+YY]));
		logger.add_datapoint("ID_d.ftdd[L][AZ]","ra", &(footdd[LEFT][3+ZZ]));

		logger.add_datapoint("ID_d.ftdd[R][AX]","ra", &(footdd[RIGHT][3+XX]));
		logger.add_datapoint("ID_d.ftdd[R][AY]","ra", &(footdd[RIGHT][3+YY]));
		logger.add_datapoint("ID_d.ftdd[R][AZ]","ra", &(footdd[RIGHT][3+ZZ]));

		logger.add_datapoint("ID_d.hadd[L][X]","a", &(handdd[LEFT][XX]));
		logger.add_datapoint("ID_d.hadd[L][Y]","a", &(handdd[LEFT][YY]));
		logger.add_datapoint("ID_d.hadd[L][Z]","a", &(handdd[LEFT][ZZ]));

		logger.add_datapoint("ID_d.hadd[R][X]","a", &(handdd[RIGHT][XX]));
		logger.add_datapoint("ID_d.hadd[R][Y]","a", &(handdd[RIGHT][YY]));
		logger.add_datapoint("ID_d.hadd[R][Z]","a", &(handdd[RIGHT][ZZ]));

		logger.add_datapoint("ID_d.hadd[L][AX]","ra", &(handdd[LEFT][3+XX]));
		logger.add_datapoint("ID_d.hadd[L][AY]","ra", &(handdd[LEFT][3+YY]));
		logger.add_datapoint("ID_d.hadd[L][AZ]","ra", &(handdd[LEFT][3+ZZ]));

		logger.add_datapoint("ID_d.hadd[R][AX]","ra", &(handdd[RIGHT][3+XX]));
		logger.add_datapoint("ID_d.hadd[R][AY]","ra", &(handdd[RIGHT][3+YY]));
		logger.add_datapoint("ID_d.hadd[R][AZ]","ra", &(handdd[RIGHT][3+ZZ]));

		logger.add_datapoint("ID_d.heelSup[L]","-", &(heelSupport[LEFT]));
		logger.add_datapoint("ID_d.heelSup[R]","-", &(heelSupport[RIGHT]));
	}
};

#endif
