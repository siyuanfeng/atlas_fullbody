#ifndef TRAJ_DEFINED
#define TRAJ_DEFINED
#include <iostream>

enum SplineType {
	Linear,
	Cubic,
	Quintic,
	Blank
};

class Knot {
	public:
		Knot();
		Knot(SplineType inType, double t, double p, double v, double a);
		void print();
		void extrapolate(double t, double *p, double *v, double *a);

		SplineType type;
		double time;
		double pos;
		double vel;
		double acc;
		Knot *prev;
		Knot *next;
};

class TrajEW {
	public:
		TrajEW();
		TrajEW(SplineType overRide);
		void addKnot(SplineType type, double t, double p, double v=0, double a=0);
		Knot *addKnot(double t, double p, double v, double a);
		void addMove(double t0, double t, double p, double v, double a);
		void addMove(double t0, double t, double p);
		void addRelMove(double t0, double t, double dp);
		void addKnot(double t, double p, double v);
		void addKnot(double t, double p);
		void insert(Knot *k);
		void print();
		void clear();
		void read(double t, double *p, double *v=NULL, double *a=NULL);
		double readPos(double t);
		void resetInitPos(double p);
		void resetInitPos(double p, int N);
		void freshMove(double start, double end, double T);
		void freshMove(double start, double end, double T, SplineType type);
		void begin(double start);
		Knot *getLastKnot();
		void adjustFinalPos(double pf);
		void adjustFinalVel(double vf);
		void setConstant(double p);

		static double quinticPos(double t);

		Knot *start;
		int N_knots;
		SplineType overRideType;
};


#endif
