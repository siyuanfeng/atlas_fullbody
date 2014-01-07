#include "TrajEW.h"
#include <stdio.h>
#include <iostream>
#include <algorithm>

Knot::Knot() {
	type = Linear;
	time = pos = vel = acc = 0.0;
	prev = next = NULL;
}

Knot::Knot(SplineType inType, double t, double p, double v, double a) {
	type = inType;
	time = t;
	pos = p;
	vel = v;
	acc = a;
	prev = next = NULL;
}

void Knot::print() {
	char output[100];
	sprintf(output, "Type:%d, t:%g, p: %g, v: %g, a:%g\n",type, time, pos, vel, acc);
	std::cout << output;
}

void Knot::extrapolate(double t, double *p, double *v, double *a) {
	if(type == Linear) {
		if(p)	*p = pos;
		if(v)	*v = 0.0;
		if(a)	*a = 0.0;
	}
	else if(type == Cubic) {
		if(p)	*p = pos + vel*(t-time);
		if(v)	*v = vel;
		if(a)	*a = 0.0;
	}
	else if(type == Quintic) {
		double dt = t-time;
		if(p)	*p = pos + vel*dt + acc/2.0*dt*dt;
		if(v)	*v = vel + acc*dt;
		if(a)	*a = acc;
	}
	else {
		std::cout << "Bad Spline Type\n";
	}
}

TrajEW::TrajEW() {
	start = NULL;
	N_knots = 0;
	overRideType = Blank;
}

TrajEW::TrajEW(SplineType overRide) {
	start = NULL;
	N_knots = 0;
	overRideType = overRide;
}


void TrajEW::resetInitPos(double p) {
	if(!start)	printf("Uninitialized RIP\n");
	else {
		start->pos = p;
	}
}

void TrajEW::resetInitPos(double p, int N) {
	if(!start) {
		printf("Uninitialized RIPN\n");
		return;
	}
	start->pos = p;

	Knot *k = start;
	for(int i = 0; i < N-1; i++) {
		if(!k->next) {
			printf("No knot to reset\n");
			return;
		}
		k = k->next;
		k->pos = p;
	}
}

void TrajEW::freshMove(double start, double end, double T, SplineType type) {
	clear();
	overRideType = type;
	addKnot(0, start, 0, 0);
	addKnot(T, end, 0, 0);
}

void TrajEW::freshMove(double start, double end, double T) {
	clear();
	addKnot(0, start, 0, 0);
	addKnot(T, end, 0, 0);
}

void TrajEW::begin(double start) {
	clear();
	addKnot(0, start, 0, 0);
}

void TrajEW::addKnot(SplineType type, double t, double p, double v, double a) {
	Knot *k = new Knot(type, t, p, v, a);
	insert(k);
}

Knot *TrajEW::addKnot(double t, double p, double v, double a) {
	Knot *k = new Knot(Quintic, t, p, v, a);
	insert(k);
	return k;
}

void TrajEW::addMove(double t0, double t, double p, double v, double a) {
	double pp, pv, pa;
	read(t0, &pp, &pv, &pa);
	Knot *k0 = new Knot(Quintic, t0, pp, pv, pa);
	insert(k0);

	Knot *k = new Knot(Quintic, t, p, v, a);
	insert(k);
}

void TrajEW::addMove(double t0, double t, double p) {
	addMove(t0, t, p, 0.0, 0.0);
}

void TrajEW::addRelMove(double t0, double t, double dp) {
	double pp, pv, pa;
	read(t0, &pp, &pv, &pa);
	Knot *k0 = new Knot(Quintic, t0, pp, pv, pa);
	insert(k0);

	Knot *k = new Knot(Quintic, t, pp+dp, 0.0, 0.0);
	insert(k);
}

void TrajEW::addKnot(double t, double p, double v) {
	Knot *k = new Knot(Cubic, t, p, v, 0.0);
	insert(k);
}

void TrajEW::addKnot(double t, double p) {
	Knot *k = new Knot(Linear, t, p, 0.0, 0.0);
	insert(k);
}

Knot *TrajEW::getLastKnot() {
	if(!start)	return NULL;

	Knot *k = start;
	while(k->next)		k = k->next;

	return k;
}

void TrajEW::adjustFinalPos(double pf) {
	Knot *k = getLastKnot();
	k->pos = pf;
}

void TrajEW::adjustFinalVel(double vf) {
	Knot *k = getLastKnot();
	k->vel = vf;
}

void TrajEW::insert(Knot *k) {
	N_knots++;
	if(N_knots == 1) {	//if only knot
		start = k;
		return;
	}
	if(start->time > k->time) {	//insert at beginning
		k->next = start;
		start = k;
		return;
	}
	Knot *insertAfter = start;
	while(insertAfter->next) {
		if(insertAfter->next->time > k->time) {	//insert here
			insertAfter->next->prev = k;
			k->next = insertAfter->next;
			k->prev = insertAfter;
			insertAfter->next = k;
			return;
		}
		insertAfter = insertAfter->next;
	}

	//is last node
	insertAfter->next = k;
	k->prev = insertAfter;
}

void TrajEW::print() {
	Knot *k = start;
	while(k) {
		k->print();
		k = k->next;
	}
}

void TrajEW::clear() {
	Knot *k = start;
	while(k) {
		Knot *k2 = k;
		k = k->next;
		delete k2;
	}
	N_knots = 0;
	start = NULL;
	overRideType = Blank;
}

void readCubic( double t, Knot *k0, Knot *k1, double *pos, double *vel, double *acc )
{
	double a, b, c, d;
	double t1, t2, t3;
	double nv0, nv1, np0, np1;


	np0 = k0->pos;
	np1 = k1->pos;
	nv0 = k0->vel*(k1->time - k0->time);
	nv1 = k1->vel*(k1->time - k0->time);

	d = np0;
	c = nv0;
	b = 3*(np1-np0) - nv1 - 2*nv0;
	a = nv1 + nv0 - 2*(np1-np0);
	t1 = (t - k0->time)/(k1->time - k0->time);
	t2 = t1*t1;
	t3 = t2*t1;
	if(pos)		*pos = a*t3 + b*t2 + c*t1 + d;
	if(vel)		*vel = (3*a*t2 + 2*b*t1 + c)/(k1->time - k0->time);
	if(acc)		*acc = (6*a*t1 + 2*b)/((k1->time - k0->time)*(k1->time - k0->time));
}

void readLinear(double t, Knot *k0, Knot *k1, double *pos, double *vel, double *acc) {
	double t1;
	t1 = (t - k0->time)/(k1->time - k0->time);
	if(pos)		*pos = k0->pos + (k1->pos - k0->pos)*t1;
	if(vel)		*vel = (k1->pos - k0->pos)/(k1->time - k0->time);
	if(acc)		*acc = 0;
}

double TrajEW::quinticPos(double t) {
	double t3 = t*t*t;
	double t4 = t3*t;
	double t5 = t4*t;
	return 6.0*t5 - 15.0*t4 + 10.0*t3;
}

void readQuintic(double t, Knot *k0, Knot *k1, double *pos, double *vel, double *acc) {
	double a, b, c, d, e, f;
	double tmp1, tmp2, tmp3;
	double t1, t2, t3, t4, t5;
	double nv0, nv1, na0, na1;

	nv0 = k0->vel*(k1->time - k0->time);
	nv1 = k1->vel*(k1->time - k0->time);
	na0 = k0->acc*(k1->time - k0->time)*(k1->time - k0->time);
	na1 = k1->acc*(k1->time - k0->time)*(k1->time - k0->time);

	f = k0->pos;
	e = nv0;
	d = na0/2;
	tmp1 = na0/2 + nv0 + k0->pos - k1->pos;
	tmp2 = na0 + nv0 - nv1;
	tmp3 = na0 - na1;
	a = - ( tmp3 - 6*tmp2 + 12*tmp1 )/2;
	b = - ( 8*a + 4*tmp2 - 12*tmp1 )/4;
	c = - ( a + b + tmp1 );
	t1 = (t - k0->time)/(k1->time - k0->time);
	t2 = t1*t1;
	t3 = t2*t1;
	t4 = t3*t1;
	t5 = t4*t1;
	if(pos)		*pos = a*t5 + b*t4 + c*t3 + d*t2 + e*t1 + f;
	if(vel)		*vel = (5*a*t4 + 4*b*t3 + 3*c*t2 + 2*d*t1 + e)  /  (k1->time - k0->time);
	if(acc)		*acc = (20*a*t3 + 12*b*t2 + 6*c*t1 + 2*d)  /  ((k1->time - k0->time)*(k1->time - k0->time));
}

double TrajEW::readPos(double t) {
	double p;
	read(t,&p);
	return p;
}

void TrajEW::setConstant(double p) {
	clear();
	addKnot(0,p);
}


void TrajEW::read(double t, double *p, double *v, double *a) {
	double pos = 0.0, vel = 0.0, acc = 0.0;
	if(N_knots > 0) {	//if there are any knots
		if(t < start->time) {	//if before first knot
			start->extrapolate(t, p, v, a);
			return;
		}
		Knot *before = start;
		Knot *after = start->next;
		while(true) {
			if(!after) {
				before->extrapolate(t, p, v, a);
				return;
			}
			if(t < after->time) {	//if between before and after
				SplineType readType = std::max(before->type, after->type);
				if(overRideType != Blank)		readType = overRideType;
				if(readType == Linear) {
					readLinear(t, before, after, p, v, a);
					return;
				}
				if(readType == Cubic) {
					readCubic(t, before, after, p, v, a);
					return;
				}
				if(readType == Quintic) {
					readQuintic(t, before, after, p, v, a);
					return;
				}
				std::cout << "Bad Spline Type\n";
			}
			before = after;
			after = after->next;
		}

	}
}
