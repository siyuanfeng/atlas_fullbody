/*
Generated 01-Apr-2003 04:20:00 by SD/FAST, Kane's formulation
(sdfast B.2.8 #30123) on machine ID unknown
Copyright (c) 1990-1997 Symbolic Dynamics, Inc.
Copyright (c) 1990-1997 Parametric Technology Corp.
RESTRICTED RIGHTS LEGEND: Use, duplication, or disclosure by the U.S.
Government is subject to restrictions as set forth in subparagraph
(c)(1)(ii) of the Rights in Technical Data and Computer Software
clause at DFARS 52.227-7013 and similar clauses in the FAR and NASA
FAR Supplement.  Symbolic Dynamics, Inc., Mountain View, CA 94041
*/
#include <math.h>

/* These routines are passed to sdroot. */

void sdposfunc(double vars[21],
    double param[1],
    double resid[21])
{
    int i;
    double pos[22],vel[21];

    for (i = 0; i < 21; i++) {
        vel[i] = 0.;
    }
    sdang2st(vars,pos);
    sdstate(param[0],pos,vel);
    sdumotion(param[0],pos,vel);
    sdperr(resid);
}

void sdvelfunc(double vars[21],
    double param[23],
    double resid[21])
{

    sdstate(param[22],param,vars);
    sdumotion(param[22],param,vars);
    sdverr(resid);
}

void sdstatfunc(double vars[21],
    double param[22],
    double resid[42])
{
    double pos[22],qdotdum[22];

    sdang2st(vars,pos);
    sdstate(param[21],pos,param);
    sdumotion(param[21],pos,param);
    sduforce(param[21],pos,param);
    sdperr(resid);
    sdderiv(qdotdum,&resid[21]);
}

void sdstdyfunc(double vars[42],
    double param[1],
    double resid[63])
{
    double pos[22],qdotdum[22];

    sdang2st(vars,pos);
    sdstate(param[0],pos,&vars[21]);
    sdumotion(param[0],pos,&vars[21]);
    sduforce(param[0],pos,&vars[21]);
    sdperr(resid);
    sdverr(&resid[21]);
    sdderiv(qdotdum,&resid[42]);
}

/* This routine is passed to the integrator. */

void sdmotfunc(double time,
    double state[43],
    double dstate[43],
    double param[1],
    int *status)
{
    double err[21];
    int i;

    sdstate(time,state,&state[22]);
    sdumotion(time,state,&state[22]);
    sduforce(time,state,&state[22]);
    sdderiv(dstate,&dstate[22]);
    *status = 1;
    sdverr(err);
    for (i = 0; i < 21; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    sdperr(err);
    for (i = 0; i < 21; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    *status = 0;
}

/* This routine performs assembly analysis. */

void sdassemble(double time,
    double state[43],
    int lock[21],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double perrs[21],param[1];
    int i;
    double jw[441],dw[3528],rw[336];
    int iw[168],rooterr;

    sdgentime(&i);
    if (i != 42000) {
        sdseterr(50,42);
    }
    param[0] = time;
    sdst2ang(state,state);
    sdroot(sdposfunc,state,param,21,21,0,lock,tol,tol,maxevals,
      jw,dw,rw,iw,perrs,fcnt,&rooterr);
    sdposfunc(state,param,perrs);
    *fcnt = *fcnt+1;
    sdang2st(state,state);
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs initial velocity analysis. */

void sdinitvel(double time,
    double state[43],
    int lock[21],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double verrs[21],param[23];
    int i;
    double jw[441],dw[3528],rw[336];
    int iw[168],rooterr;

    sdgentime(&i);
    if (i != 42000) {
        sdseterr(51,42);
    }
    for (i = 0; i < 22; i++) {
        param[i] = state[i];
    }
    param[22] = time;
    sdroot(sdvelfunc,&state[22],param,21,21,0,lock,tol,tol,maxevals,
      jw,dw,rw,iw,verrs,fcnt,&rooterr);
    sdvelfunc(&state[22],param,verrs);
    *fcnt = *fcnt+1;
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs static analysis. */

void sdstatic(double time,
    double state[43],
    int lock[21],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[42],param[22],jw[882],dw[7938],rw[483];
    int iw[252],rooterr,i;

    sdgentime(&i);
    if (i != 42000) {
        sdseterr(52,42);
    }
    for (i = 0; i < 21; i++) {
        param[i] = state[22+i];
    }
    param[21] = time;
    sdst2ang(state,state);
    sdroot(sdstatfunc,state,param,42,21,21,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    sdstatfunc(state,param,resid);
    *fcnt = *fcnt+1;
    sdang2st(state,state);
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs steady motion analysis. */

void sdsteady(double time,
    double state[43],
    int lock[42],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[63],param[1],vars[42];
    double jw[2646],dw[22050],rw[819];
    int iw[420],rooterr,i;

    sdgentime(&i);
    if (i != 42000) {
        sdseterr(53,42);
    }
    param[0] = time;
    sdst2ang(state,vars);
    for (i = 0; i < 21; i++) {
        vars[21+i] = state[22+i];
    }
    sdroot(sdstdyfunc,vars,param,63,42,21,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    sdstdyfunc(vars,param,resid);
    *fcnt = *fcnt+1;
    sdang2st(vars,state);
    for (i = 0; i < 21; i++) {
        state[22+i] = vars[21+i];
    }
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs state integration. */

void sdmotion(double *time,
    double state[43],
    double dstate[43],
    double dt,
    double ctol,
    double tol,
    int *flag,
    int *err)
{
    static double step;
    double work[258],ttime,param[1];
    int vintgerr,which,ferr,i;

    sdgentime(&i);
    if (i != 42000) {
        sdseterr(54,42);
    }
    param[0] = ctol;
    ttime = *time;
    if (*flag != 0) {
        sdmotfunc(ttime,state,dstate,param,&ferr);
        step = dt;
        *flag = 0;
    }
    if (step <= 0.) {
        step = dt;
    }
    sdvinteg(sdmotfunc,&ttime,state,dstate,param,dt,&step,43,tol,work,&vintgerr,
      &which);
    *time = ttime;
    *err = vintgerr;
}

/* This routine performs state integration with a fixed-step integrator. */

void sdfmotion(double *time,
    double state[43],
    double dstate[43],
    double dt,
    double ctol,
    int *flag,
    double *errest,
    int *err)
{
    double work[172],ttime,param[1];
    int ferr,i;

    sdgentime(&i);
    if (i != 42000) {
        sdseterr(55,42);
    }
    param[0] = ctol;
    *err = 0;
    ttime = *time;
    if (*flag != 0) {
        sdmotfunc(ttime,state,dstate,param,&ferr);
        *flag = 0;
    }
    sdfinteg(sdmotfunc,&ttime,state,dstate,param,dt,43,work,errest,&ferr);
    if (ferr != 0) {
        *err = 1;
    }
    *time = ttime;
}
