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

void sdposfunc(double vars[14],
    double param[1],
    double resid[14])
{
    int i;
    double pos[14],vel[14];

    for (i = 0; i < 14; i++) {
        vel[i] = 0.;
    }
    sdang2st(vars,pos);
    sdstate(param[0],pos,vel);
    sdumotion(param[0],pos,vel);
    sdperr(resid);
}

void sdvelfunc(double vars[14],
    double param[15],
    double resid[14])
{

    sdstate(param[14],param,vars);
    sdumotion(param[14],param,vars);
    sdverr(resid);
}

void sdstatfunc(double vars[14],
    double param[15],
    double resid[28])
{
    double pos[14],qdotdum[14];

    sdang2st(vars,pos);
    sdstate(param[14],pos,param);
    sdumotion(param[14],pos,param);
    sduforce(param[14],pos,param);
    sdperr(resid);
    sdderiv(qdotdum,&resid[14]);
}

void sdstdyfunc(double vars[28],
    double param[1],
    double resid[42])
{
    double pos[14],qdotdum[14];

    sdang2st(vars,pos);
    sdstate(param[0],pos,&vars[14]);
    sdumotion(param[0],pos,&vars[14]);
    sduforce(param[0],pos,&vars[14]);
    sdperr(resid);
    sdverr(&resid[14]);
    sdderiv(qdotdum,&resid[28]);
}

/* This routine is passed to the integrator. */

void sdmotfunc(double time,
    double state[28],
    double dstate[28],
    double param[1],
    int *status)
{
    double err[14];
    int i;

    sdstate(time,state,&state[14]);
    sdumotion(time,state,&state[14]);
    sduforce(time,state,&state[14]);
    sdderiv(dstate,&dstate[14]);
    *status = 1;
    sdverr(err);
    for (i = 0; i < 14; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    sdperr(err);
    for (i = 0; i < 14; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    *status = 0;
}

/* This routine performs assembly analysis. */

void sdassemble(double time,
    double state[28],
    int lock[14],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double perrs[14],param[1];
    int i;
    double jw[196],dw[1568],rw[224];
    int iw[112],rooterr;

    sdgentime(&i);
    if (i != 42000) {
        sdseterr(50,42);
    }
    param[0] = time;
    sdroot(sdposfunc,state,param,14,14,0,lock,tol,tol,maxevals,
      jw,dw,rw,iw,perrs,fcnt,&rooterr);
    sdposfunc(state,param,perrs);
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

/* This routine performs initial velocity analysis. */

void sdinitvel(double time,
    double state[28],
    int lock[14],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double verrs[14],param[15];
    int i;
    double jw[196],dw[1568],rw[224];
    int iw[112],rooterr;

    sdgentime(&i);
    if (i != 42000) {
        sdseterr(51,42);
    }
    for (i = 0; i < 14; i++) {
        param[i] = state[i];
    }
    param[14] = time;
    sdroot(sdvelfunc,&state[14],param,14,14,0,lock,tol,tol,maxevals,
      jw,dw,rw,iw,verrs,fcnt,&rooterr);
    sdvelfunc(&state[14],param,verrs);
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
    double state[28],
    int lock[14],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[28],param[15],jw[392],dw[3528],rw[322];
    int iw[168],rooterr,i;

    sdgentime(&i);
    if (i != 42000) {
        sdseterr(52,42);
    }
    for (i = 0; i < 14; i++) {
        param[i] = state[14+i];
    }
    param[14] = time;
    sdroot(sdstatfunc,state,param,28,14,14,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    sdstatfunc(state,param,resid);
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

/* This routine performs steady motion analysis. */

void sdsteady(double time,
    double state[28],
    int lock[28],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[42],param[1];
    double jw[1176],dw[9800],rw[546];
    int iw[280],rooterr,i;

    sdgentime(&i);
    if (i != 42000) {
        sdseterr(53,42);
    }
    param[0] = time;
    sdroot(sdstdyfunc,state,param,42,28,14,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    sdstdyfunc(state,param,resid);
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

/* This routine performs state integration. */

void sdmotion(double *time,
    double state[28],
    double dstate[28],
    double dt,
    double ctol,
    double tol,
    int *flag,
    int *err)
{
    static double step;
    double work[168],ttime,param[1];
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
    sdvinteg(sdmotfunc,&ttime,state,dstate,param,dt,&step,28,tol,work,&vintgerr,
      &which);
    *time = ttime;
    *err = vintgerr;
}

/* This routine performs state integration with a fixed-step integrator. */

void sdfmotion(double *time,
    double state[28],
    double dstate[28],
    double dt,
    double ctol,
    int *flag,
    double *errest,
    int *err)
{
    double work[112],ttime,param[1];
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
    sdfinteg(sdmotfunc,&ttime,state,dstate,param,dt,28,work,errest,&ferr);
    if (ferr != 0) {
        *err = 1;
    }
    *time = ttime;
}
