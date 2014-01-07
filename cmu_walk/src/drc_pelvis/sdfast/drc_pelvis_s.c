/*
Generated 01-Apr-2009 04:20:00 by SD/FAST, Kane's formulation
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

void sdposfunc(double vars[34],
    double param[1],
    double resid[34])
{
    int i;
    double pos[35],vel[34];

    for (i = 0; i < 34; i++) {
        vel[i] = 0.;
    }
    sdang2st(vars,pos);
    sdstate(param[0],pos,vel);
    sdumotion(param[0],pos,vel);
    sdperr(resid);
}

void sdvelfunc(double vars[34],
    double param[36],
    double resid[34])
{

    sdstate(param[35],param,vars);
    sdumotion(param[35],param,vars);
    sdverr(resid);
}

void sdstatfunc(double vars[34],
    double param[35],
    double resid[68])
{
    double pos[35],qdotdum[35];

    sdang2st(vars,pos);
    sdstate(param[34],pos,param);
    sdumotion(param[34],pos,param);
    sduforce(param[34],pos,param);
    sdperr(resid);
    sdderiv(qdotdum,&resid[34]);
}

void sdstdyfunc(double vars[68],
    double param[1],
    double resid[102])
{
    double pos[35],qdotdum[35];

    sdang2st(vars,pos);
    sdstate(param[0],pos,&vars[34]);
    sdumotion(param[0],pos,&vars[34]);
    sduforce(param[0],pos,&vars[34]);
    sdperr(resid);
    sdverr(&resid[34]);
    sdderiv(qdotdum,&resid[68]);
}

/* This routine is passed to the integrator. */

void sdmotfunc(double time,
    double state[69],
    double dstate[69],
    double param[1],
    int *status)
{
    double err[34];
    int i;

    sdstate(time,state,&state[35]);
    sdumotion(time,state,&state[35]);
    sduforce(time,state,&state[35]);
    sdderiv(dstate,&dstate[35]);
    *status = 1;
    sdverr(err);
    for (i = 0; i < 34; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    sdperr(err);
    for (i = 0; i < 34; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    *status = 0;
}

/* This routine performs assembly analysis. */

void sdassemble(double time,
    double state[69],
    int lock[34],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double perrs[34],param[1];
    int i;
    double jw[1156],dw[9248],rw[544];
    int iw[272],rooterr;

    sdgentime(&i);
    if (i != 42000) {
        sdseterr(50,42);
    }
    param[0] = time;
    sdst2ang(state,state);
    sdroot(sdposfunc,state,param,34,34,0,lock,tol,tol,maxevals,
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
    double state[69],
    int lock[34],
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double verrs[34],param[36];
    int i;
    double jw[1156],dw[9248],rw[544];
    int iw[272],rooterr;

    sdgentime(&i);
    if (i != 42000) {
        sdseterr(51,42);
    }
    for (i = 0; i < 35; i++) {
        param[i] = state[i];
    }
    param[35] = time;
    sdroot(sdvelfunc,&state[35],param,34,34,0,lock,tol,tol,maxevals,
      jw,dw,rw,iw,verrs,fcnt,&rooterr);
    sdvelfunc(&state[35],param,verrs);
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
    double state[69],
    int lock[34],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[68],param[35],jw[2312],dw[20808],rw[782];
    int iw[408],rooterr,i;

    sdgentime(&i);
    if (i != 42000) {
        sdseterr(52,42);
    }
    for (i = 0; i < 34; i++) {
        param[i] = state[35+i];
    }
    param[34] = time;
    sdst2ang(state,state);
    sdroot(sdstatfunc,state,param,68,34,34,lock,
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
    double state[69],
    int lock[68],
    double ctol,
    double tol,
    int maxevals,
    int *fcnt,
    int *err)
{
    double resid[102],param[1],vars[68];
    double jw[6936],dw[57800],rw[1326];
    int iw[680],rooterr,i;

    sdgentime(&i);
    if (i != 42000) {
        sdseterr(53,42);
    }
    param[0] = time;
    sdst2ang(state,vars);
    for (i = 0; i < 34; i++) {
        vars[34+i] = state[35+i];
    }
    sdroot(sdstdyfunc,vars,param,102,68,34,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    sdstdyfunc(vars,param,resid);
    *fcnt = *fcnt+1;
    sdang2st(vars,state);
    for (i = 0; i < 34; i++) {
        state[35+i] = vars[34+i];
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
    double state[69],
    double dstate[69],
    double dt,
    double ctol,
    double tol,
    int *flag,
    int *err)
{
    static double step;
    double work[414],ttime,param[1];
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
    sdvinteg(sdmotfunc,&ttime,state,dstate,param,dt,&step,69,tol,work,&vintgerr,
      &which);
    *time = ttime;
    *err = vintgerr;
}

/* This routine performs state integration with a fixed-step integrator. */

void sdfmotion(double *time,
    double state[69],
    double dstate[69],
    double dt,
    double ctol,
    int *flag,
    double *errest,
    int *err)
{
    double work[276],ttime,param[1];
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
    sdfinteg(sdmotfunc,&ttime,state,dstate,param,dt,69,work,errest,&ferr);
    if (ferr != 0) {
        *err = 1;
    }
    *time = ttime;
}
