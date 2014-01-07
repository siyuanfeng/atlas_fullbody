#ifndef _SDLIB_H
#define _SDLIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

void sdserialno(int *serno);
void sdvrot(double *ivec,double *rvec,double theta,double *ovec);
void sdvcross(double *ivec1,double *ivec2,double *ovec);
void sdvaxpy(double sclr,double *ivec1,double *ivec2,double *ovec);
void sdvmul(double sclr,double *ivec,double *ovec);
void sdvsub(double *ivec1,double *ivec2,double *ovec);
void sdvadd(double *ivec1,double *ivec2,double *ovec);
void sdvset(double sclr1,double sclr2,double sclr3,double *ovec);
void sdvcopy(double *ivec,double *ovec);
double sdvnorm(double *ivec);
double sdvdot(double *ivec1,double *ivec2);
void sdquat2dc(double ie1,double ie2,double ie3,double ie4,double(*dircos)[3]);
void sdang2dc(double a1,double a2,double a3,double(*dircos)[3]);
void sddc2quat(double(*dircos)[3],double *e1,double *e2,double *e3,double *e4);
void sddc2ang(double(*dircos)[3],double *a1,double *a2,double *a3);
void sdvinteg(int(*func)(),double *time,double *st,double *dst,double *param,double dt,double *step,int neqin,double tol,double *work,int *err,int *which);
void sdfinteg(int(*func)(),double *time,double *st,double *dst,double *param,double step,int neq,double *work,double *errest,int *status);
void sdrk4m(int(*func)(),double time,double *st,double *dst0,double *param,double step,double *nst,int neq,double *work,double *errs,double *maxerr,int *which);
void sdroot(int (*func)(),double *vars,double *param,int nfunc,int nvar,int ndesin,int *lock,double rtol,double dtol,int maxeval,double *jw,double *dw,double *rw,int *iw,double *fret,int *fcnt,int *err);
void sdcalcjac(int(*func)(),double *vars,double *param,int nfunc,int nvar,int *lock,double delta,double *fval,double *ftmp,double *jw,int *fcnt,double *scale);
void sdadjvars(int(*func)(),double *vars,double *param,int nfunc,int ndes,int dnorm,int nvar,double *deltas,double step,double rerr,double tderr,double rtol,int *fcnt,double *newvars,double *newerrs);
void sdcalcerrs(double *fval,int nfunc,int ndes,int dnorm,double *maxderr,double *maxrerr,double *derrnorm);
void sdlsslv(int nr,int nc,int nra,int nca,int ndes,int *mapr,int *mapc,double tol,double *dw,double *rw,int *iw,double *w,double *b,double *x);
void sdqrslv(int nr,int nc,int nra,int nca,int *mapr,int *mapc, double tol,int *jpvt,double *qraux,double *work,int *iwork,double *w,double *b,double *x,int *rank);
void sdqrbslv(int nr,int nc,int nra,int nca,int *mapr,int *mapc,double tol,double *work,int *iwork,double *w,double *qraux,int *jpvt,double *b,double *x,int *rank);
void sdqrsl(int nr,int nc,int nra,int nca,int *mapr,int *mapc,int k,double *work,double *w,double *qraux,double *b,double *x);
void sdqrdcomp(int nr,int nc,int nra,int nca,int *mapr,int *mapc,double *w,double *qraux,int *jpvt);
void sdlduslv(int n,int na,int *map,double tol,double *work1,double *work2,double *m,double *b,double *l,double *d,double *x);
void sdldubslv(int n,int na,int *map,double *work,double *l,double *d,double *b,double *x);
void sdldubsu(int n,int na,int *map,double *l,double *b,double *x);
void sdldubsd(int n,int na,int *map,double *d,double *b,double *x);
void sdldubsl(int n,int na,int *map,double *l,double *b,double *x);
void sdldudcomp(int n,int na,int *map,double tol,double *ld,double *sum,double *m,double *l,double *d);
void sdseterr(int routine,int errnum);
void sdclearerr(void);
void sdprerrmsg(FILE *fnum,int routine,int errnum);
void sdprinterr(FILE *fnum);
void sderror(int *routine,int *errnum);

#ifdef __cplusplus
}
#endif

#endif