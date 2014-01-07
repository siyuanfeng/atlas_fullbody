#ifndef _drc_pelvis_H
#define _drc_pelvis_H

#include "SDModel.h"

typedef struct {
    int ground_,nbod_,ndof_,ncons_,nloop_,nldof_,nloopc_,nball_,nlball_,npres_,
      nuser_;
    int jtype_[29],inb_[29],outb_[29],njntdof_[29],njntc_[29],njntp_[29],firstq_
      [29],ballq_[29],firstm_[29],firstp_[29];
    int trans_[34];
} sdgtopo_drc_pelvis_t;
typedef struct {
    double grav_[3],mk_[29],ik_[29][3][3],pin_[34][3];
    double rk_[29][3],ri_[29][3],pres_[34],stabvel_,stabpos_;
    int mfrcflg_,roustate_,vpkflg_,inerflg_,mmflg_,mmlduflg_,wwflg_,ltauflg_,
      fs0flg_,ii_,mmap_[34];
    int gravq_[3],mkq_[29],ikq_[29][3][3],pinq_[34][3],rkq_[29][3],riq_[29][3],
      presq_[34],stabvelq_,stabposq_;
    double mtot_,psmkg_,rhead_[29][3],rcom_[29][3],mkrcomt_[29][3][3],psikg_[3][
      3],psrcomg_[3],psrkg_[3],psrig_[3],psmk_[34],psik_[34][3][3],psrcom_[34][3
      ],psrk_[34][3],psri_[34][3];
} sdginput_drc_pelvis_t;
typedef struct {
    double curtim_,q_[35],qn_[35],u_[34],cnk_[34][3][3],cnb_[29][3][3];
    double rnk_[34][3],vnk_[34][3],wk_[34][3],rnb_[29][3],vnb_[29][3],wb_[29][3]
      ,wbrcom_[29][3],com_[3],rnkg_[3];
    double Cik_[34][3][3],rikt_[34][3][3],Iko_[34][3][3],mkrk_[34][3][3],Cib_[29
      ][3][3];
    double Wkk_[34][3],Vkk_[34][3],dik_[34][3],rpp_[34][3],rpk_[34][3],rik_[34][
      3],rik2_[34][3];
    double rpri_[34][3],Wik_[34][3],Vik_[34][3],Wirk_[34][3],rkWkk_[34][3],
      Wkrpk_[34][3],VikWkr_[34][3];
    double perr_[34],verr_[34],aerr_[34],mult_[34],ufk_[29][3],utk_[29][3],mfk_[
      29][3],mtk_[29][3];
    double utau_[34],mtau_[34],uacc_[34],uvel_[34],upos_[35];
    double s6_,c6_,s7_,c7_,s8_,c8_,s9_,c9_,s10_,c10_,s11_,c11_,s12_,c12_,s13_,
      c13_,s14_,c14_,s15_,c15_,s16_,c16_,s17_,c17_,s18_,c18_,s19_,c19_,s20_,c20_
      ,s21_,c21_,s22_,c22_,s23_,c23_,s24_,c24_,s25_,c25_,s26_,c26_,s27_,c27_,
      s28_,c28_,s29_,c29_,s30_,c30_,s31_,c31_,s32_,c32_,s33_,c33_;
} sdgstate_drc_pelvis_t;
typedef struct {
    double fs0_[34],qdot_[35],Otk_[34][3],Atk_[34][3],AiOiWi_[34][3],Fstar_[34][
      3];
    double Tstar_[34][3],Fstark_[34][3],Tstark_[34][3],IkWk_[34][3],WkIkWk_[34][
      3],gk_[34][3],IkbWk_[29][3],WkIkbWk_[29][3];
    double w0w0_[29],w1w1_[29],w2w2_[29],w0w1_[29],w0w2_[29],w1w2_[29];
    double w00w11_[29],w00w22_[29],w11w22_[29],ww_[34][34],qraux_[34];
    double mm_[34][34],mlo_[34][34],mdi_[34],IkWpk_[34][34][3],works_[34],
      workss_[34][34];
    double Wpk_[34][34][3],Vpk_[34][34][3],VWri_[34][34][3];
    int wmap_[34],multmap_[34],jpvt_[34],wsiz_,wrank_;
} sdglhs_drc_pelvis_t;
typedef struct {
    double fs_[34],udot_[34],tauc_[34],dyad_[29][3][3],fc_[34][3],tc_[34][3];
    double ank_[34][3],onk_[34][3],Onkb_[34][3],AOnkri_[34][3],Ankb_[34][3],
      AnkAtk_[34][3],anb_[29][3],onb_[29][3],dyrcom_[29][3];
    double ffk_[34][3],ttk_[34][3],fccikt_[34][3],ffkb_[29][3],ttkb_[29][3];
} sdgrhs_drc_pelvis_t;
typedef struct {
    double temp_[3000],tmat1_[3][3],tmat2_[3][3],tvec1_[3],tvec2_[3],tvec3_[3],
      tvec4_[3],tvec5_[3];
    double tsc1_,tsc2_,tsc3_;
} sdgtemp_drc_pelvis_t;
typedef struct {
    int lasterr_,lastrou_;
} sdgerror_drc_pelvis_t;

#ifdef __cplusplus
class drc_pelvis: public SDModel{
     protected:
        sdgtopo_drc_pelvis_t sdgtopo;
        sdginput_drc_pelvis_t sdginput;
        sdgstate_drc_pelvis_t sdgstate;
        sdglhs_drc_pelvis_t sdglhs;
        sdgrhs_drc_pelvis_t sdgrhs;
        sdgtemp_drc_pelvis_t sdgtemp;
        sdgerror_drc_pelvis_t sdgerror;
        int *idx_2_joint;
        int *idx_2_axis;
        int *idx_2_aux;
        int *idx_2_motor;
        int num_controls;
                        
     public:
        drc_pelvis();
        ~drc_pelvis();
        
void sdinit(void);
void sdst2ang(double st[35],    double stang[34]);
void sdang2st(double stang[34],    double st[35]);
void sdnrmsterr(double st[35],    double normst[35],    int routine);
void sdnormst(double st[35],    double normst[35]);
void sdstate(double timein,    double qin[35],    double uin[34]);
void sdqdot(double oqdot[35]);
void sdu2qdot(double uin[34],    double oqdot[35]);
void sdpsstate(double lqin[1]);
void sddovpk(void);
void sddoltau(void);
void sddoiner(void);
void sddofs0(void);
void sddomm(int routine);
void sddommldu(int routine);
void sdlhs(int routine);
void sdmfrc(double imult[34]);
void sdequivht(double tau[34]);
void sdfs0(void);
void sdfsmult(void);
void sdfsfull(void);
void sdfsgenmult(void);
void sdfsgenfull(void);
void sdfulltrq(double udotin[34],    double multin[34],    double trqout[34]);
void sdcomptrq(double udotin[34],    double trqout[34]);
void sdmulttrq(double multin[34],    double trqout[34]);
void sdrhs(void);
void sdmassmat(double *mmat);
void sdfrcmat(double fmat[34]);
void sdpseudo(double lqout[1],    double luout[1]);
void sdpsqdot(double lqdout[1]);
void sdpsudot(double ludout[1]);
void sdperr(double errs[34]);
void sdverr(double errs[34]);
void sdaerr(double errs[34]);
int sdchkbnum(int routine,    int bnum);
int sdchkjnum(int routine,    int jnum);
int sdchkucnum(int routine,    int ucnum);
int sdchkjaxis(int routine,    int jnum,    int axnum);
int sdchkjpin(int routine,    int jnum,    int pinno);
int sdindx(int joint,    int axis);
void sdpresacc(int joint,    int axis,    double prval);
void sdpresvel(int joint,    int axis,    double prval);
void sdprespos(int joint,    int axis,    double prval);
void sdgetht(int joint,    int axis,    double *torque);
void sdhinget(int joint,    int axis,    double torque);
void sdpointf(int body,    double point[3],    double force[3]);
void sdbodyt(int body,    double torque[3]);
void sddoww(int routine);
void sdxudot0(int routine,    double oudot0[34]);
void sdudot0(double oudot0[34]);
void sdsetudot(double iudot[34]);
void sdxudotm(int routine,    double imult[34],    double oudotm[34]);
void sdudotm(double imult[34],    double oudotm[34]);
void sdderiv(double oqdot[35],    double oudot[34]);
void sdresid(double eqdot[35],    double eudot[34],    double emults[34],    double resid[103]);
void sdmult(double omults[34],    int *owrank,    int omultmap[34]);
void sdreac(double force[29][3],    double torque[29][3]);
void sdmom(double lm[3],    double am[3],    double *ke);
void sdsys(double *mtoto,    double cm[3],    double icm[3][3]);
void sdpos(int body,    double pt[3],    double loc[3]);
void sdvel(int body,    double pt[3],    double velo[3]);
void sdorient(int body,    double dircos[3][3]);
void sdangvel(int body,    double avel[3]);
void sdtrans(int frbod,    double ivec[3],    int tobod,    double ovec[3]);
void sdrel2cart(int coord,    int body,    double point[3],    double linchg[3],    double rotchg[3]);
void sdacc(int body,    double pt[3],    double accel[3]);
void sdangacc(int body,    double aacc[3]);
void sdgrav(double gravin[3]);
void sdmass(int body,    double massin);
void sdiner(int body,    double inerin[3][3]);
void sdbtj(int joint,    double btjin[3]);
void sditj(int joint,    double itjin[3]);
void sdpin(int joint,    int pinno,    double pinin[3]);
void sdpres(int joint,    int axis,    int presin);
void sdconschg(void);
void sdstab(double velin,    double posin);
void sdgetgrav(double gravout[3]);
void sdgetmass(int body,    double *massout);
void sdgetiner(int body,    double inerout[3][3]);
void sdgetbtj(int joint,    double btjout[3]);
void sdgetitj(int joint,    double itjout[3]);
void sdgetpin(int joint,    int pinno,    double pinout[3]);
void sdgetpres(int joint,    int axis,    int *presout);
void sdgetstab(double *velout,    double *posout);
void sdinfo(int info[50]);
void sdjnt(int joint,    int info[50],    int tran[6]);
void sdcons(int consno,    int info[50]);
void sdgentime(int *gentm);
// analysis routines
void sdposfunc(double vars[34],    double param[1],    double resid[34]);
void sdvelfunc(double vars[34],    double param[36],    double resid[34]);
void sdstatfunc(double vars[34],    double param[35],    double resid[68]);
void sdstdyfunc(double vars[68],    double param[1],    double resid[102]);
void sdmotfunc(double time,    double state[69],    double dstate[69],    double param[1],    int *status);
void sdassemble(double time,    double state[69],    int lock[34],    double tol,    int maxevals,    int *fcnt,    int *err);
void sdinitvel(double time,    double state[69],    int lock[34],    double tol,    int maxevals,    int *fcnt,    int *err);
void sdstatic(double time,    double state[69],    int lock[34],    double ctol,    double tol,    int maxevals,    int *fcnt,    int *err);
void sdsteady(double time,    double state[69],    int lock[68],    double ctol,    double tol,    int maxevals,    int *fcnt,    int *err);
void sdmotion(double *time,    double state[69],    double dstate[69],    double dt,    double ctol,    double tol,    int *flag,    int *err);
void sdfmotion(double *time,    double state[69],    double dstate[69],    double dt,    double ctol,    int *flag,    double *errest,    int *err);
void sdprerrmsg(FILE *fnum,    int routine,    int errnum);
void sderror(int *routine,    int *errnum);
void sdprinterr(FILE *fnum);
void sdclearerr(void);
void sdseterr(int routine,    int errnum);
void sdldudcomp(int n,    int na,    int map[],    double tol,    double ld[],    double sum[],    double m[],    double l[],    double d[]);
void sdldubsl(int n,    int na,    int map[],    double l[],    double b[],    double x[]);
void sdldubsd(int n,    int na,    int map[],    double d[],    double b[],    double x[]);
void sdldubsu(int n,    int na,    int map[],    double l[],    double b[],    double x[]);
void sdldubslv(int n,    int na,    int map[],    double work[],    double l[],    double d[],    double b[],    double x[]);
void sdlduslv(int n,    int na,    int map[],    double tol,    double work1[],    double work2[],    double m[],    double b[],    double l[],    double d[],    double x[]);
void sdqrdcomp(int nr,    int nc,    int nra,    int nca,    int mapr[],    int mapc[],    double w[],    double local_qraux[],    int local_jpvt[]);
void sdqrsl(int nr,    int nc,    int nra,    int nca,    int mapr[],    int mapc[],    int k,    double work[],    double w[],    double local_qraux[],    double b[],    double x[]);
void sdqrbslv(int nr,    int nc,    int nra,    int nca,    int mapr[],    int mapc[],    double tol,    double work[],    int iwork[],    double w[],    double local_qraux[],    int local_jpvt[],    double b[],    double x[],    int *rank);
void sdqrslv(int nr,    int nc,    int nra,    int nca,    int mapr[],    int mapc[],    double tol,    int local_jpvt[],    double local_qraux[],    double work[],    int iwork[],    double w[],    double b[],    double x[],    int *rank);
void sdlsslv(int nr,    int nc,    int nra,    int nca,    int ndes,    int mapr[],    int mapc[],    double tol,    double dw[],    double rw[],    int iw[],    double w[],    double b[],    double x[]);
void sdcalcerrs(double fval[],    int nfunc,    int ndes,    int dnorm,    double *maxderr,    double *maxrerr,    double *derrnorm);
void sdadjvars(void (drc_pelvis::*func)(double *, double *, double * ),    double vars[],    double param[],    int nfunc,    int ndes,    int dnorm,    int nvar,    double deltas[],    double step,    double rerr,    double tderr,    double rtol,    int *fcnt,    double newvars[],    double newerrs[]);
void sdcalcjac(void (drc_pelvis::*func)(double *, double *, double * ),    double vars[],    double param[],    int nfunc,    int nvar,    int lock[],    double delta,    double fval[],    double ftmp[],    double jw[],    int *fcnt,    double scale[]);
void sdroot(void (drc_pelvis::*func)(double *, double *, double *),    double vars[],    double param[],    int nfunc,    int nvar,    int ndesin,    int lock[],    double rtol,    double dtol,    int maxeval,    double jw[],    double dw[],    double rw[],    int iw[],    double fret[],    int *fcnt,    int *err);
void sdrk4m(void (drc_pelvis::*func)(double, double *, double *, double *, int* ),    double time,    double st[],    double dst0[],    double param[],    double step,    double nst[],    int neq,    double work[],    double errs[],    double *maxerr,    int *which);
void sdfinteg(void (drc_pelvis::*func)(double, double *, double *, double *, int *),    double *time,    double st[],    double dst[],    double param[],    double step,    int neq,    double work[],    double *errest,    int *status);
void sdvinteg(void (drc_pelvis::*func)(double, double *, double *, double *, int *),    double *time,    double st[],    double dst[],    double param[],    double dt,    double *step,    int neqin,    double tol,    double work[],    int *err,    int *which);
void sddc2ang(double dircos[3][3],    double *a1,    double *a2,    double *a3);
void sddc2quat(double dircos[3][3],    double *e1,    double *e2,    double *e3,    double *e4);
void sdang2dc(double a1,    double a2,    double a3,    double dircos[3][3]);
void sdquat2dc(double ie1,    double ie2,    double ie3,    double ie4,    double dircos[3][3]);
double sdvdot(double ivec1[3],    double ivec2[3]);
double sdvnorm(double ivec[3]);
void sdvcopy(double ivec[3],    double ovec[3]);
void sdvset(double sclr1,    double sclr2,    double sclr3,    double ovec[3]);
void sdvadd(double ivec1[3],    double ivec2[3],    double ovec[3]);
void sdvsub(double ivec1[3],    double ivec2[3],    double ovec[3]);
void sdvmul(double sclr,    double ivec[3],    double ovec[3]);
void sdvaxpy(double sclr,    double ivec1[3],    double ivec2[3],    double ovec[3]);
void sdvcross(double ivec1[3],    double ivec2[3],    double ovec[3]);
void sdvrot(double ivec[3],    double rvec[3],    double theta,    double ovec[3]);
void sdserialno(int *serno);

        // user-defined functions
	          void sduforce(double t, double *q, double *u) {;}
	          void sdumotion(double t, double *q, double *u) {;}
        


        void sduderiv(double t, double *y, double *dy, double *params, int *status);
        void sdueval(double *vars, double *params, double *redid);
        
        void set_aux(int *map); // pass in an array of aux numbers
        
  //      drc_pelvis operator= (drc_pelvis);
        
        int NQ(void);   // get number of q coordinates
        int NU(void);   // get number of u coordinates
		int BQ(void);	// number of base coordinates (6 if floating, 0 otherwise)
        int NM(void);   // number of control joints ("motors")
        int NBOD(void);   // number of bodies
        int NJNT(void);     // number of joints
        
        int joint(int idx); // get joint number of coordinate
        int axis(int idx);  // get axis of coordinate
        int aux(int idx); // get aux number of coordinate
        int motor(int idx); // get the coordinate of motor

	//void massmat( double *mat );

	
	//#include "drc_pelvis_ext.h"

		//double **M;
        
//         double * get_q(int i);
//         double * get_u(int i);
//         double * get_qdot(int i);
//         double * get_udot(int i);
//         double *q_local, *u_local, *qdot_local, *udot_local, *u_zero;
};
#else
typedef struct drc_pelvis drc_pelvis;
#endif



#endif
