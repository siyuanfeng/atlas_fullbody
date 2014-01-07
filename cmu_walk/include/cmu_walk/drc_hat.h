#ifndef _drc_hat_H
#define _drc_hat_H

#include "SDModel.h"

typedef struct {
    int ground_,nbod_,ndof_,ncons_,nloop_,nldof_,nloopc_,nball_,nlball_,npres_,
      nuser_;
    int jtype_[16],inb_[16],outb_[16],njntdof_[16],njntc_[16],njntp_[16],firstq_
      [16],ballq_[16],firstm_[16],firstp_[16];
    int trans_[21];
} sdgtopo_drc_hat_t;
typedef struct {
    double grav_[3],mk_[16],ik_[16][3][3],pin_[21][3];
    double rk_[16][3],ri_[16][3],pres_[21],stabvel_,stabpos_;
    int mfrcflg_,roustate_,vpkflg_,inerflg_,mmflg_,mmlduflg_,wwflg_,ltauflg_,
      fs0flg_,ii_,mmap_[21];
    int gravq_[3],mkq_[16],ikq_[16][3][3],pinq_[21][3],rkq_[16][3],riq_[16][3],
      presq_[21],stabvelq_,stabposq_;
    double mtot_,psmkg_,rhead_[16][3],rcom_[16][3],mkrcomt_[16][3][3],psikg_[3][
      3],psrcomg_[3],psrkg_[3],psrig_[3],psmk_[21],psik_[21][3][3],psrcom_[21][3
      ],psrk_[21][3],psri_[21][3];
} sdginput_drc_hat_t;
typedef struct {
    double curtim_,q_[22],qn_[22],u_[21],cnk_[21][3][3],cnb_[16][3][3];
    double rnk_[21][3],vnk_[21][3],wk_[21][3],rnb_[16][3],vnb_[16][3],wb_[16][3]
      ,wbrcom_[16][3],com_[3],rnkg_[3];
    double Cik_[21][3][3],rikt_[21][3][3],Iko_[21][3][3],mkrk_[21][3][3],Cib_[16
      ][3][3];
    double Wkk_[21][3],Vkk_[21][3],dik_[21][3],rpp_[21][3],rpk_[21][3],rik_[21][
      3],rik2_[21][3];
    double rpri_[21][3],Wik_[21][3],Vik_[21][3],Wirk_[21][3],rkWkk_[21][3],
      Wkrpk_[21][3],VikWkr_[21][3];
    double perr_[21],verr_[21],aerr_[21],mult_[21],ufk_[16][3],utk_[16][3],mfk_[
      16][3],mtk_[16][3];
    double utau_[21],mtau_[21],uacc_[21],uvel_[21],upos_[22];
    double s6_,c6_,s7_,c7_,s8_,c8_,s9_,c9_,s10_,c10_,s11_,c11_,s12_,c12_,s13_,
      c13_,s14_,c14_,s15_,c15_,s16_,c16_,s17_,c17_,s18_,c18_,s19_,c19_,s20_,c20_
      ;
} sdgstate_drc_hat_t;
typedef struct {
    double fs0_[21],qdot_[22],Otk_[21][3],Atk_[21][3],AiOiWi_[21][3],Fstar_[21][
      3];
    double Tstar_[21][3],Fstark_[21][3],Tstark_[21][3],IkWk_[21][3],WkIkWk_[21][
      3],gk_[21][3],IkbWk_[16][3],WkIkbWk_[16][3];
    double w0w0_[16],w1w1_[16],w2w2_[16],w0w1_[16],w0w2_[16],w1w2_[16];
    double w00w11_[16],w00w22_[16],w11w22_[16],ww_[21][21],qraux_[21];
    double mm_[21][21],mlo_[21][21],mdi_[21],IkWpk_[21][21][3],works_[21],
      workss_[21][21];
    double Wpk_[21][21][3],Vpk_[21][21][3],VWri_[21][21][3];
    int wmap_[21],multmap_[21],jpvt_[21],wsiz_,wrank_;
} sdglhs_drc_hat_t;
typedef struct {
    double fs_[21],udot_[21],tauc_[21],dyad_[16][3][3],fc_[21][3],tc_[21][3];
    double ank_[21][3],onk_[21][3],Onkb_[21][3],AOnkri_[21][3],Ankb_[21][3],
      AnkAtk_[21][3],anb_[16][3],onb_[16][3],dyrcom_[16][3];
    double ffk_[21][3],ttk_[21][3],fccikt_[21][3],ffkb_[16][3],ttkb_[16][3];
} sdgrhs_drc_hat_t;
typedef struct {
    double temp_[3000],tmat1_[3][3],tmat2_[3][3],tvec1_[3],tvec2_[3],tvec3_[3],
      tvec4_[3],tvec5_[3];
    double tsc1_,tsc2_,tsc3_;
} sdgtemp_drc_hat_t;
typedef struct {
    int lasterr_,lastrou_;
} sdgerror_drc_hat_t;

#ifdef __cplusplus
class drc_hat: public SDModel{
     protected:
        sdgtopo_drc_hat_t sdgtopo;
        sdginput_drc_hat_t sdginput;
        sdgstate_drc_hat_t sdgstate;
        sdglhs_drc_hat_t sdglhs;
        sdgrhs_drc_hat_t sdgrhs;
        sdgtemp_drc_hat_t sdgtemp;
        sdgerror_drc_hat_t sdgerror;
        int *idx_2_joint;
        int *idx_2_axis;
        int *idx_2_aux;
        int *idx_2_motor;
        int num_controls;
                        
     public:
        drc_hat();
        ~drc_hat();
        
void sdinit(void);
void sdst2ang(double st[22],    double stang[21]);
void sdang2st(double stang[21],    double st[22]);
void sdnrmsterr(double st[22],    double normst[22],    int routine);
void sdnormst(double st[22],    double normst[22]);
void sdstate(double timein,    double qin[22],    double uin[21]);
void sdqdot(double oqdot[22]);
void sdu2qdot(double uin[21],    double oqdot[22]);
void sdpsstate(double lqin[1]);
void sddovpk(void);
void sddoltau(void);
void sddoiner(void);
void sddofs0(void);
void sddomm(int routine);
void sddommldu(int routine);
void sdlhs(int routine);
void sdmfrc(double imult[21]);
void sdequivht(double tau[21]);
void sdfs0(void);
void sdfsmult(void);
void sdfsfull(void);
void sdfsgenmult(void);
void sdfsgenfull(void);
void sdfulltrq(double udotin[21],    double multin[21],    double trqout[21]);
void sdcomptrq(double udotin[21],    double trqout[21]);
void sdmulttrq(double multin[21],    double trqout[21]);
void sdrhs(void);
void sdmassmat(double *mmat);
void sdfrcmat(double fmat[21]);
void sdpseudo(double lqout[1],    double luout[1]);
void sdpsqdot(double lqdout[1]);
void sdpsudot(double ludout[1]);
void sdperr(double errs[21]);
void sdverr(double errs[21]);
void sdaerr(double errs[21]);
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
void sdxudot0(int routine,    double oudot0[21]);
void sdudot0(double oudot0[21]);
void sdsetudot(double iudot[21]);
void sdxudotm(int routine,    double imult[21],    double oudotm[21]);
void sdudotm(double imult[21],    double oudotm[21]);
void sdderiv(double oqdot[22],    double oudot[21]);
void sdresid(double eqdot[22],    double eudot[21],    double emults[21],    double resid[64]);
void sdmult(double omults[21],    int *owrank,    int omultmap[21]);
void sdreac(double force[16][3],    double torque[16][3]);
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
void sdposfunc(double vars[21],    double param[1],    double resid[21]);
void sdvelfunc(double vars[21],    double param[23],    double resid[21]);
void sdstatfunc(double vars[21],    double param[22],    double resid[42]);
void sdstdyfunc(double vars[42],    double param[1],    double resid[63]);
void sdmotfunc(double time,    double state[43],    double dstate[43],    double param[1],    int *status);
void sdassemble(double time,    double state[43],    int lock[21],    double tol,    int maxevals,    int *fcnt,    int *err);
void sdinitvel(double time,    double state[43],    int lock[21],    double tol,    int maxevals,    int *fcnt,    int *err);
void sdstatic(double time,    double state[43],    int lock[21],    double ctol,    double tol,    int maxevals,    int *fcnt,    int *err);
void sdsteady(double time,    double state[43],    int lock[42],    double ctol,    double tol,    int maxevals,    int *fcnt,    int *err);
void sdmotion(double *time,    double state[43],    double dstate[43],    double dt,    double ctol,    double tol,    int *flag,    int *err);
void sdfmotion(double *time,    double state[43],    double dstate[43],    double dt,    double ctol,    int *flag,    double *errest,    int *err);
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
void sdadjvars(void (drc_hat::*func)(double *, double *, double * ),    double vars[],    double param[],    int nfunc,    int ndes,    int dnorm,    int nvar,    double deltas[],    double step,    double rerr,    double tderr,    double rtol,    int *fcnt,    double newvars[],    double newerrs[]);
void sdcalcjac(void (drc_hat::*func)(double *, double *, double * ),    double vars[],    double param[],    int nfunc,    int nvar,    int lock[],    double delta,    double fval[],    double ftmp[],    double jw[],    int *fcnt,    double scale[]);
void sdroot(void (drc_hat::*func)(double *, double *, double *),    double vars[],    double param[],    int nfunc,    int nvar,    int ndesin,    int lock[],    double rtol,    double dtol,    int maxeval,    double jw[],    double dw[],    double rw[],    int iw[],    double fret[],    int *fcnt,    int *err);
void sdrk4m(void (drc_hat::*func)(double, double *, double *, double *, int* ),    double time,    double st[],    double dst0[],    double param[],    double step,    double nst[],    int neq,    double work[],    double errs[],    double *maxerr,    int *which);
void sdfinteg(void (drc_hat::*func)(double, double *, double *, double *, int *),    double *time,    double st[],    double dst[],    double param[],    double step,    int neq,    double work[],    double *errest,    int *status);
void sdvinteg(void (drc_hat::*func)(double, double *, double *, double *, int *),    double *time,    double st[],    double dst[],    double param[],    double dt,    double *step,    int neqin,    double tol,    double work[],    int *err,    int *which);
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
        
  //      drc_hat operator= (drc_hat);
        
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

	
	//#include "drc_hat_ext.h"

		//double **M;
        
//         double * get_q(int i);
//         double * get_u(int i);
//         double * get_qdot(int i);
//         double * get_udot(int i);
//         double *q_local, *u_local, *qdot_local, *udot_local, *u_zero;
};
#else
typedef struct drc_hat drc_hat;
#endif



#endif
