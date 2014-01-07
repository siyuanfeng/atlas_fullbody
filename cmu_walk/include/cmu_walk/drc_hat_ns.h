#ifndef _drc_hat_ns_H
#define _drc_hat_ns_H

#include "SDModel.h"

typedef struct {
    int ground_,nbod_,ndof_,ncons_,nloop_,nldof_,nloopc_,nball_,nlball_,npres_,
      nuser_;
    int jtype_[16],inb_[16],outb_[16],njntdof_[16],njntc_[16],njntp_[16],firstq_
      [16],ballq_[16],firstm_[16],firstp_[16];
    int trans_[18];
} sdgtopo_drc_hat_ns_t;
typedef struct {
    double grav_[3],mk_[16],ik_[16][3][3],pin_[18][3];
    double rk_[16][3],ri_[16][3],pres_[18],stabvel_,stabpos_;
    int mfrcflg_,roustate_,vpkflg_,inerflg_,mmflg_,mmlduflg_,wwflg_,ltauflg_,
      fs0flg_,ii_,mmap_[18];
    int gravq_[3],mkq_[16],ikq_[16][3][3],pinq_[18][3],rkq_[16][3],riq_[16][3],
      presq_[18],stabvelq_,stabposq_;
    double mtot_,psmkg_,rhead_[16][3],rcom_[16][3],mkrcomt_[16][3][3],psikg_[3][
      3],psrcomg_[3],psrkg_[3],psrig_[3],psmk_[18],psik_[18][3][3],psrcom_[18][3
      ],psrk_[18][3],psri_[18][3];
} sdginput_drc_hat_ns_t;
typedef struct {
    double curtim_,q_[19],qn_[19],u_[18],cnk_[18][3][3],cnb_[16][3][3];
    double rnk_[18][3],vnk_[18][3],wk_[18][3],rnb_[16][3],vnb_[16][3],wb_[16][3]
      ,wbrcom_[16][3],com_[3],rnkg_[3];
    double Cik_[18][3][3],rikt_[18][3][3],Iko_[18][3][3],mkrk_[18][3][3],Cib_[16
      ][3][3];
    double Wkk_[18][3],Vkk_[18][3],dik_[18][3],rpp_[18][3],rpk_[18][3],rik_[18][
      3],rik2_[18][3];
    double rpri_[18][3],Wik_[18][3],Vik_[18][3],Wirk_[18][3],rkWkk_[18][3],
      Wkrpk_[18][3],VikWkr_[18][3];
    double perr_[18],verr_[18],aerr_[18],mult_[18],ufk_[16][3],utk_[16][3],mfk_[
      16][3],mtk_[16][3];
    double utau_[18],mtau_[18],uacc_[18],uvel_[18],upos_[19];
    double s6_,c6_,s7_,c7_,s8_,c8_,s9_,c9_,s10_,c10_,s11_,c11_,s12_,c12_,s13_,
      c13_,s14_,c14_,s15_,c15_,s16_,c16_,s17_,c17_;
} sdgstate_drc_hat_ns_t;
typedef struct {
    double fs0_[18],qdot_[19],Otk_[18][3],Atk_[18][3],AiOiWi_[18][3],Fstar_[18][
      3];
    double Tstar_[18][3],Fstark_[18][3],Tstark_[18][3],IkWk_[18][3],WkIkWk_[18][
      3],gk_[18][3],IkbWk_[16][3],WkIkbWk_[16][3];
    double w0w0_[16],w1w1_[16],w2w2_[16],w0w1_[16],w0w2_[16],w1w2_[16];
    double w00w11_[16],w00w22_[16],w11w22_[16],ww_[18][18],qraux_[18];
    double mm_[18][18],mlo_[18][18],mdi_[18],IkWpk_[18][18][3],works_[18],
      workss_[18][18];
    double Wpk_[18][18][3],Vpk_[18][18][3],VWri_[18][18][3];
    int wmap_[18],multmap_[18],jpvt_[18],wsiz_,wrank_;
} sdglhs_drc_hat_ns_t;
typedef struct {
    double fs_[18],udot_[18],tauc_[18],dyad_[16][3][3],fc_[18][3],tc_[18][3];
    double ank_[18][3],onk_[18][3],Onkb_[18][3],AOnkri_[18][3],Ankb_[18][3],
      AnkAtk_[18][3],anb_[16][3],onb_[16][3],dyrcom_[16][3];
    double ffk_[18][3],ttk_[18][3],fccikt_[18][3],ffkb_[16][3],ttkb_[16][3];
} sdgrhs_drc_hat_ns_t;
typedef struct {
    double temp_[3000],tmat1_[3][3],tmat2_[3][3],tvec1_[3],tvec2_[3],tvec3_[3],
      tvec4_[3],tvec5_[3];
    double tsc1_,tsc2_,tsc3_;
} sdgtemp_drc_hat_ns_t;
typedef struct {
    int lasterr_,lastrou_;
} sdgerror_drc_hat_ns_t;

#ifdef __cplusplus
class drc_hat_ns: public SDModel{
     protected:
        sdgtopo_drc_hat_ns_t sdgtopo;
        sdginput_drc_hat_ns_t sdginput;
        sdgstate_drc_hat_ns_t sdgstate;
        sdglhs_drc_hat_ns_t sdglhs;
        sdgrhs_drc_hat_ns_t sdgrhs;
        sdgtemp_drc_hat_ns_t sdgtemp;
        sdgerror_drc_hat_ns_t sdgerror;
        int *idx_2_joint;
        int *idx_2_axis;
        int *idx_2_aux;
        int *idx_2_motor;
        int num_controls;
                        
     public:
        drc_hat_ns();
        ~drc_hat_ns();
        
void sdinit(void);
void sdst2ang(double st[19],    double stang[18]);
void sdang2st(double stang[18],    double st[19]);
void sdnrmsterr(double st[19],    double normst[19],    int routine);
void sdnormst(double st[19],    double normst[19]);
void sdstate(double timein,    double qin[19],    double uin[18]);
void sdqdot(double oqdot[19]);
void sdu2qdot(double uin[18],    double oqdot[19]);
void sdpsstate(double lqin[1]);
void sddovpk(void);
void sddoltau(void);
void sddoiner(void);
void sddofs0(void);
void sddomm(int routine);
void sddommldu(int routine);
void sdlhs(int routine);
void sdmfrc(double imult[18]);
void sdequivht(double tau[18]);
void sdfs0(void);
void sdfsmult(void);
void sdfsfull(void);
void sdfsgenmult(void);
void sdfsgenfull(void);
void sdfulltrq(double udotin[18],    double multin[18],    double trqout[18]);
void sdcomptrq(double udotin[18],    double trqout[18]);
void sdmulttrq(double multin[18],    double trqout[18]);
void sdrhs(void);
void sdmassmat(double *mmat);
void sdfrcmat(double fmat[18]);
void sdpseudo(double lqout[1],    double luout[1]);
void sdpsqdot(double lqdout[1]);
void sdpsudot(double ludout[1]);
void sdperr(double errs[18]);
void sdverr(double errs[18]);
void sdaerr(double errs[18]);
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
void sdxudot0(int routine,    double oudot0[18]);
void sdudot0(double oudot0[18]);
void sdsetudot(double iudot[18]);
void sdxudotm(int routine,    double imult[18],    double oudotm[18]);
void sdudotm(double imult[18],    double oudotm[18]);
void sdderiv(double oqdot[19],    double oudot[18]);
void sdresid(double eqdot[19],    double eudot[18],    double emults[18],    double resid[55]);
void sdmult(double omults[18],    int *owrank,    int omultmap[18]);
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
void sdposfunc(double vars[18],    double param[1],    double resid[18]);
void sdvelfunc(double vars[18],    double param[20],    double resid[18]);
void sdstatfunc(double vars[18],    double param[19],    double resid[36]);
void sdstdyfunc(double vars[36],    double param[1],    double resid[54]);
void sdmotfunc(double time,    double state[37],    double dstate[37],    double param[1],    int *status);
void sdassemble(double time,    double state[37],    int lock[18],    double tol,    int maxevals,    int *fcnt,    int *err);
void sdinitvel(double time,    double state[37],    int lock[18],    double tol,    int maxevals,    int *fcnt,    int *err);
void sdstatic(double time,    double state[37],    int lock[18],    double ctol,    double tol,    int maxevals,    int *fcnt,    int *err);
void sdsteady(double time,    double state[37],    int lock[36],    double ctol,    double tol,    int maxevals,    int *fcnt,    int *err);
void sdmotion(double *time,    double state[37],    double dstate[37],    double dt,    double ctol,    double tol,    int *flag,    int *err);
void sdfmotion(double *time,    double state[37],    double dstate[37],    double dt,    double ctol,    int *flag,    double *errest,    int *err);
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
void sdadjvars(void (drc_hat_ns::*func)(double *, double *, double * ),    double vars[],    double param[],    int nfunc,    int ndes,    int dnorm,    int nvar,    double deltas[],    double step,    double rerr,    double tderr,    double rtol,    int *fcnt,    double newvars[],    double newerrs[]);
void sdcalcjac(void (drc_hat_ns::*func)(double *, double *, double * ),    double vars[],    double param[],    int nfunc,    int nvar,    int lock[],    double delta,    double fval[],    double ftmp[],    double jw[],    int *fcnt,    double scale[]);
void sdroot(void (drc_hat_ns::*func)(double *, double *, double *),    double vars[],    double param[],    int nfunc,    int nvar,    int ndesin,    int lock[],    double rtol,    double dtol,    int maxeval,    double jw[],    double dw[],    double rw[],    int iw[],    double fret[],    int *fcnt,    int *err);
void sdrk4m(void (drc_hat_ns::*func)(double, double *, double *, double *, int* ),    double time,    double st[],    double dst0[],    double param[],    double step,    double nst[],    int neq,    double work[],    double errs[],    double *maxerr,    int *which);
void sdfinteg(void (drc_hat_ns::*func)(double, double *, double *, double *, int *),    double *time,    double st[],    double dst[],    double param[],    double step,    int neq,    double work[],    double *errest,    int *status);
void sdvinteg(void (drc_hat_ns::*func)(double, double *, double *, double *, int *),    double *time,    double st[],    double dst[],    double param[],    double dt,    double *step,    int neqin,    double tol,    double work[],    int *err,    int *which);
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
        
  //      drc_hat_ns operator= (drc_hat_ns);
        
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

	
	//#include "drc_hat_ns_ext.h"

		//double **M;
        
//         double * get_q(int i);
//         double * get_u(int i);
//         double * get_qdot(int i);
//         double * get_udot(int i);
//         double *q_local, *u_local, *qdot_local, *udot_local, *u_zero;
};
#else
typedef struct drc_hat_ns drc_hat_ns;
#endif



#endif
