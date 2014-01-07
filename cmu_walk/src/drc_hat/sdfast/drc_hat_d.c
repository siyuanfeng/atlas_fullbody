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


ROADMAP (drc_hat.sd)

Bodies        Inb
No  Name      body Joint type  Coords q         Multipliers
--- --------- ---- ----------- ---------------- -----------------------
 -1 $ground                                    |
  0 pelvis     -1  Sixdof        0?  1?  2?    |
                   ...           3?  4?  5? 21 |  0p  1p  2p  3p  4p  5p
  1 ltorso      0  Pin           6?            |  6p
  2 mtorso      1  Pin           7?            |  7p
  3 hat         2  Pin           8?            |  8p
  4 l_uglut     0  Pin           9?            |  9p
  5 l_lglut     4  Pin          10?            | 10p
  6 l_uleg      5  Pin          11?            | 11p
  7 l_lleg      6  Pin          12?            | 12p
  8 l_talus     7  Pin          13?            | 13p
  9 l_foot      8  Pin          14?            | 14p
 10 r_uglut     0  Pin          15?            | 15p
 11 r_lglut    10  Pin          16?            | 16p
 12 r_uleg     11  Pin          17?            | 17p
 13 r_lleg     12  Pin          18?            | 18p
 14 r_talus    13  Pin          19?            | 19p
 15 r_foot     14  Pin          20?            | 20p

*/
#include <math.h>
#include <stdio.h>

typedef struct {
    int ground_,nbod_,ndof_,ncons_,nloop_,nldof_,nloopc_,nball_,nlball_,npres_,
      nuser_;
    int jtype_[16],inb_[16],outb_[16],njntdof_[16],njntc_[16],njntp_[16],firstq_
      [16],ballq_[16],firstm_[16],firstp_[16];
    int trans_[21];
} sdgtopo_t;
#define ground (sdgtopo.ground_)
#define nbod (sdgtopo.nbod_)
#define ndof (sdgtopo.ndof_)
#define ncons (sdgtopo.ncons_)
#define nloop (sdgtopo.nloop_)
#define nldof (sdgtopo.nldof_)
#define nloopc (sdgtopo.nloopc_)
#define nball (sdgtopo.nball_)
#define nlball (sdgtopo.nlball_)
#define npres (sdgtopo.npres_)
#define nuser (sdgtopo.nuser_)
#define jtype (sdgtopo.jtype_)
#define inb (sdgtopo.inb_)
#define outb (sdgtopo.outb_)
#define njntdof (sdgtopo.njntdof_)
#define njntc (sdgtopo.njntc_)
#define njntp (sdgtopo.njntp_)
#define firstq (sdgtopo.firstq_)
#define ballq (sdgtopo.ballq_)
#define firstm (sdgtopo.firstm_)
#define firstp (sdgtopo.firstp_)
#define trans (sdgtopo.trans_)

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
} sdginput_t;
#define grav (sdginput.grav_)
#define mk (sdginput.mk_)
#define ik (sdginput.ik_)
#define pin (sdginput.pin_)
#define rk (sdginput.rk_)
#define ri (sdginput.ri_)
#define pres (sdginput.pres_)
#define stabvel (sdginput.stabvel_)
#define stabpos (sdginput.stabpos_)
#define rhead (sdginput.rhead_)
#define rcom (sdginput.rcom_)
#define psrcomg (sdginput.psrcomg_)
#define psrcom (sdginput.psrcom_)
#define mkrcomt (sdginput.mkrcomt_)
#define psmk (sdginput.psmk_)
#define psik (sdginput.psik_)
#define psrk (sdginput.psrk_)
#define psri (sdginput.psri_)
#define psmkg (sdginput.psmkg_)
#define psikg (sdginput.psikg_)
#define psrkg (sdginput.psrkg_)
#define psrig (sdginput.psrig_)
#define mtot (sdginput.mtot_)
#define mfrcflg (sdginput.mfrcflg_)
#define roustate (sdginput.roustate_)
#define vpkflg (sdginput.vpkflg_)
#define inerflg (sdginput.inerflg_)
#define mmflg (sdginput.mmflg_)
#define mmlduflg (sdginput.mmlduflg_)
#define wwflg (sdginput.wwflg_)
#define ltauflg (sdginput.ltauflg_)
#define fs0flg (sdginput.fs0flg_)
#define ii (sdginput.ii_)
#define mmap (sdginput.mmap_)
#define gravq (sdginput.gravq_)
#define mkq (sdginput.mkq_)
#define ikq (sdginput.ikq_)
#define pinq (sdginput.pinq_)
#define rkq (sdginput.rkq_)
#define riq (sdginput.riq_)
#define presq (sdginput.presq_)
#define stabvelq (sdginput.stabvelq_)
#define stabposq (sdginput.stabposq_)

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
} sdgstate_t;
#define curtim (sdgstate.curtim_)
#define q (sdgstate.q_)
#define qn (sdgstate.qn_)
#define u (sdgstate.u_)
#define cnk (sdgstate.cnk_)
#define cnb (sdgstate.cnb_)
#define rnkg (sdgstate.rnkg_)
#define rnk (sdgstate.rnk_)
#define rnb (sdgstate.rnb_)
#define vnk (sdgstate.vnk_)
#define vnb (sdgstate.vnb_)
#define wk (sdgstate.wk_)
#define wb (sdgstate.wb_)
#define com (sdgstate.com_)
#define Cik (sdgstate.Cik_)
#define Cib (sdgstate.Cib_)
#define rikt (sdgstate.rikt_)
#define Iko (sdgstate.Iko_)
#define mkrk (sdgstate.mkrk_)
#define Wkk (sdgstate.Wkk_)
#define Vkk (sdgstate.Vkk_)
#define dik (sdgstate.dik_)
#define rpp (sdgstate.rpp_)
#define rpk (sdgstate.rpk_)
#define rik (sdgstate.rik_)
#define rik2 (sdgstate.rik2_)
#define rpri (sdgstate.rpri_)
#define Wik (sdgstate.Wik_)
#define Vik (sdgstate.Vik_)
#define Wirk (sdgstate.Wirk_)
#define rkWkk (sdgstate.rkWkk_)
#define Wkrpk (sdgstate.Wkrpk_)
#define VikWkr (sdgstate.VikWkr_)
#define wbrcom (sdgstate.wbrcom_)
#define perr (sdgstate.perr_)
#define verr (sdgstate.verr_)
#define aerr (sdgstate.aerr_)
#define mult (sdgstate.mult_)
#define ufk (sdgstate.ufk_)
#define utk (sdgstate.utk_)
#define utau (sdgstate.utau_)
#define mfk (sdgstate.mfk_)
#define mtk (sdgstate.mtk_)
#define mtau (sdgstate.mtau_)
#define uacc (sdgstate.uacc_)
#define uvel (sdgstate.uvel_)
#define upos (sdgstate.upos_)
#define s6 (sdgstate.s6_)
#define c6 (sdgstate.c6_)
#define s7 (sdgstate.s7_)
#define c7 (sdgstate.c7_)
#define s8 (sdgstate.s8_)
#define c8 (sdgstate.c8_)
#define s9 (sdgstate.s9_)
#define c9 (sdgstate.c9_)
#define s10 (sdgstate.s10_)
#define c10 (sdgstate.c10_)
#define s11 (sdgstate.s11_)
#define c11 (sdgstate.c11_)
#define s12 (sdgstate.s12_)
#define c12 (sdgstate.c12_)
#define s13 (sdgstate.s13_)
#define c13 (sdgstate.c13_)
#define s14 (sdgstate.s14_)
#define c14 (sdgstate.c14_)
#define s15 (sdgstate.s15_)
#define c15 (sdgstate.c15_)
#define s16 (sdgstate.s16_)
#define c16 (sdgstate.c16_)
#define s17 (sdgstate.s17_)
#define c17 (sdgstate.c17_)
#define s18 (sdgstate.s18_)
#define c18 (sdgstate.c18_)
#define s19 (sdgstate.s19_)
#define c19 (sdgstate.c19_)
#define s20 (sdgstate.s20_)
#define c20 (sdgstate.c20_)

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
} sdglhs_t;
#define qdot (sdglhs.qdot_)
#define Otk (sdglhs.Otk_)
#define Atk (sdglhs.Atk_)
#define AiOiWi (sdglhs.AiOiWi_)
#define Fstar (sdglhs.Fstar_)
#define Tstar (sdglhs.Tstar_)
#define fs0 (sdglhs.fs0_)
#define Fstark (sdglhs.Fstark_)
#define Tstark (sdglhs.Tstark_)
#define IkWk (sdglhs.IkWk_)
#define IkbWk (sdglhs.IkbWk_)
#define WkIkWk (sdglhs.WkIkWk_)
#define WkIkbWk (sdglhs.WkIkbWk_)
#define gk (sdglhs.gk_)
#define w0w0 (sdglhs.w0w0_)
#define w1w1 (sdglhs.w1w1_)
#define w2w2 (sdglhs.w2w2_)
#define w0w1 (sdglhs.w0w1_)
#define w0w2 (sdglhs.w0w2_)
#define w1w2 (sdglhs.w1w2_)
#define w00w11 (sdglhs.w00w11_)
#define w00w22 (sdglhs.w00w22_)
#define w11w22 (sdglhs.w11w22_)
#define ww (sdglhs.ww_)
#define qraux (sdglhs.qraux_)
#define mm (sdglhs.mm_)
#define mlo (sdglhs.mlo_)
#define mdi (sdglhs.mdi_)
#define IkWpk (sdglhs.IkWpk_)
#define works (sdglhs.works_)
#define workss (sdglhs.workss_)
#define Wpk (sdglhs.Wpk_)
#define Vpk (sdglhs.Vpk_)
#define VWri (sdglhs.VWri_)
#define wmap (sdglhs.wmap_)
#define multmap (sdglhs.multmap_)
#define jpvt (sdglhs.jpvt_)
#define wsiz (sdglhs.wsiz_)
#define wrank (sdglhs.wrank_)

typedef struct {
    double fs_[21],udot_[21],tauc_[21],dyad_[16][3][3],fc_[21][3],tc_[21][3];
    double ank_[21][3],onk_[21][3],Onkb_[21][3],AOnkri_[21][3],Ankb_[21][3],
      AnkAtk_[21][3],anb_[16][3],onb_[16][3],dyrcom_[16][3];
    double ffk_[21][3],ttk_[21][3],fccikt_[21][3],ffkb_[16][3],ttkb_[16][3];
} sdgrhs_t;
#define fs (sdgrhs.fs_)
#define udot (sdgrhs.udot_)
#define ank (sdgrhs.ank_)
#define anb (sdgrhs.anb_)
#define onk (sdgrhs.onk_)
#define onb (sdgrhs.onb_)
#define Onkb (sdgrhs.Onkb_)
#define AOnkri (sdgrhs.AOnkri_)
#define Ankb (sdgrhs.Ankb_)
#define AnkAtk (sdgrhs.AnkAtk_)
#define dyrcom (sdgrhs.dyrcom_)
#define ffk (sdgrhs.ffk_)
#define ttk (sdgrhs.ttk_)
#define fccikt (sdgrhs.fccikt_)
#define ffkb (sdgrhs.ffkb_)
#define ttkb (sdgrhs.ttkb_)
#define dyad (sdgrhs.dyad_)
#define fc (sdgrhs.fc_)
#define tc (sdgrhs.tc_)
#define tauc (sdgrhs.tauc_)

typedef struct {
    double temp_[3000],tmat1_[3][3],tmat2_[3][3],tvec1_[3],tvec2_[3],tvec3_[3],
      tvec4_[3],tvec5_[3];
    double tsc1_,tsc2_,tsc3_;
} sdgtemp_t;
#define temp (sdgtemp.temp_)
#define tmat1 (sdgtemp.tmat1_)
#define tmat2 (sdgtemp.tmat2_)
#define tvec1 (sdgtemp.tvec1_)
#define tvec2 (sdgtemp.tvec2_)
#define tvec3 (sdgtemp.tvec3_)
#define tvec4 (sdgtemp.tvec4_)
#define tvec5 (sdgtemp.tvec5_)
#define tsc1 (sdgtemp.tsc1_)
#define tsc2 (sdgtemp.tsc2_)
#define tsc3 (sdgtemp.tsc3_)

sdgtopo_t sdgtopo = {
/*  Topological information
*/
    /* ground */ 1,
    /* nbod */ 16,
    /* ndof */ 21,
    /* ncons */ 21,
    /* nloop */ 0,
    /* nldof */ 0,
    /* nloopc */ 0,
    /* nball */ 1,
    /* nlball */ 0,
    /* npres */ 21,
    /* nuser */ 0,
    /* jtype[0] */ 6,
    /* jtype[1] */ 1,
    /* jtype[2] */ 1,
    /* jtype[3] */ 1,
    /* jtype[4] */ 1,
    /* jtype[5] */ 1,
    /* jtype[6] */ 1,
    /* jtype[7] */ 1,
    /* jtype[8] */ 1,
    /* jtype[9] */ 1,
    /* jtype[10] */ 1,
    /* jtype[11] */ 1,
    /* jtype[12] */ 1,
    /* jtype[13] */ 1,
    /* jtype[14] */ 1,
    /* jtype[15] */ 1,
    /* inb[0] */ -1,
    /* inb[1] */ 0,
    /* inb[2] */ 1,
    /* inb[3] */ 2,
    /* inb[4] */ 0,
    /* inb[5] */ 4,
    /* inb[6] */ 5,
    /* inb[7] */ 6,
    /* inb[8] */ 7,
    /* inb[9] */ 8,
    /* inb[10] */ 0,
    /* inb[11] */ 10,
    /* inb[12] */ 11,
    /* inb[13] */ 12,
    /* inb[14] */ 13,
    /* inb[15] */ 14,
    /* outb[0] */ 0,
    /* outb[1] */ 1,
    /* outb[2] */ 2,
    /* outb[3] */ 3,
    /* outb[4] */ 4,
    /* outb[5] */ 5,
    /* outb[6] */ 6,
    /* outb[7] */ 7,
    /* outb[8] */ 8,
    /* outb[9] */ 9,
    /* outb[10] */ 10,
    /* outb[11] */ 11,
    /* outb[12] */ 12,
    /* outb[13] */ 13,
    /* outb[14] */ 14,
    /* outb[15] */ 15,
    /* njntdof[0] */ 6,
    /* njntdof[1] */ 1,
    /* njntdof[2] */ 1,
    /* njntdof[3] */ 1,
    /* njntdof[4] */ 1,
    /* njntdof[5] */ 1,
    /* njntdof[6] */ 1,
    /* njntdof[7] */ 1,
    /* njntdof[8] */ 1,
    /* njntdof[9] */ 1,
    /* njntdof[10] */ 1,
    /* njntdof[11] */ 1,
    /* njntdof[12] */ 1,
    /* njntdof[13] */ 1,
    /* njntdof[14] */ 1,
    /* njntdof[15] */ 1,
    /* njntc[0] */ 0,
    /* njntc[1] */ 0,
    /* njntc[2] */ 0,
    /* njntc[3] */ 0,
    /* njntc[4] */ 0,
    /* njntc[5] */ 0,
    /* njntc[6] */ 0,
    /* njntc[7] */ 0,
    /* njntc[8] */ 0,
    /* njntc[9] */ 0,
    /* njntc[10] */ 0,
    /* njntc[11] */ 0,
    /* njntc[12] */ 0,
    /* njntc[13] */ 0,
    /* njntc[14] */ 0,
    /* njntc[15] */ 0,
    /* njntp[0] */ 6,
    /* njntp[1] */ 1,
    /* njntp[2] */ 1,
    /* njntp[3] */ 1,
    /* njntp[4] */ 1,
    /* njntp[5] */ 1,
    /* njntp[6] */ 1,
    /* njntp[7] */ 1,
    /* njntp[8] */ 1,
    /* njntp[9] */ 1,
    /* njntp[10] */ 1,
    /* njntp[11] */ 1,
    /* njntp[12] */ 1,
    /* njntp[13] */ 1,
    /* njntp[14] */ 1,
    /* njntp[15] */ 1,
    /* firstq[0] */ 0,
    /* firstq[1] */ 6,
    /* firstq[2] */ 7,
    /* firstq[3] */ 8,
    /* firstq[4] */ 9,
    /* firstq[5] */ 10,
    /* firstq[6] */ 11,
    /* firstq[7] */ 12,
    /* firstq[8] */ 13,
    /* firstq[9] */ 14,
    /* firstq[10] */ 15,
    /* firstq[11] */ 16,
    /* firstq[12] */ 17,
    /* firstq[13] */ 18,
    /* firstq[14] */ 19,
    /* firstq[15] */ 20,
    /* ballq[0] */ 21,
    /* ballq[1] */ -104,
    /* ballq[2] */ -104,
    /* ballq[3] */ -104,
    /* ballq[4] */ -104,
    /* ballq[5] */ -104,
    /* ballq[6] */ -104,
    /* ballq[7] */ -104,
    /* ballq[8] */ -104,
    /* ballq[9] */ -104,
    /* ballq[10] */ -104,
    /* ballq[11] */ -104,
    /* ballq[12] */ -104,
    /* ballq[13] */ -104,
    /* ballq[14] */ -104,
    /* ballq[15] */ -104,
    /* firstm[0] */ -1,
    /* firstm[1] */ -1,
    /* firstm[2] */ -1,
    /* firstm[3] */ -1,
    /* firstm[4] */ -1,
    /* firstm[5] */ -1,
    /* firstm[6] */ -1,
    /* firstm[7] */ -1,
    /* firstm[8] */ -1,
    /* firstm[9] */ -1,
    /* firstm[10] */ -1,
    /* firstm[11] */ -1,
    /* firstm[12] */ -1,
    /* firstm[13] */ -1,
    /* firstm[14] */ -1,
    /* firstm[15] */ -1,
    /* firstp[0] */ 0,
    /* firstp[1] */ 6,
    /* firstp[2] */ 7,
    /* firstp[3] */ 8,
    /* firstp[4] */ 9,
    /* firstp[5] */ 10,
    /* firstp[6] */ 11,
    /* firstp[7] */ 12,
    /* firstp[8] */ 13,
    /* firstp[9] */ 14,
    /* firstp[10] */ 15,
    /* firstp[11] */ 16,
    /* firstp[12] */ 17,
    /* firstp[13] */ 18,
    /* firstp[14] */ 19,
    /* firstp[15] */ 20,
    /* trans[0] */ 1,
    /* trans[1] */ 1,
    /* trans[2] */ 1,
    /* trans[3] */ 0,
    /* trans[4] */ 0,
    /* trans[5] */ 0,
    /* trans[6] */ 0,
    /* trans[7] */ 0,
    /* trans[8] */ 0,
    /* trans[9] */ 0,
    /* trans[10] */ 0,
    /* trans[11] */ 0,
    /* trans[12] */ 0,
    /* trans[13] */ 0,
    /* trans[14] */ 0,
    /* trans[15] */ 0,
    /* trans[16] */ 0,
    /* trans[17] */ 0,
    /* trans[18] */ 0,
    /* trans[19] */ 0,
    /* trans[20] */ 0,
};
sdginput_t sdginput = {
/* Model parameters from the input file */

/* gravity */
    /* grav[0] */ 0.,
    /* grav[1] */ 0.,
    /* grav[2] */ -9.81,

/* mass */
    /* mk[0] */ 17.982,
    /* mk[1] */ 2.409,
    /* mk[2] */ .69,
    /* mk[3] */ 94.62,
    /* mk[4] */ .648,
    /* mk[5] */ .866,
    /* mk[6] */ 9.209,
    /* mk[7] */ 5.479,
    /* mk[8] */ .125,
    /* mk[9] */ 2.05,
    /* mk[10] */ .648,
    /* mk[11] */ .866,
    /* mk[12] */ 9.209,
    /* mk[13] */ 5.479,
    /* mk[14] */ .125,
    /* mk[15] */ 2.05,

/* inertia */
    /* ik[0][0][0] */ .125569,
    /* ik[0][0][1] */ .0008,
    /* ik[0][0][2] */ -.000499757,
    /* ik[0][1][0] */ .0008,
    /* ik[0][1][1] */ .0972062,
    /* ik[0][1][2] */ -.0005,
    /* ik[0][2][0] */ -.000499757,
    /* ik[0][2][1] */ -.0005,
    /* ik[0][2][2] */ .117937,
    /* ik[1][0][0] */ .0039092,
    /* ik[1][0][1] */ -5.04491e-8,
    /* ik[1][0][2] */ -.000342157,
    /* ik[1][1][0] */ -5.04491e-8,
    /* ik[1][1][1] */ .00341694,
    /* ik[1][1][2] */ 4.87119e-7,
    /* ik[1][2][0] */ -.000342157,
    /* ik[1][2][1] */ 4.87119e-7,
    /* ik[1][2][2] */ .00174492,
    /* ik[2][0][0] */ .000454181,
    /* ik[2][0][1] */ -6.10764e-5,
    /* ik[2][0][2] */ 3.94009e-5,
    /* ik[2][1][0] */ -6.10764e-5,
    /* ik[2][1][1] */ .000483282,
    /* ik[2][1][2] */ 5.27463e-5,
    /* ik[2][2][0] */ 3.94009e-5,
    /* ik[2][2][1] */ 5.27463e-5,
    /* ik[2][2][2] */ .000444215,
    /* ik[3][0][0] */ 8.1442,
    /* ik[3][0][1] */ .00361008,
    /* ik[3][0][2] */ .91218,
    /* ik[3][1][0] */ .00361008,
    /* ik[3][1][1] */ 4.74997,
    /* ik[3][1][2] */ .000982215,
    /* ik[3][2][0] */ .91218,
    /* ik[3][2][1] */ .000982215,
    /* ik[3][2][2] */ 5.72776,
    /* ik[4][0][0] */ .00074276,
    /* ik[4][0][1] */ -3.79607e-8,
    /* ik[4][0][2] */ -2.79549e-5,
    /* ik[4][1][0] */ -3.79607e-8,
    /* ik[4][1][1] */ .000688179,
    /* ik[4][1][2] */ -3.2735e-8,
    /* ik[4][2][0] */ -2.79549e-5,
    /* ik[4][2][1] */ -3.2735e-8,
    /* ik[4][2][2] */ .00041242,
    /* ik[5][0][0] */ .000691326,
    /* ik[5][0][1] */ -2.24344e-5,
    /* ik[5][0][2] */ 2.50508e-6,
    /* ik[5][1][0] */ -2.24344e-5,
    /* ik[5][1][1] */ .00126856,
    /* ik[5][1][2] */ .000137862,
    /* ik[5][2][0] */ 2.50508e-6,
    /* ik[5][2][1] */ .000137862,
    /* ik[5][2][2] */ .00106487,
    /* ik[6][0][0] */ .09,
    /* ik[6][0][1] */ 0.,
    /* ik[6][0][2] */ 0.,
    /* ik[6][1][0] */ 0.,
    /* ik[6][1][1] */ .09,
    /* ik[6][1][2] */ 0.,
    /* ik[6][2][0] */ 0.,
    /* ik[6][2][1] */ 0.,
    /* ik[6][2][2] */ .02,
    /* ik[7][0][0] */ .077,
    /* ik[7][0][1] */ 0.,
    /* ik[7][0][2] */ -.003,
    /* ik[7][1][0] */ 0.,
    /* ik[7][1][1] */ .076,
    /* ik[7][1][2] */ 0.,
    /* ik[7][2][0] */ -.003,
    /* ik[7][2][1] */ 0.,
    /* ik[7][2][2] */ .01,
    /* ik[8][0][0] */ 1.01674e-5,
    /* ik[8][0][1] */ 0.,
    /* ik[8][0][2] */ 0.,
    /* ik[8][1][0] */ 0.,
    /* ik[8][1][1] */ 8.42775e-6,
    /* ik[8][1][2] */ 0.,
    /* ik[8][2][0] */ 0.,
    /* ik[8][2][1] */ 0.,
    /* ik[8][2][2] */ 1.30101e-5,
    /* ik[9][0][0] */ .002,
    /* ik[9][0][1] */ 0.,
    /* ik[9][0][2] */ 0.,
    /* ik[9][1][0] */ 0.,
    /* ik[9][1][1] */ .007,
    /* ik[9][1][2] */ 0.,
    /* ik[9][2][0] */ 0.,
    /* ik[9][2][1] */ 0.,
    /* ik[9][2][2] */ .008,
    /* ik[10][0][0] */ .00074276,
    /* ik[10][0][1] */ 3.79607e-8,
    /* ik[10][0][2] */ -2.79549e-5,
    /* ik[10][1][0] */ 3.79607e-8,
    /* ik[10][1][1] */ .000688179,
    /* ik[10][1][2] */ 3.2735e-8,
    /* ik[10][2][0] */ -2.79549e-5,
    /* ik[10][2][1] */ 3.2735e-8,
    /* ik[10][2][2] */ .00041242,
    /* ik[11][0][0] */ .000691326,
    /* ik[11][0][1] */ 2.24344e-5,
    /* ik[11][0][2] */ 2.50508e-6,
    /* ik[11][1][0] */ 2.24344e-5,
    /* ik[11][1][1] */ .00126856,
    /* ik[11][1][2] */ -.000137862,
    /* ik[11][2][0] */ 2.50508e-6,
    /* ik[11][2][1] */ -.000137862,
    /* ik[11][2][2] */ .00106487,
    /* ik[12][0][0] */ .09,
    /* ik[12][0][1] */ 0.,
    /* ik[12][0][2] */ 0.,
    /* ik[12][1][0] */ 0.,
    /* ik[12][1][1] */ .09,
    /* ik[12][1][2] */ 0.,
    /* ik[12][2][0] */ 0.,
    /* ik[12][2][1] */ 0.,
    /* ik[12][2][2] */ .02,
    /* ik[13][0][0] */ .077,
    /* ik[13][0][1] */ 0.,
    /* ik[13][0][2] */ -.003,
    /* ik[13][1][0] */ 0.,
    /* ik[13][1][1] */ .076,
    /* ik[13][1][2] */ 0.,
    /* ik[13][2][0] */ -.003,
    /* ik[13][2][1] */ 0.,
    /* ik[13][2][2] */ .01,
    /* ik[14][0][0] */ 1.01674e-5,
    /* ik[14][0][1] */ 0.,
    /* ik[14][0][2] */ 0.,
    /* ik[14][1][0] */ 0.,
    /* ik[14][1][1] */ 8.42775e-6,
    /* ik[14][1][2] */ 0.,
    /* ik[14][2][0] */ 0.,
    /* ik[14][2][1] */ 0.,
    /* ik[14][2][2] */ 1.30101e-5,
    /* ik[15][0][0] */ .002,
    /* ik[15][0][1] */ 0.,
    /* ik[15][0][2] */ 0.,
    /* ik[15][1][0] */ 0.,
    /* ik[15][1][1] */ .007,
    /* ik[15][1][2] */ 0.,
    /* ik[15][2][0] */ 0.,
    /* ik[15][2][1] */ 0.,
    /* ik[15][2][2] */ .008,

/* tree hinge axis vectors */
    /* pin[0][0] */ 1.,
    /* pin[0][1] */ 0.,
    /* pin[0][2] */ 0.,
    /* pin[1][0] */ 0.,
    /* pin[1][1] */ 1.,
    /* pin[1][2] */ 0.,
    /* pin[2][0] */ 0.,
    /* pin[2][1] */ 0.,
    /* pin[2][2] */ 1.,
    /* pin[3][0] */ 0.,
    /* pin[3][1] */ 0.,
    /* pin[3][2] */ 0.,
    /* pin[4][0] */ 0.,
    /* pin[4][1] */ 0.,
    /* pin[4][2] */ 0.,
    /* pin[5][0] */ 0.,
    /* pin[5][1] */ 0.,
    /* pin[5][2] */ 0.,
    /* pin[6][0] */ 0.,
    /* pin[6][1] */ 0.,
    /* pin[6][2] */ 1.,
    /* pin[7][0] */ 0.,
    /* pin[7][1] */ 1.,
    /* pin[7][2] */ 0.,
    /* pin[8][0] */ 1.,
    /* pin[8][1] */ 0.,
    /* pin[8][2] */ 0.,
    /* pin[9][0] */ 0.,
    /* pin[9][1] */ 0.,
    /* pin[9][2] */ 1.,
    /* pin[10][0] */ 1.,
    /* pin[10][1] */ 0.,
    /* pin[10][2] */ 0.,
    /* pin[11][0] */ 0.,
    /* pin[11][1] */ 1.,
    /* pin[11][2] */ 0.,
    /* pin[12][0] */ 0.,
    /* pin[12][1] */ 1.,
    /* pin[12][2] */ 0.,
    /* pin[13][0] */ 0.,
    /* pin[13][1] */ 1.,
    /* pin[13][2] */ 0.,
    /* pin[14][0] */ 1.,
    /* pin[14][1] */ 0.,
    /* pin[14][2] */ 0.,
    /* pin[15][0] */ 0.,
    /* pin[15][1] */ 0.,
    /* pin[15][2] */ 1.,
    /* pin[16][0] */ 1.,
    /* pin[16][1] */ 0.,
    /* pin[16][2] */ 0.,
    /* pin[17][0] */ 0.,
    /* pin[17][1] */ 1.,
    /* pin[17][2] */ 0.,
    /* pin[18][0] */ 0.,
    /* pin[18][1] */ 1.,
    /* pin[18][2] */ 0.,
    /* pin[19][0] */ 0.,
    /* pin[19][1] */ 1.,
    /* pin[19][2] */ 0.,
    /* pin[20][0] */ 1.,
    /* pin[20][1] */ 0.,
    /* pin[20][2] */ 0.,

/* tree bodytojoint vectors */
    /* rk[0][0] */ 0.,
    /* rk[0][1] */ 0.,
    /* rk[0][2] */ 0.,
    /* rk[1][0] */ .0112984,
    /* rk[1][1] */ 3.15366e-6,
    /* rk[1][2] */ -.0746835,
    /* rk[2][0] */ .00816266,
    /* rk[2][1] */ .0131245,
    /* rk[2][2] */ -.0305974,
    /* rk[3][0] */ .0334573,
    /* rk[3][1] */ -5.33638e-7,
    /* rk[3][2] */ -.210171,
    /* rk[4][0] */ -.00529262,
    /* rk[4][1] */ .00344732,
    /* rk[4][2] */ -.00313046,
    /* rk[5][0] */ -.0133341,
    /* rk[5][1] */ -.0170484,
    /* rk[5][2] */ .0312052,
    /* rk[6][0] */ 0.,
    /* rk[6][1] */ 0.,
    /* rk[6][2] */ .21,
    /* rk[7][0] */ -.001,
    /* rk[7][1] */ 0.,
    /* rk[7][2] */ .187,
    /* rk[8][0] */ 0.,
    /* rk[8][1] */ 0.,
    /* rk[8][2] */ 0.,
    /* rk[9][0] */ -.027,
    /* rk[9][1] */ 0.,
    /* rk[9][2] */ .067,
    /* rk[10][0] */ -.00529262,
    /* rk[10][1] */ -.00344732,
    /* rk[10][2] */ -.00313046,
    /* rk[11][0] */ -.0133341,
    /* rk[11][1] */ .0170484,
    /* rk[11][2] */ .0312052,
    /* rk[12][0] */ 0.,
    /* rk[12][1] */ 0.,
    /* rk[12][2] */ .21,
    /* rk[13][0] */ -.001,
    /* rk[13][1] */ 0.,
    /* rk[13][2] */ .187,
    /* rk[14][0] */ 0.,
    /* rk[14][1] */ 0.,
    /* rk[14][2] */ 0.,
    /* rk[15][0] */ -.027,
    /* rk[15][1] */ 0.,
    /* rk[15][2] */ .067,

/* tree inbtojoint vectors */
    /* ri[0][0] */ 0.,
    /* ri[0][1] */ 0.,
    /* ri[0][2] */ 0.,
    /* ri[1][0] */ -.0238715,
    /* ri[1][1] */ 0.,
    /* ri[1][2] */ -.0268706,
    /* ri[2][0] */ .0112984,
    /* ri[2][1] */ 3.15366e-6,
    /* ri[2][2] */ .0872865,
    /* ri[3][0] */ .00816266,
    /* ri[3][1] */ .0131245,
    /* ri[3][2] */ .0194026,
    /* ri[4][0] */ -.0113715,
    /* ri[4][1] */ .089,
    /* ri[4][2] */ -.0268706,
    /* ri[5][0] */ -.00529262,
    /* ri[5][1] */ .00344732,
    /* ri[5][2] */ -.00313046,
    /* ri[6][0] */ .0366659,
    /* ri[6][1] */ -.0170484,
    /* ri[6][2] */ -.0187948,
    /* ri[7][0] */ -.05,
    /* ri[7][1] */ 0.,
    /* ri[7][2] */ -.164,
    /* ri[8][0] */ -.001,
    /* ri[8][1] */ 0.,
    /* ri[8][2] */ -.235,
    /* ri[9][0] */ 0.,
    /* ri[9][1] */ 0.,
    /* ri[9][2] */ 0.,
    /* ri[10][0] */ -.0113715,
    /* ri[10][1] */ -.089,
    /* ri[10][2] */ -.0268706,
    /* ri[11][0] */ -.00529262,
    /* ri[11][1] */ -.00344732,
    /* ri[11][2] */ -.00313046,
    /* ri[12][0] */ .0366659,
    /* ri[12][1] */ .0170484,
    /* ri[12][2] */ -.0187948,
    /* ri[13][0] */ -.05,
    /* ri[13][1] */ 0.,
    /* ri[13][2] */ -.164,
    /* ri[14][0] */ -.001,
    /* ri[14][1] */ 0.,
    /* ri[14][2] */ -.235,
    /* ri[15][0] */ 0.,
    /* ri[15][1] */ 0.,
    /* ri[15][2] */ 0.,

/* tree prescribed motion */
    /* pres[0] */ 0.,
    /* pres[1] */ 0.,
    /* pres[2] */ 0.,
    /* pres[3] */ 0.,
    /* pres[4] */ 0.,
    /* pres[5] */ 0.,
    /* pres[6] */ 0.,
    /* pres[7] */ 0.,
    /* pres[8] */ 0.,
    /* pres[9] */ 0.,
    /* pres[10] */ 0.,
    /* pres[11] */ 0.,
    /* pres[12] */ 0.,
    /* pres[13] */ 0.,
    /* pres[14] */ 0.,
    /* pres[15] */ 0.,
    /* pres[16] */ 0.,
    /* pres[17] */ 0.,
    /* pres[18] */ 0.,
    /* pres[19] */ 0.,
    /* pres[20] */ 0.,

/* stabilization parameters */
    /* stabvel */ 0.,
    /* stabpos */ 0.,

/* miscellaneous */
    /* mfrcflg */ 0,
    /* roustate */ 0,
    /* vpkflg */ 0,
    /* inerflg */ 0,
    /* mmflg */ 0,
    /* mmlduflg */ 0,
    /* wwflg */ 0,
    /* ltauflg */ 0,
    /* fs0flg */ 0,
    /* ii */ 0,
    /* mmap[0] */ 0,
    /* mmap[1] */ 1,
    /* mmap[2] */ 2,
    /* mmap[3] */ 3,
    /* mmap[4] */ 4,
    /* mmap[5] */ 5,
    /* mmap[6] */ 6,
    /* mmap[7] */ 7,
    /* mmap[8] */ 8,
    /* mmap[9] */ 9,
    /* mmap[10] */ 10,
    /* mmap[11] */ 11,
    /* mmap[12] */ 12,
    /* mmap[13] */ 13,
    /* mmap[14] */ 14,
    /* mmap[15] */ 15,
    /* mmap[16] */ 16,
    /* mmap[17] */ 17,
    /* mmap[18] */ 18,
    /* mmap[19] */ 19,
    /* mmap[20] */ 20,

/* Which parameters were "?" (1) or "<nominal>?" (3) */
    /* gravq[0] */ 3,
    /* gravq[1] */ 3,
    /* gravq[2] */ 3,
    /* mkq[0] */ 3,
    /* mkq[1] */ 3,
    /* mkq[2] */ 3,
    /* mkq[3] */ 3,
    /* mkq[4] */ 3,
    /* mkq[5] */ 3,
    /* mkq[6] */ 3,
    /* mkq[7] */ 3,
    /* mkq[8] */ 3,
    /* mkq[9] */ 3,
    /* mkq[10] */ 3,
    /* mkq[11] */ 3,
    /* mkq[12] */ 3,
    /* mkq[13] */ 3,
    /* mkq[14] */ 3,
    /* mkq[15] */ 3,
    /* ikq[0][0][0] */ 3,
    /* ikq[0][0][1] */ 3,
    /* ikq[0][0][2] */ 3,
    /* ikq[0][1][0] */ 3,
    /* ikq[0][1][1] */ 3,
    /* ikq[0][1][2] */ 3,
    /* ikq[0][2][0] */ 3,
    /* ikq[0][2][1] */ 3,
    /* ikq[0][2][2] */ 3,
    /* ikq[1][0][0] */ 3,
    /* ikq[1][0][1] */ 3,
    /* ikq[1][0][2] */ 3,
    /* ikq[1][1][0] */ 3,
    /* ikq[1][1][1] */ 3,
    /* ikq[1][1][2] */ 3,
    /* ikq[1][2][0] */ 3,
    /* ikq[1][2][1] */ 3,
    /* ikq[1][2][2] */ 3,
    /* ikq[2][0][0] */ 3,
    /* ikq[2][0][1] */ 3,
    /* ikq[2][0][2] */ 3,
    /* ikq[2][1][0] */ 3,
    /* ikq[2][1][1] */ 3,
    /* ikq[2][1][2] */ 3,
    /* ikq[2][2][0] */ 3,
    /* ikq[2][2][1] */ 3,
    /* ikq[2][2][2] */ 3,
    /* ikq[3][0][0] */ 3,
    /* ikq[3][0][1] */ 3,
    /* ikq[3][0][2] */ 3,
    /* ikq[3][1][0] */ 3,
    /* ikq[3][1][1] */ 3,
    /* ikq[3][1][2] */ 3,
    /* ikq[3][2][0] */ 3,
    /* ikq[3][2][1] */ 3,
    /* ikq[3][2][2] */ 3,
    /* ikq[4][0][0] */ 3,
    /* ikq[4][0][1] */ 3,
    /* ikq[4][0][2] */ 3,
    /* ikq[4][1][0] */ 3,
    /* ikq[4][1][1] */ 3,
    /* ikq[4][1][2] */ 3,
    /* ikq[4][2][0] */ 3,
    /* ikq[4][2][1] */ 3,
    /* ikq[4][2][2] */ 3,
    /* ikq[5][0][0] */ 3,
    /* ikq[5][0][1] */ 3,
    /* ikq[5][0][2] */ 3,
    /* ikq[5][1][0] */ 3,
    /* ikq[5][1][1] */ 3,
    /* ikq[5][1][2] */ 3,
    /* ikq[5][2][0] */ 3,
    /* ikq[5][2][1] */ 3,
    /* ikq[5][2][2] */ 3,
    /* ikq[6][0][0] */ 3,
    /* ikq[6][0][1] */ 3,
    /* ikq[6][0][2] */ 3,
    /* ikq[6][1][0] */ 3,
    /* ikq[6][1][1] */ 3,
    /* ikq[6][1][2] */ 3,
    /* ikq[6][2][0] */ 3,
    /* ikq[6][2][1] */ 3,
    /* ikq[6][2][2] */ 3,
    /* ikq[7][0][0] */ 3,
    /* ikq[7][0][1] */ 3,
    /* ikq[7][0][2] */ 3,
    /* ikq[7][1][0] */ 3,
    /* ikq[7][1][1] */ 3,
    /* ikq[7][1][2] */ 3,
    /* ikq[7][2][0] */ 3,
    /* ikq[7][2][1] */ 3,
    /* ikq[7][2][2] */ 3,
    /* ikq[8][0][0] */ 3,
    /* ikq[8][0][1] */ 3,
    /* ikq[8][0][2] */ 3,
    /* ikq[8][1][0] */ 3,
    /* ikq[8][1][1] */ 3,
    /* ikq[8][1][2] */ 3,
    /* ikq[8][2][0] */ 3,
    /* ikq[8][2][1] */ 3,
    /* ikq[8][2][2] */ 3,
    /* ikq[9][0][0] */ 3,
    /* ikq[9][0][1] */ 3,
    /* ikq[9][0][2] */ 3,
    /* ikq[9][1][0] */ 3,
    /* ikq[9][1][1] */ 3,
    /* ikq[9][1][2] */ 3,
    /* ikq[9][2][0] */ 3,
    /* ikq[9][2][1] */ 3,
    /* ikq[9][2][2] */ 3,
    /* ikq[10][0][0] */ 3,
    /* ikq[10][0][1] */ 3,
    /* ikq[10][0][2] */ 3,
    /* ikq[10][1][0] */ 3,
    /* ikq[10][1][1] */ 3,
    /* ikq[10][1][2] */ 3,
    /* ikq[10][2][0] */ 3,
    /* ikq[10][2][1] */ 3,
    /* ikq[10][2][2] */ 3,
    /* ikq[11][0][0] */ 3,
    /* ikq[11][0][1] */ 3,
    /* ikq[11][0][2] */ 3,
    /* ikq[11][1][0] */ 3,
    /* ikq[11][1][1] */ 3,
    /* ikq[11][1][2] */ 3,
    /* ikq[11][2][0] */ 3,
    /* ikq[11][2][1] */ 3,
    /* ikq[11][2][2] */ 3,
    /* ikq[12][0][0] */ 3,
    /* ikq[12][0][1] */ 3,
    /* ikq[12][0][2] */ 3,
    /* ikq[12][1][0] */ 3,
    /* ikq[12][1][1] */ 3,
    /* ikq[12][1][2] */ 3,
    /* ikq[12][2][0] */ 3,
    /* ikq[12][2][1] */ 3,
    /* ikq[12][2][2] */ 3,
    /* ikq[13][0][0] */ 3,
    /* ikq[13][0][1] */ 3,
    /* ikq[13][0][2] */ 3,
    /* ikq[13][1][0] */ 3,
    /* ikq[13][1][1] */ 3,
    /* ikq[13][1][2] */ 3,
    /* ikq[13][2][0] */ 3,
    /* ikq[13][2][1] */ 3,
    /* ikq[13][2][2] */ 3,
    /* ikq[14][0][0] */ 3,
    /* ikq[14][0][1] */ 3,
    /* ikq[14][0][2] */ 3,
    /* ikq[14][1][0] */ 3,
    /* ikq[14][1][1] */ 3,
    /* ikq[14][1][2] */ 3,
    /* ikq[14][2][0] */ 3,
    /* ikq[14][2][1] */ 3,
    /* ikq[14][2][2] */ 3,
    /* ikq[15][0][0] */ 3,
    /* ikq[15][0][1] */ 3,
    /* ikq[15][0][2] */ 3,
    /* ikq[15][1][0] */ 3,
    /* ikq[15][1][1] */ 3,
    /* ikq[15][1][2] */ 3,
    /* ikq[15][2][0] */ 3,
    /* ikq[15][2][1] */ 3,
    /* ikq[15][2][2] */ 3,
    /* pinq[0][0] */ 3,
    /* pinq[0][1] */ 3,
    /* pinq[0][2] */ 3,
    /* pinq[1][0] */ 3,
    /* pinq[1][1] */ 3,
    /* pinq[1][2] */ 3,
    /* pinq[2][0] */ 3,
    /* pinq[2][1] */ 3,
    /* pinq[2][2] */ 3,
    /* pinq[3][0] */ 0,
    /* pinq[3][1] */ 0,
    /* pinq[3][2] */ 0,
    /* pinq[4][0] */ 0,
    /* pinq[4][1] */ 0,
    /* pinq[4][2] */ 0,
    /* pinq[5][0] */ 0,
    /* pinq[5][1] */ 0,
    /* pinq[5][2] */ 0,
    /* pinq[6][0] */ 3,
    /* pinq[6][1] */ 3,
    /* pinq[6][2] */ 3,
    /* pinq[7][0] */ 3,
    /* pinq[7][1] */ 3,
    /* pinq[7][2] */ 3,
    /* pinq[8][0] */ 3,
    /* pinq[8][1] */ 3,
    /* pinq[8][2] */ 3,
    /* pinq[9][0] */ 3,
    /* pinq[9][1] */ 3,
    /* pinq[9][2] */ 3,
    /* pinq[10][0] */ 3,
    /* pinq[10][1] */ 3,
    /* pinq[10][2] */ 3,
    /* pinq[11][0] */ 3,
    /* pinq[11][1] */ 3,
    /* pinq[11][2] */ 3,
    /* pinq[12][0] */ 3,
    /* pinq[12][1] */ 3,
    /* pinq[12][2] */ 3,
    /* pinq[13][0] */ 3,
    /* pinq[13][1] */ 3,
    /* pinq[13][2] */ 3,
    /* pinq[14][0] */ 3,
    /* pinq[14][1] */ 3,
    /* pinq[14][2] */ 3,
    /* pinq[15][0] */ 3,
    /* pinq[15][1] */ 3,
    /* pinq[15][2] */ 3,
    /* pinq[16][0] */ 3,
    /* pinq[16][1] */ 3,
    /* pinq[16][2] */ 3,
    /* pinq[17][0] */ 3,
    /* pinq[17][1] */ 3,
    /* pinq[17][2] */ 3,
    /* pinq[18][0] */ 3,
    /* pinq[18][1] */ 3,
    /* pinq[18][2] */ 3,
    /* pinq[19][0] */ 3,
    /* pinq[19][1] */ 3,
    /* pinq[19][2] */ 3,
    /* pinq[20][0] */ 3,
    /* pinq[20][1] */ 3,
    /* pinq[20][2] */ 3,
    /* rkq[0][0] */ 3,
    /* rkq[0][1] */ 3,
    /* rkq[0][2] */ 3,
    /* rkq[1][0] */ 3,
    /* rkq[1][1] */ 3,
    /* rkq[1][2] */ 3,
    /* rkq[2][0] */ 3,
    /* rkq[2][1] */ 3,
    /* rkq[2][2] */ 3,
    /* rkq[3][0] */ 3,
    /* rkq[3][1] */ 3,
    /* rkq[3][2] */ 3,
    /* rkq[4][0] */ 3,
    /* rkq[4][1] */ 3,
    /* rkq[4][2] */ 3,
    /* rkq[5][0] */ 3,
    /* rkq[5][1] */ 3,
    /* rkq[5][2] */ 3,
    /* rkq[6][0] */ 3,
    /* rkq[6][1] */ 3,
    /* rkq[6][2] */ 3,
    /* rkq[7][0] */ 3,
    /* rkq[7][1] */ 3,
    /* rkq[7][2] */ 3,
    /* rkq[8][0] */ 3,
    /* rkq[8][1] */ 3,
    /* rkq[8][2] */ 3,
    /* rkq[9][0] */ 3,
    /* rkq[9][1] */ 3,
    /* rkq[9][2] */ 3,
    /* rkq[10][0] */ 3,
    /* rkq[10][1] */ 3,
    /* rkq[10][2] */ 3,
    /* rkq[11][0] */ 3,
    /* rkq[11][1] */ 3,
    /* rkq[11][2] */ 3,
    /* rkq[12][0] */ 3,
    /* rkq[12][1] */ 3,
    /* rkq[12][2] */ 3,
    /* rkq[13][0] */ 3,
    /* rkq[13][1] */ 3,
    /* rkq[13][2] */ 3,
    /* rkq[14][0] */ 3,
    /* rkq[14][1] */ 3,
    /* rkq[14][2] */ 3,
    /* rkq[15][0] */ 3,
    /* rkq[15][1] */ 3,
    /* rkq[15][2] */ 3,
    /* riq[0][0] */ 3,
    /* riq[0][1] */ 3,
    /* riq[0][2] */ 3,
    /* riq[1][0] */ 3,
    /* riq[1][1] */ 3,
    /* riq[1][2] */ 3,
    /* riq[2][0] */ 3,
    /* riq[2][1] */ 3,
    /* riq[2][2] */ 3,
    /* riq[3][0] */ 3,
    /* riq[3][1] */ 3,
    /* riq[3][2] */ 3,
    /* riq[4][0] */ 3,
    /* riq[4][1] */ 3,
    /* riq[4][2] */ 3,
    /* riq[5][0] */ 3,
    /* riq[5][1] */ 3,
    /* riq[5][2] */ 3,
    /* riq[6][0] */ 3,
    /* riq[6][1] */ 3,
    /* riq[6][2] */ 3,
    /* riq[7][0] */ 3,
    /* riq[7][1] */ 3,
    /* riq[7][2] */ 3,
    /* riq[8][0] */ 3,
    /* riq[8][1] */ 3,
    /* riq[8][2] */ 3,
    /* riq[9][0] */ 3,
    /* riq[9][1] */ 3,
    /* riq[9][2] */ 3,
    /* riq[10][0] */ 3,
    /* riq[10][1] */ 3,
    /* riq[10][2] */ 3,
    /* riq[11][0] */ 3,
    /* riq[11][1] */ 3,
    /* riq[11][2] */ 3,
    /* riq[12][0] */ 3,
    /* riq[12][1] */ 3,
    /* riq[12][2] */ 3,
    /* riq[13][0] */ 3,
    /* riq[13][1] */ 3,
    /* riq[13][2] */ 3,
    /* riq[14][0] */ 3,
    /* riq[14][1] */ 3,
    /* riq[14][2] */ 3,
    /* riq[15][0] */ 3,
    /* riq[15][1] */ 3,
    /* riq[15][2] */ 3,
    /* presq[0] */ 3,
    /* presq[1] */ 3,
    /* presq[2] */ 3,
    /* presq[3] */ 3,
    /* presq[4] */ 3,
    /* presq[5] */ 3,
    /* presq[6] */ 3,
    /* presq[7] */ 3,
    /* presq[8] */ 3,
    /* presq[9] */ 3,
    /* presq[10] */ 3,
    /* presq[11] */ 3,
    /* presq[12] */ 3,
    /* presq[13] */ 3,
    /* presq[14] */ 3,
    /* presq[15] */ 3,
    /* presq[16] */ 3,
    /* presq[17] */ 3,
    /* presq[18] */ 3,
    /* presq[19] */ 3,
    /* presq[20] */ 3,
    /* stabvelq */ 3,
    /* stabposq */ 3,

/* End of values from input file */

};
sdgstate_t sdgstate;
sdglhs_t sdglhs;
sdgrhs_t sdgrhs;
sdgtemp_t sdgtemp;


void sdinit(void)
{
/*
Initialization routine


 This routine must be called before the first call to sdstate(), after
 supplying values for any `?' parameters in the input.
*/
    double sumsq,norminv;
    int i,j,k;


/* Check that all `?' parameters have been assigned values */

    for (k = 0; k < 3; k++) {
        if (gravq[k] == 1) {
            sdseterr(7,25);
        }
    }
    for (k = 0; k < 16; k++) {
        if (mkq[k] == 1) {
            sdseterr(7,26);
        }
        for (i = 0; i < 3; i++) {
            if (rkq[k][i] == 1) {
                sdseterr(7,29);
            }
            if (riq[k][i] == 1) {
                sdseterr(7,30);
            }
            for (j = 0; j < 3; j++) {
                if (ikq[k][i][j] == 1) {
                    sdseterr(7,27);
                }
            }
        }
    }
    for (k = 0; k < 21; k++) {
        for (i = 0; i < 3; i++) {
            if (pinq[k][i] == 1) {
                sdseterr(7,28);
            }
        }
    }

/* Normalize pin vectors if necessary */

    sumsq = ((pin[0][2]*pin[0][2])+((pin[0][0]*pin[0][0])+(pin[0][1]*pin[0][1]))
      );
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[0][0] = (norminv*pin[0][0]);
    pin[0][1] = (norminv*pin[0][1]);
    pin[0][2] = (norminv*pin[0][2]);
    sumsq = ((pin[1][2]*pin[1][2])+((pin[1][0]*pin[1][0])+(pin[1][1]*pin[1][1]))
      );
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[1][0] = (norminv*pin[1][0]);
    pin[1][1] = (norminv*pin[1][1]);
    pin[1][2] = (norminv*pin[1][2]);
    sumsq = ((pin[2][2]*pin[2][2])+((pin[2][0]*pin[2][0])+(pin[2][1]*pin[2][1]))
      );
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[2][0] = (norminv*pin[2][0]);
    pin[2][1] = (norminv*pin[2][1]);
    pin[2][2] = (norminv*pin[2][2]);
    sumsq = ((pin[6][2]*pin[6][2])+((pin[6][0]*pin[6][0])+(pin[6][1]*pin[6][1]))
      );
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[6][0] = (norminv*pin[6][0]);
    pin[6][1] = (norminv*pin[6][1]);
    pin[6][2] = (norminv*pin[6][2]);
    sumsq = ((pin[7][2]*pin[7][2])+((pin[7][0]*pin[7][0])+(pin[7][1]*pin[7][1]))
      );
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[7][0] = (norminv*pin[7][0]);
    pin[7][1] = (norminv*pin[7][1]);
    pin[7][2] = (norminv*pin[7][2]);
    sumsq = ((pin[8][2]*pin[8][2])+((pin[8][0]*pin[8][0])+(pin[8][1]*pin[8][1]))
      );
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[8][0] = (norminv*pin[8][0]);
    pin[8][1] = (norminv*pin[8][1]);
    pin[8][2] = (norminv*pin[8][2]);
    sumsq = ((pin[9][2]*pin[9][2])+((pin[9][0]*pin[9][0])+(pin[9][1]*pin[9][1]))
      );
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[9][0] = (norminv*pin[9][0]);
    pin[9][1] = (norminv*pin[9][1]);
    pin[9][2] = (norminv*pin[9][2]);
    sumsq = ((pin[10][2]*pin[10][2])+((pin[10][0]*pin[10][0])+(pin[10][1]*
      pin[10][1])));
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[10][0] = (norminv*pin[10][0]);
    pin[10][1] = (norminv*pin[10][1]);
    pin[10][2] = (norminv*pin[10][2]);
    sumsq = ((pin[11][2]*pin[11][2])+((pin[11][0]*pin[11][0])+(pin[11][1]*
      pin[11][1])));
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[11][0] = (norminv*pin[11][0]);
    pin[11][1] = (norminv*pin[11][1]);
    pin[11][2] = (norminv*pin[11][2]);
    sumsq = ((pin[12][2]*pin[12][2])+((pin[12][0]*pin[12][0])+(pin[12][1]*
      pin[12][1])));
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[12][0] = (norminv*pin[12][0]);
    pin[12][1] = (norminv*pin[12][1]);
    pin[12][2] = (norminv*pin[12][2]);
    sumsq = ((pin[13][2]*pin[13][2])+((pin[13][0]*pin[13][0])+(pin[13][1]*
      pin[13][1])));
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[13][0] = (norminv*pin[13][0]);
    pin[13][1] = (norminv*pin[13][1]);
    pin[13][2] = (norminv*pin[13][2]);
    sumsq = ((pin[14][2]*pin[14][2])+((pin[14][0]*pin[14][0])+(pin[14][1]*
      pin[14][1])));
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[14][0] = (norminv*pin[14][0]);
    pin[14][1] = (norminv*pin[14][1]);
    pin[14][2] = (norminv*pin[14][2]);
    sumsq = ((pin[15][2]*pin[15][2])+((pin[15][0]*pin[15][0])+(pin[15][1]*
      pin[15][1])));
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[15][0] = (norminv*pin[15][0]);
    pin[15][1] = (norminv*pin[15][1]);
    pin[15][2] = (norminv*pin[15][2]);
    sumsq = ((pin[16][2]*pin[16][2])+((pin[16][0]*pin[16][0])+(pin[16][1]*
      pin[16][1])));
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[16][0] = (norminv*pin[16][0]);
    pin[16][1] = (norminv*pin[16][1]);
    pin[16][2] = (norminv*pin[16][2]);
    sumsq = ((pin[17][2]*pin[17][2])+((pin[17][0]*pin[17][0])+(pin[17][1]*
      pin[17][1])));
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[17][0] = (norminv*pin[17][0]);
    pin[17][1] = (norminv*pin[17][1]);
    pin[17][2] = (norminv*pin[17][2]);
    sumsq = ((pin[18][2]*pin[18][2])+((pin[18][0]*pin[18][0])+(pin[18][1]*
      pin[18][1])));
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[18][0] = (norminv*pin[18][0]);
    pin[18][1] = (norminv*pin[18][1]);
    pin[18][2] = (norminv*pin[18][2]);
    sumsq = ((pin[19][2]*pin[19][2])+((pin[19][0]*pin[19][0])+(pin[19][1]*
      pin[19][1])));
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[19][0] = (norminv*pin[19][0]);
    pin[19][1] = (norminv*pin[19][1]);
    pin[19][2] = (norminv*pin[19][2]);
    sumsq = ((pin[20][2]*pin[20][2])+((pin[20][0]*pin[20][0])+(pin[20][1]*
      pin[20][1])));
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[20][0] = (norminv*pin[20][0]);
    pin[20][1] = (norminv*pin[20][1]);
    pin[20][2] = (norminv*pin[20][2]);

/* Zero out Vpk and Wpk */

    for (i = 0; i < 21; i++) {
        for (j = i; j <= 20; j++) {
            for (k = 0; k < 3; k++) {
                Vpk[i][j][k] = 0.;
                Wpk[i][j][k] = 0.;
            }
        }
    }

/* Compute pseudobody-related constants */

    rcom[0][0] = 0.;
    rcom[0][1] = 0.;
    rcom[0][2] = 0.;
    rcom[1][0] = 0.;
    rcom[1][1] = 0.;
    rcom[1][2] = 0.;
    rcom[2][0] = 0.;
    rcom[2][1] = 0.;
    rcom[2][2] = 0.;
    rcom[3][0] = 0.;
    rcom[3][1] = 0.;
    rcom[3][2] = 0.;
    rcom[4][0] = 0.;
    rcom[4][1] = 0.;
    rcom[4][2] = 0.;
    rcom[5][0] = 0.;
    rcom[5][1] = 0.;
    rcom[5][2] = 0.;
    rcom[6][0] = 0.;
    rcom[6][1] = 0.;
    rcom[6][2] = 0.;
    rcom[7][0] = 0.;
    rcom[7][1] = 0.;
    rcom[7][2] = 0.;
    rcom[8][0] = 0.;
    rcom[8][1] = 0.;
    rcom[8][2] = 0.;
    rcom[9][0] = 0.;
    rcom[9][1] = 0.;
    rcom[9][2] = 0.;
    rcom[10][0] = 0.;
    rcom[10][1] = 0.;
    rcom[10][2] = 0.;
    rcom[11][0] = 0.;
    rcom[11][1] = 0.;
    rcom[11][2] = 0.;
    rcom[12][0] = 0.;
    rcom[12][1] = 0.;
    rcom[12][2] = 0.;
    rcom[13][0] = 0.;
    rcom[13][1] = 0.;
    rcom[13][2] = 0.;
    rcom[14][0] = 0.;
    rcom[14][1] = 0.;
    rcom[14][2] = 0.;
    rcom[15][0] = 0.;
    rcom[15][1] = 0.;
    rcom[15][2] = 0.;
    rkWkk[6][0] = ((pin[6][2]*rk[1][1])-(pin[6][1]*rk[1][2]));
    rkWkk[6][1] = ((pin[6][0]*rk[1][2])-(pin[6][2]*rk[1][0]));
    rkWkk[6][2] = ((pin[6][1]*rk[1][0])-(pin[6][0]*rk[1][1]));
    rkWkk[7][0] = ((pin[7][2]*rk[2][1])-(pin[7][1]*rk[2][2]));
    rkWkk[7][1] = ((pin[7][0]*rk[2][2])-(pin[7][2]*rk[2][0]));
    rkWkk[7][2] = ((pin[7][1]*rk[2][0])-(pin[7][0]*rk[2][1]));
    rkWkk[8][0] = ((pin[8][2]*rk[3][1])-(pin[8][1]*rk[3][2]));
    rkWkk[8][1] = ((pin[8][0]*rk[3][2])-(pin[8][2]*rk[3][0]));
    rkWkk[8][2] = ((pin[8][1]*rk[3][0])-(pin[8][0]*rk[3][1]));
    rkWkk[9][0] = ((pin[9][2]*rk[4][1])-(pin[9][1]*rk[4][2]));
    rkWkk[9][1] = ((pin[9][0]*rk[4][2])-(pin[9][2]*rk[4][0]));
    rkWkk[9][2] = ((pin[9][1]*rk[4][0])-(pin[9][0]*rk[4][1]));
    rkWkk[10][0] = ((pin[10][2]*rk[5][1])-(pin[10][1]*rk[5][2]));
    rkWkk[10][1] = ((pin[10][0]*rk[5][2])-(pin[10][2]*rk[5][0]));
    rkWkk[10][2] = ((pin[10][1]*rk[5][0])-(pin[10][0]*rk[5][1]));
    rkWkk[11][0] = ((pin[11][2]*rk[6][1])-(pin[11][1]*rk[6][2]));
    rkWkk[11][1] = ((pin[11][0]*rk[6][2])-(pin[11][2]*rk[6][0]));
    rkWkk[11][2] = ((pin[11][1]*rk[6][0])-(pin[11][0]*rk[6][1]));
    rkWkk[12][0] = ((pin[12][2]*rk[7][1])-(pin[12][1]*rk[7][2]));
    rkWkk[12][1] = ((pin[12][0]*rk[7][2])-(pin[12][2]*rk[7][0]));
    rkWkk[12][2] = ((pin[12][1]*rk[7][0])-(pin[12][0]*rk[7][1]));
    rkWkk[13][0] = ((pin[13][2]*rk[8][1])-(pin[13][1]*rk[8][2]));
    rkWkk[13][1] = ((pin[13][0]*rk[8][2])-(pin[13][2]*rk[8][0]));
    rkWkk[13][2] = ((pin[13][1]*rk[8][0])-(pin[13][0]*rk[8][1]));
    rkWkk[14][0] = ((pin[14][2]*rk[9][1])-(pin[14][1]*rk[9][2]));
    rkWkk[14][1] = ((pin[14][0]*rk[9][2])-(pin[14][2]*rk[9][0]));
    rkWkk[14][2] = ((pin[14][1]*rk[9][0])-(pin[14][0]*rk[9][1]));
    rkWkk[15][0] = ((pin[15][2]*rk[10][1])-(pin[15][1]*rk[10][2]));
    rkWkk[15][1] = ((pin[15][0]*rk[10][2])-(pin[15][2]*rk[10][0]));
    rkWkk[15][2] = ((pin[15][1]*rk[10][0])-(pin[15][0]*rk[10][1]));
    rkWkk[16][0] = ((pin[16][2]*rk[11][1])-(pin[16][1]*rk[11][2]));
    rkWkk[16][1] = ((pin[16][0]*rk[11][2])-(pin[16][2]*rk[11][0]));
    rkWkk[16][2] = ((pin[16][1]*rk[11][0])-(pin[16][0]*rk[11][1]));
    rkWkk[17][0] = ((pin[17][2]*rk[12][1])-(pin[17][1]*rk[12][2]));
    rkWkk[17][1] = ((pin[17][0]*rk[12][2])-(pin[17][2]*rk[12][0]));
    rkWkk[17][2] = ((pin[17][1]*rk[12][0])-(pin[17][0]*rk[12][1]));
    rkWkk[18][0] = ((pin[18][2]*rk[13][1])-(pin[18][1]*rk[13][2]));
    rkWkk[18][1] = ((pin[18][0]*rk[13][2])-(pin[18][2]*rk[13][0]));
    rkWkk[18][2] = ((pin[18][1]*rk[13][0])-(pin[18][0]*rk[13][1]));
    rkWkk[19][0] = ((pin[19][2]*rk[14][1])-(pin[19][1]*rk[14][2]));
    rkWkk[19][1] = ((pin[19][0]*rk[14][2])-(pin[19][2]*rk[14][0]));
    rkWkk[19][2] = ((pin[19][1]*rk[14][0])-(pin[19][0]*rk[14][1]));
    rkWkk[20][0] = ((pin[20][2]*rk[15][1])-(pin[20][1]*rk[15][2]));
    rkWkk[20][1] = ((pin[20][0]*rk[15][2])-(pin[20][2]*rk[15][0]));
    rkWkk[20][2] = ((pin[20][1]*rk[15][0])-(pin[20][0]*rk[15][1]));
    dik[6][0] = (ri[1][0]-rk[0][0]);
    dik[6][1] = (ri[1][1]-rk[0][1]);
    dik[6][2] = (ri[1][2]-rk[0][2]);
    dik[7][0] = (ri[2][0]-rk[1][0]);
    dik[7][1] = (ri[2][1]-rk[1][1]);
    dik[7][2] = (ri[2][2]-rk[1][2]);
    dik[8][0] = (ri[3][0]-rk[2][0]);
    dik[8][1] = (ri[3][1]-rk[2][1]);
    dik[8][2] = (ri[3][2]-rk[2][2]);
    dik[9][0] = (ri[4][0]-rk[0][0]);
    dik[9][1] = (ri[4][1]-rk[0][1]);
    dik[9][2] = (ri[4][2]-rk[0][2]);
    dik[10][0] = (ri[5][0]-rk[4][0]);
    dik[10][1] = (ri[5][1]-rk[4][1]);
    dik[10][2] = (ri[5][2]-rk[4][2]);
    dik[11][0] = (ri[6][0]-rk[5][0]);
    dik[11][1] = (ri[6][1]-rk[5][1]);
    dik[11][2] = (ri[6][2]-rk[5][2]);
    dik[12][0] = (ri[7][0]-rk[6][0]);
    dik[12][1] = (ri[7][1]-rk[6][1]);
    dik[12][2] = (ri[7][2]-rk[6][2]);
    dik[13][0] = (ri[8][0]-rk[7][0]);
    dik[13][1] = (ri[8][1]-rk[7][1]);
    dik[13][2] = (ri[8][2]-rk[7][2]);
    dik[14][0] = (ri[9][0]-rk[8][0]);
    dik[14][1] = (ri[9][1]-rk[8][1]);
    dik[14][2] = (ri[9][2]-rk[8][2]);
    dik[15][0] = (ri[10][0]-rk[0][0]);
    dik[15][1] = (ri[10][1]-rk[0][1]);
    dik[15][2] = (ri[10][2]-rk[0][2]);
    dik[16][0] = (ri[11][0]-rk[10][0]);
    dik[16][1] = (ri[11][1]-rk[10][1]);
    dik[16][2] = (ri[11][2]-rk[10][2]);
    dik[17][0] = (ri[12][0]-rk[11][0]);
    dik[17][1] = (ri[12][1]-rk[11][1]);
    dik[17][2] = (ri[12][2]-rk[11][2]);
    dik[18][0] = (ri[13][0]-rk[12][0]);
    dik[18][1] = (ri[13][1]-rk[12][1]);
    dik[18][2] = (ri[13][2]-rk[12][2]);
    dik[19][0] = (ri[14][0]-rk[13][0]);
    dik[19][1] = (ri[14][1]-rk[13][1]);
    dik[19][2] = (ri[14][2]-rk[13][2]);
    dik[20][0] = (ri[15][0]-rk[14][0]);
    dik[20][1] = (ri[15][1]-rk[14][1]);
    dik[20][2] = (ri[15][2]-rk[14][2]);

/* Compute mass properties-related constants */

    mtot = (mk[15]+(mk[14]+(mk[13]+(mk[12]+(mk[11]+(mk[10]+(mk[9]+(mk[8]+(mk[7]+
      (mk[6]+(mk[5]+(mk[4]+(mk[3]+(mk[2]+(mk[0]+mk[1])))))))))))))));
    mkrk[5][0][1] = -(mk[0]*rk[0][2]);
    mkrk[5][0][2] = (mk[0]*rk[0][1]);
    mkrk[5][1][0] = (mk[0]*rk[0][2]);
    mkrk[5][1][2] = -(mk[0]*rk[0][0]);
    mkrk[5][2][0] = -(mk[0]*rk[0][1]);
    mkrk[5][2][1] = (mk[0]*rk[0][0]);
    mkrk[6][0][1] = -(mk[1]*rk[1][2]);
    mkrk[6][0][2] = (mk[1]*rk[1][1]);
    mkrk[6][1][0] = (mk[1]*rk[1][2]);
    mkrk[6][1][2] = -(mk[1]*rk[1][0]);
    mkrk[6][2][0] = -(mk[1]*rk[1][1]);
    mkrk[6][2][1] = (mk[1]*rk[1][0]);
    mkrk[7][0][1] = -(mk[2]*rk[2][2]);
    mkrk[7][0][2] = (mk[2]*rk[2][1]);
    mkrk[7][1][0] = (mk[2]*rk[2][2]);
    mkrk[7][1][2] = -(mk[2]*rk[2][0]);
    mkrk[7][2][0] = -(mk[2]*rk[2][1]);
    mkrk[7][2][1] = (mk[2]*rk[2][0]);
    mkrk[8][0][1] = -(mk[3]*rk[3][2]);
    mkrk[8][0][2] = (mk[3]*rk[3][1]);
    mkrk[8][1][0] = (mk[3]*rk[3][2]);
    mkrk[8][1][2] = -(mk[3]*rk[3][0]);
    mkrk[8][2][0] = -(mk[3]*rk[3][1]);
    mkrk[8][2][1] = (mk[3]*rk[3][0]);
    mkrk[9][0][1] = -(mk[4]*rk[4][2]);
    mkrk[9][0][2] = (mk[4]*rk[4][1]);
    mkrk[9][1][0] = (mk[4]*rk[4][2]);
    mkrk[9][1][2] = -(mk[4]*rk[4][0]);
    mkrk[9][2][0] = -(mk[4]*rk[4][1]);
    mkrk[9][2][1] = (mk[4]*rk[4][0]);
    mkrk[10][0][1] = -(mk[5]*rk[5][2]);
    mkrk[10][0][2] = (mk[5]*rk[5][1]);
    mkrk[10][1][0] = (mk[5]*rk[5][2]);
    mkrk[10][1][2] = -(mk[5]*rk[5][0]);
    mkrk[10][2][0] = -(mk[5]*rk[5][1]);
    mkrk[10][2][1] = (mk[5]*rk[5][0]);
    mkrk[11][0][1] = -(mk[6]*rk[6][2]);
    mkrk[11][0][2] = (mk[6]*rk[6][1]);
    mkrk[11][1][0] = (mk[6]*rk[6][2]);
    mkrk[11][1][2] = -(mk[6]*rk[6][0]);
    mkrk[11][2][0] = -(mk[6]*rk[6][1]);
    mkrk[11][2][1] = (mk[6]*rk[6][0]);
    mkrk[12][0][1] = -(mk[7]*rk[7][2]);
    mkrk[12][0][2] = (mk[7]*rk[7][1]);
    mkrk[12][1][0] = (mk[7]*rk[7][2]);
    mkrk[12][1][2] = -(mk[7]*rk[7][0]);
    mkrk[12][2][0] = -(mk[7]*rk[7][1]);
    mkrk[12][2][1] = (mk[7]*rk[7][0]);
    mkrk[13][0][1] = -(mk[8]*rk[8][2]);
    mkrk[13][0][2] = (mk[8]*rk[8][1]);
    mkrk[13][1][0] = (mk[8]*rk[8][2]);
    mkrk[13][1][2] = -(mk[8]*rk[8][0]);
    mkrk[13][2][0] = -(mk[8]*rk[8][1]);
    mkrk[13][2][1] = (mk[8]*rk[8][0]);
    mkrk[14][0][1] = -(mk[9]*rk[9][2]);
    mkrk[14][0][2] = (mk[9]*rk[9][1]);
    mkrk[14][1][0] = (mk[9]*rk[9][2]);
    mkrk[14][1][2] = -(mk[9]*rk[9][0]);
    mkrk[14][2][0] = -(mk[9]*rk[9][1]);
    mkrk[14][2][1] = (mk[9]*rk[9][0]);
    mkrk[15][0][1] = -(mk[10]*rk[10][2]);
    mkrk[15][0][2] = (mk[10]*rk[10][1]);
    mkrk[15][1][0] = (mk[10]*rk[10][2]);
    mkrk[15][1][2] = -(mk[10]*rk[10][0]);
    mkrk[15][2][0] = -(mk[10]*rk[10][1]);
    mkrk[15][2][1] = (mk[10]*rk[10][0]);
    mkrk[16][0][1] = -(mk[11]*rk[11][2]);
    mkrk[16][0][2] = (mk[11]*rk[11][1]);
    mkrk[16][1][0] = (mk[11]*rk[11][2]);
    mkrk[16][1][2] = -(mk[11]*rk[11][0]);
    mkrk[16][2][0] = -(mk[11]*rk[11][1]);
    mkrk[16][2][1] = (mk[11]*rk[11][0]);
    mkrk[17][0][1] = -(mk[12]*rk[12][2]);
    mkrk[17][0][2] = (mk[12]*rk[12][1]);
    mkrk[17][1][0] = (mk[12]*rk[12][2]);
    mkrk[17][1][2] = -(mk[12]*rk[12][0]);
    mkrk[17][2][0] = -(mk[12]*rk[12][1]);
    mkrk[17][2][1] = (mk[12]*rk[12][0]);
    mkrk[18][0][1] = -(mk[13]*rk[13][2]);
    mkrk[18][0][2] = (mk[13]*rk[13][1]);
    mkrk[18][1][0] = (mk[13]*rk[13][2]);
    mkrk[18][1][2] = -(mk[13]*rk[13][0]);
    mkrk[18][2][0] = -(mk[13]*rk[13][1]);
    mkrk[18][2][1] = (mk[13]*rk[13][0]);
    mkrk[19][0][1] = -(mk[14]*rk[14][2]);
    mkrk[19][0][2] = (mk[14]*rk[14][1]);
    mkrk[19][1][0] = (mk[14]*rk[14][2]);
    mkrk[19][1][2] = -(mk[14]*rk[14][0]);
    mkrk[19][2][0] = -(mk[14]*rk[14][1]);
    mkrk[19][2][1] = (mk[14]*rk[14][0]);
    mkrk[20][0][1] = -(mk[15]*rk[15][2]);
    mkrk[20][0][2] = (mk[15]*rk[15][1]);
    mkrk[20][1][0] = (mk[15]*rk[15][2]);
    mkrk[20][1][2] = -(mk[15]*rk[15][0]);
    mkrk[20][2][0] = -(mk[15]*rk[15][1]);
    mkrk[20][2][1] = (mk[15]*rk[15][0]);
    Iko[5][0][0] = (ik[0][0][0]-((mkrk[5][0][1]*rk[0][2])-(mkrk[5][0][2]*
      rk[0][1])));
    Iko[5][0][1] = (ik[0][0][1]-(mkrk[5][0][2]*rk[0][0]));
    Iko[5][0][2] = (ik[0][0][2]+(mkrk[5][0][1]*rk[0][0]));
    Iko[5][1][0] = (ik[0][1][0]+(mkrk[5][1][2]*rk[0][1]));
    Iko[5][1][1] = (ik[0][1][1]-((mkrk[5][1][2]*rk[0][0])-(mkrk[5][1][0]*
      rk[0][2])));
    Iko[5][1][2] = (ik[0][1][2]-(mkrk[5][1][0]*rk[0][1]));
    Iko[5][2][0] = (ik[0][2][0]-(mkrk[5][2][1]*rk[0][2]));
    Iko[5][2][1] = (ik[0][2][1]+(mkrk[5][2][0]*rk[0][2]));
    Iko[5][2][2] = (ik[0][2][2]-((mkrk[5][2][0]*rk[0][1])-(mkrk[5][2][1]*
      rk[0][0])));
    Iko[6][0][0] = (ik[1][0][0]-((mkrk[6][0][1]*rk[1][2])-(mkrk[6][0][2]*
      rk[1][1])));
    Iko[6][0][1] = (ik[1][0][1]-(mkrk[6][0][2]*rk[1][0]));
    Iko[6][0][2] = (ik[1][0][2]+(mkrk[6][0][1]*rk[1][0]));
    Iko[6][1][0] = (ik[1][1][0]+(mkrk[6][1][2]*rk[1][1]));
    Iko[6][1][1] = (ik[1][1][1]-((mkrk[6][1][2]*rk[1][0])-(mkrk[6][1][0]*
      rk[1][2])));
    Iko[6][1][2] = (ik[1][1][2]-(mkrk[6][1][0]*rk[1][1]));
    Iko[6][2][0] = (ik[1][2][0]-(mkrk[6][2][1]*rk[1][2]));
    Iko[6][2][1] = (ik[1][2][1]+(mkrk[6][2][0]*rk[1][2]));
    Iko[6][2][2] = (ik[1][2][2]-((mkrk[6][2][0]*rk[1][1])-(mkrk[6][2][1]*
      rk[1][0])));
    Iko[7][0][0] = (ik[2][0][0]-((mkrk[7][0][1]*rk[2][2])-(mkrk[7][0][2]*
      rk[2][1])));
    Iko[7][0][1] = (ik[2][0][1]-(mkrk[7][0][2]*rk[2][0]));
    Iko[7][0][2] = (ik[2][0][2]+(mkrk[7][0][1]*rk[2][0]));
    Iko[7][1][0] = (ik[2][1][0]+(mkrk[7][1][2]*rk[2][1]));
    Iko[7][1][1] = (ik[2][1][1]-((mkrk[7][1][2]*rk[2][0])-(mkrk[7][1][0]*
      rk[2][2])));
    Iko[7][1][2] = (ik[2][1][2]-(mkrk[7][1][0]*rk[2][1]));
    Iko[7][2][0] = (ik[2][2][0]-(mkrk[7][2][1]*rk[2][2]));
    Iko[7][2][1] = (ik[2][2][1]+(mkrk[7][2][0]*rk[2][2]));
    Iko[7][2][2] = (ik[2][2][2]-((mkrk[7][2][0]*rk[2][1])-(mkrk[7][2][1]*
      rk[2][0])));
    Iko[8][0][0] = (ik[3][0][0]-((mkrk[8][0][1]*rk[3][2])-(mkrk[8][0][2]*
      rk[3][1])));
    Iko[8][0][1] = (ik[3][0][1]-(mkrk[8][0][2]*rk[3][0]));
    Iko[8][0][2] = (ik[3][0][2]+(mkrk[8][0][1]*rk[3][0]));
    Iko[8][1][0] = (ik[3][1][0]+(mkrk[8][1][2]*rk[3][1]));
    Iko[8][1][1] = (ik[3][1][1]-((mkrk[8][1][2]*rk[3][0])-(mkrk[8][1][0]*
      rk[3][2])));
    Iko[8][1][2] = (ik[3][1][2]-(mkrk[8][1][0]*rk[3][1]));
    Iko[8][2][0] = (ik[3][2][0]-(mkrk[8][2][1]*rk[3][2]));
    Iko[8][2][1] = (ik[3][2][1]+(mkrk[8][2][0]*rk[3][2]));
    Iko[8][2][2] = (ik[3][2][2]-((mkrk[8][2][0]*rk[3][1])-(mkrk[8][2][1]*
      rk[3][0])));
    Iko[9][0][0] = (ik[4][0][0]-((mkrk[9][0][1]*rk[4][2])-(mkrk[9][0][2]*
      rk[4][1])));
    Iko[9][0][1] = (ik[4][0][1]-(mkrk[9][0][2]*rk[4][0]));
    Iko[9][0][2] = (ik[4][0][2]+(mkrk[9][0][1]*rk[4][0]));
    Iko[9][1][0] = (ik[4][1][0]+(mkrk[9][1][2]*rk[4][1]));
    Iko[9][1][1] = (ik[4][1][1]-((mkrk[9][1][2]*rk[4][0])-(mkrk[9][1][0]*
      rk[4][2])));
    Iko[9][1][2] = (ik[4][1][2]-(mkrk[9][1][0]*rk[4][1]));
    Iko[9][2][0] = (ik[4][2][0]-(mkrk[9][2][1]*rk[4][2]));
    Iko[9][2][1] = (ik[4][2][1]+(mkrk[9][2][0]*rk[4][2]));
    Iko[9][2][2] = (ik[4][2][2]-((mkrk[9][2][0]*rk[4][1])-(mkrk[9][2][1]*
      rk[4][0])));
    Iko[10][0][0] = (ik[5][0][0]-((mkrk[10][0][1]*rk[5][2])-(mkrk[10][0][2]*
      rk[5][1])));
    Iko[10][0][1] = (ik[5][0][1]-(mkrk[10][0][2]*rk[5][0]));
    Iko[10][0][2] = (ik[5][0][2]+(mkrk[10][0][1]*rk[5][0]));
    Iko[10][1][0] = (ik[5][1][0]+(mkrk[10][1][2]*rk[5][1]));
    Iko[10][1][1] = (ik[5][1][1]-((mkrk[10][1][2]*rk[5][0])-(mkrk[10][1][0]*
      rk[5][2])));
    Iko[10][1][2] = (ik[5][1][2]-(mkrk[10][1][0]*rk[5][1]));
    Iko[10][2][0] = (ik[5][2][0]-(mkrk[10][2][1]*rk[5][2]));
    Iko[10][2][1] = (ik[5][2][1]+(mkrk[10][2][0]*rk[5][2]));
    Iko[10][2][2] = (ik[5][2][2]-((mkrk[10][2][0]*rk[5][1])-(mkrk[10][2][1]*
      rk[5][0])));
    Iko[11][0][0] = (ik[6][0][0]-((mkrk[11][0][1]*rk[6][2])-(mkrk[11][0][2]*
      rk[6][1])));
    Iko[11][0][1] = (ik[6][0][1]-(mkrk[11][0][2]*rk[6][0]));
    Iko[11][0][2] = (ik[6][0][2]+(mkrk[11][0][1]*rk[6][0]));
    Iko[11][1][0] = (ik[6][1][0]+(mkrk[11][1][2]*rk[6][1]));
    Iko[11][1][1] = (ik[6][1][1]-((mkrk[11][1][2]*rk[6][0])-(mkrk[11][1][0]*
      rk[6][2])));
    Iko[11][1][2] = (ik[6][1][2]-(mkrk[11][1][0]*rk[6][1]));
    Iko[11][2][0] = (ik[6][2][0]-(mkrk[11][2][1]*rk[6][2]));
    Iko[11][2][1] = (ik[6][2][1]+(mkrk[11][2][0]*rk[6][2]));
    Iko[11][2][2] = (ik[6][2][2]-((mkrk[11][2][0]*rk[6][1])-(mkrk[11][2][1]*
      rk[6][0])));
    Iko[12][0][0] = (ik[7][0][0]-((mkrk[12][0][1]*rk[7][2])-(mkrk[12][0][2]*
      rk[7][1])));
    Iko[12][0][1] = (ik[7][0][1]-(mkrk[12][0][2]*rk[7][0]));
    Iko[12][0][2] = (ik[7][0][2]+(mkrk[12][0][1]*rk[7][0]));
    Iko[12][1][0] = (ik[7][1][0]+(mkrk[12][1][2]*rk[7][1]));
    Iko[12][1][1] = (ik[7][1][1]-((mkrk[12][1][2]*rk[7][0])-(mkrk[12][1][0]*
      rk[7][2])));
    Iko[12][1][2] = (ik[7][1][2]-(mkrk[12][1][0]*rk[7][1]));
    Iko[12][2][0] = (ik[7][2][0]-(mkrk[12][2][1]*rk[7][2]));
    Iko[12][2][1] = (ik[7][2][1]+(mkrk[12][2][0]*rk[7][2]));
    Iko[12][2][2] = (ik[7][2][2]-((mkrk[12][2][0]*rk[7][1])-(mkrk[12][2][1]*
      rk[7][0])));
    Iko[13][0][0] = (ik[8][0][0]-((mkrk[13][0][1]*rk[8][2])-(mkrk[13][0][2]*
      rk[8][1])));
    Iko[13][0][1] = (ik[8][0][1]-(mkrk[13][0][2]*rk[8][0]));
    Iko[13][0][2] = (ik[8][0][2]+(mkrk[13][0][1]*rk[8][0]));
    Iko[13][1][0] = (ik[8][1][0]+(mkrk[13][1][2]*rk[8][1]));
    Iko[13][1][1] = (ik[8][1][1]-((mkrk[13][1][2]*rk[8][0])-(mkrk[13][1][0]*
      rk[8][2])));
    Iko[13][1][2] = (ik[8][1][2]-(mkrk[13][1][0]*rk[8][1]));
    Iko[13][2][0] = (ik[8][2][0]-(mkrk[13][2][1]*rk[8][2]));
    Iko[13][2][1] = (ik[8][2][1]+(mkrk[13][2][0]*rk[8][2]));
    Iko[13][2][2] = (ik[8][2][2]-((mkrk[13][2][0]*rk[8][1])-(mkrk[13][2][1]*
      rk[8][0])));
    Iko[14][0][0] = (ik[9][0][0]-((mkrk[14][0][1]*rk[9][2])-(mkrk[14][0][2]*
      rk[9][1])));
    Iko[14][0][1] = (ik[9][0][1]-(mkrk[14][0][2]*rk[9][0]));
    Iko[14][0][2] = (ik[9][0][2]+(mkrk[14][0][1]*rk[9][0]));
    Iko[14][1][0] = (ik[9][1][0]+(mkrk[14][1][2]*rk[9][1]));
    Iko[14][1][1] = (ik[9][1][1]-((mkrk[14][1][2]*rk[9][0])-(mkrk[14][1][0]*
      rk[9][2])));
    Iko[14][1][2] = (ik[9][1][2]-(mkrk[14][1][0]*rk[9][1]));
    Iko[14][2][0] = (ik[9][2][0]-(mkrk[14][2][1]*rk[9][2]));
    Iko[14][2][1] = (ik[9][2][1]+(mkrk[14][2][0]*rk[9][2]));
    Iko[14][2][2] = (ik[9][2][2]-((mkrk[14][2][0]*rk[9][1])-(mkrk[14][2][1]*
      rk[9][0])));
    Iko[15][0][0] = (ik[10][0][0]-((mkrk[15][0][1]*rk[10][2])-(mkrk[15][0][2]*
      rk[10][1])));
    Iko[15][0][1] = (ik[10][0][1]-(mkrk[15][0][2]*rk[10][0]));
    Iko[15][0][2] = (ik[10][0][2]+(mkrk[15][0][1]*rk[10][0]));
    Iko[15][1][0] = (ik[10][1][0]+(mkrk[15][1][2]*rk[10][1]));
    Iko[15][1][1] = (ik[10][1][1]-((mkrk[15][1][2]*rk[10][0])-(mkrk[15][1][0]*
      rk[10][2])));
    Iko[15][1][2] = (ik[10][1][2]-(mkrk[15][1][0]*rk[10][1]));
    Iko[15][2][0] = (ik[10][2][0]-(mkrk[15][2][1]*rk[10][2]));
    Iko[15][2][1] = (ik[10][2][1]+(mkrk[15][2][0]*rk[10][2]));
    Iko[15][2][2] = (ik[10][2][2]-((mkrk[15][2][0]*rk[10][1])-(mkrk[15][2][1]*
      rk[10][0])));
    Iko[16][0][0] = (ik[11][0][0]-((mkrk[16][0][1]*rk[11][2])-(mkrk[16][0][2]*
      rk[11][1])));
    Iko[16][0][1] = (ik[11][0][1]-(mkrk[16][0][2]*rk[11][0]));
    Iko[16][0][2] = (ik[11][0][2]+(mkrk[16][0][1]*rk[11][0]));
    Iko[16][1][0] = (ik[11][1][0]+(mkrk[16][1][2]*rk[11][1]));
    Iko[16][1][1] = (ik[11][1][1]-((mkrk[16][1][2]*rk[11][0])-(mkrk[16][1][0]*
      rk[11][2])));
    Iko[16][1][2] = (ik[11][1][2]-(mkrk[16][1][0]*rk[11][1]));
    Iko[16][2][0] = (ik[11][2][0]-(mkrk[16][2][1]*rk[11][2]));
    Iko[16][2][1] = (ik[11][2][1]+(mkrk[16][2][0]*rk[11][2]));
    Iko[16][2][2] = (ik[11][2][2]-((mkrk[16][2][0]*rk[11][1])-(mkrk[16][2][1]*
      rk[11][0])));
    Iko[17][0][0] = (ik[12][0][0]-((mkrk[17][0][1]*rk[12][2])-(mkrk[17][0][2]*
      rk[12][1])));
    Iko[17][0][1] = (ik[12][0][1]-(mkrk[17][0][2]*rk[12][0]));
    Iko[17][0][2] = (ik[12][0][2]+(mkrk[17][0][1]*rk[12][0]));
    Iko[17][1][0] = (ik[12][1][0]+(mkrk[17][1][2]*rk[12][1]));
    Iko[17][1][1] = (ik[12][1][1]-((mkrk[17][1][2]*rk[12][0])-(mkrk[17][1][0]*
      rk[12][2])));
    Iko[17][1][2] = (ik[12][1][2]-(mkrk[17][1][0]*rk[12][1]));
    Iko[17][2][0] = (ik[12][2][0]-(mkrk[17][2][1]*rk[12][2]));
    Iko[17][2][1] = (ik[12][2][1]+(mkrk[17][2][0]*rk[12][2]));
    Iko[17][2][2] = (ik[12][2][2]-((mkrk[17][2][0]*rk[12][1])-(mkrk[17][2][1]*
      rk[12][0])));
    Iko[18][0][0] = (ik[13][0][0]-((mkrk[18][0][1]*rk[13][2])-(mkrk[18][0][2]*
      rk[13][1])));
    Iko[18][0][1] = (ik[13][0][1]-(mkrk[18][0][2]*rk[13][0]));
    Iko[18][0][2] = (ik[13][0][2]+(mkrk[18][0][1]*rk[13][0]));
    Iko[18][1][0] = (ik[13][1][0]+(mkrk[18][1][2]*rk[13][1]));
    Iko[18][1][1] = (ik[13][1][1]-((mkrk[18][1][2]*rk[13][0])-(mkrk[18][1][0]*
      rk[13][2])));
    Iko[18][1][2] = (ik[13][1][2]-(mkrk[18][1][0]*rk[13][1]));
    Iko[18][2][0] = (ik[13][2][0]-(mkrk[18][2][1]*rk[13][2]));
    Iko[18][2][1] = (ik[13][2][1]+(mkrk[18][2][0]*rk[13][2]));
    Iko[18][2][2] = (ik[13][2][2]-((mkrk[18][2][0]*rk[13][1])-(mkrk[18][2][1]*
      rk[13][0])));
    Iko[19][0][0] = (ik[14][0][0]-((mkrk[19][0][1]*rk[14][2])-(mkrk[19][0][2]*
      rk[14][1])));
    Iko[19][0][1] = (ik[14][0][1]-(mkrk[19][0][2]*rk[14][0]));
    Iko[19][0][2] = (ik[14][0][2]+(mkrk[19][0][1]*rk[14][0]));
    Iko[19][1][0] = (ik[14][1][0]+(mkrk[19][1][2]*rk[14][1]));
    Iko[19][1][1] = (ik[14][1][1]-((mkrk[19][1][2]*rk[14][0])-(mkrk[19][1][0]*
      rk[14][2])));
    Iko[19][1][2] = (ik[14][1][2]-(mkrk[19][1][0]*rk[14][1]));
    Iko[19][2][0] = (ik[14][2][0]-(mkrk[19][2][1]*rk[14][2]));
    Iko[19][2][1] = (ik[14][2][1]+(mkrk[19][2][0]*rk[14][2]));
    Iko[19][2][2] = (ik[14][2][2]-((mkrk[19][2][0]*rk[14][1])-(mkrk[19][2][1]*
      rk[14][0])));
    Iko[20][0][0] = (ik[15][0][0]-((mkrk[20][0][1]*rk[15][2])-(mkrk[20][0][2]*
      rk[15][1])));
    Iko[20][0][1] = (ik[15][0][1]-(mkrk[20][0][2]*rk[15][0]));
    Iko[20][0][2] = (ik[15][0][2]+(mkrk[20][0][1]*rk[15][0]));
    Iko[20][1][0] = (ik[15][1][0]+(mkrk[20][1][2]*rk[15][1]));
    Iko[20][1][1] = (ik[15][1][1]-((mkrk[20][1][2]*rk[15][0])-(mkrk[20][1][0]*
      rk[15][2])));
    Iko[20][1][2] = (ik[15][1][2]-(mkrk[20][1][0]*rk[15][1]));
    Iko[20][2][0] = (ik[15][2][0]-(mkrk[20][2][1]*rk[15][2]));
    Iko[20][2][1] = (ik[15][2][1]+(mkrk[20][2][0]*rk[15][2]));
    Iko[20][2][2] = (ik[15][2][2]-((mkrk[20][2][0]*rk[15][1])-(mkrk[20][2][1]*
      rk[15][0])));
    sdserialno(&i);
    if (i != 30123) {
        sdseterr(7,41);
    }
    roustate = 1;
}

/* Convert state to form using 1-2-3 Euler angles for ball joints. */

void sdst2ang(double st[22],
    double stang[21])
{
    int i;
    double dc[3][3];

    for (i = 0; i < 21; i++) {
        stang[i] = st[i];
    }
    sdquat2dc(st[3],st[4],st[5],st[21],dc);
    sddc2ang(dc,&stang[3],&stang[4],&stang[5]);
}

/* Convert 1-2-3 form of state back to Euler parameters for ball joints. */

void sdang2st(double stang[21],
    double st[22])
{
    int i;
    double dc[3][3];

    for (i = 0; i < 21; i++) {
        st[i] = stang[i];
    }
    sdang2dc(stang[3],stang[4],stang[5],dc);
    sddc2quat(dc,&st[3],&st[4],&st[5],&st[21]);
}

/* Normalize Euler parameters in state. */

void sdnrmsterr(double st[22],
    double normst[22],
    int routine)
{
    int i;
    double norm;

    for (i = 0; i < 22; i++) {
        normst[i] = st[i];
    }
    norm = sqrt(st[3]*st[3]+st[4]*st[4]+st[5]*st[5]+st[21]*st[21]);
    if (routine != 0) {
        if ((norm < .9) || (norm > 1.1)) {
            sdseterr(routine,14);
        }
    }
    if (norm == 0.) {
        normst[21] = 1.;
        norm = 1.;
    }
    norm = 1./norm;
    normst[3] = normst[3]*norm;
    normst[4] = normst[4]*norm;
    normst[5] = normst[5]*norm;
    normst[21] = normst[21]*norm;
}

void sdnormst(double st[22],
    double normst[22])
{

    sdnrmsterr(st,normst,0);
}

void sdstate(double timein,
    double qin[22],
    double uin[21])
{
/*
Compute kinematic information and store it in sdgstate.

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
    int i,j,qchg,uchg;
    double ee,stab;

    if ((roustate != 1) && (roustate != 2) && (roustate != 3)) {
        sdseterr(8,22);
        return;
    }
    if (roustate == 1) {
        for (i = 0; i < 21; i++) {
            if (presq[i] == 1) {
                sdseterr(8,31);
            }
        }
    }
/*
See if time or any qs have changed since last call
*/
    if ((roustate != 1) && (timein == curtim)) {
        qchg = 0;
        for (i = 0; i < 22; i++) {
            if (qin[i] != q[i]) {
                qchg = 1;
                break;
            }
        }
    } else {
        qchg = 1;
    }
/*
If time and qs are unchanged, check us
*/
    if (qchg == 0) {
        uchg = 0;
        for (i = 0; i < 21; i++) {
            if (uin[i] != u[i]) {
                uchg = 1;
                break;
            }
        }
    } else {
        uchg = 1;
    }
    curtim = timein;
    roustate = 2;
    if (qchg == 0) {
        goto skipqs;
    }
/*
Position-related variables need to be computed
*/
    vpkflg = 0;
    mmflg = 0;
    mmlduflg = 0;
    wwflg = 0;
    for (i = 0; i < 22; i++) {
        q[i] = qin[i];
    }
/*
Normalize Euler parameters in state
*/
    sdnrmsterr(q,qn,8);
/*
Compute sines and cosines of q
*/
    s6 = sin(q[6]);
    c6 = cos(q[6]);
    s7 = sin(q[7]);
    c7 = cos(q[7]);
    s8 = sin(q[8]);
    c8 = cos(q[8]);
    s9 = sin(q[9]);
    c9 = cos(q[9]);
    s10 = sin(q[10]);
    c10 = cos(q[10]);
    s11 = sin(q[11]);
    c11 = cos(q[11]);
    s12 = sin(q[12]);
    c12 = cos(q[12]);
    s13 = sin(q[13]);
    c13 = cos(q[13]);
    s14 = sin(q[14]);
    c14 = cos(q[14]);
    s15 = sin(q[15]);
    c15 = cos(q[15]);
    s16 = sin(q[16]);
    c16 = cos(q[16]);
    s17 = sin(q[17]);
    c17 = cos(q[17]);
    s18 = sin(q[18]);
    c18 = cos(q[18]);
    s19 = sin(q[19]);
    c19 = cos(q[19]);
    s20 = sin(q[20]);
    c20 = cos(q[20]);
/*
Compute across-axis direction cosines Cik
*/
    Cik[3][0][0] = (1.-(2.*((qn[4]*qn[4])+(qn[5]*qn[5]))));
    Cik[3][0][1] = (2.*((qn[3]*qn[4])-(qn[5]*qn[21])));
    Cik[3][0][2] = (2.*((qn[3]*qn[5])+(qn[4]*qn[21])));
    Cik[3][1][0] = (2.*((qn[3]*qn[4])+(qn[5]*qn[21])));
    Cik[3][1][1] = (1.-(2.*((qn[3]*qn[3])+(qn[5]*qn[5]))));
    Cik[3][1][2] = (2.*((qn[4]*qn[5])-(qn[3]*qn[21])));
    Cik[3][2][0] = (2.*((qn[3]*qn[5])-(qn[4]*qn[21])));
    Cik[3][2][1] = (2.*((qn[3]*qn[21])+(qn[4]*qn[5])));
    Cik[3][2][2] = (1.-(2.*((qn[3]*qn[3])+(qn[4]*qn[4]))));
    Cik[6][0][0] = ((pin[6][0]*pin[6][0])+(c6*(1.-(pin[6][0]*pin[6][0]))));
    Cik[6][0][1] = (((pin[6][0]*pin[6][1])-(pin[6][2]*s6))-(c6*(pin[6][0]*
      pin[6][1])));
    Cik[6][0][2] = (((pin[6][0]*pin[6][2])+(pin[6][1]*s6))-(c6*(pin[6][0]*
      pin[6][2])));
    Cik[6][1][0] = (((pin[6][0]*pin[6][1])+(pin[6][2]*s6))-(c6*(pin[6][0]*
      pin[6][1])));
    Cik[6][1][1] = ((pin[6][1]*pin[6][1])+(c6*(1.-(pin[6][1]*pin[6][1]))));
    Cik[6][1][2] = (((pin[6][1]*pin[6][2])-(pin[6][0]*s6))-(c6*(pin[6][1]*
      pin[6][2])));
    Cik[6][2][0] = (((pin[6][0]*pin[6][2])-(pin[6][1]*s6))-(c6*(pin[6][0]*
      pin[6][2])));
    Cik[6][2][1] = (((pin[6][0]*s6)+(pin[6][1]*pin[6][2]))-(c6*(pin[6][1]*
      pin[6][2])));
    Cik[6][2][2] = ((pin[6][2]*pin[6][2])+(c6*(1.-(pin[6][2]*pin[6][2]))));
    Cik[7][0][0] = ((pin[7][0]*pin[7][0])+(c7*(1.-(pin[7][0]*pin[7][0]))));
    Cik[7][0][1] = (((pin[7][0]*pin[7][1])-(pin[7][2]*s7))-(c7*(pin[7][0]*
      pin[7][1])));
    Cik[7][0][2] = (((pin[7][0]*pin[7][2])+(pin[7][1]*s7))-(c7*(pin[7][0]*
      pin[7][2])));
    Cik[7][1][0] = (((pin[7][0]*pin[7][1])+(pin[7][2]*s7))-(c7*(pin[7][0]*
      pin[7][1])));
    Cik[7][1][1] = ((pin[7][1]*pin[7][1])+(c7*(1.-(pin[7][1]*pin[7][1]))));
    Cik[7][1][2] = (((pin[7][1]*pin[7][2])-(pin[7][0]*s7))-(c7*(pin[7][1]*
      pin[7][2])));
    Cik[7][2][0] = (((pin[7][0]*pin[7][2])-(pin[7][1]*s7))-(c7*(pin[7][0]*
      pin[7][2])));
    Cik[7][2][1] = (((pin[7][0]*s7)+(pin[7][1]*pin[7][2]))-(c7*(pin[7][1]*
      pin[7][2])));
    Cik[7][2][2] = ((pin[7][2]*pin[7][2])+(c7*(1.-(pin[7][2]*pin[7][2]))));
    Cik[8][0][0] = ((pin[8][0]*pin[8][0])+(c8*(1.-(pin[8][0]*pin[8][0]))));
    Cik[8][0][1] = (((pin[8][0]*pin[8][1])-(pin[8][2]*s8))-(c8*(pin[8][0]*
      pin[8][1])));
    Cik[8][0][2] = (((pin[8][0]*pin[8][2])+(pin[8][1]*s8))-(c8*(pin[8][0]*
      pin[8][2])));
    Cik[8][1][0] = (((pin[8][0]*pin[8][1])+(pin[8][2]*s8))-(c8*(pin[8][0]*
      pin[8][1])));
    Cik[8][1][1] = ((pin[8][1]*pin[8][1])+(c8*(1.-(pin[8][1]*pin[8][1]))));
    Cik[8][1][2] = (((pin[8][1]*pin[8][2])-(pin[8][0]*s8))-(c8*(pin[8][1]*
      pin[8][2])));
    Cik[8][2][0] = (((pin[8][0]*pin[8][2])-(pin[8][1]*s8))-(c8*(pin[8][0]*
      pin[8][2])));
    Cik[8][2][1] = (((pin[8][0]*s8)+(pin[8][1]*pin[8][2]))-(c8*(pin[8][1]*
      pin[8][2])));
    Cik[8][2][2] = ((pin[8][2]*pin[8][2])+(c8*(1.-(pin[8][2]*pin[8][2]))));
    Cik[9][0][0] = ((pin[9][0]*pin[9][0])+(c9*(1.-(pin[9][0]*pin[9][0]))));
    Cik[9][0][1] = (((pin[9][0]*pin[9][1])-(pin[9][2]*s9))-(c9*(pin[9][0]*
      pin[9][1])));
    Cik[9][0][2] = (((pin[9][0]*pin[9][2])+(pin[9][1]*s9))-(c9*(pin[9][0]*
      pin[9][2])));
    Cik[9][1][0] = (((pin[9][0]*pin[9][1])+(pin[9][2]*s9))-(c9*(pin[9][0]*
      pin[9][1])));
    Cik[9][1][1] = ((pin[9][1]*pin[9][1])+(c9*(1.-(pin[9][1]*pin[9][1]))));
    Cik[9][1][2] = (((pin[9][1]*pin[9][2])-(pin[9][0]*s9))-(c9*(pin[9][1]*
      pin[9][2])));
    Cik[9][2][0] = (((pin[9][0]*pin[9][2])-(pin[9][1]*s9))-(c9*(pin[9][0]*
      pin[9][2])));
    Cik[9][2][1] = (((pin[9][0]*s9)+(pin[9][1]*pin[9][2]))-(c9*(pin[9][1]*
      pin[9][2])));
    Cik[9][2][2] = ((pin[9][2]*pin[9][2])+(c9*(1.-(pin[9][2]*pin[9][2]))));
    Cik[10][0][0] = ((pin[10][0]*pin[10][0])+(c10*(1.-(pin[10][0]*pin[10][0]))))
      ;
    Cik[10][0][1] = (((pin[10][0]*pin[10][1])-(pin[10][2]*s10))-(c10*(pin[10][0]
      *pin[10][1])));
    Cik[10][0][2] = (((pin[10][0]*pin[10][2])+(pin[10][1]*s10))-(c10*(pin[10][0]
      *pin[10][2])));
    Cik[10][1][0] = (((pin[10][0]*pin[10][1])+(pin[10][2]*s10))-(c10*(pin[10][0]
      *pin[10][1])));
    Cik[10][1][1] = ((pin[10][1]*pin[10][1])+(c10*(1.-(pin[10][1]*pin[10][1]))))
      ;
    Cik[10][1][2] = (((pin[10][1]*pin[10][2])-(pin[10][0]*s10))-(c10*(pin[10][1]
      *pin[10][2])));
    Cik[10][2][0] = (((pin[10][0]*pin[10][2])-(pin[10][1]*s10))-(c10*(pin[10][0]
      *pin[10][2])));
    Cik[10][2][1] = (((pin[10][0]*s10)+(pin[10][1]*pin[10][2]))-(c10*(pin[10][1]
      *pin[10][2])));
    Cik[10][2][2] = ((pin[10][2]*pin[10][2])+(c10*(1.-(pin[10][2]*pin[10][2]))))
      ;
    Cik[11][0][0] = ((pin[11][0]*pin[11][0])+(c11*(1.-(pin[11][0]*pin[11][0]))))
      ;
    Cik[11][0][1] = (((pin[11][0]*pin[11][1])-(pin[11][2]*s11))-(c11*(pin[11][0]
      *pin[11][1])));
    Cik[11][0][2] = (((pin[11][0]*pin[11][2])+(pin[11][1]*s11))-(c11*(pin[11][0]
      *pin[11][2])));
    Cik[11][1][0] = (((pin[11][0]*pin[11][1])+(pin[11][2]*s11))-(c11*(pin[11][0]
      *pin[11][1])));
    Cik[11][1][1] = ((pin[11][1]*pin[11][1])+(c11*(1.-(pin[11][1]*pin[11][1]))))
      ;
    Cik[11][1][2] = (((pin[11][1]*pin[11][2])-(pin[11][0]*s11))-(c11*(pin[11][1]
      *pin[11][2])));
    Cik[11][2][0] = (((pin[11][0]*pin[11][2])-(pin[11][1]*s11))-(c11*(pin[11][0]
      *pin[11][2])));
    Cik[11][2][1] = (((pin[11][0]*s11)+(pin[11][1]*pin[11][2]))-(c11*(pin[11][1]
      *pin[11][2])));
    Cik[11][2][2] = ((pin[11][2]*pin[11][2])+(c11*(1.-(pin[11][2]*pin[11][2]))))
      ;
    Cik[12][0][0] = ((pin[12][0]*pin[12][0])+(c12*(1.-(pin[12][0]*pin[12][0]))))
      ;
    Cik[12][0][1] = (((pin[12][0]*pin[12][1])-(pin[12][2]*s12))-(c12*(pin[12][0]
      *pin[12][1])));
    Cik[12][0][2] = (((pin[12][0]*pin[12][2])+(pin[12][1]*s12))-(c12*(pin[12][0]
      *pin[12][2])));
    Cik[12][1][0] = (((pin[12][0]*pin[12][1])+(pin[12][2]*s12))-(c12*(pin[12][0]
      *pin[12][1])));
    Cik[12][1][1] = ((pin[12][1]*pin[12][1])+(c12*(1.-(pin[12][1]*pin[12][1]))))
      ;
    Cik[12][1][2] = (((pin[12][1]*pin[12][2])-(pin[12][0]*s12))-(c12*(pin[12][1]
      *pin[12][2])));
    Cik[12][2][0] = (((pin[12][0]*pin[12][2])-(pin[12][1]*s12))-(c12*(pin[12][0]
      *pin[12][2])));
    Cik[12][2][1] = (((pin[12][0]*s12)+(pin[12][1]*pin[12][2]))-(c12*(pin[12][1]
      *pin[12][2])));
    Cik[12][2][2] = ((pin[12][2]*pin[12][2])+(c12*(1.-(pin[12][2]*pin[12][2]))))
      ;
    Cik[13][0][0] = ((pin[13][0]*pin[13][0])+(c13*(1.-(pin[13][0]*pin[13][0]))))
      ;
    Cik[13][0][1] = (((pin[13][0]*pin[13][1])-(pin[13][2]*s13))-(c13*(pin[13][0]
      *pin[13][1])));
    Cik[13][0][2] = (((pin[13][0]*pin[13][2])+(pin[13][1]*s13))-(c13*(pin[13][0]
      *pin[13][2])));
    Cik[13][1][0] = (((pin[13][0]*pin[13][1])+(pin[13][2]*s13))-(c13*(pin[13][0]
      *pin[13][1])));
    Cik[13][1][1] = ((pin[13][1]*pin[13][1])+(c13*(1.-(pin[13][1]*pin[13][1]))))
      ;
    Cik[13][1][2] = (((pin[13][1]*pin[13][2])-(pin[13][0]*s13))-(c13*(pin[13][1]
      *pin[13][2])));
    Cik[13][2][0] = (((pin[13][0]*pin[13][2])-(pin[13][1]*s13))-(c13*(pin[13][0]
      *pin[13][2])));
    Cik[13][2][1] = (((pin[13][0]*s13)+(pin[13][1]*pin[13][2]))-(c13*(pin[13][1]
      *pin[13][2])));
    Cik[13][2][2] = ((pin[13][2]*pin[13][2])+(c13*(1.-(pin[13][2]*pin[13][2]))))
      ;
    Cik[14][0][0] = ((pin[14][0]*pin[14][0])+(c14*(1.-(pin[14][0]*pin[14][0]))))
      ;
    Cik[14][0][1] = (((pin[14][0]*pin[14][1])-(pin[14][2]*s14))-(c14*(pin[14][0]
      *pin[14][1])));
    Cik[14][0][2] = (((pin[14][0]*pin[14][2])+(pin[14][1]*s14))-(c14*(pin[14][0]
      *pin[14][2])));
    Cik[14][1][0] = (((pin[14][0]*pin[14][1])+(pin[14][2]*s14))-(c14*(pin[14][0]
      *pin[14][1])));
    Cik[14][1][1] = ((pin[14][1]*pin[14][1])+(c14*(1.-(pin[14][1]*pin[14][1]))))
      ;
    Cik[14][1][2] = (((pin[14][1]*pin[14][2])-(pin[14][0]*s14))-(c14*(pin[14][1]
      *pin[14][2])));
    Cik[14][2][0] = (((pin[14][0]*pin[14][2])-(pin[14][1]*s14))-(c14*(pin[14][0]
      *pin[14][2])));
    Cik[14][2][1] = (((pin[14][0]*s14)+(pin[14][1]*pin[14][2]))-(c14*(pin[14][1]
      *pin[14][2])));
    Cik[14][2][2] = ((pin[14][2]*pin[14][2])+(c14*(1.-(pin[14][2]*pin[14][2]))))
      ;
    Cik[15][0][0] = ((pin[15][0]*pin[15][0])+(c15*(1.-(pin[15][0]*pin[15][0]))))
      ;
    Cik[15][0][1] = (((pin[15][0]*pin[15][1])-(pin[15][2]*s15))-(c15*(pin[15][0]
      *pin[15][1])));
    Cik[15][0][2] = (((pin[15][0]*pin[15][2])+(pin[15][1]*s15))-(c15*(pin[15][0]
      *pin[15][2])));
    Cik[15][1][0] = (((pin[15][0]*pin[15][1])+(pin[15][2]*s15))-(c15*(pin[15][0]
      *pin[15][1])));
    Cik[15][1][1] = ((pin[15][1]*pin[15][1])+(c15*(1.-(pin[15][1]*pin[15][1]))))
      ;
    Cik[15][1][2] = (((pin[15][1]*pin[15][2])-(pin[15][0]*s15))-(c15*(pin[15][1]
      *pin[15][2])));
    Cik[15][2][0] = (((pin[15][0]*pin[15][2])-(pin[15][1]*s15))-(c15*(pin[15][0]
      *pin[15][2])));
    Cik[15][2][1] = (((pin[15][0]*s15)+(pin[15][1]*pin[15][2]))-(c15*(pin[15][1]
      *pin[15][2])));
    Cik[15][2][2] = ((pin[15][2]*pin[15][2])+(c15*(1.-(pin[15][2]*pin[15][2]))))
      ;
    Cik[16][0][0] = ((pin[16][0]*pin[16][0])+(c16*(1.-(pin[16][0]*pin[16][0]))))
      ;
    Cik[16][0][1] = (((pin[16][0]*pin[16][1])-(pin[16][2]*s16))-(c16*(pin[16][0]
      *pin[16][1])));
    Cik[16][0][2] = (((pin[16][0]*pin[16][2])+(pin[16][1]*s16))-(c16*(pin[16][0]
      *pin[16][2])));
    Cik[16][1][0] = (((pin[16][0]*pin[16][1])+(pin[16][2]*s16))-(c16*(pin[16][0]
      *pin[16][1])));
    Cik[16][1][1] = ((pin[16][1]*pin[16][1])+(c16*(1.-(pin[16][1]*pin[16][1]))))
      ;
    Cik[16][1][2] = (((pin[16][1]*pin[16][2])-(pin[16][0]*s16))-(c16*(pin[16][1]
      *pin[16][2])));
    Cik[16][2][0] = (((pin[16][0]*pin[16][2])-(pin[16][1]*s16))-(c16*(pin[16][0]
      *pin[16][2])));
    Cik[16][2][1] = (((pin[16][0]*s16)+(pin[16][1]*pin[16][2]))-(c16*(pin[16][1]
      *pin[16][2])));
    Cik[16][2][2] = ((pin[16][2]*pin[16][2])+(c16*(1.-(pin[16][2]*pin[16][2]))))
      ;
    Cik[17][0][0] = ((pin[17][0]*pin[17][0])+(c17*(1.-(pin[17][0]*pin[17][0]))))
      ;
    Cik[17][0][1] = (((pin[17][0]*pin[17][1])-(pin[17][2]*s17))-(c17*(pin[17][0]
      *pin[17][1])));
    Cik[17][0][2] = (((pin[17][0]*pin[17][2])+(pin[17][1]*s17))-(c17*(pin[17][0]
      *pin[17][2])));
    Cik[17][1][0] = (((pin[17][0]*pin[17][1])+(pin[17][2]*s17))-(c17*(pin[17][0]
      *pin[17][1])));
    Cik[17][1][1] = ((pin[17][1]*pin[17][1])+(c17*(1.-(pin[17][1]*pin[17][1]))))
      ;
    Cik[17][1][2] = (((pin[17][1]*pin[17][2])-(pin[17][0]*s17))-(c17*(pin[17][1]
      *pin[17][2])));
    Cik[17][2][0] = (((pin[17][0]*pin[17][2])-(pin[17][1]*s17))-(c17*(pin[17][0]
      *pin[17][2])));
    Cik[17][2][1] = (((pin[17][0]*s17)+(pin[17][1]*pin[17][2]))-(c17*(pin[17][1]
      *pin[17][2])));
    Cik[17][2][2] = ((pin[17][2]*pin[17][2])+(c17*(1.-(pin[17][2]*pin[17][2]))))
      ;
    Cik[18][0][0] = ((pin[18][0]*pin[18][0])+(c18*(1.-(pin[18][0]*pin[18][0]))))
      ;
    Cik[18][0][1] = (((pin[18][0]*pin[18][1])-(pin[18][2]*s18))-(c18*(pin[18][0]
      *pin[18][1])));
    Cik[18][0][2] = (((pin[18][0]*pin[18][2])+(pin[18][1]*s18))-(c18*(pin[18][0]
      *pin[18][2])));
    Cik[18][1][0] = (((pin[18][0]*pin[18][1])+(pin[18][2]*s18))-(c18*(pin[18][0]
      *pin[18][1])));
    Cik[18][1][1] = ((pin[18][1]*pin[18][1])+(c18*(1.-(pin[18][1]*pin[18][1]))))
      ;
    Cik[18][1][2] = (((pin[18][1]*pin[18][2])-(pin[18][0]*s18))-(c18*(pin[18][1]
      *pin[18][2])));
    Cik[18][2][0] = (((pin[18][0]*pin[18][2])-(pin[18][1]*s18))-(c18*(pin[18][0]
      *pin[18][2])));
    Cik[18][2][1] = (((pin[18][0]*s18)+(pin[18][1]*pin[18][2]))-(c18*(pin[18][1]
      *pin[18][2])));
    Cik[18][2][2] = ((pin[18][2]*pin[18][2])+(c18*(1.-(pin[18][2]*pin[18][2]))))
      ;
    Cik[19][0][0] = ((pin[19][0]*pin[19][0])+(c19*(1.-(pin[19][0]*pin[19][0]))))
      ;
    Cik[19][0][1] = (((pin[19][0]*pin[19][1])-(pin[19][2]*s19))-(c19*(pin[19][0]
      *pin[19][1])));
    Cik[19][0][2] = (((pin[19][0]*pin[19][2])+(pin[19][1]*s19))-(c19*(pin[19][0]
      *pin[19][2])));
    Cik[19][1][0] = (((pin[19][0]*pin[19][1])+(pin[19][2]*s19))-(c19*(pin[19][0]
      *pin[19][1])));
    Cik[19][1][1] = ((pin[19][1]*pin[19][1])+(c19*(1.-(pin[19][1]*pin[19][1]))))
      ;
    Cik[19][1][2] = (((pin[19][1]*pin[19][2])-(pin[19][0]*s19))-(c19*(pin[19][1]
      *pin[19][2])));
    Cik[19][2][0] = (((pin[19][0]*pin[19][2])-(pin[19][1]*s19))-(c19*(pin[19][0]
      *pin[19][2])));
    Cik[19][2][1] = (((pin[19][0]*s19)+(pin[19][1]*pin[19][2]))-(c19*(pin[19][1]
      *pin[19][2])));
    Cik[19][2][2] = ((pin[19][2]*pin[19][2])+(c19*(1.-(pin[19][2]*pin[19][2]))))
      ;
    Cik[20][0][0] = ((pin[20][0]*pin[20][0])+(c20*(1.-(pin[20][0]*pin[20][0]))))
      ;
    Cik[20][0][1] = (((pin[20][0]*pin[20][1])-(pin[20][2]*s20))-(c20*(pin[20][0]
      *pin[20][1])));
    Cik[20][0][2] = (((pin[20][0]*pin[20][2])+(pin[20][1]*s20))-(c20*(pin[20][0]
      *pin[20][2])));
    Cik[20][1][0] = (((pin[20][0]*pin[20][1])+(pin[20][2]*s20))-(c20*(pin[20][0]
      *pin[20][1])));
    Cik[20][1][1] = ((pin[20][1]*pin[20][1])+(c20*(1.-(pin[20][1]*pin[20][1]))))
      ;
    Cik[20][1][2] = (((pin[20][1]*pin[20][2])-(pin[20][0]*s20))-(c20*(pin[20][1]
      *pin[20][2])));
    Cik[20][2][0] = (((pin[20][0]*pin[20][2])-(pin[20][1]*s20))-(c20*(pin[20][0]
      *pin[20][2])));
    Cik[20][2][1] = (((pin[20][0]*s20)+(pin[20][1]*pin[20][2]))-(c20*(pin[20][1]
      *pin[20][2])));
    Cik[20][2][2] = ((pin[20][2]*pin[20][2])+(c20*(1.-(pin[20][2]*pin[20][2]))))
      ;
/*
Compute across-joint direction cosines Cib
*/
/*
Compute gravity
*/
    gk[3][0] = ((Cik[3][2][0]*grav[2])+((Cik[3][0][0]*grav[0])+(Cik[3][1][0]*
      grav[1])));
    gk[3][1] = ((Cik[3][2][1]*grav[2])+((Cik[3][0][1]*grav[0])+(Cik[3][1][1]*
      grav[1])));
    gk[3][2] = ((Cik[3][2][2]*grav[2])+((Cik[3][0][2]*grav[0])+(Cik[3][1][2]*
      grav[1])));
    gk[6][0] = ((Cik[6][2][0]*gk[3][2])+((Cik[6][0][0]*gk[3][0])+(Cik[6][1][0]*
      gk[3][1])));
    gk[6][1] = ((Cik[6][2][1]*gk[3][2])+((Cik[6][0][1]*gk[3][0])+(Cik[6][1][1]*
      gk[3][1])));
    gk[6][2] = ((Cik[6][2][2]*gk[3][2])+((Cik[6][0][2]*gk[3][0])+(Cik[6][1][2]*
      gk[3][1])));
    gk[7][0] = ((Cik[7][2][0]*gk[6][2])+((Cik[7][0][0]*gk[6][0])+(Cik[7][1][0]*
      gk[6][1])));
    gk[7][1] = ((Cik[7][2][1]*gk[6][2])+((Cik[7][0][1]*gk[6][0])+(Cik[7][1][1]*
      gk[6][1])));
    gk[7][2] = ((Cik[7][2][2]*gk[6][2])+((Cik[7][0][2]*gk[6][0])+(Cik[7][1][2]*
      gk[6][1])));
    gk[8][0] = ((Cik[8][2][0]*gk[7][2])+((Cik[8][0][0]*gk[7][0])+(Cik[8][1][0]*
      gk[7][1])));
    gk[8][1] = ((Cik[8][2][1]*gk[7][2])+((Cik[8][0][1]*gk[7][0])+(Cik[8][1][1]*
      gk[7][1])));
    gk[8][2] = ((Cik[8][2][2]*gk[7][2])+((Cik[8][0][2]*gk[7][0])+(Cik[8][1][2]*
      gk[7][1])));
    gk[9][0] = ((Cik[9][2][0]*gk[3][2])+((Cik[9][0][0]*gk[3][0])+(Cik[9][1][0]*
      gk[3][1])));
    gk[9][1] = ((Cik[9][2][1]*gk[3][2])+((Cik[9][0][1]*gk[3][0])+(Cik[9][1][1]*
      gk[3][1])));
    gk[9][2] = ((Cik[9][2][2]*gk[3][2])+((Cik[9][0][2]*gk[3][0])+(Cik[9][1][2]*
      gk[3][1])));
    gk[10][0] = ((Cik[10][2][0]*gk[9][2])+((Cik[10][0][0]*gk[9][0])+(
      Cik[10][1][0]*gk[9][1])));
    gk[10][1] = ((Cik[10][2][1]*gk[9][2])+((Cik[10][0][1]*gk[9][0])+(
      Cik[10][1][1]*gk[9][1])));
    gk[10][2] = ((Cik[10][2][2]*gk[9][2])+((Cik[10][0][2]*gk[9][0])+(
      Cik[10][1][2]*gk[9][1])));
    gk[11][0] = ((Cik[11][2][0]*gk[10][2])+((Cik[11][0][0]*gk[10][0])+(
      Cik[11][1][0]*gk[10][1])));
    gk[11][1] = ((Cik[11][2][1]*gk[10][2])+((Cik[11][0][1]*gk[10][0])+(
      Cik[11][1][1]*gk[10][1])));
    gk[11][2] = ((Cik[11][2][2]*gk[10][2])+((Cik[11][0][2]*gk[10][0])+(
      Cik[11][1][2]*gk[10][1])));
    gk[12][0] = ((Cik[12][2][0]*gk[11][2])+((Cik[12][0][0]*gk[11][0])+(
      Cik[12][1][0]*gk[11][1])));
    gk[12][1] = ((Cik[12][2][1]*gk[11][2])+((Cik[12][0][1]*gk[11][0])+(
      Cik[12][1][1]*gk[11][1])));
    gk[12][2] = ((Cik[12][2][2]*gk[11][2])+((Cik[12][0][2]*gk[11][0])+(
      Cik[12][1][2]*gk[11][1])));
    gk[13][0] = ((Cik[13][2][0]*gk[12][2])+((Cik[13][0][0]*gk[12][0])+(
      Cik[13][1][0]*gk[12][1])));
    gk[13][1] = ((Cik[13][2][1]*gk[12][2])+((Cik[13][0][1]*gk[12][0])+(
      Cik[13][1][1]*gk[12][1])));
    gk[13][2] = ((Cik[13][2][2]*gk[12][2])+((Cik[13][0][2]*gk[12][0])+(
      Cik[13][1][2]*gk[12][1])));
    gk[14][0] = ((Cik[14][2][0]*gk[13][2])+((Cik[14][0][0]*gk[13][0])+(
      Cik[14][1][0]*gk[13][1])));
    gk[14][1] = ((Cik[14][2][1]*gk[13][2])+((Cik[14][0][1]*gk[13][0])+(
      Cik[14][1][1]*gk[13][1])));
    gk[14][2] = ((Cik[14][2][2]*gk[13][2])+((Cik[14][0][2]*gk[13][0])+(
      Cik[14][1][2]*gk[13][1])));
    gk[15][0] = ((Cik[15][2][0]*gk[3][2])+((Cik[15][0][0]*gk[3][0])+(
      Cik[15][1][0]*gk[3][1])));
    gk[15][1] = ((Cik[15][2][1]*gk[3][2])+((Cik[15][0][1]*gk[3][0])+(
      Cik[15][1][1]*gk[3][1])));
    gk[15][2] = ((Cik[15][2][2]*gk[3][2])+((Cik[15][0][2]*gk[3][0])+(
      Cik[15][1][2]*gk[3][1])));
    gk[16][0] = ((Cik[16][2][0]*gk[15][2])+((Cik[16][0][0]*gk[15][0])+(
      Cik[16][1][0]*gk[15][1])));
    gk[16][1] = ((Cik[16][2][1]*gk[15][2])+((Cik[16][0][1]*gk[15][0])+(
      Cik[16][1][1]*gk[15][1])));
    gk[16][2] = ((Cik[16][2][2]*gk[15][2])+((Cik[16][0][2]*gk[15][0])+(
      Cik[16][1][2]*gk[15][1])));
    gk[17][0] = ((Cik[17][2][0]*gk[16][2])+((Cik[17][0][0]*gk[16][0])+(
      Cik[17][1][0]*gk[16][1])));
    gk[17][1] = ((Cik[17][2][1]*gk[16][2])+((Cik[17][0][1]*gk[16][0])+(
      Cik[17][1][1]*gk[16][1])));
    gk[17][2] = ((Cik[17][2][2]*gk[16][2])+((Cik[17][0][2]*gk[16][0])+(
      Cik[17][1][2]*gk[16][1])));
    gk[18][0] = ((Cik[18][2][0]*gk[17][2])+((Cik[18][0][0]*gk[17][0])+(
      Cik[18][1][0]*gk[17][1])));
    gk[18][1] = ((Cik[18][2][1]*gk[17][2])+((Cik[18][0][1]*gk[17][0])+(
      Cik[18][1][1]*gk[17][1])));
    gk[18][2] = ((Cik[18][2][2]*gk[17][2])+((Cik[18][0][2]*gk[17][0])+(
      Cik[18][1][2]*gk[17][1])));
    gk[19][0] = ((Cik[19][2][0]*gk[18][2])+((Cik[19][0][0]*gk[18][0])+(
      Cik[19][1][0]*gk[18][1])));
    gk[19][1] = ((Cik[19][2][1]*gk[18][2])+((Cik[19][0][1]*gk[18][0])+(
      Cik[19][1][1]*gk[18][1])));
    gk[19][2] = ((Cik[19][2][2]*gk[18][2])+((Cik[19][0][2]*gk[18][0])+(
      Cik[19][1][2]*gk[18][1])));
    gk[20][0] = ((Cik[20][2][0]*gk[19][2])+((Cik[20][0][0]*gk[19][0])+(
      Cik[20][1][0]*gk[19][1])));
    gk[20][1] = ((Cik[20][2][1]*gk[19][2])+((Cik[20][0][1]*gk[19][0])+(
      Cik[20][1][1]*gk[19][1])));
    gk[20][2] = ((Cik[20][2][2]*gk[19][2])+((Cik[20][0][2]*gk[19][0])+(
      Cik[20][1][2]*gk[19][1])));
/*
Compute cnk & cnb (direction cosines in N)
*/
    cnk[6][0][0] = ((Cik[3][0][2]*Cik[6][2][0])+((Cik[3][0][0]*Cik[6][0][0])+(
      Cik[3][0][1]*Cik[6][1][0])));
    cnk[6][0][1] = ((Cik[3][0][2]*Cik[6][2][1])+((Cik[3][0][0]*Cik[6][0][1])+(
      Cik[3][0][1]*Cik[6][1][1])));
    cnk[6][0][2] = ((Cik[3][0][2]*Cik[6][2][2])+((Cik[3][0][0]*Cik[6][0][2])+(
      Cik[3][0][1]*Cik[6][1][2])));
    cnk[6][1][0] = ((Cik[3][1][2]*Cik[6][2][0])+((Cik[3][1][0]*Cik[6][0][0])+(
      Cik[3][1][1]*Cik[6][1][0])));
    cnk[6][1][1] = ((Cik[3][1][2]*Cik[6][2][1])+((Cik[3][1][0]*Cik[6][0][1])+(
      Cik[3][1][1]*Cik[6][1][1])));
    cnk[6][1][2] = ((Cik[3][1][2]*Cik[6][2][2])+((Cik[3][1][0]*Cik[6][0][2])+(
      Cik[3][1][1]*Cik[6][1][2])));
    cnk[6][2][0] = ((Cik[3][2][2]*Cik[6][2][0])+((Cik[3][2][0]*Cik[6][0][0])+(
      Cik[3][2][1]*Cik[6][1][0])));
    cnk[6][2][1] = ((Cik[3][2][2]*Cik[6][2][1])+((Cik[3][2][0]*Cik[6][0][1])+(
      Cik[3][2][1]*Cik[6][1][1])));
    cnk[6][2][2] = ((Cik[3][2][2]*Cik[6][2][2])+((Cik[3][2][0]*Cik[6][0][2])+(
      Cik[3][2][1]*Cik[6][1][2])));
    cnk[7][0][0] = ((Cik[7][2][0]*cnk[6][0][2])+((Cik[7][0][0]*cnk[6][0][0])+(
      Cik[7][1][0]*cnk[6][0][1])));
    cnk[7][0][1] = ((Cik[7][2][1]*cnk[6][0][2])+((Cik[7][0][1]*cnk[6][0][0])+(
      Cik[7][1][1]*cnk[6][0][1])));
    cnk[7][0][2] = ((Cik[7][2][2]*cnk[6][0][2])+((Cik[7][0][2]*cnk[6][0][0])+(
      Cik[7][1][2]*cnk[6][0][1])));
    cnk[7][1][0] = ((Cik[7][2][0]*cnk[6][1][2])+((Cik[7][0][0]*cnk[6][1][0])+(
      Cik[7][1][0]*cnk[6][1][1])));
    cnk[7][1][1] = ((Cik[7][2][1]*cnk[6][1][2])+((Cik[7][0][1]*cnk[6][1][0])+(
      Cik[7][1][1]*cnk[6][1][1])));
    cnk[7][1][2] = ((Cik[7][2][2]*cnk[6][1][2])+((Cik[7][0][2]*cnk[6][1][0])+(
      Cik[7][1][2]*cnk[6][1][1])));
    cnk[7][2][0] = ((Cik[7][2][0]*cnk[6][2][2])+((Cik[7][0][0]*cnk[6][2][0])+(
      Cik[7][1][0]*cnk[6][2][1])));
    cnk[7][2][1] = ((Cik[7][2][1]*cnk[6][2][2])+((Cik[7][0][1]*cnk[6][2][0])+(
      Cik[7][1][1]*cnk[6][2][1])));
    cnk[7][2][2] = ((Cik[7][2][2]*cnk[6][2][2])+((Cik[7][0][2]*cnk[6][2][0])+(
      Cik[7][1][2]*cnk[6][2][1])));
    cnk[8][0][0] = ((Cik[8][2][0]*cnk[7][0][2])+((Cik[8][0][0]*cnk[7][0][0])+(
      Cik[8][1][0]*cnk[7][0][1])));
    cnk[8][0][1] = ((Cik[8][2][1]*cnk[7][0][2])+((Cik[8][0][1]*cnk[7][0][0])+(
      Cik[8][1][1]*cnk[7][0][1])));
    cnk[8][0][2] = ((Cik[8][2][2]*cnk[7][0][2])+((Cik[8][0][2]*cnk[7][0][0])+(
      Cik[8][1][2]*cnk[7][0][1])));
    cnk[8][1][0] = ((Cik[8][2][0]*cnk[7][1][2])+((Cik[8][0][0]*cnk[7][1][0])+(
      Cik[8][1][0]*cnk[7][1][1])));
    cnk[8][1][1] = ((Cik[8][2][1]*cnk[7][1][2])+((Cik[8][0][1]*cnk[7][1][0])+(
      Cik[8][1][1]*cnk[7][1][1])));
    cnk[8][1][2] = ((Cik[8][2][2]*cnk[7][1][2])+((Cik[8][0][2]*cnk[7][1][0])+(
      Cik[8][1][2]*cnk[7][1][1])));
    cnk[8][2][0] = ((Cik[8][2][0]*cnk[7][2][2])+((Cik[8][0][0]*cnk[7][2][0])+(
      Cik[8][1][0]*cnk[7][2][1])));
    cnk[8][2][1] = ((Cik[8][2][1]*cnk[7][2][2])+((Cik[8][0][1]*cnk[7][2][0])+(
      Cik[8][1][1]*cnk[7][2][1])));
    cnk[8][2][2] = ((Cik[8][2][2]*cnk[7][2][2])+((Cik[8][0][2]*cnk[7][2][0])+(
      Cik[8][1][2]*cnk[7][2][1])));
    cnk[9][0][0] = ((Cik[3][0][2]*Cik[9][2][0])+((Cik[3][0][0]*Cik[9][0][0])+(
      Cik[3][0][1]*Cik[9][1][0])));
    cnk[9][0][1] = ((Cik[3][0][2]*Cik[9][2][1])+((Cik[3][0][0]*Cik[9][0][1])+(
      Cik[3][0][1]*Cik[9][1][1])));
    cnk[9][0][2] = ((Cik[3][0][2]*Cik[9][2][2])+((Cik[3][0][0]*Cik[9][0][2])+(
      Cik[3][0][1]*Cik[9][1][2])));
    cnk[9][1][0] = ((Cik[3][1][2]*Cik[9][2][0])+((Cik[3][1][0]*Cik[9][0][0])+(
      Cik[3][1][1]*Cik[9][1][0])));
    cnk[9][1][1] = ((Cik[3][1][2]*Cik[9][2][1])+((Cik[3][1][0]*Cik[9][0][1])+(
      Cik[3][1][1]*Cik[9][1][1])));
    cnk[9][1][2] = ((Cik[3][1][2]*Cik[9][2][2])+((Cik[3][1][0]*Cik[9][0][2])+(
      Cik[3][1][1]*Cik[9][1][2])));
    cnk[9][2][0] = ((Cik[3][2][2]*Cik[9][2][0])+((Cik[3][2][0]*Cik[9][0][0])+(
      Cik[3][2][1]*Cik[9][1][0])));
    cnk[9][2][1] = ((Cik[3][2][2]*Cik[9][2][1])+((Cik[3][2][0]*Cik[9][0][1])+(
      Cik[3][2][1]*Cik[9][1][1])));
    cnk[9][2][2] = ((Cik[3][2][2]*Cik[9][2][2])+((Cik[3][2][0]*Cik[9][0][2])+(
      Cik[3][2][1]*Cik[9][1][2])));
    cnk[10][0][0] = ((Cik[10][2][0]*cnk[9][0][2])+((Cik[10][0][0]*cnk[9][0][0])+
      (Cik[10][1][0]*cnk[9][0][1])));
    cnk[10][0][1] = ((Cik[10][2][1]*cnk[9][0][2])+((Cik[10][0][1]*cnk[9][0][0])+
      (Cik[10][1][1]*cnk[9][0][1])));
    cnk[10][0][2] = ((Cik[10][2][2]*cnk[9][0][2])+((Cik[10][0][2]*cnk[9][0][0])+
      (Cik[10][1][2]*cnk[9][0][1])));
    cnk[10][1][0] = ((Cik[10][2][0]*cnk[9][1][2])+((Cik[10][0][0]*cnk[9][1][0])+
      (Cik[10][1][0]*cnk[9][1][1])));
    cnk[10][1][1] = ((Cik[10][2][1]*cnk[9][1][2])+((Cik[10][0][1]*cnk[9][1][0])+
      (Cik[10][1][1]*cnk[9][1][1])));
    cnk[10][1][2] = ((Cik[10][2][2]*cnk[9][1][2])+((Cik[10][0][2]*cnk[9][1][0])+
      (Cik[10][1][2]*cnk[9][1][1])));
    cnk[10][2][0] = ((Cik[10][2][0]*cnk[9][2][2])+((Cik[10][0][0]*cnk[9][2][0])+
      (Cik[10][1][0]*cnk[9][2][1])));
    cnk[10][2][1] = ((Cik[10][2][1]*cnk[9][2][2])+((Cik[10][0][1]*cnk[9][2][0])+
      (Cik[10][1][1]*cnk[9][2][1])));
    cnk[10][2][2] = ((Cik[10][2][2]*cnk[9][2][2])+((Cik[10][0][2]*cnk[9][2][0])+
      (Cik[10][1][2]*cnk[9][2][1])));
    cnk[11][0][0] = ((Cik[11][2][0]*cnk[10][0][2])+((Cik[11][0][0]*cnk[10][0][0]
      )+(Cik[11][1][0]*cnk[10][0][1])));
    cnk[11][0][1] = ((Cik[11][2][1]*cnk[10][0][2])+((Cik[11][0][1]*cnk[10][0][0]
      )+(Cik[11][1][1]*cnk[10][0][1])));
    cnk[11][0][2] = ((Cik[11][2][2]*cnk[10][0][2])+((Cik[11][0][2]*cnk[10][0][0]
      )+(Cik[11][1][2]*cnk[10][0][1])));
    cnk[11][1][0] = ((Cik[11][2][0]*cnk[10][1][2])+((Cik[11][0][0]*cnk[10][1][0]
      )+(Cik[11][1][0]*cnk[10][1][1])));
    cnk[11][1][1] = ((Cik[11][2][1]*cnk[10][1][2])+((Cik[11][0][1]*cnk[10][1][0]
      )+(Cik[11][1][1]*cnk[10][1][1])));
    cnk[11][1][2] = ((Cik[11][2][2]*cnk[10][1][2])+((Cik[11][0][2]*cnk[10][1][0]
      )+(Cik[11][1][2]*cnk[10][1][1])));
    cnk[11][2][0] = ((Cik[11][2][0]*cnk[10][2][2])+((Cik[11][0][0]*cnk[10][2][0]
      )+(Cik[11][1][0]*cnk[10][2][1])));
    cnk[11][2][1] = ((Cik[11][2][1]*cnk[10][2][2])+((Cik[11][0][1]*cnk[10][2][0]
      )+(Cik[11][1][1]*cnk[10][2][1])));
    cnk[11][2][2] = ((Cik[11][2][2]*cnk[10][2][2])+((Cik[11][0][2]*cnk[10][2][0]
      )+(Cik[11][1][2]*cnk[10][2][1])));
    cnk[12][0][0] = ((Cik[12][2][0]*cnk[11][0][2])+((Cik[12][0][0]*cnk[11][0][0]
      )+(Cik[12][1][0]*cnk[11][0][1])));
    cnk[12][0][1] = ((Cik[12][2][1]*cnk[11][0][2])+((Cik[12][0][1]*cnk[11][0][0]
      )+(Cik[12][1][1]*cnk[11][0][1])));
    cnk[12][0][2] = ((Cik[12][2][2]*cnk[11][0][2])+((Cik[12][0][2]*cnk[11][0][0]
      )+(Cik[12][1][2]*cnk[11][0][1])));
    cnk[12][1][0] = ((Cik[12][2][0]*cnk[11][1][2])+((Cik[12][0][0]*cnk[11][1][0]
      )+(Cik[12][1][0]*cnk[11][1][1])));
    cnk[12][1][1] = ((Cik[12][2][1]*cnk[11][1][2])+((Cik[12][0][1]*cnk[11][1][0]
      )+(Cik[12][1][1]*cnk[11][1][1])));
    cnk[12][1][2] = ((Cik[12][2][2]*cnk[11][1][2])+((Cik[12][0][2]*cnk[11][1][0]
      )+(Cik[12][1][2]*cnk[11][1][1])));
    cnk[12][2][0] = ((Cik[12][2][0]*cnk[11][2][2])+((Cik[12][0][0]*cnk[11][2][0]
      )+(Cik[12][1][0]*cnk[11][2][1])));
    cnk[12][2][1] = ((Cik[12][2][1]*cnk[11][2][2])+((Cik[12][0][1]*cnk[11][2][0]
      )+(Cik[12][1][1]*cnk[11][2][1])));
    cnk[12][2][2] = ((Cik[12][2][2]*cnk[11][2][2])+((Cik[12][0][2]*cnk[11][2][0]
      )+(Cik[12][1][2]*cnk[11][2][1])));
    cnk[13][0][0] = ((Cik[13][2][0]*cnk[12][0][2])+((Cik[13][0][0]*cnk[12][0][0]
      )+(Cik[13][1][0]*cnk[12][0][1])));
    cnk[13][0][1] = ((Cik[13][2][1]*cnk[12][0][2])+((Cik[13][0][1]*cnk[12][0][0]
      )+(Cik[13][1][1]*cnk[12][0][1])));
    cnk[13][0][2] = ((Cik[13][2][2]*cnk[12][0][2])+((Cik[13][0][2]*cnk[12][0][0]
      )+(Cik[13][1][2]*cnk[12][0][1])));
    cnk[13][1][0] = ((Cik[13][2][0]*cnk[12][1][2])+((Cik[13][0][0]*cnk[12][1][0]
      )+(Cik[13][1][0]*cnk[12][1][1])));
    cnk[13][1][1] = ((Cik[13][2][1]*cnk[12][1][2])+((Cik[13][0][1]*cnk[12][1][0]
      )+(Cik[13][1][1]*cnk[12][1][1])));
    cnk[13][1][2] = ((Cik[13][2][2]*cnk[12][1][2])+((Cik[13][0][2]*cnk[12][1][0]
      )+(Cik[13][1][2]*cnk[12][1][1])));
    cnk[13][2][0] = ((Cik[13][2][0]*cnk[12][2][2])+((Cik[13][0][0]*cnk[12][2][0]
      )+(Cik[13][1][0]*cnk[12][2][1])));
    cnk[13][2][1] = ((Cik[13][2][1]*cnk[12][2][2])+((Cik[13][0][1]*cnk[12][2][0]
      )+(Cik[13][1][1]*cnk[12][2][1])));
    cnk[13][2][2] = ((Cik[13][2][2]*cnk[12][2][2])+((Cik[13][0][2]*cnk[12][2][0]
      )+(Cik[13][1][2]*cnk[12][2][1])));
    cnk[14][0][0] = ((Cik[14][2][0]*cnk[13][0][2])+((Cik[14][0][0]*cnk[13][0][0]
      )+(Cik[14][1][0]*cnk[13][0][1])));
    cnk[14][0][1] = ((Cik[14][2][1]*cnk[13][0][2])+((Cik[14][0][1]*cnk[13][0][0]
      )+(Cik[14][1][1]*cnk[13][0][1])));
    cnk[14][0][2] = ((Cik[14][2][2]*cnk[13][0][2])+((Cik[14][0][2]*cnk[13][0][0]
      )+(Cik[14][1][2]*cnk[13][0][1])));
    cnk[14][1][0] = ((Cik[14][2][0]*cnk[13][1][2])+((Cik[14][0][0]*cnk[13][1][0]
      )+(Cik[14][1][0]*cnk[13][1][1])));
    cnk[14][1][1] = ((Cik[14][2][1]*cnk[13][1][2])+((Cik[14][0][1]*cnk[13][1][0]
      )+(Cik[14][1][1]*cnk[13][1][1])));
    cnk[14][1][2] = ((Cik[14][2][2]*cnk[13][1][2])+((Cik[14][0][2]*cnk[13][1][0]
      )+(Cik[14][1][2]*cnk[13][1][1])));
    cnk[14][2][0] = ((Cik[14][2][0]*cnk[13][2][2])+((Cik[14][0][0]*cnk[13][2][0]
      )+(Cik[14][1][0]*cnk[13][2][1])));
    cnk[14][2][1] = ((Cik[14][2][1]*cnk[13][2][2])+((Cik[14][0][1]*cnk[13][2][0]
      )+(Cik[14][1][1]*cnk[13][2][1])));
    cnk[14][2][2] = ((Cik[14][2][2]*cnk[13][2][2])+((Cik[14][0][2]*cnk[13][2][0]
      )+(Cik[14][1][2]*cnk[13][2][1])));
    cnk[15][0][0] = ((Cik[3][0][2]*Cik[15][2][0])+((Cik[3][0][0]*Cik[15][0][0])+
      (Cik[3][0][1]*Cik[15][1][0])));
    cnk[15][0][1] = ((Cik[3][0][2]*Cik[15][2][1])+((Cik[3][0][0]*Cik[15][0][1])+
      (Cik[3][0][1]*Cik[15][1][1])));
    cnk[15][0][2] = ((Cik[3][0][2]*Cik[15][2][2])+((Cik[3][0][0]*Cik[15][0][2])+
      (Cik[3][0][1]*Cik[15][1][2])));
    cnk[15][1][0] = ((Cik[3][1][2]*Cik[15][2][0])+((Cik[3][1][0]*Cik[15][0][0])+
      (Cik[3][1][1]*Cik[15][1][0])));
    cnk[15][1][1] = ((Cik[3][1][2]*Cik[15][2][1])+((Cik[3][1][0]*Cik[15][0][1])+
      (Cik[3][1][1]*Cik[15][1][1])));
    cnk[15][1][2] = ((Cik[3][1][2]*Cik[15][2][2])+((Cik[3][1][0]*Cik[15][0][2])+
      (Cik[3][1][1]*Cik[15][1][2])));
    cnk[15][2][0] = ((Cik[3][2][2]*Cik[15][2][0])+((Cik[3][2][0]*Cik[15][0][0])+
      (Cik[3][2][1]*Cik[15][1][0])));
    cnk[15][2][1] = ((Cik[3][2][2]*Cik[15][2][1])+((Cik[3][2][0]*Cik[15][0][1])+
      (Cik[3][2][1]*Cik[15][1][1])));
    cnk[15][2][2] = ((Cik[3][2][2]*Cik[15][2][2])+((Cik[3][2][0]*Cik[15][0][2])+
      (Cik[3][2][1]*Cik[15][1][2])));
    cnk[16][0][0] = ((Cik[16][2][0]*cnk[15][0][2])+((Cik[16][0][0]*cnk[15][0][0]
      )+(Cik[16][1][0]*cnk[15][0][1])));
    cnk[16][0][1] = ((Cik[16][2][1]*cnk[15][0][2])+((Cik[16][0][1]*cnk[15][0][0]
      )+(Cik[16][1][1]*cnk[15][0][1])));
    cnk[16][0][2] = ((Cik[16][2][2]*cnk[15][0][2])+((Cik[16][0][2]*cnk[15][0][0]
      )+(Cik[16][1][2]*cnk[15][0][1])));
    cnk[16][1][0] = ((Cik[16][2][0]*cnk[15][1][2])+((Cik[16][0][0]*cnk[15][1][0]
      )+(Cik[16][1][0]*cnk[15][1][1])));
    cnk[16][1][1] = ((Cik[16][2][1]*cnk[15][1][2])+((Cik[16][0][1]*cnk[15][1][0]
      )+(Cik[16][1][1]*cnk[15][1][1])));
    cnk[16][1][2] = ((Cik[16][2][2]*cnk[15][1][2])+((Cik[16][0][2]*cnk[15][1][0]
      )+(Cik[16][1][2]*cnk[15][1][1])));
    cnk[16][2][0] = ((Cik[16][2][0]*cnk[15][2][2])+((Cik[16][0][0]*cnk[15][2][0]
      )+(Cik[16][1][0]*cnk[15][2][1])));
    cnk[16][2][1] = ((Cik[16][2][1]*cnk[15][2][2])+((Cik[16][0][1]*cnk[15][2][0]
      )+(Cik[16][1][1]*cnk[15][2][1])));
    cnk[16][2][2] = ((Cik[16][2][2]*cnk[15][2][2])+((Cik[16][0][2]*cnk[15][2][0]
      )+(Cik[16][1][2]*cnk[15][2][1])));
    cnk[17][0][0] = ((Cik[17][2][0]*cnk[16][0][2])+((Cik[17][0][0]*cnk[16][0][0]
      )+(Cik[17][1][0]*cnk[16][0][1])));
    cnk[17][0][1] = ((Cik[17][2][1]*cnk[16][0][2])+((Cik[17][0][1]*cnk[16][0][0]
      )+(Cik[17][1][1]*cnk[16][0][1])));
    cnk[17][0][2] = ((Cik[17][2][2]*cnk[16][0][2])+((Cik[17][0][2]*cnk[16][0][0]
      )+(Cik[17][1][2]*cnk[16][0][1])));
    cnk[17][1][0] = ((Cik[17][2][0]*cnk[16][1][2])+((Cik[17][0][0]*cnk[16][1][0]
      )+(Cik[17][1][0]*cnk[16][1][1])));
    cnk[17][1][1] = ((Cik[17][2][1]*cnk[16][1][2])+((Cik[17][0][1]*cnk[16][1][0]
      )+(Cik[17][1][1]*cnk[16][1][1])));
    cnk[17][1][2] = ((Cik[17][2][2]*cnk[16][1][2])+((Cik[17][0][2]*cnk[16][1][0]
      )+(Cik[17][1][2]*cnk[16][1][1])));
    cnk[17][2][0] = ((Cik[17][2][0]*cnk[16][2][2])+((Cik[17][0][0]*cnk[16][2][0]
      )+(Cik[17][1][0]*cnk[16][2][1])));
    cnk[17][2][1] = ((Cik[17][2][1]*cnk[16][2][2])+((Cik[17][0][1]*cnk[16][2][0]
      )+(Cik[17][1][1]*cnk[16][2][1])));
    cnk[17][2][2] = ((Cik[17][2][2]*cnk[16][2][2])+((Cik[17][0][2]*cnk[16][2][0]
      )+(Cik[17][1][2]*cnk[16][2][1])));
    cnk[18][0][0] = ((Cik[18][2][0]*cnk[17][0][2])+((Cik[18][0][0]*cnk[17][0][0]
      )+(Cik[18][1][0]*cnk[17][0][1])));
    cnk[18][0][1] = ((Cik[18][2][1]*cnk[17][0][2])+((Cik[18][0][1]*cnk[17][0][0]
      )+(Cik[18][1][1]*cnk[17][0][1])));
    cnk[18][0][2] = ((Cik[18][2][2]*cnk[17][0][2])+((Cik[18][0][2]*cnk[17][0][0]
      )+(Cik[18][1][2]*cnk[17][0][1])));
    cnk[18][1][0] = ((Cik[18][2][0]*cnk[17][1][2])+((Cik[18][0][0]*cnk[17][1][0]
      )+(Cik[18][1][0]*cnk[17][1][1])));
    cnk[18][1][1] = ((Cik[18][2][1]*cnk[17][1][2])+((Cik[18][0][1]*cnk[17][1][0]
      )+(Cik[18][1][1]*cnk[17][1][1])));
    cnk[18][1][2] = ((Cik[18][2][2]*cnk[17][1][2])+((Cik[18][0][2]*cnk[17][1][0]
      )+(Cik[18][1][2]*cnk[17][1][1])));
    cnk[18][2][0] = ((Cik[18][2][0]*cnk[17][2][2])+((Cik[18][0][0]*cnk[17][2][0]
      )+(Cik[18][1][0]*cnk[17][2][1])));
    cnk[18][2][1] = ((Cik[18][2][1]*cnk[17][2][2])+((Cik[18][0][1]*cnk[17][2][0]
      )+(Cik[18][1][1]*cnk[17][2][1])));
    cnk[18][2][2] = ((Cik[18][2][2]*cnk[17][2][2])+((Cik[18][0][2]*cnk[17][2][0]
      )+(Cik[18][1][2]*cnk[17][2][1])));
    cnk[19][0][0] = ((Cik[19][2][0]*cnk[18][0][2])+((Cik[19][0][0]*cnk[18][0][0]
      )+(Cik[19][1][0]*cnk[18][0][1])));
    cnk[19][0][1] = ((Cik[19][2][1]*cnk[18][0][2])+((Cik[19][0][1]*cnk[18][0][0]
      )+(Cik[19][1][1]*cnk[18][0][1])));
    cnk[19][0][2] = ((Cik[19][2][2]*cnk[18][0][2])+((Cik[19][0][2]*cnk[18][0][0]
      )+(Cik[19][1][2]*cnk[18][0][1])));
    cnk[19][1][0] = ((Cik[19][2][0]*cnk[18][1][2])+((Cik[19][0][0]*cnk[18][1][0]
      )+(Cik[19][1][0]*cnk[18][1][1])));
    cnk[19][1][1] = ((Cik[19][2][1]*cnk[18][1][2])+((Cik[19][0][1]*cnk[18][1][0]
      )+(Cik[19][1][1]*cnk[18][1][1])));
    cnk[19][1][2] = ((Cik[19][2][2]*cnk[18][1][2])+((Cik[19][0][2]*cnk[18][1][0]
      )+(Cik[19][1][2]*cnk[18][1][1])));
    cnk[19][2][0] = ((Cik[19][2][0]*cnk[18][2][2])+((Cik[19][0][0]*cnk[18][2][0]
      )+(Cik[19][1][0]*cnk[18][2][1])));
    cnk[19][2][1] = ((Cik[19][2][1]*cnk[18][2][2])+((Cik[19][0][1]*cnk[18][2][0]
      )+(Cik[19][1][1]*cnk[18][2][1])));
    cnk[19][2][2] = ((Cik[19][2][2]*cnk[18][2][2])+((Cik[19][0][2]*cnk[18][2][0]
      )+(Cik[19][1][2]*cnk[18][2][1])));
    cnk[20][0][0] = ((Cik[20][2][0]*cnk[19][0][2])+((Cik[20][0][0]*cnk[19][0][0]
      )+(Cik[20][1][0]*cnk[19][0][1])));
    cnk[20][0][1] = ((Cik[20][2][1]*cnk[19][0][2])+((Cik[20][0][1]*cnk[19][0][0]
      )+(Cik[20][1][1]*cnk[19][0][1])));
    cnk[20][0][2] = ((Cik[20][2][2]*cnk[19][0][2])+((Cik[20][0][2]*cnk[19][0][0]
      )+(Cik[20][1][2]*cnk[19][0][1])));
    cnk[20][1][0] = ((Cik[20][2][0]*cnk[19][1][2])+((Cik[20][0][0]*cnk[19][1][0]
      )+(Cik[20][1][0]*cnk[19][1][1])));
    cnk[20][1][1] = ((Cik[20][2][1]*cnk[19][1][2])+((Cik[20][0][1]*cnk[19][1][0]
      )+(Cik[20][1][1]*cnk[19][1][1])));
    cnk[20][1][2] = ((Cik[20][2][2]*cnk[19][1][2])+((Cik[20][0][2]*cnk[19][1][0]
      )+(Cik[20][1][2]*cnk[19][1][1])));
    cnk[20][2][0] = ((Cik[20][2][0]*cnk[19][2][2])+((Cik[20][0][0]*cnk[19][2][0]
      )+(Cik[20][1][0]*cnk[19][2][1])));
    cnk[20][2][1] = ((Cik[20][2][1]*cnk[19][2][2])+((Cik[20][0][1]*cnk[19][2][0]
      )+(Cik[20][1][1]*cnk[19][2][1])));
    cnk[20][2][2] = ((Cik[20][2][2]*cnk[19][2][2])+((Cik[20][0][2]*cnk[19][2][0]
      )+(Cik[20][1][2]*cnk[19][2][1])));
    cnb[0][0][0] = Cik[3][0][0];
    cnb[0][0][1] = Cik[3][0][1];
    cnb[0][0][2] = Cik[3][0][2];
    cnb[0][1][0] = Cik[3][1][0];
    cnb[0][1][1] = Cik[3][1][1];
    cnb[0][1][2] = Cik[3][1][2];
    cnb[0][2][0] = Cik[3][2][0];
    cnb[0][2][1] = Cik[3][2][1];
    cnb[0][2][2] = Cik[3][2][2];
    cnb[1][0][0] = cnk[6][0][0];
    cnb[1][0][1] = cnk[6][0][1];
    cnb[1][0][2] = cnk[6][0][2];
    cnb[1][1][0] = cnk[6][1][0];
    cnb[1][1][1] = cnk[6][1][1];
    cnb[1][1][2] = cnk[6][1][2];
    cnb[1][2][0] = cnk[6][2][0];
    cnb[1][2][1] = cnk[6][2][1];
    cnb[1][2][2] = cnk[6][2][2];
    cnb[2][0][0] = cnk[7][0][0];
    cnb[2][0][1] = cnk[7][0][1];
    cnb[2][0][2] = cnk[7][0][2];
    cnb[2][1][0] = cnk[7][1][0];
    cnb[2][1][1] = cnk[7][1][1];
    cnb[2][1][2] = cnk[7][1][2];
    cnb[2][2][0] = cnk[7][2][0];
    cnb[2][2][1] = cnk[7][2][1];
    cnb[2][2][2] = cnk[7][2][2];
    cnb[3][0][0] = cnk[8][0][0];
    cnb[3][0][1] = cnk[8][0][1];
    cnb[3][0][2] = cnk[8][0][2];
    cnb[3][1][0] = cnk[8][1][0];
    cnb[3][1][1] = cnk[8][1][1];
    cnb[3][1][2] = cnk[8][1][2];
    cnb[3][2][0] = cnk[8][2][0];
    cnb[3][2][1] = cnk[8][2][1];
    cnb[3][2][2] = cnk[8][2][2];
    cnb[4][0][0] = cnk[9][0][0];
    cnb[4][0][1] = cnk[9][0][1];
    cnb[4][0][2] = cnk[9][0][2];
    cnb[4][1][0] = cnk[9][1][0];
    cnb[4][1][1] = cnk[9][1][1];
    cnb[4][1][2] = cnk[9][1][2];
    cnb[4][2][0] = cnk[9][2][0];
    cnb[4][2][1] = cnk[9][2][1];
    cnb[4][2][2] = cnk[9][2][2];
    cnb[5][0][0] = cnk[10][0][0];
    cnb[5][0][1] = cnk[10][0][1];
    cnb[5][0][2] = cnk[10][0][2];
    cnb[5][1][0] = cnk[10][1][0];
    cnb[5][1][1] = cnk[10][1][1];
    cnb[5][1][2] = cnk[10][1][2];
    cnb[5][2][0] = cnk[10][2][0];
    cnb[5][2][1] = cnk[10][2][1];
    cnb[5][2][2] = cnk[10][2][2];
    cnb[6][0][0] = cnk[11][0][0];
    cnb[6][0][1] = cnk[11][0][1];
    cnb[6][0][2] = cnk[11][0][2];
    cnb[6][1][0] = cnk[11][1][0];
    cnb[6][1][1] = cnk[11][1][1];
    cnb[6][1][2] = cnk[11][1][2];
    cnb[6][2][0] = cnk[11][2][0];
    cnb[6][2][1] = cnk[11][2][1];
    cnb[6][2][2] = cnk[11][2][2];
    cnb[7][0][0] = cnk[12][0][0];
    cnb[7][0][1] = cnk[12][0][1];
    cnb[7][0][2] = cnk[12][0][2];
    cnb[7][1][0] = cnk[12][1][0];
    cnb[7][1][1] = cnk[12][1][1];
    cnb[7][1][2] = cnk[12][1][2];
    cnb[7][2][0] = cnk[12][2][0];
    cnb[7][2][1] = cnk[12][2][1];
    cnb[7][2][2] = cnk[12][2][2];
    cnb[8][0][0] = cnk[13][0][0];
    cnb[8][0][1] = cnk[13][0][1];
    cnb[8][0][2] = cnk[13][0][2];
    cnb[8][1][0] = cnk[13][1][0];
    cnb[8][1][1] = cnk[13][1][1];
    cnb[8][1][2] = cnk[13][1][2];
    cnb[8][2][0] = cnk[13][2][0];
    cnb[8][2][1] = cnk[13][2][1];
    cnb[8][2][2] = cnk[13][2][2];
    cnb[9][0][0] = cnk[14][0][0];
    cnb[9][0][1] = cnk[14][0][1];
    cnb[9][0][2] = cnk[14][0][2];
    cnb[9][1][0] = cnk[14][1][0];
    cnb[9][1][1] = cnk[14][1][1];
    cnb[9][1][2] = cnk[14][1][2];
    cnb[9][2][0] = cnk[14][2][0];
    cnb[9][2][1] = cnk[14][2][1];
    cnb[9][2][2] = cnk[14][2][2];
    cnb[10][0][0] = cnk[15][0][0];
    cnb[10][0][1] = cnk[15][0][1];
    cnb[10][0][2] = cnk[15][0][2];
    cnb[10][1][0] = cnk[15][1][0];
    cnb[10][1][1] = cnk[15][1][1];
    cnb[10][1][2] = cnk[15][1][2];
    cnb[10][2][0] = cnk[15][2][0];
    cnb[10][2][1] = cnk[15][2][1];
    cnb[10][2][2] = cnk[15][2][2];
    cnb[11][0][0] = cnk[16][0][0];
    cnb[11][0][1] = cnk[16][0][1];
    cnb[11][0][2] = cnk[16][0][2];
    cnb[11][1][0] = cnk[16][1][0];
    cnb[11][1][1] = cnk[16][1][1];
    cnb[11][1][2] = cnk[16][1][2];
    cnb[11][2][0] = cnk[16][2][0];
    cnb[11][2][1] = cnk[16][2][1];
    cnb[11][2][2] = cnk[16][2][2];
    cnb[12][0][0] = cnk[17][0][0];
    cnb[12][0][1] = cnk[17][0][1];
    cnb[12][0][2] = cnk[17][0][2];
    cnb[12][1][0] = cnk[17][1][0];
    cnb[12][1][1] = cnk[17][1][1];
    cnb[12][1][2] = cnk[17][1][2];
    cnb[12][2][0] = cnk[17][2][0];
    cnb[12][2][1] = cnk[17][2][1];
    cnb[12][2][2] = cnk[17][2][2];
    cnb[13][0][0] = cnk[18][0][0];
    cnb[13][0][1] = cnk[18][0][1];
    cnb[13][0][2] = cnk[18][0][2];
    cnb[13][1][0] = cnk[18][1][0];
    cnb[13][1][1] = cnk[18][1][1];
    cnb[13][1][2] = cnk[18][1][2];
    cnb[13][2][0] = cnk[18][2][0];
    cnb[13][2][1] = cnk[18][2][1];
    cnb[13][2][2] = cnk[18][2][2];
    cnb[14][0][0] = cnk[19][0][0];
    cnb[14][0][1] = cnk[19][0][1];
    cnb[14][0][2] = cnk[19][0][2];
    cnb[14][1][0] = cnk[19][1][0];
    cnb[14][1][1] = cnk[19][1][1];
    cnb[14][1][2] = cnk[19][1][2];
    cnb[14][2][0] = cnk[19][2][0];
    cnb[14][2][1] = cnk[19][2][1];
    cnb[14][2][2] = cnk[19][2][2];
    cnb[15][0][0] = cnk[20][0][0];
    cnb[15][0][1] = cnk[20][0][1];
    cnb[15][0][2] = cnk[20][0][2];
    cnb[15][1][0] = cnk[20][1][0];
    cnb[15][1][1] = cnk[20][1][1];
    cnb[15][1][2] = cnk[20][1][2];
    cnb[15][2][0] = cnk[20][2][0];
    cnb[15][2][1] = cnk[20][2][1];
    cnb[15][2][2] = cnk[20][2][2];
/*
Compute q-related auxiliary variables
*/
    rpp[0][0] = (pin[0][0]*q[0]);
    rpp[0][1] = (pin[0][1]*q[0]);
    rpp[0][2] = (pin[0][2]*q[0]);
    rpp[1][0] = (pin[1][0]*q[1]);
    rpp[1][1] = (pin[1][1]*q[1]);
    rpp[1][2] = (pin[1][2]*q[1]);
    rpp[2][0] = (pin[2][0]*q[2]);
    rpp[2][1] = (pin[2][1]*q[2]);
    rpp[2][2] = (pin[2][2]*q[2]);
    rpri[0][0] = (ri[0][0]+rpp[0][0]);
    rpri[0][1] = (ri[0][1]+rpp[0][1]);
    rpri[0][2] = (ri[0][2]+rpp[0][2]);
    rik[0][0] = (ri[0][0]+rpp[0][0]);
    rik[0][1] = (ri[0][1]+rpp[0][1]);
    rik[0][2] = (ri[0][2]+rpp[0][2]);
    rik[6][0] = (((Cik[6][2][0]*ri[1][2])+((Cik[6][0][0]*ri[1][0])+(Cik[6][1][0]
      *ri[1][1])))-rk[1][0]);
    rik[6][1] = (((Cik[6][2][1]*ri[1][2])+((Cik[6][0][1]*ri[1][0])+(Cik[6][1][1]
      *ri[1][1])))-rk[1][1]);
    rik[6][2] = (((Cik[6][2][2]*ri[1][2])+((Cik[6][0][2]*ri[1][0])+(Cik[6][1][2]
      *ri[1][1])))-rk[1][2]);
    rik[7][0] = (((Cik[7][2][0]*ri[2][2])+((Cik[7][0][0]*ri[2][0])+(Cik[7][1][0]
      *ri[2][1])))-rk[2][0]);
    rik[7][1] = (((Cik[7][2][1]*ri[2][2])+((Cik[7][0][1]*ri[2][0])+(Cik[7][1][1]
      *ri[2][1])))-rk[2][1]);
    rik[7][2] = (((Cik[7][2][2]*ri[2][2])+((Cik[7][0][2]*ri[2][0])+(Cik[7][1][2]
      *ri[2][1])))-rk[2][2]);
    rik[8][0] = (((Cik[8][2][0]*ri[3][2])+((Cik[8][0][0]*ri[3][0])+(Cik[8][1][0]
      *ri[3][1])))-rk[3][0]);
    rik[8][1] = (((Cik[8][2][1]*ri[3][2])+((Cik[8][0][1]*ri[3][0])+(Cik[8][1][1]
      *ri[3][1])))-rk[3][1]);
    rik[8][2] = (((Cik[8][2][2]*ri[3][2])+((Cik[8][0][2]*ri[3][0])+(Cik[8][1][2]
      *ri[3][1])))-rk[3][2]);
    rik[9][0] = (((Cik[9][2][0]*ri[4][2])+((Cik[9][0][0]*ri[4][0])+(Cik[9][1][0]
      *ri[4][1])))-rk[4][0]);
    rik[9][1] = (((Cik[9][2][1]*ri[4][2])+((Cik[9][0][1]*ri[4][0])+(Cik[9][1][1]
      *ri[4][1])))-rk[4][1]);
    rik[9][2] = (((Cik[9][2][2]*ri[4][2])+((Cik[9][0][2]*ri[4][0])+(Cik[9][1][2]
      *ri[4][1])))-rk[4][2]);
    rik[10][0] = (((Cik[10][2][0]*ri[5][2])+((Cik[10][0][0]*ri[5][0])+(
      Cik[10][1][0]*ri[5][1])))-rk[5][0]);
    rik[10][1] = (((Cik[10][2][1]*ri[5][2])+((Cik[10][0][1]*ri[5][0])+(
      Cik[10][1][1]*ri[5][1])))-rk[5][1]);
    rik[10][2] = (((Cik[10][2][2]*ri[5][2])+((Cik[10][0][2]*ri[5][0])+(
      Cik[10][1][2]*ri[5][1])))-rk[5][2]);
    rik[11][0] = (((Cik[11][2][0]*ri[6][2])+((Cik[11][0][0]*ri[6][0])+(
      Cik[11][1][0]*ri[6][1])))-rk[6][0]);
    rik[11][1] = (((Cik[11][2][1]*ri[6][2])+((Cik[11][0][1]*ri[6][0])+(
      Cik[11][1][1]*ri[6][1])))-rk[6][1]);
    rik[11][2] = (((Cik[11][2][2]*ri[6][2])+((Cik[11][0][2]*ri[6][0])+(
      Cik[11][1][2]*ri[6][1])))-rk[6][2]);
    rik[12][0] = (((Cik[12][2][0]*ri[7][2])+((Cik[12][0][0]*ri[7][0])+(
      Cik[12][1][0]*ri[7][1])))-rk[7][0]);
    rik[12][1] = (((Cik[12][2][1]*ri[7][2])+((Cik[12][0][1]*ri[7][0])+(
      Cik[12][1][1]*ri[7][1])))-rk[7][1]);
    rik[12][2] = (((Cik[12][2][2]*ri[7][2])+((Cik[12][0][2]*ri[7][0])+(
      Cik[12][1][2]*ri[7][1])))-rk[7][2]);
    rik[13][0] = (((Cik[13][2][0]*ri[8][2])+((Cik[13][0][0]*ri[8][0])+(
      Cik[13][1][0]*ri[8][1])))-rk[8][0]);
    rik[13][1] = (((Cik[13][2][1]*ri[8][2])+((Cik[13][0][1]*ri[8][0])+(
      Cik[13][1][1]*ri[8][1])))-rk[8][1]);
    rik[13][2] = (((Cik[13][2][2]*ri[8][2])+((Cik[13][0][2]*ri[8][0])+(
      Cik[13][1][2]*ri[8][1])))-rk[8][2]);
    rik[14][0] = (((Cik[14][2][0]*ri[9][2])+((Cik[14][0][0]*ri[9][0])+(
      Cik[14][1][0]*ri[9][1])))-rk[9][0]);
    rik[14][1] = (((Cik[14][2][1]*ri[9][2])+((Cik[14][0][1]*ri[9][0])+(
      Cik[14][1][1]*ri[9][1])))-rk[9][1]);
    rik[14][2] = (((Cik[14][2][2]*ri[9][2])+((Cik[14][0][2]*ri[9][0])+(
      Cik[14][1][2]*ri[9][1])))-rk[9][2]);
    rik[15][0] = (((Cik[15][2][0]*ri[10][2])+((Cik[15][0][0]*ri[10][0])+(
      Cik[15][1][0]*ri[10][1])))-rk[10][0]);
    rik[15][1] = (((Cik[15][2][1]*ri[10][2])+((Cik[15][0][1]*ri[10][0])+(
      Cik[15][1][1]*ri[10][1])))-rk[10][1]);
    rik[15][2] = (((Cik[15][2][2]*ri[10][2])+((Cik[15][0][2]*ri[10][0])+(
      Cik[15][1][2]*ri[10][1])))-rk[10][2]);
    rik[16][0] = (((Cik[16][2][0]*ri[11][2])+((Cik[16][0][0]*ri[11][0])+(
      Cik[16][1][0]*ri[11][1])))-rk[11][0]);
    rik[16][1] = (((Cik[16][2][1]*ri[11][2])+((Cik[16][0][1]*ri[11][0])+(
      Cik[16][1][1]*ri[11][1])))-rk[11][1]);
    rik[16][2] = (((Cik[16][2][2]*ri[11][2])+((Cik[16][0][2]*ri[11][0])+(
      Cik[16][1][2]*ri[11][1])))-rk[11][2]);
    rik[17][0] = (((Cik[17][2][0]*ri[12][2])+((Cik[17][0][0]*ri[12][0])+(
      Cik[17][1][0]*ri[12][1])))-rk[12][0]);
    rik[17][1] = (((Cik[17][2][1]*ri[12][2])+((Cik[17][0][1]*ri[12][0])+(
      Cik[17][1][1]*ri[12][1])))-rk[12][1]);
    rik[17][2] = (((Cik[17][2][2]*ri[12][2])+((Cik[17][0][2]*ri[12][0])+(
      Cik[17][1][2]*ri[12][1])))-rk[12][2]);
    rik[18][0] = (((Cik[18][2][0]*ri[13][2])+((Cik[18][0][0]*ri[13][0])+(
      Cik[18][1][0]*ri[13][1])))-rk[13][0]);
    rik[18][1] = (((Cik[18][2][1]*ri[13][2])+((Cik[18][0][1]*ri[13][0])+(
      Cik[18][1][1]*ri[13][1])))-rk[13][1]);
    rik[18][2] = (((Cik[18][2][2]*ri[13][2])+((Cik[18][0][2]*ri[13][0])+(
      Cik[18][1][2]*ri[13][1])))-rk[13][2]);
    rik[19][0] = (((Cik[19][2][0]*ri[14][2])+((Cik[19][0][0]*ri[14][0])+(
      Cik[19][1][0]*ri[14][1])))-rk[14][0]);
    rik[19][1] = (((Cik[19][2][1]*ri[14][2])+((Cik[19][0][1]*ri[14][0])+(
      Cik[19][1][1]*ri[14][1])))-rk[14][1]);
    rik[19][2] = (((Cik[19][2][2]*ri[14][2])+((Cik[19][0][2]*ri[14][0])+(
      Cik[19][1][2]*ri[14][1])))-rk[14][2]);
    rik[20][0] = (((Cik[20][2][0]*ri[15][2])+((Cik[20][0][0]*ri[15][0])+(
      Cik[20][1][0]*ri[15][1])))-rk[15][0]);
    rik[20][1] = (((Cik[20][2][1]*ri[15][2])+((Cik[20][0][1]*ri[15][0])+(
      Cik[20][1][1]*ri[15][1])))-rk[15][1]);
    rik[20][2] = (((Cik[20][2][2]*ri[15][2])+((Cik[20][0][2]*ri[15][0])+(
      Cik[20][1][2]*ri[15][1])))-rk[15][2]);
/*
Compute rnk & rnb (mass center locations in N)
*/
    rnk[0][0] = (ri[0][0]+rpp[0][0]);
    rnk[0][1] = (ri[0][1]+rpp[0][1]);
    rnk[0][2] = (ri[0][2]+rpp[0][2]);
    rnk[1][0] = (rnk[0][0]+rpp[1][0]);
    rnk[1][1] = (rnk[0][1]+rpp[1][1]);
    rnk[1][2] = (rnk[0][2]+rpp[1][2]);
    rnk[2][0] = (rnk[1][0]+rpp[2][0]);
    rnk[2][1] = (rnk[1][1]+rpp[2][1]);
    rnk[2][2] = (rnk[1][2]+rpp[2][2]);
    rnk[5][0] = (rnk[2][0]-((Cik[3][0][2]*rk[0][2])+((Cik[3][0][0]*rk[0][0])+(
      Cik[3][0][1]*rk[0][1]))));
    rnk[5][1] = (rnk[2][1]-((Cik[3][1][2]*rk[0][2])+((Cik[3][1][0]*rk[0][0])+(
      Cik[3][1][1]*rk[0][1]))));
    rnk[5][2] = (rnk[2][2]-((Cik[3][2][2]*rk[0][2])+((Cik[3][2][0]*rk[0][0])+(
      Cik[3][2][1]*rk[0][1]))));
    rnk[6][0] = ((rnk[5][0]+((Cik[3][0][2]*ri[1][2])+((Cik[3][0][0]*ri[1][0])+(
      Cik[3][0][1]*ri[1][1]))))-((cnk[6][0][2]*rk[1][2])+((cnk[6][0][0]*rk[1][0]
      )+(cnk[6][0][1]*rk[1][1]))));
    rnk[6][1] = ((rnk[5][1]+((Cik[3][1][2]*ri[1][2])+((Cik[3][1][0]*ri[1][0])+(
      Cik[3][1][1]*ri[1][1]))))-((cnk[6][1][2]*rk[1][2])+((cnk[6][1][0]*rk[1][0]
      )+(cnk[6][1][1]*rk[1][1]))));
    rnk[6][2] = ((rnk[5][2]+((Cik[3][2][2]*ri[1][2])+((Cik[3][2][0]*ri[1][0])+(
      Cik[3][2][1]*ri[1][1]))))-((cnk[6][2][2]*rk[1][2])+((cnk[6][2][0]*rk[1][0]
      )+(cnk[6][2][1]*rk[1][1]))));
    rnk[7][0] = ((rnk[6][0]+((cnk[6][0][2]*ri[2][2])+((cnk[6][0][0]*ri[2][0])+(
      cnk[6][0][1]*ri[2][1]))))-((cnk[7][0][2]*rk[2][2])+((cnk[7][0][0]*rk[2][0]
      )+(cnk[7][0][1]*rk[2][1]))));
    rnk[7][1] = ((rnk[6][1]+((cnk[6][1][2]*ri[2][2])+((cnk[6][1][0]*ri[2][0])+(
      cnk[6][1][1]*ri[2][1]))))-((cnk[7][1][2]*rk[2][2])+((cnk[7][1][0]*rk[2][0]
      )+(cnk[7][1][1]*rk[2][1]))));
    rnk[7][2] = ((rnk[6][2]+((cnk[6][2][2]*ri[2][2])+((cnk[6][2][0]*ri[2][0])+(
      cnk[6][2][1]*ri[2][1]))))-((cnk[7][2][2]*rk[2][2])+((cnk[7][2][0]*rk[2][0]
      )+(cnk[7][2][1]*rk[2][1]))));
    rnk[8][0] = ((rnk[7][0]+((cnk[7][0][2]*ri[3][2])+((cnk[7][0][0]*ri[3][0])+(
      cnk[7][0][1]*ri[3][1]))))-((cnk[8][0][2]*rk[3][2])+((cnk[8][0][0]*rk[3][0]
      )+(cnk[8][0][1]*rk[3][1]))));
    rnk[8][1] = ((rnk[7][1]+((cnk[7][1][2]*ri[3][2])+((cnk[7][1][0]*ri[3][0])+(
      cnk[7][1][1]*ri[3][1]))))-((cnk[8][1][2]*rk[3][2])+((cnk[8][1][0]*rk[3][0]
      )+(cnk[8][1][1]*rk[3][1]))));
    rnk[8][2] = ((rnk[7][2]+((cnk[7][2][2]*ri[3][2])+((cnk[7][2][0]*ri[3][0])+(
      cnk[7][2][1]*ri[3][1]))))-((cnk[8][2][2]*rk[3][2])+((cnk[8][2][0]*rk[3][0]
      )+(cnk[8][2][1]*rk[3][1]))));
    rnk[9][0] = ((rnk[5][0]+((Cik[3][0][2]*ri[4][2])+((Cik[3][0][0]*ri[4][0])+(
      Cik[3][0][1]*ri[4][1]))))-((cnk[9][0][2]*rk[4][2])+((cnk[9][0][0]*rk[4][0]
      )+(cnk[9][0][1]*rk[4][1]))));
    rnk[9][1] = ((rnk[5][1]+((Cik[3][1][2]*ri[4][2])+((Cik[3][1][0]*ri[4][0])+(
      Cik[3][1][1]*ri[4][1]))))-((cnk[9][1][2]*rk[4][2])+((cnk[9][1][0]*rk[4][0]
      )+(cnk[9][1][1]*rk[4][1]))));
    rnk[9][2] = ((rnk[5][2]+((Cik[3][2][2]*ri[4][2])+((Cik[3][2][0]*ri[4][0])+(
      Cik[3][2][1]*ri[4][1]))))-((cnk[9][2][2]*rk[4][2])+((cnk[9][2][0]*rk[4][0]
      )+(cnk[9][2][1]*rk[4][1]))));
    rnk[10][0] = ((rnk[9][0]+((cnk[9][0][2]*ri[5][2])+((cnk[9][0][0]*ri[5][0])+(
      cnk[9][0][1]*ri[5][1]))))-((cnk[10][0][2]*rk[5][2])+((cnk[10][0][0]*
      rk[5][0])+(cnk[10][0][1]*rk[5][1]))));
    rnk[10][1] = ((rnk[9][1]+((cnk[9][1][2]*ri[5][2])+((cnk[9][1][0]*ri[5][0])+(
      cnk[9][1][1]*ri[5][1]))))-((cnk[10][1][2]*rk[5][2])+((cnk[10][1][0]*
      rk[5][0])+(cnk[10][1][1]*rk[5][1]))));
    rnk[10][2] = ((rnk[9][2]+((cnk[9][2][2]*ri[5][2])+((cnk[9][2][0]*ri[5][0])+(
      cnk[9][2][1]*ri[5][1]))))-((cnk[10][2][2]*rk[5][2])+((cnk[10][2][0]*
      rk[5][0])+(cnk[10][2][1]*rk[5][1]))));
    rnk[11][0] = ((rnk[10][0]+((cnk[10][0][2]*ri[6][2])+((cnk[10][0][0]*ri[6][0]
      )+(cnk[10][0][1]*ri[6][1]))))-((cnk[11][0][2]*rk[6][2])+((cnk[11][0][0]*
      rk[6][0])+(cnk[11][0][1]*rk[6][1]))));
    rnk[11][1] = ((rnk[10][1]+((cnk[10][1][2]*ri[6][2])+((cnk[10][1][0]*ri[6][0]
      )+(cnk[10][1][1]*ri[6][1]))))-((cnk[11][1][2]*rk[6][2])+((cnk[11][1][0]*
      rk[6][0])+(cnk[11][1][1]*rk[6][1]))));
    rnk[11][2] = ((rnk[10][2]+((cnk[10][2][2]*ri[6][2])+((cnk[10][2][0]*ri[6][0]
      )+(cnk[10][2][1]*ri[6][1]))))-((cnk[11][2][2]*rk[6][2])+((cnk[11][2][0]*
      rk[6][0])+(cnk[11][2][1]*rk[6][1]))));
    rnk[12][0] = ((rnk[11][0]+((cnk[11][0][2]*ri[7][2])+((cnk[11][0][0]*ri[7][0]
      )+(cnk[11][0][1]*ri[7][1]))))-((cnk[12][0][2]*rk[7][2])+((cnk[12][0][0]*
      rk[7][0])+(cnk[12][0][1]*rk[7][1]))));
    rnk[12][1] = ((rnk[11][1]+((cnk[11][1][2]*ri[7][2])+((cnk[11][1][0]*ri[7][0]
      )+(cnk[11][1][1]*ri[7][1]))))-((cnk[12][1][2]*rk[7][2])+((cnk[12][1][0]*
      rk[7][0])+(cnk[12][1][1]*rk[7][1]))));
    rnk[12][2] = ((rnk[11][2]+((cnk[11][2][2]*ri[7][2])+((cnk[11][2][0]*ri[7][0]
      )+(cnk[11][2][1]*ri[7][1]))))-((cnk[12][2][2]*rk[7][2])+((cnk[12][2][0]*
      rk[7][0])+(cnk[12][2][1]*rk[7][1]))));
    rnk[13][0] = ((rnk[12][0]+((cnk[12][0][2]*ri[8][2])+((cnk[12][0][0]*ri[8][0]
      )+(cnk[12][0][1]*ri[8][1]))))-((cnk[13][0][2]*rk[8][2])+((cnk[13][0][0]*
      rk[8][0])+(cnk[13][0][1]*rk[8][1]))));
    rnk[13][1] = ((rnk[12][1]+((cnk[12][1][2]*ri[8][2])+((cnk[12][1][0]*ri[8][0]
      )+(cnk[12][1][1]*ri[8][1]))))-((cnk[13][1][2]*rk[8][2])+((cnk[13][1][0]*
      rk[8][0])+(cnk[13][1][1]*rk[8][1]))));
    rnk[13][2] = ((rnk[12][2]+((cnk[12][2][2]*ri[8][2])+((cnk[12][2][0]*ri[8][0]
      )+(cnk[12][2][1]*ri[8][1]))))-((cnk[13][2][2]*rk[8][2])+((cnk[13][2][0]*
      rk[8][0])+(cnk[13][2][1]*rk[8][1]))));
    rnk[14][0] = ((rnk[13][0]+((cnk[13][0][2]*ri[9][2])+((cnk[13][0][0]*ri[9][0]
      )+(cnk[13][0][1]*ri[9][1]))))-((cnk[14][0][2]*rk[9][2])+((cnk[14][0][0]*
      rk[9][0])+(cnk[14][0][1]*rk[9][1]))));
    rnk[14][1] = ((rnk[13][1]+((cnk[13][1][2]*ri[9][2])+((cnk[13][1][0]*ri[9][0]
      )+(cnk[13][1][1]*ri[9][1]))))-((cnk[14][1][2]*rk[9][2])+((cnk[14][1][0]*
      rk[9][0])+(cnk[14][1][1]*rk[9][1]))));
    rnk[14][2] = ((rnk[13][2]+((cnk[13][2][2]*ri[9][2])+((cnk[13][2][0]*ri[9][0]
      )+(cnk[13][2][1]*ri[9][1]))))-((cnk[14][2][2]*rk[9][2])+((cnk[14][2][0]*
      rk[9][0])+(cnk[14][2][1]*rk[9][1]))));
    rnk[15][0] = ((rnk[5][0]+((Cik[3][0][2]*ri[10][2])+((Cik[3][0][0]*ri[10][0])
      +(Cik[3][0][1]*ri[10][1]))))-((cnk[15][0][2]*rk[10][2])+((cnk[15][0][0]*
      rk[10][0])+(cnk[15][0][1]*rk[10][1]))));
    rnk[15][1] = ((rnk[5][1]+((Cik[3][1][2]*ri[10][2])+((Cik[3][1][0]*ri[10][0])
      +(Cik[3][1][1]*ri[10][1]))))-((cnk[15][1][2]*rk[10][2])+((cnk[15][1][0]*
      rk[10][0])+(cnk[15][1][1]*rk[10][1]))));
    rnk[15][2] = ((rnk[5][2]+((Cik[3][2][2]*ri[10][2])+((Cik[3][2][0]*ri[10][0])
      +(Cik[3][2][1]*ri[10][1]))))-((cnk[15][2][2]*rk[10][2])+((cnk[15][2][0]*
      rk[10][0])+(cnk[15][2][1]*rk[10][1]))));
    rnk[16][0] = ((rnk[15][0]+((cnk[15][0][2]*ri[11][2])+((cnk[15][0][0]*
      ri[11][0])+(cnk[15][0][1]*ri[11][1]))))-((cnk[16][0][2]*rk[11][2])+((
      cnk[16][0][0]*rk[11][0])+(cnk[16][0][1]*rk[11][1]))));
    rnk[16][1] = ((rnk[15][1]+((cnk[15][1][2]*ri[11][2])+((cnk[15][1][0]*
      ri[11][0])+(cnk[15][1][1]*ri[11][1]))))-((cnk[16][1][2]*rk[11][2])+((
      cnk[16][1][0]*rk[11][0])+(cnk[16][1][1]*rk[11][1]))));
    rnk[16][2] = ((rnk[15][2]+((cnk[15][2][2]*ri[11][2])+((cnk[15][2][0]*
      ri[11][0])+(cnk[15][2][1]*ri[11][1]))))-((cnk[16][2][2]*rk[11][2])+((
      cnk[16][2][0]*rk[11][0])+(cnk[16][2][1]*rk[11][1]))));
    rnk[17][0] = ((rnk[16][0]+((cnk[16][0][2]*ri[12][2])+((cnk[16][0][0]*
      ri[12][0])+(cnk[16][0][1]*ri[12][1]))))-((cnk[17][0][2]*rk[12][2])+((
      cnk[17][0][0]*rk[12][0])+(cnk[17][0][1]*rk[12][1]))));
    rnk[17][1] = ((rnk[16][1]+((cnk[16][1][2]*ri[12][2])+((cnk[16][1][0]*
      ri[12][0])+(cnk[16][1][1]*ri[12][1]))))-((cnk[17][1][2]*rk[12][2])+((
      cnk[17][1][0]*rk[12][0])+(cnk[17][1][1]*rk[12][1]))));
    rnk[17][2] = ((rnk[16][2]+((cnk[16][2][2]*ri[12][2])+((cnk[16][2][0]*
      ri[12][0])+(cnk[16][2][1]*ri[12][1]))))-((cnk[17][2][2]*rk[12][2])+((
      cnk[17][2][0]*rk[12][0])+(cnk[17][2][1]*rk[12][1]))));
    rnk[18][0] = ((rnk[17][0]+((cnk[17][0][2]*ri[13][2])+((cnk[17][0][0]*
      ri[13][0])+(cnk[17][0][1]*ri[13][1]))))-((cnk[18][0][2]*rk[13][2])+((
      cnk[18][0][0]*rk[13][0])+(cnk[18][0][1]*rk[13][1]))));
    rnk[18][1] = ((rnk[17][1]+((cnk[17][1][2]*ri[13][2])+((cnk[17][1][0]*
      ri[13][0])+(cnk[17][1][1]*ri[13][1]))))-((cnk[18][1][2]*rk[13][2])+((
      cnk[18][1][0]*rk[13][0])+(cnk[18][1][1]*rk[13][1]))));
    rnk[18][2] = ((rnk[17][2]+((cnk[17][2][2]*ri[13][2])+((cnk[17][2][0]*
      ri[13][0])+(cnk[17][2][1]*ri[13][1]))))-((cnk[18][2][2]*rk[13][2])+((
      cnk[18][2][0]*rk[13][0])+(cnk[18][2][1]*rk[13][1]))));
    rnk[19][0] = ((rnk[18][0]+((cnk[18][0][2]*ri[14][2])+((cnk[18][0][0]*
      ri[14][0])+(cnk[18][0][1]*ri[14][1]))))-((cnk[19][0][2]*rk[14][2])+((
      cnk[19][0][0]*rk[14][0])+(cnk[19][0][1]*rk[14][1]))));
    rnk[19][1] = ((rnk[18][1]+((cnk[18][1][2]*ri[14][2])+((cnk[18][1][0]*
      ri[14][0])+(cnk[18][1][1]*ri[14][1]))))-((cnk[19][1][2]*rk[14][2])+((
      cnk[19][1][0]*rk[14][0])+(cnk[19][1][1]*rk[14][1]))));
    rnk[19][2] = ((rnk[18][2]+((cnk[18][2][2]*ri[14][2])+((cnk[18][2][0]*
      ri[14][0])+(cnk[18][2][1]*ri[14][1]))))-((cnk[19][2][2]*rk[14][2])+((
      cnk[19][2][0]*rk[14][0])+(cnk[19][2][1]*rk[14][1]))));
    rnk[20][0] = ((rnk[19][0]+((cnk[19][0][2]*ri[15][2])+((cnk[19][0][0]*
      ri[15][0])+(cnk[19][0][1]*ri[15][1]))))-((cnk[20][0][2]*rk[15][2])+((
      cnk[20][0][0]*rk[15][0])+(cnk[20][0][1]*rk[15][1]))));
    rnk[20][1] = ((rnk[19][1]+((cnk[19][1][2]*ri[15][2])+((cnk[19][1][0]*
      ri[15][0])+(cnk[19][1][1]*ri[15][1]))))-((cnk[20][1][2]*rk[15][2])+((
      cnk[20][1][0]*rk[15][0])+(cnk[20][1][1]*rk[15][1]))));
    rnk[20][2] = ((rnk[19][2]+((cnk[19][2][2]*ri[15][2])+((cnk[19][2][0]*
      ri[15][0])+(cnk[19][2][1]*ri[15][1]))))-((cnk[20][2][2]*rk[15][2])+((
      cnk[20][2][0]*rk[15][0])+(cnk[20][2][1]*rk[15][1]))));
    rnb[0][0] = rnk[5][0];
    rnb[0][1] = rnk[5][1];
    rnb[0][2] = rnk[5][2];
    rnb[1][0] = rnk[6][0];
    rnb[1][1] = rnk[6][1];
    rnb[1][2] = rnk[6][2];
    rnb[2][0] = rnk[7][0];
    rnb[2][1] = rnk[7][1];
    rnb[2][2] = rnk[7][2];
    rnb[3][0] = rnk[8][0];
    rnb[3][1] = rnk[8][1];
    rnb[3][2] = rnk[8][2];
    rnb[4][0] = rnk[9][0];
    rnb[4][1] = rnk[9][1];
    rnb[4][2] = rnk[9][2];
    rnb[5][0] = rnk[10][0];
    rnb[5][1] = rnk[10][1];
    rnb[5][2] = rnk[10][2];
    rnb[6][0] = rnk[11][0];
    rnb[6][1] = rnk[11][1];
    rnb[6][2] = rnk[11][2];
    rnb[7][0] = rnk[12][0];
    rnb[7][1] = rnk[12][1];
    rnb[7][2] = rnk[12][2];
    rnb[8][0] = rnk[13][0];
    rnb[8][1] = rnk[13][1];
    rnb[8][2] = rnk[13][2];
    rnb[9][0] = rnk[14][0];
    rnb[9][1] = rnk[14][1];
    rnb[9][2] = rnk[14][2];
    rnb[10][0] = rnk[15][0];
    rnb[10][1] = rnk[15][1];
    rnb[10][2] = rnk[15][2];
    rnb[11][0] = rnk[16][0];
    rnb[11][1] = rnk[16][1];
    rnb[11][2] = rnk[16][2];
    rnb[12][0] = rnk[17][0];
    rnb[12][1] = rnk[17][1];
    rnb[12][2] = rnk[17][2];
    rnb[13][0] = rnk[18][0];
    rnb[13][1] = rnk[18][1];
    rnb[13][2] = rnk[18][2];
    rnb[14][0] = rnk[19][0];
    rnb[14][1] = rnk[19][1];
    rnb[14][2] = rnk[19][2];
    rnb[15][0] = rnk[20][0];
    rnb[15][1] = rnk[20][1];
    rnb[15][2] = rnk[20][2];
/*
Compute com (system mass center location in N)
*/
    com[0] = ((1./mtot)*((mk[15]*rnk[20][0])+((mk[14]*rnk[19][0])+((mk[13]*
      rnk[18][0])+((mk[12]*rnk[17][0])+((mk[11]*rnk[16][0])+((mk[10]*rnk[15][0])
      +((mk[9]*rnk[14][0])+((mk[8]*rnk[13][0])+((mk[7]*rnk[12][0])+((mk[6]*
      rnk[11][0])+((mk[5]*rnk[10][0])+((mk[4]*rnk[9][0])+((mk[3]*rnk[8][0])+((
      mk[2]*rnk[7][0])+((mk[0]*rnk[5][0])+(mk[1]*rnk[6][0])))))))))))))))));
    com[1] = ((1./mtot)*((mk[15]*rnk[20][1])+((mk[14]*rnk[19][1])+((mk[13]*
      rnk[18][1])+((mk[12]*rnk[17][1])+((mk[11]*rnk[16][1])+((mk[10]*rnk[15][1])
      +((mk[9]*rnk[14][1])+((mk[8]*rnk[13][1])+((mk[7]*rnk[12][1])+((mk[6]*
      rnk[11][1])+((mk[5]*rnk[10][1])+((mk[4]*rnk[9][1])+((mk[3]*rnk[8][1])+((
      mk[2]*rnk[7][1])+((mk[0]*rnk[5][1])+(mk[1]*rnk[6][1])))))))))))))))));
    com[2] = ((1./mtot)*((mk[15]*rnk[20][2])+((mk[14]*rnk[19][2])+((mk[13]*
      rnk[18][2])+((mk[12]*rnk[17][2])+((mk[11]*rnk[16][2])+((mk[10]*rnk[15][2])
      +((mk[9]*rnk[14][2])+((mk[8]*rnk[13][2])+((mk[7]*rnk[12][2])+((mk[6]*
      rnk[11][2])+((mk[5]*rnk[10][2])+((mk[4]*rnk[9][2])+((mk[3]*rnk[8][2])+((
      mk[2]*rnk[7][2])+((mk[0]*rnk[5][2])+(mk[1]*rnk[6][2])))))))))))))))));
/*
Compute constraint position errors
*/
    skipqs: ;
    if (uchg == 0) {
        goto skipus;
    }
/*
Velocity-related variables need to be computed
*/
    inerflg = 0;
    for (i = 0; i < 21; i++) {
        u[i] = uin[i];
    }
/*
Compute u-related auxiliary variables
*/
    Wik[6][0] = (pin[6][0]*u[6]);
    Wik[6][1] = (pin[6][1]*u[6]);
    Wik[6][2] = (pin[6][2]*u[6]);
    Wik[7][0] = (pin[7][0]*u[7]);
    Wik[7][1] = (pin[7][1]*u[7]);
    Wik[7][2] = (pin[7][2]*u[7]);
    Wik[8][0] = (pin[8][0]*u[8]);
    Wik[8][1] = (pin[8][1]*u[8]);
    Wik[8][2] = (pin[8][2]*u[8]);
    Wik[9][0] = (pin[9][0]*u[9]);
    Wik[9][1] = (pin[9][1]*u[9]);
    Wik[9][2] = (pin[9][2]*u[9]);
    Wik[10][0] = (pin[10][0]*u[10]);
    Wik[10][1] = (pin[10][1]*u[10]);
    Wik[10][2] = (pin[10][2]*u[10]);
    Wik[11][0] = (pin[11][0]*u[11]);
    Wik[11][1] = (pin[11][1]*u[11]);
    Wik[11][2] = (pin[11][2]*u[11]);
    Wik[12][0] = (pin[12][0]*u[12]);
    Wik[12][1] = (pin[12][1]*u[12]);
    Wik[12][2] = (pin[12][2]*u[12]);
    Wik[13][0] = (pin[13][0]*u[13]);
    Wik[13][1] = (pin[13][1]*u[13]);
    Wik[13][2] = (pin[13][2]*u[13]);
    Wik[14][0] = (pin[14][0]*u[14]);
    Wik[14][1] = (pin[14][1]*u[14]);
    Wik[14][2] = (pin[14][2]*u[14]);
    Wik[15][0] = (pin[15][0]*u[15]);
    Wik[15][1] = (pin[15][1]*u[15]);
    Wik[15][2] = (pin[15][2]*u[15]);
    Wik[16][0] = (pin[16][0]*u[16]);
    Wik[16][1] = (pin[16][1]*u[16]);
    Wik[16][2] = (pin[16][2]*u[16]);
    Wik[17][0] = (pin[17][0]*u[17]);
    Wik[17][1] = (pin[17][1]*u[17]);
    Wik[17][2] = (pin[17][2]*u[17]);
    Wik[18][0] = (pin[18][0]*u[18]);
    Wik[18][1] = (pin[18][1]*u[18]);
    Wik[18][2] = (pin[18][2]*u[18]);
    Wik[19][0] = (pin[19][0]*u[19]);
    Wik[19][1] = (pin[19][1]*u[19]);
    Wik[19][2] = (pin[19][2]*u[19]);
    Wik[20][0] = (pin[20][0]*u[20]);
    Wik[20][1] = (pin[20][1]*u[20]);
    Wik[20][2] = (pin[20][2]*u[20]);
    Vik[0][0] = (pin[0][0]*u[0]);
    Vik[0][1] = (pin[0][1]*u[0]);
    Vik[0][2] = (pin[0][2]*u[0]);
    Vik[1][0] = (pin[1][0]*u[1]);
    Vik[1][1] = (pin[1][1]*u[1]);
    Vik[1][2] = (pin[1][2]*u[1]);
    Vik[2][0] = (pin[2][0]*u[2]);
    Vik[2][1] = (pin[2][1]*u[2]);
    Vik[2][2] = (pin[2][2]*u[2]);
/*
Compute wk & wb (angular velocities)
*/
    wk[6][0] = (Wik[6][0]+((Cik[6][2][0]*u[5])+((Cik[6][0][0]*u[3])+(
      Cik[6][1][0]*u[4]))));
    wk[6][1] = (Wik[6][1]+((Cik[6][2][1]*u[5])+((Cik[6][0][1]*u[3])+(
      Cik[6][1][1]*u[4]))));
    wk[6][2] = (Wik[6][2]+((Cik[6][2][2]*u[5])+((Cik[6][0][2]*u[3])+(
      Cik[6][1][2]*u[4]))));
    wk[7][0] = (Wik[7][0]+((Cik[7][2][0]*wk[6][2])+((Cik[7][0][0]*wk[6][0])+(
      Cik[7][1][0]*wk[6][1]))));
    wk[7][1] = (Wik[7][1]+((Cik[7][2][1]*wk[6][2])+((Cik[7][0][1]*wk[6][0])+(
      Cik[7][1][1]*wk[6][1]))));
    wk[7][2] = (Wik[7][2]+((Cik[7][2][2]*wk[6][2])+((Cik[7][0][2]*wk[6][0])+(
      Cik[7][1][2]*wk[6][1]))));
    wk[8][0] = (Wik[8][0]+((Cik[8][2][0]*wk[7][2])+((Cik[8][0][0]*wk[7][0])+(
      Cik[8][1][0]*wk[7][1]))));
    wk[8][1] = (Wik[8][1]+((Cik[8][2][1]*wk[7][2])+((Cik[8][0][1]*wk[7][0])+(
      Cik[8][1][1]*wk[7][1]))));
    wk[8][2] = (Wik[8][2]+((Cik[8][2][2]*wk[7][2])+((Cik[8][0][2]*wk[7][0])+(
      Cik[8][1][2]*wk[7][1]))));
    wk[9][0] = (Wik[9][0]+((Cik[9][2][0]*u[5])+((Cik[9][0][0]*u[3])+(
      Cik[9][1][0]*u[4]))));
    wk[9][1] = (Wik[9][1]+((Cik[9][2][1]*u[5])+((Cik[9][0][1]*u[3])+(
      Cik[9][1][1]*u[4]))));
    wk[9][2] = (Wik[9][2]+((Cik[9][2][2]*u[5])+((Cik[9][0][2]*u[3])+(
      Cik[9][1][2]*u[4]))));
    wk[10][0] = (Wik[10][0]+((Cik[10][2][0]*wk[9][2])+((Cik[10][0][0]*wk[9][0])+
      (Cik[10][1][0]*wk[9][1]))));
    wk[10][1] = (Wik[10][1]+((Cik[10][2][1]*wk[9][2])+((Cik[10][0][1]*wk[9][0])+
      (Cik[10][1][1]*wk[9][1]))));
    wk[10][2] = (Wik[10][2]+((Cik[10][2][2]*wk[9][2])+((Cik[10][0][2]*wk[9][0])+
      (Cik[10][1][2]*wk[9][1]))));
    wk[11][0] = (Wik[11][0]+((Cik[11][2][0]*wk[10][2])+((Cik[11][0][0]*wk[10][0]
      )+(Cik[11][1][0]*wk[10][1]))));
    wk[11][1] = (Wik[11][1]+((Cik[11][2][1]*wk[10][2])+((Cik[11][0][1]*wk[10][0]
      )+(Cik[11][1][1]*wk[10][1]))));
    wk[11][2] = (Wik[11][2]+((Cik[11][2][2]*wk[10][2])+((Cik[11][0][2]*wk[10][0]
      )+(Cik[11][1][2]*wk[10][1]))));
    wk[12][0] = (Wik[12][0]+((Cik[12][2][0]*wk[11][2])+((Cik[12][0][0]*wk[11][0]
      )+(Cik[12][1][0]*wk[11][1]))));
    wk[12][1] = (Wik[12][1]+((Cik[12][2][1]*wk[11][2])+((Cik[12][0][1]*wk[11][0]
      )+(Cik[12][1][1]*wk[11][1]))));
    wk[12][2] = (Wik[12][2]+((Cik[12][2][2]*wk[11][2])+((Cik[12][0][2]*wk[11][0]
      )+(Cik[12][1][2]*wk[11][1]))));
    wk[13][0] = (Wik[13][0]+((Cik[13][2][0]*wk[12][2])+((Cik[13][0][0]*wk[12][0]
      )+(Cik[13][1][0]*wk[12][1]))));
    wk[13][1] = (Wik[13][1]+((Cik[13][2][1]*wk[12][2])+((Cik[13][0][1]*wk[12][0]
      )+(Cik[13][1][1]*wk[12][1]))));
    wk[13][2] = (Wik[13][2]+((Cik[13][2][2]*wk[12][2])+((Cik[13][0][2]*wk[12][0]
      )+(Cik[13][1][2]*wk[12][1]))));
    wk[14][0] = (Wik[14][0]+((Cik[14][2][0]*wk[13][2])+((Cik[14][0][0]*wk[13][0]
      )+(Cik[14][1][0]*wk[13][1]))));
    wk[14][1] = (Wik[14][1]+((Cik[14][2][1]*wk[13][2])+((Cik[14][0][1]*wk[13][0]
      )+(Cik[14][1][1]*wk[13][1]))));
    wk[14][2] = (Wik[14][2]+((Cik[14][2][2]*wk[13][2])+((Cik[14][0][2]*wk[13][0]
      )+(Cik[14][1][2]*wk[13][1]))));
    wk[15][0] = (Wik[15][0]+((Cik[15][2][0]*u[5])+((Cik[15][0][0]*u[3])+(
      Cik[15][1][0]*u[4]))));
    wk[15][1] = (Wik[15][1]+((Cik[15][2][1]*u[5])+((Cik[15][0][1]*u[3])+(
      Cik[15][1][1]*u[4]))));
    wk[15][2] = (Wik[15][2]+((Cik[15][2][2]*u[5])+((Cik[15][0][2]*u[3])+(
      Cik[15][1][2]*u[4]))));
    wk[16][0] = (Wik[16][0]+((Cik[16][2][0]*wk[15][2])+((Cik[16][0][0]*wk[15][0]
      )+(Cik[16][1][0]*wk[15][1]))));
    wk[16][1] = (Wik[16][1]+((Cik[16][2][1]*wk[15][2])+((Cik[16][0][1]*wk[15][0]
      )+(Cik[16][1][1]*wk[15][1]))));
    wk[16][2] = (Wik[16][2]+((Cik[16][2][2]*wk[15][2])+((Cik[16][0][2]*wk[15][0]
      )+(Cik[16][1][2]*wk[15][1]))));
    wk[17][0] = (Wik[17][0]+((Cik[17][2][0]*wk[16][2])+((Cik[17][0][0]*wk[16][0]
      )+(Cik[17][1][0]*wk[16][1]))));
    wk[17][1] = (Wik[17][1]+((Cik[17][2][1]*wk[16][2])+((Cik[17][0][1]*wk[16][0]
      )+(Cik[17][1][1]*wk[16][1]))));
    wk[17][2] = (Wik[17][2]+((Cik[17][2][2]*wk[16][2])+((Cik[17][0][2]*wk[16][0]
      )+(Cik[17][1][2]*wk[16][1]))));
    wk[18][0] = (Wik[18][0]+((Cik[18][2][0]*wk[17][2])+((Cik[18][0][0]*wk[17][0]
      )+(Cik[18][1][0]*wk[17][1]))));
    wk[18][1] = (Wik[18][1]+((Cik[18][2][1]*wk[17][2])+((Cik[18][0][1]*wk[17][0]
      )+(Cik[18][1][1]*wk[17][1]))));
    wk[18][2] = (Wik[18][2]+((Cik[18][2][2]*wk[17][2])+((Cik[18][0][2]*wk[17][0]
      )+(Cik[18][1][2]*wk[17][1]))));
    wk[19][0] = (Wik[19][0]+((Cik[19][2][0]*wk[18][2])+((Cik[19][0][0]*wk[18][0]
      )+(Cik[19][1][0]*wk[18][1]))));
    wk[19][1] = (Wik[19][1]+((Cik[19][2][1]*wk[18][2])+((Cik[19][0][1]*wk[18][0]
      )+(Cik[19][1][1]*wk[18][1]))));
    wk[19][2] = (Wik[19][2]+((Cik[19][2][2]*wk[18][2])+((Cik[19][0][2]*wk[18][0]
      )+(Cik[19][1][2]*wk[18][1]))));
    wk[20][0] = (Wik[20][0]+((Cik[20][2][0]*wk[19][2])+((Cik[20][0][0]*wk[19][0]
      )+(Cik[20][1][0]*wk[19][1]))));
    wk[20][1] = (Wik[20][1]+((Cik[20][2][1]*wk[19][2])+((Cik[20][0][1]*wk[19][0]
      )+(Cik[20][1][1]*wk[19][1]))));
    wk[20][2] = (Wik[20][2]+((Cik[20][2][2]*wk[19][2])+((Cik[20][0][2]*wk[19][0]
      )+(Cik[20][1][2]*wk[19][1]))));
    wb[0][0] = u[3];
    wb[0][1] = u[4];
    wb[0][2] = u[5];
    wb[1][0] = wk[6][0];
    wb[1][1] = wk[6][1];
    wb[1][2] = wk[6][2];
    wb[2][0] = wk[7][0];
    wb[2][1] = wk[7][1];
    wb[2][2] = wk[7][2];
    wb[3][0] = wk[8][0];
    wb[3][1] = wk[8][1];
    wb[3][2] = wk[8][2];
    wb[4][0] = wk[9][0];
    wb[4][1] = wk[9][1];
    wb[4][2] = wk[9][2];
    wb[5][0] = wk[10][0];
    wb[5][1] = wk[10][1];
    wb[5][2] = wk[10][2];
    wb[6][0] = wk[11][0];
    wb[6][1] = wk[11][1];
    wb[6][2] = wk[11][2];
    wb[7][0] = wk[12][0];
    wb[7][1] = wk[12][1];
    wb[7][2] = wk[12][2];
    wb[8][0] = wk[13][0];
    wb[8][1] = wk[13][1];
    wb[8][2] = wk[13][2];
    wb[9][0] = wk[14][0];
    wb[9][1] = wk[14][1];
    wb[9][2] = wk[14][2];
    wb[10][0] = wk[15][0];
    wb[10][1] = wk[15][1];
    wb[10][2] = wk[15][2];
    wb[11][0] = wk[16][0];
    wb[11][1] = wk[16][1];
    wb[11][2] = wk[16][2];
    wb[12][0] = wk[17][0];
    wb[12][1] = wk[17][1];
    wb[12][2] = wk[17][2];
    wb[13][0] = wk[18][0];
    wb[13][1] = wk[18][1];
    wb[13][2] = wk[18][2];
    wb[14][0] = wk[19][0];
    wb[14][1] = wk[19][1];
    wb[14][2] = wk[19][2];
    wb[15][0] = wk[20][0];
    wb[15][1] = wk[20][1];
    wb[15][2] = wk[20][2];
/*
Compute auxiliary variables involving wk
*/
    Wirk[6][0] = ((ri[1][2]*u[4])-(ri[1][1]*u[5]));
    Wirk[6][1] = ((ri[1][0]*u[5])-(ri[1][2]*u[3]));
    Wirk[6][2] = ((ri[1][1]*u[3])-(ri[1][0]*u[4]));
    Wirk[7][0] = ((ri[2][2]*wk[6][1])-(ri[2][1]*wk[6][2]));
    Wirk[7][1] = ((ri[2][0]*wk[6][2])-(ri[2][2]*wk[6][0]));
    Wirk[7][2] = ((ri[2][1]*wk[6][0])-(ri[2][0]*wk[6][1]));
    Wirk[8][0] = ((ri[3][2]*wk[7][1])-(ri[3][1]*wk[7][2]));
    Wirk[8][1] = ((ri[3][0]*wk[7][2])-(ri[3][2]*wk[7][0]));
    Wirk[8][2] = ((ri[3][1]*wk[7][0])-(ri[3][0]*wk[7][1]));
    Wirk[9][0] = ((ri[4][2]*u[4])-(ri[4][1]*u[5]));
    Wirk[9][1] = ((ri[4][0]*u[5])-(ri[4][2]*u[3]));
    Wirk[9][2] = ((ri[4][1]*u[3])-(ri[4][0]*u[4]));
    Wirk[10][0] = ((ri[5][2]*wk[9][1])-(ri[5][1]*wk[9][2]));
    Wirk[10][1] = ((ri[5][0]*wk[9][2])-(ri[5][2]*wk[9][0]));
    Wirk[10][2] = ((ri[5][1]*wk[9][0])-(ri[5][0]*wk[9][1]));
    Wirk[11][0] = ((ri[6][2]*wk[10][1])-(ri[6][1]*wk[10][2]));
    Wirk[11][1] = ((ri[6][0]*wk[10][2])-(ri[6][2]*wk[10][0]));
    Wirk[11][2] = ((ri[6][1]*wk[10][0])-(ri[6][0]*wk[10][1]));
    Wirk[12][0] = ((ri[7][2]*wk[11][1])-(ri[7][1]*wk[11][2]));
    Wirk[12][1] = ((ri[7][0]*wk[11][2])-(ri[7][2]*wk[11][0]));
    Wirk[12][2] = ((ri[7][1]*wk[11][0])-(ri[7][0]*wk[11][1]));
    Wirk[13][0] = ((ri[8][2]*wk[12][1])-(ri[8][1]*wk[12][2]));
    Wirk[13][1] = ((ri[8][0]*wk[12][2])-(ri[8][2]*wk[12][0]));
    Wirk[13][2] = ((ri[8][1]*wk[12][0])-(ri[8][0]*wk[12][1]));
    Wirk[14][0] = ((ri[9][2]*wk[13][1])-(ri[9][1]*wk[13][2]));
    Wirk[14][1] = ((ri[9][0]*wk[13][2])-(ri[9][2]*wk[13][0]));
    Wirk[14][2] = ((ri[9][1]*wk[13][0])-(ri[9][0]*wk[13][1]));
    Wirk[15][0] = ((ri[10][2]*u[4])-(ri[10][1]*u[5]));
    Wirk[15][1] = ((ri[10][0]*u[5])-(ri[10][2]*u[3]));
    Wirk[15][2] = ((ri[10][1]*u[3])-(ri[10][0]*u[4]));
    Wirk[16][0] = ((ri[11][2]*wk[15][1])-(ri[11][1]*wk[15][2]));
    Wirk[16][1] = ((ri[11][0]*wk[15][2])-(ri[11][2]*wk[15][0]));
    Wirk[16][2] = ((ri[11][1]*wk[15][0])-(ri[11][0]*wk[15][1]));
    Wirk[17][0] = ((ri[12][2]*wk[16][1])-(ri[12][1]*wk[16][2]));
    Wirk[17][1] = ((ri[12][0]*wk[16][2])-(ri[12][2]*wk[16][0]));
    Wirk[17][2] = ((ri[12][1]*wk[16][0])-(ri[12][0]*wk[16][1]));
    Wirk[18][0] = ((ri[13][2]*wk[17][1])-(ri[13][1]*wk[17][2]));
    Wirk[18][1] = ((ri[13][0]*wk[17][2])-(ri[13][2]*wk[17][0]));
    Wirk[18][2] = ((ri[13][1]*wk[17][0])-(ri[13][0]*wk[17][1]));
    Wirk[19][0] = ((ri[14][2]*wk[18][1])-(ri[14][1]*wk[18][2]));
    Wirk[19][1] = ((ri[14][0]*wk[18][2])-(ri[14][2]*wk[18][0]));
    Wirk[19][2] = ((ri[14][1]*wk[18][0])-(ri[14][0]*wk[18][1]));
    Wirk[20][0] = ((ri[15][2]*wk[19][1])-(ri[15][1]*wk[19][2]));
    Wirk[20][1] = ((ri[15][0]*wk[19][2])-(ri[15][2]*wk[19][0]));
    Wirk[20][2] = ((ri[15][1]*wk[19][0])-(ri[15][0]*wk[19][1]));
    Wkrpk[5][0] = ((rk[0][1]*u[5])-(rk[0][2]*u[4]));
    Wkrpk[5][1] = ((rk[0][2]*u[3])-(rk[0][0]*u[5]));
    Wkrpk[5][2] = ((rk[0][0]*u[4])-(rk[0][1]*u[3]));
    Wkrpk[6][0] = ((rk[1][1]*wk[6][2])-(rk[1][2]*wk[6][1]));
    Wkrpk[6][1] = ((rk[1][2]*wk[6][0])-(rk[1][0]*wk[6][2]));
    Wkrpk[6][2] = ((rk[1][0]*wk[6][1])-(rk[1][1]*wk[6][0]));
    Wkrpk[7][0] = ((rk[2][1]*wk[7][2])-(rk[2][2]*wk[7][1]));
    Wkrpk[7][1] = ((rk[2][2]*wk[7][0])-(rk[2][0]*wk[7][2]));
    Wkrpk[7][2] = ((rk[2][0]*wk[7][1])-(rk[2][1]*wk[7][0]));
    Wkrpk[8][0] = ((rk[3][1]*wk[8][2])-(rk[3][2]*wk[8][1]));
    Wkrpk[8][1] = ((rk[3][2]*wk[8][0])-(rk[3][0]*wk[8][2]));
    Wkrpk[8][2] = ((rk[3][0]*wk[8][1])-(rk[3][1]*wk[8][0]));
    Wkrpk[9][0] = ((rk[4][1]*wk[9][2])-(rk[4][2]*wk[9][1]));
    Wkrpk[9][1] = ((rk[4][2]*wk[9][0])-(rk[4][0]*wk[9][2]));
    Wkrpk[9][2] = ((rk[4][0]*wk[9][1])-(rk[4][1]*wk[9][0]));
    Wkrpk[10][0] = ((rk[5][1]*wk[10][2])-(rk[5][2]*wk[10][1]));
    Wkrpk[10][1] = ((rk[5][2]*wk[10][0])-(rk[5][0]*wk[10][2]));
    Wkrpk[10][2] = ((rk[5][0]*wk[10][1])-(rk[5][1]*wk[10][0]));
    Wkrpk[11][0] = ((rk[6][1]*wk[11][2])-(rk[6][2]*wk[11][1]));
    Wkrpk[11][1] = ((rk[6][2]*wk[11][0])-(rk[6][0]*wk[11][2]));
    Wkrpk[11][2] = ((rk[6][0]*wk[11][1])-(rk[6][1]*wk[11][0]));
    Wkrpk[12][0] = ((rk[7][1]*wk[12][2])-(rk[7][2]*wk[12][1]));
    Wkrpk[12][1] = ((rk[7][2]*wk[12][0])-(rk[7][0]*wk[12][2]));
    Wkrpk[12][2] = ((rk[7][0]*wk[12][1])-(rk[7][1]*wk[12][0]));
    Wkrpk[13][0] = ((rk[8][1]*wk[13][2])-(rk[8][2]*wk[13][1]));
    Wkrpk[13][1] = ((rk[8][2]*wk[13][0])-(rk[8][0]*wk[13][2]));
    Wkrpk[13][2] = ((rk[8][0]*wk[13][1])-(rk[8][1]*wk[13][0]));
    Wkrpk[14][0] = ((rk[9][1]*wk[14][2])-(rk[9][2]*wk[14][1]));
    Wkrpk[14][1] = ((rk[9][2]*wk[14][0])-(rk[9][0]*wk[14][2]));
    Wkrpk[14][2] = ((rk[9][0]*wk[14][1])-(rk[9][1]*wk[14][0]));
    Wkrpk[15][0] = ((rk[10][1]*wk[15][2])-(rk[10][2]*wk[15][1]));
    Wkrpk[15][1] = ((rk[10][2]*wk[15][0])-(rk[10][0]*wk[15][2]));
    Wkrpk[15][2] = ((rk[10][0]*wk[15][1])-(rk[10][1]*wk[15][0]));
    Wkrpk[16][0] = ((rk[11][1]*wk[16][2])-(rk[11][2]*wk[16][1]));
    Wkrpk[16][1] = ((rk[11][2]*wk[16][0])-(rk[11][0]*wk[16][2]));
    Wkrpk[16][2] = ((rk[11][0]*wk[16][1])-(rk[11][1]*wk[16][0]));
    Wkrpk[17][0] = ((rk[12][1]*wk[17][2])-(rk[12][2]*wk[17][1]));
    Wkrpk[17][1] = ((rk[12][2]*wk[17][0])-(rk[12][0]*wk[17][2]));
    Wkrpk[17][2] = ((rk[12][0]*wk[17][1])-(rk[12][1]*wk[17][0]));
    Wkrpk[18][0] = ((rk[13][1]*wk[18][2])-(rk[13][2]*wk[18][1]));
    Wkrpk[18][1] = ((rk[13][2]*wk[18][0])-(rk[13][0]*wk[18][2]));
    Wkrpk[18][2] = ((rk[13][0]*wk[18][1])-(rk[13][1]*wk[18][0]));
    Wkrpk[19][0] = ((rk[14][1]*wk[19][2])-(rk[14][2]*wk[19][1]));
    Wkrpk[19][1] = ((rk[14][2]*wk[19][0])-(rk[14][0]*wk[19][2]));
    Wkrpk[19][2] = ((rk[14][0]*wk[19][1])-(rk[14][1]*wk[19][0]));
    Wkrpk[20][0] = ((rk[15][1]*wk[20][2])-(rk[15][2]*wk[20][1]));
    Wkrpk[20][1] = ((rk[15][2]*wk[20][0])-(rk[15][0]*wk[20][2]));
    Wkrpk[20][2] = ((rk[15][0]*wk[20][1])-(rk[15][1]*wk[20][0]));
    IkWk[5][0] = ((ik[0][0][2]*u[5])+((ik[0][0][0]*u[3])+(ik[0][0][1]*u[4])));
    IkWk[5][1] = ((ik[0][1][2]*u[5])+((ik[0][1][0]*u[3])+(ik[0][1][1]*u[4])));
    IkWk[5][2] = ((ik[0][2][2]*u[5])+((ik[0][2][0]*u[3])+(ik[0][2][1]*u[4])));
    WkIkWk[5][0] = ((IkWk[5][2]*u[4])-(IkWk[5][1]*u[5]));
    WkIkWk[5][1] = ((IkWk[5][0]*u[5])-(IkWk[5][2]*u[3]));
    WkIkWk[5][2] = ((IkWk[5][1]*u[3])-(IkWk[5][0]*u[4]));
    IkWk[6][0] = ((ik[1][0][2]*wk[6][2])+((ik[1][0][0]*wk[6][0])+(ik[1][0][1]*
      wk[6][1])));
    IkWk[6][1] = ((ik[1][1][2]*wk[6][2])+((ik[1][1][0]*wk[6][0])+(ik[1][1][1]*
      wk[6][1])));
    IkWk[6][2] = ((ik[1][2][2]*wk[6][2])+((ik[1][2][0]*wk[6][0])+(ik[1][2][1]*
      wk[6][1])));
    WkIkWk[6][0] = ((IkWk[6][2]*wk[6][1])-(IkWk[6][1]*wk[6][2]));
    WkIkWk[6][1] = ((IkWk[6][0]*wk[6][2])-(IkWk[6][2]*wk[6][0]));
    WkIkWk[6][2] = ((IkWk[6][1]*wk[6][0])-(IkWk[6][0]*wk[6][1]));
    IkWk[7][0] = ((ik[2][0][2]*wk[7][2])+((ik[2][0][0]*wk[7][0])+(ik[2][0][1]*
      wk[7][1])));
    IkWk[7][1] = ((ik[2][1][2]*wk[7][2])+((ik[2][1][0]*wk[7][0])+(ik[2][1][1]*
      wk[7][1])));
    IkWk[7][2] = ((ik[2][2][2]*wk[7][2])+((ik[2][2][0]*wk[7][0])+(ik[2][2][1]*
      wk[7][1])));
    WkIkWk[7][0] = ((IkWk[7][2]*wk[7][1])-(IkWk[7][1]*wk[7][2]));
    WkIkWk[7][1] = ((IkWk[7][0]*wk[7][2])-(IkWk[7][2]*wk[7][0]));
    WkIkWk[7][2] = ((IkWk[7][1]*wk[7][0])-(IkWk[7][0]*wk[7][1]));
    IkWk[8][0] = ((ik[3][0][2]*wk[8][2])+((ik[3][0][0]*wk[8][0])+(ik[3][0][1]*
      wk[8][1])));
    IkWk[8][1] = ((ik[3][1][2]*wk[8][2])+((ik[3][1][0]*wk[8][0])+(ik[3][1][1]*
      wk[8][1])));
    IkWk[8][2] = ((ik[3][2][2]*wk[8][2])+((ik[3][2][0]*wk[8][0])+(ik[3][2][1]*
      wk[8][1])));
    WkIkWk[8][0] = ((IkWk[8][2]*wk[8][1])-(IkWk[8][1]*wk[8][2]));
    WkIkWk[8][1] = ((IkWk[8][0]*wk[8][2])-(IkWk[8][2]*wk[8][0]));
    WkIkWk[8][2] = ((IkWk[8][1]*wk[8][0])-(IkWk[8][0]*wk[8][1]));
    IkWk[9][0] = ((ik[4][0][2]*wk[9][2])+((ik[4][0][0]*wk[9][0])+(ik[4][0][1]*
      wk[9][1])));
    IkWk[9][1] = ((ik[4][1][2]*wk[9][2])+((ik[4][1][0]*wk[9][0])+(ik[4][1][1]*
      wk[9][1])));
    IkWk[9][2] = ((ik[4][2][2]*wk[9][2])+((ik[4][2][0]*wk[9][0])+(ik[4][2][1]*
      wk[9][1])));
    WkIkWk[9][0] = ((IkWk[9][2]*wk[9][1])-(IkWk[9][1]*wk[9][2]));
    WkIkWk[9][1] = ((IkWk[9][0]*wk[9][2])-(IkWk[9][2]*wk[9][0]));
    WkIkWk[9][2] = ((IkWk[9][1]*wk[9][0])-(IkWk[9][0]*wk[9][1]));
    IkWk[10][0] = ((ik[5][0][2]*wk[10][2])+((ik[5][0][0]*wk[10][0])+(ik[5][0][1]
      *wk[10][1])));
    IkWk[10][1] = ((ik[5][1][2]*wk[10][2])+((ik[5][1][0]*wk[10][0])+(ik[5][1][1]
      *wk[10][1])));
    IkWk[10][2] = ((ik[5][2][2]*wk[10][2])+((ik[5][2][0]*wk[10][0])+(ik[5][2][1]
      *wk[10][1])));
    WkIkWk[10][0] = ((IkWk[10][2]*wk[10][1])-(IkWk[10][1]*wk[10][2]));
    WkIkWk[10][1] = ((IkWk[10][0]*wk[10][2])-(IkWk[10][2]*wk[10][0]));
    WkIkWk[10][2] = ((IkWk[10][1]*wk[10][0])-(IkWk[10][0]*wk[10][1]));
    IkWk[11][0] = ((ik[6][0][2]*wk[11][2])+((ik[6][0][0]*wk[11][0])+(ik[6][0][1]
      *wk[11][1])));
    IkWk[11][1] = ((ik[6][1][2]*wk[11][2])+((ik[6][1][0]*wk[11][0])+(ik[6][1][1]
      *wk[11][1])));
    IkWk[11][2] = ((ik[6][2][2]*wk[11][2])+((ik[6][2][0]*wk[11][0])+(ik[6][2][1]
      *wk[11][1])));
    WkIkWk[11][0] = ((IkWk[11][2]*wk[11][1])-(IkWk[11][1]*wk[11][2]));
    WkIkWk[11][1] = ((IkWk[11][0]*wk[11][2])-(IkWk[11][2]*wk[11][0]));
    WkIkWk[11][2] = ((IkWk[11][1]*wk[11][0])-(IkWk[11][0]*wk[11][1]));
    IkWk[12][0] = ((ik[7][0][2]*wk[12][2])+((ik[7][0][0]*wk[12][0])+(ik[7][0][1]
      *wk[12][1])));
    IkWk[12][1] = ((ik[7][1][2]*wk[12][2])+((ik[7][1][0]*wk[12][0])+(ik[7][1][1]
      *wk[12][1])));
    IkWk[12][2] = ((ik[7][2][2]*wk[12][2])+((ik[7][2][0]*wk[12][0])+(ik[7][2][1]
      *wk[12][1])));
    WkIkWk[12][0] = ((IkWk[12][2]*wk[12][1])-(IkWk[12][1]*wk[12][2]));
    WkIkWk[12][1] = ((IkWk[12][0]*wk[12][2])-(IkWk[12][2]*wk[12][0]));
    WkIkWk[12][2] = ((IkWk[12][1]*wk[12][0])-(IkWk[12][0]*wk[12][1]));
    IkWk[13][0] = ((ik[8][0][2]*wk[13][2])+((ik[8][0][0]*wk[13][0])+(ik[8][0][1]
      *wk[13][1])));
    IkWk[13][1] = ((ik[8][1][2]*wk[13][2])+((ik[8][1][0]*wk[13][0])+(ik[8][1][1]
      *wk[13][1])));
    IkWk[13][2] = ((ik[8][2][2]*wk[13][2])+((ik[8][2][0]*wk[13][0])+(ik[8][2][1]
      *wk[13][1])));
    WkIkWk[13][0] = ((IkWk[13][2]*wk[13][1])-(IkWk[13][1]*wk[13][2]));
    WkIkWk[13][1] = ((IkWk[13][0]*wk[13][2])-(IkWk[13][2]*wk[13][0]));
    WkIkWk[13][2] = ((IkWk[13][1]*wk[13][0])-(IkWk[13][0]*wk[13][1]));
    IkWk[14][0] = ((ik[9][0][2]*wk[14][2])+((ik[9][0][0]*wk[14][0])+(ik[9][0][1]
      *wk[14][1])));
    IkWk[14][1] = ((ik[9][1][2]*wk[14][2])+((ik[9][1][0]*wk[14][0])+(ik[9][1][1]
      *wk[14][1])));
    IkWk[14][2] = ((ik[9][2][2]*wk[14][2])+((ik[9][2][0]*wk[14][0])+(ik[9][2][1]
      *wk[14][1])));
    WkIkWk[14][0] = ((IkWk[14][2]*wk[14][1])-(IkWk[14][1]*wk[14][2]));
    WkIkWk[14][1] = ((IkWk[14][0]*wk[14][2])-(IkWk[14][2]*wk[14][0]));
    WkIkWk[14][2] = ((IkWk[14][1]*wk[14][0])-(IkWk[14][0]*wk[14][1]));
    IkWk[15][0] = ((ik[10][0][2]*wk[15][2])+((ik[10][0][0]*wk[15][0])+(
      ik[10][0][1]*wk[15][1])));
    IkWk[15][1] = ((ik[10][1][2]*wk[15][2])+((ik[10][1][0]*wk[15][0])+(
      ik[10][1][1]*wk[15][1])));
    IkWk[15][2] = ((ik[10][2][2]*wk[15][2])+((ik[10][2][0]*wk[15][0])+(
      ik[10][2][1]*wk[15][1])));
    WkIkWk[15][0] = ((IkWk[15][2]*wk[15][1])-(IkWk[15][1]*wk[15][2]));
    WkIkWk[15][1] = ((IkWk[15][0]*wk[15][2])-(IkWk[15][2]*wk[15][0]));
    WkIkWk[15][2] = ((IkWk[15][1]*wk[15][0])-(IkWk[15][0]*wk[15][1]));
    IkWk[16][0] = ((ik[11][0][2]*wk[16][2])+((ik[11][0][0]*wk[16][0])+(
      ik[11][0][1]*wk[16][1])));
    IkWk[16][1] = ((ik[11][1][2]*wk[16][2])+((ik[11][1][0]*wk[16][0])+(
      ik[11][1][1]*wk[16][1])));
    IkWk[16][2] = ((ik[11][2][2]*wk[16][2])+((ik[11][2][0]*wk[16][0])+(
      ik[11][2][1]*wk[16][1])));
    WkIkWk[16][0] = ((IkWk[16][2]*wk[16][1])-(IkWk[16][1]*wk[16][2]));
    WkIkWk[16][1] = ((IkWk[16][0]*wk[16][2])-(IkWk[16][2]*wk[16][0]));
    WkIkWk[16][2] = ((IkWk[16][1]*wk[16][0])-(IkWk[16][0]*wk[16][1]));
    IkWk[17][0] = ((ik[12][0][2]*wk[17][2])+((ik[12][0][0]*wk[17][0])+(
      ik[12][0][1]*wk[17][1])));
    IkWk[17][1] = ((ik[12][1][2]*wk[17][2])+((ik[12][1][0]*wk[17][0])+(
      ik[12][1][1]*wk[17][1])));
    IkWk[17][2] = ((ik[12][2][2]*wk[17][2])+((ik[12][2][0]*wk[17][0])+(
      ik[12][2][1]*wk[17][1])));
    WkIkWk[17][0] = ((IkWk[17][2]*wk[17][1])-(IkWk[17][1]*wk[17][2]));
    WkIkWk[17][1] = ((IkWk[17][0]*wk[17][2])-(IkWk[17][2]*wk[17][0]));
    WkIkWk[17][2] = ((IkWk[17][1]*wk[17][0])-(IkWk[17][0]*wk[17][1]));
    IkWk[18][0] = ((ik[13][0][2]*wk[18][2])+((ik[13][0][0]*wk[18][0])+(
      ik[13][0][1]*wk[18][1])));
    IkWk[18][1] = ((ik[13][1][2]*wk[18][2])+((ik[13][1][0]*wk[18][0])+(
      ik[13][1][1]*wk[18][1])));
    IkWk[18][2] = ((ik[13][2][2]*wk[18][2])+((ik[13][2][0]*wk[18][0])+(
      ik[13][2][1]*wk[18][1])));
    WkIkWk[18][0] = ((IkWk[18][2]*wk[18][1])-(IkWk[18][1]*wk[18][2]));
    WkIkWk[18][1] = ((IkWk[18][0]*wk[18][2])-(IkWk[18][2]*wk[18][0]));
    WkIkWk[18][2] = ((IkWk[18][1]*wk[18][0])-(IkWk[18][0]*wk[18][1]));
    IkWk[19][0] = ((ik[14][0][2]*wk[19][2])+((ik[14][0][0]*wk[19][0])+(
      ik[14][0][1]*wk[19][1])));
    IkWk[19][1] = ((ik[14][1][2]*wk[19][2])+((ik[14][1][0]*wk[19][0])+(
      ik[14][1][1]*wk[19][1])));
    IkWk[19][2] = ((ik[14][2][2]*wk[19][2])+((ik[14][2][0]*wk[19][0])+(
      ik[14][2][1]*wk[19][1])));
    WkIkWk[19][0] = ((IkWk[19][2]*wk[19][1])-(IkWk[19][1]*wk[19][2]));
    WkIkWk[19][1] = ((IkWk[19][0]*wk[19][2])-(IkWk[19][2]*wk[19][0]));
    WkIkWk[19][2] = ((IkWk[19][1]*wk[19][0])-(IkWk[19][0]*wk[19][1]));
    IkWk[20][0] = ((ik[15][0][2]*wk[20][2])+((ik[15][0][0]*wk[20][0])+(
      ik[15][0][1]*wk[20][1])));
    IkWk[20][1] = ((ik[15][1][2]*wk[20][2])+((ik[15][1][0]*wk[20][0])+(
      ik[15][1][1]*wk[20][1])));
    IkWk[20][2] = ((ik[15][2][2]*wk[20][2])+((ik[15][2][0]*wk[20][0])+(
      ik[15][2][1]*wk[20][1])));
    WkIkWk[20][0] = ((IkWk[20][2]*wk[20][1])-(IkWk[20][1]*wk[20][2]));
    WkIkWk[20][1] = ((IkWk[20][0]*wk[20][2])-(IkWk[20][2]*wk[20][0]));
    WkIkWk[20][2] = ((IkWk[20][1]*wk[20][0])-(IkWk[20][0]*wk[20][1]));
/*
Compute temporaries for use in SDRHS
*/
    w0w0[0] = (u[3]*u[3]);
    w0w0[1] = (wk[6][0]*wk[6][0]);
    w0w0[2] = (wk[7][0]*wk[7][0]);
    w0w0[3] = (wk[8][0]*wk[8][0]);
    w0w0[4] = (wk[9][0]*wk[9][0]);
    w0w0[5] = (wk[10][0]*wk[10][0]);
    w0w0[6] = (wk[11][0]*wk[11][0]);
    w0w0[7] = (wk[12][0]*wk[12][0]);
    w0w0[8] = (wk[13][0]*wk[13][0]);
    w0w0[9] = (wk[14][0]*wk[14][0]);
    w0w0[10] = (wk[15][0]*wk[15][0]);
    w0w0[11] = (wk[16][0]*wk[16][0]);
    w0w0[12] = (wk[17][0]*wk[17][0]);
    w0w0[13] = (wk[18][0]*wk[18][0]);
    w0w0[14] = (wk[19][0]*wk[19][0]);
    w0w0[15] = (wk[20][0]*wk[20][0]);
    w1w1[0] = (u[4]*u[4]);
    w1w1[1] = (wk[6][1]*wk[6][1]);
    w1w1[2] = (wk[7][1]*wk[7][1]);
    w1w1[3] = (wk[8][1]*wk[8][1]);
    w1w1[4] = (wk[9][1]*wk[9][1]);
    w1w1[5] = (wk[10][1]*wk[10][1]);
    w1w1[6] = (wk[11][1]*wk[11][1]);
    w1w1[7] = (wk[12][1]*wk[12][1]);
    w1w1[8] = (wk[13][1]*wk[13][1]);
    w1w1[9] = (wk[14][1]*wk[14][1]);
    w1w1[10] = (wk[15][1]*wk[15][1]);
    w1w1[11] = (wk[16][1]*wk[16][1]);
    w1w1[12] = (wk[17][1]*wk[17][1]);
    w1w1[13] = (wk[18][1]*wk[18][1]);
    w1w1[14] = (wk[19][1]*wk[19][1]);
    w1w1[15] = (wk[20][1]*wk[20][1]);
    w2w2[0] = (u[5]*u[5]);
    w2w2[1] = (wk[6][2]*wk[6][2]);
    w2w2[2] = (wk[7][2]*wk[7][2]);
    w2w2[3] = (wk[8][2]*wk[8][2]);
    w2w2[4] = (wk[9][2]*wk[9][2]);
    w2w2[5] = (wk[10][2]*wk[10][2]);
    w2w2[6] = (wk[11][2]*wk[11][2]);
    w2w2[7] = (wk[12][2]*wk[12][2]);
    w2w2[8] = (wk[13][2]*wk[13][2]);
    w2w2[9] = (wk[14][2]*wk[14][2]);
    w2w2[10] = (wk[15][2]*wk[15][2]);
    w2w2[11] = (wk[16][2]*wk[16][2]);
    w2w2[12] = (wk[17][2]*wk[17][2]);
    w2w2[13] = (wk[18][2]*wk[18][2]);
    w2w2[14] = (wk[19][2]*wk[19][2]);
    w2w2[15] = (wk[20][2]*wk[20][2]);
    w0w1[0] = (u[3]*u[4]);
    w0w1[1] = (wk[6][0]*wk[6][1]);
    w0w1[2] = (wk[7][0]*wk[7][1]);
    w0w1[3] = (wk[8][0]*wk[8][1]);
    w0w1[4] = (wk[9][0]*wk[9][1]);
    w0w1[5] = (wk[10][0]*wk[10][1]);
    w0w1[6] = (wk[11][0]*wk[11][1]);
    w0w1[7] = (wk[12][0]*wk[12][1]);
    w0w1[8] = (wk[13][0]*wk[13][1]);
    w0w1[9] = (wk[14][0]*wk[14][1]);
    w0w1[10] = (wk[15][0]*wk[15][1]);
    w0w1[11] = (wk[16][0]*wk[16][1]);
    w0w1[12] = (wk[17][0]*wk[17][1]);
    w0w1[13] = (wk[18][0]*wk[18][1]);
    w0w1[14] = (wk[19][0]*wk[19][1]);
    w0w1[15] = (wk[20][0]*wk[20][1]);
    w0w2[0] = (u[3]*u[5]);
    w0w2[1] = (wk[6][0]*wk[6][2]);
    w0w2[2] = (wk[7][0]*wk[7][2]);
    w0w2[3] = (wk[8][0]*wk[8][2]);
    w0w2[4] = (wk[9][0]*wk[9][2]);
    w0w2[5] = (wk[10][0]*wk[10][2]);
    w0w2[6] = (wk[11][0]*wk[11][2]);
    w0w2[7] = (wk[12][0]*wk[12][2]);
    w0w2[8] = (wk[13][0]*wk[13][2]);
    w0w2[9] = (wk[14][0]*wk[14][2]);
    w0w2[10] = (wk[15][0]*wk[15][2]);
    w0w2[11] = (wk[16][0]*wk[16][2]);
    w0w2[12] = (wk[17][0]*wk[17][2]);
    w0w2[13] = (wk[18][0]*wk[18][2]);
    w0w2[14] = (wk[19][0]*wk[19][2]);
    w0w2[15] = (wk[20][0]*wk[20][2]);
    w1w2[0] = (u[4]*u[5]);
    w1w2[1] = (wk[6][1]*wk[6][2]);
    w1w2[2] = (wk[7][1]*wk[7][2]);
    w1w2[3] = (wk[8][1]*wk[8][2]);
    w1w2[4] = (wk[9][1]*wk[9][2]);
    w1w2[5] = (wk[10][1]*wk[10][2]);
    w1w2[6] = (wk[11][1]*wk[11][2]);
    w1w2[7] = (wk[12][1]*wk[12][2]);
    w1w2[8] = (wk[13][1]*wk[13][2]);
    w1w2[9] = (wk[14][1]*wk[14][2]);
    w1w2[10] = (wk[15][1]*wk[15][2]);
    w1w2[11] = (wk[16][1]*wk[16][2]);
    w1w2[12] = (wk[17][1]*wk[17][2]);
    w1w2[13] = (wk[18][1]*wk[18][2]);
    w1w2[14] = (wk[19][1]*wk[19][2]);
    w1w2[15] = (wk[20][1]*wk[20][2]);
    w00w11[0] = -(w0w0[0]+w1w1[0]);
    w00w11[1] = -(w0w0[1]+w1w1[1]);
    w00w11[2] = -(w0w0[2]+w1w1[2]);
    w00w11[3] = -(w0w0[3]+w1w1[3]);
    w00w11[4] = -(w0w0[4]+w1w1[4]);
    w00w11[5] = -(w0w0[5]+w1w1[5]);
    w00w11[6] = -(w0w0[6]+w1w1[6]);
    w00w11[7] = -(w0w0[7]+w1w1[7]);
    w00w11[8] = -(w0w0[8]+w1w1[8]);
    w00w11[9] = -(w0w0[9]+w1w1[9]);
    w00w11[10] = -(w0w0[10]+w1w1[10]);
    w00w11[11] = -(w0w0[11]+w1w1[11]);
    w00w11[12] = -(w0w0[12]+w1w1[12]);
    w00w11[13] = -(w0w0[13]+w1w1[13]);
    w00w11[14] = -(w0w0[14]+w1w1[14]);
    w00w11[15] = -(w0w0[15]+w1w1[15]);
    w00w22[0] = -(w0w0[0]+w2w2[0]);
    w00w22[1] = -(w0w0[1]+w2w2[1]);
    w00w22[2] = -(w0w0[2]+w2w2[2]);
    w00w22[3] = -(w0w0[3]+w2w2[3]);
    w00w22[4] = -(w0w0[4]+w2w2[4]);
    w00w22[5] = -(w0w0[5]+w2w2[5]);
    w00w22[6] = -(w0w0[6]+w2w2[6]);
    w00w22[7] = -(w0w0[7]+w2w2[7]);
    w00w22[8] = -(w0w0[8]+w2w2[8]);
    w00w22[9] = -(w0w0[9]+w2w2[9]);
    w00w22[10] = -(w0w0[10]+w2w2[10]);
    w00w22[11] = -(w0w0[11]+w2w2[11]);
    w00w22[12] = -(w0w0[12]+w2w2[12]);
    w00w22[13] = -(w0w0[13]+w2w2[13]);
    w00w22[14] = -(w0w0[14]+w2w2[14]);
    w00w22[15] = -(w0w0[15]+w2w2[15]);
    w11w22[0] = -(w1w1[0]+w2w2[0]);
    w11w22[1] = -(w1w1[1]+w2w2[1]);
    w11w22[2] = -(w1w1[2]+w2w2[2]);
    w11w22[3] = -(w1w1[3]+w2w2[3]);
    w11w22[4] = -(w1w1[4]+w2w2[4]);
    w11w22[5] = -(w1w1[5]+w2w2[5]);
    w11w22[6] = -(w1w1[6]+w2w2[6]);
    w11w22[7] = -(w1w1[7]+w2w2[7]);
    w11w22[8] = -(w1w1[8]+w2w2[8]);
    w11w22[9] = -(w1w1[9]+w2w2[9]);
    w11w22[10] = -(w1w1[10]+w2w2[10]);
    w11w22[11] = -(w1w1[11]+w2w2[11]);
    w11w22[12] = -(w1w1[12]+w2w2[12]);
    w11w22[13] = -(w1w1[13]+w2w2[13]);
    w11w22[14] = -(w1w1[14]+w2w2[14]);
    w11w22[15] = -(w1w1[15]+w2w2[15]);
/*
Compute vnk & vnb (mass center linear velocities in N)
*/
    vnk[1][0] = (Vik[0][0]+Vik[1][0]);
    vnk[1][1] = (Vik[0][1]+Vik[1][1]);
    vnk[1][2] = (Vik[0][2]+Vik[1][2]);
    vnk[2][0] = (Vik[2][0]+vnk[1][0]);
    vnk[2][1] = (Vik[2][1]+vnk[1][1]);
    vnk[2][2] = (Vik[2][2]+vnk[1][2]);
    vnk[5][0] = (vnk[2][0]+((Cik[3][0][2]*Wkrpk[5][2])+((Cik[3][0][0]*
      Wkrpk[5][0])+(Cik[3][0][1]*Wkrpk[5][1]))));
    vnk[5][1] = (vnk[2][1]+((Cik[3][1][2]*Wkrpk[5][2])+((Cik[3][1][0]*
      Wkrpk[5][0])+(Cik[3][1][1]*Wkrpk[5][1]))));
    vnk[5][2] = (vnk[2][2]+((Cik[3][2][2]*Wkrpk[5][2])+((Cik[3][2][0]*
      Wkrpk[5][0])+(Cik[3][2][1]*Wkrpk[5][1]))));
    vnk[6][0] = ((vnk[5][0]+((Cik[3][0][2]*Wirk[6][2])+((Cik[3][0][0]*Wirk[6][0]
      )+(Cik[3][0][1]*Wirk[6][1]))))+((cnk[6][0][2]*Wkrpk[6][2])+((cnk[6][0][0]*
      Wkrpk[6][0])+(cnk[6][0][1]*Wkrpk[6][1]))));
    vnk[6][1] = ((vnk[5][1]+((Cik[3][1][2]*Wirk[6][2])+((Cik[3][1][0]*Wirk[6][0]
      )+(Cik[3][1][1]*Wirk[6][1]))))+((cnk[6][1][2]*Wkrpk[6][2])+((cnk[6][1][0]*
      Wkrpk[6][0])+(cnk[6][1][1]*Wkrpk[6][1]))));
    vnk[6][2] = ((vnk[5][2]+((Cik[3][2][2]*Wirk[6][2])+((Cik[3][2][0]*Wirk[6][0]
      )+(Cik[3][2][1]*Wirk[6][1]))))+((cnk[6][2][2]*Wkrpk[6][2])+((cnk[6][2][0]*
      Wkrpk[6][0])+(cnk[6][2][1]*Wkrpk[6][1]))));
    vnk[7][0] = ((vnk[6][0]+((cnk[6][0][2]*Wirk[7][2])+((cnk[6][0][0]*Wirk[7][0]
      )+(cnk[6][0][1]*Wirk[7][1]))))+((cnk[7][0][2]*Wkrpk[7][2])+((cnk[7][0][0]*
      Wkrpk[7][0])+(cnk[7][0][1]*Wkrpk[7][1]))));
    vnk[7][1] = ((vnk[6][1]+((cnk[6][1][2]*Wirk[7][2])+((cnk[6][1][0]*Wirk[7][0]
      )+(cnk[6][1][1]*Wirk[7][1]))))+((cnk[7][1][2]*Wkrpk[7][2])+((cnk[7][1][0]*
      Wkrpk[7][0])+(cnk[7][1][1]*Wkrpk[7][1]))));
    vnk[7][2] = ((vnk[6][2]+((cnk[6][2][2]*Wirk[7][2])+((cnk[6][2][0]*Wirk[7][0]
      )+(cnk[6][2][1]*Wirk[7][1]))))+((cnk[7][2][2]*Wkrpk[7][2])+((cnk[7][2][0]*
      Wkrpk[7][0])+(cnk[7][2][1]*Wkrpk[7][1]))));
    vnk[8][0] = ((vnk[7][0]+((cnk[7][0][2]*Wirk[8][2])+((cnk[7][0][0]*Wirk[8][0]
      )+(cnk[7][0][1]*Wirk[8][1]))))+((cnk[8][0][2]*Wkrpk[8][2])+((cnk[8][0][0]*
      Wkrpk[8][0])+(cnk[8][0][1]*Wkrpk[8][1]))));
    vnk[8][1] = ((vnk[7][1]+((cnk[7][1][2]*Wirk[8][2])+((cnk[7][1][0]*Wirk[8][0]
      )+(cnk[7][1][1]*Wirk[8][1]))))+((cnk[8][1][2]*Wkrpk[8][2])+((cnk[8][1][0]*
      Wkrpk[8][0])+(cnk[8][1][1]*Wkrpk[8][1]))));
    vnk[8][2] = ((vnk[7][2]+((cnk[7][2][2]*Wirk[8][2])+((cnk[7][2][0]*Wirk[8][0]
      )+(cnk[7][2][1]*Wirk[8][1]))))+((cnk[8][2][2]*Wkrpk[8][2])+((cnk[8][2][0]*
      Wkrpk[8][0])+(cnk[8][2][1]*Wkrpk[8][1]))));
    vnk[9][0] = ((vnk[5][0]+((Cik[3][0][2]*Wirk[9][2])+((Cik[3][0][0]*Wirk[9][0]
      )+(Cik[3][0][1]*Wirk[9][1]))))+((cnk[9][0][2]*Wkrpk[9][2])+((cnk[9][0][0]*
      Wkrpk[9][0])+(cnk[9][0][1]*Wkrpk[9][1]))));
    vnk[9][1] = ((vnk[5][1]+((Cik[3][1][2]*Wirk[9][2])+((Cik[3][1][0]*Wirk[9][0]
      )+(Cik[3][1][1]*Wirk[9][1]))))+((cnk[9][1][2]*Wkrpk[9][2])+((cnk[9][1][0]*
      Wkrpk[9][0])+(cnk[9][1][1]*Wkrpk[9][1]))));
    vnk[9][2] = ((vnk[5][2]+((Cik[3][2][2]*Wirk[9][2])+((Cik[3][2][0]*Wirk[9][0]
      )+(Cik[3][2][1]*Wirk[9][1]))))+((cnk[9][2][2]*Wkrpk[9][2])+((cnk[9][2][0]*
      Wkrpk[9][0])+(cnk[9][2][1]*Wkrpk[9][1]))));
    vnk[10][0] = ((vnk[9][0]+((cnk[9][0][2]*Wirk[10][2])+((cnk[9][0][0]*
      Wirk[10][0])+(cnk[9][0][1]*Wirk[10][1]))))+((cnk[10][0][2]*Wkrpk[10][2])+(
      (cnk[10][0][0]*Wkrpk[10][0])+(cnk[10][0][1]*Wkrpk[10][1]))));
    vnk[10][1] = ((vnk[9][1]+((cnk[9][1][2]*Wirk[10][2])+((cnk[9][1][0]*
      Wirk[10][0])+(cnk[9][1][1]*Wirk[10][1]))))+((cnk[10][1][2]*Wkrpk[10][2])+(
      (cnk[10][1][0]*Wkrpk[10][0])+(cnk[10][1][1]*Wkrpk[10][1]))));
    vnk[10][2] = ((vnk[9][2]+((cnk[9][2][2]*Wirk[10][2])+((cnk[9][2][0]*
      Wirk[10][0])+(cnk[9][2][1]*Wirk[10][1]))))+((cnk[10][2][2]*Wkrpk[10][2])+(
      (cnk[10][2][0]*Wkrpk[10][0])+(cnk[10][2][1]*Wkrpk[10][1]))));
    vnk[11][0] = ((vnk[10][0]+((cnk[10][0][2]*Wirk[11][2])+((cnk[10][0][0]*
      Wirk[11][0])+(cnk[10][0][1]*Wirk[11][1]))))+((cnk[11][0][2]*Wkrpk[11][2])+
      ((cnk[11][0][0]*Wkrpk[11][0])+(cnk[11][0][1]*Wkrpk[11][1]))));
    vnk[11][1] = ((vnk[10][1]+((cnk[10][1][2]*Wirk[11][2])+((cnk[10][1][0]*
      Wirk[11][0])+(cnk[10][1][1]*Wirk[11][1]))))+((cnk[11][1][2]*Wkrpk[11][2])+
      ((cnk[11][1][0]*Wkrpk[11][0])+(cnk[11][1][1]*Wkrpk[11][1]))));
    vnk[11][2] = ((vnk[10][2]+((cnk[10][2][2]*Wirk[11][2])+((cnk[10][2][0]*
      Wirk[11][0])+(cnk[10][2][1]*Wirk[11][1]))))+((cnk[11][2][2]*Wkrpk[11][2])+
      ((cnk[11][2][0]*Wkrpk[11][0])+(cnk[11][2][1]*Wkrpk[11][1]))));
    vnk[12][0] = ((vnk[11][0]+((cnk[11][0][2]*Wirk[12][2])+((cnk[11][0][0]*
      Wirk[12][0])+(cnk[11][0][1]*Wirk[12][1]))))+((cnk[12][0][2]*Wkrpk[12][2])+
      ((cnk[12][0][0]*Wkrpk[12][0])+(cnk[12][0][1]*Wkrpk[12][1]))));
    vnk[12][1] = ((vnk[11][1]+((cnk[11][1][2]*Wirk[12][2])+((cnk[11][1][0]*
      Wirk[12][0])+(cnk[11][1][1]*Wirk[12][1]))))+((cnk[12][1][2]*Wkrpk[12][2])+
      ((cnk[12][1][0]*Wkrpk[12][0])+(cnk[12][1][1]*Wkrpk[12][1]))));
    vnk[12][2] = ((vnk[11][2]+((cnk[11][2][2]*Wirk[12][2])+((cnk[11][2][0]*
      Wirk[12][0])+(cnk[11][2][1]*Wirk[12][1]))))+((cnk[12][2][2]*Wkrpk[12][2])+
      ((cnk[12][2][0]*Wkrpk[12][0])+(cnk[12][2][1]*Wkrpk[12][1]))));
    vnk[13][0] = ((vnk[12][0]+((cnk[12][0][2]*Wirk[13][2])+((cnk[12][0][0]*
      Wirk[13][0])+(cnk[12][0][1]*Wirk[13][1]))))+((cnk[13][0][2]*Wkrpk[13][2])+
      ((cnk[13][0][0]*Wkrpk[13][0])+(cnk[13][0][1]*Wkrpk[13][1]))));
    vnk[13][1] = ((vnk[12][1]+((cnk[12][1][2]*Wirk[13][2])+((cnk[12][1][0]*
      Wirk[13][0])+(cnk[12][1][1]*Wirk[13][1]))))+((cnk[13][1][2]*Wkrpk[13][2])+
      ((cnk[13][1][0]*Wkrpk[13][0])+(cnk[13][1][1]*Wkrpk[13][1]))));
    vnk[13][2] = ((vnk[12][2]+((cnk[12][2][2]*Wirk[13][2])+((cnk[12][2][0]*
      Wirk[13][0])+(cnk[12][2][1]*Wirk[13][1]))))+((cnk[13][2][2]*Wkrpk[13][2])+
      ((cnk[13][2][0]*Wkrpk[13][0])+(cnk[13][2][1]*Wkrpk[13][1]))));
    vnk[14][0] = ((vnk[13][0]+((cnk[13][0][2]*Wirk[14][2])+((cnk[13][0][0]*
      Wirk[14][0])+(cnk[13][0][1]*Wirk[14][1]))))+((cnk[14][0][2]*Wkrpk[14][2])+
      ((cnk[14][0][0]*Wkrpk[14][0])+(cnk[14][0][1]*Wkrpk[14][1]))));
    vnk[14][1] = ((vnk[13][1]+((cnk[13][1][2]*Wirk[14][2])+((cnk[13][1][0]*
      Wirk[14][0])+(cnk[13][1][1]*Wirk[14][1]))))+((cnk[14][1][2]*Wkrpk[14][2])+
      ((cnk[14][1][0]*Wkrpk[14][0])+(cnk[14][1][1]*Wkrpk[14][1]))));
    vnk[14][2] = ((vnk[13][2]+((cnk[13][2][2]*Wirk[14][2])+((cnk[13][2][0]*
      Wirk[14][0])+(cnk[13][2][1]*Wirk[14][1]))))+((cnk[14][2][2]*Wkrpk[14][2])+
      ((cnk[14][2][0]*Wkrpk[14][0])+(cnk[14][2][1]*Wkrpk[14][1]))));
    vnk[15][0] = ((vnk[5][0]+((Cik[3][0][2]*Wirk[15][2])+((Cik[3][0][0]*
      Wirk[15][0])+(Cik[3][0][1]*Wirk[15][1]))))+((cnk[15][0][2]*Wkrpk[15][2])+(
      (cnk[15][0][0]*Wkrpk[15][0])+(cnk[15][0][1]*Wkrpk[15][1]))));
    vnk[15][1] = ((vnk[5][1]+((Cik[3][1][2]*Wirk[15][2])+((Cik[3][1][0]*
      Wirk[15][0])+(Cik[3][1][1]*Wirk[15][1]))))+((cnk[15][1][2]*Wkrpk[15][2])+(
      (cnk[15][1][0]*Wkrpk[15][0])+(cnk[15][1][1]*Wkrpk[15][1]))));
    vnk[15][2] = ((vnk[5][2]+((Cik[3][2][2]*Wirk[15][2])+((Cik[3][2][0]*
      Wirk[15][0])+(Cik[3][2][1]*Wirk[15][1]))))+((cnk[15][2][2]*Wkrpk[15][2])+(
      (cnk[15][2][0]*Wkrpk[15][0])+(cnk[15][2][1]*Wkrpk[15][1]))));
    vnk[16][0] = ((vnk[15][0]+((cnk[15][0][2]*Wirk[16][2])+((cnk[15][0][0]*
      Wirk[16][0])+(cnk[15][0][1]*Wirk[16][1]))))+((cnk[16][0][2]*Wkrpk[16][2])+
      ((cnk[16][0][0]*Wkrpk[16][0])+(cnk[16][0][1]*Wkrpk[16][1]))));
    vnk[16][1] = ((vnk[15][1]+((cnk[15][1][2]*Wirk[16][2])+((cnk[15][1][0]*
      Wirk[16][0])+(cnk[15][1][1]*Wirk[16][1]))))+((cnk[16][1][2]*Wkrpk[16][2])+
      ((cnk[16][1][0]*Wkrpk[16][0])+(cnk[16][1][1]*Wkrpk[16][1]))));
    vnk[16][2] = ((vnk[15][2]+((cnk[15][2][2]*Wirk[16][2])+((cnk[15][2][0]*
      Wirk[16][0])+(cnk[15][2][1]*Wirk[16][1]))))+((cnk[16][2][2]*Wkrpk[16][2])+
      ((cnk[16][2][0]*Wkrpk[16][0])+(cnk[16][2][1]*Wkrpk[16][1]))));
    vnk[17][0] = ((vnk[16][0]+((cnk[16][0][2]*Wirk[17][2])+((cnk[16][0][0]*
      Wirk[17][0])+(cnk[16][0][1]*Wirk[17][1]))))+((cnk[17][0][2]*Wkrpk[17][2])+
      ((cnk[17][0][0]*Wkrpk[17][0])+(cnk[17][0][1]*Wkrpk[17][1]))));
    vnk[17][1] = ((vnk[16][1]+((cnk[16][1][2]*Wirk[17][2])+((cnk[16][1][0]*
      Wirk[17][0])+(cnk[16][1][1]*Wirk[17][1]))))+((cnk[17][1][2]*Wkrpk[17][2])+
      ((cnk[17][1][0]*Wkrpk[17][0])+(cnk[17][1][1]*Wkrpk[17][1]))));
    vnk[17][2] = ((vnk[16][2]+((cnk[16][2][2]*Wirk[17][2])+((cnk[16][2][0]*
      Wirk[17][0])+(cnk[16][2][1]*Wirk[17][1]))))+((cnk[17][2][2]*Wkrpk[17][2])+
      ((cnk[17][2][0]*Wkrpk[17][0])+(cnk[17][2][1]*Wkrpk[17][1]))));
    vnk[18][0] = ((vnk[17][0]+((cnk[17][0][2]*Wirk[18][2])+((cnk[17][0][0]*
      Wirk[18][0])+(cnk[17][0][1]*Wirk[18][1]))))+((cnk[18][0][2]*Wkrpk[18][2])+
      ((cnk[18][0][0]*Wkrpk[18][0])+(cnk[18][0][1]*Wkrpk[18][1]))));
    vnk[18][1] = ((vnk[17][1]+((cnk[17][1][2]*Wirk[18][2])+((cnk[17][1][0]*
      Wirk[18][0])+(cnk[17][1][1]*Wirk[18][1]))))+((cnk[18][1][2]*Wkrpk[18][2])+
      ((cnk[18][1][0]*Wkrpk[18][0])+(cnk[18][1][1]*Wkrpk[18][1]))));
    vnk[18][2] = ((vnk[17][2]+((cnk[17][2][2]*Wirk[18][2])+((cnk[17][2][0]*
      Wirk[18][0])+(cnk[17][2][1]*Wirk[18][1]))))+((cnk[18][2][2]*Wkrpk[18][2])+
      ((cnk[18][2][0]*Wkrpk[18][0])+(cnk[18][2][1]*Wkrpk[18][1]))));
    vnk[19][0] = ((vnk[18][0]+((cnk[18][0][2]*Wirk[19][2])+((cnk[18][0][0]*
      Wirk[19][0])+(cnk[18][0][1]*Wirk[19][1]))))+((cnk[19][0][2]*Wkrpk[19][2])+
      ((cnk[19][0][0]*Wkrpk[19][0])+(cnk[19][0][1]*Wkrpk[19][1]))));
    vnk[19][1] = ((vnk[18][1]+((cnk[18][1][2]*Wirk[19][2])+((cnk[18][1][0]*
      Wirk[19][0])+(cnk[18][1][1]*Wirk[19][1]))))+((cnk[19][1][2]*Wkrpk[19][2])+
      ((cnk[19][1][0]*Wkrpk[19][0])+(cnk[19][1][1]*Wkrpk[19][1]))));
    vnk[19][2] = ((vnk[18][2]+((cnk[18][2][2]*Wirk[19][2])+((cnk[18][2][0]*
      Wirk[19][0])+(cnk[18][2][1]*Wirk[19][1]))))+((cnk[19][2][2]*Wkrpk[19][2])+
      ((cnk[19][2][0]*Wkrpk[19][0])+(cnk[19][2][1]*Wkrpk[19][1]))));
    vnk[20][0] = ((vnk[19][0]+((cnk[19][0][2]*Wirk[20][2])+((cnk[19][0][0]*
      Wirk[20][0])+(cnk[19][0][1]*Wirk[20][1]))))+((cnk[20][0][2]*Wkrpk[20][2])+
      ((cnk[20][0][0]*Wkrpk[20][0])+(cnk[20][0][1]*Wkrpk[20][1]))));
    vnk[20][1] = ((vnk[19][1]+((cnk[19][1][2]*Wirk[20][2])+((cnk[19][1][0]*
      Wirk[20][0])+(cnk[19][1][1]*Wirk[20][1]))))+((cnk[20][1][2]*Wkrpk[20][2])+
      ((cnk[20][1][0]*Wkrpk[20][0])+(cnk[20][1][1]*Wkrpk[20][1]))));
    vnk[20][2] = ((vnk[19][2]+((cnk[19][2][2]*Wirk[20][2])+((cnk[19][2][0]*
      Wirk[20][0])+(cnk[19][2][1]*Wirk[20][1]))))+((cnk[20][2][2]*Wkrpk[20][2])+
      ((cnk[20][2][0]*Wkrpk[20][0])+(cnk[20][2][1]*Wkrpk[20][1]))));
    vnb[0][0] = vnk[5][0];
    vnb[0][1] = vnk[5][1];
    vnb[0][2] = vnk[5][2];
    vnb[1][0] = vnk[6][0];
    vnb[1][1] = vnk[6][1];
    vnb[1][2] = vnk[6][2];
    vnb[2][0] = vnk[7][0];
    vnb[2][1] = vnk[7][1];
    vnb[2][2] = vnk[7][2];
    vnb[3][0] = vnk[8][0];
    vnb[3][1] = vnk[8][1];
    vnb[3][2] = vnk[8][2];
    vnb[4][0] = vnk[9][0];
    vnb[4][1] = vnk[9][1];
    vnb[4][2] = vnk[9][2];
    vnb[5][0] = vnk[10][0];
    vnb[5][1] = vnk[10][1];
    vnb[5][2] = vnk[10][2];
    vnb[6][0] = vnk[11][0];
    vnb[6][1] = vnk[11][1];
    vnb[6][2] = vnk[11][2];
    vnb[7][0] = vnk[12][0];
    vnb[7][1] = vnk[12][1];
    vnb[7][2] = vnk[12][2];
    vnb[8][0] = vnk[13][0];
    vnb[8][1] = vnk[13][1];
    vnb[8][2] = vnk[13][2];
    vnb[9][0] = vnk[14][0];
    vnb[9][1] = vnk[14][1];
    vnb[9][2] = vnk[14][2];
    vnb[10][0] = vnk[15][0];
    vnb[10][1] = vnk[15][1];
    vnb[10][2] = vnk[15][2];
    vnb[11][0] = vnk[16][0];
    vnb[11][1] = vnk[16][1];
    vnb[11][2] = vnk[16][2];
    vnb[12][0] = vnk[17][0];
    vnb[12][1] = vnk[17][1];
    vnb[12][2] = vnk[17][2];
    vnb[13][0] = vnk[18][0];
    vnb[13][1] = vnk[18][1];
    vnb[13][2] = vnk[18][2];
    vnb[14][0] = vnk[19][0];
    vnb[14][1] = vnk[19][1];
    vnb[14][2] = vnk[19][2];
    vnb[15][0] = vnk[20][0];
    vnb[15][1] = vnk[20][1];
    vnb[15][2] = vnk[20][2];
/*
Compute qdot (kinematical equations)
*/
    qdot[0] = u[0];
    qdot[1] = u[1];
    qdot[2] = u[2];
    qdot[3] = (.5*((q[21]*u[3])+((q[4]*u[5])-(q[5]*u[4]))));
    qdot[4] = (.5*((q[5]*u[3])+((q[21]*u[4])-(q[3]*u[5]))));
    qdot[5] = (.5*(((q[3]*u[4])+(q[21]*u[5]))-(q[4]*u[3])));
    qdot[21] = -(.5*((q[3]*u[3])+((q[4]*u[4])+(q[5]*u[5]))));
    if (stabvel  !=  0.) {
        ee = ((q[21]*q[21])+((q[5]*q[5])+((q[3]*q[3])+(q[4]*q[4]))));
        stab = ((stabvel*(1.-ee))/ee);
        qdot[3] = (qdot[3]+(q[3]*stab));
        qdot[4] = (qdot[4]+(q[4]*stab));
        qdot[5] = (qdot[5]+(q[5]*stab));
        qdot[21] = (qdot[21]+(q[21]*stab));
    }
    qdot[6] = u[6];
    qdot[7] = u[7];
    qdot[8] = u[8];
    qdot[9] = u[9];
    qdot[10] = u[10];
    qdot[11] = u[11];
    qdot[12] = u[12];
    qdot[13] = u[13];
    qdot[14] = u[14];
    qdot[15] = u[15];
    qdot[16] = u[16];
    qdot[17] = u[17];
    qdot[18] = u[18];
    qdot[19] = u[19];
    qdot[20] = u[20];
/*
Compute constraint velocity errors
*/
    skipus: ;
/*
Initialize applied forces and torques to zero
*/
    for (i = 0; i < 16; i++) {
        for (j = 0; j < 3; j++) {
            ufk[i][j] = 0.;
            utk[i][j] = 0.;
        }
    }
    for (i = 0; i < 21; i++) {
        utau[i] = 0.;
    }
    ltauflg = 0;
    fs0flg = 0;
/*
Initialize prescribed motions
*/
    uacc[0] = 0.;
    uvel[0] = u[0];
    upos[0] = q[0];
    uacc[1] = 0.;
    uvel[1] = u[1];
    upos[1] = q[1];
    uacc[2] = 0.;
    uvel[2] = u[2];
    upos[2] = q[2];
    uacc[3] = 0.;
    uvel[3] = u[3];
    upos[3] = q[3];
    uacc[4] = 0.;
    uvel[4] = u[4];
    upos[4] = q[4];
    uacc[5] = 0.;
    uvel[5] = u[5];
    upos[5] = q[5];
    uacc[6] = 0.;
    uvel[6] = u[6];
    upos[6] = q[6];
    uacc[7] = 0.;
    uvel[7] = u[7];
    upos[7] = q[7];
    uacc[8] = 0.;
    uvel[8] = u[8];
    upos[8] = q[8];
    uacc[9] = 0.;
    uvel[9] = u[9];
    upos[9] = q[9];
    uacc[10] = 0.;
    uvel[10] = u[10];
    upos[10] = q[10];
    uacc[11] = 0.;
    uvel[11] = u[11];
    upos[11] = q[11];
    uacc[12] = 0.;
    uvel[12] = u[12];
    upos[12] = q[12];
    uacc[13] = 0.;
    uvel[13] = u[13];
    upos[13] = q[13];
    uacc[14] = 0.;
    uvel[14] = u[14];
    upos[14] = q[14];
    uacc[15] = 0.;
    uvel[15] = u[15];
    upos[15] = q[15];
    uacc[16] = 0.;
    uvel[16] = u[16];
    upos[16] = q[16];
    uacc[17] = 0.;
    uvel[17] = u[17];
    upos[17] = q[17];
    uacc[18] = 0.;
    uvel[18] = u[18];
    upos[18] = q[18];
    uacc[19] = 0.;
    uvel[19] = u[19];
    upos[19] = q[19];
    uacc[20] = 0.;
    uvel[20] = u[20];
    upos[20] = q[20];
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain 1892 adds/subtracts/negates
                   2560 multiplies
                      4 divides
                   1472 assignments
*/
}

void sdqdot(double oqdot[22])
{
/*
Return position coordinate derivatives for tree joints.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(63,23);
        return;
    }
    for (i = 0; i <= 21; i++) {
        oqdot[i] = qdot[i];
    }
}

void sdu2qdot(double uin[21],
    double oqdot[22])
{
/*
Convert velocities to qdots.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(64,23);
        return;
    }
    for (i = 0; i <= 20; i++) {
        oqdot[i] = uin[i];
    }
    oqdot[3] = (.5*((q[21]*uin[3])+((q[4]*uin[5])-(q[5]*uin[4]))));
    oqdot[4] = (.5*((q[5]*uin[3])+((q[21]*uin[4])-(q[3]*uin[5]))));
    oqdot[5] = (.5*(((q[3]*uin[4])+(q[21]*uin[5]))-(q[4]*uin[3])));
    oqdot[21] = -(.5*((q[3]*uin[3])+((q[4]*uin[4])+(q[5]*uin[5]))));
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    9 adds/subtracts/negates
                     16 multiplies
                      0 divides
                     25 assignments
*/
}

void sdpsstate(double lqin[1])
{

    if (roustate != 2) {
        sdseterr(9,23);
        return;
    }
}

void sddovpk(void)
{

    if (vpkflg == 0) {
/*
Compute Wpk (partial angular velocities)
*/
        Wpk[3][3][0] = 1.;
        Wpk[3][4][0] = 1.;
        Wpk[3][5][0] = 1.;
        Wpk[3][6][0] = Cik[6][0][0];
        Wpk[3][6][1] = Cik[6][0][1];
        Wpk[3][6][2] = Cik[6][0][2];
        Wpk[3][7][0] = ((Cik[6][0][2]*Cik[7][2][0])+((Cik[6][0][0]*Cik[7][0][0])
          +(Cik[6][0][1]*Cik[7][1][0])));
        Wpk[3][7][1] = ((Cik[6][0][2]*Cik[7][2][1])+((Cik[6][0][0]*Cik[7][0][1])
          +(Cik[6][0][1]*Cik[7][1][1])));
        Wpk[3][7][2] = ((Cik[6][0][2]*Cik[7][2][2])+((Cik[6][0][0]*Cik[7][0][2])
          +(Cik[6][0][1]*Cik[7][1][2])));
        Wpk[3][8][0] = ((Cik[8][2][0]*Wpk[3][7][2])+((Cik[8][0][0]*Wpk[3][7][0])
          +(Cik[8][1][0]*Wpk[3][7][1])));
        Wpk[3][8][1] = ((Cik[8][2][1]*Wpk[3][7][2])+((Cik[8][0][1]*Wpk[3][7][0])
          +(Cik[8][1][1]*Wpk[3][7][1])));
        Wpk[3][8][2] = ((Cik[8][2][2]*Wpk[3][7][2])+((Cik[8][0][2]*Wpk[3][7][0])
          +(Cik[8][1][2]*Wpk[3][7][1])));
        Wpk[3][9][0] = Cik[9][0][0];
        Wpk[3][9][1] = Cik[9][0][1];
        Wpk[3][9][2] = Cik[9][0][2];
        Wpk[3][10][0] = ((Cik[9][0][2]*Cik[10][2][0])+((Cik[9][0][0]*
          Cik[10][0][0])+(Cik[9][0][1]*Cik[10][1][0])));
        Wpk[3][10][1] = ((Cik[9][0][2]*Cik[10][2][1])+((Cik[9][0][0]*
          Cik[10][0][1])+(Cik[9][0][1]*Cik[10][1][1])));
        Wpk[3][10][2] = ((Cik[9][0][2]*Cik[10][2][2])+((Cik[9][0][0]*
          Cik[10][0][2])+(Cik[9][0][1]*Cik[10][1][2])));
        Wpk[3][11][0] = ((Cik[11][2][0]*Wpk[3][10][2])+((Cik[11][0][0]*
          Wpk[3][10][0])+(Cik[11][1][0]*Wpk[3][10][1])));
        Wpk[3][11][1] = ((Cik[11][2][1]*Wpk[3][10][2])+((Cik[11][0][1]*
          Wpk[3][10][0])+(Cik[11][1][1]*Wpk[3][10][1])));
        Wpk[3][11][2] = ((Cik[11][2][2]*Wpk[3][10][2])+((Cik[11][0][2]*
          Wpk[3][10][0])+(Cik[11][1][2]*Wpk[3][10][1])));
        Wpk[3][12][0] = ((Cik[12][2][0]*Wpk[3][11][2])+((Cik[12][0][0]*
          Wpk[3][11][0])+(Cik[12][1][0]*Wpk[3][11][1])));
        Wpk[3][12][1] = ((Cik[12][2][1]*Wpk[3][11][2])+((Cik[12][0][1]*
          Wpk[3][11][0])+(Cik[12][1][1]*Wpk[3][11][1])));
        Wpk[3][12][2] = ((Cik[12][2][2]*Wpk[3][11][2])+((Cik[12][0][2]*
          Wpk[3][11][0])+(Cik[12][1][2]*Wpk[3][11][1])));
        Wpk[3][13][0] = ((Cik[13][2][0]*Wpk[3][12][2])+((Cik[13][0][0]*
          Wpk[3][12][0])+(Cik[13][1][0]*Wpk[3][12][1])));
        Wpk[3][13][1] = ((Cik[13][2][1]*Wpk[3][12][2])+((Cik[13][0][1]*
          Wpk[3][12][0])+(Cik[13][1][1]*Wpk[3][12][1])));
        Wpk[3][13][2] = ((Cik[13][2][2]*Wpk[3][12][2])+((Cik[13][0][2]*
          Wpk[3][12][0])+(Cik[13][1][2]*Wpk[3][12][1])));
        Wpk[3][14][0] = ((Cik[14][2][0]*Wpk[3][13][2])+((Cik[14][0][0]*
          Wpk[3][13][0])+(Cik[14][1][0]*Wpk[3][13][1])));
        Wpk[3][14][1] = ((Cik[14][2][1]*Wpk[3][13][2])+((Cik[14][0][1]*
          Wpk[3][13][0])+(Cik[14][1][1]*Wpk[3][13][1])));
        Wpk[3][14][2] = ((Cik[14][2][2]*Wpk[3][13][2])+((Cik[14][0][2]*
          Wpk[3][13][0])+(Cik[14][1][2]*Wpk[3][13][1])));
        Wpk[3][15][0] = Cik[15][0][0];
        Wpk[3][15][1] = Cik[15][0][1];
        Wpk[3][15][2] = Cik[15][0][2];
        Wpk[3][16][0] = ((Cik[15][0][2]*Cik[16][2][0])+((Cik[15][0][0]*
          Cik[16][0][0])+(Cik[15][0][1]*Cik[16][1][0])));
        Wpk[3][16][1] = ((Cik[15][0][2]*Cik[16][2][1])+((Cik[15][0][0]*
          Cik[16][0][1])+(Cik[15][0][1]*Cik[16][1][1])));
        Wpk[3][16][2] = ((Cik[15][0][2]*Cik[16][2][2])+((Cik[15][0][0]*
          Cik[16][0][2])+(Cik[15][0][1]*Cik[16][1][2])));
        Wpk[3][17][0] = ((Cik[17][2][0]*Wpk[3][16][2])+((Cik[17][0][0]*
          Wpk[3][16][0])+(Cik[17][1][0]*Wpk[3][16][1])));
        Wpk[3][17][1] = ((Cik[17][2][1]*Wpk[3][16][2])+((Cik[17][0][1]*
          Wpk[3][16][0])+(Cik[17][1][1]*Wpk[3][16][1])));
        Wpk[3][17][2] = ((Cik[17][2][2]*Wpk[3][16][2])+((Cik[17][0][2]*
          Wpk[3][16][0])+(Cik[17][1][2]*Wpk[3][16][1])));
        Wpk[3][18][0] = ((Cik[18][2][0]*Wpk[3][17][2])+((Cik[18][0][0]*
          Wpk[3][17][0])+(Cik[18][1][0]*Wpk[3][17][1])));
        Wpk[3][18][1] = ((Cik[18][2][1]*Wpk[3][17][2])+((Cik[18][0][1]*
          Wpk[3][17][0])+(Cik[18][1][1]*Wpk[3][17][1])));
        Wpk[3][18][2] = ((Cik[18][2][2]*Wpk[3][17][2])+((Cik[18][0][2]*
          Wpk[3][17][0])+(Cik[18][1][2]*Wpk[3][17][1])));
        Wpk[3][19][0] = ((Cik[19][2][0]*Wpk[3][18][2])+((Cik[19][0][0]*
          Wpk[3][18][0])+(Cik[19][1][0]*Wpk[3][18][1])));
        Wpk[3][19][1] = ((Cik[19][2][1]*Wpk[3][18][2])+((Cik[19][0][1]*
          Wpk[3][18][0])+(Cik[19][1][1]*Wpk[3][18][1])));
        Wpk[3][19][2] = ((Cik[19][2][2]*Wpk[3][18][2])+((Cik[19][0][2]*
          Wpk[3][18][0])+(Cik[19][1][2]*Wpk[3][18][1])));
        Wpk[3][20][0] = ((Cik[20][2][0]*Wpk[3][19][2])+((Cik[20][0][0]*
          Wpk[3][19][0])+(Cik[20][1][0]*Wpk[3][19][1])));
        Wpk[3][20][1] = ((Cik[20][2][1]*Wpk[3][19][2])+((Cik[20][0][1]*
          Wpk[3][19][0])+(Cik[20][1][1]*Wpk[3][19][1])));
        Wpk[3][20][2] = ((Cik[20][2][2]*Wpk[3][19][2])+((Cik[20][0][2]*
          Wpk[3][19][0])+(Cik[20][1][2]*Wpk[3][19][1])));
        Wpk[4][4][1] = 1.;
        Wpk[4][5][1] = 1.;
        Wpk[4][6][0] = Cik[6][1][0];
        Wpk[4][6][1] = Cik[6][1][1];
        Wpk[4][6][2] = Cik[6][1][2];
        Wpk[4][7][0] = ((Cik[6][1][2]*Cik[7][2][0])+((Cik[6][1][0]*Cik[7][0][0])
          +(Cik[6][1][1]*Cik[7][1][0])));
        Wpk[4][7][1] = ((Cik[6][1][2]*Cik[7][2][1])+((Cik[6][1][0]*Cik[7][0][1])
          +(Cik[6][1][1]*Cik[7][1][1])));
        Wpk[4][7][2] = ((Cik[6][1][2]*Cik[7][2][2])+((Cik[6][1][0]*Cik[7][0][2])
          +(Cik[6][1][1]*Cik[7][1][2])));
        Wpk[4][8][0] = ((Cik[8][2][0]*Wpk[4][7][2])+((Cik[8][0][0]*Wpk[4][7][0])
          +(Cik[8][1][0]*Wpk[4][7][1])));
        Wpk[4][8][1] = ((Cik[8][2][1]*Wpk[4][7][2])+((Cik[8][0][1]*Wpk[4][7][0])
          +(Cik[8][1][1]*Wpk[4][7][1])));
        Wpk[4][8][2] = ((Cik[8][2][2]*Wpk[4][7][2])+((Cik[8][0][2]*Wpk[4][7][0])
          +(Cik[8][1][2]*Wpk[4][7][1])));
        Wpk[4][9][0] = Cik[9][1][0];
        Wpk[4][9][1] = Cik[9][1][1];
        Wpk[4][9][2] = Cik[9][1][2];
        Wpk[4][10][0] = ((Cik[9][1][2]*Cik[10][2][0])+((Cik[9][1][0]*
          Cik[10][0][0])+(Cik[9][1][1]*Cik[10][1][0])));
        Wpk[4][10][1] = ((Cik[9][1][2]*Cik[10][2][1])+((Cik[9][1][0]*
          Cik[10][0][1])+(Cik[9][1][1]*Cik[10][1][1])));
        Wpk[4][10][2] = ((Cik[9][1][2]*Cik[10][2][2])+((Cik[9][1][0]*
          Cik[10][0][2])+(Cik[9][1][1]*Cik[10][1][2])));
        Wpk[4][11][0] = ((Cik[11][2][0]*Wpk[4][10][2])+((Cik[11][0][0]*
          Wpk[4][10][0])+(Cik[11][1][0]*Wpk[4][10][1])));
        Wpk[4][11][1] = ((Cik[11][2][1]*Wpk[4][10][2])+((Cik[11][0][1]*
          Wpk[4][10][0])+(Cik[11][1][1]*Wpk[4][10][1])));
        Wpk[4][11][2] = ((Cik[11][2][2]*Wpk[4][10][2])+((Cik[11][0][2]*
          Wpk[4][10][0])+(Cik[11][1][2]*Wpk[4][10][1])));
        Wpk[4][12][0] = ((Cik[12][2][0]*Wpk[4][11][2])+((Cik[12][0][0]*
          Wpk[4][11][0])+(Cik[12][1][0]*Wpk[4][11][1])));
        Wpk[4][12][1] = ((Cik[12][2][1]*Wpk[4][11][2])+((Cik[12][0][1]*
          Wpk[4][11][0])+(Cik[12][1][1]*Wpk[4][11][1])));
        Wpk[4][12][2] = ((Cik[12][2][2]*Wpk[4][11][2])+((Cik[12][0][2]*
          Wpk[4][11][0])+(Cik[12][1][2]*Wpk[4][11][1])));
        Wpk[4][13][0] = ((Cik[13][2][0]*Wpk[4][12][2])+((Cik[13][0][0]*
          Wpk[4][12][0])+(Cik[13][1][0]*Wpk[4][12][1])));
        Wpk[4][13][1] = ((Cik[13][2][1]*Wpk[4][12][2])+((Cik[13][0][1]*
          Wpk[4][12][0])+(Cik[13][1][1]*Wpk[4][12][1])));
        Wpk[4][13][2] = ((Cik[13][2][2]*Wpk[4][12][2])+((Cik[13][0][2]*
          Wpk[4][12][0])+(Cik[13][1][2]*Wpk[4][12][1])));
        Wpk[4][14][0] = ((Cik[14][2][0]*Wpk[4][13][2])+((Cik[14][0][0]*
          Wpk[4][13][0])+(Cik[14][1][0]*Wpk[4][13][1])));
        Wpk[4][14][1] = ((Cik[14][2][1]*Wpk[4][13][2])+((Cik[14][0][1]*
          Wpk[4][13][0])+(Cik[14][1][1]*Wpk[4][13][1])));
        Wpk[4][14][2] = ((Cik[14][2][2]*Wpk[4][13][2])+((Cik[14][0][2]*
          Wpk[4][13][0])+(Cik[14][1][2]*Wpk[4][13][1])));
        Wpk[4][15][0] = Cik[15][1][0];
        Wpk[4][15][1] = Cik[15][1][1];
        Wpk[4][15][2] = Cik[15][1][2];
        Wpk[4][16][0] = ((Cik[15][1][2]*Cik[16][2][0])+((Cik[15][1][0]*
          Cik[16][0][0])+(Cik[15][1][1]*Cik[16][1][0])));
        Wpk[4][16][1] = ((Cik[15][1][2]*Cik[16][2][1])+((Cik[15][1][0]*
          Cik[16][0][1])+(Cik[15][1][1]*Cik[16][1][1])));
        Wpk[4][16][2] = ((Cik[15][1][2]*Cik[16][2][2])+((Cik[15][1][0]*
          Cik[16][0][2])+(Cik[15][1][1]*Cik[16][1][2])));
        Wpk[4][17][0] = ((Cik[17][2][0]*Wpk[4][16][2])+((Cik[17][0][0]*
          Wpk[4][16][0])+(Cik[17][1][0]*Wpk[4][16][1])));
        Wpk[4][17][1] = ((Cik[17][2][1]*Wpk[4][16][2])+((Cik[17][0][1]*
          Wpk[4][16][0])+(Cik[17][1][1]*Wpk[4][16][1])));
        Wpk[4][17][2] = ((Cik[17][2][2]*Wpk[4][16][2])+((Cik[17][0][2]*
          Wpk[4][16][0])+(Cik[17][1][2]*Wpk[4][16][1])));
        Wpk[4][18][0] = ((Cik[18][2][0]*Wpk[4][17][2])+((Cik[18][0][0]*
          Wpk[4][17][0])+(Cik[18][1][0]*Wpk[4][17][1])));
        Wpk[4][18][1] = ((Cik[18][2][1]*Wpk[4][17][2])+((Cik[18][0][1]*
          Wpk[4][17][0])+(Cik[18][1][1]*Wpk[4][17][1])));
        Wpk[4][18][2] = ((Cik[18][2][2]*Wpk[4][17][2])+((Cik[18][0][2]*
          Wpk[4][17][0])+(Cik[18][1][2]*Wpk[4][17][1])));
        Wpk[4][19][0] = ((Cik[19][2][0]*Wpk[4][18][2])+((Cik[19][0][0]*
          Wpk[4][18][0])+(Cik[19][1][0]*Wpk[4][18][1])));
        Wpk[4][19][1] = ((Cik[19][2][1]*Wpk[4][18][2])+((Cik[19][0][1]*
          Wpk[4][18][0])+(Cik[19][1][1]*Wpk[4][18][1])));
        Wpk[4][19][2] = ((Cik[19][2][2]*Wpk[4][18][2])+((Cik[19][0][2]*
          Wpk[4][18][0])+(Cik[19][1][2]*Wpk[4][18][1])));
        Wpk[4][20][0] = ((Cik[20][2][0]*Wpk[4][19][2])+((Cik[20][0][0]*
          Wpk[4][19][0])+(Cik[20][1][0]*Wpk[4][19][1])));
        Wpk[4][20][1] = ((Cik[20][2][1]*Wpk[4][19][2])+((Cik[20][0][1]*
          Wpk[4][19][0])+(Cik[20][1][1]*Wpk[4][19][1])));
        Wpk[4][20][2] = ((Cik[20][2][2]*Wpk[4][19][2])+((Cik[20][0][2]*
          Wpk[4][19][0])+(Cik[20][1][2]*Wpk[4][19][1])));
        Wpk[5][5][2] = 1.;
        Wpk[5][6][0] = Cik[6][2][0];
        Wpk[5][6][1] = Cik[6][2][1];
        Wpk[5][6][2] = Cik[6][2][2];
        Wpk[5][7][0] = ((Cik[6][2][2]*Cik[7][2][0])+((Cik[6][2][0]*Cik[7][0][0])
          +(Cik[6][2][1]*Cik[7][1][0])));
        Wpk[5][7][1] = ((Cik[6][2][2]*Cik[7][2][1])+((Cik[6][2][0]*Cik[7][0][1])
          +(Cik[6][2][1]*Cik[7][1][1])));
        Wpk[5][7][2] = ((Cik[6][2][2]*Cik[7][2][2])+((Cik[6][2][0]*Cik[7][0][2])
          +(Cik[6][2][1]*Cik[7][1][2])));
        Wpk[5][8][0] = ((Cik[8][2][0]*Wpk[5][7][2])+((Cik[8][0][0]*Wpk[5][7][0])
          +(Cik[8][1][0]*Wpk[5][7][1])));
        Wpk[5][8][1] = ((Cik[8][2][1]*Wpk[5][7][2])+((Cik[8][0][1]*Wpk[5][7][0])
          +(Cik[8][1][1]*Wpk[5][7][1])));
        Wpk[5][8][2] = ((Cik[8][2][2]*Wpk[5][7][2])+((Cik[8][0][2]*Wpk[5][7][0])
          +(Cik[8][1][2]*Wpk[5][7][1])));
        Wpk[5][9][0] = Cik[9][2][0];
        Wpk[5][9][1] = Cik[9][2][1];
        Wpk[5][9][2] = Cik[9][2][2];
        Wpk[5][10][0] = ((Cik[9][2][2]*Cik[10][2][0])+((Cik[9][2][0]*
          Cik[10][0][0])+(Cik[9][2][1]*Cik[10][1][0])));
        Wpk[5][10][1] = ((Cik[9][2][2]*Cik[10][2][1])+((Cik[9][2][0]*
          Cik[10][0][1])+(Cik[9][2][1]*Cik[10][1][1])));
        Wpk[5][10][2] = ((Cik[9][2][2]*Cik[10][2][2])+((Cik[9][2][0]*
          Cik[10][0][2])+(Cik[9][2][1]*Cik[10][1][2])));
        Wpk[5][11][0] = ((Cik[11][2][0]*Wpk[5][10][2])+((Cik[11][0][0]*
          Wpk[5][10][0])+(Cik[11][1][0]*Wpk[5][10][1])));
        Wpk[5][11][1] = ((Cik[11][2][1]*Wpk[5][10][2])+((Cik[11][0][1]*
          Wpk[5][10][0])+(Cik[11][1][1]*Wpk[5][10][1])));
        Wpk[5][11][2] = ((Cik[11][2][2]*Wpk[5][10][2])+((Cik[11][0][2]*
          Wpk[5][10][0])+(Cik[11][1][2]*Wpk[5][10][1])));
        Wpk[5][12][0] = ((Cik[12][2][0]*Wpk[5][11][2])+((Cik[12][0][0]*
          Wpk[5][11][0])+(Cik[12][1][0]*Wpk[5][11][1])));
        Wpk[5][12][1] = ((Cik[12][2][1]*Wpk[5][11][2])+((Cik[12][0][1]*
          Wpk[5][11][0])+(Cik[12][1][1]*Wpk[5][11][1])));
        Wpk[5][12][2] = ((Cik[12][2][2]*Wpk[5][11][2])+((Cik[12][0][2]*
          Wpk[5][11][0])+(Cik[12][1][2]*Wpk[5][11][1])));
        Wpk[5][13][0] = ((Cik[13][2][0]*Wpk[5][12][2])+((Cik[13][0][0]*
          Wpk[5][12][0])+(Cik[13][1][0]*Wpk[5][12][1])));
        Wpk[5][13][1] = ((Cik[13][2][1]*Wpk[5][12][2])+((Cik[13][0][1]*
          Wpk[5][12][0])+(Cik[13][1][1]*Wpk[5][12][1])));
        Wpk[5][13][2] = ((Cik[13][2][2]*Wpk[5][12][2])+((Cik[13][0][2]*
          Wpk[5][12][0])+(Cik[13][1][2]*Wpk[5][12][1])));
        Wpk[5][14][0] = ((Cik[14][2][0]*Wpk[5][13][2])+((Cik[14][0][0]*
          Wpk[5][13][0])+(Cik[14][1][0]*Wpk[5][13][1])));
        Wpk[5][14][1] = ((Cik[14][2][1]*Wpk[5][13][2])+((Cik[14][0][1]*
          Wpk[5][13][0])+(Cik[14][1][1]*Wpk[5][13][1])));
        Wpk[5][14][2] = ((Cik[14][2][2]*Wpk[5][13][2])+((Cik[14][0][2]*
          Wpk[5][13][0])+(Cik[14][1][2]*Wpk[5][13][1])));
        Wpk[5][15][0] = Cik[15][2][0];
        Wpk[5][15][1] = Cik[15][2][1];
        Wpk[5][15][2] = Cik[15][2][2];
        Wpk[5][16][0] = ((Cik[15][2][2]*Cik[16][2][0])+((Cik[15][2][0]*
          Cik[16][0][0])+(Cik[15][2][1]*Cik[16][1][0])));
        Wpk[5][16][1] = ((Cik[15][2][2]*Cik[16][2][1])+((Cik[15][2][0]*
          Cik[16][0][1])+(Cik[15][2][1]*Cik[16][1][1])));
        Wpk[5][16][2] = ((Cik[15][2][2]*Cik[16][2][2])+((Cik[15][2][0]*
          Cik[16][0][2])+(Cik[15][2][1]*Cik[16][1][2])));
        Wpk[5][17][0] = ((Cik[17][2][0]*Wpk[5][16][2])+((Cik[17][0][0]*
          Wpk[5][16][0])+(Cik[17][1][0]*Wpk[5][16][1])));
        Wpk[5][17][1] = ((Cik[17][2][1]*Wpk[5][16][2])+((Cik[17][0][1]*
          Wpk[5][16][0])+(Cik[17][1][1]*Wpk[5][16][1])));
        Wpk[5][17][2] = ((Cik[17][2][2]*Wpk[5][16][2])+((Cik[17][0][2]*
          Wpk[5][16][0])+(Cik[17][1][2]*Wpk[5][16][1])));
        Wpk[5][18][0] = ((Cik[18][2][0]*Wpk[5][17][2])+((Cik[18][0][0]*
          Wpk[5][17][0])+(Cik[18][1][0]*Wpk[5][17][1])));
        Wpk[5][18][1] = ((Cik[18][2][1]*Wpk[5][17][2])+((Cik[18][0][1]*
          Wpk[5][17][0])+(Cik[18][1][1]*Wpk[5][17][1])));
        Wpk[5][18][2] = ((Cik[18][2][2]*Wpk[5][17][2])+((Cik[18][0][2]*
          Wpk[5][17][0])+(Cik[18][1][2]*Wpk[5][17][1])));
        Wpk[5][19][0] = ((Cik[19][2][0]*Wpk[5][18][2])+((Cik[19][0][0]*
          Wpk[5][18][0])+(Cik[19][1][0]*Wpk[5][18][1])));
        Wpk[5][19][1] = ((Cik[19][2][1]*Wpk[5][18][2])+((Cik[19][0][1]*
          Wpk[5][18][0])+(Cik[19][1][1]*Wpk[5][18][1])));
        Wpk[5][19][2] = ((Cik[19][2][2]*Wpk[5][18][2])+((Cik[19][0][2]*
          Wpk[5][18][0])+(Cik[19][1][2]*Wpk[5][18][1])));
        Wpk[5][20][0] = ((Cik[20][2][0]*Wpk[5][19][2])+((Cik[20][0][0]*
          Wpk[5][19][0])+(Cik[20][1][0]*Wpk[5][19][1])));
        Wpk[5][20][1] = ((Cik[20][2][1]*Wpk[5][19][2])+((Cik[20][0][1]*
          Wpk[5][19][0])+(Cik[20][1][1]*Wpk[5][19][1])));
        Wpk[5][20][2] = ((Cik[20][2][2]*Wpk[5][19][2])+((Cik[20][0][2]*
          Wpk[5][19][0])+(Cik[20][1][2]*Wpk[5][19][1])));
        Wpk[6][6][0] = pin[6][0];
        Wpk[6][6][1] = pin[6][1];
        Wpk[6][6][2] = pin[6][2];
        Wpk[6][7][0] = ((Cik[7][2][0]*pin[6][2])+((Cik[7][0][0]*pin[6][0])+(
          Cik[7][1][0]*pin[6][1])));
        Wpk[6][7][1] = ((Cik[7][2][1]*pin[6][2])+((Cik[7][0][1]*pin[6][0])+(
          Cik[7][1][1]*pin[6][1])));
        Wpk[6][7][2] = ((Cik[7][2][2]*pin[6][2])+((Cik[7][0][2]*pin[6][0])+(
          Cik[7][1][2]*pin[6][1])));
        Wpk[6][8][0] = ((Cik[8][2][0]*Wpk[6][7][2])+((Cik[8][0][0]*Wpk[6][7][0])
          +(Cik[8][1][0]*Wpk[6][7][1])));
        Wpk[6][8][1] = ((Cik[8][2][1]*Wpk[6][7][2])+((Cik[8][0][1]*Wpk[6][7][0])
          +(Cik[8][1][1]*Wpk[6][7][1])));
        Wpk[6][8][2] = ((Cik[8][2][2]*Wpk[6][7][2])+((Cik[8][0][2]*Wpk[6][7][0])
          +(Cik[8][1][2]*Wpk[6][7][1])));
        Wpk[7][7][0] = pin[7][0];
        Wpk[7][7][1] = pin[7][1];
        Wpk[7][7][2] = pin[7][2];
        Wpk[7][8][0] = ((Cik[8][2][0]*pin[7][2])+((Cik[8][0][0]*pin[7][0])+(
          Cik[8][1][0]*pin[7][1])));
        Wpk[7][8][1] = ((Cik[8][2][1]*pin[7][2])+((Cik[8][0][1]*pin[7][0])+(
          Cik[8][1][1]*pin[7][1])));
        Wpk[7][8][2] = ((Cik[8][2][2]*pin[7][2])+((Cik[8][0][2]*pin[7][0])+(
          Cik[8][1][2]*pin[7][1])));
        Wpk[8][8][0] = pin[8][0];
        Wpk[8][8][1] = pin[8][1];
        Wpk[8][8][2] = pin[8][2];
        Wpk[9][9][0] = pin[9][0];
        Wpk[9][9][1] = pin[9][1];
        Wpk[9][9][2] = pin[9][2];
        Wpk[9][10][0] = ((Cik[10][2][0]*pin[9][2])+((Cik[10][0][0]*pin[9][0])+(
          Cik[10][1][0]*pin[9][1])));
        Wpk[9][10][1] = ((Cik[10][2][1]*pin[9][2])+((Cik[10][0][1]*pin[9][0])+(
          Cik[10][1][1]*pin[9][1])));
        Wpk[9][10][2] = ((Cik[10][2][2]*pin[9][2])+((Cik[10][0][2]*pin[9][0])+(
          Cik[10][1][2]*pin[9][1])));
        Wpk[9][11][0] = ((Cik[11][2][0]*Wpk[9][10][2])+((Cik[11][0][0]*
          Wpk[9][10][0])+(Cik[11][1][0]*Wpk[9][10][1])));
        Wpk[9][11][1] = ((Cik[11][2][1]*Wpk[9][10][2])+((Cik[11][0][1]*
          Wpk[9][10][0])+(Cik[11][1][1]*Wpk[9][10][1])));
        Wpk[9][11][2] = ((Cik[11][2][2]*Wpk[9][10][2])+((Cik[11][0][2]*
          Wpk[9][10][0])+(Cik[11][1][2]*Wpk[9][10][1])));
        Wpk[9][12][0] = ((Cik[12][2][0]*Wpk[9][11][2])+((Cik[12][0][0]*
          Wpk[9][11][0])+(Cik[12][1][0]*Wpk[9][11][1])));
        Wpk[9][12][1] = ((Cik[12][2][1]*Wpk[9][11][2])+((Cik[12][0][1]*
          Wpk[9][11][0])+(Cik[12][1][1]*Wpk[9][11][1])));
        Wpk[9][12][2] = ((Cik[12][2][2]*Wpk[9][11][2])+((Cik[12][0][2]*
          Wpk[9][11][0])+(Cik[12][1][2]*Wpk[9][11][1])));
        Wpk[9][13][0] = ((Cik[13][2][0]*Wpk[9][12][2])+((Cik[13][0][0]*
          Wpk[9][12][0])+(Cik[13][1][0]*Wpk[9][12][1])));
        Wpk[9][13][1] = ((Cik[13][2][1]*Wpk[9][12][2])+((Cik[13][0][1]*
          Wpk[9][12][0])+(Cik[13][1][1]*Wpk[9][12][1])));
        Wpk[9][13][2] = ((Cik[13][2][2]*Wpk[9][12][2])+((Cik[13][0][2]*
          Wpk[9][12][0])+(Cik[13][1][2]*Wpk[9][12][1])));
        Wpk[9][14][0] = ((Cik[14][2][0]*Wpk[9][13][2])+((Cik[14][0][0]*
          Wpk[9][13][0])+(Cik[14][1][0]*Wpk[9][13][1])));
        Wpk[9][14][1] = ((Cik[14][2][1]*Wpk[9][13][2])+((Cik[14][0][1]*
          Wpk[9][13][0])+(Cik[14][1][1]*Wpk[9][13][1])));
        Wpk[9][14][2] = ((Cik[14][2][2]*Wpk[9][13][2])+((Cik[14][0][2]*
          Wpk[9][13][0])+(Cik[14][1][2]*Wpk[9][13][1])));
        Wpk[10][10][0] = pin[10][0];
        Wpk[10][10][1] = pin[10][1];
        Wpk[10][10][2] = pin[10][2];
        Wpk[10][11][0] = ((Cik[11][2][0]*pin[10][2])+((Cik[11][0][0]*pin[10][0])
          +(Cik[11][1][0]*pin[10][1])));
        Wpk[10][11][1] = ((Cik[11][2][1]*pin[10][2])+((Cik[11][0][1]*pin[10][0])
          +(Cik[11][1][1]*pin[10][1])));
        Wpk[10][11][2] = ((Cik[11][2][2]*pin[10][2])+((Cik[11][0][2]*pin[10][0])
          +(Cik[11][1][2]*pin[10][1])));
        Wpk[10][12][0] = ((Cik[12][2][0]*Wpk[10][11][2])+((Cik[12][0][0]*
          Wpk[10][11][0])+(Cik[12][1][0]*Wpk[10][11][1])));
        Wpk[10][12][1] = ((Cik[12][2][1]*Wpk[10][11][2])+((Cik[12][0][1]*
          Wpk[10][11][0])+(Cik[12][1][1]*Wpk[10][11][1])));
        Wpk[10][12][2] = ((Cik[12][2][2]*Wpk[10][11][2])+((Cik[12][0][2]*
          Wpk[10][11][0])+(Cik[12][1][2]*Wpk[10][11][1])));
        Wpk[10][13][0] = ((Cik[13][2][0]*Wpk[10][12][2])+((Cik[13][0][0]*
          Wpk[10][12][0])+(Cik[13][1][0]*Wpk[10][12][1])));
        Wpk[10][13][1] = ((Cik[13][2][1]*Wpk[10][12][2])+((Cik[13][0][1]*
          Wpk[10][12][0])+(Cik[13][1][1]*Wpk[10][12][1])));
        Wpk[10][13][2] = ((Cik[13][2][2]*Wpk[10][12][2])+((Cik[13][0][2]*
          Wpk[10][12][0])+(Cik[13][1][2]*Wpk[10][12][1])));
        Wpk[10][14][0] = ((Cik[14][2][0]*Wpk[10][13][2])+((Cik[14][0][0]*
          Wpk[10][13][0])+(Cik[14][1][0]*Wpk[10][13][1])));
        Wpk[10][14][1] = ((Cik[14][2][1]*Wpk[10][13][2])+((Cik[14][0][1]*
          Wpk[10][13][0])+(Cik[14][1][1]*Wpk[10][13][1])));
        Wpk[10][14][2] = ((Cik[14][2][2]*Wpk[10][13][2])+((Cik[14][0][2]*
          Wpk[10][13][0])+(Cik[14][1][2]*Wpk[10][13][1])));
        Wpk[11][11][0] = pin[11][0];
        Wpk[11][11][1] = pin[11][1];
        Wpk[11][11][2] = pin[11][2];
        Wpk[11][12][0] = ((Cik[12][2][0]*pin[11][2])+((Cik[12][0][0]*pin[11][0])
          +(Cik[12][1][0]*pin[11][1])));
        Wpk[11][12][1] = ((Cik[12][2][1]*pin[11][2])+((Cik[12][0][1]*pin[11][0])
          +(Cik[12][1][1]*pin[11][1])));
        Wpk[11][12][2] = ((Cik[12][2][2]*pin[11][2])+((Cik[12][0][2]*pin[11][0])
          +(Cik[12][1][2]*pin[11][1])));
        Wpk[11][13][0] = ((Cik[13][2][0]*Wpk[11][12][2])+((Cik[13][0][0]*
          Wpk[11][12][0])+(Cik[13][1][0]*Wpk[11][12][1])));
        Wpk[11][13][1] = ((Cik[13][2][1]*Wpk[11][12][2])+((Cik[13][0][1]*
          Wpk[11][12][0])+(Cik[13][1][1]*Wpk[11][12][1])));
        Wpk[11][13][2] = ((Cik[13][2][2]*Wpk[11][12][2])+((Cik[13][0][2]*
          Wpk[11][12][0])+(Cik[13][1][2]*Wpk[11][12][1])));
        Wpk[11][14][0] = ((Cik[14][2][0]*Wpk[11][13][2])+((Cik[14][0][0]*
          Wpk[11][13][0])+(Cik[14][1][0]*Wpk[11][13][1])));
        Wpk[11][14][1] = ((Cik[14][2][1]*Wpk[11][13][2])+((Cik[14][0][1]*
          Wpk[11][13][0])+(Cik[14][1][1]*Wpk[11][13][1])));
        Wpk[11][14][2] = ((Cik[14][2][2]*Wpk[11][13][2])+((Cik[14][0][2]*
          Wpk[11][13][0])+(Cik[14][1][2]*Wpk[11][13][1])));
        Wpk[12][12][0] = pin[12][0];
        Wpk[12][12][1] = pin[12][1];
        Wpk[12][12][2] = pin[12][2];
        Wpk[12][13][0] = ((Cik[13][2][0]*pin[12][2])+((Cik[13][0][0]*pin[12][0])
          +(Cik[13][1][0]*pin[12][1])));
        Wpk[12][13][1] = ((Cik[13][2][1]*pin[12][2])+((Cik[13][0][1]*pin[12][0])
          +(Cik[13][1][1]*pin[12][1])));
        Wpk[12][13][2] = ((Cik[13][2][2]*pin[12][2])+((Cik[13][0][2]*pin[12][0])
          +(Cik[13][1][2]*pin[12][1])));
        Wpk[12][14][0] = ((Cik[14][2][0]*Wpk[12][13][2])+((Cik[14][0][0]*
          Wpk[12][13][0])+(Cik[14][1][0]*Wpk[12][13][1])));
        Wpk[12][14][1] = ((Cik[14][2][1]*Wpk[12][13][2])+((Cik[14][0][1]*
          Wpk[12][13][0])+(Cik[14][1][1]*Wpk[12][13][1])));
        Wpk[12][14][2] = ((Cik[14][2][2]*Wpk[12][13][2])+((Cik[14][0][2]*
          Wpk[12][13][0])+(Cik[14][1][2]*Wpk[12][13][1])));
        Wpk[13][13][0] = pin[13][0];
        Wpk[13][13][1] = pin[13][1];
        Wpk[13][13][2] = pin[13][2];
        Wpk[13][14][0] = ((Cik[14][2][0]*pin[13][2])+((Cik[14][0][0]*pin[13][0])
          +(Cik[14][1][0]*pin[13][1])));
        Wpk[13][14][1] = ((Cik[14][2][1]*pin[13][2])+((Cik[14][0][1]*pin[13][0])
          +(Cik[14][1][1]*pin[13][1])));
        Wpk[13][14][2] = ((Cik[14][2][2]*pin[13][2])+((Cik[14][0][2]*pin[13][0])
          +(Cik[14][1][2]*pin[13][1])));
        Wpk[14][14][0] = pin[14][0];
        Wpk[14][14][1] = pin[14][1];
        Wpk[14][14][2] = pin[14][2];
        Wpk[15][15][0] = pin[15][0];
        Wpk[15][15][1] = pin[15][1];
        Wpk[15][15][2] = pin[15][2];
        Wpk[15][16][0] = ((Cik[16][2][0]*pin[15][2])+((Cik[16][0][0]*pin[15][0])
          +(Cik[16][1][0]*pin[15][1])));
        Wpk[15][16][1] = ((Cik[16][2][1]*pin[15][2])+((Cik[16][0][1]*pin[15][0])
          +(Cik[16][1][1]*pin[15][1])));
        Wpk[15][16][2] = ((Cik[16][2][2]*pin[15][2])+((Cik[16][0][2]*pin[15][0])
          +(Cik[16][1][2]*pin[15][1])));
        Wpk[15][17][0] = ((Cik[17][2][0]*Wpk[15][16][2])+((Cik[17][0][0]*
          Wpk[15][16][0])+(Cik[17][1][0]*Wpk[15][16][1])));
        Wpk[15][17][1] = ((Cik[17][2][1]*Wpk[15][16][2])+((Cik[17][0][1]*
          Wpk[15][16][0])+(Cik[17][1][1]*Wpk[15][16][1])));
        Wpk[15][17][2] = ((Cik[17][2][2]*Wpk[15][16][2])+((Cik[17][0][2]*
          Wpk[15][16][0])+(Cik[17][1][2]*Wpk[15][16][1])));
        Wpk[15][18][0] = ((Cik[18][2][0]*Wpk[15][17][2])+((Cik[18][0][0]*
          Wpk[15][17][0])+(Cik[18][1][0]*Wpk[15][17][1])));
        Wpk[15][18][1] = ((Cik[18][2][1]*Wpk[15][17][2])+((Cik[18][0][1]*
          Wpk[15][17][0])+(Cik[18][1][1]*Wpk[15][17][1])));
        Wpk[15][18][2] = ((Cik[18][2][2]*Wpk[15][17][2])+((Cik[18][0][2]*
          Wpk[15][17][0])+(Cik[18][1][2]*Wpk[15][17][1])));
        Wpk[15][19][0] = ((Cik[19][2][0]*Wpk[15][18][2])+((Cik[19][0][0]*
          Wpk[15][18][0])+(Cik[19][1][0]*Wpk[15][18][1])));
        Wpk[15][19][1] = ((Cik[19][2][1]*Wpk[15][18][2])+((Cik[19][0][1]*
          Wpk[15][18][0])+(Cik[19][1][1]*Wpk[15][18][1])));
        Wpk[15][19][2] = ((Cik[19][2][2]*Wpk[15][18][2])+((Cik[19][0][2]*
          Wpk[15][18][0])+(Cik[19][1][2]*Wpk[15][18][1])));
        Wpk[15][20][0] = ((Cik[20][2][0]*Wpk[15][19][2])+((Cik[20][0][0]*
          Wpk[15][19][0])+(Cik[20][1][0]*Wpk[15][19][1])));
        Wpk[15][20][1] = ((Cik[20][2][1]*Wpk[15][19][2])+((Cik[20][0][1]*
          Wpk[15][19][0])+(Cik[20][1][1]*Wpk[15][19][1])));
        Wpk[15][20][2] = ((Cik[20][2][2]*Wpk[15][19][2])+((Cik[20][0][2]*
          Wpk[15][19][0])+(Cik[20][1][2]*Wpk[15][19][1])));
        Wpk[16][16][0] = pin[16][0];
        Wpk[16][16][1] = pin[16][1];
        Wpk[16][16][2] = pin[16][2];
        Wpk[16][17][0] = ((Cik[17][2][0]*pin[16][2])+((Cik[17][0][0]*pin[16][0])
          +(Cik[17][1][0]*pin[16][1])));
        Wpk[16][17][1] = ((Cik[17][2][1]*pin[16][2])+((Cik[17][0][1]*pin[16][0])
          +(Cik[17][1][1]*pin[16][1])));
        Wpk[16][17][2] = ((Cik[17][2][2]*pin[16][2])+((Cik[17][0][2]*pin[16][0])
          +(Cik[17][1][2]*pin[16][1])));
        Wpk[16][18][0] = ((Cik[18][2][0]*Wpk[16][17][2])+((Cik[18][0][0]*
          Wpk[16][17][0])+(Cik[18][1][0]*Wpk[16][17][1])));
        Wpk[16][18][1] = ((Cik[18][2][1]*Wpk[16][17][2])+((Cik[18][0][1]*
          Wpk[16][17][0])+(Cik[18][1][1]*Wpk[16][17][1])));
        Wpk[16][18][2] = ((Cik[18][2][2]*Wpk[16][17][2])+((Cik[18][0][2]*
          Wpk[16][17][0])+(Cik[18][1][2]*Wpk[16][17][1])));
        Wpk[16][19][0] = ((Cik[19][2][0]*Wpk[16][18][2])+((Cik[19][0][0]*
          Wpk[16][18][0])+(Cik[19][1][0]*Wpk[16][18][1])));
        Wpk[16][19][1] = ((Cik[19][2][1]*Wpk[16][18][2])+((Cik[19][0][1]*
          Wpk[16][18][0])+(Cik[19][1][1]*Wpk[16][18][1])));
        Wpk[16][19][2] = ((Cik[19][2][2]*Wpk[16][18][2])+((Cik[19][0][2]*
          Wpk[16][18][0])+(Cik[19][1][2]*Wpk[16][18][1])));
        Wpk[16][20][0] = ((Cik[20][2][0]*Wpk[16][19][2])+((Cik[20][0][0]*
          Wpk[16][19][0])+(Cik[20][1][0]*Wpk[16][19][1])));
        Wpk[16][20][1] = ((Cik[20][2][1]*Wpk[16][19][2])+((Cik[20][0][1]*
          Wpk[16][19][0])+(Cik[20][1][1]*Wpk[16][19][1])));
        Wpk[16][20][2] = ((Cik[20][2][2]*Wpk[16][19][2])+((Cik[20][0][2]*
          Wpk[16][19][0])+(Cik[20][1][2]*Wpk[16][19][1])));
        Wpk[17][17][0] = pin[17][0];
        Wpk[17][17][1] = pin[17][1];
        Wpk[17][17][2] = pin[17][2];
        Wpk[17][18][0] = ((Cik[18][2][0]*pin[17][2])+((Cik[18][0][0]*pin[17][0])
          +(Cik[18][1][0]*pin[17][1])));
        Wpk[17][18][1] = ((Cik[18][2][1]*pin[17][2])+((Cik[18][0][1]*pin[17][0])
          +(Cik[18][1][1]*pin[17][1])));
        Wpk[17][18][2] = ((Cik[18][2][2]*pin[17][2])+((Cik[18][0][2]*pin[17][0])
          +(Cik[18][1][2]*pin[17][1])));
        Wpk[17][19][0] = ((Cik[19][2][0]*Wpk[17][18][2])+((Cik[19][0][0]*
          Wpk[17][18][0])+(Cik[19][1][0]*Wpk[17][18][1])));
        Wpk[17][19][1] = ((Cik[19][2][1]*Wpk[17][18][2])+((Cik[19][0][1]*
          Wpk[17][18][0])+(Cik[19][1][1]*Wpk[17][18][1])));
        Wpk[17][19][2] = ((Cik[19][2][2]*Wpk[17][18][2])+((Cik[19][0][2]*
          Wpk[17][18][0])+(Cik[19][1][2]*Wpk[17][18][1])));
        Wpk[17][20][0] = ((Cik[20][2][0]*Wpk[17][19][2])+((Cik[20][0][0]*
          Wpk[17][19][0])+(Cik[20][1][0]*Wpk[17][19][1])));
        Wpk[17][20][1] = ((Cik[20][2][1]*Wpk[17][19][2])+((Cik[20][0][1]*
          Wpk[17][19][0])+(Cik[20][1][1]*Wpk[17][19][1])));
        Wpk[17][20][2] = ((Cik[20][2][2]*Wpk[17][19][2])+((Cik[20][0][2]*
          Wpk[17][19][0])+(Cik[20][1][2]*Wpk[17][19][1])));
        Wpk[18][18][0] = pin[18][0];
        Wpk[18][18][1] = pin[18][1];
        Wpk[18][18][2] = pin[18][2];
        Wpk[18][19][0] = ((Cik[19][2][0]*pin[18][2])+((Cik[19][0][0]*pin[18][0])
          +(Cik[19][1][0]*pin[18][1])));
        Wpk[18][19][1] = ((Cik[19][2][1]*pin[18][2])+((Cik[19][0][1]*pin[18][0])
          +(Cik[19][1][1]*pin[18][1])));
        Wpk[18][19][2] = ((Cik[19][2][2]*pin[18][2])+((Cik[19][0][2]*pin[18][0])
          +(Cik[19][1][2]*pin[18][1])));
        Wpk[18][20][0] = ((Cik[20][2][0]*Wpk[18][19][2])+((Cik[20][0][0]*
          Wpk[18][19][0])+(Cik[20][1][0]*Wpk[18][19][1])));
        Wpk[18][20][1] = ((Cik[20][2][1]*Wpk[18][19][2])+((Cik[20][0][1]*
          Wpk[18][19][0])+(Cik[20][1][1]*Wpk[18][19][1])));
        Wpk[18][20][2] = ((Cik[20][2][2]*Wpk[18][19][2])+((Cik[20][0][2]*
          Wpk[18][19][0])+(Cik[20][1][2]*Wpk[18][19][1])));
        Wpk[19][19][0] = pin[19][0];
        Wpk[19][19][1] = pin[19][1];
        Wpk[19][19][2] = pin[19][2];
        Wpk[19][20][0] = ((Cik[20][2][0]*pin[19][2])+((Cik[20][0][0]*pin[19][0])
          +(Cik[20][1][0]*pin[19][1])));
        Wpk[19][20][1] = ((Cik[20][2][1]*pin[19][2])+((Cik[20][0][1]*pin[19][0])
          +(Cik[20][1][1]*pin[19][1])));
        Wpk[19][20][2] = ((Cik[20][2][2]*pin[19][2])+((Cik[20][0][2]*pin[19][0])
          +(Cik[20][1][2]*pin[19][1])));
        Wpk[20][20][0] = pin[20][0];
        Wpk[20][20][1] = pin[20][1];
        Wpk[20][20][2] = pin[20][2];
/*
Compute Vpk (partial velocities)
*/
        Vpk[0][0][0] = pin[0][0];
        Vpk[0][0][1] = pin[0][1];
        Vpk[0][0][2] = pin[0][2];
        Vpk[0][1][0] = pin[0][0];
        Vpk[0][1][1] = pin[0][1];
        Vpk[0][1][2] = pin[0][2];
        Vpk[0][2][0] = pin[0][0];
        Vpk[0][2][1] = pin[0][1];
        Vpk[0][2][2] = pin[0][2];
        Vpk[0][3][0] = ((Cik[3][2][0]*pin[0][2])+((Cik[3][0][0]*pin[0][0])+(
          Cik[3][1][0]*pin[0][1])));
        Vpk[0][3][1] = ((Cik[3][2][1]*pin[0][2])+((Cik[3][0][1]*pin[0][0])+(
          Cik[3][1][1]*pin[0][1])));
        Vpk[0][3][2] = ((Cik[3][2][2]*pin[0][2])+((Cik[3][0][2]*pin[0][0])+(
          Cik[3][1][2]*pin[0][1])));
        Vpk[0][4][0] = Vpk[0][3][0];
        Vpk[0][4][1] = Vpk[0][3][1];
        Vpk[0][4][2] = Vpk[0][3][2];
        Vpk[0][5][0] = Vpk[0][3][0];
        Vpk[0][5][1] = Vpk[0][3][1];
        Vpk[0][5][2] = Vpk[0][3][2];
        Vpk[0][6][0] = ((Cik[6][2][0]*Vpk[0][3][2])+((Cik[6][0][0]*Vpk[0][3][0])
          +(Cik[6][1][0]*Vpk[0][3][1])));
        Vpk[0][6][1] = ((Cik[6][2][1]*Vpk[0][3][2])+((Cik[6][0][1]*Vpk[0][3][0])
          +(Cik[6][1][1]*Vpk[0][3][1])));
        Vpk[0][6][2] = ((Cik[6][2][2]*Vpk[0][3][2])+((Cik[6][0][2]*Vpk[0][3][0])
          +(Cik[6][1][2]*Vpk[0][3][1])));
        Vpk[0][7][0] = ((Cik[7][2][0]*Vpk[0][6][2])+((Cik[7][0][0]*Vpk[0][6][0])
          +(Cik[7][1][0]*Vpk[0][6][1])));
        Vpk[0][7][1] = ((Cik[7][2][1]*Vpk[0][6][2])+((Cik[7][0][1]*Vpk[0][6][0])
          +(Cik[7][1][1]*Vpk[0][6][1])));
        Vpk[0][7][2] = ((Cik[7][2][2]*Vpk[0][6][2])+((Cik[7][0][2]*Vpk[0][6][0])
          +(Cik[7][1][2]*Vpk[0][6][1])));
        Vpk[0][8][0] = ((Cik[8][2][0]*Vpk[0][7][2])+((Cik[8][0][0]*Vpk[0][7][0])
          +(Cik[8][1][0]*Vpk[0][7][1])));
        Vpk[0][8][1] = ((Cik[8][2][1]*Vpk[0][7][2])+((Cik[8][0][1]*Vpk[0][7][0])
          +(Cik[8][1][1]*Vpk[0][7][1])));
        Vpk[0][8][2] = ((Cik[8][2][2]*Vpk[0][7][2])+((Cik[8][0][2]*Vpk[0][7][0])
          +(Cik[8][1][2]*Vpk[0][7][1])));
        Vpk[0][9][0] = ((Cik[9][2][0]*Vpk[0][3][2])+((Cik[9][0][0]*Vpk[0][3][0])
          +(Cik[9][1][0]*Vpk[0][3][1])));
        Vpk[0][9][1] = ((Cik[9][2][1]*Vpk[0][3][2])+((Cik[9][0][1]*Vpk[0][3][0])
          +(Cik[9][1][1]*Vpk[0][3][1])));
        Vpk[0][9][2] = ((Cik[9][2][2]*Vpk[0][3][2])+((Cik[9][0][2]*Vpk[0][3][0])
          +(Cik[9][1][2]*Vpk[0][3][1])));
        Vpk[0][10][0] = ((Cik[10][2][0]*Vpk[0][9][2])+((Cik[10][0][0]*
          Vpk[0][9][0])+(Cik[10][1][0]*Vpk[0][9][1])));
        Vpk[0][10][1] = ((Cik[10][2][1]*Vpk[0][9][2])+((Cik[10][0][1]*
          Vpk[0][9][0])+(Cik[10][1][1]*Vpk[0][9][1])));
        Vpk[0][10][2] = ((Cik[10][2][2]*Vpk[0][9][2])+((Cik[10][0][2]*
          Vpk[0][9][0])+(Cik[10][1][2]*Vpk[0][9][1])));
        Vpk[0][11][0] = ((Cik[11][2][0]*Vpk[0][10][2])+((Cik[11][0][0]*
          Vpk[0][10][0])+(Cik[11][1][0]*Vpk[0][10][1])));
        Vpk[0][11][1] = ((Cik[11][2][1]*Vpk[0][10][2])+((Cik[11][0][1]*
          Vpk[0][10][0])+(Cik[11][1][1]*Vpk[0][10][1])));
        Vpk[0][11][2] = ((Cik[11][2][2]*Vpk[0][10][2])+((Cik[11][0][2]*
          Vpk[0][10][0])+(Cik[11][1][2]*Vpk[0][10][1])));
        Vpk[0][12][0] = ((Cik[12][2][0]*Vpk[0][11][2])+((Cik[12][0][0]*
          Vpk[0][11][0])+(Cik[12][1][0]*Vpk[0][11][1])));
        Vpk[0][12][1] = ((Cik[12][2][1]*Vpk[0][11][2])+((Cik[12][0][1]*
          Vpk[0][11][0])+(Cik[12][1][1]*Vpk[0][11][1])));
        Vpk[0][12][2] = ((Cik[12][2][2]*Vpk[0][11][2])+((Cik[12][0][2]*
          Vpk[0][11][0])+(Cik[12][1][2]*Vpk[0][11][1])));
        Vpk[0][13][0] = ((Cik[13][2][0]*Vpk[0][12][2])+((Cik[13][0][0]*
          Vpk[0][12][0])+(Cik[13][1][0]*Vpk[0][12][1])));
        Vpk[0][13][1] = ((Cik[13][2][1]*Vpk[0][12][2])+((Cik[13][0][1]*
          Vpk[0][12][0])+(Cik[13][1][1]*Vpk[0][12][1])));
        Vpk[0][13][2] = ((Cik[13][2][2]*Vpk[0][12][2])+((Cik[13][0][2]*
          Vpk[0][12][0])+(Cik[13][1][2]*Vpk[0][12][1])));
        Vpk[0][14][0] = ((Cik[14][2][0]*Vpk[0][13][2])+((Cik[14][0][0]*
          Vpk[0][13][0])+(Cik[14][1][0]*Vpk[0][13][1])));
        Vpk[0][14][1] = ((Cik[14][2][1]*Vpk[0][13][2])+((Cik[14][0][1]*
          Vpk[0][13][0])+(Cik[14][1][1]*Vpk[0][13][1])));
        Vpk[0][14][2] = ((Cik[14][2][2]*Vpk[0][13][2])+((Cik[14][0][2]*
          Vpk[0][13][0])+(Cik[14][1][2]*Vpk[0][13][1])));
        Vpk[0][15][0] = ((Cik[15][2][0]*Vpk[0][3][2])+((Cik[15][0][0]*
          Vpk[0][3][0])+(Cik[15][1][0]*Vpk[0][3][1])));
        Vpk[0][15][1] = ((Cik[15][2][1]*Vpk[0][3][2])+((Cik[15][0][1]*
          Vpk[0][3][0])+(Cik[15][1][1]*Vpk[0][3][1])));
        Vpk[0][15][2] = ((Cik[15][2][2]*Vpk[0][3][2])+((Cik[15][0][2]*
          Vpk[0][3][0])+(Cik[15][1][2]*Vpk[0][3][1])));
        Vpk[0][16][0] = ((Cik[16][2][0]*Vpk[0][15][2])+((Cik[16][0][0]*
          Vpk[0][15][0])+(Cik[16][1][0]*Vpk[0][15][1])));
        Vpk[0][16][1] = ((Cik[16][2][1]*Vpk[0][15][2])+((Cik[16][0][1]*
          Vpk[0][15][0])+(Cik[16][1][1]*Vpk[0][15][1])));
        Vpk[0][16][2] = ((Cik[16][2][2]*Vpk[0][15][2])+((Cik[16][0][2]*
          Vpk[0][15][0])+(Cik[16][1][2]*Vpk[0][15][1])));
        Vpk[0][17][0] = ((Cik[17][2][0]*Vpk[0][16][2])+((Cik[17][0][0]*
          Vpk[0][16][0])+(Cik[17][1][0]*Vpk[0][16][1])));
        Vpk[0][17][1] = ((Cik[17][2][1]*Vpk[0][16][2])+((Cik[17][0][1]*
          Vpk[0][16][0])+(Cik[17][1][1]*Vpk[0][16][1])));
        Vpk[0][17][2] = ((Cik[17][2][2]*Vpk[0][16][2])+((Cik[17][0][2]*
          Vpk[0][16][0])+(Cik[17][1][2]*Vpk[0][16][1])));
        Vpk[0][18][0] = ((Cik[18][2][0]*Vpk[0][17][2])+((Cik[18][0][0]*
          Vpk[0][17][0])+(Cik[18][1][0]*Vpk[0][17][1])));
        Vpk[0][18][1] = ((Cik[18][2][1]*Vpk[0][17][2])+((Cik[18][0][1]*
          Vpk[0][17][0])+(Cik[18][1][1]*Vpk[0][17][1])));
        Vpk[0][18][2] = ((Cik[18][2][2]*Vpk[0][17][2])+((Cik[18][0][2]*
          Vpk[0][17][0])+(Cik[18][1][2]*Vpk[0][17][1])));
        Vpk[0][19][0] = ((Cik[19][2][0]*Vpk[0][18][2])+((Cik[19][0][0]*
          Vpk[0][18][0])+(Cik[19][1][0]*Vpk[0][18][1])));
        Vpk[0][19][1] = ((Cik[19][2][1]*Vpk[0][18][2])+((Cik[19][0][1]*
          Vpk[0][18][0])+(Cik[19][1][1]*Vpk[0][18][1])));
        Vpk[0][19][2] = ((Cik[19][2][2]*Vpk[0][18][2])+((Cik[19][0][2]*
          Vpk[0][18][0])+(Cik[19][1][2]*Vpk[0][18][1])));
        Vpk[0][20][0] = ((Cik[20][2][0]*Vpk[0][19][2])+((Cik[20][0][0]*
          Vpk[0][19][0])+(Cik[20][1][0]*Vpk[0][19][1])));
        Vpk[0][20][1] = ((Cik[20][2][1]*Vpk[0][19][2])+((Cik[20][0][1]*
          Vpk[0][19][0])+(Cik[20][1][1]*Vpk[0][19][1])));
        Vpk[0][20][2] = ((Cik[20][2][2]*Vpk[0][19][2])+((Cik[20][0][2]*
          Vpk[0][19][0])+(Cik[20][1][2]*Vpk[0][19][1])));
        Vpk[1][1][0] = pin[1][0];
        Vpk[1][1][1] = pin[1][1];
        Vpk[1][1][2] = pin[1][2];
        Vpk[1][2][0] = pin[1][0];
        Vpk[1][2][1] = pin[1][1];
        Vpk[1][2][2] = pin[1][2];
        Vpk[1][3][0] = ((Cik[3][2][0]*pin[1][2])+((Cik[3][0][0]*pin[1][0])+(
          Cik[3][1][0]*pin[1][1])));
        Vpk[1][3][1] = ((Cik[3][2][1]*pin[1][2])+((Cik[3][0][1]*pin[1][0])+(
          Cik[3][1][1]*pin[1][1])));
        Vpk[1][3][2] = ((Cik[3][2][2]*pin[1][2])+((Cik[3][0][2]*pin[1][0])+(
          Cik[3][1][2]*pin[1][1])));
        Vpk[1][4][0] = Vpk[1][3][0];
        Vpk[1][4][1] = Vpk[1][3][1];
        Vpk[1][4][2] = Vpk[1][3][2];
        Vpk[1][5][0] = Vpk[1][3][0];
        Vpk[1][5][1] = Vpk[1][3][1];
        Vpk[1][5][2] = Vpk[1][3][2];
        Vpk[1][6][0] = ((Cik[6][2][0]*Vpk[1][3][2])+((Cik[6][0][0]*Vpk[1][3][0])
          +(Cik[6][1][0]*Vpk[1][3][1])));
        Vpk[1][6][1] = ((Cik[6][2][1]*Vpk[1][3][2])+((Cik[6][0][1]*Vpk[1][3][0])
          +(Cik[6][1][1]*Vpk[1][3][1])));
        Vpk[1][6][2] = ((Cik[6][2][2]*Vpk[1][3][2])+((Cik[6][0][2]*Vpk[1][3][0])
          +(Cik[6][1][2]*Vpk[1][3][1])));
        Vpk[1][7][0] = ((Cik[7][2][0]*Vpk[1][6][2])+((Cik[7][0][0]*Vpk[1][6][0])
          +(Cik[7][1][0]*Vpk[1][6][1])));
        Vpk[1][7][1] = ((Cik[7][2][1]*Vpk[1][6][2])+((Cik[7][0][1]*Vpk[1][6][0])
          +(Cik[7][1][1]*Vpk[1][6][1])));
        Vpk[1][7][2] = ((Cik[7][2][2]*Vpk[1][6][2])+((Cik[7][0][2]*Vpk[1][6][0])
          +(Cik[7][1][2]*Vpk[1][6][1])));
        Vpk[1][8][0] = ((Cik[8][2][0]*Vpk[1][7][2])+((Cik[8][0][0]*Vpk[1][7][0])
          +(Cik[8][1][0]*Vpk[1][7][1])));
        Vpk[1][8][1] = ((Cik[8][2][1]*Vpk[1][7][2])+((Cik[8][0][1]*Vpk[1][7][0])
          +(Cik[8][1][1]*Vpk[1][7][1])));
        Vpk[1][8][2] = ((Cik[8][2][2]*Vpk[1][7][2])+((Cik[8][0][2]*Vpk[1][7][0])
          +(Cik[8][1][2]*Vpk[1][7][1])));
        Vpk[1][9][0] = ((Cik[9][2][0]*Vpk[1][3][2])+((Cik[9][0][0]*Vpk[1][3][0])
          +(Cik[9][1][0]*Vpk[1][3][1])));
        Vpk[1][9][1] = ((Cik[9][2][1]*Vpk[1][3][2])+((Cik[9][0][1]*Vpk[1][3][0])
          +(Cik[9][1][1]*Vpk[1][3][1])));
        Vpk[1][9][2] = ((Cik[9][2][2]*Vpk[1][3][2])+((Cik[9][0][2]*Vpk[1][3][0])
          +(Cik[9][1][2]*Vpk[1][3][1])));
        Vpk[1][10][0] = ((Cik[10][2][0]*Vpk[1][9][2])+((Cik[10][0][0]*
          Vpk[1][9][0])+(Cik[10][1][0]*Vpk[1][9][1])));
        Vpk[1][10][1] = ((Cik[10][2][1]*Vpk[1][9][2])+((Cik[10][0][1]*
          Vpk[1][9][0])+(Cik[10][1][1]*Vpk[1][9][1])));
        Vpk[1][10][2] = ((Cik[10][2][2]*Vpk[1][9][2])+((Cik[10][0][2]*
          Vpk[1][9][0])+(Cik[10][1][2]*Vpk[1][9][1])));
        Vpk[1][11][0] = ((Cik[11][2][0]*Vpk[1][10][2])+((Cik[11][0][0]*
          Vpk[1][10][0])+(Cik[11][1][0]*Vpk[1][10][1])));
        Vpk[1][11][1] = ((Cik[11][2][1]*Vpk[1][10][2])+((Cik[11][0][1]*
          Vpk[1][10][0])+(Cik[11][1][1]*Vpk[1][10][1])));
        Vpk[1][11][2] = ((Cik[11][2][2]*Vpk[1][10][2])+((Cik[11][0][2]*
          Vpk[1][10][0])+(Cik[11][1][2]*Vpk[1][10][1])));
        Vpk[1][12][0] = ((Cik[12][2][0]*Vpk[1][11][2])+((Cik[12][0][0]*
          Vpk[1][11][0])+(Cik[12][1][0]*Vpk[1][11][1])));
        Vpk[1][12][1] = ((Cik[12][2][1]*Vpk[1][11][2])+((Cik[12][0][1]*
          Vpk[1][11][0])+(Cik[12][1][1]*Vpk[1][11][1])));
        Vpk[1][12][2] = ((Cik[12][2][2]*Vpk[1][11][2])+((Cik[12][0][2]*
          Vpk[1][11][0])+(Cik[12][1][2]*Vpk[1][11][1])));
        Vpk[1][13][0] = ((Cik[13][2][0]*Vpk[1][12][2])+((Cik[13][0][0]*
          Vpk[1][12][0])+(Cik[13][1][0]*Vpk[1][12][1])));
        Vpk[1][13][1] = ((Cik[13][2][1]*Vpk[1][12][2])+((Cik[13][0][1]*
          Vpk[1][12][0])+(Cik[13][1][1]*Vpk[1][12][1])));
        Vpk[1][13][2] = ((Cik[13][2][2]*Vpk[1][12][2])+((Cik[13][0][2]*
          Vpk[1][12][0])+(Cik[13][1][2]*Vpk[1][12][1])));
        Vpk[1][14][0] = ((Cik[14][2][0]*Vpk[1][13][2])+((Cik[14][0][0]*
          Vpk[1][13][0])+(Cik[14][1][0]*Vpk[1][13][1])));
        Vpk[1][14][1] = ((Cik[14][2][1]*Vpk[1][13][2])+((Cik[14][0][1]*
          Vpk[1][13][0])+(Cik[14][1][1]*Vpk[1][13][1])));
        Vpk[1][14][2] = ((Cik[14][2][2]*Vpk[1][13][2])+((Cik[14][0][2]*
          Vpk[1][13][0])+(Cik[14][1][2]*Vpk[1][13][1])));
        Vpk[1][15][0] = ((Cik[15][2][0]*Vpk[1][3][2])+((Cik[15][0][0]*
          Vpk[1][3][0])+(Cik[15][1][0]*Vpk[1][3][1])));
        Vpk[1][15][1] = ((Cik[15][2][1]*Vpk[1][3][2])+((Cik[15][0][1]*
          Vpk[1][3][0])+(Cik[15][1][1]*Vpk[1][3][1])));
        Vpk[1][15][2] = ((Cik[15][2][2]*Vpk[1][3][2])+((Cik[15][0][2]*
          Vpk[1][3][0])+(Cik[15][1][2]*Vpk[1][3][1])));
        Vpk[1][16][0] = ((Cik[16][2][0]*Vpk[1][15][2])+((Cik[16][0][0]*
          Vpk[1][15][0])+(Cik[16][1][0]*Vpk[1][15][1])));
        Vpk[1][16][1] = ((Cik[16][2][1]*Vpk[1][15][2])+((Cik[16][0][1]*
          Vpk[1][15][0])+(Cik[16][1][1]*Vpk[1][15][1])));
        Vpk[1][16][2] = ((Cik[16][2][2]*Vpk[1][15][2])+((Cik[16][0][2]*
          Vpk[1][15][0])+(Cik[16][1][2]*Vpk[1][15][1])));
        Vpk[1][17][0] = ((Cik[17][2][0]*Vpk[1][16][2])+((Cik[17][0][0]*
          Vpk[1][16][0])+(Cik[17][1][0]*Vpk[1][16][1])));
        Vpk[1][17][1] = ((Cik[17][2][1]*Vpk[1][16][2])+((Cik[17][0][1]*
          Vpk[1][16][0])+(Cik[17][1][1]*Vpk[1][16][1])));
        Vpk[1][17][2] = ((Cik[17][2][2]*Vpk[1][16][2])+((Cik[17][0][2]*
          Vpk[1][16][0])+(Cik[17][1][2]*Vpk[1][16][1])));
        Vpk[1][18][0] = ((Cik[18][2][0]*Vpk[1][17][2])+((Cik[18][0][0]*
          Vpk[1][17][0])+(Cik[18][1][0]*Vpk[1][17][1])));
        Vpk[1][18][1] = ((Cik[18][2][1]*Vpk[1][17][2])+((Cik[18][0][1]*
          Vpk[1][17][0])+(Cik[18][1][1]*Vpk[1][17][1])));
        Vpk[1][18][2] = ((Cik[18][2][2]*Vpk[1][17][2])+((Cik[18][0][2]*
          Vpk[1][17][0])+(Cik[18][1][2]*Vpk[1][17][1])));
        Vpk[1][19][0] = ((Cik[19][2][0]*Vpk[1][18][2])+((Cik[19][0][0]*
          Vpk[1][18][0])+(Cik[19][1][0]*Vpk[1][18][1])));
        Vpk[1][19][1] = ((Cik[19][2][1]*Vpk[1][18][2])+((Cik[19][0][1]*
          Vpk[1][18][0])+(Cik[19][1][1]*Vpk[1][18][1])));
        Vpk[1][19][2] = ((Cik[19][2][2]*Vpk[1][18][2])+((Cik[19][0][2]*
          Vpk[1][18][0])+(Cik[19][1][2]*Vpk[1][18][1])));
        Vpk[1][20][0] = ((Cik[20][2][0]*Vpk[1][19][2])+((Cik[20][0][0]*
          Vpk[1][19][0])+(Cik[20][1][0]*Vpk[1][19][1])));
        Vpk[1][20][1] = ((Cik[20][2][1]*Vpk[1][19][2])+((Cik[20][0][1]*
          Vpk[1][19][0])+(Cik[20][1][1]*Vpk[1][19][1])));
        Vpk[1][20][2] = ((Cik[20][2][2]*Vpk[1][19][2])+((Cik[20][0][2]*
          Vpk[1][19][0])+(Cik[20][1][2]*Vpk[1][19][1])));
        Vpk[2][2][0] = pin[2][0];
        Vpk[2][2][1] = pin[2][1];
        Vpk[2][2][2] = pin[2][2];
        Vpk[2][3][0] = ((Cik[3][2][0]*pin[2][2])+((Cik[3][0][0]*pin[2][0])+(
          Cik[3][1][0]*pin[2][1])));
        Vpk[2][3][1] = ((Cik[3][2][1]*pin[2][2])+((Cik[3][0][1]*pin[2][0])+(
          Cik[3][1][1]*pin[2][1])));
        Vpk[2][3][2] = ((Cik[3][2][2]*pin[2][2])+((Cik[3][0][2]*pin[2][0])+(
          Cik[3][1][2]*pin[2][1])));
        Vpk[2][4][0] = Vpk[2][3][0];
        Vpk[2][4][1] = Vpk[2][3][1];
        Vpk[2][4][2] = Vpk[2][3][2];
        Vpk[2][5][0] = Vpk[2][3][0];
        Vpk[2][5][1] = Vpk[2][3][1];
        Vpk[2][5][2] = Vpk[2][3][2];
        Vpk[2][6][0] = ((Cik[6][2][0]*Vpk[2][3][2])+((Cik[6][0][0]*Vpk[2][3][0])
          +(Cik[6][1][0]*Vpk[2][3][1])));
        Vpk[2][6][1] = ((Cik[6][2][1]*Vpk[2][3][2])+((Cik[6][0][1]*Vpk[2][3][0])
          +(Cik[6][1][1]*Vpk[2][3][1])));
        Vpk[2][6][2] = ((Cik[6][2][2]*Vpk[2][3][2])+((Cik[6][0][2]*Vpk[2][3][0])
          +(Cik[6][1][2]*Vpk[2][3][1])));
        Vpk[2][7][0] = ((Cik[7][2][0]*Vpk[2][6][2])+((Cik[7][0][0]*Vpk[2][6][0])
          +(Cik[7][1][0]*Vpk[2][6][1])));
        Vpk[2][7][1] = ((Cik[7][2][1]*Vpk[2][6][2])+((Cik[7][0][1]*Vpk[2][6][0])
          +(Cik[7][1][1]*Vpk[2][6][1])));
        Vpk[2][7][2] = ((Cik[7][2][2]*Vpk[2][6][2])+((Cik[7][0][2]*Vpk[2][6][0])
          +(Cik[7][1][2]*Vpk[2][6][1])));
        Vpk[2][8][0] = ((Cik[8][2][0]*Vpk[2][7][2])+((Cik[8][0][0]*Vpk[2][7][0])
          +(Cik[8][1][0]*Vpk[2][7][1])));
        Vpk[2][8][1] = ((Cik[8][2][1]*Vpk[2][7][2])+((Cik[8][0][1]*Vpk[2][7][0])
          +(Cik[8][1][1]*Vpk[2][7][1])));
        Vpk[2][8][2] = ((Cik[8][2][2]*Vpk[2][7][2])+((Cik[8][0][2]*Vpk[2][7][0])
          +(Cik[8][1][2]*Vpk[2][7][1])));
        Vpk[2][9][0] = ((Cik[9][2][0]*Vpk[2][3][2])+((Cik[9][0][0]*Vpk[2][3][0])
          +(Cik[9][1][0]*Vpk[2][3][1])));
        Vpk[2][9][1] = ((Cik[9][2][1]*Vpk[2][3][2])+((Cik[9][0][1]*Vpk[2][3][0])
          +(Cik[9][1][1]*Vpk[2][3][1])));
        Vpk[2][9][2] = ((Cik[9][2][2]*Vpk[2][3][2])+((Cik[9][0][2]*Vpk[2][3][0])
          +(Cik[9][1][2]*Vpk[2][3][1])));
        Vpk[2][10][0] = ((Cik[10][2][0]*Vpk[2][9][2])+((Cik[10][0][0]*
          Vpk[2][9][0])+(Cik[10][1][0]*Vpk[2][9][1])));
        Vpk[2][10][1] = ((Cik[10][2][1]*Vpk[2][9][2])+((Cik[10][0][1]*
          Vpk[2][9][0])+(Cik[10][1][1]*Vpk[2][9][1])));
        Vpk[2][10][2] = ((Cik[10][2][2]*Vpk[2][9][2])+((Cik[10][0][2]*
          Vpk[2][9][0])+(Cik[10][1][2]*Vpk[2][9][1])));
        Vpk[2][11][0] = ((Cik[11][2][0]*Vpk[2][10][2])+((Cik[11][0][0]*
          Vpk[2][10][0])+(Cik[11][1][0]*Vpk[2][10][1])));
        Vpk[2][11][1] = ((Cik[11][2][1]*Vpk[2][10][2])+((Cik[11][0][1]*
          Vpk[2][10][0])+(Cik[11][1][1]*Vpk[2][10][1])));
        Vpk[2][11][2] = ((Cik[11][2][2]*Vpk[2][10][2])+((Cik[11][0][2]*
          Vpk[2][10][0])+(Cik[11][1][2]*Vpk[2][10][1])));
        Vpk[2][12][0] = ((Cik[12][2][0]*Vpk[2][11][2])+((Cik[12][0][0]*
          Vpk[2][11][0])+(Cik[12][1][0]*Vpk[2][11][1])));
        Vpk[2][12][1] = ((Cik[12][2][1]*Vpk[2][11][2])+((Cik[12][0][1]*
          Vpk[2][11][0])+(Cik[12][1][1]*Vpk[2][11][1])));
        Vpk[2][12][2] = ((Cik[12][2][2]*Vpk[2][11][2])+((Cik[12][0][2]*
          Vpk[2][11][0])+(Cik[12][1][2]*Vpk[2][11][1])));
        Vpk[2][13][0] = ((Cik[13][2][0]*Vpk[2][12][2])+((Cik[13][0][0]*
          Vpk[2][12][0])+(Cik[13][1][0]*Vpk[2][12][1])));
        Vpk[2][13][1] = ((Cik[13][2][1]*Vpk[2][12][2])+((Cik[13][0][1]*
          Vpk[2][12][0])+(Cik[13][1][1]*Vpk[2][12][1])));
        Vpk[2][13][2] = ((Cik[13][2][2]*Vpk[2][12][2])+((Cik[13][0][2]*
          Vpk[2][12][0])+(Cik[13][1][2]*Vpk[2][12][1])));
        Vpk[2][14][0] = ((Cik[14][2][0]*Vpk[2][13][2])+((Cik[14][0][0]*
          Vpk[2][13][0])+(Cik[14][1][0]*Vpk[2][13][1])));
        Vpk[2][14][1] = ((Cik[14][2][1]*Vpk[2][13][2])+((Cik[14][0][1]*
          Vpk[2][13][0])+(Cik[14][1][1]*Vpk[2][13][1])));
        Vpk[2][14][2] = ((Cik[14][2][2]*Vpk[2][13][2])+((Cik[14][0][2]*
          Vpk[2][13][0])+(Cik[14][1][2]*Vpk[2][13][1])));
        Vpk[2][15][0] = ((Cik[15][2][0]*Vpk[2][3][2])+((Cik[15][0][0]*
          Vpk[2][3][0])+(Cik[15][1][0]*Vpk[2][3][1])));
        Vpk[2][15][1] = ((Cik[15][2][1]*Vpk[2][3][2])+((Cik[15][0][1]*
          Vpk[2][3][0])+(Cik[15][1][1]*Vpk[2][3][1])));
        Vpk[2][15][2] = ((Cik[15][2][2]*Vpk[2][3][2])+((Cik[15][0][2]*
          Vpk[2][3][0])+(Cik[15][1][2]*Vpk[2][3][1])));
        Vpk[2][16][0] = ((Cik[16][2][0]*Vpk[2][15][2])+((Cik[16][0][0]*
          Vpk[2][15][0])+(Cik[16][1][0]*Vpk[2][15][1])));
        Vpk[2][16][1] = ((Cik[16][2][1]*Vpk[2][15][2])+((Cik[16][0][1]*
          Vpk[2][15][0])+(Cik[16][1][1]*Vpk[2][15][1])));
        Vpk[2][16][2] = ((Cik[16][2][2]*Vpk[2][15][2])+((Cik[16][0][2]*
          Vpk[2][15][0])+(Cik[16][1][2]*Vpk[2][15][1])));
        Vpk[2][17][0] = ((Cik[17][2][0]*Vpk[2][16][2])+((Cik[17][0][0]*
          Vpk[2][16][0])+(Cik[17][1][0]*Vpk[2][16][1])));
        Vpk[2][17][1] = ((Cik[17][2][1]*Vpk[2][16][2])+((Cik[17][0][1]*
          Vpk[2][16][0])+(Cik[17][1][1]*Vpk[2][16][1])));
        Vpk[2][17][2] = ((Cik[17][2][2]*Vpk[2][16][2])+((Cik[17][0][2]*
          Vpk[2][16][0])+(Cik[17][1][2]*Vpk[2][16][1])));
        Vpk[2][18][0] = ((Cik[18][2][0]*Vpk[2][17][2])+((Cik[18][0][0]*
          Vpk[2][17][0])+(Cik[18][1][0]*Vpk[2][17][1])));
        Vpk[2][18][1] = ((Cik[18][2][1]*Vpk[2][17][2])+((Cik[18][0][1]*
          Vpk[2][17][0])+(Cik[18][1][1]*Vpk[2][17][1])));
        Vpk[2][18][2] = ((Cik[18][2][2]*Vpk[2][17][2])+((Cik[18][0][2]*
          Vpk[2][17][0])+(Cik[18][1][2]*Vpk[2][17][1])));
        Vpk[2][19][0] = ((Cik[19][2][0]*Vpk[2][18][2])+((Cik[19][0][0]*
          Vpk[2][18][0])+(Cik[19][1][0]*Vpk[2][18][1])));
        Vpk[2][19][1] = ((Cik[19][2][1]*Vpk[2][18][2])+((Cik[19][0][1]*
          Vpk[2][18][0])+(Cik[19][1][1]*Vpk[2][18][1])));
        Vpk[2][19][2] = ((Cik[19][2][2]*Vpk[2][18][2])+((Cik[19][0][2]*
          Vpk[2][18][0])+(Cik[19][1][2]*Vpk[2][18][1])));
        Vpk[2][20][0] = ((Cik[20][2][0]*Vpk[2][19][2])+((Cik[20][0][0]*
          Vpk[2][19][0])+(Cik[20][1][0]*Vpk[2][19][1])));
        Vpk[2][20][1] = ((Cik[20][2][1]*Vpk[2][19][2])+((Cik[20][0][1]*
          Vpk[2][19][0])+(Cik[20][1][1]*Vpk[2][19][1])));
        Vpk[2][20][2] = ((Cik[20][2][2]*Vpk[2][19][2])+((Cik[20][0][2]*
          Vpk[2][19][0])+(Cik[20][1][2]*Vpk[2][19][1])));
        Vpk[3][5][1] = rk[0][2];
        Vpk[3][5][2] = -rk[0][1];
        VWri[3][6][1] = (rk[0][2]-ri[1][2]);
        VWri[3][6][2] = (ri[1][1]-rk[0][1]);
        Vpk[3][6][0] = (((Cik[6][0][2]*rk[1][1])-(Cik[6][0][1]*rk[1][2]))+((
          Cik[6][1][0]*VWri[3][6][1])+(Cik[6][2][0]*VWri[3][6][2])));
        Vpk[3][6][1] = (((Cik[6][0][0]*rk[1][2])-(Cik[6][0][2]*rk[1][0]))+((
          Cik[6][1][1]*VWri[3][6][1])+(Cik[6][2][1]*VWri[3][6][2])));
        Vpk[3][6][2] = (((Cik[6][0][1]*rk[1][0])-(Cik[6][0][0]*rk[1][1]))+((
          Cik[6][1][2]*VWri[3][6][1])+(Cik[6][2][2]*VWri[3][6][2])));
        VWri[3][7][0] = (Vpk[3][6][0]+((Cik[6][0][1]*ri[2][2])-(Cik[6][0][2]*
          ri[2][1])));
        VWri[3][7][1] = (Vpk[3][6][1]+((Cik[6][0][2]*ri[2][0])-(Cik[6][0][0]*
          ri[2][2])));
        VWri[3][7][2] = (Vpk[3][6][2]+((Cik[6][0][0]*ri[2][1])-(Cik[6][0][1]*
          ri[2][0])));
        Vpk[3][7][0] = (((Cik[7][2][0]*VWri[3][7][2])+((Cik[7][0][0]*
          VWri[3][7][0])+(Cik[7][1][0]*VWri[3][7][1])))+((rk[2][1]*Wpk[3][7][2])
          -(rk[2][2]*Wpk[3][7][1])));
        Vpk[3][7][1] = (((Cik[7][2][1]*VWri[3][7][2])+((Cik[7][0][1]*
          VWri[3][7][0])+(Cik[7][1][1]*VWri[3][7][1])))+((rk[2][2]*Wpk[3][7][0])
          -(rk[2][0]*Wpk[3][7][2])));
        Vpk[3][7][2] = (((Cik[7][2][2]*VWri[3][7][2])+((Cik[7][0][2]*
          VWri[3][7][0])+(Cik[7][1][2]*VWri[3][7][1])))+((rk[2][0]*Wpk[3][7][1])
          -(rk[2][1]*Wpk[3][7][0])));
        VWri[3][8][0] = (Vpk[3][7][0]+((ri[3][2]*Wpk[3][7][1])-(ri[3][1]*
          Wpk[3][7][2])));
        VWri[3][8][1] = (Vpk[3][7][1]+((ri[3][0]*Wpk[3][7][2])-(ri[3][2]*
          Wpk[3][7][0])));
        VWri[3][8][2] = (Vpk[3][7][2]+((ri[3][1]*Wpk[3][7][0])-(ri[3][0]*
          Wpk[3][7][1])));
        Vpk[3][8][0] = (((Cik[8][2][0]*VWri[3][8][2])+((Cik[8][0][0]*
          VWri[3][8][0])+(Cik[8][1][0]*VWri[3][8][1])))+((rk[3][1]*Wpk[3][8][2])
          -(rk[3][2]*Wpk[3][8][1])));
        Vpk[3][8][1] = (((Cik[8][2][1]*VWri[3][8][2])+((Cik[8][0][1]*
          VWri[3][8][0])+(Cik[8][1][1]*VWri[3][8][1])))+((rk[3][2]*Wpk[3][8][0])
          -(rk[3][0]*Wpk[3][8][2])));
        Vpk[3][8][2] = (((Cik[8][2][2]*VWri[3][8][2])+((Cik[8][0][2]*
          VWri[3][8][0])+(Cik[8][1][2]*VWri[3][8][1])))+((rk[3][0]*Wpk[3][8][1])
          -(rk[3][1]*Wpk[3][8][0])));
        VWri[3][9][1] = (rk[0][2]-ri[4][2]);
        VWri[3][9][2] = (ri[4][1]-rk[0][1]);
        Vpk[3][9][0] = (((Cik[9][0][2]*rk[4][1])-(Cik[9][0][1]*rk[4][2]))+((
          Cik[9][1][0]*VWri[3][9][1])+(Cik[9][2][0]*VWri[3][9][2])));
        Vpk[3][9][1] = (((Cik[9][0][0]*rk[4][2])-(Cik[9][0][2]*rk[4][0]))+((
          Cik[9][1][1]*VWri[3][9][1])+(Cik[9][2][1]*VWri[3][9][2])));
        Vpk[3][9][2] = (((Cik[9][0][1]*rk[4][0])-(Cik[9][0][0]*rk[4][1]))+((
          Cik[9][1][2]*VWri[3][9][1])+(Cik[9][2][2]*VWri[3][9][2])));
        VWri[3][10][0] = (Vpk[3][9][0]+((Cik[9][0][1]*ri[5][2])-(Cik[9][0][2]*
          ri[5][1])));
        VWri[3][10][1] = (Vpk[3][9][1]+((Cik[9][0][2]*ri[5][0])-(Cik[9][0][0]*
          ri[5][2])));
        VWri[3][10][2] = (Vpk[3][9][2]+((Cik[9][0][0]*ri[5][1])-(Cik[9][0][1]*
          ri[5][0])));
        Vpk[3][10][0] = (((Cik[10][2][0]*VWri[3][10][2])+((Cik[10][0][0]*
          VWri[3][10][0])+(Cik[10][1][0]*VWri[3][10][1])))+((rk[5][1]*
          Wpk[3][10][2])-(rk[5][2]*Wpk[3][10][1])));
        Vpk[3][10][1] = (((Cik[10][2][1]*VWri[3][10][2])+((Cik[10][0][1]*
          VWri[3][10][0])+(Cik[10][1][1]*VWri[3][10][1])))+((rk[5][2]*
          Wpk[3][10][0])-(rk[5][0]*Wpk[3][10][2])));
        Vpk[3][10][2] = (((Cik[10][2][2]*VWri[3][10][2])+((Cik[10][0][2]*
          VWri[3][10][0])+(Cik[10][1][2]*VWri[3][10][1])))+((rk[5][0]*
          Wpk[3][10][1])-(rk[5][1]*Wpk[3][10][0])));
        VWri[3][11][0] = (Vpk[3][10][0]+((ri[6][2]*Wpk[3][10][1])-(ri[6][1]*
          Wpk[3][10][2])));
        VWri[3][11][1] = (Vpk[3][10][1]+((ri[6][0]*Wpk[3][10][2])-(ri[6][2]*
          Wpk[3][10][0])));
        VWri[3][11][2] = (Vpk[3][10][2]+((ri[6][1]*Wpk[3][10][0])-(ri[6][0]*
          Wpk[3][10][1])));
        Vpk[3][11][0] = (((Cik[11][2][0]*VWri[3][11][2])+((Cik[11][0][0]*
          VWri[3][11][0])+(Cik[11][1][0]*VWri[3][11][1])))+((rk[6][1]*
          Wpk[3][11][2])-(rk[6][2]*Wpk[3][11][1])));
        Vpk[3][11][1] = (((Cik[11][2][1]*VWri[3][11][2])+((Cik[11][0][1]*
          VWri[3][11][0])+(Cik[11][1][1]*VWri[3][11][1])))+((rk[6][2]*
          Wpk[3][11][0])-(rk[6][0]*Wpk[3][11][2])));
        Vpk[3][11][2] = (((Cik[11][2][2]*VWri[3][11][2])+((Cik[11][0][2]*
          VWri[3][11][0])+(Cik[11][1][2]*VWri[3][11][1])))+((rk[6][0]*
          Wpk[3][11][1])-(rk[6][1]*Wpk[3][11][0])));
        VWri[3][12][0] = (Vpk[3][11][0]+((ri[7][2]*Wpk[3][11][1])-(ri[7][1]*
          Wpk[3][11][2])));
        VWri[3][12][1] = (Vpk[3][11][1]+((ri[7][0]*Wpk[3][11][2])-(ri[7][2]*
          Wpk[3][11][0])));
        VWri[3][12][2] = (Vpk[3][11][2]+((ri[7][1]*Wpk[3][11][0])-(ri[7][0]*
          Wpk[3][11][1])));
        Vpk[3][12][0] = (((Cik[12][2][0]*VWri[3][12][2])+((Cik[12][0][0]*
          VWri[3][12][0])+(Cik[12][1][0]*VWri[3][12][1])))+((rk[7][1]*
          Wpk[3][12][2])-(rk[7][2]*Wpk[3][12][1])));
        Vpk[3][12][1] = (((Cik[12][2][1]*VWri[3][12][2])+((Cik[12][0][1]*
          VWri[3][12][0])+(Cik[12][1][1]*VWri[3][12][1])))+((rk[7][2]*
          Wpk[3][12][0])-(rk[7][0]*Wpk[3][12][2])));
        Vpk[3][12][2] = (((Cik[12][2][2]*VWri[3][12][2])+((Cik[12][0][2]*
          VWri[3][12][0])+(Cik[12][1][2]*VWri[3][12][1])))+((rk[7][0]*
          Wpk[3][12][1])-(rk[7][1]*Wpk[3][12][0])));
        VWri[3][13][0] = (Vpk[3][12][0]+((ri[8][2]*Wpk[3][12][1])-(ri[8][1]*
          Wpk[3][12][2])));
        VWri[3][13][1] = (Vpk[3][12][1]+((ri[8][0]*Wpk[3][12][2])-(ri[8][2]*
          Wpk[3][12][0])));
        VWri[3][13][2] = (Vpk[3][12][2]+((ri[8][1]*Wpk[3][12][0])-(ri[8][0]*
          Wpk[3][12][1])));
        Vpk[3][13][0] = (((Cik[13][2][0]*VWri[3][13][2])+((Cik[13][0][0]*
          VWri[3][13][0])+(Cik[13][1][0]*VWri[3][13][1])))+((rk[8][1]*
          Wpk[3][13][2])-(rk[8][2]*Wpk[3][13][1])));
        Vpk[3][13][1] = (((Cik[13][2][1]*VWri[3][13][2])+((Cik[13][0][1]*
          VWri[3][13][0])+(Cik[13][1][1]*VWri[3][13][1])))+((rk[8][2]*
          Wpk[3][13][0])-(rk[8][0]*Wpk[3][13][2])));
        Vpk[3][13][2] = (((Cik[13][2][2]*VWri[3][13][2])+((Cik[13][0][2]*
          VWri[3][13][0])+(Cik[13][1][2]*VWri[3][13][1])))+((rk[8][0]*
          Wpk[3][13][1])-(rk[8][1]*Wpk[3][13][0])));
        VWri[3][14][0] = (Vpk[3][13][0]+((ri[9][2]*Wpk[3][13][1])-(ri[9][1]*
          Wpk[3][13][2])));
        VWri[3][14][1] = (Vpk[3][13][1]+((ri[9][0]*Wpk[3][13][2])-(ri[9][2]*
          Wpk[3][13][0])));
        VWri[3][14][2] = (Vpk[3][13][2]+((ri[9][1]*Wpk[3][13][0])-(ri[9][0]*
          Wpk[3][13][1])));
        Vpk[3][14][0] = (((Cik[14][2][0]*VWri[3][14][2])+((Cik[14][0][0]*
          VWri[3][14][0])+(Cik[14][1][0]*VWri[3][14][1])))+((rk[9][1]*
          Wpk[3][14][2])-(rk[9][2]*Wpk[3][14][1])));
        Vpk[3][14][1] = (((Cik[14][2][1]*VWri[3][14][2])+((Cik[14][0][1]*
          VWri[3][14][0])+(Cik[14][1][1]*VWri[3][14][1])))+((rk[9][2]*
          Wpk[3][14][0])-(rk[9][0]*Wpk[3][14][2])));
        Vpk[3][14][2] = (((Cik[14][2][2]*VWri[3][14][2])+((Cik[14][0][2]*
          VWri[3][14][0])+(Cik[14][1][2]*VWri[3][14][1])))+((rk[9][0]*
          Wpk[3][14][1])-(rk[9][1]*Wpk[3][14][0])));
        VWri[3][15][1] = (rk[0][2]-ri[10][2]);
        VWri[3][15][2] = (ri[10][1]-rk[0][1]);
        Vpk[3][15][0] = (((Cik[15][0][2]*rk[10][1])-(Cik[15][0][1]*rk[10][2]))+(
          (Cik[15][1][0]*VWri[3][15][1])+(Cik[15][2][0]*VWri[3][15][2])));
        Vpk[3][15][1] = (((Cik[15][0][0]*rk[10][2])-(Cik[15][0][2]*rk[10][0]))+(
          (Cik[15][1][1]*VWri[3][15][1])+(Cik[15][2][1]*VWri[3][15][2])));
        Vpk[3][15][2] = (((Cik[15][0][1]*rk[10][0])-(Cik[15][0][0]*rk[10][1]))+(
          (Cik[15][1][2]*VWri[3][15][1])+(Cik[15][2][2]*VWri[3][15][2])));
        VWri[3][16][0] = (Vpk[3][15][0]+((Cik[15][0][1]*ri[11][2])-(
          Cik[15][0][2]*ri[11][1])));
        VWri[3][16][1] = (Vpk[3][15][1]+((Cik[15][0][2]*ri[11][0])-(
          Cik[15][0][0]*ri[11][2])));
        VWri[3][16][2] = (Vpk[3][15][2]+((Cik[15][0][0]*ri[11][1])-(
          Cik[15][0][1]*ri[11][0])));
        Vpk[3][16][0] = (((Cik[16][2][0]*VWri[3][16][2])+((Cik[16][0][0]*
          VWri[3][16][0])+(Cik[16][1][0]*VWri[3][16][1])))+((rk[11][1]*
          Wpk[3][16][2])-(rk[11][2]*Wpk[3][16][1])));
        Vpk[3][16][1] = (((Cik[16][2][1]*VWri[3][16][2])+((Cik[16][0][1]*
          VWri[3][16][0])+(Cik[16][1][1]*VWri[3][16][1])))+((rk[11][2]*
          Wpk[3][16][0])-(rk[11][0]*Wpk[3][16][2])));
        Vpk[3][16][2] = (((Cik[16][2][2]*VWri[3][16][2])+((Cik[16][0][2]*
          VWri[3][16][0])+(Cik[16][1][2]*VWri[3][16][1])))+((rk[11][0]*
          Wpk[3][16][1])-(rk[11][1]*Wpk[3][16][0])));
        VWri[3][17][0] = (Vpk[3][16][0]+((ri[12][2]*Wpk[3][16][1])-(ri[12][1]*
          Wpk[3][16][2])));
        VWri[3][17][1] = (Vpk[3][16][1]+((ri[12][0]*Wpk[3][16][2])-(ri[12][2]*
          Wpk[3][16][0])));
        VWri[3][17][2] = (Vpk[3][16][2]+((ri[12][1]*Wpk[3][16][0])-(ri[12][0]*
          Wpk[3][16][1])));
        Vpk[3][17][0] = (((Cik[17][2][0]*VWri[3][17][2])+((Cik[17][0][0]*
          VWri[3][17][0])+(Cik[17][1][0]*VWri[3][17][1])))+((rk[12][1]*
          Wpk[3][17][2])-(rk[12][2]*Wpk[3][17][1])));
        Vpk[3][17][1] = (((Cik[17][2][1]*VWri[3][17][2])+((Cik[17][0][1]*
          VWri[3][17][0])+(Cik[17][1][1]*VWri[3][17][1])))+((rk[12][2]*
          Wpk[3][17][0])-(rk[12][0]*Wpk[3][17][2])));
        Vpk[3][17][2] = (((Cik[17][2][2]*VWri[3][17][2])+((Cik[17][0][2]*
          VWri[3][17][0])+(Cik[17][1][2]*VWri[3][17][1])))+((rk[12][0]*
          Wpk[3][17][1])-(rk[12][1]*Wpk[3][17][0])));
        VWri[3][18][0] = (Vpk[3][17][0]+((ri[13][2]*Wpk[3][17][1])-(ri[13][1]*
          Wpk[3][17][2])));
        VWri[3][18][1] = (Vpk[3][17][1]+((ri[13][0]*Wpk[3][17][2])-(ri[13][2]*
          Wpk[3][17][0])));
        VWri[3][18][2] = (Vpk[3][17][2]+((ri[13][1]*Wpk[3][17][0])-(ri[13][0]*
          Wpk[3][17][1])));
        Vpk[3][18][0] = (((Cik[18][2][0]*VWri[3][18][2])+((Cik[18][0][0]*
          VWri[3][18][0])+(Cik[18][1][0]*VWri[3][18][1])))+((rk[13][1]*
          Wpk[3][18][2])-(rk[13][2]*Wpk[3][18][1])));
        Vpk[3][18][1] = (((Cik[18][2][1]*VWri[3][18][2])+((Cik[18][0][1]*
          VWri[3][18][0])+(Cik[18][1][1]*VWri[3][18][1])))+((rk[13][2]*
          Wpk[3][18][0])-(rk[13][0]*Wpk[3][18][2])));
        Vpk[3][18][2] = (((Cik[18][2][2]*VWri[3][18][2])+((Cik[18][0][2]*
          VWri[3][18][0])+(Cik[18][1][2]*VWri[3][18][1])))+((rk[13][0]*
          Wpk[3][18][1])-(rk[13][1]*Wpk[3][18][0])));
        VWri[3][19][0] = (Vpk[3][18][0]+((ri[14][2]*Wpk[3][18][1])-(ri[14][1]*
          Wpk[3][18][2])));
        VWri[3][19][1] = (Vpk[3][18][1]+((ri[14][0]*Wpk[3][18][2])-(ri[14][2]*
          Wpk[3][18][0])));
        VWri[3][19][2] = (Vpk[3][18][2]+((ri[14][1]*Wpk[3][18][0])-(ri[14][0]*
          Wpk[3][18][1])));
        Vpk[3][19][0] = (((Cik[19][2][0]*VWri[3][19][2])+((Cik[19][0][0]*
          VWri[3][19][0])+(Cik[19][1][0]*VWri[3][19][1])))+((rk[14][1]*
          Wpk[3][19][2])-(rk[14][2]*Wpk[3][19][1])));
        Vpk[3][19][1] = (((Cik[19][2][1]*VWri[3][19][2])+((Cik[19][0][1]*
          VWri[3][19][0])+(Cik[19][1][1]*VWri[3][19][1])))+((rk[14][2]*
          Wpk[3][19][0])-(rk[14][0]*Wpk[3][19][2])));
        Vpk[3][19][2] = (((Cik[19][2][2]*VWri[3][19][2])+((Cik[19][0][2]*
          VWri[3][19][0])+(Cik[19][1][2]*VWri[3][19][1])))+((rk[14][0]*
          Wpk[3][19][1])-(rk[14][1]*Wpk[3][19][0])));
        VWri[3][20][0] = (Vpk[3][19][0]+((ri[15][2]*Wpk[3][19][1])-(ri[15][1]*
          Wpk[3][19][2])));
        VWri[3][20][1] = (Vpk[3][19][1]+((ri[15][0]*Wpk[3][19][2])-(ri[15][2]*
          Wpk[3][19][0])));
        VWri[3][20][2] = (Vpk[3][19][2]+((ri[15][1]*Wpk[3][19][0])-(ri[15][0]*
          Wpk[3][19][1])));
        Vpk[3][20][0] = (((Cik[20][2][0]*VWri[3][20][2])+((Cik[20][0][0]*
          VWri[3][20][0])+(Cik[20][1][0]*VWri[3][20][1])))+((rk[15][1]*
          Wpk[3][20][2])-(rk[15][2]*Wpk[3][20][1])));
        Vpk[3][20][1] = (((Cik[20][2][1]*VWri[3][20][2])+((Cik[20][0][1]*
          VWri[3][20][0])+(Cik[20][1][1]*VWri[3][20][1])))+((rk[15][2]*
          Wpk[3][20][0])-(rk[15][0]*Wpk[3][20][2])));
        Vpk[3][20][2] = (((Cik[20][2][2]*VWri[3][20][2])+((Cik[20][0][2]*
          VWri[3][20][0])+(Cik[20][1][2]*VWri[3][20][1])))+((rk[15][0]*
          Wpk[3][20][1])-(rk[15][1]*Wpk[3][20][0])));
        Vpk[4][5][0] = -rk[0][2];
        Vpk[4][5][2] = rk[0][0];
        VWri[4][6][0] = (ri[1][2]-rk[0][2]);
        VWri[4][6][2] = (rk[0][0]-ri[1][0]);
        Vpk[4][6][0] = (((Cik[6][0][0]*VWri[4][6][0])+(Cik[6][2][0]*
          VWri[4][6][2]))+((Cik[6][1][2]*rk[1][1])-(Cik[6][1][1]*rk[1][2])));
        Vpk[4][6][1] = (((Cik[6][0][1]*VWri[4][6][0])+(Cik[6][2][1]*
          VWri[4][6][2]))+((Cik[6][1][0]*rk[1][2])-(Cik[6][1][2]*rk[1][0])));
        Vpk[4][6][2] = (((Cik[6][0][2]*VWri[4][6][0])+(Cik[6][2][2]*
          VWri[4][6][2]))+((Cik[6][1][1]*rk[1][0])-(Cik[6][1][0]*rk[1][1])));
        VWri[4][7][0] = (Vpk[4][6][0]+((Cik[6][1][1]*ri[2][2])-(Cik[6][1][2]*
          ri[2][1])));
        VWri[4][7][1] = (Vpk[4][6][1]+((Cik[6][1][2]*ri[2][0])-(Cik[6][1][0]*
          ri[2][2])));
        VWri[4][7][2] = (Vpk[4][6][2]+((Cik[6][1][0]*ri[2][1])-(Cik[6][1][1]*
          ri[2][0])));
        Vpk[4][7][0] = (((Cik[7][2][0]*VWri[4][7][2])+((Cik[7][0][0]*
          VWri[4][7][0])+(Cik[7][1][0]*VWri[4][7][1])))+((rk[2][1]*Wpk[4][7][2])
          -(rk[2][2]*Wpk[4][7][1])));
        Vpk[4][7][1] = (((Cik[7][2][1]*VWri[4][7][2])+((Cik[7][0][1]*
          VWri[4][7][0])+(Cik[7][1][1]*VWri[4][7][1])))+((rk[2][2]*Wpk[4][7][0])
          -(rk[2][0]*Wpk[4][7][2])));
        Vpk[4][7][2] = (((Cik[7][2][2]*VWri[4][7][2])+((Cik[7][0][2]*
          VWri[4][7][0])+(Cik[7][1][2]*VWri[4][7][1])))+((rk[2][0]*Wpk[4][7][1])
          -(rk[2][1]*Wpk[4][7][0])));
        VWri[4][8][0] = (Vpk[4][7][0]+((ri[3][2]*Wpk[4][7][1])-(ri[3][1]*
          Wpk[4][7][2])));
        VWri[4][8][1] = (Vpk[4][7][1]+((ri[3][0]*Wpk[4][7][2])-(ri[3][2]*
          Wpk[4][7][0])));
        VWri[4][8][2] = (Vpk[4][7][2]+((ri[3][1]*Wpk[4][7][0])-(ri[3][0]*
          Wpk[4][7][1])));
        Vpk[4][8][0] = (((Cik[8][2][0]*VWri[4][8][2])+((Cik[8][0][0]*
          VWri[4][8][0])+(Cik[8][1][0]*VWri[4][8][1])))+((rk[3][1]*Wpk[4][8][2])
          -(rk[3][2]*Wpk[4][8][1])));
        Vpk[4][8][1] = (((Cik[8][2][1]*VWri[4][8][2])+((Cik[8][0][1]*
          VWri[4][8][0])+(Cik[8][1][1]*VWri[4][8][1])))+((rk[3][2]*Wpk[4][8][0])
          -(rk[3][0]*Wpk[4][8][2])));
        Vpk[4][8][2] = (((Cik[8][2][2]*VWri[4][8][2])+((Cik[8][0][2]*
          VWri[4][8][0])+(Cik[8][1][2]*VWri[4][8][1])))+((rk[3][0]*Wpk[4][8][1])
          -(rk[3][1]*Wpk[4][8][0])));
        VWri[4][9][0] = (ri[4][2]-rk[0][2]);
        VWri[4][9][2] = (rk[0][0]-ri[4][0]);
        Vpk[4][9][0] = (((Cik[9][0][0]*VWri[4][9][0])+(Cik[9][2][0]*
          VWri[4][9][2]))+((Cik[9][1][2]*rk[4][1])-(Cik[9][1][1]*rk[4][2])));
        Vpk[4][9][1] = (((Cik[9][0][1]*VWri[4][9][0])+(Cik[9][2][1]*
          VWri[4][9][2]))+((Cik[9][1][0]*rk[4][2])-(Cik[9][1][2]*rk[4][0])));
        Vpk[4][9][2] = (((Cik[9][0][2]*VWri[4][9][0])+(Cik[9][2][2]*
          VWri[4][9][2]))+((Cik[9][1][1]*rk[4][0])-(Cik[9][1][0]*rk[4][1])));
        VWri[4][10][0] = (Vpk[4][9][0]+((Cik[9][1][1]*ri[5][2])-(Cik[9][1][2]*
          ri[5][1])));
        VWri[4][10][1] = (Vpk[4][9][1]+((Cik[9][1][2]*ri[5][0])-(Cik[9][1][0]*
          ri[5][2])));
        VWri[4][10][2] = (Vpk[4][9][2]+((Cik[9][1][0]*ri[5][1])-(Cik[9][1][1]*
          ri[5][0])));
        Vpk[4][10][0] = (((Cik[10][2][0]*VWri[4][10][2])+((Cik[10][0][0]*
          VWri[4][10][0])+(Cik[10][1][0]*VWri[4][10][1])))+((rk[5][1]*
          Wpk[4][10][2])-(rk[5][2]*Wpk[4][10][1])));
        Vpk[4][10][1] = (((Cik[10][2][1]*VWri[4][10][2])+((Cik[10][0][1]*
          VWri[4][10][0])+(Cik[10][1][1]*VWri[4][10][1])))+((rk[5][2]*
          Wpk[4][10][0])-(rk[5][0]*Wpk[4][10][2])));
        Vpk[4][10][2] = (((Cik[10][2][2]*VWri[4][10][2])+((Cik[10][0][2]*
          VWri[4][10][0])+(Cik[10][1][2]*VWri[4][10][1])))+((rk[5][0]*
          Wpk[4][10][1])-(rk[5][1]*Wpk[4][10][0])));
        VWri[4][11][0] = (Vpk[4][10][0]+((ri[6][2]*Wpk[4][10][1])-(ri[6][1]*
          Wpk[4][10][2])));
        VWri[4][11][1] = (Vpk[4][10][1]+((ri[6][0]*Wpk[4][10][2])-(ri[6][2]*
          Wpk[4][10][0])));
        VWri[4][11][2] = (Vpk[4][10][2]+((ri[6][1]*Wpk[4][10][0])-(ri[6][0]*
          Wpk[4][10][1])));
        Vpk[4][11][0] = (((Cik[11][2][0]*VWri[4][11][2])+((Cik[11][0][0]*
          VWri[4][11][0])+(Cik[11][1][0]*VWri[4][11][1])))+((rk[6][1]*
          Wpk[4][11][2])-(rk[6][2]*Wpk[4][11][1])));
        Vpk[4][11][1] = (((Cik[11][2][1]*VWri[4][11][2])+((Cik[11][0][1]*
          VWri[4][11][0])+(Cik[11][1][1]*VWri[4][11][1])))+((rk[6][2]*
          Wpk[4][11][0])-(rk[6][0]*Wpk[4][11][2])));
        Vpk[4][11][2] = (((Cik[11][2][2]*VWri[4][11][2])+((Cik[11][0][2]*
          VWri[4][11][0])+(Cik[11][1][2]*VWri[4][11][1])))+((rk[6][0]*
          Wpk[4][11][1])-(rk[6][1]*Wpk[4][11][0])));
        VWri[4][12][0] = (Vpk[4][11][0]+((ri[7][2]*Wpk[4][11][1])-(ri[7][1]*
          Wpk[4][11][2])));
        VWri[4][12][1] = (Vpk[4][11][1]+((ri[7][0]*Wpk[4][11][2])-(ri[7][2]*
          Wpk[4][11][0])));
        VWri[4][12][2] = (Vpk[4][11][2]+((ri[7][1]*Wpk[4][11][0])-(ri[7][0]*
          Wpk[4][11][1])));
        Vpk[4][12][0] = (((Cik[12][2][0]*VWri[4][12][2])+((Cik[12][0][0]*
          VWri[4][12][0])+(Cik[12][1][0]*VWri[4][12][1])))+((rk[7][1]*
          Wpk[4][12][2])-(rk[7][2]*Wpk[4][12][1])));
        Vpk[4][12][1] = (((Cik[12][2][1]*VWri[4][12][2])+((Cik[12][0][1]*
          VWri[4][12][0])+(Cik[12][1][1]*VWri[4][12][1])))+((rk[7][2]*
          Wpk[4][12][0])-(rk[7][0]*Wpk[4][12][2])));
        Vpk[4][12][2] = (((Cik[12][2][2]*VWri[4][12][2])+((Cik[12][0][2]*
          VWri[4][12][0])+(Cik[12][1][2]*VWri[4][12][1])))+((rk[7][0]*
          Wpk[4][12][1])-(rk[7][1]*Wpk[4][12][0])));
        VWri[4][13][0] = (Vpk[4][12][0]+((ri[8][2]*Wpk[4][12][1])-(ri[8][1]*
          Wpk[4][12][2])));
        VWri[4][13][1] = (Vpk[4][12][1]+((ri[8][0]*Wpk[4][12][2])-(ri[8][2]*
          Wpk[4][12][0])));
        VWri[4][13][2] = (Vpk[4][12][2]+((ri[8][1]*Wpk[4][12][0])-(ri[8][0]*
          Wpk[4][12][1])));
        Vpk[4][13][0] = (((Cik[13][2][0]*VWri[4][13][2])+((Cik[13][0][0]*
          VWri[4][13][0])+(Cik[13][1][0]*VWri[4][13][1])))+((rk[8][1]*
          Wpk[4][13][2])-(rk[8][2]*Wpk[4][13][1])));
        Vpk[4][13][1] = (((Cik[13][2][1]*VWri[4][13][2])+((Cik[13][0][1]*
          VWri[4][13][0])+(Cik[13][1][1]*VWri[4][13][1])))+((rk[8][2]*
          Wpk[4][13][0])-(rk[8][0]*Wpk[4][13][2])));
        Vpk[4][13][2] = (((Cik[13][2][2]*VWri[4][13][2])+((Cik[13][0][2]*
          VWri[4][13][0])+(Cik[13][1][2]*VWri[4][13][1])))+((rk[8][0]*
          Wpk[4][13][1])-(rk[8][1]*Wpk[4][13][0])));
        VWri[4][14][0] = (Vpk[4][13][0]+((ri[9][2]*Wpk[4][13][1])-(ri[9][1]*
          Wpk[4][13][2])));
        VWri[4][14][1] = (Vpk[4][13][1]+((ri[9][0]*Wpk[4][13][2])-(ri[9][2]*
          Wpk[4][13][0])));
        VWri[4][14][2] = (Vpk[4][13][2]+((ri[9][1]*Wpk[4][13][0])-(ri[9][0]*
          Wpk[4][13][1])));
        Vpk[4][14][0] = (((Cik[14][2][0]*VWri[4][14][2])+((Cik[14][0][0]*
          VWri[4][14][0])+(Cik[14][1][0]*VWri[4][14][1])))+((rk[9][1]*
          Wpk[4][14][2])-(rk[9][2]*Wpk[4][14][1])));
        Vpk[4][14][1] = (((Cik[14][2][1]*VWri[4][14][2])+((Cik[14][0][1]*
          VWri[4][14][0])+(Cik[14][1][1]*VWri[4][14][1])))+((rk[9][2]*
          Wpk[4][14][0])-(rk[9][0]*Wpk[4][14][2])));
        Vpk[4][14][2] = (((Cik[14][2][2]*VWri[4][14][2])+((Cik[14][0][2]*
          VWri[4][14][0])+(Cik[14][1][2]*VWri[4][14][1])))+((rk[9][0]*
          Wpk[4][14][1])-(rk[9][1]*Wpk[4][14][0])));
        VWri[4][15][0] = (ri[10][2]-rk[0][2]);
        VWri[4][15][2] = (rk[0][0]-ri[10][0]);
        Vpk[4][15][0] = (((Cik[15][0][0]*VWri[4][15][0])+(Cik[15][2][0]*
          VWri[4][15][2]))+((Cik[15][1][2]*rk[10][1])-(Cik[15][1][1]*rk[10][2]))
          );
        Vpk[4][15][1] = (((Cik[15][0][1]*VWri[4][15][0])+(Cik[15][2][1]*
          VWri[4][15][2]))+((Cik[15][1][0]*rk[10][2])-(Cik[15][1][2]*rk[10][0]))
          );
        Vpk[4][15][2] = (((Cik[15][0][2]*VWri[4][15][0])+(Cik[15][2][2]*
          VWri[4][15][2]))+((Cik[15][1][1]*rk[10][0])-(Cik[15][1][0]*rk[10][1]))
          );
        VWri[4][16][0] = (Vpk[4][15][0]+((Cik[15][1][1]*ri[11][2])-(
          Cik[15][1][2]*ri[11][1])));
        VWri[4][16][1] = (Vpk[4][15][1]+((Cik[15][1][2]*ri[11][0])-(
          Cik[15][1][0]*ri[11][2])));
        VWri[4][16][2] = (Vpk[4][15][2]+((Cik[15][1][0]*ri[11][1])-(
          Cik[15][1][1]*ri[11][0])));
        Vpk[4][16][0] = (((Cik[16][2][0]*VWri[4][16][2])+((Cik[16][0][0]*
          VWri[4][16][0])+(Cik[16][1][0]*VWri[4][16][1])))+((rk[11][1]*
          Wpk[4][16][2])-(rk[11][2]*Wpk[4][16][1])));
        Vpk[4][16][1] = (((Cik[16][2][1]*VWri[4][16][2])+((Cik[16][0][1]*
          VWri[4][16][0])+(Cik[16][1][1]*VWri[4][16][1])))+((rk[11][2]*
          Wpk[4][16][0])-(rk[11][0]*Wpk[4][16][2])));
        Vpk[4][16][2] = (((Cik[16][2][2]*VWri[4][16][2])+((Cik[16][0][2]*
          VWri[4][16][0])+(Cik[16][1][2]*VWri[4][16][1])))+((rk[11][0]*
          Wpk[4][16][1])-(rk[11][1]*Wpk[4][16][0])));
        VWri[4][17][0] = (Vpk[4][16][0]+((ri[12][2]*Wpk[4][16][1])-(ri[12][1]*
          Wpk[4][16][2])));
        VWri[4][17][1] = (Vpk[4][16][1]+((ri[12][0]*Wpk[4][16][2])-(ri[12][2]*
          Wpk[4][16][0])));
        VWri[4][17][2] = (Vpk[4][16][2]+((ri[12][1]*Wpk[4][16][0])-(ri[12][0]*
          Wpk[4][16][1])));
        Vpk[4][17][0] = (((Cik[17][2][0]*VWri[4][17][2])+((Cik[17][0][0]*
          VWri[4][17][0])+(Cik[17][1][0]*VWri[4][17][1])))+((rk[12][1]*
          Wpk[4][17][2])-(rk[12][2]*Wpk[4][17][1])));
        Vpk[4][17][1] = (((Cik[17][2][1]*VWri[4][17][2])+((Cik[17][0][1]*
          VWri[4][17][0])+(Cik[17][1][1]*VWri[4][17][1])))+((rk[12][2]*
          Wpk[4][17][0])-(rk[12][0]*Wpk[4][17][2])));
        Vpk[4][17][2] = (((Cik[17][2][2]*VWri[4][17][2])+((Cik[17][0][2]*
          VWri[4][17][0])+(Cik[17][1][2]*VWri[4][17][1])))+((rk[12][0]*
          Wpk[4][17][1])-(rk[12][1]*Wpk[4][17][0])));
        VWri[4][18][0] = (Vpk[4][17][0]+((ri[13][2]*Wpk[4][17][1])-(ri[13][1]*
          Wpk[4][17][2])));
        VWri[4][18][1] = (Vpk[4][17][1]+((ri[13][0]*Wpk[4][17][2])-(ri[13][2]*
          Wpk[4][17][0])));
        VWri[4][18][2] = (Vpk[4][17][2]+((ri[13][1]*Wpk[4][17][0])-(ri[13][0]*
          Wpk[4][17][1])));
        Vpk[4][18][0] = (((Cik[18][2][0]*VWri[4][18][2])+((Cik[18][0][0]*
          VWri[4][18][0])+(Cik[18][1][0]*VWri[4][18][1])))+((rk[13][1]*
          Wpk[4][18][2])-(rk[13][2]*Wpk[4][18][1])));
        Vpk[4][18][1] = (((Cik[18][2][1]*VWri[4][18][2])+((Cik[18][0][1]*
          VWri[4][18][0])+(Cik[18][1][1]*VWri[4][18][1])))+((rk[13][2]*
          Wpk[4][18][0])-(rk[13][0]*Wpk[4][18][2])));
        Vpk[4][18][2] = (((Cik[18][2][2]*VWri[4][18][2])+((Cik[18][0][2]*
          VWri[4][18][0])+(Cik[18][1][2]*VWri[4][18][1])))+((rk[13][0]*
          Wpk[4][18][1])-(rk[13][1]*Wpk[4][18][0])));
        VWri[4][19][0] = (Vpk[4][18][0]+((ri[14][2]*Wpk[4][18][1])-(ri[14][1]*
          Wpk[4][18][2])));
        VWri[4][19][1] = (Vpk[4][18][1]+((ri[14][0]*Wpk[4][18][2])-(ri[14][2]*
          Wpk[4][18][0])));
        VWri[4][19][2] = (Vpk[4][18][2]+((ri[14][1]*Wpk[4][18][0])-(ri[14][0]*
          Wpk[4][18][1])));
        Vpk[4][19][0] = (((Cik[19][2][0]*VWri[4][19][2])+((Cik[19][0][0]*
          VWri[4][19][0])+(Cik[19][1][0]*VWri[4][19][1])))+((rk[14][1]*
          Wpk[4][19][2])-(rk[14][2]*Wpk[4][19][1])));
        Vpk[4][19][1] = (((Cik[19][2][1]*VWri[4][19][2])+((Cik[19][0][1]*
          VWri[4][19][0])+(Cik[19][1][1]*VWri[4][19][1])))+((rk[14][2]*
          Wpk[4][19][0])-(rk[14][0]*Wpk[4][19][2])));
        Vpk[4][19][2] = (((Cik[19][2][2]*VWri[4][19][2])+((Cik[19][0][2]*
          VWri[4][19][0])+(Cik[19][1][2]*VWri[4][19][1])))+((rk[14][0]*
          Wpk[4][19][1])-(rk[14][1]*Wpk[4][19][0])));
        VWri[4][20][0] = (Vpk[4][19][0]+((ri[15][2]*Wpk[4][19][1])-(ri[15][1]*
          Wpk[4][19][2])));
        VWri[4][20][1] = (Vpk[4][19][1]+((ri[15][0]*Wpk[4][19][2])-(ri[15][2]*
          Wpk[4][19][0])));
        VWri[4][20][2] = (Vpk[4][19][2]+((ri[15][1]*Wpk[4][19][0])-(ri[15][0]*
          Wpk[4][19][1])));
        Vpk[4][20][0] = (((Cik[20][2][0]*VWri[4][20][2])+((Cik[20][0][0]*
          VWri[4][20][0])+(Cik[20][1][0]*VWri[4][20][1])))+((rk[15][1]*
          Wpk[4][20][2])-(rk[15][2]*Wpk[4][20][1])));
        Vpk[4][20][1] = (((Cik[20][2][1]*VWri[4][20][2])+((Cik[20][0][1]*
          VWri[4][20][0])+(Cik[20][1][1]*VWri[4][20][1])))+((rk[15][2]*
          Wpk[4][20][0])-(rk[15][0]*Wpk[4][20][2])));
        Vpk[4][20][2] = (((Cik[20][2][2]*VWri[4][20][2])+((Cik[20][0][2]*
          VWri[4][20][0])+(Cik[20][1][2]*VWri[4][20][1])))+((rk[15][0]*
          Wpk[4][20][1])-(rk[15][1]*Wpk[4][20][0])));
        Vpk[5][5][0] = rk[0][1];
        Vpk[5][5][1] = -rk[0][0];
        VWri[5][6][0] = (rk[0][1]-ri[1][1]);
        VWri[5][6][1] = (ri[1][0]-rk[0][0]);
        Vpk[5][6][0] = (((Cik[6][0][0]*VWri[5][6][0])+(Cik[6][1][0]*
          VWri[5][6][1]))+((Cik[6][2][2]*rk[1][1])-(Cik[6][2][1]*rk[1][2])));
        Vpk[5][6][1] = (((Cik[6][0][1]*VWri[5][6][0])+(Cik[6][1][1]*
          VWri[5][6][1]))+((Cik[6][2][0]*rk[1][2])-(Cik[6][2][2]*rk[1][0])));
        Vpk[5][6][2] = (((Cik[6][0][2]*VWri[5][6][0])+(Cik[6][1][2]*
          VWri[5][6][1]))+((Cik[6][2][1]*rk[1][0])-(Cik[6][2][0]*rk[1][1])));
        VWri[5][7][0] = (Vpk[5][6][0]+((Cik[6][2][1]*ri[2][2])-(Cik[6][2][2]*
          ri[2][1])));
        VWri[5][7][1] = (Vpk[5][6][1]+((Cik[6][2][2]*ri[2][0])-(Cik[6][2][0]*
          ri[2][2])));
        VWri[5][7][2] = (Vpk[5][6][2]+((Cik[6][2][0]*ri[2][1])-(Cik[6][2][1]*
          ri[2][0])));
        Vpk[5][7][0] = (((Cik[7][2][0]*VWri[5][7][2])+((Cik[7][0][0]*
          VWri[5][7][0])+(Cik[7][1][0]*VWri[5][7][1])))+((rk[2][1]*Wpk[5][7][2])
          -(rk[2][2]*Wpk[5][7][1])));
        Vpk[5][7][1] = (((Cik[7][2][1]*VWri[5][7][2])+((Cik[7][0][1]*
          VWri[5][7][0])+(Cik[7][1][1]*VWri[5][7][1])))+((rk[2][2]*Wpk[5][7][0])
          -(rk[2][0]*Wpk[5][7][2])));
        Vpk[5][7][2] = (((Cik[7][2][2]*VWri[5][7][2])+((Cik[7][0][2]*
          VWri[5][7][0])+(Cik[7][1][2]*VWri[5][7][1])))+((rk[2][0]*Wpk[5][7][1])
          -(rk[2][1]*Wpk[5][7][0])));
        VWri[5][8][0] = (Vpk[5][7][0]+((ri[3][2]*Wpk[5][7][1])-(ri[3][1]*
          Wpk[5][7][2])));
        VWri[5][8][1] = (Vpk[5][7][1]+((ri[3][0]*Wpk[5][7][2])-(ri[3][2]*
          Wpk[5][7][0])));
        VWri[5][8][2] = (Vpk[5][7][2]+((ri[3][1]*Wpk[5][7][0])-(ri[3][0]*
          Wpk[5][7][1])));
        Vpk[5][8][0] = (((Cik[8][2][0]*VWri[5][8][2])+((Cik[8][0][0]*
          VWri[5][8][0])+(Cik[8][1][0]*VWri[5][8][1])))+((rk[3][1]*Wpk[5][8][2])
          -(rk[3][2]*Wpk[5][8][1])));
        Vpk[5][8][1] = (((Cik[8][2][1]*VWri[5][8][2])+((Cik[8][0][1]*
          VWri[5][8][0])+(Cik[8][1][1]*VWri[5][8][1])))+((rk[3][2]*Wpk[5][8][0])
          -(rk[3][0]*Wpk[5][8][2])));
        Vpk[5][8][2] = (((Cik[8][2][2]*VWri[5][8][2])+((Cik[8][0][2]*
          VWri[5][8][0])+(Cik[8][1][2]*VWri[5][8][1])))+((rk[3][0]*Wpk[5][8][1])
          -(rk[3][1]*Wpk[5][8][0])));
        VWri[5][9][0] = (rk[0][1]-ri[4][1]);
        VWri[5][9][1] = (ri[4][0]-rk[0][0]);
        Vpk[5][9][0] = (((Cik[9][0][0]*VWri[5][9][0])+(Cik[9][1][0]*
          VWri[5][9][1]))+((Cik[9][2][2]*rk[4][1])-(Cik[9][2][1]*rk[4][2])));
        Vpk[5][9][1] = (((Cik[9][0][1]*VWri[5][9][0])+(Cik[9][1][1]*
          VWri[5][9][1]))+((Cik[9][2][0]*rk[4][2])-(Cik[9][2][2]*rk[4][0])));
        Vpk[5][9][2] = (((Cik[9][0][2]*VWri[5][9][0])+(Cik[9][1][2]*
          VWri[5][9][1]))+((Cik[9][2][1]*rk[4][0])-(Cik[9][2][0]*rk[4][1])));
        VWri[5][10][0] = (Vpk[5][9][0]+((Cik[9][2][1]*ri[5][2])-(Cik[9][2][2]*
          ri[5][1])));
        VWri[5][10][1] = (Vpk[5][9][1]+((Cik[9][2][2]*ri[5][0])-(Cik[9][2][0]*
          ri[5][2])));
        VWri[5][10][2] = (Vpk[5][9][2]+((Cik[9][2][0]*ri[5][1])-(Cik[9][2][1]*
          ri[5][0])));
        Vpk[5][10][0] = (((Cik[10][2][0]*VWri[5][10][2])+((Cik[10][0][0]*
          VWri[5][10][0])+(Cik[10][1][0]*VWri[5][10][1])))+((rk[5][1]*
          Wpk[5][10][2])-(rk[5][2]*Wpk[5][10][1])));
        Vpk[5][10][1] = (((Cik[10][2][1]*VWri[5][10][2])+((Cik[10][0][1]*
          VWri[5][10][0])+(Cik[10][1][1]*VWri[5][10][1])))+((rk[5][2]*
          Wpk[5][10][0])-(rk[5][0]*Wpk[5][10][2])));
        Vpk[5][10][2] = (((Cik[10][2][2]*VWri[5][10][2])+((Cik[10][0][2]*
          VWri[5][10][0])+(Cik[10][1][2]*VWri[5][10][1])))+((rk[5][0]*
          Wpk[5][10][1])-(rk[5][1]*Wpk[5][10][0])));
        VWri[5][11][0] = (Vpk[5][10][0]+((ri[6][2]*Wpk[5][10][1])-(ri[6][1]*
          Wpk[5][10][2])));
        VWri[5][11][1] = (Vpk[5][10][1]+((ri[6][0]*Wpk[5][10][2])-(ri[6][2]*
          Wpk[5][10][0])));
        VWri[5][11][2] = (Vpk[5][10][2]+((ri[6][1]*Wpk[5][10][0])-(ri[6][0]*
          Wpk[5][10][1])));
        Vpk[5][11][0] = (((Cik[11][2][0]*VWri[5][11][2])+((Cik[11][0][0]*
          VWri[5][11][0])+(Cik[11][1][0]*VWri[5][11][1])))+((rk[6][1]*
          Wpk[5][11][2])-(rk[6][2]*Wpk[5][11][1])));
        Vpk[5][11][1] = (((Cik[11][2][1]*VWri[5][11][2])+((Cik[11][0][1]*
          VWri[5][11][0])+(Cik[11][1][1]*VWri[5][11][1])))+((rk[6][2]*
          Wpk[5][11][0])-(rk[6][0]*Wpk[5][11][2])));
        Vpk[5][11][2] = (((Cik[11][2][2]*VWri[5][11][2])+((Cik[11][0][2]*
          VWri[5][11][0])+(Cik[11][1][2]*VWri[5][11][1])))+((rk[6][0]*
          Wpk[5][11][1])-(rk[6][1]*Wpk[5][11][0])));
        VWri[5][12][0] = (Vpk[5][11][0]+((ri[7][2]*Wpk[5][11][1])-(ri[7][1]*
          Wpk[5][11][2])));
        VWri[5][12][1] = (Vpk[5][11][1]+((ri[7][0]*Wpk[5][11][2])-(ri[7][2]*
          Wpk[5][11][0])));
        VWri[5][12][2] = (Vpk[5][11][2]+((ri[7][1]*Wpk[5][11][0])-(ri[7][0]*
          Wpk[5][11][1])));
        Vpk[5][12][0] = (((Cik[12][2][0]*VWri[5][12][2])+((Cik[12][0][0]*
          VWri[5][12][0])+(Cik[12][1][0]*VWri[5][12][1])))+((rk[7][1]*
          Wpk[5][12][2])-(rk[7][2]*Wpk[5][12][1])));
        Vpk[5][12][1] = (((Cik[12][2][1]*VWri[5][12][2])+((Cik[12][0][1]*
          VWri[5][12][0])+(Cik[12][1][1]*VWri[5][12][1])))+((rk[7][2]*
          Wpk[5][12][0])-(rk[7][0]*Wpk[5][12][2])));
        Vpk[5][12][2] = (((Cik[12][2][2]*VWri[5][12][2])+((Cik[12][0][2]*
          VWri[5][12][0])+(Cik[12][1][2]*VWri[5][12][1])))+((rk[7][0]*
          Wpk[5][12][1])-(rk[7][1]*Wpk[5][12][0])));
        VWri[5][13][0] = (Vpk[5][12][0]+((ri[8][2]*Wpk[5][12][1])-(ri[8][1]*
          Wpk[5][12][2])));
        VWri[5][13][1] = (Vpk[5][12][1]+((ri[8][0]*Wpk[5][12][2])-(ri[8][2]*
          Wpk[5][12][0])));
        VWri[5][13][2] = (Vpk[5][12][2]+((ri[8][1]*Wpk[5][12][0])-(ri[8][0]*
          Wpk[5][12][1])));
        Vpk[5][13][0] = (((Cik[13][2][0]*VWri[5][13][2])+((Cik[13][0][0]*
          VWri[5][13][0])+(Cik[13][1][0]*VWri[5][13][1])))+((rk[8][1]*
          Wpk[5][13][2])-(rk[8][2]*Wpk[5][13][1])));
        Vpk[5][13][1] = (((Cik[13][2][1]*VWri[5][13][2])+((Cik[13][0][1]*
          VWri[5][13][0])+(Cik[13][1][1]*VWri[5][13][1])))+((rk[8][2]*
          Wpk[5][13][0])-(rk[8][0]*Wpk[5][13][2])));
        Vpk[5][13][2] = (((Cik[13][2][2]*VWri[5][13][2])+((Cik[13][0][2]*
          VWri[5][13][0])+(Cik[13][1][2]*VWri[5][13][1])))+((rk[8][0]*
          Wpk[5][13][1])-(rk[8][1]*Wpk[5][13][0])));
        VWri[5][14][0] = (Vpk[5][13][0]+((ri[9][2]*Wpk[5][13][1])-(ri[9][1]*
          Wpk[5][13][2])));
        VWri[5][14][1] = (Vpk[5][13][1]+((ri[9][0]*Wpk[5][13][2])-(ri[9][2]*
          Wpk[5][13][0])));
        VWri[5][14][2] = (Vpk[5][13][2]+((ri[9][1]*Wpk[5][13][0])-(ri[9][0]*
          Wpk[5][13][1])));
        Vpk[5][14][0] = (((Cik[14][2][0]*VWri[5][14][2])+((Cik[14][0][0]*
          VWri[5][14][0])+(Cik[14][1][0]*VWri[5][14][1])))+((rk[9][1]*
          Wpk[5][14][2])-(rk[9][2]*Wpk[5][14][1])));
        Vpk[5][14][1] = (((Cik[14][2][1]*VWri[5][14][2])+((Cik[14][0][1]*
          VWri[5][14][0])+(Cik[14][1][1]*VWri[5][14][1])))+((rk[9][2]*
          Wpk[5][14][0])-(rk[9][0]*Wpk[5][14][2])));
        Vpk[5][14][2] = (((Cik[14][2][2]*VWri[5][14][2])+((Cik[14][0][2]*
          VWri[5][14][0])+(Cik[14][1][2]*VWri[5][14][1])))+((rk[9][0]*
          Wpk[5][14][1])-(rk[9][1]*Wpk[5][14][0])));
        VWri[5][15][0] = (rk[0][1]-ri[10][1]);
        VWri[5][15][1] = (ri[10][0]-rk[0][0]);
        Vpk[5][15][0] = (((Cik[15][0][0]*VWri[5][15][0])+(Cik[15][1][0]*
          VWri[5][15][1]))+((Cik[15][2][2]*rk[10][1])-(Cik[15][2][1]*rk[10][2]))
          );
        Vpk[5][15][1] = (((Cik[15][0][1]*VWri[5][15][0])+(Cik[15][1][1]*
          VWri[5][15][1]))+((Cik[15][2][0]*rk[10][2])-(Cik[15][2][2]*rk[10][0]))
          );
        Vpk[5][15][2] = (((Cik[15][0][2]*VWri[5][15][0])+(Cik[15][1][2]*
          VWri[5][15][1]))+((Cik[15][2][1]*rk[10][0])-(Cik[15][2][0]*rk[10][1]))
          );
        VWri[5][16][0] = (Vpk[5][15][0]+((Cik[15][2][1]*ri[11][2])-(
          Cik[15][2][2]*ri[11][1])));
        VWri[5][16][1] = (Vpk[5][15][1]+((Cik[15][2][2]*ri[11][0])-(
          Cik[15][2][0]*ri[11][2])));
        VWri[5][16][2] = (Vpk[5][15][2]+((Cik[15][2][0]*ri[11][1])-(
          Cik[15][2][1]*ri[11][0])));
        Vpk[5][16][0] = (((Cik[16][2][0]*VWri[5][16][2])+((Cik[16][0][0]*
          VWri[5][16][0])+(Cik[16][1][0]*VWri[5][16][1])))+((rk[11][1]*
          Wpk[5][16][2])-(rk[11][2]*Wpk[5][16][1])));
        Vpk[5][16][1] = (((Cik[16][2][1]*VWri[5][16][2])+((Cik[16][0][1]*
          VWri[5][16][0])+(Cik[16][1][1]*VWri[5][16][1])))+((rk[11][2]*
          Wpk[5][16][0])-(rk[11][0]*Wpk[5][16][2])));
        Vpk[5][16][2] = (((Cik[16][2][2]*VWri[5][16][2])+((Cik[16][0][2]*
          VWri[5][16][0])+(Cik[16][1][2]*VWri[5][16][1])))+((rk[11][0]*
          Wpk[5][16][1])-(rk[11][1]*Wpk[5][16][0])));
        VWri[5][17][0] = (Vpk[5][16][0]+((ri[12][2]*Wpk[5][16][1])-(ri[12][1]*
          Wpk[5][16][2])));
        VWri[5][17][1] = (Vpk[5][16][1]+((ri[12][0]*Wpk[5][16][2])-(ri[12][2]*
          Wpk[5][16][0])));
        VWri[5][17][2] = (Vpk[5][16][2]+((ri[12][1]*Wpk[5][16][0])-(ri[12][0]*
          Wpk[5][16][1])));
        Vpk[5][17][0] = (((Cik[17][2][0]*VWri[5][17][2])+((Cik[17][0][0]*
          VWri[5][17][0])+(Cik[17][1][0]*VWri[5][17][1])))+((rk[12][1]*
          Wpk[5][17][2])-(rk[12][2]*Wpk[5][17][1])));
        Vpk[5][17][1] = (((Cik[17][2][1]*VWri[5][17][2])+((Cik[17][0][1]*
          VWri[5][17][0])+(Cik[17][1][1]*VWri[5][17][1])))+((rk[12][2]*
          Wpk[5][17][0])-(rk[12][0]*Wpk[5][17][2])));
        Vpk[5][17][2] = (((Cik[17][2][2]*VWri[5][17][2])+((Cik[17][0][2]*
          VWri[5][17][0])+(Cik[17][1][2]*VWri[5][17][1])))+((rk[12][0]*
          Wpk[5][17][1])-(rk[12][1]*Wpk[5][17][0])));
        VWri[5][18][0] = (Vpk[5][17][0]+((ri[13][2]*Wpk[5][17][1])-(ri[13][1]*
          Wpk[5][17][2])));
        VWri[5][18][1] = (Vpk[5][17][1]+((ri[13][0]*Wpk[5][17][2])-(ri[13][2]*
          Wpk[5][17][0])));
        VWri[5][18][2] = (Vpk[5][17][2]+((ri[13][1]*Wpk[5][17][0])-(ri[13][0]*
          Wpk[5][17][1])));
        Vpk[5][18][0] = (((Cik[18][2][0]*VWri[5][18][2])+((Cik[18][0][0]*
          VWri[5][18][0])+(Cik[18][1][0]*VWri[5][18][1])))+((rk[13][1]*
          Wpk[5][18][2])-(rk[13][2]*Wpk[5][18][1])));
        Vpk[5][18][1] = (((Cik[18][2][1]*VWri[5][18][2])+((Cik[18][0][1]*
          VWri[5][18][0])+(Cik[18][1][1]*VWri[5][18][1])))+((rk[13][2]*
          Wpk[5][18][0])-(rk[13][0]*Wpk[5][18][2])));
        Vpk[5][18][2] = (((Cik[18][2][2]*VWri[5][18][2])+((Cik[18][0][2]*
          VWri[5][18][0])+(Cik[18][1][2]*VWri[5][18][1])))+((rk[13][0]*
          Wpk[5][18][1])-(rk[13][1]*Wpk[5][18][0])));
        VWri[5][19][0] = (Vpk[5][18][0]+((ri[14][2]*Wpk[5][18][1])-(ri[14][1]*
          Wpk[5][18][2])));
        VWri[5][19][1] = (Vpk[5][18][1]+((ri[14][0]*Wpk[5][18][2])-(ri[14][2]*
          Wpk[5][18][0])));
        VWri[5][19][2] = (Vpk[5][18][2]+((ri[14][1]*Wpk[5][18][0])-(ri[14][0]*
          Wpk[5][18][1])));
        Vpk[5][19][0] = (((Cik[19][2][0]*VWri[5][19][2])+((Cik[19][0][0]*
          VWri[5][19][0])+(Cik[19][1][0]*VWri[5][19][1])))+((rk[14][1]*
          Wpk[5][19][2])-(rk[14][2]*Wpk[5][19][1])));
        Vpk[5][19][1] = (((Cik[19][2][1]*VWri[5][19][2])+((Cik[19][0][1]*
          VWri[5][19][0])+(Cik[19][1][1]*VWri[5][19][1])))+((rk[14][2]*
          Wpk[5][19][0])-(rk[14][0]*Wpk[5][19][2])));
        Vpk[5][19][2] = (((Cik[19][2][2]*VWri[5][19][2])+((Cik[19][0][2]*
          VWri[5][19][0])+(Cik[19][1][2]*VWri[5][19][1])))+((rk[14][0]*
          Wpk[5][19][1])-(rk[14][1]*Wpk[5][19][0])));
        VWri[5][20][0] = (Vpk[5][19][0]+((ri[15][2]*Wpk[5][19][1])-(ri[15][1]*
          Wpk[5][19][2])));
        VWri[5][20][1] = (Vpk[5][19][1]+((ri[15][0]*Wpk[5][19][2])-(ri[15][2]*
          Wpk[5][19][0])));
        VWri[5][20][2] = (Vpk[5][19][2]+((ri[15][1]*Wpk[5][19][0])-(ri[15][0]*
          Wpk[5][19][1])));
        Vpk[5][20][0] = (((Cik[20][2][0]*VWri[5][20][2])+((Cik[20][0][0]*
          VWri[5][20][0])+(Cik[20][1][0]*VWri[5][20][1])))+((rk[15][1]*
          Wpk[5][20][2])-(rk[15][2]*Wpk[5][20][1])));
        Vpk[5][20][1] = (((Cik[20][2][1]*VWri[5][20][2])+((Cik[20][0][1]*
          VWri[5][20][0])+(Cik[20][1][1]*VWri[5][20][1])))+((rk[15][2]*
          Wpk[5][20][0])-(rk[15][0]*Wpk[5][20][2])));
        Vpk[5][20][2] = (((Cik[20][2][2]*VWri[5][20][2])+((Cik[20][0][2]*
          VWri[5][20][0])+(Cik[20][1][2]*VWri[5][20][1])))+((rk[15][0]*
          Wpk[5][20][1])-(rk[15][1]*Wpk[5][20][0])));
        Vpk[6][6][0] = ((pin[6][2]*rk[1][1])-(pin[6][1]*rk[1][2]));
        Vpk[6][6][1] = ((pin[6][0]*rk[1][2])-(pin[6][2]*rk[1][0]));
        Vpk[6][6][2] = ((pin[6][1]*rk[1][0])-(pin[6][0]*rk[1][1]));
        VWri[6][7][0] = (Vpk[6][6][0]+((pin[6][1]*ri[2][2])-(pin[6][2]*ri[2][1])
          ));
        VWri[6][7][1] = (Vpk[6][6][1]+((pin[6][2]*ri[2][0])-(pin[6][0]*ri[2][2])
          ));
        VWri[6][7][2] = (Vpk[6][6][2]+((pin[6][0]*ri[2][1])-(pin[6][1]*ri[2][0])
          ));
        Vpk[6][7][0] = (((Cik[7][2][0]*VWri[6][7][2])+((Cik[7][0][0]*
          VWri[6][7][0])+(Cik[7][1][0]*VWri[6][7][1])))+((rk[2][1]*Wpk[6][7][2])
          -(rk[2][2]*Wpk[6][7][1])));
        Vpk[6][7][1] = (((Cik[7][2][1]*VWri[6][7][2])+((Cik[7][0][1]*
          VWri[6][7][0])+(Cik[7][1][1]*VWri[6][7][1])))+((rk[2][2]*Wpk[6][7][0])
          -(rk[2][0]*Wpk[6][7][2])));
        Vpk[6][7][2] = (((Cik[7][2][2]*VWri[6][7][2])+((Cik[7][0][2]*
          VWri[6][7][0])+(Cik[7][1][2]*VWri[6][7][1])))+((rk[2][0]*Wpk[6][7][1])
          -(rk[2][1]*Wpk[6][7][0])));
        VWri[6][8][0] = (Vpk[6][7][0]+((ri[3][2]*Wpk[6][7][1])-(ri[3][1]*
          Wpk[6][7][2])));
        VWri[6][8][1] = (Vpk[6][7][1]+((ri[3][0]*Wpk[6][7][2])-(ri[3][2]*
          Wpk[6][7][0])));
        VWri[6][8][2] = (Vpk[6][7][2]+((ri[3][1]*Wpk[6][7][0])-(ri[3][0]*
          Wpk[6][7][1])));
        Vpk[6][8][0] = (((Cik[8][2][0]*VWri[6][8][2])+((Cik[8][0][0]*
          VWri[6][8][0])+(Cik[8][1][0]*VWri[6][8][1])))+((rk[3][1]*Wpk[6][8][2])
          -(rk[3][2]*Wpk[6][8][1])));
        Vpk[6][8][1] = (((Cik[8][2][1]*VWri[6][8][2])+((Cik[8][0][1]*
          VWri[6][8][0])+(Cik[8][1][1]*VWri[6][8][1])))+((rk[3][2]*Wpk[6][8][0])
          -(rk[3][0]*Wpk[6][8][2])));
        Vpk[6][8][2] = (((Cik[8][2][2]*VWri[6][8][2])+((Cik[8][0][2]*
          VWri[6][8][0])+(Cik[8][1][2]*VWri[6][8][1])))+((rk[3][0]*Wpk[6][8][1])
          -(rk[3][1]*Wpk[6][8][0])));
        Vpk[7][7][0] = ((pin[7][2]*rk[2][1])-(pin[7][1]*rk[2][2]));
        Vpk[7][7][1] = ((pin[7][0]*rk[2][2])-(pin[7][2]*rk[2][0]));
        Vpk[7][7][2] = ((pin[7][1]*rk[2][0])-(pin[7][0]*rk[2][1]));
        VWri[7][8][0] = (Vpk[7][7][0]+((pin[7][1]*ri[3][2])-(pin[7][2]*ri[3][1])
          ));
        VWri[7][8][1] = (Vpk[7][7][1]+((pin[7][2]*ri[3][0])-(pin[7][0]*ri[3][2])
          ));
        VWri[7][8][2] = (Vpk[7][7][2]+((pin[7][0]*ri[3][1])-(pin[7][1]*ri[3][0])
          ));
        Vpk[7][8][0] = (((Cik[8][2][0]*VWri[7][8][2])+((Cik[8][0][0]*
          VWri[7][8][0])+(Cik[8][1][0]*VWri[7][8][1])))+((rk[3][1]*Wpk[7][8][2])
          -(rk[3][2]*Wpk[7][8][1])));
        Vpk[7][8][1] = (((Cik[8][2][1]*VWri[7][8][2])+((Cik[8][0][1]*
          VWri[7][8][0])+(Cik[8][1][1]*VWri[7][8][1])))+((rk[3][2]*Wpk[7][8][0])
          -(rk[3][0]*Wpk[7][8][2])));
        Vpk[7][8][2] = (((Cik[8][2][2]*VWri[7][8][2])+((Cik[8][0][2]*
          VWri[7][8][0])+(Cik[8][1][2]*VWri[7][8][1])))+((rk[3][0]*Wpk[7][8][1])
          -(rk[3][1]*Wpk[7][8][0])));
        Vpk[8][8][0] = ((pin[8][2]*rk[3][1])-(pin[8][1]*rk[3][2]));
        Vpk[8][8][1] = ((pin[8][0]*rk[3][2])-(pin[8][2]*rk[3][0]));
        Vpk[8][8][2] = ((pin[8][1]*rk[3][0])-(pin[8][0]*rk[3][1]));
        Vpk[9][9][0] = ((pin[9][2]*rk[4][1])-(pin[9][1]*rk[4][2]));
        Vpk[9][9][1] = ((pin[9][0]*rk[4][2])-(pin[9][2]*rk[4][0]));
        Vpk[9][9][2] = ((pin[9][1]*rk[4][0])-(pin[9][0]*rk[4][1]));
        VWri[9][10][0] = (Vpk[9][9][0]+((pin[9][1]*ri[5][2])-(pin[9][2]*ri[5][1]
          )));
        VWri[9][10][1] = (Vpk[9][9][1]+((pin[9][2]*ri[5][0])-(pin[9][0]*ri[5][2]
          )));
        VWri[9][10][2] = (Vpk[9][9][2]+((pin[9][0]*ri[5][1])-(pin[9][1]*ri[5][0]
          )));
        Vpk[9][10][0] = (((Cik[10][2][0]*VWri[9][10][2])+((Cik[10][0][0]*
          VWri[9][10][0])+(Cik[10][1][0]*VWri[9][10][1])))+((rk[5][1]*
          Wpk[9][10][2])-(rk[5][2]*Wpk[9][10][1])));
        Vpk[9][10][1] = (((Cik[10][2][1]*VWri[9][10][2])+((Cik[10][0][1]*
          VWri[9][10][0])+(Cik[10][1][1]*VWri[9][10][1])))+((rk[5][2]*
          Wpk[9][10][0])-(rk[5][0]*Wpk[9][10][2])));
        Vpk[9][10][2] = (((Cik[10][2][2]*VWri[9][10][2])+((Cik[10][0][2]*
          VWri[9][10][0])+(Cik[10][1][2]*VWri[9][10][1])))+((rk[5][0]*
          Wpk[9][10][1])-(rk[5][1]*Wpk[9][10][0])));
        VWri[9][11][0] = (Vpk[9][10][0]+((ri[6][2]*Wpk[9][10][1])-(ri[6][1]*
          Wpk[9][10][2])));
        VWri[9][11][1] = (Vpk[9][10][1]+((ri[6][0]*Wpk[9][10][2])-(ri[6][2]*
          Wpk[9][10][0])));
        VWri[9][11][2] = (Vpk[9][10][2]+((ri[6][1]*Wpk[9][10][0])-(ri[6][0]*
          Wpk[9][10][1])));
        Vpk[9][11][0] = (((Cik[11][2][0]*VWri[9][11][2])+((Cik[11][0][0]*
          VWri[9][11][0])+(Cik[11][1][0]*VWri[9][11][1])))+((rk[6][1]*
          Wpk[9][11][2])-(rk[6][2]*Wpk[9][11][1])));
        Vpk[9][11][1] = (((Cik[11][2][1]*VWri[9][11][2])+((Cik[11][0][1]*
          VWri[9][11][0])+(Cik[11][1][1]*VWri[9][11][1])))+((rk[6][2]*
          Wpk[9][11][0])-(rk[6][0]*Wpk[9][11][2])));
        Vpk[9][11][2] = (((Cik[11][2][2]*VWri[9][11][2])+((Cik[11][0][2]*
          VWri[9][11][0])+(Cik[11][1][2]*VWri[9][11][1])))+((rk[6][0]*
          Wpk[9][11][1])-(rk[6][1]*Wpk[9][11][0])));
        VWri[9][12][0] = (Vpk[9][11][0]+((ri[7][2]*Wpk[9][11][1])-(ri[7][1]*
          Wpk[9][11][2])));
        VWri[9][12][1] = (Vpk[9][11][1]+((ri[7][0]*Wpk[9][11][2])-(ri[7][2]*
          Wpk[9][11][0])));
        VWri[9][12][2] = (Vpk[9][11][2]+((ri[7][1]*Wpk[9][11][0])-(ri[7][0]*
          Wpk[9][11][1])));
        Vpk[9][12][0] = (((Cik[12][2][0]*VWri[9][12][2])+((Cik[12][0][0]*
          VWri[9][12][0])+(Cik[12][1][0]*VWri[9][12][1])))+((rk[7][1]*
          Wpk[9][12][2])-(rk[7][2]*Wpk[9][12][1])));
        Vpk[9][12][1] = (((Cik[12][2][1]*VWri[9][12][2])+((Cik[12][0][1]*
          VWri[9][12][0])+(Cik[12][1][1]*VWri[9][12][1])))+((rk[7][2]*
          Wpk[9][12][0])-(rk[7][0]*Wpk[9][12][2])));
        Vpk[9][12][2] = (((Cik[12][2][2]*VWri[9][12][2])+((Cik[12][0][2]*
          VWri[9][12][0])+(Cik[12][1][2]*VWri[9][12][1])))+((rk[7][0]*
          Wpk[9][12][1])-(rk[7][1]*Wpk[9][12][0])));
        VWri[9][13][0] = (Vpk[9][12][0]+((ri[8][2]*Wpk[9][12][1])-(ri[8][1]*
          Wpk[9][12][2])));
        VWri[9][13][1] = (Vpk[9][12][1]+((ri[8][0]*Wpk[9][12][2])-(ri[8][2]*
          Wpk[9][12][0])));
        VWri[9][13][2] = (Vpk[9][12][2]+((ri[8][1]*Wpk[9][12][0])-(ri[8][0]*
          Wpk[9][12][1])));
        Vpk[9][13][0] = (((Cik[13][2][0]*VWri[9][13][2])+((Cik[13][0][0]*
          VWri[9][13][0])+(Cik[13][1][0]*VWri[9][13][1])))+((rk[8][1]*
          Wpk[9][13][2])-(rk[8][2]*Wpk[9][13][1])));
        Vpk[9][13][1] = (((Cik[13][2][1]*VWri[9][13][2])+((Cik[13][0][1]*
          VWri[9][13][0])+(Cik[13][1][1]*VWri[9][13][1])))+((rk[8][2]*
          Wpk[9][13][0])-(rk[8][0]*Wpk[9][13][2])));
        Vpk[9][13][2] = (((Cik[13][2][2]*VWri[9][13][2])+((Cik[13][0][2]*
          VWri[9][13][0])+(Cik[13][1][2]*VWri[9][13][1])))+((rk[8][0]*
          Wpk[9][13][1])-(rk[8][1]*Wpk[9][13][0])));
        VWri[9][14][0] = (Vpk[9][13][0]+((ri[9][2]*Wpk[9][13][1])-(ri[9][1]*
          Wpk[9][13][2])));
        VWri[9][14][1] = (Vpk[9][13][1]+((ri[9][0]*Wpk[9][13][2])-(ri[9][2]*
          Wpk[9][13][0])));
        VWri[9][14][2] = (Vpk[9][13][2]+((ri[9][1]*Wpk[9][13][0])-(ri[9][0]*
          Wpk[9][13][1])));
        Vpk[9][14][0] = (((Cik[14][2][0]*VWri[9][14][2])+((Cik[14][0][0]*
          VWri[9][14][0])+(Cik[14][1][0]*VWri[9][14][1])))+((rk[9][1]*
          Wpk[9][14][2])-(rk[9][2]*Wpk[9][14][1])));
        Vpk[9][14][1] = (((Cik[14][2][1]*VWri[9][14][2])+((Cik[14][0][1]*
          VWri[9][14][0])+(Cik[14][1][1]*VWri[9][14][1])))+((rk[9][2]*
          Wpk[9][14][0])-(rk[9][0]*Wpk[9][14][2])));
        Vpk[9][14][2] = (((Cik[14][2][2]*VWri[9][14][2])+((Cik[14][0][2]*
          VWri[9][14][0])+(Cik[14][1][2]*VWri[9][14][1])))+((rk[9][0]*
          Wpk[9][14][1])-(rk[9][1]*Wpk[9][14][0])));
        Vpk[10][10][0] = ((pin[10][2]*rk[5][1])-(pin[10][1]*rk[5][2]));
        Vpk[10][10][1] = ((pin[10][0]*rk[5][2])-(pin[10][2]*rk[5][0]));
        Vpk[10][10][2] = ((pin[10][1]*rk[5][0])-(pin[10][0]*rk[5][1]));
        VWri[10][11][0] = (Vpk[10][10][0]+((pin[10][1]*ri[6][2])-(pin[10][2]*
          ri[6][1])));
        VWri[10][11][1] = (Vpk[10][10][1]+((pin[10][2]*ri[6][0])-(pin[10][0]*
          ri[6][2])));
        VWri[10][11][2] = (Vpk[10][10][2]+((pin[10][0]*ri[6][1])-(pin[10][1]*
          ri[6][0])));
        Vpk[10][11][0] = (((Cik[11][2][0]*VWri[10][11][2])+((Cik[11][0][0]*
          VWri[10][11][0])+(Cik[11][1][0]*VWri[10][11][1])))+((rk[6][1]*
          Wpk[10][11][2])-(rk[6][2]*Wpk[10][11][1])));
        Vpk[10][11][1] = (((Cik[11][2][1]*VWri[10][11][2])+((Cik[11][0][1]*
          VWri[10][11][0])+(Cik[11][1][1]*VWri[10][11][1])))+((rk[6][2]*
          Wpk[10][11][0])-(rk[6][0]*Wpk[10][11][2])));
        Vpk[10][11][2] = (((Cik[11][2][2]*VWri[10][11][2])+((Cik[11][0][2]*
          VWri[10][11][0])+(Cik[11][1][2]*VWri[10][11][1])))+((rk[6][0]*
          Wpk[10][11][1])-(rk[6][1]*Wpk[10][11][0])));
        VWri[10][12][0] = (Vpk[10][11][0]+((ri[7][2]*Wpk[10][11][1])-(ri[7][1]*
          Wpk[10][11][2])));
        VWri[10][12][1] = (Vpk[10][11][1]+((ri[7][0]*Wpk[10][11][2])-(ri[7][2]*
          Wpk[10][11][0])));
        VWri[10][12][2] = (Vpk[10][11][2]+((ri[7][1]*Wpk[10][11][0])-(ri[7][0]*
          Wpk[10][11][1])));
        Vpk[10][12][0] = (((Cik[12][2][0]*VWri[10][12][2])+((Cik[12][0][0]*
          VWri[10][12][0])+(Cik[12][1][0]*VWri[10][12][1])))+((rk[7][1]*
          Wpk[10][12][2])-(rk[7][2]*Wpk[10][12][1])));
        Vpk[10][12][1] = (((Cik[12][2][1]*VWri[10][12][2])+((Cik[12][0][1]*
          VWri[10][12][0])+(Cik[12][1][1]*VWri[10][12][1])))+((rk[7][2]*
          Wpk[10][12][0])-(rk[7][0]*Wpk[10][12][2])));
        Vpk[10][12][2] = (((Cik[12][2][2]*VWri[10][12][2])+((Cik[12][0][2]*
          VWri[10][12][0])+(Cik[12][1][2]*VWri[10][12][1])))+((rk[7][0]*
          Wpk[10][12][1])-(rk[7][1]*Wpk[10][12][0])));
        VWri[10][13][0] = (Vpk[10][12][0]+((ri[8][2]*Wpk[10][12][1])-(ri[8][1]*
          Wpk[10][12][2])));
        VWri[10][13][1] = (Vpk[10][12][1]+((ri[8][0]*Wpk[10][12][2])-(ri[8][2]*
          Wpk[10][12][0])));
        VWri[10][13][2] = (Vpk[10][12][2]+((ri[8][1]*Wpk[10][12][0])-(ri[8][0]*
          Wpk[10][12][1])));
        Vpk[10][13][0] = (((Cik[13][2][0]*VWri[10][13][2])+((Cik[13][0][0]*
          VWri[10][13][0])+(Cik[13][1][0]*VWri[10][13][1])))+((rk[8][1]*
          Wpk[10][13][2])-(rk[8][2]*Wpk[10][13][1])));
        Vpk[10][13][1] = (((Cik[13][2][1]*VWri[10][13][2])+((Cik[13][0][1]*
          VWri[10][13][0])+(Cik[13][1][1]*VWri[10][13][1])))+((rk[8][2]*
          Wpk[10][13][0])-(rk[8][0]*Wpk[10][13][2])));
        Vpk[10][13][2] = (((Cik[13][2][2]*VWri[10][13][2])+((Cik[13][0][2]*
          VWri[10][13][0])+(Cik[13][1][2]*VWri[10][13][1])))+((rk[8][0]*
          Wpk[10][13][1])-(rk[8][1]*Wpk[10][13][0])));
        VWri[10][14][0] = (Vpk[10][13][0]+((ri[9][2]*Wpk[10][13][1])-(ri[9][1]*
          Wpk[10][13][2])));
        VWri[10][14][1] = (Vpk[10][13][1]+((ri[9][0]*Wpk[10][13][2])-(ri[9][2]*
          Wpk[10][13][0])));
        VWri[10][14][2] = (Vpk[10][13][2]+((ri[9][1]*Wpk[10][13][0])-(ri[9][0]*
          Wpk[10][13][1])));
        Vpk[10][14][0] = (((Cik[14][2][0]*VWri[10][14][2])+((Cik[14][0][0]*
          VWri[10][14][0])+(Cik[14][1][0]*VWri[10][14][1])))+((rk[9][1]*
          Wpk[10][14][2])-(rk[9][2]*Wpk[10][14][1])));
        Vpk[10][14][1] = (((Cik[14][2][1]*VWri[10][14][2])+((Cik[14][0][1]*
          VWri[10][14][0])+(Cik[14][1][1]*VWri[10][14][1])))+((rk[9][2]*
          Wpk[10][14][0])-(rk[9][0]*Wpk[10][14][2])));
        Vpk[10][14][2] = (((Cik[14][2][2]*VWri[10][14][2])+((Cik[14][0][2]*
          VWri[10][14][0])+(Cik[14][1][2]*VWri[10][14][1])))+((rk[9][0]*
          Wpk[10][14][1])-(rk[9][1]*Wpk[10][14][0])));
        Vpk[11][11][0] = ((pin[11][2]*rk[6][1])-(pin[11][1]*rk[6][2]));
        Vpk[11][11][1] = ((pin[11][0]*rk[6][2])-(pin[11][2]*rk[6][0]));
        Vpk[11][11][2] = ((pin[11][1]*rk[6][0])-(pin[11][0]*rk[6][1]));
        VWri[11][12][0] = (Vpk[11][11][0]+((pin[11][1]*ri[7][2])-(pin[11][2]*
          ri[7][1])));
        VWri[11][12][1] = (Vpk[11][11][1]+((pin[11][2]*ri[7][0])-(pin[11][0]*
          ri[7][2])));
        VWri[11][12][2] = (Vpk[11][11][2]+((pin[11][0]*ri[7][1])-(pin[11][1]*
          ri[7][0])));
        Vpk[11][12][0] = (((Cik[12][2][0]*VWri[11][12][2])+((Cik[12][0][0]*
          VWri[11][12][0])+(Cik[12][1][0]*VWri[11][12][1])))+((rk[7][1]*
          Wpk[11][12][2])-(rk[7][2]*Wpk[11][12][1])));
        Vpk[11][12][1] = (((Cik[12][2][1]*VWri[11][12][2])+((Cik[12][0][1]*
          VWri[11][12][0])+(Cik[12][1][1]*VWri[11][12][1])))+((rk[7][2]*
          Wpk[11][12][0])-(rk[7][0]*Wpk[11][12][2])));
        Vpk[11][12][2] = (((Cik[12][2][2]*VWri[11][12][2])+((Cik[12][0][2]*
          VWri[11][12][0])+(Cik[12][1][2]*VWri[11][12][1])))+((rk[7][0]*
          Wpk[11][12][1])-(rk[7][1]*Wpk[11][12][0])));
        VWri[11][13][0] = (Vpk[11][12][0]+((ri[8][2]*Wpk[11][12][1])-(ri[8][1]*
          Wpk[11][12][2])));
        VWri[11][13][1] = (Vpk[11][12][1]+((ri[8][0]*Wpk[11][12][2])-(ri[8][2]*
          Wpk[11][12][0])));
        VWri[11][13][2] = (Vpk[11][12][2]+((ri[8][1]*Wpk[11][12][0])-(ri[8][0]*
          Wpk[11][12][1])));
        Vpk[11][13][0] = (((Cik[13][2][0]*VWri[11][13][2])+((Cik[13][0][0]*
          VWri[11][13][0])+(Cik[13][1][0]*VWri[11][13][1])))+((rk[8][1]*
          Wpk[11][13][2])-(rk[8][2]*Wpk[11][13][1])));
        Vpk[11][13][1] = (((Cik[13][2][1]*VWri[11][13][2])+((Cik[13][0][1]*
          VWri[11][13][0])+(Cik[13][1][1]*VWri[11][13][1])))+((rk[8][2]*
          Wpk[11][13][0])-(rk[8][0]*Wpk[11][13][2])));
        Vpk[11][13][2] = (((Cik[13][2][2]*VWri[11][13][2])+((Cik[13][0][2]*
          VWri[11][13][0])+(Cik[13][1][2]*VWri[11][13][1])))+((rk[8][0]*
          Wpk[11][13][1])-(rk[8][1]*Wpk[11][13][0])));
        VWri[11][14][0] = (Vpk[11][13][0]+((ri[9][2]*Wpk[11][13][1])-(ri[9][1]*
          Wpk[11][13][2])));
        VWri[11][14][1] = (Vpk[11][13][1]+((ri[9][0]*Wpk[11][13][2])-(ri[9][2]*
          Wpk[11][13][0])));
        VWri[11][14][2] = (Vpk[11][13][2]+((ri[9][1]*Wpk[11][13][0])-(ri[9][0]*
          Wpk[11][13][1])));
        Vpk[11][14][0] = (((Cik[14][2][0]*VWri[11][14][2])+((Cik[14][0][0]*
          VWri[11][14][0])+(Cik[14][1][0]*VWri[11][14][1])))+((rk[9][1]*
          Wpk[11][14][2])-(rk[9][2]*Wpk[11][14][1])));
        Vpk[11][14][1] = (((Cik[14][2][1]*VWri[11][14][2])+((Cik[14][0][1]*
          VWri[11][14][0])+(Cik[14][1][1]*VWri[11][14][1])))+((rk[9][2]*
          Wpk[11][14][0])-(rk[9][0]*Wpk[11][14][2])));
        Vpk[11][14][2] = (((Cik[14][2][2]*VWri[11][14][2])+((Cik[14][0][2]*
          VWri[11][14][0])+(Cik[14][1][2]*VWri[11][14][1])))+((rk[9][0]*
          Wpk[11][14][1])-(rk[9][1]*Wpk[11][14][0])));
        Vpk[12][12][0] = ((pin[12][2]*rk[7][1])-(pin[12][1]*rk[7][2]));
        Vpk[12][12][1] = ((pin[12][0]*rk[7][2])-(pin[12][2]*rk[7][0]));
        Vpk[12][12][2] = ((pin[12][1]*rk[7][0])-(pin[12][0]*rk[7][1]));
        VWri[12][13][0] = (Vpk[12][12][0]+((pin[12][1]*ri[8][2])-(pin[12][2]*
          ri[8][1])));
        VWri[12][13][1] = (Vpk[12][12][1]+((pin[12][2]*ri[8][0])-(pin[12][0]*
          ri[8][2])));
        VWri[12][13][2] = (Vpk[12][12][2]+((pin[12][0]*ri[8][1])-(pin[12][1]*
          ri[8][0])));
        Vpk[12][13][0] = (((Cik[13][2][0]*VWri[12][13][2])+((Cik[13][0][0]*
          VWri[12][13][0])+(Cik[13][1][0]*VWri[12][13][1])))+((rk[8][1]*
          Wpk[12][13][2])-(rk[8][2]*Wpk[12][13][1])));
        Vpk[12][13][1] = (((Cik[13][2][1]*VWri[12][13][2])+((Cik[13][0][1]*
          VWri[12][13][0])+(Cik[13][1][1]*VWri[12][13][1])))+((rk[8][2]*
          Wpk[12][13][0])-(rk[8][0]*Wpk[12][13][2])));
        Vpk[12][13][2] = (((Cik[13][2][2]*VWri[12][13][2])+((Cik[13][0][2]*
          VWri[12][13][0])+(Cik[13][1][2]*VWri[12][13][1])))+((rk[8][0]*
          Wpk[12][13][1])-(rk[8][1]*Wpk[12][13][0])));
        VWri[12][14][0] = (Vpk[12][13][0]+((ri[9][2]*Wpk[12][13][1])-(ri[9][1]*
          Wpk[12][13][2])));
        VWri[12][14][1] = (Vpk[12][13][1]+((ri[9][0]*Wpk[12][13][2])-(ri[9][2]*
          Wpk[12][13][0])));
        VWri[12][14][2] = (Vpk[12][13][2]+((ri[9][1]*Wpk[12][13][0])-(ri[9][0]*
          Wpk[12][13][1])));
        Vpk[12][14][0] = (((Cik[14][2][0]*VWri[12][14][2])+((Cik[14][0][0]*
          VWri[12][14][0])+(Cik[14][1][0]*VWri[12][14][1])))+((rk[9][1]*
          Wpk[12][14][2])-(rk[9][2]*Wpk[12][14][1])));
        Vpk[12][14][1] = (((Cik[14][2][1]*VWri[12][14][2])+((Cik[14][0][1]*
          VWri[12][14][0])+(Cik[14][1][1]*VWri[12][14][1])))+((rk[9][2]*
          Wpk[12][14][0])-(rk[9][0]*Wpk[12][14][2])));
        Vpk[12][14][2] = (((Cik[14][2][2]*VWri[12][14][2])+((Cik[14][0][2]*
          VWri[12][14][0])+(Cik[14][1][2]*VWri[12][14][1])))+((rk[9][0]*
          Wpk[12][14][1])-(rk[9][1]*Wpk[12][14][0])));
        Vpk[13][13][0] = ((pin[13][2]*rk[8][1])-(pin[13][1]*rk[8][2]));
        Vpk[13][13][1] = ((pin[13][0]*rk[8][2])-(pin[13][2]*rk[8][0]));
        Vpk[13][13][2] = ((pin[13][1]*rk[8][0])-(pin[13][0]*rk[8][1]));
        VWri[13][14][0] = (Vpk[13][13][0]+((pin[13][1]*ri[9][2])-(pin[13][2]*
          ri[9][1])));
        VWri[13][14][1] = (Vpk[13][13][1]+((pin[13][2]*ri[9][0])-(pin[13][0]*
          ri[9][2])));
        VWri[13][14][2] = (Vpk[13][13][2]+((pin[13][0]*ri[9][1])-(pin[13][1]*
          ri[9][0])));
        Vpk[13][14][0] = (((Cik[14][2][0]*VWri[13][14][2])+((Cik[14][0][0]*
          VWri[13][14][0])+(Cik[14][1][0]*VWri[13][14][1])))+((rk[9][1]*
          Wpk[13][14][2])-(rk[9][2]*Wpk[13][14][1])));
        Vpk[13][14][1] = (((Cik[14][2][1]*VWri[13][14][2])+((Cik[14][0][1]*
          VWri[13][14][0])+(Cik[14][1][1]*VWri[13][14][1])))+((rk[9][2]*
          Wpk[13][14][0])-(rk[9][0]*Wpk[13][14][2])));
        Vpk[13][14][2] = (((Cik[14][2][2]*VWri[13][14][2])+((Cik[14][0][2]*
          VWri[13][14][0])+(Cik[14][1][2]*VWri[13][14][1])))+((rk[9][0]*
          Wpk[13][14][1])-(rk[9][1]*Wpk[13][14][0])));
        Vpk[14][14][0] = ((pin[14][2]*rk[9][1])-(pin[14][1]*rk[9][2]));
        Vpk[14][14][1] = ((pin[14][0]*rk[9][2])-(pin[14][2]*rk[9][0]));
        Vpk[14][14][2] = ((pin[14][1]*rk[9][0])-(pin[14][0]*rk[9][1]));
        Vpk[15][15][0] = ((pin[15][2]*rk[10][1])-(pin[15][1]*rk[10][2]));
        Vpk[15][15][1] = ((pin[15][0]*rk[10][2])-(pin[15][2]*rk[10][0]));
        Vpk[15][15][2] = ((pin[15][1]*rk[10][0])-(pin[15][0]*rk[10][1]));
        VWri[15][16][0] = (Vpk[15][15][0]+((pin[15][1]*ri[11][2])-(pin[15][2]*
          ri[11][1])));
        VWri[15][16][1] = (Vpk[15][15][1]+((pin[15][2]*ri[11][0])-(pin[15][0]*
          ri[11][2])));
        VWri[15][16][2] = (Vpk[15][15][2]+((pin[15][0]*ri[11][1])-(pin[15][1]*
          ri[11][0])));
        Vpk[15][16][0] = (((Cik[16][2][0]*VWri[15][16][2])+((Cik[16][0][0]*
          VWri[15][16][0])+(Cik[16][1][0]*VWri[15][16][1])))+((rk[11][1]*
          Wpk[15][16][2])-(rk[11][2]*Wpk[15][16][1])));
        Vpk[15][16][1] = (((Cik[16][2][1]*VWri[15][16][2])+((Cik[16][0][1]*
          VWri[15][16][0])+(Cik[16][1][1]*VWri[15][16][1])))+((rk[11][2]*
          Wpk[15][16][0])-(rk[11][0]*Wpk[15][16][2])));
        Vpk[15][16][2] = (((Cik[16][2][2]*VWri[15][16][2])+((Cik[16][0][2]*
          VWri[15][16][0])+(Cik[16][1][2]*VWri[15][16][1])))+((rk[11][0]*
          Wpk[15][16][1])-(rk[11][1]*Wpk[15][16][0])));
        VWri[15][17][0] = (Vpk[15][16][0]+((ri[12][2]*Wpk[15][16][1])-(ri[12][1]
          *Wpk[15][16][2])));
        VWri[15][17][1] = (Vpk[15][16][1]+((ri[12][0]*Wpk[15][16][2])-(ri[12][2]
          *Wpk[15][16][0])));
        VWri[15][17][2] = (Vpk[15][16][2]+((ri[12][1]*Wpk[15][16][0])-(ri[12][0]
          *Wpk[15][16][1])));
        Vpk[15][17][0] = (((Cik[17][2][0]*VWri[15][17][2])+((Cik[17][0][0]*
          VWri[15][17][0])+(Cik[17][1][0]*VWri[15][17][1])))+((rk[12][1]*
          Wpk[15][17][2])-(rk[12][2]*Wpk[15][17][1])));
        Vpk[15][17][1] = (((Cik[17][2][1]*VWri[15][17][2])+((Cik[17][0][1]*
          VWri[15][17][0])+(Cik[17][1][1]*VWri[15][17][1])))+((rk[12][2]*
          Wpk[15][17][0])-(rk[12][0]*Wpk[15][17][2])));
        Vpk[15][17][2] = (((Cik[17][2][2]*VWri[15][17][2])+((Cik[17][0][2]*
          VWri[15][17][0])+(Cik[17][1][2]*VWri[15][17][1])))+((rk[12][0]*
          Wpk[15][17][1])-(rk[12][1]*Wpk[15][17][0])));
        VWri[15][18][0] = (Vpk[15][17][0]+((ri[13][2]*Wpk[15][17][1])-(ri[13][1]
          *Wpk[15][17][2])));
        VWri[15][18][1] = (Vpk[15][17][1]+((ri[13][0]*Wpk[15][17][2])-(ri[13][2]
          *Wpk[15][17][0])));
        VWri[15][18][2] = (Vpk[15][17][2]+((ri[13][1]*Wpk[15][17][0])-(ri[13][0]
          *Wpk[15][17][1])));
        Vpk[15][18][0] = (((Cik[18][2][0]*VWri[15][18][2])+((Cik[18][0][0]*
          VWri[15][18][0])+(Cik[18][1][0]*VWri[15][18][1])))+((rk[13][1]*
          Wpk[15][18][2])-(rk[13][2]*Wpk[15][18][1])));
        Vpk[15][18][1] = (((Cik[18][2][1]*VWri[15][18][2])+((Cik[18][0][1]*
          VWri[15][18][0])+(Cik[18][1][1]*VWri[15][18][1])))+((rk[13][2]*
          Wpk[15][18][0])-(rk[13][0]*Wpk[15][18][2])));
        Vpk[15][18][2] = (((Cik[18][2][2]*VWri[15][18][2])+((Cik[18][0][2]*
          VWri[15][18][0])+(Cik[18][1][2]*VWri[15][18][1])))+((rk[13][0]*
          Wpk[15][18][1])-(rk[13][1]*Wpk[15][18][0])));
        VWri[15][19][0] = (Vpk[15][18][0]+((ri[14][2]*Wpk[15][18][1])-(ri[14][1]
          *Wpk[15][18][2])));
        VWri[15][19][1] = (Vpk[15][18][1]+((ri[14][0]*Wpk[15][18][2])-(ri[14][2]
          *Wpk[15][18][0])));
        VWri[15][19][2] = (Vpk[15][18][2]+((ri[14][1]*Wpk[15][18][0])-(ri[14][0]
          *Wpk[15][18][1])));
        Vpk[15][19][0] = (((Cik[19][2][0]*VWri[15][19][2])+((Cik[19][0][0]*
          VWri[15][19][0])+(Cik[19][1][0]*VWri[15][19][1])))+((rk[14][1]*
          Wpk[15][19][2])-(rk[14][2]*Wpk[15][19][1])));
        Vpk[15][19][1] = (((Cik[19][2][1]*VWri[15][19][2])+((Cik[19][0][1]*
          VWri[15][19][0])+(Cik[19][1][1]*VWri[15][19][1])))+((rk[14][2]*
          Wpk[15][19][0])-(rk[14][0]*Wpk[15][19][2])));
        Vpk[15][19][2] = (((Cik[19][2][2]*VWri[15][19][2])+((Cik[19][0][2]*
          VWri[15][19][0])+(Cik[19][1][2]*VWri[15][19][1])))+((rk[14][0]*
          Wpk[15][19][1])-(rk[14][1]*Wpk[15][19][0])));
        VWri[15][20][0] = (Vpk[15][19][0]+((ri[15][2]*Wpk[15][19][1])-(ri[15][1]
          *Wpk[15][19][2])));
        VWri[15][20][1] = (Vpk[15][19][1]+((ri[15][0]*Wpk[15][19][2])-(ri[15][2]
          *Wpk[15][19][0])));
        VWri[15][20][2] = (Vpk[15][19][2]+((ri[15][1]*Wpk[15][19][0])-(ri[15][0]
          *Wpk[15][19][1])));
        Vpk[15][20][0] = (((Cik[20][2][0]*VWri[15][20][2])+((Cik[20][0][0]*
          VWri[15][20][0])+(Cik[20][1][0]*VWri[15][20][1])))+((rk[15][1]*
          Wpk[15][20][2])-(rk[15][2]*Wpk[15][20][1])));
        Vpk[15][20][1] = (((Cik[20][2][1]*VWri[15][20][2])+((Cik[20][0][1]*
          VWri[15][20][0])+(Cik[20][1][1]*VWri[15][20][1])))+((rk[15][2]*
          Wpk[15][20][0])-(rk[15][0]*Wpk[15][20][2])));
        Vpk[15][20][2] = (((Cik[20][2][2]*VWri[15][20][2])+((Cik[20][0][2]*
          VWri[15][20][0])+(Cik[20][1][2]*VWri[15][20][1])))+((rk[15][0]*
          Wpk[15][20][1])-(rk[15][1]*Wpk[15][20][0])));
        Vpk[16][16][0] = ((pin[16][2]*rk[11][1])-(pin[16][1]*rk[11][2]));
        Vpk[16][16][1] = ((pin[16][0]*rk[11][2])-(pin[16][2]*rk[11][0]));
        Vpk[16][16][2] = ((pin[16][1]*rk[11][0])-(pin[16][0]*rk[11][1]));
        VWri[16][17][0] = (Vpk[16][16][0]+((pin[16][1]*ri[12][2])-(pin[16][2]*
          ri[12][1])));
        VWri[16][17][1] = (Vpk[16][16][1]+((pin[16][2]*ri[12][0])-(pin[16][0]*
          ri[12][2])));
        VWri[16][17][2] = (Vpk[16][16][2]+((pin[16][0]*ri[12][1])-(pin[16][1]*
          ri[12][0])));
        Vpk[16][17][0] = (((Cik[17][2][0]*VWri[16][17][2])+((Cik[17][0][0]*
          VWri[16][17][0])+(Cik[17][1][0]*VWri[16][17][1])))+((rk[12][1]*
          Wpk[16][17][2])-(rk[12][2]*Wpk[16][17][1])));
        Vpk[16][17][1] = (((Cik[17][2][1]*VWri[16][17][2])+((Cik[17][0][1]*
          VWri[16][17][0])+(Cik[17][1][1]*VWri[16][17][1])))+((rk[12][2]*
          Wpk[16][17][0])-(rk[12][0]*Wpk[16][17][2])));
        Vpk[16][17][2] = (((Cik[17][2][2]*VWri[16][17][2])+((Cik[17][0][2]*
          VWri[16][17][0])+(Cik[17][1][2]*VWri[16][17][1])))+((rk[12][0]*
          Wpk[16][17][1])-(rk[12][1]*Wpk[16][17][0])));
        VWri[16][18][0] = (Vpk[16][17][0]+((ri[13][2]*Wpk[16][17][1])-(ri[13][1]
          *Wpk[16][17][2])));
        VWri[16][18][1] = (Vpk[16][17][1]+((ri[13][0]*Wpk[16][17][2])-(ri[13][2]
          *Wpk[16][17][0])));
        VWri[16][18][2] = (Vpk[16][17][2]+((ri[13][1]*Wpk[16][17][0])-(ri[13][0]
          *Wpk[16][17][1])));
        Vpk[16][18][0] = (((Cik[18][2][0]*VWri[16][18][2])+((Cik[18][0][0]*
          VWri[16][18][0])+(Cik[18][1][0]*VWri[16][18][1])))+((rk[13][1]*
          Wpk[16][18][2])-(rk[13][2]*Wpk[16][18][1])));
        Vpk[16][18][1] = (((Cik[18][2][1]*VWri[16][18][2])+((Cik[18][0][1]*
          VWri[16][18][0])+(Cik[18][1][1]*VWri[16][18][1])))+((rk[13][2]*
          Wpk[16][18][0])-(rk[13][0]*Wpk[16][18][2])));
        Vpk[16][18][2] = (((Cik[18][2][2]*VWri[16][18][2])+((Cik[18][0][2]*
          VWri[16][18][0])+(Cik[18][1][2]*VWri[16][18][1])))+((rk[13][0]*
          Wpk[16][18][1])-(rk[13][1]*Wpk[16][18][0])));
        VWri[16][19][0] = (Vpk[16][18][0]+((ri[14][2]*Wpk[16][18][1])-(ri[14][1]
          *Wpk[16][18][2])));
        VWri[16][19][1] = (Vpk[16][18][1]+((ri[14][0]*Wpk[16][18][2])-(ri[14][2]
          *Wpk[16][18][0])));
        VWri[16][19][2] = (Vpk[16][18][2]+((ri[14][1]*Wpk[16][18][0])-(ri[14][0]
          *Wpk[16][18][1])));
        Vpk[16][19][0] = (((Cik[19][2][0]*VWri[16][19][2])+((Cik[19][0][0]*
          VWri[16][19][0])+(Cik[19][1][0]*VWri[16][19][1])))+((rk[14][1]*
          Wpk[16][19][2])-(rk[14][2]*Wpk[16][19][1])));
        Vpk[16][19][1] = (((Cik[19][2][1]*VWri[16][19][2])+((Cik[19][0][1]*
          VWri[16][19][0])+(Cik[19][1][1]*VWri[16][19][1])))+((rk[14][2]*
          Wpk[16][19][0])-(rk[14][0]*Wpk[16][19][2])));
        Vpk[16][19][2] = (((Cik[19][2][2]*VWri[16][19][2])+((Cik[19][0][2]*
          VWri[16][19][0])+(Cik[19][1][2]*VWri[16][19][1])))+((rk[14][0]*
          Wpk[16][19][1])-(rk[14][1]*Wpk[16][19][0])));
        VWri[16][20][0] = (Vpk[16][19][0]+((ri[15][2]*Wpk[16][19][1])-(ri[15][1]
          *Wpk[16][19][2])));
        VWri[16][20][1] = (Vpk[16][19][1]+((ri[15][0]*Wpk[16][19][2])-(ri[15][2]
          *Wpk[16][19][0])));
        VWri[16][20][2] = (Vpk[16][19][2]+((ri[15][1]*Wpk[16][19][0])-(ri[15][0]
          *Wpk[16][19][1])));
        Vpk[16][20][0] = (((Cik[20][2][0]*VWri[16][20][2])+((Cik[20][0][0]*
          VWri[16][20][0])+(Cik[20][1][0]*VWri[16][20][1])))+((rk[15][1]*
          Wpk[16][20][2])-(rk[15][2]*Wpk[16][20][1])));
        Vpk[16][20][1] = (((Cik[20][2][1]*VWri[16][20][2])+((Cik[20][0][1]*
          VWri[16][20][0])+(Cik[20][1][1]*VWri[16][20][1])))+((rk[15][2]*
          Wpk[16][20][0])-(rk[15][0]*Wpk[16][20][2])));
        Vpk[16][20][2] = (((Cik[20][2][2]*VWri[16][20][2])+((Cik[20][0][2]*
          VWri[16][20][0])+(Cik[20][1][2]*VWri[16][20][1])))+((rk[15][0]*
          Wpk[16][20][1])-(rk[15][1]*Wpk[16][20][0])));
        Vpk[17][17][0] = ((pin[17][2]*rk[12][1])-(pin[17][1]*rk[12][2]));
        Vpk[17][17][1] = ((pin[17][0]*rk[12][2])-(pin[17][2]*rk[12][0]));
        Vpk[17][17][2] = ((pin[17][1]*rk[12][0])-(pin[17][0]*rk[12][1]));
        VWri[17][18][0] = (Vpk[17][17][0]+((pin[17][1]*ri[13][2])-(pin[17][2]*
          ri[13][1])));
        VWri[17][18][1] = (Vpk[17][17][1]+((pin[17][2]*ri[13][0])-(pin[17][0]*
          ri[13][2])));
        VWri[17][18][2] = (Vpk[17][17][2]+((pin[17][0]*ri[13][1])-(pin[17][1]*
          ri[13][0])));
        Vpk[17][18][0] = (((Cik[18][2][0]*VWri[17][18][2])+((Cik[18][0][0]*
          VWri[17][18][0])+(Cik[18][1][0]*VWri[17][18][1])))+((rk[13][1]*
          Wpk[17][18][2])-(rk[13][2]*Wpk[17][18][1])));
        Vpk[17][18][1] = (((Cik[18][2][1]*VWri[17][18][2])+((Cik[18][0][1]*
          VWri[17][18][0])+(Cik[18][1][1]*VWri[17][18][1])))+((rk[13][2]*
          Wpk[17][18][0])-(rk[13][0]*Wpk[17][18][2])));
        Vpk[17][18][2] = (((Cik[18][2][2]*VWri[17][18][2])+((Cik[18][0][2]*
          VWri[17][18][0])+(Cik[18][1][2]*VWri[17][18][1])))+((rk[13][0]*
          Wpk[17][18][1])-(rk[13][1]*Wpk[17][18][0])));
        VWri[17][19][0] = (Vpk[17][18][0]+((ri[14][2]*Wpk[17][18][1])-(ri[14][1]
          *Wpk[17][18][2])));
        VWri[17][19][1] = (Vpk[17][18][1]+((ri[14][0]*Wpk[17][18][2])-(ri[14][2]
          *Wpk[17][18][0])));
        VWri[17][19][2] = (Vpk[17][18][2]+((ri[14][1]*Wpk[17][18][0])-(ri[14][0]
          *Wpk[17][18][1])));
        Vpk[17][19][0] = (((Cik[19][2][0]*VWri[17][19][2])+((Cik[19][0][0]*
          VWri[17][19][0])+(Cik[19][1][0]*VWri[17][19][1])))+((rk[14][1]*
          Wpk[17][19][2])-(rk[14][2]*Wpk[17][19][1])));
        Vpk[17][19][1] = (((Cik[19][2][1]*VWri[17][19][2])+((Cik[19][0][1]*
          VWri[17][19][0])+(Cik[19][1][1]*VWri[17][19][1])))+((rk[14][2]*
          Wpk[17][19][0])-(rk[14][0]*Wpk[17][19][2])));
        Vpk[17][19][2] = (((Cik[19][2][2]*VWri[17][19][2])+((Cik[19][0][2]*
          VWri[17][19][0])+(Cik[19][1][2]*VWri[17][19][1])))+((rk[14][0]*
          Wpk[17][19][1])-(rk[14][1]*Wpk[17][19][0])));
        VWri[17][20][0] = (Vpk[17][19][0]+((ri[15][2]*Wpk[17][19][1])-(ri[15][1]
          *Wpk[17][19][2])));
        VWri[17][20][1] = (Vpk[17][19][1]+((ri[15][0]*Wpk[17][19][2])-(ri[15][2]
          *Wpk[17][19][0])));
        VWri[17][20][2] = (Vpk[17][19][2]+((ri[15][1]*Wpk[17][19][0])-(ri[15][0]
          *Wpk[17][19][1])));
        Vpk[17][20][0] = (((Cik[20][2][0]*VWri[17][20][2])+((Cik[20][0][0]*
          VWri[17][20][0])+(Cik[20][1][0]*VWri[17][20][1])))+((rk[15][1]*
          Wpk[17][20][2])-(rk[15][2]*Wpk[17][20][1])));
        Vpk[17][20][1] = (((Cik[20][2][1]*VWri[17][20][2])+((Cik[20][0][1]*
          VWri[17][20][0])+(Cik[20][1][1]*VWri[17][20][1])))+((rk[15][2]*
          Wpk[17][20][0])-(rk[15][0]*Wpk[17][20][2])));
        Vpk[17][20][2] = (((Cik[20][2][2]*VWri[17][20][2])+((Cik[20][0][2]*
          VWri[17][20][0])+(Cik[20][1][2]*VWri[17][20][1])))+((rk[15][0]*
          Wpk[17][20][1])-(rk[15][1]*Wpk[17][20][0])));
        Vpk[18][18][0] = ((pin[18][2]*rk[13][1])-(pin[18][1]*rk[13][2]));
        Vpk[18][18][1] = ((pin[18][0]*rk[13][2])-(pin[18][2]*rk[13][0]));
        Vpk[18][18][2] = ((pin[18][1]*rk[13][0])-(pin[18][0]*rk[13][1]));
        VWri[18][19][0] = (Vpk[18][18][0]+((pin[18][1]*ri[14][2])-(pin[18][2]*
          ri[14][1])));
        VWri[18][19][1] = (Vpk[18][18][1]+((pin[18][2]*ri[14][0])-(pin[18][0]*
          ri[14][2])));
        VWri[18][19][2] = (Vpk[18][18][2]+((pin[18][0]*ri[14][1])-(pin[18][1]*
          ri[14][0])));
        Vpk[18][19][0] = (((Cik[19][2][0]*VWri[18][19][2])+((Cik[19][0][0]*
          VWri[18][19][0])+(Cik[19][1][0]*VWri[18][19][1])))+((rk[14][1]*
          Wpk[18][19][2])-(rk[14][2]*Wpk[18][19][1])));
        Vpk[18][19][1] = (((Cik[19][2][1]*VWri[18][19][2])+((Cik[19][0][1]*
          VWri[18][19][0])+(Cik[19][1][1]*VWri[18][19][1])))+((rk[14][2]*
          Wpk[18][19][0])-(rk[14][0]*Wpk[18][19][2])));
        Vpk[18][19][2] = (((Cik[19][2][2]*VWri[18][19][2])+((Cik[19][0][2]*
          VWri[18][19][0])+(Cik[19][1][2]*VWri[18][19][1])))+((rk[14][0]*
          Wpk[18][19][1])-(rk[14][1]*Wpk[18][19][0])));
        VWri[18][20][0] = (Vpk[18][19][0]+((ri[15][2]*Wpk[18][19][1])-(ri[15][1]
          *Wpk[18][19][2])));
        VWri[18][20][1] = (Vpk[18][19][1]+((ri[15][0]*Wpk[18][19][2])-(ri[15][2]
          *Wpk[18][19][0])));
        VWri[18][20][2] = (Vpk[18][19][2]+((ri[15][1]*Wpk[18][19][0])-(ri[15][0]
          *Wpk[18][19][1])));
        Vpk[18][20][0] = (((Cik[20][2][0]*VWri[18][20][2])+((Cik[20][0][0]*
          VWri[18][20][0])+(Cik[20][1][0]*VWri[18][20][1])))+((rk[15][1]*
          Wpk[18][20][2])-(rk[15][2]*Wpk[18][20][1])));
        Vpk[18][20][1] = (((Cik[20][2][1]*VWri[18][20][2])+((Cik[20][0][1]*
          VWri[18][20][0])+(Cik[20][1][1]*VWri[18][20][1])))+((rk[15][2]*
          Wpk[18][20][0])-(rk[15][0]*Wpk[18][20][2])));
        Vpk[18][20][2] = (((Cik[20][2][2]*VWri[18][20][2])+((Cik[20][0][2]*
          VWri[18][20][0])+(Cik[20][1][2]*VWri[18][20][1])))+((rk[15][0]*
          Wpk[18][20][1])-(rk[15][1]*Wpk[18][20][0])));
        Vpk[19][19][0] = ((pin[19][2]*rk[14][1])-(pin[19][1]*rk[14][2]));
        Vpk[19][19][1] = ((pin[19][0]*rk[14][2])-(pin[19][2]*rk[14][0]));
        Vpk[19][19][2] = ((pin[19][1]*rk[14][0])-(pin[19][0]*rk[14][1]));
        VWri[19][20][0] = (Vpk[19][19][0]+((pin[19][1]*ri[15][2])-(pin[19][2]*
          ri[15][1])));
        VWri[19][20][1] = (Vpk[19][19][1]+((pin[19][2]*ri[15][0])-(pin[19][0]*
          ri[15][2])));
        VWri[19][20][2] = (Vpk[19][19][2]+((pin[19][0]*ri[15][1])-(pin[19][1]*
          ri[15][0])));
        Vpk[19][20][0] = (((Cik[20][2][0]*VWri[19][20][2])+((Cik[20][0][0]*
          VWri[19][20][0])+(Cik[20][1][0]*VWri[19][20][1])))+((rk[15][1]*
          Wpk[19][20][2])-(rk[15][2]*Wpk[19][20][1])));
        Vpk[19][20][1] = (((Cik[20][2][1]*VWri[19][20][2])+((Cik[20][0][1]*
          VWri[19][20][0])+(Cik[20][1][1]*VWri[19][20][1])))+((rk[15][2]*
          Wpk[19][20][0])-(rk[15][0]*Wpk[19][20][2])));
        Vpk[19][20][2] = (((Cik[20][2][2]*VWri[19][20][2])+((Cik[20][0][2]*
          VWri[19][20][0])+(Cik[20][1][2]*VWri[19][20][1])))+((rk[15][0]*
          Wpk[19][20][1])-(rk[15][1]*Wpk[19][20][0])));
        Vpk[20][20][0] = ((pin[20][2]*rk[15][1])-(pin[20][1]*rk[15][2]));
        Vpk[20][20][1] = ((pin[20][0]*rk[15][2])-(pin[20][2]*rk[15][0]));
        Vpk[20][20][2] = ((pin[20][1]*rk[15][0])-(pin[20][0]*rk[15][1]));
        vpkflg = 1;
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain 2091 adds/subtracts/negates
                   2700 multiplies
                      0 divides
                    975 assignments
*/
}

void sddoltau(void)
{

/*
Compute effect of loop hinge torques
*/
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      0 assignments
*/
}

void sddoiner(void)
{

/*
Compute inertial accelerations and related temps
*/
    if (inerflg == 0) {
/*
Compute Otk (inertial angular acceleration)
*/
        Otk[6][0] = ((Wik[6][2]*wk[6][1])-(Wik[6][1]*wk[6][2]));
        Otk[6][1] = ((Wik[6][0]*wk[6][2])-(Wik[6][2]*wk[6][0]));
        Otk[6][2] = ((Wik[6][1]*wk[6][0])-(Wik[6][0]*wk[6][1]));
        Otk[7][0] = (((Cik[7][2][0]*Otk[6][2])+((Cik[7][0][0]*Otk[6][0])+(
          Cik[7][1][0]*Otk[6][1])))+((Wik[7][2]*wk[7][1])-(Wik[7][1]*wk[7][2])))
          ;
        Otk[7][1] = (((Cik[7][2][1]*Otk[6][2])+((Cik[7][0][1]*Otk[6][0])+(
          Cik[7][1][1]*Otk[6][1])))+((Wik[7][0]*wk[7][2])-(Wik[7][2]*wk[7][0])))
          ;
        Otk[7][2] = (((Cik[7][2][2]*Otk[6][2])+((Cik[7][0][2]*Otk[6][0])+(
          Cik[7][1][2]*Otk[6][1])))+((Wik[7][1]*wk[7][0])-(Wik[7][0]*wk[7][1])))
          ;
        Otk[8][0] = (((Cik[8][2][0]*Otk[7][2])+((Cik[8][0][0]*Otk[7][0])+(
          Cik[8][1][0]*Otk[7][1])))+((Wik[8][2]*wk[8][1])-(Wik[8][1]*wk[8][2])))
          ;
        Otk[8][1] = (((Cik[8][2][1]*Otk[7][2])+((Cik[8][0][1]*Otk[7][0])+(
          Cik[8][1][1]*Otk[7][1])))+((Wik[8][0]*wk[8][2])-(Wik[8][2]*wk[8][0])))
          ;
        Otk[8][2] = (((Cik[8][2][2]*Otk[7][2])+((Cik[8][0][2]*Otk[7][0])+(
          Cik[8][1][2]*Otk[7][1])))+((Wik[8][1]*wk[8][0])-(Wik[8][0]*wk[8][1])))
          ;
        Otk[9][0] = ((Wik[9][2]*wk[9][1])-(Wik[9][1]*wk[9][2]));
        Otk[9][1] = ((Wik[9][0]*wk[9][2])-(Wik[9][2]*wk[9][0]));
        Otk[9][2] = ((Wik[9][1]*wk[9][0])-(Wik[9][0]*wk[9][1]));
        Otk[10][0] = (((Cik[10][2][0]*Otk[9][2])+((Cik[10][0][0]*Otk[9][0])+(
          Cik[10][1][0]*Otk[9][1])))+((Wik[10][2]*wk[10][1])-(Wik[10][1]*
          wk[10][2])));
        Otk[10][1] = (((Cik[10][2][1]*Otk[9][2])+((Cik[10][0][1]*Otk[9][0])+(
          Cik[10][1][1]*Otk[9][1])))+((Wik[10][0]*wk[10][2])-(Wik[10][2]*
          wk[10][0])));
        Otk[10][2] = (((Cik[10][2][2]*Otk[9][2])+((Cik[10][0][2]*Otk[9][0])+(
          Cik[10][1][2]*Otk[9][1])))+((Wik[10][1]*wk[10][0])-(Wik[10][0]*
          wk[10][1])));
        Otk[11][0] = (((Cik[11][2][0]*Otk[10][2])+((Cik[11][0][0]*Otk[10][0])+(
          Cik[11][1][0]*Otk[10][1])))+((Wik[11][2]*wk[11][1])-(Wik[11][1]*
          wk[11][2])));
        Otk[11][1] = (((Cik[11][2][1]*Otk[10][2])+((Cik[11][0][1]*Otk[10][0])+(
          Cik[11][1][1]*Otk[10][1])))+((Wik[11][0]*wk[11][2])-(Wik[11][2]*
          wk[11][0])));
        Otk[11][2] = (((Cik[11][2][2]*Otk[10][2])+((Cik[11][0][2]*Otk[10][0])+(
          Cik[11][1][2]*Otk[10][1])))+((Wik[11][1]*wk[11][0])-(Wik[11][0]*
          wk[11][1])));
        Otk[12][0] = (((Cik[12][2][0]*Otk[11][2])+((Cik[12][0][0]*Otk[11][0])+(
          Cik[12][1][0]*Otk[11][1])))+((Wik[12][2]*wk[12][1])-(Wik[12][1]*
          wk[12][2])));
        Otk[12][1] = (((Cik[12][2][1]*Otk[11][2])+((Cik[12][0][1]*Otk[11][0])+(
          Cik[12][1][1]*Otk[11][1])))+((Wik[12][0]*wk[12][2])-(Wik[12][2]*
          wk[12][0])));
        Otk[12][2] = (((Cik[12][2][2]*Otk[11][2])+((Cik[12][0][2]*Otk[11][0])+(
          Cik[12][1][2]*Otk[11][1])))+((Wik[12][1]*wk[12][0])-(Wik[12][0]*
          wk[12][1])));
        Otk[13][0] = (((Cik[13][2][0]*Otk[12][2])+((Cik[13][0][0]*Otk[12][0])+(
          Cik[13][1][0]*Otk[12][1])))+((Wik[13][2]*wk[13][1])-(Wik[13][1]*
          wk[13][2])));
        Otk[13][1] = (((Cik[13][2][1]*Otk[12][2])+((Cik[13][0][1]*Otk[12][0])+(
          Cik[13][1][1]*Otk[12][1])))+((Wik[13][0]*wk[13][2])-(Wik[13][2]*
          wk[13][0])));
        Otk[13][2] = (((Cik[13][2][2]*Otk[12][2])+((Cik[13][0][2]*Otk[12][0])+(
          Cik[13][1][2]*Otk[12][1])))+((Wik[13][1]*wk[13][0])-(Wik[13][0]*
          wk[13][1])));
        Otk[14][0] = (((Cik[14][2][0]*Otk[13][2])+((Cik[14][0][0]*Otk[13][0])+(
          Cik[14][1][0]*Otk[13][1])))+((Wik[14][2]*wk[14][1])-(Wik[14][1]*
          wk[14][2])));
        Otk[14][1] = (((Cik[14][2][1]*Otk[13][2])+((Cik[14][0][1]*Otk[13][0])+(
          Cik[14][1][1]*Otk[13][1])))+((Wik[14][0]*wk[14][2])-(Wik[14][2]*
          wk[14][0])));
        Otk[14][2] = (((Cik[14][2][2]*Otk[13][2])+((Cik[14][0][2]*Otk[13][0])+(
          Cik[14][1][2]*Otk[13][1])))+((Wik[14][1]*wk[14][0])-(Wik[14][0]*
          wk[14][1])));
        Otk[15][0] = ((Wik[15][2]*wk[15][1])-(Wik[15][1]*wk[15][2]));
        Otk[15][1] = ((Wik[15][0]*wk[15][2])-(Wik[15][2]*wk[15][0]));
        Otk[15][2] = ((Wik[15][1]*wk[15][0])-(Wik[15][0]*wk[15][1]));
        Otk[16][0] = (((Cik[16][2][0]*Otk[15][2])+((Cik[16][0][0]*Otk[15][0])+(
          Cik[16][1][0]*Otk[15][1])))+((Wik[16][2]*wk[16][1])-(Wik[16][1]*
          wk[16][2])));
        Otk[16][1] = (((Cik[16][2][1]*Otk[15][2])+((Cik[16][0][1]*Otk[15][0])+(
          Cik[16][1][1]*Otk[15][1])))+((Wik[16][0]*wk[16][2])-(Wik[16][2]*
          wk[16][0])));
        Otk[16][2] = (((Cik[16][2][2]*Otk[15][2])+((Cik[16][0][2]*Otk[15][0])+(
          Cik[16][1][2]*Otk[15][1])))+((Wik[16][1]*wk[16][0])-(Wik[16][0]*
          wk[16][1])));
        Otk[17][0] = (((Cik[17][2][0]*Otk[16][2])+((Cik[17][0][0]*Otk[16][0])+(
          Cik[17][1][0]*Otk[16][1])))+((Wik[17][2]*wk[17][1])-(Wik[17][1]*
          wk[17][2])));
        Otk[17][1] = (((Cik[17][2][1]*Otk[16][2])+((Cik[17][0][1]*Otk[16][0])+(
          Cik[17][1][1]*Otk[16][1])))+((Wik[17][0]*wk[17][2])-(Wik[17][2]*
          wk[17][0])));
        Otk[17][2] = (((Cik[17][2][2]*Otk[16][2])+((Cik[17][0][2]*Otk[16][0])+(
          Cik[17][1][2]*Otk[16][1])))+((Wik[17][1]*wk[17][0])-(Wik[17][0]*
          wk[17][1])));
        Otk[18][0] = (((Cik[18][2][0]*Otk[17][2])+((Cik[18][0][0]*Otk[17][0])+(
          Cik[18][1][0]*Otk[17][1])))+((Wik[18][2]*wk[18][1])-(Wik[18][1]*
          wk[18][2])));
        Otk[18][1] = (((Cik[18][2][1]*Otk[17][2])+((Cik[18][0][1]*Otk[17][0])+(
          Cik[18][1][1]*Otk[17][1])))+((Wik[18][0]*wk[18][2])-(Wik[18][2]*
          wk[18][0])));
        Otk[18][2] = (((Cik[18][2][2]*Otk[17][2])+((Cik[18][0][2]*Otk[17][0])+(
          Cik[18][1][2]*Otk[17][1])))+((Wik[18][1]*wk[18][0])-(Wik[18][0]*
          wk[18][1])));
        Otk[19][0] = (((Cik[19][2][0]*Otk[18][2])+((Cik[19][0][0]*Otk[18][0])+(
          Cik[19][1][0]*Otk[18][1])))+((Wik[19][2]*wk[19][1])-(Wik[19][1]*
          wk[19][2])));
        Otk[19][1] = (((Cik[19][2][1]*Otk[18][2])+((Cik[19][0][1]*Otk[18][0])+(
          Cik[19][1][1]*Otk[18][1])))+((Wik[19][0]*wk[19][2])-(Wik[19][2]*
          wk[19][0])));
        Otk[19][2] = (((Cik[19][2][2]*Otk[18][2])+((Cik[19][0][2]*Otk[18][0])+(
          Cik[19][1][2]*Otk[18][1])))+((Wik[19][1]*wk[19][0])-(Wik[19][0]*
          wk[19][1])));
        Otk[20][0] = (((Cik[20][2][0]*Otk[19][2])+((Cik[20][0][0]*Otk[19][0])+(
          Cik[20][1][0]*Otk[19][1])))+((Wik[20][2]*wk[20][1])-(Wik[20][1]*
          wk[20][2])));
        Otk[20][1] = (((Cik[20][2][1]*Otk[19][2])+((Cik[20][0][1]*Otk[19][0])+(
          Cik[20][1][1]*Otk[19][1])))+((Wik[20][0]*wk[20][2])-(Wik[20][2]*
          wk[20][0])));
        Otk[20][2] = (((Cik[20][2][2]*Otk[19][2])+((Cik[20][0][2]*Otk[19][0])+(
          Cik[20][1][2]*Otk[19][1])))+((Wik[20][1]*wk[20][0])-(Wik[20][0]*
          wk[20][1])));
/*
Compute Atk (inertial linear acceleration)
*/
        Atk[5][0] = ((u[4]*Wkrpk[5][2])-(u[5]*Wkrpk[5][1]));
        Atk[5][1] = ((u[5]*Wkrpk[5][0])-(u[3]*Wkrpk[5][2]));
        Atk[5][2] = ((u[3]*Wkrpk[5][1])-(u[4]*Wkrpk[5][0]));
        AiOiWi[6][0] = (Atk[5][0]+((u[4]*Wirk[6][2])-(u[5]*Wirk[6][1])));
        AiOiWi[6][1] = (Atk[5][1]+((u[5]*Wirk[6][0])-(u[3]*Wirk[6][2])));
        AiOiWi[6][2] = (Atk[5][2]+((u[3]*Wirk[6][1])-(u[4]*Wirk[6][0])));
        Atk[6][0] = (((AiOiWi[6][2]*Cik[6][2][0])+((AiOiWi[6][0]*Cik[6][0][0])+(
          AiOiWi[6][1]*Cik[6][1][0])))+(((Otk[6][2]*rk[1][1])-(Otk[6][1]*
          rk[1][2]))+((wk[6][1]*Wkrpk[6][2])-(wk[6][2]*Wkrpk[6][1]))));
        Atk[6][1] = (((AiOiWi[6][2]*Cik[6][2][1])+((AiOiWi[6][0]*Cik[6][0][1])+(
          AiOiWi[6][1]*Cik[6][1][1])))+(((Otk[6][0]*rk[1][2])-(Otk[6][2]*
          rk[1][0]))+((wk[6][2]*Wkrpk[6][0])-(wk[6][0]*Wkrpk[6][2]))));
        Atk[6][2] = (((AiOiWi[6][2]*Cik[6][2][2])+((AiOiWi[6][0]*Cik[6][0][2])+(
          AiOiWi[6][1]*Cik[6][1][2])))+(((Otk[6][1]*rk[1][0])-(Otk[6][0]*
          rk[1][1]))+((wk[6][0]*Wkrpk[6][1])-(wk[6][1]*Wkrpk[6][0]))));
        AiOiWi[7][0] = (Atk[6][0]+(((Otk[6][1]*ri[2][2])-(Otk[6][2]*ri[2][1]))+(
          (Wirk[7][2]*wk[6][1])-(Wirk[7][1]*wk[6][2]))));
        AiOiWi[7][1] = (Atk[6][1]+(((Otk[6][2]*ri[2][0])-(Otk[6][0]*ri[2][2]))+(
          (Wirk[7][0]*wk[6][2])-(Wirk[7][2]*wk[6][0]))));
        AiOiWi[7][2] = (Atk[6][2]+(((Otk[6][0]*ri[2][1])-(Otk[6][1]*ri[2][0]))+(
          (Wirk[7][1]*wk[6][0])-(Wirk[7][0]*wk[6][1]))));
        Atk[7][0] = (((AiOiWi[7][2]*Cik[7][2][0])+((AiOiWi[7][0]*Cik[7][0][0])+(
          AiOiWi[7][1]*Cik[7][1][0])))+(((Otk[7][2]*rk[2][1])-(Otk[7][1]*
          rk[2][2]))+((wk[7][1]*Wkrpk[7][2])-(wk[7][2]*Wkrpk[7][1]))));
        Atk[7][1] = (((AiOiWi[7][2]*Cik[7][2][1])+((AiOiWi[7][0]*Cik[7][0][1])+(
          AiOiWi[7][1]*Cik[7][1][1])))+(((Otk[7][0]*rk[2][2])-(Otk[7][2]*
          rk[2][0]))+((wk[7][2]*Wkrpk[7][0])-(wk[7][0]*Wkrpk[7][2]))));
        Atk[7][2] = (((AiOiWi[7][2]*Cik[7][2][2])+((AiOiWi[7][0]*Cik[7][0][2])+(
          AiOiWi[7][1]*Cik[7][1][2])))+(((Otk[7][1]*rk[2][0])-(Otk[7][0]*
          rk[2][1]))+((wk[7][0]*Wkrpk[7][1])-(wk[7][1]*Wkrpk[7][0]))));
        AiOiWi[8][0] = (Atk[7][0]+(((Otk[7][1]*ri[3][2])-(Otk[7][2]*ri[3][1]))+(
          (Wirk[8][2]*wk[7][1])-(Wirk[8][1]*wk[7][2]))));
        AiOiWi[8][1] = (Atk[7][1]+(((Otk[7][2]*ri[3][0])-(Otk[7][0]*ri[3][2]))+(
          (Wirk[8][0]*wk[7][2])-(Wirk[8][2]*wk[7][0]))));
        AiOiWi[8][2] = (Atk[7][2]+(((Otk[7][0]*ri[3][1])-(Otk[7][1]*ri[3][0]))+(
          (Wirk[8][1]*wk[7][0])-(Wirk[8][0]*wk[7][1]))));
        Atk[8][0] = (((AiOiWi[8][2]*Cik[8][2][0])+((AiOiWi[8][0]*Cik[8][0][0])+(
          AiOiWi[8][1]*Cik[8][1][0])))+(((Otk[8][2]*rk[3][1])-(Otk[8][1]*
          rk[3][2]))+((wk[8][1]*Wkrpk[8][2])-(wk[8][2]*Wkrpk[8][1]))));
        Atk[8][1] = (((AiOiWi[8][2]*Cik[8][2][1])+((AiOiWi[8][0]*Cik[8][0][1])+(
          AiOiWi[8][1]*Cik[8][1][1])))+(((Otk[8][0]*rk[3][2])-(Otk[8][2]*
          rk[3][0]))+((wk[8][2]*Wkrpk[8][0])-(wk[8][0]*Wkrpk[8][2]))));
        Atk[8][2] = (((AiOiWi[8][2]*Cik[8][2][2])+((AiOiWi[8][0]*Cik[8][0][2])+(
          AiOiWi[8][1]*Cik[8][1][2])))+(((Otk[8][1]*rk[3][0])-(Otk[8][0]*
          rk[3][1]))+((wk[8][0]*Wkrpk[8][1])-(wk[8][1]*Wkrpk[8][0]))));
        AiOiWi[9][0] = (Atk[5][0]+((u[4]*Wirk[9][2])-(u[5]*Wirk[9][1])));
        AiOiWi[9][1] = (Atk[5][1]+((u[5]*Wirk[9][0])-(u[3]*Wirk[9][2])));
        AiOiWi[9][2] = (Atk[5][2]+((u[3]*Wirk[9][1])-(u[4]*Wirk[9][0])));
        Atk[9][0] = (((AiOiWi[9][2]*Cik[9][2][0])+((AiOiWi[9][0]*Cik[9][0][0])+(
          AiOiWi[9][1]*Cik[9][1][0])))+(((Otk[9][2]*rk[4][1])-(Otk[9][1]*
          rk[4][2]))+((wk[9][1]*Wkrpk[9][2])-(wk[9][2]*Wkrpk[9][1]))));
        Atk[9][1] = (((AiOiWi[9][2]*Cik[9][2][1])+((AiOiWi[9][0]*Cik[9][0][1])+(
          AiOiWi[9][1]*Cik[9][1][1])))+(((Otk[9][0]*rk[4][2])-(Otk[9][2]*
          rk[4][0]))+((wk[9][2]*Wkrpk[9][0])-(wk[9][0]*Wkrpk[9][2]))));
        Atk[9][2] = (((AiOiWi[9][2]*Cik[9][2][2])+((AiOiWi[9][0]*Cik[9][0][2])+(
          AiOiWi[9][1]*Cik[9][1][2])))+(((Otk[9][1]*rk[4][0])-(Otk[9][0]*
          rk[4][1]))+((wk[9][0]*Wkrpk[9][1])-(wk[9][1]*Wkrpk[9][0]))));
        AiOiWi[10][0] = (Atk[9][0]+(((Otk[9][1]*ri[5][2])-(Otk[9][2]*ri[5][1]))+
          ((Wirk[10][2]*wk[9][1])-(Wirk[10][1]*wk[9][2]))));
        AiOiWi[10][1] = (Atk[9][1]+(((Otk[9][2]*ri[5][0])-(Otk[9][0]*ri[5][2]))+
          ((Wirk[10][0]*wk[9][2])-(Wirk[10][2]*wk[9][0]))));
        AiOiWi[10][2] = (Atk[9][2]+(((Otk[9][0]*ri[5][1])-(Otk[9][1]*ri[5][0]))+
          ((Wirk[10][1]*wk[9][0])-(Wirk[10][0]*wk[9][1]))));
        Atk[10][0] = (((AiOiWi[10][2]*Cik[10][2][0])+((AiOiWi[10][0]*
          Cik[10][0][0])+(AiOiWi[10][1]*Cik[10][1][0])))+(((Otk[10][2]*rk[5][1])
          -(Otk[10][1]*rk[5][2]))+((wk[10][1]*Wkrpk[10][2])-(wk[10][2]*
          Wkrpk[10][1]))));
        Atk[10][1] = (((AiOiWi[10][2]*Cik[10][2][1])+((AiOiWi[10][0]*
          Cik[10][0][1])+(AiOiWi[10][1]*Cik[10][1][1])))+(((Otk[10][0]*rk[5][2])
          -(Otk[10][2]*rk[5][0]))+((wk[10][2]*Wkrpk[10][0])-(wk[10][0]*
          Wkrpk[10][2]))));
        Atk[10][2] = (((AiOiWi[10][2]*Cik[10][2][2])+((AiOiWi[10][0]*
          Cik[10][0][2])+(AiOiWi[10][1]*Cik[10][1][2])))+(((Otk[10][1]*rk[5][0])
          -(Otk[10][0]*rk[5][1]))+((wk[10][0]*Wkrpk[10][1])-(wk[10][1]*
          Wkrpk[10][0]))));
        AiOiWi[11][0] = (Atk[10][0]+(((Otk[10][1]*ri[6][2])-(Otk[10][2]*ri[6][1]
          ))+((Wirk[11][2]*wk[10][1])-(Wirk[11][1]*wk[10][2]))));
        AiOiWi[11][1] = (Atk[10][1]+(((Otk[10][2]*ri[6][0])-(Otk[10][0]*ri[6][2]
          ))+((Wirk[11][0]*wk[10][2])-(Wirk[11][2]*wk[10][0]))));
        AiOiWi[11][2] = (Atk[10][2]+(((Otk[10][0]*ri[6][1])-(Otk[10][1]*ri[6][0]
          ))+((Wirk[11][1]*wk[10][0])-(Wirk[11][0]*wk[10][1]))));
        Atk[11][0] = (((AiOiWi[11][2]*Cik[11][2][0])+((AiOiWi[11][0]*
          Cik[11][0][0])+(AiOiWi[11][1]*Cik[11][1][0])))+(((Otk[11][2]*rk[6][1])
          -(Otk[11][1]*rk[6][2]))+((wk[11][1]*Wkrpk[11][2])-(wk[11][2]*
          Wkrpk[11][1]))));
        Atk[11][1] = (((AiOiWi[11][2]*Cik[11][2][1])+((AiOiWi[11][0]*
          Cik[11][0][1])+(AiOiWi[11][1]*Cik[11][1][1])))+(((Otk[11][0]*rk[6][2])
          -(Otk[11][2]*rk[6][0]))+((wk[11][2]*Wkrpk[11][0])-(wk[11][0]*
          Wkrpk[11][2]))));
        Atk[11][2] = (((AiOiWi[11][2]*Cik[11][2][2])+((AiOiWi[11][0]*
          Cik[11][0][2])+(AiOiWi[11][1]*Cik[11][1][2])))+(((Otk[11][1]*rk[6][0])
          -(Otk[11][0]*rk[6][1]))+((wk[11][0]*Wkrpk[11][1])-(wk[11][1]*
          Wkrpk[11][0]))));
        AiOiWi[12][0] = (Atk[11][0]+(((Otk[11][1]*ri[7][2])-(Otk[11][2]*ri[7][1]
          ))+((Wirk[12][2]*wk[11][1])-(Wirk[12][1]*wk[11][2]))));
        AiOiWi[12][1] = (Atk[11][1]+(((Otk[11][2]*ri[7][0])-(Otk[11][0]*ri[7][2]
          ))+((Wirk[12][0]*wk[11][2])-(Wirk[12][2]*wk[11][0]))));
        AiOiWi[12][2] = (Atk[11][2]+(((Otk[11][0]*ri[7][1])-(Otk[11][1]*ri[7][0]
          ))+((Wirk[12][1]*wk[11][0])-(Wirk[12][0]*wk[11][1]))));
        Atk[12][0] = (((AiOiWi[12][2]*Cik[12][2][0])+((AiOiWi[12][0]*
          Cik[12][0][0])+(AiOiWi[12][1]*Cik[12][1][0])))+(((Otk[12][2]*rk[7][1])
          -(Otk[12][1]*rk[7][2]))+((wk[12][1]*Wkrpk[12][2])-(wk[12][2]*
          Wkrpk[12][1]))));
        Atk[12][1] = (((AiOiWi[12][2]*Cik[12][2][1])+((AiOiWi[12][0]*
          Cik[12][0][1])+(AiOiWi[12][1]*Cik[12][1][1])))+(((Otk[12][0]*rk[7][2])
          -(Otk[12][2]*rk[7][0]))+((wk[12][2]*Wkrpk[12][0])-(wk[12][0]*
          Wkrpk[12][2]))));
        Atk[12][2] = (((AiOiWi[12][2]*Cik[12][2][2])+((AiOiWi[12][0]*
          Cik[12][0][2])+(AiOiWi[12][1]*Cik[12][1][2])))+(((Otk[12][1]*rk[7][0])
          -(Otk[12][0]*rk[7][1]))+((wk[12][0]*Wkrpk[12][1])-(wk[12][1]*
          Wkrpk[12][0]))));
        AiOiWi[13][0] = (Atk[12][0]+(((Otk[12][1]*ri[8][2])-(Otk[12][2]*ri[8][1]
          ))+((Wirk[13][2]*wk[12][1])-(Wirk[13][1]*wk[12][2]))));
        AiOiWi[13][1] = (Atk[12][1]+(((Otk[12][2]*ri[8][0])-(Otk[12][0]*ri[8][2]
          ))+((Wirk[13][0]*wk[12][2])-(Wirk[13][2]*wk[12][0]))));
        AiOiWi[13][2] = (Atk[12][2]+(((Otk[12][0]*ri[8][1])-(Otk[12][1]*ri[8][0]
          ))+((Wirk[13][1]*wk[12][0])-(Wirk[13][0]*wk[12][1]))));
        Atk[13][0] = (((AiOiWi[13][2]*Cik[13][2][0])+((AiOiWi[13][0]*
          Cik[13][0][0])+(AiOiWi[13][1]*Cik[13][1][0])))+(((Otk[13][2]*rk[8][1])
          -(Otk[13][1]*rk[8][2]))+((wk[13][1]*Wkrpk[13][2])-(wk[13][2]*
          Wkrpk[13][1]))));
        Atk[13][1] = (((AiOiWi[13][2]*Cik[13][2][1])+((AiOiWi[13][0]*
          Cik[13][0][1])+(AiOiWi[13][1]*Cik[13][1][1])))+(((Otk[13][0]*rk[8][2])
          -(Otk[13][2]*rk[8][0]))+((wk[13][2]*Wkrpk[13][0])-(wk[13][0]*
          Wkrpk[13][2]))));
        Atk[13][2] = (((AiOiWi[13][2]*Cik[13][2][2])+((AiOiWi[13][0]*
          Cik[13][0][2])+(AiOiWi[13][1]*Cik[13][1][2])))+(((Otk[13][1]*rk[8][0])
          -(Otk[13][0]*rk[8][1]))+((wk[13][0]*Wkrpk[13][1])-(wk[13][1]*
          Wkrpk[13][0]))));
        AiOiWi[14][0] = (Atk[13][0]+(((Otk[13][1]*ri[9][2])-(Otk[13][2]*ri[9][1]
          ))+((Wirk[14][2]*wk[13][1])-(Wirk[14][1]*wk[13][2]))));
        AiOiWi[14][1] = (Atk[13][1]+(((Otk[13][2]*ri[9][0])-(Otk[13][0]*ri[9][2]
          ))+((Wirk[14][0]*wk[13][2])-(Wirk[14][2]*wk[13][0]))));
        AiOiWi[14][2] = (Atk[13][2]+(((Otk[13][0]*ri[9][1])-(Otk[13][1]*ri[9][0]
          ))+((Wirk[14][1]*wk[13][0])-(Wirk[14][0]*wk[13][1]))));
        Atk[14][0] = (((AiOiWi[14][2]*Cik[14][2][0])+((AiOiWi[14][0]*
          Cik[14][0][0])+(AiOiWi[14][1]*Cik[14][1][0])))+(((Otk[14][2]*rk[9][1])
          -(Otk[14][1]*rk[9][2]))+((wk[14][1]*Wkrpk[14][2])-(wk[14][2]*
          Wkrpk[14][1]))));
        Atk[14][1] = (((AiOiWi[14][2]*Cik[14][2][1])+((AiOiWi[14][0]*
          Cik[14][0][1])+(AiOiWi[14][1]*Cik[14][1][1])))+(((Otk[14][0]*rk[9][2])
          -(Otk[14][2]*rk[9][0]))+((wk[14][2]*Wkrpk[14][0])-(wk[14][0]*
          Wkrpk[14][2]))));
        Atk[14][2] = (((AiOiWi[14][2]*Cik[14][2][2])+((AiOiWi[14][0]*
          Cik[14][0][2])+(AiOiWi[14][1]*Cik[14][1][2])))+(((Otk[14][1]*rk[9][0])
          -(Otk[14][0]*rk[9][1]))+((wk[14][0]*Wkrpk[14][1])-(wk[14][1]*
          Wkrpk[14][0]))));
        AiOiWi[15][0] = (Atk[5][0]+((u[4]*Wirk[15][2])-(u[5]*Wirk[15][1])));
        AiOiWi[15][1] = (Atk[5][1]+((u[5]*Wirk[15][0])-(u[3]*Wirk[15][2])));
        AiOiWi[15][2] = (Atk[5][2]+((u[3]*Wirk[15][1])-(u[4]*Wirk[15][0])));
        Atk[15][0] = (((AiOiWi[15][2]*Cik[15][2][0])+((AiOiWi[15][0]*
          Cik[15][0][0])+(AiOiWi[15][1]*Cik[15][1][0])))+(((Otk[15][2]*rk[10][1]
          )-(Otk[15][1]*rk[10][2]))+((wk[15][1]*Wkrpk[15][2])-(wk[15][2]*
          Wkrpk[15][1]))));
        Atk[15][1] = (((AiOiWi[15][2]*Cik[15][2][1])+((AiOiWi[15][0]*
          Cik[15][0][1])+(AiOiWi[15][1]*Cik[15][1][1])))+(((Otk[15][0]*rk[10][2]
          )-(Otk[15][2]*rk[10][0]))+((wk[15][2]*Wkrpk[15][0])-(wk[15][0]*
          Wkrpk[15][2]))));
        Atk[15][2] = (((AiOiWi[15][2]*Cik[15][2][2])+((AiOiWi[15][0]*
          Cik[15][0][2])+(AiOiWi[15][1]*Cik[15][1][2])))+(((Otk[15][1]*rk[10][0]
          )-(Otk[15][0]*rk[10][1]))+((wk[15][0]*Wkrpk[15][1])-(wk[15][1]*
          Wkrpk[15][0]))));
        AiOiWi[16][0] = (Atk[15][0]+(((Otk[15][1]*ri[11][2])-(Otk[15][2]*
          ri[11][1]))+((Wirk[16][2]*wk[15][1])-(Wirk[16][1]*wk[15][2]))));
        AiOiWi[16][1] = (Atk[15][1]+(((Otk[15][2]*ri[11][0])-(Otk[15][0]*
          ri[11][2]))+((Wirk[16][0]*wk[15][2])-(Wirk[16][2]*wk[15][0]))));
        AiOiWi[16][2] = (Atk[15][2]+(((Otk[15][0]*ri[11][1])-(Otk[15][1]*
          ri[11][0]))+((Wirk[16][1]*wk[15][0])-(Wirk[16][0]*wk[15][1]))));
        Atk[16][0] = (((AiOiWi[16][2]*Cik[16][2][0])+((AiOiWi[16][0]*
          Cik[16][0][0])+(AiOiWi[16][1]*Cik[16][1][0])))+(((Otk[16][2]*rk[11][1]
          )-(Otk[16][1]*rk[11][2]))+((wk[16][1]*Wkrpk[16][2])-(wk[16][2]*
          Wkrpk[16][1]))));
        Atk[16][1] = (((AiOiWi[16][2]*Cik[16][2][1])+((AiOiWi[16][0]*
          Cik[16][0][1])+(AiOiWi[16][1]*Cik[16][1][1])))+(((Otk[16][0]*rk[11][2]
          )-(Otk[16][2]*rk[11][0]))+((wk[16][2]*Wkrpk[16][0])-(wk[16][0]*
          Wkrpk[16][2]))));
        Atk[16][2] = (((AiOiWi[16][2]*Cik[16][2][2])+((AiOiWi[16][0]*
          Cik[16][0][2])+(AiOiWi[16][1]*Cik[16][1][2])))+(((Otk[16][1]*rk[11][0]
          )-(Otk[16][0]*rk[11][1]))+((wk[16][0]*Wkrpk[16][1])-(wk[16][1]*
          Wkrpk[16][0]))));
        AiOiWi[17][0] = (Atk[16][0]+(((Otk[16][1]*ri[12][2])-(Otk[16][2]*
          ri[12][1]))+((Wirk[17][2]*wk[16][1])-(Wirk[17][1]*wk[16][2]))));
        AiOiWi[17][1] = (Atk[16][1]+(((Otk[16][2]*ri[12][0])-(Otk[16][0]*
          ri[12][2]))+((Wirk[17][0]*wk[16][2])-(Wirk[17][2]*wk[16][0]))));
        AiOiWi[17][2] = (Atk[16][2]+(((Otk[16][0]*ri[12][1])-(Otk[16][1]*
          ri[12][0]))+((Wirk[17][1]*wk[16][0])-(Wirk[17][0]*wk[16][1]))));
        Atk[17][0] = (((AiOiWi[17][2]*Cik[17][2][0])+((AiOiWi[17][0]*
          Cik[17][0][0])+(AiOiWi[17][1]*Cik[17][1][0])))+(((Otk[17][2]*rk[12][1]
          )-(Otk[17][1]*rk[12][2]))+((wk[17][1]*Wkrpk[17][2])-(wk[17][2]*
          Wkrpk[17][1]))));
        Atk[17][1] = (((AiOiWi[17][2]*Cik[17][2][1])+((AiOiWi[17][0]*
          Cik[17][0][1])+(AiOiWi[17][1]*Cik[17][1][1])))+(((Otk[17][0]*rk[12][2]
          )-(Otk[17][2]*rk[12][0]))+((wk[17][2]*Wkrpk[17][0])-(wk[17][0]*
          Wkrpk[17][2]))));
        Atk[17][2] = (((AiOiWi[17][2]*Cik[17][2][2])+((AiOiWi[17][0]*
          Cik[17][0][2])+(AiOiWi[17][1]*Cik[17][1][2])))+(((Otk[17][1]*rk[12][0]
          )-(Otk[17][0]*rk[12][1]))+((wk[17][0]*Wkrpk[17][1])-(wk[17][1]*
          Wkrpk[17][0]))));
        AiOiWi[18][0] = (Atk[17][0]+(((Otk[17][1]*ri[13][2])-(Otk[17][2]*
          ri[13][1]))+((Wirk[18][2]*wk[17][1])-(Wirk[18][1]*wk[17][2]))));
        AiOiWi[18][1] = (Atk[17][1]+(((Otk[17][2]*ri[13][0])-(Otk[17][0]*
          ri[13][2]))+((Wirk[18][0]*wk[17][2])-(Wirk[18][2]*wk[17][0]))));
        AiOiWi[18][2] = (Atk[17][2]+(((Otk[17][0]*ri[13][1])-(Otk[17][1]*
          ri[13][0]))+((Wirk[18][1]*wk[17][0])-(Wirk[18][0]*wk[17][1]))));
        Atk[18][0] = (((AiOiWi[18][2]*Cik[18][2][0])+((AiOiWi[18][0]*
          Cik[18][0][0])+(AiOiWi[18][1]*Cik[18][1][0])))+(((Otk[18][2]*rk[13][1]
          )-(Otk[18][1]*rk[13][2]))+((wk[18][1]*Wkrpk[18][2])-(wk[18][2]*
          Wkrpk[18][1]))));
        Atk[18][1] = (((AiOiWi[18][2]*Cik[18][2][1])+((AiOiWi[18][0]*
          Cik[18][0][1])+(AiOiWi[18][1]*Cik[18][1][1])))+(((Otk[18][0]*rk[13][2]
          )-(Otk[18][2]*rk[13][0]))+((wk[18][2]*Wkrpk[18][0])-(wk[18][0]*
          Wkrpk[18][2]))));
        Atk[18][2] = (((AiOiWi[18][2]*Cik[18][2][2])+((AiOiWi[18][0]*
          Cik[18][0][2])+(AiOiWi[18][1]*Cik[18][1][2])))+(((Otk[18][1]*rk[13][0]
          )-(Otk[18][0]*rk[13][1]))+((wk[18][0]*Wkrpk[18][1])-(wk[18][1]*
          Wkrpk[18][0]))));
        AiOiWi[19][0] = (Atk[18][0]+(((Otk[18][1]*ri[14][2])-(Otk[18][2]*
          ri[14][1]))+((Wirk[19][2]*wk[18][1])-(Wirk[19][1]*wk[18][2]))));
        AiOiWi[19][1] = (Atk[18][1]+(((Otk[18][2]*ri[14][0])-(Otk[18][0]*
          ri[14][2]))+((Wirk[19][0]*wk[18][2])-(Wirk[19][2]*wk[18][0]))));
        AiOiWi[19][2] = (Atk[18][2]+(((Otk[18][0]*ri[14][1])-(Otk[18][1]*
          ri[14][0]))+((Wirk[19][1]*wk[18][0])-(Wirk[19][0]*wk[18][1]))));
        Atk[19][0] = (((AiOiWi[19][2]*Cik[19][2][0])+((AiOiWi[19][0]*
          Cik[19][0][0])+(AiOiWi[19][1]*Cik[19][1][0])))+(((Otk[19][2]*rk[14][1]
          )-(Otk[19][1]*rk[14][2]))+((wk[19][1]*Wkrpk[19][2])-(wk[19][2]*
          Wkrpk[19][1]))));
        Atk[19][1] = (((AiOiWi[19][2]*Cik[19][2][1])+((AiOiWi[19][0]*
          Cik[19][0][1])+(AiOiWi[19][1]*Cik[19][1][1])))+(((Otk[19][0]*rk[14][2]
          )-(Otk[19][2]*rk[14][0]))+((wk[19][2]*Wkrpk[19][0])-(wk[19][0]*
          Wkrpk[19][2]))));
        Atk[19][2] = (((AiOiWi[19][2]*Cik[19][2][2])+((AiOiWi[19][0]*
          Cik[19][0][2])+(AiOiWi[19][1]*Cik[19][1][2])))+(((Otk[19][1]*rk[14][0]
          )-(Otk[19][0]*rk[14][1]))+((wk[19][0]*Wkrpk[19][1])-(wk[19][1]*
          Wkrpk[19][0]))));
        AiOiWi[20][0] = (Atk[19][0]+(((Otk[19][1]*ri[15][2])-(Otk[19][2]*
          ri[15][1]))+((Wirk[20][2]*wk[19][1])-(Wirk[20][1]*wk[19][2]))));
        AiOiWi[20][1] = (Atk[19][1]+(((Otk[19][2]*ri[15][0])-(Otk[19][0]*
          ri[15][2]))+((Wirk[20][0]*wk[19][2])-(Wirk[20][2]*wk[19][0]))));
        AiOiWi[20][2] = (Atk[19][2]+(((Otk[19][0]*ri[15][1])-(Otk[19][1]*
          ri[15][0]))+((Wirk[20][1]*wk[19][0])-(Wirk[20][0]*wk[19][1]))));
        Atk[20][0] = (((AiOiWi[20][2]*Cik[20][2][0])+((AiOiWi[20][0]*
          Cik[20][0][0])+(AiOiWi[20][1]*Cik[20][1][0])))+(((Otk[20][2]*rk[15][1]
          )-(Otk[20][1]*rk[15][2]))+((wk[20][1]*Wkrpk[20][2])-(wk[20][2]*
          Wkrpk[20][1]))));
        Atk[20][1] = (((AiOiWi[20][2]*Cik[20][2][1])+((AiOiWi[20][0]*
          Cik[20][0][1])+(AiOiWi[20][1]*Cik[20][1][1])))+(((Otk[20][0]*rk[15][2]
          )-(Otk[20][2]*rk[15][0]))+((wk[20][2]*Wkrpk[20][0])-(wk[20][0]*
          Wkrpk[20][2]))));
        Atk[20][2] = (((AiOiWi[20][2]*Cik[20][2][2])+((AiOiWi[20][0]*
          Cik[20][0][2])+(AiOiWi[20][1]*Cik[20][1][2])))+(((Otk[20][1]*rk[15][0]
          )-(Otk[20][0]*rk[15][1]))+((wk[20][0]*Wkrpk[20][1])-(wk[20][1]*
          Wkrpk[20][0]))));
        inerflg = 1;
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  588 adds/subtracts/negates
                    681 multiplies
                      0 divides
                    138 assignments
*/
}

void sddofs0(void)
{

/*
Compute effect of all applied loads
*/
    if (fs0flg == 0) {
        sddoltau();
        sddoiner();
/*
Compute Fstar (forces)
*/
        Fstar[5][0] = ((mk[0]*(Atk[5][0]-gk[3][0]))-ufk[0][0]);
        Fstar[5][1] = ((mk[0]*(Atk[5][1]-gk[3][1]))-ufk[0][1]);
        Fstar[5][2] = ((mk[0]*(Atk[5][2]-gk[3][2]))-ufk[0][2]);
        Fstar[6][0] = ((mk[1]*(Atk[6][0]-gk[6][0]))-ufk[1][0]);
        Fstar[6][1] = ((mk[1]*(Atk[6][1]-gk[6][1]))-ufk[1][1]);
        Fstar[6][2] = ((mk[1]*(Atk[6][2]-gk[6][2]))-ufk[1][2]);
        Fstar[7][0] = ((mk[2]*(Atk[7][0]-gk[7][0]))-ufk[2][0]);
        Fstar[7][1] = ((mk[2]*(Atk[7][1]-gk[7][1]))-ufk[2][1]);
        Fstar[7][2] = ((mk[2]*(Atk[7][2]-gk[7][2]))-ufk[2][2]);
        Fstar[8][0] = ((mk[3]*(Atk[8][0]-gk[8][0]))-ufk[3][0]);
        Fstar[8][1] = ((mk[3]*(Atk[8][1]-gk[8][1]))-ufk[3][1]);
        Fstar[8][2] = ((mk[3]*(Atk[8][2]-gk[8][2]))-ufk[3][2]);
        Fstar[9][0] = ((mk[4]*(Atk[9][0]-gk[9][0]))-ufk[4][0]);
        Fstar[9][1] = ((mk[4]*(Atk[9][1]-gk[9][1]))-ufk[4][1]);
        Fstar[9][2] = ((mk[4]*(Atk[9][2]-gk[9][2]))-ufk[4][2]);
        Fstar[10][0] = ((mk[5]*(Atk[10][0]-gk[10][0]))-ufk[5][0]);
        Fstar[10][1] = ((mk[5]*(Atk[10][1]-gk[10][1]))-ufk[5][1]);
        Fstar[10][2] = ((mk[5]*(Atk[10][2]-gk[10][2]))-ufk[5][2]);
        Fstar[11][0] = ((mk[6]*(Atk[11][0]-gk[11][0]))-ufk[6][0]);
        Fstar[11][1] = ((mk[6]*(Atk[11][1]-gk[11][1]))-ufk[6][1]);
        Fstar[11][2] = ((mk[6]*(Atk[11][2]-gk[11][2]))-ufk[6][2]);
        Fstar[12][0] = ((mk[7]*(Atk[12][0]-gk[12][0]))-ufk[7][0]);
        Fstar[12][1] = ((mk[7]*(Atk[12][1]-gk[12][1]))-ufk[7][1]);
        Fstar[12][2] = ((mk[7]*(Atk[12][2]-gk[12][2]))-ufk[7][2]);
        Fstar[13][0] = ((mk[8]*(Atk[13][0]-gk[13][0]))-ufk[8][0]);
        Fstar[13][1] = ((mk[8]*(Atk[13][1]-gk[13][1]))-ufk[8][1]);
        Fstar[13][2] = ((mk[8]*(Atk[13][2]-gk[13][2]))-ufk[8][2]);
        Fstar[14][0] = ((mk[9]*(Atk[14][0]-gk[14][0]))-ufk[9][0]);
        Fstar[14][1] = ((mk[9]*(Atk[14][1]-gk[14][1]))-ufk[9][1]);
        Fstar[14][2] = ((mk[9]*(Atk[14][2]-gk[14][2]))-ufk[9][2]);
        Fstar[15][0] = ((mk[10]*(Atk[15][0]-gk[15][0]))-ufk[10][0]);
        Fstar[15][1] = ((mk[10]*(Atk[15][1]-gk[15][1]))-ufk[10][1]);
        Fstar[15][2] = ((mk[10]*(Atk[15][2]-gk[15][2]))-ufk[10][2]);
        Fstar[16][0] = ((mk[11]*(Atk[16][0]-gk[16][0]))-ufk[11][0]);
        Fstar[16][1] = ((mk[11]*(Atk[16][1]-gk[16][1]))-ufk[11][1]);
        Fstar[16][2] = ((mk[11]*(Atk[16][2]-gk[16][2]))-ufk[11][2]);
        Fstar[17][0] = ((mk[12]*(Atk[17][0]-gk[17][0]))-ufk[12][0]);
        Fstar[17][1] = ((mk[12]*(Atk[17][1]-gk[17][1]))-ufk[12][1]);
        Fstar[17][2] = ((mk[12]*(Atk[17][2]-gk[17][2]))-ufk[12][2]);
        Fstar[18][0] = ((mk[13]*(Atk[18][0]-gk[18][0]))-ufk[13][0]);
        Fstar[18][1] = ((mk[13]*(Atk[18][1]-gk[18][1]))-ufk[13][1]);
        Fstar[18][2] = ((mk[13]*(Atk[18][2]-gk[18][2]))-ufk[13][2]);
        Fstar[19][0] = ((mk[14]*(Atk[19][0]-gk[19][0]))-ufk[14][0]);
        Fstar[19][1] = ((mk[14]*(Atk[19][1]-gk[19][1]))-ufk[14][1]);
        Fstar[19][2] = ((mk[14]*(Atk[19][2]-gk[19][2]))-ufk[14][2]);
        Fstar[20][0] = ((mk[15]*(Atk[20][0]-gk[20][0]))-ufk[15][0]);
        Fstar[20][1] = ((mk[15]*(Atk[20][1]-gk[20][1]))-ufk[15][1]);
        Fstar[20][2] = ((mk[15]*(Atk[20][2]-gk[20][2]))-ufk[15][2]);
/*
Compute Tstar (torques)
*/
        Tstar[5][0] = (WkIkWk[5][0]-utk[0][0]);
        Tstar[5][1] = (WkIkWk[5][1]-utk[0][1]);
        Tstar[5][2] = (WkIkWk[5][2]-utk[0][2]);
        Tstar[6][0] = ((WkIkWk[6][0]+((ik[1][0][2]*Otk[6][2])+((ik[1][0][0]*
          Otk[6][0])+(ik[1][0][1]*Otk[6][1]))))-utk[1][0]);
        Tstar[6][1] = ((WkIkWk[6][1]+((ik[1][1][2]*Otk[6][2])+((ik[1][1][0]*
          Otk[6][0])+(ik[1][1][1]*Otk[6][1]))))-utk[1][1]);
        Tstar[6][2] = ((WkIkWk[6][2]+((ik[1][2][2]*Otk[6][2])+((ik[1][2][0]*
          Otk[6][0])+(ik[1][2][1]*Otk[6][1]))))-utk[1][2]);
        Tstar[7][0] = ((WkIkWk[7][0]+((ik[2][0][2]*Otk[7][2])+((ik[2][0][0]*
          Otk[7][0])+(ik[2][0][1]*Otk[7][1]))))-utk[2][0]);
        Tstar[7][1] = ((WkIkWk[7][1]+((ik[2][1][2]*Otk[7][2])+((ik[2][1][0]*
          Otk[7][0])+(ik[2][1][1]*Otk[7][1]))))-utk[2][1]);
        Tstar[7][2] = ((WkIkWk[7][2]+((ik[2][2][2]*Otk[7][2])+((ik[2][2][0]*
          Otk[7][0])+(ik[2][2][1]*Otk[7][1]))))-utk[2][2]);
        Tstar[8][0] = ((WkIkWk[8][0]+((ik[3][0][2]*Otk[8][2])+((ik[3][0][0]*
          Otk[8][0])+(ik[3][0][1]*Otk[8][1]))))-utk[3][0]);
        Tstar[8][1] = ((WkIkWk[8][1]+((ik[3][1][2]*Otk[8][2])+((ik[3][1][0]*
          Otk[8][0])+(ik[3][1][1]*Otk[8][1]))))-utk[3][1]);
        Tstar[8][2] = ((WkIkWk[8][2]+((ik[3][2][2]*Otk[8][2])+((ik[3][2][0]*
          Otk[8][0])+(ik[3][2][1]*Otk[8][1]))))-utk[3][2]);
        Tstar[9][0] = ((WkIkWk[9][0]+((ik[4][0][2]*Otk[9][2])+((ik[4][0][0]*
          Otk[9][0])+(ik[4][0][1]*Otk[9][1]))))-utk[4][0]);
        Tstar[9][1] = ((WkIkWk[9][1]+((ik[4][1][2]*Otk[9][2])+((ik[4][1][0]*
          Otk[9][0])+(ik[4][1][1]*Otk[9][1]))))-utk[4][1]);
        Tstar[9][2] = ((WkIkWk[9][2]+((ik[4][2][2]*Otk[9][2])+((ik[4][2][0]*
          Otk[9][0])+(ik[4][2][1]*Otk[9][1]))))-utk[4][2]);
        Tstar[10][0] = ((WkIkWk[10][0]+((ik[5][0][2]*Otk[10][2])+((ik[5][0][0]*
          Otk[10][0])+(ik[5][0][1]*Otk[10][1]))))-utk[5][0]);
        Tstar[10][1] = ((WkIkWk[10][1]+((ik[5][1][2]*Otk[10][2])+((ik[5][1][0]*
          Otk[10][0])+(ik[5][1][1]*Otk[10][1]))))-utk[5][1]);
        Tstar[10][2] = ((WkIkWk[10][2]+((ik[5][2][2]*Otk[10][2])+((ik[5][2][0]*
          Otk[10][0])+(ik[5][2][1]*Otk[10][1]))))-utk[5][2]);
        Tstar[11][0] = ((WkIkWk[11][0]+((ik[6][0][2]*Otk[11][2])+((ik[6][0][0]*
          Otk[11][0])+(ik[6][0][1]*Otk[11][1]))))-utk[6][0]);
        Tstar[11][1] = ((WkIkWk[11][1]+((ik[6][1][2]*Otk[11][2])+((ik[6][1][0]*
          Otk[11][0])+(ik[6][1][1]*Otk[11][1]))))-utk[6][1]);
        Tstar[11][2] = ((WkIkWk[11][2]+((ik[6][2][2]*Otk[11][2])+((ik[6][2][0]*
          Otk[11][0])+(ik[6][2][1]*Otk[11][1]))))-utk[6][2]);
        Tstar[12][0] = ((WkIkWk[12][0]+((ik[7][0][2]*Otk[12][2])+((ik[7][0][0]*
          Otk[12][0])+(ik[7][0][1]*Otk[12][1]))))-utk[7][0]);
        Tstar[12][1] = ((WkIkWk[12][1]+((ik[7][1][2]*Otk[12][2])+((ik[7][1][0]*
          Otk[12][0])+(ik[7][1][1]*Otk[12][1]))))-utk[7][1]);
        Tstar[12][2] = ((WkIkWk[12][2]+((ik[7][2][2]*Otk[12][2])+((ik[7][2][0]*
          Otk[12][0])+(ik[7][2][1]*Otk[12][1]))))-utk[7][2]);
        Tstar[13][0] = ((WkIkWk[13][0]+((ik[8][0][2]*Otk[13][2])+((ik[8][0][0]*
          Otk[13][0])+(ik[8][0][1]*Otk[13][1]))))-utk[8][0]);
        Tstar[13][1] = ((WkIkWk[13][1]+((ik[8][1][2]*Otk[13][2])+((ik[8][1][0]*
          Otk[13][0])+(ik[8][1][1]*Otk[13][1]))))-utk[8][1]);
        Tstar[13][2] = ((WkIkWk[13][2]+((ik[8][2][2]*Otk[13][2])+((ik[8][2][0]*
          Otk[13][0])+(ik[8][2][1]*Otk[13][1]))))-utk[8][2]);
        Tstar[14][0] = ((WkIkWk[14][0]+((ik[9][0][2]*Otk[14][2])+((ik[9][0][0]*
          Otk[14][0])+(ik[9][0][1]*Otk[14][1]))))-utk[9][0]);
        Tstar[14][1] = ((WkIkWk[14][1]+((ik[9][1][2]*Otk[14][2])+((ik[9][1][0]*
          Otk[14][0])+(ik[9][1][1]*Otk[14][1]))))-utk[9][1]);
        Tstar[14][2] = ((WkIkWk[14][2]+((ik[9][2][2]*Otk[14][2])+((ik[9][2][0]*
          Otk[14][0])+(ik[9][2][1]*Otk[14][1]))))-utk[9][2]);
        Tstar[15][0] = ((WkIkWk[15][0]+((ik[10][0][2]*Otk[15][2])+((ik[10][0][0]
          *Otk[15][0])+(ik[10][0][1]*Otk[15][1]))))-utk[10][0]);
        Tstar[15][1] = ((WkIkWk[15][1]+((ik[10][1][2]*Otk[15][2])+((ik[10][1][0]
          *Otk[15][0])+(ik[10][1][1]*Otk[15][1]))))-utk[10][1]);
        Tstar[15][2] = ((WkIkWk[15][2]+((ik[10][2][2]*Otk[15][2])+((ik[10][2][0]
          *Otk[15][0])+(ik[10][2][1]*Otk[15][1]))))-utk[10][2]);
        Tstar[16][0] = ((WkIkWk[16][0]+((ik[11][0][2]*Otk[16][2])+((ik[11][0][0]
          *Otk[16][0])+(ik[11][0][1]*Otk[16][1]))))-utk[11][0]);
        Tstar[16][1] = ((WkIkWk[16][1]+((ik[11][1][2]*Otk[16][2])+((ik[11][1][0]
          *Otk[16][0])+(ik[11][1][1]*Otk[16][1]))))-utk[11][1]);
        Tstar[16][2] = ((WkIkWk[16][2]+((ik[11][2][2]*Otk[16][2])+((ik[11][2][0]
          *Otk[16][0])+(ik[11][2][1]*Otk[16][1]))))-utk[11][2]);
        Tstar[17][0] = ((WkIkWk[17][0]+((ik[12][0][2]*Otk[17][2])+((ik[12][0][0]
          *Otk[17][0])+(ik[12][0][1]*Otk[17][1]))))-utk[12][0]);
        Tstar[17][1] = ((WkIkWk[17][1]+((ik[12][1][2]*Otk[17][2])+((ik[12][1][0]
          *Otk[17][0])+(ik[12][1][1]*Otk[17][1]))))-utk[12][1]);
        Tstar[17][2] = ((WkIkWk[17][2]+((ik[12][2][2]*Otk[17][2])+((ik[12][2][0]
          *Otk[17][0])+(ik[12][2][1]*Otk[17][1]))))-utk[12][2]);
        Tstar[18][0] = ((WkIkWk[18][0]+((ik[13][0][2]*Otk[18][2])+((ik[13][0][0]
          *Otk[18][0])+(ik[13][0][1]*Otk[18][1]))))-utk[13][0]);
        Tstar[18][1] = ((WkIkWk[18][1]+((ik[13][1][2]*Otk[18][2])+((ik[13][1][0]
          *Otk[18][0])+(ik[13][1][1]*Otk[18][1]))))-utk[13][1]);
        Tstar[18][2] = ((WkIkWk[18][2]+((ik[13][2][2]*Otk[18][2])+((ik[13][2][0]
          *Otk[18][0])+(ik[13][2][1]*Otk[18][1]))))-utk[13][2]);
        Tstar[19][0] = ((WkIkWk[19][0]+((ik[14][0][2]*Otk[19][2])+((ik[14][0][0]
          *Otk[19][0])+(ik[14][0][1]*Otk[19][1]))))-utk[14][0]);
        Tstar[19][1] = ((WkIkWk[19][1]+((ik[14][1][2]*Otk[19][2])+((ik[14][1][0]
          *Otk[19][0])+(ik[14][1][1]*Otk[19][1]))))-utk[14][1]);
        Tstar[19][2] = ((WkIkWk[19][2]+((ik[14][2][2]*Otk[19][2])+((ik[14][2][0]
          *Otk[19][0])+(ik[14][2][1]*Otk[19][1]))))-utk[14][2]);
        Tstar[20][0] = ((WkIkWk[20][0]+((ik[15][0][2]*Otk[20][2])+((ik[15][0][0]
          *Otk[20][0])+(ik[15][0][1]*Otk[20][1]))))-utk[15][0]);
        Tstar[20][1] = ((WkIkWk[20][1]+((ik[15][1][2]*Otk[20][2])+((ik[15][1][0]
          *Otk[20][0])+(ik[15][1][1]*Otk[20][1]))))-utk[15][1]);
        Tstar[20][2] = ((WkIkWk[20][2]+((ik[15][2][2]*Otk[20][2])+((ik[15][2][0]
          *Otk[20][0])+(ik[15][2][1]*Otk[20][1]))))-utk[15][2]);
/*
Compute fs0 (RHS ignoring constraints)
*/
        sddovpk();
        temp[0] = (((Fstar[8][2]*Vpk[0][8][2])+((Fstar[8][0]*Vpk[0][8][0])+(
          Fstar[8][1]*Vpk[0][8][1])))+(((Fstar[7][2]*Vpk[0][7][2])+((Fstar[7][0]
          *Vpk[0][7][0])+(Fstar[7][1]*Vpk[0][7][1])))+(((Fstar[5][2]*
          Vpk[0][3][2])+((Fstar[5][0]*Vpk[0][3][0])+(Fstar[5][1]*Vpk[0][3][1])))
          +((Fstar[6][2]*Vpk[0][6][2])+((Fstar[6][0]*Vpk[0][6][0])+(Fstar[6][1]*
          Vpk[0][6][1]))))));
        temp[1] = (((Fstar[12][2]*Vpk[0][12][2])+((Fstar[12][0]*Vpk[0][12][0])+(
          Fstar[12][1]*Vpk[0][12][1])))+(((Fstar[11][2]*Vpk[0][11][2])+((
          Fstar[11][0]*Vpk[0][11][0])+(Fstar[11][1]*Vpk[0][11][1])))+(((
          Fstar[10][2]*Vpk[0][10][2])+((Fstar[10][0]*Vpk[0][10][0])+(
          Fstar[10][1]*Vpk[0][10][1])))+(((Fstar[9][2]*Vpk[0][9][2])+((
          Fstar[9][0]*Vpk[0][9][0])+(Fstar[9][1]*Vpk[0][9][1])))+temp[0]))));
        temp[2] = (((Fstar[16][2]*Vpk[0][16][2])+((Fstar[16][0]*Vpk[0][16][0])+(
          Fstar[16][1]*Vpk[0][16][1])))+(((Fstar[15][2]*Vpk[0][15][2])+((
          Fstar[15][0]*Vpk[0][15][0])+(Fstar[15][1]*Vpk[0][15][1])))+(((
          Fstar[14][2]*Vpk[0][14][2])+((Fstar[14][0]*Vpk[0][14][0])+(
          Fstar[14][1]*Vpk[0][14][1])))+(((Fstar[13][2]*Vpk[0][13][2])+((
          Fstar[13][0]*Vpk[0][13][0])+(Fstar[13][1]*Vpk[0][13][1])))+temp[1]))))
          ;
        fs0[0] = (utau[0]-(((Fstar[20][2]*Vpk[0][20][2])+((Fstar[20][0]*
          Vpk[0][20][0])+(Fstar[20][1]*Vpk[0][20][1])))+(((Fstar[19][2]*
          Vpk[0][19][2])+((Fstar[19][0]*Vpk[0][19][0])+(Fstar[19][1]*
          Vpk[0][19][1])))+(((Fstar[18][2]*Vpk[0][18][2])+((Fstar[18][0]*
          Vpk[0][18][0])+(Fstar[18][1]*Vpk[0][18][1])))+(((Fstar[17][2]*
          Vpk[0][17][2])+((Fstar[17][0]*Vpk[0][17][0])+(Fstar[17][1]*
          Vpk[0][17][1])))+temp[2])))));
        temp[0] = (((Fstar[8][2]*Vpk[1][8][2])+((Fstar[8][0]*Vpk[1][8][0])+(
          Fstar[8][1]*Vpk[1][8][1])))+(((Fstar[7][2]*Vpk[1][7][2])+((Fstar[7][0]
          *Vpk[1][7][0])+(Fstar[7][1]*Vpk[1][7][1])))+(((Fstar[5][2]*
          Vpk[1][3][2])+((Fstar[5][0]*Vpk[1][3][0])+(Fstar[5][1]*Vpk[1][3][1])))
          +((Fstar[6][2]*Vpk[1][6][2])+((Fstar[6][0]*Vpk[1][6][0])+(Fstar[6][1]*
          Vpk[1][6][1]))))));
        temp[1] = (((Fstar[12][2]*Vpk[1][12][2])+((Fstar[12][0]*Vpk[1][12][0])+(
          Fstar[12][1]*Vpk[1][12][1])))+(((Fstar[11][2]*Vpk[1][11][2])+((
          Fstar[11][0]*Vpk[1][11][0])+(Fstar[11][1]*Vpk[1][11][1])))+(((
          Fstar[10][2]*Vpk[1][10][2])+((Fstar[10][0]*Vpk[1][10][0])+(
          Fstar[10][1]*Vpk[1][10][1])))+(((Fstar[9][2]*Vpk[1][9][2])+((
          Fstar[9][0]*Vpk[1][9][0])+(Fstar[9][1]*Vpk[1][9][1])))+temp[0]))));
        temp[2] = (((Fstar[16][2]*Vpk[1][16][2])+((Fstar[16][0]*Vpk[1][16][0])+(
          Fstar[16][1]*Vpk[1][16][1])))+(((Fstar[15][2]*Vpk[1][15][2])+((
          Fstar[15][0]*Vpk[1][15][0])+(Fstar[15][1]*Vpk[1][15][1])))+(((
          Fstar[14][2]*Vpk[1][14][2])+((Fstar[14][0]*Vpk[1][14][0])+(
          Fstar[14][1]*Vpk[1][14][1])))+(((Fstar[13][2]*Vpk[1][13][2])+((
          Fstar[13][0]*Vpk[1][13][0])+(Fstar[13][1]*Vpk[1][13][1])))+temp[1]))))
          ;
        fs0[1] = (utau[1]-(((Fstar[20][2]*Vpk[1][20][2])+((Fstar[20][0]*
          Vpk[1][20][0])+(Fstar[20][1]*Vpk[1][20][1])))+(((Fstar[19][2]*
          Vpk[1][19][2])+((Fstar[19][0]*Vpk[1][19][0])+(Fstar[19][1]*
          Vpk[1][19][1])))+(((Fstar[18][2]*Vpk[1][18][2])+((Fstar[18][0]*
          Vpk[1][18][0])+(Fstar[18][1]*Vpk[1][18][1])))+(((Fstar[17][2]*
          Vpk[1][17][2])+((Fstar[17][0]*Vpk[1][17][0])+(Fstar[17][1]*
          Vpk[1][17][1])))+temp[2])))));
        temp[0] = (((Fstar[8][2]*Vpk[2][8][2])+((Fstar[8][0]*Vpk[2][8][0])+(
          Fstar[8][1]*Vpk[2][8][1])))+(((Fstar[7][2]*Vpk[2][7][2])+((Fstar[7][0]
          *Vpk[2][7][0])+(Fstar[7][1]*Vpk[2][7][1])))+(((Fstar[5][2]*
          Vpk[2][3][2])+((Fstar[5][0]*Vpk[2][3][0])+(Fstar[5][1]*Vpk[2][3][1])))
          +((Fstar[6][2]*Vpk[2][6][2])+((Fstar[6][0]*Vpk[2][6][0])+(Fstar[6][1]*
          Vpk[2][6][1]))))));
        temp[1] = (((Fstar[12][2]*Vpk[2][12][2])+((Fstar[12][0]*Vpk[2][12][0])+(
          Fstar[12][1]*Vpk[2][12][1])))+(((Fstar[11][2]*Vpk[2][11][2])+((
          Fstar[11][0]*Vpk[2][11][0])+(Fstar[11][1]*Vpk[2][11][1])))+(((
          Fstar[10][2]*Vpk[2][10][2])+((Fstar[10][0]*Vpk[2][10][0])+(
          Fstar[10][1]*Vpk[2][10][1])))+(((Fstar[9][2]*Vpk[2][9][2])+((
          Fstar[9][0]*Vpk[2][9][0])+(Fstar[9][1]*Vpk[2][9][1])))+temp[0]))));
        temp[2] = (((Fstar[16][2]*Vpk[2][16][2])+((Fstar[16][0]*Vpk[2][16][0])+(
          Fstar[16][1]*Vpk[2][16][1])))+(((Fstar[15][2]*Vpk[2][15][2])+((
          Fstar[15][0]*Vpk[2][15][0])+(Fstar[15][1]*Vpk[2][15][1])))+(((
          Fstar[14][2]*Vpk[2][14][2])+((Fstar[14][0]*Vpk[2][14][0])+(
          Fstar[14][1]*Vpk[2][14][1])))+(((Fstar[13][2]*Vpk[2][13][2])+((
          Fstar[13][0]*Vpk[2][13][0])+(Fstar[13][1]*Vpk[2][13][1])))+temp[1]))))
          ;
        fs0[2] = (utau[2]-(((Fstar[20][2]*Vpk[2][20][2])+((Fstar[20][0]*
          Vpk[2][20][0])+(Fstar[20][1]*Vpk[2][20][1])))+(((Fstar[19][2]*
          Vpk[2][19][2])+((Fstar[19][0]*Vpk[2][19][0])+(Fstar[19][1]*
          Vpk[2][19][1])))+(((Fstar[18][2]*Vpk[2][18][2])+((Fstar[18][0]*
          Vpk[2][18][0])+(Fstar[18][1]*Vpk[2][18][1])))+(((Fstar[17][2]*
          Vpk[2][17][2])+((Fstar[17][0]*Vpk[2][17][0])+(Fstar[17][1]*
          Vpk[2][17][1])))+temp[2])))));
        temp[0] = (((Tstar[5][0]+((Fstar[5][1]*rk[0][2])-(Fstar[5][2]*rk[0][1]))
          )+(((Cik[6][0][2]*Tstar[6][2])+((Cik[6][0][0]*Tstar[6][0])+(
          Cik[6][0][1]*Tstar[6][1])))+((Fstar[6][2]*Vpk[3][6][2])+((Fstar[6][0]*
          Vpk[3][6][0])+(Fstar[6][1]*Vpk[3][6][1])))))+(((Fstar[7][2]*
          Vpk[3][7][2])+((Fstar[7][0]*Vpk[3][7][0])+(Fstar[7][1]*Vpk[3][7][1])))
          +((Tstar[7][2]*Wpk[3][7][2])+((Tstar[7][0]*Wpk[3][7][0])+(Tstar[7][1]*
          Wpk[3][7][1])))));
        temp[1] = ((((Cik[9][0][2]*Tstar[9][2])+((Cik[9][0][0]*Tstar[9][0])+(
          Cik[9][0][1]*Tstar[9][1])))+((Fstar[9][2]*Vpk[3][9][2])+((Fstar[9][0]*
          Vpk[3][9][0])+(Fstar[9][1]*Vpk[3][9][1]))))+((((Fstar[8][2]*
          Vpk[3][8][2])+((Fstar[8][0]*Vpk[3][8][0])+(Fstar[8][1]*Vpk[3][8][1])))
          +((Tstar[8][2]*Wpk[3][8][2])+((Tstar[8][0]*Wpk[3][8][0])+(Tstar[8][1]*
          Wpk[3][8][1]))))+temp[0]));
        temp[2] = ((((Fstar[11][2]*Vpk[3][11][2])+((Fstar[11][0]*Vpk[3][11][0])+
          (Fstar[11][1]*Vpk[3][11][1])))+((Tstar[11][2]*Wpk[3][11][2])+((
          Tstar[11][0]*Wpk[3][11][0])+(Tstar[11][1]*Wpk[3][11][1]))))+((((
          Fstar[10][2]*Vpk[3][10][2])+((Fstar[10][0]*Vpk[3][10][0])+(
          Fstar[10][1]*Vpk[3][10][1])))+((Tstar[10][2]*Wpk[3][10][2])+((
          Tstar[10][0]*Wpk[3][10][0])+(Tstar[10][1]*Wpk[3][10][1]))))+temp[1]));
        temp[3] = ((((Fstar[13][2]*Vpk[3][13][2])+((Fstar[13][0]*Vpk[3][13][0])+
          (Fstar[13][1]*Vpk[3][13][1])))+((Tstar[13][2]*Wpk[3][13][2])+((
          Tstar[13][0]*Wpk[3][13][0])+(Tstar[13][1]*Wpk[3][13][1]))))+((((
          Fstar[12][2]*Vpk[3][12][2])+((Fstar[12][0]*Vpk[3][12][0])+(
          Fstar[12][1]*Vpk[3][12][1])))+((Tstar[12][2]*Wpk[3][12][2])+((
          Tstar[12][0]*Wpk[3][12][0])+(Tstar[12][1]*Wpk[3][12][1]))))+temp[2]));
        temp[4] = ((((Cik[15][0][2]*Tstar[15][2])+((Cik[15][0][0]*Tstar[15][0])+
          (Cik[15][0][1]*Tstar[15][1])))+((Fstar[15][2]*Vpk[3][15][2])+((
          Fstar[15][0]*Vpk[3][15][0])+(Fstar[15][1]*Vpk[3][15][1]))))+((((
          Fstar[14][2]*Vpk[3][14][2])+((Fstar[14][0]*Vpk[3][14][0])+(
          Fstar[14][1]*Vpk[3][14][1])))+((Tstar[14][2]*Wpk[3][14][2])+((
          Tstar[14][0]*Wpk[3][14][0])+(Tstar[14][1]*Wpk[3][14][1]))))+temp[3]));
        temp[5] = ((((Fstar[17][2]*Vpk[3][17][2])+((Fstar[17][0]*Vpk[3][17][0])+
          (Fstar[17][1]*Vpk[3][17][1])))+((Tstar[17][2]*Wpk[3][17][2])+((
          Tstar[17][0]*Wpk[3][17][0])+(Tstar[17][1]*Wpk[3][17][1]))))+((((
          Fstar[16][2]*Vpk[3][16][2])+((Fstar[16][0]*Vpk[3][16][0])+(
          Fstar[16][1]*Vpk[3][16][1])))+((Tstar[16][2]*Wpk[3][16][2])+((
          Tstar[16][0]*Wpk[3][16][0])+(Tstar[16][1]*Wpk[3][16][1]))))+temp[4]));
        temp[6] = ((((Fstar[19][2]*Vpk[3][19][2])+((Fstar[19][0]*Vpk[3][19][0])+
          (Fstar[19][1]*Vpk[3][19][1])))+((Tstar[19][2]*Wpk[3][19][2])+((
          Tstar[19][0]*Wpk[3][19][0])+(Tstar[19][1]*Wpk[3][19][1]))))+((((
          Fstar[18][2]*Vpk[3][18][2])+((Fstar[18][0]*Vpk[3][18][0])+(
          Fstar[18][1]*Vpk[3][18][1])))+((Tstar[18][2]*Wpk[3][18][2])+((
          Tstar[18][0]*Wpk[3][18][0])+(Tstar[18][1]*Wpk[3][18][1]))))+temp[5]));
        fs0[3] = (utau[3]-((((Fstar[20][2]*Vpk[3][20][2])+((Fstar[20][0]*
          Vpk[3][20][0])+(Fstar[20][1]*Vpk[3][20][1])))+((Tstar[20][2]*
          Wpk[3][20][2])+((Tstar[20][0]*Wpk[3][20][0])+(Tstar[20][1]*
          Wpk[3][20][1]))))+temp[6]));
        temp[0] = (((Tstar[5][1]+((Fstar[5][2]*rk[0][0])-(Fstar[5][0]*rk[0][2]))
          )+(((Cik[6][1][2]*Tstar[6][2])+((Cik[6][1][0]*Tstar[6][0])+(
          Cik[6][1][1]*Tstar[6][1])))+((Fstar[6][2]*Vpk[4][6][2])+((Fstar[6][0]*
          Vpk[4][6][0])+(Fstar[6][1]*Vpk[4][6][1])))))+(((Fstar[7][2]*
          Vpk[4][7][2])+((Fstar[7][0]*Vpk[4][7][0])+(Fstar[7][1]*Vpk[4][7][1])))
          +((Tstar[7][2]*Wpk[4][7][2])+((Tstar[7][0]*Wpk[4][7][0])+(Tstar[7][1]*
          Wpk[4][7][1])))));
        temp[1] = ((((Cik[9][1][2]*Tstar[9][2])+((Cik[9][1][0]*Tstar[9][0])+(
          Cik[9][1][1]*Tstar[9][1])))+((Fstar[9][2]*Vpk[4][9][2])+((Fstar[9][0]*
          Vpk[4][9][0])+(Fstar[9][1]*Vpk[4][9][1]))))+((((Fstar[8][2]*
          Vpk[4][8][2])+((Fstar[8][0]*Vpk[4][8][0])+(Fstar[8][1]*Vpk[4][8][1])))
          +((Tstar[8][2]*Wpk[4][8][2])+((Tstar[8][0]*Wpk[4][8][0])+(Tstar[8][1]*
          Wpk[4][8][1]))))+temp[0]));
        temp[2] = ((((Fstar[11][2]*Vpk[4][11][2])+((Fstar[11][0]*Vpk[4][11][0])+
          (Fstar[11][1]*Vpk[4][11][1])))+((Tstar[11][2]*Wpk[4][11][2])+((
          Tstar[11][0]*Wpk[4][11][0])+(Tstar[11][1]*Wpk[4][11][1]))))+((((
          Fstar[10][2]*Vpk[4][10][2])+((Fstar[10][0]*Vpk[4][10][0])+(
          Fstar[10][1]*Vpk[4][10][1])))+((Tstar[10][2]*Wpk[4][10][2])+((
          Tstar[10][0]*Wpk[4][10][0])+(Tstar[10][1]*Wpk[4][10][1]))))+temp[1]));
        temp[3] = ((((Fstar[13][2]*Vpk[4][13][2])+((Fstar[13][0]*Vpk[4][13][0])+
          (Fstar[13][1]*Vpk[4][13][1])))+((Tstar[13][2]*Wpk[4][13][2])+((
          Tstar[13][0]*Wpk[4][13][0])+(Tstar[13][1]*Wpk[4][13][1]))))+((((
          Fstar[12][2]*Vpk[4][12][2])+((Fstar[12][0]*Vpk[4][12][0])+(
          Fstar[12][1]*Vpk[4][12][1])))+((Tstar[12][2]*Wpk[4][12][2])+((
          Tstar[12][0]*Wpk[4][12][0])+(Tstar[12][1]*Wpk[4][12][1]))))+temp[2]));
        temp[4] = ((((Cik[15][1][2]*Tstar[15][2])+((Cik[15][1][0]*Tstar[15][0])+
          (Cik[15][1][1]*Tstar[15][1])))+((Fstar[15][2]*Vpk[4][15][2])+((
          Fstar[15][0]*Vpk[4][15][0])+(Fstar[15][1]*Vpk[4][15][1]))))+((((
          Fstar[14][2]*Vpk[4][14][2])+((Fstar[14][0]*Vpk[4][14][0])+(
          Fstar[14][1]*Vpk[4][14][1])))+((Tstar[14][2]*Wpk[4][14][2])+((
          Tstar[14][0]*Wpk[4][14][0])+(Tstar[14][1]*Wpk[4][14][1]))))+temp[3]));
        temp[5] = ((((Fstar[17][2]*Vpk[4][17][2])+((Fstar[17][0]*Vpk[4][17][0])+
          (Fstar[17][1]*Vpk[4][17][1])))+((Tstar[17][2]*Wpk[4][17][2])+((
          Tstar[17][0]*Wpk[4][17][0])+(Tstar[17][1]*Wpk[4][17][1]))))+((((
          Fstar[16][2]*Vpk[4][16][2])+((Fstar[16][0]*Vpk[4][16][0])+(
          Fstar[16][1]*Vpk[4][16][1])))+((Tstar[16][2]*Wpk[4][16][2])+((
          Tstar[16][0]*Wpk[4][16][0])+(Tstar[16][1]*Wpk[4][16][1]))))+temp[4]));
        temp[6] = ((((Fstar[19][2]*Vpk[4][19][2])+((Fstar[19][0]*Vpk[4][19][0])+
          (Fstar[19][1]*Vpk[4][19][1])))+((Tstar[19][2]*Wpk[4][19][2])+((
          Tstar[19][0]*Wpk[4][19][0])+(Tstar[19][1]*Wpk[4][19][1]))))+((((
          Fstar[18][2]*Vpk[4][18][2])+((Fstar[18][0]*Vpk[4][18][0])+(
          Fstar[18][1]*Vpk[4][18][1])))+((Tstar[18][2]*Wpk[4][18][2])+((
          Tstar[18][0]*Wpk[4][18][0])+(Tstar[18][1]*Wpk[4][18][1]))))+temp[5]));
        fs0[4] = (utau[4]-((((Fstar[20][2]*Vpk[4][20][2])+((Fstar[20][0]*
          Vpk[4][20][0])+(Fstar[20][1]*Vpk[4][20][1])))+((Tstar[20][2]*
          Wpk[4][20][2])+((Tstar[20][0]*Wpk[4][20][0])+(Tstar[20][1]*
          Wpk[4][20][1]))))+temp[6]));
        temp[0] = (((Tstar[5][2]+((Fstar[5][0]*rk[0][1])-(Fstar[5][1]*rk[0][0]))
          )+(((Cik[6][2][2]*Tstar[6][2])+((Cik[6][2][0]*Tstar[6][0])+(
          Cik[6][2][1]*Tstar[6][1])))+((Fstar[6][2]*Vpk[5][6][2])+((Fstar[6][0]*
          Vpk[5][6][0])+(Fstar[6][1]*Vpk[5][6][1])))))+(((Fstar[7][2]*
          Vpk[5][7][2])+((Fstar[7][0]*Vpk[5][7][0])+(Fstar[7][1]*Vpk[5][7][1])))
          +((Tstar[7][2]*Wpk[5][7][2])+((Tstar[7][0]*Wpk[5][7][0])+(Tstar[7][1]*
          Wpk[5][7][1])))));
        temp[1] = ((((Cik[9][2][2]*Tstar[9][2])+((Cik[9][2][0]*Tstar[9][0])+(
          Cik[9][2][1]*Tstar[9][1])))+((Fstar[9][2]*Vpk[5][9][2])+((Fstar[9][0]*
          Vpk[5][9][0])+(Fstar[9][1]*Vpk[5][9][1]))))+((((Fstar[8][2]*
          Vpk[5][8][2])+((Fstar[8][0]*Vpk[5][8][0])+(Fstar[8][1]*Vpk[5][8][1])))
          +((Tstar[8][2]*Wpk[5][8][2])+((Tstar[8][0]*Wpk[5][8][0])+(Tstar[8][1]*
          Wpk[5][8][1]))))+temp[0]));
        temp[2] = ((((Fstar[11][2]*Vpk[5][11][2])+((Fstar[11][0]*Vpk[5][11][0])+
          (Fstar[11][1]*Vpk[5][11][1])))+((Tstar[11][2]*Wpk[5][11][2])+((
          Tstar[11][0]*Wpk[5][11][0])+(Tstar[11][1]*Wpk[5][11][1]))))+((((
          Fstar[10][2]*Vpk[5][10][2])+((Fstar[10][0]*Vpk[5][10][0])+(
          Fstar[10][1]*Vpk[5][10][1])))+((Tstar[10][2]*Wpk[5][10][2])+((
          Tstar[10][0]*Wpk[5][10][0])+(Tstar[10][1]*Wpk[5][10][1]))))+temp[1]));
        temp[3] = ((((Fstar[13][2]*Vpk[5][13][2])+((Fstar[13][0]*Vpk[5][13][0])+
          (Fstar[13][1]*Vpk[5][13][1])))+((Tstar[13][2]*Wpk[5][13][2])+((
          Tstar[13][0]*Wpk[5][13][0])+(Tstar[13][1]*Wpk[5][13][1]))))+((((
          Fstar[12][2]*Vpk[5][12][2])+((Fstar[12][0]*Vpk[5][12][0])+(
          Fstar[12][1]*Vpk[5][12][1])))+((Tstar[12][2]*Wpk[5][12][2])+((
          Tstar[12][0]*Wpk[5][12][0])+(Tstar[12][1]*Wpk[5][12][1]))))+temp[2]));
        temp[4] = ((((Cik[15][2][2]*Tstar[15][2])+((Cik[15][2][0]*Tstar[15][0])+
          (Cik[15][2][1]*Tstar[15][1])))+((Fstar[15][2]*Vpk[5][15][2])+((
          Fstar[15][0]*Vpk[5][15][0])+(Fstar[15][1]*Vpk[5][15][1]))))+((((
          Fstar[14][2]*Vpk[5][14][2])+((Fstar[14][0]*Vpk[5][14][0])+(
          Fstar[14][1]*Vpk[5][14][1])))+((Tstar[14][2]*Wpk[5][14][2])+((
          Tstar[14][0]*Wpk[5][14][0])+(Tstar[14][1]*Wpk[5][14][1]))))+temp[3]));
        temp[5] = ((((Fstar[17][2]*Vpk[5][17][2])+((Fstar[17][0]*Vpk[5][17][0])+
          (Fstar[17][1]*Vpk[5][17][1])))+((Tstar[17][2]*Wpk[5][17][2])+((
          Tstar[17][0]*Wpk[5][17][0])+(Tstar[17][1]*Wpk[5][17][1]))))+((((
          Fstar[16][2]*Vpk[5][16][2])+((Fstar[16][0]*Vpk[5][16][0])+(
          Fstar[16][1]*Vpk[5][16][1])))+((Tstar[16][2]*Wpk[5][16][2])+((
          Tstar[16][0]*Wpk[5][16][0])+(Tstar[16][1]*Wpk[5][16][1]))))+temp[4]));
        temp[6] = ((((Fstar[19][2]*Vpk[5][19][2])+((Fstar[19][0]*Vpk[5][19][0])+
          (Fstar[19][1]*Vpk[5][19][1])))+((Tstar[19][2]*Wpk[5][19][2])+((
          Tstar[19][0]*Wpk[5][19][0])+(Tstar[19][1]*Wpk[5][19][1]))))+((((
          Fstar[18][2]*Vpk[5][18][2])+((Fstar[18][0]*Vpk[5][18][0])+(
          Fstar[18][1]*Vpk[5][18][1])))+((Tstar[18][2]*Wpk[5][18][2])+((
          Tstar[18][0]*Wpk[5][18][0])+(Tstar[18][1]*Wpk[5][18][1]))))+temp[5]));
        fs0[5] = (utau[5]-((((Fstar[20][2]*Vpk[5][20][2])+((Fstar[20][0]*
          Vpk[5][20][0])+(Fstar[20][1]*Vpk[5][20][1])))+((Tstar[20][2]*
          Wpk[5][20][2])+((Tstar[20][0]*Wpk[5][20][0])+(Tstar[20][1]*
          Wpk[5][20][1]))))+temp[6]));
        temp[0] = ((((Fstar[6][2]*Vpk[6][6][2])+((Fstar[6][0]*Vpk[6][6][0])+(
          Fstar[6][1]*Vpk[6][6][1])))+((pin[6][2]*Tstar[6][2])+((pin[6][0]*
          Tstar[6][0])+(pin[6][1]*Tstar[6][1]))))+(((Fstar[7][2]*Vpk[6][7][2])+(
          (Fstar[7][0]*Vpk[6][7][0])+(Fstar[7][1]*Vpk[6][7][1])))+((Tstar[7][2]*
          Wpk[6][7][2])+((Tstar[7][0]*Wpk[6][7][0])+(Tstar[7][1]*Wpk[6][7][1])))
          ));
        fs0[6] = (utau[6]-((((Fstar[8][2]*Vpk[6][8][2])+((Fstar[8][0]*
          Vpk[6][8][0])+(Fstar[8][1]*Vpk[6][8][1])))+((Tstar[8][2]*Wpk[6][8][2])
          +((Tstar[8][0]*Wpk[6][8][0])+(Tstar[8][1]*Wpk[6][8][1]))))+temp[0]));
        fs0[7] = (utau[7]-((((Fstar[7][2]*Vpk[7][7][2])+((Fstar[7][0]*
          Vpk[7][7][0])+(Fstar[7][1]*Vpk[7][7][1])))+((pin[7][2]*Tstar[7][2])+((
          pin[7][0]*Tstar[7][0])+(pin[7][1]*Tstar[7][1]))))+(((Fstar[8][2]*
          Vpk[7][8][2])+((Fstar[8][0]*Vpk[7][8][0])+(Fstar[8][1]*Vpk[7][8][1])))
          +((Tstar[8][2]*Wpk[7][8][2])+((Tstar[8][0]*Wpk[7][8][0])+(Tstar[8][1]*
          Wpk[7][8][1]))))));
        fs0[8] = (utau[8]-(((Fstar[8][2]*Vpk[8][8][2])+((Fstar[8][0]*
          Vpk[8][8][0])+(Fstar[8][1]*Vpk[8][8][1])))+((pin[8][2]*Tstar[8][2])+((
          pin[8][0]*Tstar[8][0])+(pin[8][1]*Tstar[8][1])))));
        temp[0] = ((((Fstar[9][2]*Vpk[9][9][2])+((Fstar[9][0]*Vpk[9][9][0])+(
          Fstar[9][1]*Vpk[9][9][1])))+((pin[9][2]*Tstar[9][2])+((pin[9][0]*
          Tstar[9][0])+(pin[9][1]*Tstar[9][1]))))+(((Fstar[10][2]*Vpk[9][10][2])
          +((Fstar[10][0]*Vpk[9][10][0])+(Fstar[10][1]*Vpk[9][10][1])))+((
          Tstar[10][2]*Wpk[9][10][2])+((Tstar[10][0]*Wpk[9][10][0])+(
          Tstar[10][1]*Wpk[9][10][1])))));
        temp[1] = ((((Fstar[12][2]*Vpk[9][12][2])+((Fstar[12][0]*Vpk[9][12][0])+
          (Fstar[12][1]*Vpk[9][12][1])))+((Tstar[12][2]*Wpk[9][12][2])+((
          Tstar[12][0]*Wpk[9][12][0])+(Tstar[12][1]*Wpk[9][12][1]))))+((((
          Fstar[11][2]*Vpk[9][11][2])+((Fstar[11][0]*Vpk[9][11][0])+(
          Fstar[11][1]*Vpk[9][11][1])))+((Tstar[11][2]*Wpk[9][11][2])+((
          Tstar[11][0]*Wpk[9][11][0])+(Tstar[11][1]*Wpk[9][11][1]))))+temp[0]));
        fs0[9] = (utau[9]-((((Fstar[14][2]*Vpk[9][14][2])+((Fstar[14][0]*
          Vpk[9][14][0])+(Fstar[14][1]*Vpk[9][14][1])))+((Tstar[14][2]*
          Wpk[9][14][2])+((Tstar[14][0]*Wpk[9][14][0])+(Tstar[14][1]*
          Wpk[9][14][1]))))+((((Fstar[13][2]*Vpk[9][13][2])+((Fstar[13][0]*
          Vpk[9][13][0])+(Fstar[13][1]*Vpk[9][13][1])))+((Tstar[13][2]*
          Wpk[9][13][2])+((Tstar[13][0]*Wpk[9][13][0])+(Tstar[13][1]*
          Wpk[9][13][1]))))+temp[1])));
        temp[0] = ((((Fstar[10][2]*Vpk[10][10][2])+((Fstar[10][0]*Vpk[10][10][0]
          )+(Fstar[10][1]*Vpk[10][10][1])))+((pin[10][2]*Tstar[10][2])+((
          pin[10][0]*Tstar[10][0])+(pin[10][1]*Tstar[10][1]))))+(((Fstar[11][2]*
          Vpk[10][11][2])+((Fstar[11][0]*Vpk[10][11][0])+(Fstar[11][1]*
          Vpk[10][11][1])))+((Tstar[11][2]*Wpk[10][11][2])+((Tstar[11][0]*
          Wpk[10][11][0])+(Tstar[11][1]*Wpk[10][11][1])))));
        temp[1] = ((((Fstar[13][2]*Vpk[10][13][2])+((Fstar[13][0]*Vpk[10][13][0]
          )+(Fstar[13][1]*Vpk[10][13][1])))+((Tstar[13][2]*Wpk[10][13][2])+((
          Tstar[13][0]*Wpk[10][13][0])+(Tstar[13][1]*Wpk[10][13][1]))))+((((
          Fstar[12][2]*Vpk[10][12][2])+((Fstar[12][0]*Vpk[10][12][0])+(
          Fstar[12][1]*Vpk[10][12][1])))+((Tstar[12][2]*Wpk[10][12][2])+((
          Tstar[12][0]*Wpk[10][12][0])+(Tstar[12][1]*Wpk[10][12][1]))))+temp[0])
          );
        fs0[10] = (utau[10]-((((Fstar[14][2]*Vpk[10][14][2])+((Fstar[14][0]*
          Vpk[10][14][0])+(Fstar[14][1]*Vpk[10][14][1])))+((Tstar[14][2]*
          Wpk[10][14][2])+((Tstar[14][0]*Wpk[10][14][0])+(Tstar[14][1]*
          Wpk[10][14][1]))))+temp[1]));
        temp[0] = ((((Fstar[11][2]*Vpk[11][11][2])+((Fstar[11][0]*Vpk[11][11][0]
          )+(Fstar[11][1]*Vpk[11][11][1])))+((pin[11][2]*Tstar[11][2])+((
          pin[11][0]*Tstar[11][0])+(pin[11][1]*Tstar[11][1]))))+(((Fstar[12][2]*
          Vpk[11][12][2])+((Fstar[12][0]*Vpk[11][12][0])+(Fstar[12][1]*
          Vpk[11][12][1])))+((Tstar[12][2]*Wpk[11][12][2])+((Tstar[12][0]*
          Wpk[11][12][0])+(Tstar[12][1]*Wpk[11][12][1])))));
        fs0[11] = (utau[11]-((((Fstar[14][2]*Vpk[11][14][2])+((Fstar[14][0]*
          Vpk[11][14][0])+(Fstar[14][1]*Vpk[11][14][1])))+((Tstar[14][2]*
          Wpk[11][14][2])+((Tstar[14][0]*Wpk[11][14][0])+(Tstar[14][1]*
          Wpk[11][14][1]))))+((((Fstar[13][2]*Vpk[11][13][2])+((Fstar[13][0]*
          Vpk[11][13][0])+(Fstar[13][1]*Vpk[11][13][1])))+((Tstar[13][2]*
          Wpk[11][13][2])+((Tstar[13][0]*Wpk[11][13][0])+(Tstar[13][1]*
          Wpk[11][13][1]))))+temp[0])));
        temp[0] = ((((Fstar[12][2]*Vpk[12][12][2])+((Fstar[12][0]*Vpk[12][12][0]
          )+(Fstar[12][1]*Vpk[12][12][1])))+((pin[12][2]*Tstar[12][2])+((
          pin[12][0]*Tstar[12][0])+(pin[12][1]*Tstar[12][1]))))+(((Fstar[13][2]*
          Vpk[12][13][2])+((Fstar[13][0]*Vpk[12][13][0])+(Fstar[13][1]*
          Vpk[12][13][1])))+((Tstar[13][2]*Wpk[12][13][2])+((Tstar[13][0]*
          Wpk[12][13][0])+(Tstar[13][1]*Wpk[12][13][1])))));
        fs0[12] = (utau[12]-((((Fstar[14][2]*Vpk[12][14][2])+((Fstar[14][0]*
          Vpk[12][14][0])+(Fstar[14][1]*Vpk[12][14][1])))+((Tstar[14][2]*
          Wpk[12][14][2])+((Tstar[14][0]*Wpk[12][14][0])+(Tstar[14][1]*
          Wpk[12][14][1]))))+temp[0]));
        fs0[13] = (utau[13]-((((Fstar[13][2]*Vpk[13][13][2])+((Fstar[13][0]*
          Vpk[13][13][0])+(Fstar[13][1]*Vpk[13][13][1])))+((pin[13][2]*
          Tstar[13][2])+((pin[13][0]*Tstar[13][0])+(pin[13][1]*Tstar[13][1]))))+
          (((Fstar[14][2]*Vpk[13][14][2])+((Fstar[14][0]*Vpk[13][14][0])+(
          Fstar[14][1]*Vpk[13][14][1])))+((Tstar[14][2]*Wpk[13][14][2])+((
          Tstar[14][0]*Wpk[13][14][0])+(Tstar[14][1]*Wpk[13][14][1]))))));
        fs0[14] = (utau[14]-(((Fstar[14][2]*Vpk[14][14][2])+((Fstar[14][0]*
          Vpk[14][14][0])+(Fstar[14][1]*Vpk[14][14][1])))+((pin[14][2]*
          Tstar[14][2])+((pin[14][0]*Tstar[14][0])+(pin[14][1]*Tstar[14][1])))))
          ;
        temp[0] = ((((Fstar[15][2]*Vpk[15][15][2])+((Fstar[15][0]*Vpk[15][15][0]
          )+(Fstar[15][1]*Vpk[15][15][1])))+((pin[15][2]*Tstar[15][2])+((
          pin[15][0]*Tstar[15][0])+(pin[15][1]*Tstar[15][1]))))+(((Fstar[16][2]*
          Vpk[15][16][2])+((Fstar[16][0]*Vpk[15][16][0])+(Fstar[16][1]*
          Vpk[15][16][1])))+((Tstar[16][2]*Wpk[15][16][2])+((Tstar[16][0]*
          Wpk[15][16][0])+(Tstar[16][1]*Wpk[15][16][1])))));
        temp[1] = ((((Fstar[18][2]*Vpk[15][18][2])+((Fstar[18][0]*Vpk[15][18][0]
          )+(Fstar[18][1]*Vpk[15][18][1])))+((Tstar[18][2]*Wpk[15][18][2])+((
          Tstar[18][0]*Wpk[15][18][0])+(Tstar[18][1]*Wpk[15][18][1]))))+((((
          Fstar[17][2]*Vpk[15][17][2])+((Fstar[17][0]*Vpk[15][17][0])+(
          Fstar[17][1]*Vpk[15][17][1])))+((Tstar[17][2]*Wpk[15][17][2])+((
          Tstar[17][0]*Wpk[15][17][0])+(Tstar[17][1]*Wpk[15][17][1]))))+temp[0])
          );
        fs0[15] = (utau[15]-((((Fstar[20][2]*Vpk[15][20][2])+((Fstar[20][0]*
          Vpk[15][20][0])+(Fstar[20][1]*Vpk[15][20][1])))+((Tstar[20][2]*
          Wpk[15][20][2])+((Tstar[20][0]*Wpk[15][20][0])+(Tstar[20][1]*
          Wpk[15][20][1]))))+((((Fstar[19][2]*Vpk[15][19][2])+((Fstar[19][0]*
          Vpk[15][19][0])+(Fstar[19][1]*Vpk[15][19][1])))+((Tstar[19][2]*
          Wpk[15][19][2])+((Tstar[19][0]*Wpk[15][19][0])+(Tstar[19][1]*
          Wpk[15][19][1]))))+temp[1])));
        temp[0] = ((((Fstar[16][2]*Vpk[16][16][2])+((Fstar[16][0]*Vpk[16][16][0]
          )+(Fstar[16][1]*Vpk[16][16][1])))+((pin[16][2]*Tstar[16][2])+((
          pin[16][0]*Tstar[16][0])+(pin[16][1]*Tstar[16][1]))))+(((Fstar[17][2]*
          Vpk[16][17][2])+((Fstar[17][0]*Vpk[16][17][0])+(Fstar[17][1]*
          Vpk[16][17][1])))+((Tstar[17][2]*Wpk[16][17][2])+((Tstar[17][0]*
          Wpk[16][17][0])+(Tstar[17][1]*Wpk[16][17][1])))));
        temp[1] = ((((Fstar[19][2]*Vpk[16][19][2])+((Fstar[19][0]*Vpk[16][19][0]
          )+(Fstar[19][1]*Vpk[16][19][1])))+((Tstar[19][2]*Wpk[16][19][2])+((
          Tstar[19][0]*Wpk[16][19][0])+(Tstar[19][1]*Wpk[16][19][1]))))+((((
          Fstar[18][2]*Vpk[16][18][2])+((Fstar[18][0]*Vpk[16][18][0])+(
          Fstar[18][1]*Vpk[16][18][1])))+((Tstar[18][2]*Wpk[16][18][2])+((
          Tstar[18][0]*Wpk[16][18][0])+(Tstar[18][1]*Wpk[16][18][1]))))+temp[0])
          );
        fs0[16] = (utau[16]-((((Fstar[20][2]*Vpk[16][20][2])+((Fstar[20][0]*
          Vpk[16][20][0])+(Fstar[20][1]*Vpk[16][20][1])))+((Tstar[20][2]*
          Wpk[16][20][2])+((Tstar[20][0]*Wpk[16][20][0])+(Tstar[20][1]*
          Wpk[16][20][1]))))+temp[1]));
        temp[0] = ((((Fstar[17][2]*Vpk[17][17][2])+((Fstar[17][0]*Vpk[17][17][0]
          )+(Fstar[17][1]*Vpk[17][17][1])))+((pin[17][2]*Tstar[17][2])+((
          pin[17][0]*Tstar[17][0])+(pin[17][1]*Tstar[17][1]))))+(((Fstar[18][2]*
          Vpk[17][18][2])+((Fstar[18][0]*Vpk[17][18][0])+(Fstar[18][1]*
          Vpk[17][18][1])))+((Tstar[18][2]*Wpk[17][18][2])+((Tstar[18][0]*
          Wpk[17][18][0])+(Tstar[18][1]*Wpk[17][18][1])))));
        fs0[17] = (utau[17]-((((Fstar[20][2]*Vpk[17][20][2])+((Fstar[20][0]*
          Vpk[17][20][0])+(Fstar[20][1]*Vpk[17][20][1])))+((Tstar[20][2]*
          Wpk[17][20][2])+((Tstar[20][0]*Wpk[17][20][0])+(Tstar[20][1]*
          Wpk[17][20][1]))))+((((Fstar[19][2]*Vpk[17][19][2])+((Fstar[19][0]*
          Vpk[17][19][0])+(Fstar[19][1]*Vpk[17][19][1])))+((Tstar[19][2]*
          Wpk[17][19][2])+((Tstar[19][0]*Wpk[17][19][0])+(Tstar[19][1]*
          Wpk[17][19][1]))))+temp[0])));
        temp[0] = ((((Fstar[18][2]*Vpk[18][18][2])+((Fstar[18][0]*Vpk[18][18][0]
          )+(Fstar[18][1]*Vpk[18][18][1])))+((pin[18][2]*Tstar[18][2])+((
          pin[18][0]*Tstar[18][0])+(pin[18][1]*Tstar[18][1]))))+(((Fstar[19][2]*
          Vpk[18][19][2])+((Fstar[19][0]*Vpk[18][19][0])+(Fstar[19][1]*
          Vpk[18][19][1])))+((Tstar[19][2]*Wpk[18][19][2])+((Tstar[19][0]*
          Wpk[18][19][0])+(Tstar[19][1]*Wpk[18][19][1])))));
        fs0[18] = (utau[18]-((((Fstar[20][2]*Vpk[18][20][2])+((Fstar[20][0]*
          Vpk[18][20][0])+(Fstar[20][1]*Vpk[18][20][1])))+((Tstar[20][2]*
          Wpk[18][20][2])+((Tstar[20][0]*Wpk[18][20][0])+(Tstar[20][1]*
          Wpk[18][20][1]))))+temp[0]));
        fs0[19] = (utau[19]-((((Fstar[19][2]*Vpk[19][19][2])+((Fstar[19][0]*
          Vpk[19][19][0])+(Fstar[19][1]*Vpk[19][19][1])))+((pin[19][2]*
          Tstar[19][2])+((pin[19][0]*Tstar[19][0])+(pin[19][1]*Tstar[19][1]))))+
          (((Fstar[20][2]*Vpk[19][20][2])+((Fstar[20][0]*Vpk[19][20][0])+(
          Fstar[20][1]*Vpk[19][20][1])))+((Tstar[20][2]*Wpk[19][20][2])+((
          Tstar[20][0]*Wpk[19][20][0])+(Tstar[20][1]*Wpk[19][20][1]))))));
        fs0[20] = (utau[20]-(((Fstar[20][2]*Vpk[20][20][2])+((Fstar[20][0]*
          Vpk[20][20][0])+(Fstar[20][1]*Vpk[20][20][1])))+((pin[20][2]*
          Tstar[20][2])+((pin[20][0]*Tstar[20][0])+(pin[20][1]*Tstar[20][1])))))
          ;
        fs0flg = 1;
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  990 adds/subtracts/negates
                    891 multiplies
                      0 divides
                    160 assignments
*/
}

void sddomm(int routine)
{
    int dumroutine,errnum;
    int i;

    if (mmflg == 0) {
/*
Compute mass matrix (MM)
*/
        sddovpk();
        IkWpk[3][6][0] = ((Cik[6][0][2]*ik[1][0][2])+((Cik[6][0][0]*ik[1][0][0])
          +(Cik[6][0][1]*ik[1][0][1])));
        IkWpk[3][6][1] = ((Cik[6][0][2]*ik[1][1][2])+((Cik[6][0][0]*ik[1][1][0])
          +(Cik[6][0][1]*ik[1][1][1])));
        IkWpk[3][6][2] = ((Cik[6][0][2]*ik[1][2][2])+((Cik[6][0][0]*ik[1][2][0])
          +(Cik[6][0][1]*ik[1][2][1])));
        IkWpk[3][7][0] = ((ik[2][0][2]*Wpk[3][7][2])+((ik[2][0][0]*Wpk[3][7][0])
          +(ik[2][0][1]*Wpk[3][7][1])));
        IkWpk[3][7][1] = ((ik[2][1][2]*Wpk[3][7][2])+((ik[2][1][0]*Wpk[3][7][0])
          +(ik[2][1][1]*Wpk[3][7][1])));
        IkWpk[3][7][2] = ((ik[2][2][2]*Wpk[3][7][2])+((ik[2][2][0]*Wpk[3][7][0])
          +(ik[2][2][1]*Wpk[3][7][1])));
        IkWpk[3][8][0] = ((ik[3][0][2]*Wpk[3][8][2])+((ik[3][0][0]*Wpk[3][8][0])
          +(ik[3][0][1]*Wpk[3][8][1])));
        IkWpk[3][8][1] = ((ik[3][1][2]*Wpk[3][8][2])+((ik[3][1][0]*Wpk[3][8][0])
          +(ik[3][1][1]*Wpk[3][8][1])));
        IkWpk[3][8][2] = ((ik[3][2][2]*Wpk[3][8][2])+((ik[3][2][0]*Wpk[3][8][0])
          +(ik[3][2][1]*Wpk[3][8][1])));
        IkWpk[3][9][0] = ((Cik[9][0][2]*ik[4][0][2])+((Cik[9][0][0]*ik[4][0][0])
          +(Cik[9][0][1]*ik[4][0][1])));
        IkWpk[3][9][1] = ((Cik[9][0][2]*ik[4][1][2])+((Cik[9][0][0]*ik[4][1][0])
          +(Cik[9][0][1]*ik[4][1][1])));
        IkWpk[3][9][2] = ((Cik[9][0][2]*ik[4][2][2])+((Cik[9][0][0]*ik[4][2][0])
          +(Cik[9][0][1]*ik[4][2][1])));
        IkWpk[3][10][0] = ((ik[5][0][2]*Wpk[3][10][2])+((ik[5][0][0]*
          Wpk[3][10][0])+(ik[5][0][1]*Wpk[3][10][1])));
        IkWpk[3][10][1] = ((ik[5][1][2]*Wpk[3][10][2])+((ik[5][1][0]*
          Wpk[3][10][0])+(ik[5][1][1]*Wpk[3][10][1])));
        IkWpk[3][10][2] = ((ik[5][2][2]*Wpk[3][10][2])+((ik[5][2][0]*
          Wpk[3][10][0])+(ik[5][2][1]*Wpk[3][10][1])));
        IkWpk[3][11][0] = ((ik[6][0][2]*Wpk[3][11][2])+((ik[6][0][0]*
          Wpk[3][11][0])+(ik[6][0][1]*Wpk[3][11][1])));
        IkWpk[3][11][1] = ((ik[6][1][2]*Wpk[3][11][2])+((ik[6][1][0]*
          Wpk[3][11][0])+(ik[6][1][1]*Wpk[3][11][1])));
        IkWpk[3][11][2] = ((ik[6][2][2]*Wpk[3][11][2])+((ik[6][2][0]*
          Wpk[3][11][0])+(ik[6][2][1]*Wpk[3][11][1])));
        IkWpk[3][12][0] = ((ik[7][0][2]*Wpk[3][12][2])+((ik[7][0][0]*
          Wpk[3][12][0])+(ik[7][0][1]*Wpk[3][12][1])));
        IkWpk[3][12][1] = ((ik[7][1][2]*Wpk[3][12][2])+((ik[7][1][0]*
          Wpk[3][12][0])+(ik[7][1][1]*Wpk[3][12][1])));
        IkWpk[3][12][2] = ((ik[7][2][2]*Wpk[3][12][2])+((ik[7][2][0]*
          Wpk[3][12][0])+(ik[7][2][1]*Wpk[3][12][1])));
        IkWpk[3][13][0] = ((ik[8][0][2]*Wpk[3][13][2])+((ik[8][0][0]*
          Wpk[3][13][0])+(ik[8][0][1]*Wpk[3][13][1])));
        IkWpk[3][13][1] = ((ik[8][1][2]*Wpk[3][13][2])+((ik[8][1][0]*
          Wpk[3][13][0])+(ik[8][1][1]*Wpk[3][13][1])));
        IkWpk[3][13][2] = ((ik[8][2][2]*Wpk[3][13][2])+((ik[8][2][0]*
          Wpk[3][13][0])+(ik[8][2][1]*Wpk[3][13][1])));
        IkWpk[3][14][0] = ((ik[9][0][2]*Wpk[3][14][2])+((ik[9][0][0]*
          Wpk[3][14][0])+(ik[9][0][1]*Wpk[3][14][1])));
        IkWpk[3][14][1] = ((ik[9][1][2]*Wpk[3][14][2])+((ik[9][1][0]*
          Wpk[3][14][0])+(ik[9][1][1]*Wpk[3][14][1])));
        IkWpk[3][14][2] = ((ik[9][2][2]*Wpk[3][14][2])+((ik[9][2][0]*
          Wpk[3][14][0])+(ik[9][2][1]*Wpk[3][14][1])));
        IkWpk[3][15][0] = ((Cik[15][0][2]*ik[10][0][2])+((Cik[15][0][0]*
          ik[10][0][0])+(Cik[15][0][1]*ik[10][0][1])));
        IkWpk[3][15][1] = ((Cik[15][0][2]*ik[10][1][2])+((Cik[15][0][0]*
          ik[10][1][0])+(Cik[15][0][1]*ik[10][1][1])));
        IkWpk[3][15][2] = ((Cik[15][0][2]*ik[10][2][2])+((Cik[15][0][0]*
          ik[10][2][0])+(Cik[15][0][1]*ik[10][2][1])));
        IkWpk[3][16][0] = ((ik[11][0][2]*Wpk[3][16][2])+((ik[11][0][0]*
          Wpk[3][16][0])+(ik[11][0][1]*Wpk[3][16][1])));
        IkWpk[3][16][1] = ((ik[11][1][2]*Wpk[3][16][2])+((ik[11][1][0]*
          Wpk[3][16][0])+(ik[11][1][1]*Wpk[3][16][1])));
        IkWpk[3][16][2] = ((ik[11][2][2]*Wpk[3][16][2])+((ik[11][2][0]*
          Wpk[3][16][0])+(ik[11][2][1]*Wpk[3][16][1])));
        IkWpk[3][17][0] = ((ik[12][0][2]*Wpk[3][17][2])+((ik[12][0][0]*
          Wpk[3][17][0])+(ik[12][0][1]*Wpk[3][17][1])));
        IkWpk[3][17][1] = ((ik[12][1][2]*Wpk[3][17][2])+((ik[12][1][0]*
          Wpk[3][17][0])+(ik[12][1][1]*Wpk[3][17][1])));
        IkWpk[3][17][2] = ((ik[12][2][2]*Wpk[3][17][2])+((ik[12][2][0]*
          Wpk[3][17][0])+(ik[12][2][1]*Wpk[3][17][1])));
        IkWpk[3][18][0] = ((ik[13][0][2]*Wpk[3][18][2])+((ik[13][0][0]*
          Wpk[3][18][0])+(ik[13][0][1]*Wpk[3][18][1])));
        IkWpk[3][18][1] = ((ik[13][1][2]*Wpk[3][18][2])+((ik[13][1][0]*
          Wpk[3][18][0])+(ik[13][1][1]*Wpk[3][18][1])));
        IkWpk[3][18][2] = ((ik[13][2][2]*Wpk[3][18][2])+((ik[13][2][0]*
          Wpk[3][18][0])+(ik[13][2][1]*Wpk[3][18][1])));
        IkWpk[3][19][0] = ((ik[14][0][2]*Wpk[3][19][2])+((ik[14][0][0]*
          Wpk[3][19][0])+(ik[14][0][1]*Wpk[3][19][1])));
        IkWpk[3][19][1] = ((ik[14][1][2]*Wpk[3][19][2])+((ik[14][1][0]*
          Wpk[3][19][0])+(ik[14][1][1]*Wpk[3][19][1])));
        IkWpk[3][19][2] = ((ik[14][2][2]*Wpk[3][19][2])+((ik[14][2][0]*
          Wpk[3][19][0])+(ik[14][2][1]*Wpk[3][19][1])));
        IkWpk[3][20][0] = ((ik[15][0][2]*Wpk[3][20][2])+((ik[15][0][0]*
          Wpk[3][20][0])+(ik[15][0][1]*Wpk[3][20][1])));
        IkWpk[3][20][1] = ((ik[15][1][2]*Wpk[3][20][2])+((ik[15][1][0]*
          Wpk[3][20][0])+(ik[15][1][1]*Wpk[3][20][1])));
        IkWpk[3][20][2] = ((ik[15][2][2]*Wpk[3][20][2])+((ik[15][2][0]*
          Wpk[3][20][0])+(ik[15][2][1]*Wpk[3][20][1])));
        IkWpk[4][6][0] = ((Cik[6][1][2]*ik[1][0][2])+((Cik[6][1][0]*ik[1][0][0])
          +(Cik[6][1][1]*ik[1][0][1])));
        IkWpk[4][6][1] = ((Cik[6][1][2]*ik[1][1][2])+((Cik[6][1][0]*ik[1][1][0])
          +(Cik[6][1][1]*ik[1][1][1])));
        IkWpk[4][6][2] = ((Cik[6][1][2]*ik[1][2][2])+((Cik[6][1][0]*ik[1][2][0])
          +(Cik[6][1][1]*ik[1][2][1])));
        IkWpk[4][7][0] = ((ik[2][0][2]*Wpk[4][7][2])+((ik[2][0][0]*Wpk[4][7][0])
          +(ik[2][0][1]*Wpk[4][7][1])));
        IkWpk[4][7][1] = ((ik[2][1][2]*Wpk[4][7][2])+((ik[2][1][0]*Wpk[4][7][0])
          +(ik[2][1][1]*Wpk[4][7][1])));
        IkWpk[4][7][2] = ((ik[2][2][2]*Wpk[4][7][2])+((ik[2][2][0]*Wpk[4][7][0])
          +(ik[2][2][1]*Wpk[4][7][1])));
        IkWpk[4][8][0] = ((ik[3][0][2]*Wpk[4][8][2])+((ik[3][0][0]*Wpk[4][8][0])
          +(ik[3][0][1]*Wpk[4][8][1])));
        IkWpk[4][8][1] = ((ik[3][1][2]*Wpk[4][8][2])+((ik[3][1][0]*Wpk[4][8][0])
          +(ik[3][1][1]*Wpk[4][8][1])));
        IkWpk[4][8][2] = ((ik[3][2][2]*Wpk[4][8][2])+((ik[3][2][0]*Wpk[4][8][0])
          +(ik[3][2][1]*Wpk[4][8][1])));
        IkWpk[4][9][0] = ((Cik[9][1][2]*ik[4][0][2])+((Cik[9][1][0]*ik[4][0][0])
          +(Cik[9][1][1]*ik[4][0][1])));
        IkWpk[4][9][1] = ((Cik[9][1][2]*ik[4][1][2])+((Cik[9][1][0]*ik[4][1][0])
          +(Cik[9][1][1]*ik[4][1][1])));
        IkWpk[4][9][2] = ((Cik[9][1][2]*ik[4][2][2])+((Cik[9][1][0]*ik[4][2][0])
          +(Cik[9][1][1]*ik[4][2][1])));
        IkWpk[4][10][0] = ((ik[5][0][2]*Wpk[4][10][2])+((ik[5][0][0]*
          Wpk[4][10][0])+(ik[5][0][1]*Wpk[4][10][1])));
        IkWpk[4][10][1] = ((ik[5][1][2]*Wpk[4][10][2])+((ik[5][1][0]*
          Wpk[4][10][0])+(ik[5][1][1]*Wpk[4][10][1])));
        IkWpk[4][10][2] = ((ik[5][2][2]*Wpk[4][10][2])+((ik[5][2][0]*
          Wpk[4][10][0])+(ik[5][2][1]*Wpk[4][10][1])));
        IkWpk[4][11][0] = ((ik[6][0][2]*Wpk[4][11][2])+((ik[6][0][0]*
          Wpk[4][11][0])+(ik[6][0][1]*Wpk[4][11][1])));
        IkWpk[4][11][1] = ((ik[6][1][2]*Wpk[4][11][2])+((ik[6][1][0]*
          Wpk[4][11][0])+(ik[6][1][1]*Wpk[4][11][1])));
        IkWpk[4][11][2] = ((ik[6][2][2]*Wpk[4][11][2])+((ik[6][2][0]*
          Wpk[4][11][0])+(ik[6][2][1]*Wpk[4][11][1])));
        IkWpk[4][12][0] = ((ik[7][0][2]*Wpk[4][12][2])+((ik[7][0][0]*
          Wpk[4][12][0])+(ik[7][0][1]*Wpk[4][12][1])));
        IkWpk[4][12][1] = ((ik[7][1][2]*Wpk[4][12][2])+((ik[7][1][0]*
          Wpk[4][12][0])+(ik[7][1][1]*Wpk[4][12][1])));
        IkWpk[4][12][2] = ((ik[7][2][2]*Wpk[4][12][2])+((ik[7][2][0]*
          Wpk[4][12][0])+(ik[7][2][1]*Wpk[4][12][1])));
        IkWpk[4][13][0] = ((ik[8][0][2]*Wpk[4][13][2])+((ik[8][0][0]*
          Wpk[4][13][0])+(ik[8][0][1]*Wpk[4][13][1])));
        IkWpk[4][13][1] = ((ik[8][1][2]*Wpk[4][13][2])+((ik[8][1][0]*
          Wpk[4][13][0])+(ik[8][1][1]*Wpk[4][13][1])));
        IkWpk[4][13][2] = ((ik[8][2][2]*Wpk[4][13][2])+((ik[8][2][0]*
          Wpk[4][13][0])+(ik[8][2][1]*Wpk[4][13][1])));
        IkWpk[4][14][0] = ((ik[9][0][2]*Wpk[4][14][2])+((ik[9][0][0]*
          Wpk[4][14][0])+(ik[9][0][1]*Wpk[4][14][1])));
        IkWpk[4][14][1] = ((ik[9][1][2]*Wpk[4][14][2])+((ik[9][1][0]*
          Wpk[4][14][0])+(ik[9][1][1]*Wpk[4][14][1])));
        IkWpk[4][14][2] = ((ik[9][2][2]*Wpk[4][14][2])+((ik[9][2][0]*
          Wpk[4][14][0])+(ik[9][2][1]*Wpk[4][14][1])));
        IkWpk[4][15][0] = ((Cik[15][1][2]*ik[10][0][2])+((Cik[15][1][0]*
          ik[10][0][0])+(Cik[15][1][1]*ik[10][0][1])));
        IkWpk[4][15][1] = ((Cik[15][1][2]*ik[10][1][2])+((Cik[15][1][0]*
          ik[10][1][0])+(Cik[15][1][1]*ik[10][1][1])));
        IkWpk[4][15][2] = ((Cik[15][1][2]*ik[10][2][2])+((Cik[15][1][0]*
          ik[10][2][0])+(Cik[15][1][1]*ik[10][2][1])));
        IkWpk[4][16][0] = ((ik[11][0][2]*Wpk[4][16][2])+((ik[11][0][0]*
          Wpk[4][16][0])+(ik[11][0][1]*Wpk[4][16][1])));
        IkWpk[4][16][1] = ((ik[11][1][2]*Wpk[4][16][2])+((ik[11][1][0]*
          Wpk[4][16][0])+(ik[11][1][1]*Wpk[4][16][1])));
        IkWpk[4][16][2] = ((ik[11][2][2]*Wpk[4][16][2])+((ik[11][2][0]*
          Wpk[4][16][0])+(ik[11][2][1]*Wpk[4][16][1])));
        IkWpk[4][17][0] = ((ik[12][0][2]*Wpk[4][17][2])+((ik[12][0][0]*
          Wpk[4][17][0])+(ik[12][0][1]*Wpk[4][17][1])));
        IkWpk[4][17][1] = ((ik[12][1][2]*Wpk[4][17][2])+((ik[12][1][0]*
          Wpk[4][17][0])+(ik[12][1][1]*Wpk[4][17][1])));
        IkWpk[4][17][2] = ((ik[12][2][2]*Wpk[4][17][2])+((ik[12][2][0]*
          Wpk[4][17][0])+(ik[12][2][1]*Wpk[4][17][1])));
        IkWpk[4][18][0] = ((ik[13][0][2]*Wpk[4][18][2])+((ik[13][0][0]*
          Wpk[4][18][0])+(ik[13][0][1]*Wpk[4][18][1])));
        IkWpk[4][18][1] = ((ik[13][1][2]*Wpk[4][18][2])+((ik[13][1][0]*
          Wpk[4][18][0])+(ik[13][1][1]*Wpk[4][18][1])));
        IkWpk[4][18][2] = ((ik[13][2][2]*Wpk[4][18][2])+((ik[13][2][0]*
          Wpk[4][18][0])+(ik[13][2][1]*Wpk[4][18][1])));
        IkWpk[4][19][0] = ((ik[14][0][2]*Wpk[4][19][2])+((ik[14][0][0]*
          Wpk[4][19][0])+(ik[14][0][1]*Wpk[4][19][1])));
        IkWpk[4][19][1] = ((ik[14][1][2]*Wpk[4][19][2])+((ik[14][1][0]*
          Wpk[4][19][0])+(ik[14][1][1]*Wpk[4][19][1])));
        IkWpk[4][19][2] = ((ik[14][2][2]*Wpk[4][19][2])+((ik[14][2][0]*
          Wpk[4][19][0])+(ik[14][2][1]*Wpk[4][19][1])));
        IkWpk[4][20][0] = ((ik[15][0][2]*Wpk[4][20][2])+((ik[15][0][0]*
          Wpk[4][20][0])+(ik[15][0][1]*Wpk[4][20][1])));
        IkWpk[4][20][1] = ((ik[15][1][2]*Wpk[4][20][2])+((ik[15][1][0]*
          Wpk[4][20][0])+(ik[15][1][1]*Wpk[4][20][1])));
        IkWpk[4][20][2] = ((ik[15][2][2]*Wpk[4][20][2])+((ik[15][2][0]*
          Wpk[4][20][0])+(ik[15][2][1]*Wpk[4][20][1])));
        IkWpk[5][6][0] = ((Cik[6][2][2]*ik[1][0][2])+((Cik[6][2][0]*ik[1][0][0])
          +(Cik[6][2][1]*ik[1][0][1])));
        IkWpk[5][6][1] = ((Cik[6][2][2]*ik[1][1][2])+((Cik[6][2][0]*ik[1][1][0])
          +(Cik[6][2][1]*ik[1][1][1])));
        IkWpk[5][6][2] = ((Cik[6][2][2]*ik[1][2][2])+((Cik[6][2][0]*ik[1][2][0])
          +(Cik[6][2][1]*ik[1][2][1])));
        IkWpk[5][7][0] = ((ik[2][0][2]*Wpk[5][7][2])+((ik[2][0][0]*Wpk[5][7][0])
          +(ik[2][0][1]*Wpk[5][7][1])));
        IkWpk[5][7][1] = ((ik[2][1][2]*Wpk[5][7][2])+((ik[2][1][0]*Wpk[5][7][0])
          +(ik[2][1][1]*Wpk[5][7][1])));
        IkWpk[5][7][2] = ((ik[2][2][2]*Wpk[5][7][2])+((ik[2][2][0]*Wpk[5][7][0])
          +(ik[2][2][1]*Wpk[5][7][1])));
        IkWpk[5][8][0] = ((ik[3][0][2]*Wpk[5][8][2])+((ik[3][0][0]*Wpk[5][8][0])
          +(ik[3][0][1]*Wpk[5][8][1])));
        IkWpk[5][8][1] = ((ik[3][1][2]*Wpk[5][8][2])+((ik[3][1][0]*Wpk[5][8][0])
          +(ik[3][1][1]*Wpk[5][8][1])));
        IkWpk[5][8][2] = ((ik[3][2][2]*Wpk[5][8][2])+((ik[3][2][0]*Wpk[5][8][0])
          +(ik[3][2][1]*Wpk[5][8][1])));
        IkWpk[5][9][0] = ((Cik[9][2][2]*ik[4][0][2])+((Cik[9][2][0]*ik[4][0][0])
          +(Cik[9][2][1]*ik[4][0][1])));
        IkWpk[5][9][1] = ((Cik[9][2][2]*ik[4][1][2])+((Cik[9][2][0]*ik[4][1][0])
          +(Cik[9][2][1]*ik[4][1][1])));
        IkWpk[5][9][2] = ((Cik[9][2][2]*ik[4][2][2])+((Cik[9][2][0]*ik[4][2][0])
          +(Cik[9][2][1]*ik[4][2][1])));
        IkWpk[5][10][0] = ((ik[5][0][2]*Wpk[5][10][2])+((ik[5][0][0]*
          Wpk[5][10][0])+(ik[5][0][1]*Wpk[5][10][1])));
        IkWpk[5][10][1] = ((ik[5][1][2]*Wpk[5][10][2])+((ik[5][1][0]*
          Wpk[5][10][0])+(ik[5][1][1]*Wpk[5][10][1])));
        IkWpk[5][10][2] = ((ik[5][2][2]*Wpk[5][10][2])+((ik[5][2][0]*
          Wpk[5][10][0])+(ik[5][2][1]*Wpk[5][10][1])));
        IkWpk[5][11][0] = ((ik[6][0][2]*Wpk[5][11][2])+((ik[6][0][0]*
          Wpk[5][11][0])+(ik[6][0][1]*Wpk[5][11][1])));
        IkWpk[5][11][1] = ((ik[6][1][2]*Wpk[5][11][2])+((ik[6][1][0]*
          Wpk[5][11][0])+(ik[6][1][1]*Wpk[5][11][1])));
        IkWpk[5][11][2] = ((ik[6][2][2]*Wpk[5][11][2])+((ik[6][2][0]*
          Wpk[5][11][0])+(ik[6][2][1]*Wpk[5][11][1])));
        IkWpk[5][12][0] = ((ik[7][0][2]*Wpk[5][12][2])+((ik[7][0][0]*
          Wpk[5][12][0])+(ik[7][0][1]*Wpk[5][12][1])));
        IkWpk[5][12][1] = ((ik[7][1][2]*Wpk[5][12][2])+((ik[7][1][0]*
          Wpk[5][12][0])+(ik[7][1][1]*Wpk[5][12][1])));
        IkWpk[5][12][2] = ((ik[7][2][2]*Wpk[5][12][2])+((ik[7][2][0]*
          Wpk[5][12][0])+(ik[7][2][1]*Wpk[5][12][1])));
        IkWpk[5][13][0] = ((ik[8][0][2]*Wpk[5][13][2])+((ik[8][0][0]*
          Wpk[5][13][0])+(ik[8][0][1]*Wpk[5][13][1])));
        IkWpk[5][13][1] = ((ik[8][1][2]*Wpk[5][13][2])+((ik[8][1][0]*
          Wpk[5][13][0])+(ik[8][1][1]*Wpk[5][13][1])));
        IkWpk[5][13][2] = ((ik[8][2][2]*Wpk[5][13][2])+((ik[8][2][0]*
          Wpk[5][13][0])+(ik[8][2][1]*Wpk[5][13][1])));
        IkWpk[5][14][0] = ((ik[9][0][2]*Wpk[5][14][2])+((ik[9][0][0]*
          Wpk[5][14][0])+(ik[9][0][1]*Wpk[5][14][1])));
        IkWpk[5][14][1] = ((ik[9][1][2]*Wpk[5][14][2])+((ik[9][1][0]*
          Wpk[5][14][0])+(ik[9][1][1]*Wpk[5][14][1])));
        IkWpk[5][14][2] = ((ik[9][2][2]*Wpk[5][14][2])+((ik[9][2][0]*
          Wpk[5][14][0])+(ik[9][2][1]*Wpk[5][14][1])));
        IkWpk[5][15][0] = ((Cik[15][2][2]*ik[10][0][2])+((Cik[15][2][0]*
          ik[10][0][0])+(Cik[15][2][1]*ik[10][0][1])));
        IkWpk[5][15][1] = ((Cik[15][2][2]*ik[10][1][2])+((Cik[15][2][0]*
          ik[10][1][0])+(Cik[15][2][1]*ik[10][1][1])));
        IkWpk[5][15][2] = ((Cik[15][2][2]*ik[10][2][2])+((Cik[15][2][0]*
          ik[10][2][0])+(Cik[15][2][1]*ik[10][2][1])));
        IkWpk[5][16][0] = ((ik[11][0][2]*Wpk[5][16][2])+((ik[11][0][0]*
          Wpk[5][16][0])+(ik[11][0][1]*Wpk[5][16][1])));
        IkWpk[5][16][1] = ((ik[11][1][2]*Wpk[5][16][2])+((ik[11][1][0]*
          Wpk[5][16][0])+(ik[11][1][1]*Wpk[5][16][1])));
        IkWpk[5][16][2] = ((ik[11][2][2]*Wpk[5][16][2])+((ik[11][2][0]*
          Wpk[5][16][0])+(ik[11][2][1]*Wpk[5][16][1])));
        IkWpk[5][17][0] = ((ik[12][0][2]*Wpk[5][17][2])+((ik[12][0][0]*
          Wpk[5][17][0])+(ik[12][0][1]*Wpk[5][17][1])));
        IkWpk[5][17][1] = ((ik[12][1][2]*Wpk[5][17][2])+((ik[12][1][0]*
          Wpk[5][17][0])+(ik[12][1][1]*Wpk[5][17][1])));
        IkWpk[5][17][2] = ((ik[12][2][2]*Wpk[5][17][2])+((ik[12][2][0]*
          Wpk[5][17][0])+(ik[12][2][1]*Wpk[5][17][1])));
        IkWpk[5][18][0] = ((ik[13][0][2]*Wpk[5][18][2])+((ik[13][0][0]*
          Wpk[5][18][0])+(ik[13][0][1]*Wpk[5][18][1])));
        IkWpk[5][18][1] = ((ik[13][1][2]*Wpk[5][18][2])+((ik[13][1][0]*
          Wpk[5][18][0])+(ik[13][1][1]*Wpk[5][18][1])));
        IkWpk[5][18][2] = ((ik[13][2][2]*Wpk[5][18][2])+((ik[13][2][0]*
          Wpk[5][18][0])+(ik[13][2][1]*Wpk[5][18][1])));
        IkWpk[5][19][0] = ((ik[14][0][2]*Wpk[5][19][2])+((ik[14][0][0]*
          Wpk[5][19][0])+(ik[14][0][1]*Wpk[5][19][1])));
        IkWpk[5][19][1] = ((ik[14][1][2]*Wpk[5][19][2])+((ik[14][1][0]*
          Wpk[5][19][0])+(ik[14][1][1]*Wpk[5][19][1])));
        IkWpk[5][19][2] = ((ik[14][2][2]*Wpk[5][19][2])+((ik[14][2][0]*
          Wpk[5][19][0])+(ik[14][2][1]*Wpk[5][19][1])));
        IkWpk[5][20][0] = ((ik[15][0][2]*Wpk[5][20][2])+((ik[15][0][0]*
          Wpk[5][20][0])+(ik[15][0][1]*Wpk[5][20][1])));
        IkWpk[5][20][1] = ((ik[15][1][2]*Wpk[5][20][2])+((ik[15][1][0]*
          Wpk[5][20][0])+(ik[15][1][1]*Wpk[5][20][1])));
        IkWpk[5][20][2] = ((ik[15][2][2]*Wpk[5][20][2])+((ik[15][2][0]*
          Wpk[5][20][0])+(ik[15][2][1]*Wpk[5][20][1])));
        IkWpk[6][6][0] = ((ik[1][0][2]*pin[6][2])+((ik[1][0][0]*pin[6][0])+(
          ik[1][0][1]*pin[6][1])));
        IkWpk[6][6][1] = ((ik[1][1][2]*pin[6][2])+((ik[1][1][0]*pin[6][0])+(
          ik[1][1][1]*pin[6][1])));
        IkWpk[6][6][2] = ((ik[1][2][2]*pin[6][2])+((ik[1][2][0]*pin[6][0])+(
          ik[1][2][1]*pin[6][1])));
        IkWpk[6][7][0] = ((ik[2][0][2]*Wpk[6][7][2])+((ik[2][0][0]*Wpk[6][7][0])
          +(ik[2][0][1]*Wpk[6][7][1])));
        IkWpk[6][7][1] = ((ik[2][1][2]*Wpk[6][7][2])+((ik[2][1][0]*Wpk[6][7][0])
          +(ik[2][1][1]*Wpk[6][7][1])));
        IkWpk[6][7][2] = ((ik[2][2][2]*Wpk[6][7][2])+((ik[2][2][0]*Wpk[6][7][0])
          +(ik[2][2][1]*Wpk[6][7][1])));
        IkWpk[6][8][0] = ((ik[3][0][2]*Wpk[6][8][2])+((ik[3][0][0]*Wpk[6][8][0])
          +(ik[3][0][1]*Wpk[6][8][1])));
        IkWpk[6][8][1] = ((ik[3][1][2]*Wpk[6][8][2])+((ik[3][1][0]*Wpk[6][8][0])
          +(ik[3][1][1]*Wpk[6][8][1])));
        IkWpk[6][8][2] = ((ik[3][2][2]*Wpk[6][8][2])+((ik[3][2][0]*Wpk[6][8][0])
          +(ik[3][2][1]*Wpk[6][8][1])));
        IkWpk[7][7][0] = ((ik[2][0][2]*pin[7][2])+((ik[2][0][0]*pin[7][0])+(
          ik[2][0][1]*pin[7][1])));
        IkWpk[7][7][1] = ((ik[2][1][2]*pin[7][2])+((ik[2][1][0]*pin[7][0])+(
          ik[2][1][1]*pin[7][1])));
        IkWpk[7][7][2] = ((ik[2][2][2]*pin[7][2])+((ik[2][2][0]*pin[7][0])+(
          ik[2][2][1]*pin[7][1])));
        IkWpk[7][8][0] = ((ik[3][0][2]*Wpk[7][8][2])+((ik[3][0][0]*Wpk[7][8][0])
          +(ik[3][0][1]*Wpk[7][8][1])));
        IkWpk[7][8][1] = ((ik[3][1][2]*Wpk[7][8][2])+((ik[3][1][0]*Wpk[7][8][0])
          +(ik[3][1][1]*Wpk[7][8][1])));
        IkWpk[7][8][2] = ((ik[3][2][2]*Wpk[7][8][2])+((ik[3][2][0]*Wpk[7][8][0])
          +(ik[3][2][1]*Wpk[7][8][1])));
        IkWpk[8][8][0] = ((ik[3][0][2]*pin[8][2])+((ik[3][0][0]*pin[8][0])+(
          ik[3][0][1]*pin[8][1])));
        IkWpk[8][8][1] = ((ik[3][1][2]*pin[8][2])+((ik[3][1][0]*pin[8][0])+(
          ik[3][1][1]*pin[8][1])));
        IkWpk[8][8][2] = ((ik[3][2][2]*pin[8][2])+((ik[3][2][0]*pin[8][0])+(
          ik[3][2][1]*pin[8][1])));
        IkWpk[9][9][0] = ((ik[4][0][2]*pin[9][2])+((ik[4][0][0]*pin[9][0])+(
          ik[4][0][1]*pin[9][1])));
        IkWpk[9][9][1] = ((ik[4][1][2]*pin[9][2])+((ik[4][1][0]*pin[9][0])+(
          ik[4][1][1]*pin[9][1])));
        IkWpk[9][9][2] = ((ik[4][2][2]*pin[9][2])+((ik[4][2][0]*pin[9][0])+(
          ik[4][2][1]*pin[9][1])));
        IkWpk[9][10][0] = ((ik[5][0][2]*Wpk[9][10][2])+((ik[5][0][0]*
          Wpk[9][10][0])+(ik[5][0][1]*Wpk[9][10][1])));
        IkWpk[9][10][1] = ((ik[5][1][2]*Wpk[9][10][2])+((ik[5][1][0]*
          Wpk[9][10][0])+(ik[5][1][1]*Wpk[9][10][1])));
        IkWpk[9][10][2] = ((ik[5][2][2]*Wpk[9][10][2])+((ik[5][2][0]*
          Wpk[9][10][0])+(ik[5][2][1]*Wpk[9][10][1])));
        IkWpk[9][11][0] = ((ik[6][0][2]*Wpk[9][11][2])+((ik[6][0][0]*
          Wpk[9][11][0])+(ik[6][0][1]*Wpk[9][11][1])));
        IkWpk[9][11][1] = ((ik[6][1][2]*Wpk[9][11][2])+((ik[6][1][0]*
          Wpk[9][11][0])+(ik[6][1][1]*Wpk[9][11][1])));
        IkWpk[9][11][2] = ((ik[6][2][2]*Wpk[9][11][2])+((ik[6][2][0]*
          Wpk[9][11][0])+(ik[6][2][1]*Wpk[9][11][1])));
        IkWpk[9][12][0] = ((ik[7][0][2]*Wpk[9][12][2])+((ik[7][0][0]*
          Wpk[9][12][0])+(ik[7][0][1]*Wpk[9][12][1])));
        IkWpk[9][12][1] = ((ik[7][1][2]*Wpk[9][12][2])+((ik[7][1][0]*
          Wpk[9][12][0])+(ik[7][1][1]*Wpk[9][12][1])));
        IkWpk[9][12][2] = ((ik[7][2][2]*Wpk[9][12][2])+((ik[7][2][0]*
          Wpk[9][12][0])+(ik[7][2][1]*Wpk[9][12][1])));
        IkWpk[9][13][0] = ((ik[8][0][2]*Wpk[9][13][2])+((ik[8][0][0]*
          Wpk[9][13][0])+(ik[8][0][1]*Wpk[9][13][1])));
        IkWpk[9][13][1] = ((ik[8][1][2]*Wpk[9][13][2])+((ik[8][1][0]*
          Wpk[9][13][0])+(ik[8][1][1]*Wpk[9][13][1])));
        IkWpk[9][13][2] = ((ik[8][2][2]*Wpk[9][13][2])+((ik[8][2][0]*
          Wpk[9][13][0])+(ik[8][2][1]*Wpk[9][13][1])));
        IkWpk[9][14][0] = ((ik[9][0][2]*Wpk[9][14][2])+((ik[9][0][0]*
          Wpk[9][14][0])+(ik[9][0][1]*Wpk[9][14][1])));
        IkWpk[9][14][1] = ((ik[9][1][2]*Wpk[9][14][2])+((ik[9][1][0]*
          Wpk[9][14][0])+(ik[9][1][1]*Wpk[9][14][1])));
        IkWpk[9][14][2] = ((ik[9][2][2]*Wpk[9][14][2])+((ik[9][2][0]*
          Wpk[9][14][0])+(ik[9][2][1]*Wpk[9][14][1])));
        IkWpk[10][10][0] = ((ik[5][0][2]*pin[10][2])+((ik[5][0][0]*pin[10][0])+(
          ik[5][0][1]*pin[10][1])));
        IkWpk[10][10][1] = ((ik[5][1][2]*pin[10][2])+((ik[5][1][0]*pin[10][0])+(
          ik[5][1][1]*pin[10][1])));
        IkWpk[10][10][2] = ((ik[5][2][2]*pin[10][2])+((ik[5][2][0]*pin[10][0])+(
          ik[5][2][1]*pin[10][1])));
        IkWpk[10][11][0] = ((ik[6][0][2]*Wpk[10][11][2])+((ik[6][0][0]*
          Wpk[10][11][0])+(ik[6][0][1]*Wpk[10][11][1])));
        IkWpk[10][11][1] = ((ik[6][1][2]*Wpk[10][11][2])+((ik[6][1][0]*
          Wpk[10][11][0])+(ik[6][1][1]*Wpk[10][11][1])));
        IkWpk[10][11][2] = ((ik[6][2][2]*Wpk[10][11][2])+((ik[6][2][0]*
          Wpk[10][11][0])+(ik[6][2][1]*Wpk[10][11][1])));
        IkWpk[10][12][0] = ((ik[7][0][2]*Wpk[10][12][2])+((ik[7][0][0]*
          Wpk[10][12][0])+(ik[7][0][1]*Wpk[10][12][1])));
        IkWpk[10][12][1] = ((ik[7][1][2]*Wpk[10][12][2])+((ik[7][1][0]*
          Wpk[10][12][0])+(ik[7][1][1]*Wpk[10][12][1])));
        IkWpk[10][12][2] = ((ik[7][2][2]*Wpk[10][12][2])+((ik[7][2][0]*
          Wpk[10][12][0])+(ik[7][2][1]*Wpk[10][12][1])));
        IkWpk[10][13][0] = ((ik[8][0][2]*Wpk[10][13][2])+((ik[8][0][0]*
          Wpk[10][13][0])+(ik[8][0][1]*Wpk[10][13][1])));
        IkWpk[10][13][1] = ((ik[8][1][2]*Wpk[10][13][2])+((ik[8][1][0]*
          Wpk[10][13][0])+(ik[8][1][1]*Wpk[10][13][1])));
        IkWpk[10][13][2] = ((ik[8][2][2]*Wpk[10][13][2])+((ik[8][2][0]*
          Wpk[10][13][0])+(ik[8][2][1]*Wpk[10][13][1])));
        IkWpk[10][14][0] = ((ik[9][0][2]*Wpk[10][14][2])+((ik[9][0][0]*
          Wpk[10][14][0])+(ik[9][0][1]*Wpk[10][14][1])));
        IkWpk[10][14][1] = ((ik[9][1][2]*Wpk[10][14][2])+((ik[9][1][0]*
          Wpk[10][14][0])+(ik[9][1][1]*Wpk[10][14][1])));
        IkWpk[10][14][2] = ((ik[9][2][2]*Wpk[10][14][2])+((ik[9][2][0]*
          Wpk[10][14][0])+(ik[9][2][1]*Wpk[10][14][1])));
        IkWpk[11][11][0] = ((ik[6][0][2]*pin[11][2])+((ik[6][0][0]*pin[11][0])+(
          ik[6][0][1]*pin[11][1])));
        IkWpk[11][11][1] = ((ik[6][1][2]*pin[11][2])+((ik[6][1][0]*pin[11][0])+(
          ik[6][1][1]*pin[11][1])));
        IkWpk[11][11][2] = ((ik[6][2][2]*pin[11][2])+((ik[6][2][0]*pin[11][0])+(
          ik[6][2][1]*pin[11][1])));
        IkWpk[11][12][0] = ((ik[7][0][2]*Wpk[11][12][2])+((ik[7][0][0]*
          Wpk[11][12][0])+(ik[7][0][1]*Wpk[11][12][1])));
        IkWpk[11][12][1] = ((ik[7][1][2]*Wpk[11][12][2])+((ik[7][1][0]*
          Wpk[11][12][0])+(ik[7][1][1]*Wpk[11][12][1])));
        IkWpk[11][12][2] = ((ik[7][2][2]*Wpk[11][12][2])+((ik[7][2][0]*
          Wpk[11][12][0])+(ik[7][2][1]*Wpk[11][12][1])));
        IkWpk[11][13][0] = ((ik[8][0][2]*Wpk[11][13][2])+((ik[8][0][0]*
          Wpk[11][13][0])+(ik[8][0][1]*Wpk[11][13][1])));
        IkWpk[11][13][1] = ((ik[8][1][2]*Wpk[11][13][2])+((ik[8][1][0]*
          Wpk[11][13][0])+(ik[8][1][1]*Wpk[11][13][1])));
        IkWpk[11][13][2] = ((ik[8][2][2]*Wpk[11][13][2])+((ik[8][2][0]*
          Wpk[11][13][0])+(ik[8][2][1]*Wpk[11][13][1])));
        IkWpk[11][14][0] = ((ik[9][0][2]*Wpk[11][14][2])+((ik[9][0][0]*
          Wpk[11][14][0])+(ik[9][0][1]*Wpk[11][14][1])));
        IkWpk[11][14][1] = ((ik[9][1][2]*Wpk[11][14][2])+((ik[9][1][0]*
          Wpk[11][14][0])+(ik[9][1][1]*Wpk[11][14][1])));
        IkWpk[11][14][2] = ((ik[9][2][2]*Wpk[11][14][2])+((ik[9][2][0]*
          Wpk[11][14][0])+(ik[9][2][1]*Wpk[11][14][1])));
        IkWpk[12][12][0] = ((ik[7][0][2]*pin[12][2])+((ik[7][0][0]*pin[12][0])+(
          ik[7][0][1]*pin[12][1])));
        IkWpk[12][12][1] = ((ik[7][1][2]*pin[12][2])+((ik[7][1][0]*pin[12][0])+(
          ik[7][1][1]*pin[12][1])));
        IkWpk[12][12][2] = ((ik[7][2][2]*pin[12][2])+((ik[7][2][0]*pin[12][0])+(
          ik[7][2][1]*pin[12][1])));
        IkWpk[12][13][0] = ((ik[8][0][2]*Wpk[12][13][2])+((ik[8][0][0]*
          Wpk[12][13][0])+(ik[8][0][1]*Wpk[12][13][1])));
        IkWpk[12][13][1] = ((ik[8][1][2]*Wpk[12][13][2])+((ik[8][1][0]*
          Wpk[12][13][0])+(ik[8][1][1]*Wpk[12][13][1])));
        IkWpk[12][13][2] = ((ik[8][2][2]*Wpk[12][13][2])+((ik[8][2][0]*
          Wpk[12][13][0])+(ik[8][2][1]*Wpk[12][13][1])));
        IkWpk[12][14][0] = ((ik[9][0][2]*Wpk[12][14][2])+((ik[9][0][0]*
          Wpk[12][14][0])+(ik[9][0][1]*Wpk[12][14][1])));
        IkWpk[12][14][1] = ((ik[9][1][2]*Wpk[12][14][2])+((ik[9][1][0]*
          Wpk[12][14][0])+(ik[9][1][1]*Wpk[12][14][1])));
        IkWpk[12][14][2] = ((ik[9][2][2]*Wpk[12][14][2])+((ik[9][2][0]*
          Wpk[12][14][0])+(ik[9][2][1]*Wpk[12][14][1])));
        IkWpk[13][13][0] = ((ik[8][0][2]*pin[13][2])+((ik[8][0][0]*pin[13][0])+(
          ik[8][0][1]*pin[13][1])));
        IkWpk[13][13][1] = ((ik[8][1][2]*pin[13][2])+((ik[8][1][0]*pin[13][0])+(
          ik[8][1][1]*pin[13][1])));
        IkWpk[13][13][2] = ((ik[8][2][2]*pin[13][2])+((ik[8][2][0]*pin[13][0])+(
          ik[8][2][1]*pin[13][1])));
        IkWpk[13][14][0] = ((ik[9][0][2]*Wpk[13][14][2])+((ik[9][0][0]*
          Wpk[13][14][0])+(ik[9][0][1]*Wpk[13][14][1])));
        IkWpk[13][14][1] = ((ik[9][1][2]*Wpk[13][14][2])+((ik[9][1][0]*
          Wpk[13][14][0])+(ik[9][1][1]*Wpk[13][14][1])));
        IkWpk[13][14][2] = ((ik[9][2][2]*Wpk[13][14][2])+((ik[9][2][0]*
          Wpk[13][14][0])+(ik[9][2][1]*Wpk[13][14][1])));
        IkWpk[14][14][0] = ((ik[9][0][2]*pin[14][2])+((ik[9][0][0]*pin[14][0])+(
          ik[9][0][1]*pin[14][1])));
        IkWpk[14][14][1] = ((ik[9][1][2]*pin[14][2])+((ik[9][1][0]*pin[14][0])+(
          ik[9][1][1]*pin[14][1])));
        IkWpk[14][14][2] = ((ik[9][2][2]*pin[14][2])+((ik[9][2][0]*pin[14][0])+(
          ik[9][2][1]*pin[14][1])));
        IkWpk[15][15][0] = ((ik[10][0][2]*pin[15][2])+((ik[10][0][0]*pin[15][0])
          +(ik[10][0][1]*pin[15][1])));
        IkWpk[15][15][1] = ((ik[10][1][2]*pin[15][2])+((ik[10][1][0]*pin[15][0])
          +(ik[10][1][1]*pin[15][1])));
        IkWpk[15][15][2] = ((ik[10][2][2]*pin[15][2])+((ik[10][2][0]*pin[15][0])
          +(ik[10][2][1]*pin[15][1])));
        IkWpk[15][16][0] = ((ik[11][0][2]*Wpk[15][16][2])+((ik[11][0][0]*
          Wpk[15][16][0])+(ik[11][0][1]*Wpk[15][16][1])));
        IkWpk[15][16][1] = ((ik[11][1][2]*Wpk[15][16][2])+((ik[11][1][0]*
          Wpk[15][16][0])+(ik[11][1][1]*Wpk[15][16][1])));
        IkWpk[15][16][2] = ((ik[11][2][2]*Wpk[15][16][2])+((ik[11][2][0]*
          Wpk[15][16][0])+(ik[11][2][1]*Wpk[15][16][1])));
        IkWpk[15][17][0] = ((ik[12][0][2]*Wpk[15][17][2])+((ik[12][0][0]*
          Wpk[15][17][0])+(ik[12][0][1]*Wpk[15][17][1])));
        IkWpk[15][17][1] = ((ik[12][1][2]*Wpk[15][17][2])+((ik[12][1][0]*
          Wpk[15][17][0])+(ik[12][1][1]*Wpk[15][17][1])));
        IkWpk[15][17][2] = ((ik[12][2][2]*Wpk[15][17][2])+((ik[12][2][0]*
          Wpk[15][17][0])+(ik[12][2][1]*Wpk[15][17][1])));
        IkWpk[15][18][0] = ((ik[13][0][2]*Wpk[15][18][2])+((ik[13][0][0]*
          Wpk[15][18][0])+(ik[13][0][1]*Wpk[15][18][1])));
        IkWpk[15][18][1] = ((ik[13][1][2]*Wpk[15][18][2])+((ik[13][1][0]*
          Wpk[15][18][0])+(ik[13][1][1]*Wpk[15][18][1])));
        IkWpk[15][18][2] = ((ik[13][2][2]*Wpk[15][18][2])+((ik[13][2][0]*
          Wpk[15][18][0])+(ik[13][2][1]*Wpk[15][18][1])));
        IkWpk[15][19][0] = ((ik[14][0][2]*Wpk[15][19][2])+((ik[14][0][0]*
          Wpk[15][19][0])+(ik[14][0][1]*Wpk[15][19][1])));
        IkWpk[15][19][1] = ((ik[14][1][2]*Wpk[15][19][2])+((ik[14][1][0]*
          Wpk[15][19][0])+(ik[14][1][1]*Wpk[15][19][1])));
        IkWpk[15][19][2] = ((ik[14][2][2]*Wpk[15][19][2])+((ik[14][2][0]*
          Wpk[15][19][0])+(ik[14][2][1]*Wpk[15][19][1])));
        IkWpk[15][20][0] = ((ik[15][0][2]*Wpk[15][20][2])+((ik[15][0][0]*
          Wpk[15][20][0])+(ik[15][0][1]*Wpk[15][20][1])));
        IkWpk[15][20][1] = ((ik[15][1][2]*Wpk[15][20][2])+((ik[15][1][0]*
          Wpk[15][20][0])+(ik[15][1][1]*Wpk[15][20][1])));
        IkWpk[15][20][2] = ((ik[15][2][2]*Wpk[15][20][2])+((ik[15][2][0]*
          Wpk[15][20][0])+(ik[15][2][1]*Wpk[15][20][1])));
        IkWpk[16][16][0] = ((ik[11][0][2]*pin[16][2])+((ik[11][0][0]*pin[16][0])
          +(ik[11][0][1]*pin[16][1])));
        IkWpk[16][16][1] = ((ik[11][1][2]*pin[16][2])+((ik[11][1][0]*pin[16][0])
          +(ik[11][1][1]*pin[16][1])));
        IkWpk[16][16][2] = ((ik[11][2][2]*pin[16][2])+((ik[11][2][0]*pin[16][0])
          +(ik[11][2][1]*pin[16][1])));
        IkWpk[16][17][0] = ((ik[12][0][2]*Wpk[16][17][2])+((ik[12][0][0]*
          Wpk[16][17][0])+(ik[12][0][1]*Wpk[16][17][1])));
        IkWpk[16][17][1] = ((ik[12][1][2]*Wpk[16][17][2])+((ik[12][1][0]*
          Wpk[16][17][0])+(ik[12][1][1]*Wpk[16][17][1])));
        IkWpk[16][17][2] = ((ik[12][2][2]*Wpk[16][17][2])+((ik[12][2][0]*
          Wpk[16][17][0])+(ik[12][2][1]*Wpk[16][17][1])));
        IkWpk[16][18][0] = ((ik[13][0][2]*Wpk[16][18][2])+((ik[13][0][0]*
          Wpk[16][18][0])+(ik[13][0][1]*Wpk[16][18][1])));
        IkWpk[16][18][1] = ((ik[13][1][2]*Wpk[16][18][2])+((ik[13][1][0]*
          Wpk[16][18][0])+(ik[13][1][1]*Wpk[16][18][1])));
        IkWpk[16][18][2] = ((ik[13][2][2]*Wpk[16][18][2])+((ik[13][2][0]*
          Wpk[16][18][0])+(ik[13][2][1]*Wpk[16][18][1])));
        IkWpk[16][19][0] = ((ik[14][0][2]*Wpk[16][19][2])+((ik[14][0][0]*
          Wpk[16][19][0])+(ik[14][0][1]*Wpk[16][19][1])));
        IkWpk[16][19][1] = ((ik[14][1][2]*Wpk[16][19][2])+((ik[14][1][0]*
          Wpk[16][19][0])+(ik[14][1][1]*Wpk[16][19][1])));
        IkWpk[16][19][2] = ((ik[14][2][2]*Wpk[16][19][2])+((ik[14][2][0]*
          Wpk[16][19][0])+(ik[14][2][1]*Wpk[16][19][1])));
        IkWpk[16][20][0] = ((ik[15][0][2]*Wpk[16][20][2])+((ik[15][0][0]*
          Wpk[16][20][0])+(ik[15][0][1]*Wpk[16][20][1])));
        IkWpk[16][20][1] = ((ik[15][1][2]*Wpk[16][20][2])+((ik[15][1][0]*
          Wpk[16][20][0])+(ik[15][1][1]*Wpk[16][20][1])));
        IkWpk[16][20][2] = ((ik[15][2][2]*Wpk[16][20][2])+((ik[15][2][0]*
          Wpk[16][20][0])+(ik[15][2][1]*Wpk[16][20][1])));
        IkWpk[17][17][0] = ((ik[12][0][2]*pin[17][2])+((ik[12][0][0]*pin[17][0])
          +(ik[12][0][1]*pin[17][1])));
        IkWpk[17][17][1] = ((ik[12][1][2]*pin[17][2])+((ik[12][1][0]*pin[17][0])
          +(ik[12][1][1]*pin[17][1])));
        IkWpk[17][17][2] = ((ik[12][2][2]*pin[17][2])+((ik[12][2][0]*pin[17][0])
          +(ik[12][2][1]*pin[17][1])));
        IkWpk[17][18][0] = ((ik[13][0][2]*Wpk[17][18][2])+((ik[13][0][0]*
          Wpk[17][18][0])+(ik[13][0][1]*Wpk[17][18][1])));
        IkWpk[17][18][1] = ((ik[13][1][2]*Wpk[17][18][2])+((ik[13][1][0]*
          Wpk[17][18][0])+(ik[13][1][1]*Wpk[17][18][1])));
        IkWpk[17][18][2] = ((ik[13][2][2]*Wpk[17][18][2])+((ik[13][2][0]*
          Wpk[17][18][0])+(ik[13][2][1]*Wpk[17][18][1])));
        IkWpk[17][19][0] = ((ik[14][0][2]*Wpk[17][19][2])+((ik[14][0][0]*
          Wpk[17][19][0])+(ik[14][0][1]*Wpk[17][19][1])));
        IkWpk[17][19][1] = ((ik[14][1][2]*Wpk[17][19][2])+((ik[14][1][0]*
          Wpk[17][19][0])+(ik[14][1][1]*Wpk[17][19][1])));
        IkWpk[17][19][2] = ((ik[14][2][2]*Wpk[17][19][2])+((ik[14][2][0]*
          Wpk[17][19][0])+(ik[14][2][1]*Wpk[17][19][1])));
        IkWpk[17][20][0] = ((ik[15][0][2]*Wpk[17][20][2])+((ik[15][0][0]*
          Wpk[17][20][0])+(ik[15][0][1]*Wpk[17][20][1])));
        IkWpk[17][20][1] = ((ik[15][1][2]*Wpk[17][20][2])+((ik[15][1][0]*
          Wpk[17][20][0])+(ik[15][1][1]*Wpk[17][20][1])));
        IkWpk[17][20][2] = ((ik[15][2][2]*Wpk[17][20][2])+((ik[15][2][0]*
          Wpk[17][20][0])+(ik[15][2][1]*Wpk[17][20][1])));
        IkWpk[18][18][0] = ((ik[13][0][2]*pin[18][2])+((ik[13][0][0]*pin[18][0])
          +(ik[13][0][1]*pin[18][1])));
        IkWpk[18][18][1] = ((ik[13][1][2]*pin[18][2])+((ik[13][1][0]*pin[18][0])
          +(ik[13][1][1]*pin[18][1])));
        IkWpk[18][18][2] = ((ik[13][2][2]*pin[18][2])+((ik[13][2][0]*pin[18][0])
          +(ik[13][2][1]*pin[18][1])));
        IkWpk[18][19][0] = ((ik[14][0][2]*Wpk[18][19][2])+((ik[14][0][0]*
          Wpk[18][19][0])+(ik[14][0][1]*Wpk[18][19][1])));
        IkWpk[18][19][1] = ((ik[14][1][2]*Wpk[18][19][2])+((ik[14][1][0]*
          Wpk[18][19][0])+(ik[14][1][1]*Wpk[18][19][1])));
        IkWpk[18][19][2] = ((ik[14][2][2]*Wpk[18][19][2])+((ik[14][2][0]*
          Wpk[18][19][0])+(ik[14][2][1]*Wpk[18][19][1])));
        IkWpk[18][20][0] = ((ik[15][0][2]*Wpk[18][20][2])+((ik[15][0][0]*
          Wpk[18][20][0])+(ik[15][0][1]*Wpk[18][20][1])));
        IkWpk[18][20][1] = ((ik[15][1][2]*Wpk[18][20][2])+((ik[15][1][0]*
          Wpk[18][20][0])+(ik[15][1][1]*Wpk[18][20][1])));
        IkWpk[18][20][2] = ((ik[15][2][2]*Wpk[18][20][2])+((ik[15][2][0]*
          Wpk[18][20][0])+(ik[15][2][1]*Wpk[18][20][1])));
        IkWpk[19][19][0] = ((ik[14][0][2]*pin[19][2])+((ik[14][0][0]*pin[19][0])
          +(ik[14][0][1]*pin[19][1])));
        IkWpk[19][19][1] = ((ik[14][1][2]*pin[19][2])+((ik[14][1][0]*pin[19][0])
          +(ik[14][1][1]*pin[19][1])));
        IkWpk[19][19][2] = ((ik[14][2][2]*pin[19][2])+((ik[14][2][0]*pin[19][0])
          +(ik[14][2][1]*pin[19][1])));
        IkWpk[19][20][0] = ((ik[15][0][2]*Wpk[19][20][2])+((ik[15][0][0]*
          Wpk[19][20][0])+(ik[15][0][1]*Wpk[19][20][1])));
        IkWpk[19][20][1] = ((ik[15][1][2]*Wpk[19][20][2])+((ik[15][1][0]*
          Wpk[19][20][0])+(ik[15][1][1]*Wpk[19][20][1])));
        IkWpk[19][20][2] = ((ik[15][2][2]*Wpk[19][20][2])+((ik[15][2][0]*
          Wpk[19][20][0])+(ik[15][2][1]*Wpk[19][20][1])));
        IkWpk[20][20][0] = ((ik[15][0][2]*pin[20][2])+((ik[15][0][0]*pin[20][0])
          +(ik[15][0][1]*pin[20][1])));
        IkWpk[20][20][1] = ((ik[15][1][2]*pin[20][2])+((ik[15][1][0]*pin[20][0])
          +(ik[15][1][1]*pin[20][1])));
        IkWpk[20][20][2] = ((ik[15][2][2]*pin[20][2])+((ik[15][2][0]*pin[20][0])
          +(ik[15][2][1]*pin[20][1])));
        temp[0] = ((mk[2]*((Vpk[0][7][2]*Vpk[0][7][2])+((Vpk[0][7][0]*
          Vpk[0][7][0])+(Vpk[0][7][1]*Vpk[0][7][1]))))+((mk[0]*((Vpk[0][3][2]*
          Vpk[0][3][2])+((Vpk[0][3][0]*Vpk[0][3][0])+(Vpk[0][3][1]*Vpk[0][3][1])
          )))+(mk[1]*((Vpk[0][6][2]*Vpk[0][6][2])+((Vpk[0][6][0]*Vpk[0][6][0])+(
          Vpk[0][6][1]*Vpk[0][6][1]))))));
        temp[1] = ((mk[5]*((Vpk[0][10][2]*Vpk[0][10][2])+((Vpk[0][10][0]*
          Vpk[0][10][0])+(Vpk[0][10][1]*Vpk[0][10][1]))))+((mk[4]*((Vpk[0][9][2]
          *Vpk[0][9][2])+((Vpk[0][9][0]*Vpk[0][9][0])+(Vpk[0][9][1]*Vpk[0][9][1]
          ))))+((mk[3]*((Vpk[0][8][2]*Vpk[0][8][2])+((Vpk[0][8][0]*Vpk[0][8][0])
          +(Vpk[0][8][1]*Vpk[0][8][1]))))+temp[0])));
        temp[2] = ((mk[8]*((Vpk[0][13][2]*Vpk[0][13][2])+((Vpk[0][13][0]*
          Vpk[0][13][0])+(Vpk[0][13][1]*Vpk[0][13][1]))))+((mk[7]*((
          Vpk[0][12][2]*Vpk[0][12][2])+((Vpk[0][12][0]*Vpk[0][12][0])+(
          Vpk[0][12][1]*Vpk[0][12][1]))))+((mk[6]*((Vpk[0][11][2]*Vpk[0][11][2])
          +((Vpk[0][11][0]*Vpk[0][11][0])+(Vpk[0][11][1]*Vpk[0][11][1]))))+
          temp[1])));
        temp[3] = ((mk[11]*((Vpk[0][16][2]*Vpk[0][16][2])+((Vpk[0][16][0]*
          Vpk[0][16][0])+(Vpk[0][16][1]*Vpk[0][16][1]))))+((mk[10]*((
          Vpk[0][15][2]*Vpk[0][15][2])+((Vpk[0][15][0]*Vpk[0][15][0])+(
          Vpk[0][15][1]*Vpk[0][15][1]))))+((mk[9]*((Vpk[0][14][2]*Vpk[0][14][2])
          +((Vpk[0][14][0]*Vpk[0][14][0])+(Vpk[0][14][1]*Vpk[0][14][1]))))+
          temp[2])));
        temp[4] = ((mk[14]*((Vpk[0][19][2]*Vpk[0][19][2])+((Vpk[0][19][0]*
          Vpk[0][19][0])+(Vpk[0][19][1]*Vpk[0][19][1]))))+((mk[13]*((
          Vpk[0][18][2]*Vpk[0][18][2])+((Vpk[0][18][0]*Vpk[0][18][0])+(
          Vpk[0][18][1]*Vpk[0][18][1]))))+((mk[12]*((Vpk[0][17][2]*Vpk[0][17][2]
          )+((Vpk[0][17][0]*Vpk[0][17][0])+(Vpk[0][17][1]*Vpk[0][17][1]))))+
          temp[3])));
        mm[0][0] = ((mk[15]*((Vpk[0][20][2]*Vpk[0][20][2])+((Vpk[0][20][0]*
          Vpk[0][20][0])+(Vpk[0][20][1]*Vpk[0][20][1]))))+temp[4]);
        temp[0] = ((mk[2]*((Vpk[0][7][2]*Vpk[1][7][2])+((Vpk[0][7][0]*
          Vpk[1][7][0])+(Vpk[0][7][1]*Vpk[1][7][1]))))+((mk[0]*((Vpk[0][3][2]*
          Vpk[1][3][2])+((Vpk[0][3][0]*Vpk[1][3][0])+(Vpk[0][3][1]*Vpk[1][3][1])
          )))+(mk[1]*((Vpk[0][6][2]*Vpk[1][6][2])+((Vpk[0][6][0]*Vpk[1][6][0])+(
          Vpk[0][6][1]*Vpk[1][6][1]))))));
        temp[1] = ((mk[5]*((Vpk[0][10][2]*Vpk[1][10][2])+((Vpk[0][10][0]*
          Vpk[1][10][0])+(Vpk[0][10][1]*Vpk[1][10][1]))))+((mk[4]*((Vpk[0][9][2]
          *Vpk[1][9][2])+((Vpk[0][9][0]*Vpk[1][9][0])+(Vpk[0][9][1]*Vpk[1][9][1]
          ))))+((mk[3]*((Vpk[0][8][2]*Vpk[1][8][2])+((Vpk[0][8][0]*Vpk[1][8][0])
          +(Vpk[0][8][1]*Vpk[1][8][1]))))+temp[0])));
        temp[2] = ((mk[8]*((Vpk[0][13][2]*Vpk[1][13][2])+((Vpk[0][13][0]*
          Vpk[1][13][0])+(Vpk[0][13][1]*Vpk[1][13][1]))))+((mk[7]*((
          Vpk[0][12][2]*Vpk[1][12][2])+((Vpk[0][12][0]*Vpk[1][12][0])+(
          Vpk[0][12][1]*Vpk[1][12][1]))))+((mk[6]*((Vpk[0][11][2]*Vpk[1][11][2])
          +((Vpk[0][11][0]*Vpk[1][11][0])+(Vpk[0][11][1]*Vpk[1][11][1]))))+
          temp[1])));
        temp[3] = ((mk[11]*((Vpk[0][16][2]*Vpk[1][16][2])+((Vpk[0][16][0]*
          Vpk[1][16][0])+(Vpk[0][16][1]*Vpk[1][16][1]))))+((mk[10]*((
          Vpk[0][15][2]*Vpk[1][15][2])+((Vpk[0][15][0]*Vpk[1][15][0])+(
          Vpk[0][15][1]*Vpk[1][15][1]))))+((mk[9]*((Vpk[0][14][2]*Vpk[1][14][2])
          +((Vpk[0][14][0]*Vpk[1][14][0])+(Vpk[0][14][1]*Vpk[1][14][1]))))+
          temp[2])));
        temp[4] = ((mk[14]*((Vpk[0][19][2]*Vpk[1][19][2])+((Vpk[0][19][0]*
          Vpk[1][19][0])+(Vpk[0][19][1]*Vpk[1][19][1]))))+((mk[13]*((
          Vpk[0][18][2]*Vpk[1][18][2])+((Vpk[0][18][0]*Vpk[1][18][0])+(
          Vpk[0][18][1]*Vpk[1][18][1]))))+((mk[12]*((Vpk[0][17][2]*Vpk[1][17][2]
          )+((Vpk[0][17][0]*Vpk[1][17][0])+(Vpk[0][17][1]*Vpk[1][17][1]))))+
          temp[3])));
        mm[0][1] = ((mk[15]*((Vpk[0][20][2]*Vpk[1][20][2])+((Vpk[0][20][0]*
          Vpk[1][20][0])+(Vpk[0][20][1]*Vpk[1][20][1]))))+temp[4]);
        temp[0] = ((mk[2]*((Vpk[0][7][2]*Vpk[2][7][2])+((Vpk[0][7][0]*
          Vpk[2][7][0])+(Vpk[0][7][1]*Vpk[2][7][1]))))+((mk[0]*((Vpk[0][3][2]*
          Vpk[2][3][2])+((Vpk[0][3][0]*Vpk[2][3][0])+(Vpk[0][3][1]*Vpk[2][3][1])
          )))+(mk[1]*((Vpk[0][6][2]*Vpk[2][6][2])+((Vpk[0][6][0]*Vpk[2][6][0])+(
          Vpk[0][6][1]*Vpk[2][6][1]))))));
        temp[1] = ((mk[5]*((Vpk[0][10][2]*Vpk[2][10][2])+((Vpk[0][10][0]*
          Vpk[2][10][0])+(Vpk[0][10][1]*Vpk[2][10][1]))))+((mk[4]*((Vpk[0][9][2]
          *Vpk[2][9][2])+((Vpk[0][9][0]*Vpk[2][9][0])+(Vpk[0][9][1]*Vpk[2][9][1]
          ))))+((mk[3]*((Vpk[0][8][2]*Vpk[2][8][2])+((Vpk[0][8][0]*Vpk[2][8][0])
          +(Vpk[0][8][1]*Vpk[2][8][1]))))+temp[0])));
        temp[2] = ((mk[8]*((Vpk[0][13][2]*Vpk[2][13][2])+((Vpk[0][13][0]*
          Vpk[2][13][0])+(Vpk[0][13][1]*Vpk[2][13][1]))))+((mk[7]*((
          Vpk[0][12][2]*Vpk[2][12][2])+((Vpk[0][12][0]*Vpk[2][12][0])+(
          Vpk[0][12][1]*Vpk[2][12][1]))))+((mk[6]*((Vpk[0][11][2]*Vpk[2][11][2])
          +((Vpk[0][11][0]*Vpk[2][11][0])+(Vpk[0][11][1]*Vpk[2][11][1]))))+
          temp[1])));
        temp[3] = ((mk[11]*((Vpk[0][16][2]*Vpk[2][16][2])+((Vpk[0][16][0]*
          Vpk[2][16][0])+(Vpk[0][16][1]*Vpk[2][16][1]))))+((mk[10]*((
          Vpk[0][15][2]*Vpk[2][15][2])+((Vpk[0][15][0]*Vpk[2][15][0])+(
          Vpk[0][15][1]*Vpk[2][15][1]))))+((mk[9]*((Vpk[0][14][2]*Vpk[2][14][2])
          +((Vpk[0][14][0]*Vpk[2][14][0])+(Vpk[0][14][1]*Vpk[2][14][1]))))+
          temp[2])));
        temp[4] = ((mk[14]*((Vpk[0][19][2]*Vpk[2][19][2])+((Vpk[0][19][0]*
          Vpk[2][19][0])+(Vpk[0][19][1]*Vpk[2][19][1]))))+((mk[13]*((
          Vpk[0][18][2]*Vpk[2][18][2])+((Vpk[0][18][0]*Vpk[2][18][0])+(
          Vpk[0][18][1]*Vpk[2][18][1]))))+((mk[12]*((Vpk[0][17][2]*Vpk[2][17][2]
          )+((Vpk[0][17][0]*Vpk[2][17][0])+(Vpk[0][17][1]*Vpk[2][17][1]))))+
          temp[3])));
        mm[0][2] = ((mk[15]*((Vpk[0][20][2]*Vpk[2][20][2])+((Vpk[0][20][0]*
          Vpk[2][20][0])+(Vpk[0][20][1]*Vpk[2][20][1]))))+temp[4]);
        temp[0] = ((mk[3]*((Vpk[0][8][2]*Vpk[3][8][2])+((Vpk[0][8][0]*
          Vpk[3][8][0])+(Vpk[0][8][1]*Vpk[3][8][1]))))+((mk[2]*((Vpk[0][7][2]*
          Vpk[3][7][2])+((Vpk[0][7][0]*Vpk[3][7][0])+(Vpk[0][7][1]*Vpk[3][7][1])
          )))+((mk[0]*((rk[0][2]*Vpk[0][3][1])-(rk[0][1]*Vpk[0][3][2])))+(mk[1]*
          ((Vpk[0][6][2]*Vpk[3][6][2])+((Vpk[0][6][0]*Vpk[3][6][0])+(
          Vpk[0][6][1]*Vpk[3][6][1])))))));
        temp[1] = ((mk[6]*((Vpk[0][11][2]*Vpk[3][11][2])+((Vpk[0][11][0]*
          Vpk[3][11][0])+(Vpk[0][11][1]*Vpk[3][11][1]))))+((mk[5]*((
          Vpk[0][10][2]*Vpk[3][10][2])+((Vpk[0][10][0]*Vpk[3][10][0])+(
          Vpk[0][10][1]*Vpk[3][10][1]))))+((mk[4]*((Vpk[0][9][2]*Vpk[3][9][2])+(
          (Vpk[0][9][0]*Vpk[3][9][0])+(Vpk[0][9][1]*Vpk[3][9][1]))))+temp[0])));
        temp[2] = ((mk[9]*((Vpk[0][14][2]*Vpk[3][14][2])+((Vpk[0][14][0]*
          Vpk[3][14][0])+(Vpk[0][14][1]*Vpk[3][14][1]))))+((mk[8]*((
          Vpk[0][13][2]*Vpk[3][13][2])+((Vpk[0][13][0]*Vpk[3][13][0])+(
          Vpk[0][13][1]*Vpk[3][13][1]))))+((mk[7]*((Vpk[0][12][2]*Vpk[3][12][2])
          +((Vpk[0][12][0]*Vpk[3][12][0])+(Vpk[0][12][1]*Vpk[3][12][1]))))+
          temp[1])));
        temp[3] = ((mk[12]*((Vpk[0][17][2]*Vpk[3][17][2])+((Vpk[0][17][0]*
          Vpk[3][17][0])+(Vpk[0][17][1]*Vpk[3][17][1]))))+((mk[11]*((
          Vpk[0][16][2]*Vpk[3][16][2])+((Vpk[0][16][0]*Vpk[3][16][0])+(
          Vpk[0][16][1]*Vpk[3][16][1]))))+((mk[10]*((Vpk[0][15][2]*Vpk[3][15][2]
          )+((Vpk[0][15][0]*Vpk[3][15][0])+(Vpk[0][15][1]*Vpk[3][15][1]))))+
          temp[2])));
        mm[0][3] = ((mk[15]*((Vpk[0][20][2]*Vpk[3][20][2])+((Vpk[0][20][0]*
          Vpk[3][20][0])+(Vpk[0][20][1]*Vpk[3][20][1]))))+((mk[14]*((
          Vpk[0][19][2]*Vpk[3][19][2])+((Vpk[0][19][0]*Vpk[3][19][0])+(
          Vpk[0][19][1]*Vpk[3][19][1]))))+((mk[13]*((Vpk[0][18][2]*Vpk[3][18][2]
          )+((Vpk[0][18][0]*Vpk[3][18][0])+(Vpk[0][18][1]*Vpk[3][18][1]))))+
          temp[3])));
        temp[0] = ((mk[3]*((Vpk[0][8][2]*Vpk[4][8][2])+((Vpk[0][8][0]*
          Vpk[4][8][0])+(Vpk[0][8][1]*Vpk[4][8][1]))))+((mk[2]*((Vpk[0][7][2]*
          Vpk[4][7][2])+((Vpk[0][7][0]*Vpk[4][7][0])+(Vpk[0][7][1]*Vpk[4][7][1])
          )))+((mk[0]*((rk[0][0]*Vpk[0][3][2])-(rk[0][2]*Vpk[0][3][0])))+(mk[1]*
          ((Vpk[0][6][2]*Vpk[4][6][2])+((Vpk[0][6][0]*Vpk[4][6][0])+(
          Vpk[0][6][1]*Vpk[4][6][1])))))));
        temp[1] = ((mk[6]*((Vpk[0][11][2]*Vpk[4][11][2])+((Vpk[0][11][0]*
          Vpk[4][11][0])+(Vpk[0][11][1]*Vpk[4][11][1]))))+((mk[5]*((
          Vpk[0][10][2]*Vpk[4][10][2])+((Vpk[0][10][0]*Vpk[4][10][0])+(
          Vpk[0][10][1]*Vpk[4][10][1]))))+((mk[4]*((Vpk[0][9][2]*Vpk[4][9][2])+(
          (Vpk[0][9][0]*Vpk[4][9][0])+(Vpk[0][9][1]*Vpk[4][9][1]))))+temp[0])));
        temp[2] = ((mk[9]*((Vpk[0][14][2]*Vpk[4][14][2])+((Vpk[0][14][0]*
          Vpk[4][14][0])+(Vpk[0][14][1]*Vpk[4][14][1]))))+((mk[8]*((
          Vpk[0][13][2]*Vpk[4][13][2])+((Vpk[0][13][0]*Vpk[4][13][0])+(
          Vpk[0][13][1]*Vpk[4][13][1]))))+((mk[7]*((Vpk[0][12][2]*Vpk[4][12][2])
          +((Vpk[0][12][0]*Vpk[4][12][0])+(Vpk[0][12][1]*Vpk[4][12][1]))))+
          temp[1])));
        temp[3] = ((mk[12]*((Vpk[0][17][2]*Vpk[4][17][2])+((Vpk[0][17][0]*
          Vpk[4][17][0])+(Vpk[0][17][1]*Vpk[4][17][1]))))+((mk[11]*((
          Vpk[0][16][2]*Vpk[4][16][2])+((Vpk[0][16][0]*Vpk[4][16][0])+(
          Vpk[0][16][1]*Vpk[4][16][1]))))+((mk[10]*((Vpk[0][15][2]*Vpk[4][15][2]
          )+((Vpk[0][15][0]*Vpk[4][15][0])+(Vpk[0][15][1]*Vpk[4][15][1]))))+
          temp[2])));
        mm[0][4] = ((mk[15]*((Vpk[0][20][2]*Vpk[4][20][2])+((Vpk[0][20][0]*
          Vpk[4][20][0])+(Vpk[0][20][1]*Vpk[4][20][1]))))+((mk[14]*((
          Vpk[0][19][2]*Vpk[4][19][2])+((Vpk[0][19][0]*Vpk[4][19][0])+(
          Vpk[0][19][1]*Vpk[4][19][1]))))+((mk[13]*((Vpk[0][18][2]*Vpk[4][18][2]
          )+((Vpk[0][18][0]*Vpk[4][18][0])+(Vpk[0][18][1]*Vpk[4][18][1]))))+
          temp[3])));
        temp[0] = ((mk[3]*((Vpk[0][8][2]*Vpk[5][8][2])+((Vpk[0][8][0]*
          Vpk[5][8][0])+(Vpk[0][8][1]*Vpk[5][8][1]))))+((mk[2]*((Vpk[0][7][2]*
          Vpk[5][7][2])+((Vpk[0][7][0]*Vpk[5][7][0])+(Vpk[0][7][1]*Vpk[5][7][1])
          )))+((mk[0]*((rk[0][1]*Vpk[0][3][0])-(rk[0][0]*Vpk[0][3][1])))+(mk[1]*
          ((Vpk[0][6][2]*Vpk[5][6][2])+((Vpk[0][6][0]*Vpk[5][6][0])+(
          Vpk[0][6][1]*Vpk[5][6][1])))))));
        temp[1] = ((mk[6]*((Vpk[0][11][2]*Vpk[5][11][2])+((Vpk[0][11][0]*
          Vpk[5][11][0])+(Vpk[0][11][1]*Vpk[5][11][1]))))+((mk[5]*((
          Vpk[0][10][2]*Vpk[5][10][2])+((Vpk[0][10][0]*Vpk[5][10][0])+(
          Vpk[0][10][1]*Vpk[5][10][1]))))+((mk[4]*((Vpk[0][9][2]*Vpk[5][9][2])+(
          (Vpk[0][9][0]*Vpk[5][9][0])+(Vpk[0][9][1]*Vpk[5][9][1]))))+temp[0])));
        temp[2] = ((mk[9]*((Vpk[0][14][2]*Vpk[5][14][2])+((Vpk[0][14][0]*
          Vpk[5][14][0])+(Vpk[0][14][1]*Vpk[5][14][1]))))+((mk[8]*((
          Vpk[0][13][2]*Vpk[5][13][2])+((Vpk[0][13][0]*Vpk[5][13][0])+(
          Vpk[0][13][1]*Vpk[5][13][1]))))+((mk[7]*((Vpk[0][12][2]*Vpk[5][12][2])
          +((Vpk[0][12][0]*Vpk[5][12][0])+(Vpk[0][12][1]*Vpk[5][12][1]))))+
          temp[1])));
        temp[3] = ((mk[12]*((Vpk[0][17][2]*Vpk[5][17][2])+((Vpk[0][17][0]*
          Vpk[5][17][0])+(Vpk[0][17][1]*Vpk[5][17][1]))))+((mk[11]*((
          Vpk[0][16][2]*Vpk[5][16][2])+((Vpk[0][16][0]*Vpk[5][16][0])+(
          Vpk[0][16][1]*Vpk[5][16][1]))))+((mk[10]*((Vpk[0][15][2]*Vpk[5][15][2]
          )+((Vpk[0][15][0]*Vpk[5][15][0])+(Vpk[0][15][1]*Vpk[5][15][1]))))+
          temp[2])));
        mm[0][5] = ((mk[15]*((Vpk[0][20][2]*Vpk[5][20][2])+((Vpk[0][20][0]*
          Vpk[5][20][0])+(Vpk[0][20][1]*Vpk[5][20][1]))))+((mk[14]*((
          Vpk[0][19][2]*Vpk[5][19][2])+((Vpk[0][19][0]*Vpk[5][19][0])+(
          Vpk[0][19][1]*Vpk[5][19][1]))))+((mk[13]*((Vpk[0][18][2]*Vpk[5][18][2]
          )+((Vpk[0][18][0]*Vpk[5][18][0])+(Vpk[0][18][1]*Vpk[5][18][1]))))+
          temp[3])));
        mm[0][6] = ((mk[3]*((Vpk[0][8][2]*Vpk[6][8][2])+((Vpk[0][8][0]*
          Vpk[6][8][0])+(Vpk[0][8][1]*Vpk[6][8][1]))))+((mk[1]*((Vpk[0][6][2]*
          Vpk[6][6][2])+((Vpk[0][6][0]*Vpk[6][6][0])+(Vpk[0][6][1]*Vpk[6][6][1])
          )))+(mk[2]*((Vpk[0][7][2]*Vpk[6][7][2])+((Vpk[0][7][0]*Vpk[6][7][0])+(
          Vpk[0][7][1]*Vpk[6][7][1]))))));
        mm[0][7] = ((mk[2]*((Vpk[0][7][2]*Vpk[7][7][2])+((Vpk[0][7][0]*
          Vpk[7][7][0])+(Vpk[0][7][1]*Vpk[7][7][1]))))+(mk[3]*((Vpk[0][8][2]*
          Vpk[7][8][2])+((Vpk[0][8][0]*Vpk[7][8][0])+(Vpk[0][8][1]*Vpk[7][8][1])
          ))));
        mm[0][8] = (mk[3]*((Vpk[0][8][2]*Vpk[8][8][2])+((Vpk[0][8][0]*
          Vpk[8][8][0])+(Vpk[0][8][1]*Vpk[8][8][1]))));
        temp[0] = ((mk[6]*((Vpk[0][11][2]*Vpk[9][11][2])+((Vpk[0][11][0]*
          Vpk[9][11][0])+(Vpk[0][11][1]*Vpk[9][11][1]))))+((mk[4]*((Vpk[0][9][2]
          *Vpk[9][9][2])+((Vpk[0][9][0]*Vpk[9][9][0])+(Vpk[0][9][1]*Vpk[9][9][1]
          ))))+(mk[5]*((Vpk[0][10][2]*Vpk[9][10][2])+((Vpk[0][10][0]*
          Vpk[9][10][0])+(Vpk[0][10][1]*Vpk[9][10][1]))))));
        mm[0][9] = ((mk[9]*((Vpk[0][14][2]*Vpk[9][14][2])+((Vpk[0][14][0]*
          Vpk[9][14][0])+(Vpk[0][14][1]*Vpk[9][14][1]))))+((mk[8]*((
          Vpk[0][13][2]*Vpk[9][13][2])+((Vpk[0][13][0]*Vpk[9][13][0])+(
          Vpk[0][13][1]*Vpk[9][13][1]))))+((mk[7]*((Vpk[0][12][2]*Vpk[9][12][2])
          +((Vpk[0][12][0]*Vpk[9][12][0])+(Vpk[0][12][1]*Vpk[9][12][1]))))+
          temp[0])));
        temp[0] = ((mk[7]*((Vpk[0][12][2]*Vpk[10][12][2])+((Vpk[0][12][0]*
          Vpk[10][12][0])+(Vpk[0][12][1]*Vpk[10][12][1]))))+((mk[5]*((
          Vpk[0][10][2]*Vpk[10][10][2])+((Vpk[0][10][0]*Vpk[10][10][0])+(
          Vpk[0][10][1]*Vpk[10][10][1]))))+(mk[6]*((Vpk[0][11][2]*Vpk[10][11][2]
          )+((Vpk[0][11][0]*Vpk[10][11][0])+(Vpk[0][11][1]*Vpk[10][11][1]))))));
        mm[0][10] = ((mk[9]*((Vpk[0][14][2]*Vpk[10][14][2])+((Vpk[0][14][0]*
          Vpk[10][14][0])+(Vpk[0][14][1]*Vpk[10][14][1]))))+((mk[8]*((
          Vpk[0][13][2]*Vpk[10][13][2])+((Vpk[0][13][0]*Vpk[10][13][0])+(
          Vpk[0][13][1]*Vpk[10][13][1]))))+temp[0]));
        temp[0] = ((mk[8]*((Vpk[0][13][2]*Vpk[11][13][2])+((Vpk[0][13][0]*
          Vpk[11][13][0])+(Vpk[0][13][1]*Vpk[11][13][1]))))+((mk[6]*((
          Vpk[0][11][2]*Vpk[11][11][2])+((Vpk[0][11][0]*Vpk[11][11][0])+(
          Vpk[0][11][1]*Vpk[11][11][1]))))+(mk[7]*((Vpk[0][12][2]*Vpk[11][12][2]
          )+((Vpk[0][12][0]*Vpk[11][12][0])+(Vpk[0][12][1]*Vpk[11][12][1]))))));
        mm[0][11] = ((mk[9]*((Vpk[0][14][2]*Vpk[11][14][2])+((Vpk[0][14][0]*
          Vpk[11][14][0])+(Vpk[0][14][1]*Vpk[11][14][1]))))+temp[0]);
        mm[0][12] = ((mk[9]*((Vpk[0][14][2]*Vpk[12][14][2])+((Vpk[0][14][0]*
          Vpk[12][14][0])+(Vpk[0][14][1]*Vpk[12][14][1]))))+((mk[7]*((
          Vpk[0][12][2]*Vpk[12][12][2])+((Vpk[0][12][0]*Vpk[12][12][0])+(
          Vpk[0][12][1]*Vpk[12][12][1]))))+(mk[8]*((Vpk[0][13][2]*Vpk[12][13][2]
          )+((Vpk[0][13][0]*Vpk[12][13][0])+(Vpk[0][13][1]*Vpk[12][13][1]))))));
        mm[0][13] = ((mk[8]*((Vpk[0][13][2]*Vpk[13][13][2])+((Vpk[0][13][0]*
          Vpk[13][13][0])+(Vpk[0][13][1]*Vpk[13][13][1]))))+(mk[9]*((
          Vpk[0][14][2]*Vpk[13][14][2])+((Vpk[0][14][0]*Vpk[13][14][0])+(
          Vpk[0][14][1]*Vpk[13][14][1])))));
        mm[0][14] = (mk[9]*((Vpk[0][14][2]*Vpk[14][14][2])+((Vpk[0][14][0]*
          Vpk[14][14][0])+(Vpk[0][14][1]*Vpk[14][14][1]))));
        temp[0] = ((mk[12]*((Vpk[0][17][2]*Vpk[15][17][2])+((Vpk[0][17][0]*
          Vpk[15][17][0])+(Vpk[0][17][1]*Vpk[15][17][1]))))+((mk[10]*((
          Vpk[0][15][2]*Vpk[15][15][2])+((Vpk[0][15][0]*Vpk[15][15][0])+(
          Vpk[0][15][1]*Vpk[15][15][1]))))+(mk[11]*((Vpk[0][16][2]*
          Vpk[15][16][2])+((Vpk[0][16][0]*Vpk[15][16][0])+(Vpk[0][16][1]*
          Vpk[15][16][1]))))));
        mm[0][15] = ((mk[15]*((Vpk[0][20][2]*Vpk[15][20][2])+((Vpk[0][20][0]*
          Vpk[15][20][0])+(Vpk[0][20][1]*Vpk[15][20][1]))))+((mk[14]*((
          Vpk[0][19][2]*Vpk[15][19][2])+((Vpk[0][19][0]*Vpk[15][19][0])+(
          Vpk[0][19][1]*Vpk[15][19][1]))))+((mk[13]*((Vpk[0][18][2]*
          Vpk[15][18][2])+((Vpk[0][18][0]*Vpk[15][18][0])+(Vpk[0][18][1]*
          Vpk[15][18][1]))))+temp[0])));
        temp[0] = ((mk[13]*((Vpk[0][18][2]*Vpk[16][18][2])+((Vpk[0][18][0]*
          Vpk[16][18][0])+(Vpk[0][18][1]*Vpk[16][18][1]))))+((mk[11]*((
          Vpk[0][16][2]*Vpk[16][16][2])+((Vpk[0][16][0]*Vpk[16][16][0])+(
          Vpk[0][16][1]*Vpk[16][16][1]))))+(mk[12]*((Vpk[0][17][2]*
          Vpk[16][17][2])+((Vpk[0][17][0]*Vpk[16][17][0])+(Vpk[0][17][1]*
          Vpk[16][17][1]))))));
        mm[0][16] = ((mk[15]*((Vpk[0][20][2]*Vpk[16][20][2])+((Vpk[0][20][0]*
          Vpk[16][20][0])+(Vpk[0][20][1]*Vpk[16][20][1]))))+((mk[14]*((
          Vpk[0][19][2]*Vpk[16][19][2])+((Vpk[0][19][0]*Vpk[16][19][0])+(
          Vpk[0][19][1]*Vpk[16][19][1]))))+temp[0]));
        temp[0] = ((mk[14]*((Vpk[0][19][2]*Vpk[17][19][2])+((Vpk[0][19][0]*
          Vpk[17][19][0])+(Vpk[0][19][1]*Vpk[17][19][1]))))+((mk[12]*((
          Vpk[0][17][2]*Vpk[17][17][2])+((Vpk[0][17][0]*Vpk[17][17][0])+(
          Vpk[0][17][1]*Vpk[17][17][1]))))+(mk[13]*((Vpk[0][18][2]*
          Vpk[17][18][2])+((Vpk[0][18][0]*Vpk[17][18][0])+(Vpk[0][18][1]*
          Vpk[17][18][1]))))));
        mm[0][17] = ((mk[15]*((Vpk[0][20][2]*Vpk[17][20][2])+((Vpk[0][20][0]*
          Vpk[17][20][0])+(Vpk[0][20][1]*Vpk[17][20][1]))))+temp[0]);
        mm[0][18] = ((mk[15]*((Vpk[0][20][2]*Vpk[18][20][2])+((Vpk[0][20][0]*
          Vpk[18][20][0])+(Vpk[0][20][1]*Vpk[18][20][1]))))+((mk[13]*((
          Vpk[0][18][2]*Vpk[18][18][2])+((Vpk[0][18][0]*Vpk[18][18][0])+(
          Vpk[0][18][1]*Vpk[18][18][1]))))+(mk[14]*((Vpk[0][19][2]*
          Vpk[18][19][2])+((Vpk[0][19][0]*Vpk[18][19][0])+(Vpk[0][19][1]*
          Vpk[18][19][1]))))));
        mm[0][19] = ((mk[14]*((Vpk[0][19][2]*Vpk[19][19][2])+((Vpk[0][19][0]*
          Vpk[19][19][0])+(Vpk[0][19][1]*Vpk[19][19][1]))))+(mk[15]*((
          Vpk[0][20][2]*Vpk[19][20][2])+((Vpk[0][20][0]*Vpk[19][20][0])+(
          Vpk[0][20][1]*Vpk[19][20][1])))));
        mm[0][20] = (mk[15]*((Vpk[0][20][2]*Vpk[20][20][2])+((Vpk[0][20][0]*
          Vpk[20][20][0])+(Vpk[0][20][1]*Vpk[20][20][1]))));
        temp[0] = ((mk[2]*((Vpk[1][7][2]*Vpk[1][7][2])+((Vpk[1][7][0]*
          Vpk[1][7][0])+(Vpk[1][7][1]*Vpk[1][7][1]))))+((mk[0]*((Vpk[1][3][2]*
          Vpk[1][3][2])+((Vpk[1][3][0]*Vpk[1][3][0])+(Vpk[1][3][1]*Vpk[1][3][1])
          )))+(mk[1]*((Vpk[1][6][2]*Vpk[1][6][2])+((Vpk[1][6][0]*Vpk[1][6][0])+(
          Vpk[1][6][1]*Vpk[1][6][1]))))));
        temp[1] = ((mk[5]*((Vpk[1][10][2]*Vpk[1][10][2])+((Vpk[1][10][0]*
          Vpk[1][10][0])+(Vpk[1][10][1]*Vpk[1][10][1]))))+((mk[4]*((Vpk[1][9][2]
          *Vpk[1][9][2])+((Vpk[1][9][0]*Vpk[1][9][0])+(Vpk[1][9][1]*Vpk[1][9][1]
          ))))+((mk[3]*((Vpk[1][8][2]*Vpk[1][8][2])+((Vpk[1][8][0]*Vpk[1][8][0])
          +(Vpk[1][8][1]*Vpk[1][8][1]))))+temp[0])));
        temp[2] = ((mk[8]*((Vpk[1][13][2]*Vpk[1][13][2])+((Vpk[1][13][0]*
          Vpk[1][13][0])+(Vpk[1][13][1]*Vpk[1][13][1]))))+((mk[7]*((
          Vpk[1][12][2]*Vpk[1][12][2])+((Vpk[1][12][0]*Vpk[1][12][0])+(
          Vpk[1][12][1]*Vpk[1][12][1]))))+((mk[6]*((Vpk[1][11][2]*Vpk[1][11][2])
          +((Vpk[1][11][0]*Vpk[1][11][0])+(Vpk[1][11][1]*Vpk[1][11][1]))))+
          temp[1])));
        temp[3] = ((mk[11]*((Vpk[1][16][2]*Vpk[1][16][2])+((Vpk[1][16][0]*
          Vpk[1][16][0])+(Vpk[1][16][1]*Vpk[1][16][1]))))+((mk[10]*((
          Vpk[1][15][2]*Vpk[1][15][2])+((Vpk[1][15][0]*Vpk[1][15][0])+(
          Vpk[1][15][1]*Vpk[1][15][1]))))+((mk[9]*((Vpk[1][14][2]*Vpk[1][14][2])
          +((Vpk[1][14][0]*Vpk[1][14][0])+(Vpk[1][14][1]*Vpk[1][14][1]))))+
          temp[2])));
        temp[4] = ((mk[14]*((Vpk[1][19][2]*Vpk[1][19][2])+((Vpk[1][19][0]*
          Vpk[1][19][0])+(Vpk[1][19][1]*Vpk[1][19][1]))))+((mk[13]*((
          Vpk[1][18][2]*Vpk[1][18][2])+((Vpk[1][18][0]*Vpk[1][18][0])+(
          Vpk[1][18][1]*Vpk[1][18][1]))))+((mk[12]*((Vpk[1][17][2]*Vpk[1][17][2]
          )+((Vpk[1][17][0]*Vpk[1][17][0])+(Vpk[1][17][1]*Vpk[1][17][1]))))+
          temp[3])));
        mm[1][1] = ((mk[15]*((Vpk[1][20][2]*Vpk[1][20][2])+((Vpk[1][20][0]*
          Vpk[1][20][0])+(Vpk[1][20][1]*Vpk[1][20][1]))))+temp[4]);
        temp[0] = ((mk[2]*((Vpk[1][7][2]*Vpk[2][7][2])+((Vpk[1][7][0]*
          Vpk[2][7][0])+(Vpk[1][7][1]*Vpk[2][7][1]))))+((mk[0]*((Vpk[1][3][2]*
          Vpk[2][3][2])+((Vpk[1][3][0]*Vpk[2][3][0])+(Vpk[1][3][1]*Vpk[2][3][1])
          )))+(mk[1]*((Vpk[1][6][2]*Vpk[2][6][2])+((Vpk[1][6][0]*Vpk[2][6][0])+(
          Vpk[1][6][1]*Vpk[2][6][1]))))));
        temp[1] = ((mk[5]*((Vpk[1][10][2]*Vpk[2][10][2])+((Vpk[1][10][0]*
          Vpk[2][10][0])+(Vpk[1][10][1]*Vpk[2][10][1]))))+((mk[4]*((Vpk[1][9][2]
          *Vpk[2][9][2])+((Vpk[1][9][0]*Vpk[2][9][0])+(Vpk[1][9][1]*Vpk[2][9][1]
          ))))+((mk[3]*((Vpk[1][8][2]*Vpk[2][8][2])+((Vpk[1][8][0]*Vpk[2][8][0])
          +(Vpk[1][8][1]*Vpk[2][8][1]))))+temp[0])));
        temp[2] = ((mk[8]*((Vpk[1][13][2]*Vpk[2][13][2])+((Vpk[1][13][0]*
          Vpk[2][13][0])+(Vpk[1][13][1]*Vpk[2][13][1]))))+((mk[7]*((
          Vpk[1][12][2]*Vpk[2][12][2])+((Vpk[1][12][0]*Vpk[2][12][0])+(
          Vpk[1][12][1]*Vpk[2][12][1]))))+((mk[6]*((Vpk[1][11][2]*Vpk[2][11][2])
          +((Vpk[1][11][0]*Vpk[2][11][0])+(Vpk[1][11][1]*Vpk[2][11][1]))))+
          temp[1])));
        temp[3] = ((mk[11]*((Vpk[1][16][2]*Vpk[2][16][2])+((Vpk[1][16][0]*
          Vpk[2][16][0])+(Vpk[1][16][1]*Vpk[2][16][1]))))+((mk[10]*((
          Vpk[1][15][2]*Vpk[2][15][2])+((Vpk[1][15][0]*Vpk[2][15][0])+(
          Vpk[1][15][1]*Vpk[2][15][1]))))+((mk[9]*((Vpk[1][14][2]*Vpk[2][14][2])
          +((Vpk[1][14][0]*Vpk[2][14][0])+(Vpk[1][14][1]*Vpk[2][14][1]))))+
          temp[2])));
        temp[4] = ((mk[14]*((Vpk[1][19][2]*Vpk[2][19][2])+((Vpk[1][19][0]*
          Vpk[2][19][0])+(Vpk[1][19][1]*Vpk[2][19][1]))))+((mk[13]*((
          Vpk[1][18][2]*Vpk[2][18][2])+((Vpk[1][18][0]*Vpk[2][18][0])+(
          Vpk[1][18][1]*Vpk[2][18][1]))))+((mk[12]*((Vpk[1][17][2]*Vpk[2][17][2]
          )+((Vpk[1][17][0]*Vpk[2][17][0])+(Vpk[1][17][1]*Vpk[2][17][1]))))+
          temp[3])));
        mm[1][2] = ((mk[15]*((Vpk[1][20][2]*Vpk[2][20][2])+((Vpk[1][20][0]*
          Vpk[2][20][0])+(Vpk[1][20][1]*Vpk[2][20][1]))))+temp[4]);
        temp[0] = ((mk[3]*((Vpk[1][8][2]*Vpk[3][8][2])+((Vpk[1][8][0]*
          Vpk[3][8][0])+(Vpk[1][8][1]*Vpk[3][8][1]))))+((mk[2]*((Vpk[1][7][2]*
          Vpk[3][7][2])+((Vpk[1][7][0]*Vpk[3][7][0])+(Vpk[1][7][1]*Vpk[3][7][1])
          )))+((mk[0]*((rk[0][2]*Vpk[1][3][1])-(rk[0][1]*Vpk[1][3][2])))+(mk[1]*
          ((Vpk[1][6][2]*Vpk[3][6][2])+((Vpk[1][6][0]*Vpk[3][6][0])+(
          Vpk[1][6][1]*Vpk[3][6][1])))))));
        temp[1] = ((mk[6]*((Vpk[1][11][2]*Vpk[3][11][2])+((Vpk[1][11][0]*
          Vpk[3][11][0])+(Vpk[1][11][1]*Vpk[3][11][1]))))+((mk[5]*((
          Vpk[1][10][2]*Vpk[3][10][2])+((Vpk[1][10][0]*Vpk[3][10][0])+(
          Vpk[1][10][1]*Vpk[3][10][1]))))+((mk[4]*((Vpk[1][9][2]*Vpk[3][9][2])+(
          (Vpk[1][9][0]*Vpk[3][9][0])+(Vpk[1][9][1]*Vpk[3][9][1]))))+temp[0])));
        temp[2] = ((mk[9]*((Vpk[1][14][2]*Vpk[3][14][2])+((Vpk[1][14][0]*
          Vpk[3][14][0])+(Vpk[1][14][1]*Vpk[3][14][1]))))+((mk[8]*((
          Vpk[1][13][2]*Vpk[3][13][2])+((Vpk[1][13][0]*Vpk[3][13][0])+(
          Vpk[1][13][1]*Vpk[3][13][1]))))+((mk[7]*((Vpk[1][12][2]*Vpk[3][12][2])
          +((Vpk[1][12][0]*Vpk[3][12][0])+(Vpk[1][12][1]*Vpk[3][12][1]))))+
          temp[1])));
        temp[3] = ((mk[12]*((Vpk[1][17][2]*Vpk[3][17][2])+((Vpk[1][17][0]*
          Vpk[3][17][0])+(Vpk[1][17][1]*Vpk[3][17][1]))))+((mk[11]*((
          Vpk[1][16][2]*Vpk[3][16][2])+((Vpk[1][16][0]*Vpk[3][16][0])+(
          Vpk[1][16][1]*Vpk[3][16][1]))))+((mk[10]*((Vpk[1][15][2]*Vpk[3][15][2]
          )+((Vpk[1][15][0]*Vpk[3][15][0])+(Vpk[1][15][1]*Vpk[3][15][1]))))+
          temp[2])));
        mm[1][3] = ((mk[15]*((Vpk[1][20][2]*Vpk[3][20][2])+((Vpk[1][20][0]*
          Vpk[3][20][0])+(Vpk[1][20][1]*Vpk[3][20][1]))))+((mk[14]*((
          Vpk[1][19][2]*Vpk[3][19][2])+((Vpk[1][19][0]*Vpk[3][19][0])+(
          Vpk[1][19][1]*Vpk[3][19][1]))))+((mk[13]*((Vpk[1][18][2]*Vpk[3][18][2]
          )+((Vpk[1][18][0]*Vpk[3][18][0])+(Vpk[1][18][1]*Vpk[3][18][1]))))+
          temp[3])));
        temp[0] = ((mk[3]*((Vpk[1][8][2]*Vpk[4][8][2])+((Vpk[1][8][0]*
          Vpk[4][8][0])+(Vpk[1][8][1]*Vpk[4][8][1]))))+((mk[2]*((Vpk[1][7][2]*
          Vpk[4][7][2])+((Vpk[1][7][0]*Vpk[4][7][0])+(Vpk[1][7][1]*Vpk[4][7][1])
          )))+((mk[0]*((rk[0][0]*Vpk[1][3][2])-(rk[0][2]*Vpk[1][3][0])))+(mk[1]*
          ((Vpk[1][6][2]*Vpk[4][6][2])+((Vpk[1][6][0]*Vpk[4][6][0])+(
          Vpk[1][6][1]*Vpk[4][6][1])))))));
        temp[1] = ((mk[6]*((Vpk[1][11][2]*Vpk[4][11][2])+((Vpk[1][11][0]*
          Vpk[4][11][0])+(Vpk[1][11][1]*Vpk[4][11][1]))))+((mk[5]*((
          Vpk[1][10][2]*Vpk[4][10][2])+((Vpk[1][10][0]*Vpk[4][10][0])+(
          Vpk[1][10][1]*Vpk[4][10][1]))))+((mk[4]*((Vpk[1][9][2]*Vpk[4][9][2])+(
          (Vpk[1][9][0]*Vpk[4][9][0])+(Vpk[1][9][1]*Vpk[4][9][1]))))+temp[0])));
        temp[2] = ((mk[9]*((Vpk[1][14][2]*Vpk[4][14][2])+((Vpk[1][14][0]*
          Vpk[4][14][0])+(Vpk[1][14][1]*Vpk[4][14][1]))))+((mk[8]*((
          Vpk[1][13][2]*Vpk[4][13][2])+((Vpk[1][13][0]*Vpk[4][13][0])+(
          Vpk[1][13][1]*Vpk[4][13][1]))))+((mk[7]*((Vpk[1][12][2]*Vpk[4][12][2])
          +((Vpk[1][12][0]*Vpk[4][12][0])+(Vpk[1][12][1]*Vpk[4][12][1]))))+
          temp[1])));
        temp[3] = ((mk[12]*((Vpk[1][17][2]*Vpk[4][17][2])+((Vpk[1][17][0]*
          Vpk[4][17][0])+(Vpk[1][17][1]*Vpk[4][17][1]))))+((mk[11]*((
          Vpk[1][16][2]*Vpk[4][16][2])+((Vpk[1][16][0]*Vpk[4][16][0])+(
          Vpk[1][16][1]*Vpk[4][16][1]))))+((mk[10]*((Vpk[1][15][2]*Vpk[4][15][2]
          )+((Vpk[1][15][0]*Vpk[4][15][0])+(Vpk[1][15][1]*Vpk[4][15][1]))))+
          temp[2])));
        mm[1][4] = ((mk[15]*((Vpk[1][20][2]*Vpk[4][20][2])+((Vpk[1][20][0]*
          Vpk[4][20][0])+(Vpk[1][20][1]*Vpk[4][20][1]))))+((mk[14]*((
          Vpk[1][19][2]*Vpk[4][19][2])+((Vpk[1][19][0]*Vpk[4][19][0])+(
          Vpk[1][19][1]*Vpk[4][19][1]))))+((mk[13]*((Vpk[1][18][2]*Vpk[4][18][2]
          )+((Vpk[1][18][0]*Vpk[4][18][0])+(Vpk[1][18][1]*Vpk[4][18][1]))))+
          temp[3])));
        temp[0] = ((mk[3]*((Vpk[1][8][2]*Vpk[5][8][2])+((Vpk[1][8][0]*
          Vpk[5][8][0])+(Vpk[1][8][1]*Vpk[5][8][1]))))+((mk[2]*((Vpk[1][7][2]*
          Vpk[5][7][2])+((Vpk[1][7][0]*Vpk[5][7][0])+(Vpk[1][7][1]*Vpk[5][7][1])
          )))+((mk[0]*((rk[0][1]*Vpk[1][3][0])-(rk[0][0]*Vpk[1][3][1])))+(mk[1]*
          ((Vpk[1][6][2]*Vpk[5][6][2])+((Vpk[1][6][0]*Vpk[5][6][0])+(
          Vpk[1][6][1]*Vpk[5][6][1])))))));
        temp[1] = ((mk[6]*((Vpk[1][11][2]*Vpk[5][11][2])+((Vpk[1][11][0]*
          Vpk[5][11][0])+(Vpk[1][11][1]*Vpk[5][11][1]))))+((mk[5]*((
          Vpk[1][10][2]*Vpk[5][10][2])+((Vpk[1][10][0]*Vpk[5][10][0])+(
          Vpk[1][10][1]*Vpk[5][10][1]))))+((mk[4]*((Vpk[1][9][2]*Vpk[5][9][2])+(
          (Vpk[1][9][0]*Vpk[5][9][0])+(Vpk[1][9][1]*Vpk[5][9][1]))))+temp[0])));
        temp[2] = ((mk[9]*((Vpk[1][14][2]*Vpk[5][14][2])+((Vpk[1][14][0]*
          Vpk[5][14][0])+(Vpk[1][14][1]*Vpk[5][14][1]))))+((mk[8]*((
          Vpk[1][13][2]*Vpk[5][13][2])+((Vpk[1][13][0]*Vpk[5][13][0])+(
          Vpk[1][13][1]*Vpk[5][13][1]))))+((mk[7]*((Vpk[1][12][2]*Vpk[5][12][2])
          +((Vpk[1][12][0]*Vpk[5][12][0])+(Vpk[1][12][1]*Vpk[5][12][1]))))+
          temp[1])));
        temp[3] = ((mk[12]*((Vpk[1][17][2]*Vpk[5][17][2])+((Vpk[1][17][0]*
          Vpk[5][17][0])+(Vpk[1][17][1]*Vpk[5][17][1]))))+((mk[11]*((
          Vpk[1][16][2]*Vpk[5][16][2])+((Vpk[1][16][0]*Vpk[5][16][0])+(
          Vpk[1][16][1]*Vpk[5][16][1]))))+((mk[10]*((Vpk[1][15][2]*Vpk[5][15][2]
          )+((Vpk[1][15][0]*Vpk[5][15][0])+(Vpk[1][15][1]*Vpk[5][15][1]))))+
          temp[2])));
        mm[1][5] = ((mk[15]*((Vpk[1][20][2]*Vpk[5][20][2])+((Vpk[1][20][0]*
          Vpk[5][20][0])+(Vpk[1][20][1]*Vpk[5][20][1]))))+((mk[14]*((
          Vpk[1][19][2]*Vpk[5][19][2])+((Vpk[1][19][0]*Vpk[5][19][0])+(
          Vpk[1][19][1]*Vpk[5][19][1]))))+((mk[13]*((Vpk[1][18][2]*Vpk[5][18][2]
          )+((Vpk[1][18][0]*Vpk[5][18][0])+(Vpk[1][18][1]*Vpk[5][18][1]))))+
          temp[3])));
        mm[1][6] = ((mk[3]*((Vpk[1][8][2]*Vpk[6][8][2])+((Vpk[1][8][0]*
          Vpk[6][8][0])+(Vpk[1][8][1]*Vpk[6][8][1]))))+((mk[1]*((Vpk[1][6][2]*
          Vpk[6][6][2])+((Vpk[1][6][0]*Vpk[6][6][0])+(Vpk[1][6][1]*Vpk[6][6][1])
          )))+(mk[2]*((Vpk[1][7][2]*Vpk[6][7][2])+((Vpk[1][7][0]*Vpk[6][7][0])+(
          Vpk[1][7][1]*Vpk[6][7][1]))))));
        mm[1][7] = ((mk[2]*((Vpk[1][7][2]*Vpk[7][7][2])+((Vpk[1][7][0]*
          Vpk[7][7][0])+(Vpk[1][7][1]*Vpk[7][7][1]))))+(mk[3]*((Vpk[1][8][2]*
          Vpk[7][8][2])+((Vpk[1][8][0]*Vpk[7][8][0])+(Vpk[1][8][1]*Vpk[7][8][1])
          ))));
        mm[1][8] = (mk[3]*((Vpk[1][8][2]*Vpk[8][8][2])+((Vpk[1][8][0]*
          Vpk[8][8][0])+(Vpk[1][8][1]*Vpk[8][8][1]))));
        temp[0] = ((mk[6]*((Vpk[1][11][2]*Vpk[9][11][2])+((Vpk[1][11][0]*
          Vpk[9][11][0])+(Vpk[1][11][1]*Vpk[9][11][1]))))+((mk[4]*((Vpk[1][9][2]
          *Vpk[9][9][2])+((Vpk[1][9][0]*Vpk[9][9][0])+(Vpk[1][9][1]*Vpk[9][9][1]
          ))))+(mk[5]*((Vpk[1][10][2]*Vpk[9][10][2])+((Vpk[1][10][0]*
          Vpk[9][10][0])+(Vpk[1][10][1]*Vpk[9][10][1]))))));
        mm[1][9] = ((mk[9]*((Vpk[1][14][2]*Vpk[9][14][2])+((Vpk[1][14][0]*
          Vpk[9][14][0])+(Vpk[1][14][1]*Vpk[9][14][1]))))+((mk[8]*((
          Vpk[1][13][2]*Vpk[9][13][2])+((Vpk[1][13][0]*Vpk[9][13][0])+(
          Vpk[1][13][1]*Vpk[9][13][1]))))+((mk[7]*((Vpk[1][12][2]*Vpk[9][12][2])
          +((Vpk[1][12][0]*Vpk[9][12][0])+(Vpk[1][12][1]*Vpk[9][12][1]))))+
          temp[0])));
        temp[0] = ((mk[7]*((Vpk[1][12][2]*Vpk[10][12][2])+((Vpk[1][12][0]*
          Vpk[10][12][0])+(Vpk[1][12][1]*Vpk[10][12][1]))))+((mk[5]*((
          Vpk[1][10][2]*Vpk[10][10][2])+((Vpk[1][10][0]*Vpk[10][10][0])+(
          Vpk[1][10][1]*Vpk[10][10][1]))))+(mk[6]*((Vpk[1][11][2]*Vpk[10][11][2]
          )+((Vpk[1][11][0]*Vpk[10][11][0])+(Vpk[1][11][1]*Vpk[10][11][1]))))));
        mm[1][10] = ((mk[9]*((Vpk[1][14][2]*Vpk[10][14][2])+((Vpk[1][14][0]*
          Vpk[10][14][0])+(Vpk[1][14][1]*Vpk[10][14][1]))))+((mk[8]*((
          Vpk[1][13][2]*Vpk[10][13][2])+((Vpk[1][13][0]*Vpk[10][13][0])+(
          Vpk[1][13][1]*Vpk[10][13][1]))))+temp[0]));
        temp[0] = ((mk[8]*((Vpk[1][13][2]*Vpk[11][13][2])+((Vpk[1][13][0]*
          Vpk[11][13][0])+(Vpk[1][13][1]*Vpk[11][13][1]))))+((mk[6]*((
          Vpk[1][11][2]*Vpk[11][11][2])+((Vpk[1][11][0]*Vpk[11][11][0])+(
          Vpk[1][11][1]*Vpk[11][11][1]))))+(mk[7]*((Vpk[1][12][2]*Vpk[11][12][2]
          )+((Vpk[1][12][0]*Vpk[11][12][0])+(Vpk[1][12][1]*Vpk[11][12][1]))))));
        mm[1][11] = ((mk[9]*((Vpk[1][14][2]*Vpk[11][14][2])+((Vpk[1][14][0]*
          Vpk[11][14][0])+(Vpk[1][14][1]*Vpk[11][14][1]))))+temp[0]);
        mm[1][12] = ((mk[9]*((Vpk[1][14][2]*Vpk[12][14][2])+((Vpk[1][14][0]*
          Vpk[12][14][0])+(Vpk[1][14][1]*Vpk[12][14][1]))))+((mk[7]*((
          Vpk[1][12][2]*Vpk[12][12][2])+((Vpk[1][12][0]*Vpk[12][12][0])+(
          Vpk[1][12][1]*Vpk[12][12][1]))))+(mk[8]*((Vpk[1][13][2]*Vpk[12][13][2]
          )+((Vpk[1][13][0]*Vpk[12][13][0])+(Vpk[1][13][1]*Vpk[12][13][1]))))));
        mm[1][13] = ((mk[8]*((Vpk[1][13][2]*Vpk[13][13][2])+((Vpk[1][13][0]*
          Vpk[13][13][0])+(Vpk[1][13][1]*Vpk[13][13][1]))))+(mk[9]*((
          Vpk[1][14][2]*Vpk[13][14][2])+((Vpk[1][14][0]*Vpk[13][14][0])+(
          Vpk[1][14][1]*Vpk[13][14][1])))));
        mm[1][14] = (mk[9]*((Vpk[1][14][2]*Vpk[14][14][2])+((Vpk[1][14][0]*
          Vpk[14][14][0])+(Vpk[1][14][1]*Vpk[14][14][1]))));
        temp[0] = ((mk[12]*((Vpk[1][17][2]*Vpk[15][17][2])+((Vpk[1][17][0]*
          Vpk[15][17][0])+(Vpk[1][17][1]*Vpk[15][17][1]))))+((mk[10]*((
          Vpk[1][15][2]*Vpk[15][15][2])+((Vpk[1][15][0]*Vpk[15][15][0])+(
          Vpk[1][15][1]*Vpk[15][15][1]))))+(mk[11]*((Vpk[1][16][2]*
          Vpk[15][16][2])+((Vpk[1][16][0]*Vpk[15][16][0])+(Vpk[1][16][1]*
          Vpk[15][16][1]))))));
        mm[1][15] = ((mk[15]*((Vpk[1][20][2]*Vpk[15][20][2])+((Vpk[1][20][0]*
          Vpk[15][20][0])+(Vpk[1][20][1]*Vpk[15][20][1]))))+((mk[14]*((
          Vpk[1][19][2]*Vpk[15][19][2])+((Vpk[1][19][0]*Vpk[15][19][0])+(
          Vpk[1][19][1]*Vpk[15][19][1]))))+((mk[13]*((Vpk[1][18][2]*
          Vpk[15][18][2])+((Vpk[1][18][0]*Vpk[15][18][0])+(Vpk[1][18][1]*
          Vpk[15][18][1]))))+temp[0])));
        temp[0] = ((mk[13]*((Vpk[1][18][2]*Vpk[16][18][2])+((Vpk[1][18][0]*
          Vpk[16][18][0])+(Vpk[1][18][1]*Vpk[16][18][1]))))+((mk[11]*((
          Vpk[1][16][2]*Vpk[16][16][2])+((Vpk[1][16][0]*Vpk[16][16][0])+(
          Vpk[1][16][1]*Vpk[16][16][1]))))+(mk[12]*((Vpk[1][17][2]*
          Vpk[16][17][2])+((Vpk[1][17][0]*Vpk[16][17][0])+(Vpk[1][17][1]*
          Vpk[16][17][1]))))));
        mm[1][16] = ((mk[15]*((Vpk[1][20][2]*Vpk[16][20][2])+((Vpk[1][20][0]*
          Vpk[16][20][0])+(Vpk[1][20][1]*Vpk[16][20][1]))))+((mk[14]*((
          Vpk[1][19][2]*Vpk[16][19][2])+((Vpk[1][19][0]*Vpk[16][19][0])+(
          Vpk[1][19][1]*Vpk[16][19][1]))))+temp[0]));
        temp[0] = ((mk[14]*((Vpk[1][19][2]*Vpk[17][19][2])+((Vpk[1][19][0]*
          Vpk[17][19][0])+(Vpk[1][19][1]*Vpk[17][19][1]))))+((mk[12]*((
          Vpk[1][17][2]*Vpk[17][17][2])+((Vpk[1][17][0]*Vpk[17][17][0])+(
          Vpk[1][17][1]*Vpk[17][17][1]))))+(mk[13]*((Vpk[1][18][2]*
          Vpk[17][18][2])+((Vpk[1][18][0]*Vpk[17][18][0])+(Vpk[1][18][1]*
          Vpk[17][18][1]))))));
        mm[1][17] = ((mk[15]*((Vpk[1][20][2]*Vpk[17][20][2])+((Vpk[1][20][0]*
          Vpk[17][20][0])+(Vpk[1][20][1]*Vpk[17][20][1]))))+temp[0]);
        mm[1][18] = ((mk[15]*((Vpk[1][20][2]*Vpk[18][20][2])+((Vpk[1][20][0]*
          Vpk[18][20][0])+(Vpk[1][20][1]*Vpk[18][20][1]))))+((mk[13]*((
          Vpk[1][18][2]*Vpk[18][18][2])+((Vpk[1][18][0]*Vpk[18][18][0])+(
          Vpk[1][18][1]*Vpk[18][18][1]))))+(mk[14]*((Vpk[1][19][2]*
          Vpk[18][19][2])+((Vpk[1][19][0]*Vpk[18][19][0])+(Vpk[1][19][1]*
          Vpk[18][19][1]))))));
        mm[1][19] = ((mk[14]*((Vpk[1][19][2]*Vpk[19][19][2])+((Vpk[1][19][0]*
          Vpk[19][19][0])+(Vpk[1][19][1]*Vpk[19][19][1]))))+(mk[15]*((
          Vpk[1][20][2]*Vpk[19][20][2])+((Vpk[1][20][0]*Vpk[19][20][0])+(
          Vpk[1][20][1]*Vpk[19][20][1])))));
        mm[1][20] = (mk[15]*((Vpk[1][20][2]*Vpk[20][20][2])+((Vpk[1][20][0]*
          Vpk[20][20][0])+(Vpk[1][20][1]*Vpk[20][20][1]))));
        temp[0] = ((mk[2]*((Vpk[2][7][2]*Vpk[2][7][2])+((Vpk[2][7][0]*
          Vpk[2][7][0])+(Vpk[2][7][1]*Vpk[2][7][1]))))+((mk[0]*((Vpk[2][3][2]*
          Vpk[2][3][2])+((Vpk[2][3][0]*Vpk[2][3][0])+(Vpk[2][3][1]*Vpk[2][3][1])
          )))+(mk[1]*((Vpk[2][6][2]*Vpk[2][6][2])+((Vpk[2][6][0]*Vpk[2][6][0])+(
          Vpk[2][6][1]*Vpk[2][6][1]))))));
        temp[1] = ((mk[5]*((Vpk[2][10][2]*Vpk[2][10][2])+((Vpk[2][10][0]*
          Vpk[2][10][0])+(Vpk[2][10][1]*Vpk[2][10][1]))))+((mk[4]*((Vpk[2][9][2]
          *Vpk[2][9][2])+((Vpk[2][9][0]*Vpk[2][9][0])+(Vpk[2][9][1]*Vpk[2][9][1]
          ))))+((mk[3]*((Vpk[2][8][2]*Vpk[2][8][2])+((Vpk[2][8][0]*Vpk[2][8][0])
          +(Vpk[2][8][1]*Vpk[2][8][1]))))+temp[0])));
        temp[2] = ((mk[8]*((Vpk[2][13][2]*Vpk[2][13][2])+((Vpk[2][13][0]*
          Vpk[2][13][0])+(Vpk[2][13][1]*Vpk[2][13][1]))))+((mk[7]*((
          Vpk[2][12][2]*Vpk[2][12][2])+((Vpk[2][12][0]*Vpk[2][12][0])+(
          Vpk[2][12][1]*Vpk[2][12][1]))))+((mk[6]*((Vpk[2][11][2]*Vpk[2][11][2])
          +((Vpk[2][11][0]*Vpk[2][11][0])+(Vpk[2][11][1]*Vpk[2][11][1]))))+
          temp[1])));
        temp[3] = ((mk[11]*((Vpk[2][16][2]*Vpk[2][16][2])+((Vpk[2][16][0]*
          Vpk[2][16][0])+(Vpk[2][16][1]*Vpk[2][16][1]))))+((mk[10]*((
          Vpk[2][15][2]*Vpk[2][15][2])+((Vpk[2][15][0]*Vpk[2][15][0])+(
          Vpk[2][15][1]*Vpk[2][15][1]))))+((mk[9]*((Vpk[2][14][2]*Vpk[2][14][2])
          +((Vpk[2][14][0]*Vpk[2][14][0])+(Vpk[2][14][1]*Vpk[2][14][1]))))+
          temp[2])));
        temp[4] = ((mk[14]*((Vpk[2][19][2]*Vpk[2][19][2])+((Vpk[2][19][0]*
          Vpk[2][19][0])+(Vpk[2][19][1]*Vpk[2][19][1]))))+((mk[13]*((
          Vpk[2][18][2]*Vpk[2][18][2])+((Vpk[2][18][0]*Vpk[2][18][0])+(
          Vpk[2][18][1]*Vpk[2][18][1]))))+((mk[12]*((Vpk[2][17][2]*Vpk[2][17][2]
          )+((Vpk[2][17][0]*Vpk[2][17][0])+(Vpk[2][17][1]*Vpk[2][17][1]))))+
          temp[3])));
        mm[2][2] = ((mk[15]*((Vpk[2][20][2]*Vpk[2][20][2])+((Vpk[2][20][0]*
          Vpk[2][20][0])+(Vpk[2][20][1]*Vpk[2][20][1]))))+temp[4]);
        temp[0] = ((mk[3]*((Vpk[2][8][2]*Vpk[3][8][2])+((Vpk[2][8][0]*
          Vpk[3][8][0])+(Vpk[2][8][1]*Vpk[3][8][1]))))+((mk[2]*((Vpk[2][7][2]*
          Vpk[3][7][2])+((Vpk[2][7][0]*Vpk[3][7][0])+(Vpk[2][7][1]*Vpk[3][7][1])
          )))+((mk[0]*((rk[0][2]*Vpk[2][3][1])-(rk[0][1]*Vpk[2][3][2])))+(mk[1]*
          ((Vpk[2][6][2]*Vpk[3][6][2])+((Vpk[2][6][0]*Vpk[3][6][0])+(
          Vpk[2][6][1]*Vpk[3][6][1])))))));
        temp[1] = ((mk[6]*((Vpk[2][11][2]*Vpk[3][11][2])+((Vpk[2][11][0]*
          Vpk[3][11][0])+(Vpk[2][11][1]*Vpk[3][11][1]))))+((mk[5]*((
          Vpk[2][10][2]*Vpk[3][10][2])+((Vpk[2][10][0]*Vpk[3][10][0])+(
          Vpk[2][10][1]*Vpk[3][10][1]))))+((mk[4]*((Vpk[2][9][2]*Vpk[3][9][2])+(
          (Vpk[2][9][0]*Vpk[3][9][0])+(Vpk[2][9][1]*Vpk[3][9][1]))))+temp[0])));
        temp[2] = ((mk[9]*((Vpk[2][14][2]*Vpk[3][14][2])+((Vpk[2][14][0]*
          Vpk[3][14][0])+(Vpk[2][14][1]*Vpk[3][14][1]))))+((mk[8]*((
          Vpk[2][13][2]*Vpk[3][13][2])+((Vpk[2][13][0]*Vpk[3][13][0])+(
          Vpk[2][13][1]*Vpk[3][13][1]))))+((mk[7]*((Vpk[2][12][2]*Vpk[3][12][2])
          +((Vpk[2][12][0]*Vpk[3][12][0])+(Vpk[2][12][1]*Vpk[3][12][1]))))+
          temp[1])));
        temp[3] = ((mk[12]*((Vpk[2][17][2]*Vpk[3][17][2])+((Vpk[2][17][0]*
          Vpk[3][17][0])+(Vpk[2][17][1]*Vpk[3][17][1]))))+((mk[11]*((
          Vpk[2][16][2]*Vpk[3][16][2])+((Vpk[2][16][0]*Vpk[3][16][0])+(
          Vpk[2][16][1]*Vpk[3][16][1]))))+((mk[10]*((Vpk[2][15][2]*Vpk[3][15][2]
          )+((Vpk[2][15][0]*Vpk[3][15][0])+(Vpk[2][15][1]*Vpk[3][15][1]))))+
          temp[2])));
        mm[2][3] = ((mk[15]*((Vpk[2][20][2]*Vpk[3][20][2])+((Vpk[2][20][0]*
          Vpk[3][20][0])+(Vpk[2][20][1]*Vpk[3][20][1]))))+((mk[14]*((
          Vpk[2][19][2]*Vpk[3][19][2])+((Vpk[2][19][0]*Vpk[3][19][0])+(
          Vpk[2][19][1]*Vpk[3][19][1]))))+((mk[13]*((Vpk[2][18][2]*Vpk[3][18][2]
          )+((Vpk[2][18][0]*Vpk[3][18][0])+(Vpk[2][18][1]*Vpk[3][18][1]))))+
          temp[3])));
        temp[0] = ((mk[3]*((Vpk[2][8][2]*Vpk[4][8][2])+((Vpk[2][8][0]*
          Vpk[4][8][0])+(Vpk[2][8][1]*Vpk[4][8][1]))))+((mk[2]*((Vpk[2][7][2]*
          Vpk[4][7][2])+((Vpk[2][7][0]*Vpk[4][7][0])+(Vpk[2][7][1]*Vpk[4][7][1])
          )))+((mk[0]*((rk[0][0]*Vpk[2][3][2])-(rk[0][2]*Vpk[2][3][0])))+(mk[1]*
          ((Vpk[2][6][2]*Vpk[4][6][2])+((Vpk[2][6][0]*Vpk[4][6][0])+(
          Vpk[2][6][1]*Vpk[4][6][1])))))));
        temp[1] = ((mk[6]*((Vpk[2][11][2]*Vpk[4][11][2])+((Vpk[2][11][0]*
          Vpk[4][11][0])+(Vpk[2][11][1]*Vpk[4][11][1]))))+((mk[5]*((
          Vpk[2][10][2]*Vpk[4][10][2])+((Vpk[2][10][0]*Vpk[4][10][0])+(
          Vpk[2][10][1]*Vpk[4][10][1]))))+((mk[4]*((Vpk[2][9][2]*Vpk[4][9][2])+(
          (Vpk[2][9][0]*Vpk[4][9][0])+(Vpk[2][9][1]*Vpk[4][9][1]))))+temp[0])));
        temp[2] = ((mk[9]*((Vpk[2][14][2]*Vpk[4][14][2])+((Vpk[2][14][0]*
          Vpk[4][14][0])+(Vpk[2][14][1]*Vpk[4][14][1]))))+((mk[8]*((
          Vpk[2][13][2]*Vpk[4][13][2])+((Vpk[2][13][0]*Vpk[4][13][0])+(
          Vpk[2][13][1]*Vpk[4][13][1]))))+((mk[7]*((Vpk[2][12][2]*Vpk[4][12][2])
          +((Vpk[2][12][0]*Vpk[4][12][0])+(Vpk[2][12][1]*Vpk[4][12][1]))))+
          temp[1])));
        temp[3] = ((mk[12]*((Vpk[2][17][2]*Vpk[4][17][2])+((Vpk[2][17][0]*
          Vpk[4][17][0])+(Vpk[2][17][1]*Vpk[4][17][1]))))+((mk[11]*((
          Vpk[2][16][2]*Vpk[4][16][2])+((Vpk[2][16][0]*Vpk[4][16][0])+(
          Vpk[2][16][1]*Vpk[4][16][1]))))+((mk[10]*((Vpk[2][15][2]*Vpk[4][15][2]
          )+((Vpk[2][15][0]*Vpk[4][15][0])+(Vpk[2][15][1]*Vpk[4][15][1]))))+
          temp[2])));
        mm[2][4] = ((mk[15]*((Vpk[2][20][2]*Vpk[4][20][2])+((Vpk[2][20][0]*
          Vpk[4][20][0])+(Vpk[2][20][1]*Vpk[4][20][1]))))+((mk[14]*((
          Vpk[2][19][2]*Vpk[4][19][2])+((Vpk[2][19][0]*Vpk[4][19][0])+(
          Vpk[2][19][1]*Vpk[4][19][1]))))+((mk[13]*((Vpk[2][18][2]*Vpk[4][18][2]
          )+((Vpk[2][18][0]*Vpk[4][18][0])+(Vpk[2][18][1]*Vpk[4][18][1]))))+
          temp[3])));
        temp[0] = ((mk[3]*((Vpk[2][8][2]*Vpk[5][8][2])+((Vpk[2][8][0]*
          Vpk[5][8][0])+(Vpk[2][8][1]*Vpk[5][8][1]))))+((mk[2]*((Vpk[2][7][2]*
          Vpk[5][7][2])+((Vpk[2][7][0]*Vpk[5][7][0])+(Vpk[2][7][1]*Vpk[5][7][1])
          )))+((mk[0]*((rk[0][1]*Vpk[2][3][0])-(rk[0][0]*Vpk[2][3][1])))+(mk[1]*
          ((Vpk[2][6][2]*Vpk[5][6][2])+((Vpk[2][6][0]*Vpk[5][6][0])+(
          Vpk[2][6][1]*Vpk[5][6][1])))))));
        temp[1] = ((mk[6]*((Vpk[2][11][2]*Vpk[5][11][2])+((Vpk[2][11][0]*
          Vpk[5][11][0])+(Vpk[2][11][1]*Vpk[5][11][1]))))+((mk[5]*((
          Vpk[2][10][2]*Vpk[5][10][2])+((Vpk[2][10][0]*Vpk[5][10][0])+(
          Vpk[2][10][1]*Vpk[5][10][1]))))+((mk[4]*((Vpk[2][9][2]*Vpk[5][9][2])+(
          (Vpk[2][9][0]*Vpk[5][9][0])+(Vpk[2][9][1]*Vpk[5][9][1]))))+temp[0])));
        temp[2] = ((mk[9]*((Vpk[2][14][2]*Vpk[5][14][2])+((Vpk[2][14][0]*
          Vpk[5][14][0])+(Vpk[2][14][1]*Vpk[5][14][1]))))+((mk[8]*((
          Vpk[2][13][2]*Vpk[5][13][2])+((Vpk[2][13][0]*Vpk[5][13][0])+(
          Vpk[2][13][1]*Vpk[5][13][1]))))+((mk[7]*((Vpk[2][12][2]*Vpk[5][12][2])
          +((Vpk[2][12][0]*Vpk[5][12][0])+(Vpk[2][12][1]*Vpk[5][12][1]))))+
          temp[1])));
        temp[3] = ((mk[12]*((Vpk[2][17][2]*Vpk[5][17][2])+((Vpk[2][17][0]*
          Vpk[5][17][0])+(Vpk[2][17][1]*Vpk[5][17][1]))))+((mk[11]*((
          Vpk[2][16][2]*Vpk[5][16][2])+((Vpk[2][16][0]*Vpk[5][16][0])+(
          Vpk[2][16][1]*Vpk[5][16][1]))))+((mk[10]*((Vpk[2][15][2]*Vpk[5][15][2]
          )+((Vpk[2][15][0]*Vpk[5][15][0])+(Vpk[2][15][1]*Vpk[5][15][1]))))+
          temp[2])));
        mm[2][5] = ((mk[15]*((Vpk[2][20][2]*Vpk[5][20][2])+((Vpk[2][20][0]*
          Vpk[5][20][0])+(Vpk[2][20][1]*Vpk[5][20][1]))))+((mk[14]*((
          Vpk[2][19][2]*Vpk[5][19][2])+((Vpk[2][19][0]*Vpk[5][19][0])+(
          Vpk[2][19][1]*Vpk[5][19][1]))))+((mk[13]*((Vpk[2][18][2]*Vpk[5][18][2]
          )+((Vpk[2][18][0]*Vpk[5][18][0])+(Vpk[2][18][1]*Vpk[5][18][1]))))+
          temp[3])));
        mm[2][6] = ((mk[3]*((Vpk[2][8][2]*Vpk[6][8][2])+((Vpk[2][8][0]*
          Vpk[6][8][0])+(Vpk[2][8][1]*Vpk[6][8][1]))))+((mk[1]*((Vpk[2][6][2]*
          Vpk[6][6][2])+((Vpk[2][6][0]*Vpk[6][6][0])+(Vpk[2][6][1]*Vpk[6][6][1])
          )))+(mk[2]*((Vpk[2][7][2]*Vpk[6][7][2])+((Vpk[2][7][0]*Vpk[6][7][0])+(
          Vpk[2][7][1]*Vpk[6][7][1]))))));
        mm[2][7] = ((mk[2]*((Vpk[2][7][2]*Vpk[7][7][2])+((Vpk[2][7][0]*
          Vpk[7][7][0])+(Vpk[2][7][1]*Vpk[7][7][1]))))+(mk[3]*((Vpk[2][8][2]*
          Vpk[7][8][2])+((Vpk[2][8][0]*Vpk[7][8][0])+(Vpk[2][8][1]*Vpk[7][8][1])
          ))));
        mm[2][8] = (mk[3]*((Vpk[2][8][2]*Vpk[8][8][2])+((Vpk[2][8][0]*
          Vpk[8][8][0])+(Vpk[2][8][1]*Vpk[8][8][1]))));
        temp[0] = ((mk[6]*((Vpk[2][11][2]*Vpk[9][11][2])+((Vpk[2][11][0]*
          Vpk[9][11][0])+(Vpk[2][11][1]*Vpk[9][11][1]))))+((mk[4]*((Vpk[2][9][2]
          *Vpk[9][9][2])+((Vpk[2][9][0]*Vpk[9][9][0])+(Vpk[2][9][1]*Vpk[9][9][1]
          ))))+(mk[5]*((Vpk[2][10][2]*Vpk[9][10][2])+((Vpk[2][10][0]*
          Vpk[9][10][0])+(Vpk[2][10][1]*Vpk[9][10][1]))))));
        mm[2][9] = ((mk[9]*((Vpk[2][14][2]*Vpk[9][14][2])+((Vpk[2][14][0]*
          Vpk[9][14][0])+(Vpk[2][14][1]*Vpk[9][14][1]))))+((mk[8]*((
          Vpk[2][13][2]*Vpk[9][13][2])+((Vpk[2][13][0]*Vpk[9][13][0])+(
          Vpk[2][13][1]*Vpk[9][13][1]))))+((mk[7]*((Vpk[2][12][2]*Vpk[9][12][2])
          +((Vpk[2][12][0]*Vpk[9][12][0])+(Vpk[2][12][1]*Vpk[9][12][1]))))+
          temp[0])));
        temp[0] = ((mk[7]*((Vpk[2][12][2]*Vpk[10][12][2])+((Vpk[2][12][0]*
          Vpk[10][12][0])+(Vpk[2][12][1]*Vpk[10][12][1]))))+((mk[5]*((
          Vpk[2][10][2]*Vpk[10][10][2])+((Vpk[2][10][0]*Vpk[10][10][0])+(
          Vpk[2][10][1]*Vpk[10][10][1]))))+(mk[6]*((Vpk[2][11][2]*Vpk[10][11][2]
          )+((Vpk[2][11][0]*Vpk[10][11][0])+(Vpk[2][11][1]*Vpk[10][11][1]))))));
        mm[2][10] = ((mk[9]*((Vpk[2][14][2]*Vpk[10][14][2])+((Vpk[2][14][0]*
          Vpk[10][14][0])+(Vpk[2][14][1]*Vpk[10][14][1]))))+((mk[8]*((
          Vpk[2][13][2]*Vpk[10][13][2])+((Vpk[2][13][0]*Vpk[10][13][0])+(
          Vpk[2][13][1]*Vpk[10][13][1]))))+temp[0]));
        temp[0] = ((mk[8]*((Vpk[2][13][2]*Vpk[11][13][2])+((Vpk[2][13][0]*
          Vpk[11][13][0])+(Vpk[2][13][1]*Vpk[11][13][1]))))+((mk[6]*((
          Vpk[2][11][2]*Vpk[11][11][2])+((Vpk[2][11][0]*Vpk[11][11][0])+(
          Vpk[2][11][1]*Vpk[11][11][1]))))+(mk[7]*((Vpk[2][12][2]*Vpk[11][12][2]
          )+((Vpk[2][12][0]*Vpk[11][12][0])+(Vpk[2][12][1]*Vpk[11][12][1]))))));
        mm[2][11] = ((mk[9]*((Vpk[2][14][2]*Vpk[11][14][2])+((Vpk[2][14][0]*
          Vpk[11][14][0])+(Vpk[2][14][1]*Vpk[11][14][1]))))+temp[0]);
        mm[2][12] = ((mk[9]*((Vpk[2][14][2]*Vpk[12][14][2])+((Vpk[2][14][0]*
          Vpk[12][14][0])+(Vpk[2][14][1]*Vpk[12][14][1]))))+((mk[7]*((
          Vpk[2][12][2]*Vpk[12][12][2])+((Vpk[2][12][0]*Vpk[12][12][0])+(
          Vpk[2][12][1]*Vpk[12][12][1]))))+(mk[8]*((Vpk[2][13][2]*Vpk[12][13][2]
          )+((Vpk[2][13][0]*Vpk[12][13][0])+(Vpk[2][13][1]*Vpk[12][13][1]))))));
        mm[2][13] = ((mk[8]*((Vpk[2][13][2]*Vpk[13][13][2])+((Vpk[2][13][0]*
          Vpk[13][13][0])+(Vpk[2][13][1]*Vpk[13][13][1]))))+(mk[9]*((
          Vpk[2][14][2]*Vpk[13][14][2])+((Vpk[2][14][0]*Vpk[13][14][0])+(
          Vpk[2][14][1]*Vpk[13][14][1])))));
        mm[2][14] = (mk[9]*((Vpk[2][14][2]*Vpk[14][14][2])+((Vpk[2][14][0]*
          Vpk[14][14][0])+(Vpk[2][14][1]*Vpk[14][14][1]))));
        temp[0] = ((mk[12]*((Vpk[2][17][2]*Vpk[15][17][2])+((Vpk[2][17][0]*
          Vpk[15][17][0])+(Vpk[2][17][1]*Vpk[15][17][1]))))+((mk[10]*((
          Vpk[2][15][2]*Vpk[15][15][2])+((Vpk[2][15][0]*Vpk[15][15][0])+(
          Vpk[2][15][1]*Vpk[15][15][1]))))+(mk[11]*((Vpk[2][16][2]*
          Vpk[15][16][2])+((Vpk[2][16][0]*Vpk[15][16][0])+(Vpk[2][16][1]*
          Vpk[15][16][1]))))));
        mm[2][15] = ((mk[15]*((Vpk[2][20][2]*Vpk[15][20][2])+((Vpk[2][20][0]*
          Vpk[15][20][0])+(Vpk[2][20][1]*Vpk[15][20][1]))))+((mk[14]*((
          Vpk[2][19][2]*Vpk[15][19][2])+((Vpk[2][19][0]*Vpk[15][19][0])+(
          Vpk[2][19][1]*Vpk[15][19][1]))))+((mk[13]*((Vpk[2][18][2]*
          Vpk[15][18][2])+((Vpk[2][18][0]*Vpk[15][18][0])+(Vpk[2][18][1]*
          Vpk[15][18][1]))))+temp[0])));
        temp[0] = ((mk[13]*((Vpk[2][18][2]*Vpk[16][18][2])+((Vpk[2][18][0]*
          Vpk[16][18][0])+(Vpk[2][18][1]*Vpk[16][18][1]))))+((mk[11]*((
          Vpk[2][16][2]*Vpk[16][16][2])+((Vpk[2][16][0]*Vpk[16][16][0])+(
          Vpk[2][16][1]*Vpk[16][16][1]))))+(mk[12]*((Vpk[2][17][2]*
          Vpk[16][17][2])+((Vpk[2][17][0]*Vpk[16][17][0])+(Vpk[2][17][1]*
          Vpk[16][17][1]))))));
        mm[2][16] = ((mk[15]*((Vpk[2][20][2]*Vpk[16][20][2])+((Vpk[2][20][0]*
          Vpk[16][20][0])+(Vpk[2][20][1]*Vpk[16][20][1]))))+((mk[14]*((
          Vpk[2][19][2]*Vpk[16][19][2])+((Vpk[2][19][0]*Vpk[16][19][0])+(
          Vpk[2][19][1]*Vpk[16][19][1]))))+temp[0]));
        temp[0] = ((mk[14]*((Vpk[2][19][2]*Vpk[17][19][2])+((Vpk[2][19][0]*
          Vpk[17][19][0])+(Vpk[2][19][1]*Vpk[17][19][1]))))+((mk[12]*((
          Vpk[2][17][2]*Vpk[17][17][2])+((Vpk[2][17][0]*Vpk[17][17][0])+(
          Vpk[2][17][1]*Vpk[17][17][1]))))+(mk[13]*((Vpk[2][18][2]*
          Vpk[17][18][2])+((Vpk[2][18][0]*Vpk[17][18][0])+(Vpk[2][18][1]*
          Vpk[17][18][1]))))));
        mm[2][17] = ((mk[15]*((Vpk[2][20][2]*Vpk[17][20][2])+((Vpk[2][20][0]*
          Vpk[17][20][0])+(Vpk[2][20][1]*Vpk[17][20][1]))))+temp[0]);
        mm[2][18] = ((mk[15]*((Vpk[2][20][2]*Vpk[18][20][2])+((Vpk[2][20][0]*
          Vpk[18][20][0])+(Vpk[2][20][1]*Vpk[18][20][1]))))+((mk[13]*((
          Vpk[2][18][2]*Vpk[18][18][2])+((Vpk[2][18][0]*Vpk[18][18][0])+(
          Vpk[2][18][1]*Vpk[18][18][1]))))+(mk[14]*((Vpk[2][19][2]*
          Vpk[18][19][2])+((Vpk[2][19][0]*Vpk[18][19][0])+(Vpk[2][19][1]*
          Vpk[18][19][1]))))));
        mm[2][19] = ((mk[14]*((Vpk[2][19][2]*Vpk[19][19][2])+((Vpk[2][19][0]*
          Vpk[19][19][0])+(Vpk[2][19][1]*Vpk[19][19][1]))))+(mk[15]*((
          Vpk[2][20][2]*Vpk[19][20][2])+((Vpk[2][20][0]*Vpk[19][20][0])+(
          Vpk[2][20][1]*Vpk[19][20][1])))));
        mm[2][20] = (mk[15]*((Vpk[2][20][2]*Vpk[20][20][2])+((Vpk[2][20][0]*
          Vpk[20][20][0])+(Vpk[2][20][1]*Vpk[20][20][1]))));
        temp[0] = ((ik[0][0][0]+(mk[0]*((rk[0][1]*rk[0][1])+(rk[0][2]*rk[0][2]))
          ))+((mk[1]*((Vpk[3][6][2]*Vpk[3][6][2])+((Vpk[3][6][0]*Vpk[3][6][0])+(
          Vpk[3][6][1]*Vpk[3][6][1]))))+((Cik[6][0][2]*IkWpk[3][6][2])+((
          Cik[6][0][0]*IkWpk[3][6][0])+(Cik[6][0][1]*IkWpk[3][6][1])))));
        temp[1] = (temp[0]+((mk[2]*((Vpk[3][7][2]*Vpk[3][7][2])+((Vpk[3][7][0]*
          Vpk[3][7][0])+(Vpk[3][7][1]*Vpk[3][7][1]))))+((IkWpk[3][7][2]*
          Wpk[3][7][2])+((IkWpk[3][7][0]*Wpk[3][7][0])+(IkWpk[3][7][1]*
          Wpk[3][7][1])))));
        temp[2] = (((mk[3]*((Vpk[3][8][2]*Vpk[3][8][2])+((Vpk[3][8][0]*
          Vpk[3][8][0])+(Vpk[3][8][1]*Vpk[3][8][1]))))+((IkWpk[3][8][2]*
          Wpk[3][8][2])+((IkWpk[3][8][0]*Wpk[3][8][0])+(IkWpk[3][8][1]*
          Wpk[3][8][1]))))+temp[1]);
        temp[3] = (((mk[4]*((Vpk[3][9][2]*Vpk[3][9][2])+((Vpk[3][9][0]*
          Vpk[3][9][0])+(Vpk[3][9][1]*Vpk[3][9][1]))))+((Cik[9][0][2]*
          IkWpk[3][9][2])+((Cik[9][0][0]*IkWpk[3][9][0])+(Cik[9][0][1]*
          IkWpk[3][9][1]))))+temp[2]);
        temp[4] = (((mk[5]*((Vpk[3][10][2]*Vpk[3][10][2])+((Vpk[3][10][0]*
          Vpk[3][10][0])+(Vpk[3][10][1]*Vpk[3][10][1]))))+((IkWpk[3][10][2]*
          Wpk[3][10][2])+((IkWpk[3][10][0]*Wpk[3][10][0])+(IkWpk[3][10][1]*
          Wpk[3][10][1]))))+temp[3]);
        temp[5] = (((mk[6]*((Vpk[3][11][2]*Vpk[3][11][2])+((Vpk[3][11][0]*
          Vpk[3][11][0])+(Vpk[3][11][1]*Vpk[3][11][1]))))+((IkWpk[3][11][2]*
          Wpk[3][11][2])+((IkWpk[3][11][0]*Wpk[3][11][0])+(IkWpk[3][11][1]*
          Wpk[3][11][1]))))+temp[4]);
        temp[6] = (((mk[7]*((Vpk[3][12][2]*Vpk[3][12][2])+((Vpk[3][12][0]*
          Vpk[3][12][0])+(Vpk[3][12][1]*Vpk[3][12][1]))))+((IkWpk[3][12][2]*
          Wpk[3][12][2])+((IkWpk[3][12][0]*Wpk[3][12][0])+(IkWpk[3][12][1]*
          Wpk[3][12][1]))))+temp[5]);
        temp[7] = (((mk[8]*((Vpk[3][13][2]*Vpk[3][13][2])+((Vpk[3][13][0]*
          Vpk[3][13][0])+(Vpk[3][13][1]*Vpk[3][13][1]))))+((IkWpk[3][13][2]*
          Wpk[3][13][2])+((IkWpk[3][13][0]*Wpk[3][13][0])+(IkWpk[3][13][1]*
          Wpk[3][13][1]))))+temp[6]);
        temp[8] = (((mk[9]*((Vpk[3][14][2]*Vpk[3][14][2])+((Vpk[3][14][0]*
          Vpk[3][14][0])+(Vpk[3][14][1]*Vpk[3][14][1]))))+((IkWpk[3][14][2]*
          Wpk[3][14][2])+((IkWpk[3][14][0]*Wpk[3][14][0])+(IkWpk[3][14][1]*
          Wpk[3][14][1]))))+temp[7]);
        temp[9] = (((mk[10]*((Vpk[3][15][2]*Vpk[3][15][2])+((Vpk[3][15][0]*
          Vpk[3][15][0])+(Vpk[3][15][1]*Vpk[3][15][1]))))+((Cik[15][0][2]*
          IkWpk[3][15][2])+((Cik[15][0][0]*IkWpk[3][15][0])+(Cik[15][0][1]*
          IkWpk[3][15][1]))))+temp[8]);
        temp[10] = (((mk[11]*((Vpk[3][16][2]*Vpk[3][16][2])+((Vpk[3][16][0]*
          Vpk[3][16][0])+(Vpk[3][16][1]*Vpk[3][16][1]))))+((IkWpk[3][16][2]*
          Wpk[3][16][2])+((IkWpk[3][16][0]*Wpk[3][16][0])+(IkWpk[3][16][1]*
          Wpk[3][16][1]))))+temp[9]);
        temp[11] = (((mk[12]*((Vpk[3][17][2]*Vpk[3][17][2])+((Vpk[3][17][0]*
          Vpk[3][17][0])+(Vpk[3][17][1]*Vpk[3][17][1]))))+((IkWpk[3][17][2]*
          Wpk[3][17][2])+((IkWpk[3][17][0]*Wpk[3][17][0])+(IkWpk[3][17][1]*
          Wpk[3][17][1]))))+temp[10]);
        temp[12] = (((mk[13]*((Vpk[3][18][2]*Vpk[3][18][2])+((Vpk[3][18][0]*
          Vpk[3][18][0])+(Vpk[3][18][1]*Vpk[3][18][1]))))+((IkWpk[3][18][2]*
          Wpk[3][18][2])+((IkWpk[3][18][0]*Wpk[3][18][0])+(IkWpk[3][18][1]*
          Wpk[3][18][1]))))+temp[11]);
        temp[13] = (((mk[14]*((Vpk[3][19][2]*Vpk[3][19][2])+((Vpk[3][19][0]*
          Vpk[3][19][0])+(Vpk[3][19][1]*Vpk[3][19][1]))))+((IkWpk[3][19][2]*
          Wpk[3][19][2])+((IkWpk[3][19][0]*Wpk[3][19][0])+(IkWpk[3][19][1]*
          Wpk[3][19][1]))))+temp[12]);
        mm[3][3] = (((mk[15]*((Vpk[3][20][2]*Vpk[3][20][2])+((Vpk[3][20][0]*
          Vpk[3][20][0])+(Vpk[3][20][1]*Vpk[3][20][1]))))+((IkWpk[3][20][2]*
          Wpk[3][20][2])+((IkWpk[3][20][0]*Wpk[3][20][0])+(IkWpk[3][20][1]*
          Wpk[3][20][1]))))+temp[13]);
        temp[0] = ((ik[0][0][1]-(mk[0]*(rk[0][0]*rk[0][1])))+((mk[1]*((
          Vpk[3][6][2]*Vpk[4][6][2])+((Vpk[3][6][0]*Vpk[4][6][0])+(Vpk[3][6][1]*
          Vpk[4][6][1]))))+((Cik[6][0][2]*IkWpk[4][6][2])+((Cik[6][0][0]*
          IkWpk[4][6][0])+(Cik[6][0][1]*IkWpk[4][6][1])))));
        temp[1] = (temp[0]+((mk[2]*((Vpk[3][7][2]*Vpk[4][7][2])+((Vpk[3][7][0]*
          Vpk[4][7][0])+(Vpk[3][7][1]*Vpk[4][7][1]))))+((IkWpk[4][7][2]*
          Wpk[3][7][2])+((IkWpk[4][7][0]*Wpk[3][7][0])+(IkWpk[4][7][1]*
          Wpk[3][7][1])))));
        temp[2] = (((mk[3]*((Vpk[3][8][2]*Vpk[4][8][2])+((Vpk[3][8][0]*
          Vpk[4][8][0])+(Vpk[3][8][1]*Vpk[4][8][1]))))+((IkWpk[4][8][2]*
          Wpk[3][8][2])+((IkWpk[4][8][0]*Wpk[3][8][0])+(IkWpk[4][8][1]*
          Wpk[3][8][1]))))+temp[1]);
        temp[3] = (((mk[4]*((Vpk[3][9][2]*Vpk[4][9][2])+((Vpk[3][9][0]*
          Vpk[4][9][0])+(Vpk[3][9][1]*Vpk[4][9][1]))))+((Cik[9][0][2]*
          IkWpk[4][9][2])+((Cik[9][0][0]*IkWpk[4][9][0])+(Cik[9][0][1]*
          IkWpk[4][9][1]))))+temp[2]);
        temp[4] = (((mk[5]*((Vpk[3][10][2]*Vpk[4][10][2])+((Vpk[3][10][0]*
          Vpk[4][10][0])+(Vpk[3][10][1]*Vpk[4][10][1]))))+((IkWpk[4][10][2]*
          Wpk[3][10][2])+((IkWpk[4][10][0]*Wpk[3][10][0])+(IkWpk[4][10][1]*
          Wpk[3][10][1]))))+temp[3]);
        temp[5] = (((mk[6]*((Vpk[3][11][2]*Vpk[4][11][2])+((Vpk[3][11][0]*
          Vpk[4][11][0])+(Vpk[3][11][1]*Vpk[4][11][1]))))+((IkWpk[4][11][2]*
          Wpk[3][11][2])+((IkWpk[4][11][0]*Wpk[3][11][0])+(IkWpk[4][11][1]*
          Wpk[3][11][1]))))+temp[4]);
        temp[6] = (((mk[7]*((Vpk[3][12][2]*Vpk[4][12][2])+((Vpk[3][12][0]*
          Vpk[4][12][0])+(Vpk[3][12][1]*Vpk[4][12][1]))))+((IkWpk[4][12][2]*
          Wpk[3][12][2])+((IkWpk[4][12][0]*Wpk[3][12][0])+(IkWpk[4][12][1]*
          Wpk[3][12][1]))))+temp[5]);
        temp[7] = (((mk[8]*((Vpk[3][13][2]*Vpk[4][13][2])+((Vpk[3][13][0]*
          Vpk[4][13][0])+(Vpk[3][13][1]*Vpk[4][13][1]))))+((IkWpk[4][13][2]*
          Wpk[3][13][2])+((IkWpk[4][13][0]*Wpk[3][13][0])+(IkWpk[4][13][1]*
          Wpk[3][13][1]))))+temp[6]);
        temp[8] = (((mk[9]*((Vpk[3][14][2]*Vpk[4][14][2])+((Vpk[3][14][0]*
          Vpk[4][14][0])+(Vpk[3][14][1]*Vpk[4][14][1]))))+((IkWpk[4][14][2]*
          Wpk[3][14][2])+((IkWpk[4][14][0]*Wpk[3][14][0])+(IkWpk[4][14][1]*
          Wpk[3][14][1]))))+temp[7]);
        temp[9] = (((mk[10]*((Vpk[3][15][2]*Vpk[4][15][2])+((Vpk[3][15][0]*
          Vpk[4][15][0])+(Vpk[3][15][1]*Vpk[4][15][1]))))+((Cik[15][0][2]*
          IkWpk[4][15][2])+((Cik[15][0][0]*IkWpk[4][15][0])+(Cik[15][0][1]*
          IkWpk[4][15][1]))))+temp[8]);
        temp[10] = (((mk[11]*((Vpk[3][16][2]*Vpk[4][16][2])+((Vpk[3][16][0]*
          Vpk[4][16][0])+(Vpk[3][16][1]*Vpk[4][16][1]))))+((IkWpk[4][16][2]*
          Wpk[3][16][2])+((IkWpk[4][16][0]*Wpk[3][16][0])+(IkWpk[4][16][1]*
          Wpk[3][16][1]))))+temp[9]);
        temp[11] = (((mk[12]*((Vpk[3][17][2]*Vpk[4][17][2])+((Vpk[3][17][0]*
          Vpk[4][17][0])+(Vpk[3][17][1]*Vpk[4][17][1]))))+((IkWpk[4][17][2]*
          Wpk[3][17][2])+((IkWpk[4][17][0]*Wpk[3][17][0])+(IkWpk[4][17][1]*
          Wpk[3][17][1]))))+temp[10]);
        temp[12] = (((mk[13]*((Vpk[3][18][2]*Vpk[4][18][2])+((Vpk[3][18][0]*
          Vpk[4][18][0])+(Vpk[3][18][1]*Vpk[4][18][1]))))+((IkWpk[4][18][2]*
          Wpk[3][18][2])+((IkWpk[4][18][0]*Wpk[3][18][0])+(IkWpk[4][18][1]*
          Wpk[3][18][1]))))+temp[11]);
        temp[13] = (((mk[14]*((Vpk[3][19][2]*Vpk[4][19][2])+((Vpk[3][19][0]*
          Vpk[4][19][0])+(Vpk[3][19][1]*Vpk[4][19][1]))))+((IkWpk[4][19][2]*
          Wpk[3][19][2])+((IkWpk[4][19][0]*Wpk[3][19][0])+(IkWpk[4][19][1]*
          Wpk[3][19][1]))))+temp[12]);
        mm[3][4] = (((mk[15]*((Vpk[3][20][2]*Vpk[4][20][2])+((Vpk[3][20][0]*
          Vpk[4][20][0])+(Vpk[3][20][1]*Vpk[4][20][1]))))+((IkWpk[4][20][2]*
          Wpk[3][20][2])+((IkWpk[4][20][0]*Wpk[3][20][0])+(IkWpk[4][20][1]*
          Wpk[3][20][1]))))+temp[13]);
        temp[0] = ((ik[0][0][2]-(mk[0]*(rk[0][0]*rk[0][2])))+((mk[1]*((
          Vpk[3][6][2]*Vpk[5][6][2])+((Vpk[3][6][0]*Vpk[5][6][0])+(Vpk[3][6][1]*
          Vpk[5][6][1]))))+((Cik[6][0][2]*IkWpk[5][6][2])+((Cik[6][0][0]*
          IkWpk[5][6][0])+(Cik[6][0][1]*IkWpk[5][6][1])))));
        temp[1] = (temp[0]+((mk[2]*((Vpk[3][7][2]*Vpk[5][7][2])+((Vpk[3][7][0]*
          Vpk[5][7][0])+(Vpk[3][7][1]*Vpk[5][7][1]))))+((IkWpk[5][7][2]*
          Wpk[3][7][2])+((IkWpk[5][7][0]*Wpk[3][7][0])+(IkWpk[5][7][1]*
          Wpk[3][7][1])))));
        temp[2] = (((mk[3]*((Vpk[3][8][2]*Vpk[5][8][2])+((Vpk[3][8][0]*
          Vpk[5][8][0])+(Vpk[3][8][1]*Vpk[5][8][1]))))+((IkWpk[5][8][2]*
          Wpk[3][8][2])+((IkWpk[5][8][0]*Wpk[3][8][0])+(IkWpk[5][8][1]*
          Wpk[3][8][1]))))+temp[1]);
        temp[3] = (((mk[4]*((Vpk[3][9][2]*Vpk[5][9][2])+((Vpk[3][9][0]*
          Vpk[5][9][0])+(Vpk[3][9][1]*Vpk[5][9][1]))))+((Cik[9][0][2]*
          IkWpk[5][9][2])+((Cik[9][0][0]*IkWpk[5][9][0])+(Cik[9][0][1]*
          IkWpk[5][9][1]))))+temp[2]);
        temp[4] = (((mk[5]*((Vpk[3][10][2]*Vpk[5][10][2])+((Vpk[3][10][0]*
          Vpk[5][10][0])+(Vpk[3][10][1]*Vpk[5][10][1]))))+((IkWpk[5][10][2]*
          Wpk[3][10][2])+((IkWpk[5][10][0]*Wpk[3][10][0])+(IkWpk[5][10][1]*
          Wpk[3][10][1]))))+temp[3]);
        temp[5] = (((mk[6]*((Vpk[3][11][2]*Vpk[5][11][2])+((Vpk[3][11][0]*
          Vpk[5][11][0])+(Vpk[3][11][1]*Vpk[5][11][1]))))+((IkWpk[5][11][2]*
          Wpk[3][11][2])+((IkWpk[5][11][0]*Wpk[3][11][0])+(IkWpk[5][11][1]*
          Wpk[3][11][1]))))+temp[4]);
        temp[6] = (((mk[7]*((Vpk[3][12][2]*Vpk[5][12][2])+((Vpk[3][12][0]*
          Vpk[5][12][0])+(Vpk[3][12][1]*Vpk[5][12][1]))))+((IkWpk[5][12][2]*
          Wpk[3][12][2])+((IkWpk[5][12][0]*Wpk[3][12][0])+(IkWpk[5][12][1]*
          Wpk[3][12][1]))))+temp[5]);
        temp[7] = (((mk[8]*((Vpk[3][13][2]*Vpk[5][13][2])+((Vpk[3][13][0]*
          Vpk[5][13][0])+(Vpk[3][13][1]*Vpk[5][13][1]))))+((IkWpk[5][13][2]*
          Wpk[3][13][2])+((IkWpk[5][13][0]*Wpk[3][13][0])+(IkWpk[5][13][1]*
          Wpk[3][13][1]))))+temp[6]);
        temp[8] = (((mk[9]*((Vpk[3][14][2]*Vpk[5][14][2])+((Vpk[3][14][0]*
          Vpk[5][14][0])+(Vpk[3][14][1]*Vpk[5][14][1]))))+((IkWpk[5][14][2]*
          Wpk[3][14][2])+((IkWpk[5][14][0]*Wpk[3][14][0])+(IkWpk[5][14][1]*
          Wpk[3][14][1]))))+temp[7]);
        temp[9] = (((mk[10]*((Vpk[3][15][2]*Vpk[5][15][2])+((Vpk[3][15][0]*
          Vpk[5][15][0])+(Vpk[3][15][1]*Vpk[5][15][1]))))+((Cik[15][0][2]*
          IkWpk[5][15][2])+((Cik[15][0][0]*IkWpk[5][15][0])+(Cik[15][0][1]*
          IkWpk[5][15][1]))))+temp[8]);
        temp[10] = (((mk[11]*((Vpk[3][16][2]*Vpk[5][16][2])+((Vpk[3][16][0]*
          Vpk[5][16][0])+(Vpk[3][16][1]*Vpk[5][16][1]))))+((IkWpk[5][16][2]*
          Wpk[3][16][2])+((IkWpk[5][16][0]*Wpk[3][16][0])+(IkWpk[5][16][1]*
          Wpk[3][16][1]))))+temp[9]);
        temp[11] = (((mk[12]*((Vpk[3][17][2]*Vpk[5][17][2])+((Vpk[3][17][0]*
          Vpk[5][17][0])+(Vpk[3][17][1]*Vpk[5][17][1]))))+((IkWpk[5][17][2]*
          Wpk[3][17][2])+((IkWpk[5][17][0]*Wpk[3][17][0])+(IkWpk[5][17][1]*
          Wpk[3][17][1]))))+temp[10]);
        temp[12] = (((mk[13]*((Vpk[3][18][2]*Vpk[5][18][2])+((Vpk[3][18][0]*
          Vpk[5][18][0])+(Vpk[3][18][1]*Vpk[5][18][1]))))+((IkWpk[5][18][2]*
          Wpk[3][18][2])+((IkWpk[5][18][0]*Wpk[3][18][0])+(IkWpk[5][18][1]*
          Wpk[3][18][1]))))+temp[11]);
        temp[13] = (((mk[14]*((Vpk[3][19][2]*Vpk[5][19][2])+((Vpk[3][19][0]*
          Vpk[5][19][0])+(Vpk[3][19][1]*Vpk[5][19][1]))))+((IkWpk[5][19][2]*
          Wpk[3][19][2])+((IkWpk[5][19][0]*Wpk[3][19][0])+(IkWpk[5][19][1]*
          Wpk[3][19][1]))))+temp[12]);
        mm[3][5] = (((mk[15]*((Vpk[3][20][2]*Vpk[5][20][2])+((Vpk[3][20][0]*
          Vpk[5][20][0])+(Vpk[3][20][1]*Vpk[5][20][1]))))+((IkWpk[5][20][2]*
          Wpk[3][20][2])+((IkWpk[5][20][0]*Wpk[3][20][0])+(IkWpk[5][20][1]*
          Wpk[3][20][1]))))+temp[13]);
        temp[0] = (((mk[1]*((Vpk[3][6][2]*Vpk[6][6][2])+((Vpk[3][6][0]*
          Vpk[6][6][0])+(Vpk[3][6][1]*Vpk[6][6][1]))))+((Cik[6][0][2]*
          IkWpk[6][6][2])+((Cik[6][0][0]*IkWpk[6][6][0])+(Cik[6][0][1]*
          IkWpk[6][6][1]))))+((mk[2]*((Vpk[3][7][2]*Vpk[6][7][2])+((Vpk[3][7][0]
          *Vpk[6][7][0])+(Vpk[3][7][1]*Vpk[6][7][1]))))+((IkWpk[6][7][2]*
          Wpk[3][7][2])+((IkWpk[6][7][0]*Wpk[3][7][0])+(IkWpk[6][7][1]*
          Wpk[3][7][1])))));
        mm[3][6] = (((mk[3]*((Vpk[3][8][2]*Vpk[6][8][2])+((Vpk[3][8][0]*
          Vpk[6][8][0])+(Vpk[3][8][1]*Vpk[6][8][1]))))+((IkWpk[6][8][2]*
          Wpk[3][8][2])+((IkWpk[6][8][0]*Wpk[3][8][0])+(IkWpk[6][8][1]*
          Wpk[3][8][1]))))+temp[0]);
        mm[3][7] = (((mk[2]*((Vpk[3][7][2]*Vpk[7][7][2])+((Vpk[3][7][0]*
          Vpk[7][7][0])+(Vpk[3][7][1]*Vpk[7][7][1]))))+((IkWpk[7][7][2]*
          Wpk[3][7][2])+((IkWpk[7][7][0]*Wpk[3][7][0])+(IkWpk[7][7][1]*
          Wpk[3][7][1]))))+((mk[3]*((Vpk[3][8][2]*Vpk[7][8][2])+((Vpk[3][8][0]*
          Vpk[7][8][0])+(Vpk[3][8][1]*Vpk[7][8][1]))))+((IkWpk[7][8][2]*
          Wpk[3][8][2])+((IkWpk[7][8][0]*Wpk[3][8][0])+(IkWpk[7][8][1]*
          Wpk[3][8][1])))));
        mm[3][8] = ((mk[3]*((Vpk[3][8][2]*Vpk[8][8][2])+((Vpk[3][8][0]*
          Vpk[8][8][0])+(Vpk[3][8][1]*Vpk[8][8][1]))))+((IkWpk[8][8][2]*
          Wpk[3][8][2])+((IkWpk[8][8][0]*Wpk[3][8][0])+(IkWpk[8][8][1]*
          Wpk[3][8][1]))));
        temp[0] = (((mk[4]*((Vpk[3][9][2]*Vpk[9][9][2])+((Vpk[3][9][0]*
          Vpk[9][9][0])+(Vpk[3][9][1]*Vpk[9][9][1]))))+((Cik[9][0][2]*
          IkWpk[9][9][2])+((Cik[9][0][0]*IkWpk[9][9][0])+(Cik[9][0][1]*
          IkWpk[9][9][1]))))+((mk[5]*((Vpk[3][10][2]*Vpk[9][10][2])+((
          Vpk[3][10][0]*Vpk[9][10][0])+(Vpk[3][10][1]*Vpk[9][10][1]))))+((
          IkWpk[9][10][2]*Wpk[3][10][2])+((IkWpk[9][10][0]*Wpk[3][10][0])+(
          IkWpk[9][10][1]*Wpk[3][10][1])))));
        temp[1] = (((mk[6]*((Vpk[3][11][2]*Vpk[9][11][2])+((Vpk[3][11][0]*
          Vpk[9][11][0])+(Vpk[3][11][1]*Vpk[9][11][1]))))+((IkWpk[9][11][2]*
          Wpk[3][11][2])+((IkWpk[9][11][0]*Wpk[3][11][0])+(IkWpk[9][11][1]*
          Wpk[3][11][1]))))+temp[0]);
        temp[2] = (((mk[7]*((Vpk[3][12][2]*Vpk[9][12][2])+((Vpk[3][12][0]*
          Vpk[9][12][0])+(Vpk[3][12][1]*Vpk[9][12][1]))))+((IkWpk[9][12][2]*
          Wpk[3][12][2])+((IkWpk[9][12][0]*Wpk[3][12][0])+(IkWpk[9][12][1]*
          Wpk[3][12][1]))))+temp[1]);
        temp[3] = (((mk[8]*((Vpk[3][13][2]*Vpk[9][13][2])+((Vpk[3][13][0]*
          Vpk[9][13][0])+(Vpk[3][13][1]*Vpk[9][13][1]))))+((IkWpk[9][13][2]*
          Wpk[3][13][2])+((IkWpk[9][13][0]*Wpk[3][13][0])+(IkWpk[9][13][1]*
          Wpk[3][13][1]))))+temp[2]);
        mm[3][9] = (((mk[9]*((Vpk[3][14][2]*Vpk[9][14][2])+((Vpk[3][14][0]*
          Vpk[9][14][0])+(Vpk[3][14][1]*Vpk[9][14][1]))))+((IkWpk[9][14][2]*
          Wpk[3][14][2])+((IkWpk[9][14][0]*Wpk[3][14][0])+(IkWpk[9][14][1]*
          Wpk[3][14][1]))))+temp[3]);
        temp[0] = (((mk[5]*((Vpk[3][10][2]*Vpk[10][10][2])+((Vpk[3][10][0]*
          Vpk[10][10][0])+(Vpk[3][10][1]*Vpk[10][10][1]))))+((IkWpk[10][10][2]*
          Wpk[3][10][2])+((IkWpk[10][10][0]*Wpk[3][10][0])+(IkWpk[10][10][1]*
          Wpk[3][10][1]))))+((mk[6]*((Vpk[3][11][2]*Vpk[10][11][2])+((
          Vpk[3][11][0]*Vpk[10][11][0])+(Vpk[3][11][1]*Vpk[10][11][1]))))+((
          IkWpk[10][11][2]*Wpk[3][11][2])+((IkWpk[10][11][0]*Wpk[3][11][0])+(
          IkWpk[10][11][1]*Wpk[3][11][1])))));
        temp[1] = (((mk[7]*((Vpk[3][12][2]*Vpk[10][12][2])+((Vpk[3][12][0]*
          Vpk[10][12][0])+(Vpk[3][12][1]*Vpk[10][12][1]))))+((IkWpk[10][12][2]*
          Wpk[3][12][2])+((IkWpk[10][12][0]*Wpk[3][12][0])+(IkWpk[10][12][1]*
          Wpk[3][12][1]))))+temp[0]);
        temp[2] = (((mk[8]*((Vpk[3][13][2]*Vpk[10][13][2])+((Vpk[3][13][0]*
          Vpk[10][13][0])+(Vpk[3][13][1]*Vpk[10][13][1]))))+((IkWpk[10][13][2]*
          Wpk[3][13][2])+((IkWpk[10][13][0]*Wpk[3][13][0])+(IkWpk[10][13][1]*
          Wpk[3][13][1]))))+temp[1]);
        mm[3][10] = (((mk[9]*((Vpk[3][14][2]*Vpk[10][14][2])+((Vpk[3][14][0]*
          Vpk[10][14][0])+(Vpk[3][14][1]*Vpk[10][14][1]))))+((IkWpk[10][14][2]*
          Wpk[3][14][2])+((IkWpk[10][14][0]*Wpk[3][14][0])+(IkWpk[10][14][1]*
          Wpk[3][14][1]))))+temp[2]);
        temp[0] = (((mk[6]*((Vpk[3][11][2]*Vpk[11][11][2])+((Vpk[3][11][0]*
          Vpk[11][11][0])+(Vpk[3][11][1]*Vpk[11][11][1]))))+((IkWpk[11][11][2]*
          Wpk[3][11][2])+((IkWpk[11][11][0]*Wpk[3][11][0])+(IkWpk[11][11][1]*
          Wpk[3][11][1]))))+((mk[7]*((Vpk[3][12][2]*Vpk[11][12][2])+((
          Vpk[3][12][0]*Vpk[11][12][0])+(Vpk[3][12][1]*Vpk[11][12][1]))))+((
          IkWpk[11][12][2]*Wpk[3][12][2])+((IkWpk[11][12][0]*Wpk[3][12][0])+(
          IkWpk[11][12][1]*Wpk[3][12][1])))));
        temp[1] = (((mk[8]*((Vpk[3][13][2]*Vpk[11][13][2])+((Vpk[3][13][0]*
          Vpk[11][13][0])+(Vpk[3][13][1]*Vpk[11][13][1]))))+((IkWpk[11][13][2]*
          Wpk[3][13][2])+((IkWpk[11][13][0]*Wpk[3][13][0])+(IkWpk[11][13][1]*
          Wpk[3][13][1]))))+temp[0]);
        mm[3][11] = (((mk[9]*((Vpk[3][14][2]*Vpk[11][14][2])+((Vpk[3][14][0]*
          Vpk[11][14][0])+(Vpk[3][14][1]*Vpk[11][14][1]))))+((IkWpk[11][14][2]*
          Wpk[3][14][2])+((IkWpk[11][14][0]*Wpk[3][14][0])+(IkWpk[11][14][1]*
          Wpk[3][14][1]))))+temp[1]);
        temp[0] = (((mk[7]*((Vpk[3][12][2]*Vpk[12][12][2])+((Vpk[3][12][0]*
          Vpk[12][12][0])+(Vpk[3][12][1]*Vpk[12][12][1]))))+((IkWpk[12][12][2]*
          Wpk[3][12][2])+((IkWpk[12][12][0]*Wpk[3][12][0])+(IkWpk[12][12][1]*
          Wpk[3][12][1]))))+((mk[8]*((Vpk[3][13][2]*Vpk[12][13][2])+((
          Vpk[3][13][0]*Vpk[12][13][0])+(Vpk[3][13][1]*Vpk[12][13][1]))))+((
          IkWpk[12][13][2]*Wpk[3][13][2])+((IkWpk[12][13][0]*Wpk[3][13][0])+(
          IkWpk[12][13][1]*Wpk[3][13][1])))));
        mm[3][12] = (((mk[9]*((Vpk[3][14][2]*Vpk[12][14][2])+((Vpk[3][14][0]*
          Vpk[12][14][0])+(Vpk[3][14][1]*Vpk[12][14][1]))))+((IkWpk[12][14][2]*
          Wpk[3][14][2])+((IkWpk[12][14][0]*Wpk[3][14][0])+(IkWpk[12][14][1]*
          Wpk[3][14][1]))))+temp[0]);
        mm[3][13] = (((mk[8]*((Vpk[3][13][2]*Vpk[13][13][2])+((Vpk[3][13][0]*
          Vpk[13][13][0])+(Vpk[3][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          Wpk[3][13][2])+((IkWpk[13][13][0]*Wpk[3][13][0])+(IkWpk[13][13][1]*
          Wpk[3][13][1]))))+((mk[9]*((Vpk[3][14][2]*Vpk[13][14][2])+((
          Vpk[3][14][0]*Vpk[13][14][0])+(Vpk[3][14][1]*Vpk[13][14][1]))))+((
          IkWpk[13][14][2]*Wpk[3][14][2])+((IkWpk[13][14][0]*Wpk[3][14][0])+(
          IkWpk[13][14][1]*Wpk[3][14][1])))));
        mm[3][14] = ((mk[9]*((Vpk[3][14][2]*Vpk[14][14][2])+((Vpk[3][14][0]*
          Vpk[14][14][0])+(Vpk[3][14][1]*Vpk[14][14][1]))))+((IkWpk[14][14][2]*
          Wpk[3][14][2])+((IkWpk[14][14][0]*Wpk[3][14][0])+(IkWpk[14][14][1]*
          Wpk[3][14][1]))));
        temp[0] = (((mk[10]*((Vpk[3][15][2]*Vpk[15][15][2])+((Vpk[3][15][0]*
          Vpk[15][15][0])+(Vpk[3][15][1]*Vpk[15][15][1]))))+((Cik[15][0][2]*
          IkWpk[15][15][2])+((Cik[15][0][0]*IkWpk[15][15][0])+(Cik[15][0][1]*
          IkWpk[15][15][1]))))+((mk[11]*((Vpk[3][16][2]*Vpk[15][16][2])+((
          Vpk[3][16][0]*Vpk[15][16][0])+(Vpk[3][16][1]*Vpk[15][16][1]))))+((
          IkWpk[15][16][2]*Wpk[3][16][2])+((IkWpk[15][16][0]*Wpk[3][16][0])+(
          IkWpk[15][16][1]*Wpk[3][16][1])))));
        temp[1] = (((mk[12]*((Vpk[3][17][2]*Vpk[15][17][2])+((Vpk[3][17][0]*
          Vpk[15][17][0])+(Vpk[3][17][1]*Vpk[15][17][1]))))+((IkWpk[15][17][2]*
          Wpk[3][17][2])+((IkWpk[15][17][0]*Wpk[3][17][0])+(IkWpk[15][17][1]*
          Wpk[3][17][1]))))+temp[0]);
        temp[2] = (((mk[13]*((Vpk[3][18][2]*Vpk[15][18][2])+((Vpk[3][18][0]*
          Vpk[15][18][0])+(Vpk[3][18][1]*Vpk[15][18][1]))))+((IkWpk[15][18][2]*
          Wpk[3][18][2])+((IkWpk[15][18][0]*Wpk[3][18][0])+(IkWpk[15][18][1]*
          Wpk[3][18][1]))))+temp[1]);
        temp[3] = (((mk[14]*((Vpk[3][19][2]*Vpk[15][19][2])+((Vpk[3][19][0]*
          Vpk[15][19][0])+(Vpk[3][19][1]*Vpk[15][19][1]))))+((IkWpk[15][19][2]*
          Wpk[3][19][2])+((IkWpk[15][19][0]*Wpk[3][19][0])+(IkWpk[15][19][1]*
          Wpk[3][19][1]))))+temp[2]);
        mm[3][15] = (((mk[15]*((Vpk[3][20][2]*Vpk[15][20][2])+((Vpk[3][20][0]*
          Vpk[15][20][0])+(Vpk[3][20][1]*Vpk[15][20][1]))))+((IkWpk[15][20][2]*
          Wpk[3][20][2])+((IkWpk[15][20][0]*Wpk[3][20][0])+(IkWpk[15][20][1]*
          Wpk[3][20][1]))))+temp[3]);
        temp[0] = (((mk[11]*((Vpk[3][16][2]*Vpk[16][16][2])+((Vpk[3][16][0]*
          Vpk[16][16][0])+(Vpk[3][16][1]*Vpk[16][16][1]))))+((IkWpk[16][16][2]*
          Wpk[3][16][2])+((IkWpk[16][16][0]*Wpk[3][16][0])+(IkWpk[16][16][1]*
          Wpk[3][16][1]))))+((mk[12]*((Vpk[3][17][2]*Vpk[16][17][2])+((
          Vpk[3][17][0]*Vpk[16][17][0])+(Vpk[3][17][1]*Vpk[16][17][1]))))+((
          IkWpk[16][17][2]*Wpk[3][17][2])+((IkWpk[16][17][0]*Wpk[3][17][0])+(
          IkWpk[16][17][1]*Wpk[3][17][1])))));
        temp[1] = (((mk[13]*((Vpk[3][18][2]*Vpk[16][18][2])+((Vpk[3][18][0]*
          Vpk[16][18][0])+(Vpk[3][18][1]*Vpk[16][18][1]))))+((IkWpk[16][18][2]*
          Wpk[3][18][2])+((IkWpk[16][18][0]*Wpk[3][18][0])+(IkWpk[16][18][1]*
          Wpk[3][18][1]))))+temp[0]);
        temp[2] = (((mk[14]*((Vpk[3][19][2]*Vpk[16][19][2])+((Vpk[3][19][0]*
          Vpk[16][19][0])+(Vpk[3][19][1]*Vpk[16][19][1]))))+((IkWpk[16][19][2]*
          Wpk[3][19][2])+((IkWpk[16][19][0]*Wpk[3][19][0])+(IkWpk[16][19][1]*
          Wpk[3][19][1]))))+temp[1]);
        mm[3][16] = (((mk[15]*((Vpk[3][20][2]*Vpk[16][20][2])+((Vpk[3][20][0]*
          Vpk[16][20][0])+(Vpk[3][20][1]*Vpk[16][20][1]))))+((IkWpk[16][20][2]*
          Wpk[3][20][2])+((IkWpk[16][20][0]*Wpk[3][20][0])+(IkWpk[16][20][1]*
          Wpk[3][20][1]))))+temp[2]);
        temp[0] = (((mk[12]*((Vpk[3][17][2]*Vpk[17][17][2])+((Vpk[3][17][0]*
          Vpk[17][17][0])+(Vpk[3][17][1]*Vpk[17][17][1]))))+((IkWpk[17][17][2]*
          Wpk[3][17][2])+((IkWpk[17][17][0]*Wpk[3][17][0])+(IkWpk[17][17][1]*
          Wpk[3][17][1]))))+((mk[13]*((Vpk[3][18][2]*Vpk[17][18][2])+((
          Vpk[3][18][0]*Vpk[17][18][0])+(Vpk[3][18][1]*Vpk[17][18][1]))))+((
          IkWpk[17][18][2]*Wpk[3][18][2])+((IkWpk[17][18][0]*Wpk[3][18][0])+(
          IkWpk[17][18][1]*Wpk[3][18][1])))));
        temp[1] = (((mk[14]*((Vpk[3][19][2]*Vpk[17][19][2])+((Vpk[3][19][0]*
          Vpk[17][19][0])+(Vpk[3][19][1]*Vpk[17][19][1]))))+((IkWpk[17][19][2]*
          Wpk[3][19][2])+((IkWpk[17][19][0]*Wpk[3][19][0])+(IkWpk[17][19][1]*
          Wpk[3][19][1]))))+temp[0]);
        mm[3][17] = (((mk[15]*((Vpk[3][20][2]*Vpk[17][20][2])+((Vpk[3][20][0]*
          Vpk[17][20][0])+(Vpk[3][20][1]*Vpk[17][20][1]))))+((IkWpk[17][20][2]*
          Wpk[3][20][2])+((IkWpk[17][20][0]*Wpk[3][20][0])+(IkWpk[17][20][1]*
          Wpk[3][20][1]))))+temp[1]);
        temp[0] = (((mk[13]*((Vpk[3][18][2]*Vpk[18][18][2])+((Vpk[3][18][0]*
          Vpk[18][18][0])+(Vpk[3][18][1]*Vpk[18][18][1]))))+((IkWpk[18][18][2]*
          Wpk[3][18][2])+((IkWpk[18][18][0]*Wpk[3][18][0])+(IkWpk[18][18][1]*
          Wpk[3][18][1]))))+((mk[14]*((Vpk[3][19][2]*Vpk[18][19][2])+((
          Vpk[3][19][0]*Vpk[18][19][0])+(Vpk[3][19][1]*Vpk[18][19][1]))))+((
          IkWpk[18][19][2]*Wpk[3][19][2])+((IkWpk[18][19][0]*Wpk[3][19][0])+(
          IkWpk[18][19][1]*Wpk[3][19][1])))));
        mm[3][18] = (((mk[15]*((Vpk[3][20][2]*Vpk[18][20][2])+((Vpk[3][20][0]*
          Vpk[18][20][0])+(Vpk[3][20][1]*Vpk[18][20][1]))))+((IkWpk[18][20][2]*
          Wpk[3][20][2])+((IkWpk[18][20][0]*Wpk[3][20][0])+(IkWpk[18][20][1]*
          Wpk[3][20][1]))))+temp[0]);
        mm[3][19] = (((mk[14]*((Vpk[3][19][2]*Vpk[19][19][2])+((Vpk[3][19][0]*
          Vpk[19][19][0])+(Vpk[3][19][1]*Vpk[19][19][1]))))+((IkWpk[19][19][2]*
          Wpk[3][19][2])+((IkWpk[19][19][0]*Wpk[3][19][0])+(IkWpk[19][19][1]*
          Wpk[3][19][1]))))+((mk[15]*((Vpk[3][20][2]*Vpk[19][20][2])+((
          Vpk[3][20][0]*Vpk[19][20][0])+(Vpk[3][20][1]*Vpk[19][20][1]))))+((
          IkWpk[19][20][2]*Wpk[3][20][2])+((IkWpk[19][20][0]*Wpk[3][20][0])+(
          IkWpk[19][20][1]*Wpk[3][20][1])))));
        mm[3][20] = ((mk[15]*((Vpk[3][20][2]*Vpk[20][20][2])+((Vpk[3][20][0]*
          Vpk[20][20][0])+(Vpk[3][20][1]*Vpk[20][20][1]))))+((IkWpk[20][20][2]*
          Wpk[3][20][2])+((IkWpk[20][20][0]*Wpk[3][20][0])+(IkWpk[20][20][1]*
          Wpk[3][20][1]))));
        temp[0] = ((ik[0][1][1]+(mk[0]*((rk[0][0]*rk[0][0])+(rk[0][2]*rk[0][2]))
          ))+((mk[1]*((Vpk[4][6][2]*Vpk[4][6][2])+((Vpk[4][6][0]*Vpk[4][6][0])+(
          Vpk[4][6][1]*Vpk[4][6][1]))))+((Cik[6][1][2]*IkWpk[4][6][2])+((
          Cik[6][1][0]*IkWpk[4][6][0])+(Cik[6][1][1]*IkWpk[4][6][1])))));
        temp[1] = (temp[0]+((mk[2]*((Vpk[4][7][2]*Vpk[4][7][2])+((Vpk[4][7][0]*
          Vpk[4][7][0])+(Vpk[4][7][1]*Vpk[4][7][1]))))+((IkWpk[4][7][2]*
          Wpk[4][7][2])+((IkWpk[4][7][0]*Wpk[4][7][0])+(IkWpk[4][7][1]*
          Wpk[4][7][1])))));
        temp[2] = (((mk[3]*((Vpk[4][8][2]*Vpk[4][8][2])+((Vpk[4][8][0]*
          Vpk[4][8][0])+(Vpk[4][8][1]*Vpk[4][8][1]))))+((IkWpk[4][8][2]*
          Wpk[4][8][2])+((IkWpk[4][8][0]*Wpk[4][8][0])+(IkWpk[4][8][1]*
          Wpk[4][8][1]))))+temp[1]);
        temp[3] = (((mk[4]*((Vpk[4][9][2]*Vpk[4][9][2])+((Vpk[4][9][0]*
          Vpk[4][9][0])+(Vpk[4][9][1]*Vpk[4][9][1]))))+((Cik[9][1][2]*
          IkWpk[4][9][2])+((Cik[9][1][0]*IkWpk[4][9][0])+(Cik[9][1][1]*
          IkWpk[4][9][1]))))+temp[2]);
        temp[4] = (((mk[5]*((Vpk[4][10][2]*Vpk[4][10][2])+((Vpk[4][10][0]*
          Vpk[4][10][0])+(Vpk[4][10][1]*Vpk[4][10][1]))))+((IkWpk[4][10][2]*
          Wpk[4][10][2])+((IkWpk[4][10][0]*Wpk[4][10][0])+(IkWpk[4][10][1]*
          Wpk[4][10][1]))))+temp[3]);
        temp[5] = (((mk[6]*((Vpk[4][11][2]*Vpk[4][11][2])+((Vpk[4][11][0]*
          Vpk[4][11][0])+(Vpk[4][11][1]*Vpk[4][11][1]))))+((IkWpk[4][11][2]*
          Wpk[4][11][2])+((IkWpk[4][11][0]*Wpk[4][11][0])+(IkWpk[4][11][1]*
          Wpk[4][11][1]))))+temp[4]);
        temp[6] = (((mk[7]*((Vpk[4][12][2]*Vpk[4][12][2])+((Vpk[4][12][0]*
          Vpk[4][12][0])+(Vpk[4][12][1]*Vpk[4][12][1]))))+((IkWpk[4][12][2]*
          Wpk[4][12][2])+((IkWpk[4][12][0]*Wpk[4][12][0])+(IkWpk[4][12][1]*
          Wpk[4][12][1]))))+temp[5]);
        temp[7] = (((mk[8]*((Vpk[4][13][2]*Vpk[4][13][2])+((Vpk[4][13][0]*
          Vpk[4][13][0])+(Vpk[4][13][1]*Vpk[4][13][1]))))+((IkWpk[4][13][2]*
          Wpk[4][13][2])+((IkWpk[4][13][0]*Wpk[4][13][0])+(IkWpk[4][13][1]*
          Wpk[4][13][1]))))+temp[6]);
        temp[8] = (((mk[9]*((Vpk[4][14][2]*Vpk[4][14][2])+((Vpk[4][14][0]*
          Vpk[4][14][0])+(Vpk[4][14][1]*Vpk[4][14][1]))))+((IkWpk[4][14][2]*
          Wpk[4][14][2])+((IkWpk[4][14][0]*Wpk[4][14][0])+(IkWpk[4][14][1]*
          Wpk[4][14][1]))))+temp[7]);
        temp[9] = (((mk[10]*((Vpk[4][15][2]*Vpk[4][15][2])+((Vpk[4][15][0]*
          Vpk[4][15][0])+(Vpk[4][15][1]*Vpk[4][15][1]))))+((Cik[15][1][2]*
          IkWpk[4][15][2])+((Cik[15][1][0]*IkWpk[4][15][0])+(Cik[15][1][1]*
          IkWpk[4][15][1]))))+temp[8]);
        temp[10] = (((mk[11]*((Vpk[4][16][2]*Vpk[4][16][2])+((Vpk[4][16][0]*
          Vpk[4][16][0])+(Vpk[4][16][1]*Vpk[4][16][1]))))+((IkWpk[4][16][2]*
          Wpk[4][16][2])+((IkWpk[4][16][0]*Wpk[4][16][0])+(IkWpk[4][16][1]*
          Wpk[4][16][1]))))+temp[9]);
        temp[11] = (((mk[12]*((Vpk[4][17][2]*Vpk[4][17][2])+((Vpk[4][17][0]*
          Vpk[4][17][0])+(Vpk[4][17][1]*Vpk[4][17][1]))))+((IkWpk[4][17][2]*
          Wpk[4][17][2])+((IkWpk[4][17][0]*Wpk[4][17][0])+(IkWpk[4][17][1]*
          Wpk[4][17][1]))))+temp[10]);
        temp[12] = (((mk[13]*((Vpk[4][18][2]*Vpk[4][18][2])+((Vpk[4][18][0]*
          Vpk[4][18][0])+(Vpk[4][18][1]*Vpk[4][18][1]))))+((IkWpk[4][18][2]*
          Wpk[4][18][2])+((IkWpk[4][18][0]*Wpk[4][18][0])+(IkWpk[4][18][1]*
          Wpk[4][18][1]))))+temp[11]);
        temp[13] = (((mk[14]*((Vpk[4][19][2]*Vpk[4][19][2])+((Vpk[4][19][0]*
          Vpk[4][19][0])+(Vpk[4][19][1]*Vpk[4][19][1]))))+((IkWpk[4][19][2]*
          Wpk[4][19][2])+((IkWpk[4][19][0]*Wpk[4][19][0])+(IkWpk[4][19][1]*
          Wpk[4][19][1]))))+temp[12]);
        mm[4][4] = (((mk[15]*((Vpk[4][20][2]*Vpk[4][20][2])+((Vpk[4][20][0]*
          Vpk[4][20][0])+(Vpk[4][20][1]*Vpk[4][20][1]))))+((IkWpk[4][20][2]*
          Wpk[4][20][2])+((IkWpk[4][20][0]*Wpk[4][20][0])+(IkWpk[4][20][1]*
          Wpk[4][20][1]))))+temp[13]);
        temp[0] = ((ik[0][1][2]-(mk[0]*(rk[0][1]*rk[0][2])))+((mk[1]*((
          Vpk[4][6][2]*Vpk[5][6][2])+((Vpk[4][6][0]*Vpk[5][6][0])+(Vpk[4][6][1]*
          Vpk[5][6][1]))))+((Cik[6][1][2]*IkWpk[5][6][2])+((Cik[6][1][0]*
          IkWpk[5][6][0])+(Cik[6][1][1]*IkWpk[5][6][1])))));
        temp[1] = (temp[0]+((mk[2]*((Vpk[4][7][2]*Vpk[5][7][2])+((Vpk[4][7][0]*
          Vpk[5][7][0])+(Vpk[4][7][1]*Vpk[5][7][1]))))+((IkWpk[5][7][2]*
          Wpk[4][7][2])+((IkWpk[5][7][0]*Wpk[4][7][0])+(IkWpk[5][7][1]*
          Wpk[4][7][1])))));
        temp[2] = (((mk[3]*((Vpk[4][8][2]*Vpk[5][8][2])+((Vpk[4][8][0]*
          Vpk[5][8][0])+(Vpk[4][8][1]*Vpk[5][8][1]))))+((IkWpk[5][8][2]*
          Wpk[4][8][2])+((IkWpk[5][8][0]*Wpk[4][8][0])+(IkWpk[5][8][1]*
          Wpk[4][8][1]))))+temp[1]);
        temp[3] = (((mk[4]*((Vpk[4][9][2]*Vpk[5][9][2])+((Vpk[4][9][0]*
          Vpk[5][9][0])+(Vpk[4][9][1]*Vpk[5][9][1]))))+((Cik[9][1][2]*
          IkWpk[5][9][2])+((Cik[9][1][0]*IkWpk[5][9][0])+(Cik[9][1][1]*
          IkWpk[5][9][1]))))+temp[2]);
        temp[4] = (((mk[5]*((Vpk[4][10][2]*Vpk[5][10][2])+((Vpk[4][10][0]*
          Vpk[5][10][0])+(Vpk[4][10][1]*Vpk[5][10][1]))))+((IkWpk[5][10][2]*
          Wpk[4][10][2])+((IkWpk[5][10][0]*Wpk[4][10][0])+(IkWpk[5][10][1]*
          Wpk[4][10][1]))))+temp[3]);
        temp[5] = (((mk[6]*((Vpk[4][11][2]*Vpk[5][11][2])+((Vpk[4][11][0]*
          Vpk[5][11][0])+(Vpk[4][11][1]*Vpk[5][11][1]))))+((IkWpk[5][11][2]*
          Wpk[4][11][2])+((IkWpk[5][11][0]*Wpk[4][11][0])+(IkWpk[5][11][1]*
          Wpk[4][11][1]))))+temp[4]);
        temp[6] = (((mk[7]*((Vpk[4][12][2]*Vpk[5][12][2])+((Vpk[4][12][0]*
          Vpk[5][12][0])+(Vpk[4][12][1]*Vpk[5][12][1]))))+((IkWpk[5][12][2]*
          Wpk[4][12][2])+((IkWpk[5][12][0]*Wpk[4][12][0])+(IkWpk[5][12][1]*
          Wpk[4][12][1]))))+temp[5]);
        temp[7] = (((mk[8]*((Vpk[4][13][2]*Vpk[5][13][2])+((Vpk[4][13][0]*
          Vpk[5][13][0])+(Vpk[4][13][1]*Vpk[5][13][1]))))+((IkWpk[5][13][2]*
          Wpk[4][13][2])+((IkWpk[5][13][0]*Wpk[4][13][0])+(IkWpk[5][13][1]*
          Wpk[4][13][1]))))+temp[6]);
        temp[8] = (((mk[9]*((Vpk[4][14][2]*Vpk[5][14][2])+((Vpk[4][14][0]*
          Vpk[5][14][0])+(Vpk[4][14][1]*Vpk[5][14][1]))))+((IkWpk[5][14][2]*
          Wpk[4][14][2])+((IkWpk[5][14][0]*Wpk[4][14][0])+(IkWpk[5][14][1]*
          Wpk[4][14][1]))))+temp[7]);
        temp[9] = (((mk[10]*((Vpk[4][15][2]*Vpk[5][15][2])+((Vpk[4][15][0]*
          Vpk[5][15][0])+(Vpk[4][15][1]*Vpk[5][15][1]))))+((Cik[15][1][2]*
          IkWpk[5][15][2])+((Cik[15][1][0]*IkWpk[5][15][0])+(Cik[15][1][1]*
          IkWpk[5][15][1]))))+temp[8]);
        temp[10] = (((mk[11]*((Vpk[4][16][2]*Vpk[5][16][2])+((Vpk[4][16][0]*
          Vpk[5][16][0])+(Vpk[4][16][1]*Vpk[5][16][1]))))+((IkWpk[5][16][2]*
          Wpk[4][16][2])+((IkWpk[5][16][0]*Wpk[4][16][0])+(IkWpk[5][16][1]*
          Wpk[4][16][1]))))+temp[9]);
        temp[11] = (((mk[12]*((Vpk[4][17][2]*Vpk[5][17][2])+((Vpk[4][17][0]*
          Vpk[5][17][0])+(Vpk[4][17][1]*Vpk[5][17][1]))))+((IkWpk[5][17][2]*
          Wpk[4][17][2])+((IkWpk[5][17][0]*Wpk[4][17][0])+(IkWpk[5][17][1]*
          Wpk[4][17][1]))))+temp[10]);
        temp[12] = (((mk[13]*((Vpk[4][18][2]*Vpk[5][18][2])+((Vpk[4][18][0]*
          Vpk[5][18][0])+(Vpk[4][18][1]*Vpk[5][18][1]))))+((IkWpk[5][18][2]*
          Wpk[4][18][2])+((IkWpk[5][18][0]*Wpk[4][18][0])+(IkWpk[5][18][1]*
          Wpk[4][18][1]))))+temp[11]);
        temp[13] = (((mk[14]*((Vpk[4][19][2]*Vpk[5][19][2])+((Vpk[4][19][0]*
          Vpk[5][19][0])+(Vpk[4][19][1]*Vpk[5][19][1]))))+((IkWpk[5][19][2]*
          Wpk[4][19][2])+((IkWpk[5][19][0]*Wpk[4][19][0])+(IkWpk[5][19][1]*
          Wpk[4][19][1]))))+temp[12]);
        mm[4][5] = (((mk[15]*((Vpk[4][20][2]*Vpk[5][20][2])+((Vpk[4][20][0]*
          Vpk[5][20][0])+(Vpk[4][20][1]*Vpk[5][20][1]))))+((IkWpk[5][20][2]*
          Wpk[4][20][2])+((IkWpk[5][20][0]*Wpk[4][20][0])+(IkWpk[5][20][1]*
          Wpk[4][20][1]))))+temp[13]);
        temp[0] = (((mk[1]*((Vpk[4][6][2]*Vpk[6][6][2])+((Vpk[4][6][0]*
          Vpk[6][6][0])+(Vpk[4][6][1]*Vpk[6][6][1]))))+((Cik[6][1][2]*
          IkWpk[6][6][2])+((Cik[6][1][0]*IkWpk[6][6][0])+(Cik[6][1][1]*
          IkWpk[6][6][1]))))+((mk[2]*((Vpk[4][7][2]*Vpk[6][7][2])+((Vpk[4][7][0]
          *Vpk[6][7][0])+(Vpk[4][7][1]*Vpk[6][7][1]))))+((IkWpk[6][7][2]*
          Wpk[4][7][2])+((IkWpk[6][7][0]*Wpk[4][7][0])+(IkWpk[6][7][1]*
          Wpk[4][7][1])))));
        mm[4][6] = (((mk[3]*((Vpk[4][8][2]*Vpk[6][8][2])+((Vpk[4][8][0]*
          Vpk[6][8][0])+(Vpk[4][8][1]*Vpk[6][8][1]))))+((IkWpk[6][8][2]*
          Wpk[4][8][2])+((IkWpk[6][8][0]*Wpk[4][8][0])+(IkWpk[6][8][1]*
          Wpk[4][8][1]))))+temp[0]);
        mm[4][7] = (((mk[2]*((Vpk[4][7][2]*Vpk[7][7][2])+((Vpk[4][7][0]*
          Vpk[7][7][0])+(Vpk[4][7][1]*Vpk[7][7][1]))))+((IkWpk[7][7][2]*
          Wpk[4][7][2])+((IkWpk[7][7][0]*Wpk[4][7][0])+(IkWpk[7][7][1]*
          Wpk[4][7][1]))))+((mk[3]*((Vpk[4][8][2]*Vpk[7][8][2])+((Vpk[4][8][0]*
          Vpk[7][8][0])+(Vpk[4][8][1]*Vpk[7][8][1]))))+((IkWpk[7][8][2]*
          Wpk[4][8][2])+((IkWpk[7][8][0]*Wpk[4][8][0])+(IkWpk[7][8][1]*
          Wpk[4][8][1])))));
        mm[4][8] = ((mk[3]*((Vpk[4][8][2]*Vpk[8][8][2])+((Vpk[4][8][0]*
          Vpk[8][8][0])+(Vpk[4][8][1]*Vpk[8][8][1]))))+((IkWpk[8][8][2]*
          Wpk[4][8][2])+((IkWpk[8][8][0]*Wpk[4][8][0])+(IkWpk[8][8][1]*
          Wpk[4][8][1]))));
        temp[0] = (((mk[4]*((Vpk[4][9][2]*Vpk[9][9][2])+((Vpk[4][9][0]*
          Vpk[9][9][0])+(Vpk[4][9][1]*Vpk[9][9][1]))))+((Cik[9][1][2]*
          IkWpk[9][9][2])+((Cik[9][1][0]*IkWpk[9][9][0])+(Cik[9][1][1]*
          IkWpk[9][9][1]))))+((mk[5]*((Vpk[4][10][2]*Vpk[9][10][2])+((
          Vpk[4][10][0]*Vpk[9][10][0])+(Vpk[4][10][1]*Vpk[9][10][1]))))+((
          IkWpk[9][10][2]*Wpk[4][10][2])+((IkWpk[9][10][0]*Wpk[4][10][0])+(
          IkWpk[9][10][1]*Wpk[4][10][1])))));
        temp[1] = (((mk[6]*((Vpk[4][11][2]*Vpk[9][11][2])+((Vpk[4][11][0]*
          Vpk[9][11][0])+(Vpk[4][11][1]*Vpk[9][11][1]))))+((IkWpk[9][11][2]*
          Wpk[4][11][2])+((IkWpk[9][11][0]*Wpk[4][11][0])+(IkWpk[9][11][1]*
          Wpk[4][11][1]))))+temp[0]);
        temp[2] = (((mk[7]*((Vpk[4][12][2]*Vpk[9][12][2])+((Vpk[4][12][0]*
          Vpk[9][12][0])+(Vpk[4][12][1]*Vpk[9][12][1]))))+((IkWpk[9][12][2]*
          Wpk[4][12][2])+((IkWpk[9][12][0]*Wpk[4][12][0])+(IkWpk[9][12][1]*
          Wpk[4][12][1]))))+temp[1]);
        temp[3] = (((mk[8]*((Vpk[4][13][2]*Vpk[9][13][2])+((Vpk[4][13][0]*
          Vpk[9][13][0])+(Vpk[4][13][1]*Vpk[9][13][1]))))+((IkWpk[9][13][2]*
          Wpk[4][13][2])+((IkWpk[9][13][0]*Wpk[4][13][0])+(IkWpk[9][13][1]*
          Wpk[4][13][1]))))+temp[2]);
        mm[4][9] = (((mk[9]*((Vpk[4][14][2]*Vpk[9][14][2])+((Vpk[4][14][0]*
          Vpk[9][14][0])+(Vpk[4][14][1]*Vpk[9][14][1]))))+((IkWpk[9][14][2]*
          Wpk[4][14][2])+((IkWpk[9][14][0]*Wpk[4][14][0])+(IkWpk[9][14][1]*
          Wpk[4][14][1]))))+temp[3]);
        temp[0] = (((mk[5]*((Vpk[4][10][2]*Vpk[10][10][2])+((Vpk[4][10][0]*
          Vpk[10][10][0])+(Vpk[4][10][1]*Vpk[10][10][1]))))+((IkWpk[10][10][2]*
          Wpk[4][10][2])+((IkWpk[10][10][0]*Wpk[4][10][0])+(IkWpk[10][10][1]*
          Wpk[4][10][1]))))+((mk[6]*((Vpk[4][11][2]*Vpk[10][11][2])+((
          Vpk[4][11][0]*Vpk[10][11][0])+(Vpk[4][11][1]*Vpk[10][11][1]))))+((
          IkWpk[10][11][2]*Wpk[4][11][2])+((IkWpk[10][11][0]*Wpk[4][11][0])+(
          IkWpk[10][11][1]*Wpk[4][11][1])))));
        temp[1] = (((mk[7]*((Vpk[4][12][2]*Vpk[10][12][2])+((Vpk[4][12][0]*
          Vpk[10][12][0])+(Vpk[4][12][1]*Vpk[10][12][1]))))+((IkWpk[10][12][2]*
          Wpk[4][12][2])+((IkWpk[10][12][0]*Wpk[4][12][0])+(IkWpk[10][12][1]*
          Wpk[4][12][1]))))+temp[0]);
        temp[2] = (((mk[8]*((Vpk[4][13][2]*Vpk[10][13][2])+((Vpk[4][13][0]*
          Vpk[10][13][0])+(Vpk[4][13][1]*Vpk[10][13][1]))))+((IkWpk[10][13][2]*
          Wpk[4][13][2])+((IkWpk[10][13][0]*Wpk[4][13][0])+(IkWpk[10][13][1]*
          Wpk[4][13][1]))))+temp[1]);
        mm[4][10] = (((mk[9]*((Vpk[4][14][2]*Vpk[10][14][2])+((Vpk[4][14][0]*
          Vpk[10][14][0])+(Vpk[4][14][1]*Vpk[10][14][1]))))+((IkWpk[10][14][2]*
          Wpk[4][14][2])+((IkWpk[10][14][0]*Wpk[4][14][0])+(IkWpk[10][14][1]*
          Wpk[4][14][1]))))+temp[2]);
        temp[0] = (((mk[6]*((Vpk[4][11][2]*Vpk[11][11][2])+((Vpk[4][11][0]*
          Vpk[11][11][0])+(Vpk[4][11][1]*Vpk[11][11][1]))))+((IkWpk[11][11][2]*
          Wpk[4][11][2])+((IkWpk[11][11][0]*Wpk[4][11][0])+(IkWpk[11][11][1]*
          Wpk[4][11][1]))))+((mk[7]*((Vpk[4][12][2]*Vpk[11][12][2])+((
          Vpk[4][12][0]*Vpk[11][12][0])+(Vpk[4][12][1]*Vpk[11][12][1]))))+((
          IkWpk[11][12][2]*Wpk[4][12][2])+((IkWpk[11][12][0]*Wpk[4][12][0])+(
          IkWpk[11][12][1]*Wpk[4][12][1])))));
        temp[1] = (((mk[8]*((Vpk[4][13][2]*Vpk[11][13][2])+((Vpk[4][13][0]*
          Vpk[11][13][0])+(Vpk[4][13][1]*Vpk[11][13][1]))))+((IkWpk[11][13][2]*
          Wpk[4][13][2])+((IkWpk[11][13][0]*Wpk[4][13][0])+(IkWpk[11][13][1]*
          Wpk[4][13][1]))))+temp[0]);
        mm[4][11] = (((mk[9]*((Vpk[4][14][2]*Vpk[11][14][2])+((Vpk[4][14][0]*
          Vpk[11][14][0])+(Vpk[4][14][1]*Vpk[11][14][1]))))+((IkWpk[11][14][2]*
          Wpk[4][14][2])+((IkWpk[11][14][0]*Wpk[4][14][0])+(IkWpk[11][14][1]*
          Wpk[4][14][1]))))+temp[1]);
        temp[0] = (((mk[7]*((Vpk[4][12][2]*Vpk[12][12][2])+((Vpk[4][12][0]*
          Vpk[12][12][0])+(Vpk[4][12][1]*Vpk[12][12][1]))))+((IkWpk[12][12][2]*
          Wpk[4][12][2])+((IkWpk[12][12][0]*Wpk[4][12][0])+(IkWpk[12][12][1]*
          Wpk[4][12][1]))))+((mk[8]*((Vpk[4][13][2]*Vpk[12][13][2])+((
          Vpk[4][13][0]*Vpk[12][13][0])+(Vpk[4][13][1]*Vpk[12][13][1]))))+((
          IkWpk[12][13][2]*Wpk[4][13][2])+((IkWpk[12][13][0]*Wpk[4][13][0])+(
          IkWpk[12][13][1]*Wpk[4][13][1])))));
        mm[4][12] = (((mk[9]*((Vpk[4][14][2]*Vpk[12][14][2])+((Vpk[4][14][0]*
          Vpk[12][14][0])+(Vpk[4][14][1]*Vpk[12][14][1]))))+((IkWpk[12][14][2]*
          Wpk[4][14][2])+((IkWpk[12][14][0]*Wpk[4][14][0])+(IkWpk[12][14][1]*
          Wpk[4][14][1]))))+temp[0]);
        mm[4][13] = (((mk[8]*((Vpk[4][13][2]*Vpk[13][13][2])+((Vpk[4][13][0]*
          Vpk[13][13][0])+(Vpk[4][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          Wpk[4][13][2])+((IkWpk[13][13][0]*Wpk[4][13][0])+(IkWpk[13][13][1]*
          Wpk[4][13][1]))))+((mk[9]*((Vpk[4][14][2]*Vpk[13][14][2])+((
          Vpk[4][14][0]*Vpk[13][14][0])+(Vpk[4][14][1]*Vpk[13][14][1]))))+((
          IkWpk[13][14][2]*Wpk[4][14][2])+((IkWpk[13][14][0]*Wpk[4][14][0])+(
          IkWpk[13][14][1]*Wpk[4][14][1])))));
        mm[4][14] = ((mk[9]*((Vpk[4][14][2]*Vpk[14][14][2])+((Vpk[4][14][0]*
          Vpk[14][14][0])+(Vpk[4][14][1]*Vpk[14][14][1]))))+((IkWpk[14][14][2]*
          Wpk[4][14][2])+((IkWpk[14][14][0]*Wpk[4][14][0])+(IkWpk[14][14][1]*
          Wpk[4][14][1]))));
        temp[0] = (((mk[10]*((Vpk[4][15][2]*Vpk[15][15][2])+((Vpk[4][15][0]*
          Vpk[15][15][0])+(Vpk[4][15][1]*Vpk[15][15][1]))))+((Cik[15][1][2]*
          IkWpk[15][15][2])+((Cik[15][1][0]*IkWpk[15][15][0])+(Cik[15][1][1]*
          IkWpk[15][15][1]))))+((mk[11]*((Vpk[4][16][2]*Vpk[15][16][2])+((
          Vpk[4][16][0]*Vpk[15][16][0])+(Vpk[4][16][1]*Vpk[15][16][1]))))+((
          IkWpk[15][16][2]*Wpk[4][16][2])+((IkWpk[15][16][0]*Wpk[4][16][0])+(
          IkWpk[15][16][1]*Wpk[4][16][1])))));
        temp[1] = (((mk[12]*((Vpk[4][17][2]*Vpk[15][17][2])+((Vpk[4][17][0]*
          Vpk[15][17][0])+(Vpk[4][17][1]*Vpk[15][17][1]))))+((IkWpk[15][17][2]*
          Wpk[4][17][2])+((IkWpk[15][17][0]*Wpk[4][17][0])+(IkWpk[15][17][1]*
          Wpk[4][17][1]))))+temp[0]);
        temp[2] = (((mk[13]*((Vpk[4][18][2]*Vpk[15][18][2])+((Vpk[4][18][0]*
          Vpk[15][18][0])+(Vpk[4][18][1]*Vpk[15][18][1]))))+((IkWpk[15][18][2]*
          Wpk[4][18][2])+((IkWpk[15][18][0]*Wpk[4][18][0])+(IkWpk[15][18][1]*
          Wpk[4][18][1]))))+temp[1]);
        temp[3] = (((mk[14]*((Vpk[4][19][2]*Vpk[15][19][2])+((Vpk[4][19][0]*
          Vpk[15][19][0])+(Vpk[4][19][1]*Vpk[15][19][1]))))+((IkWpk[15][19][2]*
          Wpk[4][19][2])+((IkWpk[15][19][0]*Wpk[4][19][0])+(IkWpk[15][19][1]*
          Wpk[4][19][1]))))+temp[2]);
        mm[4][15] = (((mk[15]*((Vpk[4][20][2]*Vpk[15][20][2])+((Vpk[4][20][0]*
          Vpk[15][20][0])+(Vpk[4][20][1]*Vpk[15][20][1]))))+((IkWpk[15][20][2]*
          Wpk[4][20][2])+((IkWpk[15][20][0]*Wpk[4][20][0])+(IkWpk[15][20][1]*
          Wpk[4][20][1]))))+temp[3]);
        temp[0] = (((mk[11]*((Vpk[4][16][2]*Vpk[16][16][2])+((Vpk[4][16][0]*
          Vpk[16][16][0])+(Vpk[4][16][1]*Vpk[16][16][1]))))+((IkWpk[16][16][2]*
          Wpk[4][16][2])+((IkWpk[16][16][0]*Wpk[4][16][0])+(IkWpk[16][16][1]*
          Wpk[4][16][1]))))+((mk[12]*((Vpk[4][17][2]*Vpk[16][17][2])+((
          Vpk[4][17][0]*Vpk[16][17][0])+(Vpk[4][17][1]*Vpk[16][17][1]))))+((
          IkWpk[16][17][2]*Wpk[4][17][2])+((IkWpk[16][17][0]*Wpk[4][17][0])+(
          IkWpk[16][17][1]*Wpk[4][17][1])))));
        temp[1] = (((mk[13]*((Vpk[4][18][2]*Vpk[16][18][2])+((Vpk[4][18][0]*
          Vpk[16][18][0])+(Vpk[4][18][1]*Vpk[16][18][1]))))+((IkWpk[16][18][2]*
          Wpk[4][18][2])+((IkWpk[16][18][0]*Wpk[4][18][0])+(IkWpk[16][18][1]*
          Wpk[4][18][1]))))+temp[0]);
        temp[2] = (((mk[14]*((Vpk[4][19][2]*Vpk[16][19][2])+((Vpk[4][19][0]*
          Vpk[16][19][0])+(Vpk[4][19][1]*Vpk[16][19][1]))))+((IkWpk[16][19][2]*
          Wpk[4][19][2])+((IkWpk[16][19][0]*Wpk[4][19][0])+(IkWpk[16][19][1]*
          Wpk[4][19][1]))))+temp[1]);
        mm[4][16] = (((mk[15]*((Vpk[4][20][2]*Vpk[16][20][2])+((Vpk[4][20][0]*
          Vpk[16][20][0])+(Vpk[4][20][1]*Vpk[16][20][1]))))+((IkWpk[16][20][2]*
          Wpk[4][20][2])+((IkWpk[16][20][0]*Wpk[4][20][0])+(IkWpk[16][20][1]*
          Wpk[4][20][1]))))+temp[2]);
        temp[0] = (((mk[12]*((Vpk[4][17][2]*Vpk[17][17][2])+((Vpk[4][17][0]*
          Vpk[17][17][0])+(Vpk[4][17][1]*Vpk[17][17][1]))))+((IkWpk[17][17][2]*
          Wpk[4][17][2])+((IkWpk[17][17][0]*Wpk[4][17][0])+(IkWpk[17][17][1]*
          Wpk[4][17][1]))))+((mk[13]*((Vpk[4][18][2]*Vpk[17][18][2])+((
          Vpk[4][18][0]*Vpk[17][18][0])+(Vpk[4][18][1]*Vpk[17][18][1]))))+((
          IkWpk[17][18][2]*Wpk[4][18][2])+((IkWpk[17][18][0]*Wpk[4][18][0])+(
          IkWpk[17][18][1]*Wpk[4][18][1])))));
        temp[1] = (((mk[14]*((Vpk[4][19][2]*Vpk[17][19][2])+((Vpk[4][19][0]*
          Vpk[17][19][0])+(Vpk[4][19][1]*Vpk[17][19][1]))))+((IkWpk[17][19][2]*
          Wpk[4][19][2])+((IkWpk[17][19][0]*Wpk[4][19][0])+(IkWpk[17][19][1]*
          Wpk[4][19][1]))))+temp[0]);
        mm[4][17] = (((mk[15]*((Vpk[4][20][2]*Vpk[17][20][2])+((Vpk[4][20][0]*
          Vpk[17][20][0])+(Vpk[4][20][1]*Vpk[17][20][1]))))+((IkWpk[17][20][2]*
          Wpk[4][20][2])+((IkWpk[17][20][0]*Wpk[4][20][0])+(IkWpk[17][20][1]*
          Wpk[4][20][1]))))+temp[1]);
        temp[0] = (((mk[13]*((Vpk[4][18][2]*Vpk[18][18][2])+((Vpk[4][18][0]*
          Vpk[18][18][0])+(Vpk[4][18][1]*Vpk[18][18][1]))))+((IkWpk[18][18][2]*
          Wpk[4][18][2])+((IkWpk[18][18][0]*Wpk[4][18][0])+(IkWpk[18][18][1]*
          Wpk[4][18][1]))))+((mk[14]*((Vpk[4][19][2]*Vpk[18][19][2])+((
          Vpk[4][19][0]*Vpk[18][19][0])+(Vpk[4][19][1]*Vpk[18][19][1]))))+((
          IkWpk[18][19][2]*Wpk[4][19][2])+((IkWpk[18][19][0]*Wpk[4][19][0])+(
          IkWpk[18][19][1]*Wpk[4][19][1])))));
        mm[4][18] = (((mk[15]*((Vpk[4][20][2]*Vpk[18][20][2])+((Vpk[4][20][0]*
          Vpk[18][20][0])+(Vpk[4][20][1]*Vpk[18][20][1]))))+((IkWpk[18][20][2]*
          Wpk[4][20][2])+((IkWpk[18][20][0]*Wpk[4][20][0])+(IkWpk[18][20][1]*
          Wpk[4][20][1]))))+temp[0]);
        mm[4][19] = (((mk[14]*((Vpk[4][19][2]*Vpk[19][19][2])+((Vpk[4][19][0]*
          Vpk[19][19][0])+(Vpk[4][19][1]*Vpk[19][19][1]))))+((IkWpk[19][19][2]*
          Wpk[4][19][2])+((IkWpk[19][19][0]*Wpk[4][19][0])+(IkWpk[19][19][1]*
          Wpk[4][19][1]))))+((mk[15]*((Vpk[4][20][2]*Vpk[19][20][2])+((
          Vpk[4][20][0]*Vpk[19][20][0])+(Vpk[4][20][1]*Vpk[19][20][1]))))+((
          IkWpk[19][20][2]*Wpk[4][20][2])+((IkWpk[19][20][0]*Wpk[4][20][0])+(
          IkWpk[19][20][1]*Wpk[4][20][1])))));
        mm[4][20] = ((mk[15]*((Vpk[4][20][2]*Vpk[20][20][2])+((Vpk[4][20][0]*
          Vpk[20][20][0])+(Vpk[4][20][1]*Vpk[20][20][1]))))+((IkWpk[20][20][2]*
          Wpk[4][20][2])+((IkWpk[20][20][0]*Wpk[4][20][0])+(IkWpk[20][20][1]*
          Wpk[4][20][1]))));
        temp[0] = ((ik[0][2][2]+(mk[0]*((rk[0][0]*rk[0][0])+(rk[0][1]*rk[0][1]))
          ))+((mk[1]*((Vpk[5][6][2]*Vpk[5][6][2])+((Vpk[5][6][0]*Vpk[5][6][0])+(
          Vpk[5][6][1]*Vpk[5][6][1]))))+((Cik[6][2][2]*IkWpk[5][6][2])+((
          Cik[6][2][0]*IkWpk[5][6][0])+(Cik[6][2][1]*IkWpk[5][6][1])))));
        temp[1] = (temp[0]+((mk[2]*((Vpk[5][7][2]*Vpk[5][7][2])+((Vpk[5][7][0]*
          Vpk[5][7][0])+(Vpk[5][7][1]*Vpk[5][7][1]))))+((IkWpk[5][7][2]*
          Wpk[5][7][2])+((IkWpk[5][7][0]*Wpk[5][7][0])+(IkWpk[5][7][1]*
          Wpk[5][7][1])))));
        temp[2] = (((mk[3]*((Vpk[5][8][2]*Vpk[5][8][2])+((Vpk[5][8][0]*
          Vpk[5][8][0])+(Vpk[5][8][1]*Vpk[5][8][1]))))+((IkWpk[5][8][2]*
          Wpk[5][8][2])+((IkWpk[5][8][0]*Wpk[5][8][0])+(IkWpk[5][8][1]*
          Wpk[5][8][1]))))+temp[1]);
        temp[3] = (((mk[4]*((Vpk[5][9][2]*Vpk[5][9][2])+((Vpk[5][9][0]*
          Vpk[5][9][0])+(Vpk[5][9][1]*Vpk[5][9][1]))))+((Cik[9][2][2]*
          IkWpk[5][9][2])+((Cik[9][2][0]*IkWpk[5][9][0])+(Cik[9][2][1]*
          IkWpk[5][9][1]))))+temp[2]);
        temp[4] = (((mk[5]*((Vpk[5][10][2]*Vpk[5][10][2])+((Vpk[5][10][0]*
          Vpk[5][10][0])+(Vpk[5][10][1]*Vpk[5][10][1]))))+((IkWpk[5][10][2]*
          Wpk[5][10][2])+((IkWpk[5][10][0]*Wpk[5][10][0])+(IkWpk[5][10][1]*
          Wpk[5][10][1]))))+temp[3]);
        temp[5] = (((mk[6]*((Vpk[5][11][2]*Vpk[5][11][2])+((Vpk[5][11][0]*
          Vpk[5][11][0])+(Vpk[5][11][1]*Vpk[5][11][1]))))+((IkWpk[5][11][2]*
          Wpk[5][11][2])+((IkWpk[5][11][0]*Wpk[5][11][0])+(IkWpk[5][11][1]*
          Wpk[5][11][1]))))+temp[4]);
        temp[6] = (((mk[7]*((Vpk[5][12][2]*Vpk[5][12][2])+((Vpk[5][12][0]*
          Vpk[5][12][0])+(Vpk[5][12][1]*Vpk[5][12][1]))))+((IkWpk[5][12][2]*
          Wpk[5][12][2])+((IkWpk[5][12][0]*Wpk[5][12][0])+(IkWpk[5][12][1]*
          Wpk[5][12][1]))))+temp[5]);
        temp[7] = (((mk[8]*((Vpk[5][13][2]*Vpk[5][13][2])+((Vpk[5][13][0]*
          Vpk[5][13][0])+(Vpk[5][13][1]*Vpk[5][13][1]))))+((IkWpk[5][13][2]*
          Wpk[5][13][2])+((IkWpk[5][13][0]*Wpk[5][13][0])+(IkWpk[5][13][1]*
          Wpk[5][13][1]))))+temp[6]);
        temp[8] = (((mk[9]*((Vpk[5][14][2]*Vpk[5][14][2])+((Vpk[5][14][0]*
          Vpk[5][14][0])+(Vpk[5][14][1]*Vpk[5][14][1]))))+((IkWpk[5][14][2]*
          Wpk[5][14][2])+((IkWpk[5][14][0]*Wpk[5][14][0])+(IkWpk[5][14][1]*
          Wpk[5][14][1]))))+temp[7]);
        temp[9] = (((mk[10]*((Vpk[5][15][2]*Vpk[5][15][2])+((Vpk[5][15][0]*
          Vpk[5][15][0])+(Vpk[5][15][1]*Vpk[5][15][1]))))+((Cik[15][2][2]*
          IkWpk[5][15][2])+((Cik[15][2][0]*IkWpk[5][15][0])+(Cik[15][2][1]*
          IkWpk[5][15][1]))))+temp[8]);
        temp[10] = (((mk[11]*((Vpk[5][16][2]*Vpk[5][16][2])+((Vpk[5][16][0]*
          Vpk[5][16][0])+(Vpk[5][16][1]*Vpk[5][16][1]))))+((IkWpk[5][16][2]*
          Wpk[5][16][2])+((IkWpk[5][16][0]*Wpk[5][16][0])+(IkWpk[5][16][1]*
          Wpk[5][16][1]))))+temp[9]);
        temp[11] = (((mk[12]*((Vpk[5][17][2]*Vpk[5][17][2])+((Vpk[5][17][0]*
          Vpk[5][17][0])+(Vpk[5][17][1]*Vpk[5][17][1]))))+((IkWpk[5][17][2]*
          Wpk[5][17][2])+((IkWpk[5][17][0]*Wpk[5][17][0])+(IkWpk[5][17][1]*
          Wpk[5][17][1]))))+temp[10]);
        temp[12] = (((mk[13]*((Vpk[5][18][2]*Vpk[5][18][2])+((Vpk[5][18][0]*
          Vpk[5][18][0])+(Vpk[5][18][1]*Vpk[5][18][1]))))+((IkWpk[5][18][2]*
          Wpk[5][18][2])+((IkWpk[5][18][0]*Wpk[5][18][0])+(IkWpk[5][18][1]*
          Wpk[5][18][1]))))+temp[11]);
        temp[13] = (((mk[14]*((Vpk[5][19][2]*Vpk[5][19][2])+((Vpk[5][19][0]*
          Vpk[5][19][0])+(Vpk[5][19][1]*Vpk[5][19][1]))))+((IkWpk[5][19][2]*
          Wpk[5][19][2])+((IkWpk[5][19][0]*Wpk[5][19][0])+(IkWpk[5][19][1]*
          Wpk[5][19][1]))))+temp[12]);
        mm[5][5] = (((mk[15]*((Vpk[5][20][2]*Vpk[5][20][2])+((Vpk[5][20][0]*
          Vpk[5][20][0])+(Vpk[5][20][1]*Vpk[5][20][1]))))+((IkWpk[5][20][2]*
          Wpk[5][20][2])+((IkWpk[5][20][0]*Wpk[5][20][0])+(IkWpk[5][20][1]*
          Wpk[5][20][1]))))+temp[13]);
        temp[0] = (((mk[1]*((Vpk[5][6][2]*Vpk[6][6][2])+((Vpk[5][6][0]*
          Vpk[6][6][0])+(Vpk[5][6][1]*Vpk[6][6][1]))))+((Cik[6][2][2]*
          IkWpk[6][6][2])+((Cik[6][2][0]*IkWpk[6][6][0])+(Cik[6][2][1]*
          IkWpk[6][6][1]))))+((mk[2]*((Vpk[5][7][2]*Vpk[6][7][2])+((Vpk[5][7][0]
          *Vpk[6][7][0])+(Vpk[5][7][1]*Vpk[6][7][1]))))+((IkWpk[6][7][2]*
          Wpk[5][7][2])+((IkWpk[6][7][0]*Wpk[5][7][0])+(IkWpk[6][7][1]*
          Wpk[5][7][1])))));
        mm[5][6] = (((mk[3]*((Vpk[5][8][2]*Vpk[6][8][2])+((Vpk[5][8][0]*
          Vpk[6][8][0])+(Vpk[5][8][1]*Vpk[6][8][1]))))+((IkWpk[6][8][2]*
          Wpk[5][8][2])+((IkWpk[6][8][0]*Wpk[5][8][0])+(IkWpk[6][8][1]*
          Wpk[5][8][1]))))+temp[0]);
        mm[5][7] = (((mk[2]*((Vpk[5][7][2]*Vpk[7][7][2])+((Vpk[5][7][0]*
          Vpk[7][7][0])+(Vpk[5][7][1]*Vpk[7][7][1]))))+((IkWpk[7][7][2]*
          Wpk[5][7][2])+((IkWpk[7][7][0]*Wpk[5][7][0])+(IkWpk[7][7][1]*
          Wpk[5][7][1]))))+((mk[3]*((Vpk[5][8][2]*Vpk[7][8][2])+((Vpk[5][8][0]*
          Vpk[7][8][0])+(Vpk[5][8][1]*Vpk[7][8][1]))))+((IkWpk[7][8][2]*
          Wpk[5][8][2])+((IkWpk[7][8][0]*Wpk[5][8][0])+(IkWpk[7][8][1]*
          Wpk[5][8][1])))));
        mm[5][8] = ((mk[3]*((Vpk[5][8][2]*Vpk[8][8][2])+((Vpk[5][8][0]*
          Vpk[8][8][0])+(Vpk[5][8][1]*Vpk[8][8][1]))))+((IkWpk[8][8][2]*
          Wpk[5][8][2])+((IkWpk[8][8][0]*Wpk[5][8][0])+(IkWpk[8][8][1]*
          Wpk[5][8][1]))));
        temp[0] = (((mk[4]*((Vpk[5][9][2]*Vpk[9][9][2])+((Vpk[5][9][0]*
          Vpk[9][9][0])+(Vpk[5][9][1]*Vpk[9][9][1]))))+((Cik[9][2][2]*
          IkWpk[9][9][2])+((Cik[9][2][0]*IkWpk[9][9][0])+(Cik[9][2][1]*
          IkWpk[9][9][1]))))+((mk[5]*((Vpk[5][10][2]*Vpk[9][10][2])+((
          Vpk[5][10][0]*Vpk[9][10][0])+(Vpk[5][10][1]*Vpk[9][10][1]))))+((
          IkWpk[9][10][2]*Wpk[5][10][2])+((IkWpk[9][10][0]*Wpk[5][10][0])+(
          IkWpk[9][10][1]*Wpk[5][10][1])))));
        temp[1] = (((mk[6]*((Vpk[5][11][2]*Vpk[9][11][2])+((Vpk[5][11][0]*
          Vpk[9][11][0])+(Vpk[5][11][1]*Vpk[9][11][1]))))+((IkWpk[9][11][2]*
          Wpk[5][11][2])+((IkWpk[9][11][0]*Wpk[5][11][0])+(IkWpk[9][11][1]*
          Wpk[5][11][1]))))+temp[0]);
        temp[2] = (((mk[7]*((Vpk[5][12][2]*Vpk[9][12][2])+((Vpk[5][12][0]*
          Vpk[9][12][0])+(Vpk[5][12][1]*Vpk[9][12][1]))))+((IkWpk[9][12][2]*
          Wpk[5][12][2])+((IkWpk[9][12][0]*Wpk[5][12][0])+(IkWpk[9][12][1]*
          Wpk[5][12][1]))))+temp[1]);
        temp[3] = (((mk[8]*((Vpk[5][13][2]*Vpk[9][13][2])+((Vpk[5][13][0]*
          Vpk[9][13][0])+(Vpk[5][13][1]*Vpk[9][13][1]))))+((IkWpk[9][13][2]*
          Wpk[5][13][2])+((IkWpk[9][13][0]*Wpk[5][13][0])+(IkWpk[9][13][1]*
          Wpk[5][13][1]))))+temp[2]);
        mm[5][9] = (((mk[9]*((Vpk[5][14][2]*Vpk[9][14][2])+((Vpk[5][14][0]*
          Vpk[9][14][0])+(Vpk[5][14][1]*Vpk[9][14][1]))))+((IkWpk[9][14][2]*
          Wpk[5][14][2])+((IkWpk[9][14][0]*Wpk[5][14][0])+(IkWpk[9][14][1]*
          Wpk[5][14][1]))))+temp[3]);
        temp[0] = (((mk[5]*((Vpk[5][10][2]*Vpk[10][10][2])+((Vpk[5][10][0]*
          Vpk[10][10][0])+(Vpk[5][10][1]*Vpk[10][10][1]))))+((IkWpk[10][10][2]*
          Wpk[5][10][2])+((IkWpk[10][10][0]*Wpk[5][10][0])+(IkWpk[10][10][1]*
          Wpk[5][10][1]))))+((mk[6]*((Vpk[5][11][2]*Vpk[10][11][2])+((
          Vpk[5][11][0]*Vpk[10][11][0])+(Vpk[5][11][1]*Vpk[10][11][1]))))+((
          IkWpk[10][11][2]*Wpk[5][11][2])+((IkWpk[10][11][0]*Wpk[5][11][0])+(
          IkWpk[10][11][1]*Wpk[5][11][1])))));
        temp[1] = (((mk[7]*((Vpk[5][12][2]*Vpk[10][12][2])+((Vpk[5][12][0]*
          Vpk[10][12][0])+(Vpk[5][12][1]*Vpk[10][12][1]))))+((IkWpk[10][12][2]*
          Wpk[5][12][2])+((IkWpk[10][12][0]*Wpk[5][12][0])+(IkWpk[10][12][1]*
          Wpk[5][12][1]))))+temp[0]);
        temp[2] = (((mk[8]*((Vpk[5][13][2]*Vpk[10][13][2])+((Vpk[5][13][0]*
          Vpk[10][13][0])+(Vpk[5][13][1]*Vpk[10][13][1]))))+((IkWpk[10][13][2]*
          Wpk[5][13][2])+((IkWpk[10][13][0]*Wpk[5][13][0])+(IkWpk[10][13][1]*
          Wpk[5][13][1]))))+temp[1]);
        mm[5][10] = (((mk[9]*((Vpk[5][14][2]*Vpk[10][14][2])+((Vpk[5][14][0]*
          Vpk[10][14][0])+(Vpk[5][14][1]*Vpk[10][14][1]))))+((IkWpk[10][14][2]*
          Wpk[5][14][2])+((IkWpk[10][14][0]*Wpk[5][14][0])+(IkWpk[10][14][1]*
          Wpk[5][14][1]))))+temp[2]);
        temp[0] = (((mk[6]*((Vpk[5][11][2]*Vpk[11][11][2])+((Vpk[5][11][0]*
          Vpk[11][11][0])+(Vpk[5][11][1]*Vpk[11][11][1]))))+((IkWpk[11][11][2]*
          Wpk[5][11][2])+((IkWpk[11][11][0]*Wpk[5][11][0])+(IkWpk[11][11][1]*
          Wpk[5][11][1]))))+((mk[7]*((Vpk[5][12][2]*Vpk[11][12][2])+((
          Vpk[5][12][0]*Vpk[11][12][0])+(Vpk[5][12][1]*Vpk[11][12][1]))))+((
          IkWpk[11][12][2]*Wpk[5][12][2])+((IkWpk[11][12][0]*Wpk[5][12][0])+(
          IkWpk[11][12][1]*Wpk[5][12][1])))));
        temp[1] = (((mk[8]*((Vpk[5][13][2]*Vpk[11][13][2])+((Vpk[5][13][0]*
          Vpk[11][13][0])+(Vpk[5][13][1]*Vpk[11][13][1]))))+((IkWpk[11][13][2]*
          Wpk[5][13][2])+((IkWpk[11][13][0]*Wpk[5][13][0])+(IkWpk[11][13][1]*
          Wpk[5][13][1]))))+temp[0]);
        mm[5][11] = (((mk[9]*((Vpk[5][14][2]*Vpk[11][14][2])+((Vpk[5][14][0]*
          Vpk[11][14][0])+(Vpk[5][14][1]*Vpk[11][14][1]))))+((IkWpk[11][14][2]*
          Wpk[5][14][2])+((IkWpk[11][14][0]*Wpk[5][14][0])+(IkWpk[11][14][1]*
          Wpk[5][14][1]))))+temp[1]);
        temp[0] = (((mk[7]*((Vpk[5][12][2]*Vpk[12][12][2])+((Vpk[5][12][0]*
          Vpk[12][12][0])+(Vpk[5][12][1]*Vpk[12][12][1]))))+((IkWpk[12][12][2]*
          Wpk[5][12][2])+((IkWpk[12][12][0]*Wpk[5][12][0])+(IkWpk[12][12][1]*
          Wpk[5][12][1]))))+((mk[8]*((Vpk[5][13][2]*Vpk[12][13][2])+((
          Vpk[5][13][0]*Vpk[12][13][0])+(Vpk[5][13][1]*Vpk[12][13][1]))))+((
          IkWpk[12][13][2]*Wpk[5][13][2])+((IkWpk[12][13][0]*Wpk[5][13][0])+(
          IkWpk[12][13][1]*Wpk[5][13][1])))));
        mm[5][12] = (((mk[9]*((Vpk[5][14][2]*Vpk[12][14][2])+((Vpk[5][14][0]*
          Vpk[12][14][0])+(Vpk[5][14][1]*Vpk[12][14][1]))))+((IkWpk[12][14][2]*
          Wpk[5][14][2])+((IkWpk[12][14][0]*Wpk[5][14][0])+(IkWpk[12][14][1]*
          Wpk[5][14][1]))))+temp[0]);
        mm[5][13] = (((mk[8]*((Vpk[5][13][2]*Vpk[13][13][2])+((Vpk[5][13][0]*
          Vpk[13][13][0])+(Vpk[5][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          Wpk[5][13][2])+((IkWpk[13][13][0]*Wpk[5][13][0])+(IkWpk[13][13][1]*
          Wpk[5][13][1]))))+((mk[9]*((Vpk[5][14][2]*Vpk[13][14][2])+((
          Vpk[5][14][0]*Vpk[13][14][0])+(Vpk[5][14][1]*Vpk[13][14][1]))))+((
          IkWpk[13][14][2]*Wpk[5][14][2])+((IkWpk[13][14][0]*Wpk[5][14][0])+(
          IkWpk[13][14][1]*Wpk[5][14][1])))));
        mm[5][14] = ((mk[9]*((Vpk[5][14][2]*Vpk[14][14][2])+((Vpk[5][14][0]*
          Vpk[14][14][0])+(Vpk[5][14][1]*Vpk[14][14][1]))))+((IkWpk[14][14][2]*
          Wpk[5][14][2])+((IkWpk[14][14][0]*Wpk[5][14][0])+(IkWpk[14][14][1]*
          Wpk[5][14][1]))));
        temp[0] = (((mk[10]*((Vpk[5][15][2]*Vpk[15][15][2])+((Vpk[5][15][0]*
          Vpk[15][15][0])+(Vpk[5][15][1]*Vpk[15][15][1]))))+((Cik[15][2][2]*
          IkWpk[15][15][2])+((Cik[15][2][0]*IkWpk[15][15][0])+(Cik[15][2][1]*
          IkWpk[15][15][1]))))+((mk[11]*((Vpk[5][16][2]*Vpk[15][16][2])+((
          Vpk[5][16][0]*Vpk[15][16][0])+(Vpk[5][16][1]*Vpk[15][16][1]))))+((
          IkWpk[15][16][2]*Wpk[5][16][2])+((IkWpk[15][16][0]*Wpk[5][16][0])+(
          IkWpk[15][16][1]*Wpk[5][16][1])))));
        temp[1] = (((mk[12]*((Vpk[5][17][2]*Vpk[15][17][2])+((Vpk[5][17][0]*
          Vpk[15][17][0])+(Vpk[5][17][1]*Vpk[15][17][1]))))+((IkWpk[15][17][2]*
          Wpk[5][17][2])+((IkWpk[15][17][0]*Wpk[5][17][0])+(IkWpk[15][17][1]*
          Wpk[5][17][1]))))+temp[0]);
        temp[2] = (((mk[13]*((Vpk[5][18][2]*Vpk[15][18][2])+((Vpk[5][18][0]*
          Vpk[15][18][0])+(Vpk[5][18][1]*Vpk[15][18][1]))))+((IkWpk[15][18][2]*
          Wpk[5][18][2])+((IkWpk[15][18][0]*Wpk[5][18][0])+(IkWpk[15][18][1]*
          Wpk[5][18][1]))))+temp[1]);
        temp[3] = (((mk[14]*((Vpk[5][19][2]*Vpk[15][19][2])+((Vpk[5][19][0]*
          Vpk[15][19][0])+(Vpk[5][19][1]*Vpk[15][19][1]))))+((IkWpk[15][19][2]*
          Wpk[5][19][2])+((IkWpk[15][19][0]*Wpk[5][19][0])+(IkWpk[15][19][1]*
          Wpk[5][19][1]))))+temp[2]);
        mm[5][15] = (((mk[15]*((Vpk[5][20][2]*Vpk[15][20][2])+((Vpk[5][20][0]*
          Vpk[15][20][0])+(Vpk[5][20][1]*Vpk[15][20][1]))))+((IkWpk[15][20][2]*
          Wpk[5][20][2])+((IkWpk[15][20][0]*Wpk[5][20][0])+(IkWpk[15][20][1]*
          Wpk[5][20][1]))))+temp[3]);
        temp[0] = (((mk[11]*((Vpk[5][16][2]*Vpk[16][16][2])+((Vpk[5][16][0]*
          Vpk[16][16][0])+(Vpk[5][16][1]*Vpk[16][16][1]))))+((IkWpk[16][16][2]*
          Wpk[5][16][2])+((IkWpk[16][16][0]*Wpk[5][16][0])+(IkWpk[16][16][1]*
          Wpk[5][16][1]))))+((mk[12]*((Vpk[5][17][2]*Vpk[16][17][2])+((
          Vpk[5][17][0]*Vpk[16][17][0])+(Vpk[5][17][1]*Vpk[16][17][1]))))+((
          IkWpk[16][17][2]*Wpk[5][17][2])+((IkWpk[16][17][0]*Wpk[5][17][0])+(
          IkWpk[16][17][1]*Wpk[5][17][1])))));
        temp[1] = (((mk[13]*((Vpk[5][18][2]*Vpk[16][18][2])+((Vpk[5][18][0]*
          Vpk[16][18][0])+(Vpk[5][18][1]*Vpk[16][18][1]))))+((IkWpk[16][18][2]*
          Wpk[5][18][2])+((IkWpk[16][18][0]*Wpk[5][18][0])+(IkWpk[16][18][1]*
          Wpk[5][18][1]))))+temp[0]);
        temp[2] = (((mk[14]*((Vpk[5][19][2]*Vpk[16][19][2])+((Vpk[5][19][0]*
          Vpk[16][19][0])+(Vpk[5][19][1]*Vpk[16][19][1]))))+((IkWpk[16][19][2]*
          Wpk[5][19][2])+((IkWpk[16][19][0]*Wpk[5][19][0])+(IkWpk[16][19][1]*
          Wpk[5][19][1]))))+temp[1]);
        mm[5][16] = (((mk[15]*((Vpk[5][20][2]*Vpk[16][20][2])+((Vpk[5][20][0]*
          Vpk[16][20][0])+(Vpk[5][20][1]*Vpk[16][20][1]))))+((IkWpk[16][20][2]*
          Wpk[5][20][2])+((IkWpk[16][20][0]*Wpk[5][20][0])+(IkWpk[16][20][1]*
          Wpk[5][20][1]))))+temp[2]);
        temp[0] = (((mk[12]*((Vpk[5][17][2]*Vpk[17][17][2])+((Vpk[5][17][0]*
          Vpk[17][17][0])+(Vpk[5][17][1]*Vpk[17][17][1]))))+((IkWpk[17][17][2]*
          Wpk[5][17][2])+((IkWpk[17][17][0]*Wpk[5][17][0])+(IkWpk[17][17][1]*
          Wpk[5][17][1]))))+((mk[13]*((Vpk[5][18][2]*Vpk[17][18][2])+((
          Vpk[5][18][0]*Vpk[17][18][0])+(Vpk[5][18][1]*Vpk[17][18][1]))))+((
          IkWpk[17][18][2]*Wpk[5][18][2])+((IkWpk[17][18][0]*Wpk[5][18][0])+(
          IkWpk[17][18][1]*Wpk[5][18][1])))));
        temp[1] = (((mk[14]*((Vpk[5][19][2]*Vpk[17][19][2])+((Vpk[5][19][0]*
          Vpk[17][19][0])+(Vpk[5][19][1]*Vpk[17][19][1]))))+((IkWpk[17][19][2]*
          Wpk[5][19][2])+((IkWpk[17][19][0]*Wpk[5][19][0])+(IkWpk[17][19][1]*
          Wpk[5][19][1]))))+temp[0]);
        mm[5][17] = (((mk[15]*((Vpk[5][20][2]*Vpk[17][20][2])+((Vpk[5][20][0]*
          Vpk[17][20][0])+(Vpk[5][20][1]*Vpk[17][20][1]))))+((IkWpk[17][20][2]*
          Wpk[5][20][2])+((IkWpk[17][20][0]*Wpk[5][20][0])+(IkWpk[17][20][1]*
          Wpk[5][20][1]))))+temp[1]);
        temp[0] = (((mk[13]*((Vpk[5][18][2]*Vpk[18][18][2])+((Vpk[5][18][0]*
          Vpk[18][18][0])+(Vpk[5][18][1]*Vpk[18][18][1]))))+((IkWpk[18][18][2]*
          Wpk[5][18][2])+((IkWpk[18][18][0]*Wpk[5][18][0])+(IkWpk[18][18][1]*
          Wpk[5][18][1]))))+((mk[14]*((Vpk[5][19][2]*Vpk[18][19][2])+((
          Vpk[5][19][0]*Vpk[18][19][0])+(Vpk[5][19][1]*Vpk[18][19][1]))))+((
          IkWpk[18][19][2]*Wpk[5][19][2])+((IkWpk[18][19][0]*Wpk[5][19][0])+(
          IkWpk[18][19][1]*Wpk[5][19][1])))));
        mm[5][18] = (((mk[15]*((Vpk[5][20][2]*Vpk[18][20][2])+((Vpk[5][20][0]*
          Vpk[18][20][0])+(Vpk[5][20][1]*Vpk[18][20][1]))))+((IkWpk[18][20][2]*
          Wpk[5][20][2])+((IkWpk[18][20][0]*Wpk[5][20][0])+(IkWpk[18][20][1]*
          Wpk[5][20][1]))))+temp[0]);
        mm[5][19] = (((mk[14]*((Vpk[5][19][2]*Vpk[19][19][2])+((Vpk[5][19][0]*
          Vpk[19][19][0])+(Vpk[5][19][1]*Vpk[19][19][1]))))+((IkWpk[19][19][2]*
          Wpk[5][19][2])+((IkWpk[19][19][0]*Wpk[5][19][0])+(IkWpk[19][19][1]*
          Wpk[5][19][1]))))+((mk[15]*((Vpk[5][20][2]*Vpk[19][20][2])+((
          Vpk[5][20][0]*Vpk[19][20][0])+(Vpk[5][20][1]*Vpk[19][20][1]))))+((
          IkWpk[19][20][2]*Wpk[5][20][2])+((IkWpk[19][20][0]*Wpk[5][20][0])+(
          IkWpk[19][20][1]*Wpk[5][20][1])))));
        mm[5][20] = ((mk[15]*((Vpk[5][20][2]*Vpk[20][20][2])+((Vpk[5][20][0]*
          Vpk[20][20][0])+(Vpk[5][20][1]*Vpk[20][20][1]))))+((IkWpk[20][20][2]*
          Wpk[5][20][2])+((IkWpk[20][20][0]*Wpk[5][20][0])+(IkWpk[20][20][1]*
          Wpk[5][20][1]))));
        temp[0] = (((mk[1]*((Vpk[6][6][2]*Vpk[6][6][2])+((Vpk[6][6][0]*
          Vpk[6][6][0])+(Vpk[6][6][1]*Vpk[6][6][1]))))+((IkWpk[6][6][2]*
          pin[6][2])+((IkWpk[6][6][0]*pin[6][0])+(IkWpk[6][6][1]*pin[6][1]))))+(
          (mk[2]*((Vpk[6][7][2]*Vpk[6][7][2])+((Vpk[6][7][0]*Vpk[6][7][0])+(
          Vpk[6][7][1]*Vpk[6][7][1]))))+((IkWpk[6][7][2]*Wpk[6][7][2])+((
          IkWpk[6][7][0]*Wpk[6][7][0])+(IkWpk[6][7][1]*Wpk[6][7][1])))));
        mm[6][6] = (((mk[3]*((Vpk[6][8][2]*Vpk[6][8][2])+((Vpk[6][8][0]*
          Vpk[6][8][0])+(Vpk[6][8][1]*Vpk[6][8][1]))))+((IkWpk[6][8][2]*
          Wpk[6][8][2])+((IkWpk[6][8][0]*Wpk[6][8][0])+(IkWpk[6][8][1]*
          Wpk[6][8][1]))))+temp[0]);
        mm[6][7] = (((mk[2]*((Vpk[6][7][2]*Vpk[7][7][2])+((Vpk[6][7][0]*
          Vpk[7][7][0])+(Vpk[6][7][1]*Vpk[7][7][1]))))+((IkWpk[7][7][2]*
          Wpk[6][7][2])+((IkWpk[7][7][0]*Wpk[6][7][0])+(IkWpk[7][7][1]*
          Wpk[6][7][1]))))+((mk[3]*((Vpk[6][8][2]*Vpk[7][8][2])+((Vpk[6][8][0]*
          Vpk[7][8][0])+(Vpk[6][8][1]*Vpk[7][8][1]))))+((IkWpk[7][8][2]*
          Wpk[6][8][2])+((IkWpk[7][8][0]*Wpk[6][8][0])+(IkWpk[7][8][1]*
          Wpk[6][8][1])))));
        mm[6][8] = ((mk[3]*((Vpk[6][8][2]*Vpk[8][8][2])+((Vpk[6][8][0]*
          Vpk[8][8][0])+(Vpk[6][8][1]*Vpk[8][8][1]))))+((IkWpk[8][8][2]*
          Wpk[6][8][2])+((IkWpk[8][8][0]*Wpk[6][8][0])+(IkWpk[8][8][1]*
          Wpk[6][8][1]))));
        mm[6][9] = 0.;
        mm[6][10] = 0.;
        mm[6][11] = 0.;
        mm[6][12] = 0.;
        mm[6][13] = 0.;
        mm[6][14] = 0.;
        mm[6][15] = 0.;
        mm[6][16] = 0.;
        mm[6][17] = 0.;
        mm[6][18] = 0.;
        mm[6][19] = 0.;
        mm[6][20] = 0.;
        mm[7][7] = (((mk[2]*((Vpk[7][7][2]*Vpk[7][7][2])+((Vpk[7][7][0]*
          Vpk[7][7][0])+(Vpk[7][7][1]*Vpk[7][7][1]))))+((IkWpk[7][7][2]*
          pin[7][2])+((IkWpk[7][7][0]*pin[7][0])+(IkWpk[7][7][1]*pin[7][1]))))+(
          (mk[3]*((Vpk[7][8][2]*Vpk[7][8][2])+((Vpk[7][8][0]*Vpk[7][8][0])+(
          Vpk[7][8][1]*Vpk[7][8][1]))))+((IkWpk[7][8][2]*Wpk[7][8][2])+((
          IkWpk[7][8][0]*Wpk[7][8][0])+(IkWpk[7][8][1]*Wpk[7][8][1])))));
        mm[7][8] = ((mk[3]*((Vpk[7][8][2]*Vpk[8][8][2])+((Vpk[7][8][0]*
          Vpk[8][8][0])+(Vpk[7][8][1]*Vpk[8][8][1]))))+((IkWpk[8][8][2]*
          Wpk[7][8][2])+((IkWpk[8][8][0]*Wpk[7][8][0])+(IkWpk[8][8][1]*
          Wpk[7][8][1]))));
        mm[7][9] = 0.;
        mm[7][10] = 0.;
        mm[7][11] = 0.;
        mm[7][12] = 0.;
        mm[7][13] = 0.;
        mm[7][14] = 0.;
        mm[7][15] = 0.;
        mm[7][16] = 0.;
        mm[7][17] = 0.;
        mm[7][18] = 0.;
        mm[7][19] = 0.;
        mm[7][20] = 0.;
        mm[8][8] = ((mk[3]*((Vpk[8][8][2]*Vpk[8][8][2])+((Vpk[8][8][0]*
          Vpk[8][8][0])+(Vpk[8][8][1]*Vpk[8][8][1]))))+((IkWpk[8][8][2]*
          pin[8][2])+((IkWpk[8][8][0]*pin[8][0])+(IkWpk[8][8][1]*pin[8][1]))));
        mm[8][9] = 0.;
        mm[8][10] = 0.;
        mm[8][11] = 0.;
        mm[8][12] = 0.;
        mm[8][13] = 0.;
        mm[8][14] = 0.;
        mm[8][15] = 0.;
        mm[8][16] = 0.;
        mm[8][17] = 0.;
        mm[8][18] = 0.;
        mm[8][19] = 0.;
        mm[8][20] = 0.;
        temp[0] = (((mk[4]*((Vpk[9][9][2]*Vpk[9][9][2])+((Vpk[9][9][0]*
          Vpk[9][9][0])+(Vpk[9][9][1]*Vpk[9][9][1]))))+((IkWpk[9][9][2]*
          pin[9][2])+((IkWpk[9][9][0]*pin[9][0])+(IkWpk[9][9][1]*pin[9][1]))))+(
          (mk[5]*((Vpk[9][10][2]*Vpk[9][10][2])+((Vpk[9][10][0]*Vpk[9][10][0])+(
          Vpk[9][10][1]*Vpk[9][10][1]))))+((IkWpk[9][10][2]*Wpk[9][10][2])+((
          IkWpk[9][10][0]*Wpk[9][10][0])+(IkWpk[9][10][1]*Wpk[9][10][1])))));
        temp[1] = (((mk[6]*((Vpk[9][11][2]*Vpk[9][11][2])+((Vpk[9][11][0]*
          Vpk[9][11][0])+(Vpk[9][11][1]*Vpk[9][11][1]))))+((IkWpk[9][11][2]*
          Wpk[9][11][2])+((IkWpk[9][11][0]*Wpk[9][11][0])+(IkWpk[9][11][1]*
          Wpk[9][11][1]))))+temp[0]);
        temp[2] = (((mk[7]*((Vpk[9][12][2]*Vpk[9][12][2])+((Vpk[9][12][0]*
          Vpk[9][12][0])+(Vpk[9][12][1]*Vpk[9][12][1]))))+((IkWpk[9][12][2]*
          Wpk[9][12][2])+((IkWpk[9][12][0]*Wpk[9][12][0])+(IkWpk[9][12][1]*
          Wpk[9][12][1]))))+temp[1]);
        temp[3] = (((mk[8]*((Vpk[9][13][2]*Vpk[9][13][2])+((Vpk[9][13][0]*
          Vpk[9][13][0])+(Vpk[9][13][1]*Vpk[9][13][1]))))+((IkWpk[9][13][2]*
          Wpk[9][13][2])+((IkWpk[9][13][0]*Wpk[9][13][0])+(IkWpk[9][13][1]*
          Wpk[9][13][1]))))+temp[2]);
        mm[9][9] = (((mk[9]*((Vpk[9][14][2]*Vpk[9][14][2])+((Vpk[9][14][0]*
          Vpk[9][14][0])+(Vpk[9][14][1]*Vpk[9][14][1]))))+((IkWpk[9][14][2]*
          Wpk[9][14][2])+((IkWpk[9][14][0]*Wpk[9][14][0])+(IkWpk[9][14][1]*
          Wpk[9][14][1]))))+temp[3]);
        temp[0] = (((mk[5]*((Vpk[9][10][2]*Vpk[10][10][2])+((Vpk[9][10][0]*
          Vpk[10][10][0])+(Vpk[9][10][1]*Vpk[10][10][1]))))+((IkWpk[10][10][2]*
          Wpk[9][10][2])+((IkWpk[10][10][0]*Wpk[9][10][0])+(IkWpk[10][10][1]*
          Wpk[9][10][1]))))+((mk[6]*((Vpk[9][11][2]*Vpk[10][11][2])+((
          Vpk[9][11][0]*Vpk[10][11][0])+(Vpk[9][11][1]*Vpk[10][11][1]))))+((
          IkWpk[10][11][2]*Wpk[9][11][2])+((IkWpk[10][11][0]*Wpk[9][11][0])+(
          IkWpk[10][11][1]*Wpk[9][11][1])))));
        temp[1] = (((mk[7]*((Vpk[9][12][2]*Vpk[10][12][2])+((Vpk[9][12][0]*
          Vpk[10][12][0])+(Vpk[9][12][1]*Vpk[10][12][1]))))+((IkWpk[10][12][2]*
          Wpk[9][12][2])+((IkWpk[10][12][0]*Wpk[9][12][0])+(IkWpk[10][12][1]*
          Wpk[9][12][1]))))+temp[0]);
        temp[2] = (((mk[8]*((Vpk[9][13][2]*Vpk[10][13][2])+((Vpk[9][13][0]*
          Vpk[10][13][0])+(Vpk[9][13][1]*Vpk[10][13][1]))))+((IkWpk[10][13][2]*
          Wpk[9][13][2])+((IkWpk[10][13][0]*Wpk[9][13][0])+(IkWpk[10][13][1]*
          Wpk[9][13][1]))))+temp[1]);
        mm[9][10] = (((mk[9]*((Vpk[9][14][2]*Vpk[10][14][2])+((Vpk[9][14][0]*
          Vpk[10][14][0])+(Vpk[9][14][1]*Vpk[10][14][1]))))+((IkWpk[10][14][2]*
          Wpk[9][14][2])+((IkWpk[10][14][0]*Wpk[9][14][0])+(IkWpk[10][14][1]*
          Wpk[9][14][1]))))+temp[2]);
        temp[0] = (((mk[6]*((Vpk[9][11][2]*Vpk[11][11][2])+((Vpk[9][11][0]*
          Vpk[11][11][0])+(Vpk[9][11][1]*Vpk[11][11][1]))))+((IkWpk[11][11][2]*
          Wpk[9][11][2])+((IkWpk[11][11][0]*Wpk[9][11][0])+(IkWpk[11][11][1]*
          Wpk[9][11][1]))))+((mk[7]*((Vpk[9][12][2]*Vpk[11][12][2])+((
          Vpk[9][12][0]*Vpk[11][12][0])+(Vpk[9][12][1]*Vpk[11][12][1]))))+((
          IkWpk[11][12][2]*Wpk[9][12][2])+((IkWpk[11][12][0]*Wpk[9][12][0])+(
          IkWpk[11][12][1]*Wpk[9][12][1])))));
        temp[1] = (((mk[8]*((Vpk[9][13][2]*Vpk[11][13][2])+((Vpk[9][13][0]*
          Vpk[11][13][0])+(Vpk[9][13][1]*Vpk[11][13][1]))))+((IkWpk[11][13][2]*
          Wpk[9][13][2])+((IkWpk[11][13][0]*Wpk[9][13][0])+(IkWpk[11][13][1]*
          Wpk[9][13][1]))))+temp[0]);
        mm[9][11] = (((mk[9]*((Vpk[9][14][2]*Vpk[11][14][2])+((Vpk[9][14][0]*
          Vpk[11][14][0])+(Vpk[9][14][1]*Vpk[11][14][1]))))+((IkWpk[11][14][2]*
          Wpk[9][14][2])+((IkWpk[11][14][0]*Wpk[9][14][0])+(IkWpk[11][14][1]*
          Wpk[9][14][1]))))+temp[1]);
        temp[0] = (((mk[7]*((Vpk[9][12][2]*Vpk[12][12][2])+((Vpk[9][12][0]*
          Vpk[12][12][0])+(Vpk[9][12][1]*Vpk[12][12][1]))))+((IkWpk[12][12][2]*
          Wpk[9][12][2])+((IkWpk[12][12][0]*Wpk[9][12][0])+(IkWpk[12][12][1]*
          Wpk[9][12][1]))))+((mk[8]*((Vpk[9][13][2]*Vpk[12][13][2])+((
          Vpk[9][13][0]*Vpk[12][13][0])+(Vpk[9][13][1]*Vpk[12][13][1]))))+((
          IkWpk[12][13][2]*Wpk[9][13][2])+((IkWpk[12][13][0]*Wpk[9][13][0])+(
          IkWpk[12][13][1]*Wpk[9][13][1])))));
        mm[9][12] = (((mk[9]*((Vpk[9][14][2]*Vpk[12][14][2])+((Vpk[9][14][0]*
          Vpk[12][14][0])+(Vpk[9][14][1]*Vpk[12][14][1]))))+((IkWpk[12][14][2]*
          Wpk[9][14][2])+((IkWpk[12][14][0]*Wpk[9][14][0])+(IkWpk[12][14][1]*
          Wpk[9][14][1]))))+temp[0]);
        mm[9][13] = (((mk[8]*((Vpk[9][13][2]*Vpk[13][13][2])+((Vpk[9][13][0]*
          Vpk[13][13][0])+(Vpk[9][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          Wpk[9][13][2])+((IkWpk[13][13][0]*Wpk[9][13][0])+(IkWpk[13][13][1]*
          Wpk[9][13][1]))))+((mk[9]*((Vpk[9][14][2]*Vpk[13][14][2])+((
          Vpk[9][14][0]*Vpk[13][14][0])+(Vpk[9][14][1]*Vpk[13][14][1]))))+((
          IkWpk[13][14][2]*Wpk[9][14][2])+((IkWpk[13][14][0]*Wpk[9][14][0])+(
          IkWpk[13][14][1]*Wpk[9][14][1])))));
        mm[9][14] = ((mk[9]*((Vpk[9][14][2]*Vpk[14][14][2])+((Vpk[9][14][0]*
          Vpk[14][14][0])+(Vpk[9][14][1]*Vpk[14][14][1]))))+((IkWpk[14][14][2]*
          Wpk[9][14][2])+((IkWpk[14][14][0]*Wpk[9][14][0])+(IkWpk[14][14][1]*
          Wpk[9][14][1]))));
        mm[9][15] = 0.;
        mm[9][16] = 0.;
        mm[9][17] = 0.;
        mm[9][18] = 0.;
        mm[9][19] = 0.;
        mm[9][20] = 0.;
        temp[0] = (((mk[5]*((Vpk[10][10][2]*Vpk[10][10][2])+((Vpk[10][10][0]*
          Vpk[10][10][0])+(Vpk[10][10][1]*Vpk[10][10][1]))))+((IkWpk[10][10][2]*
          pin[10][2])+((IkWpk[10][10][0]*pin[10][0])+(IkWpk[10][10][1]*
          pin[10][1]))))+((mk[6]*((Vpk[10][11][2]*Vpk[10][11][2])+((
          Vpk[10][11][0]*Vpk[10][11][0])+(Vpk[10][11][1]*Vpk[10][11][1]))))+((
          IkWpk[10][11][2]*Wpk[10][11][2])+((IkWpk[10][11][0]*Wpk[10][11][0])+(
          IkWpk[10][11][1]*Wpk[10][11][1])))));
        temp[1] = (((mk[7]*((Vpk[10][12][2]*Vpk[10][12][2])+((Vpk[10][12][0]*
          Vpk[10][12][0])+(Vpk[10][12][1]*Vpk[10][12][1]))))+((IkWpk[10][12][2]*
          Wpk[10][12][2])+((IkWpk[10][12][0]*Wpk[10][12][0])+(IkWpk[10][12][1]*
          Wpk[10][12][1]))))+temp[0]);
        temp[2] = (((mk[8]*((Vpk[10][13][2]*Vpk[10][13][2])+((Vpk[10][13][0]*
          Vpk[10][13][0])+(Vpk[10][13][1]*Vpk[10][13][1]))))+((IkWpk[10][13][2]*
          Wpk[10][13][2])+((IkWpk[10][13][0]*Wpk[10][13][0])+(IkWpk[10][13][1]*
          Wpk[10][13][1]))))+temp[1]);
        mm[10][10] = (((mk[9]*((Vpk[10][14][2]*Vpk[10][14][2])+((Vpk[10][14][0]*
          Vpk[10][14][0])+(Vpk[10][14][1]*Vpk[10][14][1]))))+((IkWpk[10][14][2]*
          Wpk[10][14][2])+((IkWpk[10][14][0]*Wpk[10][14][0])+(IkWpk[10][14][1]*
          Wpk[10][14][1]))))+temp[2]);
        temp[0] = (((mk[6]*((Vpk[10][11][2]*Vpk[11][11][2])+((Vpk[10][11][0]*
          Vpk[11][11][0])+(Vpk[10][11][1]*Vpk[11][11][1]))))+((IkWpk[11][11][2]*
          Wpk[10][11][2])+((IkWpk[11][11][0]*Wpk[10][11][0])+(IkWpk[11][11][1]*
          Wpk[10][11][1]))))+((mk[7]*((Vpk[10][12][2]*Vpk[11][12][2])+((
          Vpk[10][12][0]*Vpk[11][12][0])+(Vpk[10][12][1]*Vpk[11][12][1]))))+((
          IkWpk[11][12][2]*Wpk[10][12][2])+((IkWpk[11][12][0]*Wpk[10][12][0])+(
          IkWpk[11][12][1]*Wpk[10][12][1])))));
        temp[1] = (((mk[8]*((Vpk[10][13][2]*Vpk[11][13][2])+((Vpk[10][13][0]*
          Vpk[11][13][0])+(Vpk[10][13][1]*Vpk[11][13][1]))))+((IkWpk[11][13][2]*
          Wpk[10][13][2])+((IkWpk[11][13][0]*Wpk[10][13][0])+(IkWpk[11][13][1]*
          Wpk[10][13][1]))))+temp[0]);
        mm[10][11] = (((mk[9]*((Vpk[10][14][2]*Vpk[11][14][2])+((Vpk[10][14][0]*
          Vpk[11][14][0])+(Vpk[10][14][1]*Vpk[11][14][1]))))+((IkWpk[11][14][2]*
          Wpk[10][14][2])+((IkWpk[11][14][0]*Wpk[10][14][0])+(IkWpk[11][14][1]*
          Wpk[10][14][1]))))+temp[1]);
        temp[0] = (((mk[7]*((Vpk[10][12][2]*Vpk[12][12][2])+((Vpk[10][12][0]*
          Vpk[12][12][0])+(Vpk[10][12][1]*Vpk[12][12][1]))))+((IkWpk[12][12][2]*
          Wpk[10][12][2])+((IkWpk[12][12][0]*Wpk[10][12][0])+(IkWpk[12][12][1]*
          Wpk[10][12][1]))))+((mk[8]*((Vpk[10][13][2]*Vpk[12][13][2])+((
          Vpk[10][13][0]*Vpk[12][13][0])+(Vpk[10][13][1]*Vpk[12][13][1]))))+((
          IkWpk[12][13][2]*Wpk[10][13][2])+((IkWpk[12][13][0]*Wpk[10][13][0])+(
          IkWpk[12][13][1]*Wpk[10][13][1])))));
        mm[10][12] = (((mk[9]*((Vpk[10][14][2]*Vpk[12][14][2])+((Vpk[10][14][0]*
          Vpk[12][14][0])+(Vpk[10][14][1]*Vpk[12][14][1]))))+((IkWpk[12][14][2]*
          Wpk[10][14][2])+((IkWpk[12][14][0]*Wpk[10][14][0])+(IkWpk[12][14][1]*
          Wpk[10][14][1]))))+temp[0]);
        mm[10][13] = (((mk[8]*((Vpk[10][13][2]*Vpk[13][13][2])+((Vpk[10][13][0]*
          Vpk[13][13][0])+(Vpk[10][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          Wpk[10][13][2])+((IkWpk[13][13][0]*Wpk[10][13][0])+(IkWpk[13][13][1]*
          Wpk[10][13][1]))))+((mk[9]*((Vpk[10][14][2]*Vpk[13][14][2])+((
          Vpk[10][14][0]*Vpk[13][14][0])+(Vpk[10][14][1]*Vpk[13][14][1]))))+((
          IkWpk[13][14][2]*Wpk[10][14][2])+((IkWpk[13][14][0]*Wpk[10][14][0])+(
          IkWpk[13][14][1]*Wpk[10][14][1])))));
        mm[10][14] = ((mk[9]*((Vpk[10][14][2]*Vpk[14][14][2])+((Vpk[10][14][0]*
          Vpk[14][14][0])+(Vpk[10][14][1]*Vpk[14][14][1]))))+((IkWpk[14][14][2]*
          Wpk[10][14][2])+((IkWpk[14][14][0]*Wpk[10][14][0])+(IkWpk[14][14][1]*
          Wpk[10][14][1]))));
        mm[10][15] = 0.;
        mm[10][16] = 0.;
        mm[10][17] = 0.;
        mm[10][18] = 0.;
        mm[10][19] = 0.;
        mm[10][20] = 0.;
        temp[0] = (((mk[6]*((Vpk[11][11][2]*Vpk[11][11][2])+((Vpk[11][11][0]*
          Vpk[11][11][0])+(Vpk[11][11][1]*Vpk[11][11][1]))))+((IkWpk[11][11][2]*
          pin[11][2])+((IkWpk[11][11][0]*pin[11][0])+(IkWpk[11][11][1]*
          pin[11][1]))))+((mk[7]*((Vpk[11][12][2]*Vpk[11][12][2])+((
          Vpk[11][12][0]*Vpk[11][12][0])+(Vpk[11][12][1]*Vpk[11][12][1]))))+((
          IkWpk[11][12][2]*Wpk[11][12][2])+((IkWpk[11][12][0]*Wpk[11][12][0])+(
          IkWpk[11][12][1]*Wpk[11][12][1])))));
        temp[1] = (((mk[8]*((Vpk[11][13][2]*Vpk[11][13][2])+((Vpk[11][13][0]*
          Vpk[11][13][0])+(Vpk[11][13][1]*Vpk[11][13][1]))))+((IkWpk[11][13][2]*
          Wpk[11][13][2])+((IkWpk[11][13][0]*Wpk[11][13][0])+(IkWpk[11][13][1]*
          Wpk[11][13][1]))))+temp[0]);
        mm[11][11] = (((mk[9]*((Vpk[11][14][2]*Vpk[11][14][2])+((Vpk[11][14][0]*
          Vpk[11][14][0])+(Vpk[11][14][1]*Vpk[11][14][1]))))+((IkWpk[11][14][2]*
          Wpk[11][14][2])+((IkWpk[11][14][0]*Wpk[11][14][0])+(IkWpk[11][14][1]*
          Wpk[11][14][1]))))+temp[1]);
        temp[0] = (((mk[7]*((Vpk[11][12][2]*Vpk[12][12][2])+((Vpk[11][12][0]*
          Vpk[12][12][0])+(Vpk[11][12][1]*Vpk[12][12][1]))))+((IkWpk[12][12][2]*
          Wpk[11][12][2])+((IkWpk[12][12][0]*Wpk[11][12][0])+(IkWpk[12][12][1]*
          Wpk[11][12][1]))))+((mk[8]*((Vpk[11][13][2]*Vpk[12][13][2])+((
          Vpk[11][13][0]*Vpk[12][13][0])+(Vpk[11][13][1]*Vpk[12][13][1]))))+((
          IkWpk[12][13][2]*Wpk[11][13][2])+((IkWpk[12][13][0]*Wpk[11][13][0])+(
          IkWpk[12][13][1]*Wpk[11][13][1])))));
        mm[11][12] = (((mk[9]*((Vpk[11][14][2]*Vpk[12][14][2])+((Vpk[11][14][0]*
          Vpk[12][14][0])+(Vpk[11][14][1]*Vpk[12][14][1]))))+((IkWpk[12][14][2]*
          Wpk[11][14][2])+((IkWpk[12][14][0]*Wpk[11][14][0])+(IkWpk[12][14][1]*
          Wpk[11][14][1]))))+temp[0]);
        mm[11][13] = (((mk[8]*((Vpk[11][13][2]*Vpk[13][13][2])+((Vpk[11][13][0]*
          Vpk[13][13][0])+(Vpk[11][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          Wpk[11][13][2])+((IkWpk[13][13][0]*Wpk[11][13][0])+(IkWpk[13][13][1]*
          Wpk[11][13][1]))))+((mk[9]*((Vpk[11][14][2]*Vpk[13][14][2])+((
          Vpk[11][14][0]*Vpk[13][14][0])+(Vpk[11][14][1]*Vpk[13][14][1]))))+((
          IkWpk[13][14][2]*Wpk[11][14][2])+((IkWpk[13][14][0]*Wpk[11][14][0])+(
          IkWpk[13][14][1]*Wpk[11][14][1])))));
        mm[11][14] = ((mk[9]*((Vpk[11][14][2]*Vpk[14][14][2])+((Vpk[11][14][0]*
          Vpk[14][14][0])+(Vpk[11][14][1]*Vpk[14][14][1]))))+((IkWpk[14][14][2]*
          Wpk[11][14][2])+((IkWpk[14][14][0]*Wpk[11][14][0])+(IkWpk[14][14][1]*
          Wpk[11][14][1]))));
        mm[11][15] = 0.;
        mm[11][16] = 0.;
        mm[11][17] = 0.;
        mm[11][18] = 0.;
        mm[11][19] = 0.;
        mm[11][20] = 0.;
        temp[0] = (((mk[7]*((Vpk[12][12][2]*Vpk[12][12][2])+((Vpk[12][12][0]*
          Vpk[12][12][0])+(Vpk[12][12][1]*Vpk[12][12][1]))))+((IkWpk[12][12][2]*
          pin[12][2])+((IkWpk[12][12][0]*pin[12][0])+(IkWpk[12][12][1]*
          pin[12][1]))))+((mk[8]*((Vpk[12][13][2]*Vpk[12][13][2])+((
          Vpk[12][13][0]*Vpk[12][13][0])+(Vpk[12][13][1]*Vpk[12][13][1]))))+((
          IkWpk[12][13][2]*Wpk[12][13][2])+((IkWpk[12][13][0]*Wpk[12][13][0])+(
          IkWpk[12][13][1]*Wpk[12][13][1])))));
        mm[12][12] = (((mk[9]*((Vpk[12][14][2]*Vpk[12][14][2])+((Vpk[12][14][0]*
          Vpk[12][14][0])+(Vpk[12][14][1]*Vpk[12][14][1]))))+((IkWpk[12][14][2]*
          Wpk[12][14][2])+((IkWpk[12][14][0]*Wpk[12][14][0])+(IkWpk[12][14][1]*
          Wpk[12][14][1]))))+temp[0]);
        mm[12][13] = (((mk[8]*((Vpk[12][13][2]*Vpk[13][13][2])+((Vpk[12][13][0]*
          Vpk[13][13][0])+(Vpk[12][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          Wpk[12][13][2])+((IkWpk[13][13][0]*Wpk[12][13][0])+(IkWpk[13][13][1]*
          Wpk[12][13][1]))))+((mk[9]*((Vpk[12][14][2]*Vpk[13][14][2])+((
          Vpk[12][14][0]*Vpk[13][14][0])+(Vpk[12][14][1]*Vpk[13][14][1]))))+((
          IkWpk[13][14][2]*Wpk[12][14][2])+((IkWpk[13][14][0]*Wpk[12][14][0])+(
          IkWpk[13][14][1]*Wpk[12][14][1])))));
        mm[12][14] = ((mk[9]*((Vpk[12][14][2]*Vpk[14][14][2])+((Vpk[12][14][0]*
          Vpk[14][14][0])+(Vpk[12][14][1]*Vpk[14][14][1]))))+((IkWpk[14][14][2]*
          Wpk[12][14][2])+((IkWpk[14][14][0]*Wpk[12][14][0])+(IkWpk[14][14][1]*
          Wpk[12][14][1]))));
        mm[12][15] = 0.;
        mm[12][16] = 0.;
        mm[12][17] = 0.;
        mm[12][18] = 0.;
        mm[12][19] = 0.;
        mm[12][20] = 0.;
        mm[13][13] = (((mk[8]*((Vpk[13][13][2]*Vpk[13][13][2])+((Vpk[13][13][0]*
          Vpk[13][13][0])+(Vpk[13][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          pin[13][2])+((IkWpk[13][13][0]*pin[13][0])+(IkWpk[13][13][1]*
          pin[13][1]))))+((mk[9]*((Vpk[13][14][2]*Vpk[13][14][2])+((
          Vpk[13][14][0]*Vpk[13][14][0])+(Vpk[13][14][1]*Vpk[13][14][1]))))+((
          IkWpk[13][14][2]*Wpk[13][14][2])+((IkWpk[13][14][0]*Wpk[13][14][0])+(
          IkWpk[13][14][1]*Wpk[13][14][1])))));
        mm[13][14] = ((mk[9]*((Vpk[13][14][2]*Vpk[14][14][2])+((Vpk[13][14][0]*
          Vpk[14][14][0])+(Vpk[13][14][1]*Vpk[14][14][1]))))+((IkWpk[14][14][2]*
          Wpk[13][14][2])+((IkWpk[14][14][0]*Wpk[13][14][0])+(IkWpk[14][14][1]*
          Wpk[13][14][1]))));
        mm[13][15] = 0.;
        mm[13][16] = 0.;
        mm[13][17] = 0.;
        mm[13][18] = 0.;
        mm[13][19] = 0.;
        mm[13][20] = 0.;
        mm[14][14] = ((mk[9]*((Vpk[14][14][2]*Vpk[14][14][2])+((Vpk[14][14][0]*
          Vpk[14][14][0])+(Vpk[14][14][1]*Vpk[14][14][1]))))+((IkWpk[14][14][2]*
          pin[14][2])+((IkWpk[14][14][0]*pin[14][0])+(IkWpk[14][14][1]*
          pin[14][1]))));
        mm[14][15] = 0.;
        mm[14][16] = 0.;
        mm[14][17] = 0.;
        mm[14][18] = 0.;
        mm[14][19] = 0.;
        mm[14][20] = 0.;
        temp[0] = (((mk[10]*((Vpk[15][15][2]*Vpk[15][15][2])+((Vpk[15][15][0]*
          Vpk[15][15][0])+(Vpk[15][15][1]*Vpk[15][15][1]))))+((IkWpk[15][15][2]*
          pin[15][2])+((IkWpk[15][15][0]*pin[15][0])+(IkWpk[15][15][1]*
          pin[15][1]))))+((mk[11]*((Vpk[15][16][2]*Vpk[15][16][2])+((
          Vpk[15][16][0]*Vpk[15][16][0])+(Vpk[15][16][1]*Vpk[15][16][1]))))+((
          IkWpk[15][16][2]*Wpk[15][16][2])+((IkWpk[15][16][0]*Wpk[15][16][0])+(
          IkWpk[15][16][1]*Wpk[15][16][1])))));
        temp[1] = (((mk[12]*((Vpk[15][17][2]*Vpk[15][17][2])+((Vpk[15][17][0]*
          Vpk[15][17][0])+(Vpk[15][17][1]*Vpk[15][17][1]))))+((IkWpk[15][17][2]*
          Wpk[15][17][2])+((IkWpk[15][17][0]*Wpk[15][17][0])+(IkWpk[15][17][1]*
          Wpk[15][17][1]))))+temp[0]);
        temp[2] = (((mk[13]*((Vpk[15][18][2]*Vpk[15][18][2])+((Vpk[15][18][0]*
          Vpk[15][18][0])+(Vpk[15][18][1]*Vpk[15][18][1]))))+((IkWpk[15][18][2]*
          Wpk[15][18][2])+((IkWpk[15][18][0]*Wpk[15][18][0])+(IkWpk[15][18][1]*
          Wpk[15][18][1]))))+temp[1]);
        temp[3] = (((mk[14]*((Vpk[15][19][2]*Vpk[15][19][2])+((Vpk[15][19][0]*
          Vpk[15][19][0])+(Vpk[15][19][1]*Vpk[15][19][1]))))+((IkWpk[15][19][2]*
          Wpk[15][19][2])+((IkWpk[15][19][0]*Wpk[15][19][0])+(IkWpk[15][19][1]*
          Wpk[15][19][1]))))+temp[2]);
        mm[15][15] = (((mk[15]*((Vpk[15][20][2]*Vpk[15][20][2])+((Vpk[15][20][0]
          *Vpk[15][20][0])+(Vpk[15][20][1]*Vpk[15][20][1]))))+((IkWpk[15][20][2]
          *Wpk[15][20][2])+((IkWpk[15][20][0]*Wpk[15][20][0])+(IkWpk[15][20][1]*
          Wpk[15][20][1]))))+temp[3]);
        temp[0] = (((mk[11]*((Vpk[15][16][2]*Vpk[16][16][2])+((Vpk[15][16][0]*
          Vpk[16][16][0])+(Vpk[15][16][1]*Vpk[16][16][1]))))+((IkWpk[16][16][2]*
          Wpk[15][16][2])+((IkWpk[16][16][0]*Wpk[15][16][0])+(IkWpk[16][16][1]*
          Wpk[15][16][1]))))+((mk[12]*((Vpk[15][17][2]*Vpk[16][17][2])+((
          Vpk[15][17][0]*Vpk[16][17][0])+(Vpk[15][17][1]*Vpk[16][17][1]))))+((
          IkWpk[16][17][2]*Wpk[15][17][2])+((IkWpk[16][17][0]*Wpk[15][17][0])+(
          IkWpk[16][17][1]*Wpk[15][17][1])))));
        temp[1] = (((mk[13]*((Vpk[15][18][2]*Vpk[16][18][2])+((Vpk[15][18][0]*
          Vpk[16][18][0])+(Vpk[15][18][1]*Vpk[16][18][1]))))+((IkWpk[16][18][2]*
          Wpk[15][18][2])+((IkWpk[16][18][0]*Wpk[15][18][0])+(IkWpk[16][18][1]*
          Wpk[15][18][1]))))+temp[0]);
        temp[2] = (((mk[14]*((Vpk[15][19][2]*Vpk[16][19][2])+((Vpk[15][19][0]*
          Vpk[16][19][0])+(Vpk[15][19][1]*Vpk[16][19][1]))))+((IkWpk[16][19][2]*
          Wpk[15][19][2])+((IkWpk[16][19][0]*Wpk[15][19][0])+(IkWpk[16][19][1]*
          Wpk[15][19][1]))))+temp[1]);
        mm[15][16] = (((mk[15]*((Vpk[15][20][2]*Vpk[16][20][2])+((Vpk[15][20][0]
          *Vpk[16][20][0])+(Vpk[15][20][1]*Vpk[16][20][1]))))+((IkWpk[16][20][2]
          *Wpk[15][20][2])+((IkWpk[16][20][0]*Wpk[15][20][0])+(IkWpk[16][20][1]*
          Wpk[15][20][1]))))+temp[2]);
        temp[0] = (((mk[12]*((Vpk[15][17][2]*Vpk[17][17][2])+((Vpk[15][17][0]*
          Vpk[17][17][0])+(Vpk[15][17][1]*Vpk[17][17][1]))))+((IkWpk[17][17][2]*
          Wpk[15][17][2])+((IkWpk[17][17][0]*Wpk[15][17][0])+(IkWpk[17][17][1]*
          Wpk[15][17][1]))))+((mk[13]*((Vpk[15][18][2]*Vpk[17][18][2])+((
          Vpk[15][18][0]*Vpk[17][18][0])+(Vpk[15][18][1]*Vpk[17][18][1]))))+((
          IkWpk[17][18][2]*Wpk[15][18][2])+((IkWpk[17][18][0]*Wpk[15][18][0])+(
          IkWpk[17][18][1]*Wpk[15][18][1])))));
        temp[1] = (((mk[14]*((Vpk[15][19][2]*Vpk[17][19][2])+((Vpk[15][19][0]*
          Vpk[17][19][0])+(Vpk[15][19][1]*Vpk[17][19][1]))))+((IkWpk[17][19][2]*
          Wpk[15][19][2])+((IkWpk[17][19][0]*Wpk[15][19][0])+(IkWpk[17][19][1]*
          Wpk[15][19][1]))))+temp[0]);
        mm[15][17] = (((mk[15]*((Vpk[15][20][2]*Vpk[17][20][2])+((Vpk[15][20][0]
          *Vpk[17][20][0])+(Vpk[15][20][1]*Vpk[17][20][1]))))+((IkWpk[17][20][2]
          *Wpk[15][20][2])+((IkWpk[17][20][0]*Wpk[15][20][0])+(IkWpk[17][20][1]*
          Wpk[15][20][1]))))+temp[1]);
        temp[0] = (((mk[13]*((Vpk[15][18][2]*Vpk[18][18][2])+((Vpk[15][18][0]*
          Vpk[18][18][0])+(Vpk[15][18][1]*Vpk[18][18][1]))))+((IkWpk[18][18][2]*
          Wpk[15][18][2])+((IkWpk[18][18][0]*Wpk[15][18][0])+(IkWpk[18][18][1]*
          Wpk[15][18][1]))))+((mk[14]*((Vpk[15][19][2]*Vpk[18][19][2])+((
          Vpk[15][19][0]*Vpk[18][19][0])+(Vpk[15][19][1]*Vpk[18][19][1]))))+((
          IkWpk[18][19][2]*Wpk[15][19][2])+((IkWpk[18][19][0]*Wpk[15][19][0])+(
          IkWpk[18][19][1]*Wpk[15][19][1])))));
        mm[15][18] = (((mk[15]*((Vpk[15][20][2]*Vpk[18][20][2])+((Vpk[15][20][0]
          *Vpk[18][20][0])+(Vpk[15][20][1]*Vpk[18][20][1]))))+((IkWpk[18][20][2]
          *Wpk[15][20][2])+((IkWpk[18][20][0]*Wpk[15][20][0])+(IkWpk[18][20][1]*
          Wpk[15][20][1]))))+temp[0]);
        mm[15][19] = (((mk[14]*((Vpk[15][19][2]*Vpk[19][19][2])+((Vpk[15][19][0]
          *Vpk[19][19][0])+(Vpk[15][19][1]*Vpk[19][19][1]))))+((IkWpk[19][19][2]
          *Wpk[15][19][2])+((IkWpk[19][19][0]*Wpk[15][19][0])+(IkWpk[19][19][1]*
          Wpk[15][19][1]))))+((mk[15]*((Vpk[15][20][2]*Vpk[19][20][2])+((
          Vpk[15][20][0]*Vpk[19][20][0])+(Vpk[15][20][1]*Vpk[19][20][1]))))+((
          IkWpk[19][20][2]*Wpk[15][20][2])+((IkWpk[19][20][0]*Wpk[15][20][0])+(
          IkWpk[19][20][1]*Wpk[15][20][1])))));
        mm[15][20] = ((mk[15]*((Vpk[15][20][2]*Vpk[20][20][2])+((Vpk[15][20][0]*
          Vpk[20][20][0])+(Vpk[15][20][1]*Vpk[20][20][1]))))+((IkWpk[20][20][2]*
          Wpk[15][20][2])+((IkWpk[20][20][0]*Wpk[15][20][0])+(IkWpk[20][20][1]*
          Wpk[15][20][1]))));
        temp[0] = (((mk[11]*((Vpk[16][16][2]*Vpk[16][16][2])+((Vpk[16][16][0]*
          Vpk[16][16][0])+(Vpk[16][16][1]*Vpk[16][16][1]))))+((IkWpk[16][16][2]*
          pin[16][2])+((IkWpk[16][16][0]*pin[16][0])+(IkWpk[16][16][1]*
          pin[16][1]))))+((mk[12]*((Vpk[16][17][2]*Vpk[16][17][2])+((
          Vpk[16][17][0]*Vpk[16][17][0])+(Vpk[16][17][1]*Vpk[16][17][1]))))+((
          IkWpk[16][17][2]*Wpk[16][17][2])+((IkWpk[16][17][0]*Wpk[16][17][0])+(
          IkWpk[16][17][1]*Wpk[16][17][1])))));
        temp[1] = (((mk[13]*((Vpk[16][18][2]*Vpk[16][18][2])+((Vpk[16][18][0]*
          Vpk[16][18][0])+(Vpk[16][18][1]*Vpk[16][18][1]))))+((IkWpk[16][18][2]*
          Wpk[16][18][2])+((IkWpk[16][18][0]*Wpk[16][18][0])+(IkWpk[16][18][1]*
          Wpk[16][18][1]))))+temp[0]);
        temp[2] = (((mk[14]*((Vpk[16][19][2]*Vpk[16][19][2])+((Vpk[16][19][0]*
          Vpk[16][19][0])+(Vpk[16][19][1]*Vpk[16][19][1]))))+((IkWpk[16][19][2]*
          Wpk[16][19][2])+((IkWpk[16][19][0]*Wpk[16][19][0])+(IkWpk[16][19][1]*
          Wpk[16][19][1]))))+temp[1]);
        mm[16][16] = (((mk[15]*((Vpk[16][20][2]*Vpk[16][20][2])+((Vpk[16][20][0]
          *Vpk[16][20][0])+(Vpk[16][20][1]*Vpk[16][20][1]))))+((IkWpk[16][20][2]
          *Wpk[16][20][2])+((IkWpk[16][20][0]*Wpk[16][20][0])+(IkWpk[16][20][1]*
          Wpk[16][20][1]))))+temp[2]);
        temp[0] = (((mk[12]*((Vpk[16][17][2]*Vpk[17][17][2])+((Vpk[16][17][0]*
          Vpk[17][17][0])+(Vpk[16][17][1]*Vpk[17][17][1]))))+((IkWpk[17][17][2]*
          Wpk[16][17][2])+((IkWpk[17][17][0]*Wpk[16][17][0])+(IkWpk[17][17][1]*
          Wpk[16][17][1]))))+((mk[13]*((Vpk[16][18][2]*Vpk[17][18][2])+((
          Vpk[16][18][0]*Vpk[17][18][0])+(Vpk[16][18][1]*Vpk[17][18][1]))))+((
          IkWpk[17][18][2]*Wpk[16][18][2])+((IkWpk[17][18][0]*Wpk[16][18][0])+(
          IkWpk[17][18][1]*Wpk[16][18][1])))));
        temp[1] = (((mk[14]*((Vpk[16][19][2]*Vpk[17][19][2])+((Vpk[16][19][0]*
          Vpk[17][19][0])+(Vpk[16][19][1]*Vpk[17][19][1]))))+((IkWpk[17][19][2]*
          Wpk[16][19][2])+((IkWpk[17][19][0]*Wpk[16][19][0])+(IkWpk[17][19][1]*
          Wpk[16][19][1]))))+temp[0]);
        mm[16][17] = (((mk[15]*((Vpk[16][20][2]*Vpk[17][20][2])+((Vpk[16][20][0]
          *Vpk[17][20][0])+(Vpk[16][20][1]*Vpk[17][20][1]))))+((IkWpk[17][20][2]
          *Wpk[16][20][2])+((IkWpk[17][20][0]*Wpk[16][20][0])+(IkWpk[17][20][1]*
          Wpk[16][20][1]))))+temp[1]);
        temp[0] = (((mk[13]*((Vpk[16][18][2]*Vpk[18][18][2])+((Vpk[16][18][0]*
          Vpk[18][18][0])+(Vpk[16][18][1]*Vpk[18][18][1]))))+((IkWpk[18][18][2]*
          Wpk[16][18][2])+((IkWpk[18][18][0]*Wpk[16][18][0])+(IkWpk[18][18][1]*
          Wpk[16][18][1]))))+((mk[14]*((Vpk[16][19][2]*Vpk[18][19][2])+((
          Vpk[16][19][0]*Vpk[18][19][0])+(Vpk[16][19][1]*Vpk[18][19][1]))))+((
          IkWpk[18][19][2]*Wpk[16][19][2])+((IkWpk[18][19][0]*Wpk[16][19][0])+(
          IkWpk[18][19][1]*Wpk[16][19][1])))));
        mm[16][18] = (((mk[15]*((Vpk[16][20][2]*Vpk[18][20][2])+((Vpk[16][20][0]
          *Vpk[18][20][0])+(Vpk[16][20][1]*Vpk[18][20][1]))))+((IkWpk[18][20][2]
          *Wpk[16][20][2])+((IkWpk[18][20][0]*Wpk[16][20][0])+(IkWpk[18][20][1]*
          Wpk[16][20][1]))))+temp[0]);
        mm[16][19] = (((mk[14]*((Vpk[16][19][2]*Vpk[19][19][2])+((Vpk[16][19][0]
          *Vpk[19][19][0])+(Vpk[16][19][1]*Vpk[19][19][1]))))+((IkWpk[19][19][2]
          *Wpk[16][19][2])+((IkWpk[19][19][0]*Wpk[16][19][0])+(IkWpk[19][19][1]*
          Wpk[16][19][1]))))+((mk[15]*((Vpk[16][20][2]*Vpk[19][20][2])+((
          Vpk[16][20][0]*Vpk[19][20][0])+(Vpk[16][20][1]*Vpk[19][20][1]))))+((
          IkWpk[19][20][2]*Wpk[16][20][2])+((IkWpk[19][20][0]*Wpk[16][20][0])+(
          IkWpk[19][20][1]*Wpk[16][20][1])))));
        mm[16][20] = ((mk[15]*((Vpk[16][20][2]*Vpk[20][20][2])+((Vpk[16][20][0]*
          Vpk[20][20][0])+(Vpk[16][20][1]*Vpk[20][20][1]))))+((IkWpk[20][20][2]*
          Wpk[16][20][2])+((IkWpk[20][20][0]*Wpk[16][20][0])+(IkWpk[20][20][1]*
          Wpk[16][20][1]))));
        temp[0] = (((mk[12]*((Vpk[17][17][2]*Vpk[17][17][2])+((Vpk[17][17][0]*
          Vpk[17][17][0])+(Vpk[17][17][1]*Vpk[17][17][1]))))+((IkWpk[17][17][2]*
          pin[17][2])+((IkWpk[17][17][0]*pin[17][0])+(IkWpk[17][17][1]*
          pin[17][1]))))+((mk[13]*((Vpk[17][18][2]*Vpk[17][18][2])+((
          Vpk[17][18][0]*Vpk[17][18][0])+(Vpk[17][18][1]*Vpk[17][18][1]))))+((
          IkWpk[17][18][2]*Wpk[17][18][2])+((IkWpk[17][18][0]*Wpk[17][18][0])+(
          IkWpk[17][18][1]*Wpk[17][18][1])))));
        temp[1] = (((mk[14]*((Vpk[17][19][2]*Vpk[17][19][2])+((Vpk[17][19][0]*
          Vpk[17][19][0])+(Vpk[17][19][1]*Vpk[17][19][1]))))+((IkWpk[17][19][2]*
          Wpk[17][19][2])+((IkWpk[17][19][0]*Wpk[17][19][0])+(IkWpk[17][19][1]*
          Wpk[17][19][1]))))+temp[0]);
        mm[17][17] = (((mk[15]*((Vpk[17][20][2]*Vpk[17][20][2])+((Vpk[17][20][0]
          *Vpk[17][20][0])+(Vpk[17][20][1]*Vpk[17][20][1]))))+((IkWpk[17][20][2]
          *Wpk[17][20][2])+((IkWpk[17][20][0]*Wpk[17][20][0])+(IkWpk[17][20][1]*
          Wpk[17][20][1]))))+temp[1]);
        temp[0] = (((mk[13]*((Vpk[17][18][2]*Vpk[18][18][2])+((Vpk[17][18][0]*
          Vpk[18][18][0])+(Vpk[17][18][1]*Vpk[18][18][1]))))+((IkWpk[18][18][2]*
          Wpk[17][18][2])+((IkWpk[18][18][0]*Wpk[17][18][0])+(IkWpk[18][18][1]*
          Wpk[17][18][1]))))+((mk[14]*((Vpk[17][19][2]*Vpk[18][19][2])+((
          Vpk[17][19][0]*Vpk[18][19][0])+(Vpk[17][19][1]*Vpk[18][19][1]))))+((
          IkWpk[18][19][2]*Wpk[17][19][2])+((IkWpk[18][19][0]*Wpk[17][19][0])+(
          IkWpk[18][19][1]*Wpk[17][19][1])))));
        mm[17][18] = (((mk[15]*((Vpk[17][20][2]*Vpk[18][20][2])+((Vpk[17][20][0]
          *Vpk[18][20][0])+(Vpk[17][20][1]*Vpk[18][20][1]))))+((IkWpk[18][20][2]
          *Wpk[17][20][2])+((IkWpk[18][20][0]*Wpk[17][20][0])+(IkWpk[18][20][1]*
          Wpk[17][20][1]))))+temp[0]);
        mm[17][19] = (((mk[14]*((Vpk[17][19][2]*Vpk[19][19][2])+((Vpk[17][19][0]
          *Vpk[19][19][0])+(Vpk[17][19][1]*Vpk[19][19][1]))))+((IkWpk[19][19][2]
          *Wpk[17][19][2])+((IkWpk[19][19][0]*Wpk[17][19][0])+(IkWpk[19][19][1]*
          Wpk[17][19][1]))))+((mk[15]*((Vpk[17][20][2]*Vpk[19][20][2])+((
          Vpk[17][20][0]*Vpk[19][20][0])+(Vpk[17][20][1]*Vpk[19][20][1]))))+((
          IkWpk[19][20][2]*Wpk[17][20][2])+((IkWpk[19][20][0]*Wpk[17][20][0])+(
          IkWpk[19][20][1]*Wpk[17][20][1])))));
        mm[17][20] = ((mk[15]*((Vpk[17][20][2]*Vpk[20][20][2])+((Vpk[17][20][0]*
          Vpk[20][20][0])+(Vpk[17][20][1]*Vpk[20][20][1]))))+((IkWpk[20][20][2]*
          Wpk[17][20][2])+((IkWpk[20][20][0]*Wpk[17][20][0])+(IkWpk[20][20][1]*
          Wpk[17][20][1]))));
        temp[0] = (((mk[13]*((Vpk[18][18][2]*Vpk[18][18][2])+((Vpk[18][18][0]*
          Vpk[18][18][0])+(Vpk[18][18][1]*Vpk[18][18][1]))))+((IkWpk[18][18][2]*
          pin[18][2])+((IkWpk[18][18][0]*pin[18][0])+(IkWpk[18][18][1]*
          pin[18][1]))))+((mk[14]*((Vpk[18][19][2]*Vpk[18][19][2])+((
          Vpk[18][19][0]*Vpk[18][19][0])+(Vpk[18][19][1]*Vpk[18][19][1]))))+((
          IkWpk[18][19][2]*Wpk[18][19][2])+((IkWpk[18][19][0]*Wpk[18][19][0])+(
          IkWpk[18][19][1]*Wpk[18][19][1])))));
        mm[18][18] = (((mk[15]*((Vpk[18][20][2]*Vpk[18][20][2])+((Vpk[18][20][0]
          *Vpk[18][20][0])+(Vpk[18][20][1]*Vpk[18][20][1]))))+((IkWpk[18][20][2]
          *Wpk[18][20][2])+((IkWpk[18][20][0]*Wpk[18][20][0])+(IkWpk[18][20][1]*
          Wpk[18][20][1]))))+temp[0]);
        mm[18][19] = (((mk[14]*((Vpk[18][19][2]*Vpk[19][19][2])+((Vpk[18][19][0]
          *Vpk[19][19][0])+(Vpk[18][19][1]*Vpk[19][19][1]))))+((IkWpk[19][19][2]
          *Wpk[18][19][2])+((IkWpk[19][19][0]*Wpk[18][19][0])+(IkWpk[19][19][1]*
          Wpk[18][19][1]))))+((mk[15]*((Vpk[18][20][2]*Vpk[19][20][2])+((
          Vpk[18][20][0]*Vpk[19][20][0])+(Vpk[18][20][1]*Vpk[19][20][1]))))+((
          IkWpk[19][20][2]*Wpk[18][20][2])+((IkWpk[19][20][0]*Wpk[18][20][0])+(
          IkWpk[19][20][1]*Wpk[18][20][1])))));
        mm[18][20] = ((mk[15]*((Vpk[18][20][2]*Vpk[20][20][2])+((Vpk[18][20][0]*
          Vpk[20][20][0])+(Vpk[18][20][1]*Vpk[20][20][1]))))+((IkWpk[20][20][2]*
          Wpk[18][20][2])+((IkWpk[20][20][0]*Wpk[18][20][0])+(IkWpk[20][20][1]*
          Wpk[18][20][1]))));
        mm[19][19] = (((mk[14]*((Vpk[19][19][2]*Vpk[19][19][2])+((Vpk[19][19][0]
          *Vpk[19][19][0])+(Vpk[19][19][1]*Vpk[19][19][1]))))+((IkWpk[19][19][2]
          *pin[19][2])+((IkWpk[19][19][0]*pin[19][0])+(IkWpk[19][19][1]*
          pin[19][1]))))+((mk[15]*((Vpk[19][20][2]*Vpk[19][20][2])+((
          Vpk[19][20][0]*Vpk[19][20][0])+(Vpk[19][20][1]*Vpk[19][20][1]))))+((
          IkWpk[19][20][2]*Wpk[19][20][2])+((IkWpk[19][20][0]*Wpk[19][20][0])+(
          IkWpk[19][20][1]*Wpk[19][20][1])))));
        mm[19][20] = ((mk[15]*((Vpk[19][20][2]*Vpk[20][20][2])+((Vpk[19][20][0]*
          Vpk[20][20][0])+(Vpk[19][20][1]*Vpk[20][20][1]))))+((IkWpk[20][20][2]*
          Wpk[19][20][2])+((IkWpk[20][20][0]*Wpk[19][20][0])+(IkWpk[20][20][1]*
          Wpk[19][20][1]))));
        mm[20][20] = ((mk[15]*((Vpk[20][20][2]*Vpk[20][20][2])+((Vpk[20][20][0]*
          Vpk[20][20][0])+(Vpk[20][20][1]*Vpk[20][20][1]))))+((IkWpk[20][20][2]*
          pin[20][2])+((IkWpk[20][20][0]*pin[20][0])+(IkWpk[20][20][1]*
          pin[20][1]))));
/*
Check for singular mass matrix
*/
        for (i = 0; i < 21; i++) {
            if (mm[i][i] < 1e-13) {
                sdseterr(routine,47);
            }
        }
        sderror(&dumroutine,&errnum);
        if (errnum == 0) {
            mmflg = 1;
        }
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain 3693 adds/subtracts/negates
                   4871 multiplies
                      0 divides
                    782 assignments
*/
}

void sddommldu(int routine)
{
    int i;
    int dumroutine,errnum;

    if (mmlduflg == 0) {
        sddomm(routine);
/*
Numerically decompose the mass matrix
*/
        sdldudcomp(21,21,mmap,1e-13,workss,works,mm,mlo,mdi);
/*
Check for singular mass matrix
*/
        for (i = 0; i < 21; i++) {
            if (mdi[i] <= 1e-13) {
                sdseterr(routine,47);
            }
        }
        sderror(&dumroutine,&errnum);
        if (errnum == 0) {
            mmlduflg = 1;
        }
    }
}

void sdlhs(int routine)
{
/* Compute all remaining state- and force-dependent quantities
*/

    roustate = 2;
    sddommldu(routine);
    sddofs0();
}

void sdmfrc(double imult[21])
{
/*
Calculate forces due to constraint multipliers.

*/

    if (pres[0]  !=  0.) {
        mtau[0] = imult[0];
    } else {
        mtau[0] = 0.;
    }
    if (pres[1]  !=  0.) {
        mtau[1] = imult[1];
    } else {
        mtau[1] = 0.;
    }
    if (pres[2]  !=  0.) {
        mtau[2] = imult[2];
    } else {
        mtau[2] = 0.;
    }
    if (pres[3]  !=  0.) {
        mtau[3] = imult[3];
    } else {
        mtau[3] = 0.;
    }
    if (pres[4]  !=  0.) {
        mtau[4] = imult[4];
    } else {
        mtau[4] = 0.;
    }
    if (pres[5]  !=  0.) {
        mtau[5] = imult[5];
    } else {
        mtau[5] = 0.;
    }
    if (pres[6]  !=  0.) {
        mtau[6] = imult[6];
    } else {
        mtau[6] = 0.;
    }
    if (pres[7]  !=  0.) {
        mtau[7] = imult[7];
    } else {
        mtau[7] = 0.;
    }
    if (pres[8]  !=  0.) {
        mtau[8] = imult[8];
    } else {
        mtau[8] = 0.;
    }
    if (pres[9]  !=  0.) {
        mtau[9] = imult[9];
    } else {
        mtau[9] = 0.;
    }
    if (pres[10]  !=  0.) {
        mtau[10] = imult[10];
    } else {
        mtau[10] = 0.;
    }
    if (pres[11]  !=  0.) {
        mtau[11] = imult[11];
    } else {
        mtau[11] = 0.;
    }
    if (pres[12]  !=  0.) {
        mtau[12] = imult[12];
    } else {
        mtau[12] = 0.;
    }
    if (pres[13]  !=  0.) {
        mtau[13] = imult[13];
    } else {
        mtau[13] = 0.;
    }
    if (pres[14]  !=  0.) {
        mtau[14] = imult[14];
    } else {
        mtau[14] = 0.;
    }
    if (pres[15]  !=  0.) {
        mtau[15] = imult[15];
    } else {
        mtau[15] = 0.;
    }
    if (pres[16]  !=  0.) {
        mtau[16] = imult[16];
    } else {
        mtau[16] = 0.;
    }
    if (pres[17]  !=  0.) {
        mtau[17] = imult[17];
    } else {
        mtau[17] = 0.;
    }
    if (pres[18]  !=  0.) {
        mtau[18] = imult[18];
    } else {
        mtau[18] = 0.;
    }
    if (pres[19]  !=  0.) {
        mtau[19] = imult[19];
    } else {
        mtau[19] = 0.;
    }
    if (pres[20]  !=  0.) {
        mtau[20] = imult[20];
    } else {
        mtau[20] = 0.;
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     42 assignments
*/
}

void sdequivht(double tau[21])
{
/* Compute tree hinge torques to match effect of applied loads
*/
    double fstareq[21][3],tstareq[21][3];

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(56,23);
        return;
    }
/*
Compute fstareq (forces)
*/
    fstareq[5][0] = -(ufk[0][0]+(gk[3][0]*mk[0]));
    fstareq[5][1] = -(ufk[0][1]+(gk[3][1]*mk[0]));
    fstareq[5][2] = -(ufk[0][2]+(gk[3][2]*mk[0]));
    fstareq[6][0] = -(ufk[1][0]+(gk[6][0]*mk[1]));
    fstareq[6][1] = -(ufk[1][1]+(gk[6][1]*mk[1]));
    fstareq[6][2] = -(ufk[1][2]+(gk[6][2]*mk[1]));
    fstareq[7][0] = -(ufk[2][0]+(gk[7][0]*mk[2]));
    fstareq[7][1] = -(ufk[2][1]+(gk[7][1]*mk[2]));
    fstareq[7][2] = -(ufk[2][2]+(gk[7][2]*mk[2]));
    fstareq[8][0] = -(ufk[3][0]+(gk[8][0]*mk[3]));
    fstareq[8][1] = -(ufk[3][1]+(gk[8][1]*mk[3]));
    fstareq[8][2] = -(ufk[3][2]+(gk[8][2]*mk[3]));
    fstareq[9][0] = -(ufk[4][0]+(gk[9][0]*mk[4]));
    fstareq[9][1] = -(ufk[4][1]+(gk[9][1]*mk[4]));
    fstareq[9][2] = -(ufk[4][2]+(gk[9][2]*mk[4]));
    fstareq[10][0] = -(ufk[5][0]+(gk[10][0]*mk[5]));
    fstareq[10][1] = -(ufk[5][1]+(gk[10][1]*mk[5]));
    fstareq[10][2] = -(ufk[5][2]+(gk[10][2]*mk[5]));
    fstareq[11][0] = -(ufk[6][0]+(gk[11][0]*mk[6]));
    fstareq[11][1] = -(ufk[6][1]+(gk[11][1]*mk[6]));
    fstareq[11][2] = -(ufk[6][2]+(gk[11][2]*mk[6]));
    fstareq[12][0] = -(ufk[7][0]+(gk[12][0]*mk[7]));
    fstareq[12][1] = -(ufk[7][1]+(gk[12][1]*mk[7]));
    fstareq[12][2] = -(ufk[7][2]+(gk[12][2]*mk[7]));
    fstareq[13][0] = -(ufk[8][0]+(gk[13][0]*mk[8]));
    fstareq[13][1] = -(ufk[8][1]+(gk[13][1]*mk[8]));
    fstareq[13][2] = -(ufk[8][2]+(gk[13][2]*mk[8]));
    fstareq[14][0] = -(ufk[9][0]+(gk[14][0]*mk[9]));
    fstareq[14][1] = -(ufk[9][1]+(gk[14][1]*mk[9]));
    fstareq[14][2] = -(ufk[9][2]+(gk[14][2]*mk[9]));
    fstareq[15][0] = -(ufk[10][0]+(gk[15][0]*mk[10]));
    fstareq[15][1] = -(ufk[10][1]+(gk[15][1]*mk[10]));
    fstareq[15][2] = -(ufk[10][2]+(gk[15][2]*mk[10]));
    fstareq[16][0] = -(ufk[11][0]+(gk[16][0]*mk[11]));
    fstareq[16][1] = -(ufk[11][1]+(gk[16][1]*mk[11]));
    fstareq[16][2] = -(ufk[11][2]+(gk[16][2]*mk[11]));
    fstareq[17][0] = -(ufk[12][0]+(gk[17][0]*mk[12]));
    fstareq[17][1] = -(ufk[12][1]+(gk[17][1]*mk[12]));
    fstareq[17][2] = -(ufk[12][2]+(gk[17][2]*mk[12]));
    fstareq[18][0] = -(ufk[13][0]+(gk[18][0]*mk[13]));
    fstareq[18][1] = -(ufk[13][1]+(gk[18][1]*mk[13]));
    fstareq[18][2] = -(ufk[13][2]+(gk[18][2]*mk[13]));
    fstareq[19][0] = -(ufk[14][0]+(gk[19][0]*mk[14]));
    fstareq[19][1] = -(ufk[14][1]+(gk[19][1]*mk[14]));
    fstareq[19][2] = -(ufk[14][2]+(gk[19][2]*mk[14]));
    fstareq[20][0] = -(ufk[15][0]+(gk[20][0]*mk[15]));
    fstareq[20][1] = -(ufk[15][1]+(gk[20][1]*mk[15]));
    fstareq[20][2] = -(ufk[15][2]+(gk[20][2]*mk[15]));
/*
Compute tstareq (torques)
*/
/*
Compute taus (RHS ignoring constraints and inertial forces)
*/
    sddovpk();
    temp[0] = (((fstareq[8][2]*Vpk[0][8][2])+((fstareq[8][0]*Vpk[0][8][0])+(
      fstareq[8][1]*Vpk[0][8][1])))+(((fstareq[7][2]*Vpk[0][7][2])+((
      fstareq[7][0]*Vpk[0][7][0])+(fstareq[7][1]*Vpk[0][7][1])))+(((
      fstareq[5][2]*Vpk[0][3][2])+((fstareq[5][0]*Vpk[0][3][0])+(fstareq[5][1]*
      Vpk[0][3][1])))+((fstareq[6][2]*Vpk[0][6][2])+((fstareq[6][0]*Vpk[0][6][0]
      )+(fstareq[6][1]*Vpk[0][6][1]))))));
    temp[1] = (((fstareq[12][2]*Vpk[0][12][2])+((fstareq[12][0]*Vpk[0][12][0])+(
      fstareq[12][1]*Vpk[0][12][1])))+(((fstareq[11][2]*Vpk[0][11][2])+((
      fstareq[11][0]*Vpk[0][11][0])+(fstareq[11][1]*Vpk[0][11][1])))+(((
      fstareq[10][2]*Vpk[0][10][2])+((fstareq[10][0]*Vpk[0][10][0])+(
      fstareq[10][1]*Vpk[0][10][1])))+(((fstareq[9][2]*Vpk[0][9][2])+((
      fstareq[9][0]*Vpk[0][9][0])+(fstareq[9][1]*Vpk[0][9][1])))+temp[0]))));
    temp[2] = (((fstareq[16][2]*Vpk[0][16][2])+((fstareq[16][0]*Vpk[0][16][0])+(
      fstareq[16][1]*Vpk[0][16][1])))+(((fstareq[15][2]*Vpk[0][15][2])+((
      fstareq[15][0]*Vpk[0][15][0])+(fstareq[15][1]*Vpk[0][15][1])))+(((
      fstareq[14][2]*Vpk[0][14][2])+((fstareq[14][0]*Vpk[0][14][0])+(
      fstareq[14][1]*Vpk[0][14][1])))+(((fstareq[13][2]*Vpk[0][13][2])+((
      fstareq[13][0]*Vpk[0][13][0])+(fstareq[13][1]*Vpk[0][13][1])))+temp[1]))))
      ;
    tau[0] = (utau[0]-(((fstareq[20][2]*Vpk[0][20][2])+((fstareq[20][0]*
      Vpk[0][20][0])+(fstareq[20][1]*Vpk[0][20][1])))+(((fstareq[19][2]*
      Vpk[0][19][2])+((fstareq[19][0]*Vpk[0][19][0])+(fstareq[19][1]*
      Vpk[0][19][1])))+(((fstareq[18][2]*Vpk[0][18][2])+((fstareq[18][0]*
      Vpk[0][18][0])+(fstareq[18][1]*Vpk[0][18][1])))+(((fstareq[17][2]*
      Vpk[0][17][2])+((fstareq[17][0]*Vpk[0][17][0])+(fstareq[17][1]*
      Vpk[0][17][1])))+temp[2])))));
    temp[0] = (((fstareq[8][2]*Vpk[1][8][2])+((fstareq[8][0]*Vpk[1][8][0])+(
      fstareq[8][1]*Vpk[1][8][1])))+(((fstareq[7][2]*Vpk[1][7][2])+((
      fstareq[7][0]*Vpk[1][7][0])+(fstareq[7][1]*Vpk[1][7][1])))+(((
      fstareq[5][2]*Vpk[1][3][2])+((fstareq[5][0]*Vpk[1][3][0])+(fstareq[5][1]*
      Vpk[1][3][1])))+((fstareq[6][2]*Vpk[1][6][2])+((fstareq[6][0]*Vpk[1][6][0]
      )+(fstareq[6][1]*Vpk[1][6][1]))))));
    temp[1] = (((fstareq[12][2]*Vpk[1][12][2])+((fstareq[12][0]*Vpk[1][12][0])+(
      fstareq[12][1]*Vpk[1][12][1])))+(((fstareq[11][2]*Vpk[1][11][2])+((
      fstareq[11][0]*Vpk[1][11][0])+(fstareq[11][1]*Vpk[1][11][1])))+(((
      fstareq[10][2]*Vpk[1][10][2])+((fstareq[10][0]*Vpk[1][10][0])+(
      fstareq[10][1]*Vpk[1][10][1])))+(((fstareq[9][2]*Vpk[1][9][2])+((
      fstareq[9][0]*Vpk[1][9][0])+(fstareq[9][1]*Vpk[1][9][1])))+temp[0]))));
    temp[2] = (((fstareq[16][2]*Vpk[1][16][2])+((fstareq[16][0]*Vpk[1][16][0])+(
      fstareq[16][1]*Vpk[1][16][1])))+(((fstareq[15][2]*Vpk[1][15][2])+((
      fstareq[15][0]*Vpk[1][15][0])+(fstareq[15][1]*Vpk[1][15][1])))+(((
      fstareq[14][2]*Vpk[1][14][2])+((fstareq[14][0]*Vpk[1][14][0])+(
      fstareq[14][1]*Vpk[1][14][1])))+(((fstareq[13][2]*Vpk[1][13][2])+((
      fstareq[13][0]*Vpk[1][13][0])+(fstareq[13][1]*Vpk[1][13][1])))+temp[1]))))
      ;
    tau[1] = (utau[1]-(((fstareq[20][2]*Vpk[1][20][2])+((fstareq[20][0]*
      Vpk[1][20][0])+(fstareq[20][1]*Vpk[1][20][1])))+(((fstareq[19][2]*
      Vpk[1][19][2])+((fstareq[19][0]*Vpk[1][19][0])+(fstareq[19][1]*
      Vpk[1][19][1])))+(((fstareq[18][2]*Vpk[1][18][2])+((fstareq[18][0]*
      Vpk[1][18][0])+(fstareq[18][1]*Vpk[1][18][1])))+(((fstareq[17][2]*
      Vpk[1][17][2])+((fstareq[17][0]*Vpk[1][17][0])+(fstareq[17][1]*
      Vpk[1][17][1])))+temp[2])))));
    temp[0] = (((fstareq[8][2]*Vpk[2][8][2])+((fstareq[8][0]*Vpk[2][8][0])+(
      fstareq[8][1]*Vpk[2][8][1])))+(((fstareq[7][2]*Vpk[2][7][2])+((
      fstareq[7][0]*Vpk[2][7][0])+(fstareq[7][1]*Vpk[2][7][1])))+(((
      fstareq[5][2]*Vpk[2][3][2])+((fstareq[5][0]*Vpk[2][3][0])+(fstareq[5][1]*
      Vpk[2][3][1])))+((fstareq[6][2]*Vpk[2][6][2])+((fstareq[6][0]*Vpk[2][6][0]
      )+(fstareq[6][1]*Vpk[2][6][1]))))));
    temp[1] = (((fstareq[12][2]*Vpk[2][12][2])+((fstareq[12][0]*Vpk[2][12][0])+(
      fstareq[12][1]*Vpk[2][12][1])))+(((fstareq[11][2]*Vpk[2][11][2])+((
      fstareq[11][0]*Vpk[2][11][0])+(fstareq[11][1]*Vpk[2][11][1])))+(((
      fstareq[10][2]*Vpk[2][10][2])+((fstareq[10][0]*Vpk[2][10][0])+(
      fstareq[10][1]*Vpk[2][10][1])))+(((fstareq[9][2]*Vpk[2][9][2])+((
      fstareq[9][0]*Vpk[2][9][0])+(fstareq[9][1]*Vpk[2][9][1])))+temp[0]))));
    temp[2] = (((fstareq[16][2]*Vpk[2][16][2])+((fstareq[16][0]*Vpk[2][16][0])+(
      fstareq[16][1]*Vpk[2][16][1])))+(((fstareq[15][2]*Vpk[2][15][2])+((
      fstareq[15][0]*Vpk[2][15][0])+(fstareq[15][1]*Vpk[2][15][1])))+(((
      fstareq[14][2]*Vpk[2][14][2])+((fstareq[14][0]*Vpk[2][14][0])+(
      fstareq[14][1]*Vpk[2][14][1])))+(((fstareq[13][2]*Vpk[2][13][2])+((
      fstareq[13][0]*Vpk[2][13][0])+(fstareq[13][1]*Vpk[2][13][1])))+temp[1]))))
      ;
    tau[2] = (utau[2]-(((fstareq[20][2]*Vpk[2][20][2])+((fstareq[20][0]*
      Vpk[2][20][0])+(fstareq[20][1]*Vpk[2][20][1])))+(((fstareq[19][2]*
      Vpk[2][19][2])+((fstareq[19][0]*Vpk[2][19][0])+(fstareq[19][1]*
      Vpk[2][19][1])))+(((fstareq[18][2]*Vpk[2][18][2])+((fstareq[18][0]*
      Vpk[2][18][0])+(fstareq[18][1]*Vpk[2][18][1])))+(((fstareq[17][2]*
      Vpk[2][17][2])+((fstareq[17][0]*Vpk[2][17][0])+(fstareq[17][1]*
      Vpk[2][17][1])))+temp[2])))));
    temp[0] = ((((fstareq[7][2]*Vpk[3][7][2])+((fstareq[7][0]*Vpk[3][7][0])+(
      fstareq[7][1]*Vpk[3][7][1])))-((utk[2][2]*Wpk[3][7][2])+((utk[2][0]*
      Wpk[3][7][0])+(utk[2][1]*Wpk[3][7][1]))))+((((fstareq[5][1]*rk[0][2])-(
      fstareq[5][2]*rk[0][1]))-utk[0][0])+(((fstareq[6][2]*Vpk[3][6][2])+((
      fstareq[6][0]*Vpk[3][6][0])+(fstareq[6][1]*Vpk[3][6][1])))-((Cik[6][0][2]*
      utk[1][2])+((Cik[6][0][0]*utk[1][0])+(Cik[6][0][1]*utk[1][1]))))));
    temp[1] = ((((fstareq[9][2]*Vpk[3][9][2])+((fstareq[9][0]*Vpk[3][9][0])+(
      fstareq[9][1]*Vpk[3][9][1])))-((Cik[9][0][2]*utk[4][2])+((Cik[9][0][0]*
      utk[4][0])+(Cik[9][0][1]*utk[4][1]))))+((((fstareq[8][2]*Vpk[3][8][2])+((
      fstareq[8][0]*Vpk[3][8][0])+(fstareq[8][1]*Vpk[3][8][1])))-((utk[3][2]*
      Wpk[3][8][2])+((utk[3][0]*Wpk[3][8][0])+(utk[3][1]*Wpk[3][8][1]))))+
      temp[0]));
    temp[2] = ((((fstareq[11][2]*Vpk[3][11][2])+((fstareq[11][0]*Vpk[3][11][0])+
      (fstareq[11][1]*Vpk[3][11][1])))-((utk[6][2]*Wpk[3][11][2])+((utk[6][0]*
      Wpk[3][11][0])+(utk[6][1]*Wpk[3][11][1]))))+((((fstareq[10][2]*
      Vpk[3][10][2])+((fstareq[10][0]*Vpk[3][10][0])+(fstareq[10][1]*
      Vpk[3][10][1])))-((utk[5][2]*Wpk[3][10][2])+((utk[5][0]*Wpk[3][10][0])+(
      utk[5][1]*Wpk[3][10][1]))))+temp[1]));
    temp[3] = ((((fstareq[13][2]*Vpk[3][13][2])+((fstareq[13][0]*Vpk[3][13][0])+
      (fstareq[13][1]*Vpk[3][13][1])))-((utk[8][2]*Wpk[3][13][2])+((utk[8][0]*
      Wpk[3][13][0])+(utk[8][1]*Wpk[3][13][1]))))+((((fstareq[12][2]*
      Vpk[3][12][2])+((fstareq[12][0]*Vpk[3][12][0])+(fstareq[12][1]*
      Vpk[3][12][1])))-((utk[7][2]*Wpk[3][12][2])+((utk[7][0]*Wpk[3][12][0])+(
      utk[7][1]*Wpk[3][12][1]))))+temp[2]));
    temp[4] = ((((fstareq[15][2]*Vpk[3][15][2])+((fstareq[15][0]*Vpk[3][15][0])+
      (fstareq[15][1]*Vpk[3][15][1])))-((Cik[15][0][2]*utk[10][2])+((
      Cik[15][0][0]*utk[10][0])+(Cik[15][0][1]*utk[10][1]))))+((((fstareq[14][2]
      *Vpk[3][14][2])+((fstareq[14][0]*Vpk[3][14][0])+(fstareq[14][1]*
      Vpk[3][14][1])))-((utk[9][2]*Wpk[3][14][2])+((utk[9][0]*Wpk[3][14][0])+(
      utk[9][1]*Wpk[3][14][1]))))+temp[3]));
    temp[5] = ((((fstareq[17][2]*Vpk[3][17][2])+((fstareq[17][0]*Vpk[3][17][0])+
      (fstareq[17][1]*Vpk[3][17][1])))-((utk[12][2]*Wpk[3][17][2])+((utk[12][0]*
      Wpk[3][17][0])+(utk[12][1]*Wpk[3][17][1]))))+((((fstareq[16][2]*
      Vpk[3][16][2])+((fstareq[16][0]*Vpk[3][16][0])+(fstareq[16][1]*
      Vpk[3][16][1])))-((utk[11][2]*Wpk[3][16][2])+((utk[11][0]*Wpk[3][16][0])+(
      utk[11][1]*Wpk[3][16][1]))))+temp[4]));
    temp[6] = ((((fstareq[19][2]*Vpk[3][19][2])+((fstareq[19][0]*Vpk[3][19][0])+
      (fstareq[19][1]*Vpk[3][19][1])))-((utk[14][2]*Wpk[3][19][2])+((utk[14][0]*
      Wpk[3][19][0])+(utk[14][1]*Wpk[3][19][1]))))+((((fstareq[18][2]*
      Vpk[3][18][2])+((fstareq[18][0]*Vpk[3][18][0])+(fstareq[18][1]*
      Vpk[3][18][1])))-((utk[13][2]*Wpk[3][18][2])+((utk[13][0]*Wpk[3][18][0])+(
      utk[13][1]*Wpk[3][18][1]))))+temp[5]));
    tau[3] = (utau[3]-((((fstareq[20][2]*Vpk[3][20][2])+((fstareq[20][0]*
      Vpk[3][20][0])+(fstareq[20][1]*Vpk[3][20][1])))-((utk[15][2]*Wpk[3][20][2]
      )+((utk[15][0]*Wpk[3][20][0])+(utk[15][1]*Wpk[3][20][1]))))+temp[6]));
    temp[0] = ((((fstareq[7][2]*Vpk[4][7][2])+((fstareq[7][0]*Vpk[4][7][0])+(
      fstareq[7][1]*Vpk[4][7][1])))-((utk[2][2]*Wpk[4][7][2])+((utk[2][0]*
      Wpk[4][7][0])+(utk[2][1]*Wpk[4][7][1]))))+((((fstareq[5][2]*rk[0][0])-(
      fstareq[5][0]*rk[0][2]))-utk[0][1])+(((fstareq[6][2]*Vpk[4][6][2])+((
      fstareq[6][0]*Vpk[4][6][0])+(fstareq[6][1]*Vpk[4][6][1])))-((Cik[6][1][2]*
      utk[1][2])+((Cik[6][1][0]*utk[1][0])+(Cik[6][1][1]*utk[1][1]))))));
    temp[1] = ((((fstareq[9][2]*Vpk[4][9][2])+((fstareq[9][0]*Vpk[4][9][0])+(
      fstareq[9][1]*Vpk[4][9][1])))-((Cik[9][1][2]*utk[4][2])+((Cik[9][1][0]*
      utk[4][0])+(Cik[9][1][1]*utk[4][1]))))+((((fstareq[8][2]*Vpk[4][8][2])+((
      fstareq[8][0]*Vpk[4][8][0])+(fstareq[8][1]*Vpk[4][8][1])))-((utk[3][2]*
      Wpk[4][8][2])+((utk[3][0]*Wpk[4][8][0])+(utk[3][1]*Wpk[4][8][1]))))+
      temp[0]));
    temp[2] = ((((fstareq[11][2]*Vpk[4][11][2])+((fstareq[11][0]*Vpk[4][11][0])+
      (fstareq[11][1]*Vpk[4][11][1])))-((utk[6][2]*Wpk[4][11][2])+((utk[6][0]*
      Wpk[4][11][0])+(utk[6][1]*Wpk[4][11][1]))))+((((fstareq[10][2]*
      Vpk[4][10][2])+((fstareq[10][0]*Vpk[4][10][0])+(fstareq[10][1]*
      Vpk[4][10][1])))-((utk[5][2]*Wpk[4][10][2])+((utk[5][0]*Wpk[4][10][0])+(
      utk[5][1]*Wpk[4][10][1]))))+temp[1]));
    temp[3] = ((((fstareq[13][2]*Vpk[4][13][2])+((fstareq[13][0]*Vpk[4][13][0])+
      (fstareq[13][1]*Vpk[4][13][1])))-((utk[8][2]*Wpk[4][13][2])+((utk[8][0]*
      Wpk[4][13][0])+(utk[8][1]*Wpk[4][13][1]))))+((((fstareq[12][2]*
      Vpk[4][12][2])+((fstareq[12][0]*Vpk[4][12][0])+(fstareq[12][1]*
      Vpk[4][12][1])))-((utk[7][2]*Wpk[4][12][2])+((utk[7][0]*Wpk[4][12][0])+(
      utk[7][1]*Wpk[4][12][1]))))+temp[2]));
    temp[4] = ((((fstareq[15][2]*Vpk[4][15][2])+((fstareq[15][0]*Vpk[4][15][0])+
      (fstareq[15][1]*Vpk[4][15][1])))-((Cik[15][1][2]*utk[10][2])+((
      Cik[15][1][0]*utk[10][0])+(Cik[15][1][1]*utk[10][1]))))+((((fstareq[14][2]
      *Vpk[4][14][2])+((fstareq[14][0]*Vpk[4][14][0])+(fstareq[14][1]*
      Vpk[4][14][1])))-((utk[9][2]*Wpk[4][14][2])+((utk[9][0]*Wpk[4][14][0])+(
      utk[9][1]*Wpk[4][14][1]))))+temp[3]));
    temp[5] = ((((fstareq[17][2]*Vpk[4][17][2])+((fstareq[17][0]*Vpk[4][17][0])+
      (fstareq[17][1]*Vpk[4][17][1])))-((utk[12][2]*Wpk[4][17][2])+((utk[12][0]*
      Wpk[4][17][0])+(utk[12][1]*Wpk[4][17][1]))))+((((fstareq[16][2]*
      Vpk[4][16][2])+((fstareq[16][0]*Vpk[4][16][0])+(fstareq[16][1]*
      Vpk[4][16][1])))-((utk[11][2]*Wpk[4][16][2])+((utk[11][0]*Wpk[4][16][0])+(
      utk[11][1]*Wpk[4][16][1]))))+temp[4]));
    temp[6] = ((((fstareq[19][2]*Vpk[4][19][2])+((fstareq[19][0]*Vpk[4][19][0])+
      (fstareq[19][1]*Vpk[4][19][1])))-((utk[14][2]*Wpk[4][19][2])+((utk[14][0]*
      Wpk[4][19][0])+(utk[14][1]*Wpk[4][19][1]))))+((((fstareq[18][2]*
      Vpk[4][18][2])+((fstareq[18][0]*Vpk[4][18][0])+(fstareq[18][1]*
      Vpk[4][18][1])))-((utk[13][2]*Wpk[4][18][2])+((utk[13][0]*Wpk[4][18][0])+(
      utk[13][1]*Wpk[4][18][1]))))+temp[5]));
    tau[4] = (utau[4]-((((fstareq[20][2]*Vpk[4][20][2])+((fstareq[20][0]*
      Vpk[4][20][0])+(fstareq[20][1]*Vpk[4][20][1])))-((utk[15][2]*Wpk[4][20][2]
      )+((utk[15][0]*Wpk[4][20][0])+(utk[15][1]*Wpk[4][20][1]))))+temp[6]));
    temp[0] = ((((fstareq[7][2]*Vpk[5][7][2])+((fstareq[7][0]*Vpk[5][7][0])+(
      fstareq[7][1]*Vpk[5][7][1])))-((utk[2][2]*Wpk[5][7][2])+((utk[2][0]*
      Wpk[5][7][0])+(utk[2][1]*Wpk[5][7][1]))))+((((fstareq[5][0]*rk[0][1])-(
      fstareq[5][1]*rk[0][0]))-utk[0][2])+(((fstareq[6][2]*Vpk[5][6][2])+((
      fstareq[6][0]*Vpk[5][6][0])+(fstareq[6][1]*Vpk[5][6][1])))-((Cik[6][2][2]*
      utk[1][2])+((Cik[6][2][0]*utk[1][0])+(Cik[6][2][1]*utk[1][1]))))));
    temp[1] = ((((fstareq[9][2]*Vpk[5][9][2])+((fstareq[9][0]*Vpk[5][9][0])+(
      fstareq[9][1]*Vpk[5][9][1])))-((Cik[9][2][2]*utk[4][2])+((Cik[9][2][0]*
      utk[4][0])+(Cik[9][2][1]*utk[4][1]))))+((((fstareq[8][2]*Vpk[5][8][2])+((
      fstareq[8][0]*Vpk[5][8][0])+(fstareq[8][1]*Vpk[5][8][1])))-((utk[3][2]*
      Wpk[5][8][2])+((utk[3][0]*Wpk[5][8][0])+(utk[3][1]*Wpk[5][8][1]))))+
      temp[0]));
    temp[2] = ((((fstareq[11][2]*Vpk[5][11][2])+((fstareq[11][0]*Vpk[5][11][0])+
      (fstareq[11][1]*Vpk[5][11][1])))-((utk[6][2]*Wpk[5][11][2])+((utk[6][0]*
      Wpk[5][11][0])+(utk[6][1]*Wpk[5][11][1]))))+((((fstareq[10][2]*
      Vpk[5][10][2])+((fstareq[10][0]*Vpk[5][10][0])+(fstareq[10][1]*
      Vpk[5][10][1])))-((utk[5][2]*Wpk[5][10][2])+((utk[5][0]*Wpk[5][10][0])+(
      utk[5][1]*Wpk[5][10][1]))))+temp[1]));
    temp[3] = ((((fstareq[13][2]*Vpk[5][13][2])+((fstareq[13][0]*Vpk[5][13][0])+
      (fstareq[13][1]*Vpk[5][13][1])))-((utk[8][2]*Wpk[5][13][2])+((utk[8][0]*
      Wpk[5][13][0])+(utk[8][1]*Wpk[5][13][1]))))+((((fstareq[12][2]*
      Vpk[5][12][2])+((fstareq[12][0]*Vpk[5][12][0])+(fstareq[12][1]*
      Vpk[5][12][1])))-((utk[7][2]*Wpk[5][12][2])+((utk[7][0]*Wpk[5][12][0])+(
      utk[7][1]*Wpk[5][12][1]))))+temp[2]));
    temp[4] = ((((fstareq[15][2]*Vpk[5][15][2])+((fstareq[15][0]*Vpk[5][15][0])+
      (fstareq[15][1]*Vpk[5][15][1])))-((Cik[15][2][2]*utk[10][2])+((
      Cik[15][2][0]*utk[10][0])+(Cik[15][2][1]*utk[10][1]))))+((((fstareq[14][2]
      *Vpk[5][14][2])+((fstareq[14][0]*Vpk[5][14][0])+(fstareq[14][1]*
      Vpk[5][14][1])))-((utk[9][2]*Wpk[5][14][2])+((utk[9][0]*Wpk[5][14][0])+(
      utk[9][1]*Wpk[5][14][1]))))+temp[3]));
    temp[5] = ((((fstareq[17][2]*Vpk[5][17][2])+((fstareq[17][0]*Vpk[5][17][0])+
      (fstareq[17][1]*Vpk[5][17][1])))-((utk[12][2]*Wpk[5][17][2])+((utk[12][0]*
      Wpk[5][17][0])+(utk[12][1]*Wpk[5][17][1]))))+((((fstareq[16][2]*
      Vpk[5][16][2])+((fstareq[16][0]*Vpk[5][16][0])+(fstareq[16][1]*
      Vpk[5][16][1])))-((utk[11][2]*Wpk[5][16][2])+((utk[11][0]*Wpk[5][16][0])+(
      utk[11][1]*Wpk[5][16][1]))))+temp[4]));
    temp[6] = ((((fstareq[19][2]*Vpk[5][19][2])+((fstareq[19][0]*Vpk[5][19][0])+
      (fstareq[19][1]*Vpk[5][19][1])))-((utk[14][2]*Wpk[5][19][2])+((utk[14][0]*
      Wpk[5][19][0])+(utk[14][1]*Wpk[5][19][1]))))+((((fstareq[18][2]*
      Vpk[5][18][2])+((fstareq[18][0]*Vpk[5][18][0])+(fstareq[18][1]*
      Vpk[5][18][1])))-((utk[13][2]*Wpk[5][18][2])+((utk[13][0]*Wpk[5][18][0])+(
      utk[13][1]*Wpk[5][18][1]))))+temp[5]));
    tau[5] = (utau[5]-((((fstareq[20][2]*Vpk[5][20][2])+((fstareq[20][0]*
      Vpk[5][20][0])+(fstareq[20][1]*Vpk[5][20][1])))-((utk[15][2]*Wpk[5][20][2]
      )+((utk[15][0]*Wpk[5][20][0])+(utk[15][1]*Wpk[5][20][1]))))+temp[6]));
    temp[0] = ((((fstareq[6][2]*Vpk[6][6][2])+((fstareq[6][0]*Vpk[6][6][0])+(
      fstareq[6][1]*Vpk[6][6][1])))-((pin[6][2]*utk[1][2])+((pin[6][0]*utk[1][0]
      )+(pin[6][1]*utk[1][1]))))+(((fstareq[7][2]*Vpk[6][7][2])+((fstareq[7][0]*
      Vpk[6][7][0])+(fstareq[7][1]*Vpk[6][7][1])))-((utk[2][2]*Wpk[6][7][2])+((
      utk[2][0]*Wpk[6][7][0])+(utk[2][1]*Wpk[6][7][1])))));
    tau[6] = (utau[6]-((((fstareq[8][2]*Vpk[6][8][2])+((fstareq[8][0]*
      Vpk[6][8][0])+(fstareq[8][1]*Vpk[6][8][1])))-((utk[3][2]*Wpk[6][8][2])+((
      utk[3][0]*Wpk[6][8][0])+(utk[3][1]*Wpk[6][8][1]))))+temp[0]));
    tau[7] = (utau[7]-((((fstareq[7][2]*Vpk[7][7][2])+((fstareq[7][0]*
      Vpk[7][7][0])+(fstareq[7][1]*Vpk[7][7][1])))-((pin[7][2]*utk[2][2])+((
      pin[7][0]*utk[2][0])+(pin[7][1]*utk[2][1]))))+(((fstareq[8][2]*
      Vpk[7][8][2])+((fstareq[8][0]*Vpk[7][8][0])+(fstareq[8][1]*Vpk[7][8][1])))
      -((utk[3][2]*Wpk[7][8][2])+((utk[3][0]*Wpk[7][8][0])+(utk[3][1]*
      Wpk[7][8][1]))))));
    tau[8] = (utau[8]-(((fstareq[8][2]*Vpk[8][8][2])+((fstareq[8][0]*
      Vpk[8][8][0])+(fstareq[8][1]*Vpk[8][8][1])))-((pin[8][2]*utk[3][2])+((
      pin[8][0]*utk[3][0])+(pin[8][1]*utk[3][1])))));
    temp[0] = ((((fstareq[9][2]*Vpk[9][9][2])+((fstareq[9][0]*Vpk[9][9][0])+(
      fstareq[9][1]*Vpk[9][9][1])))-((pin[9][2]*utk[4][2])+((pin[9][0]*utk[4][0]
      )+(pin[9][1]*utk[4][1]))))+(((fstareq[10][2]*Vpk[9][10][2])+((
      fstareq[10][0]*Vpk[9][10][0])+(fstareq[10][1]*Vpk[9][10][1])))-((utk[5][2]
      *Wpk[9][10][2])+((utk[5][0]*Wpk[9][10][0])+(utk[5][1]*Wpk[9][10][1])))));
    temp[1] = ((((fstareq[12][2]*Vpk[9][12][2])+((fstareq[12][0]*Vpk[9][12][0])+
      (fstareq[12][1]*Vpk[9][12][1])))-((utk[7][2]*Wpk[9][12][2])+((utk[7][0]*
      Wpk[9][12][0])+(utk[7][1]*Wpk[9][12][1]))))+((((fstareq[11][2]*
      Vpk[9][11][2])+((fstareq[11][0]*Vpk[9][11][0])+(fstareq[11][1]*
      Vpk[9][11][1])))-((utk[6][2]*Wpk[9][11][2])+((utk[6][0]*Wpk[9][11][0])+(
      utk[6][1]*Wpk[9][11][1]))))+temp[0]));
    tau[9] = (utau[9]-((((fstareq[14][2]*Vpk[9][14][2])+((fstareq[14][0]*
      Vpk[9][14][0])+(fstareq[14][1]*Vpk[9][14][1])))-((utk[9][2]*Wpk[9][14][2])
      +((utk[9][0]*Wpk[9][14][0])+(utk[9][1]*Wpk[9][14][1]))))+((((
      fstareq[13][2]*Vpk[9][13][2])+((fstareq[13][0]*Vpk[9][13][0])+(
      fstareq[13][1]*Vpk[9][13][1])))-((utk[8][2]*Wpk[9][13][2])+((utk[8][0]*
      Wpk[9][13][0])+(utk[8][1]*Wpk[9][13][1]))))+temp[1])));
    temp[0] = ((((fstareq[10][2]*Vpk[10][10][2])+((fstareq[10][0]*Vpk[10][10][0]
      )+(fstareq[10][1]*Vpk[10][10][1])))-((pin[10][2]*utk[5][2])+((pin[10][0]*
      utk[5][0])+(pin[10][1]*utk[5][1]))))+(((fstareq[11][2]*Vpk[10][11][2])+((
      fstareq[11][0]*Vpk[10][11][0])+(fstareq[11][1]*Vpk[10][11][1])))-((
      utk[6][2]*Wpk[10][11][2])+((utk[6][0]*Wpk[10][11][0])+(utk[6][1]*
      Wpk[10][11][1])))));
    temp[1] = ((((fstareq[13][2]*Vpk[10][13][2])+((fstareq[13][0]*Vpk[10][13][0]
      )+(fstareq[13][1]*Vpk[10][13][1])))-((utk[8][2]*Wpk[10][13][2])+((
      utk[8][0]*Wpk[10][13][0])+(utk[8][1]*Wpk[10][13][1]))))+((((fstareq[12][2]
      *Vpk[10][12][2])+((fstareq[12][0]*Vpk[10][12][0])+(fstareq[12][1]*
      Vpk[10][12][1])))-((utk[7][2]*Wpk[10][12][2])+((utk[7][0]*Wpk[10][12][0])+
      (utk[7][1]*Wpk[10][12][1]))))+temp[0]));
    tau[10] = (utau[10]-((((fstareq[14][2]*Vpk[10][14][2])+((fstareq[14][0]*
      Vpk[10][14][0])+(fstareq[14][1]*Vpk[10][14][1])))-((utk[9][2]*
      Wpk[10][14][2])+((utk[9][0]*Wpk[10][14][0])+(utk[9][1]*Wpk[10][14][1]))))+
      temp[1]));
    temp[0] = ((((fstareq[11][2]*Vpk[11][11][2])+((fstareq[11][0]*Vpk[11][11][0]
      )+(fstareq[11][1]*Vpk[11][11][1])))-((pin[11][2]*utk[6][2])+((pin[11][0]*
      utk[6][0])+(pin[11][1]*utk[6][1]))))+(((fstareq[12][2]*Vpk[11][12][2])+((
      fstareq[12][0]*Vpk[11][12][0])+(fstareq[12][1]*Vpk[11][12][1])))-((
      utk[7][2]*Wpk[11][12][2])+((utk[7][0]*Wpk[11][12][0])+(utk[7][1]*
      Wpk[11][12][1])))));
    tau[11] = (utau[11]-((((fstareq[14][2]*Vpk[11][14][2])+((fstareq[14][0]*
      Vpk[11][14][0])+(fstareq[14][1]*Vpk[11][14][1])))-((utk[9][2]*
      Wpk[11][14][2])+((utk[9][0]*Wpk[11][14][0])+(utk[9][1]*Wpk[11][14][1]))))+
      ((((fstareq[13][2]*Vpk[11][13][2])+((fstareq[13][0]*Vpk[11][13][0])+(
      fstareq[13][1]*Vpk[11][13][1])))-((utk[8][2]*Wpk[11][13][2])+((utk[8][0]*
      Wpk[11][13][0])+(utk[8][1]*Wpk[11][13][1]))))+temp[0])));
    temp[0] = ((((fstareq[12][2]*Vpk[12][12][2])+((fstareq[12][0]*Vpk[12][12][0]
      )+(fstareq[12][1]*Vpk[12][12][1])))-((pin[12][2]*utk[7][2])+((pin[12][0]*
      utk[7][0])+(pin[12][1]*utk[7][1]))))+(((fstareq[13][2]*Vpk[12][13][2])+((
      fstareq[13][0]*Vpk[12][13][0])+(fstareq[13][1]*Vpk[12][13][1])))-((
      utk[8][2]*Wpk[12][13][2])+((utk[8][0]*Wpk[12][13][0])+(utk[8][1]*
      Wpk[12][13][1])))));
    tau[12] = (utau[12]-((((fstareq[14][2]*Vpk[12][14][2])+((fstareq[14][0]*
      Vpk[12][14][0])+(fstareq[14][1]*Vpk[12][14][1])))-((utk[9][2]*
      Wpk[12][14][2])+((utk[9][0]*Wpk[12][14][0])+(utk[9][1]*Wpk[12][14][1]))))+
      temp[0]));
    tau[13] = (utau[13]-((((fstareq[13][2]*Vpk[13][13][2])+((fstareq[13][0]*
      Vpk[13][13][0])+(fstareq[13][1]*Vpk[13][13][1])))-((pin[13][2]*utk[8][2])+
      ((pin[13][0]*utk[8][0])+(pin[13][1]*utk[8][1]))))+(((fstareq[14][2]*
      Vpk[13][14][2])+((fstareq[14][0]*Vpk[13][14][0])+(fstareq[14][1]*
      Vpk[13][14][1])))-((utk[9][2]*Wpk[13][14][2])+((utk[9][0]*Wpk[13][14][0])+
      (utk[9][1]*Wpk[13][14][1]))))));
    tau[14] = (utau[14]-(((fstareq[14][2]*Vpk[14][14][2])+((fstareq[14][0]*
      Vpk[14][14][0])+(fstareq[14][1]*Vpk[14][14][1])))-((pin[14][2]*utk[9][2])+
      ((pin[14][0]*utk[9][0])+(pin[14][1]*utk[9][1])))));
    temp[0] = ((((fstareq[15][2]*Vpk[15][15][2])+((fstareq[15][0]*Vpk[15][15][0]
      )+(fstareq[15][1]*Vpk[15][15][1])))-((pin[15][2]*utk[10][2])+((pin[15][0]*
      utk[10][0])+(pin[15][1]*utk[10][1]))))+(((fstareq[16][2]*Vpk[15][16][2])+(
      (fstareq[16][0]*Vpk[15][16][0])+(fstareq[16][1]*Vpk[15][16][1])))-((
      utk[11][2]*Wpk[15][16][2])+((utk[11][0]*Wpk[15][16][0])+(utk[11][1]*
      Wpk[15][16][1])))));
    temp[1] = ((((fstareq[18][2]*Vpk[15][18][2])+((fstareq[18][0]*Vpk[15][18][0]
      )+(fstareq[18][1]*Vpk[15][18][1])))-((utk[13][2]*Wpk[15][18][2])+((
      utk[13][0]*Wpk[15][18][0])+(utk[13][1]*Wpk[15][18][1]))))+((((
      fstareq[17][2]*Vpk[15][17][2])+((fstareq[17][0]*Vpk[15][17][0])+(
      fstareq[17][1]*Vpk[15][17][1])))-((utk[12][2]*Wpk[15][17][2])+((utk[12][0]
      *Wpk[15][17][0])+(utk[12][1]*Wpk[15][17][1]))))+temp[0]));
    tau[15] = (utau[15]-((((fstareq[20][2]*Vpk[15][20][2])+((fstareq[20][0]*
      Vpk[15][20][0])+(fstareq[20][1]*Vpk[15][20][1])))-((utk[15][2]*
      Wpk[15][20][2])+((utk[15][0]*Wpk[15][20][0])+(utk[15][1]*Wpk[15][20][1])))
      )+((((fstareq[19][2]*Vpk[15][19][2])+((fstareq[19][0]*Vpk[15][19][0])+(
      fstareq[19][1]*Vpk[15][19][1])))-((utk[14][2]*Wpk[15][19][2])+((utk[14][0]
      *Wpk[15][19][0])+(utk[14][1]*Wpk[15][19][1]))))+temp[1])));
    temp[0] = ((((fstareq[16][2]*Vpk[16][16][2])+((fstareq[16][0]*Vpk[16][16][0]
      )+(fstareq[16][1]*Vpk[16][16][1])))-((pin[16][2]*utk[11][2])+((pin[16][0]*
      utk[11][0])+(pin[16][1]*utk[11][1]))))+(((fstareq[17][2]*Vpk[16][17][2])+(
      (fstareq[17][0]*Vpk[16][17][0])+(fstareq[17][1]*Vpk[16][17][1])))-((
      utk[12][2]*Wpk[16][17][2])+((utk[12][0]*Wpk[16][17][0])+(utk[12][1]*
      Wpk[16][17][1])))));
    temp[1] = ((((fstareq[19][2]*Vpk[16][19][2])+((fstareq[19][0]*Vpk[16][19][0]
      )+(fstareq[19][1]*Vpk[16][19][1])))-((utk[14][2]*Wpk[16][19][2])+((
      utk[14][0]*Wpk[16][19][0])+(utk[14][1]*Wpk[16][19][1]))))+((((
      fstareq[18][2]*Vpk[16][18][2])+((fstareq[18][0]*Vpk[16][18][0])+(
      fstareq[18][1]*Vpk[16][18][1])))-((utk[13][2]*Wpk[16][18][2])+((utk[13][0]
      *Wpk[16][18][0])+(utk[13][1]*Wpk[16][18][1]))))+temp[0]));
    tau[16] = (utau[16]-((((fstareq[20][2]*Vpk[16][20][2])+((fstareq[20][0]*
      Vpk[16][20][0])+(fstareq[20][1]*Vpk[16][20][1])))-((utk[15][2]*
      Wpk[16][20][2])+((utk[15][0]*Wpk[16][20][0])+(utk[15][1]*Wpk[16][20][1])))
      )+temp[1]));
    temp[0] = ((((fstareq[17][2]*Vpk[17][17][2])+((fstareq[17][0]*Vpk[17][17][0]
      )+(fstareq[17][1]*Vpk[17][17][1])))-((pin[17][2]*utk[12][2])+((pin[17][0]*
      utk[12][0])+(pin[17][1]*utk[12][1]))))+(((fstareq[18][2]*Vpk[17][18][2])+(
      (fstareq[18][0]*Vpk[17][18][0])+(fstareq[18][1]*Vpk[17][18][1])))-((
      utk[13][2]*Wpk[17][18][2])+((utk[13][0]*Wpk[17][18][0])+(utk[13][1]*
      Wpk[17][18][1])))));
    tau[17] = (utau[17]-((((fstareq[20][2]*Vpk[17][20][2])+((fstareq[20][0]*
      Vpk[17][20][0])+(fstareq[20][1]*Vpk[17][20][1])))-((utk[15][2]*
      Wpk[17][20][2])+((utk[15][0]*Wpk[17][20][0])+(utk[15][1]*Wpk[17][20][1])))
      )+((((fstareq[19][2]*Vpk[17][19][2])+((fstareq[19][0]*Vpk[17][19][0])+(
      fstareq[19][1]*Vpk[17][19][1])))-((utk[14][2]*Wpk[17][19][2])+((utk[14][0]
      *Wpk[17][19][0])+(utk[14][1]*Wpk[17][19][1]))))+temp[0])));
    temp[0] = ((((fstareq[18][2]*Vpk[18][18][2])+((fstareq[18][0]*Vpk[18][18][0]
      )+(fstareq[18][1]*Vpk[18][18][1])))-((pin[18][2]*utk[13][2])+((pin[18][0]*
      utk[13][0])+(pin[18][1]*utk[13][1]))))+(((fstareq[19][2]*Vpk[18][19][2])+(
      (fstareq[19][0]*Vpk[18][19][0])+(fstareq[19][1]*Vpk[18][19][1])))-((
      utk[14][2]*Wpk[18][19][2])+((utk[14][0]*Wpk[18][19][0])+(utk[14][1]*
      Wpk[18][19][1])))));
    tau[18] = (utau[18]-((((fstareq[20][2]*Vpk[18][20][2])+((fstareq[20][0]*
      Vpk[18][20][0])+(fstareq[20][1]*Vpk[18][20][1])))-((utk[15][2]*
      Wpk[18][20][2])+((utk[15][0]*Wpk[18][20][0])+(utk[15][1]*Wpk[18][20][1])))
      )+temp[0]));
    tau[19] = (utau[19]-((((fstareq[19][2]*Vpk[19][19][2])+((fstareq[19][0]*
      Vpk[19][19][0])+(fstareq[19][1]*Vpk[19][19][1])))-((pin[19][2]*utk[14][2])
      +((pin[19][0]*utk[14][0])+(pin[19][1]*utk[14][1]))))+(((fstareq[20][2]*
      Vpk[19][20][2])+((fstareq[20][0]*Vpk[19][20][0])+(fstareq[20][1]*
      Vpk[19][20][1])))-((utk[15][2]*Wpk[19][20][2])+((utk[15][0]*Wpk[19][20][0]
      )+(utk[15][1]*Wpk[19][20][1]))))));
    tau[20] = (utau[20]-(((fstareq[20][2]*Vpk[20][20][2])+((fstareq[20][0]*
      Vpk[20][20][0])+(fstareq[20][1]*Vpk[20][20][1])))-((pin[20][2]*utk[15][2])
      +((pin[20][0]*utk[15][0])+(pin[20][1]*utk[15][1])))));
/*
Op counts below do not include called subroutines
*/
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  807 adds/subtracts/negates
                    756 multiplies
                      0 divides
                    112 assignments
*/
}

void sdfs0(void)
{

/*
Compute Fs (ignoring multiplier forces)
*/
    fs[0] = fs0[0];
    fs[1] = fs0[1];
    fs[2] = fs0[2];
    fs[3] = fs0[3];
    fs[4] = fs0[4];
    fs[5] = fs0[5];
    fs[6] = fs0[6];
    fs[7] = fs0[7];
    fs[8] = fs0[8];
    fs[9] = fs0[9];
    fs[10] = fs0[10];
    fs[11] = fs0[11];
    fs[12] = fs0[12];
    fs[13] = fs0[13];
    fs[14] = fs0[14];
    fs[15] = fs0[15];
    fs[16] = fs0[16];
    fs[17] = fs0[17];
    fs[18] = fs0[18];
    fs[19] = fs0[19];
    fs[20] = fs0[20];
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     21 assignments
*/
}

void sdfsmult(void)
{

/*
Compute Fs (multiplier-generated forces only)
*/
    sddovpk();
    fs[0] = mtau[0];
    fs[1] = mtau[1];
    fs[2] = mtau[2];
    fs[3] = mtau[3];
    fs[4] = mtau[4];
    fs[5] = mtau[5];
    fs[6] = mtau[6];
    fs[7] = mtau[7];
    fs[8] = mtau[8];
    fs[9] = mtau[9];
    fs[10] = mtau[10];
    fs[11] = mtau[11];
    fs[12] = mtau[12];
    fs[13] = mtau[13];
    fs[14] = mtau[14];
    fs[15] = mtau[15];
    fs[16] = mtau[16];
    fs[17] = mtau[17];
    fs[18] = mtau[18];
    fs[19] = mtau[19];
    fs[20] = mtau[20];
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     21 assignments
*/
}

void sdfsfull(void)
{

/*
Compute Fs (including all forces)
*/
    sdfsmult();
    fs[0] = (fs[0]+fs0[0]);
    fs[1] = (fs[1]+fs0[1]);
    fs[2] = (fs[2]+fs0[2]);
    fs[3] = (fs[3]+fs0[3]);
    fs[4] = (fs[4]+fs0[4]);
    fs[5] = (fs[5]+fs0[5]);
    fs[6] = (fs[6]+fs0[6]);
    fs[7] = (fs[7]+fs0[7]);
    fs[8] = (fs[8]+fs0[8]);
    fs[9] = (fs[9]+fs0[9]);
    fs[10] = (fs[10]+fs0[10]);
    fs[11] = (fs[11]+fs0[11]);
    fs[12] = (fs[12]+fs0[12]);
    fs[13] = (fs[13]+fs0[13]);
    fs[14] = (fs[14]+fs0[14]);
    fs[15] = (fs[15]+fs0[15]);
    fs[16] = (fs[16]+fs0[16]);
    fs[17] = (fs[17]+fs0[17]);
    fs[18] = (fs[18]+fs0[18]);
    fs[19] = (fs[19]+fs0[19]);
    fs[20] = (fs[20]+fs0[20]);
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   21 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     21 assignments
*/
}

void sdfsgenmult(void)
{

/*
Compute Fs (generic multiplier-generated forces)
*/
    sddovpk();
    temp[0] = (((mfk[3][2]*Vpk[0][8][2])+((mfk[3][0]*Vpk[0][8][0])+(mfk[3][1]*
      Vpk[0][8][1])))+(((mfk[2][2]*Vpk[0][7][2])+((mfk[2][0]*Vpk[0][7][0])+(
      mfk[2][1]*Vpk[0][7][1])))+(((mfk[0][2]*Vpk[0][3][2])+((mfk[0][0]*
      Vpk[0][3][0])+(mfk[0][1]*Vpk[0][3][1])))+((mfk[1][2]*Vpk[0][6][2])+((
      mfk[1][0]*Vpk[0][6][0])+(mfk[1][1]*Vpk[0][6][1]))))));
    temp[1] = (((mfk[7][2]*Vpk[0][12][2])+((mfk[7][0]*Vpk[0][12][0])+(mfk[7][1]*
      Vpk[0][12][1])))+(((mfk[6][2]*Vpk[0][11][2])+((mfk[6][0]*Vpk[0][11][0])+(
      mfk[6][1]*Vpk[0][11][1])))+(((mfk[5][2]*Vpk[0][10][2])+((mfk[5][0]*
      Vpk[0][10][0])+(mfk[5][1]*Vpk[0][10][1])))+(((mfk[4][2]*Vpk[0][9][2])+((
      mfk[4][0]*Vpk[0][9][0])+(mfk[4][1]*Vpk[0][9][1])))+temp[0]))));
    temp[2] = (((mfk[11][2]*Vpk[0][16][2])+((mfk[11][0]*Vpk[0][16][0])+(
      mfk[11][1]*Vpk[0][16][1])))+(((mfk[10][2]*Vpk[0][15][2])+((mfk[10][0]*
      Vpk[0][15][0])+(mfk[10][1]*Vpk[0][15][1])))+(((mfk[9][2]*Vpk[0][14][2])+((
      mfk[9][0]*Vpk[0][14][0])+(mfk[9][1]*Vpk[0][14][1])))+(((mfk[8][2]*
      Vpk[0][13][2])+((mfk[8][0]*Vpk[0][13][0])+(mfk[8][1]*Vpk[0][13][1])))+
      temp[1]))));
    fs[0] = (mtau[0]+(((mfk[15][2]*Vpk[0][20][2])+((mfk[15][0]*Vpk[0][20][0])+(
      mfk[15][1]*Vpk[0][20][1])))+(((mfk[14][2]*Vpk[0][19][2])+((mfk[14][0]*
      Vpk[0][19][0])+(mfk[14][1]*Vpk[0][19][1])))+(((mfk[13][2]*Vpk[0][18][2])+(
      (mfk[13][0]*Vpk[0][18][0])+(mfk[13][1]*Vpk[0][18][1])))+(((mfk[12][2]*
      Vpk[0][17][2])+((mfk[12][0]*Vpk[0][17][0])+(mfk[12][1]*Vpk[0][17][1])))+
      temp[2])))));
    temp[0] = (((mfk[3][2]*Vpk[1][8][2])+((mfk[3][0]*Vpk[1][8][0])+(mfk[3][1]*
      Vpk[1][8][1])))+(((mfk[2][2]*Vpk[1][7][2])+((mfk[2][0]*Vpk[1][7][0])+(
      mfk[2][1]*Vpk[1][7][1])))+(((mfk[0][2]*Vpk[1][3][2])+((mfk[0][0]*
      Vpk[1][3][0])+(mfk[0][1]*Vpk[1][3][1])))+((mfk[1][2]*Vpk[1][6][2])+((
      mfk[1][0]*Vpk[1][6][0])+(mfk[1][1]*Vpk[1][6][1]))))));
    temp[1] = (((mfk[7][2]*Vpk[1][12][2])+((mfk[7][0]*Vpk[1][12][0])+(mfk[7][1]*
      Vpk[1][12][1])))+(((mfk[6][2]*Vpk[1][11][2])+((mfk[6][0]*Vpk[1][11][0])+(
      mfk[6][1]*Vpk[1][11][1])))+(((mfk[5][2]*Vpk[1][10][2])+((mfk[5][0]*
      Vpk[1][10][0])+(mfk[5][1]*Vpk[1][10][1])))+(((mfk[4][2]*Vpk[1][9][2])+((
      mfk[4][0]*Vpk[1][9][0])+(mfk[4][1]*Vpk[1][9][1])))+temp[0]))));
    temp[2] = (((mfk[11][2]*Vpk[1][16][2])+((mfk[11][0]*Vpk[1][16][0])+(
      mfk[11][1]*Vpk[1][16][1])))+(((mfk[10][2]*Vpk[1][15][2])+((mfk[10][0]*
      Vpk[1][15][0])+(mfk[10][1]*Vpk[1][15][1])))+(((mfk[9][2]*Vpk[1][14][2])+((
      mfk[9][0]*Vpk[1][14][0])+(mfk[9][1]*Vpk[1][14][1])))+(((mfk[8][2]*
      Vpk[1][13][2])+((mfk[8][0]*Vpk[1][13][0])+(mfk[8][1]*Vpk[1][13][1])))+
      temp[1]))));
    fs[1] = (mtau[1]+(((mfk[15][2]*Vpk[1][20][2])+((mfk[15][0]*Vpk[1][20][0])+(
      mfk[15][1]*Vpk[1][20][1])))+(((mfk[14][2]*Vpk[1][19][2])+((mfk[14][0]*
      Vpk[1][19][0])+(mfk[14][1]*Vpk[1][19][1])))+(((mfk[13][2]*Vpk[1][18][2])+(
      (mfk[13][0]*Vpk[1][18][0])+(mfk[13][1]*Vpk[1][18][1])))+(((mfk[12][2]*
      Vpk[1][17][2])+((mfk[12][0]*Vpk[1][17][0])+(mfk[12][1]*Vpk[1][17][1])))+
      temp[2])))));
    temp[0] = (((mfk[3][2]*Vpk[2][8][2])+((mfk[3][0]*Vpk[2][8][0])+(mfk[3][1]*
      Vpk[2][8][1])))+(((mfk[2][2]*Vpk[2][7][2])+((mfk[2][0]*Vpk[2][7][0])+(
      mfk[2][1]*Vpk[2][7][1])))+(((mfk[0][2]*Vpk[2][3][2])+((mfk[0][0]*
      Vpk[2][3][0])+(mfk[0][1]*Vpk[2][3][1])))+((mfk[1][2]*Vpk[2][6][2])+((
      mfk[1][0]*Vpk[2][6][0])+(mfk[1][1]*Vpk[2][6][1]))))));
    temp[1] = (((mfk[7][2]*Vpk[2][12][2])+((mfk[7][0]*Vpk[2][12][0])+(mfk[7][1]*
      Vpk[2][12][1])))+(((mfk[6][2]*Vpk[2][11][2])+((mfk[6][0]*Vpk[2][11][0])+(
      mfk[6][1]*Vpk[2][11][1])))+(((mfk[5][2]*Vpk[2][10][2])+((mfk[5][0]*
      Vpk[2][10][0])+(mfk[5][1]*Vpk[2][10][1])))+(((mfk[4][2]*Vpk[2][9][2])+((
      mfk[4][0]*Vpk[2][9][0])+(mfk[4][1]*Vpk[2][9][1])))+temp[0]))));
    temp[2] = (((mfk[11][2]*Vpk[2][16][2])+((mfk[11][0]*Vpk[2][16][0])+(
      mfk[11][1]*Vpk[2][16][1])))+(((mfk[10][2]*Vpk[2][15][2])+((mfk[10][0]*
      Vpk[2][15][0])+(mfk[10][1]*Vpk[2][15][1])))+(((mfk[9][2]*Vpk[2][14][2])+((
      mfk[9][0]*Vpk[2][14][0])+(mfk[9][1]*Vpk[2][14][1])))+(((mfk[8][2]*
      Vpk[2][13][2])+((mfk[8][0]*Vpk[2][13][0])+(mfk[8][1]*Vpk[2][13][1])))+
      temp[1]))));
    fs[2] = (mtau[2]+(((mfk[15][2]*Vpk[2][20][2])+((mfk[15][0]*Vpk[2][20][0])+(
      mfk[15][1]*Vpk[2][20][1])))+(((mfk[14][2]*Vpk[2][19][2])+((mfk[14][0]*
      Vpk[2][19][0])+(mfk[14][1]*Vpk[2][19][1])))+(((mfk[13][2]*Vpk[2][18][2])+(
      (mfk[13][0]*Vpk[2][18][0])+(mfk[13][1]*Vpk[2][18][1])))+(((mfk[12][2]*
      Vpk[2][17][2])+((mfk[12][0]*Vpk[2][17][0])+(mfk[12][1]*Vpk[2][17][1])))+
      temp[2])))));
    temp[0] = (((((mfk[0][2]*rk[0][1])-(mfk[0][1]*rk[0][2]))-mtk[0][0])-(((
      Cik[6][0][2]*mtk[1][2])+((Cik[6][0][0]*mtk[1][0])+(Cik[6][0][1]*mtk[1][1])
      ))+((mfk[1][2]*Vpk[3][6][2])+((mfk[1][0]*Vpk[3][6][0])+(mfk[1][1]*
      Vpk[3][6][1])))))-(((mfk[2][2]*Vpk[3][7][2])+((mfk[2][0]*Vpk[3][7][0])+(
      mfk[2][1]*Vpk[3][7][1])))+((mtk[2][2]*Wpk[3][7][2])+((mtk[2][0]*
      Wpk[3][7][0])+(mtk[2][1]*Wpk[3][7][1])))));
    temp[1] = ((temp[0]-(((mfk[3][2]*Vpk[3][8][2])+((mfk[3][0]*Vpk[3][8][0])+(
      mfk[3][1]*Vpk[3][8][1])))+((mtk[3][2]*Wpk[3][8][2])+((mtk[3][0]*
      Wpk[3][8][0])+(mtk[3][1]*Wpk[3][8][1])))))-(((Cik[9][0][2]*mtk[4][2])+((
      Cik[9][0][0]*mtk[4][0])+(Cik[9][0][1]*mtk[4][1])))+((mfk[4][2]*
      Vpk[3][9][2])+((mfk[4][0]*Vpk[3][9][0])+(mfk[4][1]*Vpk[3][9][1])))));
    temp[2] = ((temp[1]-(((mfk[5][2]*Vpk[3][10][2])+((mfk[5][0]*Vpk[3][10][0])+(
      mfk[5][1]*Vpk[3][10][1])))+((mtk[5][2]*Wpk[3][10][2])+((mtk[5][0]*
      Wpk[3][10][0])+(mtk[5][1]*Wpk[3][10][1])))))-(((mfk[6][2]*Vpk[3][11][2])+(
      (mfk[6][0]*Vpk[3][11][0])+(mfk[6][1]*Vpk[3][11][1])))+((mtk[6][2]*
      Wpk[3][11][2])+((mtk[6][0]*Wpk[3][11][0])+(mtk[6][1]*Wpk[3][11][1])))));
    temp[3] = ((temp[2]-(((mfk[7][2]*Vpk[3][12][2])+((mfk[7][0]*Vpk[3][12][0])+(
      mfk[7][1]*Vpk[3][12][1])))+((mtk[7][2]*Wpk[3][12][2])+((mtk[7][0]*
      Wpk[3][12][0])+(mtk[7][1]*Wpk[3][12][1])))))-(((mfk[8][2]*Vpk[3][13][2])+(
      (mfk[8][0]*Vpk[3][13][0])+(mfk[8][1]*Vpk[3][13][1])))+((mtk[8][2]*
      Wpk[3][13][2])+((mtk[8][0]*Wpk[3][13][0])+(mtk[8][1]*Wpk[3][13][1])))));
    temp[4] = ((temp[3]-(((mfk[9][2]*Vpk[3][14][2])+((mfk[9][0]*Vpk[3][14][0])+(
      mfk[9][1]*Vpk[3][14][1])))+((mtk[9][2]*Wpk[3][14][2])+((mtk[9][0]*
      Wpk[3][14][0])+(mtk[9][1]*Wpk[3][14][1])))))-(((Cik[15][0][2]*mtk[10][2])+
      ((Cik[15][0][0]*mtk[10][0])+(Cik[15][0][1]*mtk[10][1])))+((mfk[10][2]*
      Vpk[3][15][2])+((mfk[10][0]*Vpk[3][15][0])+(mfk[10][1]*Vpk[3][15][1])))));
    temp[5] = ((temp[4]-(((mfk[11][2]*Vpk[3][16][2])+((mfk[11][0]*Vpk[3][16][0])
      +(mfk[11][1]*Vpk[3][16][1])))+((mtk[11][2]*Wpk[3][16][2])+((mtk[11][0]*
      Wpk[3][16][0])+(mtk[11][1]*Wpk[3][16][1])))))-(((mfk[12][2]*Vpk[3][17][2])
      +((mfk[12][0]*Vpk[3][17][0])+(mfk[12][1]*Vpk[3][17][1])))+((mtk[12][2]*
      Wpk[3][17][2])+((mtk[12][0]*Wpk[3][17][0])+(mtk[12][1]*Wpk[3][17][1])))));
    temp[6] = ((temp[5]-(((mfk[13][2]*Vpk[3][18][2])+((mfk[13][0]*Vpk[3][18][0])
      +(mfk[13][1]*Vpk[3][18][1])))+((mtk[13][2]*Wpk[3][18][2])+((mtk[13][0]*
      Wpk[3][18][0])+(mtk[13][1]*Wpk[3][18][1])))))-(((mfk[14][2]*Vpk[3][19][2])
      +((mfk[14][0]*Vpk[3][19][0])+(mfk[14][1]*Vpk[3][19][1])))+((mtk[14][2]*
      Wpk[3][19][2])+((mtk[14][0]*Wpk[3][19][0])+(mtk[14][1]*Wpk[3][19][1])))));
    fs[3] = (mtau[3]-(temp[6]-(((mfk[15][2]*Vpk[3][20][2])+((mfk[15][0]*
      Vpk[3][20][0])+(mfk[15][1]*Vpk[3][20][1])))+((mtk[15][2]*Wpk[3][20][2])+((
      mtk[15][0]*Wpk[3][20][0])+(mtk[15][1]*Wpk[3][20][1]))))));
    temp[0] = (((((mfk[0][0]*rk[0][2])-(mfk[0][2]*rk[0][0]))-mtk[0][1])-(((
      Cik[6][1][2]*mtk[1][2])+((Cik[6][1][0]*mtk[1][0])+(Cik[6][1][1]*mtk[1][1])
      ))+((mfk[1][2]*Vpk[4][6][2])+((mfk[1][0]*Vpk[4][6][0])+(mfk[1][1]*
      Vpk[4][6][1])))))-(((mfk[2][2]*Vpk[4][7][2])+((mfk[2][0]*Vpk[4][7][0])+(
      mfk[2][1]*Vpk[4][7][1])))+((mtk[2][2]*Wpk[4][7][2])+((mtk[2][0]*
      Wpk[4][7][0])+(mtk[2][1]*Wpk[4][7][1])))));
    temp[1] = ((temp[0]-(((mfk[3][2]*Vpk[4][8][2])+((mfk[3][0]*Vpk[4][8][0])+(
      mfk[3][1]*Vpk[4][8][1])))+((mtk[3][2]*Wpk[4][8][2])+((mtk[3][0]*
      Wpk[4][8][0])+(mtk[3][1]*Wpk[4][8][1])))))-(((Cik[9][1][2]*mtk[4][2])+((
      Cik[9][1][0]*mtk[4][0])+(Cik[9][1][1]*mtk[4][1])))+((mfk[4][2]*
      Vpk[4][9][2])+((mfk[4][0]*Vpk[4][9][0])+(mfk[4][1]*Vpk[4][9][1])))));
    temp[2] = ((temp[1]-(((mfk[5][2]*Vpk[4][10][2])+((mfk[5][0]*Vpk[4][10][0])+(
      mfk[5][1]*Vpk[4][10][1])))+((mtk[5][2]*Wpk[4][10][2])+((mtk[5][0]*
      Wpk[4][10][0])+(mtk[5][1]*Wpk[4][10][1])))))-(((mfk[6][2]*Vpk[4][11][2])+(
      (mfk[6][0]*Vpk[4][11][0])+(mfk[6][1]*Vpk[4][11][1])))+((mtk[6][2]*
      Wpk[4][11][2])+((mtk[6][0]*Wpk[4][11][0])+(mtk[6][1]*Wpk[4][11][1])))));
    temp[3] = ((temp[2]-(((mfk[7][2]*Vpk[4][12][2])+((mfk[7][0]*Vpk[4][12][0])+(
      mfk[7][1]*Vpk[4][12][1])))+((mtk[7][2]*Wpk[4][12][2])+((mtk[7][0]*
      Wpk[4][12][0])+(mtk[7][1]*Wpk[4][12][1])))))-(((mfk[8][2]*Vpk[4][13][2])+(
      (mfk[8][0]*Vpk[4][13][0])+(mfk[8][1]*Vpk[4][13][1])))+((mtk[8][2]*
      Wpk[4][13][2])+((mtk[8][0]*Wpk[4][13][0])+(mtk[8][1]*Wpk[4][13][1])))));
    temp[4] = ((temp[3]-(((mfk[9][2]*Vpk[4][14][2])+((mfk[9][0]*Vpk[4][14][0])+(
      mfk[9][1]*Vpk[4][14][1])))+((mtk[9][2]*Wpk[4][14][2])+((mtk[9][0]*
      Wpk[4][14][0])+(mtk[9][1]*Wpk[4][14][1])))))-(((Cik[15][1][2]*mtk[10][2])+
      ((Cik[15][1][0]*mtk[10][0])+(Cik[15][1][1]*mtk[10][1])))+((mfk[10][2]*
      Vpk[4][15][2])+((mfk[10][0]*Vpk[4][15][0])+(mfk[10][1]*Vpk[4][15][1])))));
    temp[5] = ((temp[4]-(((mfk[11][2]*Vpk[4][16][2])+((mfk[11][0]*Vpk[4][16][0])
      +(mfk[11][1]*Vpk[4][16][1])))+((mtk[11][2]*Wpk[4][16][2])+((mtk[11][0]*
      Wpk[4][16][0])+(mtk[11][1]*Wpk[4][16][1])))))-(((mfk[12][2]*Vpk[4][17][2])
      +((mfk[12][0]*Vpk[4][17][0])+(mfk[12][1]*Vpk[4][17][1])))+((mtk[12][2]*
      Wpk[4][17][2])+((mtk[12][0]*Wpk[4][17][0])+(mtk[12][1]*Wpk[4][17][1])))));
    temp[6] = ((temp[5]-(((mfk[13][2]*Vpk[4][18][2])+((mfk[13][0]*Vpk[4][18][0])
      +(mfk[13][1]*Vpk[4][18][1])))+((mtk[13][2]*Wpk[4][18][2])+((mtk[13][0]*
      Wpk[4][18][0])+(mtk[13][1]*Wpk[4][18][1])))))-(((mfk[14][2]*Vpk[4][19][2])
      +((mfk[14][0]*Vpk[4][19][0])+(mfk[14][1]*Vpk[4][19][1])))+((mtk[14][2]*
      Wpk[4][19][2])+((mtk[14][0]*Wpk[4][19][0])+(mtk[14][1]*Wpk[4][19][1])))));
    fs[4] = (mtau[4]-(temp[6]-(((mfk[15][2]*Vpk[4][20][2])+((mfk[15][0]*
      Vpk[4][20][0])+(mfk[15][1]*Vpk[4][20][1])))+((mtk[15][2]*Wpk[4][20][2])+((
      mtk[15][0]*Wpk[4][20][0])+(mtk[15][1]*Wpk[4][20][1]))))));
    temp[0] = (((((mfk[0][1]*rk[0][0])-(mfk[0][0]*rk[0][1]))-mtk[0][2])-(((
      Cik[6][2][2]*mtk[1][2])+((Cik[6][2][0]*mtk[1][0])+(Cik[6][2][1]*mtk[1][1])
      ))+((mfk[1][2]*Vpk[5][6][2])+((mfk[1][0]*Vpk[5][6][0])+(mfk[1][1]*
      Vpk[5][6][1])))))-(((mfk[2][2]*Vpk[5][7][2])+((mfk[2][0]*Vpk[5][7][0])+(
      mfk[2][1]*Vpk[5][7][1])))+((mtk[2][2]*Wpk[5][7][2])+((mtk[2][0]*
      Wpk[5][7][0])+(mtk[2][1]*Wpk[5][7][1])))));
    temp[1] = ((temp[0]-(((mfk[3][2]*Vpk[5][8][2])+((mfk[3][0]*Vpk[5][8][0])+(
      mfk[3][1]*Vpk[5][8][1])))+((mtk[3][2]*Wpk[5][8][2])+((mtk[3][0]*
      Wpk[5][8][0])+(mtk[3][1]*Wpk[5][8][1])))))-(((Cik[9][2][2]*mtk[4][2])+((
      Cik[9][2][0]*mtk[4][0])+(Cik[9][2][1]*mtk[4][1])))+((mfk[4][2]*
      Vpk[5][9][2])+((mfk[4][0]*Vpk[5][9][0])+(mfk[4][1]*Vpk[5][9][1])))));
    temp[2] = ((temp[1]-(((mfk[5][2]*Vpk[5][10][2])+((mfk[5][0]*Vpk[5][10][0])+(
      mfk[5][1]*Vpk[5][10][1])))+((mtk[5][2]*Wpk[5][10][2])+((mtk[5][0]*
      Wpk[5][10][0])+(mtk[5][1]*Wpk[5][10][1])))))-(((mfk[6][2]*Vpk[5][11][2])+(
      (mfk[6][0]*Vpk[5][11][0])+(mfk[6][1]*Vpk[5][11][1])))+((mtk[6][2]*
      Wpk[5][11][2])+((mtk[6][0]*Wpk[5][11][0])+(mtk[6][1]*Wpk[5][11][1])))));
    temp[3] = ((temp[2]-(((mfk[7][2]*Vpk[5][12][2])+((mfk[7][0]*Vpk[5][12][0])+(
      mfk[7][1]*Vpk[5][12][1])))+((mtk[7][2]*Wpk[5][12][2])+((mtk[7][0]*
      Wpk[5][12][0])+(mtk[7][1]*Wpk[5][12][1])))))-(((mfk[8][2]*Vpk[5][13][2])+(
      (mfk[8][0]*Vpk[5][13][0])+(mfk[8][1]*Vpk[5][13][1])))+((mtk[8][2]*
      Wpk[5][13][2])+((mtk[8][0]*Wpk[5][13][0])+(mtk[8][1]*Wpk[5][13][1])))));
    temp[4] = ((temp[3]-(((mfk[9][2]*Vpk[5][14][2])+((mfk[9][0]*Vpk[5][14][0])+(
      mfk[9][1]*Vpk[5][14][1])))+((mtk[9][2]*Wpk[5][14][2])+((mtk[9][0]*
      Wpk[5][14][0])+(mtk[9][1]*Wpk[5][14][1])))))-(((Cik[15][2][2]*mtk[10][2])+
      ((Cik[15][2][0]*mtk[10][0])+(Cik[15][2][1]*mtk[10][1])))+((mfk[10][2]*
      Vpk[5][15][2])+((mfk[10][0]*Vpk[5][15][0])+(mfk[10][1]*Vpk[5][15][1])))));
    temp[5] = ((temp[4]-(((mfk[11][2]*Vpk[5][16][2])+((mfk[11][0]*Vpk[5][16][0])
      +(mfk[11][1]*Vpk[5][16][1])))+((mtk[11][2]*Wpk[5][16][2])+((mtk[11][0]*
      Wpk[5][16][0])+(mtk[11][1]*Wpk[5][16][1])))))-(((mfk[12][2]*Vpk[5][17][2])
      +((mfk[12][0]*Vpk[5][17][0])+(mfk[12][1]*Vpk[5][17][1])))+((mtk[12][2]*
      Wpk[5][17][2])+((mtk[12][0]*Wpk[5][17][0])+(mtk[12][1]*Wpk[5][17][1])))));
    temp[6] = ((temp[5]-(((mfk[13][2]*Vpk[5][18][2])+((mfk[13][0]*Vpk[5][18][0])
      +(mfk[13][1]*Vpk[5][18][1])))+((mtk[13][2]*Wpk[5][18][2])+((mtk[13][0]*
      Wpk[5][18][0])+(mtk[13][1]*Wpk[5][18][1])))))-(((mfk[14][2]*Vpk[5][19][2])
      +((mfk[14][0]*Vpk[5][19][0])+(mfk[14][1]*Vpk[5][19][1])))+((mtk[14][2]*
      Wpk[5][19][2])+((mtk[14][0]*Wpk[5][19][0])+(mtk[14][1]*Wpk[5][19][1])))));
    fs[5] = (mtau[5]-(temp[6]-(((mfk[15][2]*Vpk[5][20][2])+((mfk[15][0]*
      Vpk[5][20][0])+(mfk[15][1]*Vpk[5][20][1])))+((mtk[15][2]*Wpk[5][20][2])+((
      mtk[15][0]*Wpk[5][20][0])+(mtk[15][1]*Wpk[5][20][1]))))));
    temp[0] = ((((mfk[1][2]*Vpk[6][6][2])+((mfk[1][0]*Vpk[6][6][0])+(mfk[1][1]*
      Vpk[6][6][1])))+((mtk[1][2]*pin[6][2])+((mtk[1][0]*pin[6][0])+(mtk[1][1]*
      pin[6][1]))))+(((mfk[2][2]*Vpk[6][7][2])+((mfk[2][0]*Vpk[6][7][0])+(
      mfk[2][1]*Vpk[6][7][1])))+((mtk[2][2]*Wpk[6][7][2])+((mtk[2][0]*
      Wpk[6][7][0])+(mtk[2][1]*Wpk[6][7][1])))));
    fs[6] = (mtau[6]+((((mfk[3][2]*Vpk[6][8][2])+((mfk[3][0]*Vpk[6][8][0])+(
      mfk[3][1]*Vpk[6][8][1])))+((mtk[3][2]*Wpk[6][8][2])+((mtk[3][0]*
      Wpk[6][8][0])+(mtk[3][1]*Wpk[6][8][1]))))+temp[0]));
    fs[7] = (mtau[7]+((((mfk[2][2]*Vpk[7][7][2])+((mfk[2][0]*Vpk[7][7][0])+(
      mfk[2][1]*Vpk[7][7][1])))+((mtk[2][2]*pin[7][2])+((mtk[2][0]*pin[7][0])+(
      mtk[2][1]*pin[7][1]))))+(((mfk[3][2]*Vpk[7][8][2])+((mfk[3][0]*
      Vpk[7][8][0])+(mfk[3][1]*Vpk[7][8][1])))+((mtk[3][2]*Wpk[7][8][2])+((
      mtk[3][0]*Wpk[7][8][0])+(mtk[3][1]*Wpk[7][8][1]))))));
    fs[8] = (mtau[8]+(((mfk[3][2]*Vpk[8][8][2])+((mfk[3][0]*Vpk[8][8][0])+(
      mfk[3][1]*Vpk[8][8][1])))+((mtk[3][2]*pin[8][2])+((mtk[3][0]*pin[8][0])+(
      mtk[3][1]*pin[8][1])))));
    temp[0] = ((((mfk[4][2]*Vpk[9][9][2])+((mfk[4][0]*Vpk[9][9][0])+(mfk[4][1]*
      Vpk[9][9][1])))+((mtk[4][2]*pin[9][2])+((mtk[4][0]*pin[9][0])+(mtk[4][1]*
      pin[9][1]))))+(((mfk[5][2]*Vpk[9][10][2])+((mfk[5][0]*Vpk[9][10][0])+(
      mfk[5][1]*Vpk[9][10][1])))+((mtk[5][2]*Wpk[9][10][2])+((mtk[5][0]*
      Wpk[9][10][0])+(mtk[5][1]*Wpk[9][10][1])))));
    temp[1] = ((((mfk[7][2]*Vpk[9][12][2])+((mfk[7][0]*Vpk[9][12][0])+(mfk[7][1]
      *Vpk[9][12][1])))+((mtk[7][2]*Wpk[9][12][2])+((mtk[7][0]*Wpk[9][12][0])+(
      mtk[7][1]*Wpk[9][12][1]))))+((((mfk[6][2]*Vpk[9][11][2])+((mfk[6][0]*
      Vpk[9][11][0])+(mfk[6][1]*Vpk[9][11][1])))+((mtk[6][2]*Wpk[9][11][2])+((
      mtk[6][0]*Wpk[9][11][0])+(mtk[6][1]*Wpk[9][11][1]))))+temp[0]));
    fs[9] = (mtau[9]+((((mfk[9][2]*Vpk[9][14][2])+((mfk[9][0]*Vpk[9][14][0])+(
      mfk[9][1]*Vpk[9][14][1])))+((mtk[9][2]*Wpk[9][14][2])+((mtk[9][0]*
      Wpk[9][14][0])+(mtk[9][1]*Wpk[9][14][1]))))+((((mfk[8][2]*Vpk[9][13][2])+(
      (mfk[8][0]*Vpk[9][13][0])+(mfk[8][1]*Vpk[9][13][1])))+((mtk[8][2]*
      Wpk[9][13][2])+((mtk[8][0]*Wpk[9][13][0])+(mtk[8][1]*Wpk[9][13][1]))))+
      temp[1])));
    temp[0] = ((((mfk[5][2]*Vpk[10][10][2])+((mfk[5][0]*Vpk[10][10][0])+(
      mfk[5][1]*Vpk[10][10][1])))+((mtk[5][2]*pin[10][2])+((mtk[5][0]*pin[10][0]
      )+(mtk[5][1]*pin[10][1]))))+(((mfk[6][2]*Vpk[10][11][2])+((mfk[6][0]*
      Vpk[10][11][0])+(mfk[6][1]*Vpk[10][11][1])))+((mtk[6][2]*Wpk[10][11][2])+(
      (mtk[6][0]*Wpk[10][11][0])+(mtk[6][1]*Wpk[10][11][1])))));
    temp[1] = ((((mfk[8][2]*Vpk[10][13][2])+((mfk[8][0]*Vpk[10][13][0])+(
      mfk[8][1]*Vpk[10][13][1])))+((mtk[8][2]*Wpk[10][13][2])+((mtk[8][0]*
      Wpk[10][13][0])+(mtk[8][1]*Wpk[10][13][1]))))+((((mfk[7][2]*Vpk[10][12][2]
      )+((mfk[7][0]*Vpk[10][12][0])+(mfk[7][1]*Vpk[10][12][1])))+((mtk[7][2]*
      Wpk[10][12][2])+((mtk[7][0]*Wpk[10][12][0])+(mtk[7][1]*Wpk[10][12][1]))))+
      temp[0]));
    fs[10] = (mtau[10]+((((mfk[9][2]*Vpk[10][14][2])+((mfk[9][0]*Vpk[10][14][0])
      +(mfk[9][1]*Vpk[10][14][1])))+((mtk[9][2]*Wpk[10][14][2])+((mtk[9][0]*
      Wpk[10][14][0])+(mtk[9][1]*Wpk[10][14][1]))))+temp[1]));
    temp[0] = ((((mfk[6][2]*Vpk[11][11][2])+((mfk[6][0]*Vpk[11][11][0])+(
      mfk[6][1]*Vpk[11][11][1])))+((mtk[6][2]*pin[11][2])+((mtk[6][0]*pin[11][0]
      )+(mtk[6][1]*pin[11][1]))))+(((mfk[7][2]*Vpk[11][12][2])+((mfk[7][0]*
      Vpk[11][12][0])+(mfk[7][1]*Vpk[11][12][1])))+((mtk[7][2]*Wpk[11][12][2])+(
      (mtk[7][0]*Wpk[11][12][0])+(mtk[7][1]*Wpk[11][12][1])))));
    fs[11] = (mtau[11]+((((mfk[9][2]*Vpk[11][14][2])+((mfk[9][0]*Vpk[11][14][0])
      +(mfk[9][1]*Vpk[11][14][1])))+((mtk[9][2]*Wpk[11][14][2])+((mtk[9][0]*
      Wpk[11][14][0])+(mtk[9][1]*Wpk[11][14][1]))))+((((mfk[8][2]*Vpk[11][13][2]
      )+((mfk[8][0]*Vpk[11][13][0])+(mfk[8][1]*Vpk[11][13][1])))+((mtk[8][2]*
      Wpk[11][13][2])+((mtk[8][0]*Wpk[11][13][0])+(mtk[8][1]*Wpk[11][13][1]))))+
      temp[0])));
    temp[0] = ((((mfk[7][2]*Vpk[12][12][2])+((mfk[7][0]*Vpk[12][12][0])+(
      mfk[7][1]*Vpk[12][12][1])))+((mtk[7][2]*pin[12][2])+((mtk[7][0]*pin[12][0]
      )+(mtk[7][1]*pin[12][1]))))+(((mfk[8][2]*Vpk[12][13][2])+((mfk[8][0]*
      Vpk[12][13][0])+(mfk[8][1]*Vpk[12][13][1])))+((mtk[8][2]*Wpk[12][13][2])+(
      (mtk[8][0]*Wpk[12][13][0])+(mtk[8][1]*Wpk[12][13][1])))));
    fs[12] = (mtau[12]+((((mfk[9][2]*Vpk[12][14][2])+((mfk[9][0]*Vpk[12][14][0])
      +(mfk[9][1]*Vpk[12][14][1])))+((mtk[9][2]*Wpk[12][14][2])+((mtk[9][0]*
      Wpk[12][14][0])+(mtk[9][1]*Wpk[12][14][1]))))+temp[0]));
    fs[13] = (mtau[13]+((((mfk[8][2]*Vpk[13][13][2])+((mfk[8][0]*Vpk[13][13][0])
      +(mfk[8][1]*Vpk[13][13][1])))+((mtk[8][2]*pin[13][2])+((mtk[8][0]*
      pin[13][0])+(mtk[8][1]*pin[13][1]))))+(((mfk[9][2]*Vpk[13][14][2])+((
      mfk[9][0]*Vpk[13][14][0])+(mfk[9][1]*Vpk[13][14][1])))+((mtk[9][2]*
      Wpk[13][14][2])+((mtk[9][0]*Wpk[13][14][0])+(mtk[9][1]*Wpk[13][14][1])))))
      );
    fs[14] = (mtau[14]+(((mfk[9][2]*Vpk[14][14][2])+((mfk[9][0]*Vpk[14][14][0])+
      (mfk[9][1]*Vpk[14][14][1])))+((mtk[9][2]*pin[14][2])+((mtk[9][0]*
      pin[14][0])+(mtk[9][1]*pin[14][1])))));
    temp[0] = ((((mfk[10][2]*Vpk[15][15][2])+((mfk[10][0]*Vpk[15][15][0])+(
      mfk[10][1]*Vpk[15][15][1])))+((mtk[10][2]*pin[15][2])+((mtk[10][0]*
      pin[15][0])+(mtk[10][1]*pin[15][1]))))+(((mfk[11][2]*Vpk[15][16][2])+((
      mfk[11][0]*Vpk[15][16][0])+(mfk[11][1]*Vpk[15][16][1])))+((mtk[11][2]*
      Wpk[15][16][2])+((mtk[11][0]*Wpk[15][16][0])+(mtk[11][1]*Wpk[15][16][1])))
      ));
    temp[1] = ((((mfk[13][2]*Vpk[15][18][2])+((mfk[13][0]*Vpk[15][18][0])+(
      mfk[13][1]*Vpk[15][18][1])))+((mtk[13][2]*Wpk[15][18][2])+((mtk[13][0]*
      Wpk[15][18][0])+(mtk[13][1]*Wpk[15][18][1]))))+((((mfk[12][2]*
      Vpk[15][17][2])+((mfk[12][0]*Vpk[15][17][0])+(mfk[12][1]*Vpk[15][17][1])))
      +((mtk[12][2]*Wpk[15][17][2])+((mtk[12][0]*Wpk[15][17][0])+(mtk[12][1]*
      Wpk[15][17][1]))))+temp[0]));
    fs[15] = (mtau[15]+((((mfk[15][2]*Vpk[15][20][2])+((mfk[15][0]*
      Vpk[15][20][0])+(mfk[15][1]*Vpk[15][20][1])))+((mtk[15][2]*Wpk[15][20][2])
      +((mtk[15][0]*Wpk[15][20][0])+(mtk[15][1]*Wpk[15][20][1]))))+((((
      mfk[14][2]*Vpk[15][19][2])+((mfk[14][0]*Vpk[15][19][0])+(mfk[14][1]*
      Vpk[15][19][1])))+((mtk[14][2]*Wpk[15][19][2])+((mtk[14][0]*Wpk[15][19][0]
      )+(mtk[14][1]*Wpk[15][19][1]))))+temp[1])));
    temp[0] = ((((mfk[11][2]*Vpk[16][16][2])+((mfk[11][0]*Vpk[16][16][0])+(
      mfk[11][1]*Vpk[16][16][1])))+((mtk[11][2]*pin[16][2])+((mtk[11][0]*
      pin[16][0])+(mtk[11][1]*pin[16][1]))))+(((mfk[12][2]*Vpk[16][17][2])+((
      mfk[12][0]*Vpk[16][17][0])+(mfk[12][1]*Vpk[16][17][1])))+((mtk[12][2]*
      Wpk[16][17][2])+((mtk[12][0]*Wpk[16][17][0])+(mtk[12][1]*Wpk[16][17][1])))
      ));
    temp[1] = ((((mfk[14][2]*Vpk[16][19][2])+((mfk[14][0]*Vpk[16][19][0])+(
      mfk[14][1]*Vpk[16][19][1])))+((mtk[14][2]*Wpk[16][19][2])+((mtk[14][0]*
      Wpk[16][19][0])+(mtk[14][1]*Wpk[16][19][1]))))+((((mfk[13][2]*
      Vpk[16][18][2])+((mfk[13][0]*Vpk[16][18][0])+(mfk[13][1]*Vpk[16][18][1])))
      +((mtk[13][2]*Wpk[16][18][2])+((mtk[13][0]*Wpk[16][18][0])+(mtk[13][1]*
      Wpk[16][18][1]))))+temp[0]));
    fs[16] = (mtau[16]+((((mfk[15][2]*Vpk[16][20][2])+((mfk[15][0]*
      Vpk[16][20][0])+(mfk[15][1]*Vpk[16][20][1])))+((mtk[15][2]*Wpk[16][20][2])
      +((mtk[15][0]*Wpk[16][20][0])+(mtk[15][1]*Wpk[16][20][1]))))+temp[1]));
    temp[0] = ((((mfk[12][2]*Vpk[17][17][2])+((mfk[12][0]*Vpk[17][17][0])+(
      mfk[12][1]*Vpk[17][17][1])))+((mtk[12][2]*pin[17][2])+((mtk[12][0]*
      pin[17][0])+(mtk[12][1]*pin[17][1]))))+(((mfk[13][2]*Vpk[17][18][2])+((
      mfk[13][0]*Vpk[17][18][0])+(mfk[13][1]*Vpk[17][18][1])))+((mtk[13][2]*
      Wpk[17][18][2])+((mtk[13][0]*Wpk[17][18][0])+(mtk[13][1]*Wpk[17][18][1])))
      ));
    fs[17] = (mtau[17]+((((mfk[15][2]*Vpk[17][20][2])+((mfk[15][0]*
      Vpk[17][20][0])+(mfk[15][1]*Vpk[17][20][1])))+((mtk[15][2]*Wpk[17][20][2])
      +((mtk[15][0]*Wpk[17][20][0])+(mtk[15][1]*Wpk[17][20][1]))))+((((
      mfk[14][2]*Vpk[17][19][2])+((mfk[14][0]*Vpk[17][19][0])+(mfk[14][1]*
      Vpk[17][19][1])))+((mtk[14][2]*Wpk[17][19][2])+((mtk[14][0]*Wpk[17][19][0]
      )+(mtk[14][1]*Wpk[17][19][1]))))+temp[0])));
    temp[0] = ((((mfk[13][2]*Vpk[18][18][2])+((mfk[13][0]*Vpk[18][18][0])+(
      mfk[13][1]*Vpk[18][18][1])))+((mtk[13][2]*pin[18][2])+((mtk[13][0]*
      pin[18][0])+(mtk[13][1]*pin[18][1]))))+(((mfk[14][2]*Vpk[18][19][2])+((
      mfk[14][0]*Vpk[18][19][0])+(mfk[14][1]*Vpk[18][19][1])))+((mtk[14][2]*
      Wpk[18][19][2])+((mtk[14][0]*Wpk[18][19][0])+(mtk[14][1]*Wpk[18][19][1])))
      ));
    fs[18] = (mtau[18]+((((mfk[15][2]*Vpk[18][20][2])+((mfk[15][0]*
      Vpk[18][20][0])+(mfk[15][1]*Vpk[18][20][1])))+((mtk[15][2]*Wpk[18][20][2])
      +((mtk[15][0]*Wpk[18][20][0])+(mtk[15][1]*Wpk[18][20][1]))))+temp[0]));
    fs[19] = (mtau[19]+((((mfk[14][2]*Vpk[19][19][2])+((mfk[14][0]*
      Vpk[19][19][0])+(mfk[14][1]*Vpk[19][19][1])))+((mtk[14][2]*pin[19][2])+((
      mtk[14][0]*pin[19][0])+(mtk[14][1]*pin[19][1]))))+(((mfk[15][2]*
      Vpk[19][20][2])+((mfk[15][0]*Vpk[19][20][0])+(mfk[15][1]*Vpk[19][20][1])))
      +((mtk[15][2]*Wpk[19][20][2])+((mtk[15][0]*Wpk[19][20][0])+(mtk[15][1]*
      Wpk[19][20][1]))))));
    fs[20] = (mtau[20]+(((mfk[15][2]*Vpk[20][20][2])+((mfk[15][0]*Vpk[20][20][0]
      )+(mfk[15][1]*Vpk[20][20][1])))+((mtk[15][2]*pin[20][2])+((mtk[15][0]*
      pin[20][0])+(mtk[15][1]*pin[20][1])))));
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  711 adds/subtracts/negates
                    708 multiplies
                      0 divides
                     64 assignments
*/
}

void sdfsgenfull(void)
{

/*
Compute Fs (incl generic mult & other forces)
*/
    sdfsgenmult();
    fs[0] = (fs[0]+fs0[0]);
    fs[1] = (fs[1]+fs0[1]);
    fs[2] = (fs[2]+fs0[2]);
    fs[3] = (fs[3]+fs0[3]);
    fs[4] = (fs[4]+fs0[4]);
    fs[5] = (fs[5]+fs0[5]);
    fs[6] = (fs[6]+fs0[6]);
    fs[7] = (fs[7]+fs0[7]);
    fs[8] = (fs[8]+fs0[8]);
    fs[9] = (fs[9]+fs0[9]);
    fs[10] = (fs[10]+fs0[10]);
    fs[11] = (fs[11]+fs0[11]);
    fs[12] = (fs[12]+fs0[12]);
    fs[13] = (fs[13]+fs0[13]);
    fs[14] = (fs[14]+fs0[14]);
    fs[15] = (fs[15]+fs0[15]);
    fs[16] = (fs[16]+fs0[16]);
    fs[17] = (fs[17]+fs0[17]);
    fs[18] = (fs[18]+fs0[18]);
    fs[19] = (fs[19]+fs0[19]);
    fs[20] = (fs[20]+fs0[20]);
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   21 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     21 assignments
*/
}

void sdfulltrq(double udotin[21],
    double multin[21],
    double trqout[21])
{
/* Compute hinge torques which would produce indicated udots
*/
    double fstarr[21][3],tstarr[21][3],Otkr[21][3],Atir[21][3],Atkr[21][3];

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(61,23);
        return;
    }
/*
Compute multiplier-generated forces
*/
    sdmfrc(multin);
/*
Account for inertial accelerations and supplied udots
*/
    Otkr[6][0] = (((Cik[6][2][0]*udotin[5])+((Cik[6][0][0]*udotin[3])+(
      Cik[6][1][0]*udotin[4])))+((pin[6][0]*udotin[6])+((Wik[6][2]*wk[6][1])-(
      Wik[6][1]*wk[6][2]))));
    Otkr[6][1] = (((Cik[6][2][1]*udotin[5])+((Cik[6][0][1]*udotin[3])+(
      Cik[6][1][1]*udotin[4])))+((pin[6][1]*udotin[6])+((Wik[6][0]*wk[6][2])-(
      Wik[6][2]*wk[6][0]))));
    Otkr[6][2] = (((Cik[6][2][2]*udotin[5])+((Cik[6][0][2]*udotin[3])+(
      Cik[6][1][2]*udotin[4])))+((pin[6][2]*udotin[6])+((Wik[6][1]*wk[6][0])-(
      Wik[6][0]*wk[6][1]))));
    Otkr[7][0] = (((Cik[7][2][0]*Otkr[6][2])+((Cik[7][0][0]*Otkr[6][0])+(
      Cik[7][1][0]*Otkr[6][1])))+((pin[7][0]*udotin[7])+((Wik[7][2]*wk[7][1])-(
      Wik[7][1]*wk[7][2]))));
    Otkr[7][1] = (((Cik[7][2][1]*Otkr[6][2])+((Cik[7][0][1]*Otkr[6][0])+(
      Cik[7][1][1]*Otkr[6][1])))+((pin[7][1]*udotin[7])+((Wik[7][0]*wk[7][2])-(
      Wik[7][2]*wk[7][0]))));
    Otkr[7][2] = (((Cik[7][2][2]*Otkr[6][2])+((Cik[7][0][2]*Otkr[6][0])+(
      Cik[7][1][2]*Otkr[6][1])))+((pin[7][2]*udotin[7])+((Wik[7][1]*wk[7][0])-(
      Wik[7][0]*wk[7][1]))));
    Otkr[8][0] = (((Cik[8][2][0]*Otkr[7][2])+((Cik[8][0][0]*Otkr[7][0])+(
      Cik[8][1][0]*Otkr[7][1])))+((pin[8][0]*udotin[8])+((Wik[8][2]*wk[8][1])-(
      Wik[8][1]*wk[8][2]))));
    Otkr[8][1] = (((Cik[8][2][1]*Otkr[7][2])+((Cik[8][0][1]*Otkr[7][0])+(
      Cik[8][1][1]*Otkr[7][1])))+((pin[8][1]*udotin[8])+((Wik[8][0]*wk[8][2])-(
      Wik[8][2]*wk[8][0]))));
    Otkr[8][2] = (((Cik[8][2][2]*Otkr[7][2])+((Cik[8][0][2]*Otkr[7][0])+(
      Cik[8][1][2]*Otkr[7][1])))+((pin[8][2]*udotin[8])+((Wik[8][1]*wk[8][0])-(
      Wik[8][0]*wk[8][1]))));
    Otkr[9][0] = (((Cik[9][2][0]*udotin[5])+((Cik[9][0][0]*udotin[3])+(
      Cik[9][1][0]*udotin[4])))+((pin[9][0]*udotin[9])+((Wik[9][2]*wk[9][1])-(
      Wik[9][1]*wk[9][2]))));
    Otkr[9][1] = (((Cik[9][2][1]*udotin[5])+((Cik[9][0][1]*udotin[3])+(
      Cik[9][1][1]*udotin[4])))+((pin[9][1]*udotin[9])+((Wik[9][0]*wk[9][2])-(
      Wik[9][2]*wk[9][0]))));
    Otkr[9][2] = (((Cik[9][2][2]*udotin[5])+((Cik[9][0][2]*udotin[3])+(
      Cik[9][1][2]*udotin[4])))+((pin[9][2]*udotin[9])+((Wik[9][1]*wk[9][0])-(
      Wik[9][0]*wk[9][1]))));
    Otkr[10][0] = (((Cik[10][2][0]*Otkr[9][2])+((Cik[10][0][0]*Otkr[9][0])+(
      Cik[10][1][0]*Otkr[9][1])))+((pin[10][0]*udotin[10])+((Wik[10][2]*
      wk[10][1])-(Wik[10][1]*wk[10][2]))));
    Otkr[10][1] = (((Cik[10][2][1]*Otkr[9][2])+((Cik[10][0][1]*Otkr[9][0])+(
      Cik[10][1][1]*Otkr[9][1])))+((pin[10][1]*udotin[10])+((Wik[10][0]*
      wk[10][2])-(Wik[10][2]*wk[10][0]))));
    Otkr[10][2] = (((Cik[10][2][2]*Otkr[9][2])+((Cik[10][0][2]*Otkr[9][0])+(
      Cik[10][1][2]*Otkr[9][1])))+((pin[10][2]*udotin[10])+((Wik[10][1]*
      wk[10][0])-(Wik[10][0]*wk[10][1]))));
    Otkr[11][0] = (((Cik[11][2][0]*Otkr[10][2])+((Cik[11][0][0]*Otkr[10][0])+(
      Cik[11][1][0]*Otkr[10][1])))+((pin[11][0]*udotin[11])+((Wik[11][2]*
      wk[11][1])-(Wik[11][1]*wk[11][2]))));
    Otkr[11][1] = (((Cik[11][2][1]*Otkr[10][2])+((Cik[11][0][1]*Otkr[10][0])+(
      Cik[11][1][1]*Otkr[10][1])))+((pin[11][1]*udotin[11])+((Wik[11][0]*
      wk[11][2])-(Wik[11][2]*wk[11][0]))));
    Otkr[11][2] = (((Cik[11][2][2]*Otkr[10][2])+((Cik[11][0][2]*Otkr[10][0])+(
      Cik[11][1][2]*Otkr[10][1])))+((pin[11][2]*udotin[11])+((Wik[11][1]*
      wk[11][0])-(Wik[11][0]*wk[11][1]))));
    Otkr[12][0] = (((Cik[12][2][0]*Otkr[11][2])+((Cik[12][0][0]*Otkr[11][0])+(
      Cik[12][1][0]*Otkr[11][1])))+((pin[12][0]*udotin[12])+((Wik[12][2]*
      wk[12][1])-(Wik[12][1]*wk[12][2]))));
    Otkr[12][1] = (((Cik[12][2][1]*Otkr[11][2])+((Cik[12][0][1]*Otkr[11][0])+(
      Cik[12][1][1]*Otkr[11][1])))+((pin[12][1]*udotin[12])+((Wik[12][0]*
      wk[12][2])-(Wik[12][2]*wk[12][0]))));
    Otkr[12][2] = (((Cik[12][2][2]*Otkr[11][2])+((Cik[12][0][2]*Otkr[11][0])+(
      Cik[12][1][2]*Otkr[11][1])))+((pin[12][2]*udotin[12])+((Wik[12][1]*
      wk[12][0])-(Wik[12][0]*wk[12][1]))));
    Otkr[13][0] = (((Cik[13][2][0]*Otkr[12][2])+((Cik[13][0][0]*Otkr[12][0])+(
      Cik[13][1][0]*Otkr[12][1])))+((pin[13][0]*udotin[13])+((Wik[13][2]*
      wk[13][1])-(Wik[13][1]*wk[13][2]))));
    Otkr[13][1] = (((Cik[13][2][1]*Otkr[12][2])+((Cik[13][0][1]*Otkr[12][0])+(
      Cik[13][1][1]*Otkr[12][1])))+((pin[13][1]*udotin[13])+((Wik[13][0]*
      wk[13][2])-(Wik[13][2]*wk[13][0]))));
    Otkr[13][2] = (((Cik[13][2][2]*Otkr[12][2])+((Cik[13][0][2]*Otkr[12][0])+(
      Cik[13][1][2]*Otkr[12][1])))+((pin[13][2]*udotin[13])+((Wik[13][1]*
      wk[13][0])-(Wik[13][0]*wk[13][1]))));
    Otkr[14][0] = (((Cik[14][2][0]*Otkr[13][2])+((Cik[14][0][0]*Otkr[13][0])+(
      Cik[14][1][0]*Otkr[13][1])))+((pin[14][0]*udotin[14])+((Wik[14][2]*
      wk[14][1])-(Wik[14][1]*wk[14][2]))));
    Otkr[14][1] = (((Cik[14][2][1]*Otkr[13][2])+((Cik[14][0][1]*Otkr[13][0])+(
      Cik[14][1][1]*Otkr[13][1])))+((pin[14][1]*udotin[14])+((Wik[14][0]*
      wk[14][2])-(Wik[14][2]*wk[14][0]))));
    Otkr[14][2] = (((Cik[14][2][2]*Otkr[13][2])+((Cik[14][0][2]*Otkr[13][0])+(
      Cik[14][1][2]*Otkr[13][1])))+((pin[14][2]*udotin[14])+((Wik[14][1]*
      wk[14][0])-(Wik[14][0]*wk[14][1]))));
    Otkr[15][0] = (((Cik[15][2][0]*udotin[5])+((Cik[15][0][0]*udotin[3])+(
      Cik[15][1][0]*udotin[4])))+((pin[15][0]*udotin[15])+((Wik[15][2]*wk[15][1]
      )-(Wik[15][1]*wk[15][2]))));
    Otkr[15][1] = (((Cik[15][2][1]*udotin[5])+((Cik[15][0][1]*udotin[3])+(
      Cik[15][1][1]*udotin[4])))+((pin[15][1]*udotin[15])+((Wik[15][0]*wk[15][2]
      )-(Wik[15][2]*wk[15][0]))));
    Otkr[15][2] = (((Cik[15][2][2]*udotin[5])+((Cik[15][0][2]*udotin[3])+(
      Cik[15][1][2]*udotin[4])))+((pin[15][2]*udotin[15])+((Wik[15][1]*wk[15][0]
      )-(Wik[15][0]*wk[15][1]))));
    Otkr[16][0] = (((Cik[16][2][0]*Otkr[15][2])+((Cik[16][0][0]*Otkr[15][0])+(
      Cik[16][1][0]*Otkr[15][1])))+((pin[16][0]*udotin[16])+((Wik[16][2]*
      wk[16][1])-(Wik[16][1]*wk[16][2]))));
    Otkr[16][1] = (((Cik[16][2][1]*Otkr[15][2])+((Cik[16][0][1]*Otkr[15][0])+(
      Cik[16][1][1]*Otkr[15][1])))+((pin[16][1]*udotin[16])+((Wik[16][0]*
      wk[16][2])-(Wik[16][2]*wk[16][0]))));
    Otkr[16][2] = (((Cik[16][2][2]*Otkr[15][2])+((Cik[16][0][2]*Otkr[15][0])+(
      Cik[16][1][2]*Otkr[15][1])))+((pin[16][2]*udotin[16])+((Wik[16][1]*
      wk[16][0])-(Wik[16][0]*wk[16][1]))));
    Otkr[17][0] = (((Cik[17][2][0]*Otkr[16][2])+((Cik[17][0][0]*Otkr[16][0])+(
      Cik[17][1][0]*Otkr[16][1])))+((pin[17][0]*udotin[17])+((Wik[17][2]*
      wk[17][1])-(Wik[17][1]*wk[17][2]))));
    Otkr[17][1] = (((Cik[17][2][1]*Otkr[16][2])+((Cik[17][0][1]*Otkr[16][0])+(
      Cik[17][1][1]*Otkr[16][1])))+((pin[17][1]*udotin[17])+((Wik[17][0]*
      wk[17][2])-(Wik[17][2]*wk[17][0]))));
    Otkr[17][2] = (((Cik[17][2][2]*Otkr[16][2])+((Cik[17][0][2]*Otkr[16][0])+(
      Cik[17][1][2]*Otkr[16][1])))+((pin[17][2]*udotin[17])+((Wik[17][1]*
      wk[17][0])-(Wik[17][0]*wk[17][1]))));
    Otkr[18][0] = (((Cik[18][2][0]*Otkr[17][2])+((Cik[18][0][0]*Otkr[17][0])+(
      Cik[18][1][0]*Otkr[17][1])))+((pin[18][0]*udotin[18])+((Wik[18][2]*
      wk[18][1])-(Wik[18][1]*wk[18][2]))));
    Otkr[18][1] = (((Cik[18][2][1]*Otkr[17][2])+((Cik[18][0][1]*Otkr[17][0])+(
      Cik[18][1][1]*Otkr[17][1])))+((pin[18][1]*udotin[18])+((Wik[18][0]*
      wk[18][2])-(Wik[18][2]*wk[18][0]))));
    Otkr[18][2] = (((Cik[18][2][2]*Otkr[17][2])+((Cik[18][0][2]*Otkr[17][0])+(
      Cik[18][1][2]*Otkr[17][1])))+((pin[18][2]*udotin[18])+((Wik[18][1]*
      wk[18][0])-(Wik[18][0]*wk[18][1]))));
    Otkr[19][0] = (((Cik[19][2][0]*Otkr[18][2])+((Cik[19][0][0]*Otkr[18][0])+(
      Cik[19][1][0]*Otkr[18][1])))+((pin[19][0]*udotin[19])+((Wik[19][2]*
      wk[19][1])-(Wik[19][1]*wk[19][2]))));
    Otkr[19][1] = (((Cik[19][2][1]*Otkr[18][2])+((Cik[19][0][1]*Otkr[18][0])+(
      Cik[19][1][1]*Otkr[18][1])))+((pin[19][1]*udotin[19])+((Wik[19][0]*
      wk[19][2])-(Wik[19][2]*wk[19][0]))));
    Otkr[19][2] = (((Cik[19][2][2]*Otkr[18][2])+((Cik[19][0][2]*Otkr[18][0])+(
      Cik[19][1][2]*Otkr[18][1])))+((pin[19][2]*udotin[19])+((Wik[19][1]*
      wk[19][0])-(Wik[19][0]*wk[19][1]))));
    Otkr[20][0] = (((Cik[20][2][0]*Otkr[19][2])+((Cik[20][0][0]*Otkr[19][0])+(
      Cik[20][1][0]*Otkr[19][1])))+((pin[20][0]*udotin[20])+((Wik[20][2]*
      wk[20][1])-(Wik[20][1]*wk[20][2]))));
    Otkr[20][1] = (((Cik[20][2][1]*Otkr[19][2])+((Cik[20][0][1]*Otkr[19][0])+(
      Cik[20][1][1]*Otkr[19][1])))+((pin[20][1]*udotin[20])+((Wik[20][0]*
      wk[20][2])-(Wik[20][2]*wk[20][0]))));
    Otkr[20][2] = (((Cik[20][2][2]*Otkr[19][2])+((Cik[20][0][2]*Otkr[19][0])+(
      Cik[20][1][2]*Otkr[19][1])))+((pin[20][2]*udotin[20])+((Wik[20][1]*
      wk[20][0])-(Wik[20][0]*wk[20][1]))));
    Atkr[0][0] = (pin[0][0]*udotin[0]);
    Atkr[0][1] = (pin[0][1]*udotin[0]);
    Atkr[0][2] = (pin[0][2]*udotin[0]);
    Atkr[1][0] = (Atkr[0][0]+(pin[1][0]*udotin[1]));
    Atkr[1][1] = (Atkr[0][1]+(pin[1][1]*udotin[1]));
    Atkr[1][2] = (Atkr[0][2]+(pin[1][2]*udotin[1]));
    Atkr[2][0] = (Atkr[1][0]+(pin[2][0]*udotin[2]));
    Atkr[2][1] = (Atkr[1][1]+(pin[2][1]*udotin[2]));
    Atkr[2][2] = (Atkr[1][2]+(pin[2][2]*udotin[2]));
    Atkr[3][0] = ((Atkr[2][2]*Cik[3][2][0])+((Atkr[2][0]*Cik[3][0][0])+(
      Atkr[2][1]*Cik[3][1][0])));
    Atkr[3][1] = ((Atkr[2][2]*Cik[3][2][1])+((Atkr[2][0]*Cik[3][0][1])+(
      Atkr[2][1]*Cik[3][1][1])));
    Atkr[3][2] = ((Atkr[2][2]*Cik[3][2][2])+((Atkr[2][0]*Cik[3][0][2])+(
      Atkr[2][1]*Cik[3][1][2])));
    Atkr[5][0] = (Atkr[3][0]+(((rk[0][1]*udotin[5])-(rk[0][2]*udotin[4]))+((u[4]
      *Wkrpk[5][2])-(u[5]*Wkrpk[5][1]))));
    Atkr[5][1] = (Atkr[3][1]+(((rk[0][2]*udotin[3])-(rk[0][0]*udotin[5]))+((u[5]
      *Wkrpk[5][0])-(u[3]*Wkrpk[5][2]))));
    Atkr[5][2] = (Atkr[3][2]+(((rk[0][0]*udotin[4])-(rk[0][1]*udotin[3]))+((u[3]
      *Wkrpk[5][1])-(u[4]*Wkrpk[5][0]))));
    Atir[6][0] = (Atkr[5][0]+(((ri[1][2]*udotin[4])-(ri[1][1]*udotin[5]))+((u[4]
      *Wirk[6][2])-(u[5]*Wirk[6][1]))));
    Atir[6][1] = (Atkr[5][1]+(((ri[1][0]*udotin[5])-(ri[1][2]*udotin[3]))+((u[5]
      *Wirk[6][0])-(u[3]*Wirk[6][2]))));
    Atir[6][2] = (Atkr[5][2]+(((ri[1][1]*udotin[3])-(ri[1][0]*udotin[4]))+((u[3]
      *Wirk[6][1])-(u[4]*Wirk[6][0]))));
    Atkr[6][0] = (((Atir[6][2]*Cik[6][2][0])+((Atir[6][0]*Cik[6][0][0])+(
      Atir[6][1]*Cik[6][1][0])))+(((Otkr[6][2]*rk[1][1])-(Otkr[6][1]*rk[1][2]))+
      ((wk[6][1]*Wkrpk[6][2])-(wk[6][2]*Wkrpk[6][1]))));
    Atkr[6][1] = (((Atir[6][2]*Cik[6][2][1])+((Atir[6][0]*Cik[6][0][1])+(
      Atir[6][1]*Cik[6][1][1])))+(((Otkr[6][0]*rk[1][2])-(Otkr[6][2]*rk[1][0]))+
      ((wk[6][2]*Wkrpk[6][0])-(wk[6][0]*Wkrpk[6][2]))));
    Atkr[6][2] = (((Atir[6][2]*Cik[6][2][2])+((Atir[6][0]*Cik[6][0][2])+(
      Atir[6][1]*Cik[6][1][2])))+(((Otkr[6][1]*rk[1][0])-(Otkr[6][0]*rk[1][1]))+
      ((wk[6][0]*Wkrpk[6][1])-(wk[6][1]*Wkrpk[6][0]))));
    Atir[7][0] = (Atkr[6][0]+(((Otkr[6][1]*ri[2][2])-(Otkr[6][2]*ri[2][1]))+((
      Wirk[7][2]*wk[6][1])-(Wirk[7][1]*wk[6][2]))));
    Atir[7][1] = (Atkr[6][1]+(((Otkr[6][2]*ri[2][0])-(Otkr[6][0]*ri[2][2]))+((
      Wirk[7][0]*wk[6][2])-(Wirk[7][2]*wk[6][0]))));
    Atir[7][2] = (Atkr[6][2]+(((Otkr[6][0]*ri[2][1])-(Otkr[6][1]*ri[2][0]))+((
      Wirk[7][1]*wk[6][0])-(Wirk[7][0]*wk[6][1]))));
    Atkr[7][0] = (((Atir[7][2]*Cik[7][2][0])+((Atir[7][0]*Cik[7][0][0])+(
      Atir[7][1]*Cik[7][1][0])))+(((Otkr[7][2]*rk[2][1])-(Otkr[7][1]*rk[2][2]))+
      ((wk[7][1]*Wkrpk[7][2])-(wk[7][2]*Wkrpk[7][1]))));
    Atkr[7][1] = (((Atir[7][2]*Cik[7][2][1])+((Atir[7][0]*Cik[7][0][1])+(
      Atir[7][1]*Cik[7][1][1])))+(((Otkr[7][0]*rk[2][2])-(Otkr[7][2]*rk[2][0]))+
      ((wk[7][2]*Wkrpk[7][0])-(wk[7][0]*Wkrpk[7][2]))));
    Atkr[7][2] = (((Atir[7][2]*Cik[7][2][2])+((Atir[7][0]*Cik[7][0][2])+(
      Atir[7][1]*Cik[7][1][2])))+(((Otkr[7][1]*rk[2][0])-(Otkr[7][0]*rk[2][1]))+
      ((wk[7][0]*Wkrpk[7][1])-(wk[7][1]*Wkrpk[7][0]))));
    Atir[8][0] = (Atkr[7][0]+(((Otkr[7][1]*ri[3][2])-(Otkr[7][2]*ri[3][1]))+((
      Wirk[8][2]*wk[7][1])-(Wirk[8][1]*wk[7][2]))));
    Atir[8][1] = (Atkr[7][1]+(((Otkr[7][2]*ri[3][0])-(Otkr[7][0]*ri[3][2]))+((
      Wirk[8][0]*wk[7][2])-(Wirk[8][2]*wk[7][0]))));
    Atir[8][2] = (Atkr[7][2]+(((Otkr[7][0]*ri[3][1])-(Otkr[7][1]*ri[3][0]))+((
      Wirk[8][1]*wk[7][0])-(Wirk[8][0]*wk[7][1]))));
    Atkr[8][0] = (((Atir[8][2]*Cik[8][2][0])+((Atir[8][0]*Cik[8][0][0])+(
      Atir[8][1]*Cik[8][1][0])))+(((Otkr[8][2]*rk[3][1])-(Otkr[8][1]*rk[3][2]))+
      ((wk[8][1]*Wkrpk[8][2])-(wk[8][2]*Wkrpk[8][1]))));
    Atkr[8][1] = (((Atir[8][2]*Cik[8][2][1])+((Atir[8][0]*Cik[8][0][1])+(
      Atir[8][1]*Cik[8][1][1])))+(((Otkr[8][0]*rk[3][2])-(Otkr[8][2]*rk[3][0]))+
      ((wk[8][2]*Wkrpk[8][0])-(wk[8][0]*Wkrpk[8][2]))));
    Atkr[8][2] = (((Atir[8][2]*Cik[8][2][2])+((Atir[8][0]*Cik[8][0][2])+(
      Atir[8][1]*Cik[8][1][2])))+(((Otkr[8][1]*rk[3][0])-(Otkr[8][0]*rk[3][1]))+
      ((wk[8][0]*Wkrpk[8][1])-(wk[8][1]*Wkrpk[8][0]))));
    Atir[9][0] = (Atkr[5][0]+(((ri[4][2]*udotin[4])-(ri[4][1]*udotin[5]))+((u[4]
      *Wirk[9][2])-(u[5]*Wirk[9][1]))));
    Atir[9][1] = (Atkr[5][1]+(((ri[4][0]*udotin[5])-(ri[4][2]*udotin[3]))+((u[5]
      *Wirk[9][0])-(u[3]*Wirk[9][2]))));
    Atir[9][2] = (Atkr[5][2]+(((ri[4][1]*udotin[3])-(ri[4][0]*udotin[4]))+((u[3]
      *Wirk[9][1])-(u[4]*Wirk[9][0]))));
    Atkr[9][0] = (((Atir[9][2]*Cik[9][2][0])+((Atir[9][0]*Cik[9][0][0])+(
      Atir[9][1]*Cik[9][1][0])))+(((Otkr[9][2]*rk[4][1])-(Otkr[9][1]*rk[4][2]))+
      ((wk[9][1]*Wkrpk[9][2])-(wk[9][2]*Wkrpk[9][1]))));
    Atkr[9][1] = (((Atir[9][2]*Cik[9][2][1])+((Atir[9][0]*Cik[9][0][1])+(
      Atir[9][1]*Cik[9][1][1])))+(((Otkr[9][0]*rk[4][2])-(Otkr[9][2]*rk[4][0]))+
      ((wk[9][2]*Wkrpk[9][0])-(wk[9][0]*Wkrpk[9][2]))));
    Atkr[9][2] = (((Atir[9][2]*Cik[9][2][2])+((Atir[9][0]*Cik[9][0][2])+(
      Atir[9][1]*Cik[9][1][2])))+(((Otkr[9][1]*rk[4][0])-(Otkr[9][0]*rk[4][1]))+
      ((wk[9][0]*Wkrpk[9][1])-(wk[9][1]*Wkrpk[9][0]))));
    Atir[10][0] = (Atkr[9][0]+(((Otkr[9][1]*ri[5][2])-(Otkr[9][2]*ri[5][1]))+((
      Wirk[10][2]*wk[9][1])-(Wirk[10][1]*wk[9][2]))));
    Atir[10][1] = (Atkr[9][1]+(((Otkr[9][2]*ri[5][0])-(Otkr[9][0]*ri[5][2]))+((
      Wirk[10][0]*wk[9][2])-(Wirk[10][2]*wk[9][0]))));
    Atir[10][2] = (Atkr[9][2]+(((Otkr[9][0]*ri[5][1])-(Otkr[9][1]*ri[5][0]))+((
      Wirk[10][1]*wk[9][0])-(Wirk[10][0]*wk[9][1]))));
    Atkr[10][0] = (((Atir[10][2]*Cik[10][2][0])+((Atir[10][0]*Cik[10][0][0])+(
      Atir[10][1]*Cik[10][1][0])))+(((Otkr[10][2]*rk[5][1])-(Otkr[10][1]*
      rk[5][2]))+((wk[10][1]*Wkrpk[10][2])-(wk[10][2]*Wkrpk[10][1]))));
    Atkr[10][1] = (((Atir[10][2]*Cik[10][2][1])+((Atir[10][0]*Cik[10][0][1])+(
      Atir[10][1]*Cik[10][1][1])))+(((Otkr[10][0]*rk[5][2])-(Otkr[10][2]*
      rk[5][0]))+((wk[10][2]*Wkrpk[10][0])-(wk[10][0]*Wkrpk[10][2]))));
    Atkr[10][2] = (((Atir[10][2]*Cik[10][2][2])+((Atir[10][0]*Cik[10][0][2])+(
      Atir[10][1]*Cik[10][1][2])))+(((Otkr[10][1]*rk[5][0])-(Otkr[10][0]*
      rk[5][1]))+((wk[10][0]*Wkrpk[10][1])-(wk[10][1]*Wkrpk[10][0]))));
    Atir[11][0] = (Atkr[10][0]+(((Otkr[10][1]*ri[6][2])-(Otkr[10][2]*ri[6][1]))+
      ((Wirk[11][2]*wk[10][1])-(Wirk[11][1]*wk[10][2]))));
    Atir[11][1] = (Atkr[10][1]+(((Otkr[10][2]*ri[6][0])-(Otkr[10][0]*ri[6][2]))+
      ((Wirk[11][0]*wk[10][2])-(Wirk[11][2]*wk[10][0]))));
    Atir[11][2] = (Atkr[10][2]+(((Otkr[10][0]*ri[6][1])-(Otkr[10][1]*ri[6][0]))+
      ((Wirk[11][1]*wk[10][0])-(Wirk[11][0]*wk[10][1]))));
    Atkr[11][0] = (((Atir[11][2]*Cik[11][2][0])+((Atir[11][0]*Cik[11][0][0])+(
      Atir[11][1]*Cik[11][1][0])))+(((Otkr[11][2]*rk[6][1])-(Otkr[11][1]*
      rk[6][2]))+((wk[11][1]*Wkrpk[11][2])-(wk[11][2]*Wkrpk[11][1]))));
    Atkr[11][1] = (((Atir[11][2]*Cik[11][2][1])+((Atir[11][0]*Cik[11][0][1])+(
      Atir[11][1]*Cik[11][1][1])))+(((Otkr[11][0]*rk[6][2])-(Otkr[11][2]*
      rk[6][0]))+((wk[11][2]*Wkrpk[11][0])-(wk[11][0]*Wkrpk[11][2]))));
    Atkr[11][2] = (((Atir[11][2]*Cik[11][2][2])+((Atir[11][0]*Cik[11][0][2])+(
      Atir[11][1]*Cik[11][1][2])))+(((Otkr[11][1]*rk[6][0])-(Otkr[11][0]*
      rk[6][1]))+((wk[11][0]*Wkrpk[11][1])-(wk[11][1]*Wkrpk[11][0]))));
    Atir[12][0] = (Atkr[11][0]+(((Otkr[11][1]*ri[7][2])-(Otkr[11][2]*ri[7][1]))+
      ((Wirk[12][2]*wk[11][1])-(Wirk[12][1]*wk[11][2]))));
    Atir[12][1] = (Atkr[11][1]+(((Otkr[11][2]*ri[7][0])-(Otkr[11][0]*ri[7][2]))+
      ((Wirk[12][0]*wk[11][2])-(Wirk[12][2]*wk[11][0]))));
    Atir[12][2] = (Atkr[11][2]+(((Otkr[11][0]*ri[7][1])-(Otkr[11][1]*ri[7][0]))+
      ((Wirk[12][1]*wk[11][0])-(Wirk[12][0]*wk[11][1]))));
    Atkr[12][0] = (((Atir[12][2]*Cik[12][2][0])+((Atir[12][0]*Cik[12][0][0])+(
      Atir[12][1]*Cik[12][1][0])))+(((Otkr[12][2]*rk[7][1])-(Otkr[12][1]*
      rk[7][2]))+((wk[12][1]*Wkrpk[12][2])-(wk[12][2]*Wkrpk[12][1]))));
    Atkr[12][1] = (((Atir[12][2]*Cik[12][2][1])+((Atir[12][0]*Cik[12][0][1])+(
      Atir[12][1]*Cik[12][1][1])))+(((Otkr[12][0]*rk[7][2])-(Otkr[12][2]*
      rk[7][0]))+((wk[12][2]*Wkrpk[12][0])-(wk[12][0]*Wkrpk[12][2]))));
    Atkr[12][2] = (((Atir[12][2]*Cik[12][2][2])+((Atir[12][0]*Cik[12][0][2])+(
      Atir[12][1]*Cik[12][1][2])))+(((Otkr[12][1]*rk[7][0])-(Otkr[12][0]*
      rk[7][1]))+((wk[12][0]*Wkrpk[12][1])-(wk[12][1]*Wkrpk[12][0]))));
    Atir[13][0] = (Atkr[12][0]+(((Otkr[12][1]*ri[8][2])-(Otkr[12][2]*ri[8][1]))+
      ((Wirk[13][2]*wk[12][1])-(Wirk[13][1]*wk[12][2]))));
    Atir[13][1] = (Atkr[12][1]+(((Otkr[12][2]*ri[8][0])-(Otkr[12][0]*ri[8][2]))+
      ((Wirk[13][0]*wk[12][2])-(Wirk[13][2]*wk[12][0]))));
    Atir[13][2] = (Atkr[12][2]+(((Otkr[12][0]*ri[8][1])-(Otkr[12][1]*ri[8][0]))+
      ((Wirk[13][1]*wk[12][0])-(Wirk[13][0]*wk[12][1]))));
    Atkr[13][0] = (((Atir[13][2]*Cik[13][2][0])+((Atir[13][0]*Cik[13][0][0])+(
      Atir[13][1]*Cik[13][1][0])))+(((Otkr[13][2]*rk[8][1])-(Otkr[13][1]*
      rk[8][2]))+((wk[13][1]*Wkrpk[13][2])-(wk[13][2]*Wkrpk[13][1]))));
    Atkr[13][1] = (((Atir[13][2]*Cik[13][2][1])+((Atir[13][0]*Cik[13][0][1])+(
      Atir[13][1]*Cik[13][1][1])))+(((Otkr[13][0]*rk[8][2])-(Otkr[13][2]*
      rk[8][0]))+((wk[13][2]*Wkrpk[13][0])-(wk[13][0]*Wkrpk[13][2]))));
    Atkr[13][2] = (((Atir[13][2]*Cik[13][2][2])+((Atir[13][0]*Cik[13][0][2])+(
      Atir[13][1]*Cik[13][1][2])))+(((Otkr[13][1]*rk[8][0])-(Otkr[13][0]*
      rk[8][1]))+((wk[13][0]*Wkrpk[13][1])-(wk[13][1]*Wkrpk[13][0]))));
    Atir[14][0] = (Atkr[13][0]+(((Otkr[13][1]*ri[9][2])-(Otkr[13][2]*ri[9][1]))+
      ((Wirk[14][2]*wk[13][1])-(Wirk[14][1]*wk[13][2]))));
    Atir[14][1] = (Atkr[13][1]+(((Otkr[13][2]*ri[9][0])-(Otkr[13][0]*ri[9][2]))+
      ((Wirk[14][0]*wk[13][2])-(Wirk[14][2]*wk[13][0]))));
    Atir[14][2] = (Atkr[13][2]+(((Otkr[13][0]*ri[9][1])-(Otkr[13][1]*ri[9][0]))+
      ((Wirk[14][1]*wk[13][0])-(Wirk[14][0]*wk[13][1]))));
    Atkr[14][0] = (((Atir[14][2]*Cik[14][2][0])+((Atir[14][0]*Cik[14][0][0])+(
      Atir[14][1]*Cik[14][1][0])))+(((Otkr[14][2]*rk[9][1])-(Otkr[14][1]*
      rk[9][2]))+((wk[14][1]*Wkrpk[14][2])-(wk[14][2]*Wkrpk[14][1]))));
    Atkr[14][1] = (((Atir[14][2]*Cik[14][2][1])+((Atir[14][0]*Cik[14][0][1])+(
      Atir[14][1]*Cik[14][1][1])))+(((Otkr[14][0]*rk[9][2])-(Otkr[14][2]*
      rk[9][0]))+((wk[14][2]*Wkrpk[14][0])-(wk[14][0]*Wkrpk[14][2]))));
    Atkr[14][2] = (((Atir[14][2]*Cik[14][2][2])+((Atir[14][0]*Cik[14][0][2])+(
      Atir[14][1]*Cik[14][1][2])))+(((Otkr[14][1]*rk[9][0])-(Otkr[14][0]*
      rk[9][1]))+((wk[14][0]*Wkrpk[14][1])-(wk[14][1]*Wkrpk[14][0]))));
    Atir[15][0] = (Atkr[5][0]+(((ri[10][2]*udotin[4])-(ri[10][1]*udotin[5]))+((
      u[4]*Wirk[15][2])-(u[5]*Wirk[15][1]))));
    Atir[15][1] = (Atkr[5][1]+(((ri[10][0]*udotin[5])-(ri[10][2]*udotin[3]))+((
      u[5]*Wirk[15][0])-(u[3]*Wirk[15][2]))));
    Atir[15][2] = (Atkr[5][2]+(((ri[10][1]*udotin[3])-(ri[10][0]*udotin[4]))+((
      u[3]*Wirk[15][1])-(u[4]*Wirk[15][0]))));
    Atkr[15][0] = (((Atir[15][2]*Cik[15][2][0])+((Atir[15][0]*Cik[15][0][0])+(
      Atir[15][1]*Cik[15][1][0])))+(((Otkr[15][2]*rk[10][1])-(Otkr[15][1]*
      rk[10][2]))+((wk[15][1]*Wkrpk[15][2])-(wk[15][2]*Wkrpk[15][1]))));
    Atkr[15][1] = (((Atir[15][2]*Cik[15][2][1])+((Atir[15][0]*Cik[15][0][1])+(
      Atir[15][1]*Cik[15][1][1])))+(((Otkr[15][0]*rk[10][2])-(Otkr[15][2]*
      rk[10][0]))+((wk[15][2]*Wkrpk[15][0])-(wk[15][0]*Wkrpk[15][2]))));
    Atkr[15][2] = (((Atir[15][2]*Cik[15][2][2])+((Atir[15][0]*Cik[15][0][2])+(
      Atir[15][1]*Cik[15][1][2])))+(((Otkr[15][1]*rk[10][0])-(Otkr[15][0]*
      rk[10][1]))+((wk[15][0]*Wkrpk[15][1])-(wk[15][1]*Wkrpk[15][0]))));
    Atir[16][0] = (Atkr[15][0]+(((Otkr[15][1]*ri[11][2])-(Otkr[15][2]*ri[11][1])
      )+((Wirk[16][2]*wk[15][1])-(Wirk[16][1]*wk[15][2]))));
    Atir[16][1] = (Atkr[15][1]+(((Otkr[15][2]*ri[11][0])-(Otkr[15][0]*ri[11][2])
      )+((Wirk[16][0]*wk[15][2])-(Wirk[16][2]*wk[15][0]))));
    Atir[16][2] = (Atkr[15][2]+(((Otkr[15][0]*ri[11][1])-(Otkr[15][1]*ri[11][0])
      )+((Wirk[16][1]*wk[15][0])-(Wirk[16][0]*wk[15][1]))));
    Atkr[16][0] = (((Atir[16][2]*Cik[16][2][0])+((Atir[16][0]*Cik[16][0][0])+(
      Atir[16][1]*Cik[16][1][0])))+(((Otkr[16][2]*rk[11][1])-(Otkr[16][1]*
      rk[11][2]))+((wk[16][1]*Wkrpk[16][2])-(wk[16][2]*Wkrpk[16][1]))));
    Atkr[16][1] = (((Atir[16][2]*Cik[16][2][1])+((Atir[16][0]*Cik[16][0][1])+(
      Atir[16][1]*Cik[16][1][1])))+(((Otkr[16][0]*rk[11][2])-(Otkr[16][2]*
      rk[11][0]))+((wk[16][2]*Wkrpk[16][0])-(wk[16][0]*Wkrpk[16][2]))));
    Atkr[16][2] = (((Atir[16][2]*Cik[16][2][2])+((Atir[16][0]*Cik[16][0][2])+(
      Atir[16][1]*Cik[16][1][2])))+(((Otkr[16][1]*rk[11][0])-(Otkr[16][0]*
      rk[11][1]))+((wk[16][0]*Wkrpk[16][1])-(wk[16][1]*Wkrpk[16][0]))));
    Atir[17][0] = (Atkr[16][0]+(((Otkr[16][1]*ri[12][2])-(Otkr[16][2]*ri[12][1])
      )+((Wirk[17][2]*wk[16][1])-(Wirk[17][1]*wk[16][2]))));
    Atir[17][1] = (Atkr[16][1]+(((Otkr[16][2]*ri[12][0])-(Otkr[16][0]*ri[12][2])
      )+((Wirk[17][0]*wk[16][2])-(Wirk[17][2]*wk[16][0]))));
    Atir[17][2] = (Atkr[16][2]+(((Otkr[16][0]*ri[12][1])-(Otkr[16][1]*ri[12][0])
      )+((Wirk[17][1]*wk[16][0])-(Wirk[17][0]*wk[16][1]))));
    Atkr[17][0] = (((Atir[17][2]*Cik[17][2][0])+((Atir[17][0]*Cik[17][0][0])+(
      Atir[17][1]*Cik[17][1][0])))+(((Otkr[17][2]*rk[12][1])-(Otkr[17][1]*
      rk[12][2]))+((wk[17][1]*Wkrpk[17][2])-(wk[17][2]*Wkrpk[17][1]))));
    Atkr[17][1] = (((Atir[17][2]*Cik[17][2][1])+((Atir[17][0]*Cik[17][0][1])+(
      Atir[17][1]*Cik[17][1][1])))+(((Otkr[17][0]*rk[12][2])-(Otkr[17][2]*
      rk[12][0]))+((wk[17][2]*Wkrpk[17][0])-(wk[17][0]*Wkrpk[17][2]))));
    Atkr[17][2] = (((Atir[17][2]*Cik[17][2][2])+((Atir[17][0]*Cik[17][0][2])+(
      Atir[17][1]*Cik[17][1][2])))+(((Otkr[17][1]*rk[12][0])-(Otkr[17][0]*
      rk[12][1]))+((wk[17][0]*Wkrpk[17][1])-(wk[17][1]*Wkrpk[17][0]))));
    Atir[18][0] = (Atkr[17][0]+(((Otkr[17][1]*ri[13][2])-(Otkr[17][2]*ri[13][1])
      )+((Wirk[18][2]*wk[17][1])-(Wirk[18][1]*wk[17][2]))));
    Atir[18][1] = (Atkr[17][1]+(((Otkr[17][2]*ri[13][0])-(Otkr[17][0]*ri[13][2])
      )+((Wirk[18][0]*wk[17][2])-(Wirk[18][2]*wk[17][0]))));
    Atir[18][2] = (Atkr[17][2]+(((Otkr[17][0]*ri[13][1])-(Otkr[17][1]*ri[13][0])
      )+((Wirk[18][1]*wk[17][0])-(Wirk[18][0]*wk[17][1]))));
    Atkr[18][0] = (((Atir[18][2]*Cik[18][2][0])+((Atir[18][0]*Cik[18][0][0])+(
      Atir[18][1]*Cik[18][1][0])))+(((Otkr[18][2]*rk[13][1])-(Otkr[18][1]*
      rk[13][2]))+((wk[18][1]*Wkrpk[18][2])-(wk[18][2]*Wkrpk[18][1]))));
    Atkr[18][1] = (((Atir[18][2]*Cik[18][2][1])+((Atir[18][0]*Cik[18][0][1])+(
      Atir[18][1]*Cik[18][1][1])))+(((Otkr[18][0]*rk[13][2])-(Otkr[18][2]*
      rk[13][0]))+((wk[18][2]*Wkrpk[18][0])-(wk[18][0]*Wkrpk[18][2]))));
    Atkr[18][2] = (((Atir[18][2]*Cik[18][2][2])+((Atir[18][0]*Cik[18][0][2])+(
      Atir[18][1]*Cik[18][1][2])))+(((Otkr[18][1]*rk[13][0])-(Otkr[18][0]*
      rk[13][1]))+((wk[18][0]*Wkrpk[18][1])-(wk[18][1]*Wkrpk[18][0]))));
    Atir[19][0] = (Atkr[18][0]+(((Otkr[18][1]*ri[14][2])-(Otkr[18][2]*ri[14][1])
      )+((Wirk[19][2]*wk[18][1])-(Wirk[19][1]*wk[18][2]))));
    Atir[19][1] = (Atkr[18][1]+(((Otkr[18][2]*ri[14][0])-(Otkr[18][0]*ri[14][2])
      )+((Wirk[19][0]*wk[18][2])-(Wirk[19][2]*wk[18][0]))));
    Atir[19][2] = (Atkr[18][2]+(((Otkr[18][0]*ri[14][1])-(Otkr[18][1]*ri[14][0])
      )+((Wirk[19][1]*wk[18][0])-(Wirk[19][0]*wk[18][1]))));
    Atkr[19][0] = (((Atir[19][2]*Cik[19][2][0])+((Atir[19][0]*Cik[19][0][0])+(
      Atir[19][1]*Cik[19][1][0])))+(((Otkr[19][2]*rk[14][1])-(Otkr[19][1]*
      rk[14][2]))+((wk[19][1]*Wkrpk[19][2])-(wk[19][2]*Wkrpk[19][1]))));
    Atkr[19][1] = (((Atir[19][2]*Cik[19][2][1])+((Atir[19][0]*Cik[19][0][1])+(
      Atir[19][1]*Cik[19][1][1])))+(((Otkr[19][0]*rk[14][2])-(Otkr[19][2]*
      rk[14][0]))+((wk[19][2]*Wkrpk[19][0])-(wk[19][0]*Wkrpk[19][2]))));
    Atkr[19][2] = (((Atir[19][2]*Cik[19][2][2])+((Atir[19][0]*Cik[19][0][2])+(
      Atir[19][1]*Cik[19][1][2])))+(((Otkr[19][1]*rk[14][0])-(Otkr[19][0]*
      rk[14][1]))+((wk[19][0]*Wkrpk[19][1])-(wk[19][1]*Wkrpk[19][0]))));
    Atir[20][0] = (Atkr[19][0]+(((Otkr[19][1]*ri[15][2])-(Otkr[19][2]*ri[15][1])
      )+((Wirk[20][2]*wk[19][1])-(Wirk[20][1]*wk[19][2]))));
    Atir[20][1] = (Atkr[19][1]+(((Otkr[19][2]*ri[15][0])-(Otkr[19][0]*ri[15][2])
      )+((Wirk[20][0]*wk[19][2])-(Wirk[20][2]*wk[19][0]))));
    Atir[20][2] = (Atkr[19][2]+(((Otkr[19][0]*ri[15][1])-(Otkr[19][1]*ri[15][0])
      )+((Wirk[20][1]*wk[19][0])-(Wirk[20][0]*wk[19][1]))));
    Atkr[20][0] = (((Atir[20][2]*Cik[20][2][0])+((Atir[20][0]*Cik[20][0][0])+(
      Atir[20][1]*Cik[20][1][0])))+(((Otkr[20][2]*rk[15][1])-(Otkr[20][1]*
      rk[15][2]))+((wk[20][1]*Wkrpk[20][2])-(wk[20][2]*Wkrpk[20][1]))));
    Atkr[20][1] = (((Atir[20][2]*Cik[20][2][1])+((Atir[20][0]*Cik[20][0][1])+(
      Atir[20][1]*Cik[20][1][1])))+(((Otkr[20][0]*rk[15][2])-(Otkr[20][2]*
      rk[15][0]))+((wk[20][2]*Wkrpk[20][0])-(wk[20][0]*Wkrpk[20][2]))));
    Atkr[20][2] = (((Atir[20][2]*Cik[20][2][2])+((Atir[20][0]*Cik[20][0][2])+(
      Atir[20][1]*Cik[20][1][2])))+(((Otkr[20][1]*rk[15][0])-(Otkr[20][0]*
      rk[15][1]))+((wk[20][0]*Wkrpk[20][1])-(wk[20][1]*Wkrpk[20][0]))));
/*
Accumulate all forces and torques
*/
    fstarr[5][0] = (ufk[0][0]+(mk[0]*(gk[3][0]-Atkr[5][0])));
    fstarr[5][1] = (ufk[0][1]+(mk[0]*(gk[3][1]-Atkr[5][1])));
    fstarr[5][2] = (ufk[0][2]+(mk[0]*(gk[3][2]-Atkr[5][2])));
    fstarr[6][0] = (ufk[1][0]+(mk[1]*(gk[6][0]-Atkr[6][0])));
    fstarr[6][1] = (ufk[1][1]+(mk[1]*(gk[6][1]-Atkr[6][1])));
    fstarr[6][2] = (ufk[1][2]+(mk[1]*(gk[6][2]-Atkr[6][2])));
    fstarr[7][0] = (ufk[2][0]+(mk[2]*(gk[7][0]-Atkr[7][0])));
    fstarr[7][1] = (ufk[2][1]+(mk[2]*(gk[7][1]-Atkr[7][1])));
    fstarr[7][2] = (ufk[2][2]+(mk[2]*(gk[7][2]-Atkr[7][2])));
    fstarr[8][0] = (ufk[3][0]+(mk[3]*(gk[8][0]-Atkr[8][0])));
    fstarr[8][1] = (ufk[3][1]+(mk[3]*(gk[8][1]-Atkr[8][1])));
    fstarr[8][2] = (ufk[3][2]+(mk[3]*(gk[8][2]-Atkr[8][2])));
    fstarr[9][0] = (ufk[4][0]+(mk[4]*(gk[9][0]-Atkr[9][0])));
    fstarr[9][1] = (ufk[4][1]+(mk[4]*(gk[9][1]-Atkr[9][1])));
    fstarr[9][2] = (ufk[4][2]+(mk[4]*(gk[9][2]-Atkr[9][2])));
    fstarr[10][0] = (ufk[5][0]+(mk[5]*(gk[10][0]-Atkr[10][0])));
    fstarr[10][1] = (ufk[5][1]+(mk[5]*(gk[10][1]-Atkr[10][1])));
    fstarr[10][2] = (ufk[5][2]+(mk[5]*(gk[10][2]-Atkr[10][2])));
    fstarr[11][0] = (ufk[6][0]+(mk[6]*(gk[11][0]-Atkr[11][0])));
    fstarr[11][1] = (ufk[6][1]+(mk[6]*(gk[11][1]-Atkr[11][1])));
    fstarr[11][2] = (ufk[6][2]+(mk[6]*(gk[11][2]-Atkr[11][2])));
    fstarr[12][0] = (ufk[7][0]+(mk[7]*(gk[12][0]-Atkr[12][0])));
    fstarr[12][1] = (ufk[7][1]+(mk[7]*(gk[12][1]-Atkr[12][1])));
    fstarr[12][2] = (ufk[7][2]+(mk[7]*(gk[12][2]-Atkr[12][2])));
    fstarr[13][0] = (ufk[8][0]+(mk[8]*(gk[13][0]-Atkr[13][0])));
    fstarr[13][1] = (ufk[8][1]+(mk[8]*(gk[13][1]-Atkr[13][1])));
    fstarr[13][2] = (ufk[8][2]+(mk[8]*(gk[13][2]-Atkr[13][2])));
    fstarr[14][0] = (ufk[9][0]+(mk[9]*(gk[14][0]-Atkr[14][0])));
    fstarr[14][1] = (ufk[9][1]+(mk[9]*(gk[14][1]-Atkr[14][1])));
    fstarr[14][2] = (ufk[9][2]+(mk[9]*(gk[14][2]-Atkr[14][2])));
    fstarr[15][0] = (ufk[10][0]+(mk[10]*(gk[15][0]-Atkr[15][0])));
    fstarr[15][1] = (ufk[10][1]+(mk[10]*(gk[15][1]-Atkr[15][1])));
    fstarr[15][2] = (ufk[10][2]+(mk[10]*(gk[15][2]-Atkr[15][2])));
    fstarr[16][0] = (ufk[11][0]+(mk[11]*(gk[16][0]-Atkr[16][0])));
    fstarr[16][1] = (ufk[11][1]+(mk[11]*(gk[16][1]-Atkr[16][1])));
    fstarr[16][2] = (ufk[11][2]+(mk[11]*(gk[16][2]-Atkr[16][2])));
    fstarr[17][0] = (ufk[12][0]+(mk[12]*(gk[17][0]-Atkr[17][0])));
    fstarr[17][1] = (ufk[12][1]+(mk[12]*(gk[17][1]-Atkr[17][1])));
    fstarr[17][2] = (ufk[12][2]+(mk[12]*(gk[17][2]-Atkr[17][2])));
    fstarr[18][0] = (ufk[13][0]+(mk[13]*(gk[18][0]-Atkr[18][0])));
    fstarr[18][1] = (ufk[13][1]+(mk[13]*(gk[18][1]-Atkr[18][1])));
    fstarr[18][2] = (ufk[13][2]+(mk[13]*(gk[18][2]-Atkr[18][2])));
    fstarr[19][0] = (ufk[14][0]+(mk[14]*(gk[19][0]-Atkr[19][0])));
    fstarr[19][1] = (ufk[14][1]+(mk[14]*(gk[19][1]-Atkr[19][1])));
    fstarr[19][2] = (ufk[14][2]+(mk[14]*(gk[19][2]-Atkr[19][2])));
    fstarr[20][0] = (ufk[15][0]+(mk[15]*(gk[20][0]-Atkr[20][0])));
    fstarr[20][1] = (ufk[15][1]+(mk[15]*(gk[20][1]-Atkr[20][1])));
    fstarr[20][2] = (ufk[15][2]+(mk[15]*(gk[20][2]-Atkr[20][2])));
    tstarr[5][0] = (utk[0][0]-(WkIkWk[5][0]+((ik[0][0][2]*udotin[5])+((
      ik[0][0][0]*udotin[3])+(ik[0][0][1]*udotin[4])))));
    tstarr[5][1] = (utk[0][1]-(WkIkWk[5][1]+((ik[0][1][2]*udotin[5])+((
      ik[0][1][0]*udotin[3])+(ik[0][1][1]*udotin[4])))));
    tstarr[5][2] = (utk[0][2]-(WkIkWk[5][2]+((ik[0][2][2]*udotin[5])+((
      ik[0][2][0]*udotin[3])+(ik[0][2][1]*udotin[4])))));
    tstarr[6][0] = (utk[1][0]-(WkIkWk[6][0]+((ik[1][0][2]*Otkr[6][2])+((
      ik[1][0][0]*Otkr[6][0])+(ik[1][0][1]*Otkr[6][1])))));
    tstarr[6][1] = (utk[1][1]-(WkIkWk[6][1]+((ik[1][1][2]*Otkr[6][2])+((
      ik[1][1][0]*Otkr[6][0])+(ik[1][1][1]*Otkr[6][1])))));
    tstarr[6][2] = (utk[1][2]-(WkIkWk[6][2]+((ik[1][2][2]*Otkr[6][2])+((
      ik[1][2][0]*Otkr[6][0])+(ik[1][2][1]*Otkr[6][1])))));
    tstarr[7][0] = (utk[2][0]-(WkIkWk[7][0]+((ik[2][0][2]*Otkr[7][2])+((
      ik[2][0][0]*Otkr[7][0])+(ik[2][0][1]*Otkr[7][1])))));
    tstarr[7][1] = (utk[2][1]-(WkIkWk[7][1]+((ik[2][1][2]*Otkr[7][2])+((
      ik[2][1][0]*Otkr[7][0])+(ik[2][1][1]*Otkr[7][1])))));
    tstarr[7][2] = (utk[2][2]-(WkIkWk[7][2]+((ik[2][2][2]*Otkr[7][2])+((
      ik[2][2][0]*Otkr[7][0])+(ik[2][2][1]*Otkr[7][1])))));
    tstarr[8][0] = (utk[3][0]-(WkIkWk[8][0]+((ik[3][0][2]*Otkr[8][2])+((
      ik[3][0][0]*Otkr[8][0])+(ik[3][0][1]*Otkr[8][1])))));
    tstarr[8][1] = (utk[3][1]-(WkIkWk[8][1]+((ik[3][1][2]*Otkr[8][2])+((
      ik[3][1][0]*Otkr[8][0])+(ik[3][1][1]*Otkr[8][1])))));
    tstarr[8][2] = (utk[3][2]-(WkIkWk[8][2]+((ik[3][2][2]*Otkr[8][2])+((
      ik[3][2][0]*Otkr[8][0])+(ik[3][2][1]*Otkr[8][1])))));
    tstarr[9][0] = (utk[4][0]-(WkIkWk[9][0]+((ik[4][0][2]*Otkr[9][2])+((
      ik[4][0][0]*Otkr[9][0])+(ik[4][0][1]*Otkr[9][1])))));
    tstarr[9][1] = (utk[4][1]-(WkIkWk[9][1]+((ik[4][1][2]*Otkr[9][2])+((
      ik[4][1][0]*Otkr[9][0])+(ik[4][1][1]*Otkr[9][1])))));
    tstarr[9][2] = (utk[4][2]-(WkIkWk[9][2]+((ik[4][2][2]*Otkr[9][2])+((
      ik[4][2][0]*Otkr[9][0])+(ik[4][2][1]*Otkr[9][1])))));
    tstarr[10][0] = (utk[5][0]-(WkIkWk[10][0]+((ik[5][0][2]*Otkr[10][2])+((
      ik[5][0][0]*Otkr[10][0])+(ik[5][0][1]*Otkr[10][1])))));
    tstarr[10][1] = (utk[5][1]-(WkIkWk[10][1]+((ik[5][1][2]*Otkr[10][2])+((
      ik[5][1][0]*Otkr[10][0])+(ik[5][1][1]*Otkr[10][1])))));
    tstarr[10][2] = (utk[5][2]-(WkIkWk[10][2]+((ik[5][2][2]*Otkr[10][2])+((
      ik[5][2][0]*Otkr[10][0])+(ik[5][2][1]*Otkr[10][1])))));
    tstarr[11][0] = (utk[6][0]-(WkIkWk[11][0]+((ik[6][0][2]*Otkr[11][2])+((
      ik[6][0][0]*Otkr[11][0])+(ik[6][0][1]*Otkr[11][1])))));
    tstarr[11][1] = (utk[6][1]-(WkIkWk[11][1]+((ik[6][1][2]*Otkr[11][2])+((
      ik[6][1][0]*Otkr[11][0])+(ik[6][1][1]*Otkr[11][1])))));
    tstarr[11][2] = (utk[6][2]-(WkIkWk[11][2]+((ik[6][2][2]*Otkr[11][2])+((
      ik[6][2][0]*Otkr[11][0])+(ik[6][2][1]*Otkr[11][1])))));
    tstarr[12][0] = (utk[7][0]-(WkIkWk[12][0]+((ik[7][0][2]*Otkr[12][2])+((
      ik[7][0][0]*Otkr[12][0])+(ik[7][0][1]*Otkr[12][1])))));
    tstarr[12][1] = (utk[7][1]-(WkIkWk[12][1]+((ik[7][1][2]*Otkr[12][2])+((
      ik[7][1][0]*Otkr[12][0])+(ik[7][1][1]*Otkr[12][1])))));
    tstarr[12][2] = (utk[7][2]-(WkIkWk[12][2]+((ik[7][2][2]*Otkr[12][2])+((
      ik[7][2][0]*Otkr[12][0])+(ik[7][2][1]*Otkr[12][1])))));
    tstarr[13][0] = (utk[8][0]-(WkIkWk[13][0]+((ik[8][0][2]*Otkr[13][2])+((
      ik[8][0][0]*Otkr[13][0])+(ik[8][0][1]*Otkr[13][1])))));
    tstarr[13][1] = (utk[8][1]-(WkIkWk[13][1]+((ik[8][1][2]*Otkr[13][2])+((
      ik[8][1][0]*Otkr[13][0])+(ik[8][1][1]*Otkr[13][1])))));
    tstarr[13][2] = (utk[8][2]-(WkIkWk[13][2]+((ik[8][2][2]*Otkr[13][2])+((
      ik[8][2][0]*Otkr[13][0])+(ik[8][2][1]*Otkr[13][1])))));
    tstarr[14][0] = (utk[9][0]-(WkIkWk[14][0]+((ik[9][0][2]*Otkr[14][2])+((
      ik[9][0][0]*Otkr[14][0])+(ik[9][0][1]*Otkr[14][1])))));
    tstarr[14][1] = (utk[9][1]-(WkIkWk[14][1]+((ik[9][1][2]*Otkr[14][2])+((
      ik[9][1][0]*Otkr[14][0])+(ik[9][1][1]*Otkr[14][1])))));
    tstarr[14][2] = (utk[9][2]-(WkIkWk[14][2]+((ik[9][2][2]*Otkr[14][2])+((
      ik[9][2][0]*Otkr[14][0])+(ik[9][2][1]*Otkr[14][1])))));
    tstarr[15][0] = (utk[10][0]-(WkIkWk[15][0]+((ik[10][0][2]*Otkr[15][2])+((
      ik[10][0][0]*Otkr[15][0])+(ik[10][0][1]*Otkr[15][1])))));
    tstarr[15][1] = (utk[10][1]-(WkIkWk[15][1]+((ik[10][1][2]*Otkr[15][2])+((
      ik[10][1][0]*Otkr[15][0])+(ik[10][1][1]*Otkr[15][1])))));
    tstarr[15][2] = (utk[10][2]-(WkIkWk[15][2]+((ik[10][2][2]*Otkr[15][2])+((
      ik[10][2][0]*Otkr[15][0])+(ik[10][2][1]*Otkr[15][1])))));
    tstarr[16][0] = (utk[11][0]-(WkIkWk[16][0]+((ik[11][0][2]*Otkr[16][2])+((
      ik[11][0][0]*Otkr[16][0])+(ik[11][0][1]*Otkr[16][1])))));
    tstarr[16][1] = (utk[11][1]-(WkIkWk[16][1]+((ik[11][1][2]*Otkr[16][2])+((
      ik[11][1][0]*Otkr[16][0])+(ik[11][1][1]*Otkr[16][1])))));
    tstarr[16][2] = (utk[11][2]-(WkIkWk[16][2]+((ik[11][2][2]*Otkr[16][2])+((
      ik[11][2][0]*Otkr[16][0])+(ik[11][2][1]*Otkr[16][1])))));
    tstarr[17][0] = (utk[12][0]-(WkIkWk[17][0]+((ik[12][0][2]*Otkr[17][2])+((
      ik[12][0][0]*Otkr[17][0])+(ik[12][0][1]*Otkr[17][1])))));
    tstarr[17][1] = (utk[12][1]-(WkIkWk[17][1]+((ik[12][1][2]*Otkr[17][2])+((
      ik[12][1][0]*Otkr[17][0])+(ik[12][1][1]*Otkr[17][1])))));
    tstarr[17][2] = (utk[12][2]-(WkIkWk[17][2]+((ik[12][2][2]*Otkr[17][2])+((
      ik[12][2][0]*Otkr[17][0])+(ik[12][2][1]*Otkr[17][1])))));
    tstarr[18][0] = (utk[13][0]-(WkIkWk[18][0]+((ik[13][0][2]*Otkr[18][2])+((
      ik[13][0][0]*Otkr[18][0])+(ik[13][0][1]*Otkr[18][1])))));
    tstarr[18][1] = (utk[13][1]-(WkIkWk[18][1]+((ik[13][1][2]*Otkr[18][2])+((
      ik[13][1][0]*Otkr[18][0])+(ik[13][1][1]*Otkr[18][1])))));
    tstarr[18][2] = (utk[13][2]-(WkIkWk[18][2]+((ik[13][2][2]*Otkr[18][2])+((
      ik[13][2][0]*Otkr[18][0])+(ik[13][2][1]*Otkr[18][1])))));
    tstarr[19][0] = (utk[14][0]-(WkIkWk[19][0]+((ik[14][0][2]*Otkr[19][2])+((
      ik[14][0][0]*Otkr[19][0])+(ik[14][0][1]*Otkr[19][1])))));
    tstarr[19][1] = (utk[14][1]-(WkIkWk[19][1]+((ik[14][1][2]*Otkr[19][2])+((
      ik[14][1][0]*Otkr[19][0])+(ik[14][1][1]*Otkr[19][1])))));
    tstarr[19][2] = (utk[14][2]-(WkIkWk[19][2]+((ik[14][2][2]*Otkr[19][2])+((
      ik[14][2][0]*Otkr[19][0])+(ik[14][2][1]*Otkr[19][1])))));
    tstarr[20][0] = (utk[15][0]-(WkIkWk[20][0]+((ik[15][0][2]*Otkr[20][2])+((
      ik[15][0][0]*Otkr[20][0])+(ik[15][0][1]*Otkr[20][1])))));
    tstarr[20][1] = (utk[15][1]-(WkIkWk[20][1]+((ik[15][1][2]*Otkr[20][2])+((
      ik[15][1][0]*Otkr[20][0])+(ik[15][1][1]*Otkr[20][1])))));
    tstarr[20][2] = (utk[15][2]-(WkIkWk[20][2]+((ik[15][2][2]*Otkr[20][2])+((
      ik[15][2][0]*Otkr[20][0])+(ik[15][2][1]*Otkr[20][1])))));
/*
Now calculate the torques
*/
    sddovpk();
    temp[0] = (((fstarr[8][2]*Vpk[0][8][2])+((fstarr[8][0]*Vpk[0][8][0])+(
      fstarr[8][1]*Vpk[0][8][1])))+(((fstarr[7][2]*Vpk[0][7][2])+((fstarr[7][0]*
      Vpk[0][7][0])+(fstarr[7][1]*Vpk[0][7][1])))+(((fstarr[5][2]*Vpk[0][3][2])+
      ((fstarr[5][0]*Vpk[0][3][0])+(fstarr[5][1]*Vpk[0][3][1])))+((fstarr[6][2]*
      Vpk[0][6][2])+((fstarr[6][0]*Vpk[0][6][0])+(fstarr[6][1]*Vpk[0][6][1])))))
      );
    temp[1] = (((fstarr[12][2]*Vpk[0][12][2])+((fstarr[12][0]*Vpk[0][12][0])+(
      fstarr[12][1]*Vpk[0][12][1])))+(((fstarr[11][2]*Vpk[0][11][2])+((
      fstarr[11][0]*Vpk[0][11][0])+(fstarr[11][1]*Vpk[0][11][1])))+(((
      fstarr[10][2]*Vpk[0][10][2])+((fstarr[10][0]*Vpk[0][10][0])+(fstarr[10][1]
      *Vpk[0][10][1])))+(((fstarr[9][2]*Vpk[0][9][2])+((fstarr[9][0]*
      Vpk[0][9][0])+(fstarr[9][1]*Vpk[0][9][1])))+temp[0]))));
    temp[2] = (((fstarr[16][2]*Vpk[0][16][2])+((fstarr[16][0]*Vpk[0][16][0])+(
      fstarr[16][1]*Vpk[0][16][1])))+(((fstarr[15][2]*Vpk[0][15][2])+((
      fstarr[15][0]*Vpk[0][15][0])+(fstarr[15][1]*Vpk[0][15][1])))+(((
      fstarr[14][2]*Vpk[0][14][2])+((fstarr[14][0]*Vpk[0][14][0])+(fstarr[14][1]
      *Vpk[0][14][1])))+(((fstarr[13][2]*Vpk[0][13][2])+((fstarr[13][0]*
      Vpk[0][13][0])+(fstarr[13][1]*Vpk[0][13][1])))+temp[1]))));
    trqout[0] = -((mtau[0]+utau[0])+(((fstarr[20][2]*Vpk[0][20][2])+((
      fstarr[20][0]*Vpk[0][20][0])+(fstarr[20][1]*Vpk[0][20][1])))+(((
      fstarr[19][2]*Vpk[0][19][2])+((fstarr[19][0]*Vpk[0][19][0])+(fstarr[19][1]
      *Vpk[0][19][1])))+(((fstarr[18][2]*Vpk[0][18][2])+((fstarr[18][0]*
      Vpk[0][18][0])+(fstarr[18][1]*Vpk[0][18][1])))+(((fstarr[17][2]*
      Vpk[0][17][2])+((fstarr[17][0]*Vpk[0][17][0])+(fstarr[17][1]*Vpk[0][17][1]
      )))+temp[2])))));
    temp[0] = (((fstarr[8][2]*Vpk[1][8][2])+((fstarr[8][0]*Vpk[1][8][0])+(
      fstarr[8][1]*Vpk[1][8][1])))+(((fstarr[7][2]*Vpk[1][7][2])+((fstarr[7][0]*
      Vpk[1][7][0])+(fstarr[7][1]*Vpk[1][7][1])))+(((fstarr[5][2]*Vpk[1][3][2])+
      ((fstarr[5][0]*Vpk[1][3][0])+(fstarr[5][1]*Vpk[1][3][1])))+((fstarr[6][2]*
      Vpk[1][6][2])+((fstarr[6][0]*Vpk[1][6][0])+(fstarr[6][1]*Vpk[1][6][1])))))
      );
    temp[1] = (((fstarr[12][2]*Vpk[1][12][2])+((fstarr[12][0]*Vpk[1][12][0])+(
      fstarr[12][1]*Vpk[1][12][1])))+(((fstarr[11][2]*Vpk[1][11][2])+((
      fstarr[11][0]*Vpk[1][11][0])+(fstarr[11][1]*Vpk[1][11][1])))+(((
      fstarr[10][2]*Vpk[1][10][2])+((fstarr[10][0]*Vpk[1][10][0])+(fstarr[10][1]
      *Vpk[1][10][1])))+(((fstarr[9][2]*Vpk[1][9][2])+((fstarr[9][0]*
      Vpk[1][9][0])+(fstarr[9][1]*Vpk[1][9][1])))+temp[0]))));
    temp[2] = (((fstarr[16][2]*Vpk[1][16][2])+((fstarr[16][0]*Vpk[1][16][0])+(
      fstarr[16][1]*Vpk[1][16][1])))+(((fstarr[15][2]*Vpk[1][15][2])+((
      fstarr[15][0]*Vpk[1][15][0])+(fstarr[15][1]*Vpk[1][15][1])))+(((
      fstarr[14][2]*Vpk[1][14][2])+((fstarr[14][0]*Vpk[1][14][0])+(fstarr[14][1]
      *Vpk[1][14][1])))+(((fstarr[13][2]*Vpk[1][13][2])+((fstarr[13][0]*
      Vpk[1][13][0])+(fstarr[13][1]*Vpk[1][13][1])))+temp[1]))));
    trqout[1] = -((mtau[1]+utau[1])+(((fstarr[20][2]*Vpk[1][20][2])+((
      fstarr[20][0]*Vpk[1][20][0])+(fstarr[20][1]*Vpk[1][20][1])))+(((
      fstarr[19][2]*Vpk[1][19][2])+((fstarr[19][0]*Vpk[1][19][0])+(fstarr[19][1]
      *Vpk[1][19][1])))+(((fstarr[18][2]*Vpk[1][18][2])+((fstarr[18][0]*
      Vpk[1][18][0])+(fstarr[18][1]*Vpk[1][18][1])))+(((fstarr[17][2]*
      Vpk[1][17][2])+((fstarr[17][0]*Vpk[1][17][0])+(fstarr[17][1]*Vpk[1][17][1]
      )))+temp[2])))));
    temp[0] = (((fstarr[8][2]*Vpk[2][8][2])+((fstarr[8][0]*Vpk[2][8][0])+(
      fstarr[8][1]*Vpk[2][8][1])))+(((fstarr[7][2]*Vpk[2][7][2])+((fstarr[7][0]*
      Vpk[2][7][0])+(fstarr[7][1]*Vpk[2][7][1])))+(((fstarr[5][2]*Vpk[2][3][2])+
      ((fstarr[5][0]*Vpk[2][3][0])+(fstarr[5][1]*Vpk[2][3][1])))+((fstarr[6][2]*
      Vpk[2][6][2])+((fstarr[6][0]*Vpk[2][6][0])+(fstarr[6][1]*Vpk[2][6][1])))))
      );
    temp[1] = (((fstarr[12][2]*Vpk[2][12][2])+((fstarr[12][0]*Vpk[2][12][0])+(
      fstarr[12][1]*Vpk[2][12][1])))+(((fstarr[11][2]*Vpk[2][11][2])+((
      fstarr[11][0]*Vpk[2][11][0])+(fstarr[11][1]*Vpk[2][11][1])))+(((
      fstarr[10][2]*Vpk[2][10][2])+((fstarr[10][0]*Vpk[2][10][0])+(fstarr[10][1]
      *Vpk[2][10][1])))+(((fstarr[9][2]*Vpk[2][9][2])+((fstarr[9][0]*
      Vpk[2][9][0])+(fstarr[9][1]*Vpk[2][9][1])))+temp[0]))));
    temp[2] = (((fstarr[16][2]*Vpk[2][16][2])+((fstarr[16][0]*Vpk[2][16][0])+(
      fstarr[16][1]*Vpk[2][16][1])))+(((fstarr[15][2]*Vpk[2][15][2])+((
      fstarr[15][0]*Vpk[2][15][0])+(fstarr[15][1]*Vpk[2][15][1])))+(((
      fstarr[14][2]*Vpk[2][14][2])+((fstarr[14][0]*Vpk[2][14][0])+(fstarr[14][1]
      *Vpk[2][14][1])))+(((fstarr[13][2]*Vpk[2][13][2])+((fstarr[13][0]*
      Vpk[2][13][0])+(fstarr[13][1]*Vpk[2][13][1])))+temp[1]))));
    trqout[2] = -((mtau[2]+utau[2])+(((fstarr[20][2]*Vpk[2][20][2])+((
      fstarr[20][0]*Vpk[2][20][0])+(fstarr[20][1]*Vpk[2][20][1])))+(((
      fstarr[19][2]*Vpk[2][19][2])+((fstarr[19][0]*Vpk[2][19][0])+(fstarr[19][1]
      *Vpk[2][19][1])))+(((fstarr[18][2]*Vpk[2][18][2])+((fstarr[18][0]*
      Vpk[2][18][0])+(fstarr[18][1]*Vpk[2][18][1])))+(((fstarr[17][2]*
      Vpk[2][17][2])+((fstarr[17][0]*Vpk[2][17][0])+(fstarr[17][1]*Vpk[2][17][1]
      )))+temp[2])))));
    temp[0] = (((tstarr[5][0]+((fstarr[5][1]*rk[0][2])-(fstarr[5][2]*rk[0][1])))
      +(((Cik[6][0][2]*tstarr[6][2])+((Cik[6][0][0]*tstarr[6][0])+(Cik[6][0][1]*
      tstarr[6][1])))+((fstarr[6][2]*Vpk[3][6][2])+((fstarr[6][0]*Vpk[3][6][0])+
      (fstarr[6][1]*Vpk[3][6][1])))))+(((fstarr[7][2]*Vpk[3][7][2])+((
      fstarr[7][0]*Vpk[3][7][0])+(fstarr[7][1]*Vpk[3][7][1])))+((tstarr[7][2]*
      Wpk[3][7][2])+((tstarr[7][0]*Wpk[3][7][0])+(tstarr[7][1]*Wpk[3][7][1])))))
      ;
    temp[1] = ((((Cik[9][0][2]*tstarr[9][2])+((Cik[9][0][0]*tstarr[9][0])+(
      Cik[9][0][1]*tstarr[9][1])))+((fstarr[9][2]*Vpk[3][9][2])+((fstarr[9][0]*
      Vpk[3][9][0])+(fstarr[9][1]*Vpk[3][9][1]))))+((((fstarr[8][2]*Vpk[3][8][2]
      )+((fstarr[8][0]*Vpk[3][8][0])+(fstarr[8][1]*Vpk[3][8][1])))+((
      tstarr[8][2]*Wpk[3][8][2])+((tstarr[8][0]*Wpk[3][8][0])+(tstarr[8][1]*
      Wpk[3][8][1]))))+temp[0]));
    temp[2] = ((((fstarr[11][2]*Vpk[3][11][2])+((fstarr[11][0]*Vpk[3][11][0])+(
      fstarr[11][1]*Vpk[3][11][1])))+((tstarr[11][2]*Wpk[3][11][2])+((
      tstarr[11][0]*Wpk[3][11][0])+(tstarr[11][1]*Wpk[3][11][1]))))+((((
      fstarr[10][2]*Vpk[3][10][2])+((fstarr[10][0]*Vpk[3][10][0])+(fstarr[10][1]
      *Vpk[3][10][1])))+((tstarr[10][2]*Wpk[3][10][2])+((tstarr[10][0]*
      Wpk[3][10][0])+(tstarr[10][1]*Wpk[3][10][1]))))+temp[1]));
    temp[3] = ((((fstarr[13][2]*Vpk[3][13][2])+((fstarr[13][0]*Vpk[3][13][0])+(
      fstarr[13][1]*Vpk[3][13][1])))+((tstarr[13][2]*Wpk[3][13][2])+((
      tstarr[13][0]*Wpk[3][13][0])+(tstarr[13][1]*Wpk[3][13][1]))))+((((
      fstarr[12][2]*Vpk[3][12][2])+((fstarr[12][0]*Vpk[3][12][0])+(fstarr[12][1]
      *Vpk[3][12][1])))+((tstarr[12][2]*Wpk[3][12][2])+((tstarr[12][0]*
      Wpk[3][12][0])+(tstarr[12][1]*Wpk[3][12][1]))))+temp[2]));
    temp[4] = ((((Cik[15][0][2]*tstarr[15][2])+((Cik[15][0][0]*tstarr[15][0])+(
      Cik[15][0][1]*tstarr[15][1])))+((fstarr[15][2]*Vpk[3][15][2])+((
      fstarr[15][0]*Vpk[3][15][0])+(fstarr[15][1]*Vpk[3][15][1]))))+((((
      fstarr[14][2]*Vpk[3][14][2])+((fstarr[14][0]*Vpk[3][14][0])+(fstarr[14][1]
      *Vpk[3][14][1])))+((tstarr[14][2]*Wpk[3][14][2])+((tstarr[14][0]*
      Wpk[3][14][0])+(tstarr[14][1]*Wpk[3][14][1]))))+temp[3]));
    temp[5] = ((((fstarr[17][2]*Vpk[3][17][2])+((fstarr[17][0]*Vpk[3][17][0])+(
      fstarr[17][1]*Vpk[3][17][1])))+((tstarr[17][2]*Wpk[3][17][2])+((
      tstarr[17][0]*Wpk[3][17][0])+(tstarr[17][1]*Wpk[3][17][1]))))+((((
      fstarr[16][2]*Vpk[3][16][2])+((fstarr[16][0]*Vpk[3][16][0])+(fstarr[16][1]
      *Vpk[3][16][1])))+((tstarr[16][2]*Wpk[3][16][2])+((tstarr[16][0]*
      Wpk[3][16][0])+(tstarr[16][1]*Wpk[3][16][1]))))+temp[4]));
    temp[6] = ((((fstarr[19][2]*Vpk[3][19][2])+((fstarr[19][0]*Vpk[3][19][0])+(
      fstarr[19][1]*Vpk[3][19][1])))+((tstarr[19][2]*Wpk[3][19][2])+((
      tstarr[19][0]*Wpk[3][19][0])+(tstarr[19][1]*Wpk[3][19][1]))))+((((
      fstarr[18][2]*Vpk[3][18][2])+((fstarr[18][0]*Vpk[3][18][0])+(fstarr[18][1]
      *Vpk[3][18][1])))+((tstarr[18][2]*Wpk[3][18][2])+((tstarr[18][0]*
      Wpk[3][18][0])+(tstarr[18][1]*Wpk[3][18][1]))))+temp[5]));
    trqout[3] = -((mtau[3]+utau[3])+((((fstarr[20][2]*Vpk[3][20][2])+((
      fstarr[20][0]*Vpk[3][20][0])+(fstarr[20][1]*Vpk[3][20][1])))+((
      tstarr[20][2]*Wpk[3][20][2])+((tstarr[20][0]*Wpk[3][20][0])+(tstarr[20][1]
      *Wpk[3][20][1]))))+temp[6]));
    temp[0] = (((tstarr[5][1]+((fstarr[5][2]*rk[0][0])-(fstarr[5][0]*rk[0][2])))
      +(((Cik[6][1][2]*tstarr[6][2])+((Cik[6][1][0]*tstarr[6][0])+(Cik[6][1][1]*
      tstarr[6][1])))+((fstarr[6][2]*Vpk[4][6][2])+((fstarr[6][0]*Vpk[4][6][0])+
      (fstarr[6][1]*Vpk[4][6][1])))))+(((fstarr[7][2]*Vpk[4][7][2])+((
      fstarr[7][0]*Vpk[4][7][0])+(fstarr[7][1]*Vpk[4][7][1])))+((tstarr[7][2]*
      Wpk[4][7][2])+((tstarr[7][0]*Wpk[4][7][0])+(tstarr[7][1]*Wpk[4][7][1])))))
      ;
    temp[1] = ((((Cik[9][1][2]*tstarr[9][2])+((Cik[9][1][0]*tstarr[9][0])+(
      Cik[9][1][1]*tstarr[9][1])))+((fstarr[9][2]*Vpk[4][9][2])+((fstarr[9][0]*
      Vpk[4][9][0])+(fstarr[9][1]*Vpk[4][9][1]))))+((((fstarr[8][2]*Vpk[4][8][2]
      )+((fstarr[8][0]*Vpk[4][8][0])+(fstarr[8][1]*Vpk[4][8][1])))+((
      tstarr[8][2]*Wpk[4][8][2])+((tstarr[8][0]*Wpk[4][8][0])+(tstarr[8][1]*
      Wpk[4][8][1]))))+temp[0]));
    temp[2] = ((((fstarr[11][2]*Vpk[4][11][2])+((fstarr[11][0]*Vpk[4][11][0])+(
      fstarr[11][1]*Vpk[4][11][1])))+((tstarr[11][2]*Wpk[4][11][2])+((
      tstarr[11][0]*Wpk[4][11][0])+(tstarr[11][1]*Wpk[4][11][1]))))+((((
      fstarr[10][2]*Vpk[4][10][2])+((fstarr[10][0]*Vpk[4][10][0])+(fstarr[10][1]
      *Vpk[4][10][1])))+((tstarr[10][2]*Wpk[4][10][2])+((tstarr[10][0]*
      Wpk[4][10][0])+(tstarr[10][1]*Wpk[4][10][1]))))+temp[1]));
    temp[3] = ((((fstarr[13][2]*Vpk[4][13][2])+((fstarr[13][0]*Vpk[4][13][0])+(
      fstarr[13][1]*Vpk[4][13][1])))+((tstarr[13][2]*Wpk[4][13][2])+((
      tstarr[13][0]*Wpk[4][13][0])+(tstarr[13][1]*Wpk[4][13][1]))))+((((
      fstarr[12][2]*Vpk[4][12][2])+((fstarr[12][0]*Vpk[4][12][0])+(fstarr[12][1]
      *Vpk[4][12][1])))+((tstarr[12][2]*Wpk[4][12][2])+((tstarr[12][0]*
      Wpk[4][12][0])+(tstarr[12][1]*Wpk[4][12][1]))))+temp[2]));
    temp[4] = ((((Cik[15][1][2]*tstarr[15][2])+((Cik[15][1][0]*tstarr[15][0])+(
      Cik[15][1][1]*tstarr[15][1])))+((fstarr[15][2]*Vpk[4][15][2])+((
      fstarr[15][0]*Vpk[4][15][0])+(fstarr[15][1]*Vpk[4][15][1]))))+((((
      fstarr[14][2]*Vpk[4][14][2])+((fstarr[14][0]*Vpk[4][14][0])+(fstarr[14][1]
      *Vpk[4][14][1])))+((tstarr[14][2]*Wpk[4][14][2])+((tstarr[14][0]*
      Wpk[4][14][0])+(tstarr[14][1]*Wpk[4][14][1]))))+temp[3]));
    temp[5] = ((((fstarr[17][2]*Vpk[4][17][2])+((fstarr[17][0]*Vpk[4][17][0])+(
      fstarr[17][1]*Vpk[4][17][1])))+((tstarr[17][2]*Wpk[4][17][2])+((
      tstarr[17][0]*Wpk[4][17][0])+(tstarr[17][1]*Wpk[4][17][1]))))+((((
      fstarr[16][2]*Vpk[4][16][2])+((fstarr[16][0]*Vpk[4][16][0])+(fstarr[16][1]
      *Vpk[4][16][1])))+((tstarr[16][2]*Wpk[4][16][2])+((tstarr[16][0]*
      Wpk[4][16][0])+(tstarr[16][1]*Wpk[4][16][1]))))+temp[4]));
    temp[6] = ((((fstarr[19][2]*Vpk[4][19][2])+((fstarr[19][0]*Vpk[4][19][0])+(
      fstarr[19][1]*Vpk[4][19][1])))+((tstarr[19][2]*Wpk[4][19][2])+((
      tstarr[19][0]*Wpk[4][19][0])+(tstarr[19][1]*Wpk[4][19][1]))))+((((
      fstarr[18][2]*Vpk[4][18][2])+((fstarr[18][0]*Vpk[4][18][0])+(fstarr[18][1]
      *Vpk[4][18][1])))+((tstarr[18][2]*Wpk[4][18][2])+((tstarr[18][0]*
      Wpk[4][18][0])+(tstarr[18][1]*Wpk[4][18][1]))))+temp[5]));
    trqout[4] = -((mtau[4]+utau[4])+((((fstarr[20][2]*Vpk[4][20][2])+((
      fstarr[20][0]*Vpk[4][20][0])+(fstarr[20][1]*Vpk[4][20][1])))+((
      tstarr[20][2]*Wpk[4][20][2])+((tstarr[20][0]*Wpk[4][20][0])+(tstarr[20][1]
      *Wpk[4][20][1]))))+temp[6]));
    temp[0] = (((tstarr[5][2]+((fstarr[5][0]*rk[0][1])-(fstarr[5][1]*rk[0][0])))
      +(((Cik[6][2][2]*tstarr[6][2])+((Cik[6][2][0]*tstarr[6][0])+(Cik[6][2][1]*
      tstarr[6][1])))+((fstarr[6][2]*Vpk[5][6][2])+((fstarr[6][0]*Vpk[5][6][0])+
      (fstarr[6][1]*Vpk[5][6][1])))))+(((fstarr[7][2]*Vpk[5][7][2])+((
      fstarr[7][0]*Vpk[5][7][0])+(fstarr[7][1]*Vpk[5][7][1])))+((tstarr[7][2]*
      Wpk[5][7][2])+((tstarr[7][0]*Wpk[5][7][0])+(tstarr[7][1]*Wpk[5][7][1])))))
      ;
    temp[1] = ((((Cik[9][2][2]*tstarr[9][2])+((Cik[9][2][0]*tstarr[9][0])+(
      Cik[9][2][1]*tstarr[9][1])))+((fstarr[9][2]*Vpk[5][9][2])+((fstarr[9][0]*
      Vpk[5][9][0])+(fstarr[9][1]*Vpk[5][9][1]))))+((((fstarr[8][2]*Vpk[5][8][2]
      )+((fstarr[8][0]*Vpk[5][8][0])+(fstarr[8][1]*Vpk[5][8][1])))+((
      tstarr[8][2]*Wpk[5][8][2])+((tstarr[8][0]*Wpk[5][8][0])+(tstarr[8][1]*
      Wpk[5][8][1]))))+temp[0]));
    temp[2] = ((((fstarr[11][2]*Vpk[5][11][2])+((fstarr[11][0]*Vpk[5][11][0])+(
      fstarr[11][1]*Vpk[5][11][1])))+((tstarr[11][2]*Wpk[5][11][2])+((
      tstarr[11][0]*Wpk[5][11][0])+(tstarr[11][1]*Wpk[5][11][1]))))+((((
      fstarr[10][2]*Vpk[5][10][2])+((fstarr[10][0]*Vpk[5][10][0])+(fstarr[10][1]
      *Vpk[5][10][1])))+((tstarr[10][2]*Wpk[5][10][2])+((tstarr[10][0]*
      Wpk[5][10][0])+(tstarr[10][1]*Wpk[5][10][1]))))+temp[1]));
    temp[3] = ((((fstarr[13][2]*Vpk[5][13][2])+((fstarr[13][0]*Vpk[5][13][0])+(
      fstarr[13][1]*Vpk[5][13][1])))+((tstarr[13][2]*Wpk[5][13][2])+((
      tstarr[13][0]*Wpk[5][13][0])+(tstarr[13][1]*Wpk[5][13][1]))))+((((
      fstarr[12][2]*Vpk[5][12][2])+((fstarr[12][0]*Vpk[5][12][0])+(fstarr[12][1]
      *Vpk[5][12][1])))+((tstarr[12][2]*Wpk[5][12][2])+((tstarr[12][0]*
      Wpk[5][12][0])+(tstarr[12][1]*Wpk[5][12][1]))))+temp[2]));
    temp[4] = ((((Cik[15][2][2]*tstarr[15][2])+((Cik[15][2][0]*tstarr[15][0])+(
      Cik[15][2][1]*tstarr[15][1])))+((fstarr[15][2]*Vpk[5][15][2])+((
      fstarr[15][0]*Vpk[5][15][0])+(fstarr[15][1]*Vpk[5][15][1]))))+((((
      fstarr[14][2]*Vpk[5][14][2])+((fstarr[14][0]*Vpk[5][14][0])+(fstarr[14][1]
      *Vpk[5][14][1])))+((tstarr[14][2]*Wpk[5][14][2])+((tstarr[14][0]*
      Wpk[5][14][0])+(tstarr[14][1]*Wpk[5][14][1]))))+temp[3]));
    temp[5] = ((((fstarr[17][2]*Vpk[5][17][2])+((fstarr[17][0]*Vpk[5][17][0])+(
      fstarr[17][1]*Vpk[5][17][1])))+((tstarr[17][2]*Wpk[5][17][2])+((
      tstarr[17][0]*Wpk[5][17][0])+(tstarr[17][1]*Wpk[5][17][1]))))+((((
      fstarr[16][2]*Vpk[5][16][2])+((fstarr[16][0]*Vpk[5][16][0])+(fstarr[16][1]
      *Vpk[5][16][1])))+((tstarr[16][2]*Wpk[5][16][2])+((tstarr[16][0]*
      Wpk[5][16][0])+(tstarr[16][1]*Wpk[5][16][1]))))+temp[4]));
    temp[6] = ((((fstarr[19][2]*Vpk[5][19][2])+((fstarr[19][0]*Vpk[5][19][0])+(
      fstarr[19][1]*Vpk[5][19][1])))+((tstarr[19][2]*Wpk[5][19][2])+((
      tstarr[19][0]*Wpk[5][19][0])+(tstarr[19][1]*Wpk[5][19][1]))))+((((
      fstarr[18][2]*Vpk[5][18][2])+((fstarr[18][0]*Vpk[5][18][0])+(fstarr[18][1]
      *Vpk[5][18][1])))+((tstarr[18][2]*Wpk[5][18][2])+((tstarr[18][0]*
      Wpk[5][18][0])+(tstarr[18][1]*Wpk[5][18][1]))))+temp[5]));
    trqout[5] = -((mtau[5]+utau[5])+((((fstarr[20][2]*Vpk[5][20][2])+((
      fstarr[20][0]*Vpk[5][20][0])+(fstarr[20][1]*Vpk[5][20][1])))+((
      tstarr[20][2]*Wpk[5][20][2])+((tstarr[20][0]*Wpk[5][20][0])+(tstarr[20][1]
      *Wpk[5][20][1]))))+temp[6]));
    temp[0] = ((((fstarr[6][2]*Vpk[6][6][2])+((fstarr[6][0]*Vpk[6][6][0])+(
      fstarr[6][1]*Vpk[6][6][1])))+((pin[6][2]*tstarr[6][2])+((pin[6][0]*
      tstarr[6][0])+(pin[6][1]*tstarr[6][1]))))+(((fstarr[7][2]*Vpk[6][7][2])+((
      fstarr[7][0]*Vpk[6][7][0])+(fstarr[7][1]*Vpk[6][7][1])))+((tstarr[7][2]*
      Wpk[6][7][2])+((tstarr[7][0]*Wpk[6][7][0])+(tstarr[7][1]*Wpk[6][7][1])))))
      ;
    trqout[6] = -((mtau[6]+utau[6])+((((fstarr[8][2]*Vpk[6][8][2])+((
      fstarr[8][0]*Vpk[6][8][0])+(fstarr[8][1]*Vpk[6][8][1])))+((tstarr[8][2]*
      Wpk[6][8][2])+((tstarr[8][0]*Wpk[6][8][0])+(tstarr[8][1]*Wpk[6][8][1]))))+
      temp[0]));
    trqout[7] = -((mtau[7]+utau[7])+((((fstarr[7][2]*Vpk[7][7][2])+((
      fstarr[7][0]*Vpk[7][7][0])+(fstarr[7][1]*Vpk[7][7][1])))+((pin[7][2]*
      tstarr[7][2])+((pin[7][0]*tstarr[7][0])+(pin[7][1]*tstarr[7][1]))))+(((
      fstarr[8][2]*Vpk[7][8][2])+((fstarr[8][0]*Vpk[7][8][0])+(fstarr[8][1]*
      Vpk[7][8][1])))+((tstarr[8][2]*Wpk[7][8][2])+((tstarr[8][0]*Wpk[7][8][0])+
      (tstarr[8][1]*Wpk[7][8][1]))))));
    trqout[8] = -((mtau[8]+utau[8])+(((fstarr[8][2]*Vpk[8][8][2])+((fstarr[8][0]
      *Vpk[8][8][0])+(fstarr[8][1]*Vpk[8][8][1])))+((pin[8][2]*tstarr[8][2])+((
      pin[8][0]*tstarr[8][0])+(pin[8][1]*tstarr[8][1])))));
    temp[0] = ((((fstarr[9][2]*Vpk[9][9][2])+((fstarr[9][0]*Vpk[9][9][0])+(
      fstarr[9][1]*Vpk[9][9][1])))+((pin[9][2]*tstarr[9][2])+((pin[9][0]*
      tstarr[9][0])+(pin[9][1]*tstarr[9][1]))))+(((fstarr[10][2]*Vpk[9][10][2])+
      ((fstarr[10][0]*Vpk[9][10][0])+(fstarr[10][1]*Vpk[9][10][1])))+((
      tstarr[10][2]*Wpk[9][10][2])+((tstarr[10][0]*Wpk[9][10][0])+(tstarr[10][1]
      *Wpk[9][10][1])))));
    temp[1] = ((((fstarr[12][2]*Vpk[9][12][2])+((fstarr[12][0]*Vpk[9][12][0])+(
      fstarr[12][1]*Vpk[9][12][1])))+((tstarr[12][2]*Wpk[9][12][2])+((
      tstarr[12][0]*Wpk[9][12][0])+(tstarr[12][1]*Wpk[9][12][1]))))+((((
      fstarr[11][2]*Vpk[9][11][2])+((fstarr[11][0]*Vpk[9][11][0])+(fstarr[11][1]
      *Vpk[9][11][1])))+((tstarr[11][2]*Wpk[9][11][2])+((tstarr[11][0]*
      Wpk[9][11][0])+(tstarr[11][1]*Wpk[9][11][1]))))+temp[0]));
    trqout[9] = -((mtau[9]+utau[9])+((((fstarr[14][2]*Vpk[9][14][2])+((
      fstarr[14][0]*Vpk[9][14][0])+(fstarr[14][1]*Vpk[9][14][1])))+((
      tstarr[14][2]*Wpk[9][14][2])+((tstarr[14][0]*Wpk[9][14][0])+(tstarr[14][1]
      *Wpk[9][14][1]))))+((((fstarr[13][2]*Vpk[9][13][2])+((fstarr[13][0]*
      Vpk[9][13][0])+(fstarr[13][1]*Vpk[9][13][1])))+((tstarr[13][2]*
      Wpk[9][13][2])+((tstarr[13][0]*Wpk[9][13][0])+(tstarr[13][1]*Wpk[9][13][1]
      ))))+temp[1])));
    temp[0] = ((((fstarr[10][2]*Vpk[10][10][2])+((fstarr[10][0]*Vpk[10][10][0])+
      (fstarr[10][1]*Vpk[10][10][1])))+((pin[10][2]*tstarr[10][2])+((pin[10][0]*
      tstarr[10][0])+(pin[10][1]*tstarr[10][1]))))+(((fstarr[11][2]*
      Vpk[10][11][2])+((fstarr[11][0]*Vpk[10][11][0])+(fstarr[11][1]*
      Vpk[10][11][1])))+((tstarr[11][2]*Wpk[10][11][2])+((tstarr[11][0]*
      Wpk[10][11][0])+(tstarr[11][1]*Wpk[10][11][1])))));
    temp[1] = ((((fstarr[13][2]*Vpk[10][13][2])+((fstarr[13][0]*Vpk[10][13][0])+
      (fstarr[13][1]*Vpk[10][13][1])))+((tstarr[13][2]*Wpk[10][13][2])+((
      tstarr[13][0]*Wpk[10][13][0])+(tstarr[13][1]*Wpk[10][13][1]))))+((((
      fstarr[12][2]*Vpk[10][12][2])+((fstarr[12][0]*Vpk[10][12][0])+(
      fstarr[12][1]*Vpk[10][12][1])))+((tstarr[12][2]*Wpk[10][12][2])+((
      tstarr[12][0]*Wpk[10][12][0])+(tstarr[12][1]*Wpk[10][12][1]))))+temp[0]));
    trqout[10] = -((mtau[10]+utau[10])+((((fstarr[14][2]*Vpk[10][14][2])+((
      fstarr[14][0]*Vpk[10][14][0])+(fstarr[14][1]*Vpk[10][14][1])))+((
      tstarr[14][2]*Wpk[10][14][2])+((tstarr[14][0]*Wpk[10][14][0])+(
      tstarr[14][1]*Wpk[10][14][1]))))+temp[1]));
    temp[0] = ((((fstarr[11][2]*Vpk[11][11][2])+((fstarr[11][0]*Vpk[11][11][0])+
      (fstarr[11][1]*Vpk[11][11][1])))+((pin[11][2]*tstarr[11][2])+((pin[11][0]*
      tstarr[11][0])+(pin[11][1]*tstarr[11][1]))))+(((fstarr[12][2]*
      Vpk[11][12][2])+((fstarr[12][0]*Vpk[11][12][0])+(fstarr[12][1]*
      Vpk[11][12][1])))+((tstarr[12][2]*Wpk[11][12][2])+((tstarr[12][0]*
      Wpk[11][12][0])+(tstarr[12][1]*Wpk[11][12][1])))));
    trqout[11] = -((mtau[11]+utau[11])+((((fstarr[14][2]*Vpk[11][14][2])+((
      fstarr[14][0]*Vpk[11][14][0])+(fstarr[14][1]*Vpk[11][14][1])))+((
      tstarr[14][2]*Wpk[11][14][2])+((tstarr[14][0]*Wpk[11][14][0])+(
      tstarr[14][1]*Wpk[11][14][1]))))+((((fstarr[13][2]*Vpk[11][13][2])+((
      fstarr[13][0]*Vpk[11][13][0])+(fstarr[13][1]*Vpk[11][13][1])))+((
      tstarr[13][2]*Wpk[11][13][2])+((tstarr[13][0]*Wpk[11][13][0])+(
      tstarr[13][1]*Wpk[11][13][1]))))+temp[0])));
    temp[0] = ((((fstarr[12][2]*Vpk[12][12][2])+((fstarr[12][0]*Vpk[12][12][0])+
      (fstarr[12][1]*Vpk[12][12][1])))+((pin[12][2]*tstarr[12][2])+((pin[12][0]*
      tstarr[12][0])+(pin[12][1]*tstarr[12][1]))))+(((fstarr[13][2]*
      Vpk[12][13][2])+((fstarr[13][0]*Vpk[12][13][0])+(fstarr[13][1]*
      Vpk[12][13][1])))+((tstarr[13][2]*Wpk[12][13][2])+((tstarr[13][0]*
      Wpk[12][13][0])+(tstarr[13][1]*Wpk[12][13][1])))));
    trqout[12] = -((mtau[12]+utau[12])+((((fstarr[14][2]*Vpk[12][14][2])+((
      fstarr[14][0]*Vpk[12][14][0])+(fstarr[14][1]*Vpk[12][14][1])))+((
      tstarr[14][2]*Wpk[12][14][2])+((tstarr[14][0]*Wpk[12][14][0])+(
      tstarr[14][1]*Wpk[12][14][1]))))+temp[0]));
    trqout[13] = -((mtau[13]+utau[13])+((((fstarr[13][2]*Vpk[13][13][2])+((
      fstarr[13][0]*Vpk[13][13][0])+(fstarr[13][1]*Vpk[13][13][1])))+((
      pin[13][2]*tstarr[13][2])+((pin[13][0]*tstarr[13][0])+(pin[13][1]*
      tstarr[13][1]))))+(((fstarr[14][2]*Vpk[13][14][2])+((fstarr[14][0]*
      Vpk[13][14][0])+(fstarr[14][1]*Vpk[13][14][1])))+((tstarr[14][2]*
      Wpk[13][14][2])+((tstarr[14][0]*Wpk[13][14][0])+(tstarr[14][1]*
      Wpk[13][14][1]))))));
    trqout[14] = -((mtau[14]+utau[14])+(((fstarr[14][2]*Vpk[14][14][2])+((
      fstarr[14][0]*Vpk[14][14][0])+(fstarr[14][1]*Vpk[14][14][1])))+((
      pin[14][2]*tstarr[14][2])+((pin[14][0]*tstarr[14][0])+(pin[14][1]*
      tstarr[14][1])))));
    temp[0] = ((((fstarr[15][2]*Vpk[15][15][2])+((fstarr[15][0]*Vpk[15][15][0])+
      (fstarr[15][1]*Vpk[15][15][1])))+((pin[15][2]*tstarr[15][2])+((pin[15][0]*
      tstarr[15][0])+(pin[15][1]*tstarr[15][1]))))+(((fstarr[16][2]*
      Vpk[15][16][2])+((fstarr[16][0]*Vpk[15][16][0])+(fstarr[16][1]*
      Vpk[15][16][1])))+((tstarr[16][2]*Wpk[15][16][2])+((tstarr[16][0]*
      Wpk[15][16][0])+(tstarr[16][1]*Wpk[15][16][1])))));
    temp[1] = ((((fstarr[18][2]*Vpk[15][18][2])+((fstarr[18][0]*Vpk[15][18][0])+
      (fstarr[18][1]*Vpk[15][18][1])))+((tstarr[18][2]*Wpk[15][18][2])+((
      tstarr[18][0]*Wpk[15][18][0])+(tstarr[18][1]*Wpk[15][18][1]))))+((((
      fstarr[17][2]*Vpk[15][17][2])+((fstarr[17][0]*Vpk[15][17][0])+(
      fstarr[17][1]*Vpk[15][17][1])))+((tstarr[17][2]*Wpk[15][17][2])+((
      tstarr[17][0]*Wpk[15][17][0])+(tstarr[17][1]*Wpk[15][17][1]))))+temp[0]));
    trqout[15] = -((mtau[15]+utau[15])+((((fstarr[20][2]*Vpk[15][20][2])+((
      fstarr[20][0]*Vpk[15][20][0])+(fstarr[20][1]*Vpk[15][20][1])))+((
      tstarr[20][2]*Wpk[15][20][2])+((tstarr[20][0]*Wpk[15][20][0])+(
      tstarr[20][1]*Wpk[15][20][1]))))+((((fstarr[19][2]*Vpk[15][19][2])+((
      fstarr[19][0]*Vpk[15][19][0])+(fstarr[19][1]*Vpk[15][19][1])))+((
      tstarr[19][2]*Wpk[15][19][2])+((tstarr[19][0]*Wpk[15][19][0])+(
      tstarr[19][1]*Wpk[15][19][1]))))+temp[1])));
    temp[0] = ((((fstarr[16][2]*Vpk[16][16][2])+((fstarr[16][0]*Vpk[16][16][0])+
      (fstarr[16][1]*Vpk[16][16][1])))+((pin[16][2]*tstarr[16][2])+((pin[16][0]*
      tstarr[16][0])+(pin[16][1]*tstarr[16][1]))))+(((fstarr[17][2]*
      Vpk[16][17][2])+((fstarr[17][0]*Vpk[16][17][0])+(fstarr[17][1]*
      Vpk[16][17][1])))+((tstarr[17][2]*Wpk[16][17][2])+((tstarr[17][0]*
      Wpk[16][17][0])+(tstarr[17][1]*Wpk[16][17][1])))));
    temp[1] = ((((fstarr[19][2]*Vpk[16][19][2])+((fstarr[19][0]*Vpk[16][19][0])+
      (fstarr[19][1]*Vpk[16][19][1])))+((tstarr[19][2]*Wpk[16][19][2])+((
      tstarr[19][0]*Wpk[16][19][0])+(tstarr[19][1]*Wpk[16][19][1]))))+((((
      fstarr[18][2]*Vpk[16][18][2])+((fstarr[18][0]*Vpk[16][18][0])+(
      fstarr[18][1]*Vpk[16][18][1])))+((tstarr[18][2]*Wpk[16][18][2])+((
      tstarr[18][0]*Wpk[16][18][0])+(tstarr[18][1]*Wpk[16][18][1]))))+temp[0]));
    trqout[16] = -((mtau[16]+utau[16])+((((fstarr[20][2]*Vpk[16][20][2])+((
      fstarr[20][0]*Vpk[16][20][0])+(fstarr[20][1]*Vpk[16][20][1])))+((
      tstarr[20][2]*Wpk[16][20][2])+((tstarr[20][0]*Wpk[16][20][0])+(
      tstarr[20][1]*Wpk[16][20][1]))))+temp[1]));
    temp[0] = ((((fstarr[17][2]*Vpk[17][17][2])+((fstarr[17][0]*Vpk[17][17][0])+
      (fstarr[17][1]*Vpk[17][17][1])))+((pin[17][2]*tstarr[17][2])+((pin[17][0]*
      tstarr[17][0])+(pin[17][1]*tstarr[17][1]))))+(((fstarr[18][2]*
      Vpk[17][18][2])+((fstarr[18][0]*Vpk[17][18][0])+(fstarr[18][1]*
      Vpk[17][18][1])))+((tstarr[18][2]*Wpk[17][18][2])+((tstarr[18][0]*
      Wpk[17][18][0])+(tstarr[18][1]*Wpk[17][18][1])))));
    trqout[17] = -((mtau[17]+utau[17])+((((fstarr[20][2]*Vpk[17][20][2])+((
      fstarr[20][0]*Vpk[17][20][0])+(fstarr[20][1]*Vpk[17][20][1])))+((
      tstarr[20][2]*Wpk[17][20][2])+((tstarr[20][0]*Wpk[17][20][0])+(
      tstarr[20][1]*Wpk[17][20][1]))))+((((fstarr[19][2]*Vpk[17][19][2])+((
      fstarr[19][0]*Vpk[17][19][0])+(fstarr[19][1]*Vpk[17][19][1])))+((
      tstarr[19][2]*Wpk[17][19][2])+((tstarr[19][0]*Wpk[17][19][0])+(
      tstarr[19][1]*Wpk[17][19][1]))))+temp[0])));
    temp[0] = ((((fstarr[18][2]*Vpk[18][18][2])+((fstarr[18][0]*Vpk[18][18][0])+
      (fstarr[18][1]*Vpk[18][18][1])))+((pin[18][2]*tstarr[18][2])+((pin[18][0]*
      tstarr[18][0])+(pin[18][1]*tstarr[18][1]))))+(((fstarr[19][2]*
      Vpk[18][19][2])+((fstarr[19][0]*Vpk[18][19][0])+(fstarr[19][1]*
      Vpk[18][19][1])))+((tstarr[19][2]*Wpk[18][19][2])+((tstarr[19][0]*
      Wpk[18][19][0])+(tstarr[19][1]*Wpk[18][19][1])))));
    trqout[18] = -((mtau[18]+utau[18])+((((fstarr[20][2]*Vpk[18][20][2])+((
      fstarr[20][0]*Vpk[18][20][0])+(fstarr[20][1]*Vpk[18][20][1])))+((
      tstarr[20][2]*Wpk[18][20][2])+((tstarr[20][0]*Wpk[18][20][0])+(
      tstarr[20][1]*Wpk[18][20][1]))))+temp[0]));
    trqout[19] = -((mtau[19]+utau[19])+((((fstarr[19][2]*Vpk[19][19][2])+((
      fstarr[19][0]*Vpk[19][19][0])+(fstarr[19][1]*Vpk[19][19][1])))+((
      pin[19][2]*tstarr[19][2])+((pin[19][0]*tstarr[19][0])+(pin[19][1]*
      tstarr[19][1]))))+(((fstarr[20][2]*Vpk[19][20][2])+((fstarr[20][0]*
      Vpk[19][20][0])+(fstarr[20][1]*Vpk[19][20][1])))+((tstarr[20][2]*
      Wpk[19][20][2])+((tstarr[20][0]*Wpk[19][20][0])+(tstarr[20][1]*
      Wpk[19][20][1]))))));
    trqout[20] = -((mtau[20]+utau[20])+(((fstarr[20][2]*Vpk[20][20][2])+((
      fstarr[20][0]*Vpk[20][20][0])+(fstarr[20][1]*Vpk[20][20][1])))+((
      pin[20][2]*tstarr[20][2])+((pin[20][0]*tstarr[20][0])+(pin[20][1]*
      tstarr[20][1])))));
/*
Op counts below do not include called subroutines
*/
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain 1740 adds/subtracts/negates
                   1695 multiplies
                      0 divides
                    310 assignments
*/
}

void sdcomptrq(double udotin[21],
    double trqout[21])
{
/* Compute hinge torques to produce these udots, ignoring constraints
*/
    int i;
    double multin[21];

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(60,23);
        return;
    }
    for (i = 0; i < 21; i++) {
        multin[i] = 0.;
    }
    sdfulltrq(udotin,multin,trqout);
}

void sdmulttrq(double multin[21],
    double trqout[21])
{
/* Compute hinge trqs which would be produced by these mults.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(65,23);
        return;
    }
    sdmfrc(multin);
    sdfsmult();
    for (i = 0; i < 21; i++) {
        trqout[i] = fs[i];
    }
}

void sdrhs(void)
{
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

/*
Compute hinge torques for tree hinges
*/
    tauc[0] = (mtau[0]+utau[0]);
    tauc[1] = (mtau[1]+utau[1]);
    tauc[2] = (mtau[2]+utau[2]);
    tauc[3] = (mtau[3]+utau[3]);
    tauc[4] = (mtau[4]+utau[4]);
    tauc[5] = (mtau[5]+utau[5]);
    tauc[6] = (mtau[6]+utau[6]);
    tauc[7] = (mtau[7]+utau[7]);
    tauc[8] = (mtau[8]+utau[8]);
    tauc[9] = (mtau[9]+utau[9]);
    tauc[10] = (mtau[10]+utau[10]);
    tauc[11] = (mtau[11]+utau[11]);
    tauc[12] = (mtau[12]+utau[12]);
    tauc[13] = (mtau[13]+utau[13]);
    tauc[14] = (mtau[14]+utau[14]);
    tauc[15] = (mtau[15]+utau[15]);
    tauc[16] = (mtau[16]+utau[16]);
    tauc[17] = (mtau[17]+utau[17]);
    tauc[18] = (mtau[18]+utau[18]);
    tauc[19] = (mtau[19]+utau[19]);
    tauc[20] = (mtau[20]+utau[20]);
    sddoiner();
/*
Compute onk & onb (angular accels in N)
*/
    Onkb[6][0] = ((pin[6][0]*udot[6])+((Cik[6][2][0]*udot[5])+((Cik[6][0][0]*
      udot[3])+(Cik[6][1][0]*udot[4]))));
    Onkb[6][1] = ((pin[6][1]*udot[6])+((Cik[6][2][1]*udot[5])+((Cik[6][0][1]*
      udot[3])+(Cik[6][1][1]*udot[4]))));
    Onkb[6][2] = ((pin[6][2]*udot[6])+((Cik[6][2][2]*udot[5])+((Cik[6][0][2]*
      udot[3])+(Cik[6][1][2]*udot[4]))));
    Onkb[7][0] = ((pin[7][0]*udot[7])+((Cik[7][2][0]*Onkb[6][2])+((Cik[7][0][0]*
      Onkb[6][0])+(Cik[7][1][0]*Onkb[6][1]))));
    Onkb[7][1] = ((pin[7][1]*udot[7])+((Cik[7][2][1]*Onkb[6][2])+((Cik[7][0][1]*
      Onkb[6][0])+(Cik[7][1][1]*Onkb[6][1]))));
    Onkb[7][2] = ((pin[7][2]*udot[7])+((Cik[7][2][2]*Onkb[6][2])+((Cik[7][0][2]*
      Onkb[6][0])+(Cik[7][1][2]*Onkb[6][1]))));
    Onkb[8][0] = ((pin[8][0]*udot[8])+((Cik[8][2][0]*Onkb[7][2])+((Cik[8][0][0]*
      Onkb[7][0])+(Cik[8][1][0]*Onkb[7][1]))));
    Onkb[8][1] = ((pin[8][1]*udot[8])+((Cik[8][2][1]*Onkb[7][2])+((Cik[8][0][1]*
      Onkb[7][0])+(Cik[8][1][1]*Onkb[7][1]))));
    Onkb[8][2] = ((pin[8][2]*udot[8])+((Cik[8][2][2]*Onkb[7][2])+((Cik[8][0][2]*
      Onkb[7][0])+(Cik[8][1][2]*Onkb[7][1]))));
    Onkb[9][0] = ((pin[9][0]*udot[9])+((Cik[9][2][0]*udot[5])+((Cik[9][0][0]*
      udot[3])+(Cik[9][1][0]*udot[4]))));
    Onkb[9][1] = ((pin[9][1]*udot[9])+((Cik[9][2][1]*udot[5])+((Cik[9][0][1]*
      udot[3])+(Cik[9][1][1]*udot[4]))));
    Onkb[9][2] = ((pin[9][2]*udot[9])+((Cik[9][2][2]*udot[5])+((Cik[9][0][2]*
      udot[3])+(Cik[9][1][2]*udot[4]))));
    Onkb[10][0] = ((pin[10][0]*udot[10])+((Cik[10][2][0]*Onkb[9][2])+((
      Cik[10][0][0]*Onkb[9][0])+(Cik[10][1][0]*Onkb[9][1]))));
    Onkb[10][1] = ((pin[10][1]*udot[10])+((Cik[10][2][1]*Onkb[9][2])+((
      Cik[10][0][1]*Onkb[9][0])+(Cik[10][1][1]*Onkb[9][1]))));
    Onkb[10][2] = ((pin[10][2]*udot[10])+((Cik[10][2][2]*Onkb[9][2])+((
      Cik[10][0][2]*Onkb[9][0])+(Cik[10][1][2]*Onkb[9][1]))));
    Onkb[11][0] = ((pin[11][0]*udot[11])+((Cik[11][2][0]*Onkb[10][2])+((
      Cik[11][0][0]*Onkb[10][0])+(Cik[11][1][0]*Onkb[10][1]))));
    Onkb[11][1] = ((pin[11][1]*udot[11])+((Cik[11][2][1]*Onkb[10][2])+((
      Cik[11][0][1]*Onkb[10][0])+(Cik[11][1][1]*Onkb[10][1]))));
    Onkb[11][2] = ((pin[11][2]*udot[11])+((Cik[11][2][2]*Onkb[10][2])+((
      Cik[11][0][2]*Onkb[10][0])+(Cik[11][1][2]*Onkb[10][1]))));
    Onkb[12][0] = ((pin[12][0]*udot[12])+((Cik[12][2][0]*Onkb[11][2])+((
      Cik[12][0][0]*Onkb[11][0])+(Cik[12][1][0]*Onkb[11][1]))));
    Onkb[12][1] = ((pin[12][1]*udot[12])+((Cik[12][2][1]*Onkb[11][2])+((
      Cik[12][0][1]*Onkb[11][0])+(Cik[12][1][1]*Onkb[11][1]))));
    Onkb[12][2] = ((pin[12][2]*udot[12])+((Cik[12][2][2]*Onkb[11][2])+((
      Cik[12][0][2]*Onkb[11][0])+(Cik[12][1][2]*Onkb[11][1]))));
    Onkb[13][0] = ((pin[13][0]*udot[13])+((Cik[13][2][0]*Onkb[12][2])+((
      Cik[13][0][0]*Onkb[12][0])+(Cik[13][1][0]*Onkb[12][1]))));
    Onkb[13][1] = ((pin[13][1]*udot[13])+((Cik[13][2][1]*Onkb[12][2])+((
      Cik[13][0][1]*Onkb[12][0])+(Cik[13][1][1]*Onkb[12][1]))));
    Onkb[13][2] = ((pin[13][2]*udot[13])+((Cik[13][2][2]*Onkb[12][2])+((
      Cik[13][0][2]*Onkb[12][0])+(Cik[13][1][2]*Onkb[12][1]))));
    Onkb[14][0] = ((pin[14][0]*udot[14])+((Cik[14][2][0]*Onkb[13][2])+((
      Cik[14][0][0]*Onkb[13][0])+(Cik[14][1][0]*Onkb[13][1]))));
    Onkb[14][1] = ((pin[14][1]*udot[14])+((Cik[14][2][1]*Onkb[13][2])+((
      Cik[14][0][1]*Onkb[13][0])+(Cik[14][1][1]*Onkb[13][1]))));
    Onkb[14][2] = ((pin[14][2]*udot[14])+((Cik[14][2][2]*Onkb[13][2])+((
      Cik[14][0][2]*Onkb[13][0])+(Cik[14][1][2]*Onkb[13][1]))));
    Onkb[15][0] = ((pin[15][0]*udot[15])+((Cik[15][2][0]*udot[5])+((
      Cik[15][0][0]*udot[3])+(Cik[15][1][0]*udot[4]))));
    Onkb[15][1] = ((pin[15][1]*udot[15])+((Cik[15][2][1]*udot[5])+((
      Cik[15][0][1]*udot[3])+(Cik[15][1][1]*udot[4]))));
    Onkb[15][2] = ((pin[15][2]*udot[15])+((Cik[15][2][2]*udot[5])+((
      Cik[15][0][2]*udot[3])+(Cik[15][1][2]*udot[4]))));
    Onkb[16][0] = ((pin[16][0]*udot[16])+((Cik[16][2][0]*Onkb[15][2])+((
      Cik[16][0][0]*Onkb[15][0])+(Cik[16][1][0]*Onkb[15][1]))));
    Onkb[16][1] = ((pin[16][1]*udot[16])+((Cik[16][2][1]*Onkb[15][2])+((
      Cik[16][0][1]*Onkb[15][0])+(Cik[16][1][1]*Onkb[15][1]))));
    Onkb[16][2] = ((pin[16][2]*udot[16])+((Cik[16][2][2]*Onkb[15][2])+((
      Cik[16][0][2]*Onkb[15][0])+(Cik[16][1][2]*Onkb[15][1]))));
    Onkb[17][0] = ((pin[17][0]*udot[17])+((Cik[17][2][0]*Onkb[16][2])+((
      Cik[17][0][0]*Onkb[16][0])+(Cik[17][1][0]*Onkb[16][1]))));
    Onkb[17][1] = ((pin[17][1]*udot[17])+((Cik[17][2][1]*Onkb[16][2])+((
      Cik[17][0][1]*Onkb[16][0])+(Cik[17][1][1]*Onkb[16][1]))));
    Onkb[17][2] = ((pin[17][2]*udot[17])+((Cik[17][2][2]*Onkb[16][2])+((
      Cik[17][0][2]*Onkb[16][0])+(Cik[17][1][2]*Onkb[16][1]))));
    Onkb[18][0] = ((pin[18][0]*udot[18])+((Cik[18][2][0]*Onkb[17][2])+((
      Cik[18][0][0]*Onkb[17][0])+(Cik[18][1][0]*Onkb[17][1]))));
    Onkb[18][1] = ((pin[18][1]*udot[18])+((Cik[18][2][1]*Onkb[17][2])+((
      Cik[18][0][1]*Onkb[17][0])+(Cik[18][1][1]*Onkb[17][1]))));
    Onkb[18][2] = ((pin[18][2]*udot[18])+((Cik[18][2][2]*Onkb[17][2])+((
      Cik[18][0][2]*Onkb[17][0])+(Cik[18][1][2]*Onkb[17][1]))));
    Onkb[19][0] = ((pin[19][0]*udot[19])+((Cik[19][2][0]*Onkb[18][2])+((
      Cik[19][0][0]*Onkb[18][0])+(Cik[19][1][0]*Onkb[18][1]))));
    Onkb[19][1] = ((pin[19][1]*udot[19])+((Cik[19][2][1]*Onkb[18][2])+((
      Cik[19][0][1]*Onkb[18][0])+(Cik[19][1][1]*Onkb[18][1]))));
    Onkb[19][2] = ((pin[19][2]*udot[19])+((Cik[19][2][2]*Onkb[18][2])+((
      Cik[19][0][2]*Onkb[18][0])+(Cik[19][1][2]*Onkb[18][1]))));
    Onkb[20][0] = ((pin[20][0]*udot[20])+((Cik[20][2][0]*Onkb[19][2])+((
      Cik[20][0][0]*Onkb[19][0])+(Cik[20][1][0]*Onkb[19][1]))));
    Onkb[20][1] = ((pin[20][1]*udot[20])+((Cik[20][2][1]*Onkb[19][2])+((
      Cik[20][0][1]*Onkb[19][0])+(Cik[20][1][1]*Onkb[19][1]))));
    Onkb[20][2] = ((pin[20][2]*udot[20])+((Cik[20][2][2]*Onkb[19][2])+((
      Cik[20][0][2]*Onkb[19][0])+(Cik[20][1][2]*Onkb[19][1]))));
    onk[6][0] = (Onkb[6][0]+Otk[6][0]);
    onk[6][1] = (Onkb[6][1]+Otk[6][1]);
    onk[6][2] = (Onkb[6][2]+Otk[6][2]);
    onk[7][0] = (Onkb[7][0]+Otk[7][0]);
    onk[7][1] = (Onkb[7][1]+Otk[7][1]);
    onk[7][2] = (Onkb[7][2]+Otk[7][2]);
    onk[8][0] = (Onkb[8][0]+Otk[8][0]);
    onk[8][1] = (Onkb[8][1]+Otk[8][1]);
    onk[8][2] = (Onkb[8][2]+Otk[8][2]);
    onk[9][0] = (Onkb[9][0]+Otk[9][0]);
    onk[9][1] = (Onkb[9][1]+Otk[9][1]);
    onk[9][2] = (Onkb[9][2]+Otk[9][2]);
    onk[10][0] = (Onkb[10][0]+Otk[10][0]);
    onk[10][1] = (Onkb[10][1]+Otk[10][1]);
    onk[10][2] = (Onkb[10][2]+Otk[10][2]);
    onk[11][0] = (Onkb[11][0]+Otk[11][0]);
    onk[11][1] = (Onkb[11][1]+Otk[11][1]);
    onk[11][2] = (Onkb[11][2]+Otk[11][2]);
    onk[12][0] = (Onkb[12][0]+Otk[12][0]);
    onk[12][1] = (Onkb[12][1]+Otk[12][1]);
    onk[12][2] = (Onkb[12][2]+Otk[12][2]);
    onk[13][0] = (Onkb[13][0]+Otk[13][0]);
    onk[13][1] = (Onkb[13][1]+Otk[13][1]);
    onk[13][2] = (Onkb[13][2]+Otk[13][2]);
    onk[14][0] = (Onkb[14][0]+Otk[14][0]);
    onk[14][1] = (Onkb[14][1]+Otk[14][1]);
    onk[14][2] = (Onkb[14][2]+Otk[14][2]);
    onk[15][0] = (Onkb[15][0]+Otk[15][0]);
    onk[15][1] = (Onkb[15][1]+Otk[15][1]);
    onk[15][2] = (Onkb[15][2]+Otk[15][2]);
    onk[16][0] = (Onkb[16][0]+Otk[16][0]);
    onk[16][1] = (Onkb[16][1]+Otk[16][1]);
    onk[16][2] = (Onkb[16][2]+Otk[16][2]);
    onk[17][0] = (Onkb[17][0]+Otk[17][0]);
    onk[17][1] = (Onkb[17][1]+Otk[17][1]);
    onk[17][2] = (Onkb[17][2]+Otk[17][2]);
    onk[18][0] = (Onkb[18][0]+Otk[18][0]);
    onk[18][1] = (Onkb[18][1]+Otk[18][1]);
    onk[18][2] = (Onkb[18][2]+Otk[18][2]);
    onk[19][0] = (Onkb[19][0]+Otk[19][0]);
    onk[19][1] = (Onkb[19][1]+Otk[19][1]);
    onk[19][2] = (Onkb[19][2]+Otk[19][2]);
    onk[20][0] = (Onkb[20][0]+Otk[20][0]);
    onk[20][1] = (Onkb[20][1]+Otk[20][1]);
    onk[20][2] = (Onkb[20][2]+Otk[20][2]);
    onb[0][0] = udot[3];
    onb[0][1] = udot[4];
    onb[0][2] = udot[5];
    onb[1][0] = onk[6][0];
    onb[1][1] = onk[6][1];
    onb[1][2] = onk[6][2];
    onb[2][0] = onk[7][0];
    onb[2][1] = onk[7][1];
    onb[2][2] = onk[7][2];
    onb[3][0] = onk[8][0];
    onb[3][1] = onk[8][1];
    onb[3][2] = onk[8][2];
    onb[4][0] = onk[9][0];
    onb[4][1] = onk[9][1];
    onb[4][2] = onk[9][2];
    onb[5][0] = onk[10][0];
    onb[5][1] = onk[10][1];
    onb[5][2] = onk[10][2];
    onb[6][0] = onk[11][0];
    onb[6][1] = onk[11][1];
    onb[6][2] = onk[11][2];
    onb[7][0] = onk[12][0];
    onb[7][1] = onk[12][1];
    onb[7][2] = onk[12][2];
    onb[8][0] = onk[13][0];
    onb[8][1] = onk[13][1];
    onb[8][2] = onk[13][2];
    onb[9][0] = onk[14][0];
    onb[9][1] = onk[14][1];
    onb[9][2] = onk[14][2];
    onb[10][0] = onk[15][0];
    onb[10][1] = onk[15][1];
    onb[10][2] = onk[15][2];
    onb[11][0] = onk[16][0];
    onb[11][1] = onk[16][1];
    onb[11][2] = onk[16][2];
    onb[12][0] = onk[17][0];
    onb[12][1] = onk[17][1];
    onb[12][2] = onk[17][2];
    onb[13][0] = onk[18][0];
    onb[13][1] = onk[18][1];
    onb[13][2] = onk[18][2];
    onb[14][0] = onk[19][0];
    onb[14][1] = onk[19][1];
    onb[14][2] = onk[19][2];
    onb[15][0] = onk[20][0];
    onb[15][1] = onk[20][1];
    onb[15][2] = onk[20][2];
/*
Compute acceleration dyadics
*/
    dyad[0][0][0] = w11w22[0];
    dyad[0][0][1] = (w0w1[0]-udot[5]);
    dyad[0][0][2] = (udot[4]+w0w2[0]);
    dyad[0][1][0] = (udot[5]+w0w1[0]);
    dyad[0][1][1] = w00w22[0];
    dyad[0][1][2] = (w1w2[0]-udot[3]);
    dyad[0][2][0] = (w0w2[0]-udot[4]);
    dyad[0][2][1] = (udot[3]+w1w2[0]);
    dyad[0][2][2] = w00w11[0];
    dyad[1][0][0] = w11w22[1];
    dyad[1][0][1] = (w0w1[1]-onk[6][2]);
    dyad[1][0][2] = (onk[6][1]+w0w2[1]);
    dyad[1][1][0] = (onk[6][2]+w0w1[1]);
    dyad[1][1][1] = w00w22[1];
    dyad[1][1][2] = (w1w2[1]-onk[6][0]);
    dyad[1][2][0] = (w0w2[1]-onk[6][1]);
    dyad[1][2][1] = (onk[6][0]+w1w2[1]);
    dyad[1][2][2] = w00w11[1];
    dyad[2][0][0] = w11w22[2];
    dyad[2][0][1] = (w0w1[2]-onk[7][2]);
    dyad[2][0][2] = (onk[7][1]+w0w2[2]);
    dyad[2][1][0] = (onk[7][2]+w0w1[2]);
    dyad[2][1][1] = w00w22[2];
    dyad[2][1][2] = (w1w2[2]-onk[7][0]);
    dyad[2][2][0] = (w0w2[2]-onk[7][1]);
    dyad[2][2][1] = (onk[7][0]+w1w2[2]);
    dyad[2][2][2] = w00w11[2];
    dyad[3][0][0] = w11w22[3];
    dyad[3][0][1] = (w0w1[3]-onk[8][2]);
    dyad[3][0][2] = (onk[8][1]+w0w2[3]);
    dyad[3][1][0] = (onk[8][2]+w0w1[3]);
    dyad[3][1][1] = w00w22[3];
    dyad[3][1][2] = (w1w2[3]-onk[8][0]);
    dyad[3][2][0] = (w0w2[3]-onk[8][1]);
    dyad[3][2][1] = (onk[8][0]+w1w2[3]);
    dyad[3][2][2] = w00w11[3];
    dyad[4][0][0] = w11w22[4];
    dyad[4][0][1] = (w0w1[4]-onk[9][2]);
    dyad[4][0][2] = (onk[9][1]+w0w2[4]);
    dyad[4][1][0] = (onk[9][2]+w0w1[4]);
    dyad[4][1][1] = w00w22[4];
    dyad[4][1][2] = (w1w2[4]-onk[9][0]);
    dyad[4][2][0] = (w0w2[4]-onk[9][1]);
    dyad[4][2][1] = (onk[9][0]+w1w2[4]);
    dyad[4][2][2] = w00w11[4];
    dyad[5][0][0] = w11w22[5];
    dyad[5][0][1] = (w0w1[5]-onk[10][2]);
    dyad[5][0][2] = (onk[10][1]+w0w2[5]);
    dyad[5][1][0] = (onk[10][2]+w0w1[5]);
    dyad[5][1][1] = w00w22[5];
    dyad[5][1][2] = (w1w2[5]-onk[10][0]);
    dyad[5][2][0] = (w0w2[5]-onk[10][1]);
    dyad[5][2][1] = (onk[10][0]+w1w2[5]);
    dyad[5][2][2] = w00w11[5];
    dyad[6][0][0] = w11w22[6];
    dyad[6][0][1] = (w0w1[6]-onk[11][2]);
    dyad[6][0][2] = (onk[11][1]+w0w2[6]);
    dyad[6][1][0] = (onk[11][2]+w0w1[6]);
    dyad[6][1][1] = w00w22[6];
    dyad[6][1][2] = (w1w2[6]-onk[11][0]);
    dyad[6][2][0] = (w0w2[6]-onk[11][1]);
    dyad[6][2][1] = (onk[11][0]+w1w2[6]);
    dyad[6][2][2] = w00w11[6];
    dyad[7][0][0] = w11w22[7];
    dyad[7][0][1] = (w0w1[7]-onk[12][2]);
    dyad[7][0][2] = (onk[12][1]+w0w2[7]);
    dyad[7][1][0] = (onk[12][2]+w0w1[7]);
    dyad[7][1][1] = w00w22[7];
    dyad[7][1][2] = (w1w2[7]-onk[12][0]);
    dyad[7][2][0] = (w0w2[7]-onk[12][1]);
    dyad[7][2][1] = (onk[12][0]+w1w2[7]);
    dyad[7][2][2] = w00w11[7];
    dyad[8][0][0] = w11w22[8];
    dyad[8][0][1] = (w0w1[8]-onk[13][2]);
    dyad[8][0][2] = (onk[13][1]+w0w2[8]);
    dyad[8][1][0] = (onk[13][2]+w0w1[8]);
    dyad[8][1][1] = w00w22[8];
    dyad[8][1][2] = (w1w2[8]-onk[13][0]);
    dyad[8][2][0] = (w0w2[8]-onk[13][1]);
    dyad[8][2][1] = (onk[13][0]+w1w2[8]);
    dyad[8][2][2] = w00w11[8];
    dyad[9][0][0] = w11w22[9];
    dyad[9][0][1] = (w0w1[9]-onk[14][2]);
    dyad[9][0][2] = (onk[14][1]+w0w2[9]);
    dyad[9][1][0] = (onk[14][2]+w0w1[9]);
    dyad[9][1][1] = w00w22[9];
    dyad[9][1][2] = (w1w2[9]-onk[14][0]);
    dyad[9][2][0] = (w0w2[9]-onk[14][1]);
    dyad[9][2][1] = (onk[14][0]+w1w2[9]);
    dyad[9][2][2] = w00w11[9];
    dyad[10][0][0] = w11w22[10];
    dyad[10][0][1] = (w0w1[10]-onk[15][2]);
    dyad[10][0][2] = (onk[15][1]+w0w2[10]);
    dyad[10][1][0] = (onk[15][2]+w0w1[10]);
    dyad[10][1][1] = w00w22[10];
    dyad[10][1][2] = (w1w2[10]-onk[15][0]);
    dyad[10][2][0] = (w0w2[10]-onk[15][1]);
    dyad[10][2][1] = (onk[15][0]+w1w2[10]);
    dyad[10][2][2] = w00w11[10];
    dyad[11][0][0] = w11w22[11];
    dyad[11][0][1] = (w0w1[11]-onk[16][2]);
    dyad[11][0][2] = (onk[16][1]+w0w2[11]);
    dyad[11][1][0] = (onk[16][2]+w0w1[11]);
    dyad[11][1][1] = w00w22[11];
    dyad[11][1][2] = (w1w2[11]-onk[16][0]);
    dyad[11][2][0] = (w0w2[11]-onk[16][1]);
    dyad[11][2][1] = (onk[16][0]+w1w2[11]);
    dyad[11][2][2] = w00w11[11];
    dyad[12][0][0] = w11w22[12];
    dyad[12][0][1] = (w0w1[12]-onk[17][2]);
    dyad[12][0][2] = (onk[17][1]+w0w2[12]);
    dyad[12][1][0] = (onk[17][2]+w0w1[12]);
    dyad[12][1][1] = w00w22[12];
    dyad[12][1][2] = (w1w2[12]-onk[17][0]);
    dyad[12][2][0] = (w0w2[12]-onk[17][1]);
    dyad[12][2][1] = (onk[17][0]+w1w2[12]);
    dyad[12][2][2] = w00w11[12];
    dyad[13][0][0] = w11w22[13];
    dyad[13][0][1] = (w0w1[13]-onk[18][2]);
    dyad[13][0][2] = (onk[18][1]+w0w2[13]);
    dyad[13][1][0] = (onk[18][2]+w0w1[13]);
    dyad[13][1][1] = w00w22[13];
    dyad[13][1][2] = (w1w2[13]-onk[18][0]);
    dyad[13][2][0] = (w0w2[13]-onk[18][1]);
    dyad[13][2][1] = (onk[18][0]+w1w2[13]);
    dyad[13][2][2] = w00w11[13];
    dyad[14][0][0] = w11w22[14];
    dyad[14][0][1] = (w0w1[14]-onk[19][2]);
    dyad[14][0][2] = (onk[19][1]+w0w2[14]);
    dyad[14][1][0] = (onk[19][2]+w0w1[14]);
    dyad[14][1][1] = w00w22[14];
    dyad[14][1][2] = (w1w2[14]-onk[19][0]);
    dyad[14][2][0] = (w0w2[14]-onk[19][1]);
    dyad[14][2][1] = (onk[19][0]+w1w2[14]);
    dyad[14][2][2] = w00w11[14];
    dyad[15][0][0] = w11w22[15];
    dyad[15][0][1] = (w0w1[15]-onk[20][2]);
    dyad[15][0][2] = (onk[20][1]+w0w2[15]);
    dyad[15][1][0] = (onk[20][2]+w0w1[15]);
    dyad[15][1][1] = w00w22[15];
    dyad[15][1][2] = (w1w2[15]-onk[20][0]);
    dyad[15][2][0] = (w0w2[15]-onk[20][1]);
    dyad[15][2][1] = (onk[20][0]+w1w2[15]);
    dyad[15][2][2] = w00w11[15];
/*
Compute ank & anb (mass center linear accels in N)
*/
    Ankb[0][0] = (pin[0][0]*udot[0]);
    Ankb[0][1] = (pin[0][1]*udot[0]);
    Ankb[0][2] = (pin[0][2]*udot[0]);
    Ankb[1][0] = (Ankb[0][0]+(pin[1][0]*udot[1]));
    Ankb[1][1] = (Ankb[0][1]+(pin[1][1]*udot[1]));
    Ankb[1][2] = (Ankb[0][2]+(pin[1][2]*udot[1]));
    Ankb[2][0] = (Ankb[1][0]+(pin[2][0]*udot[2]));
    Ankb[2][1] = (Ankb[1][1]+(pin[2][1]*udot[2]));
    Ankb[2][2] = (Ankb[1][2]+(pin[2][2]*udot[2]));
    Ankb[3][0] = ((Ankb[2][2]*Cik[3][2][0])+((Ankb[2][0]*Cik[3][0][0])+(
      Ankb[2][1]*Cik[3][1][0])));
    Ankb[3][1] = ((Ankb[2][2]*Cik[3][2][1])+((Ankb[2][0]*Cik[3][0][1])+(
      Ankb[2][1]*Cik[3][1][1])));
    Ankb[3][2] = ((Ankb[2][2]*Cik[3][2][2])+((Ankb[2][0]*Cik[3][0][2])+(
      Ankb[2][1]*Cik[3][1][2])));
    Ankb[5][0] = (Ankb[3][0]+((rk[0][1]*udot[5])-(rk[0][2]*udot[4])));
    Ankb[5][1] = (Ankb[3][1]+((rk[0][2]*udot[3])-(rk[0][0]*udot[5])));
    Ankb[5][2] = (Ankb[3][2]+((rk[0][0]*udot[4])-(rk[0][1]*udot[3])));
    AOnkri[6][0] = (Ankb[5][0]+((ri[1][2]*udot[4])-(ri[1][1]*udot[5])));
    AOnkri[6][1] = (Ankb[5][1]+((ri[1][0]*udot[5])-(ri[1][2]*udot[3])));
    AOnkri[6][2] = (Ankb[5][2]+((ri[1][1]*udot[3])-(ri[1][0]*udot[4])));
    Ankb[6][0] = (((AOnkri[6][2]*Cik[6][2][0])+((AOnkri[6][0]*Cik[6][0][0])+(
      AOnkri[6][1]*Cik[6][1][0])))+((Onkb[6][2]*rk[1][1])-(Onkb[6][1]*rk[1][2]))
      );
    Ankb[6][1] = (((AOnkri[6][2]*Cik[6][2][1])+((AOnkri[6][0]*Cik[6][0][1])+(
      AOnkri[6][1]*Cik[6][1][1])))+((Onkb[6][0]*rk[1][2])-(Onkb[6][2]*rk[1][0]))
      );
    Ankb[6][2] = (((AOnkri[6][2]*Cik[6][2][2])+((AOnkri[6][0]*Cik[6][0][2])+(
      AOnkri[6][1]*Cik[6][1][2])))+((Onkb[6][1]*rk[1][0])-(Onkb[6][0]*rk[1][1]))
      );
    AOnkri[7][0] = (Ankb[6][0]+((Onkb[6][1]*ri[2][2])-(Onkb[6][2]*ri[2][1])));
    AOnkri[7][1] = (Ankb[6][1]+((Onkb[6][2]*ri[2][0])-(Onkb[6][0]*ri[2][2])));
    AOnkri[7][2] = (Ankb[6][2]+((Onkb[6][0]*ri[2][1])-(Onkb[6][1]*ri[2][0])));
    Ankb[7][0] = (((AOnkri[7][2]*Cik[7][2][0])+((AOnkri[7][0]*Cik[7][0][0])+(
      AOnkri[7][1]*Cik[7][1][0])))+((Onkb[7][2]*rk[2][1])-(Onkb[7][1]*rk[2][2]))
      );
    Ankb[7][1] = (((AOnkri[7][2]*Cik[7][2][1])+((AOnkri[7][0]*Cik[7][0][1])+(
      AOnkri[7][1]*Cik[7][1][1])))+((Onkb[7][0]*rk[2][2])-(Onkb[7][2]*rk[2][0]))
      );
    Ankb[7][2] = (((AOnkri[7][2]*Cik[7][2][2])+((AOnkri[7][0]*Cik[7][0][2])+(
      AOnkri[7][1]*Cik[7][1][2])))+((Onkb[7][1]*rk[2][0])-(Onkb[7][0]*rk[2][1]))
      );
    AOnkri[8][0] = (Ankb[7][0]+((Onkb[7][1]*ri[3][2])-(Onkb[7][2]*ri[3][1])));
    AOnkri[8][1] = (Ankb[7][1]+((Onkb[7][2]*ri[3][0])-(Onkb[7][0]*ri[3][2])));
    AOnkri[8][2] = (Ankb[7][2]+((Onkb[7][0]*ri[3][1])-(Onkb[7][1]*ri[3][0])));
    Ankb[8][0] = (((AOnkri[8][2]*Cik[8][2][0])+((AOnkri[8][0]*Cik[8][0][0])+(
      AOnkri[8][1]*Cik[8][1][0])))+((Onkb[8][2]*rk[3][1])-(Onkb[8][1]*rk[3][2]))
      );
    Ankb[8][1] = (((AOnkri[8][2]*Cik[8][2][1])+((AOnkri[8][0]*Cik[8][0][1])+(
      AOnkri[8][1]*Cik[8][1][1])))+((Onkb[8][0]*rk[3][2])-(Onkb[8][2]*rk[3][0]))
      );
    Ankb[8][2] = (((AOnkri[8][2]*Cik[8][2][2])+((AOnkri[8][0]*Cik[8][0][2])+(
      AOnkri[8][1]*Cik[8][1][2])))+((Onkb[8][1]*rk[3][0])-(Onkb[8][0]*rk[3][1]))
      );
    AOnkri[9][0] = (Ankb[5][0]+((ri[4][2]*udot[4])-(ri[4][1]*udot[5])));
    AOnkri[9][1] = (Ankb[5][1]+((ri[4][0]*udot[5])-(ri[4][2]*udot[3])));
    AOnkri[9][2] = (Ankb[5][2]+((ri[4][1]*udot[3])-(ri[4][0]*udot[4])));
    Ankb[9][0] = (((AOnkri[9][2]*Cik[9][2][0])+((AOnkri[9][0]*Cik[9][0][0])+(
      AOnkri[9][1]*Cik[9][1][0])))+((Onkb[9][2]*rk[4][1])-(Onkb[9][1]*rk[4][2]))
      );
    Ankb[9][1] = (((AOnkri[9][2]*Cik[9][2][1])+((AOnkri[9][0]*Cik[9][0][1])+(
      AOnkri[9][1]*Cik[9][1][1])))+((Onkb[9][0]*rk[4][2])-(Onkb[9][2]*rk[4][0]))
      );
    Ankb[9][2] = (((AOnkri[9][2]*Cik[9][2][2])+((AOnkri[9][0]*Cik[9][0][2])+(
      AOnkri[9][1]*Cik[9][1][2])))+((Onkb[9][1]*rk[4][0])-(Onkb[9][0]*rk[4][1]))
      );
    AOnkri[10][0] = (Ankb[9][0]+((Onkb[9][1]*ri[5][2])-(Onkb[9][2]*ri[5][1])));
    AOnkri[10][1] = (Ankb[9][1]+((Onkb[9][2]*ri[5][0])-(Onkb[9][0]*ri[5][2])));
    AOnkri[10][2] = (Ankb[9][2]+((Onkb[9][0]*ri[5][1])-(Onkb[9][1]*ri[5][0])));
    Ankb[10][0] = (((AOnkri[10][2]*Cik[10][2][0])+((AOnkri[10][0]*Cik[10][0][0])
      +(AOnkri[10][1]*Cik[10][1][0])))+((Onkb[10][2]*rk[5][1])-(Onkb[10][1]*
      rk[5][2])));
    Ankb[10][1] = (((AOnkri[10][2]*Cik[10][2][1])+((AOnkri[10][0]*Cik[10][0][1])
      +(AOnkri[10][1]*Cik[10][1][1])))+((Onkb[10][0]*rk[5][2])-(Onkb[10][2]*
      rk[5][0])));
    Ankb[10][2] = (((AOnkri[10][2]*Cik[10][2][2])+((AOnkri[10][0]*Cik[10][0][2])
      +(AOnkri[10][1]*Cik[10][1][2])))+((Onkb[10][1]*rk[5][0])-(Onkb[10][0]*
      rk[5][1])));
    AOnkri[11][0] = (Ankb[10][0]+((Onkb[10][1]*ri[6][2])-(Onkb[10][2]*ri[6][1]))
      );
    AOnkri[11][1] = (Ankb[10][1]+((Onkb[10][2]*ri[6][0])-(Onkb[10][0]*ri[6][2]))
      );
    AOnkri[11][2] = (Ankb[10][2]+((Onkb[10][0]*ri[6][1])-(Onkb[10][1]*ri[6][0]))
      );
    Ankb[11][0] = (((AOnkri[11][2]*Cik[11][2][0])+((AOnkri[11][0]*Cik[11][0][0])
      +(AOnkri[11][1]*Cik[11][1][0])))+((Onkb[11][2]*rk[6][1])-(Onkb[11][1]*
      rk[6][2])));
    Ankb[11][1] = (((AOnkri[11][2]*Cik[11][2][1])+((AOnkri[11][0]*Cik[11][0][1])
      +(AOnkri[11][1]*Cik[11][1][1])))+((Onkb[11][0]*rk[6][2])-(Onkb[11][2]*
      rk[6][0])));
    Ankb[11][2] = (((AOnkri[11][2]*Cik[11][2][2])+((AOnkri[11][0]*Cik[11][0][2])
      +(AOnkri[11][1]*Cik[11][1][2])))+((Onkb[11][1]*rk[6][0])-(Onkb[11][0]*
      rk[6][1])));
    AOnkri[12][0] = (Ankb[11][0]+((Onkb[11][1]*ri[7][2])-(Onkb[11][2]*ri[7][1]))
      );
    AOnkri[12][1] = (Ankb[11][1]+((Onkb[11][2]*ri[7][0])-(Onkb[11][0]*ri[7][2]))
      );
    AOnkri[12][2] = (Ankb[11][2]+((Onkb[11][0]*ri[7][1])-(Onkb[11][1]*ri[7][0]))
      );
    Ankb[12][0] = (((AOnkri[12][2]*Cik[12][2][0])+((AOnkri[12][0]*Cik[12][0][0])
      +(AOnkri[12][1]*Cik[12][1][0])))+((Onkb[12][2]*rk[7][1])-(Onkb[12][1]*
      rk[7][2])));
    Ankb[12][1] = (((AOnkri[12][2]*Cik[12][2][1])+((AOnkri[12][0]*Cik[12][0][1])
      +(AOnkri[12][1]*Cik[12][1][1])))+((Onkb[12][0]*rk[7][2])-(Onkb[12][2]*
      rk[7][0])));
    Ankb[12][2] = (((AOnkri[12][2]*Cik[12][2][2])+((AOnkri[12][0]*Cik[12][0][2])
      +(AOnkri[12][1]*Cik[12][1][2])))+((Onkb[12][1]*rk[7][0])-(Onkb[12][0]*
      rk[7][1])));
    AOnkri[13][0] = (Ankb[12][0]+((Onkb[12][1]*ri[8][2])-(Onkb[12][2]*ri[8][1]))
      );
    AOnkri[13][1] = (Ankb[12][1]+((Onkb[12][2]*ri[8][0])-(Onkb[12][0]*ri[8][2]))
      );
    AOnkri[13][2] = (Ankb[12][2]+((Onkb[12][0]*ri[8][1])-(Onkb[12][1]*ri[8][0]))
      );
    Ankb[13][0] = (((AOnkri[13][2]*Cik[13][2][0])+((AOnkri[13][0]*Cik[13][0][0])
      +(AOnkri[13][1]*Cik[13][1][0])))+((Onkb[13][2]*rk[8][1])-(Onkb[13][1]*
      rk[8][2])));
    Ankb[13][1] = (((AOnkri[13][2]*Cik[13][2][1])+((AOnkri[13][0]*Cik[13][0][1])
      +(AOnkri[13][1]*Cik[13][1][1])))+((Onkb[13][0]*rk[8][2])-(Onkb[13][2]*
      rk[8][0])));
    Ankb[13][2] = (((AOnkri[13][2]*Cik[13][2][2])+((AOnkri[13][0]*Cik[13][0][2])
      +(AOnkri[13][1]*Cik[13][1][2])))+((Onkb[13][1]*rk[8][0])-(Onkb[13][0]*
      rk[8][1])));
    AOnkri[14][0] = (Ankb[13][0]+((Onkb[13][1]*ri[9][2])-(Onkb[13][2]*ri[9][1]))
      );
    AOnkri[14][1] = (Ankb[13][1]+((Onkb[13][2]*ri[9][0])-(Onkb[13][0]*ri[9][2]))
      );
    AOnkri[14][2] = (Ankb[13][2]+((Onkb[13][0]*ri[9][1])-(Onkb[13][1]*ri[9][0]))
      );
    Ankb[14][0] = (((AOnkri[14][2]*Cik[14][2][0])+((AOnkri[14][0]*Cik[14][0][0])
      +(AOnkri[14][1]*Cik[14][1][0])))+((Onkb[14][2]*rk[9][1])-(Onkb[14][1]*
      rk[9][2])));
    Ankb[14][1] = (((AOnkri[14][2]*Cik[14][2][1])+((AOnkri[14][0]*Cik[14][0][1])
      +(AOnkri[14][1]*Cik[14][1][1])))+((Onkb[14][0]*rk[9][2])-(Onkb[14][2]*
      rk[9][0])));
    Ankb[14][2] = (((AOnkri[14][2]*Cik[14][2][2])+((AOnkri[14][0]*Cik[14][0][2])
      +(AOnkri[14][1]*Cik[14][1][2])))+((Onkb[14][1]*rk[9][0])-(Onkb[14][0]*
      rk[9][1])));
    AOnkri[15][0] = (Ankb[5][0]+((ri[10][2]*udot[4])-(ri[10][1]*udot[5])));
    AOnkri[15][1] = (Ankb[5][1]+((ri[10][0]*udot[5])-(ri[10][2]*udot[3])));
    AOnkri[15][2] = (Ankb[5][2]+((ri[10][1]*udot[3])-(ri[10][0]*udot[4])));
    Ankb[15][0] = (((AOnkri[15][2]*Cik[15][2][0])+((AOnkri[15][0]*Cik[15][0][0])
      +(AOnkri[15][1]*Cik[15][1][0])))+((Onkb[15][2]*rk[10][1])-(Onkb[15][1]*
      rk[10][2])));
    Ankb[15][1] = (((AOnkri[15][2]*Cik[15][2][1])+((AOnkri[15][0]*Cik[15][0][1])
      +(AOnkri[15][1]*Cik[15][1][1])))+((Onkb[15][0]*rk[10][2])-(Onkb[15][2]*
      rk[10][0])));
    Ankb[15][2] = (((AOnkri[15][2]*Cik[15][2][2])+((AOnkri[15][0]*Cik[15][0][2])
      +(AOnkri[15][1]*Cik[15][1][2])))+((Onkb[15][1]*rk[10][0])-(Onkb[15][0]*
      rk[10][1])));
    AOnkri[16][0] = (Ankb[15][0]+((Onkb[15][1]*ri[11][2])-(Onkb[15][2]*ri[11][1]
      )));
    AOnkri[16][1] = (Ankb[15][1]+((Onkb[15][2]*ri[11][0])-(Onkb[15][0]*ri[11][2]
      )));
    AOnkri[16][2] = (Ankb[15][2]+((Onkb[15][0]*ri[11][1])-(Onkb[15][1]*ri[11][0]
      )));
    Ankb[16][0] = (((AOnkri[16][2]*Cik[16][2][0])+((AOnkri[16][0]*Cik[16][0][0])
      +(AOnkri[16][1]*Cik[16][1][0])))+((Onkb[16][2]*rk[11][1])-(Onkb[16][1]*
      rk[11][2])));
    Ankb[16][1] = (((AOnkri[16][2]*Cik[16][2][1])+((AOnkri[16][0]*Cik[16][0][1])
      +(AOnkri[16][1]*Cik[16][1][1])))+((Onkb[16][0]*rk[11][2])-(Onkb[16][2]*
      rk[11][0])));
    Ankb[16][2] = (((AOnkri[16][2]*Cik[16][2][2])+((AOnkri[16][0]*Cik[16][0][2])
      +(AOnkri[16][1]*Cik[16][1][2])))+((Onkb[16][1]*rk[11][0])-(Onkb[16][0]*
      rk[11][1])));
    AOnkri[17][0] = (Ankb[16][0]+((Onkb[16][1]*ri[12][2])-(Onkb[16][2]*ri[12][1]
      )));
    AOnkri[17][1] = (Ankb[16][1]+((Onkb[16][2]*ri[12][0])-(Onkb[16][0]*ri[12][2]
      )));
    AOnkri[17][2] = (Ankb[16][2]+((Onkb[16][0]*ri[12][1])-(Onkb[16][1]*ri[12][0]
      )));
    Ankb[17][0] = (((AOnkri[17][2]*Cik[17][2][0])+((AOnkri[17][0]*Cik[17][0][0])
      +(AOnkri[17][1]*Cik[17][1][0])))+((Onkb[17][2]*rk[12][1])-(Onkb[17][1]*
      rk[12][2])));
    Ankb[17][1] = (((AOnkri[17][2]*Cik[17][2][1])+((AOnkri[17][0]*Cik[17][0][1])
      +(AOnkri[17][1]*Cik[17][1][1])))+((Onkb[17][0]*rk[12][2])-(Onkb[17][2]*
      rk[12][0])));
    Ankb[17][2] = (((AOnkri[17][2]*Cik[17][2][2])+((AOnkri[17][0]*Cik[17][0][2])
      +(AOnkri[17][1]*Cik[17][1][2])))+((Onkb[17][1]*rk[12][0])-(Onkb[17][0]*
      rk[12][1])));
    AOnkri[18][0] = (Ankb[17][0]+((Onkb[17][1]*ri[13][2])-(Onkb[17][2]*ri[13][1]
      )));
    AOnkri[18][1] = (Ankb[17][1]+((Onkb[17][2]*ri[13][0])-(Onkb[17][0]*ri[13][2]
      )));
    AOnkri[18][2] = (Ankb[17][2]+((Onkb[17][0]*ri[13][1])-(Onkb[17][1]*ri[13][0]
      )));
    Ankb[18][0] = (((AOnkri[18][2]*Cik[18][2][0])+((AOnkri[18][0]*Cik[18][0][0])
      +(AOnkri[18][1]*Cik[18][1][0])))+((Onkb[18][2]*rk[13][1])-(Onkb[18][1]*
      rk[13][2])));
    Ankb[18][1] = (((AOnkri[18][2]*Cik[18][2][1])+((AOnkri[18][0]*Cik[18][0][1])
      +(AOnkri[18][1]*Cik[18][1][1])))+((Onkb[18][0]*rk[13][2])-(Onkb[18][2]*
      rk[13][0])));
    Ankb[18][2] = (((AOnkri[18][2]*Cik[18][2][2])+((AOnkri[18][0]*Cik[18][0][2])
      +(AOnkri[18][1]*Cik[18][1][2])))+((Onkb[18][1]*rk[13][0])-(Onkb[18][0]*
      rk[13][1])));
    AOnkri[19][0] = (Ankb[18][0]+((Onkb[18][1]*ri[14][2])-(Onkb[18][2]*ri[14][1]
      )));
    AOnkri[19][1] = (Ankb[18][1]+((Onkb[18][2]*ri[14][0])-(Onkb[18][0]*ri[14][2]
      )));
    AOnkri[19][2] = (Ankb[18][2]+((Onkb[18][0]*ri[14][1])-(Onkb[18][1]*ri[14][0]
      )));
    Ankb[19][0] = (((AOnkri[19][2]*Cik[19][2][0])+((AOnkri[19][0]*Cik[19][0][0])
      +(AOnkri[19][1]*Cik[19][1][0])))+((Onkb[19][2]*rk[14][1])-(Onkb[19][1]*
      rk[14][2])));
    Ankb[19][1] = (((AOnkri[19][2]*Cik[19][2][1])+((AOnkri[19][0]*Cik[19][0][1])
      +(AOnkri[19][1]*Cik[19][1][1])))+((Onkb[19][0]*rk[14][2])-(Onkb[19][2]*
      rk[14][0])));
    Ankb[19][2] = (((AOnkri[19][2]*Cik[19][2][2])+((AOnkri[19][0]*Cik[19][0][2])
      +(AOnkri[19][1]*Cik[19][1][2])))+((Onkb[19][1]*rk[14][0])-(Onkb[19][0]*
      rk[14][1])));
    AOnkri[20][0] = (Ankb[19][0]+((Onkb[19][1]*ri[15][2])-(Onkb[19][2]*ri[15][1]
      )));
    AOnkri[20][1] = (Ankb[19][1]+((Onkb[19][2]*ri[15][0])-(Onkb[19][0]*ri[15][2]
      )));
    AOnkri[20][2] = (Ankb[19][2]+((Onkb[19][0]*ri[15][1])-(Onkb[19][1]*ri[15][0]
      )));
    Ankb[20][0] = (((AOnkri[20][2]*Cik[20][2][0])+((AOnkri[20][0]*Cik[20][0][0])
      +(AOnkri[20][1]*Cik[20][1][0])))+((Onkb[20][2]*rk[15][1])-(Onkb[20][1]*
      rk[15][2])));
    Ankb[20][1] = (((AOnkri[20][2]*Cik[20][2][1])+((AOnkri[20][0]*Cik[20][0][1])
      +(AOnkri[20][1]*Cik[20][1][1])))+((Onkb[20][0]*rk[15][2])-(Onkb[20][2]*
      rk[15][0])));
    Ankb[20][2] = (((AOnkri[20][2]*Cik[20][2][2])+((AOnkri[20][0]*Cik[20][0][2])
      +(AOnkri[20][1]*Cik[20][1][2])))+((Onkb[20][1]*rk[15][0])-(Onkb[20][0]*
      rk[15][1])));
    AnkAtk[5][0] = (Ankb[5][0]+Atk[5][0]);
    AnkAtk[5][1] = (Ankb[5][1]+Atk[5][1]);
    AnkAtk[5][2] = (Ankb[5][2]+Atk[5][2]);
    ank[5][0] = ((AnkAtk[5][2]*Cik[3][0][2])+((AnkAtk[5][0]*Cik[3][0][0])+(
      AnkAtk[5][1]*Cik[3][0][1])));
    ank[5][1] = ((AnkAtk[5][2]*Cik[3][1][2])+((AnkAtk[5][0]*Cik[3][1][0])+(
      AnkAtk[5][1]*Cik[3][1][1])));
    ank[5][2] = ((AnkAtk[5][2]*Cik[3][2][2])+((AnkAtk[5][0]*Cik[3][2][0])+(
      AnkAtk[5][1]*Cik[3][2][1])));
    AnkAtk[6][0] = (Ankb[6][0]+Atk[6][0]);
    AnkAtk[6][1] = (Ankb[6][1]+Atk[6][1]);
    AnkAtk[6][2] = (Ankb[6][2]+Atk[6][2]);
    ank[6][0] = ((AnkAtk[6][2]*cnk[6][0][2])+((AnkAtk[6][0]*cnk[6][0][0])+(
      AnkAtk[6][1]*cnk[6][0][1])));
    ank[6][1] = ((AnkAtk[6][2]*cnk[6][1][2])+((AnkAtk[6][0]*cnk[6][1][0])+(
      AnkAtk[6][1]*cnk[6][1][1])));
    ank[6][2] = ((AnkAtk[6][2]*cnk[6][2][2])+((AnkAtk[6][0]*cnk[6][2][0])+(
      AnkAtk[6][1]*cnk[6][2][1])));
    AnkAtk[7][0] = (Ankb[7][0]+Atk[7][0]);
    AnkAtk[7][1] = (Ankb[7][1]+Atk[7][1]);
    AnkAtk[7][2] = (Ankb[7][2]+Atk[7][2]);
    ank[7][0] = ((AnkAtk[7][2]*cnk[7][0][2])+((AnkAtk[7][0]*cnk[7][0][0])+(
      AnkAtk[7][1]*cnk[7][0][1])));
    ank[7][1] = ((AnkAtk[7][2]*cnk[7][1][2])+((AnkAtk[7][0]*cnk[7][1][0])+(
      AnkAtk[7][1]*cnk[7][1][1])));
    ank[7][2] = ((AnkAtk[7][2]*cnk[7][2][2])+((AnkAtk[7][0]*cnk[7][2][0])+(
      AnkAtk[7][1]*cnk[7][2][1])));
    AnkAtk[8][0] = (Ankb[8][0]+Atk[8][0]);
    AnkAtk[8][1] = (Ankb[8][1]+Atk[8][1]);
    AnkAtk[8][2] = (Ankb[8][2]+Atk[8][2]);
    ank[8][0] = ((AnkAtk[8][2]*cnk[8][0][2])+((AnkAtk[8][0]*cnk[8][0][0])+(
      AnkAtk[8][1]*cnk[8][0][1])));
    ank[8][1] = ((AnkAtk[8][2]*cnk[8][1][2])+((AnkAtk[8][0]*cnk[8][1][0])+(
      AnkAtk[8][1]*cnk[8][1][1])));
    ank[8][2] = ((AnkAtk[8][2]*cnk[8][2][2])+((AnkAtk[8][0]*cnk[8][2][0])+(
      AnkAtk[8][1]*cnk[8][2][1])));
    AnkAtk[9][0] = (Ankb[9][0]+Atk[9][0]);
    AnkAtk[9][1] = (Ankb[9][1]+Atk[9][1]);
    AnkAtk[9][2] = (Ankb[9][2]+Atk[9][2]);
    ank[9][0] = ((AnkAtk[9][2]*cnk[9][0][2])+((AnkAtk[9][0]*cnk[9][0][0])+(
      AnkAtk[9][1]*cnk[9][0][1])));
    ank[9][1] = ((AnkAtk[9][2]*cnk[9][1][2])+((AnkAtk[9][0]*cnk[9][1][0])+(
      AnkAtk[9][1]*cnk[9][1][1])));
    ank[9][2] = ((AnkAtk[9][2]*cnk[9][2][2])+((AnkAtk[9][0]*cnk[9][2][0])+(
      AnkAtk[9][1]*cnk[9][2][1])));
    AnkAtk[10][0] = (Ankb[10][0]+Atk[10][0]);
    AnkAtk[10][1] = (Ankb[10][1]+Atk[10][1]);
    AnkAtk[10][2] = (Ankb[10][2]+Atk[10][2]);
    ank[10][0] = ((AnkAtk[10][2]*cnk[10][0][2])+((AnkAtk[10][0]*cnk[10][0][0])+(
      AnkAtk[10][1]*cnk[10][0][1])));
    ank[10][1] = ((AnkAtk[10][2]*cnk[10][1][2])+((AnkAtk[10][0]*cnk[10][1][0])+(
      AnkAtk[10][1]*cnk[10][1][1])));
    ank[10][2] = ((AnkAtk[10][2]*cnk[10][2][2])+((AnkAtk[10][0]*cnk[10][2][0])+(
      AnkAtk[10][1]*cnk[10][2][1])));
    AnkAtk[11][0] = (Ankb[11][0]+Atk[11][0]);
    AnkAtk[11][1] = (Ankb[11][1]+Atk[11][1]);
    AnkAtk[11][2] = (Ankb[11][2]+Atk[11][2]);
    ank[11][0] = ((AnkAtk[11][2]*cnk[11][0][2])+((AnkAtk[11][0]*cnk[11][0][0])+(
      AnkAtk[11][1]*cnk[11][0][1])));
    ank[11][1] = ((AnkAtk[11][2]*cnk[11][1][2])+((AnkAtk[11][0]*cnk[11][1][0])+(
      AnkAtk[11][1]*cnk[11][1][1])));
    ank[11][2] = ((AnkAtk[11][2]*cnk[11][2][2])+((AnkAtk[11][0]*cnk[11][2][0])+(
      AnkAtk[11][1]*cnk[11][2][1])));
    AnkAtk[12][0] = (Ankb[12][0]+Atk[12][0]);
    AnkAtk[12][1] = (Ankb[12][1]+Atk[12][1]);
    AnkAtk[12][2] = (Ankb[12][2]+Atk[12][2]);
    ank[12][0] = ((AnkAtk[12][2]*cnk[12][0][2])+((AnkAtk[12][0]*cnk[12][0][0])+(
      AnkAtk[12][1]*cnk[12][0][1])));
    ank[12][1] = ((AnkAtk[12][2]*cnk[12][1][2])+((AnkAtk[12][0]*cnk[12][1][0])+(
      AnkAtk[12][1]*cnk[12][1][1])));
    ank[12][2] = ((AnkAtk[12][2]*cnk[12][2][2])+((AnkAtk[12][0]*cnk[12][2][0])+(
      AnkAtk[12][1]*cnk[12][2][1])));
    AnkAtk[13][0] = (Ankb[13][0]+Atk[13][0]);
    AnkAtk[13][1] = (Ankb[13][1]+Atk[13][1]);
    AnkAtk[13][2] = (Ankb[13][2]+Atk[13][2]);
    ank[13][0] = ((AnkAtk[13][2]*cnk[13][0][2])+((AnkAtk[13][0]*cnk[13][0][0])+(
      AnkAtk[13][1]*cnk[13][0][1])));
    ank[13][1] = ((AnkAtk[13][2]*cnk[13][1][2])+((AnkAtk[13][0]*cnk[13][1][0])+(
      AnkAtk[13][1]*cnk[13][1][1])));
    ank[13][2] = ((AnkAtk[13][2]*cnk[13][2][2])+((AnkAtk[13][0]*cnk[13][2][0])+(
      AnkAtk[13][1]*cnk[13][2][1])));
    AnkAtk[14][0] = (Ankb[14][0]+Atk[14][0]);
    AnkAtk[14][1] = (Ankb[14][1]+Atk[14][1]);
    AnkAtk[14][2] = (Ankb[14][2]+Atk[14][2]);
    ank[14][0] = ((AnkAtk[14][2]*cnk[14][0][2])+((AnkAtk[14][0]*cnk[14][0][0])+(
      AnkAtk[14][1]*cnk[14][0][1])));
    ank[14][1] = ((AnkAtk[14][2]*cnk[14][1][2])+((AnkAtk[14][0]*cnk[14][1][0])+(
      AnkAtk[14][1]*cnk[14][1][1])));
    ank[14][2] = ((AnkAtk[14][2]*cnk[14][2][2])+((AnkAtk[14][0]*cnk[14][2][0])+(
      AnkAtk[14][1]*cnk[14][2][1])));
    AnkAtk[15][0] = (Ankb[15][0]+Atk[15][0]);
    AnkAtk[15][1] = (Ankb[15][1]+Atk[15][1]);
    AnkAtk[15][2] = (Ankb[15][2]+Atk[15][2]);
    ank[15][0] = ((AnkAtk[15][2]*cnk[15][0][2])+((AnkAtk[15][0]*cnk[15][0][0])+(
      AnkAtk[15][1]*cnk[15][0][1])));
    ank[15][1] = ((AnkAtk[15][2]*cnk[15][1][2])+((AnkAtk[15][0]*cnk[15][1][0])+(
      AnkAtk[15][1]*cnk[15][1][1])));
    ank[15][2] = ((AnkAtk[15][2]*cnk[15][2][2])+((AnkAtk[15][0]*cnk[15][2][0])+(
      AnkAtk[15][1]*cnk[15][2][1])));
    AnkAtk[16][0] = (Ankb[16][0]+Atk[16][0]);
    AnkAtk[16][1] = (Ankb[16][1]+Atk[16][1]);
    AnkAtk[16][2] = (Ankb[16][2]+Atk[16][2]);
    ank[16][0] = ((AnkAtk[16][2]*cnk[16][0][2])+((AnkAtk[16][0]*cnk[16][0][0])+(
      AnkAtk[16][1]*cnk[16][0][1])));
    ank[16][1] = ((AnkAtk[16][2]*cnk[16][1][2])+((AnkAtk[16][0]*cnk[16][1][0])+(
      AnkAtk[16][1]*cnk[16][1][1])));
    ank[16][2] = ((AnkAtk[16][2]*cnk[16][2][2])+((AnkAtk[16][0]*cnk[16][2][0])+(
      AnkAtk[16][1]*cnk[16][2][1])));
    AnkAtk[17][0] = (Ankb[17][0]+Atk[17][0]);
    AnkAtk[17][1] = (Ankb[17][1]+Atk[17][1]);
    AnkAtk[17][2] = (Ankb[17][2]+Atk[17][2]);
    ank[17][0] = ((AnkAtk[17][2]*cnk[17][0][2])+((AnkAtk[17][0]*cnk[17][0][0])+(
      AnkAtk[17][1]*cnk[17][0][1])));
    ank[17][1] = ((AnkAtk[17][2]*cnk[17][1][2])+((AnkAtk[17][0]*cnk[17][1][0])+(
      AnkAtk[17][1]*cnk[17][1][1])));
    ank[17][2] = ((AnkAtk[17][2]*cnk[17][2][2])+((AnkAtk[17][0]*cnk[17][2][0])+(
      AnkAtk[17][1]*cnk[17][2][1])));
    AnkAtk[18][0] = (Ankb[18][0]+Atk[18][0]);
    AnkAtk[18][1] = (Ankb[18][1]+Atk[18][1]);
    AnkAtk[18][2] = (Ankb[18][2]+Atk[18][2]);
    ank[18][0] = ((AnkAtk[18][2]*cnk[18][0][2])+((AnkAtk[18][0]*cnk[18][0][0])+(
      AnkAtk[18][1]*cnk[18][0][1])));
    ank[18][1] = ((AnkAtk[18][2]*cnk[18][1][2])+((AnkAtk[18][0]*cnk[18][1][0])+(
      AnkAtk[18][1]*cnk[18][1][1])));
    ank[18][2] = ((AnkAtk[18][2]*cnk[18][2][2])+((AnkAtk[18][0]*cnk[18][2][0])+(
      AnkAtk[18][1]*cnk[18][2][1])));
    AnkAtk[19][0] = (Ankb[19][0]+Atk[19][0]);
    AnkAtk[19][1] = (Ankb[19][1]+Atk[19][1]);
    AnkAtk[19][2] = (Ankb[19][2]+Atk[19][2]);
    ank[19][0] = ((AnkAtk[19][2]*cnk[19][0][2])+((AnkAtk[19][0]*cnk[19][0][0])+(
      AnkAtk[19][1]*cnk[19][0][1])));
    ank[19][1] = ((AnkAtk[19][2]*cnk[19][1][2])+((AnkAtk[19][0]*cnk[19][1][0])+(
      AnkAtk[19][1]*cnk[19][1][1])));
    ank[19][2] = ((AnkAtk[19][2]*cnk[19][2][2])+((AnkAtk[19][0]*cnk[19][2][0])+(
      AnkAtk[19][1]*cnk[19][2][1])));
    AnkAtk[20][0] = (Ankb[20][0]+Atk[20][0]);
    AnkAtk[20][1] = (Ankb[20][1]+Atk[20][1]);
    AnkAtk[20][2] = (Ankb[20][2]+Atk[20][2]);
    ank[20][0] = ((AnkAtk[20][2]*cnk[20][0][2])+((AnkAtk[20][0]*cnk[20][0][0])+(
      AnkAtk[20][1]*cnk[20][0][1])));
    ank[20][1] = ((AnkAtk[20][2]*cnk[20][1][2])+((AnkAtk[20][0]*cnk[20][1][0])+(
      AnkAtk[20][1]*cnk[20][1][1])));
    ank[20][2] = ((AnkAtk[20][2]*cnk[20][2][2])+((AnkAtk[20][0]*cnk[20][2][0])+(
      AnkAtk[20][1]*cnk[20][2][1])));
    anb[0][0] = ank[5][0];
    anb[0][1] = ank[5][1];
    anb[0][2] = ank[5][2];
    anb[1][0] = ank[6][0];
    anb[1][1] = ank[6][1];
    anb[1][2] = ank[6][2];
    anb[2][0] = ank[7][0];
    anb[2][1] = ank[7][1];
    anb[2][2] = ank[7][2];
    anb[3][0] = ank[8][0];
    anb[3][1] = ank[8][1];
    anb[3][2] = ank[8][2];
    anb[4][0] = ank[9][0];
    anb[4][1] = ank[9][1];
    anb[4][2] = ank[9][2];
    anb[5][0] = ank[10][0];
    anb[5][1] = ank[10][1];
    anb[5][2] = ank[10][2];
    anb[6][0] = ank[11][0];
    anb[6][1] = ank[11][1];
    anb[6][2] = ank[11][2];
    anb[7][0] = ank[12][0];
    anb[7][1] = ank[12][1];
    anb[7][2] = ank[12][2];
    anb[8][0] = ank[13][0];
    anb[8][1] = ank[13][1];
    anb[8][2] = ank[13][2];
    anb[9][0] = ank[14][0];
    anb[9][1] = ank[14][1];
    anb[9][2] = ank[14][2];
    anb[10][0] = ank[15][0];
    anb[10][1] = ank[15][1];
    anb[10][2] = ank[15][2];
    anb[11][0] = ank[16][0];
    anb[11][1] = ank[16][1];
    anb[11][2] = ank[16][2];
    anb[12][0] = ank[17][0];
    anb[12][1] = ank[17][1];
    anb[12][2] = ank[17][2];
    anb[13][0] = ank[18][0];
    anb[13][1] = ank[18][1];
    anb[13][2] = ank[18][2];
    anb[14][0] = ank[19][0];
    anb[14][1] = ank[19][1];
    anb[14][2] = ank[19][2];
    anb[15][0] = ank[20][0];
    anb[15][1] = ank[20][1];
    anb[15][2] = ank[20][2];
/*
Compute constraint acceleration errors
*/
    roustate = 3;
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  729 adds/subtracts/negates
                    663 multiplies
                      0 divides
                    552 assignments
*/
}

void sdmassmat(double mmat[21][21])
{
/* Return the system mass matrix (LHS)
*/
    int i,j;

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(57,23);
        return;
    }
    sddomm(57);
    for (i = 0; i < 21; i++) {
        for (j = i; j <= 20; j++) {
            mmat[i][j] = mm[i][j];
            mmat[j][i] = mm[i][j];
        }
    }
}

void sdfrcmat(double fmat[21])
{
/* Return the system force matrix (RHS), excluding constraints
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(58,23);
        return;
    }
    sddofs0();
    for (i = 0; i < 21; i++) {
        fmat[i] = fs0[i];
    }
}

void sdpseudo(double lqout[1],
    double luout[1])
{
/*
Return pseudo-coordinates for loop joints.

*/
/*
There are no loop joints in this system.

*/
}

void sdpsqdot(double lqdout[1])
{
/*
Return pseudo-coordinate derivatives for loop joints.

*/
/*
There are no loop joints in this system.

*/
}

void sdpsudot(double ludout[1])
{
/*
Return pseudo-coordinate accelerations for loop joints.

*/
/*
There are no loop joints in this system.

*/
}

void sdperr(double errs[21])
{
/*
Return position constraint errors.

*/

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(26,23);
        return;
    }
    if (pres[0]  !=  0.) {
        perr[0] = (q[0]-upos[0]);
    } else {
        perr[0] = 0.;
    }
    if (pres[1]  !=  0.) {
        perr[1] = (q[1]-upos[1]);
    } else {
        perr[1] = 0.;
    }
    if (pres[2]  !=  0.) {
        perr[2] = (q[2]-upos[2]);
    } else {
        perr[2] = 0.;
    }
    if (pres[3]  !=  0.) {
        perr[3] = (q[3]-upos[3]);
    } else {
        perr[3] = 0.;
    }
    if (pres[4]  !=  0.) {
        perr[4] = (q[4]-upos[4]);
    } else {
        perr[4] = 0.;
    }
    if (pres[5]  !=  0.) {
        perr[5] = (q[5]-upos[5]);
    } else {
        perr[5] = 0.;
    }
    if (pres[6]  !=  0.) {
        perr[6] = (q[6]-upos[6]);
    } else {
        perr[6] = 0.;
    }
    if (pres[7]  !=  0.) {
        perr[7] = (q[7]-upos[7]);
    } else {
        perr[7] = 0.;
    }
    if (pres[8]  !=  0.) {
        perr[8] = (q[8]-upos[8]);
    } else {
        perr[8] = 0.;
    }
    if (pres[9]  !=  0.) {
        perr[9] = (q[9]-upos[9]);
    } else {
        perr[9] = 0.;
    }
    if (pres[10]  !=  0.) {
        perr[10] = (q[10]-upos[10]);
    } else {
        perr[10] = 0.;
    }
    if (pres[11]  !=  0.) {
        perr[11] = (q[11]-upos[11]);
    } else {
        perr[11] = 0.;
    }
    if (pres[12]  !=  0.) {
        perr[12] = (q[12]-upos[12]);
    } else {
        perr[12] = 0.;
    }
    if (pres[13]  !=  0.) {
        perr[13] = (q[13]-upos[13]);
    } else {
        perr[13] = 0.;
    }
    if (pres[14]  !=  0.) {
        perr[14] = (q[14]-upos[14]);
    } else {
        perr[14] = 0.;
    }
    if (pres[15]  !=  0.) {
        perr[15] = (q[15]-upos[15]);
    } else {
        perr[15] = 0.;
    }
    if (pres[16]  !=  0.) {
        perr[16] = (q[16]-upos[16]);
    } else {
        perr[16] = 0.;
    }
    if (pres[17]  !=  0.) {
        perr[17] = (q[17]-upos[17]);
    } else {
        perr[17] = 0.;
    }
    if (pres[18]  !=  0.) {
        perr[18] = (q[18]-upos[18]);
    } else {
        perr[18] = 0.;
    }
    if (pres[19]  !=  0.) {
        perr[19] = (q[19]-upos[19]);
    } else {
        perr[19] = 0.;
    }
    if (pres[20]  !=  0.) {
        perr[20] = (q[20]-upos[20]);
    } else {
        perr[20] = 0.;
    }
    errs[0] = perr[0];
    errs[1] = perr[1];
    errs[2] = perr[2];
    errs[3] = perr[3];
    errs[4] = perr[4];
    errs[5] = perr[5];
    errs[6] = perr[6];
    errs[7] = perr[7];
    errs[8] = perr[8];
    errs[9] = perr[9];
    errs[10] = perr[10];
    errs[11] = perr[11];
    errs[12] = perr[12];
    errs[13] = perr[13];
    errs[14] = perr[14];
    errs[15] = perr[15];
    errs[16] = perr[16];
    errs[17] = perr[17];
    errs[18] = perr[18];
    errs[19] = perr[19];
    errs[20] = perr[20];
}

void sdverr(double errs[21])
{
/*
Return velocity constraint errors.

*/

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(27,23);
        return;
    }
    if (pres[0]  !=  0.) {
        verr[0] = (u[0]-uvel[0]);
    } else {
        verr[0] = 0.;
    }
    if (pres[1]  !=  0.) {
        verr[1] = (u[1]-uvel[1]);
    } else {
        verr[1] = 0.;
    }
    if (pres[2]  !=  0.) {
        verr[2] = (u[2]-uvel[2]);
    } else {
        verr[2] = 0.;
    }
    if (pres[3]  !=  0.) {
        verr[3] = (u[3]-uvel[3]);
    } else {
        verr[3] = 0.;
    }
    if (pres[4]  !=  0.) {
        verr[4] = (u[4]-uvel[4]);
    } else {
        verr[4] = 0.;
    }
    if (pres[5]  !=  0.) {
        verr[5] = (u[5]-uvel[5]);
    } else {
        verr[5] = 0.;
    }
    if (pres[6]  !=  0.) {
        verr[6] = (u[6]-uvel[6]);
    } else {
        verr[6] = 0.;
    }
    if (pres[7]  !=  0.) {
        verr[7] = (u[7]-uvel[7]);
    } else {
        verr[7] = 0.;
    }
    if (pres[8]  !=  0.) {
        verr[8] = (u[8]-uvel[8]);
    } else {
        verr[8] = 0.;
    }
    if (pres[9]  !=  0.) {
        verr[9] = (u[9]-uvel[9]);
    } else {
        verr[9] = 0.;
    }
    if (pres[10]  !=  0.) {
        verr[10] = (u[10]-uvel[10]);
    } else {
        verr[10] = 0.;
    }
    if (pres[11]  !=  0.) {
        verr[11] = (u[11]-uvel[11]);
    } else {
        verr[11] = 0.;
    }
    if (pres[12]  !=  0.) {
        verr[12] = (u[12]-uvel[12]);
    } else {
        verr[12] = 0.;
    }
    if (pres[13]  !=  0.) {
        verr[13] = (u[13]-uvel[13]);
    } else {
        verr[13] = 0.;
    }
    if (pres[14]  !=  0.) {
        verr[14] = (u[14]-uvel[14]);
    } else {
        verr[14] = 0.;
    }
    if (pres[15]  !=  0.) {
        verr[15] = (u[15]-uvel[15]);
    } else {
        verr[15] = 0.;
    }
    if (pres[16]  !=  0.) {
        verr[16] = (u[16]-uvel[16]);
    } else {
        verr[16] = 0.;
    }
    if (pres[17]  !=  0.) {
        verr[17] = (u[17]-uvel[17]);
    } else {
        verr[17] = 0.;
    }
    if (pres[18]  !=  0.) {
        verr[18] = (u[18]-uvel[18]);
    } else {
        verr[18] = 0.;
    }
    if (pres[19]  !=  0.) {
        verr[19] = (u[19]-uvel[19]);
    } else {
        verr[19] = 0.;
    }
    if (pres[20]  !=  0.) {
        verr[20] = (u[20]-uvel[20]);
    } else {
        verr[20] = 0.;
    }
    errs[0] = verr[0];
    errs[1] = verr[1];
    errs[2] = verr[2];
    errs[3] = verr[3];
    errs[4] = verr[4];
    errs[5] = verr[5];
    errs[6] = verr[6];
    errs[7] = verr[7];
    errs[8] = verr[8];
    errs[9] = verr[9];
    errs[10] = verr[10];
    errs[11] = verr[11];
    errs[12] = verr[12];
    errs[13] = verr[13];
    errs[14] = verr[14];
    errs[15] = verr[15];
    errs[16] = verr[16];
    errs[17] = verr[17];
    errs[18] = verr[18];
    errs[19] = verr[19];
    errs[20] = verr[20];
}

void sdaerr(double errs[21])
{
/*
Return acceleration constraint errors.

*/

    if (roustate != 3) {
        sdseterr(35,24);
        return;
    }
    if (pres[0]  !=  0.) {
        aerr[0] = (udot[0]-uacc[0]);
    } else {
        aerr[0] = 0.;
    }
    if (pres[1]  !=  0.) {
        aerr[1] = (udot[1]-uacc[1]);
    } else {
        aerr[1] = 0.;
    }
    if (pres[2]  !=  0.) {
        aerr[2] = (udot[2]-uacc[2]);
    } else {
        aerr[2] = 0.;
    }
    if (pres[3]  !=  0.) {
        aerr[3] = (udot[3]-uacc[3]);
    } else {
        aerr[3] = 0.;
    }
    if (pres[4]  !=  0.) {
        aerr[4] = (udot[4]-uacc[4]);
    } else {
        aerr[4] = 0.;
    }
    if (pres[5]  !=  0.) {
        aerr[5] = (udot[5]-uacc[5]);
    } else {
        aerr[5] = 0.;
    }
    if (pres[6]  !=  0.) {
        aerr[6] = (udot[6]-uacc[6]);
    } else {
        aerr[6] = 0.;
    }
    if (pres[7]  !=  0.) {
        aerr[7] = (udot[7]-uacc[7]);
    } else {
        aerr[7] = 0.;
    }
    if (pres[8]  !=  0.) {
        aerr[8] = (udot[8]-uacc[8]);
    } else {
        aerr[8] = 0.;
    }
    if (pres[9]  !=  0.) {
        aerr[9] = (udot[9]-uacc[9]);
    } else {
        aerr[9] = 0.;
    }
    if (pres[10]  !=  0.) {
        aerr[10] = (udot[10]-uacc[10]);
    } else {
        aerr[10] = 0.;
    }
    if (pres[11]  !=  0.) {
        aerr[11] = (udot[11]-uacc[11]);
    } else {
        aerr[11] = 0.;
    }
    if (pres[12]  !=  0.) {
        aerr[12] = (udot[12]-uacc[12]);
    } else {
        aerr[12] = 0.;
    }
    if (pres[13]  !=  0.) {
        aerr[13] = (udot[13]-uacc[13]);
    } else {
        aerr[13] = 0.;
    }
    if (pres[14]  !=  0.) {
        aerr[14] = (udot[14]-uacc[14]);
    } else {
        aerr[14] = 0.;
    }
    if (pres[15]  !=  0.) {
        aerr[15] = (udot[15]-uacc[15]);
    } else {
        aerr[15] = 0.;
    }
    if (pres[16]  !=  0.) {
        aerr[16] = (udot[16]-uacc[16]);
    } else {
        aerr[16] = 0.;
    }
    if (pres[17]  !=  0.) {
        aerr[17] = (udot[17]-uacc[17]);
    } else {
        aerr[17] = 0.;
    }
    if (pres[18]  !=  0.) {
        aerr[18] = (udot[18]-uacc[18]);
    } else {
        aerr[18] = 0.;
    }
    if (pres[19]  !=  0.) {
        aerr[19] = (udot[19]-uacc[19]);
    } else {
        aerr[19] = 0.;
    }
    if (pres[20]  !=  0.) {
        aerr[20] = (udot[20]-uacc[20]);
    } else {
        aerr[20] = 0.;
    }
    errs[0] = aerr[0];
    errs[1] = aerr[1];
    errs[2] = aerr[2];
    errs[3] = aerr[3];
    errs[4] = aerr[4];
    errs[5] = aerr[5];
    errs[6] = aerr[6];
    errs[7] = aerr[7];
    errs[8] = aerr[8];
    errs[9] = aerr[9];
    errs[10] = aerr[10];
    errs[11] = aerr[11];
    errs[12] = aerr[12];
    errs[13] = aerr[13];
    errs[14] = aerr[14];
    errs[15] = aerr[15];
    errs[16] = aerr[16];
    errs[17] = aerr[17];
    errs[18] = aerr[18];
    errs[19] = aerr[19];
    errs[20] = aerr[20];
}
int 
sdchkbnum(int routine,
    int bnum)
{

    if ((bnum < -1) || (bnum > 15)) {
        sdseterr(routine,15);
        return 1;
    }
    return 0;
}
int 
sdchkjnum(int routine,
    int jnum)
{

    if ((jnum < 0) || (jnum > 15)) {
        sdseterr(routine,16);
        return 1;
    }
    return 0;
}
int 
sdchkucnum(int routine,
    int ucnum)
{

    if ((ucnum < 0) || (ucnum > -1)) {
        sdseterr(routine,21);
        return 1;
    }
    return 0;
}
int 
sdchkjaxis(int routine,
    int jnum,
    int axnum)
{
    int maxax;

    if (sdchkjnum(routine,jnum) != 0) {
        return 1;
    }
    if ((axnum < 0) || (axnum > 6)) {
        sdseterr(routine,17);
        return 1;
    }
    maxax = njntdof[jnum]-1;
    if ((jtype[jnum] == 4) || (jtype[jnum] == 6) || (jtype[jnum] == 21)) {
        maxax = maxax+1;
    }
    if (axnum > maxax) {
        sdseterr(routine,18);
        return 1;
    }
    return 0;
}
int 
sdchkjpin(int routine,
    int jnum,
    int pinno)
{
    int maxax,pinok;

    if (sdchkjnum(routine,jnum) != 0) {
        return 1;
    }
    if ((pinno < 0) || (pinno > 5)) {
        sdseterr(routine,17);
        return 1;
    }
    if (njntdof[jnum] >= 3) {
        maxax = 2;
    } else {
        maxax = njntdof[jnum]-1;
    }
    if (jtype[jnum] == 4) {
        maxax = -1;
    }
    if (jtype[jnum] == 7) {
        maxax = 0;
    }
    pinok = 0;
    if (pinno <= maxax) {
        pinok = 1;
    }
    if (pinok == 0) {
        sdseterr(routine,18);
        return 1;
    }
    return 0;
}
int 
sdindx(int joint,
    int axis)
{
    int offs,gotit;

    if (sdchkjaxis(36,joint,axis) != 0) {
        return 0;
    }
    gotit = 0;
    if (jtype[joint] == 4) {
        if (axis == 3) {
            offs = ballq[joint];
            gotit = 1;
        }
    } else {
        if ((jtype[joint] == 6) || (jtype[joint] == 21)) {
            if (axis == 6) {
                offs = ballq[joint];
                gotit = 1;
            }
        }
    }
    if (gotit == 0) {
        offs = firstq[joint]+axis;
    }
    return offs;
}

void sdpresacc(int joint,
    int axis,
    double prval)
{

    if (sdchkjaxis(13,joint,axis) != 0) {
        return;
    }
    if (roustate != 2) {
        sdseterr(13,23);
        return;
    }
    if (pres[sdindx(joint,axis)]  !=  0.) {
        uacc[sdindx(joint,axis)] = prval;
    }
}

void sdpresvel(int joint,
    int axis,
    double prval)
{

    if (sdchkjaxis(14,joint,axis) != 0) {
        return;
    }
    if (roustate != 2) {
        sdseterr(14,23);
        return;
    }
    if (pres[sdindx(joint,axis)]  !=  0.) {
        uvel[sdindx(joint,axis)] = prval;
    }
}

void sdprespos(int joint,
    int axis,
    double prval)
{

    if (sdchkjaxis(15,joint,axis) != 0) {
        return;
    }
    if (roustate != 2) {
        sdseterr(15,23);
        return;
    }
    if (pres[sdindx(joint,axis)]  !=  0.) {
        upos[sdindx(joint,axis)] = prval;
    }
}

void sdgetht(int joint,
    int axis,
    double *torque)
{

    if (sdchkjaxis(30,joint,axis) != 0) {
        return;
    }
    if (roustate != 3) {
        sdseterr(30,24);
        return;
    }
    *torque = tauc[sdindx(joint,axis)];
}

void sdhinget(int joint,
    int axis,
    double torque)
{

    if (sdchkjaxis(10,joint,axis) != 0) {
        return;
    }
    if (roustate != 2) {
        sdseterr(10,23);
        return;
    }
    if (mfrcflg != 0) {
        mtau[sdindx(joint,axis)] = mtau[sdindx(joint,axis)]+torque;
    } else {
        fs0flg = 0;
        utau[sdindx(joint,axis)] = utau[sdindx(joint,axis)]+torque;
    }
}

void sdpointf(int body,
    double point[3],
    double force[3])
{
    double torque[3];

    if (sdchkbnum(11,body) != 0) {
        return;
    }
    if (roustate != 2) {
        sdseterr(11,23);
        return;
    }
    if (body == -1) {
        return;
    }
    torque[0] = point[1]*force[2]-point[2]*force[1];
    torque[1] = point[2]*force[0]-point[0]*force[2];
    torque[2] = point[0]*force[1]-point[1]*force[0];
    if (mfrcflg != 0) {
        mfk[body][0] = mfk[body][0]+force[0];
        mtk[body][0] = mtk[body][0]+torque[0];
        mfk[body][1] = mfk[body][1]+force[1];
        mtk[body][1] = mtk[body][1]+torque[1];
        mfk[body][2] = mfk[body][2]+force[2];
        mtk[body][2] = mtk[body][2]+torque[2];
    } else {
        fs0flg = 0;
        ufk[body][0] = ufk[body][0]+force[0];
        utk[body][0] = utk[body][0]+torque[0];
        ufk[body][1] = ufk[body][1]+force[1];
        utk[body][1] = utk[body][1]+torque[1];
        ufk[body][2] = ufk[body][2]+force[2];
        utk[body][2] = utk[body][2]+torque[2];
    }
}

void sdbodyt(int body,
    double torque[3])
{

    if (sdchkbnum(12,body) != 0) {
        return;
    }
    if (roustate != 2) {
        sdseterr(12,23);
        return;
    }
    if (body == -1) {
        return;
    }
    if (mfrcflg != 0) {
        mtk[body][0] = mtk[body][0]+torque[0];
        mtk[body][1] = mtk[body][1]+torque[1];
        mtk[body][2] = mtk[body][2]+torque[2];
    } else {
        fs0flg = 0;
        utk[body][0] = utk[body][0]+torque[0];
        utk[body][1] = utk[body][1]+torque[1];
        utk[body][2] = utk[body][2]+torque[2];
    }
}

void sddoww(int routine)
{
    double pp[21][21],dpp[21][21];
    int i,j,c;
    double sum;
    double dfk[16][3],dtk[16][3],dtau[21],dltci[1][3],dltc[1][3],dlfci[1][3],
      dlfc[1][3];
    double dTinb[1][3],dToutb[1][3],dltaufi[1][3],dltaufo[1][3],dltauti[1][3],
      dltauto[1][3];
    double dfs[21],row[21],dinvrow[21];

    roustate = 2;
    if (wwflg == 0) {
/*
Compute constraint effects
*/
        sddovpk();
        sddommldu(routine);
/*
Constraint 0 (prescribed motion)
*/
        if (pres[0]  !=  0.) {
            dtau[0] = 1.;
        } else {
            dtau[0] = 0.;
        }
        dfs[0] = dtau[0];
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[0][i] = row[i];
            dpp[i][0] = dinvrow[i];
        }
        wmap[0] = 0;
/*
Constraint 1 (prescribed motion)
*/
        if (pres[1]  !=  0.) {
            dtau[1] = 1.;
        } else {
            dtau[1] = 0.;
        }
        dfs[1] = dtau[1];
        dfs[0] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[1][i] = row[i];
            dpp[i][1] = dinvrow[i];
        }
        wmap[1] = 1;
/*
Constraint 2 (prescribed motion)
*/
        if (pres[2]  !=  0.) {
            dtau[2] = 1.;
        } else {
            dtau[2] = 0.;
        }
        dfs[2] = dtau[2];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[2][i] = row[i];
            dpp[i][2] = dinvrow[i];
        }
        wmap[2] = 2;
/*
Constraint 3 (prescribed motion)
*/
        if (pres[3]  !=  0.) {
            dtau[3] = 1.;
        } else {
            dtau[3] = 0.;
        }
        dfs[3] = dtau[3];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[3][i] = row[i];
            dpp[i][3] = dinvrow[i];
        }
        wmap[3] = 3;
/*
Constraint 4 (prescribed motion)
*/
        if (pres[4]  !=  0.) {
            dtau[4] = 1.;
        } else {
            dtau[4] = 0.;
        }
        dfs[4] = dtau[4];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[4][i] = row[i];
            dpp[i][4] = dinvrow[i];
        }
        wmap[4] = 4;
/*
Constraint 5 (prescribed motion)
*/
        if (pres[5]  !=  0.) {
            dtau[5] = 1.;
        } else {
            dtau[5] = 0.;
        }
        dfs[5] = dtau[5];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[5][i] = row[i];
            dpp[i][5] = dinvrow[i];
        }
        wmap[5] = 5;
/*
Constraint 6 (prescribed motion)
*/
        if (pres[6]  !=  0.) {
            dtau[6] = 1.;
        } else {
            dtau[6] = 0.;
        }
        dfs[6] = dtau[6];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[6][i] = row[i];
            dpp[i][6] = dinvrow[i];
        }
        wmap[6] = 6;
/*
Constraint 7 (prescribed motion)
*/
        if (pres[7]  !=  0.) {
            dtau[7] = 1.;
        } else {
            dtau[7] = 0.;
        }
        dfs[7] = dtau[7];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[7][i] = row[i];
            dpp[i][7] = dinvrow[i];
        }
        wmap[7] = 7;
/*
Constraint 8 (prescribed motion)
*/
        if (pres[8]  !=  0.) {
            dtau[8] = 1.;
        } else {
            dtau[8] = 0.;
        }
        dfs[8] = dtau[8];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[8][i] = row[i];
            dpp[i][8] = dinvrow[i];
        }
        wmap[8] = 8;
/*
Constraint 9 (prescribed motion)
*/
        if (pres[9]  !=  0.) {
            dtau[9] = 1.;
        } else {
            dtau[9] = 0.;
        }
        dfs[9] = dtau[9];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[9][i] = row[i];
            dpp[i][9] = dinvrow[i];
        }
        wmap[9] = 9;
/*
Constraint 10 (prescribed motion)
*/
        if (pres[10]  !=  0.) {
            dtau[10] = 1.;
        } else {
            dtau[10] = 0.;
        }
        dfs[10] = dtau[10];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[10][i] = row[i];
            dpp[i][10] = dinvrow[i];
        }
        wmap[10] = 10;
/*
Constraint 11 (prescribed motion)
*/
        if (pres[11]  !=  0.) {
            dtau[11] = 1.;
        } else {
            dtau[11] = 0.;
        }
        dfs[11] = dtau[11];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[11][i] = row[i];
            dpp[i][11] = dinvrow[i];
        }
        wmap[11] = 11;
/*
Constraint 12 (prescribed motion)
*/
        if (pres[12]  !=  0.) {
            dtau[12] = 1.;
        } else {
            dtau[12] = 0.;
        }
        dfs[12] = dtau[12];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[12][i] = row[i];
            dpp[i][12] = dinvrow[i];
        }
        wmap[12] = 12;
/*
Constraint 13 (prescribed motion)
*/
        if (pres[13]  !=  0.) {
            dtau[13] = 1.;
        } else {
            dtau[13] = 0.;
        }
        dfs[13] = dtau[13];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[13][i] = row[i];
            dpp[i][13] = dinvrow[i];
        }
        wmap[13] = 13;
/*
Constraint 14 (prescribed motion)
*/
        if (pres[14]  !=  0.) {
            dtau[14] = 1.;
        } else {
            dtau[14] = 0.;
        }
        dfs[14] = dtau[14];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[14][i] = row[i];
            dpp[i][14] = dinvrow[i];
        }
        wmap[14] = 14;
/*
Constraint 15 (prescribed motion)
*/
        if (pres[15]  !=  0.) {
            dtau[15] = 1.;
        } else {
            dtau[15] = 0.;
        }
        dfs[15] = dtau[15];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[15][i] = row[i];
            dpp[i][15] = dinvrow[i];
        }
        wmap[15] = 15;
/*
Constraint 16 (prescribed motion)
*/
        if (pres[16]  !=  0.) {
            dtau[16] = 1.;
        } else {
            dtau[16] = 0.;
        }
        dfs[16] = dtau[16];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[16][i] = row[i];
            dpp[i][16] = dinvrow[i];
        }
        wmap[16] = 16;
/*
Constraint 17 (prescribed motion)
*/
        if (pres[17]  !=  0.) {
            dtau[17] = 1.;
        } else {
            dtau[17] = 0.;
        }
        dfs[17] = dtau[17];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[17][i] = row[i];
            dpp[i][17] = dinvrow[i];
        }
        wmap[17] = 17;
/*
Constraint 18 (prescribed motion)
*/
        if (pres[18]  !=  0.) {
            dtau[18] = 1.;
        } else {
            dtau[18] = 0.;
        }
        dfs[18] = dtau[18];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[19] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[18][i] = row[i];
            dpp[i][18] = dinvrow[i];
        }
        wmap[18] = 18;
/*
Constraint 19 (prescribed motion)
*/
        if (pres[19]  !=  0.) {
            dtau[19] = 1.;
        } else {
            dtau[19] = 0.;
        }
        dfs[19] = dtau[19];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[20] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[19][i] = row[i];
            dpp[i][19] = dinvrow[i];
        }
        wmap[19] = 19;
/*
Constraint 20 (prescribed motion)
*/
        if (pres[20]  !=  0.) {
            dtau[20] = 1.;
        } else {
            dtau[20] = 0.;
        }
        dfs[20] = dtau[20];
        dfs[0] = 0.;
        dfs[1] = 0.;
        dfs[2] = 0.;
        dfs[3] = 0.;
        dfs[4] = 0.;
        dfs[5] = 0.;
        dfs[6] = 0.;
        dfs[7] = 0.;
        dfs[8] = 0.;
        dfs[9] = 0.;
        dfs[10] = 0.;
        dfs[11] = 0.;
        dfs[12] = 0.;
        dfs[13] = 0.;
        dfs[14] = 0.;
        dfs[15] = 0.;
        dfs[16] = 0.;
        dfs[17] = 0.;
        dfs[18] = 0.;
        dfs[19] = 0.;
        sdldubsl(21,21,mmap,mlo,dfs,row);
        sdldubsd(21,21,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 20; i++) {
            pp[20][i] = row[i];
            dpp[i][20] = dinvrow[i];
        }
        wmap[20] = 20;
/*
Produce constraint coefficient matrix WW
*/
        for (c = 0; c <= 20; c++) {
            for (i = c; i <= 20; i++) {
                sum = 0.;
                for (j = 0; j <= 20; j++) {
                    sum = sum+pp[wmap[c]][j]*dpp[j][wmap[i]];
                }
                ww[wmap[c]][wmap[i]] = sum;
                ww[wmap[i]][wmap[c]] = sum;
            }
        }
/*
Form QR decomposition of WW
*/
        sdqrdcomp(21,21,21,21,wmap,wmap,ww,qraux,jpvt);
        wwflg = 1;
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain 4851 adds/subtracts/negates
                   4851 multiplies
                      0 divides
                   6909 assignments
*/
}

void sdxudot0(int routine,
    double oudot0[21])
{
/*
Compute unconstrained equations
*/
    int i;

    sdlhs(routine);
/*
Solve equations ignoring constraints
*/
    sdfs0();
    sdldubslv(21,21,mmap,works,mlo,mdi,fs,udot);
    for (i = 0; i <= 20; i++) {
        oudot0[i] = udot[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     21 assignments
*/
}

void sdudot0(double oudot0[21])
{

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(66,23);
        return;
    }
    sdxudot0(66,oudot0);
}

void sdsetudot(double iudot[21])
{
/*
Assign udots and advance to stage Dynamics Ready
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(68,23);
        return;
    }
    for (i = 0; i <= 20; i++) {
        udot[i] = iudot[i];
    }
    sdrhs();
}

void sdxudotm(int routine,
    double imult[21],
    double oudotm[21])
{
/*
Compute udots due only to multipliers
*/
    int i;

    sdlhs(routine);
    sdmfrc(imult);
    sdfsmult();
    sdldubslv(21,21,mmap,works,mlo,mdi,fs,udot);
    for (i = 0; i <= 20; i++) {
        oudotm[i] = udot[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     21 assignments
*/
}

void sdudotm(double imult[21],
    double oudotm[21])
{

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(67,23);
        return;
    }
    sdxudotm(67,imult,oudotm);
}

void sdderiv(double oqdot[22],
    double oudot[21])
{
/*
This is the derivative section for a 16-body ground-based
system with 21 hinge degree(s) of freedom.
21 of the degrees of freedom may follow prescribed motion.
There are 21 constraints.
*/
    double workr[21],bb[21],b0[21],v0[21],p0[21];
    int iwork[21];
    int i,j;
    double udot0[21],udot1[21];

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(17,23);
        return;
    }
    if (stabvelq == 1) {
        sdseterr(17,32);
    }
    if (stabposq == 1) {
        sdseterr(17,33);
    }
    wsiz = 21;
/*
Compute unconstrained equations
*/
    sdxudot0(17,udot0);
    sdrhs();
    sdaerr(b0);
    if (stabvel  !=  0.) {
        sdverr(v0);
    }
    if (stabpos  !=  0.) {
        sdperr(p0);
    }
/*
Stabilize constraints using Baumgarte's method
*/
    for (i = 0; i <= 20; i++) {
        bb[i] = -b0[i];
    }
    if (stabvel  !=  0.) {
        for (i = 0; i <= 20; i++) {
            bb[i] = bb[i]-stabvel*v0[i];
        }
    }
    if (stabpos  !=  0.) {
        for (i = 0; i <= 20; i++) {
            bb[i] = bb[i]-stabpos*p0[i];
        }
    }
/*
Compute and decompose constraint matrix WW
*/
    sddoww(17);
/*
Numerically solve for constraint multipliers
*/
    sdqrbslv(21,21,21,21,wmap,wmap,1e-13,workr,iwork,ww,qraux,jpvt,bb,mult,&
      wrank);
    for (i = 0; i <= 20; i++) {
        multmap[i] = 0;
    }
    for (i = 0; i < wrank; i++) {
        multmap[jpvt[i]] = 1;
    }
    j = 0;
    for (i = 0; i <= 20; i++) {
        if (multmap[i] != 0) {
            multmap[j] = wmap[i];
            j = j+1;
        }
    }
/*
Compute final udots
*/
    sdxudotm(17,mult,udot1);
    for (i = 0; i <= 20; i++) {
        udot[i] = udot0[i]+udot1[i];
    }
    sdrhs();
    for (i = 0; i <= 21; i++) {
        oqdot[i] = qdot[i];
    }
    for (i = 0; i <= 20; i++) {
        oudot[i] = udot[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   84 adds/subtracts/negates
                     42 multiplies
                      0 divides
                    127 assignments
*/
}
/*
Compute residuals for use with DAE integrator
*/

void sdresid(double eqdot[22],
    double eudot[21],
    double emults[21],
    double resid[64])
{
    int i;
    double uderrs[21],p0[21];

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(16,23);
        return;
    }
    if (stabposq == 1) {
        sdseterr(16,33);
    }
    sdfulltrq(eudot,emults,uderrs);
    for (i = 0; i < 22; i++) {
        resid[i] = eqdot[i]-qdot[i];
    }
    for (i = 0; i < 21; i++) {
        resid[22+i] = uderrs[i];
    }
    sdverr(&resid[43]);
    if (stabpos  !=  0.) {
        sdperr(p0);
        for (i = 0; i < 21; i++) {
            resid[43+i] = resid[43+i]+stabpos*p0[i];
        }
    }
    for (i = 0; i < 21; i++) {
        udot[i] = eudot[i];
    }
    for (i = 0; i < 21; i++) {
        mult[i] = emults[i];
    }
    sdrhs();
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   43 adds/subtracts/negates
                     21 multiplies
                      0 divides
                    106 assignments
*/
}

void sdmult(double omults[21],
    int *owrank,
    int omultmap[21])
{
    int i;

    if (roustate != 3) {
        sdseterr(34,24);
        return;
    }
    for (i = 0; i < 21; i++) {
        omults[i] = mult[i];
        if (i <= wrank-1) {
            omultmap[i] = multmap[i];
        } else {
            omultmap[i] = -1;
        }
    }
    *owrank = wrank;
}

void sdreac(double force[16][3],
    double torque[16][3])
{
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

    if (roustate != 3) {
        sdseterr(31,24);
        return;
    }
/*
Compute reaction forces for non-weld tree joints
*/
    fc[20][0] = ((mk[15]*(AnkAtk[20][0]-gk[20][0]))-ufk[15][0]);
    fc[20][1] = ((mk[15]*(AnkAtk[20][1]-gk[20][1]))-ufk[15][1]);
    fc[20][2] = ((mk[15]*(AnkAtk[20][2]-gk[20][2]))-ufk[15][2]);
    tc[20][0] = ((WkIkWk[20][0]+((ik[15][0][2]*onk[20][2])+((ik[15][0][0]*
      onk[20][0])+(ik[15][0][1]*onk[20][1]))))-(utk[15][0]+((fc[20][2]*rk[15][1]
      )-(fc[20][1]*rk[15][2]))));
    tc[20][1] = ((WkIkWk[20][1]+((ik[15][1][2]*onk[20][2])+((ik[15][1][0]*
      onk[20][0])+(ik[15][1][1]*onk[20][1]))))-(utk[15][1]+((fc[20][0]*rk[15][2]
      )-(fc[20][2]*rk[15][0]))));
    tc[20][2] = ((WkIkWk[20][2]+((ik[15][2][2]*onk[20][2])+((ik[15][2][0]*
      onk[20][0])+(ik[15][2][1]*onk[20][1]))))-(utk[15][2]+((fc[20][1]*rk[15][0]
      )-(fc[20][0]*rk[15][1]))));
    fccikt[20][0] = ((Cik[20][0][2]*fc[20][2])+((Cik[20][0][0]*fc[20][0])+(
      Cik[20][0][1]*fc[20][1])));
    fccikt[20][1] = ((Cik[20][1][2]*fc[20][2])+((Cik[20][1][0]*fc[20][0])+(
      Cik[20][1][1]*fc[20][1])));
    fccikt[20][2] = ((Cik[20][2][2]*fc[20][2])+((Cik[20][2][0]*fc[20][0])+(
      Cik[20][2][1]*fc[20][1])));
    ffk[19][0] = (ufk[14][0]-fccikt[20][0]);
    ffk[19][1] = (ufk[14][1]-fccikt[20][1]);
    ffk[19][2] = (ufk[14][2]-fccikt[20][2]);
    ttk[19][0] = (utk[14][0]-(((Cik[20][0][2]*tc[20][2])+((Cik[20][0][0]*
      tc[20][0])+(Cik[20][0][1]*tc[20][1])))+((fccikt[20][2]*ri[15][1])-(
      fccikt[20][1]*ri[15][2]))));
    ttk[19][1] = (utk[14][1]-(((Cik[20][1][2]*tc[20][2])+((Cik[20][1][0]*
      tc[20][0])+(Cik[20][1][1]*tc[20][1])))+((fccikt[20][0]*ri[15][2])-(
      fccikt[20][2]*ri[15][0]))));
    ttk[19][2] = (utk[14][2]-(((Cik[20][2][2]*tc[20][2])+((Cik[20][2][0]*
      tc[20][0])+(Cik[20][2][1]*tc[20][1])))+((fccikt[20][1]*ri[15][0])-(
      fccikt[20][0]*ri[15][1]))));
    fc[19][0] = ((mk[14]*(AnkAtk[19][0]-gk[19][0]))-ffk[19][0]);
    fc[19][1] = ((mk[14]*(AnkAtk[19][1]-gk[19][1]))-ffk[19][1]);
    fc[19][2] = ((mk[14]*(AnkAtk[19][2]-gk[19][2]))-ffk[19][2]);
    tc[19][0] = ((WkIkWk[19][0]+((ik[14][0][2]*onk[19][2])+((ik[14][0][0]*
      onk[19][0])+(ik[14][0][1]*onk[19][1]))))-(ttk[19][0]+((fc[19][2]*rk[14][1]
      )-(fc[19][1]*rk[14][2]))));
    tc[19][1] = ((WkIkWk[19][1]+((ik[14][1][2]*onk[19][2])+((ik[14][1][0]*
      onk[19][0])+(ik[14][1][1]*onk[19][1]))))-(ttk[19][1]+((fc[19][0]*rk[14][2]
      )-(fc[19][2]*rk[14][0]))));
    tc[19][2] = ((WkIkWk[19][2]+((ik[14][2][2]*onk[19][2])+((ik[14][2][0]*
      onk[19][0])+(ik[14][2][1]*onk[19][1]))))-(ttk[19][2]+((fc[19][1]*rk[14][0]
      )-(fc[19][0]*rk[14][1]))));
    fccikt[19][0] = ((Cik[19][0][2]*fc[19][2])+((Cik[19][0][0]*fc[19][0])+(
      Cik[19][0][1]*fc[19][1])));
    fccikt[19][1] = ((Cik[19][1][2]*fc[19][2])+((Cik[19][1][0]*fc[19][0])+(
      Cik[19][1][1]*fc[19][1])));
    fccikt[19][2] = ((Cik[19][2][2]*fc[19][2])+((Cik[19][2][0]*fc[19][0])+(
      Cik[19][2][1]*fc[19][1])));
    ffk[18][0] = (ufk[13][0]-fccikt[19][0]);
    ffk[18][1] = (ufk[13][1]-fccikt[19][1]);
    ffk[18][2] = (ufk[13][2]-fccikt[19][2]);
    ttk[18][0] = (utk[13][0]-(((Cik[19][0][2]*tc[19][2])+((Cik[19][0][0]*
      tc[19][0])+(Cik[19][0][1]*tc[19][1])))+((fccikt[19][2]*ri[14][1])-(
      fccikt[19][1]*ri[14][2]))));
    ttk[18][1] = (utk[13][1]-(((Cik[19][1][2]*tc[19][2])+((Cik[19][1][0]*
      tc[19][0])+(Cik[19][1][1]*tc[19][1])))+((fccikt[19][0]*ri[14][2])-(
      fccikt[19][2]*ri[14][0]))));
    ttk[18][2] = (utk[13][2]-(((Cik[19][2][2]*tc[19][2])+((Cik[19][2][0]*
      tc[19][0])+(Cik[19][2][1]*tc[19][1])))+((fccikt[19][1]*ri[14][0])-(
      fccikt[19][0]*ri[14][1]))));
    fc[18][0] = ((mk[13]*(AnkAtk[18][0]-gk[18][0]))-ffk[18][0]);
    fc[18][1] = ((mk[13]*(AnkAtk[18][1]-gk[18][1]))-ffk[18][1]);
    fc[18][2] = ((mk[13]*(AnkAtk[18][2]-gk[18][2]))-ffk[18][2]);
    tc[18][0] = ((WkIkWk[18][0]+((ik[13][0][2]*onk[18][2])+((ik[13][0][0]*
      onk[18][0])+(ik[13][0][1]*onk[18][1]))))-(ttk[18][0]+((fc[18][2]*rk[13][1]
      )-(fc[18][1]*rk[13][2]))));
    tc[18][1] = ((WkIkWk[18][1]+((ik[13][1][2]*onk[18][2])+((ik[13][1][0]*
      onk[18][0])+(ik[13][1][1]*onk[18][1]))))-(ttk[18][1]+((fc[18][0]*rk[13][2]
      )-(fc[18][2]*rk[13][0]))));
    tc[18][2] = ((WkIkWk[18][2]+((ik[13][2][2]*onk[18][2])+((ik[13][2][0]*
      onk[18][0])+(ik[13][2][1]*onk[18][1]))))-(ttk[18][2]+((fc[18][1]*rk[13][0]
      )-(fc[18][0]*rk[13][1]))));
    fccikt[18][0] = ((Cik[18][0][2]*fc[18][2])+((Cik[18][0][0]*fc[18][0])+(
      Cik[18][0][1]*fc[18][1])));
    fccikt[18][1] = ((Cik[18][1][2]*fc[18][2])+((Cik[18][1][0]*fc[18][0])+(
      Cik[18][1][1]*fc[18][1])));
    fccikt[18][2] = ((Cik[18][2][2]*fc[18][2])+((Cik[18][2][0]*fc[18][0])+(
      Cik[18][2][1]*fc[18][1])));
    ffk[17][0] = (ufk[12][0]-fccikt[18][0]);
    ffk[17][1] = (ufk[12][1]-fccikt[18][1]);
    ffk[17][2] = (ufk[12][2]-fccikt[18][2]);
    ttk[17][0] = (utk[12][0]-(((Cik[18][0][2]*tc[18][2])+((Cik[18][0][0]*
      tc[18][0])+(Cik[18][0][1]*tc[18][1])))+((fccikt[18][2]*ri[13][1])-(
      fccikt[18][1]*ri[13][2]))));
    ttk[17][1] = (utk[12][1]-(((Cik[18][1][2]*tc[18][2])+((Cik[18][1][0]*
      tc[18][0])+(Cik[18][1][1]*tc[18][1])))+((fccikt[18][0]*ri[13][2])-(
      fccikt[18][2]*ri[13][0]))));
    ttk[17][2] = (utk[12][2]-(((Cik[18][2][2]*tc[18][2])+((Cik[18][2][0]*
      tc[18][0])+(Cik[18][2][1]*tc[18][1])))+((fccikt[18][1]*ri[13][0])-(
      fccikt[18][0]*ri[13][1]))));
    fc[17][0] = ((mk[12]*(AnkAtk[17][0]-gk[17][0]))-ffk[17][0]);
    fc[17][1] = ((mk[12]*(AnkAtk[17][1]-gk[17][1]))-ffk[17][1]);
    fc[17][2] = ((mk[12]*(AnkAtk[17][2]-gk[17][2]))-ffk[17][2]);
    tc[17][0] = ((WkIkWk[17][0]+((ik[12][0][2]*onk[17][2])+((ik[12][0][0]*
      onk[17][0])+(ik[12][0][1]*onk[17][1]))))-(ttk[17][0]+((fc[17][2]*rk[12][1]
      )-(fc[17][1]*rk[12][2]))));
    tc[17][1] = ((WkIkWk[17][1]+((ik[12][1][2]*onk[17][2])+((ik[12][1][0]*
      onk[17][0])+(ik[12][1][1]*onk[17][1]))))-(ttk[17][1]+((fc[17][0]*rk[12][2]
      )-(fc[17][2]*rk[12][0]))));
    tc[17][2] = ((WkIkWk[17][2]+((ik[12][2][2]*onk[17][2])+((ik[12][2][0]*
      onk[17][0])+(ik[12][2][1]*onk[17][1]))))-(ttk[17][2]+((fc[17][1]*rk[12][0]
      )-(fc[17][0]*rk[12][1]))));
    fccikt[17][0] = ((Cik[17][0][2]*fc[17][2])+((Cik[17][0][0]*fc[17][0])+(
      Cik[17][0][1]*fc[17][1])));
    fccikt[17][1] = ((Cik[17][1][2]*fc[17][2])+((Cik[17][1][0]*fc[17][0])+(
      Cik[17][1][1]*fc[17][1])));
    fccikt[17][2] = ((Cik[17][2][2]*fc[17][2])+((Cik[17][2][0]*fc[17][0])+(
      Cik[17][2][1]*fc[17][1])));
    ffk[16][0] = (ufk[11][0]-fccikt[17][0]);
    ffk[16][1] = (ufk[11][1]-fccikt[17][1]);
    ffk[16][2] = (ufk[11][2]-fccikt[17][2]);
    ttk[16][0] = (utk[11][0]-(((Cik[17][0][2]*tc[17][2])+((Cik[17][0][0]*
      tc[17][0])+(Cik[17][0][1]*tc[17][1])))+((fccikt[17][2]*ri[12][1])-(
      fccikt[17][1]*ri[12][2]))));
    ttk[16][1] = (utk[11][1]-(((Cik[17][1][2]*tc[17][2])+((Cik[17][1][0]*
      tc[17][0])+(Cik[17][1][1]*tc[17][1])))+((fccikt[17][0]*ri[12][2])-(
      fccikt[17][2]*ri[12][0]))));
    ttk[16][2] = (utk[11][2]-(((Cik[17][2][2]*tc[17][2])+((Cik[17][2][0]*
      tc[17][0])+(Cik[17][2][1]*tc[17][1])))+((fccikt[17][1]*ri[12][0])-(
      fccikt[17][0]*ri[12][1]))));
    fc[16][0] = ((mk[11]*(AnkAtk[16][0]-gk[16][0]))-ffk[16][0]);
    fc[16][1] = ((mk[11]*(AnkAtk[16][1]-gk[16][1]))-ffk[16][1]);
    fc[16][2] = ((mk[11]*(AnkAtk[16][2]-gk[16][2]))-ffk[16][2]);
    tc[16][0] = ((WkIkWk[16][0]+((ik[11][0][2]*onk[16][2])+((ik[11][0][0]*
      onk[16][0])+(ik[11][0][1]*onk[16][1]))))-(ttk[16][0]+((fc[16][2]*rk[11][1]
      )-(fc[16][1]*rk[11][2]))));
    tc[16][1] = ((WkIkWk[16][1]+((ik[11][1][2]*onk[16][2])+((ik[11][1][0]*
      onk[16][0])+(ik[11][1][1]*onk[16][1]))))-(ttk[16][1]+((fc[16][0]*rk[11][2]
      )-(fc[16][2]*rk[11][0]))));
    tc[16][2] = ((WkIkWk[16][2]+((ik[11][2][2]*onk[16][2])+((ik[11][2][0]*
      onk[16][0])+(ik[11][2][1]*onk[16][1]))))-(ttk[16][2]+((fc[16][1]*rk[11][0]
      )-(fc[16][0]*rk[11][1]))));
    fccikt[16][0] = ((Cik[16][0][2]*fc[16][2])+((Cik[16][0][0]*fc[16][0])+(
      Cik[16][0][1]*fc[16][1])));
    fccikt[16][1] = ((Cik[16][1][2]*fc[16][2])+((Cik[16][1][0]*fc[16][0])+(
      Cik[16][1][1]*fc[16][1])));
    fccikt[16][2] = ((Cik[16][2][2]*fc[16][2])+((Cik[16][2][0]*fc[16][0])+(
      Cik[16][2][1]*fc[16][1])));
    ffk[15][0] = (ufk[10][0]-fccikt[16][0]);
    ffk[15][1] = (ufk[10][1]-fccikt[16][1]);
    ffk[15][2] = (ufk[10][2]-fccikt[16][2]);
    ttk[15][0] = (utk[10][0]-(((Cik[16][0][2]*tc[16][2])+((Cik[16][0][0]*
      tc[16][0])+(Cik[16][0][1]*tc[16][1])))+((fccikt[16][2]*ri[11][1])-(
      fccikt[16][1]*ri[11][2]))));
    ttk[15][1] = (utk[10][1]-(((Cik[16][1][2]*tc[16][2])+((Cik[16][1][0]*
      tc[16][0])+(Cik[16][1][1]*tc[16][1])))+((fccikt[16][0]*ri[11][2])-(
      fccikt[16][2]*ri[11][0]))));
    ttk[15][2] = (utk[10][2]-(((Cik[16][2][2]*tc[16][2])+((Cik[16][2][0]*
      tc[16][0])+(Cik[16][2][1]*tc[16][1])))+((fccikt[16][1]*ri[11][0])-(
      fccikt[16][0]*ri[11][1]))));
    fc[15][0] = ((mk[10]*(AnkAtk[15][0]-gk[15][0]))-ffk[15][0]);
    fc[15][1] = ((mk[10]*(AnkAtk[15][1]-gk[15][1]))-ffk[15][1]);
    fc[15][2] = ((mk[10]*(AnkAtk[15][2]-gk[15][2]))-ffk[15][2]);
    tc[15][0] = ((WkIkWk[15][0]+((ik[10][0][2]*onk[15][2])+((ik[10][0][0]*
      onk[15][0])+(ik[10][0][1]*onk[15][1]))))-(ttk[15][0]+((fc[15][2]*rk[10][1]
      )-(fc[15][1]*rk[10][2]))));
    tc[15][1] = ((WkIkWk[15][1]+((ik[10][1][2]*onk[15][2])+((ik[10][1][0]*
      onk[15][0])+(ik[10][1][1]*onk[15][1]))))-(ttk[15][1]+((fc[15][0]*rk[10][2]
      )-(fc[15][2]*rk[10][0]))));
    tc[15][2] = ((WkIkWk[15][2]+((ik[10][2][2]*onk[15][2])+((ik[10][2][0]*
      onk[15][0])+(ik[10][2][1]*onk[15][1]))))-(ttk[15][2]+((fc[15][1]*rk[10][0]
      )-(fc[15][0]*rk[10][1]))));
    fccikt[15][0] = ((Cik[15][0][2]*fc[15][2])+((Cik[15][0][0]*fc[15][0])+(
      Cik[15][0][1]*fc[15][1])));
    fccikt[15][1] = ((Cik[15][1][2]*fc[15][2])+((Cik[15][1][0]*fc[15][0])+(
      Cik[15][1][1]*fc[15][1])));
    fccikt[15][2] = ((Cik[15][2][2]*fc[15][2])+((Cik[15][2][0]*fc[15][0])+(
      Cik[15][2][1]*fc[15][1])));
    ffk[5][0] = (ufk[0][0]-fccikt[15][0]);
    ffk[5][1] = (ufk[0][1]-fccikt[15][1]);
    ffk[5][2] = (ufk[0][2]-fccikt[15][2]);
    ttk[5][0] = (utk[0][0]-(((Cik[15][0][2]*tc[15][2])+((Cik[15][0][0]*tc[15][0]
      )+(Cik[15][0][1]*tc[15][1])))+((fccikt[15][2]*ri[10][1])-(fccikt[15][1]*
      ri[10][2]))));
    ttk[5][1] = (utk[0][1]-(((Cik[15][1][2]*tc[15][2])+((Cik[15][1][0]*tc[15][0]
      )+(Cik[15][1][1]*tc[15][1])))+((fccikt[15][0]*ri[10][2])-(fccikt[15][2]*
      ri[10][0]))));
    ttk[5][2] = (utk[0][2]-(((Cik[15][2][2]*tc[15][2])+((Cik[15][2][0]*tc[15][0]
      )+(Cik[15][2][1]*tc[15][1])))+((fccikt[15][1]*ri[10][0])-(fccikt[15][0]*
      ri[10][1]))));
    fc[14][0] = ((mk[9]*(AnkAtk[14][0]-gk[14][0]))-ufk[9][0]);
    fc[14][1] = ((mk[9]*(AnkAtk[14][1]-gk[14][1]))-ufk[9][1]);
    fc[14][2] = ((mk[9]*(AnkAtk[14][2]-gk[14][2]))-ufk[9][2]);
    tc[14][0] = ((WkIkWk[14][0]+((ik[9][0][2]*onk[14][2])+((ik[9][0][0]*
      onk[14][0])+(ik[9][0][1]*onk[14][1]))))-(utk[9][0]+((fc[14][2]*rk[9][1])-(
      fc[14][1]*rk[9][2]))));
    tc[14][1] = ((WkIkWk[14][1]+((ik[9][1][2]*onk[14][2])+((ik[9][1][0]*
      onk[14][0])+(ik[9][1][1]*onk[14][1]))))-(utk[9][1]+((fc[14][0]*rk[9][2])-(
      fc[14][2]*rk[9][0]))));
    tc[14][2] = ((WkIkWk[14][2]+((ik[9][2][2]*onk[14][2])+((ik[9][2][0]*
      onk[14][0])+(ik[9][2][1]*onk[14][1]))))-(utk[9][2]+((fc[14][1]*rk[9][0])-(
      fc[14][0]*rk[9][1]))));
    fccikt[14][0] = ((Cik[14][0][2]*fc[14][2])+((Cik[14][0][0]*fc[14][0])+(
      Cik[14][0][1]*fc[14][1])));
    fccikt[14][1] = ((Cik[14][1][2]*fc[14][2])+((Cik[14][1][0]*fc[14][0])+(
      Cik[14][1][1]*fc[14][1])));
    fccikt[14][2] = ((Cik[14][2][2]*fc[14][2])+((Cik[14][2][0]*fc[14][0])+(
      Cik[14][2][1]*fc[14][1])));
    ffk[13][0] = (ufk[8][0]-fccikt[14][0]);
    ffk[13][1] = (ufk[8][1]-fccikt[14][1]);
    ffk[13][2] = (ufk[8][2]-fccikt[14][2]);
    ttk[13][0] = (utk[8][0]-(((Cik[14][0][2]*tc[14][2])+((Cik[14][0][0]*
      tc[14][0])+(Cik[14][0][1]*tc[14][1])))+((fccikt[14][2]*ri[9][1])-(
      fccikt[14][1]*ri[9][2]))));
    ttk[13][1] = (utk[8][1]-(((Cik[14][1][2]*tc[14][2])+((Cik[14][1][0]*
      tc[14][0])+(Cik[14][1][1]*tc[14][1])))+((fccikt[14][0]*ri[9][2])-(
      fccikt[14][2]*ri[9][0]))));
    ttk[13][2] = (utk[8][2]-(((Cik[14][2][2]*tc[14][2])+((Cik[14][2][0]*
      tc[14][0])+(Cik[14][2][1]*tc[14][1])))+((fccikt[14][1]*ri[9][0])-(
      fccikt[14][0]*ri[9][1]))));
    fc[13][0] = ((mk[8]*(AnkAtk[13][0]-gk[13][0]))-ffk[13][0]);
    fc[13][1] = ((mk[8]*(AnkAtk[13][1]-gk[13][1]))-ffk[13][1]);
    fc[13][2] = ((mk[8]*(AnkAtk[13][2]-gk[13][2]))-ffk[13][2]);
    tc[13][0] = ((WkIkWk[13][0]+((ik[8][0][2]*onk[13][2])+((ik[8][0][0]*
      onk[13][0])+(ik[8][0][1]*onk[13][1]))))-(ttk[13][0]+((fc[13][2]*rk[8][1])-
      (fc[13][1]*rk[8][2]))));
    tc[13][1] = ((WkIkWk[13][1]+((ik[8][1][2]*onk[13][2])+((ik[8][1][0]*
      onk[13][0])+(ik[8][1][1]*onk[13][1]))))-(ttk[13][1]+((fc[13][0]*rk[8][2])-
      (fc[13][2]*rk[8][0]))));
    tc[13][2] = ((WkIkWk[13][2]+((ik[8][2][2]*onk[13][2])+((ik[8][2][0]*
      onk[13][0])+(ik[8][2][1]*onk[13][1]))))-(ttk[13][2]+((fc[13][1]*rk[8][0])-
      (fc[13][0]*rk[8][1]))));
    fccikt[13][0] = ((Cik[13][0][2]*fc[13][2])+((Cik[13][0][0]*fc[13][0])+(
      Cik[13][0][1]*fc[13][1])));
    fccikt[13][1] = ((Cik[13][1][2]*fc[13][2])+((Cik[13][1][0]*fc[13][0])+(
      Cik[13][1][1]*fc[13][1])));
    fccikt[13][2] = ((Cik[13][2][2]*fc[13][2])+((Cik[13][2][0]*fc[13][0])+(
      Cik[13][2][1]*fc[13][1])));
    ffk[12][0] = (ufk[7][0]-fccikt[13][0]);
    ffk[12][1] = (ufk[7][1]-fccikt[13][1]);
    ffk[12][2] = (ufk[7][2]-fccikt[13][2]);
    ttk[12][0] = (utk[7][0]-(((Cik[13][0][2]*tc[13][2])+((Cik[13][0][0]*
      tc[13][0])+(Cik[13][0][1]*tc[13][1])))+((fccikt[13][2]*ri[8][1])-(
      fccikt[13][1]*ri[8][2]))));
    ttk[12][1] = (utk[7][1]-(((Cik[13][1][2]*tc[13][2])+((Cik[13][1][0]*
      tc[13][0])+(Cik[13][1][1]*tc[13][1])))+((fccikt[13][0]*ri[8][2])-(
      fccikt[13][2]*ri[8][0]))));
    ttk[12][2] = (utk[7][2]-(((Cik[13][2][2]*tc[13][2])+((Cik[13][2][0]*
      tc[13][0])+(Cik[13][2][1]*tc[13][1])))+((fccikt[13][1]*ri[8][0])-(
      fccikt[13][0]*ri[8][1]))));
    fc[12][0] = ((mk[7]*(AnkAtk[12][0]-gk[12][0]))-ffk[12][0]);
    fc[12][1] = ((mk[7]*(AnkAtk[12][1]-gk[12][1]))-ffk[12][1]);
    fc[12][2] = ((mk[7]*(AnkAtk[12][2]-gk[12][2]))-ffk[12][2]);
    tc[12][0] = ((WkIkWk[12][0]+((ik[7][0][2]*onk[12][2])+((ik[7][0][0]*
      onk[12][0])+(ik[7][0][1]*onk[12][1]))))-(ttk[12][0]+((fc[12][2]*rk[7][1])-
      (fc[12][1]*rk[7][2]))));
    tc[12][1] = ((WkIkWk[12][1]+((ik[7][1][2]*onk[12][2])+((ik[7][1][0]*
      onk[12][0])+(ik[7][1][1]*onk[12][1]))))-(ttk[12][1]+((fc[12][0]*rk[7][2])-
      (fc[12][2]*rk[7][0]))));
    tc[12][2] = ((WkIkWk[12][2]+((ik[7][2][2]*onk[12][2])+((ik[7][2][0]*
      onk[12][0])+(ik[7][2][1]*onk[12][1]))))-(ttk[12][2]+((fc[12][1]*rk[7][0])-
      (fc[12][0]*rk[7][1]))));
    fccikt[12][0] = ((Cik[12][0][2]*fc[12][2])+((Cik[12][0][0]*fc[12][0])+(
      Cik[12][0][1]*fc[12][1])));
    fccikt[12][1] = ((Cik[12][1][2]*fc[12][2])+((Cik[12][1][0]*fc[12][0])+(
      Cik[12][1][1]*fc[12][1])));
    fccikt[12][2] = ((Cik[12][2][2]*fc[12][2])+((Cik[12][2][0]*fc[12][0])+(
      Cik[12][2][1]*fc[12][1])));
    ffk[11][0] = (ufk[6][0]-fccikt[12][0]);
    ffk[11][1] = (ufk[6][1]-fccikt[12][1]);
    ffk[11][2] = (ufk[6][2]-fccikt[12][2]);
    ttk[11][0] = (utk[6][0]-(((Cik[12][0][2]*tc[12][2])+((Cik[12][0][0]*
      tc[12][0])+(Cik[12][0][1]*tc[12][1])))+((fccikt[12][2]*ri[7][1])-(
      fccikt[12][1]*ri[7][2]))));
    ttk[11][1] = (utk[6][1]-(((Cik[12][1][2]*tc[12][2])+((Cik[12][1][0]*
      tc[12][0])+(Cik[12][1][1]*tc[12][1])))+((fccikt[12][0]*ri[7][2])-(
      fccikt[12][2]*ri[7][0]))));
    ttk[11][2] = (utk[6][2]-(((Cik[12][2][2]*tc[12][2])+((Cik[12][2][0]*
      tc[12][0])+(Cik[12][2][1]*tc[12][1])))+((fccikt[12][1]*ri[7][0])-(
      fccikt[12][0]*ri[7][1]))));
    fc[11][0] = ((mk[6]*(AnkAtk[11][0]-gk[11][0]))-ffk[11][0]);
    fc[11][1] = ((mk[6]*(AnkAtk[11][1]-gk[11][1]))-ffk[11][1]);
    fc[11][2] = ((mk[6]*(AnkAtk[11][2]-gk[11][2]))-ffk[11][2]);
    tc[11][0] = ((WkIkWk[11][0]+((ik[6][0][2]*onk[11][2])+((ik[6][0][0]*
      onk[11][0])+(ik[6][0][1]*onk[11][1]))))-(ttk[11][0]+((fc[11][2]*rk[6][1])-
      (fc[11][1]*rk[6][2]))));
    tc[11][1] = ((WkIkWk[11][1]+((ik[6][1][2]*onk[11][2])+((ik[6][1][0]*
      onk[11][0])+(ik[6][1][1]*onk[11][1]))))-(ttk[11][1]+((fc[11][0]*rk[6][2])-
      (fc[11][2]*rk[6][0]))));
    tc[11][2] = ((WkIkWk[11][2]+((ik[6][2][2]*onk[11][2])+((ik[6][2][0]*
      onk[11][0])+(ik[6][2][1]*onk[11][1]))))-(ttk[11][2]+((fc[11][1]*rk[6][0])-
      (fc[11][0]*rk[6][1]))));
    fccikt[11][0] = ((Cik[11][0][2]*fc[11][2])+((Cik[11][0][0]*fc[11][0])+(
      Cik[11][0][1]*fc[11][1])));
    fccikt[11][1] = ((Cik[11][1][2]*fc[11][2])+((Cik[11][1][0]*fc[11][0])+(
      Cik[11][1][1]*fc[11][1])));
    fccikt[11][2] = ((Cik[11][2][2]*fc[11][2])+((Cik[11][2][0]*fc[11][0])+(
      Cik[11][2][1]*fc[11][1])));
    ffk[10][0] = (ufk[5][0]-fccikt[11][0]);
    ffk[10][1] = (ufk[5][1]-fccikt[11][1]);
    ffk[10][2] = (ufk[5][2]-fccikt[11][2]);
    ttk[10][0] = (utk[5][0]-(((Cik[11][0][2]*tc[11][2])+((Cik[11][0][0]*
      tc[11][0])+(Cik[11][0][1]*tc[11][1])))+((fccikt[11][2]*ri[6][1])-(
      fccikt[11][1]*ri[6][2]))));
    ttk[10][1] = (utk[5][1]-(((Cik[11][1][2]*tc[11][2])+((Cik[11][1][0]*
      tc[11][0])+(Cik[11][1][1]*tc[11][1])))+((fccikt[11][0]*ri[6][2])-(
      fccikt[11][2]*ri[6][0]))));
    ttk[10][2] = (utk[5][2]-(((Cik[11][2][2]*tc[11][2])+((Cik[11][2][0]*
      tc[11][0])+(Cik[11][2][1]*tc[11][1])))+((fccikt[11][1]*ri[6][0])-(
      fccikt[11][0]*ri[6][1]))));
    fc[10][0] = ((mk[5]*(AnkAtk[10][0]-gk[10][0]))-ffk[10][0]);
    fc[10][1] = ((mk[5]*(AnkAtk[10][1]-gk[10][1]))-ffk[10][1]);
    fc[10][2] = ((mk[5]*(AnkAtk[10][2]-gk[10][2]))-ffk[10][2]);
    tc[10][0] = ((WkIkWk[10][0]+((ik[5][0][2]*onk[10][2])+((ik[5][0][0]*
      onk[10][0])+(ik[5][0][1]*onk[10][1]))))-(ttk[10][0]+((fc[10][2]*rk[5][1])-
      (fc[10][1]*rk[5][2]))));
    tc[10][1] = ((WkIkWk[10][1]+((ik[5][1][2]*onk[10][2])+((ik[5][1][0]*
      onk[10][0])+(ik[5][1][1]*onk[10][1]))))-(ttk[10][1]+((fc[10][0]*rk[5][2])-
      (fc[10][2]*rk[5][0]))));
    tc[10][2] = ((WkIkWk[10][2]+((ik[5][2][2]*onk[10][2])+((ik[5][2][0]*
      onk[10][0])+(ik[5][2][1]*onk[10][1]))))-(ttk[10][2]+((fc[10][1]*rk[5][0])-
      (fc[10][0]*rk[5][1]))));
    fccikt[10][0] = ((Cik[10][0][2]*fc[10][2])+((Cik[10][0][0]*fc[10][0])+(
      Cik[10][0][1]*fc[10][1])));
    fccikt[10][1] = ((Cik[10][1][2]*fc[10][2])+((Cik[10][1][0]*fc[10][0])+(
      Cik[10][1][1]*fc[10][1])));
    fccikt[10][2] = ((Cik[10][2][2]*fc[10][2])+((Cik[10][2][0]*fc[10][0])+(
      Cik[10][2][1]*fc[10][1])));
    ffk[9][0] = (ufk[4][0]-fccikt[10][0]);
    ffk[9][1] = (ufk[4][1]-fccikt[10][1]);
    ffk[9][2] = (ufk[4][2]-fccikt[10][2]);
    ttk[9][0] = (utk[4][0]-(((Cik[10][0][2]*tc[10][2])+((Cik[10][0][0]*tc[10][0]
      )+(Cik[10][0][1]*tc[10][1])))+((fccikt[10][2]*ri[5][1])-(fccikt[10][1]*
      ri[5][2]))));
    ttk[9][1] = (utk[4][1]-(((Cik[10][1][2]*tc[10][2])+((Cik[10][1][0]*tc[10][0]
      )+(Cik[10][1][1]*tc[10][1])))+((fccikt[10][0]*ri[5][2])-(fccikt[10][2]*
      ri[5][0]))));
    ttk[9][2] = (utk[4][2]-(((Cik[10][2][2]*tc[10][2])+((Cik[10][2][0]*tc[10][0]
      )+(Cik[10][2][1]*tc[10][1])))+((fccikt[10][1]*ri[5][0])-(fccikt[10][0]*
      ri[5][1]))));
    fc[9][0] = ((mk[4]*(AnkAtk[9][0]-gk[9][0]))-ffk[9][0]);
    fc[9][1] = ((mk[4]*(AnkAtk[9][1]-gk[9][1]))-ffk[9][1]);
    fc[9][2] = ((mk[4]*(AnkAtk[9][2]-gk[9][2]))-ffk[9][2]);
    tc[9][0] = ((WkIkWk[9][0]+((ik[4][0][2]*onk[9][2])+((ik[4][0][0]*onk[9][0])+
      (ik[4][0][1]*onk[9][1]))))-(ttk[9][0]+((fc[9][2]*rk[4][1])-(fc[9][1]*
      rk[4][2]))));
    tc[9][1] = ((WkIkWk[9][1]+((ik[4][1][2]*onk[9][2])+((ik[4][1][0]*onk[9][0])+
      (ik[4][1][1]*onk[9][1]))))-(ttk[9][1]+((fc[9][0]*rk[4][2])-(fc[9][2]*
      rk[4][0]))));
    tc[9][2] = ((WkIkWk[9][2]+((ik[4][2][2]*onk[9][2])+((ik[4][2][0]*onk[9][0])+
      (ik[4][2][1]*onk[9][1]))))-(ttk[9][2]+((fc[9][1]*rk[4][0])-(fc[9][0]*
      rk[4][1]))));
    fccikt[9][0] = ((Cik[9][0][2]*fc[9][2])+((Cik[9][0][0]*fc[9][0])+(
      Cik[9][0][1]*fc[9][1])));
    fccikt[9][1] = ((Cik[9][1][2]*fc[9][2])+((Cik[9][1][0]*fc[9][0])+(
      Cik[9][1][1]*fc[9][1])));
    fccikt[9][2] = ((Cik[9][2][2]*fc[9][2])+((Cik[9][2][0]*fc[9][0])+(
      Cik[9][2][1]*fc[9][1])));
    ffk[5][0] = (ffk[5][0]-fccikt[9][0]);
    ffk[5][1] = (ffk[5][1]-fccikt[9][1]);
    ffk[5][2] = (ffk[5][2]-fccikt[9][2]);
    ttk[5][0] = (ttk[5][0]-(((Cik[9][0][2]*tc[9][2])+((Cik[9][0][0]*tc[9][0])+(
      Cik[9][0][1]*tc[9][1])))+((fccikt[9][2]*ri[4][1])-(fccikt[9][1]*ri[4][2]))
      ));
    ttk[5][1] = (ttk[5][1]-(((Cik[9][1][2]*tc[9][2])+((Cik[9][1][0]*tc[9][0])+(
      Cik[9][1][1]*tc[9][1])))+((fccikt[9][0]*ri[4][2])-(fccikt[9][2]*ri[4][0]))
      ));
    ttk[5][2] = (ttk[5][2]-(((Cik[9][2][2]*tc[9][2])+((Cik[9][2][0]*tc[9][0])+(
      Cik[9][2][1]*tc[9][1])))+((fccikt[9][1]*ri[4][0])-(fccikt[9][0]*ri[4][1]))
      ));
    fc[8][0] = ((mk[3]*(AnkAtk[8][0]-gk[8][0]))-ufk[3][0]);
    fc[8][1] = ((mk[3]*(AnkAtk[8][1]-gk[8][1]))-ufk[3][1]);
    fc[8][2] = ((mk[3]*(AnkAtk[8][2]-gk[8][2]))-ufk[3][2]);
    tc[8][0] = ((WkIkWk[8][0]+((ik[3][0][2]*onk[8][2])+((ik[3][0][0]*onk[8][0])+
      (ik[3][0][1]*onk[8][1]))))-(utk[3][0]+((fc[8][2]*rk[3][1])-(fc[8][1]*
      rk[3][2]))));
    tc[8][1] = ((WkIkWk[8][1]+((ik[3][1][2]*onk[8][2])+((ik[3][1][0]*onk[8][0])+
      (ik[3][1][1]*onk[8][1]))))-(utk[3][1]+((fc[8][0]*rk[3][2])-(fc[8][2]*
      rk[3][0]))));
    tc[8][2] = ((WkIkWk[8][2]+((ik[3][2][2]*onk[8][2])+((ik[3][2][0]*onk[8][0])+
      (ik[3][2][1]*onk[8][1]))))-(utk[3][2]+((fc[8][1]*rk[3][0])-(fc[8][0]*
      rk[3][1]))));
    fccikt[8][0] = ((Cik[8][0][2]*fc[8][2])+((Cik[8][0][0]*fc[8][0])+(
      Cik[8][0][1]*fc[8][1])));
    fccikt[8][1] = ((Cik[8][1][2]*fc[8][2])+((Cik[8][1][0]*fc[8][0])+(
      Cik[8][1][1]*fc[8][1])));
    fccikt[8][2] = ((Cik[8][2][2]*fc[8][2])+((Cik[8][2][0]*fc[8][0])+(
      Cik[8][2][1]*fc[8][1])));
    ffk[7][0] = (ufk[2][0]-fccikt[8][0]);
    ffk[7][1] = (ufk[2][1]-fccikt[8][1]);
    ffk[7][2] = (ufk[2][2]-fccikt[8][2]);
    ttk[7][0] = (utk[2][0]-(((Cik[8][0][2]*tc[8][2])+((Cik[8][0][0]*tc[8][0])+(
      Cik[8][0][1]*tc[8][1])))+((fccikt[8][2]*ri[3][1])-(fccikt[8][1]*ri[3][2]))
      ));
    ttk[7][1] = (utk[2][1]-(((Cik[8][1][2]*tc[8][2])+((Cik[8][1][0]*tc[8][0])+(
      Cik[8][1][1]*tc[8][1])))+((fccikt[8][0]*ri[3][2])-(fccikt[8][2]*ri[3][0]))
      ));
    ttk[7][2] = (utk[2][2]-(((Cik[8][2][2]*tc[8][2])+((Cik[8][2][0]*tc[8][0])+(
      Cik[8][2][1]*tc[8][1])))+((fccikt[8][1]*ri[3][0])-(fccikt[8][0]*ri[3][1]))
      ));
    fc[7][0] = ((mk[2]*(AnkAtk[7][0]-gk[7][0]))-ffk[7][0]);
    fc[7][1] = ((mk[2]*(AnkAtk[7][1]-gk[7][1]))-ffk[7][1]);
    fc[7][2] = ((mk[2]*(AnkAtk[7][2]-gk[7][2]))-ffk[7][2]);
    tc[7][0] = ((WkIkWk[7][0]+((ik[2][0][2]*onk[7][2])+((ik[2][0][0]*onk[7][0])+
      (ik[2][0][1]*onk[7][1]))))-(ttk[7][0]+((fc[7][2]*rk[2][1])-(fc[7][1]*
      rk[2][2]))));
    tc[7][1] = ((WkIkWk[7][1]+((ik[2][1][2]*onk[7][2])+((ik[2][1][0]*onk[7][0])+
      (ik[2][1][1]*onk[7][1]))))-(ttk[7][1]+((fc[7][0]*rk[2][2])-(fc[7][2]*
      rk[2][0]))));
    tc[7][2] = ((WkIkWk[7][2]+((ik[2][2][2]*onk[7][2])+((ik[2][2][0]*onk[7][0])+
      (ik[2][2][1]*onk[7][1]))))-(ttk[7][2]+((fc[7][1]*rk[2][0])-(fc[7][0]*
      rk[2][1]))));
    fccikt[7][0] = ((Cik[7][0][2]*fc[7][2])+((Cik[7][0][0]*fc[7][0])+(
      Cik[7][0][1]*fc[7][1])));
    fccikt[7][1] = ((Cik[7][1][2]*fc[7][2])+((Cik[7][1][0]*fc[7][0])+(
      Cik[7][1][1]*fc[7][1])));
    fccikt[7][2] = ((Cik[7][2][2]*fc[7][2])+((Cik[7][2][0]*fc[7][0])+(
      Cik[7][2][1]*fc[7][1])));
    ffk[6][0] = (ufk[1][0]-fccikt[7][0]);
    ffk[6][1] = (ufk[1][1]-fccikt[7][1]);
    ffk[6][2] = (ufk[1][2]-fccikt[7][2]);
    ttk[6][0] = (utk[1][0]-(((Cik[7][0][2]*tc[7][2])+((Cik[7][0][0]*tc[7][0])+(
      Cik[7][0][1]*tc[7][1])))+((fccikt[7][2]*ri[2][1])-(fccikt[7][1]*ri[2][2]))
      ));
    ttk[6][1] = (utk[1][1]-(((Cik[7][1][2]*tc[7][2])+((Cik[7][1][0]*tc[7][0])+(
      Cik[7][1][1]*tc[7][1])))+((fccikt[7][0]*ri[2][2])-(fccikt[7][2]*ri[2][0]))
      ));
    ttk[6][2] = (utk[1][2]-(((Cik[7][2][2]*tc[7][2])+((Cik[7][2][0]*tc[7][0])+(
      Cik[7][2][1]*tc[7][1])))+((fccikt[7][1]*ri[2][0])-(fccikt[7][0]*ri[2][1]))
      ));
    fc[6][0] = ((mk[1]*(AnkAtk[6][0]-gk[6][0]))-ffk[6][0]);
    fc[6][1] = ((mk[1]*(AnkAtk[6][1]-gk[6][1]))-ffk[6][1]);
    fc[6][2] = ((mk[1]*(AnkAtk[6][2]-gk[6][2]))-ffk[6][2]);
    tc[6][0] = ((WkIkWk[6][0]+((ik[1][0][2]*onk[6][2])+((ik[1][0][0]*onk[6][0])+
      (ik[1][0][1]*onk[6][1]))))-(ttk[6][0]+((fc[6][2]*rk[1][1])-(fc[6][1]*
      rk[1][2]))));
    tc[6][1] = ((WkIkWk[6][1]+((ik[1][1][2]*onk[6][2])+((ik[1][1][0]*onk[6][0])+
      (ik[1][1][1]*onk[6][1]))))-(ttk[6][1]+((fc[6][0]*rk[1][2])-(fc[6][2]*
      rk[1][0]))));
    tc[6][2] = ((WkIkWk[6][2]+((ik[1][2][2]*onk[6][2])+((ik[1][2][0]*onk[6][0])+
      (ik[1][2][1]*onk[6][1]))))-(ttk[6][2]+((fc[6][1]*rk[1][0])-(fc[6][0]*
      rk[1][1]))));
    fccikt[6][0] = ((Cik[6][0][2]*fc[6][2])+((Cik[6][0][0]*fc[6][0])+(
      Cik[6][0][1]*fc[6][1])));
    fccikt[6][1] = ((Cik[6][1][2]*fc[6][2])+((Cik[6][1][0]*fc[6][0])+(
      Cik[6][1][1]*fc[6][1])));
    fccikt[6][2] = ((Cik[6][2][2]*fc[6][2])+((Cik[6][2][0]*fc[6][0])+(
      Cik[6][2][1]*fc[6][1])));
    ffk[5][0] = (ffk[5][0]-fccikt[6][0]);
    ffk[5][1] = (ffk[5][1]-fccikt[6][1]);
    ffk[5][2] = (ffk[5][2]-fccikt[6][2]);
    ttk[5][0] = (ttk[5][0]-(((Cik[6][0][2]*tc[6][2])+((Cik[6][0][0]*tc[6][0])+(
      Cik[6][0][1]*tc[6][1])))+((fccikt[6][2]*ri[1][1])-(fccikt[6][1]*ri[1][2]))
      ));
    ttk[5][1] = (ttk[5][1]-(((Cik[6][1][2]*tc[6][2])+((Cik[6][1][0]*tc[6][0])+(
      Cik[6][1][1]*tc[6][1])))+((fccikt[6][0]*ri[1][2])-(fccikt[6][2]*ri[1][0]))
      ));
    ttk[5][2] = (ttk[5][2]-(((Cik[6][2][2]*tc[6][2])+((Cik[6][2][0]*tc[6][0])+(
      Cik[6][2][1]*tc[6][1])))+((fccikt[6][1]*ri[1][0])-(fccikt[6][0]*ri[1][1]))
      ));
    fc[5][0] = ((mk[0]*(AnkAtk[5][0]-gk[3][0]))-ffk[5][0]);
    fc[5][1] = ((mk[0]*(AnkAtk[5][1]-gk[3][1]))-ffk[5][1]);
    fc[5][2] = ((mk[0]*(AnkAtk[5][2]-gk[3][2]))-ffk[5][2]);
    tc[5][0] = ((WkIkWk[5][0]+((ik[0][0][2]*udot[5])+((ik[0][0][0]*udot[3])+(
      ik[0][0][1]*udot[4]))))-(ttk[5][0]+((fc[5][2]*rk[0][1])-(fc[5][1]*rk[0][2]
      ))));
    tc[5][1] = ((WkIkWk[5][1]+((ik[0][1][2]*udot[5])+((ik[0][1][0]*udot[3])+(
      ik[0][1][1]*udot[4]))))-(ttk[5][1]+((fc[5][0]*rk[0][2])-(fc[5][2]*rk[0][0]
      ))));
    tc[5][2] = ((WkIkWk[5][2]+((ik[0][2][2]*udot[5])+((ik[0][2][0]*udot[3])+(
      ik[0][2][1]*udot[4]))))-(ttk[5][2]+((fc[5][1]*rk[0][0])-(fc[5][0]*rk[0][1]
      ))));
    fccikt[5][0] = fc[5][0];
    fccikt[5][1] = fc[5][1];
    fccikt[5][2] = fc[5][2];
    ffk[4][0] = -fccikt[5][0];
    ffk[4][1] = -fccikt[5][1];
    ffk[4][2] = -fccikt[5][2];
    ttk[4][0] = -tc[5][0];
    ttk[4][1] = -tc[5][1];
    ttk[4][2] = -tc[5][2];
    fc[4][0] = -ffk[4][0];
    fc[4][1] = -ffk[4][1];
    fc[4][2] = -ffk[4][2];
    tc[4][0] = -ttk[4][0];
    tc[4][1] = -ttk[4][1];
    tc[4][2] = -ttk[4][2];
    fccikt[4][0] = fc[4][0];
    fccikt[4][1] = fc[4][1];
    fccikt[4][2] = fc[4][2];
    ffk[3][0] = -fccikt[4][0];
    ffk[3][1] = -fccikt[4][1];
    ffk[3][2] = -fccikt[4][2];
    ttk[3][0] = -tc[4][0];
    ttk[3][1] = -tc[4][1];
    ttk[3][2] = -tc[4][2];
    fc[3][0] = -ffk[3][0];
    fc[3][1] = -ffk[3][1];
    fc[3][2] = -ffk[3][2];
    tc[3][0] = -ttk[3][0];
    tc[3][1] = -ttk[3][1];
    tc[3][2] = -ttk[3][2];
    fccikt[3][0] = ((Cik[3][0][2]*fc[3][2])+((Cik[3][0][0]*fc[3][0])+(
      Cik[3][0][1]*fc[3][1])));
    fccikt[3][1] = ((Cik[3][1][2]*fc[3][2])+((Cik[3][1][0]*fc[3][0])+(
      Cik[3][1][1]*fc[3][1])));
    fccikt[3][2] = ((Cik[3][2][2]*fc[3][2])+((Cik[3][2][0]*fc[3][0])+(
      Cik[3][2][1]*fc[3][1])));
    ffk[2][0] = -fccikt[3][0];
    ffk[2][1] = -fccikt[3][1];
    ffk[2][2] = -fccikt[3][2];
    ttk[2][0] = -((Cik[3][0][2]*tc[3][2])+((Cik[3][0][0]*tc[3][0])+(Cik[3][0][1]
      *tc[3][1])));
    ttk[2][1] = -((Cik[3][1][2]*tc[3][2])+((Cik[3][1][0]*tc[3][0])+(Cik[3][1][1]
      *tc[3][1])));
    ttk[2][2] = -((Cik[3][2][2]*tc[3][2])+((Cik[3][2][0]*tc[3][0])+(Cik[3][2][1]
      *tc[3][1])));
    fc[2][0] = -ffk[2][0];
    fc[2][1] = -ffk[2][1];
    fc[2][2] = -ffk[2][2];
    tc[2][0] = -ttk[2][0];
    tc[2][1] = -ttk[2][1];
    tc[2][2] = -ttk[2][2];
    fccikt[2][0] = fc[2][0];
    fccikt[2][1] = fc[2][1];
    fccikt[2][2] = fc[2][2];
    ffk[1][0] = -fccikt[2][0];
    ffk[1][1] = -fccikt[2][1];
    ffk[1][2] = -fccikt[2][2];
    ttk[1][0] = -(tc[2][0]+((fccikt[2][2]*rpp[2][1])-(fccikt[2][1]*rpp[2][2])));
    ttk[1][1] = -(tc[2][1]+((fccikt[2][0]*rpp[2][2])-(fccikt[2][2]*rpp[2][0])));
    ttk[1][2] = -(tc[2][2]+((fccikt[2][1]*rpp[2][0])-(fccikt[2][0]*rpp[2][1])));
    fc[1][0] = -ffk[1][0];
    fc[1][1] = -ffk[1][1];
    fc[1][2] = -ffk[1][2];
    tc[1][0] = -ttk[1][0];
    tc[1][1] = -ttk[1][1];
    tc[1][2] = -ttk[1][2];
    fccikt[1][0] = fc[1][0];
    fccikt[1][1] = fc[1][1];
    fccikt[1][2] = fc[1][2];
    ffk[0][0] = -fccikt[1][0];
    ffk[0][1] = -fccikt[1][1];
    ffk[0][2] = -fccikt[1][2];
    ttk[0][0] = -(tc[1][0]+((fccikt[1][2]*rpp[1][1])-(fccikt[1][1]*rpp[1][2])));
    ttk[0][1] = -(tc[1][1]+((fccikt[1][0]*rpp[1][2])-(fccikt[1][2]*rpp[1][0])));
    ttk[0][2] = -(tc[1][2]+((fccikt[1][1]*rpp[1][0])-(fccikt[1][0]*rpp[1][1])));
    fc[0][0] = -ffk[0][0];
    fc[0][1] = -ffk[0][1];
    fc[0][2] = -ffk[0][2];
    tc[0][0] = -ttk[0][0];
    tc[0][1] = -ttk[0][1];
    tc[0][2] = -ttk[0][2];
    force[0][0] = fc[5][0];
    torque[0][0] = tc[5][0];
    force[0][1] = fc[5][1];
    torque[0][1] = tc[5][1];
    force[0][2] = fc[5][2];
    torque[0][2] = tc[5][2];
    force[1][0] = fc[6][0];
    torque[1][0] = tc[6][0];
    force[1][1] = fc[6][1];
    torque[1][1] = tc[6][1];
    force[1][2] = fc[6][2];
    torque[1][2] = tc[6][2];
    force[2][0] = fc[7][0];
    torque[2][0] = tc[7][0];
    force[2][1] = fc[7][1];
    torque[2][1] = tc[7][1];
    force[2][2] = fc[7][2];
    torque[2][2] = tc[7][2];
    force[3][0] = fc[8][0];
    torque[3][0] = tc[8][0];
    force[3][1] = fc[8][1];
    torque[3][1] = tc[8][1];
    force[3][2] = fc[8][2];
    torque[3][2] = tc[8][2];
    force[4][0] = fc[9][0];
    torque[4][0] = tc[9][0];
    force[4][1] = fc[9][1];
    torque[4][1] = tc[9][1];
    force[4][2] = fc[9][2];
    torque[4][2] = tc[9][2];
    force[5][0] = fc[10][0];
    torque[5][0] = tc[10][0];
    force[5][1] = fc[10][1];
    torque[5][1] = tc[10][1];
    force[5][2] = fc[10][2];
    torque[5][2] = tc[10][2];
    force[6][0] = fc[11][0];
    torque[6][0] = tc[11][0];
    force[6][1] = fc[11][1];
    torque[6][1] = tc[11][1];
    force[6][2] = fc[11][2];
    torque[6][2] = tc[11][2];
    force[7][0] = fc[12][0];
    torque[7][0] = tc[12][0];
    force[7][1] = fc[12][1];
    torque[7][1] = tc[12][1];
    force[7][2] = fc[12][2];
    torque[7][2] = tc[12][2];
    force[8][0] = fc[13][0];
    torque[8][0] = tc[13][0];
    force[8][1] = fc[13][1];
    torque[8][1] = tc[13][1];
    force[8][2] = fc[13][2];
    torque[8][2] = tc[13][2];
    force[9][0] = fc[14][0];
    torque[9][0] = tc[14][0];
    force[9][1] = fc[14][1];
    torque[9][1] = tc[14][1];
    force[9][2] = fc[14][2];
    torque[9][2] = tc[14][2];
    force[10][0] = fc[15][0];
    torque[10][0] = tc[15][0];
    force[10][1] = fc[15][1];
    torque[10][1] = tc[15][1];
    force[10][2] = fc[15][2];
    torque[10][2] = tc[15][2];
    force[11][0] = fc[16][0];
    torque[11][0] = tc[16][0];
    force[11][1] = fc[16][1];
    torque[11][1] = tc[16][1];
    force[11][2] = fc[16][2];
    torque[11][2] = tc[16][2];
    force[12][0] = fc[17][0];
    torque[12][0] = tc[17][0];
    force[12][1] = fc[17][1];
    torque[12][1] = tc[17][1];
    force[12][2] = fc[17][2];
    torque[12][2] = tc[17][2];
    force[13][0] = fc[18][0];
    torque[13][0] = tc[18][0];
    force[13][1] = fc[18][1];
    torque[13][1] = tc[18][1];
    force[13][2] = fc[18][2];
    torque[13][2] = tc[18][2];
    force[14][0] = fc[19][0];
    torque[14][0] = tc[19][0];
    force[14][1] = fc[19][1];
    torque[14][1] = tc[19][1];
    force[14][2] = fc[19][2];
    torque[14][2] = tc[19][2];
    force[15][0] = fc[20][0];
    torque[15][0] = tc[20][0];
    force[15][1] = fc[20][1];
    torque[15][1] = tc[20][1];
    force[15][2] = fc[20][2];
    torque[15][2] = tc[20][2];
/*
Compute reaction forces for tree weld joints
*/
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  828 adds/subtracts/negates
                    678 multiplies
                      0 divides
                    402 assignments
*/
}

void sdmom(double lm[3],
    double am[3],
    double *ke)
{
/*
Compute system linear and angular momentum, and kinetic energy.

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
    double lk[16][3],hnk[16][3];

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(19,23);
        return;
    }
    lk[0][0] = (mk[0]*vnk[5][0]);
    lk[0][1] = (mk[0]*vnk[5][1]);
    lk[0][2] = (mk[0]*vnk[5][2]);
    lk[1][0] = (mk[1]*vnk[6][0]);
    lk[1][1] = (mk[1]*vnk[6][1]);
    lk[1][2] = (mk[1]*vnk[6][2]);
    lk[2][0] = (mk[2]*vnk[7][0]);
    lk[2][1] = (mk[2]*vnk[7][1]);
    lk[2][2] = (mk[2]*vnk[7][2]);
    lk[3][0] = (mk[3]*vnk[8][0]);
    lk[3][1] = (mk[3]*vnk[8][1]);
    lk[3][2] = (mk[3]*vnk[8][2]);
    lk[4][0] = (mk[4]*vnk[9][0]);
    lk[4][1] = (mk[4]*vnk[9][1]);
    lk[4][2] = (mk[4]*vnk[9][2]);
    lk[5][0] = (mk[5]*vnk[10][0]);
    lk[5][1] = (mk[5]*vnk[10][1]);
    lk[5][2] = (mk[5]*vnk[10][2]);
    lk[6][0] = (mk[6]*vnk[11][0]);
    lk[6][1] = (mk[6]*vnk[11][1]);
    lk[6][2] = (mk[6]*vnk[11][2]);
    lk[7][0] = (mk[7]*vnk[12][0]);
    lk[7][1] = (mk[7]*vnk[12][1]);
    lk[7][2] = (mk[7]*vnk[12][2]);
    lk[8][0] = (mk[8]*vnk[13][0]);
    lk[8][1] = (mk[8]*vnk[13][1]);
    lk[8][2] = (mk[8]*vnk[13][2]);
    lk[9][0] = (mk[9]*vnk[14][0]);
    lk[9][1] = (mk[9]*vnk[14][1]);
    lk[9][2] = (mk[9]*vnk[14][2]);
    lk[10][0] = (mk[10]*vnk[15][0]);
    lk[10][1] = (mk[10]*vnk[15][1]);
    lk[10][2] = (mk[10]*vnk[15][2]);
    lk[11][0] = (mk[11]*vnk[16][0]);
    lk[11][1] = (mk[11]*vnk[16][1]);
    lk[11][2] = (mk[11]*vnk[16][2]);
    lk[12][0] = (mk[12]*vnk[17][0]);
    lk[12][1] = (mk[12]*vnk[17][1]);
    lk[12][2] = (mk[12]*vnk[17][2]);
    lk[13][0] = (mk[13]*vnk[18][0]);
    lk[13][1] = (mk[13]*vnk[18][1]);
    lk[13][2] = (mk[13]*vnk[18][2]);
    lk[14][0] = (mk[14]*vnk[19][0]);
    lk[14][1] = (mk[14]*vnk[19][1]);
    lk[14][2] = (mk[14]*vnk[19][2]);
    lk[15][0] = (mk[15]*vnk[20][0]);
    lk[15][1] = (mk[15]*vnk[20][1]);
    lk[15][2] = (mk[15]*vnk[20][2]);
    hnk[0][0] = ((ik[0][0][2]*u[5])+((ik[0][0][0]*u[3])+(ik[0][0][1]*u[4])));
    hnk[0][1] = ((ik[0][1][2]*u[5])+((ik[0][1][0]*u[3])+(ik[0][1][1]*u[4])));
    hnk[0][2] = ((ik[0][2][2]*u[5])+((ik[0][2][0]*u[3])+(ik[0][2][1]*u[4])));
    hnk[1][0] = ((ik[1][0][2]*wk[6][2])+((ik[1][0][0]*wk[6][0])+(ik[1][0][1]*
      wk[6][1])));
    hnk[1][1] = ((ik[1][1][2]*wk[6][2])+((ik[1][1][0]*wk[6][0])+(ik[1][1][1]*
      wk[6][1])));
    hnk[1][2] = ((ik[1][2][2]*wk[6][2])+((ik[1][2][0]*wk[6][0])+(ik[1][2][1]*
      wk[6][1])));
    hnk[2][0] = ((ik[2][0][2]*wk[7][2])+((ik[2][0][0]*wk[7][0])+(ik[2][0][1]*
      wk[7][1])));
    hnk[2][1] = ((ik[2][1][2]*wk[7][2])+((ik[2][1][0]*wk[7][0])+(ik[2][1][1]*
      wk[7][1])));
    hnk[2][2] = ((ik[2][2][2]*wk[7][2])+((ik[2][2][0]*wk[7][0])+(ik[2][2][1]*
      wk[7][1])));
    hnk[3][0] = ((ik[3][0][2]*wk[8][2])+((ik[3][0][0]*wk[8][0])+(ik[3][0][1]*
      wk[8][1])));
    hnk[3][1] = ((ik[3][1][2]*wk[8][2])+((ik[3][1][0]*wk[8][0])+(ik[3][1][1]*
      wk[8][1])));
    hnk[3][2] = ((ik[3][2][2]*wk[8][2])+((ik[3][2][0]*wk[8][0])+(ik[3][2][1]*
      wk[8][1])));
    hnk[4][0] = ((ik[4][0][2]*wk[9][2])+((ik[4][0][0]*wk[9][0])+(ik[4][0][1]*
      wk[9][1])));
    hnk[4][1] = ((ik[4][1][2]*wk[9][2])+((ik[4][1][0]*wk[9][0])+(ik[4][1][1]*
      wk[9][1])));
    hnk[4][2] = ((ik[4][2][2]*wk[9][2])+((ik[4][2][0]*wk[9][0])+(ik[4][2][1]*
      wk[9][1])));
    hnk[5][0] = ((ik[5][0][2]*wk[10][2])+((ik[5][0][0]*wk[10][0])+(ik[5][0][1]*
      wk[10][1])));
    hnk[5][1] = ((ik[5][1][2]*wk[10][2])+((ik[5][1][0]*wk[10][0])+(ik[5][1][1]*
      wk[10][1])));
    hnk[5][2] = ((ik[5][2][2]*wk[10][2])+((ik[5][2][0]*wk[10][0])+(ik[5][2][1]*
      wk[10][1])));
    hnk[6][0] = ((ik[6][0][2]*wk[11][2])+((ik[6][0][0]*wk[11][0])+(ik[6][0][1]*
      wk[11][1])));
    hnk[6][1] = ((ik[6][1][2]*wk[11][2])+((ik[6][1][0]*wk[11][0])+(ik[6][1][1]*
      wk[11][1])));
    hnk[6][2] = ((ik[6][2][2]*wk[11][2])+((ik[6][2][0]*wk[11][0])+(ik[6][2][1]*
      wk[11][1])));
    hnk[7][0] = ((ik[7][0][2]*wk[12][2])+((ik[7][0][0]*wk[12][0])+(ik[7][0][1]*
      wk[12][1])));
    hnk[7][1] = ((ik[7][1][2]*wk[12][2])+((ik[7][1][0]*wk[12][0])+(ik[7][1][1]*
      wk[12][1])));
    hnk[7][2] = ((ik[7][2][2]*wk[12][2])+((ik[7][2][0]*wk[12][0])+(ik[7][2][1]*
      wk[12][1])));
    hnk[8][0] = ((ik[8][0][2]*wk[13][2])+((ik[8][0][0]*wk[13][0])+(ik[8][0][1]*
      wk[13][1])));
    hnk[8][1] = ((ik[8][1][2]*wk[13][2])+((ik[8][1][0]*wk[13][0])+(ik[8][1][1]*
      wk[13][1])));
    hnk[8][2] = ((ik[8][2][2]*wk[13][2])+((ik[8][2][0]*wk[13][0])+(ik[8][2][1]*
      wk[13][1])));
    hnk[9][0] = ((ik[9][0][2]*wk[14][2])+((ik[9][0][0]*wk[14][0])+(ik[9][0][1]*
      wk[14][1])));
    hnk[9][1] = ((ik[9][1][2]*wk[14][2])+((ik[9][1][0]*wk[14][0])+(ik[9][1][1]*
      wk[14][1])));
    hnk[9][2] = ((ik[9][2][2]*wk[14][2])+((ik[9][2][0]*wk[14][0])+(ik[9][2][1]*
      wk[14][1])));
    hnk[10][0] = ((ik[10][0][2]*wk[15][2])+((ik[10][0][0]*wk[15][0])+(
      ik[10][0][1]*wk[15][1])));
    hnk[10][1] = ((ik[10][1][2]*wk[15][2])+((ik[10][1][0]*wk[15][0])+(
      ik[10][1][1]*wk[15][1])));
    hnk[10][2] = ((ik[10][2][2]*wk[15][2])+((ik[10][2][0]*wk[15][0])+(
      ik[10][2][1]*wk[15][1])));
    hnk[11][0] = ((ik[11][0][2]*wk[16][2])+((ik[11][0][0]*wk[16][0])+(
      ik[11][0][1]*wk[16][1])));
    hnk[11][1] = ((ik[11][1][2]*wk[16][2])+((ik[11][1][0]*wk[16][0])+(
      ik[11][1][1]*wk[16][1])));
    hnk[11][2] = ((ik[11][2][2]*wk[16][2])+((ik[11][2][0]*wk[16][0])+(
      ik[11][2][1]*wk[16][1])));
    hnk[12][0] = ((ik[12][0][2]*wk[17][2])+((ik[12][0][0]*wk[17][0])+(
      ik[12][0][1]*wk[17][1])));
    hnk[12][1] = ((ik[12][1][2]*wk[17][2])+((ik[12][1][0]*wk[17][0])+(
      ik[12][1][1]*wk[17][1])));
    hnk[12][2] = ((ik[12][2][2]*wk[17][2])+((ik[12][2][0]*wk[17][0])+(
      ik[12][2][1]*wk[17][1])));
    hnk[13][0] = ((ik[13][0][2]*wk[18][2])+((ik[13][0][0]*wk[18][0])+(
      ik[13][0][1]*wk[18][1])));
    hnk[13][1] = ((ik[13][1][2]*wk[18][2])+((ik[13][1][0]*wk[18][0])+(
      ik[13][1][1]*wk[18][1])));
    hnk[13][2] = ((ik[13][2][2]*wk[18][2])+((ik[13][2][0]*wk[18][0])+(
      ik[13][2][1]*wk[18][1])));
    hnk[14][0] = ((ik[14][0][2]*wk[19][2])+((ik[14][0][0]*wk[19][0])+(
      ik[14][0][1]*wk[19][1])));
    hnk[14][1] = ((ik[14][1][2]*wk[19][2])+((ik[14][1][0]*wk[19][0])+(
      ik[14][1][1]*wk[19][1])));
    hnk[14][2] = ((ik[14][2][2]*wk[19][2])+((ik[14][2][0]*wk[19][0])+(
      ik[14][2][1]*wk[19][1])));
    hnk[15][0] = ((ik[15][0][2]*wk[20][2])+((ik[15][0][0]*wk[20][0])+(
      ik[15][0][1]*wk[20][1])));
    hnk[15][1] = ((ik[15][1][2]*wk[20][2])+((ik[15][1][0]*wk[20][0])+(
      ik[15][1][1]*wk[20][1])));
    hnk[15][2] = ((ik[15][2][2]*wk[20][2])+((ik[15][2][0]*wk[20][0])+(
      ik[15][2][1]*wk[20][1])));
    lm[0] = (lk[15][0]+(lk[14][0]+(lk[13][0]+(lk[12][0]+(lk[11][0]+(lk[10][0]+(
      lk[9][0]+(lk[8][0]+(lk[7][0]+(lk[6][0]+(lk[5][0]+(lk[4][0]+(lk[3][0]+(
      lk[2][0]+(lk[0][0]+lk[1][0])))))))))))))));
    lm[1] = (lk[15][1]+(lk[14][1]+(lk[13][1]+(lk[12][1]+(lk[11][1]+(lk[10][1]+(
      lk[9][1]+(lk[8][1]+(lk[7][1]+(lk[6][1]+(lk[5][1]+(lk[4][1]+(lk[3][1]+(
      lk[2][1]+(lk[0][1]+lk[1][1])))))))))))))));
    lm[2] = (lk[15][2]+(lk[14][2]+(lk[13][2]+(lk[12][2]+(lk[11][2]+(lk[10][2]+(
      lk[9][2]+(lk[8][2]+(lk[7][2]+(lk[6][2]+(lk[5][2]+(lk[4][2]+(lk[3][2]+(
      lk[2][2]+(lk[0][2]+lk[1][2])))))))))))))));
    temp[0] = ((((cnk[7][0][2]*hnk[2][2])+((cnk[7][0][0]*hnk[2][0])+(
      cnk[7][0][1]*hnk[2][1])))+((lk[2][2]*rnk[7][1])-(lk[2][1]*rnk[7][2])))+(((
      (Cik[3][0][2]*hnk[0][2])+((Cik[3][0][0]*hnk[0][0])+(Cik[3][0][1]*hnk[0][1]
      )))+((lk[0][2]*rnk[5][1])-(lk[0][1]*rnk[5][2])))+(((cnk[6][0][2]*hnk[1][2]
      )+((cnk[6][0][0]*hnk[1][0])+(cnk[6][0][1]*hnk[1][1])))+((lk[1][2]*
      rnk[6][1])-(lk[1][1]*rnk[6][2])))));
    temp[1] = ((((cnk[9][0][2]*hnk[4][2])+((cnk[9][0][0]*hnk[4][0])+(
      cnk[9][0][1]*hnk[4][1])))+((lk[4][2]*rnk[9][1])-(lk[4][1]*rnk[9][2])))+(((
      (cnk[8][0][2]*hnk[3][2])+((cnk[8][0][0]*hnk[3][0])+(cnk[8][0][1]*hnk[3][1]
      )))+((lk[3][2]*rnk[8][1])-(lk[3][1]*rnk[8][2])))+temp[0]));
    temp[2] = ((((cnk[11][0][2]*hnk[6][2])+((cnk[11][0][0]*hnk[6][0])+(
      cnk[11][0][1]*hnk[6][1])))+((lk[6][2]*rnk[11][1])-(lk[6][1]*rnk[11][2])))+
      ((((cnk[10][0][2]*hnk[5][2])+((cnk[10][0][0]*hnk[5][0])+(cnk[10][0][1]*
      hnk[5][1])))+((lk[5][2]*rnk[10][1])-(lk[5][1]*rnk[10][2])))+temp[1]));
    temp[3] = ((((cnk[13][0][2]*hnk[8][2])+((cnk[13][0][0]*hnk[8][0])+(
      cnk[13][0][1]*hnk[8][1])))+((lk[8][2]*rnk[13][1])-(lk[8][1]*rnk[13][2])))+
      ((((cnk[12][0][2]*hnk[7][2])+((cnk[12][0][0]*hnk[7][0])+(cnk[12][0][1]*
      hnk[7][1])))+((lk[7][2]*rnk[12][1])-(lk[7][1]*rnk[12][2])))+temp[2]));
    temp[4] = ((((cnk[15][0][2]*hnk[10][2])+((cnk[15][0][0]*hnk[10][0])+(
      cnk[15][0][1]*hnk[10][1])))+((lk[10][2]*rnk[15][1])-(lk[10][1]*rnk[15][2])
      ))+((((cnk[14][0][2]*hnk[9][2])+((cnk[14][0][0]*hnk[9][0])+(cnk[14][0][1]*
      hnk[9][1])))+((lk[9][2]*rnk[14][1])-(lk[9][1]*rnk[14][2])))+temp[3]));
    temp[5] = ((((cnk[17][0][2]*hnk[12][2])+((cnk[17][0][0]*hnk[12][0])+(
      cnk[17][0][1]*hnk[12][1])))+((lk[12][2]*rnk[17][1])-(lk[12][1]*rnk[17][2])
      ))+((((cnk[16][0][2]*hnk[11][2])+((cnk[16][0][0]*hnk[11][0])+(
      cnk[16][0][1]*hnk[11][1])))+((lk[11][2]*rnk[16][1])-(lk[11][1]*rnk[16][2])
      ))+temp[4]));
    temp[6] = ((((cnk[19][0][2]*hnk[14][2])+((cnk[19][0][0]*hnk[14][0])+(
      cnk[19][0][1]*hnk[14][1])))+((lk[14][2]*rnk[19][1])-(lk[14][1]*rnk[19][2])
      ))+((((cnk[18][0][2]*hnk[13][2])+((cnk[18][0][0]*hnk[13][0])+(
      cnk[18][0][1]*hnk[13][1])))+((lk[13][2]*rnk[18][1])-(lk[13][1]*rnk[18][2])
      ))+temp[5]));
    am[0] = (((((cnk[20][0][2]*hnk[15][2])+((cnk[20][0][0]*hnk[15][0])+(
      cnk[20][0][1]*hnk[15][1])))+((lk[15][2]*rnk[20][1])-(lk[15][1]*rnk[20][2])
      ))+temp[6])-((com[1]*lm[2])-(com[2]*lm[1])));
    temp[0] = ((((cnk[7][1][2]*hnk[2][2])+((cnk[7][1][0]*hnk[2][0])+(
      cnk[7][1][1]*hnk[2][1])))+((lk[2][0]*rnk[7][2])-(lk[2][2]*rnk[7][0])))+(((
      (Cik[3][1][2]*hnk[0][2])+((Cik[3][1][0]*hnk[0][0])+(Cik[3][1][1]*hnk[0][1]
      )))+((lk[0][0]*rnk[5][2])-(lk[0][2]*rnk[5][0])))+(((cnk[6][1][2]*hnk[1][2]
      )+((cnk[6][1][0]*hnk[1][0])+(cnk[6][1][1]*hnk[1][1])))+((lk[1][0]*
      rnk[6][2])-(lk[1][2]*rnk[6][0])))));
    temp[1] = ((((cnk[9][1][2]*hnk[4][2])+((cnk[9][1][0]*hnk[4][0])+(
      cnk[9][1][1]*hnk[4][1])))+((lk[4][0]*rnk[9][2])-(lk[4][2]*rnk[9][0])))+(((
      (cnk[8][1][2]*hnk[3][2])+((cnk[8][1][0]*hnk[3][0])+(cnk[8][1][1]*hnk[3][1]
      )))+((lk[3][0]*rnk[8][2])-(lk[3][2]*rnk[8][0])))+temp[0]));
    temp[2] = ((((cnk[11][1][2]*hnk[6][2])+((cnk[11][1][0]*hnk[6][0])+(
      cnk[11][1][1]*hnk[6][1])))+((lk[6][0]*rnk[11][2])-(lk[6][2]*rnk[11][0])))+
      ((((cnk[10][1][2]*hnk[5][2])+((cnk[10][1][0]*hnk[5][0])+(cnk[10][1][1]*
      hnk[5][1])))+((lk[5][0]*rnk[10][2])-(lk[5][2]*rnk[10][0])))+temp[1]));
    temp[3] = ((((cnk[13][1][2]*hnk[8][2])+((cnk[13][1][0]*hnk[8][0])+(
      cnk[13][1][1]*hnk[8][1])))+((lk[8][0]*rnk[13][2])-(lk[8][2]*rnk[13][0])))+
      ((((cnk[12][1][2]*hnk[7][2])+((cnk[12][1][0]*hnk[7][0])+(cnk[12][1][1]*
      hnk[7][1])))+((lk[7][0]*rnk[12][2])-(lk[7][2]*rnk[12][0])))+temp[2]));
    temp[4] = ((((cnk[15][1][2]*hnk[10][2])+((cnk[15][1][0]*hnk[10][0])+(
      cnk[15][1][1]*hnk[10][1])))+((lk[10][0]*rnk[15][2])-(lk[10][2]*rnk[15][0])
      ))+((((cnk[14][1][2]*hnk[9][2])+((cnk[14][1][0]*hnk[9][0])+(cnk[14][1][1]*
      hnk[9][1])))+((lk[9][0]*rnk[14][2])-(lk[9][2]*rnk[14][0])))+temp[3]));
    temp[5] = ((((cnk[17][1][2]*hnk[12][2])+((cnk[17][1][0]*hnk[12][0])+(
      cnk[17][1][1]*hnk[12][1])))+((lk[12][0]*rnk[17][2])-(lk[12][2]*rnk[17][0])
      ))+((((cnk[16][1][2]*hnk[11][2])+((cnk[16][1][0]*hnk[11][0])+(
      cnk[16][1][1]*hnk[11][1])))+((lk[11][0]*rnk[16][2])-(lk[11][2]*rnk[16][0])
      ))+temp[4]));
    temp[6] = ((((cnk[19][1][2]*hnk[14][2])+((cnk[19][1][0]*hnk[14][0])+(
      cnk[19][1][1]*hnk[14][1])))+((lk[14][0]*rnk[19][2])-(lk[14][2]*rnk[19][0])
      ))+((((cnk[18][1][2]*hnk[13][2])+((cnk[18][1][0]*hnk[13][0])+(
      cnk[18][1][1]*hnk[13][1])))+((lk[13][0]*rnk[18][2])-(lk[13][2]*rnk[18][0])
      ))+temp[5]));
    am[1] = (((((cnk[20][1][2]*hnk[15][2])+((cnk[20][1][0]*hnk[15][0])+(
      cnk[20][1][1]*hnk[15][1])))+((lk[15][0]*rnk[20][2])-(lk[15][2]*rnk[20][0])
      ))+temp[6])-((com[2]*lm[0])-(com[0]*lm[2])));
    temp[0] = ((((cnk[7][2][2]*hnk[2][2])+((cnk[7][2][0]*hnk[2][0])+(
      cnk[7][2][1]*hnk[2][1])))+((lk[2][1]*rnk[7][0])-(lk[2][0]*rnk[7][1])))+(((
      (Cik[3][2][2]*hnk[0][2])+((Cik[3][2][0]*hnk[0][0])+(Cik[3][2][1]*hnk[0][1]
      )))+((lk[0][1]*rnk[5][0])-(lk[0][0]*rnk[5][1])))+(((cnk[6][2][2]*hnk[1][2]
      )+((cnk[6][2][0]*hnk[1][0])+(cnk[6][2][1]*hnk[1][1])))+((lk[1][1]*
      rnk[6][0])-(lk[1][0]*rnk[6][1])))));
    temp[1] = ((((cnk[9][2][2]*hnk[4][2])+((cnk[9][2][0]*hnk[4][0])+(
      cnk[9][2][1]*hnk[4][1])))+((lk[4][1]*rnk[9][0])-(lk[4][0]*rnk[9][1])))+(((
      (cnk[8][2][2]*hnk[3][2])+((cnk[8][2][0]*hnk[3][0])+(cnk[8][2][1]*hnk[3][1]
      )))+((lk[3][1]*rnk[8][0])-(lk[3][0]*rnk[8][1])))+temp[0]));
    temp[2] = ((((cnk[11][2][2]*hnk[6][2])+((cnk[11][2][0]*hnk[6][0])+(
      cnk[11][2][1]*hnk[6][1])))+((lk[6][1]*rnk[11][0])-(lk[6][0]*rnk[11][1])))+
      ((((cnk[10][2][2]*hnk[5][2])+((cnk[10][2][0]*hnk[5][0])+(cnk[10][2][1]*
      hnk[5][1])))+((lk[5][1]*rnk[10][0])-(lk[5][0]*rnk[10][1])))+temp[1]));
    temp[3] = ((((cnk[13][2][2]*hnk[8][2])+((cnk[13][2][0]*hnk[8][0])+(
      cnk[13][2][1]*hnk[8][1])))+((lk[8][1]*rnk[13][0])-(lk[8][0]*rnk[13][1])))+
      ((((cnk[12][2][2]*hnk[7][2])+((cnk[12][2][0]*hnk[7][0])+(cnk[12][2][1]*
      hnk[7][1])))+((lk[7][1]*rnk[12][0])-(lk[7][0]*rnk[12][1])))+temp[2]));
    temp[4] = ((((cnk[15][2][2]*hnk[10][2])+((cnk[15][2][0]*hnk[10][0])+(
      cnk[15][2][1]*hnk[10][1])))+((lk[10][1]*rnk[15][0])-(lk[10][0]*rnk[15][1])
      ))+((((cnk[14][2][2]*hnk[9][2])+((cnk[14][2][0]*hnk[9][0])+(cnk[14][2][1]*
      hnk[9][1])))+((lk[9][1]*rnk[14][0])-(lk[9][0]*rnk[14][1])))+temp[3]));
    temp[5] = ((((cnk[17][2][2]*hnk[12][2])+((cnk[17][2][0]*hnk[12][0])+(
      cnk[17][2][1]*hnk[12][1])))+((lk[12][1]*rnk[17][0])-(lk[12][0]*rnk[17][1])
      ))+((((cnk[16][2][2]*hnk[11][2])+((cnk[16][2][0]*hnk[11][0])+(
      cnk[16][2][1]*hnk[11][1])))+((lk[11][1]*rnk[16][0])-(lk[11][0]*rnk[16][1])
      ))+temp[4]));
    temp[6] = ((((cnk[19][2][2]*hnk[14][2])+((cnk[19][2][0]*hnk[14][0])+(
      cnk[19][2][1]*hnk[14][1])))+((lk[14][1]*rnk[19][0])-(lk[14][0]*rnk[19][1])
      ))+((((cnk[18][2][2]*hnk[13][2])+((cnk[18][2][0]*hnk[13][0])+(
      cnk[18][2][1]*hnk[13][1])))+((lk[13][1]*rnk[18][0])-(lk[13][0]*rnk[18][1])
      ))+temp[5]));
    am[2] = (((((cnk[20][2][2]*hnk[15][2])+((cnk[20][2][0]*hnk[15][0])+(
      cnk[20][2][1]*hnk[15][1])))+((lk[15][1]*rnk[20][0])-(lk[15][0]*rnk[20][1])
      ))+temp[6])-((com[0]*lm[1])-(com[1]*lm[0])));
    temp[0] = ((((hnk[0][2]*u[5])+((hnk[0][0]*u[3])+(hnk[0][1]*u[4])))+((
      lk[0][2]*vnk[5][2])+((lk[0][0]*vnk[5][0])+(lk[0][1]*vnk[5][1]))))+(((
      hnk[1][2]*wk[6][2])+((hnk[1][0]*wk[6][0])+(hnk[1][1]*wk[6][1])))+((
      lk[1][2]*vnk[6][2])+((lk[1][0]*vnk[6][0])+(lk[1][1]*vnk[6][1])))));
    temp[1] = ((((hnk[3][2]*wk[8][2])+((hnk[3][0]*wk[8][0])+(hnk[3][1]*wk[8][1])
      ))+((lk[3][2]*vnk[8][2])+((lk[3][0]*vnk[8][0])+(lk[3][1]*vnk[8][1]))))+(((
      (hnk[2][2]*wk[7][2])+((hnk[2][0]*wk[7][0])+(hnk[2][1]*wk[7][1])))+((
      lk[2][2]*vnk[7][2])+((lk[2][0]*vnk[7][0])+(lk[2][1]*vnk[7][1]))))+temp[0])
      );
    temp[2] = ((((hnk[5][2]*wk[10][2])+((hnk[5][0]*wk[10][0])+(hnk[5][1]*
      wk[10][1])))+((lk[5][2]*vnk[10][2])+((lk[5][0]*vnk[10][0])+(lk[5][1]*
      vnk[10][1]))))+((((hnk[4][2]*wk[9][2])+((hnk[4][0]*wk[9][0])+(hnk[4][1]*
      wk[9][1])))+((lk[4][2]*vnk[9][2])+((lk[4][0]*vnk[9][0])+(lk[4][1]*
      vnk[9][1]))))+temp[1]));
    temp[3] = ((((hnk[7][2]*wk[12][2])+((hnk[7][0]*wk[12][0])+(hnk[7][1]*
      wk[12][1])))+((lk[7][2]*vnk[12][2])+((lk[7][0]*vnk[12][0])+(lk[7][1]*
      vnk[12][1]))))+((((hnk[6][2]*wk[11][2])+((hnk[6][0]*wk[11][0])+(hnk[6][1]*
      wk[11][1])))+((lk[6][2]*vnk[11][2])+((lk[6][0]*vnk[11][0])+(lk[6][1]*
      vnk[11][1]))))+temp[2]));
    temp[4] = ((((hnk[9][2]*wk[14][2])+((hnk[9][0]*wk[14][0])+(hnk[9][1]*
      wk[14][1])))+((lk[9][2]*vnk[14][2])+((lk[9][0]*vnk[14][0])+(lk[9][1]*
      vnk[14][1]))))+((((hnk[8][2]*wk[13][2])+((hnk[8][0]*wk[13][0])+(hnk[8][1]*
      wk[13][1])))+((lk[8][2]*vnk[13][2])+((lk[8][0]*vnk[13][0])+(lk[8][1]*
      vnk[13][1]))))+temp[3]));
    temp[5] = ((((hnk[11][2]*wk[16][2])+((hnk[11][0]*wk[16][0])+(hnk[11][1]*
      wk[16][1])))+((lk[11][2]*vnk[16][2])+((lk[11][0]*vnk[16][0])+(lk[11][1]*
      vnk[16][1]))))+((((hnk[10][2]*wk[15][2])+((hnk[10][0]*wk[15][0])+(
      hnk[10][1]*wk[15][1])))+((lk[10][2]*vnk[15][2])+((lk[10][0]*vnk[15][0])+(
      lk[10][1]*vnk[15][1]))))+temp[4]));
    temp[6] = ((((hnk[13][2]*wk[18][2])+((hnk[13][0]*wk[18][0])+(hnk[13][1]*
      wk[18][1])))+((lk[13][2]*vnk[18][2])+((lk[13][0]*vnk[18][0])+(lk[13][1]*
      vnk[18][1]))))+((((hnk[12][2]*wk[17][2])+((hnk[12][0]*wk[17][0])+(
      hnk[12][1]*wk[17][1])))+((lk[12][2]*vnk[17][2])+((lk[12][0]*vnk[17][0])+(
      lk[12][1]*vnk[17][1]))))+temp[5]));
    *ke = (.5*((((hnk[15][2]*wk[20][2])+((hnk[15][0]*wk[20][0])+(hnk[15][1]*
      wk[20][1])))+((lk[15][2]*vnk[20][2])+((lk[15][0]*vnk[20][0])+(lk[15][1]*
      vnk[20][1]))))+((((hnk[14][2]*wk[19][2])+((hnk[14][0]*wk[19][0])+(
      hnk[14][1]*wk[19][1])))+((lk[14][2]*vnk[19][2])+((lk[14][0]*vnk[19][0])+(
      lk[14][1]*vnk[19][1]))))+temp[6])));
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  479 adds/subtracts/negates
                    535 multiplies
                      0 divides
                    131 assignments
*/
}

void sdsys(double *mtoto,
    double cm[3],
    double icm[3][3])
{
/*
Compute system total mass, and instantaneous center of mass and
inertia matrix.

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
    double ikcnkt[21][3][3];

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(20,23);
        return;
    }
    *mtoto = mtot;
    cm[0] = com[0];
    cm[1] = com[1];
    cm[2] = com[2];
    ikcnkt[5][0][0] = ((Cik[3][0][2]*ik[0][0][2])+((Cik[3][0][0]*ik[0][0][0])+(
      Cik[3][0][1]*ik[0][0][1])));
    ikcnkt[5][0][1] = ((Cik[3][1][2]*ik[0][0][2])+((Cik[3][1][0]*ik[0][0][0])+(
      Cik[3][1][1]*ik[0][0][1])));
    ikcnkt[5][0][2] = ((Cik[3][2][2]*ik[0][0][2])+((Cik[3][2][0]*ik[0][0][0])+(
      Cik[3][2][1]*ik[0][0][1])));
    ikcnkt[5][1][0] = ((Cik[3][0][2]*ik[0][1][2])+((Cik[3][0][0]*ik[0][1][0])+(
      Cik[3][0][1]*ik[0][1][1])));
    ikcnkt[5][1][1] = ((Cik[3][1][2]*ik[0][1][2])+((Cik[3][1][0]*ik[0][1][0])+(
      Cik[3][1][1]*ik[0][1][1])));
    ikcnkt[5][1][2] = ((Cik[3][2][2]*ik[0][1][2])+((Cik[3][2][0]*ik[0][1][0])+(
      Cik[3][2][1]*ik[0][1][1])));
    ikcnkt[5][2][0] = ((Cik[3][0][2]*ik[0][2][2])+((Cik[3][0][0]*ik[0][2][0])+(
      Cik[3][0][1]*ik[0][2][1])));
    ikcnkt[5][2][1] = ((Cik[3][1][2]*ik[0][2][2])+((Cik[3][1][0]*ik[0][2][0])+(
      Cik[3][1][1]*ik[0][2][1])));
    ikcnkt[5][2][2] = ((Cik[3][2][2]*ik[0][2][2])+((Cik[3][2][0]*ik[0][2][0])+(
      Cik[3][2][1]*ik[0][2][1])));
    ikcnkt[6][0][0] = ((cnk[6][0][2]*ik[1][0][2])+((cnk[6][0][0]*ik[1][0][0])+(
      cnk[6][0][1]*ik[1][0][1])));
    ikcnkt[6][0][1] = ((cnk[6][1][2]*ik[1][0][2])+((cnk[6][1][0]*ik[1][0][0])+(
      cnk[6][1][1]*ik[1][0][1])));
    ikcnkt[6][0][2] = ((cnk[6][2][2]*ik[1][0][2])+((cnk[6][2][0]*ik[1][0][0])+(
      cnk[6][2][1]*ik[1][0][1])));
    ikcnkt[6][1][0] = ((cnk[6][0][2]*ik[1][1][2])+((cnk[6][0][0]*ik[1][1][0])+(
      cnk[6][0][1]*ik[1][1][1])));
    ikcnkt[6][1][1] = ((cnk[6][1][2]*ik[1][1][2])+((cnk[6][1][0]*ik[1][1][0])+(
      cnk[6][1][1]*ik[1][1][1])));
    ikcnkt[6][1][2] = ((cnk[6][2][2]*ik[1][1][2])+((cnk[6][2][0]*ik[1][1][0])+(
      cnk[6][2][1]*ik[1][1][1])));
    ikcnkt[6][2][0] = ((cnk[6][0][2]*ik[1][2][2])+((cnk[6][0][0]*ik[1][2][0])+(
      cnk[6][0][1]*ik[1][2][1])));
    ikcnkt[6][2][1] = ((cnk[6][1][2]*ik[1][2][2])+((cnk[6][1][0]*ik[1][2][0])+(
      cnk[6][1][1]*ik[1][2][1])));
    ikcnkt[6][2][2] = ((cnk[6][2][2]*ik[1][2][2])+((cnk[6][2][0]*ik[1][2][0])+(
      cnk[6][2][1]*ik[1][2][1])));
    ikcnkt[7][0][0] = ((cnk[7][0][2]*ik[2][0][2])+((cnk[7][0][0]*ik[2][0][0])+(
      cnk[7][0][1]*ik[2][0][1])));
    ikcnkt[7][0][1] = ((cnk[7][1][2]*ik[2][0][2])+((cnk[7][1][0]*ik[2][0][0])+(
      cnk[7][1][1]*ik[2][0][1])));
    ikcnkt[7][0][2] = ((cnk[7][2][2]*ik[2][0][2])+((cnk[7][2][0]*ik[2][0][0])+(
      cnk[7][2][1]*ik[2][0][1])));
    ikcnkt[7][1][0] = ((cnk[7][0][2]*ik[2][1][2])+((cnk[7][0][0]*ik[2][1][0])+(
      cnk[7][0][1]*ik[2][1][1])));
    ikcnkt[7][1][1] = ((cnk[7][1][2]*ik[2][1][2])+((cnk[7][1][0]*ik[2][1][0])+(
      cnk[7][1][1]*ik[2][1][1])));
    ikcnkt[7][1][2] = ((cnk[7][2][2]*ik[2][1][2])+((cnk[7][2][0]*ik[2][1][0])+(
      cnk[7][2][1]*ik[2][1][1])));
    ikcnkt[7][2][0] = ((cnk[7][0][2]*ik[2][2][2])+((cnk[7][0][0]*ik[2][2][0])+(
      cnk[7][0][1]*ik[2][2][1])));
    ikcnkt[7][2][1] = ((cnk[7][1][2]*ik[2][2][2])+((cnk[7][1][0]*ik[2][2][0])+(
      cnk[7][1][1]*ik[2][2][1])));
    ikcnkt[7][2][2] = ((cnk[7][2][2]*ik[2][2][2])+((cnk[7][2][0]*ik[2][2][0])+(
      cnk[7][2][1]*ik[2][2][1])));
    ikcnkt[8][0][0] = ((cnk[8][0][2]*ik[3][0][2])+((cnk[8][0][0]*ik[3][0][0])+(
      cnk[8][0][1]*ik[3][0][1])));
    ikcnkt[8][0][1] = ((cnk[8][1][2]*ik[3][0][2])+((cnk[8][1][0]*ik[3][0][0])+(
      cnk[8][1][1]*ik[3][0][1])));
    ikcnkt[8][0][2] = ((cnk[8][2][2]*ik[3][0][2])+((cnk[8][2][0]*ik[3][0][0])+(
      cnk[8][2][1]*ik[3][0][1])));
    ikcnkt[8][1][0] = ((cnk[8][0][2]*ik[3][1][2])+((cnk[8][0][0]*ik[3][1][0])+(
      cnk[8][0][1]*ik[3][1][1])));
    ikcnkt[8][1][1] = ((cnk[8][1][2]*ik[3][1][2])+((cnk[8][1][0]*ik[3][1][0])+(
      cnk[8][1][1]*ik[3][1][1])));
    ikcnkt[8][1][2] = ((cnk[8][2][2]*ik[3][1][2])+((cnk[8][2][0]*ik[3][1][0])+(
      cnk[8][2][1]*ik[3][1][1])));
    ikcnkt[8][2][0] = ((cnk[8][0][2]*ik[3][2][2])+((cnk[8][0][0]*ik[3][2][0])+(
      cnk[8][0][1]*ik[3][2][1])));
    ikcnkt[8][2][1] = ((cnk[8][1][2]*ik[3][2][2])+((cnk[8][1][0]*ik[3][2][0])+(
      cnk[8][1][1]*ik[3][2][1])));
    ikcnkt[8][2][2] = ((cnk[8][2][2]*ik[3][2][2])+((cnk[8][2][0]*ik[3][2][0])+(
      cnk[8][2][1]*ik[3][2][1])));
    ikcnkt[9][0][0] = ((cnk[9][0][2]*ik[4][0][2])+((cnk[9][0][0]*ik[4][0][0])+(
      cnk[9][0][1]*ik[4][0][1])));
    ikcnkt[9][0][1] = ((cnk[9][1][2]*ik[4][0][2])+((cnk[9][1][0]*ik[4][0][0])+(
      cnk[9][1][1]*ik[4][0][1])));
    ikcnkt[9][0][2] = ((cnk[9][2][2]*ik[4][0][2])+((cnk[9][2][0]*ik[4][0][0])+(
      cnk[9][2][1]*ik[4][0][1])));
    ikcnkt[9][1][0] = ((cnk[9][0][2]*ik[4][1][2])+((cnk[9][0][0]*ik[4][1][0])+(
      cnk[9][0][1]*ik[4][1][1])));
    ikcnkt[9][1][1] = ((cnk[9][1][2]*ik[4][1][2])+((cnk[9][1][0]*ik[4][1][0])+(
      cnk[9][1][1]*ik[4][1][1])));
    ikcnkt[9][1][2] = ((cnk[9][2][2]*ik[4][1][2])+((cnk[9][2][0]*ik[4][1][0])+(
      cnk[9][2][1]*ik[4][1][1])));
    ikcnkt[9][2][0] = ((cnk[9][0][2]*ik[4][2][2])+((cnk[9][0][0]*ik[4][2][0])+(
      cnk[9][0][1]*ik[4][2][1])));
    ikcnkt[9][2][1] = ((cnk[9][1][2]*ik[4][2][2])+((cnk[9][1][0]*ik[4][2][0])+(
      cnk[9][1][1]*ik[4][2][1])));
    ikcnkt[9][2][2] = ((cnk[9][2][2]*ik[4][2][2])+((cnk[9][2][0]*ik[4][2][0])+(
      cnk[9][2][1]*ik[4][2][1])));
    ikcnkt[10][0][0] = ((cnk[10][0][2]*ik[5][0][2])+((cnk[10][0][0]*ik[5][0][0])
      +(cnk[10][0][1]*ik[5][0][1])));
    ikcnkt[10][0][1] = ((cnk[10][1][2]*ik[5][0][2])+((cnk[10][1][0]*ik[5][0][0])
      +(cnk[10][1][1]*ik[5][0][1])));
    ikcnkt[10][0][2] = ((cnk[10][2][2]*ik[5][0][2])+((cnk[10][2][0]*ik[5][0][0])
      +(cnk[10][2][1]*ik[5][0][1])));
    ikcnkt[10][1][0] = ((cnk[10][0][2]*ik[5][1][2])+((cnk[10][0][0]*ik[5][1][0])
      +(cnk[10][0][1]*ik[5][1][1])));
    ikcnkt[10][1][1] = ((cnk[10][1][2]*ik[5][1][2])+((cnk[10][1][0]*ik[5][1][0])
      +(cnk[10][1][1]*ik[5][1][1])));
    ikcnkt[10][1][2] = ((cnk[10][2][2]*ik[5][1][2])+((cnk[10][2][0]*ik[5][1][0])
      +(cnk[10][2][1]*ik[5][1][1])));
    ikcnkt[10][2][0] = ((cnk[10][0][2]*ik[5][2][2])+((cnk[10][0][0]*ik[5][2][0])
      +(cnk[10][0][1]*ik[5][2][1])));
    ikcnkt[10][2][1] = ((cnk[10][1][2]*ik[5][2][2])+((cnk[10][1][0]*ik[5][2][0])
      +(cnk[10][1][1]*ik[5][2][1])));
    ikcnkt[10][2][2] = ((cnk[10][2][2]*ik[5][2][2])+((cnk[10][2][0]*ik[5][2][0])
      +(cnk[10][2][1]*ik[5][2][1])));
    ikcnkt[11][0][0] = ((cnk[11][0][2]*ik[6][0][2])+((cnk[11][0][0]*ik[6][0][0])
      +(cnk[11][0][1]*ik[6][0][1])));
    ikcnkt[11][0][1] = ((cnk[11][1][2]*ik[6][0][2])+((cnk[11][1][0]*ik[6][0][0])
      +(cnk[11][1][1]*ik[6][0][1])));
    ikcnkt[11][0][2] = ((cnk[11][2][2]*ik[6][0][2])+((cnk[11][2][0]*ik[6][0][0])
      +(cnk[11][2][1]*ik[6][0][1])));
    ikcnkt[11][1][0] = ((cnk[11][0][2]*ik[6][1][2])+((cnk[11][0][0]*ik[6][1][0])
      +(cnk[11][0][1]*ik[6][1][1])));
    ikcnkt[11][1][1] = ((cnk[11][1][2]*ik[6][1][2])+((cnk[11][1][0]*ik[6][1][0])
      +(cnk[11][1][1]*ik[6][1][1])));
    ikcnkt[11][1][2] = ((cnk[11][2][2]*ik[6][1][2])+((cnk[11][2][0]*ik[6][1][0])
      +(cnk[11][2][1]*ik[6][1][1])));
    ikcnkt[11][2][0] = ((cnk[11][0][2]*ik[6][2][2])+((cnk[11][0][0]*ik[6][2][0])
      +(cnk[11][0][1]*ik[6][2][1])));
    ikcnkt[11][2][1] = ((cnk[11][1][2]*ik[6][2][2])+((cnk[11][1][0]*ik[6][2][0])
      +(cnk[11][1][1]*ik[6][2][1])));
    ikcnkt[11][2][2] = ((cnk[11][2][2]*ik[6][2][2])+((cnk[11][2][0]*ik[6][2][0])
      +(cnk[11][2][1]*ik[6][2][1])));
    ikcnkt[12][0][0] = ((cnk[12][0][2]*ik[7][0][2])+((cnk[12][0][0]*ik[7][0][0])
      +(cnk[12][0][1]*ik[7][0][1])));
    ikcnkt[12][0][1] = ((cnk[12][1][2]*ik[7][0][2])+((cnk[12][1][0]*ik[7][0][0])
      +(cnk[12][1][1]*ik[7][0][1])));
    ikcnkt[12][0][2] = ((cnk[12][2][2]*ik[7][0][2])+((cnk[12][2][0]*ik[7][0][0])
      +(cnk[12][2][1]*ik[7][0][1])));
    ikcnkt[12][1][0] = ((cnk[12][0][2]*ik[7][1][2])+((cnk[12][0][0]*ik[7][1][0])
      +(cnk[12][0][1]*ik[7][1][1])));
    ikcnkt[12][1][1] = ((cnk[12][1][2]*ik[7][1][2])+((cnk[12][1][0]*ik[7][1][0])
      +(cnk[12][1][1]*ik[7][1][1])));
    ikcnkt[12][1][2] = ((cnk[12][2][2]*ik[7][1][2])+((cnk[12][2][0]*ik[7][1][0])
      +(cnk[12][2][1]*ik[7][1][1])));
    ikcnkt[12][2][0] = ((cnk[12][0][2]*ik[7][2][2])+((cnk[12][0][0]*ik[7][2][0])
      +(cnk[12][0][1]*ik[7][2][1])));
    ikcnkt[12][2][1] = ((cnk[12][1][2]*ik[7][2][2])+((cnk[12][1][0]*ik[7][2][0])
      +(cnk[12][1][1]*ik[7][2][1])));
    ikcnkt[12][2][2] = ((cnk[12][2][2]*ik[7][2][2])+((cnk[12][2][0]*ik[7][2][0])
      +(cnk[12][2][1]*ik[7][2][1])));
    ikcnkt[13][0][0] = ((cnk[13][0][2]*ik[8][0][2])+((cnk[13][0][0]*ik[8][0][0])
      +(cnk[13][0][1]*ik[8][0][1])));
    ikcnkt[13][0][1] = ((cnk[13][1][2]*ik[8][0][2])+((cnk[13][1][0]*ik[8][0][0])
      +(cnk[13][1][1]*ik[8][0][1])));
    ikcnkt[13][0][2] = ((cnk[13][2][2]*ik[8][0][2])+((cnk[13][2][0]*ik[8][0][0])
      +(cnk[13][2][1]*ik[8][0][1])));
    ikcnkt[13][1][0] = ((cnk[13][0][2]*ik[8][1][2])+((cnk[13][0][0]*ik[8][1][0])
      +(cnk[13][0][1]*ik[8][1][1])));
    ikcnkt[13][1][1] = ((cnk[13][1][2]*ik[8][1][2])+((cnk[13][1][0]*ik[8][1][0])
      +(cnk[13][1][1]*ik[8][1][1])));
    ikcnkt[13][1][2] = ((cnk[13][2][2]*ik[8][1][2])+((cnk[13][2][0]*ik[8][1][0])
      +(cnk[13][2][1]*ik[8][1][1])));
    ikcnkt[13][2][0] = ((cnk[13][0][2]*ik[8][2][2])+((cnk[13][0][0]*ik[8][2][0])
      +(cnk[13][0][1]*ik[8][2][1])));
    ikcnkt[13][2][1] = ((cnk[13][1][2]*ik[8][2][2])+((cnk[13][1][0]*ik[8][2][0])
      +(cnk[13][1][1]*ik[8][2][1])));
    ikcnkt[13][2][2] = ((cnk[13][2][2]*ik[8][2][2])+((cnk[13][2][0]*ik[8][2][0])
      +(cnk[13][2][1]*ik[8][2][1])));
    ikcnkt[14][0][0] = ((cnk[14][0][2]*ik[9][0][2])+((cnk[14][0][0]*ik[9][0][0])
      +(cnk[14][0][1]*ik[9][0][1])));
    ikcnkt[14][0][1] = ((cnk[14][1][2]*ik[9][0][2])+((cnk[14][1][0]*ik[9][0][0])
      +(cnk[14][1][1]*ik[9][0][1])));
    ikcnkt[14][0][2] = ((cnk[14][2][2]*ik[9][0][2])+((cnk[14][2][0]*ik[9][0][0])
      +(cnk[14][2][1]*ik[9][0][1])));
    ikcnkt[14][1][0] = ((cnk[14][0][2]*ik[9][1][2])+((cnk[14][0][0]*ik[9][1][0])
      +(cnk[14][0][1]*ik[9][1][1])));
    ikcnkt[14][1][1] = ((cnk[14][1][2]*ik[9][1][2])+((cnk[14][1][0]*ik[9][1][0])
      +(cnk[14][1][1]*ik[9][1][1])));
    ikcnkt[14][1][2] = ((cnk[14][2][2]*ik[9][1][2])+((cnk[14][2][0]*ik[9][1][0])
      +(cnk[14][2][1]*ik[9][1][1])));
    ikcnkt[14][2][0] = ((cnk[14][0][2]*ik[9][2][2])+((cnk[14][0][0]*ik[9][2][0])
      +(cnk[14][0][1]*ik[9][2][1])));
    ikcnkt[14][2][1] = ((cnk[14][1][2]*ik[9][2][2])+((cnk[14][1][0]*ik[9][2][0])
      +(cnk[14][1][1]*ik[9][2][1])));
    ikcnkt[14][2][2] = ((cnk[14][2][2]*ik[9][2][2])+((cnk[14][2][0]*ik[9][2][0])
      +(cnk[14][2][1]*ik[9][2][1])));
    ikcnkt[15][0][0] = ((cnk[15][0][2]*ik[10][0][2])+((cnk[15][0][0]*
      ik[10][0][0])+(cnk[15][0][1]*ik[10][0][1])));
    ikcnkt[15][0][1] = ((cnk[15][1][2]*ik[10][0][2])+((cnk[15][1][0]*
      ik[10][0][0])+(cnk[15][1][1]*ik[10][0][1])));
    ikcnkt[15][0][2] = ((cnk[15][2][2]*ik[10][0][2])+((cnk[15][2][0]*
      ik[10][0][0])+(cnk[15][2][1]*ik[10][0][1])));
    ikcnkt[15][1][0] = ((cnk[15][0][2]*ik[10][1][2])+((cnk[15][0][0]*
      ik[10][1][0])+(cnk[15][0][1]*ik[10][1][1])));
    ikcnkt[15][1][1] = ((cnk[15][1][2]*ik[10][1][2])+((cnk[15][1][0]*
      ik[10][1][0])+(cnk[15][1][1]*ik[10][1][1])));
    ikcnkt[15][1][2] = ((cnk[15][2][2]*ik[10][1][2])+((cnk[15][2][0]*
      ik[10][1][0])+(cnk[15][2][1]*ik[10][1][1])));
    ikcnkt[15][2][0] = ((cnk[15][0][2]*ik[10][2][2])+((cnk[15][0][0]*
      ik[10][2][0])+(cnk[15][0][1]*ik[10][2][1])));
    ikcnkt[15][2][1] = ((cnk[15][1][2]*ik[10][2][2])+((cnk[15][1][0]*
      ik[10][2][0])+(cnk[15][1][1]*ik[10][2][1])));
    ikcnkt[15][2][2] = ((cnk[15][2][2]*ik[10][2][2])+((cnk[15][2][0]*
      ik[10][2][0])+(cnk[15][2][1]*ik[10][2][1])));
    ikcnkt[16][0][0] = ((cnk[16][0][2]*ik[11][0][2])+((cnk[16][0][0]*
      ik[11][0][0])+(cnk[16][0][1]*ik[11][0][1])));
    ikcnkt[16][0][1] = ((cnk[16][1][2]*ik[11][0][2])+((cnk[16][1][0]*
      ik[11][0][0])+(cnk[16][1][1]*ik[11][0][1])));
    ikcnkt[16][0][2] = ((cnk[16][2][2]*ik[11][0][2])+((cnk[16][2][0]*
      ik[11][0][0])+(cnk[16][2][1]*ik[11][0][1])));
    ikcnkt[16][1][0] = ((cnk[16][0][2]*ik[11][1][2])+((cnk[16][0][0]*
      ik[11][1][0])+(cnk[16][0][1]*ik[11][1][1])));
    ikcnkt[16][1][1] = ((cnk[16][1][2]*ik[11][1][2])+((cnk[16][1][0]*
      ik[11][1][0])+(cnk[16][1][1]*ik[11][1][1])));
    ikcnkt[16][1][2] = ((cnk[16][2][2]*ik[11][1][2])+((cnk[16][2][0]*
      ik[11][1][0])+(cnk[16][2][1]*ik[11][1][1])));
    ikcnkt[16][2][0] = ((cnk[16][0][2]*ik[11][2][2])+((cnk[16][0][0]*
      ik[11][2][0])+(cnk[16][0][1]*ik[11][2][1])));
    ikcnkt[16][2][1] = ((cnk[16][1][2]*ik[11][2][2])+((cnk[16][1][0]*
      ik[11][2][0])+(cnk[16][1][1]*ik[11][2][1])));
    ikcnkt[16][2][2] = ((cnk[16][2][2]*ik[11][2][2])+((cnk[16][2][0]*
      ik[11][2][0])+(cnk[16][2][1]*ik[11][2][1])));
    ikcnkt[17][0][0] = ((cnk[17][0][2]*ik[12][0][2])+((cnk[17][0][0]*
      ik[12][0][0])+(cnk[17][0][1]*ik[12][0][1])));
    ikcnkt[17][0][1] = ((cnk[17][1][2]*ik[12][0][2])+((cnk[17][1][0]*
      ik[12][0][0])+(cnk[17][1][1]*ik[12][0][1])));
    ikcnkt[17][0][2] = ((cnk[17][2][2]*ik[12][0][2])+((cnk[17][2][0]*
      ik[12][0][0])+(cnk[17][2][1]*ik[12][0][1])));
    ikcnkt[17][1][0] = ((cnk[17][0][2]*ik[12][1][2])+((cnk[17][0][0]*
      ik[12][1][0])+(cnk[17][0][1]*ik[12][1][1])));
    ikcnkt[17][1][1] = ((cnk[17][1][2]*ik[12][1][2])+((cnk[17][1][0]*
      ik[12][1][0])+(cnk[17][1][1]*ik[12][1][1])));
    ikcnkt[17][1][2] = ((cnk[17][2][2]*ik[12][1][2])+((cnk[17][2][0]*
      ik[12][1][0])+(cnk[17][2][1]*ik[12][1][1])));
    ikcnkt[17][2][0] = ((cnk[17][0][2]*ik[12][2][2])+((cnk[17][0][0]*
      ik[12][2][0])+(cnk[17][0][1]*ik[12][2][1])));
    ikcnkt[17][2][1] = ((cnk[17][1][2]*ik[12][2][2])+((cnk[17][1][0]*
      ik[12][2][0])+(cnk[17][1][1]*ik[12][2][1])));
    ikcnkt[17][2][2] = ((cnk[17][2][2]*ik[12][2][2])+((cnk[17][2][0]*
      ik[12][2][0])+(cnk[17][2][1]*ik[12][2][1])));
    ikcnkt[18][0][0] = ((cnk[18][0][2]*ik[13][0][2])+((cnk[18][0][0]*
      ik[13][0][0])+(cnk[18][0][1]*ik[13][0][1])));
    ikcnkt[18][0][1] = ((cnk[18][1][2]*ik[13][0][2])+((cnk[18][1][0]*
      ik[13][0][0])+(cnk[18][1][1]*ik[13][0][1])));
    ikcnkt[18][0][2] = ((cnk[18][2][2]*ik[13][0][2])+((cnk[18][2][0]*
      ik[13][0][0])+(cnk[18][2][1]*ik[13][0][1])));
    ikcnkt[18][1][0] = ((cnk[18][0][2]*ik[13][1][2])+((cnk[18][0][0]*
      ik[13][1][0])+(cnk[18][0][1]*ik[13][1][1])));
    ikcnkt[18][1][1] = ((cnk[18][1][2]*ik[13][1][2])+((cnk[18][1][0]*
      ik[13][1][0])+(cnk[18][1][1]*ik[13][1][1])));
    ikcnkt[18][1][2] = ((cnk[18][2][2]*ik[13][1][2])+((cnk[18][2][0]*
      ik[13][1][0])+(cnk[18][2][1]*ik[13][1][1])));
    ikcnkt[18][2][0] = ((cnk[18][0][2]*ik[13][2][2])+((cnk[18][0][0]*
      ik[13][2][0])+(cnk[18][0][1]*ik[13][2][1])));
    ikcnkt[18][2][1] = ((cnk[18][1][2]*ik[13][2][2])+((cnk[18][1][0]*
      ik[13][2][0])+(cnk[18][1][1]*ik[13][2][1])));
    ikcnkt[18][2][2] = ((cnk[18][2][2]*ik[13][2][2])+((cnk[18][2][0]*
      ik[13][2][0])+(cnk[18][2][1]*ik[13][2][1])));
    ikcnkt[19][0][0] = ((cnk[19][0][2]*ik[14][0][2])+((cnk[19][0][0]*
      ik[14][0][0])+(cnk[19][0][1]*ik[14][0][1])));
    ikcnkt[19][0][1] = ((cnk[19][1][2]*ik[14][0][2])+((cnk[19][1][0]*
      ik[14][0][0])+(cnk[19][1][1]*ik[14][0][1])));
    ikcnkt[19][0][2] = ((cnk[19][2][2]*ik[14][0][2])+((cnk[19][2][0]*
      ik[14][0][0])+(cnk[19][2][1]*ik[14][0][1])));
    ikcnkt[19][1][0] = ((cnk[19][0][2]*ik[14][1][2])+((cnk[19][0][0]*
      ik[14][1][0])+(cnk[19][0][1]*ik[14][1][1])));
    ikcnkt[19][1][1] = ((cnk[19][1][2]*ik[14][1][2])+((cnk[19][1][0]*
      ik[14][1][0])+(cnk[19][1][1]*ik[14][1][1])));
    ikcnkt[19][1][2] = ((cnk[19][2][2]*ik[14][1][2])+((cnk[19][2][0]*
      ik[14][1][0])+(cnk[19][2][1]*ik[14][1][1])));
    ikcnkt[19][2][0] = ((cnk[19][0][2]*ik[14][2][2])+((cnk[19][0][0]*
      ik[14][2][0])+(cnk[19][0][1]*ik[14][2][1])));
    ikcnkt[19][2][1] = ((cnk[19][1][2]*ik[14][2][2])+((cnk[19][1][0]*
      ik[14][2][0])+(cnk[19][1][1]*ik[14][2][1])));
    ikcnkt[19][2][2] = ((cnk[19][2][2]*ik[14][2][2])+((cnk[19][2][0]*
      ik[14][2][0])+(cnk[19][2][1]*ik[14][2][1])));
    ikcnkt[20][0][0] = ((cnk[20][0][2]*ik[15][0][2])+((cnk[20][0][0]*
      ik[15][0][0])+(cnk[20][0][1]*ik[15][0][1])));
    ikcnkt[20][0][1] = ((cnk[20][1][2]*ik[15][0][2])+((cnk[20][1][0]*
      ik[15][0][0])+(cnk[20][1][1]*ik[15][0][1])));
    ikcnkt[20][0][2] = ((cnk[20][2][2]*ik[15][0][2])+((cnk[20][2][0]*
      ik[15][0][0])+(cnk[20][2][1]*ik[15][0][1])));
    ikcnkt[20][1][0] = ((cnk[20][0][2]*ik[15][1][2])+((cnk[20][0][0]*
      ik[15][1][0])+(cnk[20][0][1]*ik[15][1][1])));
    ikcnkt[20][1][1] = ((cnk[20][1][2]*ik[15][1][2])+((cnk[20][1][0]*
      ik[15][1][0])+(cnk[20][1][1]*ik[15][1][1])));
    ikcnkt[20][1][2] = ((cnk[20][2][2]*ik[15][1][2])+((cnk[20][2][0]*
      ik[15][1][0])+(cnk[20][2][1]*ik[15][1][1])));
    ikcnkt[20][2][0] = ((cnk[20][0][2]*ik[15][2][2])+((cnk[20][0][0]*
      ik[15][2][0])+(cnk[20][0][1]*ik[15][2][1])));
    ikcnkt[20][2][1] = ((cnk[20][1][2]*ik[15][2][2])+((cnk[20][1][0]*
      ik[15][2][0])+(cnk[20][1][1]*ik[15][2][1])));
    ikcnkt[20][2][2] = ((cnk[20][2][2]*ik[15][2][2])+((cnk[20][2][0]*
      ik[15][2][0])+(cnk[20][2][1]*ik[15][2][1])));
    temp[0] = (((mk[0]*((rnk[5][1]*rnk[5][1])+(rnk[5][2]*rnk[5][2])))+((
      Cik[3][0][2]*ikcnkt[5][2][0])+((Cik[3][0][0]*ikcnkt[5][0][0])+(
      Cik[3][0][1]*ikcnkt[5][1][0]))))+((mk[1]*((rnk[6][1]*rnk[6][1])+(rnk[6][2]
      *rnk[6][2])))+((cnk[6][0][2]*ikcnkt[6][2][0])+((cnk[6][0][0]*
      ikcnkt[6][0][0])+(cnk[6][0][1]*ikcnkt[6][1][0])))));
    temp[1] = (((mk[3]*((rnk[8][1]*rnk[8][1])+(rnk[8][2]*rnk[8][2])))+((
      cnk[8][0][2]*ikcnkt[8][2][0])+((cnk[8][0][0]*ikcnkt[8][0][0])+(
      cnk[8][0][1]*ikcnkt[8][1][0]))))+(((mk[2]*((rnk[7][1]*rnk[7][1])+(
      rnk[7][2]*rnk[7][2])))+((cnk[7][0][2]*ikcnkt[7][2][0])+((cnk[7][0][0]*
      ikcnkt[7][0][0])+(cnk[7][0][1]*ikcnkt[7][1][0]))))+temp[0]));
    temp[2] = (((mk[5]*((rnk[10][1]*rnk[10][1])+(rnk[10][2]*rnk[10][2])))+((
      cnk[10][0][2]*ikcnkt[10][2][0])+((cnk[10][0][0]*ikcnkt[10][0][0])+(
      cnk[10][0][1]*ikcnkt[10][1][0]))))+(((mk[4]*((rnk[9][1]*rnk[9][1])+(
      rnk[9][2]*rnk[9][2])))+((cnk[9][0][2]*ikcnkt[9][2][0])+((cnk[9][0][0]*
      ikcnkt[9][0][0])+(cnk[9][0][1]*ikcnkt[9][1][0]))))+temp[1]));
    temp[3] = (((mk[7]*((rnk[12][1]*rnk[12][1])+(rnk[12][2]*rnk[12][2])))+((
      cnk[12][0][2]*ikcnkt[12][2][0])+((cnk[12][0][0]*ikcnkt[12][0][0])+(
      cnk[12][0][1]*ikcnkt[12][1][0]))))+(((mk[6]*((rnk[11][1]*rnk[11][1])+(
      rnk[11][2]*rnk[11][2])))+((cnk[11][0][2]*ikcnkt[11][2][0])+((cnk[11][0][0]
      *ikcnkt[11][0][0])+(cnk[11][0][1]*ikcnkt[11][1][0]))))+temp[2]));
    temp[4] = (((mk[9]*((rnk[14][1]*rnk[14][1])+(rnk[14][2]*rnk[14][2])))+((
      cnk[14][0][2]*ikcnkt[14][2][0])+((cnk[14][0][0]*ikcnkt[14][0][0])+(
      cnk[14][0][1]*ikcnkt[14][1][0]))))+(((mk[8]*((rnk[13][1]*rnk[13][1])+(
      rnk[13][2]*rnk[13][2])))+((cnk[13][0][2]*ikcnkt[13][2][0])+((cnk[13][0][0]
      *ikcnkt[13][0][0])+(cnk[13][0][1]*ikcnkt[13][1][0]))))+temp[3]));
    temp[5] = (((mk[11]*((rnk[16][1]*rnk[16][1])+(rnk[16][2]*rnk[16][2])))+((
      cnk[16][0][2]*ikcnkt[16][2][0])+((cnk[16][0][0]*ikcnkt[16][0][0])+(
      cnk[16][0][1]*ikcnkt[16][1][0]))))+(((mk[10]*((rnk[15][1]*rnk[15][1])+(
      rnk[15][2]*rnk[15][2])))+((cnk[15][0][2]*ikcnkt[15][2][0])+((cnk[15][0][0]
      *ikcnkt[15][0][0])+(cnk[15][0][1]*ikcnkt[15][1][0]))))+temp[4]));
    temp[6] = (((mk[13]*((rnk[18][1]*rnk[18][1])+(rnk[18][2]*rnk[18][2])))+((
      cnk[18][0][2]*ikcnkt[18][2][0])+((cnk[18][0][0]*ikcnkt[18][0][0])+(
      cnk[18][0][1]*ikcnkt[18][1][0]))))+(((mk[12]*((rnk[17][1]*rnk[17][1])+(
      rnk[17][2]*rnk[17][2])))+((cnk[17][0][2]*ikcnkt[17][2][0])+((cnk[17][0][0]
      *ikcnkt[17][0][0])+(cnk[17][0][1]*ikcnkt[17][1][0]))))+temp[5]));
    icm[0][0] = ((((mk[15]*((rnk[20][1]*rnk[20][1])+(rnk[20][2]*rnk[20][2])))+((
      cnk[20][0][2]*ikcnkt[20][2][0])+((cnk[20][0][0]*ikcnkt[20][0][0])+(
      cnk[20][0][1]*ikcnkt[20][1][0]))))+(((mk[14]*((rnk[19][1]*rnk[19][1])+(
      rnk[19][2]*rnk[19][2])))+((cnk[19][0][2]*ikcnkt[19][2][0])+((cnk[19][0][0]
      *ikcnkt[19][0][0])+(cnk[19][0][1]*ikcnkt[19][1][0]))))+temp[6]))-(mtot*((
      com[1]*com[1])+(com[2]*com[2]))));
    temp[0] = ((((cnk[7][0][2]*ikcnkt[7][2][1])+((cnk[7][0][0]*ikcnkt[7][0][1])+
      (cnk[7][0][1]*ikcnkt[7][1][1])))-(mk[2]*(rnk[7][0]*rnk[7][1])))+((((
      Cik[3][0][2]*ikcnkt[5][2][1])+((Cik[3][0][0]*ikcnkt[5][0][1])+(
      Cik[3][0][1]*ikcnkt[5][1][1])))-(mk[0]*(rnk[5][0]*rnk[5][1])))+(((
      cnk[6][0][2]*ikcnkt[6][2][1])+((cnk[6][0][0]*ikcnkt[6][0][1])+(
      cnk[6][0][1]*ikcnkt[6][1][1])))-(mk[1]*(rnk[6][0]*rnk[6][1])))));
    temp[1] = ((((cnk[10][0][2]*ikcnkt[10][2][1])+((cnk[10][0][0]*
      ikcnkt[10][0][1])+(cnk[10][0][1]*ikcnkt[10][1][1])))-(mk[5]*(rnk[10][0]*
      rnk[10][1])))+((((cnk[9][0][2]*ikcnkt[9][2][1])+((cnk[9][0][0]*
      ikcnkt[9][0][1])+(cnk[9][0][1]*ikcnkt[9][1][1])))-(mk[4]*(rnk[9][0]*
      rnk[9][1])))+((((cnk[8][0][2]*ikcnkt[8][2][1])+((cnk[8][0][0]*
      ikcnkt[8][0][1])+(cnk[8][0][1]*ikcnkt[8][1][1])))-(mk[3]*(rnk[8][0]*
      rnk[8][1])))+temp[0])));
    temp[2] = ((((cnk[13][0][2]*ikcnkt[13][2][1])+((cnk[13][0][0]*
      ikcnkt[13][0][1])+(cnk[13][0][1]*ikcnkt[13][1][1])))-(mk[8]*(rnk[13][0]*
      rnk[13][1])))+((((cnk[12][0][2]*ikcnkt[12][2][1])+((cnk[12][0][0]*
      ikcnkt[12][0][1])+(cnk[12][0][1]*ikcnkt[12][1][1])))-(mk[7]*(rnk[12][0]*
      rnk[12][1])))+((((cnk[11][0][2]*ikcnkt[11][2][1])+((cnk[11][0][0]*
      ikcnkt[11][0][1])+(cnk[11][0][1]*ikcnkt[11][1][1])))-(mk[6]*(rnk[11][0]*
      rnk[11][1])))+temp[1])));
    temp[3] = ((((cnk[16][0][2]*ikcnkt[16][2][1])+((cnk[16][0][0]*
      ikcnkt[16][0][1])+(cnk[16][0][1]*ikcnkt[16][1][1])))-(mk[11]*(rnk[16][0]*
      rnk[16][1])))+((((cnk[15][0][2]*ikcnkt[15][2][1])+((cnk[15][0][0]*
      ikcnkt[15][0][1])+(cnk[15][0][1]*ikcnkt[15][1][1])))-(mk[10]*(rnk[15][0]*
      rnk[15][1])))+((((cnk[14][0][2]*ikcnkt[14][2][1])+((cnk[14][0][0]*
      ikcnkt[14][0][1])+(cnk[14][0][1]*ikcnkt[14][1][1])))-(mk[9]*(rnk[14][0]*
      rnk[14][1])))+temp[2])));
    temp[4] = ((((cnk[19][0][2]*ikcnkt[19][2][1])+((cnk[19][0][0]*
      ikcnkt[19][0][1])+(cnk[19][0][1]*ikcnkt[19][1][1])))-(mk[14]*(rnk[19][0]*
      rnk[19][1])))+((((cnk[18][0][2]*ikcnkt[18][2][1])+((cnk[18][0][0]*
      ikcnkt[18][0][1])+(cnk[18][0][1]*ikcnkt[18][1][1])))-(mk[13]*(rnk[18][0]*
      rnk[18][1])))+((((cnk[17][0][2]*ikcnkt[17][2][1])+((cnk[17][0][0]*
      ikcnkt[17][0][1])+(cnk[17][0][1]*ikcnkt[17][1][1])))-(mk[12]*(rnk[17][0]*
      rnk[17][1])))+temp[3])));
    icm[0][1] = ((mtot*(com[0]*com[1]))+((((cnk[20][0][2]*ikcnkt[20][2][1])+((
      cnk[20][0][0]*ikcnkt[20][0][1])+(cnk[20][0][1]*ikcnkt[20][1][1])))-(mk[15]
      *(rnk[20][0]*rnk[20][1])))+temp[4]));
    temp[0] = ((((cnk[7][0][2]*ikcnkt[7][2][2])+((cnk[7][0][0]*ikcnkt[7][0][2])+
      (cnk[7][0][1]*ikcnkt[7][1][2])))-(mk[2]*(rnk[7][0]*rnk[7][2])))+((((
      Cik[3][0][2]*ikcnkt[5][2][2])+((Cik[3][0][0]*ikcnkt[5][0][2])+(
      Cik[3][0][1]*ikcnkt[5][1][2])))-(mk[0]*(rnk[5][0]*rnk[5][2])))+(((
      cnk[6][0][2]*ikcnkt[6][2][2])+((cnk[6][0][0]*ikcnkt[6][0][2])+(
      cnk[6][0][1]*ikcnkt[6][1][2])))-(mk[1]*(rnk[6][0]*rnk[6][2])))));
    temp[1] = ((((cnk[10][0][2]*ikcnkt[10][2][2])+((cnk[10][0][0]*
      ikcnkt[10][0][2])+(cnk[10][0][1]*ikcnkt[10][1][2])))-(mk[5]*(rnk[10][0]*
      rnk[10][2])))+((((cnk[9][0][2]*ikcnkt[9][2][2])+((cnk[9][0][0]*
      ikcnkt[9][0][2])+(cnk[9][0][1]*ikcnkt[9][1][2])))-(mk[4]*(rnk[9][0]*
      rnk[9][2])))+((((cnk[8][0][2]*ikcnkt[8][2][2])+((cnk[8][0][0]*
      ikcnkt[8][0][2])+(cnk[8][0][1]*ikcnkt[8][1][2])))-(mk[3]*(rnk[8][0]*
      rnk[8][2])))+temp[0])));
    temp[2] = ((((cnk[13][0][2]*ikcnkt[13][2][2])+((cnk[13][0][0]*
      ikcnkt[13][0][2])+(cnk[13][0][1]*ikcnkt[13][1][2])))-(mk[8]*(rnk[13][0]*
      rnk[13][2])))+((((cnk[12][0][2]*ikcnkt[12][2][2])+((cnk[12][0][0]*
      ikcnkt[12][0][2])+(cnk[12][0][1]*ikcnkt[12][1][2])))-(mk[7]*(rnk[12][0]*
      rnk[12][2])))+((((cnk[11][0][2]*ikcnkt[11][2][2])+((cnk[11][0][0]*
      ikcnkt[11][0][2])+(cnk[11][0][1]*ikcnkt[11][1][2])))-(mk[6]*(rnk[11][0]*
      rnk[11][2])))+temp[1])));
    temp[3] = ((((cnk[16][0][2]*ikcnkt[16][2][2])+((cnk[16][0][0]*
      ikcnkt[16][0][2])+(cnk[16][0][1]*ikcnkt[16][1][2])))-(mk[11]*(rnk[16][0]*
      rnk[16][2])))+((((cnk[15][0][2]*ikcnkt[15][2][2])+((cnk[15][0][0]*
      ikcnkt[15][0][2])+(cnk[15][0][1]*ikcnkt[15][1][2])))-(mk[10]*(rnk[15][0]*
      rnk[15][2])))+((((cnk[14][0][2]*ikcnkt[14][2][2])+((cnk[14][0][0]*
      ikcnkt[14][0][2])+(cnk[14][0][1]*ikcnkt[14][1][2])))-(mk[9]*(rnk[14][0]*
      rnk[14][2])))+temp[2])));
    temp[4] = ((((cnk[19][0][2]*ikcnkt[19][2][2])+((cnk[19][0][0]*
      ikcnkt[19][0][2])+(cnk[19][0][1]*ikcnkt[19][1][2])))-(mk[14]*(rnk[19][0]*
      rnk[19][2])))+((((cnk[18][0][2]*ikcnkt[18][2][2])+((cnk[18][0][0]*
      ikcnkt[18][0][2])+(cnk[18][0][1]*ikcnkt[18][1][2])))-(mk[13]*(rnk[18][0]*
      rnk[18][2])))+((((cnk[17][0][2]*ikcnkt[17][2][2])+((cnk[17][0][0]*
      ikcnkt[17][0][2])+(cnk[17][0][1]*ikcnkt[17][1][2])))-(mk[12]*(rnk[17][0]*
      rnk[17][2])))+temp[3])));
    icm[0][2] = ((mtot*(com[0]*com[2]))+((((cnk[20][0][2]*ikcnkt[20][2][2])+((
      cnk[20][0][0]*ikcnkt[20][0][2])+(cnk[20][0][1]*ikcnkt[20][1][2])))-(mk[15]
      *(rnk[20][0]*rnk[20][2])))+temp[4]));
    icm[1][0] = icm[0][1];
    temp[0] = (((mk[0]*((rnk[5][0]*rnk[5][0])+(rnk[5][2]*rnk[5][2])))+((
      Cik[3][1][2]*ikcnkt[5][2][1])+((Cik[3][1][0]*ikcnkt[5][0][1])+(
      Cik[3][1][1]*ikcnkt[5][1][1]))))+((mk[1]*((rnk[6][0]*rnk[6][0])+(rnk[6][2]
      *rnk[6][2])))+((cnk[6][1][2]*ikcnkt[6][2][1])+((cnk[6][1][0]*
      ikcnkt[6][0][1])+(cnk[6][1][1]*ikcnkt[6][1][1])))));
    temp[1] = (((mk[3]*((rnk[8][0]*rnk[8][0])+(rnk[8][2]*rnk[8][2])))+((
      cnk[8][1][2]*ikcnkt[8][2][1])+((cnk[8][1][0]*ikcnkt[8][0][1])+(
      cnk[8][1][1]*ikcnkt[8][1][1]))))+(((mk[2]*((rnk[7][0]*rnk[7][0])+(
      rnk[7][2]*rnk[7][2])))+((cnk[7][1][2]*ikcnkt[7][2][1])+((cnk[7][1][0]*
      ikcnkt[7][0][1])+(cnk[7][1][1]*ikcnkt[7][1][1]))))+temp[0]));
    temp[2] = (((mk[5]*((rnk[10][0]*rnk[10][0])+(rnk[10][2]*rnk[10][2])))+((
      cnk[10][1][2]*ikcnkt[10][2][1])+((cnk[10][1][0]*ikcnkt[10][0][1])+(
      cnk[10][1][1]*ikcnkt[10][1][1]))))+(((mk[4]*((rnk[9][0]*rnk[9][0])+(
      rnk[9][2]*rnk[9][2])))+((cnk[9][1][2]*ikcnkt[9][2][1])+((cnk[9][1][0]*
      ikcnkt[9][0][1])+(cnk[9][1][1]*ikcnkt[9][1][1]))))+temp[1]));
    temp[3] = (((mk[7]*((rnk[12][0]*rnk[12][0])+(rnk[12][2]*rnk[12][2])))+((
      cnk[12][1][2]*ikcnkt[12][2][1])+((cnk[12][1][0]*ikcnkt[12][0][1])+(
      cnk[12][1][1]*ikcnkt[12][1][1]))))+(((mk[6]*((rnk[11][0]*rnk[11][0])+(
      rnk[11][2]*rnk[11][2])))+((cnk[11][1][2]*ikcnkt[11][2][1])+((cnk[11][1][0]
      *ikcnkt[11][0][1])+(cnk[11][1][1]*ikcnkt[11][1][1]))))+temp[2]));
    temp[4] = (((mk[9]*((rnk[14][0]*rnk[14][0])+(rnk[14][2]*rnk[14][2])))+((
      cnk[14][1][2]*ikcnkt[14][2][1])+((cnk[14][1][0]*ikcnkt[14][0][1])+(
      cnk[14][1][1]*ikcnkt[14][1][1]))))+(((mk[8]*((rnk[13][0]*rnk[13][0])+(
      rnk[13][2]*rnk[13][2])))+((cnk[13][1][2]*ikcnkt[13][2][1])+((cnk[13][1][0]
      *ikcnkt[13][0][1])+(cnk[13][1][1]*ikcnkt[13][1][1]))))+temp[3]));
    temp[5] = (((mk[11]*((rnk[16][0]*rnk[16][0])+(rnk[16][2]*rnk[16][2])))+((
      cnk[16][1][2]*ikcnkt[16][2][1])+((cnk[16][1][0]*ikcnkt[16][0][1])+(
      cnk[16][1][1]*ikcnkt[16][1][1]))))+(((mk[10]*((rnk[15][0]*rnk[15][0])+(
      rnk[15][2]*rnk[15][2])))+((cnk[15][1][2]*ikcnkt[15][2][1])+((cnk[15][1][0]
      *ikcnkt[15][0][1])+(cnk[15][1][1]*ikcnkt[15][1][1]))))+temp[4]));
    temp[6] = (((mk[13]*((rnk[18][0]*rnk[18][0])+(rnk[18][2]*rnk[18][2])))+((
      cnk[18][1][2]*ikcnkt[18][2][1])+((cnk[18][1][0]*ikcnkt[18][0][1])+(
      cnk[18][1][1]*ikcnkt[18][1][1]))))+(((mk[12]*((rnk[17][0]*rnk[17][0])+(
      rnk[17][2]*rnk[17][2])))+((cnk[17][1][2]*ikcnkt[17][2][1])+((cnk[17][1][0]
      *ikcnkt[17][0][1])+(cnk[17][1][1]*ikcnkt[17][1][1]))))+temp[5]));
    icm[1][1] = ((((mk[15]*((rnk[20][0]*rnk[20][0])+(rnk[20][2]*rnk[20][2])))+((
      cnk[20][1][2]*ikcnkt[20][2][1])+((cnk[20][1][0]*ikcnkt[20][0][1])+(
      cnk[20][1][1]*ikcnkt[20][1][1]))))+(((mk[14]*((rnk[19][0]*rnk[19][0])+(
      rnk[19][2]*rnk[19][2])))+((cnk[19][1][2]*ikcnkt[19][2][1])+((cnk[19][1][0]
      *ikcnkt[19][0][1])+(cnk[19][1][1]*ikcnkt[19][1][1]))))+temp[6]))-(mtot*((
      com[0]*com[0])+(com[2]*com[2]))));
    temp[0] = ((((cnk[7][1][2]*ikcnkt[7][2][2])+((cnk[7][1][0]*ikcnkt[7][0][2])+
      (cnk[7][1][1]*ikcnkt[7][1][2])))-(mk[2]*(rnk[7][1]*rnk[7][2])))+((((
      Cik[3][1][2]*ikcnkt[5][2][2])+((Cik[3][1][0]*ikcnkt[5][0][2])+(
      Cik[3][1][1]*ikcnkt[5][1][2])))-(mk[0]*(rnk[5][1]*rnk[5][2])))+(((
      cnk[6][1][2]*ikcnkt[6][2][2])+((cnk[6][1][0]*ikcnkt[6][0][2])+(
      cnk[6][1][1]*ikcnkt[6][1][2])))-(mk[1]*(rnk[6][1]*rnk[6][2])))));
    temp[1] = ((((cnk[10][1][2]*ikcnkt[10][2][2])+((cnk[10][1][0]*
      ikcnkt[10][0][2])+(cnk[10][1][1]*ikcnkt[10][1][2])))-(mk[5]*(rnk[10][1]*
      rnk[10][2])))+((((cnk[9][1][2]*ikcnkt[9][2][2])+((cnk[9][1][0]*
      ikcnkt[9][0][2])+(cnk[9][1][1]*ikcnkt[9][1][2])))-(mk[4]*(rnk[9][1]*
      rnk[9][2])))+((((cnk[8][1][2]*ikcnkt[8][2][2])+((cnk[8][1][0]*
      ikcnkt[8][0][2])+(cnk[8][1][1]*ikcnkt[8][1][2])))-(mk[3]*(rnk[8][1]*
      rnk[8][2])))+temp[0])));
    temp[2] = ((((cnk[13][1][2]*ikcnkt[13][2][2])+((cnk[13][1][0]*
      ikcnkt[13][0][2])+(cnk[13][1][1]*ikcnkt[13][1][2])))-(mk[8]*(rnk[13][1]*
      rnk[13][2])))+((((cnk[12][1][2]*ikcnkt[12][2][2])+((cnk[12][1][0]*
      ikcnkt[12][0][2])+(cnk[12][1][1]*ikcnkt[12][1][2])))-(mk[7]*(rnk[12][1]*
      rnk[12][2])))+((((cnk[11][1][2]*ikcnkt[11][2][2])+((cnk[11][1][0]*
      ikcnkt[11][0][2])+(cnk[11][1][1]*ikcnkt[11][1][2])))-(mk[6]*(rnk[11][1]*
      rnk[11][2])))+temp[1])));
    temp[3] = ((((cnk[16][1][2]*ikcnkt[16][2][2])+((cnk[16][1][0]*
      ikcnkt[16][0][2])+(cnk[16][1][1]*ikcnkt[16][1][2])))-(mk[11]*(rnk[16][1]*
      rnk[16][2])))+((((cnk[15][1][2]*ikcnkt[15][2][2])+((cnk[15][1][0]*
      ikcnkt[15][0][2])+(cnk[15][1][1]*ikcnkt[15][1][2])))-(mk[10]*(rnk[15][1]*
      rnk[15][2])))+((((cnk[14][1][2]*ikcnkt[14][2][2])+((cnk[14][1][0]*
      ikcnkt[14][0][2])+(cnk[14][1][1]*ikcnkt[14][1][2])))-(mk[9]*(rnk[14][1]*
      rnk[14][2])))+temp[2])));
    temp[4] = ((((cnk[19][1][2]*ikcnkt[19][2][2])+((cnk[19][1][0]*
      ikcnkt[19][0][2])+(cnk[19][1][1]*ikcnkt[19][1][2])))-(mk[14]*(rnk[19][1]*
      rnk[19][2])))+((((cnk[18][1][2]*ikcnkt[18][2][2])+((cnk[18][1][0]*
      ikcnkt[18][0][2])+(cnk[18][1][1]*ikcnkt[18][1][2])))-(mk[13]*(rnk[18][1]*
      rnk[18][2])))+((((cnk[17][1][2]*ikcnkt[17][2][2])+((cnk[17][1][0]*
      ikcnkt[17][0][2])+(cnk[17][1][1]*ikcnkt[17][1][2])))-(mk[12]*(rnk[17][1]*
      rnk[17][2])))+temp[3])));
    icm[1][2] = ((mtot*(com[1]*com[2]))+((((cnk[20][1][2]*ikcnkt[20][2][2])+((
      cnk[20][1][0]*ikcnkt[20][0][2])+(cnk[20][1][1]*ikcnkt[20][1][2])))-(mk[15]
      *(rnk[20][1]*rnk[20][2])))+temp[4]));
    icm[2][0] = icm[0][2];
    icm[2][1] = icm[1][2];
    temp[0] = (((mk[0]*((rnk[5][0]*rnk[5][0])+(rnk[5][1]*rnk[5][1])))+((
      Cik[3][2][2]*ikcnkt[5][2][2])+((Cik[3][2][0]*ikcnkt[5][0][2])+(
      Cik[3][2][1]*ikcnkt[5][1][2]))))+((mk[1]*((rnk[6][0]*rnk[6][0])+(rnk[6][1]
      *rnk[6][1])))+((cnk[6][2][2]*ikcnkt[6][2][2])+((cnk[6][2][0]*
      ikcnkt[6][0][2])+(cnk[6][2][1]*ikcnkt[6][1][2])))));
    temp[1] = (((mk[3]*((rnk[8][0]*rnk[8][0])+(rnk[8][1]*rnk[8][1])))+((
      cnk[8][2][2]*ikcnkt[8][2][2])+((cnk[8][2][0]*ikcnkt[8][0][2])+(
      cnk[8][2][1]*ikcnkt[8][1][2]))))+(((mk[2]*((rnk[7][0]*rnk[7][0])+(
      rnk[7][1]*rnk[7][1])))+((cnk[7][2][2]*ikcnkt[7][2][2])+((cnk[7][2][0]*
      ikcnkt[7][0][2])+(cnk[7][2][1]*ikcnkt[7][1][2]))))+temp[0]));
    temp[2] = (((mk[5]*((rnk[10][0]*rnk[10][0])+(rnk[10][1]*rnk[10][1])))+((
      cnk[10][2][2]*ikcnkt[10][2][2])+((cnk[10][2][0]*ikcnkt[10][0][2])+(
      cnk[10][2][1]*ikcnkt[10][1][2]))))+(((mk[4]*((rnk[9][0]*rnk[9][0])+(
      rnk[9][1]*rnk[9][1])))+((cnk[9][2][2]*ikcnkt[9][2][2])+((cnk[9][2][0]*
      ikcnkt[9][0][2])+(cnk[9][2][1]*ikcnkt[9][1][2]))))+temp[1]));
    temp[3] = (((mk[7]*((rnk[12][0]*rnk[12][0])+(rnk[12][1]*rnk[12][1])))+((
      cnk[12][2][2]*ikcnkt[12][2][2])+((cnk[12][2][0]*ikcnkt[12][0][2])+(
      cnk[12][2][1]*ikcnkt[12][1][2]))))+(((mk[6]*((rnk[11][0]*rnk[11][0])+(
      rnk[11][1]*rnk[11][1])))+((cnk[11][2][2]*ikcnkt[11][2][2])+((cnk[11][2][0]
      *ikcnkt[11][0][2])+(cnk[11][2][1]*ikcnkt[11][1][2]))))+temp[2]));
    temp[4] = (((mk[9]*((rnk[14][0]*rnk[14][0])+(rnk[14][1]*rnk[14][1])))+((
      cnk[14][2][2]*ikcnkt[14][2][2])+((cnk[14][2][0]*ikcnkt[14][0][2])+(
      cnk[14][2][1]*ikcnkt[14][1][2]))))+(((mk[8]*((rnk[13][0]*rnk[13][0])+(
      rnk[13][1]*rnk[13][1])))+((cnk[13][2][2]*ikcnkt[13][2][2])+((cnk[13][2][0]
      *ikcnkt[13][0][2])+(cnk[13][2][1]*ikcnkt[13][1][2]))))+temp[3]));
    temp[5] = (((mk[11]*((rnk[16][0]*rnk[16][0])+(rnk[16][1]*rnk[16][1])))+((
      cnk[16][2][2]*ikcnkt[16][2][2])+((cnk[16][2][0]*ikcnkt[16][0][2])+(
      cnk[16][2][1]*ikcnkt[16][1][2]))))+(((mk[10]*((rnk[15][0]*rnk[15][0])+(
      rnk[15][1]*rnk[15][1])))+((cnk[15][2][2]*ikcnkt[15][2][2])+((cnk[15][2][0]
      *ikcnkt[15][0][2])+(cnk[15][2][1]*ikcnkt[15][1][2]))))+temp[4]));
    temp[6] = (((mk[13]*((rnk[18][0]*rnk[18][0])+(rnk[18][1]*rnk[18][1])))+((
      cnk[18][2][2]*ikcnkt[18][2][2])+((cnk[18][2][0]*ikcnkt[18][0][2])+(
      cnk[18][2][1]*ikcnkt[18][1][2]))))+(((mk[12]*((rnk[17][0]*rnk[17][0])+(
      rnk[17][1]*rnk[17][1])))+((cnk[17][2][2]*ikcnkt[17][2][2])+((cnk[17][2][0]
      *ikcnkt[17][0][2])+(cnk[17][2][1]*ikcnkt[17][1][2]))))+temp[5]));
    icm[2][2] = ((((mk[15]*((rnk[20][0]*rnk[20][0])+(rnk[20][1]*rnk[20][1])))+((
      cnk[20][2][2]*ikcnkt[20][2][2])+((cnk[20][2][0]*ikcnkt[20][0][2])+(
      cnk[20][2][1]*ikcnkt[20][1][2]))))+(((mk[14]*((rnk[19][0]*rnk[19][0])+(
      rnk[19][1]*rnk[19][1])))+((cnk[19][2][2]*ikcnkt[19][2][2])+((cnk[19][2][0]
      *ikcnkt[19][0][2])+(cnk[19][2][1]*ikcnkt[19][1][2]))))+temp[6]))-(mtot*((
      com[0]*com[0])+(com[1]*com[1]))));
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  723 adds/subtracts/negates
                    975 multiplies
                      0 divides
                    193 assignments
*/
}

void sdpos(int body,
    double pt[3],
    double loc[3])
{
/*
Return inertial frame location of a point on a body.

*/
    double pv[3];

    if (sdchkbnum(21,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(21,23);
        return;
    }
    if (body  ==  -1) {
        loc[0] = pt[0];
        loc[1] = pt[1];
        loc[2] = pt[2];
    } else {
        pv[0] = rnb[body][0]+pt[0]*cnb[body][0][0]+pt[1]*cnb[body][0][1]+pt[2]*
          cnb[body][0][2];
        pv[1] = rnb[body][1]+pt[0]*cnb[body][1][0]+pt[1]*cnb[body][1][1]+pt[2]*
          cnb[body][1][2];
        pv[2] = rnb[body][2]+pt[0]*cnb[body][2][0]+pt[1]*cnb[body][2][1]+pt[2]*
          cnb[body][2][2];
        loc[0] = pv[0];
        loc[1] = pv[1];
        loc[2] = pv[2];
    }
}

void sdvel(int body,
    double pt[3],
    double velo[3])
{
/*
Return inertial frame velocity of a point on a body.

*/
    double pv[3];

    if (sdchkbnum(22,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(22,23);
        return;
    }
    if (body  ==  -1) {
        velo[0] = 0.;
        velo[1] = 0.;
        velo[2] = 0.;
    } else {
        pv[0] = wb[body][1]*pt[2]-wb[body][2]*pt[1];
        pv[1] = wb[body][2]*pt[0]-wb[body][0]*pt[2];
        pv[2] = wb[body][0]*pt[1]-wb[body][1]*pt[0];
        velo[0] = vnb[body][0]+pv[0]*cnb[body][0][0]+pv[1]*cnb[body][0][1]+pv[2]
          *cnb[body][0][2];
        velo[1] = vnb[body][1]+pv[0]*cnb[body][1][0]+pv[1]*cnb[body][1][1]+pv[2]
          *cnb[body][1][2];
        velo[2] = vnb[body][2]+pv[0]*cnb[body][2][0]+pv[1]*cnb[body][2][1]+pv[2]
          *cnb[body][2][2];
    }
}

void sdorient(int body,
    double dircos[3][3])
{
/*
Return orientation of body w.r.t. ground frame.

*/

    if (sdchkbnum(23,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(23,23);
        return;
    }
    if (body == -1) {
        dircos[0][0] = 1.;
        dircos[0][1] = 0.;
        dircos[0][2] = 0.;
        dircos[1][0] = 0.;
        dircos[1][1] = 1.;
        dircos[1][2] = 0.;
        dircos[2][0] = 0.;
        dircos[2][1] = 0.;
        dircos[2][2] = 1.;
    } else {
        dircos[0][0] = cnb[body][0][0];
        dircos[0][1] = cnb[body][0][1];
        dircos[0][2] = cnb[body][0][2];
        dircos[1][0] = cnb[body][1][0];
        dircos[1][1] = cnb[body][1][1];
        dircos[1][2] = cnb[body][1][2];
        dircos[2][0] = cnb[body][2][0];
        dircos[2][1] = cnb[body][2][1];
        dircos[2][2] = cnb[body][2][2];
    }
}

void sdangvel(int body,
    double avel[3])
{
/*
Return angular velocity of the body.

*/

    if (sdchkbnum(24,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(24,23);
        return;
    }
    if (body == -1) {
        avel[0] = 0.;
        avel[1] = 0.;
        avel[2] = 0.;
    } else {
        avel[0] = wb[body][0];
        avel[1] = wb[body][1];
        avel[2] = wb[body][2];
    }
}

void sdtrans(int frbod,
    double ivec[3],
    int tobod,
    double ovec[3])
{
/*
Transform ivec from frbod frame to tobod frame.

*/
    double pv[3];

    if (sdchkbnum(25,frbod) != 0) {
        return;
    }
    if (sdchkbnum(25,tobod) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(25,23);
        return;
    }
    if (frbod == tobod) {
        sdvcopy(ivec,ovec);
        return;
    }
    if (frbod == -1) {
        sdvcopy(ivec,pv);
        ovec[0] = pv[0]*cnb[tobod][0][0]+pv[1]*cnb[tobod][1][0]+pv[2]*cnb[tobod
          ][2][0];
        ovec[1] = pv[0]*cnb[tobod][0][1]+pv[1]*cnb[tobod][1][1]+pv[2]*cnb[tobod
          ][2][1];
        ovec[2] = pv[0]*cnb[tobod][0][2]+pv[1]*cnb[tobod][1][2]+pv[2]*cnb[tobod
          ][2][2];
        return;
    }
    if (tobod == -1) {
        sdvcopy(ivec,pv);
        ovec[0] = pv[0]*cnb[frbod][0][0]+pv[1]*cnb[frbod][0][1]+pv[2]*cnb[frbod
          ][0][2];
        ovec[1] = pv[0]*cnb[frbod][1][0]+pv[1]*cnb[frbod][1][1]+pv[2]*cnb[frbod
          ][1][2];
        ovec[2] = pv[0]*cnb[frbod][2][0]+pv[1]*cnb[frbod][2][1]+pv[2]*cnb[frbod
          ][2][2];
        return;
    }
    pv[0] = ivec[0]*cnb[frbod][0][0]+ivec[1]*cnb[frbod][0][1]+ivec[2]*cnb[frbod
      ][0][2];
    pv[1] = ivec[0]*cnb[frbod][1][0]+ivec[1]*cnb[frbod][1][1]+ivec[2]*cnb[frbod
      ][1][2];
    pv[2] = ivec[0]*cnb[frbod][2][0]+ivec[1]*cnb[frbod][2][1]+ivec[2]*cnb[frbod
      ][2][2];
    ovec[0] = pv[0]*cnb[tobod][0][0]+pv[1]*cnb[tobod][1][0]+pv[2]*cnb[tobod][2][
      0];
    ovec[1] = pv[0]*cnb[tobod][0][1]+pv[1]*cnb[tobod][1][1]+pv[2]*cnb[tobod][2][
      1];
    ovec[2] = pv[0]*cnb[tobod][0][2]+pv[1]*cnb[tobod][1][2]+pv[2]*cnb[tobod][2][
      2];
}

void sdrel2cart(int coord,
    int body,
    double point[3],
    double linchg[3],
    double rotchg[3])
{
/* Return derivative of pt loc and body orient w.r.t. hinge rate
*/
    int x,i,gnd;
    double lin[3],pv[3];

    if ((coord < 0) || (coord > 20)) {
        sdseterr(59,45);
        return;
    }
    if (sdchkbnum(59,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(59,23);
        return;
    }
    gnd = -1;
    if (body == gnd) {
        x = -1;
    } else {
        x = firstq[body]+njntdof[body]-1;
    }
    if (x < coord) {
        sdvset(0.,0.,0.,linchg);
        sdvset(0.,0.,0.,rotchg);
        return;
    }
    sddovpk();
    for (i = 0; i < 3; i++) {
        rotchg[i] = Wpk[coord][x][i];
        lin[i] = Vpk[coord][x][i];
    }
    if (body == gnd) {
        sdvcopy(point,pv);
    } else {
        pv[0] = rcom[body][0]+point[0];
        pv[1] = rcom[body][1]+point[1];
        pv[2] = rcom[body][2]+point[2];
    }
    sdvcross(rotchg,pv,linchg);
    sdvadd(linchg,lin,linchg);
}

void sdacc(int body,
    double pt[3],
    double accel[3])
{
/*
Return linear acceleration a point of the specified body.

*/
    double pv[3];

    if (sdchkbnum(32,body) != 0) {
        return;
    }
    if (roustate != 3) {
        sdseterr(32,24);
        return;
    }
    if (body  ==  -1) {
        accel[0] = 0.;
        accel[1] = 0.;
        accel[2] = 0.;
    } else {
        pv[0] = pt[0]*dyad[body][0][0]+pt[1]*dyad[body][0][1]+pt[2]*dyad[body][0
          ][2];
        pv[1] = pt[0]*dyad[body][1][0]+pt[1]*dyad[body][1][1]+pt[2]*dyad[body][1
          ][2];
        pv[2] = pt[0]*dyad[body][2][0]+pt[1]*dyad[body][2][1]+pt[2]*dyad[body][2
          ][2];
        accel[0] = anb[body][0]+pv[0]*cnb[body][0][0]+pv[1]*cnb[body][0][1]+pv[2
          ]*cnb[body][0][2];
        accel[1] = anb[body][1]+pv[0]*cnb[body][1][0]+pv[1]*cnb[body][1][1]+pv[2
          ]*cnb[body][1][2];
        accel[2] = anb[body][2]+pv[0]*cnb[body][2][0]+pv[1]*cnb[body][2][1]+pv[2
          ]*cnb[body][2][2];
    }
}

void sdangacc(int body,
    double aacc[3])
{
/*
Return angular acceleration of the body.

*/

    if (sdchkbnum(33,body) != 0) {
        return;
    }
    if (roustate != 3) {
        sdseterr(33,24);
        return;
    }
    if (body == -1) {
        aacc[0] = 0.;
        aacc[1] = 0.;
        aacc[2] = 0.;
    } else {
        aacc[0] = onb[body][0];
        aacc[1] = onb[body][1];
        aacc[2] = onb[body][2];
    }
}

void sdgrav(double gravin[3])
{

    grav[0] = gravin[0];
    gravq[0] = 3;
    grav[1] = gravin[1];
    gravq[1] = 3;
    grav[2] = gravin[2];
    gravq[2] = 3;
    roustate = 0;
}

void sdmass(int body,
    double massin)
{

    if (sdchkbnum(2,body) != 0) {
        return;
    }
    if (body == -1) {
        sdseterr(2,15);
        return;
    }
    if (mkq[body] != 0) {
        mk[body] = massin;
        mkq[body] = 3;
    } else {
        sdseterr(2,19);
    }
    roustate = 0;
}

void sdiner(int body,
    double inerin[3][3])
{
    int anyques;

    if (sdchkbnum(3,body) != 0) {
        return;
    }
    if (body == -1) {
        sdseterr(3,15);
        return;
    }
    anyques = 0;
    if (ikq[body][0][0]  !=  0) {
        ik[body][0][0] = inerin[0][0];
        ikq[body][0][0] = 3;
        anyques = 1;
    }
    if (ikq[body][0][1]  !=  0) {
        ik[body][0][1] = inerin[0][1];
        ikq[body][0][1] = 3;
        ik[body][1][0] = inerin[0][1];
        ikq[body][1][0] = 3;
        anyques = 1;
    }
    if (ikq[body][0][2]  !=  0) {
        ik[body][0][2] = inerin[0][2];
        ikq[body][0][2] = 3;
        ik[body][2][0] = inerin[0][2];
        ikq[body][2][0] = 3;
        anyques = 1;
    }
    if (ikq[body][1][1]  !=  0) {
        ik[body][1][1] = inerin[1][1];
        ikq[body][1][1] = 3;
        anyques = 1;
    }
    if (ikq[body][1][2]  !=  0) {
        ik[body][1][2] = inerin[1][2];
        ikq[body][1][2] = 3;
        ik[body][2][1] = inerin[1][2];
        ikq[body][2][1] = 3;
        anyques = 1;
    }
    if (ikq[body][2][2]  !=  0) {
        ik[body][2][2] = inerin[2][2];
        ikq[body][2][2] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        sdseterr(3,19);
    }
    roustate = 0;
}

void sdbtj(int joint,
    double btjin[3])
{
    int anyques;

    if (sdchkjnum(4,joint) != 0) {
        return;
    }
    anyques = 0;
    if (rkq[joint][0]  !=  0) {
        rk[joint][0] = btjin[0];
        rkq[joint][0] = 3;
        anyques = 1;
    }
    if (rkq[joint][1]  !=  0) {
        rk[joint][1] = btjin[1];
        rkq[joint][1] = 3;
        anyques = 1;
    }
    if (rkq[joint][2]  !=  0) {
        rk[joint][2] = btjin[2];
        rkq[joint][2] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        sdseterr(4,19);
    }
    roustate = 0;
}

void sditj(int joint,
    double itjin[3])
{
    int anyques;

    if (sdchkjnum(5,joint) != 0) {
        return;
    }
    anyques = 0;
    if (riq[joint][0]  !=  0) {
        ri[joint][0] = itjin[0];
        riq[joint][0] = 3;
        anyques = 1;
    }
    if (riq[joint][1]  !=  0) {
        ri[joint][1] = itjin[1];
        riq[joint][1] = 3;
        anyques = 1;
    }
    if (riq[joint][2]  !=  0) {
        ri[joint][2] = itjin[2];
        riq[joint][2] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        sdseterr(5,19);
    }
    roustate = 0;
}

void sdpin(int joint,
    int pinno,
    double pinin[3])
{
    int anyques,offs;

    if (sdchkjpin(6,joint,pinno) != 0) {
        return;
    }
    anyques = 0;
    offs = firstq[joint]+pinno;
    if (jtype[joint] == 21) {
        offs = offs+3;
    }
    if (jtype[joint] == 11) {
        offs = offs+1;
    }
    if (pinq[offs][0]  !=  0) {
        pin[offs][0] = pinin[0];
        pinq[offs][0] = 3;
        anyques = 1;
    }
    if (pinq[offs][1]  !=  0) {
        pin[offs][1] = pinin[1];
        pinq[offs][1] = 3;
        anyques = 1;
    }
    if (pinq[offs][2]  !=  0) {
        pin[offs][2] = pinin[2];
        pinq[offs][2] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        sdseterr(6,19);
    }
    roustate = 0;
}

void sdpres(int joint,
    int axis,
    int presin)
{
    int anyques;

    if (sdchkjaxis(37,joint,axis) != 0) {
        return;
    }
    if ((presin != 0) && (presin != 1)) {
        sdseterr(37,20);
    }
    anyques = 0;
    if (presq[sdindx(joint,axis)]  !=  0) {
        if (presin  !=  0) {
            pres[sdindx(joint,axis)] = 1.;
        } else {
            pres[sdindx(joint,axis)] = 0.;
        }
        presq[sdindx(joint,axis)] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        sdseterr(37,19);
    }
    wwflg = 0;
}

void sdconschg(void)
{

    wwflg = 0;
}

void sdstab(double velin,
    double posin)
{

    stabvel = velin;
    stabvelq = 3;
    stabpos = posin;
    stabposq = 3;
}

void sdgetgrav(double gravout[3])
{

    gravout[0] = grav[0];
    gravout[1] = grav[1];
    gravout[2] = grav[2];
}

void sdgetmass(int body,
    double *massout)
{

    if (sdchkbnum(40,body) != 0) {
        return;
    }
    if (body == -1) {
        sdseterr(40,15);
        return;
    }
    *massout = mk[body];
}

void sdgetiner(int body,
    double inerout[3][3])
{

    if (sdchkbnum(41,body) != 0) {
        return;
    }
    if (body == -1) {
        sdseterr(41,15);
        return;
    }
    inerout[0][0] = ik[body][0][0];
    inerout[0][1] = ik[body][0][1];
    inerout[0][2] = ik[body][0][2];
    inerout[1][0] = ik[body][1][0];
    inerout[1][1] = ik[body][1][1];
    inerout[1][2] = ik[body][1][2];
    inerout[2][0] = ik[body][2][0];
    inerout[2][1] = ik[body][2][1];
    inerout[2][2] = ik[body][2][2];
}

void sdgetbtj(int joint,
    double btjout[3])
{

    if (sdchkjnum(42,joint) != 0) {
        return;
    }
    btjout[0] = rk[joint][0];
    btjout[1] = rk[joint][1];
    btjout[2] = rk[joint][2];
}

void sdgetitj(int joint,
    double itjout[3])
{

    if (sdchkjnum(43,joint) != 0) {
        return;
    }
    itjout[0] = ri[joint][0];
    itjout[1] = ri[joint][1];
    itjout[2] = ri[joint][2];
}

void sdgetpin(int joint,
    int pinno,
    double pinout[3])
{
    int offs;

    if (sdchkjpin(44,joint,pinno) != 0) {
        return;
    }
    offs = firstq[joint]+pinno;
    if (jtype[joint] == 21) {
        offs = offs+3;
    }
    if (jtype[joint] == 11) {
        offs = offs+1;
    }
    pinout[0] = pin[offs][0];
    pinout[1] = pin[offs][1];
    pinout[2] = pin[offs][2];
}

void sdgetpres(int joint,
    int axis,
    int *presout)
{

    if (sdchkjaxis(45,joint,axis) != 0) {
        return;
    }
    if (pres[sdindx(joint,axis)]  !=  0.) {
        *presout = 1;
    } else {
        *presout = 0;
    }
}

void sdgetstab(double *velout,
    double *posout)
{

    *velout = stabvel;
    *posout = stabpos;
}

void sdinfo(int info[50])
{

    info[0] = ground;
    info[1] = nbod;
    info[2] = ndof;
    info[3] = ncons;
    info[4] = nloop;
    info[5] = nldof;
    info[6] = nloopc;
    info[7] = nball;
    info[8] = nlball;
    info[9] = npres;
    info[10] = nuser;
    info[11] = 21;
/* info entries from 12-49 are reserved */
}

void sdjnt(int joint,
    int info[50],
    int tran[6])
{
    int i,offs;

    if (sdchkjnum(48,joint) != 0) {
        return;
    }
    info[0] = jtype[joint];
    info[1] = 0;
    offs = 0;
    info[2] = inb[joint];
    info[3] = outb[joint];
    info[4] = njntdof[joint];
    info[5] = njntc[joint];
    info[6] = njntp[joint];
    info[7] = firstq[joint];
    info[8] = ballq[joint];
    info[9] = firstm[joint];
    info[10] = firstp[joint];
/* info entries from 11-49 are reserved */

    for (i = 0; i <= 5; i++) {
        if (i  <  njntdof[joint]) {
            tran[i] = trans[offs+firstq[joint]+i];
        } else {
            tran[i] = -1;
        }
    }
}

void sdcons(int consno,
    int info[50])
{

    if (sdchkucnum(49,consno) != 0) {
        return;
    }
/* There are no user constraints in this problem. */
}

void sdgentime(int *gentm)
{

    *gentm = 42000;
}
/*
Done. CPU seconds used: 0.00  Memory used: 0 bytes.
Equation complexity:
  sdstate:  1892 adds  2560 multiplies     4 divides  1472 assignments
  sdderiv: 31498 adds 33928 multiplies   543 divides 33290 assignments
  sdresid:  4603 adds  5079 multiplies     0 divides  1985 assignments
  sdreac:    828 adds   678 multiplies     0 divides   402 assignments
  sdmom:     479 adds   535 multiplies     0 divides   131 assignments
  sdsys:     723 adds   975 multiplies     0 divides   193 assignments
*/
