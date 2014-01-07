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


ROADMAP (drc_pelvis_upper.sd)

Bodies        Inb
No  Name      body Joint type  Coords q         Multipliers
--- --------- ---- ----------- ---------------- -----------------------
 -1 $ground                                    |
  0 utorso     -1  Pin           0?            |  0p
  1 head        0  Pin           1?            |  1p
  2 l_clav      0  Pin           2?            |  2p
  3 l_scap      2  Pin           3?            |  3p
  4 l_uarm      3  Pin           4?            |  4p
  5 l_larm      4  Pin           5?            |  5p
  6 l_farm      5  Pin           6?            |  6p
  7 l_hand      6  Pin           7?            |  7p
  8 r_clav      0  Pin           8?            |  8p
  9 r_scap      8  Pin           9?            |  9p
 10 r_uarm      9  Pin          10?            | 10p
 11 r_larm     10  Pin          11?            | 11p
 12 r_farm     11  Pin          12?            | 12p
 13 r_hand     12  Pin          13?            | 13p

*/
#include <math.h>
#include <stdio.h>

typedef struct {
    int ground_,nbod_,ndof_,ncons_,nloop_,nldof_,nloopc_,nball_,nlball_,npres_,
      nuser_;
    int jtype_[14],inb_[14],outb_[14],njntdof_[14],njntc_[14],njntp_[14],firstq_
      [14],ballq_[14],firstm_[14],firstp_[14];
    int trans_[14];
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
    double grav_[3],mk_[14],ik_[14][3][3],pin_[14][3];
    double rk_[14][3],ri_[14][3],pres_[14],stabvel_,stabpos_;
    int mfrcflg_,roustate_,vpkflg_,inerflg_,mmflg_,mmlduflg_,wwflg_,ltauflg_,
      fs0flg_,ii_,mmap_[14];
    int gravq_[3],mkq_[14],ikq_[14][3][3],pinq_[14][3],rkq_[14][3],riq_[14][3],
      presq_[14],stabvelq_,stabposq_;
    double mtot_,psmkg_,rhead_[14][3],rcom_[14][3],mkrcomt_[14][3][3],psikg_[3][
      3],psrcomg_[3],psrkg_[3],psrig_[3],psmk_[14],psik_[14][3][3],psrcom_[14][3
      ],psrk_[14][3],psri_[14][3];
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
    double curtim_,q_[14],qn_[14],u_[14],cnk_[14][3][3],cnb_[14][3][3];
    double rnk_[14][3],vnk_[14][3],wk_[14][3],rnb_[14][3],vnb_[14][3],wb_[14][3]
      ,wbrcom_[14][3],com_[3],rnkg_[3];
    double Cik_[14][3][3],rikt_[14][3][3],Iko_[14][3][3],mkrk_[14][3][3],Cib_[14
      ][3][3];
    double Wkk_[14][3],Vkk_[14][3],dik_[14][3],rpp_[14][3],rpk_[14][3],rik_[14][
      3],rik2_[14][3];
    double rpri_[14][3],Wik_[14][3],Vik_[14][3],Wirk_[14][3],rkWkk_[14][3],
      Wkrpk_[14][3],VikWkr_[14][3];
    double perr_[14],verr_[14],aerr_[14],mult_[14],ufk_[14][3],utk_[14][3],mfk_[
      14][3],mtk_[14][3];
    double utau_[14],mtau_[14],uacc_[14],uvel_[14],upos_[14];
    double s0_,c0_,s1_,c1_,s2_,c2_,s3_,c3_,s4_,c4_,s5_,c5_,s6_,c6_,s7_,c7_,s8_,
      c8_,s9_,c9_,s10_,c10_,s11_,c11_,s12_,c12_,s13_,c13_;
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
#define s0 (sdgstate.s0_)
#define c0 (sdgstate.c0_)
#define s1 (sdgstate.s1_)
#define c1 (sdgstate.c1_)
#define s2 (sdgstate.s2_)
#define c2 (sdgstate.c2_)
#define s3 (sdgstate.s3_)
#define c3 (sdgstate.c3_)
#define s4 (sdgstate.s4_)
#define c4 (sdgstate.c4_)
#define s5 (sdgstate.s5_)
#define c5 (sdgstate.c5_)
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

typedef struct {
    double fs0_[14],qdot_[14],Otk_[14][3],Atk_[14][3],AiOiWi_[14][3],Fstar_[14][
      3];
    double Tstar_[14][3],Fstark_[14][3],Tstark_[14][3],IkWk_[14][3],WkIkWk_[14][
      3],gk_[14][3],IkbWk_[14][3],WkIkbWk_[14][3];
    double w0w0_[14],w1w1_[14],w2w2_[14],w0w1_[14],w0w2_[14],w1w2_[14];
    double w00w11_[14],w00w22_[14],w11w22_[14],ww_[14][14],qraux_[14];
    double mm_[14][14],mlo_[14][14],mdi_[14],IkWpk_[14][14][3],works_[14],
      workss_[14][14];
    double Wpk_[14][14][3],Vpk_[14][14][3],VWri_[14][14][3];
    int wmap_[14],multmap_[14],jpvt_[14],wsiz_,wrank_;
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
    double fs_[14],udot_[14],tauc_[14],dyad_[14][3][3],fc_[14][3],tc_[14][3];
    double ank_[14][3],onk_[14][3],Onkb_[14][3],AOnkri_[14][3],Ankb_[14][3],
      AnkAtk_[14][3],anb_[14][3],onb_[14][3],dyrcom_[14][3];
    double ffk_[14][3],ttk_[14][3],fccikt_[14][3],ffkb_[14][3],ttkb_[14][3];
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
    /* nbod */ 14,
    /* ndof */ 14,
    /* ncons */ 14,
    /* nloop */ 0,
    /* nldof */ 0,
    /* nloopc */ 0,
    /* nball */ 0,
    /* nlball */ 0,
    /* npres */ 14,
    /* nuser */ 0,
    /* jtype[0] */ 1,
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
    /* inb[0] */ -1,
    /* inb[1] */ 0,
    /* inb[2] */ 0,
    /* inb[3] */ 2,
    /* inb[4] */ 3,
    /* inb[5] */ 4,
    /* inb[6] */ 5,
    /* inb[7] */ 6,
    /* inb[8] */ 0,
    /* inb[9] */ 8,
    /* inb[10] */ 9,
    /* inb[11] */ 10,
    /* inb[12] */ 11,
    /* inb[13] */ 12,
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
    /* njntdof[0] */ 1,
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
    /* njntp[0] */ 1,
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
    /* firstq[0] */ 0,
    /* firstq[1] */ 1,
    /* firstq[2] */ 2,
    /* firstq[3] */ 3,
    /* firstq[4] */ 4,
    /* firstq[5] */ 5,
    /* firstq[6] */ 6,
    /* firstq[7] */ 7,
    /* firstq[8] */ 8,
    /* firstq[9] */ 9,
    /* firstq[10] */ 10,
    /* firstq[11] */ 11,
    /* firstq[12] */ 12,
    /* firstq[13] */ 13,
    /* ballq[0] */ -104,
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
    /* firstp[0] */ 0,
    /* firstp[1] */ 1,
    /* firstp[2] */ 2,
    /* firstp[3] */ 3,
    /* firstp[4] */ 4,
    /* firstp[5] */ 5,
    /* firstp[6] */ 6,
    /* firstp[7] */ 7,
    /* firstp[8] */ 8,
    /* firstp[9] */ 9,
    /* firstp[10] */ 10,
    /* firstp[11] */ 11,
    /* firstp[12] */ 12,
    /* firstp[13] */ 13,
    /* trans[0] */ 0,
    /* trans[1] */ 0,
    /* trans[2] */ 0,
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
};
sdginput_t sdginput = {
/* Model parameters from the input file */

/* gravity */
    /* grav[0] */ 0.,
    /* grav[1] */ 0.,
    /* grav[2] */ -9.81,

/* mass */
    /* mk[0] */ 15.2272,
    /* mk[1] */ 1.477574,
    /* mk[2] */ 2.369,
    /* mk[3] */ 2.707,
    /* mk[4] */ 1.881,
    /* mk[5] */ 2.148,
    /* mk[6] */ .981,
    /* mk[7] */ 5.394,
    /* mk[8] */ 2.369,
    /* mk[9] */ 2.707,
    /* mk[10] */ 1.881,
    /* mk[11] */ 2.148,
    /* mk[12] */ .981,
    /* mk[13] */ 5.394,

/* inertia */
    /* ik[0][0][0] */ .395,
    /* ik[0][0][1] */ 0.,
    /* ik[0][0][2] */ .083,
    /* ik[0][1][0] */ 0.,
    /* ik[0][1][1] */ .544742,
    /* ik[0][1][2] */ -.003,
    /* ik[0][2][0] */ .083,
    /* ik[0][2][1] */ -.003,
    /* ik[0][2][2] */ .10973,
    /* ik[1][0][0] */ .00418903653814214,
    /* ik[1][0][1] */ -2.35278149506389e-6,
    /* ik[1][0][2] */ -.00101651793086611,
    /* ik[1][1][0] */ -2.35278149506389e-6,
    /* ik[1][1][1] */ .00444693042161052,
    /* ik[1][1][2] */ -1.86690202773907e-6,
    /* ik[1][2][0] */ -.00101651793086611,
    /* ik[1][2][1] */ -1.86690202773907e-6,
    /* ik[1][2][2] */ .00367874242901065,
    /* ik[2][0][0] */ .004,
    /* ik[2][0][1] */ .001,
    /* ik[2][0][2] */ 0.,
    /* ik[2][1][0] */ .001,
    /* ik[2][1][1] */ .006,
    /* ik[2][1][2] */ 0.,
    /* ik[2][2][0] */ 0.,
    /* ik[2][2][1] */ 0.,
    /* ik[2][2][2] */ .007,
    /* ik[3][0][0] */ .01,
    /* ik[3][0][1] */ 0.,
    /* ik[3][0][2] */ 0.,
    /* ik[3][1][0] */ 0.,
    /* ik[3][1][1] */ .005,
    /* ik[3][1][2] */ 0.,
    /* ik[3][2][0] */ 0.,
    /* ik[3][2][1] */ 0.,
    /* ik[3][2][2] */ .013,
    /* ik[4][0][0] */ .002,
    /* ik[4][0][1] */ 0.,
    /* ik[4][0][2] */ 0.,
    /* ik[4][1][0] */ 0.,
    /* ik[4][1][1] */ .003,
    /* ik[4][1][2] */ 0.,
    /* ik[4][2][0] */ 0.,
    /* ik[4][2][1] */ 0.,
    /* ik[4][2][2] */ .003,
    /* ik[5][0][0] */ .005,
    /* ik[5][0][1] */ 0.,
    /* ik[5][0][2] */ 0.,
    /* ik[5][1][0] */ 0.,
    /* ik[5][1][1] */ .003,
    /* ik[5][1][2] */ 0.,
    /* ik[5][2][0] */ 0.,
    /* ik[5][2][1] */ 0.,
    /* ik[5][2][2] */ .006,
    /* ik[6][0][0] */ .003,
    /* ik[6][0][1] */ 0.,
    /* ik[6][0][2] */ 0.,
    /* ik[6][1][0] */ 0.,
    /* ik[6][1][1] */ .001,
    /* ik[6][1][2] */ 0.,
    /* ik[6][2][0] */ 0.,
    /* ik[6][2][1] */ 0.,
    /* ik[6][2][2] */ .003,
    /* ik[7][0][0] */ .0610247538962229,
    /* ik[7][0][1] */ .000807757508053441,
    /* ik[7][0][2] */ .00179517233146595,
    /* ik[7][1][0] */ .000807757508053441,
    /* ik[7][1][1] */ .0200234313085275,
    /* ik[7][1][2] */ -.00589549614920727,
    /* ik[7][2][0] */ .00179517233146595,
    /* ik[7][2][1] */ -.00589549614920727,
    /* ik[7][2][2] */ .0507317450425557,
    /* ik[8][0][0] */ .004,
    /* ik[8][0][1] */ -.001,
    /* ik[8][0][2] */ 0.,
    /* ik[8][1][0] */ -.001,
    /* ik[8][1][1] */ .006,
    /* ik[8][1][2] */ 0.,
    /* ik[8][2][0] */ 0.,
    /* ik[8][2][1] */ 0.,
    /* ik[8][2][2] */ .007,
    /* ik[9][0][0] */ .01,
    /* ik[9][0][1] */ 0.,
    /* ik[9][0][2] */ 0.,
    /* ik[9][1][0] */ 0.,
    /* ik[9][1][1] */ .005,
    /* ik[9][1][2] */ 0.,
    /* ik[9][2][0] */ 0.,
    /* ik[9][2][1] */ 0.,
    /* ik[9][2][2] */ .013,
    /* ik[10][0][0] */ .002,
    /* ik[10][0][1] */ 0.,
    /* ik[10][0][2] */ 0.,
    /* ik[10][1][0] */ 0.,
    /* ik[10][1][1] */ .003,
    /* ik[10][1][2] */ 0.,
    /* ik[10][2][0] */ 0.,
    /* ik[10][2][1] */ 0.,
    /* ik[10][2][2] */ .003,
    /* ik[11][0][0] */ .005,
    /* ik[11][0][1] */ 0.,
    /* ik[11][0][2] */ 0.,
    /* ik[11][1][0] */ 0.,
    /* ik[11][1][1] */ .003,
    /* ik[11][1][2] */ 0.,
    /* ik[11][2][0] */ 0.,
    /* ik[11][2][1] */ 0.,
    /* ik[11][2][2] */ .006,
    /* ik[12][0][0] */ .003,
    /* ik[12][0][1] */ 0.,
    /* ik[12][0][2] */ 0.,
    /* ik[12][1][0] */ 0.,
    /* ik[12][1][1] */ .001,
    /* ik[12][1][2] */ 0.,
    /* ik[12][2][0] */ 0.,
    /* ik[12][2][1] */ 0.,
    /* ik[12][2][2] */ .003,
    /* ik[13][0][0] */ .0610247538962229,
    /* ik[13][0][1] */ -.000807757508053441,
    /* ik[13][0][2] */ .00179517233146595,
    /* ik[13][1][0] */ -.000807757508053441,
    /* ik[13][1][1] */ .0200234313085275,
    /* ik[13][1][2] */ .00589549614920727,
    /* ik[13][2][0] */ .00179517233146595,
    /* ik[13][2][1] */ .00589549614920727,
    /* ik[13][2][2] */ .0507317450425557,

/* tree hinge axis vectors */
    /* pin[0][0] */ 1.,
    /* pin[0][1] */ 0.,
    /* pin[0][2] */ 0.,
    /* pin[1][0] */ 0.,
    /* pin[1][1] */ 1.,
    /* pin[1][2] */ 0.,
    /* pin[2][0] */ 0.,
    /* pin[2][1] */ .5,
    /* pin[2][2] */ .866025302838276,
    /* pin[3][0] */ 1.,
    /* pin[3][1] */ 0.,
    /* pin[3][2] */ 0.,
    /* pin[4][0] */ 0.,
    /* pin[4][1] */ 1.,
    /* pin[4][2] */ 0.,
    /* pin[5][0] */ 1.,
    /* pin[5][1] */ 0.,
    /* pin[5][2] */ 0.,
    /* pin[6][0] */ 0.,
    /* pin[6][1] */ 1.,
    /* pin[6][2] */ 0.,
    /* pin[7][0] */ 1.,
    /* pin[7][1] */ 0.,
    /* pin[7][2] */ 0.,
    /* pin[8][0] */ 0.,
    /* pin[8][1] */ .5,
    /* pin[8][2] */ -.866025302838276,
    /* pin[9][0] */ 1.,
    /* pin[9][1] */ 0.,
    /* pin[9][2] */ 0.,
    /* pin[10][0] */ 0.,
    /* pin[10][1] */ 1.,
    /* pin[10][2] */ 0.,
    /* pin[11][0] */ 1.,
    /* pin[11][1] */ 0.,
    /* pin[11][2] */ 0.,
    /* pin[12][0] */ 0.,
    /* pin[12][1] */ 1.,
    /* pin[12][2] */ 0.,
    /* pin[13][0] */ 1.,
    /* pin[13][1] */ 0.,
    /* pin[13][2] */ 0.,

/* tree bodytojoint vectors */
    /* rk[0][0] */ -.02,
    /* rk[0][1] */ .001,
    /* rk[0][2] */ -.211,
    /* rk[1][0] */ .0738018469067539,
    /* rk[1][1] */ -4.84921274799096e-5,
    /* rk[1][2] */ -.0299299972676836,
    /* rk[2][0] */ -.014,
    /* rk[2][1] */ -.058,
    /* rk[2][2] */ -.029,
    /* rk[3][0] */ .002,
    /* rk[3][1] */ -.108,
    /* rk[3][2] */ 0.,
    /* rk[4][0] */ -.007,
    /* rk[4][1] */ -.114,
    /* rk[4][2] */ -.008,
    /* rk[5][0] */ .003,
    /* rk[5][1] */ -.099,
    /* rk[5][2] */ .014,
    /* rk[6][0] */ 0.,
    /* rk[6][1] */ -.041,
    /* rk[6][2] */ 0.,
    /* rk[7][0] */ .00105503165687245,
    /* rk[7][1] */ -.114261222326107,
    /* rk[7][2] */ -.015858773894309,
    /* rk[8][0] */ -.014,
    /* rk[8][1] */ .058,
    /* rk[8][2] */ -.029,
    /* rk[9][0] */ .002,
    /* rk[9][1] */ .108,
    /* rk[9][2] */ 0.,
    /* rk[10][0] */ -.007,
    /* rk[10][1] */ .114,
    /* rk[10][2] */ -.008,
    /* rk[11][0] */ .003,
    /* rk[11][1] */ .099,
    /* rk[11][2] */ .014,
    /* rk[12][0] */ 0.,
    /* rk[12][1] */ .041,
    /* rk[12][2] */ 0.,
    /* rk[13][0] */ .00105503165687245,
    /* rk[13][1] */ .114261222326107,
    /* rk[13][2] */ -.015858773894309,

/* tree inbtojoint vectors */
    /* ri[0][0] */ 0.,
    /* ri[0][1] */ 0.,
    /* ri[0][2] */ 0.,
    /* ri[1][0] */ .19672,
    /* ri[1][1] */ .001,
    /* ri[1][2] */ .32696,
    /* ri[2][0] */ .004,
    /* ri[2][1] */ .222,
    /* ri[2][2] */ .078,
    /* ri[3][0] */ -.014,
    /* ri[3][1] */ .017,
    /* ri[3][2] */ .007,
    /* ri[4][0] */ .002,
    /* ri[4][1] */ .077,
    /* ri[4][2] */ 0.,
    /* ri[5][0] */ -.007,
    /* ri[5][1] */ .007,
    /* ri[5][2] */ .005,
    /* ri[6][0] */ .003,
    /* ri[6][1] */ .089,
    /* ri[6][2] */ .001,
    /* ri[7][0] */ 0.,
    /* ri[7][1] */ .017,
    /* ri[7][2] */ 0.,
    /* ri[8][0] */ .004,
    /* ri[8][1] */ -.22,
    /* ri[8][2] */ .078,
    /* ri[9][0] */ -.014,
    /* ri[9][1] */ -.017,
    /* ri[9][2] */ .007,
    /* ri[10][0] */ .002,
    /* ri[10][1] */ -.077,
    /* ri[10][2] */ 0.,
    /* ri[11][0] */ -.007,
    /* ri[11][1] */ -.007,
    /* ri[11][2] */ .005,
    /* ri[12][0] */ .003,
    /* ri[12][1] */ -.089,
    /* ri[12][2] */ .001,
    /* ri[13][0] */ 0.,
    /* ri[13][1] */ -.017,
    /* ri[13][2] */ 0.,

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
    /* pinq[0][0] */ 3,
    /* pinq[0][1] */ 3,
    /* pinq[0][2] */ 3,
    /* pinq[1][0] */ 3,
    /* pinq[1][1] */ 3,
    /* pinq[1][2] */ 3,
    /* pinq[2][0] */ 3,
    /* pinq[2][1] */ 3,
    /* pinq[2][2] */ 3,
    /* pinq[3][0] */ 3,
    /* pinq[3][1] */ 3,
    /* pinq[3][2] */ 3,
    /* pinq[4][0] */ 3,
    /* pinq[4][1] */ 3,
    /* pinq[4][2] */ 3,
    /* pinq[5][0] */ 3,
    /* pinq[5][1] */ 3,
    /* pinq[5][2] */ 3,
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
    for (k = 0; k < 14; k++) {
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
    for (k = 0; k < 14; k++) {
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
    sumsq = ((pin[3][2]*pin[3][2])+((pin[3][0]*pin[3][0])+(pin[3][1]*pin[3][1]))
      );
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[3][0] = (norminv*pin[3][0]);
    pin[3][1] = (norminv*pin[3][1]);
    pin[3][2] = (norminv*pin[3][2]);
    sumsq = ((pin[4][2]*pin[4][2])+((pin[4][0]*pin[4][0])+(pin[4][1]*pin[4][1]))
      );
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[4][0] = (norminv*pin[4][0]);
    pin[4][1] = (norminv*pin[4][1]);
    pin[4][2] = (norminv*pin[4][2]);
    sumsq = ((pin[5][2]*pin[5][2])+((pin[5][0]*pin[5][0])+(pin[5][1]*pin[5][1]))
      );
    if ((sumsq < 1e-10)  ) {
        norminv = 0.;
        sdseterr(7,1);
    } else {
        norminv = (1./sqrt(sumsq));
    }
    pin[5][0] = (norminv*pin[5][0]);
    pin[5][1] = (norminv*pin[5][1]);
    pin[5][2] = (norminv*pin[5][2]);
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

/* Zero out Vpk and Wpk */

    for (i = 0; i < 14; i++) {
        for (j = i; j <= 13; j++) {
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
    rkWkk[0][0] = ((pin[0][2]*rk[0][1])-(pin[0][1]*rk[0][2]));
    rkWkk[0][1] = ((pin[0][0]*rk[0][2])-(pin[0][2]*rk[0][0]));
    rkWkk[0][2] = ((pin[0][1]*rk[0][0])-(pin[0][0]*rk[0][1]));
    rkWkk[1][0] = ((pin[1][2]*rk[1][1])-(pin[1][1]*rk[1][2]));
    rkWkk[1][1] = ((pin[1][0]*rk[1][2])-(pin[1][2]*rk[1][0]));
    rkWkk[1][2] = ((pin[1][1]*rk[1][0])-(pin[1][0]*rk[1][1]));
    rkWkk[2][0] = ((pin[2][2]*rk[2][1])-(pin[2][1]*rk[2][2]));
    rkWkk[2][1] = ((pin[2][0]*rk[2][2])-(pin[2][2]*rk[2][0]));
    rkWkk[2][2] = ((pin[2][1]*rk[2][0])-(pin[2][0]*rk[2][1]));
    rkWkk[3][0] = ((pin[3][2]*rk[3][1])-(pin[3][1]*rk[3][2]));
    rkWkk[3][1] = ((pin[3][0]*rk[3][2])-(pin[3][2]*rk[3][0]));
    rkWkk[3][2] = ((pin[3][1]*rk[3][0])-(pin[3][0]*rk[3][1]));
    rkWkk[4][0] = ((pin[4][2]*rk[4][1])-(pin[4][1]*rk[4][2]));
    rkWkk[4][1] = ((pin[4][0]*rk[4][2])-(pin[4][2]*rk[4][0]));
    rkWkk[4][2] = ((pin[4][1]*rk[4][0])-(pin[4][0]*rk[4][1]));
    rkWkk[5][0] = ((pin[5][2]*rk[5][1])-(pin[5][1]*rk[5][2]));
    rkWkk[5][1] = ((pin[5][0]*rk[5][2])-(pin[5][2]*rk[5][0]));
    rkWkk[5][2] = ((pin[5][1]*rk[5][0])-(pin[5][0]*rk[5][1]));
    rkWkk[6][0] = ((pin[6][2]*rk[6][1])-(pin[6][1]*rk[6][2]));
    rkWkk[6][1] = ((pin[6][0]*rk[6][2])-(pin[6][2]*rk[6][0]));
    rkWkk[6][2] = ((pin[6][1]*rk[6][0])-(pin[6][0]*rk[6][1]));
    rkWkk[7][0] = ((pin[7][2]*rk[7][1])-(pin[7][1]*rk[7][2]));
    rkWkk[7][1] = ((pin[7][0]*rk[7][2])-(pin[7][2]*rk[7][0]));
    rkWkk[7][2] = ((pin[7][1]*rk[7][0])-(pin[7][0]*rk[7][1]));
    rkWkk[8][0] = ((pin[8][2]*rk[8][1])-(pin[8][1]*rk[8][2]));
    rkWkk[8][1] = ((pin[8][0]*rk[8][2])-(pin[8][2]*rk[8][0]));
    rkWkk[8][2] = ((pin[8][1]*rk[8][0])-(pin[8][0]*rk[8][1]));
    rkWkk[9][0] = ((pin[9][2]*rk[9][1])-(pin[9][1]*rk[9][2]));
    rkWkk[9][1] = ((pin[9][0]*rk[9][2])-(pin[9][2]*rk[9][0]));
    rkWkk[9][2] = ((pin[9][1]*rk[9][0])-(pin[9][0]*rk[9][1]));
    rkWkk[10][0] = ((pin[10][2]*rk[10][1])-(pin[10][1]*rk[10][2]));
    rkWkk[10][1] = ((pin[10][0]*rk[10][2])-(pin[10][2]*rk[10][0]));
    rkWkk[10][2] = ((pin[10][1]*rk[10][0])-(pin[10][0]*rk[10][1]));
    rkWkk[11][0] = ((pin[11][2]*rk[11][1])-(pin[11][1]*rk[11][2]));
    rkWkk[11][1] = ((pin[11][0]*rk[11][2])-(pin[11][2]*rk[11][0]));
    rkWkk[11][2] = ((pin[11][1]*rk[11][0])-(pin[11][0]*rk[11][1]));
    rkWkk[12][0] = ((pin[12][2]*rk[12][1])-(pin[12][1]*rk[12][2]));
    rkWkk[12][1] = ((pin[12][0]*rk[12][2])-(pin[12][2]*rk[12][0]));
    rkWkk[12][2] = ((pin[12][1]*rk[12][0])-(pin[12][0]*rk[12][1]));
    rkWkk[13][0] = ((pin[13][2]*rk[13][1])-(pin[13][1]*rk[13][2]));
    rkWkk[13][1] = ((pin[13][0]*rk[13][2])-(pin[13][2]*rk[13][0]));
    rkWkk[13][2] = ((pin[13][1]*rk[13][0])-(pin[13][0]*rk[13][1]));
    dik[1][0] = (ri[1][0]-rk[0][0]);
    dik[1][1] = (ri[1][1]-rk[0][1]);
    dik[1][2] = (ri[1][2]-rk[0][2]);
    dik[2][0] = (ri[2][0]-rk[0][0]);
    dik[2][1] = (ri[2][1]-rk[0][1]);
    dik[2][2] = (ri[2][2]-rk[0][2]);
    dik[3][0] = (ri[3][0]-rk[2][0]);
    dik[3][1] = (ri[3][1]-rk[2][1]);
    dik[3][2] = (ri[3][2]-rk[2][2]);
    dik[4][0] = (ri[4][0]-rk[3][0]);
    dik[4][1] = (ri[4][1]-rk[3][1]);
    dik[4][2] = (ri[4][2]-rk[3][2]);
    dik[5][0] = (ri[5][0]-rk[4][0]);
    dik[5][1] = (ri[5][1]-rk[4][1]);
    dik[5][2] = (ri[5][2]-rk[4][2]);
    dik[6][0] = (ri[6][0]-rk[5][0]);
    dik[6][1] = (ri[6][1]-rk[5][1]);
    dik[6][2] = (ri[6][2]-rk[5][2]);
    dik[7][0] = (ri[7][0]-rk[6][0]);
    dik[7][1] = (ri[7][1]-rk[6][1]);
    dik[7][2] = (ri[7][2]-rk[6][2]);
    dik[8][0] = (ri[8][0]-rk[0][0]);
    dik[8][1] = (ri[8][1]-rk[0][1]);
    dik[8][2] = (ri[8][2]-rk[0][2]);
    dik[9][0] = (ri[9][0]-rk[8][0]);
    dik[9][1] = (ri[9][1]-rk[8][1]);
    dik[9][2] = (ri[9][2]-rk[8][2]);
    dik[10][0] = (ri[10][0]-rk[9][0]);
    dik[10][1] = (ri[10][1]-rk[9][1]);
    dik[10][2] = (ri[10][2]-rk[9][2]);
    dik[11][0] = (ri[11][0]-rk[10][0]);
    dik[11][1] = (ri[11][1]-rk[10][1]);
    dik[11][2] = (ri[11][2]-rk[10][2]);
    dik[12][0] = (ri[12][0]-rk[11][0]);
    dik[12][1] = (ri[12][1]-rk[11][1]);
    dik[12][2] = (ri[12][2]-rk[11][2]);
    dik[13][0] = (ri[13][0]-rk[12][0]);
    dik[13][1] = (ri[13][1]-rk[12][1]);
    dik[13][2] = (ri[13][2]-rk[12][2]);

/* Compute mass properties-related constants */

    mtot = (mk[13]+(mk[12]+(mk[11]+(mk[10]+(mk[9]+(mk[8]+(mk[7]+(mk[6]+(mk[5]+(
      mk[4]+(mk[3]+(mk[2]+(mk[0]+mk[1])))))))))))));
    mkrk[0][0][1] = -(mk[0]*rk[0][2]);
    mkrk[0][0][2] = (mk[0]*rk[0][1]);
    mkrk[0][1][0] = (mk[0]*rk[0][2]);
    mkrk[0][1][2] = -(mk[0]*rk[0][0]);
    mkrk[0][2][0] = -(mk[0]*rk[0][1]);
    mkrk[0][2][1] = (mk[0]*rk[0][0]);
    mkrk[1][0][1] = -(mk[1]*rk[1][2]);
    mkrk[1][0][2] = (mk[1]*rk[1][1]);
    mkrk[1][1][0] = (mk[1]*rk[1][2]);
    mkrk[1][1][2] = -(mk[1]*rk[1][0]);
    mkrk[1][2][0] = -(mk[1]*rk[1][1]);
    mkrk[1][2][1] = (mk[1]*rk[1][0]);
    mkrk[2][0][1] = -(mk[2]*rk[2][2]);
    mkrk[2][0][2] = (mk[2]*rk[2][1]);
    mkrk[2][1][0] = (mk[2]*rk[2][2]);
    mkrk[2][1][2] = -(mk[2]*rk[2][0]);
    mkrk[2][2][0] = -(mk[2]*rk[2][1]);
    mkrk[2][2][1] = (mk[2]*rk[2][0]);
    mkrk[3][0][1] = -(mk[3]*rk[3][2]);
    mkrk[3][0][2] = (mk[3]*rk[3][1]);
    mkrk[3][1][0] = (mk[3]*rk[3][2]);
    mkrk[3][1][2] = -(mk[3]*rk[3][0]);
    mkrk[3][2][0] = -(mk[3]*rk[3][1]);
    mkrk[3][2][1] = (mk[3]*rk[3][0]);
    mkrk[4][0][1] = -(mk[4]*rk[4][2]);
    mkrk[4][0][2] = (mk[4]*rk[4][1]);
    mkrk[4][1][0] = (mk[4]*rk[4][2]);
    mkrk[4][1][2] = -(mk[4]*rk[4][0]);
    mkrk[4][2][0] = -(mk[4]*rk[4][1]);
    mkrk[4][2][1] = (mk[4]*rk[4][0]);
    mkrk[5][0][1] = -(mk[5]*rk[5][2]);
    mkrk[5][0][2] = (mk[5]*rk[5][1]);
    mkrk[5][1][0] = (mk[5]*rk[5][2]);
    mkrk[5][1][2] = -(mk[5]*rk[5][0]);
    mkrk[5][2][0] = -(mk[5]*rk[5][1]);
    mkrk[5][2][1] = (mk[5]*rk[5][0]);
    mkrk[6][0][1] = -(mk[6]*rk[6][2]);
    mkrk[6][0][2] = (mk[6]*rk[6][1]);
    mkrk[6][1][0] = (mk[6]*rk[6][2]);
    mkrk[6][1][2] = -(mk[6]*rk[6][0]);
    mkrk[6][2][0] = -(mk[6]*rk[6][1]);
    mkrk[6][2][1] = (mk[6]*rk[6][0]);
    mkrk[7][0][1] = -(mk[7]*rk[7][2]);
    mkrk[7][0][2] = (mk[7]*rk[7][1]);
    mkrk[7][1][0] = (mk[7]*rk[7][2]);
    mkrk[7][1][2] = -(mk[7]*rk[7][0]);
    mkrk[7][2][0] = -(mk[7]*rk[7][1]);
    mkrk[7][2][1] = (mk[7]*rk[7][0]);
    mkrk[8][0][1] = -(mk[8]*rk[8][2]);
    mkrk[8][0][2] = (mk[8]*rk[8][1]);
    mkrk[8][1][0] = (mk[8]*rk[8][2]);
    mkrk[8][1][2] = -(mk[8]*rk[8][0]);
    mkrk[8][2][0] = -(mk[8]*rk[8][1]);
    mkrk[8][2][1] = (mk[8]*rk[8][0]);
    mkrk[9][0][1] = -(mk[9]*rk[9][2]);
    mkrk[9][0][2] = (mk[9]*rk[9][1]);
    mkrk[9][1][0] = (mk[9]*rk[9][2]);
    mkrk[9][1][2] = -(mk[9]*rk[9][0]);
    mkrk[9][2][0] = -(mk[9]*rk[9][1]);
    mkrk[9][2][1] = (mk[9]*rk[9][0]);
    mkrk[10][0][1] = -(mk[10]*rk[10][2]);
    mkrk[10][0][2] = (mk[10]*rk[10][1]);
    mkrk[10][1][0] = (mk[10]*rk[10][2]);
    mkrk[10][1][2] = -(mk[10]*rk[10][0]);
    mkrk[10][2][0] = -(mk[10]*rk[10][1]);
    mkrk[10][2][1] = (mk[10]*rk[10][0]);
    mkrk[11][0][1] = -(mk[11]*rk[11][2]);
    mkrk[11][0][2] = (mk[11]*rk[11][1]);
    mkrk[11][1][0] = (mk[11]*rk[11][2]);
    mkrk[11][1][2] = -(mk[11]*rk[11][0]);
    mkrk[11][2][0] = -(mk[11]*rk[11][1]);
    mkrk[11][2][1] = (mk[11]*rk[11][0]);
    mkrk[12][0][1] = -(mk[12]*rk[12][2]);
    mkrk[12][0][2] = (mk[12]*rk[12][1]);
    mkrk[12][1][0] = (mk[12]*rk[12][2]);
    mkrk[12][1][2] = -(mk[12]*rk[12][0]);
    mkrk[12][2][0] = -(mk[12]*rk[12][1]);
    mkrk[12][2][1] = (mk[12]*rk[12][0]);
    mkrk[13][0][1] = -(mk[13]*rk[13][2]);
    mkrk[13][0][2] = (mk[13]*rk[13][1]);
    mkrk[13][1][0] = (mk[13]*rk[13][2]);
    mkrk[13][1][2] = -(mk[13]*rk[13][0]);
    mkrk[13][2][0] = -(mk[13]*rk[13][1]);
    mkrk[13][2][1] = (mk[13]*rk[13][0]);
    Iko[0][0][0] = (ik[0][0][0]-((mkrk[0][0][1]*rk[0][2])-(mkrk[0][0][2]*
      rk[0][1])));
    Iko[0][0][1] = (ik[0][0][1]-(mkrk[0][0][2]*rk[0][0]));
    Iko[0][0][2] = (ik[0][0][2]+(mkrk[0][0][1]*rk[0][0]));
    Iko[0][1][0] = (ik[0][1][0]+(mkrk[0][1][2]*rk[0][1]));
    Iko[0][1][1] = (ik[0][1][1]-((mkrk[0][1][2]*rk[0][0])-(mkrk[0][1][0]*
      rk[0][2])));
    Iko[0][1][2] = (ik[0][1][2]-(mkrk[0][1][0]*rk[0][1]));
    Iko[0][2][0] = (ik[0][2][0]-(mkrk[0][2][1]*rk[0][2]));
    Iko[0][2][1] = (ik[0][2][1]+(mkrk[0][2][0]*rk[0][2]));
    Iko[0][2][2] = (ik[0][2][2]-((mkrk[0][2][0]*rk[0][1])-(mkrk[0][2][1]*
      rk[0][0])));
    Iko[1][0][0] = (ik[1][0][0]-((mkrk[1][0][1]*rk[1][2])-(mkrk[1][0][2]*
      rk[1][1])));
    Iko[1][0][1] = (ik[1][0][1]-(mkrk[1][0][2]*rk[1][0]));
    Iko[1][0][2] = (ik[1][0][2]+(mkrk[1][0][1]*rk[1][0]));
    Iko[1][1][0] = (ik[1][1][0]+(mkrk[1][1][2]*rk[1][1]));
    Iko[1][1][1] = (ik[1][1][1]-((mkrk[1][1][2]*rk[1][0])-(mkrk[1][1][0]*
      rk[1][2])));
    Iko[1][1][2] = (ik[1][1][2]-(mkrk[1][1][0]*rk[1][1]));
    Iko[1][2][0] = (ik[1][2][0]-(mkrk[1][2][1]*rk[1][2]));
    Iko[1][2][1] = (ik[1][2][1]+(mkrk[1][2][0]*rk[1][2]));
    Iko[1][2][2] = (ik[1][2][2]-((mkrk[1][2][0]*rk[1][1])-(mkrk[1][2][1]*
      rk[1][0])));
    Iko[2][0][0] = (ik[2][0][0]-((mkrk[2][0][1]*rk[2][2])-(mkrk[2][0][2]*
      rk[2][1])));
    Iko[2][0][1] = (ik[2][0][1]-(mkrk[2][0][2]*rk[2][0]));
    Iko[2][0][2] = (ik[2][0][2]+(mkrk[2][0][1]*rk[2][0]));
    Iko[2][1][0] = (ik[2][1][0]+(mkrk[2][1][2]*rk[2][1]));
    Iko[2][1][1] = (ik[2][1][1]-((mkrk[2][1][2]*rk[2][0])-(mkrk[2][1][0]*
      rk[2][2])));
    Iko[2][1][2] = (ik[2][1][2]-(mkrk[2][1][0]*rk[2][1]));
    Iko[2][2][0] = (ik[2][2][0]-(mkrk[2][2][1]*rk[2][2]));
    Iko[2][2][1] = (ik[2][2][1]+(mkrk[2][2][0]*rk[2][2]));
    Iko[2][2][2] = (ik[2][2][2]-((mkrk[2][2][0]*rk[2][1])-(mkrk[2][2][1]*
      rk[2][0])));
    Iko[3][0][0] = (ik[3][0][0]-((mkrk[3][0][1]*rk[3][2])-(mkrk[3][0][2]*
      rk[3][1])));
    Iko[3][0][1] = (ik[3][0][1]-(mkrk[3][0][2]*rk[3][0]));
    Iko[3][0][2] = (ik[3][0][2]+(mkrk[3][0][1]*rk[3][0]));
    Iko[3][1][0] = (ik[3][1][0]+(mkrk[3][1][2]*rk[3][1]));
    Iko[3][1][1] = (ik[3][1][1]-((mkrk[3][1][2]*rk[3][0])-(mkrk[3][1][0]*
      rk[3][2])));
    Iko[3][1][2] = (ik[3][1][2]-(mkrk[3][1][0]*rk[3][1]));
    Iko[3][2][0] = (ik[3][2][0]-(mkrk[3][2][1]*rk[3][2]));
    Iko[3][2][1] = (ik[3][2][1]+(mkrk[3][2][0]*rk[3][2]));
    Iko[3][2][2] = (ik[3][2][2]-((mkrk[3][2][0]*rk[3][1])-(mkrk[3][2][1]*
      rk[3][0])));
    Iko[4][0][0] = (ik[4][0][0]-((mkrk[4][0][1]*rk[4][2])-(mkrk[4][0][2]*
      rk[4][1])));
    Iko[4][0][1] = (ik[4][0][1]-(mkrk[4][0][2]*rk[4][0]));
    Iko[4][0][2] = (ik[4][0][2]+(mkrk[4][0][1]*rk[4][0]));
    Iko[4][1][0] = (ik[4][1][0]+(mkrk[4][1][2]*rk[4][1]));
    Iko[4][1][1] = (ik[4][1][1]-((mkrk[4][1][2]*rk[4][0])-(mkrk[4][1][0]*
      rk[4][2])));
    Iko[4][1][2] = (ik[4][1][2]-(mkrk[4][1][0]*rk[4][1]));
    Iko[4][2][0] = (ik[4][2][0]-(mkrk[4][2][1]*rk[4][2]));
    Iko[4][2][1] = (ik[4][2][1]+(mkrk[4][2][0]*rk[4][2]));
    Iko[4][2][2] = (ik[4][2][2]-((mkrk[4][2][0]*rk[4][1])-(mkrk[4][2][1]*
      rk[4][0])));
    Iko[5][0][0] = (ik[5][0][0]-((mkrk[5][0][1]*rk[5][2])-(mkrk[5][0][2]*
      rk[5][1])));
    Iko[5][0][1] = (ik[5][0][1]-(mkrk[5][0][2]*rk[5][0]));
    Iko[5][0][2] = (ik[5][0][2]+(mkrk[5][0][1]*rk[5][0]));
    Iko[5][1][0] = (ik[5][1][0]+(mkrk[5][1][2]*rk[5][1]));
    Iko[5][1][1] = (ik[5][1][1]-((mkrk[5][1][2]*rk[5][0])-(mkrk[5][1][0]*
      rk[5][2])));
    Iko[5][1][2] = (ik[5][1][2]-(mkrk[5][1][0]*rk[5][1]));
    Iko[5][2][0] = (ik[5][2][0]-(mkrk[5][2][1]*rk[5][2]));
    Iko[5][2][1] = (ik[5][2][1]+(mkrk[5][2][0]*rk[5][2]));
    Iko[5][2][2] = (ik[5][2][2]-((mkrk[5][2][0]*rk[5][1])-(mkrk[5][2][1]*
      rk[5][0])));
    Iko[6][0][0] = (ik[6][0][0]-((mkrk[6][0][1]*rk[6][2])-(mkrk[6][0][2]*
      rk[6][1])));
    Iko[6][0][1] = (ik[6][0][1]-(mkrk[6][0][2]*rk[6][0]));
    Iko[6][0][2] = (ik[6][0][2]+(mkrk[6][0][1]*rk[6][0]));
    Iko[6][1][0] = (ik[6][1][0]+(mkrk[6][1][2]*rk[6][1]));
    Iko[6][1][1] = (ik[6][1][1]-((mkrk[6][1][2]*rk[6][0])-(mkrk[6][1][0]*
      rk[6][2])));
    Iko[6][1][2] = (ik[6][1][2]-(mkrk[6][1][0]*rk[6][1]));
    Iko[6][2][0] = (ik[6][2][0]-(mkrk[6][2][1]*rk[6][2]));
    Iko[6][2][1] = (ik[6][2][1]+(mkrk[6][2][0]*rk[6][2]));
    Iko[6][2][2] = (ik[6][2][2]-((mkrk[6][2][0]*rk[6][1])-(mkrk[6][2][1]*
      rk[6][0])));
    Iko[7][0][0] = (ik[7][0][0]-((mkrk[7][0][1]*rk[7][2])-(mkrk[7][0][2]*
      rk[7][1])));
    Iko[7][0][1] = (ik[7][0][1]-(mkrk[7][0][2]*rk[7][0]));
    Iko[7][0][2] = (ik[7][0][2]+(mkrk[7][0][1]*rk[7][0]));
    Iko[7][1][0] = (ik[7][1][0]+(mkrk[7][1][2]*rk[7][1]));
    Iko[7][1][1] = (ik[7][1][1]-((mkrk[7][1][2]*rk[7][0])-(mkrk[7][1][0]*
      rk[7][2])));
    Iko[7][1][2] = (ik[7][1][2]-(mkrk[7][1][0]*rk[7][1]));
    Iko[7][2][0] = (ik[7][2][0]-(mkrk[7][2][1]*rk[7][2]));
    Iko[7][2][1] = (ik[7][2][1]+(mkrk[7][2][0]*rk[7][2]));
    Iko[7][2][2] = (ik[7][2][2]-((mkrk[7][2][0]*rk[7][1])-(mkrk[7][2][1]*
      rk[7][0])));
    Iko[8][0][0] = (ik[8][0][0]-((mkrk[8][0][1]*rk[8][2])-(mkrk[8][0][2]*
      rk[8][1])));
    Iko[8][0][1] = (ik[8][0][1]-(mkrk[8][0][2]*rk[8][0]));
    Iko[8][0][2] = (ik[8][0][2]+(mkrk[8][0][1]*rk[8][0]));
    Iko[8][1][0] = (ik[8][1][0]+(mkrk[8][1][2]*rk[8][1]));
    Iko[8][1][1] = (ik[8][1][1]-((mkrk[8][1][2]*rk[8][0])-(mkrk[8][1][0]*
      rk[8][2])));
    Iko[8][1][2] = (ik[8][1][2]-(mkrk[8][1][0]*rk[8][1]));
    Iko[8][2][0] = (ik[8][2][0]-(mkrk[8][2][1]*rk[8][2]));
    Iko[8][2][1] = (ik[8][2][1]+(mkrk[8][2][0]*rk[8][2]));
    Iko[8][2][2] = (ik[8][2][2]-((mkrk[8][2][0]*rk[8][1])-(mkrk[8][2][1]*
      rk[8][0])));
    Iko[9][0][0] = (ik[9][0][0]-((mkrk[9][0][1]*rk[9][2])-(mkrk[9][0][2]*
      rk[9][1])));
    Iko[9][0][1] = (ik[9][0][1]-(mkrk[9][0][2]*rk[9][0]));
    Iko[9][0][2] = (ik[9][0][2]+(mkrk[9][0][1]*rk[9][0]));
    Iko[9][1][0] = (ik[9][1][0]+(mkrk[9][1][2]*rk[9][1]));
    Iko[9][1][1] = (ik[9][1][1]-((mkrk[9][1][2]*rk[9][0])-(mkrk[9][1][0]*
      rk[9][2])));
    Iko[9][1][2] = (ik[9][1][2]-(mkrk[9][1][0]*rk[9][1]));
    Iko[9][2][0] = (ik[9][2][0]-(mkrk[9][2][1]*rk[9][2]));
    Iko[9][2][1] = (ik[9][2][1]+(mkrk[9][2][0]*rk[9][2]));
    Iko[9][2][2] = (ik[9][2][2]-((mkrk[9][2][0]*rk[9][1])-(mkrk[9][2][1]*
      rk[9][0])));
    Iko[10][0][0] = (ik[10][0][0]-((mkrk[10][0][1]*rk[10][2])-(mkrk[10][0][2]*
      rk[10][1])));
    Iko[10][0][1] = (ik[10][0][1]-(mkrk[10][0][2]*rk[10][0]));
    Iko[10][0][2] = (ik[10][0][2]+(mkrk[10][0][1]*rk[10][0]));
    Iko[10][1][0] = (ik[10][1][0]+(mkrk[10][1][2]*rk[10][1]));
    Iko[10][1][1] = (ik[10][1][1]-((mkrk[10][1][2]*rk[10][0])-(mkrk[10][1][0]*
      rk[10][2])));
    Iko[10][1][2] = (ik[10][1][2]-(mkrk[10][1][0]*rk[10][1]));
    Iko[10][2][0] = (ik[10][2][0]-(mkrk[10][2][1]*rk[10][2]));
    Iko[10][2][1] = (ik[10][2][1]+(mkrk[10][2][0]*rk[10][2]));
    Iko[10][2][2] = (ik[10][2][2]-((mkrk[10][2][0]*rk[10][1])-(mkrk[10][2][1]*
      rk[10][0])));
    Iko[11][0][0] = (ik[11][0][0]-((mkrk[11][0][1]*rk[11][2])-(mkrk[11][0][2]*
      rk[11][1])));
    Iko[11][0][1] = (ik[11][0][1]-(mkrk[11][0][2]*rk[11][0]));
    Iko[11][0][2] = (ik[11][0][2]+(mkrk[11][0][1]*rk[11][0]));
    Iko[11][1][0] = (ik[11][1][0]+(mkrk[11][1][2]*rk[11][1]));
    Iko[11][1][1] = (ik[11][1][1]-((mkrk[11][1][2]*rk[11][0])-(mkrk[11][1][0]*
      rk[11][2])));
    Iko[11][1][2] = (ik[11][1][2]-(mkrk[11][1][0]*rk[11][1]));
    Iko[11][2][0] = (ik[11][2][0]-(mkrk[11][2][1]*rk[11][2]));
    Iko[11][2][1] = (ik[11][2][1]+(mkrk[11][2][0]*rk[11][2]));
    Iko[11][2][2] = (ik[11][2][2]-((mkrk[11][2][0]*rk[11][1])-(mkrk[11][2][1]*
      rk[11][0])));
    Iko[12][0][0] = (ik[12][0][0]-((mkrk[12][0][1]*rk[12][2])-(mkrk[12][0][2]*
      rk[12][1])));
    Iko[12][0][1] = (ik[12][0][1]-(mkrk[12][0][2]*rk[12][0]));
    Iko[12][0][2] = (ik[12][0][2]+(mkrk[12][0][1]*rk[12][0]));
    Iko[12][1][0] = (ik[12][1][0]+(mkrk[12][1][2]*rk[12][1]));
    Iko[12][1][1] = (ik[12][1][1]-((mkrk[12][1][2]*rk[12][0])-(mkrk[12][1][0]*
      rk[12][2])));
    Iko[12][1][2] = (ik[12][1][2]-(mkrk[12][1][0]*rk[12][1]));
    Iko[12][2][0] = (ik[12][2][0]-(mkrk[12][2][1]*rk[12][2]));
    Iko[12][2][1] = (ik[12][2][1]+(mkrk[12][2][0]*rk[12][2]));
    Iko[12][2][2] = (ik[12][2][2]-((mkrk[12][2][0]*rk[12][1])-(mkrk[12][2][1]*
      rk[12][0])));
    Iko[13][0][0] = (ik[13][0][0]-((mkrk[13][0][1]*rk[13][2])-(mkrk[13][0][2]*
      rk[13][1])));
    Iko[13][0][1] = (ik[13][0][1]-(mkrk[13][0][2]*rk[13][0]));
    Iko[13][0][2] = (ik[13][0][2]+(mkrk[13][0][1]*rk[13][0]));
    Iko[13][1][0] = (ik[13][1][0]+(mkrk[13][1][2]*rk[13][1]));
    Iko[13][1][1] = (ik[13][1][1]-((mkrk[13][1][2]*rk[13][0])-(mkrk[13][1][0]*
      rk[13][2])));
    Iko[13][1][2] = (ik[13][1][2]-(mkrk[13][1][0]*rk[13][1]));
    Iko[13][2][0] = (ik[13][2][0]-(mkrk[13][2][1]*rk[13][2]));
    Iko[13][2][1] = (ik[13][2][1]+(mkrk[13][2][0]*rk[13][2]));
    Iko[13][2][2] = (ik[13][2][2]-((mkrk[13][2][0]*rk[13][1])-(mkrk[13][2][1]*
      rk[13][0])));
    sdserialno(&i);
    if (i != 30123) {
        sdseterr(7,41);
    }
    roustate = 1;
}

/* Convert state to form using 1-2-3 Euler angles for ball joints. */

void sdst2ang(double st[14],
    double stang[14])
{
    int i;

    for (i = 0; i < 14; i++) {
        stang[i] = st[i];
    }
}

/* Convert 1-2-3 form of state back to Euler parameters for ball joints. */

void sdang2st(double stang[14],
    double st[14])
{
    int i;

    for (i = 0; i < 14; i++) {
        st[i] = stang[i];
    }
}

/* Normalize Euler parameters in state. */

void sdnrmsterr(double st[14],
    double normst[14],
    int routine)
{
    int i;

    for (i = 0; i < 14; i++) {
        normst[i] = st[i];
    }
}

void sdnormst(double st[14],
    double normst[14])
{

    sdnrmsterr(st,normst,0);
}

void sdstate(double timein,
    double qin[14],
    double uin[14])
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

    if ((roustate != 1) && (roustate != 2) && (roustate != 3)) {
        sdseterr(8,22);
        return;
    }
    if (roustate == 1) {
        for (i = 0; i < 14; i++) {
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
        for (i = 0; i < 14; i++) {
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
        for (i = 0; i < 14; i++) {
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
    for (i = 0; i < 14; i++) {
        q[i] = qin[i];
    }
/*
Compute sines and cosines of q
*/
    s0 = sin(q[0]);
    c0 = cos(q[0]);
    s1 = sin(q[1]);
    c1 = cos(q[1]);
    s2 = sin(q[2]);
    c2 = cos(q[2]);
    s3 = sin(q[3]);
    c3 = cos(q[3]);
    s4 = sin(q[4]);
    c4 = cos(q[4]);
    s5 = sin(q[5]);
    c5 = cos(q[5]);
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
/*
Compute across-axis direction cosines Cik
*/
    Cik[0][0][0] = ((pin[0][0]*pin[0][0])+(c0*(1.-(pin[0][0]*pin[0][0]))));
    Cik[0][0][1] = (((pin[0][0]*pin[0][1])-(pin[0][2]*s0))-(c0*(pin[0][0]*
      pin[0][1])));
    Cik[0][0][2] = (((pin[0][0]*pin[0][2])+(pin[0][1]*s0))-(c0*(pin[0][0]*
      pin[0][2])));
    Cik[0][1][0] = (((pin[0][0]*pin[0][1])+(pin[0][2]*s0))-(c0*(pin[0][0]*
      pin[0][1])));
    Cik[0][1][1] = ((pin[0][1]*pin[0][1])+(c0*(1.-(pin[0][1]*pin[0][1]))));
    Cik[0][1][2] = (((pin[0][1]*pin[0][2])-(pin[0][0]*s0))-(c0*(pin[0][1]*
      pin[0][2])));
    Cik[0][2][0] = (((pin[0][0]*pin[0][2])-(pin[0][1]*s0))-(c0*(pin[0][0]*
      pin[0][2])));
    Cik[0][2][1] = (((pin[0][0]*s0)+(pin[0][1]*pin[0][2]))-(c0*(pin[0][1]*
      pin[0][2])));
    Cik[0][2][2] = ((pin[0][2]*pin[0][2])+(c0*(1.-(pin[0][2]*pin[0][2]))));
    Cik[1][0][0] = ((pin[1][0]*pin[1][0])+(c1*(1.-(pin[1][0]*pin[1][0]))));
    Cik[1][0][1] = (((pin[1][0]*pin[1][1])-(pin[1][2]*s1))-(c1*(pin[1][0]*
      pin[1][1])));
    Cik[1][0][2] = (((pin[1][0]*pin[1][2])+(pin[1][1]*s1))-(c1*(pin[1][0]*
      pin[1][2])));
    Cik[1][1][0] = (((pin[1][0]*pin[1][1])+(pin[1][2]*s1))-(c1*(pin[1][0]*
      pin[1][1])));
    Cik[1][1][1] = ((pin[1][1]*pin[1][1])+(c1*(1.-(pin[1][1]*pin[1][1]))));
    Cik[1][1][2] = (((pin[1][1]*pin[1][2])-(pin[1][0]*s1))-(c1*(pin[1][1]*
      pin[1][2])));
    Cik[1][2][0] = (((pin[1][0]*pin[1][2])-(pin[1][1]*s1))-(c1*(pin[1][0]*
      pin[1][2])));
    Cik[1][2][1] = (((pin[1][0]*s1)+(pin[1][1]*pin[1][2]))-(c1*(pin[1][1]*
      pin[1][2])));
    Cik[1][2][2] = ((pin[1][2]*pin[1][2])+(c1*(1.-(pin[1][2]*pin[1][2]))));
    Cik[2][0][0] = ((pin[2][0]*pin[2][0])+(c2*(1.-(pin[2][0]*pin[2][0]))));
    Cik[2][0][1] = (((pin[2][0]*pin[2][1])-(pin[2][2]*s2))-(c2*(pin[2][0]*
      pin[2][1])));
    Cik[2][0][2] = (((pin[2][0]*pin[2][2])+(pin[2][1]*s2))-(c2*(pin[2][0]*
      pin[2][2])));
    Cik[2][1][0] = (((pin[2][0]*pin[2][1])+(pin[2][2]*s2))-(c2*(pin[2][0]*
      pin[2][1])));
    Cik[2][1][1] = ((pin[2][1]*pin[2][1])+(c2*(1.-(pin[2][1]*pin[2][1]))));
    Cik[2][1][2] = (((pin[2][1]*pin[2][2])-(pin[2][0]*s2))-(c2*(pin[2][1]*
      pin[2][2])));
    Cik[2][2][0] = (((pin[2][0]*pin[2][2])-(pin[2][1]*s2))-(c2*(pin[2][0]*
      pin[2][2])));
    Cik[2][2][1] = (((pin[2][0]*s2)+(pin[2][1]*pin[2][2]))-(c2*(pin[2][1]*
      pin[2][2])));
    Cik[2][2][2] = ((pin[2][2]*pin[2][2])+(c2*(1.-(pin[2][2]*pin[2][2]))));
    Cik[3][0][0] = ((pin[3][0]*pin[3][0])+(c3*(1.-(pin[3][0]*pin[3][0]))));
    Cik[3][0][1] = (((pin[3][0]*pin[3][1])-(pin[3][2]*s3))-(c3*(pin[3][0]*
      pin[3][1])));
    Cik[3][0][2] = (((pin[3][0]*pin[3][2])+(pin[3][1]*s3))-(c3*(pin[3][0]*
      pin[3][2])));
    Cik[3][1][0] = (((pin[3][0]*pin[3][1])+(pin[3][2]*s3))-(c3*(pin[3][0]*
      pin[3][1])));
    Cik[3][1][1] = ((pin[3][1]*pin[3][1])+(c3*(1.-(pin[3][1]*pin[3][1]))));
    Cik[3][1][2] = (((pin[3][1]*pin[3][2])-(pin[3][0]*s3))-(c3*(pin[3][1]*
      pin[3][2])));
    Cik[3][2][0] = (((pin[3][0]*pin[3][2])-(pin[3][1]*s3))-(c3*(pin[3][0]*
      pin[3][2])));
    Cik[3][2][1] = (((pin[3][0]*s3)+(pin[3][1]*pin[3][2]))-(c3*(pin[3][1]*
      pin[3][2])));
    Cik[3][2][2] = ((pin[3][2]*pin[3][2])+(c3*(1.-(pin[3][2]*pin[3][2]))));
    Cik[4][0][0] = ((pin[4][0]*pin[4][0])+(c4*(1.-(pin[4][0]*pin[4][0]))));
    Cik[4][0][1] = (((pin[4][0]*pin[4][1])-(pin[4][2]*s4))-(c4*(pin[4][0]*
      pin[4][1])));
    Cik[4][0][2] = (((pin[4][0]*pin[4][2])+(pin[4][1]*s4))-(c4*(pin[4][0]*
      pin[4][2])));
    Cik[4][1][0] = (((pin[4][0]*pin[4][1])+(pin[4][2]*s4))-(c4*(pin[4][0]*
      pin[4][1])));
    Cik[4][1][1] = ((pin[4][1]*pin[4][1])+(c4*(1.-(pin[4][1]*pin[4][1]))));
    Cik[4][1][2] = (((pin[4][1]*pin[4][2])-(pin[4][0]*s4))-(c4*(pin[4][1]*
      pin[4][2])));
    Cik[4][2][0] = (((pin[4][0]*pin[4][2])-(pin[4][1]*s4))-(c4*(pin[4][0]*
      pin[4][2])));
    Cik[4][2][1] = (((pin[4][0]*s4)+(pin[4][1]*pin[4][2]))-(c4*(pin[4][1]*
      pin[4][2])));
    Cik[4][2][2] = ((pin[4][2]*pin[4][2])+(c4*(1.-(pin[4][2]*pin[4][2]))));
    Cik[5][0][0] = ((pin[5][0]*pin[5][0])+(c5*(1.-(pin[5][0]*pin[5][0]))));
    Cik[5][0][1] = (((pin[5][0]*pin[5][1])-(pin[5][2]*s5))-(c5*(pin[5][0]*
      pin[5][1])));
    Cik[5][0][2] = (((pin[5][0]*pin[5][2])+(pin[5][1]*s5))-(c5*(pin[5][0]*
      pin[5][2])));
    Cik[5][1][0] = (((pin[5][0]*pin[5][1])+(pin[5][2]*s5))-(c5*(pin[5][0]*
      pin[5][1])));
    Cik[5][1][1] = ((pin[5][1]*pin[5][1])+(c5*(1.-(pin[5][1]*pin[5][1]))));
    Cik[5][1][2] = (((pin[5][1]*pin[5][2])-(pin[5][0]*s5))-(c5*(pin[5][1]*
      pin[5][2])));
    Cik[5][2][0] = (((pin[5][0]*pin[5][2])-(pin[5][1]*s5))-(c5*(pin[5][0]*
      pin[5][2])));
    Cik[5][2][1] = (((pin[5][0]*s5)+(pin[5][1]*pin[5][2]))-(c5*(pin[5][1]*
      pin[5][2])));
    Cik[5][2][2] = ((pin[5][2]*pin[5][2])+(c5*(1.-(pin[5][2]*pin[5][2]))));
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
/*
Compute across-joint direction cosines Cib
*/
/*
Compute gravity
*/
    gk[0][0] = ((Cik[0][2][0]*grav[2])+((Cik[0][0][0]*grav[0])+(Cik[0][1][0]*
      grav[1])));
    gk[0][1] = ((Cik[0][2][1]*grav[2])+((Cik[0][0][1]*grav[0])+(Cik[0][1][1]*
      grav[1])));
    gk[0][2] = ((Cik[0][2][2]*grav[2])+((Cik[0][0][2]*grav[0])+(Cik[0][1][2]*
      grav[1])));
    gk[1][0] = ((Cik[1][2][0]*gk[0][2])+((Cik[1][0][0]*gk[0][0])+(Cik[1][1][0]*
      gk[0][1])));
    gk[1][1] = ((Cik[1][2][1]*gk[0][2])+((Cik[1][0][1]*gk[0][0])+(Cik[1][1][1]*
      gk[0][1])));
    gk[1][2] = ((Cik[1][2][2]*gk[0][2])+((Cik[1][0][2]*gk[0][0])+(Cik[1][1][2]*
      gk[0][1])));
    gk[2][0] = ((Cik[2][2][0]*gk[0][2])+((Cik[2][0][0]*gk[0][0])+(Cik[2][1][0]*
      gk[0][1])));
    gk[2][1] = ((Cik[2][2][1]*gk[0][2])+((Cik[2][0][1]*gk[0][0])+(Cik[2][1][1]*
      gk[0][1])));
    gk[2][2] = ((Cik[2][2][2]*gk[0][2])+((Cik[2][0][2]*gk[0][0])+(Cik[2][1][2]*
      gk[0][1])));
    gk[3][0] = ((Cik[3][2][0]*gk[2][2])+((Cik[3][0][0]*gk[2][0])+(Cik[3][1][0]*
      gk[2][1])));
    gk[3][1] = ((Cik[3][2][1]*gk[2][2])+((Cik[3][0][1]*gk[2][0])+(Cik[3][1][1]*
      gk[2][1])));
    gk[3][2] = ((Cik[3][2][2]*gk[2][2])+((Cik[3][0][2]*gk[2][0])+(Cik[3][1][2]*
      gk[2][1])));
    gk[4][0] = ((Cik[4][2][0]*gk[3][2])+((Cik[4][0][0]*gk[3][0])+(Cik[4][1][0]*
      gk[3][1])));
    gk[4][1] = ((Cik[4][2][1]*gk[3][2])+((Cik[4][0][1]*gk[3][0])+(Cik[4][1][1]*
      gk[3][1])));
    gk[4][2] = ((Cik[4][2][2]*gk[3][2])+((Cik[4][0][2]*gk[3][0])+(Cik[4][1][2]*
      gk[3][1])));
    gk[5][0] = ((Cik[5][2][0]*gk[4][2])+((Cik[5][0][0]*gk[4][0])+(Cik[5][1][0]*
      gk[4][1])));
    gk[5][1] = ((Cik[5][2][1]*gk[4][2])+((Cik[5][0][1]*gk[4][0])+(Cik[5][1][1]*
      gk[4][1])));
    gk[5][2] = ((Cik[5][2][2]*gk[4][2])+((Cik[5][0][2]*gk[4][0])+(Cik[5][1][2]*
      gk[4][1])));
    gk[6][0] = ((Cik[6][2][0]*gk[5][2])+((Cik[6][0][0]*gk[5][0])+(Cik[6][1][0]*
      gk[5][1])));
    gk[6][1] = ((Cik[6][2][1]*gk[5][2])+((Cik[6][0][1]*gk[5][0])+(Cik[6][1][1]*
      gk[5][1])));
    gk[6][2] = ((Cik[6][2][2]*gk[5][2])+((Cik[6][0][2]*gk[5][0])+(Cik[6][1][2]*
      gk[5][1])));
    gk[7][0] = ((Cik[7][2][0]*gk[6][2])+((Cik[7][0][0]*gk[6][0])+(Cik[7][1][0]*
      gk[6][1])));
    gk[7][1] = ((Cik[7][2][1]*gk[6][2])+((Cik[7][0][1]*gk[6][0])+(Cik[7][1][1]*
      gk[6][1])));
    gk[7][2] = ((Cik[7][2][2]*gk[6][2])+((Cik[7][0][2]*gk[6][0])+(Cik[7][1][2]*
      gk[6][1])));
    gk[8][0] = ((Cik[8][2][0]*gk[0][2])+((Cik[8][0][0]*gk[0][0])+(Cik[8][1][0]*
      gk[0][1])));
    gk[8][1] = ((Cik[8][2][1]*gk[0][2])+((Cik[8][0][1]*gk[0][0])+(Cik[8][1][1]*
      gk[0][1])));
    gk[8][2] = ((Cik[8][2][2]*gk[0][2])+((Cik[8][0][2]*gk[0][0])+(Cik[8][1][2]*
      gk[0][1])));
    gk[9][0] = ((Cik[9][2][0]*gk[8][2])+((Cik[9][0][0]*gk[8][0])+(Cik[9][1][0]*
      gk[8][1])));
    gk[9][1] = ((Cik[9][2][1]*gk[8][2])+((Cik[9][0][1]*gk[8][0])+(Cik[9][1][1]*
      gk[8][1])));
    gk[9][2] = ((Cik[9][2][2]*gk[8][2])+((Cik[9][0][2]*gk[8][0])+(Cik[9][1][2]*
      gk[8][1])));
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
/*
Compute cnk & cnb (direction cosines in N)
*/
    cnk[1][0][0] = ((Cik[0][0][2]*Cik[1][2][0])+((Cik[0][0][0]*Cik[1][0][0])+(
      Cik[0][0][1]*Cik[1][1][0])));
    cnk[1][0][1] = ((Cik[0][0][2]*Cik[1][2][1])+((Cik[0][0][0]*Cik[1][0][1])+(
      Cik[0][0][1]*Cik[1][1][1])));
    cnk[1][0][2] = ((Cik[0][0][2]*Cik[1][2][2])+((Cik[0][0][0]*Cik[1][0][2])+(
      Cik[0][0][1]*Cik[1][1][2])));
    cnk[1][1][0] = ((Cik[0][1][2]*Cik[1][2][0])+((Cik[0][1][0]*Cik[1][0][0])+(
      Cik[0][1][1]*Cik[1][1][0])));
    cnk[1][1][1] = ((Cik[0][1][2]*Cik[1][2][1])+((Cik[0][1][0]*Cik[1][0][1])+(
      Cik[0][1][1]*Cik[1][1][1])));
    cnk[1][1][2] = ((Cik[0][1][2]*Cik[1][2][2])+((Cik[0][1][0]*Cik[1][0][2])+(
      Cik[0][1][1]*Cik[1][1][2])));
    cnk[1][2][0] = ((Cik[0][2][2]*Cik[1][2][0])+((Cik[0][2][0]*Cik[1][0][0])+(
      Cik[0][2][1]*Cik[1][1][0])));
    cnk[1][2][1] = ((Cik[0][2][2]*Cik[1][2][1])+((Cik[0][2][0]*Cik[1][0][1])+(
      Cik[0][2][1]*Cik[1][1][1])));
    cnk[1][2][2] = ((Cik[0][2][2]*Cik[1][2][2])+((Cik[0][2][0]*Cik[1][0][2])+(
      Cik[0][2][1]*Cik[1][1][2])));
    cnk[2][0][0] = ((Cik[0][0][2]*Cik[2][2][0])+((Cik[0][0][0]*Cik[2][0][0])+(
      Cik[0][0][1]*Cik[2][1][0])));
    cnk[2][0][1] = ((Cik[0][0][2]*Cik[2][2][1])+((Cik[0][0][0]*Cik[2][0][1])+(
      Cik[0][0][1]*Cik[2][1][1])));
    cnk[2][0][2] = ((Cik[0][0][2]*Cik[2][2][2])+((Cik[0][0][0]*Cik[2][0][2])+(
      Cik[0][0][1]*Cik[2][1][2])));
    cnk[2][1][0] = ((Cik[0][1][2]*Cik[2][2][0])+((Cik[0][1][0]*Cik[2][0][0])+(
      Cik[0][1][1]*Cik[2][1][0])));
    cnk[2][1][1] = ((Cik[0][1][2]*Cik[2][2][1])+((Cik[0][1][0]*Cik[2][0][1])+(
      Cik[0][1][1]*Cik[2][1][1])));
    cnk[2][1][2] = ((Cik[0][1][2]*Cik[2][2][2])+((Cik[0][1][0]*Cik[2][0][2])+(
      Cik[0][1][1]*Cik[2][1][2])));
    cnk[2][2][0] = ((Cik[0][2][2]*Cik[2][2][0])+((Cik[0][2][0]*Cik[2][0][0])+(
      Cik[0][2][1]*Cik[2][1][0])));
    cnk[2][2][1] = ((Cik[0][2][2]*Cik[2][2][1])+((Cik[0][2][0]*Cik[2][0][1])+(
      Cik[0][2][1]*Cik[2][1][1])));
    cnk[2][2][2] = ((Cik[0][2][2]*Cik[2][2][2])+((Cik[0][2][0]*Cik[2][0][2])+(
      Cik[0][2][1]*Cik[2][1][2])));
    cnk[3][0][0] = ((Cik[3][2][0]*cnk[2][0][2])+((Cik[3][0][0]*cnk[2][0][0])+(
      Cik[3][1][0]*cnk[2][0][1])));
    cnk[3][0][1] = ((Cik[3][2][1]*cnk[2][0][2])+((Cik[3][0][1]*cnk[2][0][0])+(
      Cik[3][1][1]*cnk[2][0][1])));
    cnk[3][0][2] = ((Cik[3][2][2]*cnk[2][0][2])+((Cik[3][0][2]*cnk[2][0][0])+(
      Cik[3][1][2]*cnk[2][0][1])));
    cnk[3][1][0] = ((Cik[3][2][0]*cnk[2][1][2])+((Cik[3][0][0]*cnk[2][1][0])+(
      Cik[3][1][0]*cnk[2][1][1])));
    cnk[3][1][1] = ((Cik[3][2][1]*cnk[2][1][2])+((Cik[3][0][1]*cnk[2][1][0])+(
      Cik[3][1][1]*cnk[2][1][1])));
    cnk[3][1][2] = ((Cik[3][2][2]*cnk[2][1][2])+((Cik[3][0][2]*cnk[2][1][0])+(
      Cik[3][1][2]*cnk[2][1][1])));
    cnk[3][2][0] = ((Cik[3][2][0]*cnk[2][2][2])+((Cik[3][0][0]*cnk[2][2][0])+(
      Cik[3][1][0]*cnk[2][2][1])));
    cnk[3][2][1] = ((Cik[3][2][1]*cnk[2][2][2])+((Cik[3][0][1]*cnk[2][2][0])+(
      Cik[3][1][1]*cnk[2][2][1])));
    cnk[3][2][2] = ((Cik[3][2][2]*cnk[2][2][2])+((Cik[3][0][2]*cnk[2][2][0])+(
      Cik[3][1][2]*cnk[2][2][1])));
    cnk[4][0][0] = ((Cik[4][2][0]*cnk[3][0][2])+((Cik[4][0][0]*cnk[3][0][0])+(
      Cik[4][1][0]*cnk[3][0][1])));
    cnk[4][0][1] = ((Cik[4][2][1]*cnk[3][0][2])+((Cik[4][0][1]*cnk[3][0][0])+(
      Cik[4][1][1]*cnk[3][0][1])));
    cnk[4][0][2] = ((Cik[4][2][2]*cnk[3][0][2])+((Cik[4][0][2]*cnk[3][0][0])+(
      Cik[4][1][2]*cnk[3][0][1])));
    cnk[4][1][0] = ((Cik[4][2][0]*cnk[3][1][2])+((Cik[4][0][0]*cnk[3][1][0])+(
      Cik[4][1][0]*cnk[3][1][1])));
    cnk[4][1][1] = ((Cik[4][2][1]*cnk[3][1][2])+((Cik[4][0][1]*cnk[3][1][0])+(
      Cik[4][1][1]*cnk[3][1][1])));
    cnk[4][1][2] = ((Cik[4][2][2]*cnk[3][1][2])+((Cik[4][0][2]*cnk[3][1][0])+(
      Cik[4][1][2]*cnk[3][1][1])));
    cnk[4][2][0] = ((Cik[4][2][0]*cnk[3][2][2])+((Cik[4][0][0]*cnk[3][2][0])+(
      Cik[4][1][0]*cnk[3][2][1])));
    cnk[4][2][1] = ((Cik[4][2][1]*cnk[3][2][2])+((Cik[4][0][1]*cnk[3][2][0])+(
      Cik[4][1][1]*cnk[3][2][1])));
    cnk[4][2][2] = ((Cik[4][2][2]*cnk[3][2][2])+((Cik[4][0][2]*cnk[3][2][0])+(
      Cik[4][1][2]*cnk[3][2][1])));
    cnk[5][0][0] = ((Cik[5][2][0]*cnk[4][0][2])+((Cik[5][0][0]*cnk[4][0][0])+(
      Cik[5][1][0]*cnk[4][0][1])));
    cnk[5][0][1] = ((Cik[5][2][1]*cnk[4][0][2])+((Cik[5][0][1]*cnk[4][0][0])+(
      Cik[5][1][1]*cnk[4][0][1])));
    cnk[5][0][2] = ((Cik[5][2][2]*cnk[4][0][2])+((Cik[5][0][2]*cnk[4][0][0])+(
      Cik[5][1][2]*cnk[4][0][1])));
    cnk[5][1][0] = ((Cik[5][2][0]*cnk[4][1][2])+((Cik[5][0][0]*cnk[4][1][0])+(
      Cik[5][1][0]*cnk[4][1][1])));
    cnk[5][1][1] = ((Cik[5][2][1]*cnk[4][1][2])+((Cik[5][0][1]*cnk[4][1][0])+(
      Cik[5][1][1]*cnk[4][1][1])));
    cnk[5][1][2] = ((Cik[5][2][2]*cnk[4][1][2])+((Cik[5][0][2]*cnk[4][1][0])+(
      Cik[5][1][2]*cnk[4][1][1])));
    cnk[5][2][0] = ((Cik[5][2][0]*cnk[4][2][2])+((Cik[5][0][0]*cnk[4][2][0])+(
      Cik[5][1][0]*cnk[4][2][1])));
    cnk[5][2][1] = ((Cik[5][2][1]*cnk[4][2][2])+((Cik[5][0][1]*cnk[4][2][0])+(
      Cik[5][1][1]*cnk[4][2][1])));
    cnk[5][2][2] = ((Cik[5][2][2]*cnk[4][2][2])+((Cik[5][0][2]*cnk[4][2][0])+(
      Cik[5][1][2]*cnk[4][2][1])));
    cnk[6][0][0] = ((Cik[6][2][0]*cnk[5][0][2])+((Cik[6][0][0]*cnk[5][0][0])+(
      Cik[6][1][0]*cnk[5][0][1])));
    cnk[6][0][1] = ((Cik[6][2][1]*cnk[5][0][2])+((Cik[6][0][1]*cnk[5][0][0])+(
      Cik[6][1][1]*cnk[5][0][1])));
    cnk[6][0][2] = ((Cik[6][2][2]*cnk[5][0][2])+((Cik[6][0][2]*cnk[5][0][0])+(
      Cik[6][1][2]*cnk[5][0][1])));
    cnk[6][1][0] = ((Cik[6][2][0]*cnk[5][1][2])+((Cik[6][0][0]*cnk[5][1][0])+(
      Cik[6][1][0]*cnk[5][1][1])));
    cnk[6][1][1] = ((Cik[6][2][1]*cnk[5][1][2])+((Cik[6][0][1]*cnk[5][1][0])+(
      Cik[6][1][1]*cnk[5][1][1])));
    cnk[6][1][2] = ((Cik[6][2][2]*cnk[5][1][2])+((Cik[6][0][2]*cnk[5][1][0])+(
      Cik[6][1][2]*cnk[5][1][1])));
    cnk[6][2][0] = ((Cik[6][2][0]*cnk[5][2][2])+((Cik[6][0][0]*cnk[5][2][0])+(
      Cik[6][1][0]*cnk[5][2][1])));
    cnk[6][2][1] = ((Cik[6][2][1]*cnk[5][2][2])+((Cik[6][0][1]*cnk[5][2][0])+(
      Cik[6][1][1]*cnk[5][2][1])));
    cnk[6][2][2] = ((Cik[6][2][2]*cnk[5][2][2])+((Cik[6][0][2]*cnk[5][2][0])+(
      Cik[6][1][2]*cnk[5][2][1])));
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
    cnk[8][0][0] = ((Cik[0][0][2]*Cik[8][2][0])+((Cik[0][0][0]*Cik[8][0][0])+(
      Cik[0][0][1]*Cik[8][1][0])));
    cnk[8][0][1] = ((Cik[0][0][2]*Cik[8][2][1])+((Cik[0][0][0]*Cik[8][0][1])+(
      Cik[0][0][1]*Cik[8][1][1])));
    cnk[8][0][2] = ((Cik[0][0][2]*Cik[8][2][2])+((Cik[0][0][0]*Cik[8][0][2])+(
      Cik[0][0][1]*Cik[8][1][2])));
    cnk[8][1][0] = ((Cik[0][1][2]*Cik[8][2][0])+((Cik[0][1][0]*Cik[8][0][0])+(
      Cik[0][1][1]*Cik[8][1][0])));
    cnk[8][1][1] = ((Cik[0][1][2]*Cik[8][2][1])+((Cik[0][1][0]*Cik[8][0][1])+(
      Cik[0][1][1]*Cik[8][1][1])));
    cnk[8][1][2] = ((Cik[0][1][2]*Cik[8][2][2])+((Cik[0][1][0]*Cik[8][0][2])+(
      Cik[0][1][1]*Cik[8][1][2])));
    cnk[8][2][0] = ((Cik[0][2][2]*Cik[8][2][0])+((Cik[0][2][0]*Cik[8][0][0])+(
      Cik[0][2][1]*Cik[8][1][0])));
    cnk[8][2][1] = ((Cik[0][2][2]*Cik[8][2][1])+((Cik[0][2][0]*Cik[8][0][1])+(
      Cik[0][2][1]*Cik[8][1][1])));
    cnk[8][2][2] = ((Cik[0][2][2]*Cik[8][2][2])+((Cik[0][2][0]*Cik[8][0][2])+(
      Cik[0][2][1]*Cik[8][1][2])));
    cnk[9][0][0] = ((Cik[9][2][0]*cnk[8][0][2])+((Cik[9][0][0]*cnk[8][0][0])+(
      Cik[9][1][0]*cnk[8][0][1])));
    cnk[9][0][1] = ((Cik[9][2][1]*cnk[8][0][2])+((Cik[9][0][1]*cnk[8][0][0])+(
      Cik[9][1][1]*cnk[8][0][1])));
    cnk[9][0][2] = ((Cik[9][2][2]*cnk[8][0][2])+((Cik[9][0][2]*cnk[8][0][0])+(
      Cik[9][1][2]*cnk[8][0][1])));
    cnk[9][1][0] = ((Cik[9][2][0]*cnk[8][1][2])+((Cik[9][0][0]*cnk[8][1][0])+(
      Cik[9][1][0]*cnk[8][1][1])));
    cnk[9][1][1] = ((Cik[9][2][1]*cnk[8][1][2])+((Cik[9][0][1]*cnk[8][1][0])+(
      Cik[9][1][1]*cnk[8][1][1])));
    cnk[9][1][2] = ((Cik[9][2][2]*cnk[8][1][2])+((Cik[9][0][2]*cnk[8][1][0])+(
      Cik[9][1][2]*cnk[8][1][1])));
    cnk[9][2][0] = ((Cik[9][2][0]*cnk[8][2][2])+((Cik[9][0][0]*cnk[8][2][0])+(
      Cik[9][1][0]*cnk[8][2][1])));
    cnk[9][2][1] = ((Cik[9][2][1]*cnk[8][2][2])+((Cik[9][0][1]*cnk[8][2][0])+(
      Cik[9][1][1]*cnk[8][2][1])));
    cnk[9][2][2] = ((Cik[9][2][2]*cnk[8][2][2])+((Cik[9][0][2]*cnk[8][2][0])+(
      Cik[9][1][2]*cnk[8][2][1])));
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
    cnb[0][0][0] = Cik[0][0][0];
    cnb[0][0][1] = Cik[0][0][1];
    cnb[0][0][2] = Cik[0][0][2];
    cnb[0][1][0] = Cik[0][1][0];
    cnb[0][1][1] = Cik[0][1][1];
    cnb[0][1][2] = Cik[0][1][2];
    cnb[0][2][0] = Cik[0][2][0];
    cnb[0][2][1] = Cik[0][2][1];
    cnb[0][2][2] = Cik[0][2][2];
    cnb[1][0][0] = cnk[1][0][0];
    cnb[1][0][1] = cnk[1][0][1];
    cnb[1][0][2] = cnk[1][0][2];
    cnb[1][1][0] = cnk[1][1][0];
    cnb[1][1][1] = cnk[1][1][1];
    cnb[1][1][2] = cnk[1][1][2];
    cnb[1][2][0] = cnk[1][2][0];
    cnb[1][2][1] = cnk[1][2][1];
    cnb[1][2][2] = cnk[1][2][2];
    cnb[2][0][0] = cnk[2][0][0];
    cnb[2][0][1] = cnk[2][0][1];
    cnb[2][0][2] = cnk[2][0][2];
    cnb[2][1][0] = cnk[2][1][0];
    cnb[2][1][1] = cnk[2][1][1];
    cnb[2][1][2] = cnk[2][1][2];
    cnb[2][2][0] = cnk[2][2][0];
    cnb[2][2][1] = cnk[2][2][1];
    cnb[2][2][2] = cnk[2][2][2];
    cnb[3][0][0] = cnk[3][0][0];
    cnb[3][0][1] = cnk[3][0][1];
    cnb[3][0][2] = cnk[3][0][2];
    cnb[3][1][0] = cnk[3][1][0];
    cnb[3][1][1] = cnk[3][1][1];
    cnb[3][1][2] = cnk[3][1][2];
    cnb[3][2][0] = cnk[3][2][0];
    cnb[3][2][1] = cnk[3][2][1];
    cnb[3][2][2] = cnk[3][2][2];
    cnb[4][0][0] = cnk[4][0][0];
    cnb[4][0][1] = cnk[4][0][1];
    cnb[4][0][2] = cnk[4][0][2];
    cnb[4][1][0] = cnk[4][1][0];
    cnb[4][1][1] = cnk[4][1][1];
    cnb[4][1][2] = cnk[4][1][2];
    cnb[4][2][0] = cnk[4][2][0];
    cnb[4][2][1] = cnk[4][2][1];
    cnb[4][2][2] = cnk[4][2][2];
    cnb[5][0][0] = cnk[5][0][0];
    cnb[5][0][1] = cnk[5][0][1];
    cnb[5][0][2] = cnk[5][0][2];
    cnb[5][1][0] = cnk[5][1][0];
    cnb[5][1][1] = cnk[5][1][1];
    cnb[5][1][2] = cnk[5][1][2];
    cnb[5][2][0] = cnk[5][2][0];
    cnb[5][2][1] = cnk[5][2][1];
    cnb[5][2][2] = cnk[5][2][2];
    cnb[6][0][0] = cnk[6][0][0];
    cnb[6][0][1] = cnk[6][0][1];
    cnb[6][0][2] = cnk[6][0][2];
    cnb[6][1][0] = cnk[6][1][0];
    cnb[6][1][1] = cnk[6][1][1];
    cnb[6][1][2] = cnk[6][1][2];
    cnb[6][2][0] = cnk[6][2][0];
    cnb[6][2][1] = cnk[6][2][1];
    cnb[6][2][2] = cnk[6][2][2];
    cnb[7][0][0] = cnk[7][0][0];
    cnb[7][0][1] = cnk[7][0][1];
    cnb[7][0][2] = cnk[7][0][2];
    cnb[7][1][0] = cnk[7][1][0];
    cnb[7][1][1] = cnk[7][1][1];
    cnb[7][1][2] = cnk[7][1][2];
    cnb[7][2][0] = cnk[7][2][0];
    cnb[7][2][1] = cnk[7][2][1];
    cnb[7][2][2] = cnk[7][2][2];
    cnb[8][0][0] = cnk[8][0][0];
    cnb[8][0][1] = cnk[8][0][1];
    cnb[8][0][2] = cnk[8][0][2];
    cnb[8][1][0] = cnk[8][1][0];
    cnb[8][1][1] = cnk[8][1][1];
    cnb[8][1][2] = cnk[8][1][2];
    cnb[8][2][0] = cnk[8][2][0];
    cnb[8][2][1] = cnk[8][2][1];
    cnb[8][2][2] = cnk[8][2][2];
    cnb[9][0][0] = cnk[9][0][0];
    cnb[9][0][1] = cnk[9][0][1];
    cnb[9][0][2] = cnk[9][0][2];
    cnb[9][1][0] = cnk[9][1][0];
    cnb[9][1][1] = cnk[9][1][1];
    cnb[9][1][2] = cnk[9][1][2];
    cnb[9][2][0] = cnk[9][2][0];
    cnb[9][2][1] = cnk[9][2][1];
    cnb[9][2][2] = cnk[9][2][2];
    cnb[10][0][0] = cnk[10][0][0];
    cnb[10][0][1] = cnk[10][0][1];
    cnb[10][0][2] = cnk[10][0][2];
    cnb[10][1][0] = cnk[10][1][0];
    cnb[10][1][1] = cnk[10][1][1];
    cnb[10][1][2] = cnk[10][1][2];
    cnb[10][2][0] = cnk[10][2][0];
    cnb[10][2][1] = cnk[10][2][1];
    cnb[10][2][2] = cnk[10][2][2];
    cnb[11][0][0] = cnk[11][0][0];
    cnb[11][0][1] = cnk[11][0][1];
    cnb[11][0][2] = cnk[11][0][2];
    cnb[11][1][0] = cnk[11][1][0];
    cnb[11][1][1] = cnk[11][1][1];
    cnb[11][1][2] = cnk[11][1][2];
    cnb[11][2][0] = cnk[11][2][0];
    cnb[11][2][1] = cnk[11][2][1];
    cnb[11][2][2] = cnk[11][2][2];
    cnb[12][0][0] = cnk[12][0][0];
    cnb[12][0][1] = cnk[12][0][1];
    cnb[12][0][2] = cnk[12][0][2];
    cnb[12][1][0] = cnk[12][1][0];
    cnb[12][1][1] = cnk[12][1][1];
    cnb[12][1][2] = cnk[12][1][2];
    cnb[12][2][0] = cnk[12][2][0];
    cnb[12][2][1] = cnk[12][2][1];
    cnb[12][2][2] = cnk[12][2][2];
    cnb[13][0][0] = cnk[13][0][0];
    cnb[13][0][1] = cnk[13][0][1];
    cnb[13][0][2] = cnk[13][0][2];
    cnb[13][1][0] = cnk[13][1][0];
    cnb[13][1][1] = cnk[13][1][1];
    cnb[13][1][2] = cnk[13][1][2];
    cnb[13][2][0] = cnk[13][2][0];
    cnb[13][2][1] = cnk[13][2][1];
    cnb[13][2][2] = cnk[13][2][2];
/*
Compute q-related auxiliary variables
*/
    rik[0][0] = (((Cik[0][2][0]*ri[0][2])+((Cik[0][0][0]*ri[0][0])+(Cik[0][1][0]
      *ri[0][1])))-rk[0][0]);
    rik[0][1] = (((Cik[0][2][1]*ri[0][2])+((Cik[0][0][1]*ri[0][0])+(Cik[0][1][1]
      *ri[0][1])))-rk[0][1]);
    rik[0][2] = (((Cik[0][2][2]*ri[0][2])+((Cik[0][0][2]*ri[0][0])+(Cik[0][1][2]
      *ri[0][1])))-rk[0][2]);
    rik[1][0] = (((Cik[1][2][0]*ri[1][2])+((Cik[1][0][0]*ri[1][0])+(Cik[1][1][0]
      *ri[1][1])))-rk[1][0]);
    rik[1][1] = (((Cik[1][2][1]*ri[1][2])+((Cik[1][0][1]*ri[1][0])+(Cik[1][1][1]
      *ri[1][1])))-rk[1][1]);
    rik[1][2] = (((Cik[1][2][2]*ri[1][2])+((Cik[1][0][2]*ri[1][0])+(Cik[1][1][2]
      *ri[1][1])))-rk[1][2]);
    rik[2][0] = (((Cik[2][2][0]*ri[2][2])+((Cik[2][0][0]*ri[2][0])+(Cik[2][1][0]
      *ri[2][1])))-rk[2][0]);
    rik[2][1] = (((Cik[2][2][1]*ri[2][2])+((Cik[2][0][1]*ri[2][0])+(Cik[2][1][1]
      *ri[2][1])))-rk[2][1]);
    rik[2][2] = (((Cik[2][2][2]*ri[2][2])+((Cik[2][0][2]*ri[2][0])+(Cik[2][1][2]
      *ri[2][1])))-rk[2][2]);
    rik[3][0] = (((Cik[3][2][0]*ri[3][2])+((Cik[3][0][0]*ri[3][0])+(Cik[3][1][0]
      *ri[3][1])))-rk[3][0]);
    rik[3][1] = (((Cik[3][2][1]*ri[3][2])+((Cik[3][0][1]*ri[3][0])+(Cik[3][1][1]
      *ri[3][1])))-rk[3][1]);
    rik[3][2] = (((Cik[3][2][2]*ri[3][2])+((Cik[3][0][2]*ri[3][0])+(Cik[3][1][2]
      *ri[3][1])))-rk[3][2]);
    rik[4][0] = (((Cik[4][2][0]*ri[4][2])+((Cik[4][0][0]*ri[4][0])+(Cik[4][1][0]
      *ri[4][1])))-rk[4][0]);
    rik[4][1] = (((Cik[4][2][1]*ri[4][2])+((Cik[4][0][1]*ri[4][0])+(Cik[4][1][1]
      *ri[4][1])))-rk[4][1]);
    rik[4][2] = (((Cik[4][2][2]*ri[4][2])+((Cik[4][0][2]*ri[4][0])+(Cik[4][1][2]
      *ri[4][1])))-rk[4][2]);
    rik[5][0] = (((Cik[5][2][0]*ri[5][2])+((Cik[5][0][0]*ri[5][0])+(Cik[5][1][0]
      *ri[5][1])))-rk[5][0]);
    rik[5][1] = (((Cik[5][2][1]*ri[5][2])+((Cik[5][0][1]*ri[5][0])+(Cik[5][1][1]
      *ri[5][1])))-rk[5][1]);
    rik[5][2] = (((Cik[5][2][2]*ri[5][2])+((Cik[5][0][2]*ri[5][0])+(Cik[5][1][2]
      *ri[5][1])))-rk[5][2]);
    rik[6][0] = (((Cik[6][2][0]*ri[6][2])+((Cik[6][0][0]*ri[6][0])+(Cik[6][1][0]
      *ri[6][1])))-rk[6][0]);
    rik[6][1] = (((Cik[6][2][1]*ri[6][2])+((Cik[6][0][1]*ri[6][0])+(Cik[6][1][1]
      *ri[6][1])))-rk[6][1]);
    rik[6][2] = (((Cik[6][2][2]*ri[6][2])+((Cik[6][0][2]*ri[6][0])+(Cik[6][1][2]
      *ri[6][1])))-rk[6][2]);
    rik[7][0] = (((Cik[7][2][0]*ri[7][2])+((Cik[7][0][0]*ri[7][0])+(Cik[7][1][0]
      *ri[7][1])))-rk[7][0]);
    rik[7][1] = (((Cik[7][2][1]*ri[7][2])+((Cik[7][0][1]*ri[7][0])+(Cik[7][1][1]
      *ri[7][1])))-rk[7][1]);
    rik[7][2] = (((Cik[7][2][2]*ri[7][2])+((Cik[7][0][2]*ri[7][0])+(Cik[7][1][2]
      *ri[7][1])))-rk[7][2]);
    rik[8][0] = (((Cik[8][2][0]*ri[8][2])+((Cik[8][0][0]*ri[8][0])+(Cik[8][1][0]
      *ri[8][1])))-rk[8][0]);
    rik[8][1] = (((Cik[8][2][1]*ri[8][2])+((Cik[8][0][1]*ri[8][0])+(Cik[8][1][1]
      *ri[8][1])))-rk[8][1]);
    rik[8][2] = (((Cik[8][2][2]*ri[8][2])+((Cik[8][0][2]*ri[8][0])+(Cik[8][1][2]
      *ri[8][1])))-rk[8][2]);
    rik[9][0] = (((Cik[9][2][0]*ri[9][2])+((Cik[9][0][0]*ri[9][0])+(Cik[9][1][0]
      *ri[9][1])))-rk[9][0]);
    rik[9][1] = (((Cik[9][2][1]*ri[9][2])+((Cik[9][0][1]*ri[9][0])+(Cik[9][1][1]
      *ri[9][1])))-rk[9][1]);
    rik[9][2] = (((Cik[9][2][2]*ri[9][2])+((Cik[9][0][2]*ri[9][0])+(Cik[9][1][2]
      *ri[9][1])))-rk[9][2]);
    rik[10][0] = (((Cik[10][2][0]*ri[10][2])+((Cik[10][0][0]*ri[10][0])+(
      Cik[10][1][0]*ri[10][1])))-rk[10][0]);
    rik[10][1] = (((Cik[10][2][1]*ri[10][2])+((Cik[10][0][1]*ri[10][0])+(
      Cik[10][1][1]*ri[10][1])))-rk[10][1]);
    rik[10][2] = (((Cik[10][2][2]*ri[10][2])+((Cik[10][0][2]*ri[10][0])+(
      Cik[10][1][2]*ri[10][1])))-rk[10][2]);
    rik[11][0] = (((Cik[11][2][0]*ri[11][2])+((Cik[11][0][0]*ri[11][0])+(
      Cik[11][1][0]*ri[11][1])))-rk[11][0]);
    rik[11][1] = (((Cik[11][2][1]*ri[11][2])+((Cik[11][0][1]*ri[11][0])+(
      Cik[11][1][1]*ri[11][1])))-rk[11][1]);
    rik[11][2] = (((Cik[11][2][2]*ri[11][2])+((Cik[11][0][2]*ri[11][0])+(
      Cik[11][1][2]*ri[11][1])))-rk[11][2]);
    rik[12][0] = (((Cik[12][2][0]*ri[12][2])+((Cik[12][0][0]*ri[12][0])+(
      Cik[12][1][0]*ri[12][1])))-rk[12][0]);
    rik[12][1] = (((Cik[12][2][1]*ri[12][2])+((Cik[12][0][1]*ri[12][0])+(
      Cik[12][1][1]*ri[12][1])))-rk[12][1]);
    rik[12][2] = (((Cik[12][2][2]*ri[12][2])+((Cik[12][0][2]*ri[12][0])+(
      Cik[12][1][2]*ri[12][1])))-rk[12][2]);
    rik[13][0] = (((Cik[13][2][0]*ri[13][2])+((Cik[13][0][0]*ri[13][0])+(
      Cik[13][1][0]*ri[13][1])))-rk[13][0]);
    rik[13][1] = (((Cik[13][2][1]*ri[13][2])+((Cik[13][0][1]*ri[13][0])+(
      Cik[13][1][1]*ri[13][1])))-rk[13][1]);
    rik[13][2] = (((Cik[13][2][2]*ri[13][2])+((Cik[13][0][2]*ri[13][0])+(
      Cik[13][1][2]*ri[13][1])))-rk[13][2]);
/*
Compute rnk & rnb (mass center locations in N)
*/
    rnk[0][0] = (ri[0][0]-((Cik[0][0][2]*rk[0][2])+((Cik[0][0][0]*rk[0][0])+(
      Cik[0][0][1]*rk[0][1]))));
    rnk[0][1] = (ri[0][1]-((Cik[0][1][2]*rk[0][2])+((Cik[0][1][0]*rk[0][0])+(
      Cik[0][1][1]*rk[0][1]))));
    rnk[0][2] = (ri[0][2]-((Cik[0][2][2]*rk[0][2])+((Cik[0][2][0]*rk[0][0])+(
      Cik[0][2][1]*rk[0][1]))));
    rnk[1][0] = ((rnk[0][0]+((Cik[0][0][2]*ri[1][2])+((Cik[0][0][0]*ri[1][0])+(
      Cik[0][0][1]*ri[1][1]))))-((cnk[1][0][2]*rk[1][2])+((cnk[1][0][0]*rk[1][0]
      )+(cnk[1][0][1]*rk[1][1]))));
    rnk[1][1] = ((rnk[0][1]+((Cik[0][1][2]*ri[1][2])+((Cik[0][1][0]*ri[1][0])+(
      Cik[0][1][1]*ri[1][1]))))-((cnk[1][1][2]*rk[1][2])+((cnk[1][1][0]*rk[1][0]
      )+(cnk[1][1][1]*rk[1][1]))));
    rnk[1][2] = ((rnk[0][2]+((Cik[0][2][2]*ri[1][2])+((Cik[0][2][0]*ri[1][0])+(
      Cik[0][2][1]*ri[1][1]))))-((cnk[1][2][2]*rk[1][2])+((cnk[1][2][0]*rk[1][0]
      )+(cnk[1][2][1]*rk[1][1]))));
    rnk[2][0] = ((rnk[0][0]+((Cik[0][0][2]*ri[2][2])+((Cik[0][0][0]*ri[2][0])+(
      Cik[0][0][1]*ri[2][1]))))-((cnk[2][0][2]*rk[2][2])+((cnk[2][0][0]*rk[2][0]
      )+(cnk[2][0][1]*rk[2][1]))));
    rnk[2][1] = ((rnk[0][1]+((Cik[0][1][2]*ri[2][2])+((Cik[0][1][0]*ri[2][0])+(
      Cik[0][1][1]*ri[2][1]))))-((cnk[2][1][2]*rk[2][2])+((cnk[2][1][0]*rk[2][0]
      )+(cnk[2][1][1]*rk[2][1]))));
    rnk[2][2] = ((rnk[0][2]+((Cik[0][2][2]*ri[2][2])+((Cik[0][2][0]*ri[2][0])+(
      Cik[0][2][1]*ri[2][1]))))-((cnk[2][2][2]*rk[2][2])+((cnk[2][2][0]*rk[2][0]
      )+(cnk[2][2][1]*rk[2][1]))));
    rnk[3][0] = ((rnk[2][0]+((cnk[2][0][2]*ri[3][2])+((cnk[2][0][0]*ri[3][0])+(
      cnk[2][0][1]*ri[3][1]))))-((cnk[3][0][2]*rk[3][2])+((cnk[3][0][0]*rk[3][0]
      )+(cnk[3][0][1]*rk[3][1]))));
    rnk[3][1] = ((rnk[2][1]+((cnk[2][1][2]*ri[3][2])+((cnk[2][1][0]*ri[3][0])+(
      cnk[2][1][1]*ri[3][1]))))-((cnk[3][1][2]*rk[3][2])+((cnk[3][1][0]*rk[3][0]
      )+(cnk[3][1][1]*rk[3][1]))));
    rnk[3][2] = ((rnk[2][2]+((cnk[2][2][2]*ri[3][2])+((cnk[2][2][0]*ri[3][0])+(
      cnk[2][2][1]*ri[3][1]))))-((cnk[3][2][2]*rk[3][2])+((cnk[3][2][0]*rk[3][0]
      )+(cnk[3][2][1]*rk[3][1]))));
    rnk[4][0] = ((rnk[3][0]+((cnk[3][0][2]*ri[4][2])+((cnk[3][0][0]*ri[4][0])+(
      cnk[3][0][1]*ri[4][1]))))-((cnk[4][0][2]*rk[4][2])+((cnk[4][0][0]*rk[4][0]
      )+(cnk[4][0][1]*rk[4][1]))));
    rnk[4][1] = ((rnk[3][1]+((cnk[3][1][2]*ri[4][2])+((cnk[3][1][0]*ri[4][0])+(
      cnk[3][1][1]*ri[4][1]))))-((cnk[4][1][2]*rk[4][2])+((cnk[4][1][0]*rk[4][0]
      )+(cnk[4][1][1]*rk[4][1]))));
    rnk[4][2] = ((rnk[3][2]+((cnk[3][2][2]*ri[4][2])+((cnk[3][2][0]*ri[4][0])+(
      cnk[3][2][1]*ri[4][1]))))-((cnk[4][2][2]*rk[4][2])+((cnk[4][2][0]*rk[4][0]
      )+(cnk[4][2][1]*rk[4][1]))));
    rnk[5][0] = ((rnk[4][0]+((cnk[4][0][2]*ri[5][2])+((cnk[4][0][0]*ri[5][0])+(
      cnk[4][0][1]*ri[5][1]))))-((cnk[5][0][2]*rk[5][2])+((cnk[5][0][0]*rk[5][0]
      )+(cnk[5][0][1]*rk[5][1]))));
    rnk[5][1] = ((rnk[4][1]+((cnk[4][1][2]*ri[5][2])+((cnk[4][1][0]*ri[5][0])+(
      cnk[4][1][1]*ri[5][1]))))-((cnk[5][1][2]*rk[5][2])+((cnk[5][1][0]*rk[5][0]
      )+(cnk[5][1][1]*rk[5][1]))));
    rnk[5][2] = ((rnk[4][2]+((cnk[4][2][2]*ri[5][2])+((cnk[4][2][0]*ri[5][0])+(
      cnk[4][2][1]*ri[5][1]))))-((cnk[5][2][2]*rk[5][2])+((cnk[5][2][0]*rk[5][0]
      )+(cnk[5][2][1]*rk[5][1]))));
    rnk[6][0] = ((rnk[5][0]+((cnk[5][0][2]*ri[6][2])+((cnk[5][0][0]*ri[6][0])+(
      cnk[5][0][1]*ri[6][1]))))-((cnk[6][0][2]*rk[6][2])+((cnk[6][0][0]*rk[6][0]
      )+(cnk[6][0][1]*rk[6][1]))));
    rnk[6][1] = ((rnk[5][1]+((cnk[5][1][2]*ri[6][2])+((cnk[5][1][0]*ri[6][0])+(
      cnk[5][1][1]*ri[6][1]))))-((cnk[6][1][2]*rk[6][2])+((cnk[6][1][0]*rk[6][0]
      )+(cnk[6][1][1]*rk[6][1]))));
    rnk[6][2] = ((rnk[5][2]+((cnk[5][2][2]*ri[6][2])+((cnk[5][2][0]*ri[6][0])+(
      cnk[5][2][1]*ri[6][1]))))-((cnk[6][2][2]*rk[6][2])+((cnk[6][2][0]*rk[6][0]
      )+(cnk[6][2][1]*rk[6][1]))));
    rnk[7][0] = ((rnk[6][0]+((cnk[6][0][2]*ri[7][2])+((cnk[6][0][0]*ri[7][0])+(
      cnk[6][0][1]*ri[7][1]))))-((cnk[7][0][2]*rk[7][2])+((cnk[7][0][0]*rk[7][0]
      )+(cnk[7][0][1]*rk[7][1]))));
    rnk[7][1] = ((rnk[6][1]+((cnk[6][1][2]*ri[7][2])+((cnk[6][1][0]*ri[7][0])+(
      cnk[6][1][1]*ri[7][1]))))-((cnk[7][1][2]*rk[7][2])+((cnk[7][1][0]*rk[7][0]
      )+(cnk[7][1][1]*rk[7][1]))));
    rnk[7][2] = ((rnk[6][2]+((cnk[6][2][2]*ri[7][2])+((cnk[6][2][0]*ri[7][0])+(
      cnk[6][2][1]*ri[7][1]))))-((cnk[7][2][2]*rk[7][2])+((cnk[7][2][0]*rk[7][0]
      )+(cnk[7][2][1]*rk[7][1]))));
    rnk[8][0] = ((rnk[0][0]+((Cik[0][0][2]*ri[8][2])+((Cik[0][0][0]*ri[8][0])+(
      Cik[0][0][1]*ri[8][1]))))-((cnk[8][0][2]*rk[8][2])+((cnk[8][0][0]*rk[8][0]
      )+(cnk[8][0][1]*rk[8][1]))));
    rnk[8][1] = ((rnk[0][1]+((Cik[0][1][2]*ri[8][2])+((Cik[0][1][0]*ri[8][0])+(
      Cik[0][1][1]*ri[8][1]))))-((cnk[8][1][2]*rk[8][2])+((cnk[8][1][0]*rk[8][0]
      )+(cnk[8][1][1]*rk[8][1]))));
    rnk[8][2] = ((rnk[0][2]+((Cik[0][2][2]*ri[8][2])+((Cik[0][2][0]*ri[8][0])+(
      Cik[0][2][1]*ri[8][1]))))-((cnk[8][2][2]*rk[8][2])+((cnk[8][2][0]*rk[8][0]
      )+(cnk[8][2][1]*rk[8][1]))));
    rnk[9][0] = ((rnk[8][0]+((cnk[8][0][2]*ri[9][2])+((cnk[8][0][0]*ri[9][0])+(
      cnk[8][0][1]*ri[9][1]))))-((cnk[9][0][2]*rk[9][2])+((cnk[9][0][0]*rk[9][0]
      )+(cnk[9][0][1]*rk[9][1]))));
    rnk[9][1] = ((rnk[8][1]+((cnk[8][1][2]*ri[9][2])+((cnk[8][1][0]*ri[9][0])+(
      cnk[8][1][1]*ri[9][1]))))-((cnk[9][1][2]*rk[9][2])+((cnk[9][1][0]*rk[9][0]
      )+(cnk[9][1][1]*rk[9][1]))));
    rnk[9][2] = ((rnk[8][2]+((cnk[8][2][2]*ri[9][2])+((cnk[8][2][0]*ri[9][0])+(
      cnk[8][2][1]*ri[9][1]))))-((cnk[9][2][2]*rk[9][2])+((cnk[9][2][0]*rk[9][0]
      )+(cnk[9][2][1]*rk[9][1]))));
    rnk[10][0] = ((rnk[9][0]+((cnk[9][0][2]*ri[10][2])+((cnk[9][0][0]*ri[10][0])
      +(cnk[9][0][1]*ri[10][1]))))-((cnk[10][0][2]*rk[10][2])+((cnk[10][0][0]*
      rk[10][0])+(cnk[10][0][1]*rk[10][1]))));
    rnk[10][1] = ((rnk[9][1]+((cnk[9][1][2]*ri[10][2])+((cnk[9][1][0]*ri[10][0])
      +(cnk[9][1][1]*ri[10][1]))))-((cnk[10][1][2]*rk[10][2])+((cnk[10][1][0]*
      rk[10][0])+(cnk[10][1][1]*rk[10][1]))));
    rnk[10][2] = ((rnk[9][2]+((cnk[9][2][2]*ri[10][2])+((cnk[9][2][0]*ri[10][0])
      +(cnk[9][2][1]*ri[10][1]))))-((cnk[10][2][2]*rk[10][2])+((cnk[10][2][0]*
      rk[10][0])+(cnk[10][2][1]*rk[10][1]))));
    rnk[11][0] = ((rnk[10][0]+((cnk[10][0][2]*ri[11][2])+((cnk[10][0][0]*
      ri[11][0])+(cnk[10][0][1]*ri[11][1]))))-((cnk[11][0][2]*rk[11][2])+((
      cnk[11][0][0]*rk[11][0])+(cnk[11][0][1]*rk[11][1]))));
    rnk[11][1] = ((rnk[10][1]+((cnk[10][1][2]*ri[11][2])+((cnk[10][1][0]*
      ri[11][0])+(cnk[10][1][1]*ri[11][1]))))-((cnk[11][1][2]*rk[11][2])+((
      cnk[11][1][0]*rk[11][0])+(cnk[11][1][1]*rk[11][1]))));
    rnk[11][2] = ((rnk[10][2]+((cnk[10][2][2]*ri[11][2])+((cnk[10][2][0]*
      ri[11][0])+(cnk[10][2][1]*ri[11][1]))))-((cnk[11][2][2]*rk[11][2])+((
      cnk[11][2][0]*rk[11][0])+(cnk[11][2][1]*rk[11][1]))));
    rnk[12][0] = ((rnk[11][0]+((cnk[11][0][2]*ri[12][2])+((cnk[11][0][0]*
      ri[12][0])+(cnk[11][0][1]*ri[12][1]))))-((cnk[12][0][2]*rk[12][2])+((
      cnk[12][0][0]*rk[12][0])+(cnk[12][0][1]*rk[12][1]))));
    rnk[12][1] = ((rnk[11][1]+((cnk[11][1][2]*ri[12][2])+((cnk[11][1][0]*
      ri[12][0])+(cnk[11][1][1]*ri[12][1]))))-((cnk[12][1][2]*rk[12][2])+((
      cnk[12][1][0]*rk[12][0])+(cnk[12][1][1]*rk[12][1]))));
    rnk[12][2] = ((rnk[11][2]+((cnk[11][2][2]*ri[12][2])+((cnk[11][2][0]*
      ri[12][0])+(cnk[11][2][1]*ri[12][1]))))-((cnk[12][2][2]*rk[12][2])+((
      cnk[12][2][0]*rk[12][0])+(cnk[12][2][1]*rk[12][1]))));
    rnk[13][0] = ((rnk[12][0]+((cnk[12][0][2]*ri[13][2])+((cnk[12][0][0]*
      ri[13][0])+(cnk[12][0][1]*ri[13][1]))))-((cnk[13][0][2]*rk[13][2])+((
      cnk[13][0][0]*rk[13][0])+(cnk[13][0][1]*rk[13][1]))));
    rnk[13][1] = ((rnk[12][1]+((cnk[12][1][2]*ri[13][2])+((cnk[12][1][0]*
      ri[13][0])+(cnk[12][1][1]*ri[13][1]))))-((cnk[13][1][2]*rk[13][2])+((
      cnk[13][1][0]*rk[13][0])+(cnk[13][1][1]*rk[13][1]))));
    rnk[13][2] = ((rnk[12][2]+((cnk[12][2][2]*ri[13][2])+((cnk[12][2][0]*
      ri[13][0])+(cnk[12][2][1]*ri[13][1]))))-((cnk[13][2][2]*rk[13][2])+((
      cnk[13][2][0]*rk[13][0])+(cnk[13][2][1]*rk[13][1]))));
    rnb[0][0] = rnk[0][0];
    rnb[0][1] = rnk[0][1];
    rnb[0][2] = rnk[0][2];
    rnb[1][0] = rnk[1][0];
    rnb[1][1] = rnk[1][1];
    rnb[1][2] = rnk[1][2];
    rnb[2][0] = rnk[2][0];
    rnb[2][1] = rnk[2][1];
    rnb[2][2] = rnk[2][2];
    rnb[3][0] = rnk[3][0];
    rnb[3][1] = rnk[3][1];
    rnb[3][2] = rnk[3][2];
    rnb[4][0] = rnk[4][0];
    rnb[4][1] = rnk[4][1];
    rnb[4][2] = rnk[4][2];
    rnb[5][0] = rnk[5][0];
    rnb[5][1] = rnk[5][1];
    rnb[5][2] = rnk[5][2];
    rnb[6][0] = rnk[6][0];
    rnb[6][1] = rnk[6][1];
    rnb[6][2] = rnk[6][2];
    rnb[7][0] = rnk[7][0];
    rnb[7][1] = rnk[7][1];
    rnb[7][2] = rnk[7][2];
    rnb[8][0] = rnk[8][0];
    rnb[8][1] = rnk[8][1];
    rnb[8][2] = rnk[8][2];
    rnb[9][0] = rnk[9][0];
    rnb[9][1] = rnk[9][1];
    rnb[9][2] = rnk[9][2];
    rnb[10][0] = rnk[10][0];
    rnb[10][1] = rnk[10][1];
    rnb[10][2] = rnk[10][2];
    rnb[11][0] = rnk[11][0];
    rnb[11][1] = rnk[11][1];
    rnb[11][2] = rnk[11][2];
    rnb[12][0] = rnk[12][0];
    rnb[12][1] = rnk[12][1];
    rnb[12][2] = rnk[12][2];
    rnb[13][0] = rnk[13][0];
    rnb[13][1] = rnk[13][1];
    rnb[13][2] = rnk[13][2];
/*
Compute com (system mass center location in N)
*/
    com[0] = ((1./mtot)*((mk[13]*rnk[13][0])+((mk[12]*rnk[12][0])+((mk[11]*
      rnk[11][0])+((mk[10]*rnk[10][0])+((mk[9]*rnk[9][0])+((mk[8]*rnk[8][0])+((
      mk[7]*rnk[7][0])+((mk[6]*rnk[6][0])+((mk[5]*rnk[5][0])+((mk[4]*rnk[4][0])+
      ((mk[3]*rnk[3][0])+((mk[2]*rnk[2][0])+((mk[0]*rnk[0][0])+(mk[1]*rnk[1][0])
      ))))))))))))));
    com[1] = ((1./mtot)*((mk[13]*rnk[13][1])+((mk[12]*rnk[12][1])+((mk[11]*
      rnk[11][1])+((mk[10]*rnk[10][1])+((mk[9]*rnk[9][1])+((mk[8]*rnk[8][1])+((
      mk[7]*rnk[7][1])+((mk[6]*rnk[6][1])+((mk[5]*rnk[5][1])+((mk[4]*rnk[4][1])+
      ((mk[3]*rnk[3][1])+((mk[2]*rnk[2][1])+((mk[0]*rnk[0][1])+(mk[1]*rnk[1][1])
      ))))))))))))));
    com[2] = ((1./mtot)*((mk[13]*rnk[13][2])+((mk[12]*rnk[12][2])+((mk[11]*
      rnk[11][2])+((mk[10]*rnk[10][2])+((mk[9]*rnk[9][2])+((mk[8]*rnk[8][2])+((
      mk[7]*rnk[7][2])+((mk[6]*rnk[6][2])+((mk[5]*rnk[5][2])+((mk[4]*rnk[4][2])+
      ((mk[3]*rnk[3][2])+((mk[2]*rnk[2][2])+((mk[0]*rnk[0][2])+(mk[1]*rnk[1][2])
      ))))))))))))));
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
    for (i = 0; i < 14; i++) {
        u[i] = uin[i];
    }
/*
Compute u-related auxiliary variables
*/
    Wik[0][0] = (pin[0][0]*u[0]);
    Wik[0][1] = (pin[0][1]*u[0]);
    Wik[0][2] = (pin[0][2]*u[0]);
    Wik[1][0] = (pin[1][0]*u[1]);
    Wik[1][1] = (pin[1][1]*u[1]);
    Wik[1][2] = (pin[1][2]*u[1]);
    Wik[2][0] = (pin[2][0]*u[2]);
    Wik[2][1] = (pin[2][1]*u[2]);
    Wik[2][2] = (pin[2][2]*u[2]);
    Wik[3][0] = (pin[3][0]*u[3]);
    Wik[3][1] = (pin[3][1]*u[3]);
    Wik[3][2] = (pin[3][2]*u[3]);
    Wik[4][0] = (pin[4][0]*u[4]);
    Wik[4][1] = (pin[4][1]*u[4]);
    Wik[4][2] = (pin[4][2]*u[4]);
    Wik[5][0] = (pin[5][0]*u[5]);
    Wik[5][1] = (pin[5][1]*u[5]);
    Wik[5][2] = (pin[5][2]*u[5]);
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
/*
Compute wk & wb (angular velocities)
*/
    wk[1][0] = (Wik[1][0]+((Cik[1][2][0]*Wik[0][2])+((Cik[1][0][0]*Wik[0][0])+(
      Cik[1][1][0]*Wik[0][1]))));
    wk[1][1] = (Wik[1][1]+((Cik[1][2][1]*Wik[0][2])+((Cik[1][0][1]*Wik[0][0])+(
      Cik[1][1][1]*Wik[0][1]))));
    wk[1][2] = (Wik[1][2]+((Cik[1][2][2]*Wik[0][2])+((Cik[1][0][2]*Wik[0][0])+(
      Cik[1][1][2]*Wik[0][1]))));
    wk[2][0] = (Wik[2][0]+((Cik[2][2][0]*Wik[0][2])+((Cik[2][0][0]*Wik[0][0])+(
      Cik[2][1][0]*Wik[0][1]))));
    wk[2][1] = (Wik[2][1]+((Cik[2][2][1]*Wik[0][2])+((Cik[2][0][1]*Wik[0][0])+(
      Cik[2][1][1]*Wik[0][1]))));
    wk[2][2] = (Wik[2][2]+((Cik[2][2][2]*Wik[0][2])+((Cik[2][0][2]*Wik[0][0])+(
      Cik[2][1][2]*Wik[0][1]))));
    wk[3][0] = (Wik[3][0]+((Cik[3][2][0]*wk[2][2])+((Cik[3][0][0]*wk[2][0])+(
      Cik[3][1][0]*wk[2][1]))));
    wk[3][1] = (Wik[3][1]+((Cik[3][2][1]*wk[2][2])+((Cik[3][0][1]*wk[2][0])+(
      Cik[3][1][1]*wk[2][1]))));
    wk[3][2] = (Wik[3][2]+((Cik[3][2][2]*wk[2][2])+((Cik[3][0][2]*wk[2][0])+(
      Cik[3][1][2]*wk[2][1]))));
    wk[4][0] = (Wik[4][0]+((Cik[4][2][0]*wk[3][2])+((Cik[4][0][0]*wk[3][0])+(
      Cik[4][1][0]*wk[3][1]))));
    wk[4][1] = (Wik[4][1]+((Cik[4][2][1]*wk[3][2])+((Cik[4][0][1]*wk[3][0])+(
      Cik[4][1][1]*wk[3][1]))));
    wk[4][2] = (Wik[4][2]+((Cik[4][2][2]*wk[3][2])+((Cik[4][0][2]*wk[3][0])+(
      Cik[4][1][2]*wk[3][1]))));
    wk[5][0] = (Wik[5][0]+((Cik[5][2][0]*wk[4][2])+((Cik[5][0][0]*wk[4][0])+(
      Cik[5][1][0]*wk[4][1]))));
    wk[5][1] = (Wik[5][1]+((Cik[5][2][1]*wk[4][2])+((Cik[5][0][1]*wk[4][0])+(
      Cik[5][1][1]*wk[4][1]))));
    wk[5][2] = (Wik[5][2]+((Cik[5][2][2]*wk[4][2])+((Cik[5][0][2]*wk[4][0])+(
      Cik[5][1][2]*wk[4][1]))));
    wk[6][0] = (Wik[6][0]+((Cik[6][2][0]*wk[5][2])+((Cik[6][0][0]*wk[5][0])+(
      Cik[6][1][0]*wk[5][1]))));
    wk[6][1] = (Wik[6][1]+((Cik[6][2][1]*wk[5][2])+((Cik[6][0][1]*wk[5][0])+(
      Cik[6][1][1]*wk[5][1]))));
    wk[6][2] = (Wik[6][2]+((Cik[6][2][2]*wk[5][2])+((Cik[6][0][2]*wk[5][0])+(
      Cik[6][1][2]*wk[5][1]))));
    wk[7][0] = (Wik[7][0]+((Cik[7][2][0]*wk[6][2])+((Cik[7][0][0]*wk[6][0])+(
      Cik[7][1][0]*wk[6][1]))));
    wk[7][1] = (Wik[7][1]+((Cik[7][2][1]*wk[6][2])+((Cik[7][0][1]*wk[6][0])+(
      Cik[7][1][1]*wk[6][1]))));
    wk[7][2] = (Wik[7][2]+((Cik[7][2][2]*wk[6][2])+((Cik[7][0][2]*wk[6][0])+(
      Cik[7][1][2]*wk[6][1]))));
    wk[8][0] = (Wik[8][0]+((Cik[8][2][0]*Wik[0][2])+((Cik[8][0][0]*Wik[0][0])+(
      Cik[8][1][0]*Wik[0][1]))));
    wk[8][1] = (Wik[8][1]+((Cik[8][2][1]*Wik[0][2])+((Cik[8][0][1]*Wik[0][0])+(
      Cik[8][1][1]*Wik[0][1]))));
    wk[8][2] = (Wik[8][2]+((Cik[8][2][2]*Wik[0][2])+((Cik[8][0][2]*Wik[0][0])+(
      Cik[8][1][2]*Wik[0][1]))));
    wk[9][0] = (Wik[9][0]+((Cik[9][2][0]*wk[8][2])+((Cik[9][0][0]*wk[8][0])+(
      Cik[9][1][0]*wk[8][1]))));
    wk[9][1] = (Wik[9][1]+((Cik[9][2][1]*wk[8][2])+((Cik[9][0][1]*wk[8][0])+(
      Cik[9][1][1]*wk[8][1]))));
    wk[9][2] = (Wik[9][2]+((Cik[9][2][2]*wk[8][2])+((Cik[9][0][2]*wk[8][0])+(
      Cik[9][1][2]*wk[8][1]))));
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
    wb[0][0] = Wik[0][0];
    wb[0][1] = Wik[0][1];
    wb[0][2] = Wik[0][2];
    wb[1][0] = wk[1][0];
    wb[1][1] = wk[1][1];
    wb[1][2] = wk[1][2];
    wb[2][0] = wk[2][0];
    wb[2][1] = wk[2][1];
    wb[2][2] = wk[2][2];
    wb[3][0] = wk[3][0];
    wb[3][1] = wk[3][1];
    wb[3][2] = wk[3][2];
    wb[4][0] = wk[4][0];
    wb[4][1] = wk[4][1];
    wb[4][2] = wk[4][2];
    wb[5][0] = wk[5][0];
    wb[5][1] = wk[5][1];
    wb[5][2] = wk[5][2];
    wb[6][0] = wk[6][0];
    wb[6][1] = wk[6][1];
    wb[6][2] = wk[6][2];
    wb[7][0] = wk[7][0];
    wb[7][1] = wk[7][1];
    wb[7][2] = wk[7][2];
    wb[8][0] = wk[8][0];
    wb[8][1] = wk[8][1];
    wb[8][2] = wk[8][2];
    wb[9][0] = wk[9][0];
    wb[9][1] = wk[9][1];
    wb[9][2] = wk[9][2];
    wb[10][0] = wk[10][0];
    wb[10][1] = wk[10][1];
    wb[10][2] = wk[10][2];
    wb[11][0] = wk[11][0];
    wb[11][1] = wk[11][1];
    wb[11][2] = wk[11][2];
    wb[12][0] = wk[12][0];
    wb[12][1] = wk[12][1];
    wb[12][2] = wk[12][2];
    wb[13][0] = wk[13][0];
    wb[13][1] = wk[13][1];
    wb[13][2] = wk[13][2];
/*
Compute auxiliary variables involving wk
*/
    Wirk[1][0] = ((ri[1][2]*Wik[0][1])-(ri[1][1]*Wik[0][2]));
    Wirk[1][1] = ((ri[1][0]*Wik[0][2])-(ri[1][2]*Wik[0][0]));
    Wirk[1][2] = ((ri[1][1]*Wik[0][0])-(ri[1][0]*Wik[0][1]));
    Wirk[2][0] = ((ri[2][2]*Wik[0][1])-(ri[2][1]*Wik[0][2]));
    Wirk[2][1] = ((ri[2][0]*Wik[0][2])-(ri[2][2]*Wik[0][0]));
    Wirk[2][2] = ((ri[2][1]*Wik[0][0])-(ri[2][0]*Wik[0][1]));
    Wirk[3][0] = ((ri[3][2]*wk[2][1])-(ri[3][1]*wk[2][2]));
    Wirk[3][1] = ((ri[3][0]*wk[2][2])-(ri[3][2]*wk[2][0]));
    Wirk[3][2] = ((ri[3][1]*wk[2][0])-(ri[3][0]*wk[2][1]));
    Wirk[4][0] = ((ri[4][2]*wk[3][1])-(ri[4][1]*wk[3][2]));
    Wirk[4][1] = ((ri[4][0]*wk[3][2])-(ri[4][2]*wk[3][0]));
    Wirk[4][2] = ((ri[4][1]*wk[3][0])-(ri[4][0]*wk[3][1]));
    Wirk[5][0] = ((ri[5][2]*wk[4][1])-(ri[5][1]*wk[4][2]));
    Wirk[5][1] = ((ri[5][0]*wk[4][2])-(ri[5][2]*wk[4][0]));
    Wirk[5][2] = ((ri[5][1]*wk[4][0])-(ri[5][0]*wk[4][1]));
    Wirk[6][0] = ((ri[6][2]*wk[5][1])-(ri[6][1]*wk[5][2]));
    Wirk[6][1] = ((ri[6][0]*wk[5][2])-(ri[6][2]*wk[5][0]));
    Wirk[6][2] = ((ri[6][1]*wk[5][0])-(ri[6][0]*wk[5][1]));
    Wirk[7][0] = ((ri[7][2]*wk[6][1])-(ri[7][1]*wk[6][2]));
    Wirk[7][1] = ((ri[7][0]*wk[6][2])-(ri[7][2]*wk[6][0]));
    Wirk[7][2] = ((ri[7][1]*wk[6][0])-(ri[7][0]*wk[6][1]));
    Wirk[8][0] = ((ri[8][2]*Wik[0][1])-(ri[8][1]*Wik[0][2]));
    Wirk[8][1] = ((ri[8][0]*Wik[0][2])-(ri[8][2]*Wik[0][0]));
    Wirk[8][2] = ((ri[8][1]*Wik[0][0])-(ri[8][0]*Wik[0][1]));
    Wirk[9][0] = ((ri[9][2]*wk[8][1])-(ri[9][1]*wk[8][2]));
    Wirk[9][1] = ((ri[9][0]*wk[8][2])-(ri[9][2]*wk[8][0]));
    Wirk[9][2] = ((ri[9][1]*wk[8][0])-(ri[9][0]*wk[8][1]));
    Wirk[10][0] = ((ri[10][2]*wk[9][1])-(ri[10][1]*wk[9][2]));
    Wirk[10][1] = ((ri[10][0]*wk[9][2])-(ri[10][2]*wk[9][0]));
    Wirk[10][2] = ((ri[10][1]*wk[9][0])-(ri[10][0]*wk[9][1]));
    Wirk[11][0] = ((ri[11][2]*wk[10][1])-(ri[11][1]*wk[10][2]));
    Wirk[11][1] = ((ri[11][0]*wk[10][2])-(ri[11][2]*wk[10][0]));
    Wirk[11][2] = ((ri[11][1]*wk[10][0])-(ri[11][0]*wk[10][1]));
    Wirk[12][0] = ((ri[12][2]*wk[11][1])-(ri[12][1]*wk[11][2]));
    Wirk[12][1] = ((ri[12][0]*wk[11][2])-(ri[12][2]*wk[11][0]));
    Wirk[12][2] = ((ri[12][1]*wk[11][0])-(ri[12][0]*wk[11][1]));
    Wirk[13][0] = ((ri[13][2]*wk[12][1])-(ri[13][1]*wk[12][2]));
    Wirk[13][1] = ((ri[13][0]*wk[12][2])-(ri[13][2]*wk[12][0]));
    Wirk[13][2] = ((ri[13][1]*wk[12][0])-(ri[13][0]*wk[12][1]));
    Wkrpk[0][0] = ((rk[0][1]*Wik[0][2])-(rk[0][2]*Wik[0][1]));
    Wkrpk[0][1] = ((rk[0][2]*Wik[0][0])-(rk[0][0]*Wik[0][2]));
    Wkrpk[0][2] = ((rk[0][0]*Wik[0][1])-(rk[0][1]*Wik[0][0]));
    Wkrpk[1][0] = ((rk[1][1]*wk[1][2])-(rk[1][2]*wk[1][1]));
    Wkrpk[1][1] = ((rk[1][2]*wk[1][0])-(rk[1][0]*wk[1][2]));
    Wkrpk[1][2] = ((rk[1][0]*wk[1][1])-(rk[1][1]*wk[1][0]));
    Wkrpk[2][0] = ((rk[2][1]*wk[2][2])-(rk[2][2]*wk[2][1]));
    Wkrpk[2][1] = ((rk[2][2]*wk[2][0])-(rk[2][0]*wk[2][2]));
    Wkrpk[2][2] = ((rk[2][0]*wk[2][1])-(rk[2][1]*wk[2][0]));
    Wkrpk[3][0] = ((rk[3][1]*wk[3][2])-(rk[3][2]*wk[3][1]));
    Wkrpk[3][1] = ((rk[3][2]*wk[3][0])-(rk[3][0]*wk[3][2]));
    Wkrpk[3][2] = ((rk[3][0]*wk[3][1])-(rk[3][1]*wk[3][0]));
    Wkrpk[4][0] = ((rk[4][1]*wk[4][2])-(rk[4][2]*wk[4][1]));
    Wkrpk[4][1] = ((rk[4][2]*wk[4][0])-(rk[4][0]*wk[4][2]));
    Wkrpk[4][2] = ((rk[4][0]*wk[4][1])-(rk[4][1]*wk[4][0]));
    Wkrpk[5][0] = ((rk[5][1]*wk[5][2])-(rk[5][2]*wk[5][1]));
    Wkrpk[5][1] = ((rk[5][2]*wk[5][0])-(rk[5][0]*wk[5][2]));
    Wkrpk[5][2] = ((rk[5][0]*wk[5][1])-(rk[5][1]*wk[5][0]));
    Wkrpk[6][0] = ((rk[6][1]*wk[6][2])-(rk[6][2]*wk[6][1]));
    Wkrpk[6][1] = ((rk[6][2]*wk[6][0])-(rk[6][0]*wk[6][2]));
    Wkrpk[6][2] = ((rk[6][0]*wk[6][1])-(rk[6][1]*wk[6][0]));
    Wkrpk[7][0] = ((rk[7][1]*wk[7][2])-(rk[7][2]*wk[7][1]));
    Wkrpk[7][1] = ((rk[7][2]*wk[7][0])-(rk[7][0]*wk[7][2]));
    Wkrpk[7][2] = ((rk[7][0]*wk[7][1])-(rk[7][1]*wk[7][0]));
    Wkrpk[8][0] = ((rk[8][1]*wk[8][2])-(rk[8][2]*wk[8][1]));
    Wkrpk[8][1] = ((rk[8][2]*wk[8][0])-(rk[8][0]*wk[8][2]));
    Wkrpk[8][2] = ((rk[8][0]*wk[8][1])-(rk[8][1]*wk[8][0]));
    Wkrpk[9][0] = ((rk[9][1]*wk[9][2])-(rk[9][2]*wk[9][1]));
    Wkrpk[9][1] = ((rk[9][2]*wk[9][0])-(rk[9][0]*wk[9][2]));
    Wkrpk[9][2] = ((rk[9][0]*wk[9][1])-(rk[9][1]*wk[9][0]));
    Wkrpk[10][0] = ((rk[10][1]*wk[10][2])-(rk[10][2]*wk[10][1]));
    Wkrpk[10][1] = ((rk[10][2]*wk[10][0])-(rk[10][0]*wk[10][2]));
    Wkrpk[10][2] = ((rk[10][0]*wk[10][1])-(rk[10][1]*wk[10][0]));
    Wkrpk[11][0] = ((rk[11][1]*wk[11][2])-(rk[11][2]*wk[11][1]));
    Wkrpk[11][1] = ((rk[11][2]*wk[11][0])-(rk[11][0]*wk[11][2]));
    Wkrpk[11][2] = ((rk[11][0]*wk[11][1])-(rk[11][1]*wk[11][0]));
    Wkrpk[12][0] = ((rk[12][1]*wk[12][2])-(rk[12][2]*wk[12][1]));
    Wkrpk[12][1] = ((rk[12][2]*wk[12][0])-(rk[12][0]*wk[12][2]));
    Wkrpk[12][2] = ((rk[12][0]*wk[12][1])-(rk[12][1]*wk[12][0]));
    Wkrpk[13][0] = ((rk[13][1]*wk[13][2])-(rk[13][2]*wk[13][1]));
    Wkrpk[13][1] = ((rk[13][2]*wk[13][0])-(rk[13][0]*wk[13][2]));
    Wkrpk[13][2] = ((rk[13][0]*wk[13][1])-(rk[13][1]*wk[13][0]));
    IkWk[0][0] = ((ik[0][0][2]*Wik[0][2])+((ik[0][0][0]*Wik[0][0])+(ik[0][0][1]*
      Wik[0][1])));
    IkWk[0][1] = ((ik[0][1][2]*Wik[0][2])+((ik[0][1][0]*Wik[0][0])+(ik[0][1][1]*
      Wik[0][1])));
    IkWk[0][2] = ((ik[0][2][2]*Wik[0][2])+((ik[0][2][0]*Wik[0][0])+(ik[0][2][1]*
      Wik[0][1])));
    WkIkWk[0][0] = ((IkWk[0][2]*Wik[0][1])-(IkWk[0][1]*Wik[0][2]));
    WkIkWk[0][1] = ((IkWk[0][0]*Wik[0][2])-(IkWk[0][2]*Wik[0][0]));
    WkIkWk[0][2] = ((IkWk[0][1]*Wik[0][0])-(IkWk[0][0]*Wik[0][1]));
    IkWk[1][0] = ((ik[1][0][2]*wk[1][2])+((ik[1][0][0]*wk[1][0])+(ik[1][0][1]*
      wk[1][1])));
    IkWk[1][1] = ((ik[1][1][2]*wk[1][2])+((ik[1][1][0]*wk[1][0])+(ik[1][1][1]*
      wk[1][1])));
    IkWk[1][2] = ((ik[1][2][2]*wk[1][2])+((ik[1][2][0]*wk[1][0])+(ik[1][2][1]*
      wk[1][1])));
    WkIkWk[1][0] = ((IkWk[1][2]*wk[1][1])-(IkWk[1][1]*wk[1][2]));
    WkIkWk[1][1] = ((IkWk[1][0]*wk[1][2])-(IkWk[1][2]*wk[1][0]));
    WkIkWk[1][2] = ((IkWk[1][1]*wk[1][0])-(IkWk[1][0]*wk[1][1]));
    IkWk[2][0] = ((ik[2][0][2]*wk[2][2])+((ik[2][0][0]*wk[2][0])+(ik[2][0][1]*
      wk[2][1])));
    IkWk[2][1] = ((ik[2][1][2]*wk[2][2])+((ik[2][1][0]*wk[2][0])+(ik[2][1][1]*
      wk[2][1])));
    IkWk[2][2] = ((ik[2][2][2]*wk[2][2])+((ik[2][2][0]*wk[2][0])+(ik[2][2][1]*
      wk[2][1])));
    WkIkWk[2][0] = ((IkWk[2][2]*wk[2][1])-(IkWk[2][1]*wk[2][2]));
    WkIkWk[2][1] = ((IkWk[2][0]*wk[2][2])-(IkWk[2][2]*wk[2][0]));
    WkIkWk[2][2] = ((IkWk[2][1]*wk[2][0])-(IkWk[2][0]*wk[2][1]));
    IkWk[3][0] = ((ik[3][0][2]*wk[3][2])+((ik[3][0][0]*wk[3][0])+(ik[3][0][1]*
      wk[3][1])));
    IkWk[3][1] = ((ik[3][1][2]*wk[3][2])+((ik[3][1][0]*wk[3][0])+(ik[3][1][1]*
      wk[3][1])));
    IkWk[3][2] = ((ik[3][2][2]*wk[3][2])+((ik[3][2][0]*wk[3][0])+(ik[3][2][1]*
      wk[3][1])));
    WkIkWk[3][0] = ((IkWk[3][2]*wk[3][1])-(IkWk[3][1]*wk[3][2]));
    WkIkWk[3][1] = ((IkWk[3][0]*wk[3][2])-(IkWk[3][2]*wk[3][0]));
    WkIkWk[3][2] = ((IkWk[3][1]*wk[3][0])-(IkWk[3][0]*wk[3][1]));
    IkWk[4][0] = ((ik[4][0][2]*wk[4][2])+((ik[4][0][0]*wk[4][0])+(ik[4][0][1]*
      wk[4][1])));
    IkWk[4][1] = ((ik[4][1][2]*wk[4][2])+((ik[4][1][0]*wk[4][0])+(ik[4][1][1]*
      wk[4][1])));
    IkWk[4][2] = ((ik[4][2][2]*wk[4][2])+((ik[4][2][0]*wk[4][0])+(ik[4][2][1]*
      wk[4][1])));
    WkIkWk[4][0] = ((IkWk[4][2]*wk[4][1])-(IkWk[4][1]*wk[4][2]));
    WkIkWk[4][1] = ((IkWk[4][0]*wk[4][2])-(IkWk[4][2]*wk[4][0]));
    WkIkWk[4][2] = ((IkWk[4][1]*wk[4][0])-(IkWk[4][0]*wk[4][1]));
    IkWk[5][0] = ((ik[5][0][2]*wk[5][2])+((ik[5][0][0]*wk[5][0])+(ik[5][0][1]*
      wk[5][1])));
    IkWk[5][1] = ((ik[5][1][2]*wk[5][2])+((ik[5][1][0]*wk[5][0])+(ik[5][1][1]*
      wk[5][1])));
    IkWk[5][2] = ((ik[5][2][2]*wk[5][2])+((ik[5][2][0]*wk[5][0])+(ik[5][2][1]*
      wk[5][1])));
    WkIkWk[5][0] = ((IkWk[5][2]*wk[5][1])-(IkWk[5][1]*wk[5][2]));
    WkIkWk[5][1] = ((IkWk[5][0]*wk[5][2])-(IkWk[5][2]*wk[5][0]));
    WkIkWk[5][2] = ((IkWk[5][1]*wk[5][0])-(IkWk[5][0]*wk[5][1]));
    IkWk[6][0] = ((ik[6][0][2]*wk[6][2])+((ik[6][0][0]*wk[6][0])+(ik[6][0][1]*
      wk[6][1])));
    IkWk[6][1] = ((ik[6][1][2]*wk[6][2])+((ik[6][1][0]*wk[6][0])+(ik[6][1][1]*
      wk[6][1])));
    IkWk[6][2] = ((ik[6][2][2]*wk[6][2])+((ik[6][2][0]*wk[6][0])+(ik[6][2][1]*
      wk[6][1])));
    WkIkWk[6][0] = ((IkWk[6][2]*wk[6][1])-(IkWk[6][1]*wk[6][2]));
    WkIkWk[6][1] = ((IkWk[6][0]*wk[6][2])-(IkWk[6][2]*wk[6][0]));
    WkIkWk[6][2] = ((IkWk[6][1]*wk[6][0])-(IkWk[6][0]*wk[6][1]));
    IkWk[7][0] = ((ik[7][0][2]*wk[7][2])+((ik[7][0][0]*wk[7][0])+(ik[7][0][1]*
      wk[7][1])));
    IkWk[7][1] = ((ik[7][1][2]*wk[7][2])+((ik[7][1][0]*wk[7][0])+(ik[7][1][1]*
      wk[7][1])));
    IkWk[7][2] = ((ik[7][2][2]*wk[7][2])+((ik[7][2][0]*wk[7][0])+(ik[7][2][1]*
      wk[7][1])));
    WkIkWk[7][0] = ((IkWk[7][2]*wk[7][1])-(IkWk[7][1]*wk[7][2]));
    WkIkWk[7][1] = ((IkWk[7][0]*wk[7][2])-(IkWk[7][2]*wk[7][0]));
    WkIkWk[7][2] = ((IkWk[7][1]*wk[7][0])-(IkWk[7][0]*wk[7][1]));
    IkWk[8][0] = ((ik[8][0][2]*wk[8][2])+((ik[8][0][0]*wk[8][0])+(ik[8][0][1]*
      wk[8][1])));
    IkWk[8][1] = ((ik[8][1][2]*wk[8][2])+((ik[8][1][0]*wk[8][0])+(ik[8][1][1]*
      wk[8][1])));
    IkWk[8][2] = ((ik[8][2][2]*wk[8][2])+((ik[8][2][0]*wk[8][0])+(ik[8][2][1]*
      wk[8][1])));
    WkIkWk[8][0] = ((IkWk[8][2]*wk[8][1])-(IkWk[8][1]*wk[8][2]));
    WkIkWk[8][1] = ((IkWk[8][0]*wk[8][2])-(IkWk[8][2]*wk[8][0]));
    WkIkWk[8][2] = ((IkWk[8][1]*wk[8][0])-(IkWk[8][0]*wk[8][1]));
    IkWk[9][0] = ((ik[9][0][2]*wk[9][2])+((ik[9][0][0]*wk[9][0])+(ik[9][0][1]*
      wk[9][1])));
    IkWk[9][1] = ((ik[9][1][2]*wk[9][2])+((ik[9][1][0]*wk[9][0])+(ik[9][1][1]*
      wk[9][1])));
    IkWk[9][2] = ((ik[9][2][2]*wk[9][2])+((ik[9][2][0]*wk[9][0])+(ik[9][2][1]*
      wk[9][1])));
    WkIkWk[9][0] = ((IkWk[9][2]*wk[9][1])-(IkWk[9][1]*wk[9][2]));
    WkIkWk[9][1] = ((IkWk[9][0]*wk[9][2])-(IkWk[9][2]*wk[9][0]));
    WkIkWk[9][2] = ((IkWk[9][1]*wk[9][0])-(IkWk[9][0]*wk[9][1]));
    IkWk[10][0] = ((ik[10][0][2]*wk[10][2])+((ik[10][0][0]*wk[10][0])+(
      ik[10][0][1]*wk[10][1])));
    IkWk[10][1] = ((ik[10][1][2]*wk[10][2])+((ik[10][1][0]*wk[10][0])+(
      ik[10][1][1]*wk[10][1])));
    IkWk[10][2] = ((ik[10][2][2]*wk[10][2])+((ik[10][2][0]*wk[10][0])+(
      ik[10][2][1]*wk[10][1])));
    WkIkWk[10][0] = ((IkWk[10][2]*wk[10][1])-(IkWk[10][1]*wk[10][2]));
    WkIkWk[10][1] = ((IkWk[10][0]*wk[10][2])-(IkWk[10][2]*wk[10][0]));
    WkIkWk[10][2] = ((IkWk[10][1]*wk[10][0])-(IkWk[10][0]*wk[10][1]));
    IkWk[11][0] = ((ik[11][0][2]*wk[11][2])+((ik[11][0][0]*wk[11][0])+(
      ik[11][0][1]*wk[11][1])));
    IkWk[11][1] = ((ik[11][1][2]*wk[11][2])+((ik[11][1][0]*wk[11][0])+(
      ik[11][1][1]*wk[11][1])));
    IkWk[11][2] = ((ik[11][2][2]*wk[11][2])+((ik[11][2][0]*wk[11][0])+(
      ik[11][2][1]*wk[11][1])));
    WkIkWk[11][0] = ((IkWk[11][2]*wk[11][1])-(IkWk[11][1]*wk[11][2]));
    WkIkWk[11][1] = ((IkWk[11][0]*wk[11][2])-(IkWk[11][2]*wk[11][0]));
    WkIkWk[11][2] = ((IkWk[11][1]*wk[11][0])-(IkWk[11][0]*wk[11][1]));
    IkWk[12][0] = ((ik[12][0][2]*wk[12][2])+((ik[12][0][0]*wk[12][0])+(
      ik[12][0][1]*wk[12][1])));
    IkWk[12][1] = ((ik[12][1][2]*wk[12][2])+((ik[12][1][0]*wk[12][0])+(
      ik[12][1][1]*wk[12][1])));
    IkWk[12][2] = ((ik[12][2][2]*wk[12][2])+((ik[12][2][0]*wk[12][0])+(
      ik[12][2][1]*wk[12][1])));
    WkIkWk[12][0] = ((IkWk[12][2]*wk[12][1])-(IkWk[12][1]*wk[12][2]));
    WkIkWk[12][1] = ((IkWk[12][0]*wk[12][2])-(IkWk[12][2]*wk[12][0]));
    WkIkWk[12][2] = ((IkWk[12][1]*wk[12][0])-(IkWk[12][0]*wk[12][1]));
    IkWk[13][0] = ((ik[13][0][2]*wk[13][2])+((ik[13][0][0]*wk[13][0])+(
      ik[13][0][1]*wk[13][1])));
    IkWk[13][1] = ((ik[13][1][2]*wk[13][2])+((ik[13][1][0]*wk[13][0])+(
      ik[13][1][1]*wk[13][1])));
    IkWk[13][2] = ((ik[13][2][2]*wk[13][2])+((ik[13][2][0]*wk[13][0])+(
      ik[13][2][1]*wk[13][1])));
    WkIkWk[13][0] = ((IkWk[13][2]*wk[13][1])-(IkWk[13][1]*wk[13][2]));
    WkIkWk[13][1] = ((IkWk[13][0]*wk[13][2])-(IkWk[13][2]*wk[13][0]));
    WkIkWk[13][2] = ((IkWk[13][1]*wk[13][0])-(IkWk[13][0]*wk[13][1]));
/*
Compute temporaries for use in SDRHS
*/
    w0w0[0] = (Wik[0][0]*Wik[0][0]);
    w0w0[1] = (wk[1][0]*wk[1][0]);
    w0w0[2] = (wk[2][0]*wk[2][0]);
    w0w0[3] = (wk[3][0]*wk[3][0]);
    w0w0[4] = (wk[4][0]*wk[4][0]);
    w0w0[5] = (wk[5][0]*wk[5][0]);
    w0w0[6] = (wk[6][0]*wk[6][0]);
    w0w0[7] = (wk[7][0]*wk[7][0]);
    w0w0[8] = (wk[8][0]*wk[8][0]);
    w0w0[9] = (wk[9][0]*wk[9][0]);
    w0w0[10] = (wk[10][0]*wk[10][0]);
    w0w0[11] = (wk[11][0]*wk[11][0]);
    w0w0[12] = (wk[12][0]*wk[12][0]);
    w0w0[13] = (wk[13][0]*wk[13][0]);
    w1w1[0] = (Wik[0][1]*Wik[0][1]);
    w1w1[1] = (wk[1][1]*wk[1][1]);
    w1w1[2] = (wk[2][1]*wk[2][1]);
    w1w1[3] = (wk[3][1]*wk[3][1]);
    w1w1[4] = (wk[4][1]*wk[4][1]);
    w1w1[5] = (wk[5][1]*wk[5][1]);
    w1w1[6] = (wk[6][1]*wk[6][1]);
    w1w1[7] = (wk[7][1]*wk[7][1]);
    w1w1[8] = (wk[8][1]*wk[8][1]);
    w1w1[9] = (wk[9][1]*wk[9][1]);
    w1w1[10] = (wk[10][1]*wk[10][1]);
    w1w1[11] = (wk[11][1]*wk[11][1]);
    w1w1[12] = (wk[12][1]*wk[12][1]);
    w1w1[13] = (wk[13][1]*wk[13][1]);
    w2w2[0] = (Wik[0][2]*Wik[0][2]);
    w2w2[1] = (wk[1][2]*wk[1][2]);
    w2w2[2] = (wk[2][2]*wk[2][2]);
    w2w2[3] = (wk[3][2]*wk[3][2]);
    w2w2[4] = (wk[4][2]*wk[4][2]);
    w2w2[5] = (wk[5][2]*wk[5][2]);
    w2w2[6] = (wk[6][2]*wk[6][2]);
    w2w2[7] = (wk[7][2]*wk[7][2]);
    w2w2[8] = (wk[8][2]*wk[8][2]);
    w2w2[9] = (wk[9][2]*wk[9][2]);
    w2w2[10] = (wk[10][2]*wk[10][2]);
    w2w2[11] = (wk[11][2]*wk[11][2]);
    w2w2[12] = (wk[12][2]*wk[12][2]);
    w2w2[13] = (wk[13][2]*wk[13][2]);
    w0w1[0] = (Wik[0][0]*Wik[0][1]);
    w0w1[1] = (wk[1][0]*wk[1][1]);
    w0w1[2] = (wk[2][0]*wk[2][1]);
    w0w1[3] = (wk[3][0]*wk[3][1]);
    w0w1[4] = (wk[4][0]*wk[4][1]);
    w0w1[5] = (wk[5][0]*wk[5][1]);
    w0w1[6] = (wk[6][0]*wk[6][1]);
    w0w1[7] = (wk[7][0]*wk[7][1]);
    w0w1[8] = (wk[8][0]*wk[8][1]);
    w0w1[9] = (wk[9][0]*wk[9][1]);
    w0w1[10] = (wk[10][0]*wk[10][1]);
    w0w1[11] = (wk[11][0]*wk[11][1]);
    w0w1[12] = (wk[12][0]*wk[12][1]);
    w0w1[13] = (wk[13][0]*wk[13][1]);
    w0w2[0] = (Wik[0][0]*Wik[0][2]);
    w0w2[1] = (wk[1][0]*wk[1][2]);
    w0w2[2] = (wk[2][0]*wk[2][2]);
    w0w2[3] = (wk[3][0]*wk[3][2]);
    w0w2[4] = (wk[4][0]*wk[4][2]);
    w0w2[5] = (wk[5][0]*wk[5][2]);
    w0w2[6] = (wk[6][0]*wk[6][2]);
    w0w2[7] = (wk[7][0]*wk[7][2]);
    w0w2[8] = (wk[8][0]*wk[8][2]);
    w0w2[9] = (wk[9][0]*wk[9][2]);
    w0w2[10] = (wk[10][0]*wk[10][2]);
    w0w2[11] = (wk[11][0]*wk[11][2]);
    w0w2[12] = (wk[12][0]*wk[12][2]);
    w0w2[13] = (wk[13][0]*wk[13][2]);
    w1w2[0] = (Wik[0][1]*Wik[0][2]);
    w1w2[1] = (wk[1][1]*wk[1][2]);
    w1w2[2] = (wk[2][1]*wk[2][2]);
    w1w2[3] = (wk[3][1]*wk[3][2]);
    w1w2[4] = (wk[4][1]*wk[4][2]);
    w1w2[5] = (wk[5][1]*wk[5][2]);
    w1w2[6] = (wk[6][1]*wk[6][2]);
    w1w2[7] = (wk[7][1]*wk[7][2]);
    w1w2[8] = (wk[8][1]*wk[8][2]);
    w1w2[9] = (wk[9][1]*wk[9][2]);
    w1w2[10] = (wk[10][1]*wk[10][2]);
    w1w2[11] = (wk[11][1]*wk[11][2]);
    w1w2[12] = (wk[12][1]*wk[12][2]);
    w1w2[13] = (wk[13][1]*wk[13][2]);
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
/*
Compute vnk & vnb (mass center linear velocities in N)
*/
    vnk[0][0] = ((Cik[0][0][2]*Wkrpk[0][2])+((Cik[0][0][0]*Wkrpk[0][0])+(
      Cik[0][0][1]*Wkrpk[0][1])));
    vnk[0][1] = ((Cik[0][1][2]*Wkrpk[0][2])+((Cik[0][1][0]*Wkrpk[0][0])+(
      Cik[0][1][1]*Wkrpk[0][1])));
    vnk[0][2] = ((Cik[0][2][2]*Wkrpk[0][2])+((Cik[0][2][0]*Wkrpk[0][0])+(
      Cik[0][2][1]*Wkrpk[0][1])));
    vnk[1][0] = ((vnk[0][0]+((Cik[0][0][2]*Wirk[1][2])+((Cik[0][0][0]*Wirk[1][0]
      )+(Cik[0][0][1]*Wirk[1][1]))))+((cnk[1][0][2]*Wkrpk[1][2])+((cnk[1][0][0]*
      Wkrpk[1][0])+(cnk[1][0][1]*Wkrpk[1][1]))));
    vnk[1][1] = ((vnk[0][1]+((Cik[0][1][2]*Wirk[1][2])+((Cik[0][1][0]*Wirk[1][0]
      )+(Cik[0][1][1]*Wirk[1][1]))))+((cnk[1][1][2]*Wkrpk[1][2])+((cnk[1][1][0]*
      Wkrpk[1][0])+(cnk[1][1][1]*Wkrpk[1][1]))));
    vnk[1][2] = ((vnk[0][2]+((Cik[0][2][2]*Wirk[1][2])+((Cik[0][2][0]*Wirk[1][0]
      )+(Cik[0][2][1]*Wirk[1][1]))))+((cnk[1][2][2]*Wkrpk[1][2])+((cnk[1][2][0]*
      Wkrpk[1][0])+(cnk[1][2][1]*Wkrpk[1][1]))));
    vnk[2][0] = ((vnk[0][0]+((Cik[0][0][2]*Wirk[2][2])+((Cik[0][0][0]*Wirk[2][0]
      )+(Cik[0][0][1]*Wirk[2][1]))))+((cnk[2][0][2]*Wkrpk[2][2])+((cnk[2][0][0]*
      Wkrpk[2][0])+(cnk[2][0][1]*Wkrpk[2][1]))));
    vnk[2][1] = ((vnk[0][1]+((Cik[0][1][2]*Wirk[2][2])+((Cik[0][1][0]*Wirk[2][0]
      )+(Cik[0][1][1]*Wirk[2][1]))))+((cnk[2][1][2]*Wkrpk[2][2])+((cnk[2][1][0]*
      Wkrpk[2][0])+(cnk[2][1][1]*Wkrpk[2][1]))));
    vnk[2][2] = ((vnk[0][2]+((Cik[0][2][2]*Wirk[2][2])+((Cik[0][2][0]*Wirk[2][0]
      )+(Cik[0][2][1]*Wirk[2][1]))))+((cnk[2][2][2]*Wkrpk[2][2])+((cnk[2][2][0]*
      Wkrpk[2][0])+(cnk[2][2][1]*Wkrpk[2][1]))));
    vnk[3][0] = ((vnk[2][0]+((cnk[2][0][2]*Wirk[3][2])+((cnk[2][0][0]*Wirk[3][0]
      )+(cnk[2][0][1]*Wirk[3][1]))))+((cnk[3][0][2]*Wkrpk[3][2])+((cnk[3][0][0]*
      Wkrpk[3][0])+(cnk[3][0][1]*Wkrpk[3][1]))));
    vnk[3][1] = ((vnk[2][1]+((cnk[2][1][2]*Wirk[3][2])+((cnk[2][1][0]*Wirk[3][0]
      )+(cnk[2][1][1]*Wirk[3][1]))))+((cnk[3][1][2]*Wkrpk[3][2])+((cnk[3][1][0]*
      Wkrpk[3][0])+(cnk[3][1][1]*Wkrpk[3][1]))));
    vnk[3][2] = ((vnk[2][2]+((cnk[2][2][2]*Wirk[3][2])+((cnk[2][2][0]*Wirk[3][0]
      )+(cnk[2][2][1]*Wirk[3][1]))))+((cnk[3][2][2]*Wkrpk[3][2])+((cnk[3][2][0]*
      Wkrpk[3][0])+(cnk[3][2][1]*Wkrpk[3][1]))));
    vnk[4][0] = ((vnk[3][0]+((cnk[3][0][2]*Wirk[4][2])+((cnk[3][0][0]*Wirk[4][0]
      )+(cnk[3][0][1]*Wirk[4][1]))))+((cnk[4][0][2]*Wkrpk[4][2])+((cnk[4][0][0]*
      Wkrpk[4][0])+(cnk[4][0][1]*Wkrpk[4][1]))));
    vnk[4][1] = ((vnk[3][1]+((cnk[3][1][2]*Wirk[4][2])+((cnk[3][1][0]*Wirk[4][0]
      )+(cnk[3][1][1]*Wirk[4][1]))))+((cnk[4][1][2]*Wkrpk[4][2])+((cnk[4][1][0]*
      Wkrpk[4][0])+(cnk[4][1][1]*Wkrpk[4][1]))));
    vnk[4][2] = ((vnk[3][2]+((cnk[3][2][2]*Wirk[4][2])+((cnk[3][2][0]*Wirk[4][0]
      )+(cnk[3][2][1]*Wirk[4][1]))))+((cnk[4][2][2]*Wkrpk[4][2])+((cnk[4][2][0]*
      Wkrpk[4][0])+(cnk[4][2][1]*Wkrpk[4][1]))));
    vnk[5][0] = ((vnk[4][0]+((cnk[4][0][2]*Wirk[5][2])+((cnk[4][0][0]*Wirk[5][0]
      )+(cnk[4][0][1]*Wirk[5][1]))))+((cnk[5][0][2]*Wkrpk[5][2])+((cnk[5][0][0]*
      Wkrpk[5][0])+(cnk[5][0][1]*Wkrpk[5][1]))));
    vnk[5][1] = ((vnk[4][1]+((cnk[4][1][2]*Wirk[5][2])+((cnk[4][1][0]*Wirk[5][0]
      )+(cnk[4][1][1]*Wirk[5][1]))))+((cnk[5][1][2]*Wkrpk[5][2])+((cnk[5][1][0]*
      Wkrpk[5][0])+(cnk[5][1][1]*Wkrpk[5][1]))));
    vnk[5][2] = ((vnk[4][2]+((cnk[4][2][2]*Wirk[5][2])+((cnk[4][2][0]*Wirk[5][0]
      )+(cnk[4][2][1]*Wirk[5][1]))))+((cnk[5][2][2]*Wkrpk[5][2])+((cnk[5][2][0]*
      Wkrpk[5][0])+(cnk[5][2][1]*Wkrpk[5][1]))));
    vnk[6][0] = ((vnk[5][0]+((cnk[5][0][2]*Wirk[6][2])+((cnk[5][0][0]*Wirk[6][0]
      )+(cnk[5][0][1]*Wirk[6][1]))))+((cnk[6][0][2]*Wkrpk[6][2])+((cnk[6][0][0]*
      Wkrpk[6][0])+(cnk[6][0][1]*Wkrpk[6][1]))));
    vnk[6][1] = ((vnk[5][1]+((cnk[5][1][2]*Wirk[6][2])+((cnk[5][1][0]*Wirk[6][0]
      )+(cnk[5][1][1]*Wirk[6][1]))))+((cnk[6][1][2]*Wkrpk[6][2])+((cnk[6][1][0]*
      Wkrpk[6][0])+(cnk[6][1][1]*Wkrpk[6][1]))));
    vnk[6][2] = ((vnk[5][2]+((cnk[5][2][2]*Wirk[6][2])+((cnk[5][2][0]*Wirk[6][0]
      )+(cnk[5][2][1]*Wirk[6][1]))))+((cnk[6][2][2]*Wkrpk[6][2])+((cnk[6][2][0]*
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
    vnk[8][0] = ((vnk[0][0]+((Cik[0][0][2]*Wirk[8][2])+((Cik[0][0][0]*Wirk[8][0]
      )+(Cik[0][0][1]*Wirk[8][1]))))+((cnk[8][0][2]*Wkrpk[8][2])+((cnk[8][0][0]*
      Wkrpk[8][0])+(cnk[8][0][1]*Wkrpk[8][1]))));
    vnk[8][1] = ((vnk[0][1]+((Cik[0][1][2]*Wirk[8][2])+((Cik[0][1][0]*Wirk[8][0]
      )+(Cik[0][1][1]*Wirk[8][1]))))+((cnk[8][1][2]*Wkrpk[8][2])+((cnk[8][1][0]*
      Wkrpk[8][0])+(cnk[8][1][1]*Wkrpk[8][1]))));
    vnk[8][2] = ((vnk[0][2]+((Cik[0][2][2]*Wirk[8][2])+((Cik[0][2][0]*Wirk[8][0]
      )+(Cik[0][2][1]*Wirk[8][1]))))+((cnk[8][2][2]*Wkrpk[8][2])+((cnk[8][2][0]*
      Wkrpk[8][0])+(cnk[8][2][1]*Wkrpk[8][1]))));
    vnk[9][0] = ((vnk[8][0]+((cnk[8][0][2]*Wirk[9][2])+((cnk[8][0][0]*Wirk[9][0]
      )+(cnk[8][0][1]*Wirk[9][1]))))+((cnk[9][0][2]*Wkrpk[9][2])+((cnk[9][0][0]*
      Wkrpk[9][0])+(cnk[9][0][1]*Wkrpk[9][1]))));
    vnk[9][1] = ((vnk[8][1]+((cnk[8][1][2]*Wirk[9][2])+((cnk[8][1][0]*Wirk[9][0]
      )+(cnk[8][1][1]*Wirk[9][1]))))+((cnk[9][1][2]*Wkrpk[9][2])+((cnk[9][1][0]*
      Wkrpk[9][0])+(cnk[9][1][1]*Wkrpk[9][1]))));
    vnk[9][2] = ((vnk[8][2]+((cnk[8][2][2]*Wirk[9][2])+((cnk[8][2][0]*Wirk[9][0]
      )+(cnk[8][2][1]*Wirk[9][1]))))+((cnk[9][2][2]*Wkrpk[9][2])+((cnk[9][2][0]*
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
    vnb[0][0] = vnk[0][0];
    vnb[0][1] = vnk[0][1];
    vnb[0][2] = vnk[0][2];
    vnb[1][0] = vnk[1][0];
    vnb[1][1] = vnk[1][1];
    vnb[1][2] = vnk[1][2];
    vnb[2][0] = vnk[2][0];
    vnb[2][1] = vnk[2][1];
    vnb[2][2] = vnk[2][2];
    vnb[3][0] = vnk[3][0];
    vnb[3][1] = vnk[3][1];
    vnb[3][2] = vnk[3][2];
    vnb[4][0] = vnk[4][0];
    vnb[4][1] = vnk[4][1];
    vnb[4][2] = vnk[4][2];
    vnb[5][0] = vnk[5][0];
    vnb[5][1] = vnk[5][1];
    vnb[5][2] = vnk[5][2];
    vnb[6][0] = vnk[6][0];
    vnb[6][1] = vnk[6][1];
    vnb[6][2] = vnk[6][2];
    vnb[7][0] = vnk[7][0];
    vnb[7][1] = vnk[7][1];
    vnb[7][2] = vnk[7][2];
    vnb[8][0] = vnk[8][0];
    vnb[8][1] = vnk[8][1];
    vnb[8][2] = vnk[8][2];
    vnb[9][0] = vnk[9][0];
    vnb[9][1] = vnk[9][1];
    vnb[9][2] = vnk[9][2];
    vnb[10][0] = vnk[10][0];
    vnb[10][1] = vnk[10][1];
    vnb[10][2] = vnk[10][2];
    vnb[11][0] = vnk[11][0];
    vnb[11][1] = vnk[11][1];
    vnb[11][2] = vnk[11][2];
    vnb[12][0] = vnk[12][0];
    vnb[12][1] = vnk[12][1];
    vnb[12][2] = vnk[12][2];
    vnb[13][0] = vnk[13][0];
    vnb[13][1] = vnk[13][1];
    vnb[13][2] = vnk[13][2];
/*
Compute qdot (kinematical equations)
*/
    qdot[0] = u[0];
    qdot[1] = u[1];
    qdot[2] = u[2];
    qdot[3] = u[3];
    qdot[4] = u[4];
    qdot[5] = u[5];
    qdot[6] = u[6];
    qdot[7] = u[7];
    qdot[8] = u[8];
    qdot[9] = u[9];
    qdot[10] = u[10];
    qdot[11] = u[11];
    qdot[12] = u[12];
    qdot[13] = u[13];
/*
Compute constraint velocity errors
*/
    skipus: ;
/*
Initialize applied forces and torques to zero
*/
    for (i = 0; i < 14; i++) {
        for (j = 0; j < 3; j++) {
            ufk[i][j] = 0.;
            utk[i][j] = 0.;
        }
    }
    for (i = 0; i < 14; i++) {
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
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain 1626 adds/subtracts/negates
                   2211 multiplies
                      3 divides
                   1220 assignments
*/
}

void sdqdot(double oqdot[14])
{
/*
Return position coordinate derivatives for tree joints.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(63,23);
        return;
    }
    for (i = 0; i <= 13; i++) {
        oqdot[i] = qdot[i];
    }
}

void sdu2qdot(double uin[14],
    double oqdot[14])
{
/*
Convert velocities to qdots.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(64,23);
        return;
    }
    for (i = 0; i <= 13; i++) {
        oqdot[i] = uin[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     14 assignments
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
        Wpk[0][0][0] = pin[0][0];
        Wpk[0][0][1] = pin[0][1];
        Wpk[0][0][2] = pin[0][2];
        Wpk[0][1][0] = ((Cik[1][2][0]*pin[0][2])+((Cik[1][0][0]*pin[0][0])+(
          Cik[1][1][0]*pin[0][1])));
        Wpk[0][1][1] = ((Cik[1][2][1]*pin[0][2])+((Cik[1][0][1]*pin[0][0])+(
          Cik[1][1][1]*pin[0][1])));
        Wpk[0][1][2] = ((Cik[1][2][2]*pin[0][2])+((Cik[1][0][2]*pin[0][0])+(
          Cik[1][1][2]*pin[0][1])));
        Wpk[0][2][0] = ((Cik[2][2][0]*pin[0][2])+((Cik[2][0][0]*pin[0][0])+(
          Cik[2][1][0]*pin[0][1])));
        Wpk[0][2][1] = ((Cik[2][2][1]*pin[0][2])+((Cik[2][0][1]*pin[0][0])+(
          Cik[2][1][1]*pin[0][1])));
        Wpk[0][2][2] = ((Cik[2][2][2]*pin[0][2])+((Cik[2][0][2]*pin[0][0])+(
          Cik[2][1][2]*pin[0][1])));
        Wpk[0][3][0] = ((Cik[3][2][0]*Wpk[0][2][2])+((Cik[3][0][0]*Wpk[0][2][0])
          +(Cik[3][1][0]*Wpk[0][2][1])));
        Wpk[0][3][1] = ((Cik[3][2][1]*Wpk[0][2][2])+((Cik[3][0][1]*Wpk[0][2][0])
          +(Cik[3][1][1]*Wpk[0][2][1])));
        Wpk[0][3][2] = ((Cik[3][2][2]*Wpk[0][2][2])+((Cik[3][0][2]*Wpk[0][2][0])
          +(Cik[3][1][2]*Wpk[0][2][1])));
        Wpk[0][4][0] = ((Cik[4][2][0]*Wpk[0][3][2])+((Cik[4][0][0]*Wpk[0][3][0])
          +(Cik[4][1][0]*Wpk[0][3][1])));
        Wpk[0][4][1] = ((Cik[4][2][1]*Wpk[0][3][2])+((Cik[4][0][1]*Wpk[0][3][0])
          +(Cik[4][1][1]*Wpk[0][3][1])));
        Wpk[0][4][2] = ((Cik[4][2][2]*Wpk[0][3][2])+((Cik[4][0][2]*Wpk[0][3][0])
          +(Cik[4][1][2]*Wpk[0][3][1])));
        Wpk[0][5][0] = ((Cik[5][2][0]*Wpk[0][4][2])+((Cik[5][0][0]*Wpk[0][4][0])
          +(Cik[5][1][0]*Wpk[0][4][1])));
        Wpk[0][5][1] = ((Cik[5][2][1]*Wpk[0][4][2])+((Cik[5][0][1]*Wpk[0][4][0])
          +(Cik[5][1][1]*Wpk[0][4][1])));
        Wpk[0][5][2] = ((Cik[5][2][2]*Wpk[0][4][2])+((Cik[5][0][2]*Wpk[0][4][0])
          +(Cik[5][1][2]*Wpk[0][4][1])));
        Wpk[0][6][0] = ((Cik[6][2][0]*Wpk[0][5][2])+((Cik[6][0][0]*Wpk[0][5][0])
          +(Cik[6][1][0]*Wpk[0][5][1])));
        Wpk[0][6][1] = ((Cik[6][2][1]*Wpk[0][5][2])+((Cik[6][0][1]*Wpk[0][5][0])
          +(Cik[6][1][1]*Wpk[0][5][1])));
        Wpk[0][6][2] = ((Cik[6][2][2]*Wpk[0][5][2])+((Cik[6][0][2]*Wpk[0][5][0])
          +(Cik[6][1][2]*Wpk[0][5][1])));
        Wpk[0][7][0] = ((Cik[7][2][0]*Wpk[0][6][2])+((Cik[7][0][0]*Wpk[0][6][0])
          +(Cik[7][1][0]*Wpk[0][6][1])));
        Wpk[0][7][1] = ((Cik[7][2][1]*Wpk[0][6][2])+((Cik[7][0][1]*Wpk[0][6][0])
          +(Cik[7][1][1]*Wpk[0][6][1])));
        Wpk[0][7][2] = ((Cik[7][2][2]*Wpk[0][6][2])+((Cik[7][0][2]*Wpk[0][6][0])
          +(Cik[7][1][2]*Wpk[0][6][1])));
        Wpk[0][8][0] = ((Cik[8][2][0]*pin[0][2])+((Cik[8][0][0]*pin[0][0])+(
          Cik[8][1][0]*pin[0][1])));
        Wpk[0][8][1] = ((Cik[8][2][1]*pin[0][2])+((Cik[8][0][1]*pin[0][0])+(
          Cik[8][1][1]*pin[0][1])));
        Wpk[0][8][2] = ((Cik[8][2][2]*pin[0][2])+((Cik[8][0][2]*pin[0][0])+(
          Cik[8][1][2]*pin[0][1])));
        Wpk[0][9][0] = ((Cik[9][2][0]*Wpk[0][8][2])+((Cik[9][0][0]*Wpk[0][8][0])
          +(Cik[9][1][0]*Wpk[0][8][1])));
        Wpk[0][9][1] = ((Cik[9][2][1]*Wpk[0][8][2])+((Cik[9][0][1]*Wpk[0][8][0])
          +(Cik[9][1][1]*Wpk[0][8][1])));
        Wpk[0][9][2] = ((Cik[9][2][2]*Wpk[0][8][2])+((Cik[9][0][2]*Wpk[0][8][0])
          +(Cik[9][1][2]*Wpk[0][8][1])));
        Wpk[0][10][0] = ((Cik[10][2][0]*Wpk[0][9][2])+((Cik[10][0][0]*
          Wpk[0][9][0])+(Cik[10][1][0]*Wpk[0][9][1])));
        Wpk[0][10][1] = ((Cik[10][2][1]*Wpk[0][9][2])+((Cik[10][0][1]*
          Wpk[0][9][0])+(Cik[10][1][1]*Wpk[0][9][1])));
        Wpk[0][10][2] = ((Cik[10][2][2]*Wpk[0][9][2])+((Cik[10][0][2]*
          Wpk[0][9][0])+(Cik[10][1][2]*Wpk[0][9][1])));
        Wpk[0][11][0] = ((Cik[11][2][0]*Wpk[0][10][2])+((Cik[11][0][0]*
          Wpk[0][10][0])+(Cik[11][1][0]*Wpk[0][10][1])));
        Wpk[0][11][1] = ((Cik[11][2][1]*Wpk[0][10][2])+((Cik[11][0][1]*
          Wpk[0][10][0])+(Cik[11][1][1]*Wpk[0][10][1])));
        Wpk[0][11][2] = ((Cik[11][2][2]*Wpk[0][10][2])+((Cik[11][0][2]*
          Wpk[0][10][0])+(Cik[11][1][2]*Wpk[0][10][1])));
        Wpk[0][12][0] = ((Cik[12][2][0]*Wpk[0][11][2])+((Cik[12][0][0]*
          Wpk[0][11][0])+(Cik[12][1][0]*Wpk[0][11][1])));
        Wpk[0][12][1] = ((Cik[12][2][1]*Wpk[0][11][2])+((Cik[12][0][1]*
          Wpk[0][11][0])+(Cik[12][1][1]*Wpk[0][11][1])));
        Wpk[0][12][2] = ((Cik[12][2][2]*Wpk[0][11][2])+((Cik[12][0][2]*
          Wpk[0][11][0])+(Cik[12][1][2]*Wpk[0][11][1])));
        Wpk[0][13][0] = ((Cik[13][2][0]*Wpk[0][12][2])+((Cik[13][0][0]*
          Wpk[0][12][0])+(Cik[13][1][0]*Wpk[0][12][1])));
        Wpk[0][13][1] = ((Cik[13][2][1]*Wpk[0][12][2])+((Cik[13][0][1]*
          Wpk[0][12][0])+(Cik[13][1][1]*Wpk[0][12][1])));
        Wpk[0][13][2] = ((Cik[13][2][2]*Wpk[0][12][2])+((Cik[13][0][2]*
          Wpk[0][12][0])+(Cik[13][1][2]*Wpk[0][12][1])));
        Wpk[1][1][0] = pin[1][0];
        Wpk[1][1][1] = pin[1][1];
        Wpk[1][1][2] = pin[1][2];
        Wpk[2][2][0] = pin[2][0];
        Wpk[2][2][1] = pin[2][1];
        Wpk[2][2][2] = pin[2][2];
        Wpk[2][3][0] = ((Cik[3][2][0]*pin[2][2])+((Cik[3][0][0]*pin[2][0])+(
          Cik[3][1][0]*pin[2][1])));
        Wpk[2][3][1] = ((Cik[3][2][1]*pin[2][2])+((Cik[3][0][1]*pin[2][0])+(
          Cik[3][1][1]*pin[2][1])));
        Wpk[2][3][2] = ((Cik[3][2][2]*pin[2][2])+((Cik[3][0][2]*pin[2][0])+(
          Cik[3][1][2]*pin[2][1])));
        Wpk[2][4][0] = ((Cik[4][2][0]*Wpk[2][3][2])+((Cik[4][0][0]*Wpk[2][3][0])
          +(Cik[4][1][0]*Wpk[2][3][1])));
        Wpk[2][4][1] = ((Cik[4][2][1]*Wpk[2][3][2])+((Cik[4][0][1]*Wpk[2][3][0])
          +(Cik[4][1][1]*Wpk[2][3][1])));
        Wpk[2][4][2] = ((Cik[4][2][2]*Wpk[2][3][2])+((Cik[4][0][2]*Wpk[2][3][0])
          +(Cik[4][1][2]*Wpk[2][3][1])));
        Wpk[2][5][0] = ((Cik[5][2][0]*Wpk[2][4][2])+((Cik[5][0][0]*Wpk[2][4][0])
          +(Cik[5][1][0]*Wpk[2][4][1])));
        Wpk[2][5][1] = ((Cik[5][2][1]*Wpk[2][4][2])+((Cik[5][0][1]*Wpk[2][4][0])
          +(Cik[5][1][1]*Wpk[2][4][1])));
        Wpk[2][5][2] = ((Cik[5][2][2]*Wpk[2][4][2])+((Cik[5][0][2]*Wpk[2][4][0])
          +(Cik[5][1][2]*Wpk[2][4][1])));
        Wpk[2][6][0] = ((Cik[6][2][0]*Wpk[2][5][2])+((Cik[6][0][0]*Wpk[2][5][0])
          +(Cik[6][1][0]*Wpk[2][5][1])));
        Wpk[2][6][1] = ((Cik[6][2][1]*Wpk[2][5][2])+((Cik[6][0][1]*Wpk[2][5][0])
          +(Cik[6][1][1]*Wpk[2][5][1])));
        Wpk[2][6][2] = ((Cik[6][2][2]*Wpk[2][5][2])+((Cik[6][0][2]*Wpk[2][5][0])
          +(Cik[6][1][2]*Wpk[2][5][1])));
        Wpk[2][7][0] = ((Cik[7][2][0]*Wpk[2][6][2])+((Cik[7][0][0]*Wpk[2][6][0])
          +(Cik[7][1][0]*Wpk[2][6][1])));
        Wpk[2][7][1] = ((Cik[7][2][1]*Wpk[2][6][2])+((Cik[7][0][1]*Wpk[2][6][0])
          +(Cik[7][1][1]*Wpk[2][6][1])));
        Wpk[2][7][2] = ((Cik[7][2][2]*Wpk[2][6][2])+((Cik[7][0][2]*Wpk[2][6][0])
          +(Cik[7][1][2]*Wpk[2][6][1])));
        Wpk[3][3][0] = pin[3][0];
        Wpk[3][3][1] = pin[3][1];
        Wpk[3][3][2] = pin[3][2];
        Wpk[3][4][0] = ((Cik[4][2][0]*pin[3][2])+((Cik[4][0][0]*pin[3][0])+(
          Cik[4][1][0]*pin[3][1])));
        Wpk[3][4][1] = ((Cik[4][2][1]*pin[3][2])+((Cik[4][0][1]*pin[3][0])+(
          Cik[4][1][1]*pin[3][1])));
        Wpk[3][4][2] = ((Cik[4][2][2]*pin[3][2])+((Cik[4][0][2]*pin[3][0])+(
          Cik[4][1][2]*pin[3][1])));
        Wpk[3][5][0] = ((Cik[5][2][0]*Wpk[3][4][2])+((Cik[5][0][0]*Wpk[3][4][0])
          +(Cik[5][1][0]*Wpk[3][4][1])));
        Wpk[3][5][1] = ((Cik[5][2][1]*Wpk[3][4][2])+((Cik[5][0][1]*Wpk[3][4][0])
          +(Cik[5][1][1]*Wpk[3][4][1])));
        Wpk[3][5][2] = ((Cik[5][2][2]*Wpk[3][4][2])+((Cik[5][0][2]*Wpk[3][4][0])
          +(Cik[5][1][2]*Wpk[3][4][1])));
        Wpk[3][6][0] = ((Cik[6][2][0]*Wpk[3][5][2])+((Cik[6][0][0]*Wpk[3][5][0])
          +(Cik[6][1][0]*Wpk[3][5][1])));
        Wpk[3][6][1] = ((Cik[6][2][1]*Wpk[3][5][2])+((Cik[6][0][1]*Wpk[3][5][0])
          +(Cik[6][1][1]*Wpk[3][5][1])));
        Wpk[3][6][2] = ((Cik[6][2][2]*Wpk[3][5][2])+((Cik[6][0][2]*Wpk[3][5][0])
          +(Cik[6][1][2]*Wpk[3][5][1])));
        Wpk[3][7][0] = ((Cik[7][2][0]*Wpk[3][6][2])+((Cik[7][0][0]*Wpk[3][6][0])
          +(Cik[7][1][0]*Wpk[3][6][1])));
        Wpk[3][7][1] = ((Cik[7][2][1]*Wpk[3][6][2])+((Cik[7][0][1]*Wpk[3][6][0])
          +(Cik[7][1][1]*Wpk[3][6][1])));
        Wpk[3][7][2] = ((Cik[7][2][2]*Wpk[3][6][2])+((Cik[7][0][2]*Wpk[3][6][0])
          +(Cik[7][1][2]*Wpk[3][6][1])));
        Wpk[4][4][0] = pin[4][0];
        Wpk[4][4][1] = pin[4][1];
        Wpk[4][4][2] = pin[4][2];
        Wpk[4][5][0] = ((Cik[5][2][0]*pin[4][2])+((Cik[5][0][0]*pin[4][0])+(
          Cik[5][1][0]*pin[4][1])));
        Wpk[4][5][1] = ((Cik[5][2][1]*pin[4][2])+((Cik[5][0][1]*pin[4][0])+(
          Cik[5][1][1]*pin[4][1])));
        Wpk[4][5][2] = ((Cik[5][2][2]*pin[4][2])+((Cik[5][0][2]*pin[4][0])+(
          Cik[5][1][2]*pin[4][1])));
        Wpk[4][6][0] = ((Cik[6][2][0]*Wpk[4][5][2])+((Cik[6][0][0]*Wpk[4][5][0])
          +(Cik[6][1][0]*Wpk[4][5][1])));
        Wpk[4][6][1] = ((Cik[6][2][1]*Wpk[4][5][2])+((Cik[6][0][1]*Wpk[4][5][0])
          +(Cik[6][1][1]*Wpk[4][5][1])));
        Wpk[4][6][2] = ((Cik[6][2][2]*Wpk[4][5][2])+((Cik[6][0][2]*Wpk[4][5][0])
          +(Cik[6][1][2]*Wpk[4][5][1])));
        Wpk[4][7][0] = ((Cik[7][2][0]*Wpk[4][6][2])+((Cik[7][0][0]*Wpk[4][6][0])
          +(Cik[7][1][0]*Wpk[4][6][1])));
        Wpk[4][7][1] = ((Cik[7][2][1]*Wpk[4][6][2])+((Cik[7][0][1]*Wpk[4][6][0])
          +(Cik[7][1][1]*Wpk[4][6][1])));
        Wpk[4][7][2] = ((Cik[7][2][2]*Wpk[4][6][2])+((Cik[7][0][2]*Wpk[4][6][0])
          +(Cik[7][1][2]*Wpk[4][6][1])));
        Wpk[5][5][0] = pin[5][0];
        Wpk[5][5][1] = pin[5][1];
        Wpk[5][5][2] = pin[5][2];
        Wpk[5][6][0] = ((Cik[6][2][0]*pin[5][2])+((Cik[6][0][0]*pin[5][0])+(
          Cik[6][1][0]*pin[5][1])));
        Wpk[5][6][1] = ((Cik[6][2][1]*pin[5][2])+((Cik[6][0][1]*pin[5][0])+(
          Cik[6][1][1]*pin[5][1])));
        Wpk[5][6][2] = ((Cik[6][2][2]*pin[5][2])+((Cik[6][0][2]*pin[5][0])+(
          Cik[6][1][2]*pin[5][1])));
        Wpk[5][7][0] = ((Cik[7][2][0]*Wpk[5][6][2])+((Cik[7][0][0]*Wpk[5][6][0])
          +(Cik[7][1][0]*Wpk[5][6][1])));
        Wpk[5][7][1] = ((Cik[7][2][1]*Wpk[5][6][2])+((Cik[7][0][1]*Wpk[5][6][0])
          +(Cik[7][1][1]*Wpk[5][6][1])));
        Wpk[5][7][2] = ((Cik[7][2][2]*Wpk[5][6][2])+((Cik[7][0][2]*Wpk[5][6][0])
          +(Cik[7][1][2]*Wpk[5][6][1])));
        Wpk[6][6][0] = pin[6][0];
        Wpk[6][6][1] = pin[6][1];
        Wpk[6][6][2] = pin[6][2];
        Wpk[6][7][0] = ((Cik[7][2][0]*pin[6][2])+((Cik[7][0][0]*pin[6][0])+(
          Cik[7][1][0]*pin[6][1])));
        Wpk[6][7][1] = ((Cik[7][2][1]*pin[6][2])+((Cik[7][0][1]*pin[6][0])+(
          Cik[7][1][1]*pin[6][1])));
        Wpk[6][7][2] = ((Cik[7][2][2]*pin[6][2])+((Cik[7][0][2]*pin[6][0])+(
          Cik[7][1][2]*pin[6][1])));
        Wpk[7][7][0] = pin[7][0];
        Wpk[7][7][1] = pin[7][1];
        Wpk[7][7][2] = pin[7][2];
        Wpk[8][8][0] = pin[8][0];
        Wpk[8][8][1] = pin[8][1];
        Wpk[8][8][2] = pin[8][2];
        Wpk[8][9][0] = ((Cik[9][2][0]*pin[8][2])+((Cik[9][0][0]*pin[8][0])+(
          Cik[9][1][0]*pin[8][1])));
        Wpk[8][9][1] = ((Cik[9][2][1]*pin[8][2])+((Cik[9][0][1]*pin[8][0])+(
          Cik[9][1][1]*pin[8][1])));
        Wpk[8][9][2] = ((Cik[9][2][2]*pin[8][2])+((Cik[9][0][2]*pin[8][0])+(
          Cik[9][1][2]*pin[8][1])));
        Wpk[8][10][0] = ((Cik[10][2][0]*Wpk[8][9][2])+((Cik[10][0][0]*
          Wpk[8][9][0])+(Cik[10][1][0]*Wpk[8][9][1])));
        Wpk[8][10][1] = ((Cik[10][2][1]*Wpk[8][9][2])+((Cik[10][0][1]*
          Wpk[8][9][0])+(Cik[10][1][1]*Wpk[8][9][1])));
        Wpk[8][10][2] = ((Cik[10][2][2]*Wpk[8][9][2])+((Cik[10][0][2]*
          Wpk[8][9][0])+(Cik[10][1][2]*Wpk[8][9][1])));
        Wpk[8][11][0] = ((Cik[11][2][0]*Wpk[8][10][2])+((Cik[11][0][0]*
          Wpk[8][10][0])+(Cik[11][1][0]*Wpk[8][10][1])));
        Wpk[8][11][1] = ((Cik[11][2][1]*Wpk[8][10][2])+((Cik[11][0][1]*
          Wpk[8][10][0])+(Cik[11][1][1]*Wpk[8][10][1])));
        Wpk[8][11][2] = ((Cik[11][2][2]*Wpk[8][10][2])+((Cik[11][0][2]*
          Wpk[8][10][0])+(Cik[11][1][2]*Wpk[8][10][1])));
        Wpk[8][12][0] = ((Cik[12][2][0]*Wpk[8][11][2])+((Cik[12][0][0]*
          Wpk[8][11][0])+(Cik[12][1][0]*Wpk[8][11][1])));
        Wpk[8][12][1] = ((Cik[12][2][1]*Wpk[8][11][2])+((Cik[12][0][1]*
          Wpk[8][11][0])+(Cik[12][1][1]*Wpk[8][11][1])));
        Wpk[8][12][2] = ((Cik[12][2][2]*Wpk[8][11][2])+((Cik[12][0][2]*
          Wpk[8][11][0])+(Cik[12][1][2]*Wpk[8][11][1])));
        Wpk[8][13][0] = ((Cik[13][2][0]*Wpk[8][12][2])+((Cik[13][0][0]*
          Wpk[8][12][0])+(Cik[13][1][0]*Wpk[8][12][1])));
        Wpk[8][13][1] = ((Cik[13][2][1]*Wpk[8][12][2])+((Cik[13][0][1]*
          Wpk[8][12][0])+(Cik[13][1][1]*Wpk[8][12][1])));
        Wpk[8][13][2] = ((Cik[13][2][2]*Wpk[8][12][2])+((Cik[13][0][2]*
          Wpk[8][12][0])+(Cik[13][1][2]*Wpk[8][12][1])));
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
        Wpk[12][12][0] = pin[12][0];
        Wpk[12][12][1] = pin[12][1];
        Wpk[12][12][2] = pin[12][2];
        Wpk[12][13][0] = ((Cik[13][2][0]*pin[12][2])+((Cik[13][0][0]*pin[12][0])
          +(Cik[13][1][0]*pin[12][1])));
        Wpk[12][13][1] = ((Cik[13][2][1]*pin[12][2])+((Cik[13][0][1]*pin[12][0])
          +(Cik[13][1][1]*pin[12][1])));
        Wpk[12][13][2] = ((Cik[13][2][2]*pin[12][2])+((Cik[13][0][2]*pin[12][0])
          +(Cik[13][1][2]*pin[12][1])));
        Wpk[13][13][0] = pin[13][0];
        Wpk[13][13][1] = pin[13][1];
        Wpk[13][13][2] = pin[13][2];
/*
Compute Vpk (partial velocities)
*/
        Vpk[0][0][0] = ((pin[0][2]*rk[0][1])-(pin[0][1]*rk[0][2]));
        Vpk[0][0][1] = ((pin[0][0]*rk[0][2])-(pin[0][2]*rk[0][0]));
        Vpk[0][0][2] = ((pin[0][1]*rk[0][0])-(pin[0][0]*rk[0][1]));
        VWri[0][1][0] = (Vpk[0][0][0]+((pin[0][1]*ri[1][2])-(pin[0][2]*ri[1][1])
          ));
        VWri[0][1][1] = (Vpk[0][0][1]+((pin[0][2]*ri[1][0])-(pin[0][0]*ri[1][2])
          ));
        VWri[0][1][2] = (Vpk[0][0][2]+((pin[0][0]*ri[1][1])-(pin[0][1]*ri[1][0])
          ));
        Vpk[0][1][0] = (((Cik[1][2][0]*VWri[0][1][2])+((Cik[1][0][0]*
          VWri[0][1][0])+(Cik[1][1][0]*VWri[0][1][1])))+((rk[1][1]*Wpk[0][1][2])
          -(rk[1][2]*Wpk[0][1][1])));
        Vpk[0][1][1] = (((Cik[1][2][1]*VWri[0][1][2])+((Cik[1][0][1]*
          VWri[0][1][0])+(Cik[1][1][1]*VWri[0][1][1])))+((rk[1][2]*Wpk[0][1][0])
          -(rk[1][0]*Wpk[0][1][2])));
        Vpk[0][1][2] = (((Cik[1][2][2]*VWri[0][1][2])+((Cik[1][0][2]*
          VWri[0][1][0])+(Cik[1][1][2]*VWri[0][1][1])))+((rk[1][0]*Wpk[0][1][1])
          -(rk[1][1]*Wpk[0][1][0])));
        VWri[0][2][0] = (Vpk[0][0][0]+((pin[0][1]*ri[2][2])-(pin[0][2]*ri[2][1])
          ));
        VWri[0][2][1] = (Vpk[0][0][1]+((pin[0][2]*ri[2][0])-(pin[0][0]*ri[2][2])
          ));
        VWri[0][2][2] = (Vpk[0][0][2]+((pin[0][0]*ri[2][1])-(pin[0][1]*ri[2][0])
          ));
        Vpk[0][2][0] = (((Cik[2][2][0]*VWri[0][2][2])+((Cik[2][0][0]*
          VWri[0][2][0])+(Cik[2][1][0]*VWri[0][2][1])))+((rk[2][1]*Wpk[0][2][2])
          -(rk[2][2]*Wpk[0][2][1])));
        Vpk[0][2][1] = (((Cik[2][2][1]*VWri[0][2][2])+((Cik[2][0][1]*
          VWri[0][2][0])+(Cik[2][1][1]*VWri[0][2][1])))+((rk[2][2]*Wpk[0][2][0])
          -(rk[2][0]*Wpk[0][2][2])));
        Vpk[0][2][2] = (((Cik[2][2][2]*VWri[0][2][2])+((Cik[2][0][2]*
          VWri[0][2][0])+(Cik[2][1][2]*VWri[0][2][1])))+((rk[2][0]*Wpk[0][2][1])
          -(rk[2][1]*Wpk[0][2][0])));
        VWri[0][3][0] = (Vpk[0][2][0]+((ri[3][2]*Wpk[0][2][1])-(ri[3][1]*
          Wpk[0][2][2])));
        VWri[0][3][1] = (Vpk[0][2][1]+((ri[3][0]*Wpk[0][2][2])-(ri[3][2]*
          Wpk[0][2][0])));
        VWri[0][3][2] = (Vpk[0][2][2]+((ri[3][1]*Wpk[0][2][0])-(ri[3][0]*
          Wpk[0][2][1])));
        Vpk[0][3][0] = (((Cik[3][2][0]*VWri[0][3][2])+((Cik[3][0][0]*
          VWri[0][3][0])+(Cik[3][1][0]*VWri[0][3][1])))+((rk[3][1]*Wpk[0][3][2])
          -(rk[3][2]*Wpk[0][3][1])));
        Vpk[0][3][1] = (((Cik[3][2][1]*VWri[0][3][2])+((Cik[3][0][1]*
          VWri[0][3][0])+(Cik[3][1][1]*VWri[0][3][1])))+((rk[3][2]*Wpk[0][3][0])
          -(rk[3][0]*Wpk[0][3][2])));
        Vpk[0][3][2] = (((Cik[3][2][2]*VWri[0][3][2])+((Cik[3][0][2]*
          VWri[0][3][0])+(Cik[3][1][2]*VWri[0][3][1])))+((rk[3][0]*Wpk[0][3][1])
          -(rk[3][1]*Wpk[0][3][0])));
        VWri[0][4][0] = (Vpk[0][3][0]+((ri[4][2]*Wpk[0][3][1])-(ri[4][1]*
          Wpk[0][3][2])));
        VWri[0][4][1] = (Vpk[0][3][1]+((ri[4][0]*Wpk[0][3][2])-(ri[4][2]*
          Wpk[0][3][0])));
        VWri[0][4][2] = (Vpk[0][3][2]+((ri[4][1]*Wpk[0][3][0])-(ri[4][0]*
          Wpk[0][3][1])));
        Vpk[0][4][0] = (((Cik[4][2][0]*VWri[0][4][2])+((Cik[4][0][0]*
          VWri[0][4][0])+(Cik[4][1][0]*VWri[0][4][1])))+((rk[4][1]*Wpk[0][4][2])
          -(rk[4][2]*Wpk[0][4][1])));
        Vpk[0][4][1] = (((Cik[4][2][1]*VWri[0][4][2])+((Cik[4][0][1]*
          VWri[0][4][0])+(Cik[4][1][1]*VWri[0][4][1])))+((rk[4][2]*Wpk[0][4][0])
          -(rk[4][0]*Wpk[0][4][2])));
        Vpk[0][4][2] = (((Cik[4][2][2]*VWri[0][4][2])+((Cik[4][0][2]*
          VWri[0][4][0])+(Cik[4][1][2]*VWri[0][4][1])))+((rk[4][0]*Wpk[0][4][1])
          -(rk[4][1]*Wpk[0][4][0])));
        VWri[0][5][0] = (Vpk[0][4][0]+((ri[5][2]*Wpk[0][4][1])-(ri[5][1]*
          Wpk[0][4][2])));
        VWri[0][5][1] = (Vpk[0][4][1]+((ri[5][0]*Wpk[0][4][2])-(ri[5][2]*
          Wpk[0][4][0])));
        VWri[0][5][2] = (Vpk[0][4][2]+((ri[5][1]*Wpk[0][4][0])-(ri[5][0]*
          Wpk[0][4][1])));
        Vpk[0][5][0] = (((Cik[5][2][0]*VWri[0][5][2])+((Cik[5][0][0]*
          VWri[0][5][0])+(Cik[5][1][0]*VWri[0][5][1])))+((rk[5][1]*Wpk[0][5][2])
          -(rk[5][2]*Wpk[0][5][1])));
        Vpk[0][5][1] = (((Cik[5][2][1]*VWri[0][5][2])+((Cik[5][0][1]*
          VWri[0][5][0])+(Cik[5][1][1]*VWri[0][5][1])))+((rk[5][2]*Wpk[0][5][0])
          -(rk[5][0]*Wpk[0][5][2])));
        Vpk[0][5][2] = (((Cik[5][2][2]*VWri[0][5][2])+((Cik[5][0][2]*
          VWri[0][5][0])+(Cik[5][1][2]*VWri[0][5][1])))+((rk[5][0]*Wpk[0][5][1])
          -(rk[5][1]*Wpk[0][5][0])));
        VWri[0][6][0] = (Vpk[0][5][0]+((ri[6][2]*Wpk[0][5][1])-(ri[6][1]*
          Wpk[0][5][2])));
        VWri[0][6][1] = (Vpk[0][5][1]+((ri[6][0]*Wpk[0][5][2])-(ri[6][2]*
          Wpk[0][5][0])));
        VWri[0][6][2] = (Vpk[0][5][2]+((ri[6][1]*Wpk[0][5][0])-(ri[6][0]*
          Wpk[0][5][1])));
        Vpk[0][6][0] = (((Cik[6][2][0]*VWri[0][6][2])+((Cik[6][0][0]*
          VWri[0][6][0])+(Cik[6][1][0]*VWri[0][6][1])))+((rk[6][1]*Wpk[0][6][2])
          -(rk[6][2]*Wpk[0][6][1])));
        Vpk[0][6][1] = (((Cik[6][2][1]*VWri[0][6][2])+((Cik[6][0][1]*
          VWri[0][6][0])+(Cik[6][1][1]*VWri[0][6][1])))+((rk[6][2]*Wpk[0][6][0])
          -(rk[6][0]*Wpk[0][6][2])));
        Vpk[0][6][2] = (((Cik[6][2][2]*VWri[0][6][2])+((Cik[6][0][2]*
          VWri[0][6][0])+(Cik[6][1][2]*VWri[0][6][1])))+((rk[6][0]*Wpk[0][6][1])
          -(rk[6][1]*Wpk[0][6][0])));
        VWri[0][7][0] = (Vpk[0][6][0]+((ri[7][2]*Wpk[0][6][1])-(ri[7][1]*
          Wpk[0][6][2])));
        VWri[0][7][1] = (Vpk[0][6][1]+((ri[7][0]*Wpk[0][6][2])-(ri[7][2]*
          Wpk[0][6][0])));
        VWri[0][7][2] = (Vpk[0][6][2]+((ri[7][1]*Wpk[0][6][0])-(ri[7][0]*
          Wpk[0][6][1])));
        Vpk[0][7][0] = (((Cik[7][2][0]*VWri[0][7][2])+((Cik[7][0][0]*
          VWri[0][7][0])+(Cik[7][1][0]*VWri[0][7][1])))+((rk[7][1]*Wpk[0][7][2])
          -(rk[7][2]*Wpk[0][7][1])));
        Vpk[0][7][1] = (((Cik[7][2][1]*VWri[0][7][2])+((Cik[7][0][1]*
          VWri[0][7][0])+(Cik[7][1][1]*VWri[0][7][1])))+((rk[7][2]*Wpk[0][7][0])
          -(rk[7][0]*Wpk[0][7][2])));
        Vpk[0][7][2] = (((Cik[7][2][2]*VWri[0][7][2])+((Cik[7][0][2]*
          VWri[0][7][0])+(Cik[7][1][2]*VWri[0][7][1])))+((rk[7][0]*Wpk[0][7][1])
          -(rk[7][1]*Wpk[0][7][0])));
        VWri[0][8][0] = (Vpk[0][0][0]+((pin[0][1]*ri[8][2])-(pin[0][2]*ri[8][1])
          ));
        VWri[0][8][1] = (Vpk[0][0][1]+((pin[0][2]*ri[8][0])-(pin[0][0]*ri[8][2])
          ));
        VWri[0][8][2] = (Vpk[0][0][2]+((pin[0][0]*ri[8][1])-(pin[0][1]*ri[8][0])
          ));
        Vpk[0][8][0] = (((Cik[8][2][0]*VWri[0][8][2])+((Cik[8][0][0]*
          VWri[0][8][0])+(Cik[8][1][0]*VWri[0][8][1])))+((rk[8][1]*Wpk[0][8][2])
          -(rk[8][2]*Wpk[0][8][1])));
        Vpk[0][8][1] = (((Cik[8][2][1]*VWri[0][8][2])+((Cik[8][0][1]*
          VWri[0][8][0])+(Cik[8][1][1]*VWri[0][8][1])))+((rk[8][2]*Wpk[0][8][0])
          -(rk[8][0]*Wpk[0][8][2])));
        Vpk[0][8][2] = (((Cik[8][2][2]*VWri[0][8][2])+((Cik[8][0][2]*
          VWri[0][8][0])+(Cik[8][1][2]*VWri[0][8][1])))+((rk[8][0]*Wpk[0][8][1])
          -(rk[8][1]*Wpk[0][8][0])));
        VWri[0][9][0] = (Vpk[0][8][0]+((ri[9][2]*Wpk[0][8][1])-(ri[9][1]*
          Wpk[0][8][2])));
        VWri[0][9][1] = (Vpk[0][8][1]+((ri[9][0]*Wpk[0][8][2])-(ri[9][2]*
          Wpk[0][8][0])));
        VWri[0][9][2] = (Vpk[0][8][2]+((ri[9][1]*Wpk[0][8][0])-(ri[9][0]*
          Wpk[0][8][1])));
        Vpk[0][9][0] = (((Cik[9][2][0]*VWri[0][9][2])+((Cik[9][0][0]*
          VWri[0][9][0])+(Cik[9][1][0]*VWri[0][9][1])))+((rk[9][1]*Wpk[0][9][2])
          -(rk[9][2]*Wpk[0][9][1])));
        Vpk[0][9][1] = (((Cik[9][2][1]*VWri[0][9][2])+((Cik[9][0][1]*
          VWri[0][9][0])+(Cik[9][1][1]*VWri[0][9][1])))+((rk[9][2]*Wpk[0][9][0])
          -(rk[9][0]*Wpk[0][9][2])));
        Vpk[0][9][2] = (((Cik[9][2][2]*VWri[0][9][2])+((Cik[9][0][2]*
          VWri[0][9][0])+(Cik[9][1][2]*VWri[0][9][1])))+((rk[9][0]*Wpk[0][9][1])
          -(rk[9][1]*Wpk[0][9][0])));
        VWri[0][10][0] = (Vpk[0][9][0]+((ri[10][2]*Wpk[0][9][1])-(ri[10][1]*
          Wpk[0][9][2])));
        VWri[0][10][1] = (Vpk[0][9][1]+((ri[10][0]*Wpk[0][9][2])-(ri[10][2]*
          Wpk[0][9][0])));
        VWri[0][10][2] = (Vpk[0][9][2]+((ri[10][1]*Wpk[0][9][0])-(ri[10][0]*
          Wpk[0][9][1])));
        Vpk[0][10][0] = (((Cik[10][2][0]*VWri[0][10][2])+((Cik[10][0][0]*
          VWri[0][10][0])+(Cik[10][1][0]*VWri[0][10][1])))+((rk[10][1]*
          Wpk[0][10][2])-(rk[10][2]*Wpk[0][10][1])));
        Vpk[0][10][1] = (((Cik[10][2][1]*VWri[0][10][2])+((Cik[10][0][1]*
          VWri[0][10][0])+(Cik[10][1][1]*VWri[0][10][1])))+((rk[10][2]*
          Wpk[0][10][0])-(rk[10][0]*Wpk[0][10][2])));
        Vpk[0][10][2] = (((Cik[10][2][2]*VWri[0][10][2])+((Cik[10][0][2]*
          VWri[0][10][0])+(Cik[10][1][2]*VWri[0][10][1])))+((rk[10][0]*
          Wpk[0][10][1])-(rk[10][1]*Wpk[0][10][0])));
        VWri[0][11][0] = (Vpk[0][10][0]+((ri[11][2]*Wpk[0][10][1])-(ri[11][1]*
          Wpk[0][10][2])));
        VWri[0][11][1] = (Vpk[0][10][1]+((ri[11][0]*Wpk[0][10][2])-(ri[11][2]*
          Wpk[0][10][0])));
        VWri[0][11][2] = (Vpk[0][10][2]+((ri[11][1]*Wpk[0][10][0])-(ri[11][0]*
          Wpk[0][10][1])));
        Vpk[0][11][0] = (((Cik[11][2][0]*VWri[0][11][2])+((Cik[11][0][0]*
          VWri[0][11][0])+(Cik[11][1][0]*VWri[0][11][1])))+((rk[11][1]*
          Wpk[0][11][2])-(rk[11][2]*Wpk[0][11][1])));
        Vpk[0][11][1] = (((Cik[11][2][1]*VWri[0][11][2])+((Cik[11][0][1]*
          VWri[0][11][0])+(Cik[11][1][1]*VWri[0][11][1])))+((rk[11][2]*
          Wpk[0][11][0])-(rk[11][0]*Wpk[0][11][2])));
        Vpk[0][11][2] = (((Cik[11][2][2]*VWri[0][11][2])+((Cik[11][0][2]*
          VWri[0][11][0])+(Cik[11][1][2]*VWri[0][11][1])))+((rk[11][0]*
          Wpk[0][11][1])-(rk[11][1]*Wpk[0][11][0])));
        VWri[0][12][0] = (Vpk[0][11][0]+((ri[12][2]*Wpk[0][11][1])-(ri[12][1]*
          Wpk[0][11][2])));
        VWri[0][12][1] = (Vpk[0][11][1]+((ri[12][0]*Wpk[0][11][2])-(ri[12][2]*
          Wpk[0][11][0])));
        VWri[0][12][2] = (Vpk[0][11][2]+((ri[12][1]*Wpk[0][11][0])-(ri[12][0]*
          Wpk[0][11][1])));
        Vpk[0][12][0] = (((Cik[12][2][0]*VWri[0][12][2])+((Cik[12][0][0]*
          VWri[0][12][0])+(Cik[12][1][0]*VWri[0][12][1])))+((rk[12][1]*
          Wpk[0][12][2])-(rk[12][2]*Wpk[0][12][1])));
        Vpk[0][12][1] = (((Cik[12][2][1]*VWri[0][12][2])+((Cik[12][0][1]*
          VWri[0][12][0])+(Cik[12][1][1]*VWri[0][12][1])))+((rk[12][2]*
          Wpk[0][12][0])-(rk[12][0]*Wpk[0][12][2])));
        Vpk[0][12][2] = (((Cik[12][2][2]*VWri[0][12][2])+((Cik[12][0][2]*
          VWri[0][12][0])+(Cik[12][1][2]*VWri[0][12][1])))+((rk[12][0]*
          Wpk[0][12][1])-(rk[12][1]*Wpk[0][12][0])));
        VWri[0][13][0] = (Vpk[0][12][0]+((ri[13][2]*Wpk[0][12][1])-(ri[13][1]*
          Wpk[0][12][2])));
        VWri[0][13][1] = (Vpk[0][12][1]+((ri[13][0]*Wpk[0][12][2])-(ri[13][2]*
          Wpk[0][12][0])));
        VWri[0][13][2] = (Vpk[0][12][2]+((ri[13][1]*Wpk[0][12][0])-(ri[13][0]*
          Wpk[0][12][1])));
        Vpk[0][13][0] = (((Cik[13][2][0]*VWri[0][13][2])+((Cik[13][0][0]*
          VWri[0][13][0])+(Cik[13][1][0]*VWri[0][13][1])))+((rk[13][1]*
          Wpk[0][13][2])-(rk[13][2]*Wpk[0][13][1])));
        Vpk[0][13][1] = (((Cik[13][2][1]*VWri[0][13][2])+((Cik[13][0][1]*
          VWri[0][13][0])+(Cik[13][1][1]*VWri[0][13][1])))+((rk[13][2]*
          Wpk[0][13][0])-(rk[13][0]*Wpk[0][13][2])));
        Vpk[0][13][2] = (((Cik[13][2][2]*VWri[0][13][2])+((Cik[13][0][2]*
          VWri[0][13][0])+(Cik[13][1][2]*VWri[0][13][1])))+((rk[13][0]*
          Wpk[0][13][1])-(rk[13][1]*Wpk[0][13][0])));
        Vpk[1][1][0] = ((pin[1][2]*rk[1][1])-(pin[1][1]*rk[1][2]));
        Vpk[1][1][1] = ((pin[1][0]*rk[1][2])-(pin[1][2]*rk[1][0]));
        Vpk[1][1][2] = ((pin[1][1]*rk[1][0])-(pin[1][0]*rk[1][1]));
        Vpk[2][2][0] = ((pin[2][2]*rk[2][1])-(pin[2][1]*rk[2][2]));
        Vpk[2][2][1] = ((pin[2][0]*rk[2][2])-(pin[2][2]*rk[2][0]));
        Vpk[2][2][2] = ((pin[2][1]*rk[2][0])-(pin[2][0]*rk[2][1]));
        VWri[2][3][0] = (Vpk[2][2][0]+((pin[2][1]*ri[3][2])-(pin[2][2]*ri[3][1])
          ));
        VWri[2][3][1] = (Vpk[2][2][1]+((pin[2][2]*ri[3][0])-(pin[2][0]*ri[3][2])
          ));
        VWri[2][3][2] = (Vpk[2][2][2]+((pin[2][0]*ri[3][1])-(pin[2][1]*ri[3][0])
          ));
        Vpk[2][3][0] = (((Cik[3][2][0]*VWri[2][3][2])+((Cik[3][0][0]*
          VWri[2][3][0])+(Cik[3][1][0]*VWri[2][3][1])))+((rk[3][1]*Wpk[2][3][2])
          -(rk[3][2]*Wpk[2][3][1])));
        Vpk[2][3][1] = (((Cik[3][2][1]*VWri[2][3][2])+((Cik[3][0][1]*
          VWri[2][3][0])+(Cik[3][1][1]*VWri[2][3][1])))+((rk[3][2]*Wpk[2][3][0])
          -(rk[3][0]*Wpk[2][3][2])));
        Vpk[2][3][2] = (((Cik[3][2][2]*VWri[2][3][2])+((Cik[3][0][2]*
          VWri[2][3][0])+(Cik[3][1][2]*VWri[2][3][1])))+((rk[3][0]*Wpk[2][3][1])
          -(rk[3][1]*Wpk[2][3][0])));
        VWri[2][4][0] = (Vpk[2][3][0]+((ri[4][2]*Wpk[2][3][1])-(ri[4][1]*
          Wpk[2][3][2])));
        VWri[2][4][1] = (Vpk[2][3][1]+((ri[4][0]*Wpk[2][3][2])-(ri[4][2]*
          Wpk[2][3][0])));
        VWri[2][4][2] = (Vpk[2][3][2]+((ri[4][1]*Wpk[2][3][0])-(ri[4][0]*
          Wpk[2][3][1])));
        Vpk[2][4][0] = (((Cik[4][2][0]*VWri[2][4][2])+((Cik[4][0][0]*
          VWri[2][4][0])+(Cik[4][1][0]*VWri[2][4][1])))+((rk[4][1]*Wpk[2][4][2])
          -(rk[4][2]*Wpk[2][4][1])));
        Vpk[2][4][1] = (((Cik[4][2][1]*VWri[2][4][2])+((Cik[4][0][1]*
          VWri[2][4][0])+(Cik[4][1][1]*VWri[2][4][1])))+((rk[4][2]*Wpk[2][4][0])
          -(rk[4][0]*Wpk[2][4][2])));
        Vpk[2][4][2] = (((Cik[4][2][2]*VWri[2][4][2])+((Cik[4][0][2]*
          VWri[2][4][0])+(Cik[4][1][2]*VWri[2][4][1])))+((rk[4][0]*Wpk[2][4][1])
          -(rk[4][1]*Wpk[2][4][0])));
        VWri[2][5][0] = (Vpk[2][4][0]+((ri[5][2]*Wpk[2][4][1])-(ri[5][1]*
          Wpk[2][4][2])));
        VWri[2][5][1] = (Vpk[2][4][1]+((ri[5][0]*Wpk[2][4][2])-(ri[5][2]*
          Wpk[2][4][0])));
        VWri[2][5][2] = (Vpk[2][4][2]+((ri[5][1]*Wpk[2][4][0])-(ri[5][0]*
          Wpk[2][4][1])));
        Vpk[2][5][0] = (((Cik[5][2][0]*VWri[2][5][2])+((Cik[5][0][0]*
          VWri[2][5][0])+(Cik[5][1][0]*VWri[2][5][1])))+((rk[5][1]*Wpk[2][5][2])
          -(rk[5][2]*Wpk[2][5][1])));
        Vpk[2][5][1] = (((Cik[5][2][1]*VWri[2][5][2])+((Cik[5][0][1]*
          VWri[2][5][0])+(Cik[5][1][1]*VWri[2][5][1])))+((rk[5][2]*Wpk[2][5][0])
          -(rk[5][0]*Wpk[2][5][2])));
        Vpk[2][5][2] = (((Cik[5][2][2]*VWri[2][5][2])+((Cik[5][0][2]*
          VWri[2][5][0])+(Cik[5][1][2]*VWri[2][5][1])))+((rk[5][0]*Wpk[2][5][1])
          -(rk[5][1]*Wpk[2][5][0])));
        VWri[2][6][0] = (Vpk[2][5][0]+((ri[6][2]*Wpk[2][5][1])-(ri[6][1]*
          Wpk[2][5][2])));
        VWri[2][6][1] = (Vpk[2][5][1]+((ri[6][0]*Wpk[2][5][2])-(ri[6][2]*
          Wpk[2][5][0])));
        VWri[2][6][2] = (Vpk[2][5][2]+((ri[6][1]*Wpk[2][5][0])-(ri[6][0]*
          Wpk[2][5][1])));
        Vpk[2][6][0] = (((Cik[6][2][0]*VWri[2][6][2])+((Cik[6][0][0]*
          VWri[2][6][0])+(Cik[6][1][0]*VWri[2][6][1])))+((rk[6][1]*Wpk[2][6][2])
          -(rk[6][2]*Wpk[2][6][1])));
        Vpk[2][6][1] = (((Cik[6][2][1]*VWri[2][6][2])+((Cik[6][0][1]*
          VWri[2][6][0])+(Cik[6][1][1]*VWri[2][6][1])))+((rk[6][2]*Wpk[2][6][0])
          -(rk[6][0]*Wpk[2][6][2])));
        Vpk[2][6][2] = (((Cik[6][2][2]*VWri[2][6][2])+((Cik[6][0][2]*
          VWri[2][6][0])+(Cik[6][1][2]*VWri[2][6][1])))+((rk[6][0]*Wpk[2][6][1])
          -(rk[6][1]*Wpk[2][6][0])));
        VWri[2][7][0] = (Vpk[2][6][0]+((ri[7][2]*Wpk[2][6][1])-(ri[7][1]*
          Wpk[2][6][2])));
        VWri[2][7][1] = (Vpk[2][6][1]+((ri[7][0]*Wpk[2][6][2])-(ri[7][2]*
          Wpk[2][6][0])));
        VWri[2][7][2] = (Vpk[2][6][2]+((ri[7][1]*Wpk[2][6][0])-(ri[7][0]*
          Wpk[2][6][1])));
        Vpk[2][7][0] = (((Cik[7][2][0]*VWri[2][7][2])+((Cik[7][0][0]*
          VWri[2][7][0])+(Cik[7][1][0]*VWri[2][7][1])))+((rk[7][1]*Wpk[2][7][2])
          -(rk[7][2]*Wpk[2][7][1])));
        Vpk[2][7][1] = (((Cik[7][2][1]*VWri[2][7][2])+((Cik[7][0][1]*
          VWri[2][7][0])+(Cik[7][1][1]*VWri[2][7][1])))+((rk[7][2]*Wpk[2][7][0])
          -(rk[7][0]*Wpk[2][7][2])));
        Vpk[2][7][2] = (((Cik[7][2][2]*VWri[2][7][2])+((Cik[7][0][2]*
          VWri[2][7][0])+(Cik[7][1][2]*VWri[2][7][1])))+((rk[7][0]*Wpk[2][7][1])
          -(rk[7][1]*Wpk[2][7][0])));
        Vpk[3][3][0] = ((pin[3][2]*rk[3][1])-(pin[3][1]*rk[3][2]));
        Vpk[3][3][1] = ((pin[3][0]*rk[3][2])-(pin[3][2]*rk[3][0]));
        Vpk[3][3][2] = ((pin[3][1]*rk[3][0])-(pin[3][0]*rk[3][1]));
        VWri[3][4][0] = (Vpk[3][3][0]+((pin[3][1]*ri[4][2])-(pin[3][2]*ri[4][1])
          ));
        VWri[3][4][1] = (Vpk[3][3][1]+((pin[3][2]*ri[4][0])-(pin[3][0]*ri[4][2])
          ));
        VWri[3][4][2] = (Vpk[3][3][2]+((pin[3][0]*ri[4][1])-(pin[3][1]*ri[4][0])
          ));
        Vpk[3][4][0] = (((Cik[4][2][0]*VWri[3][4][2])+((Cik[4][0][0]*
          VWri[3][4][0])+(Cik[4][1][0]*VWri[3][4][1])))+((rk[4][1]*Wpk[3][4][2])
          -(rk[4][2]*Wpk[3][4][1])));
        Vpk[3][4][1] = (((Cik[4][2][1]*VWri[3][4][2])+((Cik[4][0][1]*
          VWri[3][4][0])+(Cik[4][1][1]*VWri[3][4][1])))+((rk[4][2]*Wpk[3][4][0])
          -(rk[4][0]*Wpk[3][4][2])));
        Vpk[3][4][2] = (((Cik[4][2][2]*VWri[3][4][2])+((Cik[4][0][2]*
          VWri[3][4][0])+(Cik[4][1][2]*VWri[3][4][1])))+((rk[4][0]*Wpk[3][4][1])
          -(rk[4][1]*Wpk[3][4][0])));
        VWri[3][5][0] = (Vpk[3][4][0]+((ri[5][2]*Wpk[3][4][1])-(ri[5][1]*
          Wpk[3][4][2])));
        VWri[3][5][1] = (Vpk[3][4][1]+((ri[5][0]*Wpk[3][4][2])-(ri[5][2]*
          Wpk[3][4][0])));
        VWri[3][5][2] = (Vpk[3][4][2]+((ri[5][1]*Wpk[3][4][0])-(ri[5][0]*
          Wpk[3][4][1])));
        Vpk[3][5][0] = (((Cik[5][2][0]*VWri[3][5][2])+((Cik[5][0][0]*
          VWri[3][5][0])+(Cik[5][1][0]*VWri[3][5][1])))+((rk[5][1]*Wpk[3][5][2])
          -(rk[5][2]*Wpk[3][5][1])));
        Vpk[3][5][1] = (((Cik[5][2][1]*VWri[3][5][2])+((Cik[5][0][1]*
          VWri[3][5][0])+(Cik[5][1][1]*VWri[3][5][1])))+((rk[5][2]*Wpk[3][5][0])
          -(rk[5][0]*Wpk[3][5][2])));
        Vpk[3][5][2] = (((Cik[5][2][2]*VWri[3][5][2])+((Cik[5][0][2]*
          VWri[3][5][0])+(Cik[5][1][2]*VWri[3][5][1])))+((rk[5][0]*Wpk[3][5][1])
          -(rk[5][1]*Wpk[3][5][0])));
        VWri[3][6][0] = (Vpk[3][5][0]+((ri[6][2]*Wpk[3][5][1])-(ri[6][1]*
          Wpk[3][5][2])));
        VWri[3][6][1] = (Vpk[3][5][1]+((ri[6][0]*Wpk[3][5][2])-(ri[6][2]*
          Wpk[3][5][0])));
        VWri[3][6][2] = (Vpk[3][5][2]+((ri[6][1]*Wpk[3][5][0])-(ri[6][0]*
          Wpk[3][5][1])));
        Vpk[3][6][0] = (((Cik[6][2][0]*VWri[3][6][2])+((Cik[6][0][0]*
          VWri[3][6][0])+(Cik[6][1][0]*VWri[3][6][1])))+((rk[6][1]*Wpk[3][6][2])
          -(rk[6][2]*Wpk[3][6][1])));
        Vpk[3][6][1] = (((Cik[6][2][1]*VWri[3][6][2])+((Cik[6][0][1]*
          VWri[3][6][0])+(Cik[6][1][1]*VWri[3][6][1])))+((rk[6][2]*Wpk[3][6][0])
          -(rk[6][0]*Wpk[3][6][2])));
        Vpk[3][6][2] = (((Cik[6][2][2]*VWri[3][6][2])+((Cik[6][0][2]*
          VWri[3][6][0])+(Cik[6][1][2]*VWri[3][6][1])))+((rk[6][0]*Wpk[3][6][1])
          -(rk[6][1]*Wpk[3][6][0])));
        VWri[3][7][0] = (Vpk[3][6][0]+((ri[7][2]*Wpk[3][6][1])-(ri[7][1]*
          Wpk[3][6][2])));
        VWri[3][7][1] = (Vpk[3][6][1]+((ri[7][0]*Wpk[3][6][2])-(ri[7][2]*
          Wpk[3][6][0])));
        VWri[3][7][2] = (Vpk[3][6][2]+((ri[7][1]*Wpk[3][6][0])-(ri[7][0]*
          Wpk[3][6][1])));
        Vpk[3][7][0] = (((Cik[7][2][0]*VWri[3][7][2])+((Cik[7][0][0]*
          VWri[3][7][0])+(Cik[7][1][0]*VWri[3][7][1])))+((rk[7][1]*Wpk[3][7][2])
          -(rk[7][2]*Wpk[3][7][1])));
        Vpk[3][7][1] = (((Cik[7][2][1]*VWri[3][7][2])+((Cik[7][0][1]*
          VWri[3][7][0])+(Cik[7][1][1]*VWri[3][7][1])))+((rk[7][2]*Wpk[3][7][0])
          -(rk[7][0]*Wpk[3][7][2])));
        Vpk[3][7][2] = (((Cik[7][2][2]*VWri[3][7][2])+((Cik[7][0][2]*
          VWri[3][7][0])+(Cik[7][1][2]*VWri[3][7][1])))+((rk[7][0]*Wpk[3][7][1])
          -(rk[7][1]*Wpk[3][7][0])));
        Vpk[4][4][0] = ((pin[4][2]*rk[4][1])-(pin[4][1]*rk[4][2]));
        Vpk[4][4][1] = ((pin[4][0]*rk[4][2])-(pin[4][2]*rk[4][0]));
        Vpk[4][4][2] = ((pin[4][1]*rk[4][0])-(pin[4][0]*rk[4][1]));
        VWri[4][5][0] = (Vpk[4][4][0]+((pin[4][1]*ri[5][2])-(pin[4][2]*ri[5][1])
          ));
        VWri[4][5][1] = (Vpk[4][4][1]+((pin[4][2]*ri[5][0])-(pin[4][0]*ri[5][2])
          ));
        VWri[4][5][2] = (Vpk[4][4][2]+((pin[4][0]*ri[5][1])-(pin[4][1]*ri[5][0])
          ));
        Vpk[4][5][0] = (((Cik[5][2][0]*VWri[4][5][2])+((Cik[5][0][0]*
          VWri[4][5][0])+(Cik[5][1][0]*VWri[4][5][1])))+((rk[5][1]*Wpk[4][5][2])
          -(rk[5][2]*Wpk[4][5][1])));
        Vpk[4][5][1] = (((Cik[5][2][1]*VWri[4][5][2])+((Cik[5][0][1]*
          VWri[4][5][0])+(Cik[5][1][1]*VWri[4][5][1])))+((rk[5][2]*Wpk[4][5][0])
          -(rk[5][0]*Wpk[4][5][2])));
        Vpk[4][5][2] = (((Cik[5][2][2]*VWri[4][5][2])+((Cik[5][0][2]*
          VWri[4][5][0])+(Cik[5][1][2]*VWri[4][5][1])))+((rk[5][0]*Wpk[4][5][1])
          -(rk[5][1]*Wpk[4][5][0])));
        VWri[4][6][0] = (Vpk[4][5][0]+((ri[6][2]*Wpk[4][5][1])-(ri[6][1]*
          Wpk[4][5][2])));
        VWri[4][6][1] = (Vpk[4][5][1]+((ri[6][0]*Wpk[4][5][2])-(ri[6][2]*
          Wpk[4][5][0])));
        VWri[4][6][2] = (Vpk[4][5][2]+((ri[6][1]*Wpk[4][5][0])-(ri[6][0]*
          Wpk[4][5][1])));
        Vpk[4][6][0] = (((Cik[6][2][0]*VWri[4][6][2])+((Cik[6][0][0]*
          VWri[4][6][0])+(Cik[6][1][0]*VWri[4][6][1])))+((rk[6][1]*Wpk[4][6][2])
          -(rk[6][2]*Wpk[4][6][1])));
        Vpk[4][6][1] = (((Cik[6][2][1]*VWri[4][6][2])+((Cik[6][0][1]*
          VWri[4][6][0])+(Cik[6][1][1]*VWri[4][6][1])))+((rk[6][2]*Wpk[4][6][0])
          -(rk[6][0]*Wpk[4][6][2])));
        Vpk[4][6][2] = (((Cik[6][2][2]*VWri[4][6][2])+((Cik[6][0][2]*
          VWri[4][6][0])+(Cik[6][1][2]*VWri[4][6][1])))+((rk[6][0]*Wpk[4][6][1])
          -(rk[6][1]*Wpk[4][6][0])));
        VWri[4][7][0] = (Vpk[4][6][0]+((ri[7][2]*Wpk[4][6][1])-(ri[7][1]*
          Wpk[4][6][2])));
        VWri[4][7][1] = (Vpk[4][6][1]+((ri[7][0]*Wpk[4][6][2])-(ri[7][2]*
          Wpk[4][6][0])));
        VWri[4][7][2] = (Vpk[4][6][2]+((ri[7][1]*Wpk[4][6][0])-(ri[7][0]*
          Wpk[4][6][1])));
        Vpk[4][7][0] = (((Cik[7][2][0]*VWri[4][7][2])+((Cik[7][0][0]*
          VWri[4][7][0])+(Cik[7][1][0]*VWri[4][7][1])))+((rk[7][1]*Wpk[4][7][2])
          -(rk[7][2]*Wpk[4][7][1])));
        Vpk[4][7][1] = (((Cik[7][2][1]*VWri[4][7][2])+((Cik[7][0][1]*
          VWri[4][7][0])+(Cik[7][1][1]*VWri[4][7][1])))+((rk[7][2]*Wpk[4][7][0])
          -(rk[7][0]*Wpk[4][7][2])));
        Vpk[4][7][2] = (((Cik[7][2][2]*VWri[4][7][2])+((Cik[7][0][2]*
          VWri[4][7][0])+(Cik[7][1][2]*VWri[4][7][1])))+((rk[7][0]*Wpk[4][7][1])
          -(rk[7][1]*Wpk[4][7][0])));
        Vpk[5][5][0] = ((pin[5][2]*rk[5][1])-(pin[5][1]*rk[5][2]));
        Vpk[5][5][1] = ((pin[5][0]*rk[5][2])-(pin[5][2]*rk[5][0]));
        Vpk[5][5][2] = ((pin[5][1]*rk[5][0])-(pin[5][0]*rk[5][1]));
        VWri[5][6][0] = (Vpk[5][5][0]+((pin[5][1]*ri[6][2])-(pin[5][2]*ri[6][1])
          ));
        VWri[5][6][1] = (Vpk[5][5][1]+((pin[5][2]*ri[6][0])-(pin[5][0]*ri[6][2])
          ));
        VWri[5][6][2] = (Vpk[5][5][2]+((pin[5][0]*ri[6][1])-(pin[5][1]*ri[6][0])
          ));
        Vpk[5][6][0] = (((Cik[6][2][0]*VWri[5][6][2])+((Cik[6][0][0]*
          VWri[5][6][0])+(Cik[6][1][0]*VWri[5][6][1])))+((rk[6][1]*Wpk[5][6][2])
          -(rk[6][2]*Wpk[5][6][1])));
        Vpk[5][6][1] = (((Cik[6][2][1]*VWri[5][6][2])+((Cik[6][0][1]*
          VWri[5][6][0])+(Cik[6][1][1]*VWri[5][6][1])))+((rk[6][2]*Wpk[5][6][0])
          -(rk[6][0]*Wpk[5][6][2])));
        Vpk[5][6][2] = (((Cik[6][2][2]*VWri[5][6][2])+((Cik[6][0][2]*
          VWri[5][6][0])+(Cik[6][1][2]*VWri[5][6][1])))+((rk[6][0]*Wpk[5][6][1])
          -(rk[6][1]*Wpk[5][6][0])));
        VWri[5][7][0] = (Vpk[5][6][0]+((ri[7][2]*Wpk[5][6][1])-(ri[7][1]*
          Wpk[5][6][2])));
        VWri[5][7][1] = (Vpk[5][6][1]+((ri[7][0]*Wpk[5][6][2])-(ri[7][2]*
          Wpk[5][6][0])));
        VWri[5][7][2] = (Vpk[5][6][2]+((ri[7][1]*Wpk[5][6][0])-(ri[7][0]*
          Wpk[5][6][1])));
        Vpk[5][7][0] = (((Cik[7][2][0]*VWri[5][7][2])+((Cik[7][0][0]*
          VWri[5][7][0])+(Cik[7][1][0]*VWri[5][7][1])))+((rk[7][1]*Wpk[5][7][2])
          -(rk[7][2]*Wpk[5][7][1])));
        Vpk[5][7][1] = (((Cik[7][2][1]*VWri[5][7][2])+((Cik[7][0][1]*
          VWri[5][7][0])+(Cik[7][1][1]*VWri[5][7][1])))+((rk[7][2]*Wpk[5][7][0])
          -(rk[7][0]*Wpk[5][7][2])));
        Vpk[5][7][2] = (((Cik[7][2][2]*VWri[5][7][2])+((Cik[7][0][2]*
          VWri[5][7][0])+(Cik[7][1][2]*VWri[5][7][1])))+((rk[7][0]*Wpk[5][7][1])
          -(rk[7][1]*Wpk[5][7][0])));
        Vpk[6][6][0] = ((pin[6][2]*rk[6][1])-(pin[6][1]*rk[6][2]));
        Vpk[6][6][1] = ((pin[6][0]*rk[6][2])-(pin[6][2]*rk[6][0]));
        Vpk[6][6][2] = ((pin[6][1]*rk[6][0])-(pin[6][0]*rk[6][1]));
        VWri[6][7][0] = (Vpk[6][6][0]+((pin[6][1]*ri[7][2])-(pin[6][2]*ri[7][1])
          ));
        VWri[6][7][1] = (Vpk[6][6][1]+((pin[6][2]*ri[7][0])-(pin[6][0]*ri[7][2])
          ));
        VWri[6][7][2] = (Vpk[6][6][2]+((pin[6][0]*ri[7][1])-(pin[6][1]*ri[7][0])
          ));
        Vpk[6][7][0] = (((Cik[7][2][0]*VWri[6][7][2])+((Cik[7][0][0]*
          VWri[6][7][0])+(Cik[7][1][0]*VWri[6][7][1])))+((rk[7][1]*Wpk[6][7][2])
          -(rk[7][2]*Wpk[6][7][1])));
        Vpk[6][7][1] = (((Cik[7][2][1]*VWri[6][7][2])+((Cik[7][0][1]*
          VWri[6][7][0])+(Cik[7][1][1]*VWri[6][7][1])))+((rk[7][2]*Wpk[6][7][0])
          -(rk[7][0]*Wpk[6][7][2])));
        Vpk[6][7][2] = (((Cik[7][2][2]*VWri[6][7][2])+((Cik[7][0][2]*
          VWri[6][7][0])+(Cik[7][1][2]*VWri[6][7][1])))+((rk[7][0]*Wpk[6][7][1])
          -(rk[7][1]*Wpk[6][7][0])));
        Vpk[7][7][0] = ((pin[7][2]*rk[7][1])-(pin[7][1]*rk[7][2]));
        Vpk[7][7][1] = ((pin[7][0]*rk[7][2])-(pin[7][2]*rk[7][0]));
        Vpk[7][7][2] = ((pin[7][1]*rk[7][0])-(pin[7][0]*rk[7][1]));
        Vpk[8][8][0] = ((pin[8][2]*rk[8][1])-(pin[8][1]*rk[8][2]));
        Vpk[8][8][1] = ((pin[8][0]*rk[8][2])-(pin[8][2]*rk[8][0]));
        Vpk[8][8][2] = ((pin[8][1]*rk[8][0])-(pin[8][0]*rk[8][1]));
        VWri[8][9][0] = (Vpk[8][8][0]+((pin[8][1]*ri[9][2])-(pin[8][2]*ri[9][1])
          ));
        VWri[8][9][1] = (Vpk[8][8][1]+((pin[8][2]*ri[9][0])-(pin[8][0]*ri[9][2])
          ));
        VWri[8][9][2] = (Vpk[8][8][2]+((pin[8][0]*ri[9][1])-(pin[8][1]*ri[9][0])
          ));
        Vpk[8][9][0] = (((Cik[9][2][0]*VWri[8][9][2])+((Cik[9][0][0]*
          VWri[8][9][0])+(Cik[9][1][0]*VWri[8][9][1])))+((rk[9][1]*Wpk[8][9][2])
          -(rk[9][2]*Wpk[8][9][1])));
        Vpk[8][9][1] = (((Cik[9][2][1]*VWri[8][9][2])+((Cik[9][0][1]*
          VWri[8][9][0])+(Cik[9][1][1]*VWri[8][9][1])))+((rk[9][2]*Wpk[8][9][0])
          -(rk[9][0]*Wpk[8][9][2])));
        Vpk[8][9][2] = (((Cik[9][2][2]*VWri[8][9][2])+((Cik[9][0][2]*
          VWri[8][9][0])+(Cik[9][1][2]*VWri[8][9][1])))+((rk[9][0]*Wpk[8][9][1])
          -(rk[9][1]*Wpk[8][9][0])));
        VWri[8][10][0] = (Vpk[8][9][0]+((ri[10][2]*Wpk[8][9][1])-(ri[10][1]*
          Wpk[8][9][2])));
        VWri[8][10][1] = (Vpk[8][9][1]+((ri[10][0]*Wpk[8][9][2])-(ri[10][2]*
          Wpk[8][9][0])));
        VWri[8][10][2] = (Vpk[8][9][2]+((ri[10][1]*Wpk[8][9][0])-(ri[10][0]*
          Wpk[8][9][1])));
        Vpk[8][10][0] = (((Cik[10][2][0]*VWri[8][10][2])+((Cik[10][0][0]*
          VWri[8][10][0])+(Cik[10][1][0]*VWri[8][10][1])))+((rk[10][1]*
          Wpk[8][10][2])-(rk[10][2]*Wpk[8][10][1])));
        Vpk[8][10][1] = (((Cik[10][2][1]*VWri[8][10][2])+((Cik[10][0][1]*
          VWri[8][10][0])+(Cik[10][1][1]*VWri[8][10][1])))+((rk[10][2]*
          Wpk[8][10][0])-(rk[10][0]*Wpk[8][10][2])));
        Vpk[8][10][2] = (((Cik[10][2][2]*VWri[8][10][2])+((Cik[10][0][2]*
          VWri[8][10][0])+(Cik[10][1][2]*VWri[8][10][1])))+((rk[10][0]*
          Wpk[8][10][1])-(rk[10][1]*Wpk[8][10][0])));
        VWri[8][11][0] = (Vpk[8][10][0]+((ri[11][2]*Wpk[8][10][1])-(ri[11][1]*
          Wpk[8][10][2])));
        VWri[8][11][1] = (Vpk[8][10][1]+((ri[11][0]*Wpk[8][10][2])-(ri[11][2]*
          Wpk[8][10][0])));
        VWri[8][11][2] = (Vpk[8][10][2]+((ri[11][1]*Wpk[8][10][0])-(ri[11][0]*
          Wpk[8][10][1])));
        Vpk[8][11][0] = (((Cik[11][2][0]*VWri[8][11][2])+((Cik[11][0][0]*
          VWri[8][11][0])+(Cik[11][1][0]*VWri[8][11][1])))+((rk[11][1]*
          Wpk[8][11][2])-(rk[11][2]*Wpk[8][11][1])));
        Vpk[8][11][1] = (((Cik[11][2][1]*VWri[8][11][2])+((Cik[11][0][1]*
          VWri[8][11][0])+(Cik[11][1][1]*VWri[8][11][1])))+((rk[11][2]*
          Wpk[8][11][0])-(rk[11][0]*Wpk[8][11][2])));
        Vpk[8][11][2] = (((Cik[11][2][2]*VWri[8][11][2])+((Cik[11][0][2]*
          VWri[8][11][0])+(Cik[11][1][2]*VWri[8][11][1])))+((rk[11][0]*
          Wpk[8][11][1])-(rk[11][1]*Wpk[8][11][0])));
        VWri[8][12][0] = (Vpk[8][11][0]+((ri[12][2]*Wpk[8][11][1])-(ri[12][1]*
          Wpk[8][11][2])));
        VWri[8][12][1] = (Vpk[8][11][1]+((ri[12][0]*Wpk[8][11][2])-(ri[12][2]*
          Wpk[8][11][0])));
        VWri[8][12][2] = (Vpk[8][11][2]+((ri[12][1]*Wpk[8][11][0])-(ri[12][0]*
          Wpk[8][11][1])));
        Vpk[8][12][0] = (((Cik[12][2][0]*VWri[8][12][2])+((Cik[12][0][0]*
          VWri[8][12][0])+(Cik[12][1][0]*VWri[8][12][1])))+((rk[12][1]*
          Wpk[8][12][2])-(rk[12][2]*Wpk[8][12][1])));
        Vpk[8][12][1] = (((Cik[12][2][1]*VWri[8][12][2])+((Cik[12][0][1]*
          VWri[8][12][0])+(Cik[12][1][1]*VWri[8][12][1])))+((rk[12][2]*
          Wpk[8][12][0])-(rk[12][0]*Wpk[8][12][2])));
        Vpk[8][12][2] = (((Cik[12][2][2]*VWri[8][12][2])+((Cik[12][0][2]*
          VWri[8][12][0])+(Cik[12][1][2]*VWri[8][12][1])))+((rk[12][0]*
          Wpk[8][12][1])-(rk[12][1]*Wpk[8][12][0])));
        VWri[8][13][0] = (Vpk[8][12][0]+((ri[13][2]*Wpk[8][12][1])-(ri[13][1]*
          Wpk[8][12][2])));
        VWri[8][13][1] = (Vpk[8][12][1]+((ri[13][0]*Wpk[8][12][2])-(ri[13][2]*
          Wpk[8][12][0])));
        VWri[8][13][2] = (Vpk[8][12][2]+((ri[13][1]*Wpk[8][12][0])-(ri[13][0]*
          Wpk[8][12][1])));
        Vpk[8][13][0] = (((Cik[13][2][0]*VWri[8][13][2])+((Cik[13][0][0]*
          VWri[8][13][0])+(Cik[13][1][0]*VWri[8][13][1])))+((rk[13][1]*
          Wpk[8][13][2])-(rk[13][2]*Wpk[8][13][1])));
        Vpk[8][13][1] = (((Cik[13][2][1]*VWri[8][13][2])+((Cik[13][0][1]*
          VWri[8][13][0])+(Cik[13][1][1]*VWri[8][13][1])))+((rk[13][2]*
          Wpk[8][13][0])-(rk[13][0]*Wpk[8][13][2])));
        Vpk[8][13][2] = (((Cik[13][2][2]*VWri[8][13][2])+((Cik[13][0][2]*
          VWri[8][13][0])+(Cik[13][1][2]*VWri[8][13][1])))+((rk[13][0]*
          Wpk[8][13][1])-(rk[13][1]*Wpk[8][13][0])));
        Vpk[9][9][0] = ((pin[9][2]*rk[9][1])-(pin[9][1]*rk[9][2]));
        Vpk[9][9][1] = ((pin[9][0]*rk[9][2])-(pin[9][2]*rk[9][0]));
        Vpk[9][9][2] = ((pin[9][1]*rk[9][0])-(pin[9][0]*rk[9][1]));
        VWri[9][10][0] = (Vpk[9][9][0]+((pin[9][1]*ri[10][2])-(pin[9][2]*
          ri[10][1])));
        VWri[9][10][1] = (Vpk[9][9][1]+((pin[9][2]*ri[10][0])-(pin[9][0]*
          ri[10][2])));
        VWri[9][10][2] = (Vpk[9][9][2]+((pin[9][0]*ri[10][1])-(pin[9][1]*
          ri[10][0])));
        Vpk[9][10][0] = (((Cik[10][2][0]*VWri[9][10][2])+((Cik[10][0][0]*
          VWri[9][10][0])+(Cik[10][1][0]*VWri[9][10][1])))+((rk[10][1]*
          Wpk[9][10][2])-(rk[10][2]*Wpk[9][10][1])));
        Vpk[9][10][1] = (((Cik[10][2][1]*VWri[9][10][2])+((Cik[10][0][1]*
          VWri[9][10][0])+(Cik[10][1][1]*VWri[9][10][1])))+((rk[10][2]*
          Wpk[9][10][0])-(rk[10][0]*Wpk[9][10][2])));
        Vpk[9][10][2] = (((Cik[10][2][2]*VWri[9][10][2])+((Cik[10][0][2]*
          VWri[9][10][0])+(Cik[10][1][2]*VWri[9][10][1])))+((rk[10][0]*
          Wpk[9][10][1])-(rk[10][1]*Wpk[9][10][0])));
        VWri[9][11][0] = (Vpk[9][10][0]+((ri[11][2]*Wpk[9][10][1])-(ri[11][1]*
          Wpk[9][10][2])));
        VWri[9][11][1] = (Vpk[9][10][1]+((ri[11][0]*Wpk[9][10][2])-(ri[11][2]*
          Wpk[9][10][0])));
        VWri[9][11][2] = (Vpk[9][10][2]+((ri[11][1]*Wpk[9][10][0])-(ri[11][0]*
          Wpk[9][10][1])));
        Vpk[9][11][0] = (((Cik[11][2][0]*VWri[9][11][2])+((Cik[11][0][0]*
          VWri[9][11][0])+(Cik[11][1][0]*VWri[9][11][1])))+((rk[11][1]*
          Wpk[9][11][2])-(rk[11][2]*Wpk[9][11][1])));
        Vpk[9][11][1] = (((Cik[11][2][1]*VWri[9][11][2])+((Cik[11][0][1]*
          VWri[9][11][0])+(Cik[11][1][1]*VWri[9][11][1])))+((rk[11][2]*
          Wpk[9][11][0])-(rk[11][0]*Wpk[9][11][2])));
        Vpk[9][11][2] = (((Cik[11][2][2]*VWri[9][11][2])+((Cik[11][0][2]*
          VWri[9][11][0])+(Cik[11][1][2]*VWri[9][11][1])))+((rk[11][0]*
          Wpk[9][11][1])-(rk[11][1]*Wpk[9][11][0])));
        VWri[9][12][0] = (Vpk[9][11][0]+((ri[12][2]*Wpk[9][11][1])-(ri[12][1]*
          Wpk[9][11][2])));
        VWri[9][12][1] = (Vpk[9][11][1]+((ri[12][0]*Wpk[9][11][2])-(ri[12][2]*
          Wpk[9][11][0])));
        VWri[9][12][2] = (Vpk[9][11][2]+((ri[12][1]*Wpk[9][11][0])-(ri[12][0]*
          Wpk[9][11][1])));
        Vpk[9][12][0] = (((Cik[12][2][0]*VWri[9][12][2])+((Cik[12][0][0]*
          VWri[9][12][0])+(Cik[12][1][0]*VWri[9][12][1])))+((rk[12][1]*
          Wpk[9][12][2])-(rk[12][2]*Wpk[9][12][1])));
        Vpk[9][12][1] = (((Cik[12][2][1]*VWri[9][12][2])+((Cik[12][0][1]*
          VWri[9][12][0])+(Cik[12][1][1]*VWri[9][12][1])))+((rk[12][2]*
          Wpk[9][12][0])-(rk[12][0]*Wpk[9][12][2])));
        Vpk[9][12][2] = (((Cik[12][2][2]*VWri[9][12][2])+((Cik[12][0][2]*
          VWri[9][12][0])+(Cik[12][1][2]*VWri[9][12][1])))+((rk[12][0]*
          Wpk[9][12][1])-(rk[12][1]*Wpk[9][12][0])));
        VWri[9][13][0] = (Vpk[9][12][0]+((ri[13][2]*Wpk[9][12][1])-(ri[13][1]*
          Wpk[9][12][2])));
        VWri[9][13][1] = (Vpk[9][12][1]+((ri[13][0]*Wpk[9][12][2])-(ri[13][2]*
          Wpk[9][12][0])));
        VWri[9][13][2] = (Vpk[9][12][2]+((ri[13][1]*Wpk[9][12][0])-(ri[13][0]*
          Wpk[9][12][1])));
        Vpk[9][13][0] = (((Cik[13][2][0]*VWri[9][13][2])+((Cik[13][0][0]*
          VWri[9][13][0])+(Cik[13][1][0]*VWri[9][13][1])))+((rk[13][1]*
          Wpk[9][13][2])-(rk[13][2]*Wpk[9][13][1])));
        Vpk[9][13][1] = (((Cik[13][2][1]*VWri[9][13][2])+((Cik[13][0][1]*
          VWri[9][13][0])+(Cik[13][1][1]*VWri[9][13][1])))+((rk[13][2]*
          Wpk[9][13][0])-(rk[13][0]*Wpk[9][13][2])));
        Vpk[9][13][2] = (((Cik[13][2][2]*VWri[9][13][2])+((Cik[13][0][2]*
          VWri[9][13][0])+(Cik[13][1][2]*VWri[9][13][1])))+((rk[13][0]*
          Wpk[9][13][1])-(rk[13][1]*Wpk[9][13][0])));
        Vpk[10][10][0] = ((pin[10][2]*rk[10][1])-(pin[10][1]*rk[10][2]));
        Vpk[10][10][1] = ((pin[10][0]*rk[10][2])-(pin[10][2]*rk[10][0]));
        Vpk[10][10][2] = ((pin[10][1]*rk[10][0])-(pin[10][0]*rk[10][1]));
        VWri[10][11][0] = (Vpk[10][10][0]+((pin[10][1]*ri[11][2])-(pin[10][2]*
          ri[11][1])));
        VWri[10][11][1] = (Vpk[10][10][1]+((pin[10][2]*ri[11][0])-(pin[10][0]*
          ri[11][2])));
        VWri[10][11][2] = (Vpk[10][10][2]+((pin[10][0]*ri[11][1])-(pin[10][1]*
          ri[11][0])));
        Vpk[10][11][0] = (((Cik[11][2][0]*VWri[10][11][2])+((Cik[11][0][0]*
          VWri[10][11][0])+(Cik[11][1][0]*VWri[10][11][1])))+((rk[11][1]*
          Wpk[10][11][2])-(rk[11][2]*Wpk[10][11][1])));
        Vpk[10][11][1] = (((Cik[11][2][1]*VWri[10][11][2])+((Cik[11][0][1]*
          VWri[10][11][0])+(Cik[11][1][1]*VWri[10][11][1])))+((rk[11][2]*
          Wpk[10][11][0])-(rk[11][0]*Wpk[10][11][2])));
        Vpk[10][11][2] = (((Cik[11][2][2]*VWri[10][11][2])+((Cik[11][0][2]*
          VWri[10][11][0])+(Cik[11][1][2]*VWri[10][11][1])))+((rk[11][0]*
          Wpk[10][11][1])-(rk[11][1]*Wpk[10][11][0])));
        VWri[10][12][0] = (Vpk[10][11][0]+((ri[12][2]*Wpk[10][11][1])-(ri[12][1]
          *Wpk[10][11][2])));
        VWri[10][12][1] = (Vpk[10][11][1]+((ri[12][0]*Wpk[10][11][2])-(ri[12][2]
          *Wpk[10][11][0])));
        VWri[10][12][2] = (Vpk[10][11][2]+((ri[12][1]*Wpk[10][11][0])-(ri[12][0]
          *Wpk[10][11][1])));
        Vpk[10][12][0] = (((Cik[12][2][0]*VWri[10][12][2])+((Cik[12][0][0]*
          VWri[10][12][0])+(Cik[12][1][0]*VWri[10][12][1])))+((rk[12][1]*
          Wpk[10][12][2])-(rk[12][2]*Wpk[10][12][1])));
        Vpk[10][12][1] = (((Cik[12][2][1]*VWri[10][12][2])+((Cik[12][0][1]*
          VWri[10][12][0])+(Cik[12][1][1]*VWri[10][12][1])))+((rk[12][2]*
          Wpk[10][12][0])-(rk[12][0]*Wpk[10][12][2])));
        Vpk[10][12][2] = (((Cik[12][2][2]*VWri[10][12][2])+((Cik[12][0][2]*
          VWri[10][12][0])+(Cik[12][1][2]*VWri[10][12][1])))+((rk[12][0]*
          Wpk[10][12][1])-(rk[12][1]*Wpk[10][12][0])));
        VWri[10][13][0] = (Vpk[10][12][0]+((ri[13][2]*Wpk[10][12][1])-(ri[13][1]
          *Wpk[10][12][2])));
        VWri[10][13][1] = (Vpk[10][12][1]+((ri[13][0]*Wpk[10][12][2])-(ri[13][2]
          *Wpk[10][12][0])));
        VWri[10][13][2] = (Vpk[10][12][2]+((ri[13][1]*Wpk[10][12][0])-(ri[13][0]
          *Wpk[10][12][1])));
        Vpk[10][13][0] = (((Cik[13][2][0]*VWri[10][13][2])+((Cik[13][0][0]*
          VWri[10][13][0])+(Cik[13][1][0]*VWri[10][13][1])))+((rk[13][1]*
          Wpk[10][13][2])-(rk[13][2]*Wpk[10][13][1])));
        Vpk[10][13][1] = (((Cik[13][2][1]*VWri[10][13][2])+((Cik[13][0][1]*
          VWri[10][13][0])+(Cik[13][1][1]*VWri[10][13][1])))+((rk[13][2]*
          Wpk[10][13][0])-(rk[13][0]*Wpk[10][13][2])));
        Vpk[10][13][2] = (((Cik[13][2][2]*VWri[10][13][2])+((Cik[13][0][2]*
          VWri[10][13][0])+(Cik[13][1][2]*VWri[10][13][1])))+((rk[13][0]*
          Wpk[10][13][1])-(rk[13][1]*Wpk[10][13][0])));
        Vpk[11][11][0] = ((pin[11][2]*rk[11][1])-(pin[11][1]*rk[11][2]));
        Vpk[11][11][1] = ((pin[11][0]*rk[11][2])-(pin[11][2]*rk[11][0]));
        Vpk[11][11][2] = ((pin[11][1]*rk[11][0])-(pin[11][0]*rk[11][1]));
        VWri[11][12][0] = (Vpk[11][11][0]+((pin[11][1]*ri[12][2])-(pin[11][2]*
          ri[12][1])));
        VWri[11][12][1] = (Vpk[11][11][1]+((pin[11][2]*ri[12][0])-(pin[11][0]*
          ri[12][2])));
        VWri[11][12][2] = (Vpk[11][11][2]+((pin[11][0]*ri[12][1])-(pin[11][1]*
          ri[12][0])));
        Vpk[11][12][0] = (((Cik[12][2][0]*VWri[11][12][2])+((Cik[12][0][0]*
          VWri[11][12][0])+(Cik[12][1][0]*VWri[11][12][1])))+((rk[12][1]*
          Wpk[11][12][2])-(rk[12][2]*Wpk[11][12][1])));
        Vpk[11][12][1] = (((Cik[12][2][1]*VWri[11][12][2])+((Cik[12][0][1]*
          VWri[11][12][0])+(Cik[12][1][1]*VWri[11][12][1])))+((rk[12][2]*
          Wpk[11][12][0])-(rk[12][0]*Wpk[11][12][2])));
        Vpk[11][12][2] = (((Cik[12][2][2]*VWri[11][12][2])+((Cik[12][0][2]*
          VWri[11][12][0])+(Cik[12][1][2]*VWri[11][12][1])))+((rk[12][0]*
          Wpk[11][12][1])-(rk[12][1]*Wpk[11][12][0])));
        VWri[11][13][0] = (Vpk[11][12][0]+((ri[13][2]*Wpk[11][12][1])-(ri[13][1]
          *Wpk[11][12][2])));
        VWri[11][13][1] = (Vpk[11][12][1]+((ri[13][0]*Wpk[11][12][2])-(ri[13][2]
          *Wpk[11][12][0])));
        VWri[11][13][2] = (Vpk[11][12][2]+((ri[13][1]*Wpk[11][12][0])-(ri[13][0]
          *Wpk[11][12][1])));
        Vpk[11][13][0] = (((Cik[13][2][0]*VWri[11][13][2])+((Cik[13][0][0]*
          VWri[11][13][0])+(Cik[13][1][0]*VWri[11][13][1])))+((rk[13][1]*
          Wpk[11][13][2])-(rk[13][2]*Wpk[11][13][1])));
        Vpk[11][13][1] = (((Cik[13][2][1]*VWri[11][13][2])+((Cik[13][0][1]*
          VWri[11][13][0])+(Cik[13][1][1]*VWri[11][13][1])))+((rk[13][2]*
          Wpk[11][13][0])-(rk[13][0]*Wpk[11][13][2])));
        Vpk[11][13][2] = (((Cik[13][2][2]*VWri[11][13][2])+((Cik[13][0][2]*
          VWri[11][13][0])+(Cik[13][1][2]*VWri[11][13][1])))+((rk[13][0]*
          Wpk[11][13][1])-(rk[13][1]*Wpk[11][13][0])));
        Vpk[12][12][0] = ((pin[12][2]*rk[12][1])-(pin[12][1]*rk[12][2]));
        Vpk[12][12][1] = ((pin[12][0]*rk[12][2])-(pin[12][2]*rk[12][0]));
        Vpk[12][12][2] = ((pin[12][1]*rk[12][0])-(pin[12][0]*rk[12][1]));
        VWri[12][13][0] = (Vpk[12][12][0]+((pin[12][1]*ri[13][2])-(pin[12][2]*
          ri[13][1])));
        VWri[12][13][1] = (Vpk[12][12][1]+((pin[12][2]*ri[13][0])-(pin[12][0]*
          ri[13][2])));
        VWri[12][13][2] = (Vpk[12][12][2]+((pin[12][0]*ri[13][1])-(pin[12][1]*
          ri[13][0])));
        Vpk[12][13][0] = (((Cik[13][2][0]*VWri[12][13][2])+((Cik[13][0][0]*
          VWri[12][13][0])+(Cik[13][1][0]*VWri[12][13][1])))+((rk[13][1]*
          Wpk[12][13][2])-(rk[13][2]*Wpk[12][13][1])));
        Vpk[12][13][1] = (((Cik[13][2][1]*VWri[12][13][2])+((Cik[13][0][1]*
          VWri[12][13][0])+(Cik[13][1][1]*VWri[12][13][1])))+((rk[13][2]*
          Wpk[12][13][0])-(rk[13][0]*Wpk[12][13][2])));
        Vpk[12][13][2] = (((Cik[13][2][2]*VWri[12][13][2])+((Cik[13][0][2]*
          VWri[12][13][0])+(Cik[13][1][2]*VWri[12][13][1])))+((rk[13][0]*
          Wpk[12][13][1])-(rk[13][1]*Wpk[12][13][0])));
        Vpk[13][13][0] = ((pin[13][2]*rk[13][1])-(pin[13][1]*rk[13][2]));
        Vpk[13][13][1] = ((pin[13][0]*rk[13][2])-(pin[13][2]*rk[13][0]));
        Vpk[13][13][2] = ((pin[13][1]*rk[13][0])-(pin[13][0]*rk[13][1]));
        vpkflg = 1;
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain 1074 adds/subtracts/negates
                   1374 multiplies
                      0 divides
                    471 assignments
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
        Otk[1][0] = ((Wik[1][2]*wk[1][1])-(Wik[1][1]*wk[1][2]));
        Otk[1][1] = ((Wik[1][0]*wk[1][2])-(Wik[1][2]*wk[1][0]));
        Otk[1][2] = ((Wik[1][1]*wk[1][0])-(Wik[1][0]*wk[1][1]));
        Otk[2][0] = ((Wik[2][2]*wk[2][1])-(Wik[2][1]*wk[2][2]));
        Otk[2][1] = ((Wik[2][0]*wk[2][2])-(Wik[2][2]*wk[2][0]));
        Otk[2][2] = ((Wik[2][1]*wk[2][0])-(Wik[2][0]*wk[2][1]));
        Otk[3][0] = (((Cik[3][2][0]*Otk[2][2])+((Cik[3][0][0]*Otk[2][0])+(
          Cik[3][1][0]*Otk[2][1])))+((Wik[3][2]*wk[3][1])-(Wik[3][1]*wk[3][2])))
          ;
        Otk[3][1] = (((Cik[3][2][1]*Otk[2][2])+((Cik[3][0][1]*Otk[2][0])+(
          Cik[3][1][1]*Otk[2][1])))+((Wik[3][0]*wk[3][2])-(Wik[3][2]*wk[3][0])))
          ;
        Otk[3][2] = (((Cik[3][2][2]*Otk[2][2])+((Cik[3][0][2]*Otk[2][0])+(
          Cik[3][1][2]*Otk[2][1])))+((Wik[3][1]*wk[3][0])-(Wik[3][0]*wk[3][1])))
          ;
        Otk[4][0] = (((Cik[4][2][0]*Otk[3][2])+((Cik[4][0][0]*Otk[3][0])+(
          Cik[4][1][0]*Otk[3][1])))+((Wik[4][2]*wk[4][1])-(Wik[4][1]*wk[4][2])))
          ;
        Otk[4][1] = (((Cik[4][2][1]*Otk[3][2])+((Cik[4][0][1]*Otk[3][0])+(
          Cik[4][1][1]*Otk[3][1])))+((Wik[4][0]*wk[4][2])-(Wik[4][2]*wk[4][0])))
          ;
        Otk[4][2] = (((Cik[4][2][2]*Otk[3][2])+((Cik[4][0][2]*Otk[3][0])+(
          Cik[4][1][2]*Otk[3][1])))+((Wik[4][1]*wk[4][0])-(Wik[4][0]*wk[4][1])))
          ;
        Otk[5][0] = (((Cik[5][2][0]*Otk[4][2])+((Cik[5][0][0]*Otk[4][0])+(
          Cik[5][1][0]*Otk[4][1])))+((Wik[5][2]*wk[5][1])-(Wik[5][1]*wk[5][2])))
          ;
        Otk[5][1] = (((Cik[5][2][1]*Otk[4][2])+((Cik[5][0][1]*Otk[4][0])+(
          Cik[5][1][1]*Otk[4][1])))+((Wik[5][0]*wk[5][2])-(Wik[5][2]*wk[5][0])))
          ;
        Otk[5][2] = (((Cik[5][2][2]*Otk[4][2])+((Cik[5][0][2]*Otk[4][0])+(
          Cik[5][1][2]*Otk[4][1])))+((Wik[5][1]*wk[5][0])-(Wik[5][0]*wk[5][1])))
          ;
        Otk[6][0] = (((Cik[6][2][0]*Otk[5][2])+((Cik[6][0][0]*Otk[5][0])+(
          Cik[6][1][0]*Otk[5][1])))+((Wik[6][2]*wk[6][1])-(Wik[6][1]*wk[6][2])))
          ;
        Otk[6][1] = (((Cik[6][2][1]*Otk[5][2])+((Cik[6][0][1]*Otk[5][0])+(
          Cik[6][1][1]*Otk[5][1])))+((Wik[6][0]*wk[6][2])-(Wik[6][2]*wk[6][0])))
          ;
        Otk[6][2] = (((Cik[6][2][2]*Otk[5][2])+((Cik[6][0][2]*Otk[5][0])+(
          Cik[6][1][2]*Otk[5][1])))+((Wik[6][1]*wk[6][0])-(Wik[6][0]*wk[6][1])))
          ;
        Otk[7][0] = (((Cik[7][2][0]*Otk[6][2])+((Cik[7][0][0]*Otk[6][0])+(
          Cik[7][1][0]*Otk[6][1])))+((Wik[7][2]*wk[7][1])-(Wik[7][1]*wk[7][2])))
          ;
        Otk[7][1] = (((Cik[7][2][1]*Otk[6][2])+((Cik[7][0][1]*Otk[6][0])+(
          Cik[7][1][1]*Otk[6][1])))+((Wik[7][0]*wk[7][2])-(Wik[7][2]*wk[7][0])))
          ;
        Otk[7][2] = (((Cik[7][2][2]*Otk[6][2])+((Cik[7][0][2]*Otk[6][0])+(
          Cik[7][1][2]*Otk[6][1])))+((Wik[7][1]*wk[7][0])-(Wik[7][0]*wk[7][1])))
          ;
        Otk[8][0] = ((Wik[8][2]*wk[8][1])-(Wik[8][1]*wk[8][2]));
        Otk[8][1] = ((Wik[8][0]*wk[8][2])-(Wik[8][2]*wk[8][0]));
        Otk[8][2] = ((Wik[8][1]*wk[8][0])-(Wik[8][0]*wk[8][1]));
        Otk[9][0] = (((Cik[9][2][0]*Otk[8][2])+((Cik[9][0][0]*Otk[8][0])+(
          Cik[9][1][0]*Otk[8][1])))+((Wik[9][2]*wk[9][1])-(Wik[9][1]*wk[9][2])))
          ;
        Otk[9][1] = (((Cik[9][2][1]*Otk[8][2])+((Cik[9][0][1]*Otk[8][0])+(
          Cik[9][1][1]*Otk[8][1])))+((Wik[9][0]*wk[9][2])-(Wik[9][2]*wk[9][0])))
          ;
        Otk[9][2] = (((Cik[9][2][2]*Otk[8][2])+((Cik[9][0][2]*Otk[8][0])+(
          Cik[9][1][2]*Otk[8][1])))+((Wik[9][1]*wk[9][0])-(Wik[9][0]*wk[9][1])))
          ;
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
/*
Compute Atk (inertial linear acceleration)
*/
        Atk[0][0] = ((Wik[0][1]*Wkrpk[0][2])-(Wik[0][2]*Wkrpk[0][1]));
        Atk[0][1] = ((Wik[0][2]*Wkrpk[0][0])-(Wik[0][0]*Wkrpk[0][2]));
        Atk[0][2] = ((Wik[0][0]*Wkrpk[0][1])-(Wik[0][1]*Wkrpk[0][0]));
        AiOiWi[1][0] = (Atk[0][0]+((Wik[0][1]*Wirk[1][2])-(Wik[0][2]*Wirk[1][1])
          ));
        AiOiWi[1][1] = (Atk[0][1]+((Wik[0][2]*Wirk[1][0])-(Wik[0][0]*Wirk[1][2])
          ));
        AiOiWi[1][2] = (Atk[0][2]+((Wik[0][0]*Wirk[1][1])-(Wik[0][1]*Wirk[1][0])
          ));
        Atk[1][0] = (((AiOiWi[1][2]*Cik[1][2][0])+((AiOiWi[1][0]*Cik[1][0][0])+(
          AiOiWi[1][1]*Cik[1][1][0])))+(((Otk[1][2]*rk[1][1])-(Otk[1][1]*
          rk[1][2]))+((wk[1][1]*Wkrpk[1][2])-(wk[1][2]*Wkrpk[1][1]))));
        Atk[1][1] = (((AiOiWi[1][2]*Cik[1][2][1])+((AiOiWi[1][0]*Cik[1][0][1])+(
          AiOiWi[1][1]*Cik[1][1][1])))+(((Otk[1][0]*rk[1][2])-(Otk[1][2]*
          rk[1][0]))+((wk[1][2]*Wkrpk[1][0])-(wk[1][0]*Wkrpk[1][2]))));
        Atk[1][2] = (((AiOiWi[1][2]*Cik[1][2][2])+((AiOiWi[1][0]*Cik[1][0][2])+(
          AiOiWi[1][1]*Cik[1][1][2])))+(((Otk[1][1]*rk[1][0])-(Otk[1][0]*
          rk[1][1]))+((wk[1][0]*Wkrpk[1][1])-(wk[1][1]*Wkrpk[1][0]))));
        AiOiWi[2][0] = (Atk[0][0]+((Wik[0][1]*Wirk[2][2])-(Wik[0][2]*Wirk[2][1])
          ));
        AiOiWi[2][1] = (Atk[0][1]+((Wik[0][2]*Wirk[2][0])-(Wik[0][0]*Wirk[2][2])
          ));
        AiOiWi[2][2] = (Atk[0][2]+((Wik[0][0]*Wirk[2][1])-(Wik[0][1]*Wirk[2][0])
          ));
        Atk[2][0] = (((AiOiWi[2][2]*Cik[2][2][0])+((AiOiWi[2][0]*Cik[2][0][0])+(
          AiOiWi[2][1]*Cik[2][1][0])))+(((Otk[2][2]*rk[2][1])-(Otk[2][1]*
          rk[2][2]))+((wk[2][1]*Wkrpk[2][2])-(wk[2][2]*Wkrpk[2][1]))));
        Atk[2][1] = (((AiOiWi[2][2]*Cik[2][2][1])+((AiOiWi[2][0]*Cik[2][0][1])+(
          AiOiWi[2][1]*Cik[2][1][1])))+(((Otk[2][0]*rk[2][2])-(Otk[2][2]*
          rk[2][0]))+((wk[2][2]*Wkrpk[2][0])-(wk[2][0]*Wkrpk[2][2]))));
        Atk[2][2] = (((AiOiWi[2][2]*Cik[2][2][2])+((AiOiWi[2][0]*Cik[2][0][2])+(
          AiOiWi[2][1]*Cik[2][1][2])))+(((Otk[2][1]*rk[2][0])-(Otk[2][0]*
          rk[2][1]))+((wk[2][0]*Wkrpk[2][1])-(wk[2][1]*Wkrpk[2][0]))));
        AiOiWi[3][0] = (Atk[2][0]+(((Otk[2][1]*ri[3][2])-(Otk[2][2]*ri[3][1]))+(
          (Wirk[3][2]*wk[2][1])-(Wirk[3][1]*wk[2][2]))));
        AiOiWi[3][1] = (Atk[2][1]+(((Otk[2][2]*ri[3][0])-(Otk[2][0]*ri[3][2]))+(
          (Wirk[3][0]*wk[2][2])-(Wirk[3][2]*wk[2][0]))));
        AiOiWi[3][2] = (Atk[2][2]+(((Otk[2][0]*ri[3][1])-(Otk[2][1]*ri[3][0]))+(
          (Wirk[3][1]*wk[2][0])-(Wirk[3][0]*wk[2][1]))));
        Atk[3][0] = (((AiOiWi[3][2]*Cik[3][2][0])+((AiOiWi[3][0]*Cik[3][0][0])+(
          AiOiWi[3][1]*Cik[3][1][0])))+(((Otk[3][2]*rk[3][1])-(Otk[3][1]*
          rk[3][2]))+((wk[3][1]*Wkrpk[3][2])-(wk[3][2]*Wkrpk[3][1]))));
        Atk[3][1] = (((AiOiWi[3][2]*Cik[3][2][1])+((AiOiWi[3][0]*Cik[3][0][1])+(
          AiOiWi[3][1]*Cik[3][1][1])))+(((Otk[3][0]*rk[3][2])-(Otk[3][2]*
          rk[3][0]))+((wk[3][2]*Wkrpk[3][0])-(wk[3][0]*Wkrpk[3][2]))));
        Atk[3][2] = (((AiOiWi[3][2]*Cik[3][2][2])+((AiOiWi[3][0]*Cik[3][0][2])+(
          AiOiWi[3][1]*Cik[3][1][2])))+(((Otk[3][1]*rk[3][0])-(Otk[3][0]*
          rk[3][1]))+((wk[3][0]*Wkrpk[3][1])-(wk[3][1]*Wkrpk[3][0]))));
        AiOiWi[4][0] = (Atk[3][0]+(((Otk[3][1]*ri[4][2])-(Otk[3][2]*ri[4][1]))+(
          (Wirk[4][2]*wk[3][1])-(Wirk[4][1]*wk[3][2]))));
        AiOiWi[4][1] = (Atk[3][1]+(((Otk[3][2]*ri[4][0])-(Otk[3][0]*ri[4][2]))+(
          (Wirk[4][0]*wk[3][2])-(Wirk[4][2]*wk[3][0]))));
        AiOiWi[4][2] = (Atk[3][2]+(((Otk[3][0]*ri[4][1])-(Otk[3][1]*ri[4][0]))+(
          (Wirk[4][1]*wk[3][0])-(Wirk[4][0]*wk[3][1]))));
        Atk[4][0] = (((AiOiWi[4][2]*Cik[4][2][0])+((AiOiWi[4][0]*Cik[4][0][0])+(
          AiOiWi[4][1]*Cik[4][1][0])))+(((Otk[4][2]*rk[4][1])-(Otk[4][1]*
          rk[4][2]))+((wk[4][1]*Wkrpk[4][2])-(wk[4][2]*Wkrpk[4][1]))));
        Atk[4][1] = (((AiOiWi[4][2]*Cik[4][2][1])+((AiOiWi[4][0]*Cik[4][0][1])+(
          AiOiWi[4][1]*Cik[4][1][1])))+(((Otk[4][0]*rk[4][2])-(Otk[4][2]*
          rk[4][0]))+((wk[4][2]*Wkrpk[4][0])-(wk[4][0]*Wkrpk[4][2]))));
        Atk[4][2] = (((AiOiWi[4][2]*Cik[4][2][2])+((AiOiWi[4][0]*Cik[4][0][2])+(
          AiOiWi[4][1]*Cik[4][1][2])))+(((Otk[4][1]*rk[4][0])-(Otk[4][0]*
          rk[4][1]))+((wk[4][0]*Wkrpk[4][1])-(wk[4][1]*Wkrpk[4][0]))));
        AiOiWi[5][0] = (Atk[4][0]+(((Otk[4][1]*ri[5][2])-(Otk[4][2]*ri[5][1]))+(
          (Wirk[5][2]*wk[4][1])-(Wirk[5][1]*wk[4][2]))));
        AiOiWi[5][1] = (Atk[4][1]+(((Otk[4][2]*ri[5][0])-(Otk[4][0]*ri[5][2]))+(
          (Wirk[5][0]*wk[4][2])-(Wirk[5][2]*wk[4][0]))));
        AiOiWi[5][2] = (Atk[4][2]+(((Otk[4][0]*ri[5][1])-(Otk[4][1]*ri[5][0]))+(
          (Wirk[5][1]*wk[4][0])-(Wirk[5][0]*wk[4][1]))));
        Atk[5][0] = (((AiOiWi[5][2]*Cik[5][2][0])+((AiOiWi[5][0]*Cik[5][0][0])+(
          AiOiWi[5][1]*Cik[5][1][0])))+(((Otk[5][2]*rk[5][1])-(Otk[5][1]*
          rk[5][2]))+((wk[5][1]*Wkrpk[5][2])-(wk[5][2]*Wkrpk[5][1]))));
        Atk[5][1] = (((AiOiWi[5][2]*Cik[5][2][1])+((AiOiWi[5][0]*Cik[5][0][1])+(
          AiOiWi[5][1]*Cik[5][1][1])))+(((Otk[5][0]*rk[5][2])-(Otk[5][2]*
          rk[5][0]))+((wk[5][2]*Wkrpk[5][0])-(wk[5][0]*Wkrpk[5][2]))));
        Atk[5][2] = (((AiOiWi[5][2]*Cik[5][2][2])+((AiOiWi[5][0]*Cik[5][0][2])+(
          AiOiWi[5][1]*Cik[5][1][2])))+(((Otk[5][1]*rk[5][0])-(Otk[5][0]*
          rk[5][1]))+((wk[5][0]*Wkrpk[5][1])-(wk[5][1]*Wkrpk[5][0]))));
        AiOiWi[6][0] = (Atk[5][0]+(((Otk[5][1]*ri[6][2])-(Otk[5][2]*ri[6][1]))+(
          (Wirk[6][2]*wk[5][1])-(Wirk[6][1]*wk[5][2]))));
        AiOiWi[6][1] = (Atk[5][1]+(((Otk[5][2]*ri[6][0])-(Otk[5][0]*ri[6][2]))+(
          (Wirk[6][0]*wk[5][2])-(Wirk[6][2]*wk[5][0]))));
        AiOiWi[6][2] = (Atk[5][2]+(((Otk[5][0]*ri[6][1])-(Otk[5][1]*ri[6][0]))+(
          (Wirk[6][1]*wk[5][0])-(Wirk[6][0]*wk[5][1]))));
        Atk[6][0] = (((AiOiWi[6][2]*Cik[6][2][0])+((AiOiWi[6][0]*Cik[6][0][0])+(
          AiOiWi[6][1]*Cik[6][1][0])))+(((Otk[6][2]*rk[6][1])-(Otk[6][1]*
          rk[6][2]))+((wk[6][1]*Wkrpk[6][2])-(wk[6][2]*Wkrpk[6][1]))));
        Atk[6][1] = (((AiOiWi[6][2]*Cik[6][2][1])+((AiOiWi[6][0]*Cik[6][0][1])+(
          AiOiWi[6][1]*Cik[6][1][1])))+(((Otk[6][0]*rk[6][2])-(Otk[6][2]*
          rk[6][0]))+((wk[6][2]*Wkrpk[6][0])-(wk[6][0]*Wkrpk[6][2]))));
        Atk[6][2] = (((AiOiWi[6][2]*Cik[6][2][2])+((AiOiWi[6][0]*Cik[6][0][2])+(
          AiOiWi[6][1]*Cik[6][1][2])))+(((Otk[6][1]*rk[6][0])-(Otk[6][0]*
          rk[6][1]))+((wk[6][0]*Wkrpk[6][1])-(wk[6][1]*Wkrpk[6][0]))));
        AiOiWi[7][0] = (Atk[6][0]+(((Otk[6][1]*ri[7][2])-(Otk[6][2]*ri[7][1]))+(
          (Wirk[7][2]*wk[6][1])-(Wirk[7][1]*wk[6][2]))));
        AiOiWi[7][1] = (Atk[6][1]+(((Otk[6][2]*ri[7][0])-(Otk[6][0]*ri[7][2]))+(
          (Wirk[7][0]*wk[6][2])-(Wirk[7][2]*wk[6][0]))));
        AiOiWi[7][2] = (Atk[6][2]+(((Otk[6][0]*ri[7][1])-(Otk[6][1]*ri[7][0]))+(
          (Wirk[7][1]*wk[6][0])-(Wirk[7][0]*wk[6][1]))));
        Atk[7][0] = (((AiOiWi[7][2]*Cik[7][2][0])+((AiOiWi[7][0]*Cik[7][0][0])+(
          AiOiWi[7][1]*Cik[7][1][0])))+(((Otk[7][2]*rk[7][1])-(Otk[7][1]*
          rk[7][2]))+((wk[7][1]*Wkrpk[7][2])-(wk[7][2]*Wkrpk[7][1]))));
        Atk[7][1] = (((AiOiWi[7][2]*Cik[7][2][1])+((AiOiWi[7][0]*Cik[7][0][1])+(
          AiOiWi[7][1]*Cik[7][1][1])))+(((Otk[7][0]*rk[7][2])-(Otk[7][2]*
          rk[7][0]))+((wk[7][2]*Wkrpk[7][0])-(wk[7][0]*Wkrpk[7][2]))));
        Atk[7][2] = (((AiOiWi[7][2]*Cik[7][2][2])+((AiOiWi[7][0]*Cik[7][0][2])+(
          AiOiWi[7][1]*Cik[7][1][2])))+(((Otk[7][1]*rk[7][0])-(Otk[7][0]*
          rk[7][1]))+((wk[7][0]*Wkrpk[7][1])-(wk[7][1]*Wkrpk[7][0]))));
        AiOiWi[8][0] = (Atk[0][0]+((Wik[0][1]*Wirk[8][2])-(Wik[0][2]*Wirk[8][1])
          ));
        AiOiWi[8][1] = (Atk[0][1]+((Wik[0][2]*Wirk[8][0])-(Wik[0][0]*Wirk[8][2])
          ));
        AiOiWi[8][2] = (Atk[0][2]+((Wik[0][0]*Wirk[8][1])-(Wik[0][1]*Wirk[8][0])
          ));
        Atk[8][0] = (((AiOiWi[8][2]*Cik[8][2][0])+((AiOiWi[8][0]*Cik[8][0][0])+(
          AiOiWi[8][1]*Cik[8][1][0])))+(((Otk[8][2]*rk[8][1])-(Otk[8][1]*
          rk[8][2]))+((wk[8][1]*Wkrpk[8][2])-(wk[8][2]*Wkrpk[8][1]))));
        Atk[8][1] = (((AiOiWi[8][2]*Cik[8][2][1])+((AiOiWi[8][0]*Cik[8][0][1])+(
          AiOiWi[8][1]*Cik[8][1][1])))+(((Otk[8][0]*rk[8][2])-(Otk[8][2]*
          rk[8][0]))+((wk[8][2]*Wkrpk[8][0])-(wk[8][0]*Wkrpk[8][2]))));
        Atk[8][2] = (((AiOiWi[8][2]*Cik[8][2][2])+((AiOiWi[8][0]*Cik[8][0][2])+(
          AiOiWi[8][1]*Cik[8][1][2])))+(((Otk[8][1]*rk[8][0])-(Otk[8][0]*
          rk[8][1]))+((wk[8][0]*Wkrpk[8][1])-(wk[8][1]*Wkrpk[8][0]))));
        AiOiWi[9][0] = (Atk[8][0]+(((Otk[8][1]*ri[9][2])-(Otk[8][2]*ri[9][1]))+(
          (Wirk[9][2]*wk[8][1])-(Wirk[9][1]*wk[8][2]))));
        AiOiWi[9][1] = (Atk[8][1]+(((Otk[8][2]*ri[9][0])-(Otk[8][0]*ri[9][2]))+(
          (Wirk[9][0]*wk[8][2])-(Wirk[9][2]*wk[8][0]))));
        AiOiWi[9][2] = (Atk[8][2]+(((Otk[8][0]*ri[9][1])-(Otk[8][1]*ri[9][0]))+(
          (Wirk[9][1]*wk[8][0])-(Wirk[9][0]*wk[8][1]))));
        Atk[9][0] = (((AiOiWi[9][2]*Cik[9][2][0])+((AiOiWi[9][0]*Cik[9][0][0])+(
          AiOiWi[9][1]*Cik[9][1][0])))+(((Otk[9][2]*rk[9][1])-(Otk[9][1]*
          rk[9][2]))+((wk[9][1]*Wkrpk[9][2])-(wk[9][2]*Wkrpk[9][1]))));
        Atk[9][1] = (((AiOiWi[9][2]*Cik[9][2][1])+((AiOiWi[9][0]*Cik[9][0][1])+(
          AiOiWi[9][1]*Cik[9][1][1])))+(((Otk[9][0]*rk[9][2])-(Otk[9][2]*
          rk[9][0]))+((wk[9][2]*Wkrpk[9][0])-(wk[9][0]*Wkrpk[9][2]))));
        Atk[9][2] = (((AiOiWi[9][2]*Cik[9][2][2])+((AiOiWi[9][0]*Cik[9][0][2])+(
          AiOiWi[9][1]*Cik[9][1][2])))+(((Otk[9][1]*rk[9][0])-(Otk[9][0]*
          rk[9][1]))+((wk[9][0]*Wkrpk[9][1])-(wk[9][1]*Wkrpk[9][0]))));
        AiOiWi[10][0] = (Atk[9][0]+(((Otk[9][1]*ri[10][2])-(Otk[9][2]*ri[10][1])
          )+((Wirk[10][2]*wk[9][1])-(Wirk[10][1]*wk[9][2]))));
        AiOiWi[10][1] = (Atk[9][1]+(((Otk[9][2]*ri[10][0])-(Otk[9][0]*ri[10][2])
          )+((Wirk[10][0]*wk[9][2])-(Wirk[10][2]*wk[9][0]))));
        AiOiWi[10][2] = (Atk[9][2]+(((Otk[9][0]*ri[10][1])-(Otk[9][1]*ri[10][0])
          )+((Wirk[10][1]*wk[9][0])-(Wirk[10][0]*wk[9][1]))));
        Atk[10][0] = (((AiOiWi[10][2]*Cik[10][2][0])+((AiOiWi[10][0]*
          Cik[10][0][0])+(AiOiWi[10][1]*Cik[10][1][0])))+(((Otk[10][2]*rk[10][1]
          )-(Otk[10][1]*rk[10][2]))+((wk[10][1]*Wkrpk[10][2])-(wk[10][2]*
          Wkrpk[10][1]))));
        Atk[10][1] = (((AiOiWi[10][2]*Cik[10][2][1])+((AiOiWi[10][0]*
          Cik[10][0][1])+(AiOiWi[10][1]*Cik[10][1][1])))+(((Otk[10][0]*rk[10][2]
          )-(Otk[10][2]*rk[10][0]))+((wk[10][2]*Wkrpk[10][0])-(wk[10][0]*
          Wkrpk[10][2]))));
        Atk[10][2] = (((AiOiWi[10][2]*Cik[10][2][2])+((AiOiWi[10][0]*
          Cik[10][0][2])+(AiOiWi[10][1]*Cik[10][1][2])))+(((Otk[10][1]*rk[10][0]
          )-(Otk[10][0]*rk[10][1]))+((wk[10][0]*Wkrpk[10][1])-(wk[10][1]*
          Wkrpk[10][0]))));
        AiOiWi[11][0] = (Atk[10][0]+(((Otk[10][1]*ri[11][2])-(Otk[10][2]*
          ri[11][1]))+((Wirk[11][2]*wk[10][1])-(Wirk[11][1]*wk[10][2]))));
        AiOiWi[11][1] = (Atk[10][1]+(((Otk[10][2]*ri[11][0])-(Otk[10][0]*
          ri[11][2]))+((Wirk[11][0]*wk[10][2])-(Wirk[11][2]*wk[10][0]))));
        AiOiWi[11][2] = (Atk[10][2]+(((Otk[10][0]*ri[11][1])-(Otk[10][1]*
          ri[11][0]))+((Wirk[11][1]*wk[10][0])-(Wirk[11][0]*wk[10][1]))));
        Atk[11][0] = (((AiOiWi[11][2]*Cik[11][2][0])+((AiOiWi[11][0]*
          Cik[11][0][0])+(AiOiWi[11][1]*Cik[11][1][0])))+(((Otk[11][2]*rk[11][1]
          )-(Otk[11][1]*rk[11][2]))+((wk[11][1]*Wkrpk[11][2])-(wk[11][2]*
          Wkrpk[11][1]))));
        Atk[11][1] = (((AiOiWi[11][2]*Cik[11][2][1])+((AiOiWi[11][0]*
          Cik[11][0][1])+(AiOiWi[11][1]*Cik[11][1][1])))+(((Otk[11][0]*rk[11][2]
          )-(Otk[11][2]*rk[11][0]))+((wk[11][2]*Wkrpk[11][0])-(wk[11][0]*
          Wkrpk[11][2]))));
        Atk[11][2] = (((AiOiWi[11][2]*Cik[11][2][2])+((AiOiWi[11][0]*
          Cik[11][0][2])+(AiOiWi[11][1]*Cik[11][1][2])))+(((Otk[11][1]*rk[11][0]
          )-(Otk[11][0]*rk[11][1]))+((wk[11][0]*Wkrpk[11][1])-(wk[11][1]*
          Wkrpk[11][0]))));
        AiOiWi[12][0] = (Atk[11][0]+(((Otk[11][1]*ri[12][2])-(Otk[11][2]*
          ri[12][1]))+((Wirk[12][2]*wk[11][1])-(Wirk[12][1]*wk[11][2]))));
        AiOiWi[12][1] = (Atk[11][1]+(((Otk[11][2]*ri[12][0])-(Otk[11][0]*
          ri[12][2]))+((Wirk[12][0]*wk[11][2])-(Wirk[12][2]*wk[11][0]))));
        AiOiWi[12][2] = (Atk[11][2]+(((Otk[11][0]*ri[12][1])-(Otk[11][1]*
          ri[12][0]))+((Wirk[12][1]*wk[11][0])-(Wirk[12][0]*wk[11][1]))));
        Atk[12][0] = (((AiOiWi[12][2]*Cik[12][2][0])+((AiOiWi[12][0]*
          Cik[12][0][0])+(AiOiWi[12][1]*Cik[12][1][0])))+(((Otk[12][2]*rk[12][1]
          )-(Otk[12][1]*rk[12][2]))+((wk[12][1]*Wkrpk[12][2])-(wk[12][2]*
          Wkrpk[12][1]))));
        Atk[12][1] = (((AiOiWi[12][2]*Cik[12][2][1])+((AiOiWi[12][0]*
          Cik[12][0][1])+(AiOiWi[12][1]*Cik[12][1][1])))+(((Otk[12][0]*rk[12][2]
          )-(Otk[12][2]*rk[12][0]))+((wk[12][2]*Wkrpk[12][0])-(wk[12][0]*
          Wkrpk[12][2]))));
        Atk[12][2] = (((AiOiWi[12][2]*Cik[12][2][2])+((AiOiWi[12][0]*
          Cik[12][0][2])+(AiOiWi[12][1]*Cik[12][1][2])))+(((Otk[12][1]*rk[12][0]
          )-(Otk[12][0]*rk[12][1]))+((wk[12][0]*Wkrpk[12][1])-(wk[12][1]*
          Wkrpk[12][0]))));
        AiOiWi[13][0] = (Atk[12][0]+(((Otk[12][1]*ri[13][2])-(Otk[12][2]*
          ri[13][1]))+((Wirk[13][2]*wk[12][1])-(Wirk[13][1]*wk[12][2]))));
        AiOiWi[13][1] = (Atk[12][1]+(((Otk[12][2]*ri[13][0])-(Otk[12][0]*
          ri[13][2]))+((Wirk[13][0]*wk[12][2])-(Wirk[13][2]*wk[12][0]))));
        AiOiWi[13][2] = (Atk[12][2]+(((Otk[12][0]*ri[13][1])-(Otk[12][1]*
          ri[13][0]))+((Wirk[13][1]*wk[12][0])-(Wirk[13][0]*wk[12][1]))));
        Atk[13][0] = (((AiOiWi[13][2]*Cik[13][2][0])+((AiOiWi[13][0]*
          Cik[13][0][0])+(AiOiWi[13][1]*Cik[13][1][0])))+(((Otk[13][2]*rk[13][1]
          )-(Otk[13][1]*rk[13][2]))+((wk[13][1]*Wkrpk[13][2])-(wk[13][2]*
          Wkrpk[13][1]))));
        Atk[13][1] = (((AiOiWi[13][2]*Cik[13][2][1])+((AiOiWi[13][0]*
          Cik[13][0][1])+(AiOiWi[13][1]*Cik[13][1][1])))+(((Otk[13][0]*rk[13][2]
          )-(Otk[13][2]*rk[13][0]))+((wk[13][2]*Wkrpk[13][0])-(wk[13][0]*
          Wkrpk[13][2]))));
        Atk[13][2] = (((AiOiWi[13][2]*Cik[13][2][2])+((AiOiWi[13][0]*
          Cik[13][0][2])+(AiOiWi[13][1]*Cik[13][1][2])))+(((Otk[13][1]*rk[13][0]
          )-(Otk[13][0]*rk[13][1]))+((wk[13][0]*Wkrpk[13][1])-(wk[13][1]*
          Wkrpk[13][0]))));
        inerflg = 1;
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  504 adds/subtracts/negates
                    585 multiplies
                      0 divides
                    120 assignments
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
        Fstar[0][0] = ((mk[0]*(Atk[0][0]-gk[0][0]))-ufk[0][0]);
        Fstar[0][1] = ((mk[0]*(Atk[0][1]-gk[0][1]))-ufk[0][1]);
        Fstar[0][2] = ((mk[0]*(Atk[0][2]-gk[0][2]))-ufk[0][2]);
        Fstar[1][0] = ((mk[1]*(Atk[1][0]-gk[1][0]))-ufk[1][0]);
        Fstar[1][1] = ((mk[1]*(Atk[1][1]-gk[1][1]))-ufk[1][1]);
        Fstar[1][2] = ((mk[1]*(Atk[1][2]-gk[1][2]))-ufk[1][2]);
        Fstar[2][0] = ((mk[2]*(Atk[2][0]-gk[2][0]))-ufk[2][0]);
        Fstar[2][1] = ((mk[2]*(Atk[2][1]-gk[2][1]))-ufk[2][1]);
        Fstar[2][2] = ((mk[2]*(Atk[2][2]-gk[2][2]))-ufk[2][2]);
        Fstar[3][0] = ((mk[3]*(Atk[3][0]-gk[3][0]))-ufk[3][0]);
        Fstar[3][1] = ((mk[3]*(Atk[3][1]-gk[3][1]))-ufk[3][1]);
        Fstar[3][2] = ((mk[3]*(Atk[3][2]-gk[3][2]))-ufk[3][2]);
        Fstar[4][0] = ((mk[4]*(Atk[4][0]-gk[4][0]))-ufk[4][0]);
        Fstar[4][1] = ((mk[4]*(Atk[4][1]-gk[4][1]))-ufk[4][1]);
        Fstar[4][2] = ((mk[4]*(Atk[4][2]-gk[4][2]))-ufk[4][2]);
        Fstar[5][0] = ((mk[5]*(Atk[5][0]-gk[5][0]))-ufk[5][0]);
        Fstar[5][1] = ((mk[5]*(Atk[5][1]-gk[5][1]))-ufk[5][1]);
        Fstar[5][2] = ((mk[5]*(Atk[5][2]-gk[5][2]))-ufk[5][2]);
        Fstar[6][0] = ((mk[6]*(Atk[6][0]-gk[6][0]))-ufk[6][0]);
        Fstar[6][1] = ((mk[6]*(Atk[6][1]-gk[6][1]))-ufk[6][1]);
        Fstar[6][2] = ((mk[6]*(Atk[6][2]-gk[6][2]))-ufk[6][2]);
        Fstar[7][0] = ((mk[7]*(Atk[7][0]-gk[7][0]))-ufk[7][0]);
        Fstar[7][1] = ((mk[7]*(Atk[7][1]-gk[7][1]))-ufk[7][1]);
        Fstar[7][2] = ((mk[7]*(Atk[7][2]-gk[7][2]))-ufk[7][2]);
        Fstar[8][0] = ((mk[8]*(Atk[8][0]-gk[8][0]))-ufk[8][0]);
        Fstar[8][1] = ((mk[8]*(Atk[8][1]-gk[8][1]))-ufk[8][1]);
        Fstar[8][2] = ((mk[8]*(Atk[8][2]-gk[8][2]))-ufk[8][2]);
        Fstar[9][0] = ((mk[9]*(Atk[9][0]-gk[9][0]))-ufk[9][0]);
        Fstar[9][1] = ((mk[9]*(Atk[9][1]-gk[9][1]))-ufk[9][1]);
        Fstar[9][2] = ((mk[9]*(Atk[9][2]-gk[9][2]))-ufk[9][2]);
        Fstar[10][0] = ((mk[10]*(Atk[10][0]-gk[10][0]))-ufk[10][0]);
        Fstar[10][1] = ((mk[10]*(Atk[10][1]-gk[10][1]))-ufk[10][1]);
        Fstar[10][2] = ((mk[10]*(Atk[10][2]-gk[10][2]))-ufk[10][2]);
        Fstar[11][0] = ((mk[11]*(Atk[11][0]-gk[11][0]))-ufk[11][0]);
        Fstar[11][1] = ((mk[11]*(Atk[11][1]-gk[11][1]))-ufk[11][1]);
        Fstar[11][2] = ((mk[11]*(Atk[11][2]-gk[11][2]))-ufk[11][2]);
        Fstar[12][0] = ((mk[12]*(Atk[12][0]-gk[12][0]))-ufk[12][0]);
        Fstar[12][1] = ((mk[12]*(Atk[12][1]-gk[12][1]))-ufk[12][1]);
        Fstar[12][2] = ((mk[12]*(Atk[12][2]-gk[12][2]))-ufk[12][2]);
        Fstar[13][0] = ((mk[13]*(Atk[13][0]-gk[13][0]))-ufk[13][0]);
        Fstar[13][1] = ((mk[13]*(Atk[13][1]-gk[13][1]))-ufk[13][1]);
        Fstar[13][2] = ((mk[13]*(Atk[13][2]-gk[13][2]))-ufk[13][2]);
/*
Compute Tstar (torques)
*/
        Tstar[0][0] = (WkIkWk[0][0]-utk[0][0]);
        Tstar[0][1] = (WkIkWk[0][1]-utk[0][1]);
        Tstar[0][2] = (WkIkWk[0][2]-utk[0][2]);
        Tstar[1][0] = ((WkIkWk[1][0]+((ik[1][0][2]*Otk[1][2])+((ik[1][0][0]*
          Otk[1][0])+(ik[1][0][1]*Otk[1][1]))))-utk[1][0]);
        Tstar[1][1] = ((WkIkWk[1][1]+((ik[1][1][2]*Otk[1][2])+((ik[1][1][0]*
          Otk[1][0])+(ik[1][1][1]*Otk[1][1]))))-utk[1][1]);
        Tstar[1][2] = ((WkIkWk[1][2]+((ik[1][2][2]*Otk[1][2])+((ik[1][2][0]*
          Otk[1][0])+(ik[1][2][1]*Otk[1][1]))))-utk[1][2]);
        Tstar[2][0] = ((WkIkWk[2][0]+((ik[2][0][2]*Otk[2][2])+((ik[2][0][0]*
          Otk[2][0])+(ik[2][0][1]*Otk[2][1]))))-utk[2][0]);
        Tstar[2][1] = ((WkIkWk[2][1]+((ik[2][1][2]*Otk[2][2])+((ik[2][1][0]*
          Otk[2][0])+(ik[2][1][1]*Otk[2][1]))))-utk[2][1]);
        Tstar[2][2] = ((WkIkWk[2][2]+((ik[2][2][2]*Otk[2][2])+((ik[2][2][0]*
          Otk[2][0])+(ik[2][2][1]*Otk[2][1]))))-utk[2][2]);
        Tstar[3][0] = ((WkIkWk[3][0]+((ik[3][0][2]*Otk[3][2])+((ik[3][0][0]*
          Otk[3][0])+(ik[3][0][1]*Otk[3][1]))))-utk[3][0]);
        Tstar[3][1] = ((WkIkWk[3][1]+((ik[3][1][2]*Otk[3][2])+((ik[3][1][0]*
          Otk[3][0])+(ik[3][1][1]*Otk[3][1]))))-utk[3][1]);
        Tstar[3][2] = ((WkIkWk[3][2]+((ik[3][2][2]*Otk[3][2])+((ik[3][2][0]*
          Otk[3][0])+(ik[3][2][1]*Otk[3][1]))))-utk[3][2]);
        Tstar[4][0] = ((WkIkWk[4][0]+((ik[4][0][2]*Otk[4][2])+((ik[4][0][0]*
          Otk[4][0])+(ik[4][0][1]*Otk[4][1]))))-utk[4][0]);
        Tstar[4][1] = ((WkIkWk[4][1]+((ik[4][1][2]*Otk[4][2])+((ik[4][1][0]*
          Otk[4][0])+(ik[4][1][1]*Otk[4][1]))))-utk[4][1]);
        Tstar[4][2] = ((WkIkWk[4][2]+((ik[4][2][2]*Otk[4][2])+((ik[4][2][0]*
          Otk[4][0])+(ik[4][2][1]*Otk[4][1]))))-utk[4][2]);
        Tstar[5][0] = ((WkIkWk[5][0]+((ik[5][0][2]*Otk[5][2])+((ik[5][0][0]*
          Otk[5][0])+(ik[5][0][1]*Otk[5][1]))))-utk[5][0]);
        Tstar[5][1] = ((WkIkWk[5][1]+((ik[5][1][2]*Otk[5][2])+((ik[5][1][0]*
          Otk[5][0])+(ik[5][1][1]*Otk[5][1]))))-utk[5][1]);
        Tstar[5][2] = ((WkIkWk[5][2]+((ik[5][2][2]*Otk[5][2])+((ik[5][2][0]*
          Otk[5][0])+(ik[5][2][1]*Otk[5][1]))))-utk[5][2]);
        Tstar[6][0] = ((WkIkWk[6][0]+((ik[6][0][2]*Otk[6][2])+((ik[6][0][0]*
          Otk[6][0])+(ik[6][0][1]*Otk[6][1]))))-utk[6][0]);
        Tstar[6][1] = ((WkIkWk[6][1]+((ik[6][1][2]*Otk[6][2])+((ik[6][1][0]*
          Otk[6][0])+(ik[6][1][1]*Otk[6][1]))))-utk[6][1]);
        Tstar[6][2] = ((WkIkWk[6][2]+((ik[6][2][2]*Otk[6][2])+((ik[6][2][0]*
          Otk[6][0])+(ik[6][2][1]*Otk[6][1]))))-utk[6][2]);
        Tstar[7][0] = ((WkIkWk[7][0]+((ik[7][0][2]*Otk[7][2])+((ik[7][0][0]*
          Otk[7][0])+(ik[7][0][1]*Otk[7][1]))))-utk[7][0]);
        Tstar[7][1] = ((WkIkWk[7][1]+((ik[7][1][2]*Otk[7][2])+((ik[7][1][0]*
          Otk[7][0])+(ik[7][1][1]*Otk[7][1]))))-utk[7][1]);
        Tstar[7][2] = ((WkIkWk[7][2]+((ik[7][2][2]*Otk[7][2])+((ik[7][2][0]*
          Otk[7][0])+(ik[7][2][1]*Otk[7][1]))))-utk[7][2]);
        Tstar[8][0] = ((WkIkWk[8][0]+((ik[8][0][2]*Otk[8][2])+((ik[8][0][0]*
          Otk[8][0])+(ik[8][0][1]*Otk[8][1]))))-utk[8][0]);
        Tstar[8][1] = ((WkIkWk[8][1]+((ik[8][1][2]*Otk[8][2])+((ik[8][1][0]*
          Otk[8][0])+(ik[8][1][1]*Otk[8][1]))))-utk[8][1]);
        Tstar[8][2] = ((WkIkWk[8][2]+((ik[8][2][2]*Otk[8][2])+((ik[8][2][0]*
          Otk[8][0])+(ik[8][2][1]*Otk[8][1]))))-utk[8][2]);
        Tstar[9][0] = ((WkIkWk[9][0]+((ik[9][0][2]*Otk[9][2])+((ik[9][0][0]*
          Otk[9][0])+(ik[9][0][1]*Otk[9][1]))))-utk[9][0]);
        Tstar[9][1] = ((WkIkWk[9][1]+((ik[9][1][2]*Otk[9][2])+((ik[9][1][0]*
          Otk[9][0])+(ik[9][1][1]*Otk[9][1]))))-utk[9][1]);
        Tstar[9][2] = ((WkIkWk[9][2]+((ik[9][2][2]*Otk[9][2])+((ik[9][2][0]*
          Otk[9][0])+(ik[9][2][1]*Otk[9][1]))))-utk[9][2]);
        Tstar[10][0] = ((WkIkWk[10][0]+((ik[10][0][2]*Otk[10][2])+((ik[10][0][0]
          *Otk[10][0])+(ik[10][0][1]*Otk[10][1]))))-utk[10][0]);
        Tstar[10][1] = ((WkIkWk[10][1]+((ik[10][1][2]*Otk[10][2])+((ik[10][1][0]
          *Otk[10][0])+(ik[10][1][1]*Otk[10][1]))))-utk[10][1]);
        Tstar[10][2] = ((WkIkWk[10][2]+((ik[10][2][2]*Otk[10][2])+((ik[10][2][0]
          *Otk[10][0])+(ik[10][2][1]*Otk[10][1]))))-utk[10][2]);
        Tstar[11][0] = ((WkIkWk[11][0]+((ik[11][0][2]*Otk[11][2])+((ik[11][0][0]
          *Otk[11][0])+(ik[11][0][1]*Otk[11][1]))))-utk[11][0]);
        Tstar[11][1] = ((WkIkWk[11][1]+((ik[11][1][2]*Otk[11][2])+((ik[11][1][0]
          *Otk[11][0])+(ik[11][1][1]*Otk[11][1]))))-utk[11][1]);
        Tstar[11][2] = ((WkIkWk[11][2]+((ik[11][2][2]*Otk[11][2])+((ik[11][2][0]
          *Otk[11][0])+(ik[11][2][1]*Otk[11][1]))))-utk[11][2]);
        Tstar[12][0] = ((WkIkWk[12][0]+((ik[12][0][2]*Otk[12][2])+((ik[12][0][0]
          *Otk[12][0])+(ik[12][0][1]*Otk[12][1]))))-utk[12][0]);
        Tstar[12][1] = ((WkIkWk[12][1]+((ik[12][1][2]*Otk[12][2])+((ik[12][1][0]
          *Otk[12][0])+(ik[12][1][1]*Otk[12][1]))))-utk[12][1]);
        Tstar[12][2] = ((WkIkWk[12][2]+((ik[12][2][2]*Otk[12][2])+((ik[12][2][0]
          *Otk[12][0])+(ik[12][2][1]*Otk[12][1]))))-utk[12][2]);
        Tstar[13][0] = ((WkIkWk[13][0]+((ik[13][0][2]*Otk[13][2])+((ik[13][0][0]
          *Otk[13][0])+(ik[13][0][1]*Otk[13][1]))))-utk[13][0]);
        Tstar[13][1] = ((WkIkWk[13][1]+((ik[13][1][2]*Otk[13][2])+((ik[13][1][0]
          *Otk[13][0])+(ik[13][1][1]*Otk[13][1]))))-utk[13][1]);
        Tstar[13][2] = ((WkIkWk[13][2]+((ik[13][2][2]*Otk[13][2])+((ik[13][2][0]
          *Otk[13][0])+(ik[13][2][1]*Otk[13][1]))))-utk[13][2]);
/*
Compute fs0 (RHS ignoring constraints)
*/
        sddovpk();
        temp[0] = ((((Fstar[0][2]*Vpk[0][0][2])+((Fstar[0][0]*Vpk[0][0][0])+(
          Fstar[0][1]*Vpk[0][0][1])))+((pin[0][2]*Tstar[0][2])+((pin[0][0]*
          Tstar[0][0])+(pin[0][1]*Tstar[0][1]))))+(((Fstar[1][2]*Vpk[0][1][2])+(
          (Fstar[1][0]*Vpk[0][1][0])+(Fstar[1][1]*Vpk[0][1][1])))+((Tstar[1][2]*
          Wpk[0][1][2])+((Tstar[1][0]*Wpk[0][1][0])+(Tstar[1][1]*Wpk[0][1][1])))
          ));
        temp[1] = ((((Fstar[3][2]*Vpk[0][3][2])+((Fstar[3][0]*Vpk[0][3][0])+(
          Fstar[3][1]*Vpk[0][3][1])))+((Tstar[3][2]*Wpk[0][3][2])+((Tstar[3][0]*
          Wpk[0][3][0])+(Tstar[3][1]*Wpk[0][3][1]))))+((((Fstar[2][2]*
          Vpk[0][2][2])+((Fstar[2][0]*Vpk[0][2][0])+(Fstar[2][1]*Vpk[0][2][1])))
          +((Tstar[2][2]*Wpk[0][2][2])+((Tstar[2][0]*Wpk[0][2][0])+(Tstar[2][1]*
          Wpk[0][2][1]))))+temp[0]));
        temp[2] = ((((Fstar[5][2]*Vpk[0][5][2])+((Fstar[5][0]*Vpk[0][5][0])+(
          Fstar[5][1]*Vpk[0][5][1])))+((Tstar[5][2]*Wpk[0][5][2])+((Tstar[5][0]*
          Wpk[0][5][0])+(Tstar[5][1]*Wpk[0][5][1]))))+((((Fstar[4][2]*
          Vpk[0][4][2])+((Fstar[4][0]*Vpk[0][4][0])+(Fstar[4][1]*Vpk[0][4][1])))
          +((Tstar[4][2]*Wpk[0][4][2])+((Tstar[4][0]*Wpk[0][4][0])+(Tstar[4][1]*
          Wpk[0][4][1]))))+temp[1]));
        temp[3] = ((((Fstar[7][2]*Vpk[0][7][2])+((Fstar[7][0]*Vpk[0][7][0])+(
          Fstar[7][1]*Vpk[0][7][1])))+((Tstar[7][2]*Wpk[0][7][2])+((Tstar[7][0]*
          Wpk[0][7][0])+(Tstar[7][1]*Wpk[0][7][1]))))+((((Fstar[6][2]*
          Vpk[0][6][2])+((Fstar[6][0]*Vpk[0][6][0])+(Fstar[6][1]*Vpk[0][6][1])))
          +((Tstar[6][2]*Wpk[0][6][2])+((Tstar[6][0]*Wpk[0][6][0])+(Tstar[6][1]*
          Wpk[0][6][1]))))+temp[2]));
        temp[4] = ((((Fstar[9][2]*Vpk[0][9][2])+((Fstar[9][0]*Vpk[0][9][0])+(
          Fstar[9][1]*Vpk[0][9][1])))+((Tstar[9][2]*Wpk[0][9][2])+((Tstar[9][0]*
          Wpk[0][9][0])+(Tstar[9][1]*Wpk[0][9][1]))))+((((Fstar[8][2]*
          Vpk[0][8][2])+((Fstar[8][0]*Vpk[0][8][0])+(Fstar[8][1]*Vpk[0][8][1])))
          +((Tstar[8][2]*Wpk[0][8][2])+((Tstar[8][0]*Wpk[0][8][0])+(Tstar[8][1]*
          Wpk[0][8][1]))))+temp[3]));
        temp[5] = ((((Fstar[11][2]*Vpk[0][11][2])+((Fstar[11][0]*Vpk[0][11][0])+
          (Fstar[11][1]*Vpk[0][11][1])))+((Tstar[11][2]*Wpk[0][11][2])+((
          Tstar[11][0]*Wpk[0][11][0])+(Tstar[11][1]*Wpk[0][11][1]))))+((((
          Fstar[10][2]*Vpk[0][10][2])+((Fstar[10][0]*Vpk[0][10][0])+(
          Fstar[10][1]*Vpk[0][10][1])))+((Tstar[10][2]*Wpk[0][10][2])+((
          Tstar[10][0]*Wpk[0][10][0])+(Tstar[10][1]*Wpk[0][10][1]))))+temp[4]));
        fs0[0] = (utau[0]-((((Fstar[13][2]*Vpk[0][13][2])+((Fstar[13][0]*
          Vpk[0][13][0])+(Fstar[13][1]*Vpk[0][13][1])))+((Tstar[13][2]*
          Wpk[0][13][2])+((Tstar[13][0]*Wpk[0][13][0])+(Tstar[13][1]*
          Wpk[0][13][1]))))+((((Fstar[12][2]*Vpk[0][12][2])+((Fstar[12][0]*
          Vpk[0][12][0])+(Fstar[12][1]*Vpk[0][12][1])))+((Tstar[12][2]*
          Wpk[0][12][2])+((Tstar[12][0]*Wpk[0][12][0])+(Tstar[12][1]*
          Wpk[0][12][1]))))+temp[5])));
        fs0[1] = (utau[1]-(((Fstar[1][2]*Vpk[1][1][2])+((Fstar[1][0]*
          Vpk[1][1][0])+(Fstar[1][1]*Vpk[1][1][1])))+((pin[1][2]*Tstar[1][2])+((
          pin[1][0]*Tstar[1][0])+(pin[1][1]*Tstar[1][1])))));
        temp[0] = ((((Fstar[2][2]*Vpk[2][2][2])+((Fstar[2][0]*Vpk[2][2][0])+(
          Fstar[2][1]*Vpk[2][2][1])))+((pin[2][2]*Tstar[2][2])+((pin[2][0]*
          Tstar[2][0])+(pin[2][1]*Tstar[2][1]))))+(((Fstar[3][2]*Vpk[2][3][2])+(
          (Fstar[3][0]*Vpk[2][3][0])+(Fstar[3][1]*Vpk[2][3][1])))+((Tstar[3][2]*
          Wpk[2][3][2])+((Tstar[3][0]*Wpk[2][3][0])+(Tstar[3][1]*Wpk[2][3][1])))
          ));
        temp[1] = ((((Fstar[5][2]*Vpk[2][5][2])+((Fstar[5][0]*Vpk[2][5][0])+(
          Fstar[5][1]*Vpk[2][5][1])))+((Tstar[5][2]*Wpk[2][5][2])+((Tstar[5][0]*
          Wpk[2][5][0])+(Tstar[5][1]*Wpk[2][5][1]))))+((((Fstar[4][2]*
          Vpk[2][4][2])+((Fstar[4][0]*Vpk[2][4][0])+(Fstar[4][1]*Vpk[2][4][1])))
          +((Tstar[4][2]*Wpk[2][4][2])+((Tstar[4][0]*Wpk[2][4][0])+(Tstar[4][1]*
          Wpk[2][4][1]))))+temp[0]));
        fs0[2] = (utau[2]-((((Fstar[7][2]*Vpk[2][7][2])+((Fstar[7][0]*
          Vpk[2][7][0])+(Fstar[7][1]*Vpk[2][7][1])))+((Tstar[7][2]*Wpk[2][7][2])
          +((Tstar[7][0]*Wpk[2][7][0])+(Tstar[7][1]*Wpk[2][7][1]))))+((((
          Fstar[6][2]*Vpk[2][6][2])+((Fstar[6][0]*Vpk[2][6][0])+(Fstar[6][1]*
          Vpk[2][6][1])))+((Tstar[6][2]*Wpk[2][6][2])+((Tstar[6][0]*Wpk[2][6][0]
          )+(Tstar[6][1]*Wpk[2][6][1]))))+temp[1])));
        temp[0] = ((((Fstar[3][2]*Vpk[3][3][2])+((Fstar[3][0]*Vpk[3][3][0])+(
          Fstar[3][1]*Vpk[3][3][1])))+((pin[3][2]*Tstar[3][2])+((pin[3][0]*
          Tstar[3][0])+(pin[3][1]*Tstar[3][1]))))+(((Fstar[4][2]*Vpk[3][4][2])+(
          (Fstar[4][0]*Vpk[3][4][0])+(Fstar[4][1]*Vpk[3][4][1])))+((Tstar[4][2]*
          Wpk[3][4][2])+((Tstar[4][0]*Wpk[3][4][0])+(Tstar[4][1]*Wpk[3][4][1])))
          ));
        temp[1] = ((((Fstar[6][2]*Vpk[3][6][2])+((Fstar[6][0]*Vpk[3][6][0])+(
          Fstar[6][1]*Vpk[3][6][1])))+((Tstar[6][2]*Wpk[3][6][2])+((Tstar[6][0]*
          Wpk[3][6][0])+(Tstar[6][1]*Wpk[3][6][1]))))+((((Fstar[5][2]*
          Vpk[3][5][2])+((Fstar[5][0]*Vpk[3][5][0])+(Fstar[5][1]*Vpk[3][5][1])))
          +((Tstar[5][2]*Wpk[3][5][2])+((Tstar[5][0]*Wpk[3][5][0])+(Tstar[5][1]*
          Wpk[3][5][1]))))+temp[0]));
        fs0[3] = (utau[3]-((((Fstar[7][2]*Vpk[3][7][2])+((Fstar[7][0]*
          Vpk[3][7][0])+(Fstar[7][1]*Vpk[3][7][1])))+((Tstar[7][2]*Wpk[3][7][2])
          +((Tstar[7][0]*Wpk[3][7][0])+(Tstar[7][1]*Wpk[3][7][1]))))+temp[1]));
        temp[0] = ((((Fstar[4][2]*Vpk[4][4][2])+((Fstar[4][0]*Vpk[4][4][0])+(
          Fstar[4][1]*Vpk[4][4][1])))+((pin[4][2]*Tstar[4][2])+((pin[4][0]*
          Tstar[4][0])+(pin[4][1]*Tstar[4][1]))))+(((Fstar[5][2]*Vpk[4][5][2])+(
          (Fstar[5][0]*Vpk[4][5][0])+(Fstar[5][1]*Vpk[4][5][1])))+((Tstar[5][2]*
          Wpk[4][5][2])+((Tstar[5][0]*Wpk[4][5][0])+(Tstar[5][1]*Wpk[4][5][1])))
          ));
        fs0[4] = (utau[4]-((((Fstar[7][2]*Vpk[4][7][2])+((Fstar[7][0]*
          Vpk[4][7][0])+(Fstar[7][1]*Vpk[4][7][1])))+((Tstar[7][2]*Wpk[4][7][2])
          +((Tstar[7][0]*Wpk[4][7][0])+(Tstar[7][1]*Wpk[4][7][1]))))+((((
          Fstar[6][2]*Vpk[4][6][2])+((Fstar[6][0]*Vpk[4][6][0])+(Fstar[6][1]*
          Vpk[4][6][1])))+((Tstar[6][2]*Wpk[4][6][2])+((Tstar[6][0]*Wpk[4][6][0]
          )+(Tstar[6][1]*Wpk[4][6][1]))))+temp[0])));
        temp[0] = ((((Fstar[5][2]*Vpk[5][5][2])+((Fstar[5][0]*Vpk[5][5][0])+(
          Fstar[5][1]*Vpk[5][5][1])))+((pin[5][2]*Tstar[5][2])+((pin[5][0]*
          Tstar[5][0])+(pin[5][1]*Tstar[5][1]))))+(((Fstar[6][2]*Vpk[5][6][2])+(
          (Fstar[6][0]*Vpk[5][6][0])+(Fstar[6][1]*Vpk[5][6][1])))+((Tstar[6][2]*
          Wpk[5][6][2])+((Tstar[6][0]*Wpk[5][6][0])+(Tstar[6][1]*Wpk[5][6][1])))
          ));
        fs0[5] = (utau[5]-((((Fstar[7][2]*Vpk[5][7][2])+((Fstar[7][0]*
          Vpk[5][7][0])+(Fstar[7][1]*Vpk[5][7][1])))+((Tstar[7][2]*Wpk[5][7][2])
          +((Tstar[7][0]*Wpk[5][7][0])+(Tstar[7][1]*Wpk[5][7][1]))))+temp[0]));
        fs0[6] = (utau[6]-((((Fstar[6][2]*Vpk[6][6][2])+((Fstar[6][0]*
          Vpk[6][6][0])+(Fstar[6][1]*Vpk[6][6][1])))+((pin[6][2]*Tstar[6][2])+((
          pin[6][0]*Tstar[6][0])+(pin[6][1]*Tstar[6][1]))))+(((Fstar[7][2]*
          Vpk[6][7][2])+((Fstar[7][0]*Vpk[6][7][0])+(Fstar[7][1]*Vpk[6][7][1])))
          +((Tstar[7][2]*Wpk[6][7][2])+((Tstar[7][0]*Wpk[6][7][0])+(Tstar[7][1]*
          Wpk[6][7][1]))))));
        fs0[7] = (utau[7]-(((Fstar[7][2]*Vpk[7][7][2])+((Fstar[7][0]*
          Vpk[7][7][0])+(Fstar[7][1]*Vpk[7][7][1])))+((pin[7][2]*Tstar[7][2])+((
          pin[7][0]*Tstar[7][0])+(pin[7][1]*Tstar[7][1])))));
        temp[0] = ((((Fstar[8][2]*Vpk[8][8][2])+((Fstar[8][0]*Vpk[8][8][0])+(
          Fstar[8][1]*Vpk[8][8][1])))+((pin[8][2]*Tstar[8][2])+((pin[8][0]*
          Tstar[8][0])+(pin[8][1]*Tstar[8][1]))))+(((Fstar[9][2]*Vpk[8][9][2])+(
          (Fstar[9][0]*Vpk[8][9][0])+(Fstar[9][1]*Vpk[8][9][1])))+((Tstar[9][2]*
          Wpk[8][9][2])+((Tstar[9][0]*Wpk[8][9][0])+(Tstar[9][1]*Wpk[8][9][1])))
          ));
        temp[1] = ((((Fstar[11][2]*Vpk[8][11][2])+((Fstar[11][0]*Vpk[8][11][0])+
          (Fstar[11][1]*Vpk[8][11][1])))+((Tstar[11][2]*Wpk[8][11][2])+((
          Tstar[11][0]*Wpk[8][11][0])+(Tstar[11][1]*Wpk[8][11][1]))))+((((
          Fstar[10][2]*Vpk[8][10][2])+((Fstar[10][0]*Vpk[8][10][0])+(
          Fstar[10][1]*Vpk[8][10][1])))+((Tstar[10][2]*Wpk[8][10][2])+((
          Tstar[10][0]*Wpk[8][10][0])+(Tstar[10][1]*Wpk[8][10][1]))))+temp[0]));
        fs0[8] = (utau[8]-((((Fstar[13][2]*Vpk[8][13][2])+((Fstar[13][0]*
          Vpk[8][13][0])+(Fstar[13][1]*Vpk[8][13][1])))+((Tstar[13][2]*
          Wpk[8][13][2])+((Tstar[13][0]*Wpk[8][13][0])+(Tstar[13][1]*
          Wpk[8][13][1]))))+((((Fstar[12][2]*Vpk[8][12][2])+((Fstar[12][0]*
          Vpk[8][12][0])+(Fstar[12][1]*Vpk[8][12][1])))+((Tstar[12][2]*
          Wpk[8][12][2])+((Tstar[12][0]*Wpk[8][12][0])+(Tstar[12][1]*
          Wpk[8][12][1]))))+temp[1])));
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
        fs0[9] = (utau[9]-((((Fstar[13][2]*Vpk[9][13][2])+((Fstar[13][0]*
          Vpk[9][13][0])+(Fstar[13][1]*Vpk[9][13][1])))+((Tstar[13][2]*
          Wpk[9][13][2])+((Tstar[13][0]*Wpk[9][13][0])+(Tstar[13][1]*
          Wpk[9][13][1]))))+temp[1]));
        temp[0] = ((((Fstar[10][2]*Vpk[10][10][2])+((Fstar[10][0]*Vpk[10][10][0]
          )+(Fstar[10][1]*Vpk[10][10][1])))+((pin[10][2]*Tstar[10][2])+((
          pin[10][0]*Tstar[10][0])+(pin[10][1]*Tstar[10][1]))))+(((Fstar[11][2]*
          Vpk[10][11][2])+((Fstar[11][0]*Vpk[10][11][0])+(Fstar[11][1]*
          Vpk[10][11][1])))+((Tstar[11][2]*Wpk[10][11][2])+((Tstar[11][0]*
          Wpk[10][11][0])+(Tstar[11][1]*Wpk[10][11][1])))));
        fs0[10] = (utau[10]-((((Fstar[13][2]*Vpk[10][13][2])+((Fstar[13][0]*
          Vpk[10][13][0])+(Fstar[13][1]*Vpk[10][13][1])))+((Tstar[13][2]*
          Wpk[10][13][2])+((Tstar[13][0]*Wpk[10][13][0])+(Tstar[13][1]*
          Wpk[10][13][1]))))+((((Fstar[12][2]*Vpk[10][12][2])+((Fstar[12][0]*
          Vpk[10][12][0])+(Fstar[12][1]*Vpk[10][12][1])))+((Tstar[12][2]*
          Wpk[10][12][2])+((Tstar[12][0]*Wpk[10][12][0])+(Tstar[12][1]*
          Wpk[10][12][1]))))+temp[0])));
        temp[0] = ((((Fstar[11][2]*Vpk[11][11][2])+((Fstar[11][0]*Vpk[11][11][0]
          )+(Fstar[11][1]*Vpk[11][11][1])))+((pin[11][2]*Tstar[11][2])+((
          pin[11][0]*Tstar[11][0])+(pin[11][1]*Tstar[11][1]))))+(((Fstar[12][2]*
          Vpk[11][12][2])+((Fstar[12][0]*Vpk[11][12][0])+(Fstar[12][1]*
          Vpk[11][12][1])))+((Tstar[12][2]*Wpk[11][12][2])+((Tstar[12][0]*
          Wpk[11][12][0])+(Tstar[12][1]*Wpk[11][12][1])))));
        fs0[11] = (utau[11]-((((Fstar[13][2]*Vpk[11][13][2])+((Fstar[13][0]*
          Vpk[11][13][0])+(Fstar[13][1]*Vpk[11][13][1])))+((Tstar[13][2]*
          Wpk[11][13][2])+((Tstar[13][0]*Wpk[11][13][0])+(Tstar[13][1]*
          Wpk[11][13][1]))))+temp[0]));
        fs0[12] = (utau[12]-((((Fstar[12][2]*Vpk[12][12][2])+((Fstar[12][0]*
          Vpk[12][12][0])+(Fstar[12][1]*Vpk[12][12][1])))+((pin[12][2]*
          Tstar[12][2])+((pin[12][0]*Tstar[12][0])+(pin[12][1]*Tstar[12][1]))))+
          (((Fstar[13][2]*Vpk[12][13][2])+((Fstar[13][0]*Vpk[12][13][0])+(
          Fstar[13][1]*Vpk[12][13][1])))+((Tstar[13][2]*Wpk[12][13][2])+((
          Tstar[13][0]*Wpk[12][13][0])+(Tstar[13][1]*Wpk[12][13][1]))))));
        fs0[13] = (utau[13]-(((Fstar[13][2]*Vpk[13][13][2])+((Fstar[13][0]*
          Vpk[13][13][0])+(Fstar[13][1]*Vpk[13][13][1])))+((pin[13][2]*
          Tstar[13][2])+((pin[13][0]*Tstar[13][0])+(pin[13][1]*Tstar[13][1])))))
          ;
        fs0flg = 1;
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  585 adds/subtracts/negates
                    501 multiplies
                      0 divides
                    116 assignments
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
        IkWpk[0][0][0] = ((ik[0][0][2]*pin[0][2])+((ik[0][0][0]*pin[0][0])+(
          ik[0][0][1]*pin[0][1])));
        IkWpk[0][0][1] = ((ik[0][1][2]*pin[0][2])+((ik[0][1][0]*pin[0][0])+(
          ik[0][1][1]*pin[0][1])));
        IkWpk[0][0][2] = ((ik[0][2][2]*pin[0][2])+((ik[0][2][0]*pin[0][0])+(
          ik[0][2][1]*pin[0][1])));
        IkWpk[0][1][0] = ((ik[1][0][2]*Wpk[0][1][2])+((ik[1][0][0]*Wpk[0][1][0])
          +(ik[1][0][1]*Wpk[0][1][1])));
        IkWpk[0][1][1] = ((ik[1][1][2]*Wpk[0][1][2])+((ik[1][1][0]*Wpk[0][1][0])
          +(ik[1][1][1]*Wpk[0][1][1])));
        IkWpk[0][1][2] = ((ik[1][2][2]*Wpk[0][1][2])+((ik[1][2][0]*Wpk[0][1][0])
          +(ik[1][2][1]*Wpk[0][1][1])));
        IkWpk[0][2][0] = ((ik[2][0][2]*Wpk[0][2][2])+((ik[2][0][0]*Wpk[0][2][0])
          +(ik[2][0][1]*Wpk[0][2][1])));
        IkWpk[0][2][1] = ((ik[2][1][2]*Wpk[0][2][2])+((ik[2][1][0]*Wpk[0][2][0])
          +(ik[2][1][1]*Wpk[0][2][1])));
        IkWpk[0][2][2] = ((ik[2][2][2]*Wpk[0][2][2])+((ik[2][2][0]*Wpk[0][2][0])
          +(ik[2][2][1]*Wpk[0][2][1])));
        IkWpk[0][3][0] = ((ik[3][0][2]*Wpk[0][3][2])+((ik[3][0][0]*Wpk[0][3][0])
          +(ik[3][0][1]*Wpk[0][3][1])));
        IkWpk[0][3][1] = ((ik[3][1][2]*Wpk[0][3][2])+((ik[3][1][0]*Wpk[0][3][0])
          +(ik[3][1][1]*Wpk[0][3][1])));
        IkWpk[0][3][2] = ((ik[3][2][2]*Wpk[0][3][2])+((ik[3][2][0]*Wpk[0][3][0])
          +(ik[3][2][1]*Wpk[0][3][1])));
        IkWpk[0][4][0] = ((ik[4][0][2]*Wpk[0][4][2])+((ik[4][0][0]*Wpk[0][4][0])
          +(ik[4][0][1]*Wpk[0][4][1])));
        IkWpk[0][4][1] = ((ik[4][1][2]*Wpk[0][4][2])+((ik[4][1][0]*Wpk[0][4][0])
          +(ik[4][1][1]*Wpk[0][4][1])));
        IkWpk[0][4][2] = ((ik[4][2][2]*Wpk[0][4][2])+((ik[4][2][0]*Wpk[0][4][0])
          +(ik[4][2][1]*Wpk[0][4][1])));
        IkWpk[0][5][0] = ((ik[5][0][2]*Wpk[0][5][2])+((ik[5][0][0]*Wpk[0][5][0])
          +(ik[5][0][1]*Wpk[0][5][1])));
        IkWpk[0][5][1] = ((ik[5][1][2]*Wpk[0][5][2])+((ik[5][1][0]*Wpk[0][5][0])
          +(ik[5][1][1]*Wpk[0][5][1])));
        IkWpk[0][5][2] = ((ik[5][2][2]*Wpk[0][5][2])+((ik[5][2][0]*Wpk[0][5][0])
          +(ik[5][2][1]*Wpk[0][5][1])));
        IkWpk[0][6][0] = ((ik[6][0][2]*Wpk[0][6][2])+((ik[6][0][0]*Wpk[0][6][0])
          +(ik[6][0][1]*Wpk[0][6][1])));
        IkWpk[0][6][1] = ((ik[6][1][2]*Wpk[0][6][2])+((ik[6][1][0]*Wpk[0][6][0])
          +(ik[6][1][1]*Wpk[0][6][1])));
        IkWpk[0][6][2] = ((ik[6][2][2]*Wpk[0][6][2])+((ik[6][2][0]*Wpk[0][6][0])
          +(ik[6][2][1]*Wpk[0][6][1])));
        IkWpk[0][7][0] = ((ik[7][0][2]*Wpk[0][7][2])+((ik[7][0][0]*Wpk[0][7][0])
          +(ik[7][0][1]*Wpk[0][7][1])));
        IkWpk[0][7][1] = ((ik[7][1][2]*Wpk[0][7][2])+((ik[7][1][0]*Wpk[0][7][0])
          +(ik[7][1][1]*Wpk[0][7][1])));
        IkWpk[0][7][2] = ((ik[7][2][2]*Wpk[0][7][2])+((ik[7][2][0]*Wpk[0][7][0])
          +(ik[7][2][1]*Wpk[0][7][1])));
        IkWpk[0][8][0] = ((ik[8][0][2]*Wpk[0][8][2])+((ik[8][0][0]*Wpk[0][8][0])
          +(ik[8][0][1]*Wpk[0][8][1])));
        IkWpk[0][8][1] = ((ik[8][1][2]*Wpk[0][8][2])+((ik[8][1][0]*Wpk[0][8][0])
          +(ik[8][1][1]*Wpk[0][8][1])));
        IkWpk[0][8][2] = ((ik[8][2][2]*Wpk[0][8][2])+((ik[8][2][0]*Wpk[0][8][0])
          +(ik[8][2][1]*Wpk[0][8][1])));
        IkWpk[0][9][0] = ((ik[9][0][2]*Wpk[0][9][2])+((ik[9][0][0]*Wpk[0][9][0])
          +(ik[9][0][1]*Wpk[0][9][1])));
        IkWpk[0][9][1] = ((ik[9][1][2]*Wpk[0][9][2])+((ik[9][1][0]*Wpk[0][9][0])
          +(ik[9][1][1]*Wpk[0][9][1])));
        IkWpk[0][9][2] = ((ik[9][2][2]*Wpk[0][9][2])+((ik[9][2][0]*Wpk[0][9][0])
          +(ik[9][2][1]*Wpk[0][9][1])));
        IkWpk[0][10][0] = ((ik[10][0][2]*Wpk[0][10][2])+((ik[10][0][0]*
          Wpk[0][10][0])+(ik[10][0][1]*Wpk[0][10][1])));
        IkWpk[0][10][1] = ((ik[10][1][2]*Wpk[0][10][2])+((ik[10][1][0]*
          Wpk[0][10][0])+(ik[10][1][1]*Wpk[0][10][1])));
        IkWpk[0][10][2] = ((ik[10][2][2]*Wpk[0][10][2])+((ik[10][2][0]*
          Wpk[0][10][0])+(ik[10][2][1]*Wpk[0][10][1])));
        IkWpk[0][11][0] = ((ik[11][0][2]*Wpk[0][11][2])+((ik[11][0][0]*
          Wpk[0][11][0])+(ik[11][0][1]*Wpk[0][11][1])));
        IkWpk[0][11][1] = ((ik[11][1][2]*Wpk[0][11][2])+((ik[11][1][0]*
          Wpk[0][11][0])+(ik[11][1][1]*Wpk[0][11][1])));
        IkWpk[0][11][2] = ((ik[11][2][2]*Wpk[0][11][2])+((ik[11][2][0]*
          Wpk[0][11][0])+(ik[11][2][1]*Wpk[0][11][1])));
        IkWpk[0][12][0] = ((ik[12][0][2]*Wpk[0][12][2])+((ik[12][0][0]*
          Wpk[0][12][0])+(ik[12][0][1]*Wpk[0][12][1])));
        IkWpk[0][12][1] = ((ik[12][1][2]*Wpk[0][12][2])+((ik[12][1][0]*
          Wpk[0][12][0])+(ik[12][1][1]*Wpk[0][12][1])));
        IkWpk[0][12][2] = ((ik[12][2][2]*Wpk[0][12][2])+((ik[12][2][0]*
          Wpk[0][12][0])+(ik[12][2][1]*Wpk[0][12][1])));
        IkWpk[0][13][0] = ((ik[13][0][2]*Wpk[0][13][2])+((ik[13][0][0]*
          Wpk[0][13][0])+(ik[13][0][1]*Wpk[0][13][1])));
        IkWpk[0][13][1] = ((ik[13][1][2]*Wpk[0][13][2])+((ik[13][1][0]*
          Wpk[0][13][0])+(ik[13][1][1]*Wpk[0][13][1])));
        IkWpk[0][13][2] = ((ik[13][2][2]*Wpk[0][13][2])+((ik[13][2][0]*
          Wpk[0][13][0])+(ik[13][2][1]*Wpk[0][13][1])));
        IkWpk[1][1][0] = ((ik[1][0][2]*pin[1][2])+((ik[1][0][0]*pin[1][0])+(
          ik[1][0][1]*pin[1][1])));
        IkWpk[1][1][1] = ((ik[1][1][2]*pin[1][2])+((ik[1][1][0]*pin[1][0])+(
          ik[1][1][1]*pin[1][1])));
        IkWpk[1][1][2] = ((ik[1][2][2]*pin[1][2])+((ik[1][2][0]*pin[1][0])+(
          ik[1][2][1]*pin[1][1])));
        IkWpk[2][2][0] = ((ik[2][0][2]*pin[2][2])+((ik[2][0][0]*pin[2][0])+(
          ik[2][0][1]*pin[2][1])));
        IkWpk[2][2][1] = ((ik[2][1][2]*pin[2][2])+((ik[2][1][0]*pin[2][0])+(
          ik[2][1][1]*pin[2][1])));
        IkWpk[2][2][2] = ((ik[2][2][2]*pin[2][2])+((ik[2][2][0]*pin[2][0])+(
          ik[2][2][1]*pin[2][1])));
        IkWpk[2][3][0] = ((ik[3][0][2]*Wpk[2][3][2])+((ik[3][0][0]*Wpk[2][3][0])
          +(ik[3][0][1]*Wpk[2][3][1])));
        IkWpk[2][3][1] = ((ik[3][1][2]*Wpk[2][3][2])+((ik[3][1][0]*Wpk[2][3][0])
          +(ik[3][1][1]*Wpk[2][3][1])));
        IkWpk[2][3][2] = ((ik[3][2][2]*Wpk[2][3][2])+((ik[3][2][0]*Wpk[2][3][0])
          +(ik[3][2][1]*Wpk[2][3][1])));
        IkWpk[2][4][0] = ((ik[4][0][2]*Wpk[2][4][2])+((ik[4][0][0]*Wpk[2][4][0])
          +(ik[4][0][1]*Wpk[2][4][1])));
        IkWpk[2][4][1] = ((ik[4][1][2]*Wpk[2][4][2])+((ik[4][1][0]*Wpk[2][4][0])
          +(ik[4][1][1]*Wpk[2][4][1])));
        IkWpk[2][4][2] = ((ik[4][2][2]*Wpk[2][4][2])+((ik[4][2][0]*Wpk[2][4][0])
          +(ik[4][2][1]*Wpk[2][4][1])));
        IkWpk[2][5][0] = ((ik[5][0][2]*Wpk[2][5][2])+((ik[5][0][0]*Wpk[2][5][0])
          +(ik[5][0][1]*Wpk[2][5][1])));
        IkWpk[2][5][1] = ((ik[5][1][2]*Wpk[2][5][2])+((ik[5][1][0]*Wpk[2][5][0])
          +(ik[5][1][1]*Wpk[2][5][1])));
        IkWpk[2][5][2] = ((ik[5][2][2]*Wpk[2][5][2])+((ik[5][2][0]*Wpk[2][5][0])
          +(ik[5][2][1]*Wpk[2][5][1])));
        IkWpk[2][6][0] = ((ik[6][0][2]*Wpk[2][6][2])+((ik[6][0][0]*Wpk[2][6][0])
          +(ik[6][0][1]*Wpk[2][6][1])));
        IkWpk[2][6][1] = ((ik[6][1][2]*Wpk[2][6][2])+((ik[6][1][0]*Wpk[2][6][0])
          +(ik[6][1][1]*Wpk[2][6][1])));
        IkWpk[2][6][2] = ((ik[6][2][2]*Wpk[2][6][2])+((ik[6][2][0]*Wpk[2][6][0])
          +(ik[6][2][1]*Wpk[2][6][1])));
        IkWpk[2][7][0] = ((ik[7][0][2]*Wpk[2][7][2])+((ik[7][0][0]*Wpk[2][7][0])
          +(ik[7][0][1]*Wpk[2][7][1])));
        IkWpk[2][7][1] = ((ik[7][1][2]*Wpk[2][7][2])+((ik[7][1][0]*Wpk[2][7][0])
          +(ik[7][1][1]*Wpk[2][7][1])));
        IkWpk[2][7][2] = ((ik[7][2][2]*Wpk[2][7][2])+((ik[7][2][0]*Wpk[2][7][0])
          +(ik[7][2][1]*Wpk[2][7][1])));
        IkWpk[3][3][0] = ((ik[3][0][2]*pin[3][2])+((ik[3][0][0]*pin[3][0])+(
          ik[3][0][1]*pin[3][1])));
        IkWpk[3][3][1] = ((ik[3][1][2]*pin[3][2])+((ik[3][1][0]*pin[3][0])+(
          ik[3][1][1]*pin[3][1])));
        IkWpk[3][3][2] = ((ik[3][2][2]*pin[3][2])+((ik[3][2][0]*pin[3][0])+(
          ik[3][2][1]*pin[3][1])));
        IkWpk[3][4][0] = ((ik[4][0][2]*Wpk[3][4][2])+((ik[4][0][0]*Wpk[3][4][0])
          +(ik[4][0][1]*Wpk[3][4][1])));
        IkWpk[3][4][1] = ((ik[4][1][2]*Wpk[3][4][2])+((ik[4][1][0]*Wpk[3][4][0])
          +(ik[4][1][1]*Wpk[3][4][1])));
        IkWpk[3][4][2] = ((ik[4][2][2]*Wpk[3][4][2])+((ik[4][2][0]*Wpk[3][4][0])
          +(ik[4][2][1]*Wpk[3][4][1])));
        IkWpk[3][5][0] = ((ik[5][0][2]*Wpk[3][5][2])+((ik[5][0][0]*Wpk[3][5][0])
          +(ik[5][0][1]*Wpk[3][5][1])));
        IkWpk[3][5][1] = ((ik[5][1][2]*Wpk[3][5][2])+((ik[5][1][0]*Wpk[3][5][0])
          +(ik[5][1][1]*Wpk[3][5][1])));
        IkWpk[3][5][2] = ((ik[5][2][2]*Wpk[3][5][2])+((ik[5][2][0]*Wpk[3][5][0])
          +(ik[5][2][1]*Wpk[3][5][1])));
        IkWpk[3][6][0] = ((ik[6][0][2]*Wpk[3][6][2])+((ik[6][0][0]*Wpk[3][6][0])
          +(ik[6][0][1]*Wpk[3][6][1])));
        IkWpk[3][6][1] = ((ik[6][1][2]*Wpk[3][6][2])+((ik[6][1][0]*Wpk[3][6][0])
          +(ik[6][1][1]*Wpk[3][6][1])));
        IkWpk[3][6][2] = ((ik[6][2][2]*Wpk[3][6][2])+((ik[6][2][0]*Wpk[3][6][0])
          +(ik[6][2][1]*Wpk[3][6][1])));
        IkWpk[3][7][0] = ((ik[7][0][2]*Wpk[3][7][2])+((ik[7][0][0]*Wpk[3][7][0])
          +(ik[7][0][1]*Wpk[3][7][1])));
        IkWpk[3][7][1] = ((ik[7][1][2]*Wpk[3][7][2])+((ik[7][1][0]*Wpk[3][7][0])
          +(ik[7][1][1]*Wpk[3][7][1])));
        IkWpk[3][7][2] = ((ik[7][2][2]*Wpk[3][7][2])+((ik[7][2][0]*Wpk[3][7][0])
          +(ik[7][2][1]*Wpk[3][7][1])));
        IkWpk[4][4][0] = ((ik[4][0][2]*pin[4][2])+((ik[4][0][0]*pin[4][0])+(
          ik[4][0][1]*pin[4][1])));
        IkWpk[4][4][1] = ((ik[4][1][2]*pin[4][2])+((ik[4][1][0]*pin[4][0])+(
          ik[4][1][1]*pin[4][1])));
        IkWpk[4][4][2] = ((ik[4][2][2]*pin[4][2])+((ik[4][2][0]*pin[4][0])+(
          ik[4][2][1]*pin[4][1])));
        IkWpk[4][5][0] = ((ik[5][0][2]*Wpk[4][5][2])+((ik[5][0][0]*Wpk[4][5][0])
          +(ik[5][0][1]*Wpk[4][5][1])));
        IkWpk[4][5][1] = ((ik[5][1][2]*Wpk[4][5][2])+((ik[5][1][0]*Wpk[4][5][0])
          +(ik[5][1][1]*Wpk[4][5][1])));
        IkWpk[4][5][2] = ((ik[5][2][2]*Wpk[4][5][2])+((ik[5][2][0]*Wpk[4][5][0])
          +(ik[5][2][1]*Wpk[4][5][1])));
        IkWpk[4][6][0] = ((ik[6][0][2]*Wpk[4][6][2])+((ik[6][0][0]*Wpk[4][6][0])
          +(ik[6][0][1]*Wpk[4][6][1])));
        IkWpk[4][6][1] = ((ik[6][1][2]*Wpk[4][6][2])+((ik[6][1][0]*Wpk[4][6][0])
          +(ik[6][1][1]*Wpk[4][6][1])));
        IkWpk[4][6][2] = ((ik[6][2][2]*Wpk[4][6][2])+((ik[6][2][0]*Wpk[4][6][0])
          +(ik[6][2][1]*Wpk[4][6][1])));
        IkWpk[4][7][0] = ((ik[7][0][2]*Wpk[4][7][2])+((ik[7][0][0]*Wpk[4][7][0])
          +(ik[7][0][1]*Wpk[4][7][1])));
        IkWpk[4][7][1] = ((ik[7][1][2]*Wpk[4][7][2])+((ik[7][1][0]*Wpk[4][7][0])
          +(ik[7][1][1]*Wpk[4][7][1])));
        IkWpk[4][7][2] = ((ik[7][2][2]*Wpk[4][7][2])+((ik[7][2][0]*Wpk[4][7][0])
          +(ik[7][2][1]*Wpk[4][7][1])));
        IkWpk[5][5][0] = ((ik[5][0][2]*pin[5][2])+((ik[5][0][0]*pin[5][0])+(
          ik[5][0][1]*pin[5][1])));
        IkWpk[5][5][1] = ((ik[5][1][2]*pin[5][2])+((ik[5][1][0]*pin[5][0])+(
          ik[5][1][1]*pin[5][1])));
        IkWpk[5][5][2] = ((ik[5][2][2]*pin[5][2])+((ik[5][2][0]*pin[5][0])+(
          ik[5][2][1]*pin[5][1])));
        IkWpk[5][6][0] = ((ik[6][0][2]*Wpk[5][6][2])+((ik[6][0][0]*Wpk[5][6][0])
          +(ik[6][0][1]*Wpk[5][6][1])));
        IkWpk[5][6][1] = ((ik[6][1][2]*Wpk[5][6][2])+((ik[6][1][0]*Wpk[5][6][0])
          +(ik[6][1][1]*Wpk[5][6][1])));
        IkWpk[5][6][2] = ((ik[6][2][2]*Wpk[5][6][2])+((ik[6][2][0]*Wpk[5][6][0])
          +(ik[6][2][1]*Wpk[5][6][1])));
        IkWpk[5][7][0] = ((ik[7][0][2]*Wpk[5][7][2])+((ik[7][0][0]*Wpk[5][7][0])
          +(ik[7][0][1]*Wpk[5][7][1])));
        IkWpk[5][7][1] = ((ik[7][1][2]*Wpk[5][7][2])+((ik[7][1][0]*Wpk[5][7][0])
          +(ik[7][1][1]*Wpk[5][7][1])));
        IkWpk[5][7][2] = ((ik[7][2][2]*Wpk[5][7][2])+((ik[7][2][0]*Wpk[5][7][0])
          +(ik[7][2][1]*Wpk[5][7][1])));
        IkWpk[6][6][0] = ((ik[6][0][2]*pin[6][2])+((ik[6][0][0]*pin[6][0])+(
          ik[6][0][1]*pin[6][1])));
        IkWpk[6][6][1] = ((ik[6][1][2]*pin[6][2])+((ik[6][1][0]*pin[6][0])+(
          ik[6][1][1]*pin[6][1])));
        IkWpk[6][6][2] = ((ik[6][2][2]*pin[6][2])+((ik[6][2][0]*pin[6][0])+(
          ik[6][2][1]*pin[6][1])));
        IkWpk[6][7][0] = ((ik[7][0][2]*Wpk[6][7][2])+((ik[7][0][0]*Wpk[6][7][0])
          +(ik[7][0][1]*Wpk[6][7][1])));
        IkWpk[6][7][1] = ((ik[7][1][2]*Wpk[6][7][2])+((ik[7][1][0]*Wpk[6][7][0])
          +(ik[7][1][1]*Wpk[6][7][1])));
        IkWpk[6][7][2] = ((ik[7][2][2]*Wpk[6][7][2])+((ik[7][2][0]*Wpk[6][7][0])
          +(ik[7][2][1]*Wpk[6][7][1])));
        IkWpk[7][7][0] = ((ik[7][0][2]*pin[7][2])+((ik[7][0][0]*pin[7][0])+(
          ik[7][0][1]*pin[7][1])));
        IkWpk[7][7][1] = ((ik[7][1][2]*pin[7][2])+((ik[7][1][0]*pin[7][0])+(
          ik[7][1][1]*pin[7][1])));
        IkWpk[7][7][2] = ((ik[7][2][2]*pin[7][2])+((ik[7][2][0]*pin[7][0])+(
          ik[7][2][1]*pin[7][1])));
        IkWpk[8][8][0] = ((ik[8][0][2]*pin[8][2])+((ik[8][0][0]*pin[8][0])+(
          ik[8][0][1]*pin[8][1])));
        IkWpk[8][8][1] = ((ik[8][1][2]*pin[8][2])+((ik[8][1][0]*pin[8][0])+(
          ik[8][1][1]*pin[8][1])));
        IkWpk[8][8][2] = ((ik[8][2][2]*pin[8][2])+((ik[8][2][0]*pin[8][0])+(
          ik[8][2][1]*pin[8][1])));
        IkWpk[8][9][0] = ((ik[9][0][2]*Wpk[8][9][2])+((ik[9][0][0]*Wpk[8][9][0])
          +(ik[9][0][1]*Wpk[8][9][1])));
        IkWpk[8][9][1] = ((ik[9][1][2]*Wpk[8][9][2])+((ik[9][1][0]*Wpk[8][9][0])
          +(ik[9][1][1]*Wpk[8][9][1])));
        IkWpk[8][9][2] = ((ik[9][2][2]*Wpk[8][9][2])+((ik[9][2][0]*Wpk[8][9][0])
          +(ik[9][2][1]*Wpk[8][9][1])));
        IkWpk[8][10][0] = ((ik[10][0][2]*Wpk[8][10][2])+((ik[10][0][0]*
          Wpk[8][10][0])+(ik[10][0][1]*Wpk[8][10][1])));
        IkWpk[8][10][1] = ((ik[10][1][2]*Wpk[8][10][2])+((ik[10][1][0]*
          Wpk[8][10][0])+(ik[10][1][1]*Wpk[8][10][1])));
        IkWpk[8][10][2] = ((ik[10][2][2]*Wpk[8][10][2])+((ik[10][2][0]*
          Wpk[8][10][0])+(ik[10][2][1]*Wpk[8][10][1])));
        IkWpk[8][11][0] = ((ik[11][0][2]*Wpk[8][11][2])+((ik[11][0][0]*
          Wpk[8][11][0])+(ik[11][0][1]*Wpk[8][11][1])));
        IkWpk[8][11][1] = ((ik[11][1][2]*Wpk[8][11][2])+((ik[11][1][0]*
          Wpk[8][11][0])+(ik[11][1][1]*Wpk[8][11][1])));
        IkWpk[8][11][2] = ((ik[11][2][2]*Wpk[8][11][2])+((ik[11][2][0]*
          Wpk[8][11][0])+(ik[11][2][1]*Wpk[8][11][1])));
        IkWpk[8][12][0] = ((ik[12][0][2]*Wpk[8][12][2])+((ik[12][0][0]*
          Wpk[8][12][0])+(ik[12][0][1]*Wpk[8][12][1])));
        IkWpk[8][12][1] = ((ik[12][1][2]*Wpk[8][12][2])+((ik[12][1][0]*
          Wpk[8][12][0])+(ik[12][1][1]*Wpk[8][12][1])));
        IkWpk[8][12][2] = ((ik[12][2][2]*Wpk[8][12][2])+((ik[12][2][0]*
          Wpk[8][12][0])+(ik[12][2][1]*Wpk[8][12][1])));
        IkWpk[8][13][0] = ((ik[13][0][2]*Wpk[8][13][2])+((ik[13][0][0]*
          Wpk[8][13][0])+(ik[13][0][1]*Wpk[8][13][1])));
        IkWpk[8][13][1] = ((ik[13][1][2]*Wpk[8][13][2])+((ik[13][1][0]*
          Wpk[8][13][0])+(ik[13][1][1]*Wpk[8][13][1])));
        IkWpk[8][13][2] = ((ik[13][2][2]*Wpk[8][13][2])+((ik[13][2][0]*
          Wpk[8][13][0])+(ik[13][2][1]*Wpk[8][13][1])));
        IkWpk[9][9][0] = ((ik[9][0][2]*pin[9][2])+((ik[9][0][0]*pin[9][0])+(
          ik[9][0][1]*pin[9][1])));
        IkWpk[9][9][1] = ((ik[9][1][2]*pin[9][2])+((ik[9][1][0]*pin[9][0])+(
          ik[9][1][1]*pin[9][1])));
        IkWpk[9][9][2] = ((ik[9][2][2]*pin[9][2])+((ik[9][2][0]*pin[9][0])+(
          ik[9][2][1]*pin[9][1])));
        IkWpk[9][10][0] = ((ik[10][0][2]*Wpk[9][10][2])+((ik[10][0][0]*
          Wpk[9][10][0])+(ik[10][0][1]*Wpk[9][10][1])));
        IkWpk[9][10][1] = ((ik[10][1][2]*Wpk[9][10][2])+((ik[10][1][0]*
          Wpk[9][10][0])+(ik[10][1][1]*Wpk[9][10][1])));
        IkWpk[9][10][2] = ((ik[10][2][2]*Wpk[9][10][2])+((ik[10][2][0]*
          Wpk[9][10][0])+(ik[10][2][1]*Wpk[9][10][1])));
        IkWpk[9][11][0] = ((ik[11][0][2]*Wpk[9][11][2])+((ik[11][0][0]*
          Wpk[9][11][0])+(ik[11][0][1]*Wpk[9][11][1])));
        IkWpk[9][11][1] = ((ik[11][1][2]*Wpk[9][11][2])+((ik[11][1][0]*
          Wpk[9][11][0])+(ik[11][1][1]*Wpk[9][11][1])));
        IkWpk[9][11][2] = ((ik[11][2][2]*Wpk[9][11][2])+((ik[11][2][0]*
          Wpk[9][11][0])+(ik[11][2][1]*Wpk[9][11][1])));
        IkWpk[9][12][0] = ((ik[12][0][2]*Wpk[9][12][2])+((ik[12][0][0]*
          Wpk[9][12][0])+(ik[12][0][1]*Wpk[9][12][1])));
        IkWpk[9][12][1] = ((ik[12][1][2]*Wpk[9][12][2])+((ik[12][1][0]*
          Wpk[9][12][0])+(ik[12][1][1]*Wpk[9][12][1])));
        IkWpk[9][12][2] = ((ik[12][2][2]*Wpk[9][12][2])+((ik[12][2][0]*
          Wpk[9][12][0])+(ik[12][2][1]*Wpk[9][12][1])));
        IkWpk[9][13][0] = ((ik[13][0][2]*Wpk[9][13][2])+((ik[13][0][0]*
          Wpk[9][13][0])+(ik[13][0][1]*Wpk[9][13][1])));
        IkWpk[9][13][1] = ((ik[13][1][2]*Wpk[9][13][2])+((ik[13][1][0]*
          Wpk[9][13][0])+(ik[13][1][1]*Wpk[9][13][1])));
        IkWpk[9][13][2] = ((ik[13][2][2]*Wpk[9][13][2])+((ik[13][2][0]*
          Wpk[9][13][0])+(ik[13][2][1]*Wpk[9][13][1])));
        IkWpk[10][10][0] = ((ik[10][0][2]*pin[10][2])+((ik[10][0][0]*pin[10][0])
          +(ik[10][0][1]*pin[10][1])));
        IkWpk[10][10][1] = ((ik[10][1][2]*pin[10][2])+((ik[10][1][0]*pin[10][0])
          +(ik[10][1][1]*pin[10][1])));
        IkWpk[10][10][2] = ((ik[10][2][2]*pin[10][2])+((ik[10][2][0]*pin[10][0])
          +(ik[10][2][1]*pin[10][1])));
        IkWpk[10][11][0] = ((ik[11][0][2]*Wpk[10][11][2])+((ik[11][0][0]*
          Wpk[10][11][0])+(ik[11][0][1]*Wpk[10][11][1])));
        IkWpk[10][11][1] = ((ik[11][1][2]*Wpk[10][11][2])+((ik[11][1][0]*
          Wpk[10][11][0])+(ik[11][1][1]*Wpk[10][11][1])));
        IkWpk[10][11][2] = ((ik[11][2][2]*Wpk[10][11][2])+((ik[11][2][0]*
          Wpk[10][11][0])+(ik[11][2][1]*Wpk[10][11][1])));
        IkWpk[10][12][0] = ((ik[12][0][2]*Wpk[10][12][2])+((ik[12][0][0]*
          Wpk[10][12][0])+(ik[12][0][1]*Wpk[10][12][1])));
        IkWpk[10][12][1] = ((ik[12][1][2]*Wpk[10][12][2])+((ik[12][1][0]*
          Wpk[10][12][0])+(ik[12][1][1]*Wpk[10][12][1])));
        IkWpk[10][12][2] = ((ik[12][2][2]*Wpk[10][12][2])+((ik[12][2][0]*
          Wpk[10][12][0])+(ik[12][2][1]*Wpk[10][12][1])));
        IkWpk[10][13][0] = ((ik[13][0][2]*Wpk[10][13][2])+((ik[13][0][0]*
          Wpk[10][13][0])+(ik[13][0][1]*Wpk[10][13][1])));
        IkWpk[10][13][1] = ((ik[13][1][2]*Wpk[10][13][2])+((ik[13][1][0]*
          Wpk[10][13][0])+(ik[13][1][1]*Wpk[10][13][1])));
        IkWpk[10][13][2] = ((ik[13][2][2]*Wpk[10][13][2])+((ik[13][2][0]*
          Wpk[10][13][0])+(ik[13][2][1]*Wpk[10][13][1])));
        IkWpk[11][11][0] = ((ik[11][0][2]*pin[11][2])+((ik[11][0][0]*pin[11][0])
          +(ik[11][0][1]*pin[11][1])));
        IkWpk[11][11][1] = ((ik[11][1][2]*pin[11][2])+((ik[11][1][0]*pin[11][0])
          +(ik[11][1][1]*pin[11][1])));
        IkWpk[11][11][2] = ((ik[11][2][2]*pin[11][2])+((ik[11][2][0]*pin[11][0])
          +(ik[11][2][1]*pin[11][1])));
        IkWpk[11][12][0] = ((ik[12][0][2]*Wpk[11][12][2])+((ik[12][0][0]*
          Wpk[11][12][0])+(ik[12][0][1]*Wpk[11][12][1])));
        IkWpk[11][12][1] = ((ik[12][1][2]*Wpk[11][12][2])+((ik[12][1][0]*
          Wpk[11][12][0])+(ik[12][1][1]*Wpk[11][12][1])));
        IkWpk[11][12][2] = ((ik[12][2][2]*Wpk[11][12][2])+((ik[12][2][0]*
          Wpk[11][12][0])+(ik[12][2][1]*Wpk[11][12][1])));
        IkWpk[11][13][0] = ((ik[13][0][2]*Wpk[11][13][2])+((ik[13][0][0]*
          Wpk[11][13][0])+(ik[13][0][1]*Wpk[11][13][1])));
        IkWpk[11][13][1] = ((ik[13][1][2]*Wpk[11][13][2])+((ik[13][1][0]*
          Wpk[11][13][0])+(ik[13][1][1]*Wpk[11][13][1])));
        IkWpk[11][13][2] = ((ik[13][2][2]*Wpk[11][13][2])+((ik[13][2][0]*
          Wpk[11][13][0])+(ik[13][2][1]*Wpk[11][13][1])));
        IkWpk[12][12][0] = ((ik[12][0][2]*pin[12][2])+((ik[12][0][0]*pin[12][0])
          +(ik[12][0][1]*pin[12][1])));
        IkWpk[12][12][1] = ((ik[12][1][2]*pin[12][2])+((ik[12][1][0]*pin[12][0])
          +(ik[12][1][1]*pin[12][1])));
        IkWpk[12][12][2] = ((ik[12][2][2]*pin[12][2])+((ik[12][2][0]*pin[12][0])
          +(ik[12][2][1]*pin[12][1])));
        IkWpk[12][13][0] = ((ik[13][0][2]*Wpk[12][13][2])+((ik[13][0][0]*
          Wpk[12][13][0])+(ik[13][0][1]*Wpk[12][13][1])));
        IkWpk[12][13][1] = ((ik[13][1][2]*Wpk[12][13][2])+((ik[13][1][0]*
          Wpk[12][13][0])+(ik[13][1][1]*Wpk[12][13][1])));
        IkWpk[12][13][2] = ((ik[13][2][2]*Wpk[12][13][2])+((ik[13][2][0]*
          Wpk[12][13][0])+(ik[13][2][1]*Wpk[12][13][1])));
        IkWpk[13][13][0] = ((ik[13][0][2]*pin[13][2])+((ik[13][0][0]*pin[13][0])
          +(ik[13][0][1]*pin[13][1])));
        IkWpk[13][13][1] = ((ik[13][1][2]*pin[13][2])+((ik[13][1][0]*pin[13][0])
          +(ik[13][1][1]*pin[13][1])));
        IkWpk[13][13][2] = ((ik[13][2][2]*pin[13][2])+((ik[13][2][0]*pin[13][0])
          +(ik[13][2][1]*pin[13][1])));
        temp[0] = (((mk[0]*((Vpk[0][0][2]*Vpk[0][0][2])+((Vpk[0][0][0]*
          Vpk[0][0][0])+(Vpk[0][0][1]*Vpk[0][0][1]))))+((IkWpk[0][0][2]*
          pin[0][2])+((IkWpk[0][0][0]*pin[0][0])+(IkWpk[0][0][1]*pin[0][1]))))+(
          (mk[1]*((Vpk[0][1][2]*Vpk[0][1][2])+((Vpk[0][1][0]*Vpk[0][1][0])+(
          Vpk[0][1][1]*Vpk[0][1][1]))))+((IkWpk[0][1][2]*Wpk[0][1][2])+((
          IkWpk[0][1][0]*Wpk[0][1][0])+(IkWpk[0][1][1]*Wpk[0][1][1])))));
        temp[1] = (((mk[2]*((Vpk[0][2][2]*Vpk[0][2][2])+((Vpk[0][2][0]*
          Vpk[0][2][0])+(Vpk[0][2][1]*Vpk[0][2][1]))))+((IkWpk[0][2][2]*
          Wpk[0][2][2])+((IkWpk[0][2][0]*Wpk[0][2][0])+(IkWpk[0][2][1]*
          Wpk[0][2][1]))))+temp[0]);
        temp[2] = (((mk[3]*((Vpk[0][3][2]*Vpk[0][3][2])+((Vpk[0][3][0]*
          Vpk[0][3][0])+(Vpk[0][3][1]*Vpk[0][3][1]))))+((IkWpk[0][3][2]*
          Wpk[0][3][2])+((IkWpk[0][3][0]*Wpk[0][3][0])+(IkWpk[0][3][1]*
          Wpk[0][3][1]))))+temp[1]);
        temp[3] = (((mk[4]*((Vpk[0][4][2]*Vpk[0][4][2])+((Vpk[0][4][0]*
          Vpk[0][4][0])+(Vpk[0][4][1]*Vpk[0][4][1]))))+((IkWpk[0][4][2]*
          Wpk[0][4][2])+((IkWpk[0][4][0]*Wpk[0][4][0])+(IkWpk[0][4][1]*
          Wpk[0][4][1]))))+temp[2]);
        temp[4] = (((mk[5]*((Vpk[0][5][2]*Vpk[0][5][2])+((Vpk[0][5][0]*
          Vpk[0][5][0])+(Vpk[0][5][1]*Vpk[0][5][1]))))+((IkWpk[0][5][2]*
          Wpk[0][5][2])+((IkWpk[0][5][0]*Wpk[0][5][0])+(IkWpk[0][5][1]*
          Wpk[0][5][1]))))+temp[3]);
        temp[5] = (((mk[6]*((Vpk[0][6][2]*Vpk[0][6][2])+((Vpk[0][6][0]*
          Vpk[0][6][0])+(Vpk[0][6][1]*Vpk[0][6][1]))))+((IkWpk[0][6][2]*
          Wpk[0][6][2])+((IkWpk[0][6][0]*Wpk[0][6][0])+(IkWpk[0][6][1]*
          Wpk[0][6][1]))))+temp[4]);
        temp[6] = (((mk[7]*((Vpk[0][7][2]*Vpk[0][7][2])+((Vpk[0][7][0]*
          Vpk[0][7][0])+(Vpk[0][7][1]*Vpk[0][7][1]))))+((IkWpk[0][7][2]*
          Wpk[0][7][2])+((IkWpk[0][7][0]*Wpk[0][7][0])+(IkWpk[0][7][1]*
          Wpk[0][7][1]))))+temp[5]);
        temp[7] = (((mk[8]*((Vpk[0][8][2]*Vpk[0][8][2])+((Vpk[0][8][0]*
          Vpk[0][8][0])+(Vpk[0][8][1]*Vpk[0][8][1]))))+((IkWpk[0][8][2]*
          Wpk[0][8][2])+((IkWpk[0][8][0]*Wpk[0][8][0])+(IkWpk[0][8][1]*
          Wpk[0][8][1]))))+temp[6]);
        temp[8] = (((mk[9]*((Vpk[0][9][2]*Vpk[0][9][2])+((Vpk[0][9][0]*
          Vpk[0][9][0])+(Vpk[0][9][1]*Vpk[0][9][1]))))+((IkWpk[0][9][2]*
          Wpk[0][9][2])+((IkWpk[0][9][0]*Wpk[0][9][0])+(IkWpk[0][9][1]*
          Wpk[0][9][1]))))+temp[7]);
        temp[9] = (((mk[10]*((Vpk[0][10][2]*Vpk[0][10][2])+((Vpk[0][10][0]*
          Vpk[0][10][0])+(Vpk[0][10][1]*Vpk[0][10][1]))))+((IkWpk[0][10][2]*
          Wpk[0][10][2])+((IkWpk[0][10][0]*Wpk[0][10][0])+(IkWpk[0][10][1]*
          Wpk[0][10][1]))))+temp[8]);
        temp[10] = (((mk[11]*((Vpk[0][11][2]*Vpk[0][11][2])+((Vpk[0][11][0]*
          Vpk[0][11][0])+(Vpk[0][11][1]*Vpk[0][11][1]))))+((IkWpk[0][11][2]*
          Wpk[0][11][2])+((IkWpk[0][11][0]*Wpk[0][11][0])+(IkWpk[0][11][1]*
          Wpk[0][11][1]))))+temp[9]);
        temp[11] = (((mk[12]*((Vpk[0][12][2]*Vpk[0][12][2])+((Vpk[0][12][0]*
          Vpk[0][12][0])+(Vpk[0][12][1]*Vpk[0][12][1]))))+((IkWpk[0][12][2]*
          Wpk[0][12][2])+((IkWpk[0][12][0]*Wpk[0][12][0])+(IkWpk[0][12][1]*
          Wpk[0][12][1]))))+temp[10]);
        mm[0][0] = (((mk[13]*((Vpk[0][13][2]*Vpk[0][13][2])+((Vpk[0][13][0]*
          Vpk[0][13][0])+(Vpk[0][13][1]*Vpk[0][13][1]))))+((IkWpk[0][13][2]*
          Wpk[0][13][2])+((IkWpk[0][13][0]*Wpk[0][13][0])+(IkWpk[0][13][1]*
          Wpk[0][13][1]))))+temp[11]);
        mm[0][1] = ((mk[1]*((Vpk[0][1][2]*Vpk[1][1][2])+((Vpk[0][1][0]*
          Vpk[1][1][0])+(Vpk[0][1][1]*Vpk[1][1][1]))))+((IkWpk[1][1][2]*
          Wpk[0][1][2])+((IkWpk[1][1][0]*Wpk[0][1][0])+(IkWpk[1][1][1]*
          Wpk[0][1][1]))));
        temp[0] = (((mk[2]*((Vpk[0][2][2]*Vpk[2][2][2])+((Vpk[0][2][0]*
          Vpk[2][2][0])+(Vpk[0][2][1]*Vpk[2][2][1]))))+((IkWpk[2][2][2]*
          Wpk[0][2][2])+((IkWpk[2][2][0]*Wpk[0][2][0])+(IkWpk[2][2][1]*
          Wpk[0][2][1]))))+((mk[3]*((Vpk[0][3][2]*Vpk[2][3][2])+((Vpk[0][3][0]*
          Vpk[2][3][0])+(Vpk[0][3][1]*Vpk[2][3][1]))))+((IkWpk[2][3][2]*
          Wpk[0][3][2])+((IkWpk[2][3][0]*Wpk[0][3][0])+(IkWpk[2][3][1]*
          Wpk[0][3][1])))));
        temp[1] = (((mk[4]*((Vpk[0][4][2]*Vpk[2][4][2])+((Vpk[0][4][0]*
          Vpk[2][4][0])+(Vpk[0][4][1]*Vpk[2][4][1]))))+((IkWpk[2][4][2]*
          Wpk[0][4][2])+((IkWpk[2][4][0]*Wpk[0][4][0])+(IkWpk[2][4][1]*
          Wpk[0][4][1]))))+temp[0]);
        temp[2] = (((mk[5]*((Vpk[0][5][2]*Vpk[2][5][2])+((Vpk[0][5][0]*
          Vpk[2][5][0])+(Vpk[0][5][1]*Vpk[2][5][1]))))+((IkWpk[2][5][2]*
          Wpk[0][5][2])+((IkWpk[2][5][0]*Wpk[0][5][0])+(IkWpk[2][5][1]*
          Wpk[0][5][1]))))+temp[1]);
        temp[3] = (((mk[6]*((Vpk[0][6][2]*Vpk[2][6][2])+((Vpk[0][6][0]*
          Vpk[2][6][0])+(Vpk[0][6][1]*Vpk[2][6][1]))))+((IkWpk[2][6][2]*
          Wpk[0][6][2])+((IkWpk[2][6][0]*Wpk[0][6][0])+(IkWpk[2][6][1]*
          Wpk[0][6][1]))))+temp[2]);
        mm[0][2] = (((mk[7]*((Vpk[0][7][2]*Vpk[2][7][2])+((Vpk[0][7][0]*
          Vpk[2][7][0])+(Vpk[0][7][1]*Vpk[2][7][1]))))+((IkWpk[2][7][2]*
          Wpk[0][7][2])+((IkWpk[2][7][0]*Wpk[0][7][0])+(IkWpk[2][7][1]*
          Wpk[0][7][1]))))+temp[3]);
        temp[0] = (((mk[3]*((Vpk[0][3][2]*Vpk[3][3][2])+((Vpk[0][3][0]*
          Vpk[3][3][0])+(Vpk[0][3][1]*Vpk[3][3][1]))))+((IkWpk[3][3][2]*
          Wpk[0][3][2])+((IkWpk[3][3][0]*Wpk[0][3][0])+(IkWpk[3][3][1]*
          Wpk[0][3][1]))))+((mk[4]*((Vpk[0][4][2]*Vpk[3][4][2])+((Vpk[0][4][0]*
          Vpk[3][4][0])+(Vpk[0][4][1]*Vpk[3][4][1]))))+((IkWpk[3][4][2]*
          Wpk[0][4][2])+((IkWpk[3][4][0]*Wpk[0][4][0])+(IkWpk[3][4][1]*
          Wpk[0][4][1])))));
        temp[1] = (((mk[5]*((Vpk[0][5][2]*Vpk[3][5][2])+((Vpk[0][5][0]*
          Vpk[3][5][0])+(Vpk[0][5][1]*Vpk[3][5][1]))))+((IkWpk[3][5][2]*
          Wpk[0][5][2])+((IkWpk[3][5][0]*Wpk[0][5][0])+(IkWpk[3][5][1]*
          Wpk[0][5][1]))))+temp[0]);
        temp[2] = (((mk[6]*((Vpk[0][6][2]*Vpk[3][6][2])+((Vpk[0][6][0]*
          Vpk[3][6][0])+(Vpk[0][6][1]*Vpk[3][6][1]))))+((IkWpk[3][6][2]*
          Wpk[0][6][2])+((IkWpk[3][6][0]*Wpk[0][6][0])+(IkWpk[3][6][1]*
          Wpk[0][6][1]))))+temp[1]);
        mm[0][3] = (((mk[7]*((Vpk[0][7][2]*Vpk[3][7][2])+((Vpk[0][7][0]*
          Vpk[3][7][0])+(Vpk[0][7][1]*Vpk[3][7][1]))))+((IkWpk[3][7][2]*
          Wpk[0][7][2])+((IkWpk[3][7][0]*Wpk[0][7][0])+(IkWpk[3][7][1]*
          Wpk[0][7][1]))))+temp[2]);
        temp[0] = (((mk[4]*((Vpk[0][4][2]*Vpk[4][4][2])+((Vpk[0][4][0]*
          Vpk[4][4][0])+(Vpk[0][4][1]*Vpk[4][4][1]))))+((IkWpk[4][4][2]*
          Wpk[0][4][2])+((IkWpk[4][4][0]*Wpk[0][4][0])+(IkWpk[4][4][1]*
          Wpk[0][4][1]))))+((mk[5]*((Vpk[0][5][2]*Vpk[4][5][2])+((Vpk[0][5][0]*
          Vpk[4][5][0])+(Vpk[0][5][1]*Vpk[4][5][1]))))+((IkWpk[4][5][2]*
          Wpk[0][5][2])+((IkWpk[4][5][0]*Wpk[0][5][0])+(IkWpk[4][5][1]*
          Wpk[0][5][1])))));
        temp[1] = (((mk[6]*((Vpk[0][6][2]*Vpk[4][6][2])+((Vpk[0][6][0]*
          Vpk[4][6][0])+(Vpk[0][6][1]*Vpk[4][6][1]))))+((IkWpk[4][6][2]*
          Wpk[0][6][2])+((IkWpk[4][6][0]*Wpk[0][6][0])+(IkWpk[4][6][1]*
          Wpk[0][6][1]))))+temp[0]);
        mm[0][4] = (((mk[7]*((Vpk[0][7][2]*Vpk[4][7][2])+((Vpk[0][7][0]*
          Vpk[4][7][0])+(Vpk[0][7][1]*Vpk[4][7][1]))))+((IkWpk[4][7][2]*
          Wpk[0][7][2])+((IkWpk[4][7][0]*Wpk[0][7][0])+(IkWpk[4][7][1]*
          Wpk[0][7][1]))))+temp[1]);
        temp[0] = (((mk[5]*((Vpk[0][5][2]*Vpk[5][5][2])+((Vpk[0][5][0]*
          Vpk[5][5][0])+(Vpk[0][5][1]*Vpk[5][5][1]))))+((IkWpk[5][5][2]*
          Wpk[0][5][2])+((IkWpk[5][5][0]*Wpk[0][5][0])+(IkWpk[5][5][1]*
          Wpk[0][5][1]))))+((mk[6]*((Vpk[0][6][2]*Vpk[5][6][2])+((Vpk[0][6][0]*
          Vpk[5][6][0])+(Vpk[0][6][1]*Vpk[5][6][1]))))+((IkWpk[5][6][2]*
          Wpk[0][6][2])+((IkWpk[5][6][0]*Wpk[0][6][0])+(IkWpk[5][6][1]*
          Wpk[0][6][1])))));
        mm[0][5] = (((mk[7]*((Vpk[0][7][2]*Vpk[5][7][2])+((Vpk[0][7][0]*
          Vpk[5][7][0])+(Vpk[0][7][1]*Vpk[5][7][1]))))+((IkWpk[5][7][2]*
          Wpk[0][7][2])+((IkWpk[5][7][0]*Wpk[0][7][0])+(IkWpk[5][7][1]*
          Wpk[0][7][1]))))+temp[0]);
        mm[0][6] = (((mk[6]*((Vpk[0][6][2]*Vpk[6][6][2])+((Vpk[0][6][0]*
          Vpk[6][6][0])+(Vpk[0][6][1]*Vpk[6][6][1]))))+((IkWpk[6][6][2]*
          Wpk[0][6][2])+((IkWpk[6][6][0]*Wpk[0][6][0])+(IkWpk[6][6][1]*
          Wpk[0][6][1]))))+((mk[7]*((Vpk[0][7][2]*Vpk[6][7][2])+((Vpk[0][7][0]*
          Vpk[6][7][0])+(Vpk[0][7][1]*Vpk[6][7][1]))))+((IkWpk[6][7][2]*
          Wpk[0][7][2])+((IkWpk[6][7][0]*Wpk[0][7][0])+(IkWpk[6][7][1]*
          Wpk[0][7][1])))));
        mm[0][7] = ((mk[7]*((Vpk[0][7][2]*Vpk[7][7][2])+((Vpk[0][7][0]*
          Vpk[7][7][0])+(Vpk[0][7][1]*Vpk[7][7][1]))))+((IkWpk[7][7][2]*
          Wpk[0][7][2])+((IkWpk[7][7][0]*Wpk[0][7][0])+(IkWpk[7][7][1]*
          Wpk[0][7][1]))));
        temp[0] = (((mk[8]*((Vpk[0][8][2]*Vpk[8][8][2])+((Vpk[0][8][0]*
          Vpk[8][8][0])+(Vpk[0][8][1]*Vpk[8][8][1]))))+((IkWpk[8][8][2]*
          Wpk[0][8][2])+((IkWpk[8][8][0]*Wpk[0][8][0])+(IkWpk[8][8][1]*
          Wpk[0][8][1]))))+((mk[9]*((Vpk[0][9][2]*Vpk[8][9][2])+((Vpk[0][9][0]*
          Vpk[8][9][0])+(Vpk[0][9][1]*Vpk[8][9][1]))))+((IkWpk[8][9][2]*
          Wpk[0][9][2])+((IkWpk[8][9][0]*Wpk[0][9][0])+(IkWpk[8][9][1]*
          Wpk[0][9][1])))));
        temp[1] = (((mk[10]*((Vpk[0][10][2]*Vpk[8][10][2])+((Vpk[0][10][0]*
          Vpk[8][10][0])+(Vpk[0][10][1]*Vpk[8][10][1]))))+((IkWpk[8][10][2]*
          Wpk[0][10][2])+((IkWpk[8][10][0]*Wpk[0][10][0])+(IkWpk[8][10][1]*
          Wpk[0][10][1]))))+temp[0]);
        temp[2] = (((mk[11]*((Vpk[0][11][2]*Vpk[8][11][2])+((Vpk[0][11][0]*
          Vpk[8][11][0])+(Vpk[0][11][1]*Vpk[8][11][1]))))+((IkWpk[8][11][2]*
          Wpk[0][11][2])+((IkWpk[8][11][0]*Wpk[0][11][0])+(IkWpk[8][11][1]*
          Wpk[0][11][1]))))+temp[1]);
        temp[3] = (((mk[12]*((Vpk[0][12][2]*Vpk[8][12][2])+((Vpk[0][12][0]*
          Vpk[8][12][0])+(Vpk[0][12][1]*Vpk[8][12][1]))))+((IkWpk[8][12][2]*
          Wpk[0][12][2])+((IkWpk[8][12][0]*Wpk[0][12][0])+(IkWpk[8][12][1]*
          Wpk[0][12][1]))))+temp[2]);
        mm[0][8] = (((mk[13]*((Vpk[0][13][2]*Vpk[8][13][2])+((Vpk[0][13][0]*
          Vpk[8][13][0])+(Vpk[0][13][1]*Vpk[8][13][1]))))+((IkWpk[8][13][2]*
          Wpk[0][13][2])+((IkWpk[8][13][0]*Wpk[0][13][0])+(IkWpk[8][13][1]*
          Wpk[0][13][1]))))+temp[3]);
        temp[0] = (((mk[9]*((Vpk[0][9][2]*Vpk[9][9][2])+((Vpk[0][9][0]*
          Vpk[9][9][0])+(Vpk[0][9][1]*Vpk[9][9][1]))))+((IkWpk[9][9][2]*
          Wpk[0][9][2])+((IkWpk[9][9][0]*Wpk[0][9][0])+(IkWpk[9][9][1]*
          Wpk[0][9][1]))))+((mk[10]*((Vpk[0][10][2]*Vpk[9][10][2])+((
          Vpk[0][10][0]*Vpk[9][10][0])+(Vpk[0][10][1]*Vpk[9][10][1]))))+((
          IkWpk[9][10][2]*Wpk[0][10][2])+((IkWpk[9][10][0]*Wpk[0][10][0])+(
          IkWpk[9][10][1]*Wpk[0][10][1])))));
        temp[1] = (((mk[11]*((Vpk[0][11][2]*Vpk[9][11][2])+((Vpk[0][11][0]*
          Vpk[9][11][0])+(Vpk[0][11][1]*Vpk[9][11][1]))))+((IkWpk[9][11][2]*
          Wpk[0][11][2])+((IkWpk[9][11][0]*Wpk[0][11][0])+(IkWpk[9][11][1]*
          Wpk[0][11][1]))))+temp[0]);
        temp[2] = (((mk[12]*((Vpk[0][12][2]*Vpk[9][12][2])+((Vpk[0][12][0]*
          Vpk[9][12][0])+(Vpk[0][12][1]*Vpk[9][12][1]))))+((IkWpk[9][12][2]*
          Wpk[0][12][2])+((IkWpk[9][12][0]*Wpk[0][12][0])+(IkWpk[9][12][1]*
          Wpk[0][12][1]))))+temp[1]);
        mm[0][9] = (((mk[13]*((Vpk[0][13][2]*Vpk[9][13][2])+((Vpk[0][13][0]*
          Vpk[9][13][0])+(Vpk[0][13][1]*Vpk[9][13][1]))))+((IkWpk[9][13][2]*
          Wpk[0][13][2])+((IkWpk[9][13][0]*Wpk[0][13][0])+(IkWpk[9][13][1]*
          Wpk[0][13][1]))))+temp[2]);
        temp[0] = (((mk[10]*((Vpk[0][10][2]*Vpk[10][10][2])+((Vpk[0][10][0]*
          Vpk[10][10][0])+(Vpk[0][10][1]*Vpk[10][10][1]))))+((IkWpk[10][10][2]*
          Wpk[0][10][2])+((IkWpk[10][10][0]*Wpk[0][10][0])+(IkWpk[10][10][1]*
          Wpk[0][10][1]))))+((mk[11]*((Vpk[0][11][2]*Vpk[10][11][2])+((
          Vpk[0][11][0]*Vpk[10][11][0])+(Vpk[0][11][1]*Vpk[10][11][1]))))+((
          IkWpk[10][11][2]*Wpk[0][11][2])+((IkWpk[10][11][0]*Wpk[0][11][0])+(
          IkWpk[10][11][1]*Wpk[0][11][1])))));
        temp[1] = (((mk[12]*((Vpk[0][12][2]*Vpk[10][12][2])+((Vpk[0][12][0]*
          Vpk[10][12][0])+(Vpk[0][12][1]*Vpk[10][12][1]))))+((IkWpk[10][12][2]*
          Wpk[0][12][2])+((IkWpk[10][12][0]*Wpk[0][12][0])+(IkWpk[10][12][1]*
          Wpk[0][12][1]))))+temp[0]);
        mm[0][10] = (((mk[13]*((Vpk[0][13][2]*Vpk[10][13][2])+((Vpk[0][13][0]*
          Vpk[10][13][0])+(Vpk[0][13][1]*Vpk[10][13][1]))))+((IkWpk[10][13][2]*
          Wpk[0][13][2])+((IkWpk[10][13][0]*Wpk[0][13][0])+(IkWpk[10][13][1]*
          Wpk[0][13][1]))))+temp[1]);
        temp[0] = (((mk[11]*((Vpk[0][11][2]*Vpk[11][11][2])+((Vpk[0][11][0]*
          Vpk[11][11][0])+(Vpk[0][11][1]*Vpk[11][11][1]))))+((IkWpk[11][11][2]*
          Wpk[0][11][2])+((IkWpk[11][11][0]*Wpk[0][11][0])+(IkWpk[11][11][1]*
          Wpk[0][11][1]))))+((mk[12]*((Vpk[0][12][2]*Vpk[11][12][2])+((
          Vpk[0][12][0]*Vpk[11][12][0])+(Vpk[0][12][1]*Vpk[11][12][1]))))+((
          IkWpk[11][12][2]*Wpk[0][12][2])+((IkWpk[11][12][0]*Wpk[0][12][0])+(
          IkWpk[11][12][1]*Wpk[0][12][1])))));
        mm[0][11] = (((mk[13]*((Vpk[0][13][2]*Vpk[11][13][2])+((Vpk[0][13][0]*
          Vpk[11][13][0])+(Vpk[0][13][1]*Vpk[11][13][1]))))+((IkWpk[11][13][2]*
          Wpk[0][13][2])+((IkWpk[11][13][0]*Wpk[0][13][0])+(IkWpk[11][13][1]*
          Wpk[0][13][1]))))+temp[0]);
        mm[0][12] = (((mk[12]*((Vpk[0][12][2]*Vpk[12][12][2])+((Vpk[0][12][0]*
          Vpk[12][12][0])+(Vpk[0][12][1]*Vpk[12][12][1]))))+((IkWpk[12][12][2]*
          Wpk[0][12][2])+((IkWpk[12][12][0]*Wpk[0][12][0])+(IkWpk[12][12][1]*
          Wpk[0][12][1]))))+((mk[13]*((Vpk[0][13][2]*Vpk[12][13][2])+((
          Vpk[0][13][0]*Vpk[12][13][0])+(Vpk[0][13][1]*Vpk[12][13][1]))))+((
          IkWpk[12][13][2]*Wpk[0][13][2])+((IkWpk[12][13][0]*Wpk[0][13][0])+(
          IkWpk[12][13][1]*Wpk[0][13][1])))));
        mm[0][13] = ((mk[13]*((Vpk[0][13][2]*Vpk[13][13][2])+((Vpk[0][13][0]*
          Vpk[13][13][0])+(Vpk[0][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          Wpk[0][13][2])+((IkWpk[13][13][0]*Wpk[0][13][0])+(IkWpk[13][13][1]*
          Wpk[0][13][1]))));
        mm[1][1] = ((mk[1]*((Vpk[1][1][2]*Vpk[1][1][2])+((Vpk[1][1][0]*
          Vpk[1][1][0])+(Vpk[1][1][1]*Vpk[1][1][1]))))+((IkWpk[1][1][2]*
          pin[1][2])+((IkWpk[1][1][0]*pin[1][0])+(IkWpk[1][1][1]*pin[1][1]))));
        mm[1][2] = 0.;
        mm[1][3] = 0.;
        mm[1][4] = 0.;
        mm[1][5] = 0.;
        mm[1][6] = 0.;
        mm[1][7] = 0.;
        mm[1][8] = 0.;
        mm[1][9] = 0.;
        mm[1][10] = 0.;
        mm[1][11] = 0.;
        mm[1][12] = 0.;
        mm[1][13] = 0.;
        temp[0] = (((mk[2]*((Vpk[2][2][2]*Vpk[2][2][2])+((Vpk[2][2][0]*
          Vpk[2][2][0])+(Vpk[2][2][1]*Vpk[2][2][1]))))+((IkWpk[2][2][2]*
          pin[2][2])+((IkWpk[2][2][0]*pin[2][0])+(IkWpk[2][2][1]*pin[2][1]))))+(
          (mk[3]*((Vpk[2][3][2]*Vpk[2][3][2])+((Vpk[2][3][0]*Vpk[2][3][0])+(
          Vpk[2][3][1]*Vpk[2][3][1]))))+((IkWpk[2][3][2]*Wpk[2][3][2])+((
          IkWpk[2][3][0]*Wpk[2][3][0])+(IkWpk[2][3][1]*Wpk[2][3][1])))));
        temp[1] = (((mk[4]*((Vpk[2][4][2]*Vpk[2][4][2])+((Vpk[2][4][0]*
          Vpk[2][4][0])+(Vpk[2][4][1]*Vpk[2][4][1]))))+((IkWpk[2][4][2]*
          Wpk[2][4][2])+((IkWpk[2][4][0]*Wpk[2][4][0])+(IkWpk[2][4][1]*
          Wpk[2][4][1]))))+temp[0]);
        temp[2] = (((mk[5]*((Vpk[2][5][2]*Vpk[2][5][2])+((Vpk[2][5][0]*
          Vpk[2][5][0])+(Vpk[2][5][1]*Vpk[2][5][1]))))+((IkWpk[2][5][2]*
          Wpk[2][5][2])+((IkWpk[2][5][0]*Wpk[2][5][0])+(IkWpk[2][5][1]*
          Wpk[2][5][1]))))+temp[1]);
        temp[3] = (((mk[6]*((Vpk[2][6][2]*Vpk[2][6][2])+((Vpk[2][6][0]*
          Vpk[2][6][0])+(Vpk[2][6][1]*Vpk[2][6][1]))))+((IkWpk[2][6][2]*
          Wpk[2][6][2])+((IkWpk[2][6][0]*Wpk[2][6][0])+(IkWpk[2][6][1]*
          Wpk[2][6][1]))))+temp[2]);
        mm[2][2] = (((mk[7]*((Vpk[2][7][2]*Vpk[2][7][2])+((Vpk[2][7][0]*
          Vpk[2][7][0])+(Vpk[2][7][1]*Vpk[2][7][1]))))+((IkWpk[2][7][2]*
          Wpk[2][7][2])+((IkWpk[2][7][0]*Wpk[2][7][0])+(IkWpk[2][7][1]*
          Wpk[2][7][1]))))+temp[3]);
        temp[0] = (((mk[3]*((Vpk[2][3][2]*Vpk[3][3][2])+((Vpk[2][3][0]*
          Vpk[3][3][0])+(Vpk[2][3][1]*Vpk[3][3][1]))))+((IkWpk[3][3][2]*
          Wpk[2][3][2])+((IkWpk[3][3][0]*Wpk[2][3][0])+(IkWpk[3][3][1]*
          Wpk[2][3][1]))))+((mk[4]*((Vpk[2][4][2]*Vpk[3][4][2])+((Vpk[2][4][0]*
          Vpk[3][4][0])+(Vpk[2][4][1]*Vpk[3][4][1]))))+((IkWpk[3][4][2]*
          Wpk[2][4][2])+((IkWpk[3][4][0]*Wpk[2][4][0])+(IkWpk[3][4][1]*
          Wpk[2][4][1])))));
        temp[1] = (((mk[5]*((Vpk[2][5][2]*Vpk[3][5][2])+((Vpk[2][5][0]*
          Vpk[3][5][0])+(Vpk[2][5][1]*Vpk[3][5][1]))))+((IkWpk[3][5][2]*
          Wpk[2][5][2])+((IkWpk[3][5][0]*Wpk[2][5][0])+(IkWpk[3][5][1]*
          Wpk[2][5][1]))))+temp[0]);
        temp[2] = (((mk[6]*((Vpk[2][6][2]*Vpk[3][6][2])+((Vpk[2][6][0]*
          Vpk[3][6][0])+(Vpk[2][6][1]*Vpk[3][6][1]))))+((IkWpk[3][6][2]*
          Wpk[2][6][2])+((IkWpk[3][6][0]*Wpk[2][6][0])+(IkWpk[3][6][1]*
          Wpk[2][6][1]))))+temp[1]);
        mm[2][3] = (((mk[7]*((Vpk[2][7][2]*Vpk[3][7][2])+((Vpk[2][7][0]*
          Vpk[3][7][0])+(Vpk[2][7][1]*Vpk[3][7][1]))))+((IkWpk[3][7][2]*
          Wpk[2][7][2])+((IkWpk[3][7][0]*Wpk[2][7][0])+(IkWpk[3][7][1]*
          Wpk[2][7][1]))))+temp[2]);
        temp[0] = (((mk[4]*((Vpk[2][4][2]*Vpk[4][4][2])+((Vpk[2][4][0]*
          Vpk[4][4][0])+(Vpk[2][4][1]*Vpk[4][4][1]))))+((IkWpk[4][4][2]*
          Wpk[2][4][2])+((IkWpk[4][4][0]*Wpk[2][4][0])+(IkWpk[4][4][1]*
          Wpk[2][4][1]))))+((mk[5]*((Vpk[2][5][2]*Vpk[4][5][2])+((Vpk[2][5][0]*
          Vpk[4][5][0])+(Vpk[2][5][1]*Vpk[4][5][1]))))+((IkWpk[4][5][2]*
          Wpk[2][5][2])+((IkWpk[4][5][0]*Wpk[2][5][0])+(IkWpk[4][5][1]*
          Wpk[2][5][1])))));
        temp[1] = (((mk[6]*((Vpk[2][6][2]*Vpk[4][6][2])+((Vpk[2][6][0]*
          Vpk[4][6][0])+(Vpk[2][6][1]*Vpk[4][6][1]))))+((IkWpk[4][6][2]*
          Wpk[2][6][2])+((IkWpk[4][6][0]*Wpk[2][6][0])+(IkWpk[4][6][1]*
          Wpk[2][6][1]))))+temp[0]);
        mm[2][4] = (((mk[7]*((Vpk[2][7][2]*Vpk[4][7][2])+((Vpk[2][7][0]*
          Vpk[4][7][0])+(Vpk[2][7][1]*Vpk[4][7][1]))))+((IkWpk[4][7][2]*
          Wpk[2][7][2])+((IkWpk[4][7][0]*Wpk[2][7][0])+(IkWpk[4][7][1]*
          Wpk[2][7][1]))))+temp[1]);
        temp[0] = (((mk[5]*((Vpk[2][5][2]*Vpk[5][5][2])+((Vpk[2][5][0]*
          Vpk[5][5][0])+(Vpk[2][5][1]*Vpk[5][5][1]))))+((IkWpk[5][5][2]*
          Wpk[2][5][2])+((IkWpk[5][5][0]*Wpk[2][5][0])+(IkWpk[5][5][1]*
          Wpk[2][5][1]))))+((mk[6]*((Vpk[2][6][2]*Vpk[5][6][2])+((Vpk[2][6][0]*
          Vpk[5][6][0])+(Vpk[2][6][1]*Vpk[5][6][1]))))+((IkWpk[5][6][2]*
          Wpk[2][6][2])+((IkWpk[5][6][0]*Wpk[2][6][0])+(IkWpk[5][6][1]*
          Wpk[2][6][1])))));
        mm[2][5] = (((mk[7]*((Vpk[2][7][2]*Vpk[5][7][2])+((Vpk[2][7][0]*
          Vpk[5][7][0])+(Vpk[2][7][1]*Vpk[5][7][1]))))+((IkWpk[5][7][2]*
          Wpk[2][7][2])+((IkWpk[5][7][0]*Wpk[2][7][0])+(IkWpk[5][7][1]*
          Wpk[2][7][1]))))+temp[0]);
        mm[2][6] = (((mk[6]*((Vpk[2][6][2]*Vpk[6][6][2])+((Vpk[2][6][0]*
          Vpk[6][6][0])+(Vpk[2][6][1]*Vpk[6][6][1]))))+((IkWpk[6][6][2]*
          Wpk[2][6][2])+((IkWpk[6][6][0]*Wpk[2][6][0])+(IkWpk[6][6][1]*
          Wpk[2][6][1]))))+((mk[7]*((Vpk[2][7][2]*Vpk[6][7][2])+((Vpk[2][7][0]*
          Vpk[6][7][0])+(Vpk[2][7][1]*Vpk[6][7][1]))))+((IkWpk[6][7][2]*
          Wpk[2][7][2])+((IkWpk[6][7][0]*Wpk[2][7][0])+(IkWpk[6][7][1]*
          Wpk[2][7][1])))));
        mm[2][7] = ((mk[7]*((Vpk[2][7][2]*Vpk[7][7][2])+((Vpk[2][7][0]*
          Vpk[7][7][0])+(Vpk[2][7][1]*Vpk[7][7][1]))))+((IkWpk[7][7][2]*
          Wpk[2][7][2])+((IkWpk[7][7][0]*Wpk[2][7][0])+(IkWpk[7][7][1]*
          Wpk[2][7][1]))));
        mm[2][8] = 0.;
        mm[2][9] = 0.;
        mm[2][10] = 0.;
        mm[2][11] = 0.;
        mm[2][12] = 0.;
        mm[2][13] = 0.;
        temp[0] = (((mk[3]*((Vpk[3][3][2]*Vpk[3][3][2])+((Vpk[3][3][0]*
          Vpk[3][3][0])+(Vpk[3][3][1]*Vpk[3][3][1]))))+((IkWpk[3][3][2]*
          pin[3][2])+((IkWpk[3][3][0]*pin[3][0])+(IkWpk[3][3][1]*pin[3][1]))))+(
          (mk[4]*((Vpk[3][4][2]*Vpk[3][4][2])+((Vpk[3][4][0]*Vpk[3][4][0])+(
          Vpk[3][4][1]*Vpk[3][4][1]))))+((IkWpk[3][4][2]*Wpk[3][4][2])+((
          IkWpk[3][4][0]*Wpk[3][4][0])+(IkWpk[3][4][1]*Wpk[3][4][1])))));
        temp[1] = (((mk[5]*((Vpk[3][5][2]*Vpk[3][5][2])+((Vpk[3][5][0]*
          Vpk[3][5][0])+(Vpk[3][5][1]*Vpk[3][5][1]))))+((IkWpk[3][5][2]*
          Wpk[3][5][2])+((IkWpk[3][5][0]*Wpk[3][5][0])+(IkWpk[3][5][1]*
          Wpk[3][5][1]))))+temp[0]);
        temp[2] = (((mk[6]*((Vpk[3][6][2]*Vpk[3][6][2])+((Vpk[3][6][0]*
          Vpk[3][6][0])+(Vpk[3][6][1]*Vpk[3][6][1]))))+((IkWpk[3][6][2]*
          Wpk[3][6][2])+((IkWpk[3][6][0]*Wpk[3][6][0])+(IkWpk[3][6][1]*
          Wpk[3][6][1]))))+temp[1]);
        mm[3][3] = (((mk[7]*((Vpk[3][7][2]*Vpk[3][7][2])+((Vpk[3][7][0]*
          Vpk[3][7][0])+(Vpk[3][7][1]*Vpk[3][7][1]))))+((IkWpk[3][7][2]*
          Wpk[3][7][2])+((IkWpk[3][7][0]*Wpk[3][7][0])+(IkWpk[3][7][1]*
          Wpk[3][7][1]))))+temp[2]);
        temp[0] = (((mk[4]*((Vpk[3][4][2]*Vpk[4][4][2])+((Vpk[3][4][0]*
          Vpk[4][4][0])+(Vpk[3][4][1]*Vpk[4][4][1]))))+((IkWpk[4][4][2]*
          Wpk[3][4][2])+((IkWpk[4][4][0]*Wpk[3][4][0])+(IkWpk[4][4][1]*
          Wpk[3][4][1]))))+((mk[5]*((Vpk[3][5][2]*Vpk[4][5][2])+((Vpk[3][5][0]*
          Vpk[4][5][0])+(Vpk[3][5][1]*Vpk[4][5][1]))))+((IkWpk[4][5][2]*
          Wpk[3][5][2])+((IkWpk[4][5][0]*Wpk[3][5][0])+(IkWpk[4][5][1]*
          Wpk[3][5][1])))));
        temp[1] = (((mk[6]*((Vpk[3][6][2]*Vpk[4][6][2])+((Vpk[3][6][0]*
          Vpk[4][6][0])+(Vpk[3][6][1]*Vpk[4][6][1]))))+((IkWpk[4][6][2]*
          Wpk[3][6][2])+((IkWpk[4][6][0]*Wpk[3][6][0])+(IkWpk[4][6][1]*
          Wpk[3][6][1]))))+temp[0]);
        mm[3][4] = (((mk[7]*((Vpk[3][7][2]*Vpk[4][7][2])+((Vpk[3][7][0]*
          Vpk[4][7][0])+(Vpk[3][7][1]*Vpk[4][7][1]))))+((IkWpk[4][7][2]*
          Wpk[3][7][2])+((IkWpk[4][7][0]*Wpk[3][7][0])+(IkWpk[4][7][1]*
          Wpk[3][7][1]))))+temp[1]);
        temp[0] = (((mk[5]*((Vpk[3][5][2]*Vpk[5][5][2])+((Vpk[3][5][0]*
          Vpk[5][5][0])+(Vpk[3][5][1]*Vpk[5][5][1]))))+((IkWpk[5][5][2]*
          Wpk[3][5][2])+((IkWpk[5][5][0]*Wpk[3][5][0])+(IkWpk[5][5][1]*
          Wpk[3][5][1]))))+((mk[6]*((Vpk[3][6][2]*Vpk[5][6][2])+((Vpk[3][6][0]*
          Vpk[5][6][0])+(Vpk[3][6][1]*Vpk[5][6][1]))))+((IkWpk[5][6][2]*
          Wpk[3][6][2])+((IkWpk[5][6][0]*Wpk[3][6][0])+(IkWpk[5][6][1]*
          Wpk[3][6][1])))));
        mm[3][5] = (((mk[7]*((Vpk[3][7][2]*Vpk[5][7][2])+((Vpk[3][7][0]*
          Vpk[5][7][0])+(Vpk[3][7][1]*Vpk[5][7][1]))))+((IkWpk[5][7][2]*
          Wpk[3][7][2])+((IkWpk[5][7][0]*Wpk[3][7][0])+(IkWpk[5][7][1]*
          Wpk[3][7][1]))))+temp[0]);
        mm[3][6] = (((mk[6]*((Vpk[3][6][2]*Vpk[6][6][2])+((Vpk[3][6][0]*
          Vpk[6][6][0])+(Vpk[3][6][1]*Vpk[6][6][1]))))+((IkWpk[6][6][2]*
          Wpk[3][6][2])+((IkWpk[6][6][0]*Wpk[3][6][0])+(IkWpk[6][6][1]*
          Wpk[3][6][1]))))+((mk[7]*((Vpk[3][7][2]*Vpk[6][7][2])+((Vpk[3][7][0]*
          Vpk[6][7][0])+(Vpk[3][7][1]*Vpk[6][7][1]))))+((IkWpk[6][7][2]*
          Wpk[3][7][2])+((IkWpk[6][7][0]*Wpk[3][7][0])+(IkWpk[6][7][1]*
          Wpk[3][7][1])))));
        mm[3][7] = ((mk[7]*((Vpk[3][7][2]*Vpk[7][7][2])+((Vpk[3][7][0]*
          Vpk[7][7][0])+(Vpk[3][7][1]*Vpk[7][7][1]))))+((IkWpk[7][7][2]*
          Wpk[3][7][2])+((IkWpk[7][7][0]*Wpk[3][7][0])+(IkWpk[7][7][1]*
          Wpk[3][7][1]))));
        mm[3][8] = 0.;
        mm[3][9] = 0.;
        mm[3][10] = 0.;
        mm[3][11] = 0.;
        mm[3][12] = 0.;
        mm[3][13] = 0.;
        temp[0] = (((mk[4]*((Vpk[4][4][2]*Vpk[4][4][2])+((Vpk[4][4][0]*
          Vpk[4][4][0])+(Vpk[4][4][1]*Vpk[4][4][1]))))+((IkWpk[4][4][2]*
          pin[4][2])+((IkWpk[4][4][0]*pin[4][0])+(IkWpk[4][4][1]*pin[4][1]))))+(
          (mk[5]*((Vpk[4][5][2]*Vpk[4][5][2])+((Vpk[4][5][0]*Vpk[4][5][0])+(
          Vpk[4][5][1]*Vpk[4][5][1]))))+((IkWpk[4][5][2]*Wpk[4][5][2])+((
          IkWpk[4][5][0]*Wpk[4][5][0])+(IkWpk[4][5][1]*Wpk[4][5][1])))));
        temp[1] = (((mk[6]*((Vpk[4][6][2]*Vpk[4][6][2])+((Vpk[4][6][0]*
          Vpk[4][6][0])+(Vpk[4][6][1]*Vpk[4][6][1]))))+((IkWpk[4][6][2]*
          Wpk[4][6][2])+((IkWpk[4][6][0]*Wpk[4][6][0])+(IkWpk[4][6][1]*
          Wpk[4][6][1]))))+temp[0]);
        mm[4][4] = (((mk[7]*((Vpk[4][7][2]*Vpk[4][7][2])+((Vpk[4][7][0]*
          Vpk[4][7][0])+(Vpk[4][7][1]*Vpk[4][7][1]))))+((IkWpk[4][7][2]*
          Wpk[4][7][2])+((IkWpk[4][7][0]*Wpk[4][7][0])+(IkWpk[4][7][1]*
          Wpk[4][7][1]))))+temp[1]);
        temp[0] = (((mk[5]*((Vpk[4][5][2]*Vpk[5][5][2])+((Vpk[4][5][0]*
          Vpk[5][5][0])+(Vpk[4][5][1]*Vpk[5][5][1]))))+((IkWpk[5][5][2]*
          Wpk[4][5][2])+((IkWpk[5][5][0]*Wpk[4][5][0])+(IkWpk[5][5][1]*
          Wpk[4][5][1]))))+((mk[6]*((Vpk[4][6][2]*Vpk[5][6][2])+((Vpk[4][6][0]*
          Vpk[5][6][0])+(Vpk[4][6][1]*Vpk[5][6][1]))))+((IkWpk[5][6][2]*
          Wpk[4][6][2])+((IkWpk[5][6][0]*Wpk[4][6][0])+(IkWpk[5][6][1]*
          Wpk[4][6][1])))));
        mm[4][5] = (((mk[7]*((Vpk[4][7][2]*Vpk[5][7][2])+((Vpk[4][7][0]*
          Vpk[5][7][0])+(Vpk[4][7][1]*Vpk[5][7][1]))))+((IkWpk[5][7][2]*
          Wpk[4][7][2])+((IkWpk[5][7][0]*Wpk[4][7][0])+(IkWpk[5][7][1]*
          Wpk[4][7][1]))))+temp[0]);
        mm[4][6] = (((mk[6]*((Vpk[4][6][2]*Vpk[6][6][2])+((Vpk[4][6][0]*
          Vpk[6][6][0])+(Vpk[4][6][1]*Vpk[6][6][1]))))+((IkWpk[6][6][2]*
          Wpk[4][6][2])+((IkWpk[6][6][0]*Wpk[4][6][0])+(IkWpk[6][6][1]*
          Wpk[4][6][1]))))+((mk[7]*((Vpk[4][7][2]*Vpk[6][7][2])+((Vpk[4][7][0]*
          Vpk[6][7][0])+(Vpk[4][7][1]*Vpk[6][7][1]))))+((IkWpk[6][7][2]*
          Wpk[4][7][2])+((IkWpk[6][7][0]*Wpk[4][7][0])+(IkWpk[6][7][1]*
          Wpk[4][7][1])))));
        mm[4][7] = ((mk[7]*((Vpk[4][7][2]*Vpk[7][7][2])+((Vpk[4][7][0]*
          Vpk[7][7][0])+(Vpk[4][7][1]*Vpk[7][7][1]))))+((IkWpk[7][7][2]*
          Wpk[4][7][2])+((IkWpk[7][7][0]*Wpk[4][7][0])+(IkWpk[7][7][1]*
          Wpk[4][7][1]))));
        mm[4][8] = 0.;
        mm[4][9] = 0.;
        mm[4][10] = 0.;
        mm[4][11] = 0.;
        mm[4][12] = 0.;
        mm[4][13] = 0.;
        temp[0] = (((mk[5]*((Vpk[5][5][2]*Vpk[5][5][2])+((Vpk[5][5][0]*
          Vpk[5][5][0])+(Vpk[5][5][1]*Vpk[5][5][1]))))+((IkWpk[5][5][2]*
          pin[5][2])+((IkWpk[5][5][0]*pin[5][0])+(IkWpk[5][5][1]*pin[5][1]))))+(
          (mk[6]*((Vpk[5][6][2]*Vpk[5][6][2])+((Vpk[5][6][0]*Vpk[5][6][0])+(
          Vpk[5][6][1]*Vpk[5][6][1]))))+((IkWpk[5][6][2]*Wpk[5][6][2])+((
          IkWpk[5][6][0]*Wpk[5][6][0])+(IkWpk[5][6][1]*Wpk[5][6][1])))));
        mm[5][5] = (((mk[7]*((Vpk[5][7][2]*Vpk[5][7][2])+((Vpk[5][7][0]*
          Vpk[5][7][0])+(Vpk[5][7][1]*Vpk[5][7][1]))))+((IkWpk[5][7][2]*
          Wpk[5][7][2])+((IkWpk[5][7][0]*Wpk[5][7][0])+(IkWpk[5][7][1]*
          Wpk[5][7][1]))))+temp[0]);
        mm[5][6] = (((mk[6]*((Vpk[5][6][2]*Vpk[6][6][2])+((Vpk[5][6][0]*
          Vpk[6][6][0])+(Vpk[5][6][1]*Vpk[6][6][1]))))+((IkWpk[6][6][2]*
          Wpk[5][6][2])+((IkWpk[6][6][0]*Wpk[5][6][0])+(IkWpk[6][6][1]*
          Wpk[5][6][1]))))+((mk[7]*((Vpk[5][7][2]*Vpk[6][7][2])+((Vpk[5][7][0]*
          Vpk[6][7][0])+(Vpk[5][7][1]*Vpk[6][7][1]))))+((IkWpk[6][7][2]*
          Wpk[5][7][2])+((IkWpk[6][7][0]*Wpk[5][7][0])+(IkWpk[6][7][1]*
          Wpk[5][7][1])))));
        mm[5][7] = ((mk[7]*((Vpk[5][7][2]*Vpk[7][7][2])+((Vpk[5][7][0]*
          Vpk[7][7][0])+(Vpk[5][7][1]*Vpk[7][7][1]))))+((IkWpk[7][7][2]*
          Wpk[5][7][2])+((IkWpk[7][7][0]*Wpk[5][7][0])+(IkWpk[7][7][1]*
          Wpk[5][7][1]))));
        mm[5][8] = 0.;
        mm[5][9] = 0.;
        mm[5][10] = 0.;
        mm[5][11] = 0.;
        mm[5][12] = 0.;
        mm[5][13] = 0.;
        mm[6][6] = (((mk[6]*((Vpk[6][6][2]*Vpk[6][6][2])+((Vpk[6][6][0]*
          Vpk[6][6][0])+(Vpk[6][6][1]*Vpk[6][6][1]))))+((IkWpk[6][6][2]*
          pin[6][2])+((IkWpk[6][6][0]*pin[6][0])+(IkWpk[6][6][1]*pin[6][1]))))+(
          (mk[7]*((Vpk[6][7][2]*Vpk[6][7][2])+((Vpk[6][7][0]*Vpk[6][7][0])+(
          Vpk[6][7][1]*Vpk[6][7][1]))))+((IkWpk[6][7][2]*Wpk[6][7][2])+((
          IkWpk[6][7][0]*Wpk[6][7][0])+(IkWpk[6][7][1]*Wpk[6][7][1])))));
        mm[6][7] = ((mk[7]*((Vpk[6][7][2]*Vpk[7][7][2])+((Vpk[6][7][0]*
          Vpk[7][7][0])+(Vpk[6][7][1]*Vpk[7][7][1]))))+((IkWpk[7][7][2]*
          Wpk[6][7][2])+((IkWpk[7][7][0]*Wpk[6][7][0])+(IkWpk[7][7][1]*
          Wpk[6][7][1]))));
        mm[6][8] = 0.;
        mm[6][9] = 0.;
        mm[6][10] = 0.;
        mm[6][11] = 0.;
        mm[6][12] = 0.;
        mm[6][13] = 0.;
        mm[7][7] = ((mk[7]*((Vpk[7][7][2]*Vpk[7][7][2])+((Vpk[7][7][0]*
          Vpk[7][7][0])+(Vpk[7][7][1]*Vpk[7][7][1]))))+((IkWpk[7][7][2]*
          pin[7][2])+((IkWpk[7][7][0]*pin[7][0])+(IkWpk[7][7][1]*pin[7][1]))));
        mm[7][8] = 0.;
        mm[7][9] = 0.;
        mm[7][10] = 0.;
        mm[7][11] = 0.;
        mm[7][12] = 0.;
        mm[7][13] = 0.;
        temp[0] = (((mk[8]*((Vpk[8][8][2]*Vpk[8][8][2])+((Vpk[8][8][0]*
          Vpk[8][8][0])+(Vpk[8][8][1]*Vpk[8][8][1]))))+((IkWpk[8][8][2]*
          pin[8][2])+((IkWpk[8][8][0]*pin[8][0])+(IkWpk[8][8][1]*pin[8][1]))))+(
          (mk[9]*((Vpk[8][9][2]*Vpk[8][9][2])+((Vpk[8][9][0]*Vpk[8][9][0])+(
          Vpk[8][9][1]*Vpk[8][9][1]))))+((IkWpk[8][9][2]*Wpk[8][9][2])+((
          IkWpk[8][9][0]*Wpk[8][9][0])+(IkWpk[8][9][1]*Wpk[8][9][1])))));
        temp[1] = (((mk[10]*((Vpk[8][10][2]*Vpk[8][10][2])+((Vpk[8][10][0]*
          Vpk[8][10][0])+(Vpk[8][10][1]*Vpk[8][10][1]))))+((IkWpk[8][10][2]*
          Wpk[8][10][2])+((IkWpk[8][10][0]*Wpk[8][10][0])+(IkWpk[8][10][1]*
          Wpk[8][10][1]))))+temp[0]);
        temp[2] = (((mk[11]*((Vpk[8][11][2]*Vpk[8][11][2])+((Vpk[8][11][0]*
          Vpk[8][11][0])+(Vpk[8][11][1]*Vpk[8][11][1]))))+((IkWpk[8][11][2]*
          Wpk[8][11][2])+((IkWpk[8][11][0]*Wpk[8][11][0])+(IkWpk[8][11][1]*
          Wpk[8][11][1]))))+temp[1]);
        temp[3] = (((mk[12]*((Vpk[8][12][2]*Vpk[8][12][2])+((Vpk[8][12][0]*
          Vpk[8][12][0])+(Vpk[8][12][1]*Vpk[8][12][1]))))+((IkWpk[8][12][2]*
          Wpk[8][12][2])+((IkWpk[8][12][0]*Wpk[8][12][0])+(IkWpk[8][12][1]*
          Wpk[8][12][1]))))+temp[2]);
        mm[8][8] = (((mk[13]*((Vpk[8][13][2]*Vpk[8][13][2])+((Vpk[8][13][0]*
          Vpk[8][13][0])+(Vpk[8][13][1]*Vpk[8][13][1]))))+((IkWpk[8][13][2]*
          Wpk[8][13][2])+((IkWpk[8][13][0]*Wpk[8][13][0])+(IkWpk[8][13][1]*
          Wpk[8][13][1]))))+temp[3]);
        temp[0] = (((mk[9]*((Vpk[8][9][2]*Vpk[9][9][2])+((Vpk[8][9][0]*
          Vpk[9][9][0])+(Vpk[8][9][1]*Vpk[9][9][1]))))+((IkWpk[9][9][2]*
          Wpk[8][9][2])+((IkWpk[9][9][0]*Wpk[8][9][0])+(IkWpk[9][9][1]*
          Wpk[8][9][1]))))+((mk[10]*((Vpk[8][10][2]*Vpk[9][10][2])+((
          Vpk[8][10][0]*Vpk[9][10][0])+(Vpk[8][10][1]*Vpk[9][10][1]))))+((
          IkWpk[9][10][2]*Wpk[8][10][2])+((IkWpk[9][10][0]*Wpk[8][10][0])+(
          IkWpk[9][10][1]*Wpk[8][10][1])))));
        temp[1] = (((mk[11]*((Vpk[8][11][2]*Vpk[9][11][2])+((Vpk[8][11][0]*
          Vpk[9][11][0])+(Vpk[8][11][1]*Vpk[9][11][1]))))+((IkWpk[9][11][2]*
          Wpk[8][11][2])+((IkWpk[9][11][0]*Wpk[8][11][0])+(IkWpk[9][11][1]*
          Wpk[8][11][1]))))+temp[0]);
        temp[2] = (((mk[12]*((Vpk[8][12][2]*Vpk[9][12][2])+((Vpk[8][12][0]*
          Vpk[9][12][0])+(Vpk[8][12][1]*Vpk[9][12][1]))))+((IkWpk[9][12][2]*
          Wpk[8][12][2])+((IkWpk[9][12][0]*Wpk[8][12][0])+(IkWpk[9][12][1]*
          Wpk[8][12][1]))))+temp[1]);
        mm[8][9] = (((mk[13]*((Vpk[8][13][2]*Vpk[9][13][2])+((Vpk[8][13][0]*
          Vpk[9][13][0])+(Vpk[8][13][1]*Vpk[9][13][1]))))+((IkWpk[9][13][2]*
          Wpk[8][13][2])+((IkWpk[9][13][0]*Wpk[8][13][0])+(IkWpk[9][13][1]*
          Wpk[8][13][1]))))+temp[2]);
        temp[0] = (((mk[10]*((Vpk[8][10][2]*Vpk[10][10][2])+((Vpk[8][10][0]*
          Vpk[10][10][0])+(Vpk[8][10][1]*Vpk[10][10][1]))))+((IkWpk[10][10][2]*
          Wpk[8][10][2])+((IkWpk[10][10][0]*Wpk[8][10][0])+(IkWpk[10][10][1]*
          Wpk[8][10][1]))))+((mk[11]*((Vpk[8][11][2]*Vpk[10][11][2])+((
          Vpk[8][11][0]*Vpk[10][11][0])+(Vpk[8][11][1]*Vpk[10][11][1]))))+((
          IkWpk[10][11][2]*Wpk[8][11][2])+((IkWpk[10][11][0]*Wpk[8][11][0])+(
          IkWpk[10][11][1]*Wpk[8][11][1])))));
        temp[1] = (((mk[12]*((Vpk[8][12][2]*Vpk[10][12][2])+((Vpk[8][12][0]*
          Vpk[10][12][0])+(Vpk[8][12][1]*Vpk[10][12][1]))))+((IkWpk[10][12][2]*
          Wpk[8][12][2])+((IkWpk[10][12][0]*Wpk[8][12][0])+(IkWpk[10][12][1]*
          Wpk[8][12][1]))))+temp[0]);
        mm[8][10] = (((mk[13]*((Vpk[8][13][2]*Vpk[10][13][2])+((Vpk[8][13][0]*
          Vpk[10][13][0])+(Vpk[8][13][1]*Vpk[10][13][1]))))+((IkWpk[10][13][2]*
          Wpk[8][13][2])+((IkWpk[10][13][0]*Wpk[8][13][0])+(IkWpk[10][13][1]*
          Wpk[8][13][1]))))+temp[1]);
        temp[0] = (((mk[11]*((Vpk[8][11][2]*Vpk[11][11][2])+((Vpk[8][11][0]*
          Vpk[11][11][0])+(Vpk[8][11][1]*Vpk[11][11][1]))))+((IkWpk[11][11][2]*
          Wpk[8][11][2])+((IkWpk[11][11][0]*Wpk[8][11][0])+(IkWpk[11][11][1]*
          Wpk[8][11][1]))))+((mk[12]*((Vpk[8][12][2]*Vpk[11][12][2])+((
          Vpk[8][12][0]*Vpk[11][12][0])+(Vpk[8][12][1]*Vpk[11][12][1]))))+((
          IkWpk[11][12][2]*Wpk[8][12][2])+((IkWpk[11][12][0]*Wpk[8][12][0])+(
          IkWpk[11][12][1]*Wpk[8][12][1])))));
        mm[8][11] = (((mk[13]*((Vpk[8][13][2]*Vpk[11][13][2])+((Vpk[8][13][0]*
          Vpk[11][13][0])+(Vpk[8][13][1]*Vpk[11][13][1]))))+((IkWpk[11][13][2]*
          Wpk[8][13][2])+((IkWpk[11][13][0]*Wpk[8][13][0])+(IkWpk[11][13][1]*
          Wpk[8][13][1]))))+temp[0]);
        mm[8][12] = (((mk[12]*((Vpk[8][12][2]*Vpk[12][12][2])+((Vpk[8][12][0]*
          Vpk[12][12][0])+(Vpk[8][12][1]*Vpk[12][12][1]))))+((IkWpk[12][12][2]*
          Wpk[8][12][2])+((IkWpk[12][12][0]*Wpk[8][12][0])+(IkWpk[12][12][1]*
          Wpk[8][12][1]))))+((mk[13]*((Vpk[8][13][2]*Vpk[12][13][2])+((
          Vpk[8][13][0]*Vpk[12][13][0])+(Vpk[8][13][1]*Vpk[12][13][1]))))+((
          IkWpk[12][13][2]*Wpk[8][13][2])+((IkWpk[12][13][0]*Wpk[8][13][0])+(
          IkWpk[12][13][1]*Wpk[8][13][1])))));
        mm[8][13] = ((mk[13]*((Vpk[8][13][2]*Vpk[13][13][2])+((Vpk[8][13][0]*
          Vpk[13][13][0])+(Vpk[8][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          Wpk[8][13][2])+((IkWpk[13][13][0]*Wpk[8][13][0])+(IkWpk[13][13][1]*
          Wpk[8][13][1]))));
        temp[0] = (((mk[9]*((Vpk[9][9][2]*Vpk[9][9][2])+((Vpk[9][9][0]*
          Vpk[9][9][0])+(Vpk[9][9][1]*Vpk[9][9][1]))))+((IkWpk[9][9][2]*
          pin[9][2])+((IkWpk[9][9][0]*pin[9][0])+(IkWpk[9][9][1]*pin[9][1]))))+(
          (mk[10]*((Vpk[9][10][2]*Vpk[9][10][2])+((Vpk[9][10][0]*Vpk[9][10][0])+
          (Vpk[9][10][1]*Vpk[9][10][1]))))+((IkWpk[9][10][2]*Wpk[9][10][2])+((
          IkWpk[9][10][0]*Wpk[9][10][0])+(IkWpk[9][10][1]*Wpk[9][10][1])))));
        temp[1] = (((mk[11]*((Vpk[9][11][2]*Vpk[9][11][2])+((Vpk[9][11][0]*
          Vpk[9][11][0])+(Vpk[9][11][1]*Vpk[9][11][1]))))+((IkWpk[9][11][2]*
          Wpk[9][11][2])+((IkWpk[9][11][0]*Wpk[9][11][0])+(IkWpk[9][11][1]*
          Wpk[9][11][1]))))+temp[0]);
        temp[2] = (((mk[12]*((Vpk[9][12][2]*Vpk[9][12][2])+((Vpk[9][12][0]*
          Vpk[9][12][0])+(Vpk[9][12][1]*Vpk[9][12][1]))))+((IkWpk[9][12][2]*
          Wpk[9][12][2])+((IkWpk[9][12][0]*Wpk[9][12][0])+(IkWpk[9][12][1]*
          Wpk[9][12][1]))))+temp[1]);
        mm[9][9] = (((mk[13]*((Vpk[9][13][2]*Vpk[9][13][2])+((Vpk[9][13][0]*
          Vpk[9][13][0])+(Vpk[9][13][1]*Vpk[9][13][1]))))+((IkWpk[9][13][2]*
          Wpk[9][13][2])+((IkWpk[9][13][0]*Wpk[9][13][0])+(IkWpk[9][13][1]*
          Wpk[9][13][1]))))+temp[2]);
        temp[0] = (((mk[10]*((Vpk[9][10][2]*Vpk[10][10][2])+((Vpk[9][10][0]*
          Vpk[10][10][0])+(Vpk[9][10][1]*Vpk[10][10][1]))))+((IkWpk[10][10][2]*
          Wpk[9][10][2])+((IkWpk[10][10][0]*Wpk[9][10][0])+(IkWpk[10][10][1]*
          Wpk[9][10][1]))))+((mk[11]*((Vpk[9][11][2]*Vpk[10][11][2])+((
          Vpk[9][11][0]*Vpk[10][11][0])+(Vpk[9][11][1]*Vpk[10][11][1]))))+((
          IkWpk[10][11][2]*Wpk[9][11][2])+((IkWpk[10][11][0]*Wpk[9][11][0])+(
          IkWpk[10][11][1]*Wpk[9][11][1])))));
        temp[1] = (((mk[12]*((Vpk[9][12][2]*Vpk[10][12][2])+((Vpk[9][12][0]*
          Vpk[10][12][0])+(Vpk[9][12][1]*Vpk[10][12][1]))))+((IkWpk[10][12][2]*
          Wpk[9][12][2])+((IkWpk[10][12][0]*Wpk[9][12][0])+(IkWpk[10][12][1]*
          Wpk[9][12][1]))))+temp[0]);
        mm[9][10] = (((mk[13]*((Vpk[9][13][2]*Vpk[10][13][2])+((Vpk[9][13][0]*
          Vpk[10][13][0])+(Vpk[9][13][1]*Vpk[10][13][1]))))+((IkWpk[10][13][2]*
          Wpk[9][13][2])+((IkWpk[10][13][0]*Wpk[9][13][0])+(IkWpk[10][13][1]*
          Wpk[9][13][1]))))+temp[1]);
        temp[0] = (((mk[11]*((Vpk[9][11][2]*Vpk[11][11][2])+((Vpk[9][11][0]*
          Vpk[11][11][0])+(Vpk[9][11][1]*Vpk[11][11][1]))))+((IkWpk[11][11][2]*
          Wpk[9][11][2])+((IkWpk[11][11][0]*Wpk[9][11][0])+(IkWpk[11][11][1]*
          Wpk[9][11][1]))))+((mk[12]*((Vpk[9][12][2]*Vpk[11][12][2])+((
          Vpk[9][12][0]*Vpk[11][12][0])+(Vpk[9][12][1]*Vpk[11][12][1]))))+((
          IkWpk[11][12][2]*Wpk[9][12][2])+((IkWpk[11][12][0]*Wpk[9][12][0])+(
          IkWpk[11][12][1]*Wpk[9][12][1])))));
        mm[9][11] = (((mk[13]*((Vpk[9][13][2]*Vpk[11][13][2])+((Vpk[9][13][0]*
          Vpk[11][13][0])+(Vpk[9][13][1]*Vpk[11][13][1]))))+((IkWpk[11][13][2]*
          Wpk[9][13][2])+((IkWpk[11][13][0]*Wpk[9][13][0])+(IkWpk[11][13][1]*
          Wpk[9][13][1]))))+temp[0]);
        mm[9][12] = (((mk[12]*((Vpk[9][12][2]*Vpk[12][12][2])+((Vpk[9][12][0]*
          Vpk[12][12][0])+(Vpk[9][12][1]*Vpk[12][12][1]))))+((IkWpk[12][12][2]*
          Wpk[9][12][2])+((IkWpk[12][12][0]*Wpk[9][12][0])+(IkWpk[12][12][1]*
          Wpk[9][12][1]))))+((mk[13]*((Vpk[9][13][2]*Vpk[12][13][2])+((
          Vpk[9][13][0]*Vpk[12][13][0])+(Vpk[9][13][1]*Vpk[12][13][1]))))+((
          IkWpk[12][13][2]*Wpk[9][13][2])+((IkWpk[12][13][0]*Wpk[9][13][0])+(
          IkWpk[12][13][1]*Wpk[9][13][1])))));
        mm[9][13] = ((mk[13]*((Vpk[9][13][2]*Vpk[13][13][2])+((Vpk[9][13][0]*
          Vpk[13][13][0])+(Vpk[9][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          Wpk[9][13][2])+((IkWpk[13][13][0]*Wpk[9][13][0])+(IkWpk[13][13][1]*
          Wpk[9][13][1]))));
        temp[0] = (((mk[10]*((Vpk[10][10][2]*Vpk[10][10][2])+((Vpk[10][10][0]*
          Vpk[10][10][0])+(Vpk[10][10][1]*Vpk[10][10][1]))))+((IkWpk[10][10][2]*
          pin[10][2])+((IkWpk[10][10][0]*pin[10][0])+(IkWpk[10][10][1]*
          pin[10][1]))))+((mk[11]*((Vpk[10][11][2]*Vpk[10][11][2])+((
          Vpk[10][11][0]*Vpk[10][11][0])+(Vpk[10][11][1]*Vpk[10][11][1]))))+((
          IkWpk[10][11][2]*Wpk[10][11][2])+((IkWpk[10][11][0]*Wpk[10][11][0])+(
          IkWpk[10][11][1]*Wpk[10][11][1])))));
        temp[1] = (((mk[12]*((Vpk[10][12][2]*Vpk[10][12][2])+((Vpk[10][12][0]*
          Vpk[10][12][0])+(Vpk[10][12][1]*Vpk[10][12][1]))))+((IkWpk[10][12][2]*
          Wpk[10][12][2])+((IkWpk[10][12][0]*Wpk[10][12][0])+(IkWpk[10][12][1]*
          Wpk[10][12][1]))))+temp[0]);
        mm[10][10] = (((mk[13]*((Vpk[10][13][2]*Vpk[10][13][2])+((Vpk[10][13][0]
          *Vpk[10][13][0])+(Vpk[10][13][1]*Vpk[10][13][1]))))+((IkWpk[10][13][2]
          *Wpk[10][13][2])+((IkWpk[10][13][0]*Wpk[10][13][0])+(IkWpk[10][13][1]*
          Wpk[10][13][1]))))+temp[1]);
        temp[0] = (((mk[11]*((Vpk[10][11][2]*Vpk[11][11][2])+((Vpk[10][11][0]*
          Vpk[11][11][0])+(Vpk[10][11][1]*Vpk[11][11][1]))))+((IkWpk[11][11][2]*
          Wpk[10][11][2])+((IkWpk[11][11][0]*Wpk[10][11][0])+(IkWpk[11][11][1]*
          Wpk[10][11][1]))))+((mk[12]*((Vpk[10][12][2]*Vpk[11][12][2])+((
          Vpk[10][12][0]*Vpk[11][12][0])+(Vpk[10][12][1]*Vpk[11][12][1]))))+((
          IkWpk[11][12][2]*Wpk[10][12][2])+((IkWpk[11][12][0]*Wpk[10][12][0])+(
          IkWpk[11][12][1]*Wpk[10][12][1])))));
        mm[10][11] = (((mk[13]*((Vpk[10][13][2]*Vpk[11][13][2])+((Vpk[10][13][0]
          *Vpk[11][13][0])+(Vpk[10][13][1]*Vpk[11][13][1]))))+((IkWpk[11][13][2]
          *Wpk[10][13][2])+((IkWpk[11][13][0]*Wpk[10][13][0])+(IkWpk[11][13][1]*
          Wpk[10][13][1]))))+temp[0]);
        mm[10][12] = (((mk[12]*((Vpk[10][12][2]*Vpk[12][12][2])+((Vpk[10][12][0]
          *Vpk[12][12][0])+(Vpk[10][12][1]*Vpk[12][12][1]))))+((IkWpk[12][12][2]
          *Wpk[10][12][2])+((IkWpk[12][12][0]*Wpk[10][12][0])+(IkWpk[12][12][1]*
          Wpk[10][12][1]))))+((mk[13]*((Vpk[10][13][2]*Vpk[12][13][2])+((
          Vpk[10][13][0]*Vpk[12][13][0])+(Vpk[10][13][1]*Vpk[12][13][1]))))+((
          IkWpk[12][13][2]*Wpk[10][13][2])+((IkWpk[12][13][0]*Wpk[10][13][0])+(
          IkWpk[12][13][1]*Wpk[10][13][1])))));
        mm[10][13] = ((mk[13]*((Vpk[10][13][2]*Vpk[13][13][2])+((Vpk[10][13][0]*
          Vpk[13][13][0])+(Vpk[10][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          Wpk[10][13][2])+((IkWpk[13][13][0]*Wpk[10][13][0])+(IkWpk[13][13][1]*
          Wpk[10][13][1]))));
        temp[0] = (((mk[11]*((Vpk[11][11][2]*Vpk[11][11][2])+((Vpk[11][11][0]*
          Vpk[11][11][0])+(Vpk[11][11][1]*Vpk[11][11][1]))))+((IkWpk[11][11][2]*
          pin[11][2])+((IkWpk[11][11][0]*pin[11][0])+(IkWpk[11][11][1]*
          pin[11][1]))))+((mk[12]*((Vpk[11][12][2]*Vpk[11][12][2])+((
          Vpk[11][12][0]*Vpk[11][12][0])+(Vpk[11][12][1]*Vpk[11][12][1]))))+((
          IkWpk[11][12][2]*Wpk[11][12][2])+((IkWpk[11][12][0]*Wpk[11][12][0])+(
          IkWpk[11][12][1]*Wpk[11][12][1])))));
        mm[11][11] = (((mk[13]*((Vpk[11][13][2]*Vpk[11][13][2])+((Vpk[11][13][0]
          *Vpk[11][13][0])+(Vpk[11][13][1]*Vpk[11][13][1]))))+((IkWpk[11][13][2]
          *Wpk[11][13][2])+((IkWpk[11][13][0]*Wpk[11][13][0])+(IkWpk[11][13][1]*
          Wpk[11][13][1]))))+temp[0]);
        mm[11][12] = (((mk[12]*((Vpk[11][12][2]*Vpk[12][12][2])+((Vpk[11][12][0]
          *Vpk[12][12][0])+(Vpk[11][12][1]*Vpk[12][12][1]))))+((IkWpk[12][12][2]
          *Wpk[11][12][2])+((IkWpk[12][12][0]*Wpk[11][12][0])+(IkWpk[12][12][1]*
          Wpk[11][12][1]))))+((mk[13]*((Vpk[11][13][2]*Vpk[12][13][2])+((
          Vpk[11][13][0]*Vpk[12][13][0])+(Vpk[11][13][1]*Vpk[12][13][1]))))+((
          IkWpk[12][13][2]*Wpk[11][13][2])+((IkWpk[12][13][0]*Wpk[11][13][0])+(
          IkWpk[12][13][1]*Wpk[11][13][1])))));
        mm[11][13] = ((mk[13]*((Vpk[11][13][2]*Vpk[13][13][2])+((Vpk[11][13][0]*
          Vpk[13][13][0])+(Vpk[11][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          Wpk[11][13][2])+((IkWpk[13][13][0]*Wpk[11][13][0])+(IkWpk[13][13][1]*
          Wpk[11][13][1]))));
        mm[12][12] = (((mk[12]*((Vpk[12][12][2]*Vpk[12][12][2])+((Vpk[12][12][0]
          *Vpk[12][12][0])+(Vpk[12][12][1]*Vpk[12][12][1]))))+((IkWpk[12][12][2]
          *pin[12][2])+((IkWpk[12][12][0]*pin[12][0])+(IkWpk[12][12][1]*
          pin[12][1]))))+((mk[13]*((Vpk[12][13][2]*Vpk[12][13][2])+((
          Vpk[12][13][0]*Vpk[12][13][0])+(Vpk[12][13][1]*Vpk[12][13][1]))))+((
          IkWpk[12][13][2]*Wpk[12][13][2])+((IkWpk[12][13][0]*Wpk[12][13][0])+(
          IkWpk[12][13][1]*Wpk[12][13][1])))));
        mm[12][13] = ((mk[13]*((Vpk[12][13][2]*Vpk[13][13][2])+((Vpk[12][13][0]*
          Vpk[13][13][0])+(Vpk[12][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          Wpk[12][13][2])+((IkWpk[13][13][0]*Wpk[12][13][0])+(IkWpk[13][13][1]*
          Wpk[12][13][1]))));
        mm[13][13] = ((mk[13]*((Vpk[13][13][2]*Vpk[13][13][2])+((Vpk[13][13][0]*
          Vpk[13][13][0])+(Vpk[13][13][1]*Vpk[13][13][1]))))+((IkWpk[13][13][2]*
          pin[13][2])+((IkWpk[13][13][0]*pin[13][0])+(IkWpk[13][13][1]*
          pin[13][1]))));
/*
Check for singular mass matrix
*/
        for (i = 0; i < 14; i++) {
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
 Equations contain 1305 adds/subtracts/negates
                   1703 multiplies
                      0 divides
                    348 assignments
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
        sdldudcomp(14,14,mmap,1e-13,workss,works,mm,mlo,mdi);
/*
Check for singular mass matrix
*/
        for (i = 0; i < 14; i++) {
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

void sdmfrc(double imult[14])
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
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     28 assignments
*/
}

void sdequivht(double tau[14])
{
/* Compute tree hinge torques to match effect of applied loads
*/
    double fstareq[14][3],tstareq[14][3];

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(56,23);
        return;
    }
/*
Compute fstareq (forces)
*/
    fstareq[0][0] = -(ufk[0][0]+(gk[0][0]*mk[0]));
    fstareq[0][1] = -(ufk[0][1]+(gk[0][1]*mk[0]));
    fstareq[0][2] = -(ufk[0][2]+(gk[0][2]*mk[0]));
    fstareq[1][0] = -(ufk[1][0]+(gk[1][0]*mk[1]));
    fstareq[1][1] = -(ufk[1][1]+(gk[1][1]*mk[1]));
    fstareq[1][2] = -(ufk[1][2]+(gk[1][2]*mk[1]));
    fstareq[2][0] = -(ufk[2][0]+(gk[2][0]*mk[2]));
    fstareq[2][1] = -(ufk[2][1]+(gk[2][1]*mk[2]));
    fstareq[2][2] = -(ufk[2][2]+(gk[2][2]*mk[2]));
    fstareq[3][0] = -(ufk[3][0]+(gk[3][0]*mk[3]));
    fstareq[3][1] = -(ufk[3][1]+(gk[3][1]*mk[3]));
    fstareq[3][2] = -(ufk[3][2]+(gk[3][2]*mk[3]));
    fstareq[4][0] = -(ufk[4][0]+(gk[4][0]*mk[4]));
    fstareq[4][1] = -(ufk[4][1]+(gk[4][1]*mk[4]));
    fstareq[4][2] = -(ufk[4][2]+(gk[4][2]*mk[4]));
    fstareq[5][0] = -(ufk[5][0]+(gk[5][0]*mk[5]));
    fstareq[5][1] = -(ufk[5][1]+(gk[5][1]*mk[5]));
    fstareq[5][2] = -(ufk[5][2]+(gk[5][2]*mk[5]));
    fstareq[6][0] = -(ufk[6][0]+(gk[6][0]*mk[6]));
    fstareq[6][1] = -(ufk[6][1]+(gk[6][1]*mk[6]));
    fstareq[6][2] = -(ufk[6][2]+(gk[6][2]*mk[6]));
    fstareq[7][0] = -(ufk[7][0]+(gk[7][0]*mk[7]));
    fstareq[7][1] = -(ufk[7][1]+(gk[7][1]*mk[7]));
    fstareq[7][2] = -(ufk[7][2]+(gk[7][2]*mk[7]));
    fstareq[8][0] = -(ufk[8][0]+(gk[8][0]*mk[8]));
    fstareq[8][1] = -(ufk[8][1]+(gk[8][1]*mk[8]));
    fstareq[8][2] = -(ufk[8][2]+(gk[8][2]*mk[8]));
    fstareq[9][0] = -(ufk[9][0]+(gk[9][0]*mk[9]));
    fstareq[9][1] = -(ufk[9][1]+(gk[9][1]*mk[9]));
    fstareq[9][2] = -(ufk[9][2]+(gk[9][2]*mk[9]));
    fstareq[10][0] = -(ufk[10][0]+(gk[10][0]*mk[10]));
    fstareq[10][1] = -(ufk[10][1]+(gk[10][1]*mk[10]));
    fstareq[10][2] = -(ufk[10][2]+(gk[10][2]*mk[10]));
    fstareq[11][0] = -(ufk[11][0]+(gk[11][0]*mk[11]));
    fstareq[11][1] = -(ufk[11][1]+(gk[11][1]*mk[11]));
    fstareq[11][2] = -(ufk[11][2]+(gk[11][2]*mk[11]));
    fstareq[12][0] = -(ufk[12][0]+(gk[12][0]*mk[12]));
    fstareq[12][1] = -(ufk[12][1]+(gk[12][1]*mk[12]));
    fstareq[12][2] = -(ufk[12][2]+(gk[12][2]*mk[12]));
    fstareq[13][0] = -(ufk[13][0]+(gk[13][0]*mk[13]));
    fstareq[13][1] = -(ufk[13][1]+(gk[13][1]*mk[13]));
    fstareq[13][2] = -(ufk[13][2]+(gk[13][2]*mk[13]));
/*
Compute tstareq (torques)
*/
/*
Compute taus (RHS ignoring constraints and inertial forces)
*/
    sddovpk();
    temp[0] = ((((fstareq[0][2]*Vpk[0][0][2])+((fstareq[0][0]*Vpk[0][0][0])+(
      fstareq[0][1]*Vpk[0][0][1])))-((pin[0][2]*utk[0][2])+((pin[0][0]*utk[0][0]
      )+(pin[0][1]*utk[0][1]))))+(((fstareq[1][2]*Vpk[0][1][2])+((fstareq[1][0]*
      Vpk[0][1][0])+(fstareq[1][1]*Vpk[0][1][1])))-((utk[1][2]*Wpk[0][1][2])+((
      utk[1][0]*Wpk[0][1][0])+(utk[1][1]*Wpk[0][1][1])))));
    temp[1] = ((((fstareq[3][2]*Vpk[0][3][2])+((fstareq[3][0]*Vpk[0][3][0])+(
      fstareq[3][1]*Vpk[0][3][1])))-((utk[3][2]*Wpk[0][3][2])+((utk[3][0]*
      Wpk[0][3][0])+(utk[3][1]*Wpk[0][3][1]))))+((((fstareq[2][2]*Vpk[0][2][2])+
      ((fstareq[2][0]*Vpk[0][2][0])+(fstareq[2][1]*Vpk[0][2][1])))-((utk[2][2]*
      Wpk[0][2][2])+((utk[2][0]*Wpk[0][2][0])+(utk[2][1]*Wpk[0][2][1]))))+
      temp[0]));
    temp[2] = ((((fstareq[5][2]*Vpk[0][5][2])+((fstareq[5][0]*Vpk[0][5][0])+(
      fstareq[5][1]*Vpk[0][5][1])))-((utk[5][2]*Wpk[0][5][2])+((utk[5][0]*
      Wpk[0][5][0])+(utk[5][1]*Wpk[0][5][1]))))+((((fstareq[4][2]*Vpk[0][4][2])+
      ((fstareq[4][0]*Vpk[0][4][0])+(fstareq[4][1]*Vpk[0][4][1])))-((utk[4][2]*
      Wpk[0][4][2])+((utk[4][0]*Wpk[0][4][0])+(utk[4][1]*Wpk[0][4][1]))))+
      temp[1]));
    temp[3] = ((((fstareq[7][2]*Vpk[0][7][2])+((fstareq[7][0]*Vpk[0][7][0])+(
      fstareq[7][1]*Vpk[0][7][1])))-((utk[7][2]*Wpk[0][7][2])+((utk[7][0]*
      Wpk[0][7][0])+(utk[7][1]*Wpk[0][7][1]))))+((((fstareq[6][2]*Vpk[0][6][2])+
      ((fstareq[6][0]*Vpk[0][6][0])+(fstareq[6][1]*Vpk[0][6][1])))-((utk[6][2]*
      Wpk[0][6][2])+((utk[6][0]*Wpk[0][6][0])+(utk[6][1]*Wpk[0][6][1]))))+
      temp[2]));
    temp[4] = ((((fstareq[9][2]*Vpk[0][9][2])+((fstareq[9][0]*Vpk[0][9][0])+(
      fstareq[9][1]*Vpk[0][9][1])))-((utk[9][2]*Wpk[0][9][2])+((utk[9][0]*
      Wpk[0][9][0])+(utk[9][1]*Wpk[0][9][1]))))+((((fstareq[8][2]*Vpk[0][8][2])+
      ((fstareq[8][0]*Vpk[0][8][0])+(fstareq[8][1]*Vpk[0][8][1])))-((utk[8][2]*
      Wpk[0][8][2])+((utk[8][0]*Wpk[0][8][0])+(utk[8][1]*Wpk[0][8][1]))))+
      temp[3]));
    temp[5] = ((((fstareq[11][2]*Vpk[0][11][2])+((fstareq[11][0]*Vpk[0][11][0])+
      (fstareq[11][1]*Vpk[0][11][1])))-((utk[11][2]*Wpk[0][11][2])+((utk[11][0]*
      Wpk[0][11][0])+(utk[11][1]*Wpk[0][11][1]))))+((((fstareq[10][2]*
      Vpk[0][10][2])+((fstareq[10][0]*Vpk[0][10][0])+(fstareq[10][1]*
      Vpk[0][10][1])))-((utk[10][2]*Wpk[0][10][2])+((utk[10][0]*Wpk[0][10][0])+(
      utk[10][1]*Wpk[0][10][1]))))+temp[4]));
    tau[0] = (utau[0]-((((fstareq[13][2]*Vpk[0][13][2])+((fstareq[13][0]*
      Vpk[0][13][0])+(fstareq[13][1]*Vpk[0][13][1])))-((utk[13][2]*Wpk[0][13][2]
      )+((utk[13][0]*Wpk[0][13][0])+(utk[13][1]*Wpk[0][13][1]))))+((((
      fstareq[12][2]*Vpk[0][12][2])+((fstareq[12][0]*Vpk[0][12][0])+(
      fstareq[12][1]*Vpk[0][12][1])))-((utk[12][2]*Wpk[0][12][2])+((utk[12][0]*
      Wpk[0][12][0])+(utk[12][1]*Wpk[0][12][1]))))+temp[5])));
    tau[1] = (utau[1]-(((fstareq[1][2]*Vpk[1][1][2])+((fstareq[1][0]*
      Vpk[1][1][0])+(fstareq[1][1]*Vpk[1][1][1])))-((pin[1][2]*utk[1][2])+((
      pin[1][0]*utk[1][0])+(pin[1][1]*utk[1][1])))));
    temp[0] = ((((fstareq[2][2]*Vpk[2][2][2])+((fstareq[2][0]*Vpk[2][2][0])+(
      fstareq[2][1]*Vpk[2][2][1])))-((pin[2][2]*utk[2][2])+((pin[2][0]*utk[2][0]
      )+(pin[2][1]*utk[2][1]))))+(((fstareq[3][2]*Vpk[2][3][2])+((fstareq[3][0]*
      Vpk[2][3][0])+(fstareq[3][1]*Vpk[2][3][1])))-((utk[3][2]*Wpk[2][3][2])+((
      utk[3][0]*Wpk[2][3][0])+(utk[3][1]*Wpk[2][3][1])))));
    temp[1] = ((((fstareq[5][2]*Vpk[2][5][2])+((fstareq[5][0]*Vpk[2][5][0])+(
      fstareq[5][1]*Vpk[2][5][1])))-((utk[5][2]*Wpk[2][5][2])+((utk[5][0]*
      Wpk[2][5][0])+(utk[5][1]*Wpk[2][5][1]))))+((((fstareq[4][2]*Vpk[2][4][2])+
      ((fstareq[4][0]*Vpk[2][4][0])+(fstareq[4][1]*Vpk[2][4][1])))-((utk[4][2]*
      Wpk[2][4][2])+((utk[4][0]*Wpk[2][4][0])+(utk[4][1]*Wpk[2][4][1]))))+
      temp[0]));
    tau[2] = (utau[2]-((((fstareq[7][2]*Vpk[2][7][2])+((fstareq[7][0]*
      Vpk[2][7][0])+(fstareq[7][1]*Vpk[2][7][1])))-((utk[7][2]*Wpk[2][7][2])+((
      utk[7][0]*Wpk[2][7][0])+(utk[7][1]*Wpk[2][7][1]))))+((((fstareq[6][2]*
      Vpk[2][6][2])+((fstareq[6][0]*Vpk[2][6][0])+(fstareq[6][1]*Vpk[2][6][1])))
      -((utk[6][2]*Wpk[2][6][2])+((utk[6][0]*Wpk[2][6][0])+(utk[6][1]*
      Wpk[2][6][1]))))+temp[1])));
    temp[0] = ((((fstareq[3][2]*Vpk[3][3][2])+((fstareq[3][0]*Vpk[3][3][0])+(
      fstareq[3][1]*Vpk[3][3][1])))-((pin[3][2]*utk[3][2])+((pin[3][0]*utk[3][0]
      )+(pin[3][1]*utk[3][1]))))+(((fstareq[4][2]*Vpk[3][4][2])+((fstareq[4][0]*
      Vpk[3][4][0])+(fstareq[4][1]*Vpk[3][4][1])))-((utk[4][2]*Wpk[3][4][2])+((
      utk[4][0]*Wpk[3][4][0])+(utk[4][1]*Wpk[3][4][1])))));
    temp[1] = ((((fstareq[6][2]*Vpk[3][6][2])+((fstareq[6][0]*Vpk[3][6][0])+(
      fstareq[6][1]*Vpk[3][6][1])))-((utk[6][2]*Wpk[3][6][2])+((utk[6][0]*
      Wpk[3][6][0])+(utk[6][1]*Wpk[3][6][1]))))+((((fstareq[5][2]*Vpk[3][5][2])+
      ((fstareq[5][0]*Vpk[3][5][0])+(fstareq[5][1]*Vpk[3][5][1])))-((utk[5][2]*
      Wpk[3][5][2])+((utk[5][0]*Wpk[3][5][0])+(utk[5][1]*Wpk[3][5][1]))))+
      temp[0]));
    tau[3] = (utau[3]-((((fstareq[7][2]*Vpk[3][7][2])+((fstareq[7][0]*
      Vpk[3][7][0])+(fstareq[7][1]*Vpk[3][7][1])))-((utk[7][2]*Wpk[3][7][2])+((
      utk[7][0]*Wpk[3][7][0])+(utk[7][1]*Wpk[3][7][1]))))+temp[1]));
    temp[0] = ((((fstareq[4][2]*Vpk[4][4][2])+((fstareq[4][0]*Vpk[4][4][0])+(
      fstareq[4][1]*Vpk[4][4][1])))-((pin[4][2]*utk[4][2])+((pin[4][0]*utk[4][0]
      )+(pin[4][1]*utk[4][1]))))+(((fstareq[5][2]*Vpk[4][5][2])+((fstareq[5][0]*
      Vpk[4][5][0])+(fstareq[5][1]*Vpk[4][5][1])))-((utk[5][2]*Wpk[4][5][2])+((
      utk[5][0]*Wpk[4][5][0])+(utk[5][1]*Wpk[4][5][1])))));
    tau[4] = (utau[4]-((((fstareq[7][2]*Vpk[4][7][2])+((fstareq[7][0]*
      Vpk[4][7][0])+(fstareq[7][1]*Vpk[4][7][1])))-((utk[7][2]*Wpk[4][7][2])+((
      utk[7][0]*Wpk[4][7][0])+(utk[7][1]*Wpk[4][7][1]))))+((((fstareq[6][2]*
      Vpk[4][6][2])+((fstareq[6][0]*Vpk[4][6][0])+(fstareq[6][1]*Vpk[4][6][1])))
      -((utk[6][2]*Wpk[4][6][2])+((utk[6][0]*Wpk[4][6][0])+(utk[6][1]*
      Wpk[4][6][1]))))+temp[0])));
    temp[0] = ((((fstareq[5][2]*Vpk[5][5][2])+((fstareq[5][0]*Vpk[5][5][0])+(
      fstareq[5][1]*Vpk[5][5][1])))-((pin[5][2]*utk[5][2])+((pin[5][0]*utk[5][0]
      )+(pin[5][1]*utk[5][1]))))+(((fstareq[6][2]*Vpk[5][6][2])+((fstareq[6][0]*
      Vpk[5][6][0])+(fstareq[6][1]*Vpk[5][6][1])))-((utk[6][2]*Wpk[5][6][2])+((
      utk[6][0]*Wpk[5][6][0])+(utk[6][1]*Wpk[5][6][1])))));
    tau[5] = (utau[5]-((((fstareq[7][2]*Vpk[5][7][2])+((fstareq[7][0]*
      Vpk[5][7][0])+(fstareq[7][1]*Vpk[5][7][1])))-((utk[7][2]*Wpk[5][7][2])+((
      utk[7][0]*Wpk[5][7][0])+(utk[7][1]*Wpk[5][7][1]))))+temp[0]));
    tau[6] = (utau[6]-((((fstareq[6][2]*Vpk[6][6][2])+((fstareq[6][0]*
      Vpk[6][6][0])+(fstareq[6][1]*Vpk[6][6][1])))-((pin[6][2]*utk[6][2])+((
      pin[6][0]*utk[6][0])+(pin[6][1]*utk[6][1]))))+(((fstareq[7][2]*
      Vpk[6][7][2])+((fstareq[7][0]*Vpk[6][7][0])+(fstareq[7][1]*Vpk[6][7][1])))
      -((utk[7][2]*Wpk[6][7][2])+((utk[7][0]*Wpk[6][7][0])+(utk[7][1]*
      Wpk[6][7][1]))))));
    tau[7] = (utau[7]-(((fstareq[7][2]*Vpk[7][7][2])+((fstareq[7][0]*
      Vpk[7][7][0])+(fstareq[7][1]*Vpk[7][7][1])))-((pin[7][2]*utk[7][2])+((
      pin[7][0]*utk[7][0])+(pin[7][1]*utk[7][1])))));
    temp[0] = ((((fstareq[8][2]*Vpk[8][8][2])+((fstareq[8][0]*Vpk[8][8][0])+(
      fstareq[8][1]*Vpk[8][8][1])))-((pin[8][2]*utk[8][2])+((pin[8][0]*utk[8][0]
      )+(pin[8][1]*utk[8][1]))))+(((fstareq[9][2]*Vpk[8][9][2])+((fstareq[9][0]*
      Vpk[8][9][0])+(fstareq[9][1]*Vpk[8][9][1])))-((utk[9][2]*Wpk[8][9][2])+((
      utk[9][0]*Wpk[8][9][0])+(utk[9][1]*Wpk[8][9][1])))));
    temp[1] = ((((fstareq[11][2]*Vpk[8][11][2])+((fstareq[11][0]*Vpk[8][11][0])+
      (fstareq[11][1]*Vpk[8][11][1])))-((utk[11][2]*Wpk[8][11][2])+((utk[11][0]*
      Wpk[8][11][0])+(utk[11][1]*Wpk[8][11][1]))))+((((fstareq[10][2]*
      Vpk[8][10][2])+((fstareq[10][0]*Vpk[8][10][0])+(fstareq[10][1]*
      Vpk[8][10][1])))-((utk[10][2]*Wpk[8][10][2])+((utk[10][0]*Wpk[8][10][0])+(
      utk[10][1]*Wpk[8][10][1]))))+temp[0]));
    tau[8] = (utau[8]-((((fstareq[13][2]*Vpk[8][13][2])+((fstareq[13][0]*
      Vpk[8][13][0])+(fstareq[13][1]*Vpk[8][13][1])))-((utk[13][2]*Wpk[8][13][2]
      )+((utk[13][0]*Wpk[8][13][0])+(utk[13][1]*Wpk[8][13][1]))))+((((
      fstareq[12][2]*Vpk[8][12][2])+((fstareq[12][0]*Vpk[8][12][0])+(
      fstareq[12][1]*Vpk[8][12][1])))-((utk[12][2]*Wpk[8][12][2])+((utk[12][0]*
      Wpk[8][12][0])+(utk[12][1]*Wpk[8][12][1]))))+temp[1])));
    temp[0] = ((((fstareq[9][2]*Vpk[9][9][2])+((fstareq[9][0]*Vpk[9][9][0])+(
      fstareq[9][1]*Vpk[9][9][1])))-((pin[9][2]*utk[9][2])+((pin[9][0]*utk[9][0]
      )+(pin[9][1]*utk[9][1]))))+(((fstareq[10][2]*Vpk[9][10][2])+((
      fstareq[10][0]*Vpk[9][10][0])+(fstareq[10][1]*Vpk[9][10][1])))-((
      utk[10][2]*Wpk[9][10][2])+((utk[10][0]*Wpk[9][10][0])+(utk[10][1]*
      Wpk[9][10][1])))));
    temp[1] = ((((fstareq[12][2]*Vpk[9][12][2])+((fstareq[12][0]*Vpk[9][12][0])+
      (fstareq[12][1]*Vpk[9][12][1])))-((utk[12][2]*Wpk[9][12][2])+((utk[12][0]*
      Wpk[9][12][0])+(utk[12][1]*Wpk[9][12][1]))))+((((fstareq[11][2]*
      Vpk[9][11][2])+((fstareq[11][0]*Vpk[9][11][0])+(fstareq[11][1]*
      Vpk[9][11][1])))-((utk[11][2]*Wpk[9][11][2])+((utk[11][0]*Wpk[9][11][0])+(
      utk[11][1]*Wpk[9][11][1]))))+temp[0]));
    tau[9] = (utau[9]-((((fstareq[13][2]*Vpk[9][13][2])+((fstareq[13][0]*
      Vpk[9][13][0])+(fstareq[13][1]*Vpk[9][13][1])))-((utk[13][2]*Wpk[9][13][2]
      )+((utk[13][0]*Wpk[9][13][0])+(utk[13][1]*Wpk[9][13][1]))))+temp[1]));
    temp[0] = ((((fstareq[10][2]*Vpk[10][10][2])+((fstareq[10][0]*Vpk[10][10][0]
      )+(fstareq[10][1]*Vpk[10][10][1])))-((pin[10][2]*utk[10][2])+((pin[10][0]*
      utk[10][0])+(pin[10][1]*utk[10][1]))))+(((fstareq[11][2]*Vpk[10][11][2])+(
      (fstareq[11][0]*Vpk[10][11][0])+(fstareq[11][1]*Vpk[10][11][1])))-((
      utk[11][2]*Wpk[10][11][2])+((utk[11][0]*Wpk[10][11][0])+(utk[11][1]*
      Wpk[10][11][1])))));
    tau[10] = (utau[10]-((((fstareq[13][2]*Vpk[10][13][2])+((fstareq[13][0]*
      Vpk[10][13][0])+(fstareq[13][1]*Vpk[10][13][1])))-((utk[13][2]*
      Wpk[10][13][2])+((utk[13][0]*Wpk[10][13][0])+(utk[13][1]*Wpk[10][13][1])))
      )+((((fstareq[12][2]*Vpk[10][12][2])+((fstareq[12][0]*Vpk[10][12][0])+(
      fstareq[12][1]*Vpk[10][12][1])))-((utk[12][2]*Wpk[10][12][2])+((utk[12][0]
      *Wpk[10][12][0])+(utk[12][1]*Wpk[10][12][1]))))+temp[0])));
    temp[0] = ((((fstareq[11][2]*Vpk[11][11][2])+((fstareq[11][0]*Vpk[11][11][0]
      )+(fstareq[11][1]*Vpk[11][11][1])))-((pin[11][2]*utk[11][2])+((pin[11][0]*
      utk[11][0])+(pin[11][1]*utk[11][1]))))+(((fstareq[12][2]*Vpk[11][12][2])+(
      (fstareq[12][0]*Vpk[11][12][0])+(fstareq[12][1]*Vpk[11][12][1])))-((
      utk[12][2]*Wpk[11][12][2])+((utk[12][0]*Wpk[11][12][0])+(utk[12][1]*
      Wpk[11][12][1])))));
    tau[11] = (utau[11]-((((fstareq[13][2]*Vpk[11][13][2])+((fstareq[13][0]*
      Vpk[11][13][0])+(fstareq[13][1]*Vpk[11][13][1])))-((utk[13][2]*
      Wpk[11][13][2])+((utk[13][0]*Wpk[11][13][0])+(utk[13][1]*Wpk[11][13][1])))
      )+temp[0]));
    tau[12] = (utau[12]-((((fstareq[12][2]*Vpk[12][12][2])+((fstareq[12][0]*
      Vpk[12][12][0])+(fstareq[12][1]*Vpk[12][12][1])))-((pin[12][2]*utk[12][2])
      +((pin[12][0]*utk[12][0])+(pin[12][1]*utk[12][1]))))+(((fstareq[13][2]*
      Vpk[12][13][2])+((fstareq[13][0]*Vpk[12][13][0])+(fstareq[13][1]*
      Vpk[12][13][1])))-((utk[13][2]*Wpk[12][13][2])+((utk[13][0]*Wpk[12][13][0]
      )+(utk[13][1]*Wpk[12][13][1]))))));
    tau[13] = (utau[13]-(((fstareq[13][2]*Vpk[13][13][2])+((fstareq[13][0]*
      Vpk[13][13][0])+(fstareq[13][1]*Vpk[13][13][1])))-((pin[13][2]*utk[13][2])
      +((pin[13][0]*utk[13][0])+(pin[13][1]*utk[13][1])))));
/*
Op counts below do not include called subroutines
*/
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  426 adds/subtracts/negates
                    384 multiplies
                      0 divides
                     74 assignments
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
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     14 assignments
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
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     14 assignments
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
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   14 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     14 assignments
*/
}

void sdfsgenmult(void)
{

/*
Compute Fs (generic multiplier-generated forces)
*/
    sddovpk();
    temp[0] = ((((mfk[0][2]*Vpk[0][0][2])+((mfk[0][0]*Vpk[0][0][0])+(mfk[0][1]*
      Vpk[0][0][1])))+((mtk[0][2]*pin[0][2])+((mtk[0][0]*pin[0][0])+(mtk[0][1]*
      pin[0][1]))))+(((mfk[1][2]*Vpk[0][1][2])+((mfk[1][0]*Vpk[0][1][0])+(
      mfk[1][1]*Vpk[0][1][1])))+((mtk[1][2]*Wpk[0][1][2])+((mtk[1][0]*
      Wpk[0][1][0])+(mtk[1][1]*Wpk[0][1][1])))));
    temp[1] = ((((mfk[3][2]*Vpk[0][3][2])+((mfk[3][0]*Vpk[0][3][0])+(mfk[3][1]*
      Vpk[0][3][1])))+((mtk[3][2]*Wpk[0][3][2])+((mtk[3][0]*Wpk[0][3][0])+(
      mtk[3][1]*Wpk[0][3][1]))))+((((mfk[2][2]*Vpk[0][2][2])+((mfk[2][0]*
      Vpk[0][2][0])+(mfk[2][1]*Vpk[0][2][1])))+((mtk[2][2]*Wpk[0][2][2])+((
      mtk[2][0]*Wpk[0][2][0])+(mtk[2][1]*Wpk[0][2][1]))))+temp[0]));
    temp[2] = ((((mfk[5][2]*Vpk[0][5][2])+((mfk[5][0]*Vpk[0][5][0])+(mfk[5][1]*
      Vpk[0][5][1])))+((mtk[5][2]*Wpk[0][5][2])+((mtk[5][0]*Wpk[0][5][0])+(
      mtk[5][1]*Wpk[0][5][1]))))+((((mfk[4][2]*Vpk[0][4][2])+((mfk[4][0]*
      Vpk[0][4][0])+(mfk[4][1]*Vpk[0][4][1])))+((mtk[4][2]*Wpk[0][4][2])+((
      mtk[4][0]*Wpk[0][4][0])+(mtk[4][1]*Wpk[0][4][1]))))+temp[1]));
    temp[3] = ((((mfk[7][2]*Vpk[0][7][2])+((mfk[7][0]*Vpk[0][7][0])+(mfk[7][1]*
      Vpk[0][7][1])))+((mtk[7][2]*Wpk[0][7][2])+((mtk[7][0]*Wpk[0][7][0])+(
      mtk[7][1]*Wpk[0][7][1]))))+((((mfk[6][2]*Vpk[0][6][2])+((mfk[6][0]*
      Vpk[0][6][0])+(mfk[6][1]*Vpk[0][6][1])))+((mtk[6][2]*Wpk[0][6][2])+((
      mtk[6][0]*Wpk[0][6][0])+(mtk[6][1]*Wpk[0][6][1]))))+temp[2]));
    temp[4] = ((((mfk[9][2]*Vpk[0][9][2])+((mfk[9][0]*Vpk[0][9][0])+(mfk[9][1]*
      Vpk[0][9][1])))+((mtk[9][2]*Wpk[0][9][2])+((mtk[9][0]*Wpk[0][9][0])+(
      mtk[9][1]*Wpk[0][9][1]))))+((((mfk[8][2]*Vpk[0][8][2])+((mfk[8][0]*
      Vpk[0][8][0])+(mfk[8][1]*Vpk[0][8][1])))+((mtk[8][2]*Wpk[0][8][2])+((
      mtk[8][0]*Wpk[0][8][0])+(mtk[8][1]*Wpk[0][8][1]))))+temp[3]));
    temp[5] = ((((mfk[11][2]*Vpk[0][11][2])+((mfk[11][0]*Vpk[0][11][0])+(
      mfk[11][1]*Vpk[0][11][1])))+((mtk[11][2]*Wpk[0][11][2])+((mtk[11][0]*
      Wpk[0][11][0])+(mtk[11][1]*Wpk[0][11][1]))))+((((mfk[10][2]*Vpk[0][10][2])
      +((mfk[10][0]*Vpk[0][10][0])+(mfk[10][1]*Vpk[0][10][1])))+((mtk[10][2]*
      Wpk[0][10][2])+((mtk[10][0]*Wpk[0][10][0])+(mtk[10][1]*Wpk[0][10][1]))))+
      temp[4]));
    fs[0] = (mtau[0]+((((mfk[13][2]*Vpk[0][13][2])+((mfk[13][0]*Vpk[0][13][0])+(
      mfk[13][1]*Vpk[0][13][1])))+((mtk[13][2]*Wpk[0][13][2])+((mtk[13][0]*
      Wpk[0][13][0])+(mtk[13][1]*Wpk[0][13][1]))))+((((mfk[12][2]*Vpk[0][12][2])
      +((mfk[12][0]*Vpk[0][12][0])+(mfk[12][1]*Vpk[0][12][1])))+((mtk[12][2]*
      Wpk[0][12][2])+((mtk[12][0]*Wpk[0][12][0])+(mtk[12][1]*Wpk[0][12][1]))))+
      temp[5])));
    fs[1] = (mtau[1]+(((mfk[1][2]*Vpk[1][1][2])+((mfk[1][0]*Vpk[1][1][0])+(
      mfk[1][1]*Vpk[1][1][1])))+((mtk[1][2]*pin[1][2])+((mtk[1][0]*pin[1][0])+(
      mtk[1][1]*pin[1][1])))));
    temp[0] = ((((mfk[2][2]*Vpk[2][2][2])+((mfk[2][0]*Vpk[2][2][0])+(mfk[2][1]*
      Vpk[2][2][1])))+((mtk[2][2]*pin[2][2])+((mtk[2][0]*pin[2][0])+(mtk[2][1]*
      pin[2][1]))))+(((mfk[3][2]*Vpk[2][3][2])+((mfk[3][0]*Vpk[2][3][0])+(
      mfk[3][1]*Vpk[2][3][1])))+((mtk[3][2]*Wpk[2][3][2])+((mtk[3][0]*
      Wpk[2][3][0])+(mtk[3][1]*Wpk[2][3][1])))));
    temp[1] = ((((mfk[5][2]*Vpk[2][5][2])+((mfk[5][0]*Vpk[2][5][0])+(mfk[5][1]*
      Vpk[2][5][1])))+((mtk[5][2]*Wpk[2][5][2])+((mtk[5][0]*Wpk[2][5][0])+(
      mtk[5][1]*Wpk[2][5][1]))))+((((mfk[4][2]*Vpk[2][4][2])+((mfk[4][0]*
      Vpk[2][4][0])+(mfk[4][1]*Vpk[2][4][1])))+((mtk[4][2]*Wpk[2][4][2])+((
      mtk[4][0]*Wpk[2][4][0])+(mtk[4][1]*Wpk[2][4][1]))))+temp[0]));
    fs[2] = (mtau[2]+((((mfk[7][2]*Vpk[2][7][2])+((mfk[7][0]*Vpk[2][7][0])+(
      mfk[7][1]*Vpk[2][7][1])))+((mtk[7][2]*Wpk[2][7][2])+((mtk[7][0]*
      Wpk[2][7][0])+(mtk[7][1]*Wpk[2][7][1]))))+((((mfk[6][2]*Vpk[2][6][2])+((
      mfk[6][0]*Vpk[2][6][0])+(mfk[6][1]*Vpk[2][6][1])))+((mtk[6][2]*
      Wpk[2][6][2])+((mtk[6][0]*Wpk[2][6][0])+(mtk[6][1]*Wpk[2][6][1]))))+
      temp[1])));
    temp[0] = ((((mfk[3][2]*Vpk[3][3][2])+((mfk[3][0]*Vpk[3][3][0])+(mfk[3][1]*
      Vpk[3][3][1])))+((mtk[3][2]*pin[3][2])+((mtk[3][0]*pin[3][0])+(mtk[3][1]*
      pin[3][1]))))+(((mfk[4][2]*Vpk[3][4][2])+((mfk[4][0]*Vpk[3][4][0])+(
      mfk[4][1]*Vpk[3][4][1])))+((mtk[4][2]*Wpk[3][4][2])+((mtk[4][0]*
      Wpk[3][4][0])+(mtk[4][1]*Wpk[3][4][1])))));
    temp[1] = ((((mfk[6][2]*Vpk[3][6][2])+((mfk[6][0]*Vpk[3][6][0])+(mfk[6][1]*
      Vpk[3][6][1])))+((mtk[6][2]*Wpk[3][6][2])+((mtk[6][0]*Wpk[3][6][0])+(
      mtk[6][1]*Wpk[3][6][1]))))+((((mfk[5][2]*Vpk[3][5][2])+((mfk[5][0]*
      Vpk[3][5][0])+(mfk[5][1]*Vpk[3][5][1])))+((mtk[5][2]*Wpk[3][5][2])+((
      mtk[5][0]*Wpk[3][5][0])+(mtk[5][1]*Wpk[3][5][1]))))+temp[0]));
    fs[3] = (mtau[3]+((((mfk[7][2]*Vpk[3][7][2])+((mfk[7][0]*Vpk[3][7][0])+(
      mfk[7][1]*Vpk[3][7][1])))+((mtk[7][2]*Wpk[3][7][2])+((mtk[7][0]*
      Wpk[3][7][0])+(mtk[7][1]*Wpk[3][7][1]))))+temp[1]));
    temp[0] = ((((mfk[4][2]*Vpk[4][4][2])+((mfk[4][0]*Vpk[4][4][0])+(mfk[4][1]*
      Vpk[4][4][1])))+((mtk[4][2]*pin[4][2])+((mtk[4][0]*pin[4][0])+(mtk[4][1]*
      pin[4][1]))))+(((mfk[5][2]*Vpk[4][5][2])+((mfk[5][0]*Vpk[4][5][0])+(
      mfk[5][1]*Vpk[4][5][1])))+((mtk[5][2]*Wpk[4][5][2])+((mtk[5][0]*
      Wpk[4][5][0])+(mtk[5][1]*Wpk[4][5][1])))));
    fs[4] = (mtau[4]+((((mfk[7][2]*Vpk[4][7][2])+((mfk[7][0]*Vpk[4][7][0])+(
      mfk[7][1]*Vpk[4][7][1])))+((mtk[7][2]*Wpk[4][7][2])+((mtk[7][0]*
      Wpk[4][7][0])+(mtk[7][1]*Wpk[4][7][1]))))+((((mfk[6][2]*Vpk[4][6][2])+((
      mfk[6][0]*Vpk[4][6][0])+(mfk[6][1]*Vpk[4][6][1])))+((mtk[6][2]*
      Wpk[4][6][2])+((mtk[6][0]*Wpk[4][6][0])+(mtk[6][1]*Wpk[4][6][1]))))+
      temp[0])));
    temp[0] = ((((mfk[5][2]*Vpk[5][5][2])+((mfk[5][0]*Vpk[5][5][0])+(mfk[5][1]*
      Vpk[5][5][1])))+((mtk[5][2]*pin[5][2])+((mtk[5][0]*pin[5][0])+(mtk[5][1]*
      pin[5][1]))))+(((mfk[6][2]*Vpk[5][6][2])+((mfk[6][0]*Vpk[5][6][0])+(
      mfk[6][1]*Vpk[5][6][1])))+((mtk[6][2]*Wpk[5][6][2])+((mtk[6][0]*
      Wpk[5][6][0])+(mtk[6][1]*Wpk[5][6][1])))));
    fs[5] = (mtau[5]+((((mfk[7][2]*Vpk[5][7][2])+((mfk[7][0]*Vpk[5][7][0])+(
      mfk[7][1]*Vpk[5][7][1])))+((mtk[7][2]*Wpk[5][7][2])+((mtk[7][0]*
      Wpk[5][7][0])+(mtk[7][1]*Wpk[5][7][1]))))+temp[0]));
    fs[6] = (mtau[6]+((((mfk[6][2]*Vpk[6][6][2])+((mfk[6][0]*Vpk[6][6][0])+(
      mfk[6][1]*Vpk[6][6][1])))+((mtk[6][2]*pin[6][2])+((mtk[6][0]*pin[6][0])+(
      mtk[6][1]*pin[6][1]))))+(((mfk[7][2]*Vpk[6][7][2])+((mfk[7][0]*
      Vpk[6][7][0])+(mfk[7][1]*Vpk[6][7][1])))+((mtk[7][2]*Wpk[6][7][2])+((
      mtk[7][0]*Wpk[6][7][0])+(mtk[7][1]*Wpk[6][7][1]))))));
    fs[7] = (mtau[7]+(((mfk[7][2]*Vpk[7][7][2])+((mfk[7][0]*Vpk[7][7][0])+(
      mfk[7][1]*Vpk[7][7][1])))+((mtk[7][2]*pin[7][2])+((mtk[7][0]*pin[7][0])+(
      mtk[7][1]*pin[7][1])))));
    temp[0] = ((((mfk[8][2]*Vpk[8][8][2])+((mfk[8][0]*Vpk[8][8][0])+(mfk[8][1]*
      Vpk[8][8][1])))+((mtk[8][2]*pin[8][2])+((mtk[8][0]*pin[8][0])+(mtk[8][1]*
      pin[8][1]))))+(((mfk[9][2]*Vpk[8][9][2])+((mfk[9][0]*Vpk[8][9][0])+(
      mfk[9][1]*Vpk[8][9][1])))+((mtk[9][2]*Wpk[8][9][2])+((mtk[9][0]*
      Wpk[8][9][0])+(mtk[9][1]*Wpk[8][9][1])))));
    temp[1] = ((((mfk[11][2]*Vpk[8][11][2])+((mfk[11][0]*Vpk[8][11][0])+(
      mfk[11][1]*Vpk[8][11][1])))+((mtk[11][2]*Wpk[8][11][2])+((mtk[11][0]*
      Wpk[8][11][0])+(mtk[11][1]*Wpk[8][11][1]))))+((((mfk[10][2]*Vpk[8][10][2])
      +((mfk[10][0]*Vpk[8][10][0])+(mfk[10][1]*Vpk[8][10][1])))+((mtk[10][2]*
      Wpk[8][10][2])+((mtk[10][0]*Wpk[8][10][0])+(mtk[10][1]*Wpk[8][10][1]))))+
      temp[0]));
    fs[8] = (mtau[8]+((((mfk[13][2]*Vpk[8][13][2])+((mfk[13][0]*Vpk[8][13][0])+(
      mfk[13][1]*Vpk[8][13][1])))+((mtk[13][2]*Wpk[8][13][2])+((mtk[13][0]*
      Wpk[8][13][0])+(mtk[13][1]*Wpk[8][13][1]))))+((((mfk[12][2]*Vpk[8][12][2])
      +((mfk[12][0]*Vpk[8][12][0])+(mfk[12][1]*Vpk[8][12][1])))+((mtk[12][2]*
      Wpk[8][12][2])+((mtk[12][0]*Wpk[8][12][0])+(mtk[12][1]*Wpk[8][12][1]))))+
      temp[1])));
    temp[0] = ((((mfk[9][2]*Vpk[9][9][2])+((mfk[9][0]*Vpk[9][9][0])+(mfk[9][1]*
      Vpk[9][9][1])))+((mtk[9][2]*pin[9][2])+((mtk[9][0]*pin[9][0])+(mtk[9][1]*
      pin[9][1]))))+(((mfk[10][2]*Vpk[9][10][2])+((mfk[10][0]*Vpk[9][10][0])+(
      mfk[10][1]*Vpk[9][10][1])))+((mtk[10][2]*Wpk[9][10][2])+((mtk[10][0]*
      Wpk[9][10][0])+(mtk[10][1]*Wpk[9][10][1])))));
    temp[1] = ((((mfk[12][2]*Vpk[9][12][2])+((mfk[12][0]*Vpk[9][12][0])+(
      mfk[12][1]*Vpk[9][12][1])))+((mtk[12][2]*Wpk[9][12][2])+((mtk[12][0]*
      Wpk[9][12][0])+(mtk[12][1]*Wpk[9][12][1]))))+((((mfk[11][2]*Vpk[9][11][2])
      +((mfk[11][0]*Vpk[9][11][0])+(mfk[11][1]*Vpk[9][11][1])))+((mtk[11][2]*
      Wpk[9][11][2])+((mtk[11][0]*Wpk[9][11][0])+(mtk[11][1]*Wpk[9][11][1]))))+
      temp[0]));
    fs[9] = (mtau[9]+((((mfk[13][2]*Vpk[9][13][2])+((mfk[13][0]*Vpk[9][13][0])+(
      mfk[13][1]*Vpk[9][13][1])))+((mtk[13][2]*Wpk[9][13][2])+((mtk[13][0]*
      Wpk[9][13][0])+(mtk[13][1]*Wpk[9][13][1]))))+temp[1]));
    temp[0] = ((((mfk[10][2]*Vpk[10][10][2])+((mfk[10][0]*Vpk[10][10][0])+(
      mfk[10][1]*Vpk[10][10][1])))+((mtk[10][2]*pin[10][2])+((mtk[10][0]*
      pin[10][0])+(mtk[10][1]*pin[10][1]))))+(((mfk[11][2]*Vpk[10][11][2])+((
      mfk[11][0]*Vpk[10][11][0])+(mfk[11][1]*Vpk[10][11][1])))+((mtk[11][2]*
      Wpk[10][11][2])+((mtk[11][0]*Wpk[10][11][0])+(mtk[11][1]*Wpk[10][11][1])))
      ));
    fs[10] = (mtau[10]+((((mfk[13][2]*Vpk[10][13][2])+((mfk[13][0]*
      Vpk[10][13][0])+(mfk[13][1]*Vpk[10][13][1])))+((mtk[13][2]*Wpk[10][13][2])
      +((mtk[13][0]*Wpk[10][13][0])+(mtk[13][1]*Wpk[10][13][1]))))+((((
      mfk[12][2]*Vpk[10][12][2])+((mfk[12][0]*Vpk[10][12][0])+(mfk[12][1]*
      Vpk[10][12][1])))+((mtk[12][2]*Wpk[10][12][2])+((mtk[12][0]*Wpk[10][12][0]
      )+(mtk[12][1]*Wpk[10][12][1]))))+temp[0])));
    temp[0] = ((((mfk[11][2]*Vpk[11][11][2])+((mfk[11][0]*Vpk[11][11][0])+(
      mfk[11][1]*Vpk[11][11][1])))+((mtk[11][2]*pin[11][2])+((mtk[11][0]*
      pin[11][0])+(mtk[11][1]*pin[11][1]))))+(((mfk[12][2]*Vpk[11][12][2])+((
      mfk[12][0]*Vpk[11][12][0])+(mfk[12][1]*Vpk[11][12][1])))+((mtk[12][2]*
      Wpk[11][12][2])+((mtk[12][0]*Wpk[11][12][0])+(mtk[12][1]*Wpk[11][12][1])))
      ));
    fs[11] = (mtau[11]+((((mfk[13][2]*Vpk[11][13][2])+((mfk[13][0]*
      Vpk[11][13][0])+(mfk[13][1]*Vpk[11][13][1])))+((mtk[13][2]*Wpk[11][13][2])
      +((mtk[13][0]*Wpk[11][13][0])+(mtk[13][1]*Wpk[11][13][1]))))+temp[0]));
    fs[12] = (mtau[12]+((((mfk[12][2]*Vpk[12][12][2])+((mfk[12][0]*
      Vpk[12][12][0])+(mfk[12][1]*Vpk[12][12][1])))+((mtk[12][2]*pin[12][2])+((
      mtk[12][0]*pin[12][0])+(mtk[12][1]*pin[12][1]))))+(((mfk[13][2]*
      Vpk[12][13][2])+((mfk[13][0]*Vpk[12][13][0])+(mfk[13][1]*Vpk[12][13][1])))
      +((mtk[13][2]*Wpk[12][13][2])+((mtk[13][0]*Wpk[12][13][0])+(mtk[13][1]*
      Wpk[12][13][1]))))));
    fs[13] = (mtau[13]+(((mfk[13][2]*Vpk[13][13][2])+((mfk[13][0]*Vpk[13][13][0]
      )+(mfk[13][1]*Vpk[13][13][1])))+((mtk[13][2]*pin[13][2])+((mtk[13][0]*
      pin[13][0])+(mtk[13][1]*pin[13][1])))));
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  342 adds/subtracts/negates
                    342 multiplies
                      0 divides
                     32 assignments
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
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   14 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     14 assignments
*/
}

void sdfulltrq(double udotin[14],
    double multin[14],
    double trqout[14])
{
/* Compute hinge torques which would produce indicated udots
*/
    double fstarr[14][3],tstarr[14][3],Otkr[14][3],Atir[14][3],Atkr[14][3];

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
    Otkr[0][0] = (pin[0][0]*udotin[0]);
    Otkr[0][1] = (pin[0][1]*udotin[0]);
    Otkr[0][2] = (pin[0][2]*udotin[0]);
    Otkr[1][0] = (((Cik[1][2][0]*Otkr[0][2])+((Cik[1][0][0]*Otkr[0][0])+(
      Cik[1][1][0]*Otkr[0][1])))+((pin[1][0]*udotin[1])+((Wik[1][2]*wk[1][1])-(
      Wik[1][1]*wk[1][2]))));
    Otkr[1][1] = (((Cik[1][2][1]*Otkr[0][2])+((Cik[1][0][1]*Otkr[0][0])+(
      Cik[1][1][1]*Otkr[0][1])))+((pin[1][1]*udotin[1])+((Wik[1][0]*wk[1][2])-(
      Wik[1][2]*wk[1][0]))));
    Otkr[1][2] = (((Cik[1][2][2]*Otkr[0][2])+((Cik[1][0][2]*Otkr[0][0])+(
      Cik[1][1][2]*Otkr[0][1])))+((pin[1][2]*udotin[1])+((Wik[1][1]*wk[1][0])-(
      Wik[1][0]*wk[1][1]))));
    Otkr[2][0] = (((Cik[2][2][0]*Otkr[0][2])+((Cik[2][0][0]*Otkr[0][0])+(
      Cik[2][1][0]*Otkr[0][1])))+((pin[2][0]*udotin[2])+((Wik[2][2]*wk[2][1])-(
      Wik[2][1]*wk[2][2]))));
    Otkr[2][1] = (((Cik[2][2][1]*Otkr[0][2])+((Cik[2][0][1]*Otkr[0][0])+(
      Cik[2][1][1]*Otkr[0][1])))+((pin[2][1]*udotin[2])+((Wik[2][0]*wk[2][2])-(
      Wik[2][2]*wk[2][0]))));
    Otkr[2][2] = (((Cik[2][2][2]*Otkr[0][2])+((Cik[2][0][2]*Otkr[0][0])+(
      Cik[2][1][2]*Otkr[0][1])))+((pin[2][2]*udotin[2])+((Wik[2][1]*wk[2][0])-(
      Wik[2][0]*wk[2][1]))));
    Otkr[3][0] = (((Cik[3][2][0]*Otkr[2][2])+((Cik[3][0][0]*Otkr[2][0])+(
      Cik[3][1][0]*Otkr[2][1])))+((pin[3][0]*udotin[3])+((Wik[3][2]*wk[3][1])-(
      Wik[3][1]*wk[3][2]))));
    Otkr[3][1] = (((Cik[3][2][1]*Otkr[2][2])+((Cik[3][0][1]*Otkr[2][0])+(
      Cik[3][1][1]*Otkr[2][1])))+((pin[3][1]*udotin[3])+((Wik[3][0]*wk[3][2])-(
      Wik[3][2]*wk[3][0]))));
    Otkr[3][2] = (((Cik[3][2][2]*Otkr[2][2])+((Cik[3][0][2]*Otkr[2][0])+(
      Cik[3][1][2]*Otkr[2][1])))+((pin[3][2]*udotin[3])+((Wik[3][1]*wk[3][0])-(
      Wik[3][0]*wk[3][1]))));
    Otkr[4][0] = (((Cik[4][2][0]*Otkr[3][2])+((Cik[4][0][0]*Otkr[3][0])+(
      Cik[4][1][0]*Otkr[3][1])))+((pin[4][0]*udotin[4])+((Wik[4][2]*wk[4][1])-(
      Wik[4][1]*wk[4][2]))));
    Otkr[4][1] = (((Cik[4][2][1]*Otkr[3][2])+((Cik[4][0][1]*Otkr[3][0])+(
      Cik[4][1][1]*Otkr[3][1])))+((pin[4][1]*udotin[4])+((Wik[4][0]*wk[4][2])-(
      Wik[4][2]*wk[4][0]))));
    Otkr[4][2] = (((Cik[4][2][2]*Otkr[3][2])+((Cik[4][0][2]*Otkr[3][0])+(
      Cik[4][1][2]*Otkr[3][1])))+((pin[4][2]*udotin[4])+((Wik[4][1]*wk[4][0])-(
      Wik[4][0]*wk[4][1]))));
    Otkr[5][0] = (((Cik[5][2][0]*Otkr[4][2])+((Cik[5][0][0]*Otkr[4][0])+(
      Cik[5][1][0]*Otkr[4][1])))+((pin[5][0]*udotin[5])+((Wik[5][2]*wk[5][1])-(
      Wik[5][1]*wk[5][2]))));
    Otkr[5][1] = (((Cik[5][2][1]*Otkr[4][2])+((Cik[5][0][1]*Otkr[4][0])+(
      Cik[5][1][1]*Otkr[4][1])))+((pin[5][1]*udotin[5])+((Wik[5][0]*wk[5][2])-(
      Wik[5][2]*wk[5][0]))));
    Otkr[5][2] = (((Cik[5][2][2]*Otkr[4][2])+((Cik[5][0][2]*Otkr[4][0])+(
      Cik[5][1][2]*Otkr[4][1])))+((pin[5][2]*udotin[5])+((Wik[5][1]*wk[5][0])-(
      Wik[5][0]*wk[5][1]))));
    Otkr[6][0] = (((Cik[6][2][0]*Otkr[5][2])+((Cik[6][0][0]*Otkr[5][0])+(
      Cik[6][1][0]*Otkr[5][1])))+((pin[6][0]*udotin[6])+((Wik[6][2]*wk[6][1])-(
      Wik[6][1]*wk[6][2]))));
    Otkr[6][1] = (((Cik[6][2][1]*Otkr[5][2])+((Cik[6][0][1]*Otkr[5][0])+(
      Cik[6][1][1]*Otkr[5][1])))+((pin[6][1]*udotin[6])+((Wik[6][0]*wk[6][2])-(
      Wik[6][2]*wk[6][0]))));
    Otkr[6][2] = (((Cik[6][2][2]*Otkr[5][2])+((Cik[6][0][2]*Otkr[5][0])+(
      Cik[6][1][2]*Otkr[5][1])))+((pin[6][2]*udotin[6])+((Wik[6][1]*wk[6][0])-(
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
    Otkr[8][0] = (((Cik[8][2][0]*Otkr[0][2])+((Cik[8][0][0]*Otkr[0][0])+(
      Cik[8][1][0]*Otkr[0][1])))+((pin[8][0]*udotin[8])+((Wik[8][2]*wk[8][1])-(
      Wik[8][1]*wk[8][2]))));
    Otkr[8][1] = (((Cik[8][2][1]*Otkr[0][2])+((Cik[8][0][1]*Otkr[0][0])+(
      Cik[8][1][1]*Otkr[0][1])))+((pin[8][1]*udotin[8])+((Wik[8][0]*wk[8][2])-(
      Wik[8][2]*wk[8][0]))));
    Otkr[8][2] = (((Cik[8][2][2]*Otkr[0][2])+((Cik[8][0][2]*Otkr[0][0])+(
      Cik[8][1][2]*Otkr[0][1])))+((pin[8][2]*udotin[8])+((Wik[8][1]*wk[8][0])-(
      Wik[8][0]*wk[8][1]))));
    Otkr[9][0] = (((Cik[9][2][0]*Otkr[8][2])+((Cik[9][0][0]*Otkr[8][0])+(
      Cik[9][1][0]*Otkr[8][1])))+((pin[9][0]*udotin[9])+((Wik[9][2]*wk[9][1])-(
      Wik[9][1]*wk[9][2]))));
    Otkr[9][1] = (((Cik[9][2][1]*Otkr[8][2])+((Cik[9][0][1]*Otkr[8][0])+(
      Cik[9][1][1]*Otkr[8][1])))+((pin[9][1]*udotin[9])+((Wik[9][0]*wk[9][2])-(
      Wik[9][2]*wk[9][0]))));
    Otkr[9][2] = (((Cik[9][2][2]*Otkr[8][2])+((Cik[9][0][2]*Otkr[8][0])+(
      Cik[9][1][2]*Otkr[8][1])))+((pin[9][2]*udotin[9])+((Wik[9][1]*wk[9][0])-(
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
    Atkr[0][0] = (((Otkr[0][2]*rk[0][1])-(Otkr[0][1]*rk[0][2]))+((Wik[0][1]*
      Wkrpk[0][2])-(Wik[0][2]*Wkrpk[0][1])));
    Atkr[0][1] = (((Otkr[0][0]*rk[0][2])-(Otkr[0][2]*rk[0][0]))+((Wik[0][2]*
      Wkrpk[0][0])-(Wik[0][0]*Wkrpk[0][2])));
    Atkr[0][2] = (((Otkr[0][1]*rk[0][0])-(Otkr[0][0]*rk[0][1]))+((Wik[0][0]*
      Wkrpk[0][1])-(Wik[0][1]*Wkrpk[0][0])));
    Atir[1][0] = (Atkr[0][0]+(((Otkr[0][1]*ri[1][2])-(Otkr[0][2]*ri[1][1]))+((
      Wik[0][1]*Wirk[1][2])-(Wik[0][2]*Wirk[1][1]))));
    Atir[1][1] = (Atkr[0][1]+(((Otkr[0][2]*ri[1][0])-(Otkr[0][0]*ri[1][2]))+((
      Wik[0][2]*Wirk[1][0])-(Wik[0][0]*Wirk[1][2]))));
    Atir[1][2] = (Atkr[0][2]+(((Otkr[0][0]*ri[1][1])-(Otkr[0][1]*ri[1][0]))+((
      Wik[0][0]*Wirk[1][1])-(Wik[0][1]*Wirk[1][0]))));
    Atkr[1][0] = (((Atir[1][2]*Cik[1][2][0])+((Atir[1][0]*Cik[1][0][0])+(
      Atir[1][1]*Cik[1][1][0])))+(((Otkr[1][2]*rk[1][1])-(Otkr[1][1]*rk[1][2]))+
      ((wk[1][1]*Wkrpk[1][2])-(wk[1][2]*Wkrpk[1][1]))));
    Atkr[1][1] = (((Atir[1][2]*Cik[1][2][1])+((Atir[1][0]*Cik[1][0][1])+(
      Atir[1][1]*Cik[1][1][1])))+(((Otkr[1][0]*rk[1][2])-(Otkr[1][2]*rk[1][0]))+
      ((wk[1][2]*Wkrpk[1][0])-(wk[1][0]*Wkrpk[1][2]))));
    Atkr[1][2] = (((Atir[1][2]*Cik[1][2][2])+((Atir[1][0]*Cik[1][0][2])+(
      Atir[1][1]*Cik[1][1][2])))+(((Otkr[1][1]*rk[1][0])-(Otkr[1][0]*rk[1][1]))+
      ((wk[1][0]*Wkrpk[1][1])-(wk[1][1]*Wkrpk[1][0]))));
    Atir[2][0] = (Atkr[0][0]+(((Otkr[0][1]*ri[2][2])-(Otkr[0][2]*ri[2][1]))+((
      Wik[0][1]*Wirk[2][2])-(Wik[0][2]*Wirk[2][1]))));
    Atir[2][1] = (Atkr[0][1]+(((Otkr[0][2]*ri[2][0])-(Otkr[0][0]*ri[2][2]))+((
      Wik[0][2]*Wirk[2][0])-(Wik[0][0]*Wirk[2][2]))));
    Atir[2][2] = (Atkr[0][2]+(((Otkr[0][0]*ri[2][1])-(Otkr[0][1]*ri[2][0]))+((
      Wik[0][0]*Wirk[2][1])-(Wik[0][1]*Wirk[2][0]))));
    Atkr[2][0] = (((Atir[2][2]*Cik[2][2][0])+((Atir[2][0]*Cik[2][0][0])+(
      Atir[2][1]*Cik[2][1][0])))+(((Otkr[2][2]*rk[2][1])-(Otkr[2][1]*rk[2][2]))+
      ((wk[2][1]*Wkrpk[2][2])-(wk[2][2]*Wkrpk[2][1]))));
    Atkr[2][1] = (((Atir[2][2]*Cik[2][2][1])+((Atir[2][0]*Cik[2][0][1])+(
      Atir[2][1]*Cik[2][1][1])))+(((Otkr[2][0]*rk[2][2])-(Otkr[2][2]*rk[2][0]))+
      ((wk[2][2]*Wkrpk[2][0])-(wk[2][0]*Wkrpk[2][2]))));
    Atkr[2][2] = (((Atir[2][2]*Cik[2][2][2])+((Atir[2][0]*Cik[2][0][2])+(
      Atir[2][1]*Cik[2][1][2])))+(((Otkr[2][1]*rk[2][0])-(Otkr[2][0]*rk[2][1]))+
      ((wk[2][0]*Wkrpk[2][1])-(wk[2][1]*Wkrpk[2][0]))));
    Atir[3][0] = (Atkr[2][0]+(((Otkr[2][1]*ri[3][2])-(Otkr[2][2]*ri[3][1]))+((
      Wirk[3][2]*wk[2][1])-(Wirk[3][1]*wk[2][2]))));
    Atir[3][1] = (Atkr[2][1]+(((Otkr[2][2]*ri[3][0])-(Otkr[2][0]*ri[3][2]))+((
      Wirk[3][0]*wk[2][2])-(Wirk[3][2]*wk[2][0]))));
    Atir[3][2] = (Atkr[2][2]+(((Otkr[2][0]*ri[3][1])-(Otkr[2][1]*ri[3][0]))+((
      Wirk[3][1]*wk[2][0])-(Wirk[3][0]*wk[2][1]))));
    Atkr[3][0] = (((Atir[3][2]*Cik[3][2][0])+((Atir[3][0]*Cik[3][0][0])+(
      Atir[3][1]*Cik[3][1][0])))+(((Otkr[3][2]*rk[3][1])-(Otkr[3][1]*rk[3][2]))+
      ((wk[3][1]*Wkrpk[3][2])-(wk[3][2]*Wkrpk[3][1]))));
    Atkr[3][1] = (((Atir[3][2]*Cik[3][2][1])+((Atir[3][0]*Cik[3][0][1])+(
      Atir[3][1]*Cik[3][1][1])))+(((Otkr[3][0]*rk[3][2])-(Otkr[3][2]*rk[3][0]))+
      ((wk[3][2]*Wkrpk[3][0])-(wk[3][0]*Wkrpk[3][2]))));
    Atkr[3][2] = (((Atir[3][2]*Cik[3][2][2])+((Atir[3][0]*Cik[3][0][2])+(
      Atir[3][1]*Cik[3][1][2])))+(((Otkr[3][1]*rk[3][0])-(Otkr[3][0]*rk[3][1]))+
      ((wk[3][0]*Wkrpk[3][1])-(wk[3][1]*Wkrpk[3][0]))));
    Atir[4][0] = (Atkr[3][0]+(((Otkr[3][1]*ri[4][2])-(Otkr[3][2]*ri[4][1]))+((
      Wirk[4][2]*wk[3][1])-(Wirk[4][1]*wk[3][2]))));
    Atir[4][1] = (Atkr[3][1]+(((Otkr[3][2]*ri[4][0])-(Otkr[3][0]*ri[4][2]))+((
      Wirk[4][0]*wk[3][2])-(Wirk[4][2]*wk[3][0]))));
    Atir[4][2] = (Atkr[3][2]+(((Otkr[3][0]*ri[4][1])-(Otkr[3][1]*ri[4][0]))+((
      Wirk[4][1]*wk[3][0])-(Wirk[4][0]*wk[3][1]))));
    Atkr[4][0] = (((Atir[4][2]*Cik[4][2][0])+((Atir[4][0]*Cik[4][0][0])+(
      Atir[4][1]*Cik[4][1][0])))+(((Otkr[4][2]*rk[4][1])-(Otkr[4][1]*rk[4][2]))+
      ((wk[4][1]*Wkrpk[4][2])-(wk[4][2]*Wkrpk[4][1]))));
    Atkr[4][1] = (((Atir[4][2]*Cik[4][2][1])+((Atir[4][0]*Cik[4][0][1])+(
      Atir[4][1]*Cik[4][1][1])))+(((Otkr[4][0]*rk[4][2])-(Otkr[4][2]*rk[4][0]))+
      ((wk[4][2]*Wkrpk[4][0])-(wk[4][0]*Wkrpk[4][2]))));
    Atkr[4][2] = (((Atir[4][2]*Cik[4][2][2])+((Atir[4][0]*Cik[4][0][2])+(
      Atir[4][1]*Cik[4][1][2])))+(((Otkr[4][1]*rk[4][0])-(Otkr[4][0]*rk[4][1]))+
      ((wk[4][0]*Wkrpk[4][1])-(wk[4][1]*Wkrpk[4][0]))));
    Atir[5][0] = (Atkr[4][0]+(((Otkr[4][1]*ri[5][2])-(Otkr[4][2]*ri[5][1]))+((
      Wirk[5][2]*wk[4][1])-(Wirk[5][1]*wk[4][2]))));
    Atir[5][1] = (Atkr[4][1]+(((Otkr[4][2]*ri[5][0])-(Otkr[4][0]*ri[5][2]))+((
      Wirk[5][0]*wk[4][2])-(Wirk[5][2]*wk[4][0]))));
    Atir[5][2] = (Atkr[4][2]+(((Otkr[4][0]*ri[5][1])-(Otkr[4][1]*ri[5][0]))+((
      Wirk[5][1]*wk[4][0])-(Wirk[5][0]*wk[4][1]))));
    Atkr[5][0] = (((Atir[5][2]*Cik[5][2][0])+((Atir[5][0]*Cik[5][0][0])+(
      Atir[5][1]*Cik[5][1][0])))+(((Otkr[5][2]*rk[5][1])-(Otkr[5][1]*rk[5][2]))+
      ((wk[5][1]*Wkrpk[5][2])-(wk[5][2]*Wkrpk[5][1]))));
    Atkr[5][1] = (((Atir[5][2]*Cik[5][2][1])+((Atir[5][0]*Cik[5][0][1])+(
      Atir[5][1]*Cik[5][1][1])))+(((Otkr[5][0]*rk[5][2])-(Otkr[5][2]*rk[5][0]))+
      ((wk[5][2]*Wkrpk[5][0])-(wk[5][0]*Wkrpk[5][2]))));
    Atkr[5][2] = (((Atir[5][2]*Cik[5][2][2])+((Atir[5][0]*Cik[5][0][2])+(
      Atir[5][1]*Cik[5][1][2])))+(((Otkr[5][1]*rk[5][0])-(Otkr[5][0]*rk[5][1]))+
      ((wk[5][0]*Wkrpk[5][1])-(wk[5][1]*Wkrpk[5][0]))));
    Atir[6][0] = (Atkr[5][0]+(((Otkr[5][1]*ri[6][2])-(Otkr[5][2]*ri[6][1]))+((
      Wirk[6][2]*wk[5][1])-(Wirk[6][1]*wk[5][2]))));
    Atir[6][1] = (Atkr[5][1]+(((Otkr[5][2]*ri[6][0])-(Otkr[5][0]*ri[6][2]))+((
      Wirk[6][0]*wk[5][2])-(Wirk[6][2]*wk[5][0]))));
    Atir[6][2] = (Atkr[5][2]+(((Otkr[5][0]*ri[6][1])-(Otkr[5][1]*ri[6][0]))+((
      Wirk[6][1]*wk[5][0])-(Wirk[6][0]*wk[5][1]))));
    Atkr[6][0] = (((Atir[6][2]*Cik[6][2][0])+((Atir[6][0]*Cik[6][0][0])+(
      Atir[6][1]*Cik[6][1][0])))+(((Otkr[6][2]*rk[6][1])-(Otkr[6][1]*rk[6][2]))+
      ((wk[6][1]*Wkrpk[6][2])-(wk[6][2]*Wkrpk[6][1]))));
    Atkr[6][1] = (((Atir[6][2]*Cik[6][2][1])+((Atir[6][0]*Cik[6][0][1])+(
      Atir[6][1]*Cik[6][1][1])))+(((Otkr[6][0]*rk[6][2])-(Otkr[6][2]*rk[6][0]))+
      ((wk[6][2]*Wkrpk[6][0])-(wk[6][0]*Wkrpk[6][2]))));
    Atkr[6][2] = (((Atir[6][2]*Cik[6][2][2])+((Atir[6][0]*Cik[6][0][2])+(
      Atir[6][1]*Cik[6][1][2])))+(((Otkr[6][1]*rk[6][0])-(Otkr[6][0]*rk[6][1]))+
      ((wk[6][0]*Wkrpk[6][1])-(wk[6][1]*Wkrpk[6][0]))));
    Atir[7][0] = (Atkr[6][0]+(((Otkr[6][1]*ri[7][2])-(Otkr[6][2]*ri[7][1]))+((
      Wirk[7][2]*wk[6][1])-(Wirk[7][1]*wk[6][2]))));
    Atir[7][1] = (Atkr[6][1]+(((Otkr[6][2]*ri[7][0])-(Otkr[6][0]*ri[7][2]))+((
      Wirk[7][0]*wk[6][2])-(Wirk[7][2]*wk[6][0]))));
    Atir[7][2] = (Atkr[6][2]+(((Otkr[6][0]*ri[7][1])-(Otkr[6][1]*ri[7][0]))+((
      Wirk[7][1]*wk[6][0])-(Wirk[7][0]*wk[6][1]))));
    Atkr[7][0] = (((Atir[7][2]*Cik[7][2][0])+((Atir[7][0]*Cik[7][0][0])+(
      Atir[7][1]*Cik[7][1][0])))+(((Otkr[7][2]*rk[7][1])-(Otkr[7][1]*rk[7][2]))+
      ((wk[7][1]*Wkrpk[7][2])-(wk[7][2]*Wkrpk[7][1]))));
    Atkr[7][1] = (((Atir[7][2]*Cik[7][2][1])+((Atir[7][0]*Cik[7][0][1])+(
      Atir[7][1]*Cik[7][1][1])))+(((Otkr[7][0]*rk[7][2])-(Otkr[7][2]*rk[7][0]))+
      ((wk[7][2]*Wkrpk[7][0])-(wk[7][0]*Wkrpk[7][2]))));
    Atkr[7][2] = (((Atir[7][2]*Cik[7][2][2])+((Atir[7][0]*Cik[7][0][2])+(
      Atir[7][1]*Cik[7][1][2])))+(((Otkr[7][1]*rk[7][0])-(Otkr[7][0]*rk[7][1]))+
      ((wk[7][0]*Wkrpk[7][1])-(wk[7][1]*Wkrpk[7][0]))));
    Atir[8][0] = (Atkr[0][0]+(((Otkr[0][1]*ri[8][2])-(Otkr[0][2]*ri[8][1]))+((
      Wik[0][1]*Wirk[8][2])-(Wik[0][2]*Wirk[8][1]))));
    Atir[8][1] = (Atkr[0][1]+(((Otkr[0][2]*ri[8][0])-(Otkr[0][0]*ri[8][2]))+((
      Wik[0][2]*Wirk[8][0])-(Wik[0][0]*Wirk[8][2]))));
    Atir[8][2] = (Atkr[0][2]+(((Otkr[0][0]*ri[8][1])-(Otkr[0][1]*ri[8][0]))+((
      Wik[0][0]*Wirk[8][1])-(Wik[0][1]*Wirk[8][0]))));
    Atkr[8][0] = (((Atir[8][2]*Cik[8][2][0])+((Atir[8][0]*Cik[8][0][0])+(
      Atir[8][1]*Cik[8][1][0])))+(((Otkr[8][2]*rk[8][1])-(Otkr[8][1]*rk[8][2]))+
      ((wk[8][1]*Wkrpk[8][2])-(wk[8][2]*Wkrpk[8][1]))));
    Atkr[8][1] = (((Atir[8][2]*Cik[8][2][1])+((Atir[8][0]*Cik[8][0][1])+(
      Atir[8][1]*Cik[8][1][1])))+(((Otkr[8][0]*rk[8][2])-(Otkr[8][2]*rk[8][0]))+
      ((wk[8][2]*Wkrpk[8][0])-(wk[8][0]*Wkrpk[8][2]))));
    Atkr[8][2] = (((Atir[8][2]*Cik[8][2][2])+((Atir[8][0]*Cik[8][0][2])+(
      Atir[8][1]*Cik[8][1][2])))+(((Otkr[8][1]*rk[8][0])-(Otkr[8][0]*rk[8][1]))+
      ((wk[8][0]*Wkrpk[8][1])-(wk[8][1]*Wkrpk[8][0]))));
    Atir[9][0] = (Atkr[8][0]+(((Otkr[8][1]*ri[9][2])-(Otkr[8][2]*ri[9][1]))+((
      Wirk[9][2]*wk[8][1])-(Wirk[9][1]*wk[8][2]))));
    Atir[9][1] = (Atkr[8][1]+(((Otkr[8][2]*ri[9][0])-(Otkr[8][0]*ri[9][2]))+((
      Wirk[9][0]*wk[8][2])-(Wirk[9][2]*wk[8][0]))));
    Atir[9][2] = (Atkr[8][2]+(((Otkr[8][0]*ri[9][1])-(Otkr[8][1]*ri[9][0]))+((
      Wirk[9][1]*wk[8][0])-(Wirk[9][0]*wk[8][1]))));
    Atkr[9][0] = (((Atir[9][2]*Cik[9][2][0])+((Atir[9][0]*Cik[9][0][0])+(
      Atir[9][1]*Cik[9][1][0])))+(((Otkr[9][2]*rk[9][1])-(Otkr[9][1]*rk[9][2]))+
      ((wk[9][1]*Wkrpk[9][2])-(wk[9][2]*Wkrpk[9][1]))));
    Atkr[9][1] = (((Atir[9][2]*Cik[9][2][1])+((Atir[9][0]*Cik[9][0][1])+(
      Atir[9][1]*Cik[9][1][1])))+(((Otkr[9][0]*rk[9][2])-(Otkr[9][2]*rk[9][0]))+
      ((wk[9][2]*Wkrpk[9][0])-(wk[9][0]*Wkrpk[9][2]))));
    Atkr[9][2] = (((Atir[9][2]*Cik[9][2][2])+((Atir[9][0]*Cik[9][0][2])+(
      Atir[9][1]*Cik[9][1][2])))+(((Otkr[9][1]*rk[9][0])-(Otkr[9][0]*rk[9][1]))+
      ((wk[9][0]*Wkrpk[9][1])-(wk[9][1]*Wkrpk[9][0]))));
    Atir[10][0] = (Atkr[9][0]+(((Otkr[9][1]*ri[10][2])-(Otkr[9][2]*ri[10][1]))+(
      (Wirk[10][2]*wk[9][1])-(Wirk[10][1]*wk[9][2]))));
    Atir[10][1] = (Atkr[9][1]+(((Otkr[9][2]*ri[10][0])-(Otkr[9][0]*ri[10][2]))+(
      (Wirk[10][0]*wk[9][2])-(Wirk[10][2]*wk[9][0]))));
    Atir[10][2] = (Atkr[9][2]+(((Otkr[9][0]*ri[10][1])-(Otkr[9][1]*ri[10][0]))+(
      (Wirk[10][1]*wk[9][0])-(Wirk[10][0]*wk[9][1]))));
    Atkr[10][0] = (((Atir[10][2]*Cik[10][2][0])+((Atir[10][0]*Cik[10][0][0])+(
      Atir[10][1]*Cik[10][1][0])))+(((Otkr[10][2]*rk[10][1])-(Otkr[10][1]*
      rk[10][2]))+((wk[10][1]*Wkrpk[10][2])-(wk[10][2]*Wkrpk[10][1]))));
    Atkr[10][1] = (((Atir[10][2]*Cik[10][2][1])+((Atir[10][0]*Cik[10][0][1])+(
      Atir[10][1]*Cik[10][1][1])))+(((Otkr[10][0]*rk[10][2])-(Otkr[10][2]*
      rk[10][0]))+((wk[10][2]*Wkrpk[10][0])-(wk[10][0]*Wkrpk[10][2]))));
    Atkr[10][2] = (((Atir[10][2]*Cik[10][2][2])+((Atir[10][0]*Cik[10][0][2])+(
      Atir[10][1]*Cik[10][1][2])))+(((Otkr[10][1]*rk[10][0])-(Otkr[10][0]*
      rk[10][1]))+((wk[10][0]*Wkrpk[10][1])-(wk[10][1]*Wkrpk[10][0]))));
    Atir[11][0] = (Atkr[10][0]+(((Otkr[10][1]*ri[11][2])-(Otkr[10][2]*ri[11][1])
      )+((Wirk[11][2]*wk[10][1])-(Wirk[11][1]*wk[10][2]))));
    Atir[11][1] = (Atkr[10][1]+(((Otkr[10][2]*ri[11][0])-(Otkr[10][0]*ri[11][2])
      )+((Wirk[11][0]*wk[10][2])-(Wirk[11][2]*wk[10][0]))));
    Atir[11][2] = (Atkr[10][2]+(((Otkr[10][0]*ri[11][1])-(Otkr[10][1]*ri[11][0])
      )+((Wirk[11][1]*wk[10][0])-(Wirk[11][0]*wk[10][1]))));
    Atkr[11][0] = (((Atir[11][2]*Cik[11][2][0])+((Atir[11][0]*Cik[11][0][0])+(
      Atir[11][1]*Cik[11][1][0])))+(((Otkr[11][2]*rk[11][1])-(Otkr[11][1]*
      rk[11][2]))+((wk[11][1]*Wkrpk[11][2])-(wk[11][2]*Wkrpk[11][1]))));
    Atkr[11][1] = (((Atir[11][2]*Cik[11][2][1])+((Atir[11][0]*Cik[11][0][1])+(
      Atir[11][1]*Cik[11][1][1])))+(((Otkr[11][0]*rk[11][2])-(Otkr[11][2]*
      rk[11][0]))+((wk[11][2]*Wkrpk[11][0])-(wk[11][0]*Wkrpk[11][2]))));
    Atkr[11][2] = (((Atir[11][2]*Cik[11][2][2])+((Atir[11][0]*Cik[11][0][2])+(
      Atir[11][1]*Cik[11][1][2])))+(((Otkr[11][1]*rk[11][0])-(Otkr[11][0]*
      rk[11][1]))+((wk[11][0]*Wkrpk[11][1])-(wk[11][1]*Wkrpk[11][0]))));
    Atir[12][0] = (Atkr[11][0]+(((Otkr[11][1]*ri[12][2])-(Otkr[11][2]*ri[12][1])
      )+((Wirk[12][2]*wk[11][1])-(Wirk[12][1]*wk[11][2]))));
    Atir[12][1] = (Atkr[11][1]+(((Otkr[11][2]*ri[12][0])-(Otkr[11][0]*ri[12][2])
      )+((Wirk[12][0]*wk[11][2])-(Wirk[12][2]*wk[11][0]))));
    Atir[12][2] = (Atkr[11][2]+(((Otkr[11][0]*ri[12][1])-(Otkr[11][1]*ri[12][0])
      )+((Wirk[12][1]*wk[11][0])-(Wirk[12][0]*wk[11][1]))));
    Atkr[12][0] = (((Atir[12][2]*Cik[12][2][0])+((Atir[12][0]*Cik[12][0][0])+(
      Atir[12][1]*Cik[12][1][0])))+(((Otkr[12][2]*rk[12][1])-(Otkr[12][1]*
      rk[12][2]))+((wk[12][1]*Wkrpk[12][2])-(wk[12][2]*Wkrpk[12][1]))));
    Atkr[12][1] = (((Atir[12][2]*Cik[12][2][1])+((Atir[12][0]*Cik[12][0][1])+(
      Atir[12][1]*Cik[12][1][1])))+(((Otkr[12][0]*rk[12][2])-(Otkr[12][2]*
      rk[12][0]))+((wk[12][2]*Wkrpk[12][0])-(wk[12][0]*Wkrpk[12][2]))));
    Atkr[12][2] = (((Atir[12][2]*Cik[12][2][2])+((Atir[12][0]*Cik[12][0][2])+(
      Atir[12][1]*Cik[12][1][2])))+(((Otkr[12][1]*rk[12][0])-(Otkr[12][0]*
      rk[12][1]))+((wk[12][0]*Wkrpk[12][1])-(wk[12][1]*Wkrpk[12][0]))));
    Atir[13][0] = (Atkr[12][0]+(((Otkr[12][1]*ri[13][2])-(Otkr[12][2]*ri[13][1])
      )+((Wirk[13][2]*wk[12][1])-(Wirk[13][1]*wk[12][2]))));
    Atir[13][1] = (Atkr[12][1]+(((Otkr[12][2]*ri[13][0])-(Otkr[12][0]*ri[13][2])
      )+((Wirk[13][0]*wk[12][2])-(Wirk[13][2]*wk[12][0]))));
    Atir[13][2] = (Atkr[12][2]+(((Otkr[12][0]*ri[13][1])-(Otkr[12][1]*ri[13][0])
      )+((Wirk[13][1]*wk[12][0])-(Wirk[13][0]*wk[12][1]))));
    Atkr[13][0] = (((Atir[13][2]*Cik[13][2][0])+((Atir[13][0]*Cik[13][0][0])+(
      Atir[13][1]*Cik[13][1][0])))+(((Otkr[13][2]*rk[13][1])-(Otkr[13][1]*
      rk[13][2]))+((wk[13][1]*Wkrpk[13][2])-(wk[13][2]*Wkrpk[13][1]))));
    Atkr[13][1] = (((Atir[13][2]*Cik[13][2][1])+((Atir[13][0]*Cik[13][0][1])+(
      Atir[13][1]*Cik[13][1][1])))+(((Otkr[13][0]*rk[13][2])-(Otkr[13][2]*
      rk[13][0]))+((wk[13][2]*Wkrpk[13][0])-(wk[13][0]*Wkrpk[13][2]))));
    Atkr[13][2] = (((Atir[13][2]*Cik[13][2][2])+((Atir[13][0]*Cik[13][0][2])+(
      Atir[13][1]*Cik[13][1][2])))+(((Otkr[13][1]*rk[13][0])-(Otkr[13][0]*
      rk[13][1]))+((wk[13][0]*Wkrpk[13][1])-(wk[13][1]*Wkrpk[13][0]))));
/*
Accumulate all forces and torques
*/
    fstarr[0][0] = (ufk[0][0]+(mk[0]*(gk[0][0]-Atkr[0][0])));
    fstarr[0][1] = (ufk[0][1]+(mk[0]*(gk[0][1]-Atkr[0][1])));
    fstarr[0][2] = (ufk[0][2]+(mk[0]*(gk[0][2]-Atkr[0][2])));
    fstarr[1][0] = (ufk[1][0]+(mk[1]*(gk[1][0]-Atkr[1][0])));
    fstarr[1][1] = (ufk[1][1]+(mk[1]*(gk[1][1]-Atkr[1][1])));
    fstarr[1][2] = (ufk[1][2]+(mk[1]*(gk[1][2]-Atkr[1][2])));
    fstarr[2][0] = (ufk[2][0]+(mk[2]*(gk[2][0]-Atkr[2][0])));
    fstarr[2][1] = (ufk[2][1]+(mk[2]*(gk[2][1]-Atkr[2][1])));
    fstarr[2][2] = (ufk[2][2]+(mk[2]*(gk[2][2]-Atkr[2][2])));
    fstarr[3][0] = (ufk[3][0]+(mk[3]*(gk[3][0]-Atkr[3][0])));
    fstarr[3][1] = (ufk[3][1]+(mk[3]*(gk[3][1]-Atkr[3][1])));
    fstarr[3][2] = (ufk[3][2]+(mk[3]*(gk[3][2]-Atkr[3][2])));
    fstarr[4][0] = (ufk[4][0]+(mk[4]*(gk[4][0]-Atkr[4][0])));
    fstarr[4][1] = (ufk[4][1]+(mk[4]*(gk[4][1]-Atkr[4][1])));
    fstarr[4][2] = (ufk[4][2]+(mk[4]*(gk[4][2]-Atkr[4][2])));
    fstarr[5][0] = (ufk[5][0]+(mk[5]*(gk[5][0]-Atkr[5][0])));
    fstarr[5][1] = (ufk[5][1]+(mk[5]*(gk[5][1]-Atkr[5][1])));
    fstarr[5][2] = (ufk[5][2]+(mk[5]*(gk[5][2]-Atkr[5][2])));
    fstarr[6][0] = (ufk[6][0]+(mk[6]*(gk[6][0]-Atkr[6][0])));
    fstarr[6][1] = (ufk[6][1]+(mk[6]*(gk[6][1]-Atkr[6][1])));
    fstarr[6][2] = (ufk[6][2]+(mk[6]*(gk[6][2]-Atkr[6][2])));
    fstarr[7][0] = (ufk[7][0]+(mk[7]*(gk[7][0]-Atkr[7][0])));
    fstarr[7][1] = (ufk[7][1]+(mk[7]*(gk[7][1]-Atkr[7][1])));
    fstarr[7][2] = (ufk[7][2]+(mk[7]*(gk[7][2]-Atkr[7][2])));
    fstarr[8][0] = (ufk[8][0]+(mk[8]*(gk[8][0]-Atkr[8][0])));
    fstarr[8][1] = (ufk[8][1]+(mk[8]*(gk[8][1]-Atkr[8][1])));
    fstarr[8][2] = (ufk[8][2]+(mk[8]*(gk[8][2]-Atkr[8][2])));
    fstarr[9][0] = (ufk[9][0]+(mk[9]*(gk[9][0]-Atkr[9][0])));
    fstarr[9][1] = (ufk[9][1]+(mk[9]*(gk[9][1]-Atkr[9][1])));
    fstarr[9][2] = (ufk[9][2]+(mk[9]*(gk[9][2]-Atkr[9][2])));
    fstarr[10][0] = (ufk[10][0]+(mk[10]*(gk[10][0]-Atkr[10][0])));
    fstarr[10][1] = (ufk[10][1]+(mk[10]*(gk[10][1]-Atkr[10][1])));
    fstarr[10][2] = (ufk[10][2]+(mk[10]*(gk[10][2]-Atkr[10][2])));
    fstarr[11][0] = (ufk[11][0]+(mk[11]*(gk[11][0]-Atkr[11][0])));
    fstarr[11][1] = (ufk[11][1]+(mk[11]*(gk[11][1]-Atkr[11][1])));
    fstarr[11][2] = (ufk[11][2]+(mk[11]*(gk[11][2]-Atkr[11][2])));
    fstarr[12][0] = (ufk[12][0]+(mk[12]*(gk[12][0]-Atkr[12][0])));
    fstarr[12][1] = (ufk[12][1]+(mk[12]*(gk[12][1]-Atkr[12][1])));
    fstarr[12][2] = (ufk[12][2]+(mk[12]*(gk[12][2]-Atkr[12][2])));
    fstarr[13][0] = (ufk[13][0]+(mk[13]*(gk[13][0]-Atkr[13][0])));
    fstarr[13][1] = (ufk[13][1]+(mk[13]*(gk[13][1]-Atkr[13][1])));
    fstarr[13][2] = (ufk[13][2]+(mk[13]*(gk[13][2]-Atkr[13][2])));
    tstarr[0][0] = (utk[0][0]-(WkIkWk[0][0]+((ik[0][0][2]*Otkr[0][2])+((
      ik[0][0][0]*Otkr[0][0])+(ik[0][0][1]*Otkr[0][1])))));
    tstarr[0][1] = (utk[0][1]-(WkIkWk[0][1]+((ik[0][1][2]*Otkr[0][2])+((
      ik[0][1][0]*Otkr[0][0])+(ik[0][1][1]*Otkr[0][1])))));
    tstarr[0][2] = (utk[0][2]-(WkIkWk[0][2]+((ik[0][2][2]*Otkr[0][2])+((
      ik[0][2][0]*Otkr[0][0])+(ik[0][2][1]*Otkr[0][1])))));
    tstarr[1][0] = (utk[1][0]-(WkIkWk[1][0]+((ik[1][0][2]*Otkr[1][2])+((
      ik[1][0][0]*Otkr[1][0])+(ik[1][0][1]*Otkr[1][1])))));
    tstarr[1][1] = (utk[1][1]-(WkIkWk[1][1]+((ik[1][1][2]*Otkr[1][2])+((
      ik[1][1][0]*Otkr[1][0])+(ik[1][1][1]*Otkr[1][1])))));
    tstarr[1][2] = (utk[1][2]-(WkIkWk[1][2]+((ik[1][2][2]*Otkr[1][2])+((
      ik[1][2][0]*Otkr[1][0])+(ik[1][2][1]*Otkr[1][1])))));
    tstarr[2][0] = (utk[2][0]-(WkIkWk[2][0]+((ik[2][0][2]*Otkr[2][2])+((
      ik[2][0][0]*Otkr[2][0])+(ik[2][0][1]*Otkr[2][1])))));
    tstarr[2][1] = (utk[2][1]-(WkIkWk[2][1]+((ik[2][1][2]*Otkr[2][2])+((
      ik[2][1][0]*Otkr[2][0])+(ik[2][1][1]*Otkr[2][1])))));
    tstarr[2][2] = (utk[2][2]-(WkIkWk[2][2]+((ik[2][2][2]*Otkr[2][2])+((
      ik[2][2][0]*Otkr[2][0])+(ik[2][2][1]*Otkr[2][1])))));
    tstarr[3][0] = (utk[3][0]-(WkIkWk[3][0]+((ik[3][0][2]*Otkr[3][2])+((
      ik[3][0][0]*Otkr[3][0])+(ik[3][0][1]*Otkr[3][1])))));
    tstarr[3][1] = (utk[3][1]-(WkIkWk[3][1]+((ik[3][1][2]*Otkr[3][2])+((
      ik[3][1][0]*Otkr[3][0])+(ik[3][1][1]*Otkr[3][1])))));
    tstarr[3][2] = (utk[3][2]-(WkIkWk[3][2]+((ik[3][2][2]*Otkr[3][2])+((
      ik[3][2][0]*Otkr[3][0])+(ik[3][2][1]*Otkr[3][1])))));
    tstarr[4][0] = (utk[4][0]-(WkIkWk[4][0]+((ik[4][0][2]*Otkr[4][2])+((
      ik[4][0][0]*Otkr[4][0])+(ik[4][0][1]*Otkr[4][1])))));
    tstarr[4][1] = (utk[4][1]-(WkIkWk[4][1]+((ik[4][1][2]*Otkr[4][2])+((
      ik[4][1][0]*Otkr[4][0])+(ik[4][1][1]*Otkr[4][1])))));
    tstarr[4][2] = (utk[4][2]-(WkIkWk[4][2]+((ik[4][2][2]*Otkr[4][2])+((
      ik[4][2][0]*Otkr[4][0])+(ik[4][2][1]*Otkr[4][1])))));
    tstarr[5][0] = (utk[5][0]-(WkIkWk[5][0]+((ik[5][0][2]*Otkr[5][2])+((
      ik[5][0][0]*Otkr[5][0])+(ik[5][0][1]*Otkr[5][1])))));
    tstarr[5][1] = (utk[5][1]-(WkIkWk[5][1]+((ik[5][1][2]*Otkr[5][2])+((
      ik[5][1][0]*Otkr[5][0])+(ik[5][1][1]*Otkr[5][1])))));
    tstarr[5][2] = (utk[5][2]-(WkIkWk[5][2]+((ik[5][2][2]*Otkr[5][2])+((
      ik[5][2][0]*Otkr[5][0])+(ik[5][2][1]*Otkr[5][1])))));
    tstarr[6][0] = (utk[6][0]-(WkIkWk[6][0]+((ik[6][0][2]*Otkr[6][2])+((
      ik[6][0][0]*Otkr[6][0])+(ik[6][0][1]*Otkr[6][1])))));
    tstarr[6][1] = (utk[6][1]-(WkIkWk[6][1]+((ik[6][1][2]*Otkr[6][2])+((
      ik[6][1][0]*Otkr[6][0])+(ik[6][1][1]*Otkr[6][1])))));
    tstarr[6][2] = (utk[6][2]-(WkIkWk[6][2]+((ik[6][2][2]*Otkr[6][2])+((
      ik[6][2][0]*Otkr[6][0])+(ik[6][2][1]*Otkr[6][1])))));
    tstarr[7][0] = (utk[7][0]-(WkIkWk[7][0]+((ik[7][0][2]*Otkr[7][2])+((
      ik[7][0][0]*Otkr[7][0])+(ik[7][0][1]*Otkr[7][1])))));
    tstarr[7][1] = (utk[7][1]-(WkIkWk[7][1]+((ik[7][1][2]*Otkr[7][2])+((
      ik[7][1][0]*Otkr[7][0])+(ik[7][1][1]*Otkr[7][1])))));
    tstarr[7][2] = (utk[7][2]-(WkIkWk[7][2]+((ik[7][2][2]*Otkr[7][2])+((
      ik[7][2][0]*Otkr[7][0])+(ik[7][2][1]*Otkr[7][1])))));
    tstarr[8][0] = (utk[8][0]-(WkIkWk[8][0]+((ik[8][0][2]*Otkr[8][2])+((
      ik[8][0][0]*Otkr[8][0])+(ik[8][0][1]*Otkr[8][1])))));
    tstarr[8][1] = (utk[8][1]-(WkIkWk[8][1]+((ik[8][1][2]*Otkr[8][2])+((
      ik[8][1][0]*Otkr[8][0])+(ik[8][1][1]*Otkr[8][1])))));
    tstarr[8][2] = (utk[8][2]-(WkIkWk[8][2]+((ik[8][2][2]*Otkr[8][2])+((
      ik[8][2][0]*Otkr[8][0])+(ik[8][2][1]*Otkr[8][1])))));
    tstarr[9][0] = (utk[9][0]-(WkIkWk[9][0]+((ik[9][0][2]*Otkr[9][2])+((
      ik[9][0][0]*Otkr[9][0])+(ik[9][0][1]*Otkr[9][1])))));
    tstarr[9][1] = (utk[9][1]-(WkIkWk[9][1]+((ik[9][1][2]*Otkr[9][2])+((
      ik[9][1][0]*Otkr[9][0])+(ik[9][1][1]*Otkr[9][1])))));
    tstarr[9][2] = (utk[9][2]-(WkIkWk[9][2]+((ik[9][2][2]*Otkr[9][2])+((
      ik[9][2][0]*Otkr[9][0])+(ik[9][2][1]*Otkr[9][1])))));
    tstarr[10][0] = (utk[10][0]-(WkIkWk[10][0]+((ik[10][0][2]*Otkr[10][2])+((
      ik[10][0][0]*Otkr[10][0])+(ik[10][0][1]*Otkr[10][1])))));
    tstarr[10][1] = (utk[10][1]-(WkIkWk[10][1]+((ik[10][1][2]*Otkr[10][2])+((
      ik[10][1][0]*Otkr[10][0])+(ik[10][1][1]*Otkr[10][1])))));
    tstarr[10][2] = (utk[10][2]-(WkIkWk[10][2]+((ik[10][2][2]*Otkr[10][2])+((
      ik[10][2][0]*Otkr[10][0])+(ik[10][2][1]*Otkr[10][1])))));
    tstarr[11][0] = (utk[11][0]-(WkIkWk[11][0]+((ik[11][0][2]*Otkr[11][2])+((
      ik[11][0][0]*Otkr[11][0])+(ik[11][0][1]*Otkr[11][1])))));
    tstarr[11][1] = (utk[11][1]-(WkIkWk[11][1]+((ik[11][1][2]*Otkr[11][2])+((
      ik[11][1][0]*Otkr[11][0])+(ik[11][1][1]*Otkr[11][1])))));
    tstarr[11][2] = (utk[11][2]-(WkIkWk[11][2]+((ik[11][2][2]*Otkr[11][2])+((
      ik[11][2][0]*Otkr[11][0])+(ik[11][2][1]*Otkr[11][1])))));
    tstarr[12][0] = (utk[12][0]-(WkIkWk[12][0]+((ik[12][0][2]*Otkr[12][2])+((
      ik[12][0][0]*Otkr[12][0])+(ik[12][0][1]*Otkr[12][1])))));
    tstarr[12][1] = (utk[12][1]-(WkIkWk[12][1]+((ik[12][1][2]*Otkr[12][2])+((
      ik[12][1][0]*Otkr[12][0])+(ik[12][1][1]*Otkr[12][1])))));
    tstarr[12][2] = (utk[12][2]-(WkIkWk[12][2]+((ik[12][2][2]*Otkr[12][2])+((
      ik[12][2][0]*Otkr[12][0])+(ik[12][2][1]*Otkr[12][1])))));
    tstarr[13][0] = (utk[13][0]-(WkIkWk[13][0]+((ik[13][0][2]*Otkr[13][2])+((
      ik[13][0][0]*Otkr[13][0])+(ik[13][0][1]*Otkr[13][1])))));
    tstarr[13][1] = (utk[13][1]-(WkIkWk[13][1]+((ik[13][1][2]*Otkr[13][2])+((
      ik[13][1][0]*Otkr[13][0])+(ik[13][1][1]*Otkr[13][1])))));
    tstarr[13][2] = (utk[13][2]-(WkIkWk[13][2]+((ik[13][2][2]*Otkr[13][2])+((
      ik[13][2][0]*Otkr[13][0])+(ik[13][2][1]*Otkr[13][1])))));
/*
Now calculate the torques
*/
    sddovpk();
    temp[0] = ((((fstarr[0][2]*Vpk[0][0][2])+((fstarr[0][0]*Vpk[0][0][0])+(
      fstarr[0][1]*Vpk[0][0][1])))+((pin[0][2]*tstarr[0][2])+((pin[0][0]*
      tstarr[0][0])+(pin[0][1]*tstarr[0][1]))))+(((fstarr[1][2]*Vpk[0][1][2])+((
      fstarr[1][0]*Vpk[0][1][0])+(fstarr[1][1]*Vpk[0][1][1])))+((tstarr[1][2]*
      Wpk[0][1][2])+((tstarr[1][0]*Wpk[0][1][0])+(tstarr[1][1]*Wpk[0][1][1])))))
      ;
    temp[1] = ((((fstarr[3][2]*Vpk[0][3][2])+((fstarr[3][0]*Vpk[0][3][0])+(
      fstarr[3][1]*Vpk[0][3][1])))+((tstarr[3][2]*Wpk[0][3][2])+((tstarr[3][0]*
      Wpk[0][3][0])+(tstarr[3][1]*Wpk[0][3][1]))))+((((fstarr[2][2]*Vpk[0][2][2]
      )+((fstarr[2][0]*Vpk[0][2][0])+(fstarr[2][1]*Vpk[0][2][1])))+((
      tstarr[2][2]*Wpk[0][2][2])+((tstarr[2][0]*Wpk[0][2][0])+(tstarr[2][1]*
      Wpk[0][2][1]))))+temp[0]));
    temp[2] = ((((fstarr[5][2]*Vpk[0][5][2])+((fstarr[5][0]*Vpk[0][5][0])+(
      fstarr[5][1]*Vpk[0][5][1])))+((tstarr[5][2]*Wpk[0][5][2])+((tstarr[5][0]*
      Wpk[0][5][0])+(tstarr[5][1]*Wpk[0][5][1]))))+((((fstarr[4][2]*Vpk[0][4][2]
      )+((fstarr[4][0]*Vpk[0][4][0])+(fstarr[4][1]*Vpk[0][4][1])))+((
      tstarr[4][2]*Wpk[0][4][2])+((tstarr[4][0]*Wpk[0][4][0])+(tstarr[4][1]*
      Wpk[0][4][1]))))+temp[1]));
    temp[3] = ((((fstarr[7][2]*Vpk[0][7][2])+((fstarr[7][0]*Vpk[0][7][0])+(
      fstarr[7][1]*Vpk[0][7][1])))+((tstarr[7][2]*Wpk[0][7][2])+((tstarr[7][0]*
      Wpk[0][7][0])+(tstarr[7][1]*Wpk[0][7][1]))))+((((fstarr[6][2]*Vpk[0][6][2]
      )+((fstarr[6][0]*Vpk[0][6][0])+(fstarr[6][1]*Vpk[0][6][1])))+((
      tstarr[6][2]*Wpk[0][6][2])+((tstarr[6][0]*Wpk[0][6][0])+(tstarr[6][1]*
      Wpk[0][6][1]))))+temp[2]));
    temp[4] = ((((fstarr[9][2]*Vpk[0][9][2])+((fstarr[9][0]*Vpk[0][9][0])+(
      fstarr[9][1]*Vpk[0][9][1])))+((tstarr[9][2]*Wpk[0][9][2])+((tstarr[9][0]*
      Wpk[0][9][0])+(tstarr[9][1]*Wpk[0][9][1]))))+((((fstarr[8][2]*Vpk[0][8][2]
      )+((fstarr[8][0]*Vpk[0][8][0])+(fstarr[8][1]*Vpk[0][8][1])))+((
      tstarr[8][2]*Wpk[0][8][2])+((tstarr[8][0]*Wpk[0][8][0])+(tstarr[8][1]*
      Wpk[0][8][1]))))+temp[3]));
    temp[5] = ((((fstarr[11][2]*Vpk[0][11][2])+((fstarr[11][0]*Vpk[0][11][0])+(
      fstarr[11][1]*Vpk[0][11][1])))+((tstarr[11][2]*Wpk[0][11][2])+((
      tstarr[11][0]*Wpk[0][11][0])+(tstarr[11][1]*Wpk[0][11][1]))))+((((
      fstarr[10][2]*Vpk[0][10][2])+((fstarr[10][0]*Vpk[0][10][0])+(fstarr[10][1]
      *Vpk[0][10][1])))+((tstarr[10][2]*Wpk[0][10][2])+((tstarr[10][0]*
      Wpk[0][10][0])+(tstarr[10][1]*Wpk[0][10][1]))))+temp[4]));
    trqout[0] = -((mtau[0]+utau[0])+((((fstarr[13][2]*Vpk[0][13][2])+((
      fstarr[13][0]*Vpk[0][13][0])+(fstarr[13][1]*Vpk[0][13][1])))+((
      tstarr[13][2]*Wpk[0][13][2])+((tstarr[13][0]*Wpk[0][13][0])+(tstarr[13][1]
      *Wpk[0][13][1]))))+((((fstarr[12][2]*Vpk[0][12][2])+((fstarr[12][0]*
      Vpk[0][12][0])+(fstarr[12][1]*Vpk[0][12][1])))+((tstarr[12][2]*
      Wpk[0][12][2])+((tstarr[12][0]*Wpk[0][12][0])+(tstarr[12][1]*Wpk[0][12][1]
      ))))+temp[5])));
    trqout[1] = -((mtau[1]+utau[1])+(((fstarr[1][2]*Vpk[1][1][2])+((fstarr[1][0]
      *Vpk[1][1][0])+(fstarr[1][1]*Vpk[1][1][1])))+((pin[1][2]*tstarr[1][2])+((
      pin[1][0]*tstarr[1][0])+(pin[1][1]*tstarr[1][1])))));
    temp[0] = ((((fstarr[2][2]*Vpk[2][2][2])+((fstarr[2][0]*Vpk[2][2][0])+(
      fstarr[2][1]*Vpk[2][2][1])))+((pin[2][2]*tstarr[2][2])+((pin[2][0]*
      tstarr[2][0])+(pin[2][1]*tstarr[2][1]))))+(((fstarr[3][2]*Vpk[2][3][2])+((
      fstarr[3][0]*Vpk[2][3][0])+(fstarr[3][1]*Vpk[2][3][1])))+((tstarr[3][2]*
      Wpk[2][3][2])+((tstarr[3][0]*Wpk[2][3][0])+(tstarr[3][1]*Wpk[2][3][1])))))
      ;
    temp[1] = ((((fstarr[5][2]*Vpk[2][5][2])+((fstarr[5][0]*Vpk[2][5][0])+(
      fstarr[5][1]*Vpk[2][5][1])))+((tstarr[5][2]*Wpk[2][5][2])+((tstarr[5][0]*
      Wpk[2][5][0])+(tstarr[5][1]*Wpk[2][5][1]))))+((((fstarr[4][2]*Vpk[2][4][2]
      )+((fstarr[4][0]*Vpk[2][4][0])+(fstarr[4][1]*Vpk[2][4][1])))+((
      tstarr[4][2]*Wpk[2][4][2])+((tstarr[4][0]*Wpk[2][4][0])+(tstarr[4][1]*
      Wpk[2][4][1]))))+temp[0]));
    trqout[2] = -((mtau[2]+utau[2])+((((fstarr[7][2]*Vpk[2][7][2])+((
      fstarr[7][0]*Vpk[2][7][0])+(fstarr[7][1]*Vpk[2][7][1])))+((tstarr[7][2]*
      Wpk[2][7][2])+((tstarr[7][0]*Wpk[2][7][0])+(tstarr[7][1]*Wpk[2][7][1]))))+
      ((((fstarr[6][2]*Vpk[2][6][2])+((fstarr[6][0]*Vpk[2][6][0])+(fstarr[6][1]*
      Vpk[2][6][1])))+((tstarr[6][2]*Wpk[2][6][2])+((tstarr[6][0]*Wpk[2][6][0])+
      (tstarr[6][1]*Wpk[2][6][1]))))+temp[1])));
    temp[0] = ((((fstarr[3][2]*Vpk[3][3][2])+((fstarr[3][0]*Vpk[3][3][0])+(
      fstarr[3][1]*Vpk[3][3][1])))+((pin[3][2]*tstarr[3][2])+((pin[3][0]*
      tstarr[3][0])+(pin[3][1]*tstarr[3][1]))))+(((fstarr[4][2]*Vpk[3][4][2])+((
      fstarr[4][0]*Vpk[3][4][0])+(fstarr[4][1]*Vpk[3][4][1])))+((tstarr[4][2]*
      Wpk[3][4][2])+((tstarr[4][0]*Wpk[3][4][0])+(tstarr[4][1]*Wpk[3][4][1])))))
      ;
    temp[1] = ((((fstarr[6][2]*Vpk[3][6][2])+((fstarr[6][0]*Vpk[3][6][0])+(
      fstarr[6][1]*Vpk[3][6][1])))+((tstarr[6][2]*Wpk[3][6][2])+((tstarr[6][0]*
      Wpk[3][6][0])+(tstarr[6][1]*Wpk[3][6][1]))))+((((fstarr[5][2]*Vpk[3][5][2]
      )+((fstarr[5][0]*Vpk[3][5][0])+(fstarr[5][1]*Vpk[3][5][1])))+((
      tstarr[5][2]*Wpk[3][5][2])+((tstarr[5][0]*Wpk[3][5][0])+(tstarr[5][1]*
      Wpk[3][5][1]))))+temp[0]));
    trqout[3] = -((mtau[3]+utau[3])+((((fstarr[7][2]*Vpk[3][7][2])+((
      fstarr[7][0]*Vpk[3][7][0])+(fstarr[7][1]*Vpk[3][7][1])))+((tstarr[7][2]*
      Wpk[3][7][2])+((tstarr[7][0]*Wpk[3][7][0])+(tstarr[7][1]*Wpk[3][7][1]))))+
      temp[1]));
    temp[0] = ((((fstarr[4][2]*Vpk[4][4][2])+((fstarr[4][0]*Vpk[4][4][0])+(
      fstarr[4][1]*Vpk[4][4][1])))+((pin[4][2]*tstarr[4][2])+((pin[4][0]*
      tstarr[4][0])+(pin[4][1]*tstarr[4][1]))))+(((fstarr[5][2]*Vpk[4][5][2])+((
      fstarr[5][0]*Vpk[4][5][0])+(fstarr[5][1]*Vpk[4][5][1])))+((tstarr[5][2]*
      Wpk[4][5][2])+((tstarr[5][0]*Wpk[4][5][0])+(tstarr[5][1]*Wpk[4][5][1])))))
      ;
    trqout[4] = -((mtau[4]+utau[4])+((((fstarr[7][2]*Vpk[4][7][2])+((
      fstarr[7][0]*Vpk[4][7][0])+(fstarr[7][1]*Vpk[4][7][1])))+((tstarr[7][2]*
      Wpk[4][7][2])+((tstarr[7][0]*Wpk[4][7][0])+(tstarr[7][1]*Wpk[4][7][1]))))+
      ((((fstarr[6][2]*Vpk[4][6][2])+((fstarr[6][0]*Vpk[4][6][0])+(fstarr[6][1]*
      Vpk[4][6][1])))+((tstarr[6][2]*Wpk[4][6][2])+((tstarr[6][0]*Wpk[4][6][0])+
      (tstarr[6][1]*Wpk[4][6][1]))))+temp[0])));
    temp[0] = ((((fstarr[5][2]*Vpk[5][5][2])+((fstarr[5][0]*Vpk[5][5][0])+(
      fstarr[5][1]*Vpk[5][5][1])))+((pin[5][2]*tstarr[5][2])+((pin[5][0]*
      tstarr[5][0])+(pin[5][1]*tstarr[5][1]))))+(((fstarr[6][2]*Vpk[5][6][2])+((
      fstarr[6][0]*Vpk[5][6][0])+(fstarr[6][1]*Vpk[5][6][1])))+((tstarr[6][2]*
      Wpk[5][6][2])+((tstarr[6][0]*Wpk[5][6][0])+(tstarr[6][1]*Wpk[5][6][1])))))
      ;
    trqout[5] = -((mtau[5]+utau[5])+((((fstarr[7][2]*Vpk[5][7][2])+((
      fstarr[7][0]*Vpk[5][7][0])+(fstarr[7][1]*Vpk[5][7][1])))+((tstarr[7][2]*
      Wpk[5][7][2])+((tstarr[7][0]*Wpk[5][7][0])+(tstarr[7][1]*Wpk[5][7][1]))))+
      temp[0]));
    trqout[6] = -((mtau[6]+utau[6])+((((fstarr[6][2]*Vpk[6][6][2])+((
      fstarr[6][0]*Vpk[6][6][0])+(fstarr[6][1]*Vpk[6][6][1])))+((pin[6][2]*
      tstarr[6][2])+((pin[6][0]*tstarr[6][0])+(pin[6][1]*tstarr[6][1]))))+(((
      fstarr[7][2]*Vpk[6][7][2])+((fstarr[7][0]*Vpk[6][7][0])+(fstarr[7][1]*
      Vpk[6][7][1])))+((tstarr[7][2]*Wpk[6][7][2])+((tstarr[7][0]*Wpk[6][7][0])+
      (tstarr[7][1]*Wpk[6][7][1]))))));
    trqout[7] = -((mtau[7]+utau[7])+(((fstarr[7][2]*Vpk[7][7][2])+((fstarr[7][0]
      *Vpk[7][7][0])+(fstarr[7][1]*Vpk[7][7][1])))+((pin[7][2]*tstarr[7][2])+((
      pin[7][0]*tstarr[7][0])+(pin[7][1]*tstarr[7][1])))));
    temp[0] = ((((fstarr[8][2]*Vpk[8][8][2])+((fstarr[8][0]*Vpk[8][8][0])+(
      fstarr[8][1]*Vpk[8][8][1])))+((pin[8][2]*tstarr[8][2])+((pin[8][0]*
      tstarr[8][0])+(pin[8][1]*tstarr[8][1]))))+(((fstarr[9][2]*Vpk[8][9][2])+((
      fstarr[9][0]*Vpk[8][9][0])+(fstarr[9][1]*Vpk[8][9][1])))+((tstarr[9][2]*
      Wpk[8][9][2])+((tstarr[9][0]*Wpk[8][9][0])+(tstarr[9][1]*Wpk[8][9][1])))))
      ;
    temp[1] = ((((fstarr[11][2]*Vpk[8][11][2])+((fstarr[11][0]*Vpk[8][11][0])+(
      fstarr[11][1]*Vpk[8][11][1])))+((tstarr[11][2]*Wpk[8][11][2])+((
      tstarr[11][0]*Wpk[8][11][0])+(tstarr[11][1]*Wpk[8][11][1]))))+((((
      fstarr[10][2]*Vpk[8][10][2])+((fstarr[10][0]*Vpk[8][10][0])+(fstarr[10][1]
      *Vpk[8][10][1])))+((tstarr[10][2]*Wpk[8][10][2])+((tstarr[10][0]*
      Wpk[8][10][0])+(tstarr[10][1]*Wpk[8][10][1]))))+temp[0]));
    trqout[8] = -((mtau[8]+utau[8])+((((fstarr[13][2]*Vpk[8][13][2])+((
      fstarr[13][0]*Vpk[8][13][0])+(fstarr[13][1]*Vpk[8][13][1])))+((
      tstarr[13][2]*Wpk[8][13][2])+((tstarr[13][0]*Wpk[8][13][0])+(tstarr[13][1]
      *Wpk[8][13][1]))))+((((fstarr[12][2]*Vpk[8][12][2])+((fstarr[12][0]*
      Vpk[8][12][0])+(fstarr[12][1]*Vpk[8][12][1])))+((tstarr[12][2]*
      Wpk[8][12][2])+((tstarr[12][0]*Wpk[8][12][0])+(tstarr[12][1]*Wpk[8][12][1]
      ))))+temp[1])));
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
    trqout[9] = -((mtau[9]+utau[9])+((((fstarr[13][2]*Vpk[9][13][2])+((
      fstarr[13][0]*Vpk[9][13][0])+(fstarr[13][1]*Vpk[9][13][1])))+((
      tstarr[13][2]*Wpk[9][13][2])+((tstarr[13][0]*Wpk[9][13][0])+(tstarr[13][1]
      *Wpk[9][13][1]))))+temp[1]));
    temp[0] = ((((fstarr[10][2]*Vpk[10][10][2])+((fstarr[10][0]*Vpk[10][10][0])+
      (fstarr[10][1]*Vpk[10][10][1])))+((pin[10][2]*tstarr[10][2])+((pin[10][0]*
      tstarr[10][0])+(pin[10][1]*tstarr[10][1]))))+(((fstarr[11][2]*
      Vpk[10][11][2])+((fstarr[11][0]*Vpk[10][11][0])+(fstarr[11][1]*
      Vpk[10][11][1])))+((tstarr[11][2]*Wpk[10][11][2])+((tstarr[11][0]*
      Wpk[10][11][0])+(tstarr[11][1]*Wpk[10][11][1])))));
    trqout[10] = -((mtau[10]+utau[10])+((((fstarr[13][2]*Vpk[10][13][2])+((
      fstarr[13][0]*Vpk[10][13][0])+(fstarr[13][1]*Vpk[10][13][1])))+((
      tstarr[13][2]*Wpk[10][13][2])+((tstarr[13][0]*Wpk[10][13][0])+(
      tstarr[13][1]*Wpk[10][13][1]))))+((((fstarr[12][2]*Vpk[10][12][2])+((
      fstarr[12][0]*Vpk[10][12][0])+(fstarr[12][1]*Vpk[10][12][1])))+((
      tstarr[12][2]*Wpk[10][12][2])+((tstarr[12][0]*Wpk[10][12][0])+(
      tstarr[12][1]*Wpk[10][12][1]))))+temp[0])));
    temp[0] = ((((fstarr[11][2]*Vpk[11][11][2])+((fstarr[11][0]*Vpk[11][11][0])+
      (fstarr[11][1]*Vpk[11][11][1])))+((pin[11][2]*tstarr[11][2])+((pin[11][0]*
      tstarr[11][0])+(pin[11][1]*tstarr[11][1]))))+(((fstarr[12][2]*
      Vpk[11][12][2])+((fstarr[12][0]*Vpk[11][12][0])+(fstarr[12][1]*
      Vpk[11][12][1])))+((tstarr[12][2]*Wpk[11][12][2])+((tstarr[12][0]*
      Wpk[11][12][0])+(tstarr[12][1]*Wpk[11][12][1])))));
    trqout[11] = -((mtau[11]+utau[11])+((((fstarr[13][2]*Vpk[11][13][2])+((
      fstarr[13][0]*Vpk[11][13][0])+(fstarr[13][1]*Vpk[11][13][1])))+((
      tstarr[13][2]*Wpk[11][13][2])+((tstarr[13][0]*Wpk[11][13][0])+(
      tstarr[13][1]*Wpk[11][13][1]))))+temp[0]));
    trqout[12] = -((mtau[12]+utau[12])+((((fstarr[12][2]*Vpk[12][12][2])+((
      fstarr[12][0]*Vpk[12][12][0])+(fstarr[12][1]*Vpk[12][12][1])))+((
      pin[12][2]*tstarr[12][2])+((pin[12][0]*tstarr[12][0])+(pin[12][1]*
      tstarr[12][1]))))+(((fstarr[13][2]*Vpk[12][13][2])+((fstarr[13][0]*
      Vpk[12][13][0])+(fstarr[13][1]*Vpk[12][13][1])))+((tstarr[13][2]*
      Wpk[12][13][2])+((tstarr[13][0]*Wpk[12][13][0])+(tstarr[13][1]*
      Wpk[12][13][1]))))));
    trqout[13] = -((mtau[13]+utau[13])+(((fstarr[13][2]*Vpk[13][13][2])+((
      fstarr[13][0]*Vpk[13][13][0])+(fstarr[13][1]*Vpk[13][13][1])))+((
      pin[13][2]*tstarr[13][2])+((pin[13][0]*tstarr[13][0])+(pin[13][1]*
      tstarr[13][1])))));
/*
Op counts below do not include called subroutines
*/
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain 1216 adds/subtracts/negates
                   1188 multiplies
                      0 divides
                    239 assignments
*/
}

void sdcomptrq(double udotin[14],
    double trqout[14])
{
/* Compute hinge torques to produce these udots, ignoring constraints
*/
    int i;
    double multin[14];

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(60,23);
        return;
    }
    for (i = 0; i < 14; i++) {
        multin[i] = 0.;
    }
    sdfulltrq(udotin,multin,trqout);
}

void sdmulttrq(double multin[14],
    double trqout[14])
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
    for (i = 0; i < 14; i++) {
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
    sddoiner();
/*
Compute onk & onb (angular accels in N)
*/
    Onkb[0][0] = (pin[0][0]*udot[0]);
    Onkb[0][1] = (pin[0][1]*udot[0]);
    Onkb[0][2] = (pin[0][2]*udot[0]);
    Onkb[1][0] = ((pin[1][0]*udot[1])+((Cik[1][2][0]*Onkb[0][2])+((Cik[1][0][0]*
      Onkb[0][0])+(Cik[1][1][0]*Onkb[0][1]))));
    Onkb[1][1] = ((pin[1][1]*udot[1])+((Cik[1][2][1]*Onkb[0][2])+((Cik[1][0][1]*
      Onkb[0][0])+(Cik[1][1][1]*Onkb[0][1]))));
    Onkb[1][2] = ((pin[1][2]*udot[1])+((Cik[1][2][2]*Onkb[0][2])+((Cik[1][0][2]*
      Onkb[0][0])+(Cik[1][1][2]*Onkb[0][1]))));
    Onkb[2][0] = ((pin[2][0]*udot[2])+((Cik[2][2][0]*Onkb[0][2])+((Cik[2][0][0]*
      Onkb[0][0])+(Cik[2][1][0]*Onkb[0][1]))));
    Onkb[2][1] = ((pin[2][1]*udot[2])+((Cik[2][2][1]*Onkb[0][2])+((Cik[2][0][1]*
      Onkb[0][0])+(Cik[2][1][1]*Onkb[0][1]))));
    Onkb[2][2] = ((pin[2][2]*udot[2])+((Cik[2][2][2]*Onkb[0][2])+((Cik[2][0][2]*
      Onkb[0][0])+(Cik[2][1][2]*Onkb[0][1]))));
    Onkb[3][0] = ((pin[3][0]*udot[3])+((Cik[3][2][0]*Onkb[2][2])+((Cik[3][0][0]*
      Onkb[2][0])+(Cik[3][1][0]*Onkb[2][1]))));
    Onkb[3][1] = ((pin[3][1]*udot[3])+((Cik[3][2][1]*Onkb[2][2])+((Cik[3][0][1]*
      Onkb[2][0])+(Cik[3][1][1]*Onkb[2][1]))));
    Onkb[3][2] = ((pin[3][2]*udot[3])+((Cik[3][2][2]*Onkb[2][2])+((Cik[3][0][2]*
      Onkb[2][0])+(Cik[3][1][2]*Onkb[2][1]))));
    Onkb[4][0] = ((pin[4][0]*udot[4])+((Cik[4][2][0]*Onkb[3][2])+((Cik[4][0][0]*
      Onkb[3][0])+(Cik[4][1][0]*Onkb[3][1]))));
    Onkb[4][1] = ((pin[4][1]*udot[4])+((Cik[4][2][1]*Onkb[3][2])+((Cik[4][0][1]*
      Onkb[3][0])+(Cik[4][1][1]*Onkb[3][1]))));
    Onkb[4][2] = ((pin[4][2]*udot[4])+((Cik[4][2][2]*Onkb[3][2])+((Cik[4][0][2]*
      Onkb[3][0])+(Cik[4][1][2]*Onkb[3][1]))));
    Onkb[5][0] = ((pin[5][0]*udot[5])+((Cik[5][2][0]*Onkb[4][2])+((Cik[5][0][0]*
      Onkb[4][0])+(Cik[5][1][0]*Onkb[4][1]))));
    Onkb[5][1] = ((pin[5][1]*udot[5])+((Cik[5][2][1]*Onkb[4][2])+((Cik[5][0][1]*
      Onkb[4][0])+(Cik[5][1][1]*Onkb[4][1]))));
    Onkb[5][2] = ((pin[5][2]*udot[5])+((Cik[5][2][2]*Onkb[4][2])+((Cik[5][0][2]*
      Onkb[4][0])+(Cik[5][1][2]*Onkb[4][1]))));
    Onkb[6][0] = ((pin[6][0]*udot[6])+((Cik[6][2][0]*Onkb[5][2])+((Cik[6][0][0]*
      Onkb[5][0])+(Cik[6][1][0]*Onkb[5][1]))));
    Onkb[6][1] = ((pin[6][1]*udot[6])+((Cik[6][2][1]*Onkb[5][2])+((Cik[6][0][1]*
      Onkb[5][0])+(Cik[6][1][1]*Onkb[5][1]))));
    Onkb[6][2] = ((pin[6][2]*udot[6])+((Cik[6][2][2]*Onkb[5][2])+((Cik[6][0][2]*
      Onkb[5][0])+(Cik[6][1][2]*Onkb[5][1]))));
    Onkb[7][0] = ((pin[7][0]*udot[7])+((Cik[7][2][0]*Onkb[6][2])+((Cik[7][0][0]*
      Onkb[6][0])+(Cik[7][1][0]*Onkb[6][1]))));
    Onkb[7][1] = ((pin[7][1]*udot[7])+((Cik[7][2][1]*Onkb[6][2])+((Cik[7][0][1]*
      Onkb[6][0])+(Cik[7][1][1]*Onkb[6][1]))));
    Onkb[7][2] = ((pin[7][2]*udot[7])+((Cik[7][2][2]*Onkb[6][2])+((Cik[7][0][2]*
      Onkb[6][0])+(Cik[7][1][2]*Onkb[6][1]))));
    Onkb[8][0] = ((pin[8][0]*udot[8])+((Cik[8][2][0]*Onkb[0][2])+((Cik[8][0][0]*
      Onkb[0][0])+(Cik[8][1][0]*Onkb[0][1]))));
    Onkb[8][1] = ((pin[8][1]*udot[8])+((Cik[8][2][1]*Onkb[0][2])+((Cik[8][0][1]*
      Onkb[0][0])+(Cik[8][1][1]*Onkb[0][1]))));
    Onkb[8][2] = ((pin[8][2]*udot[8])+((Cik[8][2][2]*Onkb[0][2])+((Cik[8][0][2]*
      Onkb[0][0])+(Cik[8][1][2]*Onkb[0][1]))));
    Onkb[9][0] = ((pin[9][0]*udot[9])+((Cik[9][2][0]*Onkb[8][2])+((Cik[9][0][0]*
      Onkb[8][0])+(Cik[9][1][0]*Onkb[8][1]))));
    Onkb[9][1] = ((pin[9][1]*udot[9])+((Cik[9][2][1]*Onkb[8][2])+((Cik[9][0][1]*
      Onkb[8][0])+(Cik[9][1][1]*Onkb[8][1]))));
    Onkb[9][2] = ((pin[9][2]*udot[9])+((Cik[9][2][2]*Onkb[8][2])+((Cik[9][0][2]*
      Onkb[8][0])+(Cik[9][1][2]*Onkb[8][1]))));
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
    onk[1][0] = (Onkb[1][0]+Otk[1][0]);
    onk[1][1] = (Onkb[1][1]+Otk[1][1]);
    onk[1][2] = (Onkb[1][2]+Otk[1][2]);
    onk[2][0] = (Onkb[2][0]+Otk[2][0]);
    onk[2][1] = (Onkb[2][1]+Otk[2][1]);
    onk[2][2] = (Onkb[2][2]+Otk[2][2]);
    onk[3][0] = (Onkb[3][0]+Otk[3][0]);
    onk[3][1] = (Onkb[3][1]+Otk[3][1]);
    onk[3][2] = (Onkb[3][2]+Otk[3][2]);
    onk[4][0] = (Onkb[4][0]+Otk[4][0]);
    onk[4][1] = (Onkb[4][1]+Otk[4][1]);
    onk[4][2] = (Onkb[4][2]+Otk[4][2]);
    onk[5][0] = (Onkb[5][0]+Otk[5][0]);
    onk[5][1] = (Onkb[5][1]+Otk[5][1]);
    onk[5][2] = (Onkb[5][2]+Otk[5][2]);
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
    onb[0][0] = Onkb[0][0];
    onb[0][1] = Onkb[0][1];
    onb[0][2] = Onkb[0][2];
    onb[1][0] = onk[1][0];
    onb[1][1] = onk[1][1];
    onb[1][2] = onk[1][2];
    onb[2][0] = onk[2][0];
    onb[2][1] = onk[2][1];
    onb[2][2] = onk[2][2];
    onb[3][0] = onk[3][0];
    onb[3][1] = onk[3][1];
    onb[3][2] = onk[3][2];
    onb[4][0] = onk[4][0];
    onb[4][1] = onk[4][1];
    onb[4][2] = onk[4][2];
    onb[5][0] = onk[5][0];
    onb[5][1] = onk[5][1];
    onb[5][2] = onk[5][2];
    onb[6][0] = onk[6][0];
    onb[6][1] = onk[6][1];
    onb[6][2] = onk[6][2];
    onb[7][0] = onk[7][0];
    onb[7][1] = onk[7][1];
    onb[7][2] = onk[7][2];
    onb[8][0] = onk[8][0];
    onb[8][1] = onk[8][1];
    onb[8][2] = onk[8][2];
    onb[9][0] = onk[9][0];
    onb[9][1] = onk[9][1];
    onb[9][2] = onk[9][2];
    onb[10][0] = onk[10][0];
    onb[10][1] = onk[10][1];
    onb[10][2] = onk[10][2];
    onb[11][0] = onk[11][0];
    onb[11][1] = onk[11][1];
    onb[11][2] = onk[11][2];
    onb[12][0] = onk[12][0];
    onb[12][1] = onk[12][1];
    onb[12][2] = onk[12][2];
    onb[13][0] = onk[13][0];
    onb[13][1] = onk[13][1];
    onb[13][2] = onk[13][2];
/*
Compute acceleration dyadics
*/
    dyad[0][0][0] = w11w22[0];
    dyad[0][0][1] = (w0w1[0]-Onkb[0][2]);
    dyad[0][0][2] = (Onkb[0][1]+w0w2[0]);
    dyad[0][1][0] = (Onkb[0][2]+w0w1[0]);
    dyad[0][1][1] = w00w22[0];
    dyad[0][1][2] = (w1w2[0]-Onkb[0][0]);
    dyad[0][2][0] = (w0w2[0]-Onkb[0][1]);
    dyad[0][2][1] = (Onkb[0][0]+w1w2[0]);
    dyad[0][2][2] = w00w11[0];
    dyad[1][0][0] = w11w22[1];
    dyad[1][0][1] = (w0w1[1]-onk[1][2]);
    dyad[1][0][2] = (onk[1][1]+w0w2[1]);
    dyad[1][1][0] = (onk[1][2]+w0w1[1]);
    dyad[1][1][1] = w00w22[1];
    dyad[1][1][2] = (w1w2[1]-onk[1][0]);
    dyad[1][2][0] = (w0w2[1]-onk[1][1]);
    dyad[1][2][1] = (onk[1][0]+w1w2[1]);
    dyad[1][2][2] = w00w11[1];
    dyad[2][0][0] = w11w22[2];
    dyad[2][0][1] = (w0w1[2]-onk[2][2]);
    dyad[2][0][2] = (onk[2][1]+w0w2[2]);
    dyad[2][1][0] = (onk[2][2]+w0w1[2]);
    dyad[2][1][1] = w00w22[2];
    dyad[2][1][2] = (w1w2[2]-onk[2][0]);
    dyad[2][2][0] = (w0w2[2]-onk[2][1]);
    dyad[2][2][1] = (onk[2][0]+w1w2[2]);
    dyad[2][2][2] = w00w11[2];
    dyad[3][0][0] = w11w22[3];
    dyad[3][0][1] = (w0w1[3]-onk[3][2]);
    dyad[3][0][2] = (onk[3][1]+w0w2[3]);
    dyad[3][1][0] = (onk[3][2]+w0w1[3]);
    dyad[3][1][1] = w00w22[3];
    dyad[3][1][2] = (w1w2[3]-onk[3][0]);
    dyad[3][2][0] = (w0w2[3]-onk[3][1]);
    dyad[3][2][1] = (onk[3][0]+w1w2[3]);
    dyad[3][2][2] = w00w11[3];
    dyad[4][0][0] = w11w22[4];
    dyad[4][0][1] = (w0w1[4]-onk[4][2]);
    dyad[4][0][2] = (onk[4][1]+w0w2[4]);
    dyad[4][1][0] = (onk[4][2]+w0w1[4]);
    dyad[4][1][1] = w00w22[4];
    dyad[4][1][2] = (w1w2[4]-onk[4][0]);
    dyad[4][2][0] = (w0w2[4]-onk[4][1]);
    dyad[4][2][1] = (onk[4][0]+w1w2[4]);
    dyad[4][2][2] = w00w11[4];
    dyad[5][0][0] = w11w22[5];
    dyad[5][0][1] = (w0w1[5]-onk[5][2]);
    dyad[5][0][2] = (onk[5][1]+w0w2[5]);
    dyad[5][1][0] = (onk[5][2]+w0w1[5]);
    dyad[5][1][1] = w00w22[5];
    dyad[5][1][2] = (w1w2[5]-onk[5][0]);
    dyad[5][2][0] = (w0w2[5]-onk[5][1]);
    dyad[5][2][1] = (onk[5][0]+w1w2[5]);
    dyad[5][2][2] = w00w11[5];
    dyad[6][0][0] = w11w22[6];
    dyad[6][0][1] = (w0w1[6]-onk[6][2]);
    dyad[6][0][2] = (onk[6][1]+w0w2[6]);
    dyad[6][1][0] = (onk[6][2]+w0w1[6]);
    dyad[6][1][1] = w00w22[6];
    dyad[6][1][2] = (w1w2[6]-onk[6][0]);
    dyad[6][2][0] = (w0w2[6]-onk[6][1]);
    dyad[6][2][1] = (onk[6][0]+w1w2[6]);
    dyad[6][2][2] = w00w11[6];
    dyad[7][0][0] = w11w22[7];
    dyad[7][0][1] = (w0w1[7]-onk[7][2]);
    dyad[7][0][2] = (onk[7][1]+w0w2[7]);
    dyad[7][1][0] = (onk[7][2]+w0w1[7]);
    dyad[7][1][1] = w00w22[7];
    dyad[7][1][2] = (w1w2[7]-onk[7][0]);
    dyad[7][2][0] = (w0w2[7]-onk[7][1]);
    dyad[7][2][1] = (onk[7][0]+w1w2[7]);
    dyad[7][2][2] = w00w11[7];
    dyad[8][0][0] = w11w22[8];
    dyad[8][0][1] = (w0w1[8]-onk[8][2]);
    dyad[8][0][2] = (onk[8][1]+w0w2[8]);
    dyad[8][1][0] = (onk[8][2]+w0w1[8]);
    dyad[8][1][1] = w00w22[8];
    dyad[8][1][2] = (w1w2[8]-onk[8][0]);
    dyad[8][2][0] = (w0w2[8]-onk[8][1]);
    dyad[8][2][1] = (onk[8][0]+w1w2[8]);
    dyad[8][2][2] = w00w11[8];
    dyad[9][0][0] = w11w22[9];
    dyad[9][0][1] = (w0w1[9]-onk[9][2]);
    dyad[9][0][2] = (onk[9][1]+w0w2[9]);
    dyad[9][1][0] = (onk[9][2]+w0w1[9]);
    dyad[9][1][1] = w00w22[9];
    dyad[9][1][2] = (w1w2[9]-onk[9][0]);
    dyad[9][2][0] = (w0w2[9]-onk[9][1]);
    dyad[9][2][1] = (onk[9][0]+w1w2[9]);
    dyad[9][2][2] = w00w11[9];
    dyad[10][0][0] = w11w22[10];
    dyad[10][0][1] = (w0w1[10]-onk[10][2]);
    dyad[10][0][2] = (onk[10][1]+w0w2[10]);
    dyad[10][1][0] = (onk[10][2]+w0w1[10]);
    dyad[10][1][1] = w00w22[10];
    dyad[10][1][2] = (w1w2[10]-onk[10][0]);
    dyad[10][2][0] = (w0w2[10]-onk[10][1]);
    dyad[10][2][1] = (onk[10][0]+w1w2[10]);
    dyad[10][2][2] = w00w11[10];
    dyad[11][0][0] = w11w22[11];
    dyad[11][0][1] = (w0w1[11]-onk[11][2]);
    dyad[11][0][2] = (onk[11][1]+w0w2[11]);
    dyad[11][1][0] = (onk[11][2]+w0w1[11]);
    dyad[11][1][1] = w00w22[11];
    dyad[11][1][2] = (w1w2[11]-onk[11][0]);
    dyad[11][2][0] = (w0w2[11]-onk[11][1]);
    dyad[11][2][1] = (onk[11][0]+w1w2[11]);
    dyad[11][2][2] = w00w11[11];
    dyad[12][0][0] = w11w22[12];
    dyad[12][0][1] = (w0w1[12]-onk[12][2]);
    dyad[12][0][2] = (onk[12][1]+w0w2[12]);
    dyad[12][1][0] = (onk[12][2]+w0w1[12]);
    dyad[12][1][1] = w00w22[12];
    dyad[12][1][2] = (w1w2[12]-onk[12][0]);
    dyad[12][2][0] = (w0w2[12]-onk[12][1]);
    dyad[12][2][1] = (onk[12][0]+w1w2[12]);
    dyad[12][2][2] = w00w11[12];
    dyad[13][0][0] = w11w22[13];
    dyad[13][0][1] = (w0w1[13]-onk[13][2]);
    dyad[13][0][2] = (onk[13][1]+w0w2[13]);
    dyad[13][1][0] = (onk[13][2]+w0w1[13]);
    dyad[13][1][1] = w00w22[13];
    dyad[13][1][2] = (w1w2[13]-onk[13][0]);
    dyad[13][2][0] = (w0w2[13]-onk[13][1]);
    dyad[13][2][1] = (onk[13][0]+w1w2[13]);
    dyad[13][2][2] = w00w11[13];
/*
Compute ank & anb (mass center linear accels in N)
*/
    Ankb[0][0] = ((Onkb[0][2]*rk[0][1])-(Onkb[0][1]*rk[0][2]));
    Ankb[0][1] = ((Onkb[0][0]*rk[0][2])-(Onkb[0][2]*rk[0][0]));
    Ankb[0][2] = ((Onkb[0][1]*rk[0][0])-(Onkb[0][0]*rk[0][1]));
    AOnkri[1][0] = (Ankb[0][0]+((Onkb[0][1]*ri[1][2])-(Onkb[0][2]*ri[1][1])));
    AOnkri[1][1] = (Ankb[0][1]+((Onkb[0][2]*ri[1][0])-(Onkb[0][0]*ri[1][2])));
    AOnkri[1][2] = (Ankb[0][2]+((Onkb[0][0]*ri[1][1])-(Onkb[0][1]*ri[1][0])));
    Ankb[1][0] = (((AOnkri[1][2]*Cik[1][2][0])+((AOnkri[1][0]*Cik[1][0][0])+(
      AOnkri[1][1]*Cik[1][1][0])))+((Onkb[1][2]*rk[1][1])-(Onkb[1][1]*rk[1][2]))
      );
    Ankb[1][1] = (((AOnkri[1][2]*Cik[1][2][1])+((AOnkri[1][0]*Cik[1][0][1])+(
      AOnkri[1][1]*Cik[1][1][1])))+((Onkb[1][0]*rk[1][2])-(Onkb[1][2]*rk[1][0]))
      );
    Ankb[1][2] = (((AOnkri[1][2]*Cik[1][2][2])+((AOnkri[1][0]*Cik[1][0][2])+(
      AOnkri[1][1]*Cik[1][1][2])))+((Onkb[1][1]*rk[1][0])-(Onkb[1][0]*rk[1][1]))
      );
    AOnkri[2][0] = (Ankb[0][0]+((Onkb[0][1]*ri[2][2])-(Onkb[0][2]*ri[2][1])));
    AOnkri[2][1] = (Ankb[0][1]+((Onkb[0][2]*ri[2][0])-(Onkb[0][0]*ri[2][2])));
    AOnkri[2][2] = (Ankb[0][2]+((Onkb[0][0]*ri[2][1])-(Onkb[0][1]*ri[2][0])));
    Ankb[2][0] = (((AOnkri[2][2]*Cik[2][2][0])+((AOnkri[2][0]*Cik[2][0][0])+(
      AOnkri[2][1]*Cik[2][1][0])))+((Onkb[2][2]*rk[2][1])-(Onkb[2][1]*rk[2][2]))
      );
    Ankb[2][1] = (((AOnkri[2][2]*Cik[2][2][1])+((AOnkri[2][0]*Cik[2][0][1])+(
      AOnkri[2][1]*Cik[2][1][1])))+((Onkb[2][0]*rk[2][2])-(Onkb[2][2]*rk[2][0]))
      );
    Ankb[2][2] = (((AOnkri[2][2]*Cik[2][2][2])+((AOnkri[2][0]*Cik[2][0][2])+(
      AOnkri[2][1]*Cik[2][1][2])))+((Onkb[2][1]*rk[2][0])-(Onkb[2][0]*rk[2][1]))
      );
    AOnkri[3][0] = (Ankb[2][0]+((Onkb[2][1]*ri[3][2])-(Onkb[2][2]*ri[3][1])));
    AOnkri[3][1] = (Ankb[2][1]+((Onkb[2][2]*ri[3][0])-(Onkb[2][0]*ri[3][2])));
    AOnkri[3][2] = (Ankb[2][2]+((Onkb[2][0]*ri[3][1])-(Onkb[2][1]*ri[3][0])));
    Ankb[3][0] = (((AOnkri[3][2]*Cik[3][2][0])+((AOnkri[3][0]*Cik[3][0][0])+(
      AOnkri[3][1]*Cik[3][1][0])))+((Onkb[3][2]*rk[3][1])-(Onkb[3][1]*rk[3][2]))
      );
    Ankb[3][1] = (((AOnkri[3][2]*Cik[3][2][1])+((AOnkri[3][0]*Cik[3][0][1])+(
      AOnkri[3][1]*Cik[3][1][1])))+((Onkb[3][0]*rk[3][2])-(Onkb[3][2]*rk[3][0]))
      );
    Ankb[3][2] = (((AOnkri[3][2]*Cik[3][2][2])+((AOnkri[3][0]*Cik[3][0][2])+(
      AOnkri[3][1]*Cik[3][1][2])))+((Onkb[3][1]*rk[3][0])-(Onkb[3][0]*rk[3][1]))
      );
    AOnkri[4][0] = (Ankb[3][0]+((Onkb[3][1]*ri[4][2])-(Onkb[3][2]*ri[4][1])));
    AOnkri[4][1] = (Ankb[3][1]+((Onkb[3][2]*ri[4][0])-(Onkb[3][0]*ri[4][2])));
    AOnkri[4][2] = (Ankb[3][2]+((Onkb[3][0]*ri[4][1])-(Onkb[3][1]*ri[4][0])));
    Ankb[4][0] = (((AOnkri[4][2]*Cik[4][2][0])+((AOnkri[4][0]*Cik[4][0][0])+(
      AOnkri[4][1]*Cik[4][1][0])))+((Onkb[4][2]*rk[4][1])-(Onkb[4][1]*rk[4][2]))
      );
    Ankb[4][1] = (((AOnkri[4][2]*Cik[4][2][1])+((AOnkri[4][0]*Cik[4][0][1])+(
      AOnkri[4][1]*Cik[4][1][1])))+((Onkb[4][0]*rk[4][2])-(Onkb[4][2]*rk[4][0]))
      );
    Ankb[4][2] = (((AOnkri[4][2]*Cik[4][2][2])+((AOnkri[4][0]*Cik[4][0][2])+(
      AOnkri[4][1]*Cik[4][1][2])))+((Onkb[4][1]*rk[4][0])-(Onkb[4][0]*rk[4][1]))
      );
    AOnkri[5][0] = (Ankb[4][0]+((Onkb[4][1]*ri[5][2])-(Onkb[4][2]*ri[5][1])));
    AOnkri[5][1] = (Ankb[4][1]+((Onkb[4][2]*ri[5][0])-(Onkb[4][0]*ri[5][2])));
    AOnkri[5][2] = (Ankb[4][2]+((Onkb[4][0]*ri[5][1])-(Onkb[4][1]*ri[5][0])));
    Ankb[5][0] = (((AOnkri[5][2]*Cik[5][2][0])+((AOnkri[5][0]*Cik[5][0][0])+(
      AOnkri[5][1]*Cik[5][1][0])))+((Onkb[5][2]*rk[5][1])-(Onkb[5][1]*rk[5][2]))
      );
    Ankb[5][1] = (((AOnkri[5][2]*Cik[5][2][1])+((AOnkri[5][0]*Cik[5][0][1])+(
      AOnkri[5][1]*Cik[5][1][1])))+((Onkb[5][0]*rk[5][2])-(Onkb[5][2]*rk[5][0]))
      );
    Ankb[5][2] = (((AOnkri[5][2]*Cik[5][2][2])+((AOnkri[5][0]*Cik[5][0][2])+(
      AOnkri[5][1]*Cik[5][1][2])))+((Onkb[5][1]*rk[5][0])-(Onkb[5][0]*rk[5][1]))
      );
    AOnkri[6][0] = (Ankb[5][0]+((Onkb[5][1]*ri[6][2])-(Onkb[5][2]*ri[6][1])));
    AOnkri[6][1] = (Ankb[5][1]+((Onkb[5][2]*ri[6][0])-(Onkb[5][0]*ri[6][2])));
    AOnkri[6][2] = (Ankb[5][2]+((Onkb[5][0]*ri[6][1])-(Onkb[5][1]*ri[6][0])));
    Ankb[6][0] = (((AOnkri[6][2]*Cik[6][2][0])+((AOnkri[6][0]*Cik[6][0][0])+(
      AOnkri[6][1]*Cik[6][1][0])))+((Onkb[6][2]*rk[6][1])-(Onkb[6][1]*rk[6][2]))
      );
    Ankb[6][1] = (((AOnkri[6][2]*Cik[6][2][1])+((AOnkri[6][0]*Cik[6][0][1])+(
      AOnkri[6][1]*Cik[6][1][1])))+((Onkb[6][0]*rk[6][2])-(Onkb[6][2]*rk[6][0]))
      );
    Ankb[6][2] = (((AOnkri[6][2]*Cik[6][2][2])+((AOnkri[6][0]*Cik[6][0][2])+(
      AOnkri[6][1]*Cik[6][1][2])))+((Onkb[6][1]*rk[6][0])-(Onkb[6][0]*rk[6][1]))
      );
    AOnkri[7][0] = (Ankb[6][0]+((Onkb[6][1]*ri[7][2])-(Onkb[6][2]*ri[7][1])));
    AOnkri[7][1] = (Ankb[6][1]+((Onkb[6][2]*ri[7][0])-(Onkb[6][0]*ri[7][2])));
    AOnkri[7][2] = (Ankb[6][2]+((Onkb[6][0]*ri[7][1])-(Onkb[6][1]*ri[7][0])));
    Ankb[7][0] = (((AOnkri[7][2]*Cik[7][2][0])+((AOnkri[7][0]*Cik[7][0][0])+(
      AOnkri[7][1]*Cik[7][1][0])))+((Onkb[7][2]*rk[7][1])-(Onkb[7][1]*rk[7][2]))
      );
    Ankb[7][1] = (((AOnkri[7][2]*Cik[7][2][1])+((AOnkri[7][0]*Cik[7][0][1])+(
      AOnkri[7][1]*Cik[7][1][1])))+((Onkb[7][0]*rk[7][2])-(Onkb[7][2]*rk[7][0]))
      );
    Ankb[7][2] = (((AOnkri[7][2]*Cik[7][2][2])+((AOnkri[7][0]*Cik[7][0][2])+(
      AOnkri[7][1]*Cik[7][1][2])))+((Onkb[7][1]*rk[7][0])-(Onkb[7][0]*rk[7][1]))
      );
    AOnkri[8][0] = (Ankb[0][0]+((Onkb[0][1]*ri[8][2])-(Onkb[0][2]*ri[8][1])));
    AOnkri[8][1] = (Ankb[0][1]+((Onkb[0][2]*ri[8][0])-(Onkb[0][0]*ri[8][2])));
    AOnkri[8][2] = (Ankb[0][2]+((Onkb[0][0]*ri[8][1])-(Onkb[0][1]*ri[8][0])));
    Ankb[8][0] = (((AOnkri[8][2]*Cik[8][2][0])+((AOnkri[8][0]*Cik[8][0][0])+(
      AOnkri[8][1]*Cik[8][1][0])))+((Onkb[8][2]*rk[8][1])-(Onkb[8][1]*rk[8][2]))
      );
    Ankb[8][1] = (((AOnkri[8][2]*Cik[8][2][1])+((AOnkri[8][0]*Cik[8][0][1])+(
      AOnkri[8][1]*Cik[8][1][1])))+((Onkb[8][0]*rk[8][2])-(Onkb[8][2]*rk[8][0]))
      );
    Ankb[8][2] = (((AOnkri[8][2]*Cik[8][2][2])+((AOnkri[8][0]*Cik[8][0][2])+(
      AOnkri[8][1]*Cik[8][1][2])))+((Onkb[8][1]*rk[8][0])-(Onkb[8][0]*rk[8][1]))
      );
    AOnkri[9][0] = (Ankb[8][0]+((Onkb[8][1]*ri[9][2])-(Onkb[8][2]*ri[9][1])));
    AOnkri[9][1] = (Ankb[8][1]+((Onkb[8][2]*ri[9][0])-(Onkb[8][0]*ri[9][2])));
    AOnkri[9][2] = (Ankb[8][2]+((Onkb[8][0]*ri[9][1])-(Onkb[8][1]*ri[9][0])));
    Ankb[9][0] = (((AOnkri[9][2]*Cik[9][2][0])+((AOnkri[9][0]*Cik[9][0][0])+(
      AOnkri[9][1]*Cik[9][1][0])))+((Onkb[9][2]*rk[9][1])-(Onkb[9][1]*rk[9][2]))
      );
    Ankb[9][1] = (((AOnkri[9][2]*Cik[9][2][1])+((AOnkri[9][0]*Cik[9][0][1])+(
      AOnkri[9][1]*Cik[9][1][1])))+((Onkb[9][0]*rk[9][2])-(Onkb[9][2]*rk[9][0]))
      );
    Ankb[9][2] = (((AOnkri[9][2]*Cik[9][2][2])+((AOnkri[9][0]*Cik[9][0][2])+(
      AOnkri[9][1]*Cik[9][1][2])))+((Onkb[9][1]*rk[9][0])-(Onkb[9][0]*rk[9][1]))
      );
    AOnkri[10][0] = (Ankb[9][0]+((Onkb[9][1]*ri[10][2])-(Onkb[9][2]*ri[10][1])))
      ;
    AOnkri[10][1] = (Ankb[9][1]+((Onkb[9][2]*ri[10][0])-(Onkb[9][0]*ri[10][2])))
      ;
    AOnkri[10][2] = (Ankb[9][2]+((Onkb[9][0]*ri[10][1])-(Onkb[9][1]*ri[10][0])))
      ;
    Ankb[10][0] = (((AOnkri[10][2]*Cik[10][2][0])+((AOnkri[10][0]*Cik[10][0][0])
      +(AOnkri[10][1]*Cik[10][1][0])))+((Onkb[10][2]*rk[10][1])-(Onkb[10][1]*
      rk[10][2])));
    Ankb[10][1] = (((AOnkri[10][2]*Cik[10][2][1])+((AOnkri[10][0]*Cik[10][0][1])
      +(AOnkri[10][1]*Cik[10][1][1])))+((Onkb[10][0]*rk[10][2])-(Onkb[10][2]*
      rk[10][0])));
    Ankb[10][2] = (((AOnkri[10][2]*Cik[10][2][2])+((AOnkri[10][0]*Cik[10][0][2])
      +(AOnkri[10][1]*Cik[10][1][2])))+((Onkb[10][1]*rk[10][0])-(Onkb[10][0]*
      rk[10][1])));
    AOnkri[11][0] = (Ankb[10][0]+((Onkb[10][1]*ri[11][2])-(Onkb[10][2]*ri[11][1]
      )));
    AOnkri[11][1] = (Ankb[10][1]+((Onkb[10][2]*ri[11][0])-(Onkb[10][0]*ri[11][2]
      )));
    AOnkri[11][2] = (Ankb[10][2]+((Onkb[10][0]*ri[11][1])-(Onkb[10][1]*ri[11][0]
      )));
    Ankb[11][0] = (((AOnkri[11][2]*Cik[11][2][0])+((AOnkri[11][0]*Cik[11][0][0])
      +(AOnkri[11][1]*Cik[11][1][0])))+((Onkb[11][2]*rk[11][1])-(Onkb[11][1]*
      rk[11][2])));
    Ankb[11][1] = (((AOnkri[11][2]*Cik[11][2][1])+((AOnkri[11][0]*Cik[11][0][1])
      +(AOnkri[11][1]*Cik[11][1][1])))+((Onkb[11][0]*rk[11][2])-(Onkb[11][2]*
      rk[11][0])));
    Ankb[11][2] = (((AOnkri[11][2]*Cik[11][2][2])+((AOnkri[11][0]*Cik[11][0][2])
      +(AOnkri[11][1]*Cik[11][1][2])))+((Onkb[11][1]*rk[11][0])-(Onkb[11][0]*
      rk[11][1])));
    AOnkri[12][0] = (Ankb[11][0]+((Onkb[11][1]*ri[12][2])-(Onkb[11][2]*ri[12][1]
      )));
    AOnkri[12][1] = (Ankb[11][1]+((Onkb[11][2]*ri[12][0])-(Onkb[11][0]*ri[12][2]
      )));
    AOnkri[12][2] = (Ankb[11][2]+((Onkb[11][0]*ri[12][1])-(Onkb[11][1]*ri[12][0]
      )));
    Ankb[12][0] = (((AOnkri[12][2]*Cik[12][2][0])+((AOnkri[12][0]*Cik[12][0][0])
      +(AOnkri[12][1]*Cik[12][1][0])))+((Onkb[12][2]*rk[12][1])-(Onkb[12][1]*
      rk[12][2])));
    Ankb[12][1] = (((AOnkri[12][2]*Cik[12][2][1])+((AOnkri[12][0]*Cik[12][0][1])
      +(AOnkri[12][1]*Cik[12][1][1])))+((Onkb[12][0]*rk[12][2])-(Onkb[12][2]*
      rk[12][0])));
    Ankb[12][2] = (((AOnkri[12][2]*Cik[12][2][2])+((AOnkri[12][0]*Cik[12][0][2])
      +(AOnkri[12][1]*Cik[12][1][2])))+((Onkb[12][1]*rk[12][0])-(Onkb[12][0]*
      rk[12][1])));
    AOnkri[13][0] = (Ankb[12][0]+((Onkb[12][1]*ri[13][2])-(Onkb[12][2]*ri[13][1]
      )));
    AOnkri[13][1] = (Ankb[12][1]+((Onkb[12][2]*ri[13][0])-(Onkb[12][0]*ri[13][2]
      )));
    AOnkri[13][2] = (Ankb[12][2]+((Onkb[12][0]*ri[13][1])-(Onkb[12][1]*ri[13][0]
      )));
    Ankb[13][0] = (((AOnkri[13][2]*Cik[13][2][0])+((AOnkri[13][0]*Cik[13][0][0])
      +(AOnkri[13][1]*Cik[13][1][0])))+((Onkb[13][2]*rk[13][1])-(Onkb[13][1]*
      rk[13][2])));
    Ankb[13][1] = (((AOnkri[13][2]*Cik[13][2][1])+((AOnkri[13][0]*Cik[13][0][1])
      +(AOnkri[13][1]*Cik[13][1][1])))+((Onkb[13][0]*rk[13][2])-(Onkb[13][2]*
      rk[13][0])));
    Ankb[13][2] = (((AOnkri[13][2]*Cik[13][2][2])+((AOnkri[13][0]*Cik[13][0][2])
      +(AOnkri[13][1]*Cik[13][1][2])))+((Onkb[13][1]*rk[13][0])-(Onkb[13][0]*
      rk[13][1])));
    AnkAtk[0][0] = (Ankb[0][0]+Atk[0][0]);
    AnkAtk[0][1] = (Ankb[0][1]+Atk[0][1]);
    AnkAtk[0][2] = (Ankb[0][2]+Atk[0][2]);
    ank[0][0] = ((AnkAtk[0][2]*Cik[0][0][2])+((AnkAtk[0][0]*Cik[0][0][0])+(
      AnkAtk[0][1]*Cik[0][0][1])));
    ank[0][1] = ((AnkAtk[0][2]*Cik[0][1][2])+((AnkAtk[0][0]*Cik[0][1][0])+(
      AnkAtk[0][1]*Cik[0][1][1])));
    ank[0][2] = ((AnkAtk[0][2]*Cik[0][2][2])+((AnkAtk[0][0]*Cik[0][2][0])+(
      AnkAtk[0][1]*Cik[0][2][1])));
    AnkAtk[1][0] = (Ankb[1][0]+Atk[1][0]);
    AnkAtk[1][1] = (Ankb[1][1]+Atk[1][1]);
    AnkAtk[1][2] = (Ankb[1][2]+Atk[1][2]);
    ank[1][0] = ((AnkAtk[1][2]*cnk[1][0][2])+((AnkAtk[1][0]*cnk[1][0][0])+(
      AnkAtk[1][1]*cnk[1][0][1])));
    ank[1][1] = ((AnkAtk[1][2]*cnk[1][1][2])+((AnkAtk[1][0]*cnk[1][1][0])+(
      AnkAtk[1][1]*cnk[1][1][1])));
    ank[1][2] = ((AnkAtk[1][2]*cnk[1][2][2])+((AnkAtk[1][0]*cnk[1][2][0])+(
      AnkAtk[1][1]*cnk[1][2][1])));
    AnkAtk[2][0] = (Ankb[2][0]+Atk[2][0]);
    AnkAtk[2][1] = (Ankb[2][1]+Atk[2][1]);
    AnkAtk[2][2] = (Ankb[2][2]+Atk[2][2]);
    ank[2][0] = ((AnkAtk[2][2]*cnk[2][0][2])+((AnkAtk[2][0]*cnk[2][0][0])+(
      AnkAtk[2][1]*cnk[2][0][1])));
    ank[2][1] = ((AnkAtk[2][2]*cnk[2][1][2])+((AnkAtk[2][0]*cnk[2][1][0])+(
      AnkAtk[2][1]*cnk[2][1][1])));
    ank[2][2] = ((AnkAtk[2][2]*cnk[2][2][2])+((AnkAtk[2][0]*cnk[2][2][0])+(
      AnkAtk[2][1]*cnk[2][2][1])));
    AnkAtk[3][0] = (Ankb[3][0]+Atk[3][0]);
    AnkAtk[3][1] = (Ankb[3][1]+Atk[3][1]);
    AnkAtk[3][2] = (Ankb[3][2]+Atk[3][2]);
    ank[3][0] = ((AnkAtk[3][2]*cnk[3][0][2])+((AnkAtk[3][0]*cnk[3][0][0])+(
      AnkAtk[3][1]*cnk[3][0][1])));
    ank[3][1] = ((AnkAtk[3][2]*cnk[3][1][2])+((AnkAtk[3][0]*cnk[3][1][0])+(
      AnkAtk[3][1]*cnk[3][1][1])));
    ank[3][2] = ((AnkAtk[3][2]*cnk[3][2][2])+((AnkAtk[3][0]*cnk[3][2][0])+(
      AnkAtk[3][1]*cnk[3][2][1])));
    AnkAtk[4][0] = (Ankb[4][0]+Atk[4][0]);
    AnkAtk[4][1] = (Ankb[4][1]+Atk[4][1]);
    AnkAtk[4][2] = (Ankb[4][2]+Atk[4][2]);
    ank[4][0] = ((AnkAtk[4][2]*cnk[4][0][2])+((AnkAtk[4][0]*cnk[4][0][0])+(
      AnkAtk[4][1]*cnk[4][0][1])));
    ank[4][1] = ((AnkAtk[4][2]*cnk[4][1][2])+((AnkAtk[4][0]*cnk[4][1][0])+(
      AnkAtk[4][1]*cnk[4][1][1])));
    ank[4][2] = ((AnkAtk[4][2]*cnk[4][2][2])+((AnkAtk[4][0]*cnk[4][2][0])+(
      AnkAtk[4][1]*cnk[4][2][1])));
    AnkAtk[5][0] = (Ankb[5][0]+Atk[5][0]);
    AnkAtk[5][1] = (Ankb[5][1]+Atk[5][1]);
    AnkAtk[5][2] = (Ankb[5][2]+Atk[5][2]);
    ank[5][0] = ((AnkAtk[5][2]*cnk[5][0][2])+((AnkAtk[5][0]*cnk[5][0][0])+(
      AnkAtk[5][1]*cnk[5][0][1])));
    ank[5][1] = ((AnkAtk[5][2]*cnk[5][1][2])+((AnkAtk[5][0]*cnk[5][1][0])+(
      AnkAtk[5][1]*cnk[5][1][1])));
    ank[5][2] = ((AnkAtk[5][2]*cnk[5][2][2])+((AnkAtk[5][0]*cnk[5][2][0])+(
      AnkAtk[5][1]*cnk[5][2][1])));
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
    anb[0][0] = ank[0][0];
    anb[0][1] = ank[0][1];
    anb[0][2] = ank[0][2];
    anb[1][0] = ank[1][0];
    anb[1][1] = ank[1][1];
    anb[1][2] = ank[1][2];
    anb[2][0] = ank[2][0];
    anb[2][1] = ank[2][1];
    anb[2][2] = ank[2][2];
    anb[3][0] = ank[3][0];
    anb[3][1] = ank[3][1];
    anb[3][2] = ank[3][2];
    anb[4][0] = ank[4][0];
    anb[4][1] = ank[4][1];
    anb[4][2] = ank[4][2];
    anb[5][0] = ank[5][0];
    anb[5][1] = ank[5][1];
    anb[5][2] = ank[5][2];
    anb[6][0] = ank[6][0];
    anb[6][1] = ank[6][1];
    anb[6][2] = ank[6][2];
    anb[7][0] = ank[7][0];
    anb[7][1] = ank[7][1];
    anb[7][2] = ank[7][2];
    anb[8][0] = ank[8][0];
    anb[8][1] = ank[8][1];
    anb[8][2] = ank[8][2];
    anb[9][0] = ank[9][0];
    anb[9][1] = ank[9][1];
    anb[9][2] = ank[9][2];
    anb[10][0] = ank[10][0];
    anb[10][1] = ank[10][1];
    anb[10][2] = ank[10][2];
    anb[11][0] = ank[11][0];
    anb[11][1] = ank[11][1];
    anb[11][2] = ank[11][2];
    anb[12][0] = ank[12][0];
    anb[12][1] = ank[12][1];
    anb[12][2] = ank[12][2];
    anb[13][0] = ank[13][0];
    anb[13][1] = ank[13][1];
    anb[13][2] = ank[13][2];
/*
Compute constraint acceleration errors
*/
    roustate = 3;
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  617 adds/subtracts/negates
                    564 multiplies
                      0 divides
                    470 assignments
*/
}

void sdmassmat(double mmat[14][14])
{
/* Return the system mass matrix (LHS)
*/
    int i,j;

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(57,23);
        return;
    }
    sddomm(57);
    for (i = 0; i < 14; i++) {
        for (j = i; j <= 13; j++) {
            mmat[i][j] = mm[i][j];
            mmat[j][i] = mm[i][j];
        }
    }
}

void sdfrcmat(double fmat[14])
{
/* Return the system force matrix (RHS), excluding constraints
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(58,23);
        return;
    }
    sddofs0();
    for (i = 0; i < 14; i++) {
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

void sdperr(double errs[14])
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
}

void sdverr(double errs[14])
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
}

void sdaerr(double errs[14])
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
}
int 
sdchkbnum(int routine,
    int bnum)
{

    if ((bnum < -1) || (bnum > 13)) {
        sdseterr(routine,15);
        return 1;
    }
    return 0;
}
int 
sdchkjnum(int routine,
    int jnum)
{

    if ((jnum < 0) || (jnum > 13)) {
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
    double pp[14][14],dpp[14][14];
    int i,j,c;
    double sum;
    double dfk[14][3],dtk[14][3],dtau[14],dltci[1][3],dltc[1][3],dlfci[1][3],
      dlfc[1][3];
    double dTinb[1][3],dToutb[1][3],dltaufi[1][3],dltaufo[1][3],dltauti[1][3],
      dltauto[1][3];
    double dfs[14],row[14],dinvrow[14];

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
        sdldubsl(14,14,mmap,mlo,dfs,row);
        sdldubsd(14,14,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 13; i++) {
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
        sdldubsl(14,14,mmap,mlo,dfs,row);
        sdldubsd(14,14,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 13; i++) {
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
        sdldubsl(14,14,mmap,mlo,dfs,row);
        sdldubsd(14,14,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 13; i++) {
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
        sdldubsl(14,14,mmap,mlo,dfs,row);
        sdldubsd(14,14,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 13; i++) {
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
        sdldubsl(14,14,mmap,mlo,dfs,row);
        sdldubsd(14,14,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 13; i++) {
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
        sdldubsl(14,14,mmap,mlo,dfs,row);
        sdldubsd(14,14,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 13; i++) {
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
        sdldubsl(14,14,mmap,mlo,dfs,row);
        sdldubsd(14,14,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 13; i++) {
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
        sdldubsl(14,14,mmap,mlo,dfs,row);
        sdldubsd(14,14,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 13; i++) {
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
        sdldubsl(14,14,mmap,mlo,dfs,row);
        sdldubsd(14,14,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 13; i++) {
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
        sdldubsl(14,14,mmap,mlo,dfs,row);
        sdldubsd(14,14,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 13; i++) {
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
        sdldubsl(14,14,mmap,mlo,dfs,row);
        sdldubsd(14,14,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 13; i++) {
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
        sdldubsl(14,14,mmap,mlo,dfs,row);
        sdldubsd(14,14,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 13; i++) {
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
        sdldubsl(14,14,mmap,mlo,dfs,row);
        sdldubsd(14,14,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 13; i++) {
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
        sdldubsl(14,14,mmap,mlo,dfs,row);
        sdldubsd(14,14,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 13; i++) {
            pp[13][i] = row[i];
            dpp[i][13] = dinvrow[i];
        }
        wmap[13] = 13;
/*
Produce constraint coefficient matrix WW
*/
        for (c = 0; c <= 13; c++) {
            for (i = c; i <= 13; i++) {
                sum = 0.;
                for (j = 0; j <= 13; j++) {
                    sum = sum+pp[wmap[c]][j]*dpp[j][wmap[i]];
                }
                ww[wmap[c]][wmap[i]] = sum;
                ww[wmap[i]][wmap[c]] = sum;
            }
        }
/*
Form QR decomposition of WW
*/
        sdqrdcomp(14,14,14,14,wmap,wmap,ww,qraux,jpvt);
        wwflg = 1;
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain 1470 adds/subtracts/negates
                   1470 multiplies
                      0 divides
                   2401 assignments
*/
}

void sdxudot0(int routine,
    double oudot0[14])
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
    sdldubslv(14,14,mmap,works,mlo,mdi,fs,udot);
    for (i = 0; i <= 13; i++) {
        oudot0[i] = udot[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     14 assignments
*/
}

void sdudot0(double oudot0[14])
{

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(66,23);
        return;
    }
    sdxudot0(66,oudot0);
}

void sdsetudot(double iudot[14])
{
/*
Assign udots and advance to stage Dynamics Ready
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(68,23);
        return;
    }
    for (i = 0; i <= 13; i++) {
        udot[i] = iudot[i];
    }
    sdrhs();
}

void sdxudotm(int routine,
    double imult[14],
    double oudotm[14])
{
/*
Compute udots due only to multipliers
*/
    int i;

    sdlhs(routine);
    sdmfrc(imult);
    sdfsmult();
    sdldubslv(14,14,mmap,works,mlo,mdi,fs,udot);
    for (i = 0; i <= 13; i++) {
        oudotm[i] = udot[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     14 assignments
*/
}

void sdudotm(double imult[14],
    double oudotm[14])
{

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(67,23);
        return;
    }
    sdxudotm(67,imult,oudotm);
}

void sdderiv(double oqdot[14],
    double oudot[14])
{
/*
This is the derivative section for a 14-body ground-based
system with 14 hinge degree(s) of freedom.
14 of the degrees of freedom may follow prescribed motion.
There are 14 constraints.
*/
    double workr[14],bb[14],b0[14],v0[14],p0[14];
    int iwork[14];
    int i,j;
    double udot0[14],udot1[14];

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
    wsiz = 14;
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
    for (i = 0; i <= 13; i++) {
        bb[i] = -b0[i];
    }
    if (stabvel  !=  0.) {
        for (i = 0; i <= 13; i++) {
            bb[i] = bb[i]-stabvel*v0[i];
        }
    }
    if (stabpos  !=  0.) {
        for (i = 0; i <= 13; i++) {
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
    sdqrbslv(14,14,14,14,wmap,wmap,1e-13,workr,iwork,ww,qraux,jpvt,bb,mult,&
      wrank);
    for (i = 0; i <= 13; i++) {
        multmap[i] = 0;
    }
    for (i = 0; i < wrank; i++) {
        multmap[jpvt[i]] = 1;
    }
    j = 0;
    for (i = 0; i <= 13; i++) {
        if (multmap[i] != 0) {
            multmap[j] = wmap[i];
            j = j+1;
        }
    }
/*
Compute final udots
*/
    sdxudotm(17,mult,udot1);
    for (i = 0; i <= 13; i++) {
        udot[i] = udot0[i]+udot1[i];
    }
    sdrhs();
    for (i = 0; i <= 13; i++) {
        oqdot[i] = qdot[i];
    }
    for (i = 0; i <= 13; i++) {
        oudot[i] = udot[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   56 adds/subtracts/negates
                     28 multiplies
                      0 divides
                     84 assignments
*/
}
/*
Compute residuals for use with DAE integrator
*/

void sdresid(double eqdot[14],
    double eudot[14],
    double emults[14],
    double resid[42])
{
    int i;
    double uderrs[14],p0[14];

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(16,23);
        return;
    }
    if (stabposq == 1) {
        sdseterr(16,33);
    }
    sdfulltrq(eudot,emults,uderrs);
    for (i = 0; i < 14; i++) {
        resid[i] = eqdot[i]-qdot[i];
    }
    for (i = 0; i < 14; i++) {
        resid[14+i] = uderrs[i];
    }
    sdverr(&resid[28]);
    if (stabpos  !=  0.) {
        sdperr(p0);
        for (i = 0; i < 14; i++) {
            resid[28+i] = resid[28+i]+stabpos*p0[i];
        }
    }
    for (i = 0; i < 14; i++) {
        udot[i] = eudot[i];
    }
    for (i = 0; i < 14; i++) {
        mult[i] = emults[i];
    }
    sdrhs();
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   28 adds/subtracts/negates
                     14 multiplies
                      0 divides
                     70 assignments
*/
}

void sdmult(double omults[14],
    int *owrank,
    int omultmap[14])
{
    int i;

    if (roustate != 3) {
        sdseterr(34,24);
        return;
    }
    for (i = 0; i < 14; i++) {
        omults[i] = mult[i];
        if (i <= wrank-1) {
            omultmap[i] = multmap[i];
        } else {
            omultmap[i] = -1;
        }
    }
    *owrank = wrank;
}

void sdreac(double force[14][3],
    double torque[14][3])
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
    fc[13][0] = ((mk[13]*(AnkAtk[13][0]-gk[13][0]))-ufk[13][0]);
    fc[13][1] = ((mk[13]*(AnkAtk[13][1]-gk[13][1]))-ufk[13][1]);
    fc[13][2] = ((mk[13]*(AnkAtk[13][2]-gk[13][2]))-ufk[13][2]);
    tc[13][0] = ((WkIkWk[13][0]+((ik[13][0][2]*onk[13][2])+((ik[13][0][0]*
      onk[13][0])+(ik[13][0][1]*onk[13][1]))))-(utk[13][0]+((fc[13][2]*rk[13][1]
      )-(fc[13][1]*rk[13][2]))));
    tc[13][1] = ((WkIkWk[13][1]+((ik[13][1][2]*onk[13][2])+((ik[13][1][0]*
      onk[13][0])+(ik[13][1][1]*onk[13][1]))))-(utk[13][1]+((fc[13][0]*rk[13][2]
      )-(fc[13][2]*rk[13][0]))));
    tc[13][2] = ((WkIkWk[13][2]+((ik[13][2][2]*onk[13][2])+((ik[13][2][0]*
      onk[13][0])+(ik[13][2][1]*onk[13][1]))))-(utk[13][2]+((fc[13][1]*rk[13][0]
      )-(fc[13][0]*rk[13][1]))));
    fccikt[13][0] = ((Cik[13][0][2]*fc[13][2])+((Cik[13][0][0]*fc[13][0])+(
      Cik[13][0][1]*fc[13][1])));
    fccikt[13][1] = ((Cik[13][1][2]*fc[13][2])+((Cik[13][1][0]*fc[13][0])+(
      Cik[13][1][1]*fc[13][1])));
    fccikt[13][2] = ((Cik[13][2][2]*fc[13][2])+((Cik[13][2][0]*fc[13][0])+(
      Cik[13][2][1]*fc[13][1])));
    ffk[12][0] = (ufk[12][0]-fccikt[13][0]);
    ffk[12][1] = (ufk[12][1]-fccikt[13][1]);
    ffk[12][2] = (ufk[12][2]-fccikt[13][2]);
    ttk[12][0] = (utk[12][0]-(((Cik[13][0][2]*tc[13][2])+((Cik[13][0][0]*
      tc[13][0])+(Cik[13][0][1]*tc[13][1])))+((fccikt[13][2]*ri[13][1])-(
      fccikt[13][1]*ri[13][2]))));
    ttk[12][1] = (utk[12][1]-(((Cik[13][1][2]*tc[13][2])+((Cik[13][1][0]*
      tc[13][0])+(Cik[13][1][1]*tc[13][1])))+((fccikt[13][0]*ri[13][2])-(
      fccikt[13][2]*ri[13][0]))));
    ttk[12][2] = (utk[12][2]-(((Cik[13][2][2]*tc[13][2])+((Cik[13][2][0]*
      tc[13][0])+(Cik[13][2][1]*tc[13][1])))+((fccikt[13][1]*ri[13][0])-(
      fccikt[13][0]*ri[13][1]))));
    fc[12][0] = ((mk[12]*(AnkAtk[12][0]-gk[12][0]))-ffk[12][0]);
    fc[12][1] = ((mk[12]*(AnkAtk[12][1]-gk[12][1]))-ffk[12][1]);
    fc[12][2] = ((mk[12]*(AnkAtk[12][2]-gk[12][2]))-ffk[12][2]);
    tc[12][0] = ((WkIkWk[12][0]+((ik[12][0][2]*onk[12][2])+((ik[12][0][0]*
      onk[12][0])+(ik[12][0][1]*onk[12][1]))))-(ttk[12][0]+((fc[12][2]*rk[12][1]
      )-(fc[12][1]*rk[12][2]))));
    tc[12][1] = ((WkIkWk[12][1]+((ik[12][1][2]*onk[12][2])+((ik[12][1][0]*
      onk[12][0])+(ik[12][1][1]*onk[12][1]))))-(ttk[12][1]+((fc[12][0]*rk[12][2]
      )-(fc[12][2]*rk[12][0]))));
    tc[12][2] = ((WkIkWk[12][2]+((ik[12][2][2]*onk[12][2])+((ik[12][2][0]*
      onk[12][0])+(ik[12][2][1]*onk[12][1]))))-(ttk[12][2]+((fc[12][1]*rk[12][0]
      )-(fc[12][0]*rk[12][1]))));
    fccikt[12][0] = ((Cik[12][0][2]*fc[12][2])+((Cik[12][0][0]*fc[12][0])+(
      Cik[12][0][1]*fc[12][1])));
    fccikt[12][1] = ((Cik[12][1][2]*fc[12][2])+((Cik[12][1][0]*fc[12][0])+(
      Cik[12][1][1]*fc[12][1])));
    fccikt[12][2] = ((Cik[12][2][2]*fc[12][2])+((Cik[12][2][0]*fc[12][0])+(
      Cik[12][2][1]*fc[12][1])));
    ffk[11][0] = (ufk[11][0]-fccikt[12][0]);
    ffk[11][1] = (ufk[11][1]-fccikt[12][1]);
    ffk[11][2] = (ufk[11][2]-fccikt[12][2]);
    ttk[11][0] = (utk[11][0]-(((Cik[12][0][2]*tc[12][2])+((Cik[12][0][0]*
      tc[12][0])+(Cik[12][0][1]*tc[12][1])))+((fccikt[12][2]*ri[12][1])-(
      fccikt[12][1]*ri[12][2]))));
    ttk[11][1] = (utk[11][1]-(((Cik[12][1][2]*tc[12][2])+((Cik[12][1][0]*
      tc[12][0])+(Cik[12][1][1]*tc[12][1])))+((fccikt[12][0]*ri[12][2])-(
      fccikt[12][2]*ri[12][0]))));
    ttk[11][2] = (utk[11][2]-(((Cik[12][2][2]*tc[12][2])+((Cik[12][2][0]*
      tc[12][0])+(Cik[12][2][1]*tc[12][1])))+((fccikt[12][1]*ri[12][0])-(
      fccikt[12][0]*ri[12][1]))));
    fc[11][0] = ((mk[11]*(AnkAtk[11][0]-gk[11][0]))-ffk[11][0]);
    fc[11][1] = ((mk[11]*(AnkAtk[11][1]-gk[11][1]))-ffk[11][1]);
    fc[11][2] = ((mk[11]*(AnkAtk[11][2]-gk[11][2]))-ffk[11][2]);
    tc[11][0] = ((WkIkWk[11][0]+((ik[11][0][2]*onk[11][2])+((ik[11][0][0]*
      onk[11][0])+(ik[11][0][1]*onk[11][1]))))-(ttk[11][0]+((fc[11][2]*rk[11][1]
      )-(fc[11][1]*rk[11][2]))));
    tc[11][1] = ((WkIkWk[11][1]+((ik[11][1][2]*onk[11][2])+((ik[11][1][0]*
      onk[11][0])+(ik[11][1][1]*onk[11][1]))))-(ttk[11][1]+((fc[11][0]*rk[11][2]
      )-(fc[11][2]*rk[11][0]))));
    tc[11][2] = ((WkIkWk[11][2]+((ik[11][2][2]*onk[11][2])+((ik[11][2][0]*
      onk[11][0])+(ik[11][2][1]*onk[11][1]))))-(ttk[11][2]+((fc[11][1]*rk[11][0]
      )-(fc[11][0]*rk[11][1]))));
    fccikt[11][0] = ((Cik[11][0][2]*fc[11][2])+((Cik[11][0][0]*fc[11][0])+(
      Cik[11][0][1]*fc[11][1])));
    fccikt[11][1] = ((Cik[11][1][2]*fc[11][2])+((Cik[11][1][0]*fc[11][0])+(
      Cik[11][1][1]*fc[11][1])));
    fccikt[11][2] = ((Cik[11][2][2]*fc[11][2])+((Cik[11][2][0]*fc[11][0])+(
      Cik[11][2][1]*fc[11][1])));
    ffk[10][0] = (ufk[10][0]-fccikt[11][0]);
    ffk[10][1] = (ufk[10][1]-fccikt[11][1]);
    ffk[10][2] = (ufk[10][2]-fccikt[11][2]);
    ttk[10][0] = (utk[10][0]-(((Cik[11][0][2]*tc[11][2])+((Cik[11][0][0]*
      tc[11][0])+(Cik[11][0][1]*tc[11][1])))+((fccikt[11][2]*ri[11][1])-(
      fccikt[11][1]*ri[11][2]))));
    ttk[10][1] = (utk[10][1]-(((Cik[11][1][2]*tc[11][2])+((Cik[11][1][0]*
      tc[11][0])+(Cik[11][1][1]*tc[11][1])))+((fccikt[11][0]*ri[11][2])-(
      fccikt[11][2]*ri[11][0]))));
    ttk[10][2] = (utk[10][2]-(((Cik[11][2][2]*tc[11][2])+((Cik[11][2][0]*
      tc[11][0])+(Cik[11][2][1]*tc[11][1])))+((fccikt[11][1]*ri[11][0])-(
      fccikt[11][0]*ri[11][1]))));
    fc[10][0] = ((mk[10]*(AnkAtk[10][0]-gk[10][0]))-ffk[10][0]);
    fc[10][1] = ((mk[10]*(AnkAtk[10][1]-gk[10][1]))-ffk[10][1]);
    fc[10][2] = ((mk[10]*(AnkAtk[10][2]-gk[10][2]))-ffk[10][2]);
    tc[10][0] = ((WkIkWk[10][0]+((ik[10][0][2]*onk[10][2])+((ik[10][0][0]*
      onk[10][0])+(ik[10][0][1]*onk[10][1]))))-(ttk[10][0]+((fc[10][2]*rk[10][1]
      )-(fc[10][1]*rk[10][2]))));
    tc[10][1] = ((WkIkWk[10][1]+((ik[10][1][2]*onk[10][2])+((ik[10][1][0]*
      onk[10][0])+(ik[10][1][1]*onk[10][1]))))-(ttk[10][1]+((fc[10][0]*rk[10][2]
      )-(fc[10][2]*rk[10][0]))));
    tc[10][2] = ((WkIkWk[10][2]+((ik[10][2][2]*onk[10][2])+((ik[10][2][0]*
      onk[10][0])+(ik[10][2][1]*onk[10][1]))))-(ttk[10][2]+((fc[10][1]*rk[10][0]
      )-(fc[10][0]*rk[10][1]))));
    fccikt[10][0] = ((Cik[10][0][2]*fc[10][2])+((Cik[10][0][0]*fc[10][0])+(
      Cik[10][0][1]*fc[10][1])));
    fccikt[10][1] = ((Cik[10][1][2]*fc[10][2])+((Cik[10][1][0]*fc[10][0])+(
      Cik[10][1][1]*fc[10][1])));
    fccikt[10][2] = ((Cik[10][2][2]*fc[10][2])+((Cik[10][2][0]*fc[10][0])+(
      Cik[10][2][1]*fc[10][1])));
    ffk[9][0] = (ufk[9][0]-fccikt[10][0]);
    ffk[9][1] = (ufk[9][1]-fccikt[10][1]);
    ffk[9][2] = (ufk[9][2]-fccikt[10][2]);
    ttk[9][0] = (utk[9][0]-(((Cik[10][0][2]*tc[10][2])+((Cik[10][0][0]*tc[10][0]
      )+(Cik[10][0][1]*tc[10][1])))+((fccikt[10][2]*ri[10][1])-(fccikt[10][1]*
      ri[10][2]))));
    ttk[9][1] = (utk[9][1]-(((Cik[10][1][2]*tc[10][2])+((Cik[10][1][0]*tc[10][0]
      )+(Cik[10][1][1]*tc[10][1])))+((fccikt[10][0]*ri[10][2])-(fccikt[10][2]*
      ri[10][0]))));
    ttk[9][2] = (utk[9][2]-(((Cik[10][2][2]*tc[10][2])+((Cik[10][2][0]*tc[10][0]
      )+(Cik[10][2][1]*tc[10][1])))+((fccikt[10][1]*ri[10][0])-(fccikt[10][0]*
      ri[10][1]))));
    fc[9][0] = ((mk[9]*(AnkAtk[9][0]-gk[9][0]))-ffk[9][0]);
    fc[9][1] = ((mk[9]*(AnkAtk[9][1]-gk[9][1]))-ffk[9][1]);
    fc[9][2] = ((mk[9]*(AnkAtk[9][2]-gk[9][2]))-ffk[9][2]);
    tc[9][0] = ((WkIkWk[9][0]+((ik[9][0][2]*onk[9][2])+((ik[9][0][0]*onk[9][0])+
      (ik[9][0][1]*onk[9][1]))))-(ttk[9][0]+((fc[9][2]*rk[9][1])-(fc[9][1]*
      rk[9][2]))));
    tc[9][1] = ((WkIkWk[9][1]+((ik[9][1][2]*onk[9][2])+((ik[9][1][0]*onk[9][0])+
      (ik[9][1][1]*onk[9][1]))))-(ttk[9][1]+((fc[9][0]*rk[9][2])-(fc[9][2]*
      rk[9][0]))));
    tc[9][2] = ((WkIkWk[9][2]+((ik[9][2][2]*onk[9][2])+((ik[9][2][0]*onk[9][0])+
      (ik[9][2][1]*onk[9][1]))))-(ttk[9][2]+((fc[9][1]*rk[9][0])-(fc[9][0]*
      rk[9][1]))));
    fccikt[9][0] = ((Cik[9][0][2]*fc[9][2])+((Cik[9][0][0]*fc[9][0])+(
      Cik[9][0][1]*fc[9][1])));
    fccikt[9][1] = ((Cik[9][1][2]*fc[9][2])+((Cik[9][1][0]*fc[9][0])+(
      Cik[9][1][1]*fc[9][1])));
    fccikt[9][2] = ((Cik[9][2][2]*fc[9][2])+((Cik[9][2][0]*fc[9][0])+(
      Cik[9][2][1]*fc[9][1])));
    ffk[8][0] = (ufk[8][0]-fccikt[9][0]);
    ffk[8][1] = (ufk[8][1]-fccikt[9][1]);
    ffk[8][2] = (ufk[8][2]-fccikt[9][2]);
    ttk[8][0] = (utk[8][0]-(((Cik[9][0][2]*tc[9][2])+((Cik[9][0][0]*tc[9][0])+(
      Cik[9][0][1]*tc[9][1])))+((fccikt[9][2]*ri[9][1])-(fccikt[9][1]*ri[9][2]))
      ));
    ttk[8][1] = (utk[8][1]-(((Cik[9][1][2]*tc[9][2])+((Cik[9][1][0]*tc[9][0])+(
      Cik[9][1][1]*tc[9][1])))+((fccikt[9][0]*ri[9][2])-(fccikt[9][2]*ri[9][0]))
      ));
    ttk[8][2] = (utk[8][2]-(((Cik[9][2][2]*tc[9][2])+((Cik[9][2][0]*tc[9][0])+(
      Cik[9][2][1]*tc[9][1])))+((fccikt[9][1]*ri[9][0])-(fccikt[9][0]*ri[9][1]))
      ));
    fc[8][0] = ((mk[8]*(AnkAtk[8][0]-gk[8][0]))-ffk[8][0]);
    fc[8][1] = ((mk[8]*(AnkAtk[8][1]-gk[8][1]))-ffk[8][1]);
    fc[8][2] = ((mk[8]*(AnkAtk[8][2]-gk[8][2]))-ffk[8][2]);
    tc[8][0] = ((WkIkWk[8][0]+((ik[8][0][2]*onk[8][2])+((ik[8][0][0]*onk[8][0])+
      (ik[8][0][1]*onk[8][1]))))-(ttk[8][0]+((fc[8][2]*rk[8][1])-(fc[8][1]*
      rk[8][2]))));
    tc[8][1] = ((WkIkWk[8][1]+((ik[8][1][2]*onk[8][2])+((ik[8][1][0]*onk[8][0])+
      (ik[8][1][1]*onk[8][1]))))-(ttk[8][1]+((fc[8][0]*rk[8][2])-(fc[8][2]*
      rk[8][0]))));
    tc[8][2] = ((WkIkWk[8][2]+((ik[8][2][2]*onk[8][2])+((ik[8][2][0]*onk[8][0])+
      (ik[8][2][1]*onk[8][1]))))-(ttk[8][2]+((fc[8][1]*rk[8][0])-(fc[8][0]*
      rk[8][1]))));
    fccikt[8][0] = ((Cik[8][0][2]*fc[8][2])+((Cik[8][0][0]*fc[8][0])+(
      Cik[8][0][1]*fc[8][1])));
    fccikt[8][1] = ((Cik[8][1][2]*fc[8][2])+((Cik[8][1][0]*fc[8][0])+(
      Cik[8][1][1]*fc[8][1])));
    fccikt[8][2] = ((Cik[8][2][2]*fc[8][2])+((Cik[8][2][0]*fc[8][0])+(
      Cik[8][2][1]*fc[8][1])));
    ffk[0][0] = (ufk[0][0]-fccikt[8][0]);
    ffk[0][1] = (ufk[0][1]-fccikt[8][1]);
    ffk[0][2] = (ufk[0][2]-fccikt[8][2]);
    ttk[0][0] = (utk[0][0]-(((Cik[8][0][2]*tc[8][2])+((Cik[8][0][0]*tc[8][0])+(
      Cik[8][0][1]*tc[8][1])))+((fccikt[8][2]*ri[8][1])-(fccikt[8][1]*ri[8][2]))
      ));
    ttk[0][1] = (utk[0][1]-(((Cik[8][1][2]*tc[8][2])+((Cik[8][1][0]*tc[8][0])+(
      Cik[8][1][1]*tc[8][1])))+((fccikt[8][0]*ri[8][2])-(fccikt[8][2]*ri[8][0]))
      ));
    ttk[0][2] = (utk[0][2]-(((Cik[8][2][2]*tc[8][2])+((Cik[8][2][0]*tc[8][0])+(
      Cik[8][2][1]*tc[8][1])))+((fccikt[8][1]*ri[8][0])-(fccikt[8][0]*ri[8][1]))
      ));
    fc[7][0] = ((mk[7]*(AnkAtk[7][0]-gk[7][0]))-ufk[7][0]);
    fc[7][1] = ((mk[7]*(AnkAtk[7][1]-gk[7][1]))-ufk[7][1]);
    fc[7][2] = ((mk[7]*(AnkAtk[7][2]-gk[7][2]))-ufk[7][2]);
    tc[7][0] = ((WkIkWk[7][0]+((ik[7][0][2]*onk[7][2])+((ik[7][0][0]*onk[7][0])+
      (ik[7][0][1]*onk[7][1]))))-(utk[7][0]+((fc[7][2]*rk[7][1])-(fc[7][1]*
      rk[7][2]))));
    tc[7][1] = ((WkIkWk[7][1]+((ik[7][1][2]*onk[7][2])+((ik[7][1][0]*onk[7][0])+
      (ik[7][1][1]*onk[7][1]))))-(utk[7][1]+((fc[7][0]*rk[7][2])-(fc[7][2]*
      rk[7][0]))));
    tc[7][2] = ((WkIkWk[7][2]+((ik[7][2][2]*onk[7][2])+((ik[7][2][0]*onk[7][0])+
      (ik[7][2][1]*onk[7][1]))))-(utk[7][2]+((fc[7][1]*rk[7][0])-(fc[7][0]*
      rk[7][1]))));
    fccikt[7][0] = ((Cik[7][0][2]*fc[7][2])+((Cik[7][0][0]*fc[7][0])+(
      Cik[7][0][1]*fc[7][1])));
    fccikt[7][1] = ((Cik[7][1][2]*fc[7][2])+((Cik[7][1][0]*fc[7][0])+(
      Cik[7][1][1]*fc[7][1])));
    fccikt[7][2] = ((Cik[7][2][2]*fc[7][2])+((Cik[7][2][0]*fc[7][0])+(
      Cik[7][2][1]*fc[7][1])));
    ffk[6][0] = (ufk[6][0]-fccikt[7][0]);
    ffk[6][1] = (ufk[6][1]-fccikt[7][1]);
    ffk[6][2] = (ufk[6][2]-fccikt[7][2]);
    ttk[6][0] = (utk[6][0]-(((Cik[7][0][2]*tc[7][2])+((Cik[7][0][0]*tc[7][0])+(
      Cik[7][0][1]*tc[7][1])))+((fccikt[7][2]*ri[7][1])-(fccikt[7][1]*ri[7][2]))
      ));
    ttk[6][1] = (utk[6][1]-(((Cik[7][1][2]*tc[7][2])+((Cik[7][1][0]*tc[7][0])+(
      Cik[7][1][1]*tc[7][1])))+((fccikt[7][0]*ri[7][2])-(fccikt[7][2]*ri[7][0]))
      ));
    ttk[6][2] = (utk[6][2]-(((Cik[7][2][2]*tc[7][2])+((Cik[7][2][0]*tc[7][0])+(
      Cik[7][2][1]*tc[7][1])))+((fccikt[7][1]*ri[7][0])-(fccikt[7][0]*ri[7][1]))
      ));
    fc[6][0] = ((mk[6]*(AnkAtk[6][0]-gk[6][0]))-ffk[6][0]);
    fc[6][1] = ((mk[6]*(AnkAtk[6][1]-gk[6][1]))-ffk[6][1]);
    fc[6][2] = ((mk[6]*(AnkAtk[6][2]-gk[6][2]))-ffk[6][2]);
    tc[6][0] = ((WkIkWk[6][0]+((ik[6][0][2]*onk[6][2])+((ik[6][0][0]*onk[6][0])+
      (ik[6][0][1]*onk[6][1]))))-(ttk[6][0]+((fc[6][2]*rk[6][1])-(fc[6][1]*
      rk[6][2]))));
    tc[6][1] = ((WkIkWk[6][1]+((ik[6][1][2]*onk[6][2])+((ik[6][1][0]*onk[6][0])+
      (ik[6][1][1]*onk[6][1]))))-(ttk[6][1]+((fc[6][0]*rk[6][2])-(fc[6][2]*
      rk[6][0]))));
    tc[6][2] = ((WkIkWk[6][2]+((ik[6][2][2]*onk[6][2])+((ik[6][2][0]*onk[6][0])+
      (ik[6][2][1]*onk[6][1]))))-(ttk[6][2]+((fc[6][1]*rk[6][0])-(fc[6][0]*
      rk[6][1]))));
    fccikt[6][0] = ((Cik[6][0][2]*fc[6][2])+((Cik[6][0][0]*fc[6][0])+(
      Cik[6][0][1]*fc[6][1])));
    fccikt[6][1] = ((Cik[6][1][2]*fc[6][2])+((Cik[6][1][0]*fc[6][0])+(
      Cik[6][1][1]*fc[6][1])));
    fccikt[6][2] = ((Cik[6][2][2]*fc[6][2])+((Cik[6][2][0]*fc[6][0])+(
      Cik[6][2][1]*fc[6][1])));
    ffk[5][0] = (ufk[5][0]-fccikt[6][0]);
    ffk[5][1] = (ufk[5][1]-fccikt[6][1]);
    ffk[5][2] = (ufk[5][2]-fccikt[6][2]);
    ttk[5][0] = (utk[5][0]-(((Cik[6][0][2]*tc[6][2])+((Cik[6][0][0]*tc[6][0])+(
      Cik[6][0][1]*tc[6][1])))+((fccikt[6][2]*ri[6][1])-(fccikt[6][1]*ri[6][2]))
      ));
    ttk[5][1] = (utk[5][1]-(((Cik[6][1][2]*tc[6][2])+((Cik[6][1][0]*tc[6][0])+(
      Cik[6][1][1]*tc[6][1])))+((fccikt[6][0]*ri[6][2])-(fccikt[6][2]*ri[6][0]))
      ));
    ttk[5][2] = (utk[5][2]-(((Cik[6][2][2]*tc[6][2])+((Cik[6][2][0]*tc[6][0])+(
      Cik[6][2][1]*tc[6][1])))+((fccikt[6][1]*ri[6][0])-(fccikt[6][0]*ri[6][1]))
      ));
    fc[5][0] = ((mk[5]*(AnkAtk[5][0]-gk[5][0]))-ffk[5][0]);
    fc[5][1] = ((mk[5]*(AnkAtk[5][1]-gk[5][1]))-ffk[5][1]);
    fc[5][2] = ((mk[5]*(AnkAtk[5][2]-gk[5][2]))-ffk[5][2]);
    tc[5][0] = ((WkIkWk[5][0]+((ik[5][0][2]*onk[5][2])+((ik[5][0][0]*onk[5][0])+
      (ik[5][0][1]*onk[5][1]))))-(ttk[5][0]+((fc[5][2]*rk[5][1])-(fc[5][1]*
      rk[5][2]))));
    tc[5][1] = ((WkIkWk[5][1]+((ik[5][1][2]*onk[5][2])+((ik[5][1][0]*onk[5][0])+
      (ik[5][1][1]*onk[5][1]))))-(ttk[5][1]+((fc[5][0]*rk[5][2])-(fc[5][2]*
      rk[5][0]))));
    tc[5][2] = ((WkIkWk[5][2]+((ik[5][2][2]*onk[5][2])+((ik[5][2][0]*onk[5][0])+
      (ik[5][2][1]*onk[5][1]))))-(ttk[5][2]+((fc[5][1]*rk[5][0])-(fc[5][0]*
      rk[5][1]))));
    fccikt[5][0] = ((Cik[5][0][2]*fc[5][2])+((Cik[5][0][0]*fc[5][0])+(
      Cik[5][0][1]*fc[5][1])));
    fccikt[5][1] = ((Cik[5][1][2]*fc[5][2])+((Cik[5][1][0]*fc[5][0])+(
      Cik[5][1][1]*fc[5][1])));
    fccikt[5][2] = ((Cik[5][2][2]*fc[5][2])+((Cik[5][2][0]*fc[5][0])+(
      Cik[5][2][1]*fc[5][1])));
    ffk[4][0] = (ufk[4][0]-fccikt[5][0]);
    ffk[4][1] = (ufk[4][1]-fccikt[5][1]);
    ffk[4][2] = (ufk[4][2]-fccikt[5][2]);
    ttk[4][0] = (utk[4][0]-(((Cik[5][0][2]*tc[5][2])+((Cik[5][0][0]*tc[5][0])+(
      Cik[5][0][1]*tc[5][1])))+((fccikt[5][2]*ri[5][1])-(fccikt[5][1]*ri[5][2]))
      ));
    ttk[4][1] = (utk[4][1]-(((Cik[5][1][2]*tc[5][2])+((Cik[5][1][0]*tc[5][0])+(
      Cik[5][1][1]*tc[5][1])))+((fccikt[5][0]*ri[5][2])-(fccikt[5][2]*ri[5][0]))
      ));
    ttk[4][2] = (utk[4][2]-(((Cik[5][2][2]*tc[5][2])+((Cik[5][2][0]*tc[5][0])+(
      Cik[5][2][1]*tc[5][1])))+((fccikt[5][1]*ri[5][0])-(fccikt[5][0]*ri[5][1]))
      ));
    fc[4][0] = ((mk[4]*(AnkAtk[4][0]-gk[4][0]))-ffk[4][0]);
    fc[4][1] = ((mk[4]*(AnkAtk[4][1]-gk[4][1]))-ffk[4][1]);
    fc[4][2] = ((mk[4]*(AnkAtk[4][2]-gk[4][2]))-ffk[4][2]);
    tc[4][0] = ((WkIkWk[4][0]+((ik[4][0][2]*onk[4][2])+((ik[4][0][0]*onk[4][0])+
      (ik[4][0][1]*onk[4][1]))))-(ttk[4][0]+((fc[4][2]*rk[4][1])-(fc[4][1]*
      rk[4][2]))));
    tc[4][1] = ((WkIkWk[4][1]+((ik[4][1][2]*onk[4][2])+((ik[4][1][0]*onk[4][0])+
      (ik[4][1][1]*onk[4][1]))))-(ttk[4][1]+((fc[4][0]*rk[4][2])-(fc[4][2]*
      rk[4][0]))));
    tc[4][2] = ((WkIkWk[4][2]+((ik[4][2][2]*onk[4][2])+((ik[4][2][0]*onk[4][0])+
      (ik[4][2][1]*onk[4][1]))))-(ttk[4][2]+((fc[4][1]*rk[4][0])-(fc[4][0]*
      rk[4][1]))));
    fccikt[4][0] = ((Cik[4][0][2]*fc[4][2])+((Cik[4][0][0]*fc[4][0])+(
      Cik[4][0][1]*fc[4][1])));
    fccikt[4][1] = ((Cik[4][1][2]*fc[4][2])+((Cik[4][1][0]*fc[4][0])+(
      Cik[4][1][1]*fc[4][1])));
    fccikt[4][2] = ((Cik[4][2][2]*fc[4][2])+((Cik[4][2][0]*fc[4][0])+(
      Cik[4][2][1]*fc[4][1])));
    ffk[3][0] = (ufk[3][0]-fccikt[4][0]);
    ffk[3][1] = (ufk[3][1]-fccikt[4][1]);
    ffk[3][2] = (ufk[3][2]-fccikt[4][2]);
    ttk[3][0] = (utk[3][0]-(((Cik[4][0][2]*tc[4][2])+((Cik[4][0][0]*tc[4][0])+(
      Cik[4][0][1]*tc[4][1])))+((fccikt[4][2]*ri[4][1])-(fccikt[4][1]*ri[4][2]))
      ));
    ttk[3][1] = (utk[3][1]-(((Cik[4][1][2]*tc[4][2])+((Cik[4][1][0]*tc[4][0])+(
      Cik[4][1][1]*tc[4][1])))+((fccikt[4][0]*ri[4][2])-(fccikt[4][2]*ri[4][0]))
      ));
    ttk[3][2] = (utk[3][2]-(((Cik[4][2][2]*tc[4][2])+((Cik[4][2][0]*tc[4][0])+(
      Cik[4][2][1]*tc[4][1])))+((fccikt[4][1]*ri[4][0])-(fccikt[4][0]*ri[4][1]))
      ));
    fc[3][0] = ((mk[3]*(AnkAtk[3][0]-gk[3][0]))-ffk[3][0]);
    fc[3][1] = ((mk[3]*(AnkAtk[3][1]-gk[3][1]))-ffk[3][1]);
    fc[3][2] = ((mk[3]*(AnkAtk[3][2]-gk[3][2]))-ffk[3][2]);
    tc[3][0] = ((WkIkWk[3][0]+((ik[3][0][2]*onk[3][2])+((ik[3][0][0]*onk[3][0])+
      (ik[3][0][1]*onk[3][1]))))-(ttk[3][0]+((fc[3][2]*rk[3][1])-(fc[3][1]*
      rk[3][2]))));
    tc[3][1] = ((WkIkWk[3][1]+((ik[3][1][2]*onk[3][2])+((ik[3][1][0]*onk[3][0])+
      (ik[3][1][1]*onk[3][1]))))-(ttk[3][1]+((fc[3][0]*rk[3][2])-(fc[3][2]*
      rk[3][0]))));
    tc[3][2] = ((WkIkWk[3][2]+((ik[3][2][2]*onk[3][2])+((ik[3][2][0]*onk[3][0])+
      (ik[3][2][1]*onk[3][1]))))-(ttk[3][2]+((fc[3][1]*rk[3][0])-(fc[3][0]*
      rk[3][1]))));
    fccikt[3][0] = ((Cik[3][0][2]*fc[3][2])+((Cik[3][0][0]*fc[3][0])+(
      Cik[3][0][1]*fc[3][1])));
    fccikt[3][1] = ((Cik[3][1][2]*fc[3][2])+((Cik[3][1][0]*fc[3][0])+(
      Cik[3][1][1]*fc[3][1])));
    fccikt[3][2] = ((Cik[3][2][2]*fc[3][2])+((Cik[3][2][0]*fc[3][0])+(
      Cik[3][2][1]*fc[3][1])));
    ffk[2][0] = (ufk[2][0]-fccikt[3][0]);
    ffk[2][1] = (ufk[2][1]-fccikt[3][1]);
    ffk[2][2] = (ufk[2][2]-fccikt[3][2]);
    ttk[2][0] = (utk[2][0]-(((Cik[3][0][2]*tc[3][2])+((Cik[3][0][0]*tc[3][0])+(
      Cik[3][0][1]*tc[3][1])))+((fccikt[3][2]*ri[3][1])-(fccikt[3][1]*ri[3][2]))
      ));
    ttk[2][1] = (utk[2][1]-(((Cik[3][1][2]*tc[3][2])+((Cik[3][1][0]*tc[3][0])+(
      Cik[3][1][1]*tc[3][1])))+((fccikt[3][0]*ri[3][2])-(fccikt[3][2]*ri[3][0]))
      ));
    ttk[2][2] = (utk[2][2]-(((Cik[3][2][2]*tc[3][2])+((Cik[3][2][0]*tc[3][0])+(
      Cik[3][2][1]*tc[3][1])))+((fccikt[3][1]*ri[3][0])-(fccikt[3][0]*ri[3][1]))
      ));
    fc[2][0] = ((mk[2]*(AnkAtk[2][0]-gk[2][0]))-ffk[2][0]);
    fc[2][1] = ((mk[2]*(AnkAtk[2][1]-gk[2][1]))-ffk[2][1]);
    fc[2][2] = ((mk[2]*(AnkAtk[2][2]-gk[2][2]))-ffk[2][2]);
    tc[2][0] = ((WkIkWk[2][0]+((ik[2][0][2]*onk[2][2])+((ik[2][0][0]*onk[2][0])+
      (ik[2][0][1]*onk[2][1]))))-(ttk[2][0]+((fc[2][2]*rk[2][1])-(fc[2][1]*
      rk[2][2]))));
    tc[2][1] = ((WkIkWk[2][1]+((ik[2][1][2]*onk[2][2])+((ik[2][1][0]*onk[2][0])+
      (ik[2][1][1]*onk[2][1]))))-(ttk[2][1]+((fc[2][0]*rk[2][2])-(fc[2][2]*
      rk[2][0]))));
    tc[2][2] = ((WkIkWk[2][2]+((ik[2][2][2]*onk[2][2])+((ik[2][2][0]*onk[2][0])+
      (ik[2][2][1]*onk[2][1]))))-(ttk[2][2]+((fc[2][1]*rk[2][0])-(fc[2][0]*
      rk[2][1]))));
    fccikt[2][0] = ((Cik[2][0][2]*fc[2][2])+((Cik[2][0][0]*fc[2][0])+(
      Cik[2][0][1]*fc[2][1])));
    fccikt[2][1] = ((Cik[2][1][2]*fc[2][2])+((Cik[2][1][0]*fc[2][0])+(
      Cik[2][1][1]*fc[2][1])));
    fccikt[2][2] = ((Cik[2][2][2]*fc[2][2])+((Cik[2][2][0]*fc[2][0])+(
      Cik[2][2][1]*fc[2][1])));
    ffk[0][0] = (ffk[0][0]-fccikt[2][0]);
    ffk[0][1] = (ffk[0][1]-fccikt[2][1]);
    ffk[0][2] = (ffk[0][2]-fccikt[2][2]);
    ttk[0][0] = (ttk[0][0]-(((Cik[2][0][2]*tc[2][2])+((Cik[2][0][0]*tc[2][0])+(
      Cik[2][0][1]*tc[2][1])))+((fccikt[2][2]*ri[2][1])-(fccikt[2][1]*ri[2][2]))
      ));
    ttk[0][1] = (ttk[0][1]-(((Cik[2][1][2]*tc[2][2])+((Cik[2][1][0]*tc[2][0])+(
      Cik[2][1][1]*tc[2][1])))+((fccikt[2][0]*ri[2][2])-(fccikt[2][2]*ri[2][0]))
      ));
    ttk[0][2] = (ttk[0][2]-(((Cik[2][2][2]*tc[2][2])+((Cik[2][2][0]*tc[2][0])+(
      Cik[2][2][1]*tc[2][1])))+((fccikt[2][1]*ri[2][0])-(fccikt[2][0]*ri[2][1]))
      ));
    fc[1][0] = ((mk[1]*(AnkAtk[1][0]-gk[1][0]))-ufk[1][0]);
    fc[1][1] = ((mk[1]*(AnkAtk[1][1]-gk[1][1]))-ufk[1][1]);
    fc[1][2] = ((mk[1]*(AnkAtk[1][2]-gk[1][2]))-ufk[1][2]);
    tc[1][0] = ((WkIkWk[1][0]+((ik[1][0][2]*onk[1][2])+((ik[1][0][0]*onk[1][0])+
      (ik[1][0][1]*onk[1][1]))))-(utk[1][0]+((fc[1][2]*rk[1][1])-(fc[1][1]*
      rk[1][2]))));
    tc[1][1] = ((WkIkWk[1][1]+((ik[1][1][2]*onk[1][2])+((ik[1][1][0]*onk[1][0])+
      (ik[1][1][1]*onk[1][1]))))-(utk[1][1]+((fc[1][0]*rk[1][2])-(fc[1][2]*
      rk[1][0]))));
    tc[1][2] = ((WkIkWk[1][2]+((ik[1][2][2]*onk[1][2])+((ik[1][2][0]*onk[1][0])+
      (ik[1][2][1]*onk[1][1]))))-(utk[1][2]+((fc[1][1]*rk[1][0])-(fc[1][0]*
      rk[1][1]))));
    fccikt[1][0] = ((Cik[1][0][2]*fc[1][2])+((Cik[1][0][0]*fc[1][0])+(
      Cik[1][0][1]*fc[1][1])));
    fccikt[1][1] = ((Cik[1][1][2]*fc[1][2])+((Cik[1][1][0]*fc[1][0])+(
      Cik[1][1][1]*fc[1][1])));
    fccikt[1][2] = ((Cik[1][2][2]*fc[1][2])+((Cik[1][2][0]*fc[1][0])+(
      Cik[1][2][1]*fc[1][1])));
    ffk[0][0] = (ffk[0][0]-fccikt[1][0]);
    ffk[0][1] = (ffk[0][1]-fccikt[1][1]);
    ffk[0][2] = (ffk[0][2]-fccikt[1][2]);
    ttk[0][0] = (ttk[0][0]-(((Cik[1][0][2]*tc[1][2])+((Cik[1][0][0]*tc[1][0])+(
      Cik[1][0][1]*tc[1][1])))+((fccikt[1][2]*ri[1][1])-(fccikt[1][1]*ri[1][2]))
      ));
    ttk[0][1] = (ttk[0][1]-(((Cik[1][1][2]*tc[1][2])+((Cik[1][1][0]*tc[1][0])+(
      Cik[1][1][1]*tc[1][1])))+((fccikt[1][0]*ri[1][2])-(fccikt[1][2]*ri[1][0]))
      ));
    ttk[0][2] = (ttk[0][2]-(((Cik[1][2][2]*tc[1][2])+((Cik[1][2][0]*tc[1][0])+(
      Cik[1][2][1]*tc[1][1])))+((fccikt[1][1]*ri[1][0])-(fccikt[1][0]*ri[1][1]))
      ));
    fc[0][0] = ((mk[0]*(AnkAtk[0][0]-gk[0][0]))-ffk[0][0]);
    fc[0][1] = ((mk[0]*(AnkAtk[0][1]-gk[0][1]))-ffk[0][1]);
    fc[0][2] = ((mk[0]*(AnkAtk[0][2]-gk[0][2]))-ffk[0][2]);
    tc[0][0] = ((WkIkWk[0][0]+((ik[0][0][2]*Onkb[0][2])+((ik[0][0][0]*Onkb[0][0]
      )+(ik[0][0][1]*Onkb[0][1]))))-(ttk[0][0]+((fc[0][2]*rk[0][1])-(fc[0][1]*
      rk[0][2]))));
    tc[0][1] = ((WkIkWk[0][1]+((ik[0][1][2]*Onkb[0][2])+((ik[0][1][0]*Onkb[0][0]
      )+(ik[0][1][1]*Onkb[0][1]))))-(ttk[0][1]+((fc[0][0]*rk[0][2])-(fc[0][2]*
      rk[0][0]))));
    tc[0][2] = ((WkIkWk[0][2]+((ik[0][2][2]*Onkb[0][2])+((ik[0][2][0]*Onkb[0][0]
      )+(ik[0][2][1]*Onkb[0][1]))))-(ttk[0][2]+((fc[0][1]*rk[0][0])-(fc[0][0]*
      rk[0][1]))));
    force[0][0] = fc[0][0];
    torque[0][0] = tc[0][0];
    force[0][1] = fc[0][1];
    torque[0][1] = tc[0][1];
    force[0][2] = fc[0][2];
    torque[0][2] = tc[0][2];
    force[1][0] = fc[1][0];
    torque[1][0] = tc[1][0];
    force[1][1] = fc[1][1];
    torque[1][1] = tc[1][1];
    force[1][2] = fc[1][2];
    torque[1][2] = tc[1][2];
    force[2][0] = fc[2][0];
    torque[2][0] = tc[2][0];
    force[2][1] = fc[2][1];
    torque[2][1] = tc[2][1];
    force[2][2] = fc[2][2];
    torque[2][2] = tc[2][2];
    force[3][0] = fc[3][0];
    torque[3][0] = tc[3][0];
    force[3][1] = fc[3][1];
    torque[3][1] = tc[3][1];
    force[3][2] = fc[3][2];
    torque[3][2] = tc[3][2];
    force[4][0] = fc[4][0];
    torque[4][0] = tc[4][0];
    force[4][1] = fc[4][1];
    torque[4][1] = tc[4][1];
    force[4][2] = fc[4][2];
    torque[4][2] = tc[4][2];
    force[5][0] = fc[5][0];
    torque[5][0] = tc[5][0];
    force[5][1] = fc[5][1];
    torque[5][1] = tc[5][1];
    force[5][2] = fc[5][2];
    torque[5][2] = tc[5][2];
    force[6][0] = fc[6][0];
    torque[6][0] = tc[6][0];
    force[6][1] = fc[6][1];
    torque[6][1] = tc[6][1];
    force[6][2] = fc[6][2];
    torque[6][2] = tc[6][2];
    force[7][0] = fc[7][0];
    torque[7][0] = tc[7][0];
    force[7][1] = fc[7][1];
    torque[7][1] = tc[7][1];
    force[7][2] = fc[7][2];
    torque[7][2] = tc[7][2];
    force[8][0] = fc[8][0];
    torque[8][0] = tc[8][0];
    force[8][1] = fc[8][1];
    torque[8][1] = tc[8][1];
    force[8][2] = fc[8][2];
    torque[8][2] = tc[8][2];
    force[9][0] = fc[9][0];
    torque[9][0] = tc[9][0];
    force[9][1] = fc[9][1];
    torque[9][1] = tc[9][1];
    force[9][2] = fc[9][2];
    torque[9][2] = tc[9][2];
    force[10][0] = fc[10][0];
    torque[10][0] = tc[10][0];
    force[10][1] = fc[10][1];
    torque[10][1] = tc[10][1];
    force[10][2] = fc[10][2];
    torque[10][2] = tc[10][2];
    force[11][0] = fc[11][0];
    torque[11][0] = tc[11][0];
    force[11][1] = fc[11][1];
    torque[11][1] = tc[11][1];
    force[11][2] = fc[11][2];
    torque[11][2] = tc[11][2];
    force[12][0] = fc[12][0];
    torque[12][0] = tc[12][0];
    force[12][1] = fc[12][1];
    torque[12][1] = tc[12][1];
    force[12][2] = fc[12][2];
    torque[12][2] = tc[12][2];
    force[13][0] = fc[13][0];
    torque[13][0] = tc[13][0];
    force[13][1] = fc[13][1];
    torque[13][1] = tc[13][1];
    force[13][2] = fc[13][2];
    torque[13][2] = tc[13][2];
/*
Compute reaction forces for tree weld joints
*/
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  648 adds/subtracts/negates
                    564 multiplies
                      0 divides
                    285 assignments
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
    double lk[14][3],hnk[14][3];

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(19,23);
        return;
    }
    lk[0][0] = (mk[0]*vnk[0][0]);
    lk[0][1] = (mk[0]*vnk[0][1]);
    lk[0][2] = (mk[0]*vnk[0][2]);
    lk[1][0] = (mk[1]*vnk[1][0]);
    lk[1][1] = (mk[1]*vnk[1][1]);
    lk[1][2] = (mk[1]*vnk[1][2]);
    lk[2][0] = (mk[2]*vnk[2][0]);
    lk[2][1] = (mk[2]*vnk[2][1]);
    lk[2][2] = (mk[2]*vnk[2][2]);
    lk[3][0] = (mk[3]*vnk[3][0]);
    lk[3][1] = (mk[3]*vnk[3][1]);
    lk[3][2] = (mk[3]*vnk[3][2]);
    lk[4][0] = (mk[4]*vnk[4][0]);
    lk[4][1] = (mk[4]*vnk[4][1]);
    lk[4][2] = (mk[4]*vnk[4][2]);
    lk[5][0] = (mk[5]*vnk[5][0]);
    lk[5][1] = (mk[5]*vnk[5][1]);
    lk[5][2] = (mk[5]*vnk[5][2]);
    lk[6][0] = (mk[6]*vnk[6][0]);
    lk[6][1] = (mk[6]*vnk[6][1]);
    lk[6][2] = (mk[6]*vnk[6][2]);
    lk[7][0] = (mk[7]*vnk[7][0]);
    lk[7][1] = (mk[7]*vnk[7][1]);
    lk[7][2] = (mk[7]*vnk[7][2]);
    lk[8][0] = (mk[8]*vnk[8][0]);
    lk[8][1] = (mk[8]*vnk[8][1]);
    lk[8][2] = (mk[8]*vnk[8][2]);
    lk[9][0] = (mk[9]*vnk[9][0]);
    lk[9][1] = (mk[9]*vnk[9][1]);
    lk[9][2] = (mk[9]*vnk[9][2]);
    lk[10][0] = (mk[10]*vnk[10][0]);
    lk[10][1] = (mk[10]*vnk[10][1]);
    lk[10][2] = (mk[10]*vnk[10][2]);
    lk[11][0] = (mk[11]*vnk[11][0]);
    lk[11][1] = (mk[11]*vnk[11][1]);
    lk[11][2] = (mk[11]*vnk[11][2]);
    lk[12][0] = (mk[12]*vnk[12][0]);
    lk[12][1] = (mk[12]*vnk[12][1]);
    lk[12][2] = (mk[12]*vnk[12][2]);
    lk[13][0] = (mk[13]*vnk[13][0]);
    lk[13][1] = (mk[13]*vnk[13][1]);
    lk[13][2] = (mk[13]*vnk[13][2]);
    hnk[0][0] = ((ik[0][0][2]*Wik[0][2])+((ik[0][0][0]*Wik[0][0])+(ik[0][0][1]*
      Wik[0][1])));
    hnk[0][1] = ((ik[0][1][2]*Wik[0][2])+((ik[0][1][0]*Wik[0][0])+(ik[0][1][1]*
      Wik[0][1])));
    hnk[0][2] = ((ik[0][2][2]*Wik[0][2])+((ik[0][2][0]*Wik[0][0])+(ik[0][2][1]*
      Wik[0][1])));
    hnk[1][0] = ((ik[1][0][2]*wk[1][2])+((ik[1][0][0]*wk[1][0])+(ik[1][0][1]*
      wk[1][1])));
    hnk[1][1] = ((ik[1][1][2]*wk[1][2])+((ik[1][1][0]*wk[1][0])+(ik[1][1][1]*
      wk[1][1])));
    hnk[1][2] = ((ik[1][2][2]*wk[1][2])+((ik[1][2][0]*wk[1][0])+(ik[1][2][1]*
      wk[1][1])));
    hnk[2][0] = ((ik[2][0][2]*wk[2][2])+((ik[2][0][0]*wk[2][0])+(ik[2][0][1]*
      wk[2][1])));
    hnk[2][1] = ((ik[2][1][2]*wk[2][2])+((ik[2][1][0]*wk[2][0])+(ik[2][1][1]*
      wk[2][1])));
    hnk[2][2] = ((ik[2][2][2]*wk[2][2])+((ik[2][2][0]*wk[2][0])+(ik[2][2][1]*
      wk[2][1])));
    hnk[3][0] = ((ik[3][0][2]*wk[3][2])+((ik[3][0][0]*wk[3][0])+(ik[3][0][1]*
      wk[3][1])));
    hnk[3][1] = ((ik[3][1][2]*wk[3][2])+((ik[3][1][0]*wk[3][0])+(ik[3][1][1]*
      wk[3][1])));
    hnk[3][2] = ((ik[3][2][2]*wk[3][2])+((ik[3][2][0]*wk[3][0])+(ik[3][2][1]*
      wk[3][1])));
    hnk[4][0] = ((ik[4][0][2]*wk[4][2])+((ik[4][0][0]*wk[4][0])+(ik[4][0][1]*
      wk[4][1])));
    hnk[4][1] = ((ik[4][1][2]*wk[4][2])+((ik[4][1][0]*wk[4][0])+(ik[4][1][1]*
      wk[4][1])));
    hnk[4][2] = ((ik[4][2][2]*wk[4][2])+((ik[4][2][0]*wk[4][0])+(ik[4][2][1]*
      wk[4][1])));
    hnk[5][0] = ((ik[5][0][2]*wk[5][2])+((ik[5][0][0]*wk[5][0])+(ik[5][0][1]*
      wk[5][1])));
    hnk[5][1] = ((ik[5][1][2]*wk[5][2])+((ik[5][1][0]*wk[5][0])+(ik[5][1][1]*
      wk[5][1])));
    hnk[5][2] = ((ik[5][2][2]*wk[5][2])+((ik[5][2][0]*wk[5][0])+(ik[5][2][1]*
      wk[5][1])));
    hnk[6][0] = ((ik[6][0][2]*wk[6][2])+((ik[6][0][0]*wk[6][0])+(ik[6][0][1]*
      wk[6][1])));
    hnk[6][1] = ((ik[6][1][2]*wk[6][2])+((ik[6][1][0]*wk[6][0])+(ik[6][1][1]*
      wk[6][1])));
    hnk[6][2] = ((ik[6][2][2]*wk[6][2])+((ik[6][2][0]*wk[6][0])+(ik[6][2][1]*
      wk[6][1])));
    hnk[7][0] = ((ik[7][0][2]*wk[7][2])+((ik[7][0][0]*wk[7][0])+(ik[7][0][1]*
      wk[7][1])));
    hnk[7][1] = ((ik[7][1][2]*wk[7][2])+((ik[7][1][0]*wk[7][0])+(ik[7][1][1]*
      wk[7][1])));
    hnk[7][2] = ((ik[7][2][2]*wk[7][2])+((ik[7][2][0]*wk[7][0])+(ik[7][2][1]*
      wk[7][1])));
    hnk[8][0] = ((ik[8][0][2]*wk[8][2])+((ik[8][0][0]*wk[8][0])+(ik[8][0][1]*
      wk[8][1])));
    hnk[8][1] = ((ik[8][1][2]*wk[8][2])+((ik[8][1][0]*wk[8][0])+(ik[8][1][1]*
      wk[8][1])));
    hnk[8][2] = ((ik[8][2][2]*wk[8][2])+((ik[8][2][0]*wk[8][0])+(ik[8][2][1]*
      wk[8][1])));
    hnk[9][0] = ((ik[9][0][2]*wk[9][2])+((ik[9][0][0]*wk[9][0])+(ik[9][0][1]*
      wk[9][1])));
    hnk[9][1] = ((ik[9][1][2]*wk[9][2])+((ik[9][1][0]*wk[9][0])+(ik[9][1][1]*
      wk[9][1])));
    hnk[9][2] = ((ik[9][2][2]*wk[9][2])+((ik[9][2][0]*wk[9][0])+(ik[9][2][1]*
      wk[9][1])));
    hnk[10][0] = ((ik[10][0][2]*wk[10][2])+((ik[10][0][0]*wk[10][0])+(
      ik[10][0][1]*wk[10][1])));
    hnk[10][1] = ((ik[10][1][2]*wk[10][2])+((ik[10][1][0]*wk[10][0])+(
      ik[10][1][1]*wk[10][1])));
    hnk[10][2] = ((ik[10][2][2]*wk[10][2])+((ik[10][2][0]*wk[10][0])+(
      ik[10][2][1]*wk[10][1])));
    hnk[11][0] = ((ik[11][0][2]*wk[11][2])+((ik[11][0][0]*wk[11][0])+(
      ik[11][0][1]*wk[11][1])));
    hnk[11][1] = ((ik[11][1][2]*wk[11][2])+((ik[11][1][0]*wk[11][0])+(
      ik[11][1][1]*wk[11][1])));
    hnk[11][2] = ((ik[11][2][2]*wk[11][2])+((ik[11][2][0]*wk[11][0])+(
      ik[11][2][1]*wk[11][1])));
    hnk[12][0] = ((ik[12][0][2]*wk[12][2])+((ik[12][0][0]*wk[12][0])+(
      ik[12][0][1]*wk[12][1])));
    hnk[12][1] = ((ik[12][1][2]*wk[12][2])+((ik[12][1][0]*wk[12][0])+(
      ik[12][1][1]*wk[12][1])));
    hnk[12][2] = ((ik[12][2][2]*wk[12][2])+((ik[12][2][0]*wk[12][0])+(
      ik[12][2][1]*wk[12][1])));
    hnk[13][0] = ((ik[13][0][2]*wk[13][2])+((ik[13][0][0]*wk[13][0])+(
      ik[13][0][1]*wk[13][1])));
    hnk[13][1] = ((ik[13][1][2]*wk[13][2])+((ik[13][1][0]*wk[13][0])+(
      ik[13][1][1]*wk[13][1])));
    hnk[13][2] = ((ik[13][2][2]*wk[13][2])+((ik[13][2][0]*wk[13][0])+(
      ik[13][2][1]*wk[13][1])));
    lm[0] = (lk[13][0]+(lk[12][0]+(lk[11][0]+(lk[10][0]+(lk[9][0]+(lk[8][0]+(
      lk[7][0]+(lk[6][0]+(lk[5][0]+(lk[4][0]+(lk[3][0]+(lk[2][0]+(lk[0][0]+
      lk[1][0])))))))))))));
    lm[1] = (lk[13][1]+(lk[12][1]+(lk[11][1]+(lk[10][1]+(lk[9][1]+(lk[8][1]+(
      lk[7][1]+(lk[6][1]+(lk[5][1]+(lk[4][1]+(lk[3][1]+(lk[2][1]+(lk[0][1]+
      lk[1][1])))))))))))));
    lm[2] = (lk[13][2]+(lk[12][2]+(lk[11][2]+(lk[10][2]+(lk[9][2]+(lk[8][2]+(
      lk[7][2]+(lk[6][2]+(lk[5][2]+(lk[4][2]+(lk[3][2]+(lk[2][2]+(lk[0][2]+
      lk[1][2])))))))))))));
    temp[0] = ((((cnk[2][0][2]*hnk[2][2])+((cnk[2][0][0]*hnk[2][0])+(
      cnk[2][0][1]*hnk[2][1])))+((lk[2][2]*rnk[2][1])-(lk[2][1]*rnk[2][2])))+(((
      (Cik[0][0][2]*hnk[0][2])+((Cik[0][0][0]*hnk[0][0])+(Cik[0][0][1]*hnk[0][1]
      )))+((lk[0][2]*rnk[0][1])-(lk[0][1]*rnk[0][2])))+(((cnk[1][0][2]*hnk[1][2]
      )+((cnk[1][0][0]*hnk[1][0])+(cnk[1][0][1]*hnk[1][1])))+((lk[1][2]*
      rnk[1][1])-(lk[1][1]*rnk[1][2])))));
    temp[1] = ((((cnk[4][0][2]*hnk[4][2])+((cnk[4][0][0]*hnk[4][0])+(
      cnk[4][0][1]*hnk[4][1])))+((lk[4][2]*rnk[4][1])-(lk[4][1]*rnk[4][2])))+(((
      (cnk[3][0][2]*hnk[3][2])+((cnk[3][0][0]*hnk[3][0])+(cnk[3][0][1]*hnk[3][1]
      )))+((lk[3][2]*rnk[3][1])-(lk[3][1]*rnk[3][2])))+temp[0]));
    temp[2] = ((((cnk[6][0][2]*hnk[6][2])+((cnk[6][0][0]*hnk[6][0])+(
      cnk[6][0][1]*hnk[6][1])))+((lk[6][2]*rnk[6][1])-(lk[6][1]*rnk[6][2])))+(((
      (cnk[5][0][2]*hnk[5][2])+((cnk[5][0][0]*hnk[5][0])+(cnk[5][0][1]*hnk[5][1]
      )))+((lk[5][2]*rnk[5][1])-(lk[5][1]*rnk[5][2])))+temp[1]));
    temp[3] = ((((cnk[8][0][2]*hnk[8][2])+((cnk[8][0][0]*hnk[8][0])+(
      cnk[8][0][1]*hnk[8][1])))+((lk[8][2]*rnk[8][1])-(lk[8][1]*rnk[8][2])))+(((
      (cnk[7][0][2]*hnk[7][2])+((cnk[7][0][0]*hnk[7][0])+(cnk[7][0][1]*hnk[7][1]
      )))+((lk[7][2]*rnk[7][1])-(lk[7][1]*rnk[7][2])))+temp[2]));
    temp[4] = ((((cnk[10][0][2]*hnk[10][2])+((cnk[10][0][0]*hnk[10][0])+(
      cnk[10][0][1]*hnk[10][1])))+((lk[10][2]*rnk[10][1])-(lk[10][1]*rnk[10][2])
      ))+((((cnk[9][0][2]*hnk[9][2])+((cnk[9][0][0]*hnk[9][0])+(cnk[9][0][1]*
      hnk[9][1])))+((lk[9][2]*rnk[9][1])-(lk[9][1]*rnk[9][2])))+temp[3]));
    temp[5] = ((((cnk[12][0][2]*hnk[12][2])+((cnk[12][0][0]*hnk[12][0])+(
      cnk[12][0][1]*hnk[12][1])))+((lk[12][2]*rnk[12][1])-(lk[12][1]*rnk[12][2])
      ))+((((cnk[11][0][2]*hnk[11][2])+((cnk[11][0][0]*hnk[11][0])+(
      cnk[11][0][1]*hnk[11][1])))+((lk[11][2]*rnk[11][1])-(lk[11][1]*rnk[11][2])
      ))+temp[4]));
    am[0] = (((((cnk[13][0][2]*hnk[13][2])+((cnk[13][0][0]*hnk[13][0])+(
      cnk[13][0][1]*hnk[13][1])))+((lk[13][2]*rnk[13][1])-(lk[13][1]*rnk[13][2])
      ))+temp[5])-((com[1]*lm[2])-(com[2]*lm[1])));
    temp[0] = ((((cnk[2][1][2]*hnk[2][2])+((cnk[2][1][0]*hnk[2][0])+(
      cnk[2][1][1]*hnk[2][1])))+((lk[2][0]*rnk[2][2])-(lk[2][2]*rnk[2][0])))+(((
      (Cik[0][1][2]*hnk[0][2])+((Cik[0][1][0]*hnk[0][0])+(Cik[0][1][1]*hnk[0][1]
      )))+((lk[0][0]*rnk[0][2])-(lk[0][2]*rnk[0][0])))+(((cnk[1][1][2]*hnk[1][2]
      )+((cnk[1][1][0]*hnk[1][0])+(cnk[1][1][1]*hnk[1][1])))+((lk[1][0]*
      rnk[1][2])-(lk[1][2]*rnk[1][0])))));
    temp[1] = ((((cnk[4][1][2]*hnk[4][2])+((cnk[4][1][0]*hnk[4][0])+(
      cnk[4][1][1]*hnk[4][1])))+((lk[4][0]*rnk[4][2])-(lk[4][2]*rnk[4][0])))+(((
      (cnk[3][1][2]*hnk[3][2])+((cnk[3][1][0]*hnk[3][0])+(cnk[3][1][1]*hnk[3][1]
      )))+((lk[3][0]*rnk[3][2])-(lk[3][2]*rnk[3][0])))+temp[0]));
    temp[2] = ((((cnk[6][1][2]*hnk[6][2])+((cnk[6][1][0]*hnk[6][0])+(
      cnk[6][1][1]*hnk[6][1])))+((lk[6][0]*rnk[6][2])-(lk[6][2]*rnk[6][0])))+(((
      (cnk[5][1][2]*hnk[5][2])+((cnk[5][1][0]*hnk[5][0])+(cnk[5][1][1]*hnk[5][1]
      )))+((lk[5][0]*rnk[5][2])-(lk[5][2]*rnk[5][0])))+temp[1]));
    temp[3] = ((((cnk[8][1][2]*hnk[8][2])+((cnk[8][1][0]*hnk[8][0])+(
      cnk[8][1][1]*hnk[8][1])))+((lk[8][0]*rnk[8][2])-(lk[8][2]*rnk[8][0])))+(((
      (cnk[7][1][2]*hnk[7][2])+((cnk[7][1][0]*hnk[7][0])+(cnk[7][1][1]*hnk[7][1]
      )))+((lk[7][0]*rnk[7][2])-(lk[7][2]*rnk[7][0])))+temp[2]));
    temp[4] = ((((cnk[10][1][2]*hnk[10][2])+((cnk[10][1][0]*hnk[10][0])+(
      cnk[10][1][1]*hnk[10][1])))+((lk[10][0]*rnk[10][2])-(lk[10][2]*rnk[10][0])
      ))+((((cnk[9][1][2]*hnk[9][2])+((cnk[9][1][0]*hnk[9][0])+(cnk[9][1][1]*
      hnk[9][1])))+((lk[9][0]*rnk[9][2])-(lk[9][2]*rnk[9][0])))+temp[3]));
    temp[5] = ((((cnk[12][1][2]*hnk[12][2])+((cnk[12][1][0]*hnk[12][0])+(
      cnk[12][1][1]*hnk[12][1])))+((lk[12][0]*rnk[12][2])-(lk[12][2]*rnk[12][0])
      ))+((((cnk[11][1][2]*hnk[11][2])+((cnk[11][1][0]*hnk[11][0])+(
      cnk[11][1][1]*hnk[11][1])))+((lk[11][0]*rnk[11][2])-(lk[11][2]*rnk[11][0])
      ))+temp[4]));
    am[1] = (((((cnk[13][1][2]*hnk[13][2])+((cnk[13][1][0]*hnk[13][0])+(
      cnk[13][1][1]*hnk[13][1])))+((lk[13][0]*rnk[13][2])-(lk[13][2]*rnk[13][0])
      ))+temp[5])-((com[2]*lm[0])-(com[0]*lm[2])));
    temp[0] = ((((cnk[2][2][2]*hnk[2][2])+((cnk[2][2][0]*hnk[2][0])+(
      cnk[2][2][1]*hnk[2][1])))+((lk[2][1]*rnk[2][0])-(lk[2][0]*rnk[2][1])))+(((
      (Cik[0][2][2]*hnk[0][2])+((Cik[0][2][0]*hnk[0][0])+(Cik[0][2][1]*hnk[0][1]
      )))+((lk[0][1]*rnk[0][0])-(lk[0][0]*rnk[0][1])))+(((cnk[1][2][2]*hnk[1][2]
      )+((cnk[1][2][0]*hnk[1][0])+(cnk[1][2][1]*hnk[1][1])))+((lk[1][1]*
      rnk[1][0])-(lk[1][0]*rnk[1][1])))));
    temp[1] = ((((cnk[4][2][2]*hnk[4][2])+((cnk[4][2][0]*hnk[4][0])+(
      cnk[4][2][1]*hnk[4][1])))+((lk[4][1]*rnk[4][0])-(lk[4][0]*rnk[4][1])))+(((
      (cnk[3][2][2]*hnk[3][2])+((cnk[3][2][0]*hnk[3][0])+(cnk[3][2][1]*hnk[3][1]
      )))+((lk[3][1]*rnk[3][0])-(lk[3][0]*rnk[3][1])))+temp[0]));
    temp[2] = ((((cnk[6][2][2]*hnk[6][2])+((cnk[6][2][0]*hnk[6][0])+(
      cnk[6][2][1]*hnk[6][1])))+((lk[6][1]*rnk[6][0])-(lk[6][0]*rnk[6][1])))+(((
      (cnk[5][2][2]*hnk[5][2])+((cnk[5][2][0]*hnk[5][0])+(cnk[5][2][1]*hnk[5][1]
      )))+((lk[5][1]*rnk[5][0])-(lk[5][0]*rnk[5][1])))+temp[1]));
    temp[3] = ((((cnk[8][2][2]*hnk[8][2])+((cnk[8][2][0]*hnk[8][0])+(
      cnk[8][2][1]*hnk[8][1])))+((lk[8][1]*rnk[8][0])-(lk[8][0]*rnk[8][1])))+(((
      (cnk[7][2][2]*hnk[7][2])+((cnk[7][2][0]*hnk[7][0])+(cnk[7][2][1]*hnk[7][1]
      )))+((lk[7][1]*rnk[7][0])-(lk[7][0]*rnk[7][1])))+temp[2]));
    temp[4] = ((((cnk[10][2][2]*hnk[10][2])+((cnk[10][2][0]*hnk[10][0])+(
      cnk[10][2][1]*hnk[10][1])))+((lk[10][1]*rnk[10][0])-(lk[10][0]*rnk[10][1])
      ))+((((cnk[9][2][2]*hnk[9][2])+((cnk[9][2][0]*hnk[9][0])+(cnk[9][2][1]*
      hnk[9][1])))+((lk[9][1]*rnk[9][0])-(lk[9][0]*rnk[9][1])))+temp[3]));
    temp[5] = ((((cnk[12][2][2]*hnk[12][2])+((cnk[12][2][0]*hnk[12][0])+(
      cnk[12][2][1]*hnk[12][1])))+((lk[12][1]*rnk[12][0])-(lk[12][0]*rnk[12][1])
      ))+((((cnk[11][2][2]*hnk[11][2])+((cnk[11][2][0]*hnk[11][0])+(
      cnk[11][2][1]*hnk[11][1])))+((lk[11][1]*rnk[11][0])-(lk[11][0]*rnk[11][1])
      ))+temp[4]));
    am[2] = (((((cnk[13][2][2]*hnk[13][2])+((cnk[13][2][0]*hnk[13][0])+(
      cnk[13][2][1]*hnk[13][1])))+((lk[13][1]*rnk[13][0])-(lk[13][0]*rnk[13][1])
      ))+temp[5])-((com[0]*lm[1])-(com[1]*lm[0])));
    temp[0] = ((((hnk[0][2]*Wik[0][2])+((hnk[0][0]*Wik[0][0])+(hnk[0][1]*
      Wik[0][1])))+((lk[0][2]*vnk[0][2])+((lk[0][0]*vnk[0][0])+(lk[0][1]*
      vnk[0][1]))))+(((hnk[1][2]*wk[1][2])+((hnk[1][0]*wk[1][0])+(hnk[1][1]*
      wk[1][1])))+((lk[1][2]*vnk[1][2])+((lk[1][0]*vnk[1][0])+(lk[1][1]*
      vnk[1][1])))));
    temp[1] = ((((hnk[3][2]*wk[3][2])+((hnk[3][0]*wk[3][0])+(hnk[3][1]*wk[3][1])
      ))+((lk[3][2]*vnk[3][2])+((lk[3][0]*vnk[3][0])+(lk[3][1]*vnk[3][1]))))+(((
      (hnk[2][2]*wk[2][2])+((hnk[2][0]*wk[2][0])+(hnk[2][1]*wk[2][1])))+((
      lk[2][2]*vnk[2][2])+((lk[2][0]*vnk[2][0])+(lk[2][1]*vnk[2][1]))))+temp[0])
      );
    temp[2] = ((((hnk[5][2]*wk[5][2])+((hnk[5][0]*wk[5][0])+(hnk[5][1]*wk[5][1])
      ))+((lk[5][2]*vnk[5][2])+((lk[5][0]*vnk[5][0])+(lk[5][1]*vnk[5][1]))))+(((
      (hnk[4][2]*wk[4][2])+((hnk[4][0]*wk[4][0])+(hnk[4][1]*wk[4][1])))+((
      lk[4][2]*vnk[4][2])+((lk[4][0]*vnk[4][0])+(lk[4][1]*vnk[4][1]))))+temp[1])
      );
    temp[3] = ((((hnk[7][2]*wk[7][2])+((hnk[7][0]*wk[7][0])+(hnk[7][1]*wk[7][1])
      ))+((lk[7][2]*vnk[7][2])+((lk[7][0]*vnk[7][0])+(lk[7][1]*vnk[7][1]))))+(((
      (hnk[6][2]*wk[6][2])+((hnk[6][0]*wk[6][0])+(hnk[6][1]*wk[6][1])))+((
      lk[6][2]*vnk[6][2])+((lk[6][0]*vnk[6][0])+(lk[6][1]*vnk[6][1]))))+temp[2])
      );
    temp[4] = ((((hnk[9][2]*wk[9][2])+((hnk[9][0]*wk[9][0])+(hnk[9][1]*wk[9][1])
      ))+((lk[9][2]*vnk[9][2])+((lk[9][0]*vnk[9][0])+(lk[9][1]*vnk[9][1]))))+(((
      (hnk[8][2]*wk[8][2])+((hnk[8][0]*wk[8][0])+(hnk[8][1]*wk[8][1])))+((
      lk[8][2]*vnk[8][2])+((lk[8][0]*vnk[8][0])+(lk[8][1]*vnk[8][1]))))+temp[3])
      );
    temp[5] = ((((hnk[11][2]*wk[11][2])+((hnk[11][0]*wk[11][0])+(hnk[11][1]*
      wk[11][1])))+((lk[11][2]*vnk[11][2])+((lk[11][0]*vnk[11][0])+(lk[11][1]*
      vnk[11][1]))))+((((hnk[10][2]*wk[10][2])+((hnk[10][0]*wk[10][0])+(
      hnk[10][1]*wk[10][1])))+((lk[10][2]*vnk[10][2])+((lk[10][0]*vnk[10][0])+(
      lk[10][1]*vnk[10][1]))))+temp[4]));
    *ke = (.5*((((hnk[13][2]*wk[13][2])+((hnk[13][0]*wk[13][0])+(hnk[13][1]*
      wk[13][1])))+((lk[13][2]*vnk[13][2])+((lk[13][0]*vnk[13][0])+(lk[13][1]*
      vnk[13][1]))))+((((hnk[12][2]*wk[12][2])+((hnk[12][0]*wk[12][0])+(
      hnk[12][1]*wk[12][1])))+((lk[12][2]*vnk[12][2])+((lk[12][0]*vnk[12][0])+(
      lk[12][1]*vnk[12][1]))))+temp[5])));
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  419 adds/subtracts/negates
                    469 multiplies
                      0 divides
                    115 assignments
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
    double ikcnkt[14][3][3];

    if ((roustate != 2) && (roustate != 3)) {
        sdseterr(20,23);
        return;
    }
    *mtoto = mtot;
    cm[0] = com[0];
    cm[1] = com[1];
    cm[2] = com[2];
    ikcnkt[0][0][0] = ((Cik[0][0][2]*ik[0][0][2])+((Cik[0][0][0]*ik[0][0][0])+(
      Cik[0][0][1]*ik[0][0][1])));
    ikcnkt[0][0][1] = ((Cik[0][1][2]*ik[0][0][2])+((Cik[0][1][0]*ik[0][0][0])+(
      Cik[0][1][1]*ik[0][0][1])));
    ikcnkt[0][0][2] = ((Cik[0][2][2]*ik[0][0][2])+((Cik[0][2][0]*ik[0][0][0])+(
      Cik[0][2][1]*ik[0][0][1])));
    ikcnkt[0][1][0] = ((Cik[0][0][2]*ik[0][1][2])+((Cik[0][0][0]*ik[0][1][0])+(
      Cik[0][0][1]*ik[0][1][1])));
    ikcnkt[0][1][1] = ((Cik[0][1][2]*ik[0][1][2])+((Cik[0][1][0]*ik[0][1][0])+(
      Cik[0][1][1]*ik[0][1][1])));
    ikcnkt[0][1][2] = ((Cik[0][2][2]*ik[0][1][2])+((Cik[0][2][0]*ik[0][1][0])+(
      Cik[0][2][1]*ik[0][1][1])));
    ikcnkt[0][2][0] = ((Cik[0][0][2]*ik[0][2][2])+((Cik[0][0][0]*ik[0][2][0])+(
      Cik[0][0][1]*ik[0][2][1])));
    ikcnkt[0][2][1] = ((Cik[0][1][2]*ik[0][2][2])+((Cik[0][1][0]*ik[0][2][0])+(
      Cik[0][1][1]*ik[0][2][1])));
    ikcnkt[0][2][2] = ((Cik[0][2][2]*ik[0][2][2])+((Cik[0][2][0]*ik[0][2][0])+(
      Cik[0][2][1]*ik[0][2][1])));
    ikcnkt[1][0][0] = ((cnk[1][0][2]*ik[1][0][2])+((cnk[1][0][0]*ik[1][0][0])+(
      cnk[1][0][1]*ik[1][0][1])));
    ikcnkt[1][0][1] = ((cnk[1][1][2]*ik[1][0][2])+((cnk[1][1][0]*ik[1][0][0])+(
      cnk[1][1][1]*ik[1][0][1])));
    ikcnkt[1][0][2] = ((cnk[1][2][2]*ik[1][0][2])+((cnk[1][2][0]*ik[1][0][0])+(
      cnk[1][2][1]*ik[1][0][1])));
    ikcnkt[1][1][0] = ((cnk[1][0][2]*ik[1][1][2])+((cnk[1][0][0]*ik[1][1][0])+(
      cnk[1][0][1]*ik[1][1][1])));
    ikcnkt[1][1][1] = ((cnk[1][1][2]*ik[1][1][2])+((cnk[1][1][0]*ik[1][1][0])+(
      cnk[1][1][1]*ik[1][1][1])));
    ikcnkt[1][1][2] = ((cnk[1][2][2]*ik[1][1][2])+((cnk[1][2][0]*ik[1][1][0])+(
      cnk[1][2][1]*ik[1][1][1])));
    ikcnkt[1][2][0] = ((cnk[1][0][2]*ik[1][2][2])+((cnk[1][0][0]*ik[1][2][0])+(
      cnk[1][0][1]*ik[1][2][1])));
    ikcnkt[1][2][1] = ((cnk[1][1][2]*ik[1][2][2])+((cnk[1][1][0]*ik[1][2][0])+(
      cnk[1][1][1]*ik[1][2][1])));
    ikcnkt[1][2][2] = ((cnk[1][2][2]*ik[1][2][2])+((cnk[1][2][0]*ik[1][2][0])+(
      cnk[1][2][1]*ik[1][2][1])));
    ikcnkt[2][0][0] = ((cnk[2][0][2]*ik[2][0][2])+((cnk[2][0][0]*ik[2][0][0])+(
      cnk[2][0][1]*ik[2][0][1])));
    ikcnkt[2][0][1] = ((cnk[2][1][2]*ik[2][0][2])+((cnk[2][1][0]*ik[2][0][0])+(
      cnk[2][1][1]*ik[2][0][1])));
    ikcnkt[2][0][2] = ((cnk[2][2][2]*ik[2][0][2])+((cnk[2][2][0]*ik[2][0][0])+(
      cnk[2][2][1]*ik[2][0][1])));
    ikcnkt[2][1][0] = ((cnk[2][0][2]*ik[2][1][2])+((cnk[2][0][0]*ik[2][1][0])+(
      cnk[2][0][1]*ik[2][1][1])));
    ikcnkt[2][1][1] = ((cnk[2][1][2]*ik[2][1][2])+((cnk[2][1][0]*ik[2][1][0])+(
      cnk[2][1][1]*ik[2][1][1])));
    ikcnkt[2][1][2] = ((cnk[2][2][2]*ik[2][1][2])+((cnk[2][2][0]*ik[2][1][0])+(
      cnk[2][2][1]*ik[2][1][1])));
    ikcnkt[2][2][0] = ((cnk[2][0][2]*ik[2][2][2])+((cnk[2][0][0]*ik[2][2][0])+(
      cnk[2][0][1]*ik[2][2][1])));
    ikcnkt[2][2][1] = ((cnk[2][1][2]*ik[2][2][2])+((cnk[2][1][0]*ik[2][2][0])+(
      cnk[2][1][1]*ik[2][2][1])));
    ikcnkt[2][2][2] = ((cnk[2][2][2]*ik[2][2][2])+((cnk[2][2][0]*ik[2][2][0])+(
      cnk[2][2][1]*ik[2][2][1])));
    ikcnkt[3][0][0] = ((cnk[3][0][2]*ik[3][0][2])+((cnk[3][0][0]*ik[3][0][0])+(
      cnk[3][0][1]*ik[3][0][1])));
    ikcnkt[3][0][1] = ((cnk[3][1][2]*ik[3][0][2])+((cnk[3][1][0]*ik[3][0][0])+(
      cnk[3][1][1]*ik[3][0][1])));
    ikcnkt[3][0][2] = ((cnk[3][2][2]*ik[3][0][2])+((cnk[3][2][0]*ik[3][0][0])+(
      cnk[3][2][1]*ik[3][0][1])));
    ikcnkt[3][1][0] = ((cnk[3][0][2]*ik[3][1][2])+((cnk[3][0][0]*ik[3][1][0])+(
      cnk[3][0][1]*ik[3][1][1])));
    ikcnkt[3][1][1] = ((cnk[3][1][2]*ik[3][1][2])+((cnk[3][1][0]*ik[3][1][0])+(
      cnk[3][1][1]*ik[3][1][1])));
    ikcnkt[3][1][2] = ((cnk[3][2][2]*ik[3][1][2])+((cnk[3][2][0]*ik[3][1][0])+(
      cnk[3][2][1]*ik[3][1][1])));
    ikcnkt[3][2][0] = ((cnk[3][0][2]*ik[3][2][2])+((cnk[3][0][0]*ik[3][2][0])+(
      cnk[3][0][1]*ik[3][2][1])));
    ikcnkt[3][2][1] = ((cnk[3][1][2]*ik[3][2][2])+((cnk[3][1][0]*ik[3][2][0])+(
      cnk[3][1][1]*ik[3][2][1])));
    ikcnkt[3][2][2] = ((cnk[3][2][2]*ik[3][2][2])+((cnk[3][2][0]*ik[3][2][0])+(
      cnk[3][2][1]*ik[3][2][1])));
    ikcnkt[4][0][0] = ((cnk[4][0][2]*ik[4][0][2])+((cnk[4][0][0]*ik[4][0][0])+(
      cnk[4][0][1]*ik[4][0][1])));
    ikcnkt[4][0][1] = ((cnk[4][1][2]*ik[4][0][2])+((cnk[4][1][0]*ik[4][0][0])+(
      cnk[4][1][1]*ik[4][0][1])));
    ikcnkt[4][0][2] = ((cnk[4][2][2]*ik[4][0][2])+((cnk[4][2][0]*ik[4][0][0])+(
      cnk[4][2][1]*ik[4][0][1])));
    ikcnkt[4][1][0] = ((cnk[4][0][2]*ik[4][1][2])+((cnk[4][0][0]*ik[4][1][0])+(
      cnk[4][0][1]*ik[4][1][1])));
    ikcnkt[4][1][1] = ((cnk[4][1][2]*ik[4][1][2])+((cnk[4][1][0]*ik[4][1][0])+(
      cnk[4][1][1]*ik[4][1][1])));
    ikcnkt[4][1][2] = ((cnk[4][2][2]*ik[4][1][2])+((cnk[4][2][0]*ik[4][1][0])+(
      cnk[4][2][1]*ik[4][1][1])));
    ikcnkt[4][2][0] = ((cnk[4][0][2]*ik[4][2][2])+((cnk[4][0][0]*ik[4][2][0])+(
      cnk[4][0][1]*ik[4][2][1])));
    ikcnkt[4][2][1] = ((cnk[4][1][2]*ik[4][2][2])+((cnk[4][1][0]*ik[4][2][0])+(
      cnk[4][1][1]*ik[4][2][1])));
    ikcnkt[4][2][2] = ((cnk[4][2][2]*ik[4][2][2])+((cnk[4][2][0]*ik[4][2][0])+(
      cnk[4][2][1]*ik[4][2][1])));
    ikcnkt[5][0][0] = ((cnk[5][0][2]*ik[5][0][2])+((cnk[5][0][0]*ik[5][0][0])+(
      cnk[5][0][1]*ik[5][0][1])));
    ikcnkt[5][0][1] = ((cnk[5][1][2]*ik[5][0][2])+((cnk[5][1][0]*ik[5][0][0])+(
      cnk[5][1][1]*ik[5][0][1])));
    ikcnkt[5][0][2] = ((cnk[5][2][2]*ik[5][0][2])+((cnk[5][2][0]*ik[5][0][0])+(
      cnk[5][2][1]*ik[5][0][1])));
    ikcnkt[5][1][0] = ((cnk[5][0][2]*ik[5][1][2])+((cnk[5][0][0]*ik[5][1][0])+(
      cnk[5][0][1]*ik[5][1][1])));
    ikcnkt[5][1][1] = ((cnk[5][1][2]*ik[5][1][2])+((cnk[5][1][0]*ik[5][1][0])+(
      cnk[5][1][1]*ik[5][1][1])));
    ikcnkt[5][1][2] = ((cnk[5][2][2]*ik[5][1][2])+((cnk[5][2][0]*ik[5][1][0])+(
      cnk[5][2][1]*ik[5][1][1])));
    ikcnkt[5][2][0] = ((cnk[5][0][2]*ik[5][2][2])+((cnk[5][0][0]*ik[5][2][0])+(
      cnk[5][0][1]*ik[5][2][1])));
    ikcnkt[5][2][1] = ((cnk[5][1][2]*ik[5][2][2])+((cnk[5][1][0]*ik[5][2][0])+(
      cnk[5][1][1]*ik[5][2][1])));
    ikcnkt[5][2][2] = ((cnk[5][2][2]*ik[5][2][2])+((cnk[5][2][0]*ik[5][2][0])+(
      cnk[5][2][1]*ik[5][2][1])));
    ikcnkt[6][0][0] = ((cnk[6][0][2]*ik[6][0][2])+((cnk[6][0][0]*ik[6][0][0])+(
      cnk[6][0][1]*ik[6][0][1])));
    ikcnkt[6][0][1] = ((cnk[6][1][2]*ik[6][0][2])+((cnk[6][1][0]*ik[6][0][0])+(
      cnk[6][1][1]*ik[6][0][1])));
    ikcnkt[6][0][2] = ((cnk[6][2][2]*ik[6][0][2])+((cnk[6][2][0]*ik[6][0][0])+(
      cnk[6][2][1]*ik[6][0][1])));
    ikcnkt[6][1][0] = ((cnk[6][0][2]*ik[6][1][2])+((cnk[6][0][0]*ik[6][1][0])+(
      cnk[6][0][1]*ik[6][1][1])));
    ikcnkt[6][1][1] = ((cnk[6][1][2]*ik[6][1][2])+((cnk[6][1][0]*ik[6][1][0])+(
      cnk[6][1][1]*ik[6][1][1])));
    ikcnkt[6][1][2] = ((cnk[6][2][2]*ik[6][1][2])+((cnk[6][2][0]*ik[6][1][0])+(
      cnk[6][2][1]*ik[6][1][1])));
    ikcnkt[6][2][0] = ((cnk[6][0][2]*ik[6][2][2])+((cnk[6][0][0]*ik[6][2][0])+(
      cnk[6][0][1]*ik[6][2][1])));
    ikcnkt[6][2][1] = ((cnk[6][1][2]*ik[6][2][2])+((cnk[6][1][0]*ik[6][2][0])+(
      cnk[6][1][1]*ik[6][2][1])));
    ikcnkt[6][2][2] = ((cnk[6][2][2]*ik[6][2][2])+((cnk[6][2][0]*ik[6][2][0])+(
      cnk[6][2][1]*ik[6][2][1])));
    ikcnkt[7][0][0] = ((cnk[7][0][2]*ik[7][0][2])+((cnk[7][0][0]*ik[7][0][0])+(
      cnk[7][0][1]*ik[7][0][1])));
    ikcnkt[7][0][1] = ((cnk[7][1][2]*ik[7][0][2])+((cnk[7][1][0]*ik[7][0][0])+(
      cnk[7][1][1]*ik[7][0][1])));
    ikcnkt[7][0][2] = ((cnk[7][2][2]*ik[7][0][2])+((cnk[7][2][0]*ik[7][0][0])+(
      cnk[7][2][1]*ik[7][0][1])));
    ikcnkt[7][1][0] = ((cnk[7][0][2]*ik[7][1][2])+((cnk[7][0][0]*ik[7][1][0])+(
      cnk[7][0][1]*ik[7][1][1])));
    ikcnkt[7][1][1] = ((cnk[7][1][2]*ik[7][1][2])+((cnk[7][1][0]*ik[7][1][0])+(
      cnk[7][1][1]*ik[7][1][1])));
    ikcnkt[7][1][2] = ((cnk[7][2][2]*ik[7][1][2])+((cnk[7][2][0]*ik[7][1][0])+(
      cnk[7][2][1]*ik[7][1][1])));
    ikcnkt[7][2][0] = ((cnk[7][0][2]*ik[7][2][2])+((cnk[7][0][0]*ik[7][2][0])+(
      cnk[7][0][1]*ik[7][2][1])));
    ikcnkt[7][2][1] = ((cnk[7][1][2]*ik[7][2][2])+((cnk[7][1][0]*ik[7][2][0])+(
      cnk[7][1][1]*ik[7][2][1])));
    ikcnkt[7][2][2] = ((cnk[7][2][2]*ik[7][2][2])+((cnk[7][2][0]*ik[7][2][0])+(
      cnk[7][2][1]*ik[7][2][1])));
    ikcnkt[8][0][0] = ((cnk[8][0][2]*ik[8][0][2])+((cnk[8][0][0]*ik[8][0][0])+(
      cnk[8][0][1]*ik[8][0][1])));
    ikcnkt[8][0][1] = ((cnk[8][1][2]*ik[8][0][2])+((cnk[8][1][0]*ik[8][0][0])+(
      cnk[8][1][1]*ik[8][0][1])));
    ikcnkt[8][0][2] = ((cnk[8][2][2]*ik[8][0][2])+((cnk[8][2][0]*ik[8][0][0])+(
      cnk[8][2][1]*ik[8][0][1])));
    ikcnkt[8][1][0] = ((cnk[8][0][2]*ik[8][1][2])+((cnk[8][0][0]*ik[8][1][0])+(
      cnk[8][0][1]*ik[8][1][1])));
    ikcnkt[8][1][1] = ((cnk[8][1][2]*ik[8][1][2])+((cnk[8][1][0]*ik[8][1][0])+(
      cnk[8][1][1]*ik[8][1][1])));
    ikcnkt[8][1][2] = ((cnk[8][2][2]*ik[8][1][2])+((cnk[8][2][0]*ik[8][1][0])+(
      cnk[8][2][1]*ik[8][1][1])));
    ikcnkt[8][2][0] = ((cnk[8][0][2]*ik[8][2][2])+((cnk[8][0][0]*ik[8][2][0])+(
      cnk[8][0][1]*ik[8][2][1])));
    ikcnkt[8][2][1] = ((cnk[8][1][2]*ik[8][2][2])+((cnk[8][1][0]*ik[8][2][0])+(
      cnk[8][1][1]*ik[8][2][1])));
    ikcnkt[8][2][2] = ((cnk[8][2][2]*ik[8][2][2])+((cnk[8][2][0]*ik[8][2][0])+(
      cnk[8][2][1]*ik[8][2][1])));
    ikcnkt[9][0][0] = ((cnk[9][0][2]*ik[9][0][2])+((cnk[9][0][0]*ik[9][0][0])+(
      cnk[9][0][1]*ik[9][0][1])));
    ikcnkt[9][0][1] = ((cnk[9][1][2]*ik[9][0][2])+((cnk[9][1][0]*ik[9][0][0])+(
      cnk[9][1][1]*ik[9][0][1])));
    ikcnkt[9][0][2] = ((cnk[9][2][2]*ik[9][0][2])+((cnk[9][2][0]*ik[9][0][0])+(
      cnk[9][2][1]*ik[9][0][1])));
    ikcnkt[9][1][0] = ((cnk[9][0][2]*ik[9][1][2])+((cnk[9][0][0]*ik[9][1][0])+(
      cnk[9][0][1]*ik[9][1][1])));
    ikcnkt[9][1][1] = ((cnk[9][1][2]*ik[9][1][2])+((cnk[9][1][0]*ik[9][1][0])+(
      cnk[9][1][1]*ik[9][1][1])));
    ikcnkt[9][1][2] = ((cnk[9][2][2]*ik[9][1][2])+((cnk[9][2][0]*ik[9][1][0])+(
      cnk[9][2][1]*ik[9][1][1])));
    ikcnkt[9][2][0] = ((cnk[9][0][2]*ik[9][2][2])+((cnk[9][0][0]*ik[9][2][0])+(
      cnk[9][0][1]*ik[9][2][1])));
    ikcnkt[9][2][1] = ((cnk[9][1][2]*ik[9][2][2])+((cnk[9][1][0]*ik[9][2][0])+(
      cnk[9][1][1]*ik[9][2][1])));
    ikcnkt[9][2][2] = ((cnk[9][2][2]*ik[9][2][2])+((cnk[9][2][0]*ik[9][2][0])+(
      cnk[9][2][1]*ik[9][2][1])));
    ikcnkt[10][0][0] = ((cnk[10][0][2]*ik[10][0][2])+((cnk[10][0][0]*
      ik[10][0][0])+(cnk[10][0][1]*ik[10][0][1])));
    ikcnkt[10][0][1] = ((cnk[10][1][2]*ik[10][0][2])+((cnk[10][1][0]*
      ik[10][0][0])+(cnk[10][1][1]*ik[10][0][1])));
    ikcnkt[10][0][2] = ((cnk[10][2][2]*ik[10][0][2])+((cnk[10][2][0]*
      ik[10][0][0])+(cnk[10][2][1]*ik[10][0][1])));
    ikcnkt[10][1][0] = ((cnk[10][0][2]*ik[10][1][2])+((cnk[10][0][0]*
      ik[10][1][0])+(cnk[10][0][1]*ik[10][1][1])));
    ikcnkt[10][1][1] = ((cnk[10][1][2]*ik[10][1][2])+((cnk[10][1][0]*
      ik[10][1][0])+(cnk[10][1][1]*ik[10][1][1])));
    ikcnkt[10][1][2] = ((cnk[10][2][2]*ik[10][1][2])+((cnk[10][2][0]*
      ik[10][1][0])+(cnk[10][2][1]*ik[10][1][1])));
    ikcnkt[10][2][0] = ((cnk[10][0][2]*ik[10][2][2])+((cnk[10][0][0]*
      ik[10][2][0])+(cnk[10][0][1]*ik[10][2][1])));
    ikcnkt[10][2][1] = ((cnk[10][1][2]*ik[10][2][2])+((cnk[10][1][0]*
      ik[10][2][0])+(cnk[10][1][1]*ik[10][2][1])));
    ikcnkt[10][2][2] = ((cnk[10][2][2]*ik[10][2][2])+((cnk[10][2][0]*
      ik[10][2][0])+(cnk[10][2][1]*ik[10][2][1])));
    ikcnkt[11][0][0] = ((cnk[11][0][2]*ik[11][0][2])+((cnk[11][0][0]*
      ik[11][0][0])+(cnk[11][0][1]*ik[11][0][1])));
    ikcnkt[11][0][1] = ((cnk[11][1][2]*ik[11][0][2])+((cnk[11][1][0]*
      ik[11][0][0])+(cnk[11][1][1]*ik[11][0][1])));
    ikcnkt[11][0][2] = ((cnk[11][2][2]*ik[11][0][2])+((cnk[11][2][0]*
      ik[11][0][0])+(cnk[11][2][1]*ik[11][0][1])));
    ikcnkt[11][1][0] = ((cnk[11][0][2]*ik[11][1][2])+((cnk[11][0][0]*
      ik[11][1][0])+(cnk[11][0][1]*ik[11][1][1])));
    ikcnkt[11][1][1] = ((cnk[11][1][2]*ik[11][1][2])+((cnk[11][1][0]*
      ik[11][1][0])+(cnk[11][1][1]*ik[11][1][1])));
    ikcnkt[11][1][2] = ((cnk[11][2][2]*ik[11][1][2])+((cnk[11][2][0]*
      ik[11][1][0])+(cnk[11][2][1]*ik[11][1][1])));
    ikcnkt[11][2][0] = ((cnk[11][0][2]*ik[11][2][2])+((cnk[11][0][0]*
      ik[11][2][0])+(cnk[11][0][1]*ik[11][2][1])));
    ikcnkt[11][2][1] = ((cnk[11][1][2]*ik[11][2][2])+((cnk[11][1][0]*
      ik[11][2][0])+(cnk[11][1][1]*ik[11][2][1])));
    ikcnkt[11][2][2] = ((cnk[11][2][2]*ik[11][2][2])+((cnk[11][2][0]*
      ik[11][2][0])+(cnk[11][2][1]*ik[11][2][1])));
    ikcnkt[12][0][0] = ((cnk[12][0][2]*ik[12][0][2])+((cnk[12][0][0]*
      ik[12][0][0])+(cnk[12][0][1]*ik[12][0][1])));
    ikcnkt[12][0][1] = ((cnk[12][1][2]*ik[12][0][2])+((cnk[12][1][0]*
      ik[12][0][0])+(cnk[12][1][1]*ik[12][0][1])));
    ikcnkt[12][0][2] = ((cnk[12][2][2]*ik[12][0][2])+((cnk[12][2][0]*
      ik[12][0][0])+(cnk[12][2][1]*ik[12][0][1])));
    ikcnkt[12][1][0] = ((cnk[12][0][2]*ik[12][1][2])+((cnk[12][0][0]*
      ik[12][1][0])+(cnk[12][0][1]*ik[12][1][1])));
    ikcnkt[12][1][1] = ((cnk[12][1][2]*ik[12][1][2])+((cnk[12][1][0]*
      ik[12][1][0])+(cnk[12][1][1]*ik[12][1][1])));
    ikcnkt[12][1][2] = ((cnk[12][2][2]*ik[12][1][2])+((cnk[12][2][0]*
      ik[12][1][0])+(cnk[12][2][1]*ik[12][1][1])));
    ikcnkt[12][2][0] = ((cnk[12][0][2]*ik[12][2][2])+((cnk[12][0][0]*
      ik[12][2][0])+(cnk[12][0][1]*ik[12][2][1])));
    ikcnkt[12][2][1] = ((cnk[12][1][2]*ik[12][2][2])+((cnk[12][1][0]*
      ik[12][2][0])+(cnk[12][1][1]*ik[12][2][1])));
    ikcnkt[12][2][2] = ((cnk[12][2][2]*ik[12][2][2])+((cnk[12][2][0]*
      ik[12][2][0])+(cnk[12][2][1]*ik[12][2][1])));
    ikcnkt[13][0][0] = ((cnk[13][0][2]*ik[13][0][2])+((cnk[13][0][0]*
      ik[13][0][0])+(cnk[13][0][1]*ik[13][0][1])));
    ikcnkt[13][0][1] = ((cnk[13][1][2]*ik[13][0][2])+((cnk[13][1][0]*
      ik[13][0][0])+(cnk[13][1][1]*ik[13][0][1])));
    ikcnkt[13][0][2] = ((cnk[13][2][2]*ik[13][0][2])+((cnk[13][2][0]*
      ik[13][0][0])+(cnk[13][2][1]*ik[13][0][1])));
    ikcnkt[13][1][0] = ((cnk[13][0][2]*ik[13][1][2])+((cnk[13][0][0]*
      ik[13][1][0])+(cnk[13][0][1]*ik[13][1][1])));
    ikcnkt[13][1][1] = ((cnk[13][1][2]*ik[13][1][2])+((cnk[13][1][0]*
      ik[13][1][0])+(cnk[13][1][1]*ik[13][1][1])));
    ikcnkt[13][1][2] = ((cnk[13][2][2]*ik[13][1][2])+((cnk[13][2][0]*
      ik[13][1][0])+(cnk[13][2][1]*ik[13][1][1])));
    ikcnkt[13][2][0] = ((cnk[13][0][2]*ik[13][2][2])+((cnk[13][0][0]*
      ik[13][2][0])+(cnk[13][0][1]*ik[13][2][1])));
    ikcnkt[13][2][1] = ((cnk[13][1][2]*ik[13][2][2])+((cnk[13][1][0]*
      ik[13][2][0])+(cnk[13][1][1]*ik[13][2][1])));
    ikcnkt[13][2][2] = ((cnk[13][2][2]*ik[13][2][2])+((cnk[13][2][0]*
      ik[13][2][0])+(cnk[13][2][1]*ik[13][2][1])));
    temp[0] = (((mk[0]*((rnk[0][1]*rnk[0][1])+(rnk[0][2]*rnk[0][2])))+((
      Cik[0][0][2]*ikcnkt[0][2][0])+((Cik[0][0][0]*ikcnkt[0][0][0])+(
      Cik[0][0][1]*ikcnkt[0][1][0]))))+((mk[1]*((rnk[1][1]*rnk[1][1])+(rnk[1][2]
      *rnk[1][2])))+((cnk[1][0][2]*ikcnkt[1][2][0])+((cnk[1][0][0]*
      ikcnkt[1][0][0])+(cnk[1][0][1]*ikcnkt[1][1][0])))));
    temp[1] = (((mk[3]*((rnk[3][1]*rnk[3][1])+(rnk[3][2]*rnk[3][2])))+((
      cnk[3][0][2]*ikcnkt[3][2][0])+((cnk[3][0][0]*ikcnkt[3][0][0])+(
      cnk[3][0][1]*ikcnkt[3][1][0]))))+(((mk[2]*((rnk[2][1]*rnk[2][1])+(
      rnk[2][2]*rnk[2][2])))+((cnk[2][0][2]*ikcnkt[2][2][0])+((cnk[2][0][0]*
      ikcnkt[2][0][0])+(cnk[2][0][1]*ikcnkt[2][1][0]))))+temp[0]));
    temp[2] = (((mk[5]*((rnk[5][1]*rnk[5][1])+(rnk[5][2]*rnk[5][2])))+((
      cnk[5][0][2]*ikcnkt[5][2][0])+((cnk[5][0][0]*ikcnkt[5][0][0])+(
      cnk[5][0][1]*ikcnkt[5][1][0]))))+(((mk[4]*((rnk[4][1]*rnk[4][1])+(
      rnk[4][2]*rnk[4][2])))+((cnk[4][0][2]*ikcnkt[4][2][0])+((cnk[4][0][0]*
      ikcnkt[4][0][0])+(cnk[4][0][1]*ikcnkt[4][1][0]))))+temp[1]));
    temp[3] = (((mk[7]*((rnk[7][1]*rnk[7][1])+(rnk[7][2]*rnk[7][2])))+((
      cnk[7][0][2]*ikcnkt[7][2][0])+((cnk[7][0][0]*ikcnkt[7][0][0])+(
      cnk[7][0][1]*ikcnkt[7][1][0]))))+(((mk[6]*((rnk[6][1]*rnk[6][1])+(
      rnk[6][2]*rnk[6][2])))+((cnk[6][0][2]*ikcnkt[6][2][0])+((cnk[6][0][0]*
      ikcnkt[6][0][0])+(cnk[6][0][1]*ikcnkt[6][1][0]))))+temp[2]));
    temp[4] = (((mk[9]*((rnk[9][1]*rnk[9][1])+(rnk[9][2]*rnk[9][2])))+((
      cnk[9][0][2]*ikcnkt[9][2][0])+((cnk[9][0][0]*ikcnkt[9][0][0])+(
      cnk[9][0][1]*ikcnkt[9][1][0]))))+(((mk[8]*((rnk[8][1]*rnk[8][1])+(
      rnk[8][2]*rnk[8][2])))+((cnk[8][0][2]*ikcnkt[8][2][0])+((cnk[8][0][0]*
      ikcnkt[8][0][0])+(cnk[8][0][1]*ikcnkt[8][1][0]))))+temp[3]));
    temp[5] = (((mk[11]*((rnk[11][1]*rnk[11][1])+(rnk[11][2]*rnk[11][2])))+((
      cnk[11][0][2]*ikcnkt[11][2][0])+((cnk[11][0][0]*ikcnkt[11][0][0])+(
      cnk[11][0][1]*ikcnkt[11][1][0]))))+(((mk[10]*((rnk[10][1]*rnk[10][1])+(
      rnk[10][2]*rnk[10][2])))+((cnk[10][0][2]*ikcnkt[10][2][0])+((cnk[10][0][0]
      *ikcnkt[10][0][0])+(cnk[10][0][1]*ikcnkt[10][1][0]))))+temp[4]));
    icm[0][0] = ((((mk[13]*((rnk[13][1]*rnk[13][1])+(rnk[13][2]*rnk[13][2])))+((
      cnk[13][0][2]*ikcnkt[13][2][0])+((cnk[13][0][0]*ikcnkt[13][0][0])+(
      cnk[13][0][1]*ikcnkt[13][1][0]))))+(((mk[12]*((rnk[12][1]*rnk[12][1])+(
      rnk[12][2]*rnk[12][2])))+((cnk[12][0][2]*ikcnkt[12][2][0])+((cnk[12][0][0]
      *ikcnkt[12][0][0])+(cnk[12][0][1]*ikcnkt[12][1][0]))))+temp[5]))-(mtot*((
      com[1]*com[1])+(com[2]*com[2]))));
    temp[0] = ((((cnk[2][0][2]*ikcnkt[2][2][1])+((cnk[2][0][0]*ikcnkt[2][0][1])+
      (cnk[2][0][1]*ikcnkt[2][1][1])))-(mk[2]*(rnk[2][0]*rnk[2][1])))+((((
      Cik[0][0][2]*ikcnkt[0][2][1])+((Cik[0][0][0]*ikcnkt[0][0][1])+(
      Cik[0][0][1]*ikcnkt[0][1][1])))-(mk[0]*(rnk[0][0]*rnk[0][1])))+(((
      cnk[1][0][2]*ikcnkt[1][2][1])+((cnk[1][0][0]*ikcnkt[1][0][1])+(
      cnk[1][0][1]*ikcnkt[1][1][1])))-(mk[1]*(rnk[1][0]*rnk[1][1])))));
    temp[1] = ((((cnk[5][0][2]*ikcnkt[5][2][1])+((cnk[5][0][0]*ikcnkt[5][0][1])+
      (cnk[5][0][1]*ikcnkt[5][1][1])))-(mk[5]*(rnk[5][0]*rnk[5][1])))+((((
      cnk[4][0][2]*ikcnkt[4][2][1])+((cnk[4][0][0]*ikcnkt[4][0][1])+(
      cnk[4][0][1]*ikcnkt[4][1][1])))-(mk[4]*(rnk[4][0]*rnk[4][1])))+((((
      cnk[3][0][2]*ikcnkt[3][2][1])+((cnk[3][0][0]*ikcnkt[3][0][1])+(
      cnk[3][0][1]*ikcnkt[3][1][1])))-(mk[3]*(rnk[3][0]*rnk[3][1])))+temp[0])));
    temp[2] = ((((cnk[8][0][2]*ikcnkt[8][2][1])+((cnk[8][0][0]*ikcnkt[8][0][1])+
      (cnk[8][0][1]*ikcnkt[8][1][1])))-(mk[8]*(rnk[8][0]*rnk[8][1])))+((((
      cnk[7][0][2]*ikcnkt[7][2][1])+((cnk[7][0][0]*ikcnkt[7][0][1])+(
      cnk[7][0][1]*ikcnkt[7][1][1])))-(mk[7]*(rnk[7][0]*rnk[7][1])))+((((
      cnk[6][0][2]*ikcnkt[6][2][1])+((cnk[6][0][0]*ikcnkt[6][0][1])+(
      cnk[6][0][1]*ikcnkt[6][1][1])))-(mk[6]*(rnk[6][0]*rnk[6][1])))+temp[1])));
    temp[3] = ((((cnk[11][0][2]*ikcnkt[11][2][1])+((cnk[11][0][0]*
      ikcnkt[11][0][1])+(cnk[11][0][1]*ikcnkt[11][1][1])))-(mk[11]*(rnk[11][0]*
      rnk[11][1])))+((((cnk[10][0][2]*ikcnkt[10][2][1])+((cnk[10][0][0]*
      ikcnkt[10][0][1])+(cnk[10][0][1]*ikcnkt[10][1][1])))-(mk[10]*(rnk[10][0]*
      rnk[10][1])))+((((cnk[9][0][2]*ikcnkt[9][2][1])+((cnk[9][0][0]*
      ikcnkt[9][0][1])+(cnk[9][0][1]*ikcnkt[9][1][1])))-(mk[9]*(rnk[9][0]*
      rnk[9][1])))+temp[2])));
    icm[0][1] = ((mtot*(com[0]*com[1]))+((((cnk[13][0][2]*ikcnkt[13][2][1])+((
      cnk[13][0][0]*ikcnkt[13][0][1])+(cnk[13][0][1]*ikcnkt[13][1][1])))-(mk[13]
      *(rnk[13][0]*rnk[13][1])))+((((cnk[12][0][2]*ikcnkt[12][2][1])+((
      cnk[12][0][0]*ikcnkt[12][0][1])+(cnk[12][0][1]*ikcnkt[12][1][1])))-(mk[12]
      *(rnk[12][0]*rnk[12][1])))+temp[3])));
    temp[0] = ((((cnk[2][0][2]*ikcnkt[2][2][2])+((cnk[2][0][0]*ikcnkt[2][0][2])+
      (cnk[2][0][1]*ikcnkt[2][1][2])))-(mk[2]*(rnk[2][0]*rnk[2][2])))+((((
      Cik[0][0][2]*ikcnkt[0][2][2])+((Cik[0][0][0]*ikcnkt[0][0][2])+(
      Cik[0][0][1]*ikcnkt[0][1][2])))-(mk[0]*(rnk[0][0]*rnk[0][2])))+(((
      cnk[1][0][2]*ikcnkt[1][2][2])+((cnk[1][0][0]*ikcnkt[1][0][2])+(
      cnk[1][0][1]*ikcnkt[1][1][2])))-(mk[1]*(rnk[1][0]*rnk[1][2])))));
    temp[1] = ((((cnk[5][0][2]*ikcnkt[5][2][2])+((cnk[5][0][0]*ikcnkt[5][0][2])+
      (cnk[5][0][1]*ikcnkt[5][1][2])))-(mk[5]*(rnk[5][0]*rnk[5][2])))+((((
      cnk[4][0][2]*ikcnkt[4][2][2])+((cnk[4][0][0]*ikcnkt[4][0][2])+(
      cnk[4][0][1]*ikcnkt[4][1][2])))-(mk[4]*(rnk[4][0]*rnk[4][2])))+((((
      cnk[3][0][2]*ikcnkt[3][2][2])+((cnk[3][0][0]*ikcnkt[3][0][2])+(
      cnk[3][0][1]*ikcnkt[3][1][2])))-(mk[3]*(rnk[3][0]*rnk[3][2])))+temp[0])));
    temp[2] = ((((cnk[8][0][2]*ikcnkt[8][2][2])+((cnk[8][0][0]*ikcnkt[8][0][2])+
      (cnk[8][0][1]*ikcnkt[8][1][2])))-(mk[8]*(rnk[8][0]*rnk[8][2])))+((((
      cnk[7][0][2]*ikcnkt[7][2][2])+((cnk[7][0][0]*ikcnkt[7][0][2])+(
      cnk[7][0][1]*ikcnkt[7][1][2])))-(mk[7]*(rnk[7][0]*rnk[7][2])))+((((
      cnk[6][0][2]*ikcnkt[6][2][2])+((cnk[6][0][0]*ikcnkt[6][0][2])+(
      cnk[6][0][1]*ikcnkt[6][1][2])))-(mk[6]*(rnk[6][0]*rnk[6][2])))+temp[1])));
    temp[3] = ((((cnk[11][0][2]*ikcnkt[11][2][2])+((cnk[11][0][0]*
      ikcnkt[11][0][2])+(cnk[11][0][1]*ikcnkt[11][1][2])))-(mk[11]*(rnk[11][0]*
      rnk[11][2])))+((((cnk[10][0][2]*ikcnkt[10][2][2])+((cnk[10][0][0]*
      ikcnkt[10][0][2])+(cnk[10][0][1]*ikcnkt[10][1][2])))-(mk[10]*(rnk[10][0]*
      rnk[10][2])))+((((cnk[9][0][2]*ikcnkt[9][2][2])+((cnk[9][0][0]*
      ikcnkt[9][0][2])+(cnk[9][0][1]*ikcnkt[9][1][2])))-(mk[9]*(rnk[9][0]*
      rnk[9][2])))+temp[2])));
    icm[0][2] = ((mtot*(com[0]*com[2]))+((((cnk[13][0][2]*ikcnkt[13][2][2])+((
      cnk[13][0][0]*ikcnkt[13][0][2])+(cnk[13][0][1]*ikcnkt[13][1][2])))-(mk[13]
      *(rnk[13][0]*rnk[13][2])))+((((cnk[12][0][2]*ikcnkt[12][2][2])+((
      cnk[12][0][0]*ikcnkt[12][0][2])+(cnk[12][0][1]*ikcnkt[12][1][2])))-(mk[12]
      *(rnk[12][0]*rnk[12][2])))+temp[3])));
    icm[1][0] = icm[0][1];
    temp[0] = (((mk[0]*((rnk[0][0]*rnk[0][0])+(rnk[0][2]*rnk[0][2])))+((
      Cik[0][1][2]*ikcnkt[0][2][1])+((Cik[0][1][0]*ikcnkt[0][0][1])+(
      Cik[0][1][1]*ikcnkt[0][1][1]))))+((mk[1]*((rnk[1][0]*rnk[1][0])+(rnk[1][2]
      *rnk[1][2])))+((cnk[1][1][2]*ikcnkt[1][2][1])+((cnk[1][1][0]*
      ikcnkt[1][0][1])+(cnk[1][1][1]*ikcnkt[1][1][1])))));
    temp[1] = (((mk[3]*((rnk[3][0]*rnk[3][0])+(rnk[3][2]*rnk[3][2])))+((
      cnk[3][1][2]*ikcnkt[3][2][1])+((cnk[3][1][0]*ikcnkt[3][0][1])+(
      cnk[3][1][1]*ikcnkt[3][1][1]))))+(((mk[2]*((rnk[2][0]*rnk[2][0])+(
      rnk[2][2]*rnk[2][2])))+((cnk[2][1][2]*ikcnkt[2][2][1])+((cnk[2][1][0]*
      ikcnkt[2][0][1])+(cnk[2][1][1]*ikcnkt[2][1][1]))))+temp[0]));
    temp[2] = (((mk[5]*((rnk[5][0]*rnk[5][0])+(rnk[5][2]*rnk[5][2])))+((
      cnk[5][1][2]*ikcnkt[5][2][1])+((cnk[5][1][0]*ikcnkt[5][0][1])+(
      cnk[5][1][1]*ikcnkt[5][1][1]))))+(((mk[4]*((rnk[4][0]*rnk[4][0])+(
      rnk[4][2]*rnk[4][2])))+((cnk[4][1][2]*ikcnkt[4][2][1])+((cnk[4][1][0]*
      ikcnkt[4][0][1])+(cnk[4][1][1]*ikcnkt[4][1][1]))))+temp[1]));
    temp[3] = (((mk[7]*((rnk[7][0]*rnk[7][0])+(rnk[7][2]*rnk[7][2])))+((
      cnk[7][1][2]*ikcnkt[7][2][1])+((cnk[7][1][0]*ikcnkt[7][0][1])+(
      cnk[7][1][1]*ikcnkt[7][1][1]))))+(((mk[6]*((rnk[6][0]*rnk[6][0])+(
      rnk[6][2]*rnk[6][2])))+((cnk[6][1][2]*ikcnkt[6][2][1])+((cnk[6][1][0]*
      ikcnkt[6][0][1])+(cnk[6][1][1]*ikcnkt[6][1][1]))))+temp[2]));
    temp[4] = (((mk[9]*((rnk[9][0]*rnk[9][0])+(rnk[9][2]*rnk[9][2])))+((
      cnk[9][1][2]*ikcnkt[9][2][1])+((cnk[9][1][0]*ikcnkt[9][0][1])+(
      cnk[9][1][1]*ikcnkt[9][1][1]))))+(((mk[8]*((rnk[8][0]*rnk[8][0])+(
      rnk[8][2]*rnk[8][2])))+((cnk[8][1][2]*ikcnkt[8][2][1])+((cnk[8][1][0]*
      ikcnkt[8][0][1])+(cnk[8][1][1]*ikcnkt[8][1][1]))))+temp[3]));
    temp[5] = (((mk[11]*((rnk[11][0]*rnk[11][0])+(rnk[11][2]*rnk[11][2])))+((
      cnk[11][1][2]*ikcnkt[11][2][1])+((cnk[11][1][0]*ikcnkt[11][0][1])+(
      cnk[11][1][1]*ikcnkt[11][1][1]))))+(((mk[10]*((rnk[10][0]*rnk[10][0])+(
      rnk[10][2]*rnk[10][2])))+((cnk[10][1][2]*ikcnkt[10][2][1])+((cnk[10][1][0]
      *ikcnkt[10][0][1])+(cnk[10][1][1]*ikcnkt[10][1][1]))))+temp[4]));
    icm[1][1] = ((((mk[13]*((rnk[13][0]*rnk[13][0])+(rnk[13][2]*rnk[13][2])))+((
      cnk[13][1][2]*ikcnkt[13][2][1])+((cnk[13][1][0]*ikcnkt[13][0][1])+(
      cnk[13][1][1]*ikcnkt[13][1][1]))))+(((mk[12]*((rnk[12][0]*rnk[12][0])+(
      rnk[12][2]*rnk[12][2])))+((cnk[12][1][2]*ikcnkt[12][2][1])+((cnk[12][1][0]
      *ikcnkt[12][0][1])+(cnk[12][1][1]*ikcnkt[12][1][1]))))+temp[5]))-(mtot*((
      com[0]*com[0])+(com[2]*com[2]))));
    temp[0] = ((((cnk[2][1][2]*ikcnkt[2][2][2])+((cnk[2][1][0]*ikcnkt[2][0][2])+
      (cnk[2][1][1]*ikcnkt[2][1][2])))-(mk[2]*(rnk[2][1]*rnk[2][2])))+((((
      Cik[0][1][2]*ikcnkt[0][2][2])+((Cik[0][1][0]*ikcnkt[0][0][2])+(
      Cik[0][1][1]*ikcnkt[0][1][2])))-(mk[0]*(rnk[0][1]*rnk[0][2])))+(((
      cnk[1][1][2]*ikcnkt[1][2][2])+((cnk[1][1][0]*ikcnkt[1][0][2])+(
      cnk[1][1][1]*ikcnkt[1][1][2])))-(mk[1]*(rnk[1][1]*rnk[1][2])))));
    temp[1] = ((((cnk[5][1][2]*ikcnkt[5][2][2])+((cnk[5][1][0]*ikcnkt[5][0][2])+
      (cnk[5][1][1]*ikcnkt[5][1][2])))-(mk[5]*(rnk[5][1]*rnk[5][2])))+((((
      cnk[4][1][2]*ikcnkt[4][2][2])+((cnk[4][1][0]*ikcnkt[4][0][2])+(
      cnk[4][1][1]*ikcnkt[4][1][2])))-(mk[4]*(rnk[4][1]*rnk[4][2])))+((((
      cnk[3][1][2]*ikcnkt[3][2][2])+((cnk[3][1][0]*ikcnkt[3][0][2])+(
      cnk[3][1][1]*ikcnkt[3][1][2])))-(mk[3]*(rnk[3][1]*rnk[3][2])))+temp[0])));
    temp[2] = ((((cnk[8][1][2]*ikcnkt[8][2][2])+((cnk[8][1][0]*ikcnkt[8][0][2])+
      (cnk[8][1][1]*ikcnkt[8][1][2])))-(mk[8]*(rnk[8][1]*rnk[8][2])))+((((
      cnk[7][1][2]*ikcnkt[7][2][2])+((cnk[7][1][0]*ikcnkt[7][0][2])+(
      cnk[7][1][1]*ikcnkt[7][1][2])))-(mk[7]*(rnk[7][1]*rnk[7][2])))+((((
      cnk[6][1][2]*ikcnkt[6][2][2])+((cnk[6][1][0]*ikcnkt[6][0][2])+(
      cnk[6][1][1]*ikcnkt[6][1][2])))-(mk[6]*(rnk[6][1]*rnk[6][2])))+temp[1])));
    temp[3] = ((((cnk[11][1][2]*ikcnkt[11][2][2])+((cnk[11][1][0]*
      ikcnkt[11][0][2])+(cnk[11][1][1]*ikcnkt[11][1][2])))-(mk[11]*(rnk[11][1]*
      rnk[11][2])))+((((cnk[10][1][2]*ikcnkt[10][2][2])+((cnk[10][1][0]*
      ikcnkt[10][0][2])+(cnk[10][1][1]*ikcnkt[10][1][2])))-(mk[10]*(rnk[10][1]*
      rnk[10][2])))+((((cnk[9][1][2]*ikcnkt[9][2][2])+((cnk[9][1][0]*
      ikcnkt[9][0][2])+(cnk[9][1][1]*ikcnkt[9][1][2])))-(mk[9]*(rnk[9][1]*
      rnk[9][2])))+temp[2])));
    icm[1][2] = ((mtot*(com[1]*com[2]))+((((cnk[13][1][2]*ikcnkt[13][2][2])+((
      cnk[13][1][0]*ikcnkt[13][0][2])+(cnk[13][1][1]*ikcnkt[13][1][2])))-(mk[13]
      *(rnk[13][1]*rnk[13][2])))+((((cnk[12][1][2]*ikcnkt[12][2][2])+((
      cnk[12][1][0]*ikcnkt[12][0][2])+(cnk[12][1][1]*ikcnkt[12][1][2])))-(mk[12]
      *(rnk[12][1]*rnk[12][2])))+temp[3])));
    icm[2][0] = icm[0][2];
    icm[2][1] = icm[1][2];
    temp[0] = (((mk[0]*((rnk[0][0]*rnk[0][0])+(rnk[0][1]*rnk[0][1])))+((
      Cik[0][2][2]*ikcnkt[0][2][2])+((Cik[0][2][0]*ikcnkt[0][0][2])+(
      Cik[0][2][1]*ikcnkt[0][1][2]))))+((mk[1]*((rnk[1][0]*rnk[1][0])+(rnk[1][1]
      *rnk[1][1])))+((cnk[1][2][2]*ikcnkt[1][2][2])+((cnk[1][2][0]*
      ikcnkt[1][0][2])+(cnk[1][2][1]*ikcnkt[1][1][2])))));
    temp[1] = (((mk[3]*((rnk[3][0]*rnk[3][0])+(rnk[3][1]*rnk[3][1])))+((
      cnk[3][2][2]*ikcnkt[3][2][2])+((cnk[3][2][0]*ikcnkt[3][0][2])+(
      cnk[3][2][1]*ikcnkt[3][1][2]))))+(((mk[2]*((rnk[2][0]*rnk[2][0])+(
      rnk[2][1]*rnk[2][1])))+((cnk[2][2][2]*ikcnkt[2][2][2])+((cnk[2][2][0]*
      ikcnkt[2][0][2])+(cnk[2][2][1]*ikcnkt[2][1][2]))))+temp[0]));
    temp[2] = (((mk[5]*((rnk[5][0]*rnk[5][0])+(rnk[5][1]*rnk[5][1])))+((
      cnk[5][2][2]*ikcnkt[5][2][2])+((cnk[5][2][0]*ikcnkt[5][0][2])+(
      cnk[5][2][1]*ikcnkt[5][1][2]))))+(((mk[4]*((rnk[4][0]*rnk[4][0])+(
      rnk[4][1]*rnk[4][1])))+((cnk[4][2][2]*ikcnkt[4][2][2])+((cnk[4][2][0]*
      ikcnkt[4][0][2])+(cnk[4][2][1]*ikcnkt[4][1][2]))))+temp[1]));
    temp[3] = (((mk[7]*((rnk[7][0]*rnk[7][0])+(rnk[7][1]*rnk[7][1])))+((
      cnk[7][2][2]*ikcnkt[7][2][2])+((cnk[7][2][0]*ikcnkt[7][0][2])+(
      cnk[7][2][1]*ikcnkt[7][1][2]))))+(((mk[6]*((rnk[6][0]*rnk[6][0])+(
      rnk[6][1]*rnk[6][1])))+((cnk[6][2][2]*ikcnkt[6][2][2])+((cnk[6][2][0]*
      ikcnkt[6][0][2])+(cnk[6][2][1]*ikcnkt[6][1][2]))))+temp[2]));
    temp[4] = (((mk[9]*((rnk[9][0]*rnk[9][0])+(rnk[9][1]*rnk[9][1])))+((
      cnk[9][2][2]*ikcnkt[9][2][2])+((cnk[9][2][0]*ikcnkt[9][0][2])+(
      cnk[9][2][1]*ikcnkt[9][1][2]))))+(((mk[8]*((rnk[8][0]*rnk[8][0])+(
      rnk[8][1]*rnk[8][1])))+((cnk[8][2][2]*ikcnkt[8][2][2])+((cnk[8][2][0]*
      ikcnkt[8][0][2])+(cnk[8][2][1]*ikcnkt[8][1][2]))))+temp[3]));
    temp[5] = (((mk[11]*((rnk[11][0]*rnk[11][0])+(rnk[11][1]*rnk[11][1])))+((
      cnk[11][2][2]*ikcnkt[11][2][2])+((cnk[11][2][0]*ikcnkt[11][0][2])+(
      cnk[11][2][1]*ikcnkt[11][1][2]))))+(((mk[10]*((rnk[10][0]*rnk[10][0])+(
      rnk[10][1]*rnk[10][1])))+((cnk[10][2][2]*ikcnkt[10][2][2])+((cnk[10][2][0]
      *ikcnkt[10][0][2])+(cnk[10][2][1]*ikcnkt[10][1][2]))))+temp[4]));
    icm[2][2] = ((((mk[13]*((rnk[13][0]*rnk[13][0])+(rnk[13][1]*rnk[13][1])))+((
      cnk[13][2][2]*ikcnkt[13][2][2])+((cnk[13][2][0]*ikcnkt[13][0][2])+(
      cnk[13][2][1]*ikcnkt[13][1][2]))))+(((mk[12]*((rnk[12][0]*rnk[12][0])+(
      rnk[12][1]*rnk[12][1])))+((cnk[12][2][2]*ikcnkt[12][2][2])+((cnk[12][2][0]
      *ikcnkt[12][0][2])+(cnk[12][2][1]*ikcnkt[12][1][2]))))+temp[5]))-(mtot*((
      com[0]*com[0])+(com[1]*com[1]))));
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain  633 adds/subtracts/negates
                    855 multiplies
                      0 divides
                    169 assignments
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

    if ((coord < 0) || (coord > 13)) {
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
    info[11] = 14;
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
  sdstate:  1626 adds  2211 multiplies     3 divides  1220 assignments
  sdderiv: 11847 adds 12762 multiplies   263 divides 12609 assignments
  sdresid:  2935 adds  3140 multiplies     0 divides  1278 assignments
  sdreac:    648 adds   564 multiplies     0 divides   285 assignments
  sdmom:     419 adds   469 multiplies     0 divides   115 assignments
  sdsys:     633 adds   855 multiplies     0 divides   169 assignments
*/
