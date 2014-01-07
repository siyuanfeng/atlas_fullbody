#include "sdlib.h"
#include "SDModel.h"
#include "SDModel_common.h"
#include "stdlib.h"

#define NQ  m->NQ()
#define NU  m->NU()

void sdtrans6(SDModel*m, int ib, double *ivec, int ob, double *ovec){
  m->sdtrans(ib,ivec,ob,ovec);
  m->sdtrans(ib,&ivec[3],ob,&ovec[3]);
}

void set_state(SDModel *m, double *x){
  if (x!=NULL)	m->sdstate(0.0,&x[0],&x[NQ]);
}

void body_position(SDModel *m, double *q, int body, double *offset, double *p, double *J){
  double dc[3][3];
  double *lin, *rot;
  int i;
  if (q!=NULL){		
    double *x = new double[NQ+NU];
    for(i=0;i<NQ;i++)	x[i] = q[i];
    for(i=NQ;i<NQ+NU;i++)	x[i] = 0.0;
    set_state(m, x);
    delete []x;
  }
  if (p) {
    m->sdpos(body,offset,&p[0]);
    m->sdorient(body,dc);
    sddc2ang(dc,&p[3],&p[4],&p[5]);
  }
  if (J!=NULL){
    for(i=0;i<NU;i++){
      lin = &J[6*i+0];
      rot = &J[6*i+3];
      m->sdrel2cart(i,body,offset,lin,rot);
      m->sdtrans(body,lin,-1,lin);
      m->sdtrans(body,rot,-1,rot);
    }
  }
}

void forward_dynamics(SDModel *m, double *x, double *trq, SDFext_t *fext, int n_fext, double *dstate){
  int i;
  set_state(m, x);
  if (trq!=NULL){
    for(i=0;i<m->NM();i++){
      m->sdhinget(m->joint(i),m->axis(i),trq[i]);
    }
  }
  if (fext!=NULL && n_fext != 0){
    double force[3];
    for(i=0;i<n_fext;i++){
      m->sdtrans(-1,fext[i].force,fext[i].body,force);
      m->sdpointf(fext[i].body,fext[i].pos,force);
      //m->sdbodyt(fext[i].body,fext[i].moment);
    }
  }
  m->sdderiv(&dstate[0],&dstate[NQ]);
}

void linearize_dynamics_continuous(SDModel *m, double *x, double *u, double *A, double *B){
  double delta=1e-8;
  int i,j;
  double *dx0 = new double[NQ+NU];

  forward_dynamics(m,x,u,NULL,0,dx0);

  for(i=0;i<NQ+NU;i++){
    x[i] += delta;        
    forward_dynamics(m,x,u,NULL,0,&A[(NQ+NU)*i]);
    for(j=0;j<NU+NU;j++)  A[(NQ+NU)*i + j] = (A[(NQ+NU)*i + j]-dx0[j])/delta;        
    x[i] -= delta;
  }
  for(i=0;i<m->NM();i++){
    u[i] += delta;        
    forward_dynamics(m,x,u,NULL,0,&B[(NQ+NU)*i]);
    for(j=0;j<NU+NU;j++)  B[(NQ+NU)*i + j] = (B[(NQ+NU)*i + j]-dx0[j])/delta;        
    u[i] -= delta;
  }

  free(dx0);
}

void get_model_params(SDModel *m, double *mass, double *btj, double *I, double *itj, double *pin){
  /* Updates the mass, com offset and inertia of the specified bodies
   * INPUTS:
   *  m - model pointer
   * OUTPUT:
   *  mass - vector of masses corresponding to bodies
   *  btj - vector of btj offsets [x1,y1,z1,x2,y2,z2,....]
   *  I - inertia parameters [Ixx1, Iyy1, Izz1, Iyz1, Ixy1, Ixz1, Ixx2, Iyy2,...]
   * OPTIONAL OUTPUTS:
   *  itj - vector of itj offsets [x1,y1,z1,x2,y2,z2,....]
   *  pin - vector of pin values [inb1,type1,px1,py1,pz1,inb2,type2,px2,...]
   *  
   */    
  int i,j,info[50],slider[6];    
  double iner[3][3];
  for (i=0;i<m->NBOD();i++){
    m->sdgetmass(i,&mass[i]); 
    for(j=0;j<m->NJNT();j++){
      m->sdjnt(j,info,slider);
      if(info[3]==i)  m->sdgetbtj(j,&btj[3*i]);        
    }        
    m->sdgetiner(i,iner);
    I[6*i+0] = iner[0][0];
    I[6*i+1] = iner[1][1];
    I[6*i+2] = iner[2][2];
    I[6*i+3] = iner[1][2];
    I[6*i+4] = iner[0][1];
    I[6*i+5] = iner[0][2];

    if (itj!=NULL){
      m->sdgetitj(i,&itj[3*i]); 
      if (pin!=NULL){
        m->sdjnt(i,info,slider);
        pin[5*i] = info[2];
        pin[5*i+1] = info[0];                
        m->sdgetpin(i,0,&pin[5*i+2]);
      }
    }
  }
}

void set_model_params(SDModel *m, int n, int *bodies, double *mass, double *off, double *I){
  /* Updates the mass, btj offset and inertia of the specified bodies
   * INPUTS:
   *  m - model pointer
   *  n - number of bodies to be updated
   *  bodies - vector of bodies who's parameters are to be updated
   *  mass - vector of masses corresponding to bodies
   *  off - matrix of new btj's [x1,y1,z1,x2,y2,z2,....]
   *  I - inertia parameters [Ixx1, Iyy1, Izz1, Iyz1, Ixy1, Ixz1, Ixx2, Iyy2,...]
   * OUTPUT:
   *  none
   */    
  int i,j,info[50],slider[6];    
  double itj[3],btj[3],iner[3][3],diff[3];
  for (i=0;i<n;i++){
    //printf("%i %f\n",bodies[i],mass[i]);
    if (mass!=NULL){
      m->sdmass(bodies[i],mass[i]);  
    }

    if (off!=NULL){        
      /* check all the joints to see if itj/ibj needs to be changed */
      for(j=0;j<m->NJNT();j++){   // loop through joints
        m->sdjnt(j,info,slider);
        if(info[3]==bodies[i]){     // is the current body the outboard body?
          m->sdgetbtj(j,btj);
          sdvsub(btj,&off[3*i],diff);
          m->sdbtj(j,&off[3*i]);
          break;
        }
      }
      for(j=0;j<m->NJNT();j++){   // loop through the joints again
        m->sdjnt(j,info,slider);
        if(info[2]==bodies[i]){ // is the current body an inboard body?
          m->sdgetitj(j,itj);
          sdvsub(itj,diff,itj);
          m->sditj(j,itj);
        }
      }     
    }

    if (I!=NULL){
      iner[0][0] = I[6*i+0];
      iner[1][1] = I[6*i+1];
      iner[2][2] = I[6*i+2];
      iner[1][2] = iner[2][1] = I[6*i+3];
      iner[1][0] = iner[0][1] = I[6*i+4];
      iner[2][0] = iner[0][2] = I[6*i+5];
      m->sdiner(bodies[i],iner);
    }
  }    
}

void set_model_params_all(SDModel *m, double *mass, double *off, double *I){
  /* Updates the mass, btj offset and inertia of the specified bodies
   * INPUTS:
   *  m - model pointer
   *  mass - vector of masses corresponding to bodies
   *  off - matrix of new btj's [x1,y1,z1,x2,y2,z2,....]
   *  I - inertia parameters [Ixx1, Iyy1, Izz1, Iyz1, Ixy1, Ixz1, Ixx2, Iyy2,...]
   * OUTPUT:
   *  none
   */    
  int i,j,info[50],slider[6];    
  double itj[3],btj[3],iner[3][3],diff[3];
  for (i=0;i<m->NBOD();i++){
    //printf("%i %f\n",bodies[i],mass[i]);
    m->sdmass(i,mass[i]);        

    /* check all the joints to see if itj/ibj needs to be changed */
    for(j=0;j<m->NJNT();j++){   // loop through joints
      m->sdjnt(j,info,slider);
      if(info[3]==i){     // is the current body the outboard body?
        m->sdgetbtj(j,btj);
        sdvsub(btj,&off[3*i],diff);
        m->sdbtj(j,&off[3*i]);
        break;
      }
    }
    for(j=0;j<m->NJNT();j++){   // loop through the joints again
      m->sdjnt(j,info,slider);
      if(info[2]==i){ // is the current body an inboard body?
        m->sdgetitj(j,itj);
        sdvsub(itj,diff,itj);
        m->sditj(j,itj);
      }
    }        
    iner[0][0] = I[6*i+0];
    iner[1][1] = I[6*i+1];
    iner[2][2] = I[6*i+2];
    iner[1][2] = iner[2][1] = I[6*i+3];
    iner[1][0] = iner[0][1] = I[6*i+4];
    iner[2][0] = iner[0][2] = I[6*i+5];
    m->sdiner(i,iner);
  }    
}


void inverse_dynamics(SDModel *m, double *x, double *ddq, double *trq){
  set_state(m, x);
  if (ddq==NULL){
    double *ddq0 = new double[NU];
    m->sdcomptrq(&ddq0[0],&trq[0]);
    delete ddq0;
  }else	m->sdcomptrq(&ddq[0],&trq[0]);
}

void impulse_force(SDModel *m, double *x, int body, double *f, double *x_new){
  double temp[3],zero[3]={0};
  double *G = new double[NU];
  double *dstate = new double[NQ+NU];
  int i;
  m->sdstate(0.0,x,m->u_zero);
  m->sdtrans(-1,f,body,temp);
  m->sdequivht(G);
  for(i=0;i<NU;i++)	G[i] *= -1;
  m->sdpointf(body,zero,temp);
  forward_dynamics(m, NULL, G, NULL, 0, dstate);
  for(i=0;i<NQ;i++) x_new[i] = x[i];
  for(i=0;i<NU;i++) x_new[NQ+i] = x[NQ+i] + dstate[NQ+i];
  delete []G;
  delete []dstate;
}

void angular_momentum(SDModel *m, int body, double *offset, double *H){
  // requires sdstate to be called prior
  double pos[3],mtot,com[3],Icm[3][3],LM[3],AM[3],KE,temp[3];
  sdvset(0,0,0,&H[0]); 
  m->sdpos(body,offset,pos);     
  m->sdsys(&mtot,com,Icm);
  m->sdmom(LM,AM,&KE);       
  sdvsub(com,pos,temp);
  sdvcross(temp,LM,temp);    
  sdvadd(temp,AM,&H[0]); 
}

void com_state(SDModel *m, double *p, double *v){
  // requires sdstate to be called prior    
  double mtot,Icm[3][3],AM[3],KE;   
  m->sdsys(&mtot,p,Icm);
  if (v!=NULL){
    m->sdmom(v,AM,&KE);
    sdvmul(1/mtot,v,v);
  }
}

void mass_matrix(SDModel *m, double *M){
  m->sdmassmat(M);
}

void com_acceleration(SDModel *m, double *acom){
  // requires sdstate & sdderiv to be called prior    
  int j;
  double acc[3],zero[3]={0},mi,mtot,com[3],Icm[3][3];
  m->sdsys(&mtot,com,Icm);
  sdvset(0,0,0,acom);
  for(j=0;j<m->NBOD();j++){
    m->sdgetmass(j,&mi);
    m->sdacc(j,zero,acc);
    sdvaxpy(mi/mtot,acc,acom,acom);
  }
}

void com_angular_acceleration(SDModel *m, double *Hdot){
  // requires sdstate & sdderiv to be called prior    
  int i,j,k;
  double x0[3],v0[3],xi[3],vi[3],ai[3],temp[3],Ii[3][3],zero[3]={0},mi;
  sdvset(0,0,0,Hdot);
  com_state(m,x0,v0);

  for(i=0;i<m->NBOD();i++){
    m->sdpos(i,zero,xi);
    sdvsub(xi,x0,xi);
    m->sdvel(i,zero,vi);
    m->sdgetmass(i,&mi);

    // (vi-v0)xLi
    sdvcross(v0,vi,temp);
    sdvmul(-mi,temp,temp);
    sdvadd(Hdot,temp,Hdot);

    // (xi-x0)xLi_dot
    m->sdacc(i,zero,ai);
    sdvcross(xi,ai,temp);
    sdvmul(mi,temp,temp);
    sdvadd(Hdot,temp,Hdot);

    // Hi_dot
    m->sdangacc(i,ai);
    m->sdgetiner(i,Ii);
    sdvset(0,0,0,temp);
    for(j=0;j<3;j++)
      for(k=0;k<3;k++)
        temp[j] += Ii[j][k]*ai[k];
    sdvadd(Hdot,temp,Hdot);
  }
}

/*
void com_jacobian(SDModel *m, double *J){
  double zero[3] = {0.0}, lin[3], rot[3], mi, mtot, Icm[3][3];
  m->sdsys(&mtot, lin, Icm);
  
  for (int i = 0; i < NU; i++) {
    for (int j = 0; j < 6; j++)  
      J[6*i+j] = 0;
  }

  for(int i = 0; i < m->NBOD(); i++){
    m->sdgetmass(i, &mi);
    mi = mi/mtot;
    for(int j = 0; j < NU; j++){
      m->sdrel2cart(j, i, zero, lin, rot);
      m->sdtrans(i, lin, -1, lin);
      sdvaxpy(mi, lin, &J[6*j], &J[6*j]);
    }
  }
  
  for(int j = 0; j < NU; j++){
    // assume 0 is pelvis, sfeng
    m->sdrel2cart(j, 0, zero, lin, rot);
    m->sdtrans(0, rot, -1, &J[6*j+3]);
  } 
}

original version
*/
void com_jacobian(SDModel *m, double *J){
  int i,j;
  double zero[3]={0.0},lin[3],rot[3],mi,mtot,Icm[3][3];
  m->sdsys(&mtot,lin,Icm);
  
  for (int i = 0; i < NU; i++) {
    for (int j = 0; j < 3; j++)  
      J[3*i+j] = 0;
  }

  for(i=0;i<m->NBOD();i++){
    m->sdgetmass(i,&mi);
    mi=mi/mtot;
    for(j=0;j<NU;j++){
      m->sdrel2cart(j,i,zero,lin,rot);
      m->sdtrans(i,lin,-1,lin);
      sdvaxpy(mi,lin,&J[3*j],&J[3*j]);
    }
  }    
}

void body_com_jacobian(SDModel *m, double *J){
  int i,j;
  double zero[3]={0.0},*lin,rot[3];
  for(j=0;j<m->NBOD();j++){
    for(i=0;i<NU;i++){
      lin = &J[m->NBOD()*i + 3*j];
      m->sdrel2cart(i,j,zero,lin,rot);
      m->sdtrans(i,lin,-1,lin);
    }    
  }
}
