/*
 * =====================================================================================
 *
 *       Filename:  pelv_dyn.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/02/2013 04:40:09 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */


#include "pelv_dyn_benx.h"
#include "RobotState.h"
#include "drc_pelvis_sim_defines.h"
#include "drc_pelvis.h"
#include <gsl/gsl_blas.h> 
#include <gsl/gsl_linalg.h>


bool pelv_forward_dynamics_no_F(
  int n_contacts,
  const gsl_vector *qd,
  const gsl_matrix *Jt,
  const gsl_matrix *Jd,
  const gsl_matrix *M,
  const gsl_vector *nonLin,
  const gsl_vector *tau,
  gsl_vector *qdd)
{
  int n_constraints = 6*n_contacts;
  int n_free = PELV_N_SDFAST_U - n_constraints;

  NEW_GSL_MATRIX(Q, Q_d, Q_v, PELV_N_SDFAST_U, PELV_N_SDFAST_U);
  NEW_GSL_MATRIX(R, R_d, R_v, PELV_N_SDFAST_U, n_constraints);

  gsl_matrix_view subM;
  gsl_vector_view subV;

  // QR decomp
  QPdecomp(Jt, Q, R);
  
  // Su and S
  NEW_GSL_MATRIX(Su, Su_d, Su_v, n_free, PELV_N_SDFAST_U);
  gsl_matrix_set_zero(Su);
  for (int i = 0; i < n_free; i++)
    gsl_matrix_set(Su, i, i + n_constraints, 1);

  NEW_GSL_MATRIX(S, S_d, S_v, PELV_N_SDFAST_U, N_JOINTS);
  gsl_matrix_set_zero(S);
  for (int i = 0; i < N_JOINTS; i++)
    gsl_matrix_set(S, i+6, i, 1);
  
  // P = Su*Q'
  NEW_GSL_MATRIX(P, P_d, P_v, n_free, PELV_N_SDFAST_U);
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1, Su, Q, 0.0, P);

  NEW_GSL_MATRIX(PM, PM_d, PM_v, n_free, PELV_N_SDFAST_U);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1, P, M, 0.0, PM);

  // get rhs = P(S*tau + h)
  NEW_GSL_VECTOR(tmpU, tmpU_d, tmpU_v, PELV_N_SDFAST_U);
  NEW_GSL_VECTOR(rhs, rhs_d, rhs_v, n_free);

  gsl_vector_memcpy(tmpU, nonLin);
  gsl_blas_dgemv(CblasNoTrans, 1, S, tau, 1, tmpU);
  gsl_blas_dgemv(CblasNoTrans, 1, P, tmpU, 0, rhs);

  // stack PM and J together
  NEW_GSL_MATRIX(PMJ, PMJ_d, PMJ_v, PELV_N_SDFAST_U, PELV_N_SDFAST_U);

  // upper part = PM
  subM = gsl_matrix_submatrix(PMJ, 0, 0, n_free, PELV_N_SDFAST_U);
  gsl_matrix_memcpy(&subM.matrix, PM);
  // lower part = J
  subM = gsl_matrix_submatrix(PMJ, n_free, 0, 6*n_contacts, PELV_N_SDFAST_U);
  gsl_matrix_transpose_memcpy(&subM.matrix, Jt);

  NEW_GSL_VECTOR(rhs34, rhs34_d, rhs34_v, PELV_N_SDFAST_U);
  // upper part = P(S*tau + h)
  dvec_copy(rhs34_d, rhs_d, n_free);

  // lower part = -Jd * qd
  subV = gsl_vector_subvector(rhs34, n_free, 6*n_contacts);
  gsl_blas_dgemv(CblasNoTrans, 1, Jd, qd, 0, &subV.vector);

  int s;
  gsl_permutation *perm = gsl_permutation_alloc(PMJ->size1);
  gsl_linalg_LU_decomp(PMJ, perm, &s);
  gsl_linalg_LU_solve(PMJ, perm, rhs34, qdd);
  gsl_permutation_free(perm);

  return true;
} 

bool pelv_integrate_one_step(int contactState, const gsl_matrix *Jd, const double x[PELV_N_X], const double u[PELV_N_U], double x1[PELV_N_X])
{
  static RobotState rs;
  
  // setup robot state based on give state 
  double dircos[3][3];
  double q[4], root[3], rootd[3], w[3], j[N_JOINTS], jd[N_JOINTS];
  rs.PelMod.sdang2dc(x[3], x[4], x[5], dircos);
  rs.PelMod.sddc2quat(dircos, q, q+1, q+2, q+3);
  dvec_copy(root, x, 3);
  dvec_copy(rootd, x+PELV_N_SDFAST_U, 3);
  dvec_copy(w, x+3+PELV_N_SDFAST_U, 3);
  dvec_copy(j, x+6, N_JOINTS);
  dvec_copy(jd, x+PELV_N_SDFAST_U+6, N_JOINTS);
  
  rs.computeSDFvars(root, q, rootd, w, j, jd);
  
  // get M, nonLin, J, Jd
  gsl_matrix_const_view Jt_l_v = gsl_matrix_const_view_array((const double *)(rs.J[LEFT].data()), PELV_N_SDFAST_U, 6);
  gsl_matrix_const_view Jt_r_v = gsl_matrix_const_view_array((const double *)(rs.J[RIGHT].data()), PELV_N_SDFAST_U, 6);
  
  gsl_matrix_const_view M_v = gsl_matrix_const_view_array((const double *)rs.M.data(), PELV_N_SDFAST_U, PELV_N_SDFAST_U);
  gsl_vector_const_view h_v = gsl_vector_const_view_array(rs.nonLin, PELV_N_SDFAST_U);
  gsl_vector_const_view tau_v = gsl_vector_const_view_array(u, N_JOINTS);
  gsl_vector_const_view qd_v = gsl_vector_const_view_array(rs.getSDFState()+PELV_N_SDFAST_Q, PELV_N_SDFAST_U);
  
  NEW_GSL_MATRIX(Jt, Jt_d, Jt_v, PELV_N_SDFAST_U, 12);
  gsl_matrix_view subM = gsl_matrix_submatrix(Jt, 0, 0, PELV_N_SDFAST_U, 6);
  gsl_matrix_memcpy(&subM.matrix, &Jt_l_v.matrix);
  subM = gsl_matrix_submatrix(Jt, 0, 6, PELV_N_SDFAST_U, 6);
  gsl_matrix_memcpy(&subM.matrix, &Jt_r_v.matrix);

  NEW_GSL_VECTOR(qdd, qdd_d, qdd_v, PELV_N_SDFAST_U);
  
  // compute acc
  switch(contactState) {
    case DSc:
      pelv_forward_dynamics_no_F(2, &qd_v.vector, Jt, Jd, &M_v.matrix, &h_v.vector, &tau_v.vector, qdd); 
      break;
    case SSL:
      pelv_forward_dynamics_no_F(1, &qd_v.vector, &Jt_l_v.matrix, Jd, &M_v.matrix, &h_v.vector, &tau_v.vector, qdd); 
      break;
    case SSR:
      pelv_forward_dynamics_no_F(1, &qd_v.vector, &Jt_r_v.matrix, Jd, &M_v.matrix, &h_v.vector, &tau_v.vector, qdd); 
      //pelv_forward_dynamics_no_F(1, &qd_v.vector, Jt+1, Jd+1, &M_v.matrix, &h_v.vector, &tau_v.vector, qdd); 
      break;
    default:
      printf("unknow contact state %d\n", contactState);
      exit(-1);
  }

  // Euler int
  double dt = 1e-3;
  for (int i = 0; i < PELV_N_SDFAST_U; i++) {
    x1[i] = x[i] + x[i+PELV_N_SDFAST_U]*dt;
    x1[i+PELV_N_SDFAST_U] = x[i+PELV_N_SDFAST_U] + qdd_d[i]*dt;
  } 

  return true;
}

bool pelv_one_step_cost(int contactState, const double x[PELV_N_X], const double u[PELV_N_U], const void *ref, double *c)
{
  /*
  // pos tracking
  const double *x_ref = ((const TrajPoint *)ref)->x;
  const double *u_ref = ((const TrajPoint *)ref)->u;
  double diff, tmp;
  *c = 0;

  for (int i = 0; i < PELV_N_X; i++) {
    diff = x_ref[i] - x[i];
    if (i < 6)
      tmp = diff*diff*1e4;
    else if (i < PELV_N_SDFAST_U)
      tmp = diff*diff*1e4;
    else if (i < 6+PELV_N_SDFAST_U)
      tmp = diff*diff*1e4;
    else
      tmp = diff*diff*1e4;

    *c += tmp;
  }

  // min trq
  for (int i = 0; i < PELV_N_U; i++) {
    *c += 0.5*(u[i]-u_ref[i])*(u[i]-u_ref[i])*1;
  }

*/
  return true;
}

void compute_project_matrix(int contactState, const double x[PELV_N_X], const double u[PELV_N_U], gsl_matrix *p_proj, gsl_matrix *v_proj, gsl_matrix *u_proj)
{
  size_t n_constraints;
  size_t n_free;
  switch(contactState) {
    case DSc:
      n_constraints = 12;
      break;
    case SSL:
    case SSR:
      n_constraints = 6;
      break;
    default:
      printf("unknown contact state\n");
      exit(-1);
  }

  n_free = PELV_N_SDFAST_U - n_constraints; 
  //printf("# con %d, # free %d\n", n_constraints, n_free);
  
  assert(p_proj->size1 == n_free && p_proj->size2 == PELV_N_SDFAST_U);
  assert(v_proj->size1 == n_free && v_proj->size2 == PELV_N_SDFAST_U);
  assert(u_proj->size1 == n_free && u_proj->size2 == PELV_N_U);

  // populate rs
  static RobotState rs;
  double dircos[3][3];
  double q[4], root[3], rootd[3], w[3], j[N_JOINTS], jd[N_JOINTS];
  rs.PelMod.sdang2dc(x[3], x[4], x[5], dircos);
  rs.PelMod.sddc2quat(dircos, q, q+1, q+2, q+3);
  dvec_copy(root, x, 3);
  dvec_copy(rootd, x+PELV_N_SDFAST_U, 3);
  dvec_copy(w, x+3+PELV_N_SDFAST_U, 3);
  dvec_copy(j, x+6, N_JOINTS);
  dvec_copy(jd, x+PELV_N_SDFAST_U+6, N_JOINTS);
  
  rs.computeSDFvars(root, q, rootd, w, j, jd);
  
  // build Jt
  NEW_GSL_MATRIX(bigJt, bigJt_d, bigJt_v, PELV_N_SDFAST_U, n_constraints);
  gsl_matrix_const_view Jt_l_v = gsl_matrix_const_view_array((const double *)(rs.J[LEFT].data()), PELV_N_SDFAST_U, 6);
  gsl_matrix_const_view Jt_r_v = gsl_matrix_const_view_array((const double *)(rs.J[RIGHT].data()), PELV_N_SDFAST_U, 6);
  gsl_matrix_view subM;

  NEW_GSL_MATRIX(Q, Q_d, Q_v, PELV_N_SDFAST_U, PELV_N_SDFAST_U);
  NEW_GSL_MATRIX(R, R_d, R_v, PELV_N_SDFAST_U, n_constraints);
  
  switch (contactState) {
    case DSc:
      subM = gsl_matrix_submatrix(bigJt, 0, 0, PELV_N_SDFAST_U, 6);
      gsl_matrix_memcpy(&subM.matrix, &Jt_l_v.matrix);
      subM = gsl_matrix_submatrix(bigJt, 0, 6, PELV_N_SDFAST_U, 6);
      gsl_matrix_memcpy(&subM.matrix, &Jt_r_v.matrix);
      break;
    case SSL:
      subM = gsl_matrix_submatrix(bigJt, 0, 0, PELV_N_SDFAST_U, 6);
      gsl_matrix_memcpy(&subM.matrix, &Jt_l_v.matrix);
      break;
    case SSR:
      subM = gsl_matrix_submatrix(bigJt, 0, 0, PELV_N_SDFAST_U, 6);
      gsl_matrix_memcpy(&subM.matrix, &Jt_r_v.matrix);
      break;
    default:
      printf("unknow contact state %d\n", contactState);
      exit(-1);
  }
  
  // Jt = Q*[R;0]
  QPdecomp(bigJt, Q, R);

  // position proj = Su*Qt
  subM = gsl_matrix_submatrix(Q, 0, n_constraints, PELV_N_SDFAST_U, n_free);
  gsl_matrix_transpose_memcpy(p_proj, &subM.matrix);

  gsl_matrix_transpose_memcpy(v_proj, &subM.matrix);

  NEW_GSL_MATRIX(S, S_d, S_v, PELV_N_SDFAST_U, N_JOINTS);
  gsl_matrix_set_zero(S);
  for (int i = 0; i < N_JOINTS; i++)
    gsl_matrix_set(S, i+6, i, 1); 
  gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1, &subM.matrix, S, 0, u_proj);
}

bool proj_linearize_dynamics(int contactState, const gsl_matrix *Jd, const gsl_matrix *p_proj, const gsl_matrix *v_proj, const gsl_matrix *u_proj, const gsl_vector *x, const gsl_vector *u, gsl_matrix *A, gsl_matrix *B)
{
  size_t n_constraints;
  size_t n_free; 
  size_t n_x_free;

  switch(contactState) {
    case DSc:
      n_constraints = 12;
      break;
    case SSL:
    case SSR:
      n_constraints = 6;
      break;
    default:
      printf("unknown contact state\n");
      exit(-1);
  } 
  n_free = PELV_N_SDFAST_U - n_constraints; 
  n_x_free = 2*n_free;
  
  assert(Jd->size1 == n_constraints && Jd->size2 == PELV_N_SDFAST_U);
  assert(p_proj->size1 == n_free && p_proj->size2 == PELV_N_SDFAST_U);
  assert(v_proj->size1 == n_free && v_proj->size2 == PELV_N_SDFAST_U);
  assert(u_proj->size1 == n_free && u_proj->size2 == PELV_N_U);
  assert(x->size == PELV_N_X);
  assert(u->size == PELV_N_U);
  assert(A->size1 == n_x_free && A->size2 == n_x_free);
  assert(B->size1 == n_x_free && B->size2 == n_free);
  
  NEW_GSL_VECTOR(xx34, xx34_d, xx34_v, PELV_N_X);
  NEW_GSL_VECTOR(uu28, uu28_d, uu28_v, PELV_N_U);
  NEW_GSL_VECTOR(tmp34, tmp34_d, tmp34_v, PELV_N_X);
  
  NEW_GSL_VECTOR(tmp_x0, tmp_x0_d, tmp_x0_v, n_x_free);
  NEW_GSL_VECTOR(tmp_x1, tmp_x1_d, tmp_x1_v, n_x_free);

  NEW_GSL_MATRIX(x_proj, x_proj_d, x_proj_v, n_x_free, PELV_N_X);
  gsl_matrix_set_zero(x_proj);

  gsl_matrix_view subM0 = gsl_matrix_submatrix(x_proj, 0, 0, n_free, PELV_N_SDFAST_U);
  gsl_matrix_memcpy(&subM0.matrix, p_proj);
  subM0 = gsl_matrix_submatrix(x_proj, n_free, PELV_N_SDFAST_U, n_free, PELV_N_SDFAST_U);
  gsl_matrix_memcpy(&subM0.matrix, p_proj);

  double delta = 1e-4;

  NEW_GSL_VECTOR(dx, dx_d, dx_v, n_x_free);
  NEW_GSL_VECTOR(du, du_d, du_v, n_free);

  // linearize x
  for (size_t i = 0; i < A->size2; i++) {
    //xx_d[i] = x[i] - delta;
    //gsl_blas_dgemv(CblasTrans, 1, x_proj, xx, 0, xx34);
    gsl_vector_set_zero(dx);
    dx_d[i] -= delta;
    gsl_blas_dgemv(CblasTrans, 1, x_proj, dx, 0, xx34);
    gsl_vector_add(xx34, x);

    pelv_integrate_one_step(contactState, Jd, xx34_d, u->data, tmp34_d);
    //ssl_int_one_step(xx34_d, uu34_d+6, tmp34_d);
    gsl_blas_dgemv(CblasNoTrans, 1, x_proj, tmp34, 0, tmp_x0);

    //xx_d[i] = x[i] + delta;
    //gsl_blas_dgemv(CblasTrans, 1, x_proj, xx, 0, xx34);
    gsl_vector_set_zero(dx);
    dx_d[i] += delta;
    gsl_blas_dgemv(CblasTrans, 1, x_proj, dx, 0, xx34);
    gsl_vector_add(xx34, x);

    pelv_integrate_one_step(contactState, Jd, xx34_d, u->data, tmp34_d);
    //ssl_int_one_step(xx34_d, uu34_d+6, tmp34_d);
    gsl_blas_dgemv(CblasNoTrans, 1, x_proj, tmp34, 0, tmp_x1);

    for (size_t j = 0; j < A->size1; j++)
      gsl_matrix_set(A, j, i, (tmp_x1_d[j]-tmp_x0_d[j])/(2.0*delta));
  } 
  
  // linearize U
  for (size_t i = 0; i < B->size2; i++) {
    //uu_d[i] = u->data[i] - delta;
    gsl_vector_set_zero(du);
    du_d[i] -= delta;
    gsl_blas_dgemv(CblasTrans, 1, u_proj, du, 0, uu28);
    gsl_vector_add(uu28, u);

    pelv_integrate_one_step(contactState, Jd, x->data, uu28_d, tmp34_d);
    gsl_blas_dgemv(CblasNoTrans, 1, x_proj, tmp34, 0, tmp_x0);

    //uu_d[i] = u->data[i] + delta;
    gsl_vector_set_zero(du);
    du_d[i] += delta;
    gsl_blas_dgemv(CblasTrans, 1, u_proj, du, 0, uu28);
    gsl_vector_add(uu28, u);

    pelv_integrate_one_step(contactState, Jd, x->data, uu28_d, tmp34_d);
    gsl_blas_dgemv(CblasNoTrans, 1, x_proj, tmp34, 0, tmp_x1);

    for (size_t j = 0; j < B->size1; j++)
      gsl_matrix_set(B, j, i, (tmp_x1_d[j]-tmp_x0_d[j])/(2.0*delta));
  }  

  return true;
}

bool proj_linearize_cost(
  int contactState, 
  const gsl_matrix *p_proj, 
  const gsl_matrix *v_proj, 
  const gsl_matrix *u_proj, 
  const gsl_vector *x, 
  const gsl_vector *u, 
  const void *ref, 
  gsl_vector *Lx, 
  gsl_vector *Lu)
{
  size_t n_constraints;
  size_t n_free; 
  size_t n_x_free;

  switch(contactState) {
    case DSc:
      n_constraints = 12;
      break;
    case SSL:
    case SSR:
      n_constraints = 6;
      break;
    default:
      printf("unknown contact state\n");
      exit(-1);
  } 
  
  n_free = PELV_N_SDFAST_U - n_constraints; 
  n_x_free = 2*n_free;
  
  assert(p_proj->size1 == n_free && p_proj->size2 == PELV_N_SDFAST_U);
  assert(v_proj->size1 == n_free && v_proj->size2 == PELV_N_SDFAST_U);
  assert(u_proj->size1 == n_free && u_proj->size2 == PELV_N_U);
  assert(x->size == PELV_N_X);
  assert(u->size == PELV_N_U);
  assert(Lx->size == n_x_free);
  assert(Lu->size == n_free);

  double c0, c1;
  NEW_GSL_VECTOR(xx34, xx34_d, xx34_v, PELV_N_X);
  NEW_GSL_VECTOR(uu28, uu28_d, uu28_v, PELV_N_U);
  NEW_GSL_VECTOR(tmp34, tmp34_d, tmp34_v, PELV_N_X);
  
  NEW_GSL_VECTOR(tmp_x0, tmp_x0_d, tmp_x0_v, n_x_free);
  NEW_GSL_VECTOR(tmp_x1, tmp_x1_d, tmp_x1_v, n_x_free);

  NEW_GSL_MATRIX(x_proj, x_proj_d, x_proj_v, n_x_free, PELV_N_X);
  gsl_matrix_set_zero(x_proj);

  gsl_matrix_view subM0 = gsl_matrix_submatrix(x_proj, 0, 0, n_free, PELV_N_SDFAST_U);
  gsl_matrix_memcpy(&subM0.matrix, p_proj);
  subM0 = gsl_matrix_submatrix(x_proj, n_free, PELV_N_SDFAST_U, n_free, PELV_N_SDFAST_U);
  gsl_matrix_memcpy(&subM0.matrix, p_proj);

  double delta = 1e-4;

  NEW_GSL_VECTOR(dx, dx_d, dx_v, n_x_free);
  NEW_GSL_VECTOR(du, du_d, du_v, n_free);

  // linearize x
  for (size_t i = 0; i < Lx->size; i++) {
    gsl_vector_set_zero(dx);
    dx_d[i] -= delta;
    gsl_blas_dgemv(CblasTrans, 1, x_proj, dx, 0, xx34);
    gsl_vector_add(xx34, x);

    pelv_one_step_cost(contactState, xx34_d, u->data, ref, &c0);

    gsl_vector_set_zero(dx);
    dx_d[i] += delta;
    gsl_blas_dgemv(CblasTrans, 1, x_proj, dx, 0, xx34);
    gsl_vector_add(xx34, x);

    pelv_one_step_cost(contactState, xx34_d, u->data, ref, &c1);
    
    gsl_vector_set(Lx, i, (c1-c0)/(2*delta));
  } 
  
  // linearize U
  for (size_t i = 0; i < Lu->size; i++) {
    gsl_vector_set_zero(du);
    du_d[i] -= delta;
    gsl_blas_dgemv(CblasTrans, 1, u_proj, du, 0, uu28);
    gsl_vector_add(uu28, u);

    pelv_one_step_cost(contactState, x->data, uu28_d, ref, &c0);

    //uu_d[i] = u->data[i] + delta;
    gsl_vector_set_zero(du);
    du_d[i] += delta;
    gsl_blas_dgemv(CblasTrans, 1, u_proj, du, 0, uu28);
    gsl_vector_add(uu28, u);

    pelv_one_step_cost(contactState, x->data, uu28_d, ref, &c1);
    
    gsl_vector_set(Lu, i, (c1-c0)/(2*delta));
  }  

  return true;
}

bool proj_second_order_cost(
  int contactState, 
  const gsl_matrix *p_proj, 
  const gsl_matrix *v_proj, 
  const gsl_matrix *u_proj, 
  const gsl_vector *x, 
  const gsl_vector *u, 
  const void *ref, 
  gsl_matrix *Lxx, 
  gsl_matrix *Lxu, 
  gsl_matrix *Lux, 
  gsl_matrix *Luu)
{
  size_t n_constraints;
  size_t n_free; 
  size_t n_x_free;

  switch(contactState) {
    case DSc:
      n_constraints = 12;
      break;
    case SSL:
    case SSR:
      n_constraints = 6;
      break;
    default:
      printf("unknown contact state\n");
      exit(-1);
  } 
  
  n_free = PELV_N_SDFAST_U - n_constraints; 
  n_x_free = 2*n_free;
  
  assert(p_proj->size1 == n_free && p_proj->size2 == PELV_N_SDFAST_U);
  assert(v_proj->size1 == n_free && v_proj->size2 == PELV_N_SDFAST_U);
  assert(u_proj->size1 == n_free && u_proj->size2 == PELV_N_U);
  assert(x->size == PELV_N_X);
  assert(u->size == PELV_N_U);
  assert(Lxx->size1 == n_x_free && Lxx->size2 == n_x_free);
  assert(Luu->size1 == n_free && Luu->size2 == n_free);
  assert(Lxu->size1 == n_x_free && Lxu->size2 == n_free);
  assert(Lux->size1 == n_free && Lux->size2 == n_x_free);

  NEW_GSL_VECTOR(xx34, xx34_d, xx34_v, PELV_N_X);
  NEW_GSL_VECTOR(uu28, uu28_d, uu28_v, PELV_N_U);
  NEW_GSL_VECTOR(tmp34, tmp34_d, tmp34_v, PELV_N_X);
  
  NEW_GSL_VECTOR(Lx_l, Lx_l_d, Lx_l_v, n_x_free);
  NEW_GSL_VECTOR(Lx_h, Lx_h_d, Lx_h_v, n_x_free);
  NEW_GSL_VECTOR(Lu_l, Lu_l_d, Lu_l_v, n_free);
  NEW_GSL_VECTOR(Lu_h, Lu_h_d, Lu_h_v, n_free);

  NEW_GSL_MATRIX(x_proj, x_proj_d, x_proj_v, n_x_free, PELV_N_X);
  gsl_matrix_set_zero(x_proj);

  gsl_matrix_view subM0 = gsl_matrix_submatrix(x_proj, 0, 0, n_free, PELV_N_SDFAST_U);
  gsl_matrix_memcpy(&subM0.matrix, p_proj);
  subM0 = gsl_matrix_submatrix(x_proj, n_free, PELV_N_SDFAST_U, n_free, PELV_N_SDFAST_U);
  gsl_matrix_memcpy(&subM0.matrix, p_proj);

  double delta = 1e-4;

  NEW_GSL_VECTOR(dx, dx_d, dx_v, n_x_free);
  NEW_GSL_VECTOR(du, du_d, du_v, n_free);

  // linearize x
  for (size_t i = 0; i < Lxx->size2; i++) {
    gsl_vector_set_zero(dx);
    dx_d[i] -= delta;
    gsl_blas_dgemv(CblasTrans, 1, x_proj, dx, 0, xx34);
    gsl_vector_add(xx34, x);
    
    proj_linearize_cost(contactState, p_proj, v_proj, u_proj, xx34, u, ref, Lx_l, Lu_l);

    gsl_vector_set_zero(dx);
    dx_d[i] += delta;
    gsl_blas_dgemv(CblasTrans, 1, x_proj, dx, 0, xx34);
    gsl_vector_add(xx34, x);

    proj_linearize_cost(contactState, p_proj, v_proj, u_proj, xx34, u, ref, Lx_h, Lu_h);
    
    for (size_t j = 0; j < Lxx->size1; j++)
      gsl_matrix_set(Lxx, j, i, (gsl_vector_get(Lx_h, j)-gsl_vector_get(Lx_l, j))/(2.0*delta));
    for (size_t j = 0; j < Lux->size1; j++)
      gsl_matrix_set(Lux, j, i, (gsl_vector_get(Lu_h, j)-gsl_vector_get(Lu_l, j))/(2.0*delta));
  } 
  
  // linearize U
  for (size_t i = 0; i < Luu->size2; i++) {
    gsl_vector_set_zero(du);
    du_d[i] -= delta;
    gsl_blas_dgemv(CblasTrans, 1, u_proj, du, 0, uu28);
    gsl_vector_add(uu28, u);

    proj_linearize_cost(contactState, p_proj, v_proj, u_proj, x, uu28, ref, Lx_l, Lu_l);

    gsl_vector_set_zero(du);
    du_d[i] += delta;
    gsl_blas_dgemv(CblasTrans, 1, u_proj, du, 0, uu28);
    gsl_vector_add(uu28, u);

    proj_linearize_cost(contactState, p_proj, v_proj, u_proj, x, uu28, ref, Lx_h, Lu_h);
    
    for (size_t j = 0; j < Lxu->size1; j++)
      gsl_matrix_set(Lxu, j, i, (gsl_vector_get(Lx_h, j)-gsl_vector_get(Lx_l, j))/(2.0*delta));
    for (size_t j = 0; j < Luu->size1; j++)
      gsl_matrix_set(Luu, j, i, (gsl_vector_get(Lu_h, j)-gsl_vector_get(Lu_l, j))/(2.0*delta));
  }  

  return true; 
}

void sim_2_pelv(Simulator &sim, double pelv_x[PELV_N_X])
{
  dvec_copy(pelv_x, sim.root, 6);
  dvec_copy(pelv_x+6, sim.joints, N_JOINTS);

  dvec_copy(pelv_x+PELV_N_SDFAST_U, sim.rootd, 6);
  dvec_copy(pelv_x+6+PELV_N_SDFAST_U, sim.jointsd, N_JOINTS);
}




/************************************************************************************************/
// The following are added by Ben X
/************************************************************************************************/
bool pelv_integrate_one_step(int contactState, const double x[PELV_N_X], const double u[PELV_N_U], double x1[PELV_N_X])
{
  static RobotState rs;
  
  // setup robot state based on give state 
  double dircos[3][3];
  double q[4], root[3], rootd[3], w[3], j[N_JOINTS], jd[N_JOINTS];
  rs.PelMod.sdang2dc(x[3], x[4], x[5], dircos);
  rs.PelMod.sddc2quat(dircos, q, q+1, q+2, q+3);
  dvec_copy(root, x, 3);
  dvec_copy(rootd, x+PELV_N_SDFAST_U, 3);
  dvec_copy(w, x+3+PELV_N_SDFAST_U, 3);
  dvec_copy(j, x+6, N_JOINTS);
  dvec_copy(jd, x+PELV_N_SDFAST_U+6, N_JOINTS);
  
  rs.computeSDFvars(root, q, rootd, w, j, jd);
  
  // get M, nonLin, J, Jd
  gsl_matrix_const_view Jt_l_v = gsl_matrix_const_view_array((const double *)(rs.J[LEFT].data()), PELV_N_SDFAST_U, 6);
  gsl_matrix_const_view Jt_r_v = gsl_matrix_const_view_array((const double *)(rs.J[RIGHT].data()), PELV_N_SDFAST_U, 6);

  gsl_matrix_const_view M_v = gsl_matrix_const_view_array((const double *)rs.M.data(), PELV_N_SDFAST_U, PELV_N_SDFAST_U);
  gsl_vector_const_view h_v = gsl_vector_const_view_array(rs.nonLin, PELV_N_SDFAST_U);
  gsl_vector_const_view tau_v = gsl_vector_const_view_array(u, N_JOINTS);
  gsl_vector_const_view qd_v = gsl_vector_const_view_array(rs.getSDFState()+PELV_N_SDFAST_Q, PELV_N_SDFAST_U);
  
  NEW_GSL_MATRIX(Jt, Jt_d, Jt_v, PELV_N_SDFAST_U, 12);
  gsl_matrix_view subM = gsl_matrix_submatrix(Jt, 0, 0, PELV_N_SDFAST_U, 6);
  gsl_matrix_memcpy(&subM.matrix, &Jt_l_v.matrix);
  subM = gsl_matrix_submatrix(Jt, 0, 6, PELV_N_SDFAST_U, 6);
  gsl_matrix_memcpy(&subM.matrix, &Jt_r_v.matrix);



// Added by Ben X
  gsl_matrix_const_view Jdt_l_v = gsl_matrix_const_view_array((const double *)(rs.Jd[LEFT].data()), PELV_N_SDFAST_U, 6);
  gsl_matrix_const_view Jdt_r_v = gsl_matrix_const_view_array((const double *)(rs.Jd[RIGHT].data()), PELV_N_SDFAST_U, 6); 
                              
  NEW_GSL_MATRIX(Jd, Jd_d, Jd_v, 12,PELV_N_SDFAST_U);
  gsl_matrix_view subMT = gsl_matrix_submatrix(Jd, 0, 0, 6, PELV_N_SDFAST_U);  
  subMT = gsl_matrix_submatrix(Jd, 0, 0, 6,PELV_N_SDFAST_U);
  gsl_matrix_transpose_memcpy(&subMT.matrix, &Jdt_l_v.matrix);
  subMT = gsl_matrix_submatrix(Jd, 6, 0, 6, PELV_N_SDFAST_U);
  gsl_matrix_transpose_memcpy(&subMT.matrix, &Jdt_r_v.matrix); 
  
  NEW_GSL_MATRIX(Jd_l_v, Jd_l_v_d, Jd_l_v_v, 6, PELV_N_SDFAST_U);
  gsl_matrix_transpose_memcpy(Jd_l_v, &Jdt_l_v.matrix);
  NEW_GSL_MATRIX(Jd_r_v, Jd_r_v_d, Jd_r_v_v, 6, PELV_N_SDFAST_U);
  gsl_matrix_transpose_memcpy(Jd_r_v, &Jdt_r_v.matrix); 
  

  NEW_GSL_VECTOR(qdd, qdd_d, qdd_v, PELV_N_SDFAST_U);
  
  // compute acc
  switch(contactState) {
    case DSc:
      pelv_forward_dynamics_no_F(2, &qd_v.vector, Jt, Jd, &M_v.matrix, &h_v.vector, &tau_v.vector, qdd); 
      break;
    case SSL:
      pelv_forward_dynamics_no_F(1, &qd_v.vector, &Jt_l_v.matrix, Jd_l_v, &M_v.matrix, &h_v.vector, &tau_v.vector, qdd); 
      break;
    case SSR:
      pelv_forward_dynamics_no_F(1, &qd_v.vector, &Jt_r_v.matrix, Jd_r_v, &M_v.matrix, &h_v.vector, &tau_v.vector, qdd); 
      //pelv_forward_dynamics_no_F(1, &qd_v.vector, Jt+1, Jd+1, &M_v.matrix, &h_v.vector, &tau_v.vector, qdd); 
      break;
    default:
      printf("unknow contact state %d\n", contactState);
      exit(-1);
  }

  // Euler int
  double dt = 1e-3;
  for (int i = 0; i < PELV_N_SDFAST_U; i++) {
    x1[i] = x[i] + x[i+PELV_N_SDFAST_U]*dt;
    x1[i+PELV_N_SDFAST_U] = x[i+PELV_N_SDFAST_U] + qdd_d[i]*dt;
  } 

  return true;
}

/************************************************************************************************/    
/************************************************************************************************/    
bool proj_linearize_dynamics(int contactState,const gsl_matrix *p_proj, const gsl_vector *x, const gsl_vector *u, gsl_matrix *A)
{
  size_t n_constraints;
  size_t n_free; 
  size_t n_x_free;

  switch(contactState) {
    case DSc:
      n_constraints = 12;
      break;
    case SSL:
    case SSR:
      n_constraints = 6;
      break;
    default:
      printf("unknown contact state\n");
      exit(-1);
  } 
  n_free = PELV_N_SDFAST_U - n_constraints; 
  n_x_free = 2*n_free;
  
  //assert(Jd->size1 == n_constraints && Jd->size2 == PELV_N_SDFAST_U);
  assert(p_proj->size1 == n_free && p_proj->size2 == PELV_N_SDFAST_U);
  //assert(v_proj->size1 == n_free && v_proj->size2 == PELV_N_SDFAST_U);
  //assert(u_proj->size1 == n_free && u_proj->size2 == PELV_N_U);
  assert(x->size == PELV_N_X);
  assert(u->size == PELV_N_U);
  // assert(A->size1 == n_x_free && A->size2 == n_x_free);
  // assert(B->size1 == n_x_free && B->size2 == n_free);
  
  NEW_GSL_VECTOR(xx34, xx34_d, xx34_v, PELV_N_X);
  NEW_GSL_VECTOR(uu28, uu28_d, uu28_v, PELV_N_U);
  NEW_GSL_VECTOR(tmp34, tmp34_d, tmp34_v, PELV_N_X);
  NEW_GSL_VECTOR(tmp34_1, tmp34_1_d, tmp34_1_v, PELV_N_X); 
  
  NEW_GSL_VECTOR(tmp_x0, tmp_x0_d, tmp_x0_v, n_x_free);
  NEW_GSL_VECTOR(tmp_x1, tmp_x1_d, tmp_x1_v, n_x_free);

  NEW_GSL_MATRIX(x_proj, x_proj_d, x_proj_v, n_x_free, PELV_N_X);
  gsl_matrix_set_zero(x_proj);

  gsl_matrix_view subM0 = gsl_matrix_submatrix(x_proj, 0, 0, n_free, PELV_N_SDFAST_U);
  gsl_matrix_memcpy(&subM0.matrix, p_proj);
  subM0 = gsl_matrix_submatrix(x_proj, n_free, PELV_N_SDFAST_U, n_free, PELV_N_SDFAST_U);
  gsl_matrix_memcpy(&subM0.matrix, p_proj);

  // AA is linearization of acceleration w.r.t velocity (position fixed)
  NEW_GSL_MATRIX(AA, AA_d, AA_v, n_free, n_free);
  gsl_matrix_set_zero(AA);
  // p_proj^T * AA
  NEW_GSL_MATRIX(p_proj_T_AA, p_proj_T_AA_d, p_proj_T_AA_v, PELV_N_SDFAST_U - 6, n_free);
  gsl_matrix_set_zero(p_proj_T_AA);  

// Only take into account joint velocities
  gsl_matrix_const_view p_proj_reduced = gsl_matrix_const_submatrix (p_proj, 0, 6, n_free, PELV_N_SDFAST_U-6);

  double delta = 1e-4;



  pelv_integrate_one_step(contactState, x->data, u->data, tmp34_d);
  //ssl_int_one_step(xx34_d, uu34_d+6, tmp34_d);
  gsl_blas_dgemv(CblasNoTrans, 1, x_proj, tmp34, 0, tmp_x0); 

/*  // linearize x (reduced space version)  
   NEW_GSL_VECTOR(dx, dx_d, dx_v, n_x_free);
  // NEW_GSL_VECTOR(du, du_d, du_v, n_free);  
  for (size_t i = 0; i < n_free; i++) {
    //xx_d[i] = x[i] + delta;
    //gsl_blas_dgemv(CblasTrans, 1, x_proj, xx, 0, xx34);
    gsl_vector_set_zero(dx);
    dx_d[n_free+i] += delta;
    gsl_blas_dgemv(CblasTrans, 1, x_proj, dx, 0, xx34);
    gsl_vector_add(xx34, x);

    pelv_integrate_one_step(contactState, xx34_d, u->data, tmp34_d);
    //ssl_int_one_step(xx34_d, uu34_d+6, tmp34_d);
    gsl_blas_dgemv(CblasNoTrans, 1, x_proj, tmp34, 0, tmp_x1);

    for (size_t j = 0; j < n_free; j++)
      gsl_matrix_set(AA, j, i, (tmp_x1_d[n_free+j]-tmp_x0_d[n_free+j])/(1.0*delta));
  }
  // Choose only the jointd degree of freedom in A
  gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, &p_proj_reduced.matrix, AA, 0.0, p_proj_T_AA);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, p_proj_T_AA, &p_proj_reduced.matrix, 0.0, A);    
 */

 // linearize x, normal version
 // NEW_GSL_VECTOR(dx, dx_d, dx_v, PELV_N_X);
  for (size_t i = 0; i < N_JOINTS ; i++) {
    //xx_d[i] = x[i] + delta;
    //gsl_blas_dgemv(CblasTrans, 1, x_proj, xx, 0, xx34);
    gsl_vector_set_zero(xx34);
    xx34_d[PELV_N_SDFAST_U+ 6 +i] = delta;
    // gsl_blas_dgemv(CblasTrans, 1, x_proj, dx, 0, xx34);
    gsl_vector_add(xx34, x);

    pelv_integrate_one_step(contactState, xx34_d, u->data, tmp34_1_d);
    //ssl_int_one_step(xx34_d, uu34_d+6, tmp34_d);
    // gsl_blas_dgemv(CblasNoTrans, 1, x_proj, tmp34, 0, tmp_x1);

    for (size_t j = 0; j < N_JOINTS; j++)
      gsl_matrix_set(A, j, i, (tmp34_1_d[PELV_N_SDFAST_U+ 6 +j]-tmp34_d[PELV_N_SDFAST_U+ 6 +j])/(1.0*delta));
  }   

  return true;
} 
/************************************************************************************************/   
/************************************************************************************************/   
void compute_project_matrix(int contactState, const double x[PELV_N_X], const double u[PELV_N_U], gsl_matrix *p_proj)
{
  size_t n_constraints;
  size_t n_free;
  switch(contactState) {
    case DSc:
      n_constraints = 12;
      break;
    case SSL:
    case SSR:
      n_constraints = 6;
      break;
    default:
      printf("unknown contact state\n");
      exit(-1);
  }

  n_free = PELV_N_SDFAST_U - n_constraints; 
  //printf("# con %d, # free %d\n", n_constraints, n_free);
  
  assert(p_proj->size1 == n_free && p_proj->size2 == PELV_N_SDFAST_U);
 // assert(v_proj->size1 == n_free && v_proj->size2 == PELV_N_SDFAST_U);
 // assert(u_proj->size1 == n_free && u_proj->size2 == PELV_N_U);

  // populate rs
  static RobotState rs;
  double dircos[3][3];
  double q[4], root[3], rootd[3], w[3], j[N_JOINTS], jd[N_JOINTS];
  rs.PelMod.sdang2dc(x[3], x[4], x[5], dircos);
  rs.PelMod.sddc2quat(dircos, q, q+1, q+2, q+3);
  dvec_copy(root, x, 3);
  dvec_copy(rootd, x+PELV_N_SDFAST_U, 3);
  dvec_copy(w, x+3+PELV_N_SDFAST_U, 3);
  dvec_copy(j, x+6, N_JOINTS);
  dvec_copy(jd, x+PELV_N_SDFAST_U+6, N_JOINTS);
  
  rs.computeSDFvars(root, q, rootd, w, j, jd);
  
  // build Jt
  NEW_GSL_MATRIX(bigJt, bigJt_d, bigJt_v, PELV_N_SDFAST_U, n_constraints);
  gsl_matrix_const_view Jt_l_v = gsl_matrix_const_view_array((const double *)(rs.J[LEFT].data()), PELV_N_SDFAST_U, 6);
  gsl_matrix_const_view Jt_r_v = gsl_matrix_const_view_array((const double *)(rs.J[RIGHT].data()), PELV_N_SDFAST_U, 6);
  gsl_matrix_view subM;

  NEW_GSL_MATRIX(Q, Q_d, Q_v, PELV_N_SDFAST_U, PELV_N_SDFAST_U);
  NEW_GSL_MATRIX(R, R_d, R_v, PELV_N_SDFAST_U, n_constraints);
  
  switch (contactState) {
    case DSc:
      subM = gsl_matrix_submatrix(bigJt, 0, 0, PELV_N_SDFAST_U, 6);
      gsl_matrix_memcpy(&subM.matrix, &Jt_l_v.matrix);
      subM = gsl_matrix_submatrix(bigJt, 0, 6, PELV_N_SDFAST_U, 6);
      gsl_matrix_memcpy(&subM.matrix, &Jt_r_v.matrix);
      break;
    case SSL:
      subM = gsl_matrix_submatrix(bigJt, 0, 0, PELV_N_SDFAST_U, 6);
      gsl_matrix_memcpy(&subM.matrix, &Jt_l_v.matrix);
      break;
    case SSR:
      subM = gsl_matrix_submatrix(bigJt, 0, 0, PELV_N_SDFAST_U, 6);
      gsl_matrix_memcpy(&subM.matrix, &Jt_r_v.matrix);
      break;
    default:
      printf("unknow contact state %d\n", contactState);
      exit(-1);
  }
  
  // Jt = Q*[R;0]
  QPdecomp(bigJt, Q, R);

  // position proj = Su*Qt
  subM = gsl_matrix_submatrix(Q, 0, n_constraints, PELV_N_SDFAST_U, n_free);
  gsl_matrix_transpose_memcpy(p_proj, &subM.matrix);

 // gsl_matrix_transpose_memcpy(v_proj, &subM.matrix);

 // NEW_GSL_MATRIX(S, S_d, S_v, PELV_N_SDFAST_U, N_JOINTS);
 // gsl_matrix_set_zero(S);
 // for (int i = 0; i < N_JOINTS; i++)
 //   gsl_matrix_set(S, i+6, i, 1); 
 // gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1, &subM.matrix, S, 0, u_proj);
}
