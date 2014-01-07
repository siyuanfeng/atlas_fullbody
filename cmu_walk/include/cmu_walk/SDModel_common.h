#ifndef SDFAST_COMMON
#define SDFAST_COMMON

#include "SDModel.h"

typedef struct {
	int body;
	double pos[3];
	double force[3];
    double moment[3];
}SDFext_t;

void sdtrans6(SDModel*m, int inb, double *ivec, int ob, double *ovec);

void set_state(SDModel *m, double *x);
void body_position(SDModel *m, double *q, int body, double *offset, double *p, double *J);
void forward_dynamics(SDModel *m, double *x, double *trq, SDFext_t *fext, int n_fext, double *dstate);
void inverse_dynamics(SDModel *m, double *x, double *ddq, double *trq);
void impulse_force(SDModel *m, double *x, int body, double *f, double *x_new);
void linearize_dynamics_continuous(SDModel *m, double *x, double *u, double *A, double *B);

// model updates
void get_model_params(SDModel *m, double *mass, double *btj, double *I, double *itj, double *pin);
void set_model_params(SDModel *m, int n, int *bodies, double *mass, double *com, double *I);
void set_model_params_all(SDModel *m, double *mass, double *off, double *I);

// only requires sdstate
void angular_momentum(SDModel *m, int body, double *offset, double *H);
void com_state(SDModel *m, double *p, double *v);
void mass_matrix(SDModel *m, double *M);
void com_jacobian(SDModel *m, double *J);
void body_com_jacobian(SDModel *m, double *J);

// requires sdderiv
void com_acceleration(SDModel *m, double *acom);
void com_angular_acceleration(SDModel *m, double *Hdot);

#endif
