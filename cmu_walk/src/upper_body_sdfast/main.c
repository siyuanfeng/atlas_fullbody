/*
 * =====================================================================================
 *
 *       Filename:  main.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  11/14/2013 11:34:42 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */


#include <math.h>
#include <stdio.h>

void sdumotion(double *a, double *b, double *c) {;}
void sduforce(double *a, double *b, double *c) {;}

void sdinit(void);
void sdstate(double timein, double qin[14], double uin[14]);
void sdsys(double lm[3], double am[3], double ke[3][3]);
void sdmom(double lm[3], double am[3], double *ke);
void sdgetbtj(int j, double btj[3]);
void sdpos(int b, double off[3], double pos[3]);

int main()
{
  //double q[14] = {0};
  double q[14] = 
  {
    0, 0,
    // raised
    0.7, -1.3, 2, 1.8, 0.2, 0.6, 
    0.7, 1.3, 2, -1.8, 0.2, -0.6 
    
    // side
    //0.3, -0.5, 2, 0.5, 0.2, 0, 
    //0.3, 0.5, 2, -0.5, 0.2, 0 

    // bdi
    //0.3, -1.3, 2, 0.5, 0.2, 0, 
    //0.3, 1.3, 2, -0.5, 0.2, 0 
  };
  double u[14] = {0};
  double mom[3];
  double com[3];
  double I[3][3];
  double lm[3], am[3], ke;
  
  sdinit();
  
  sdstate(0, q, u);
  
  sdsys(mom, com, I);

  double btj[3] = {0};
  double pos[3] = {0};

  sdpos(0, btj, pos);
  printf("pos %g %g %g\n\n", pos[0], pos[1], pos[2]);
  
  sdgetbtj(0, btj);
  printf("btj %g %g %g\n\n", btj[0], btj[1], btj[2]);
  sdpos(0, btj, pos);
  printf("pos %g %g %g\n\n", pos[0], pos[1], pos[2]);

  printf("%g, %g %g %g\n\n", mom[0], -com[0]+pos[0], -com[1]+pos[1], -com[2]+pos[2]);
  
  printf("%g %g %g\n", I[0][0], I[0][1], I[0][2]);
  printf("%g %g %g\n", I[1][0], I[1][1], I[1][2]);
  printf("%g %g %g\n", I[2][0], I[2][1], I[2][2]);
}
