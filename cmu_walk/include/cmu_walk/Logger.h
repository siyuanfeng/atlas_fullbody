/*
 * =====================================================================================
 *
 *       Filename:  Logger.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/13/2013 04:59:55 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#ifndef __LOGGER_H
#define __LOGGER_H

#include <stdio.h>
#include <string.h>
#include <eigen3/Eigen/Geometry>

#define LOGGER_MAX_CHANNELS 1500 
#define LOGGER_MAX_CHARS 100   

typedef struct{
	char units[LOGGER_MAX_CHARS];
	char names[LOGGER_MAX_CHARS];
  char data_type;
	const void *ptr;
} DataPoint_t;

class Logger {
  public:
    bool _inited;

    int n_points;
    int n_channels;
    int myindex;
    float *data;
    float frequency;
    DataPoint_t datapoints[LOGGER_MAX_CHANNELS]; 
    void init_(double timestep);
    static const int N_QUAT = 100;
    Eigen::Quaterniond *qPoint[N_QUAT];
    double EAbuff[3*N_QUAT];
    int nQuat;
    bool recorded;

  public:
    Logger();
    ~Logger();

    void add_datapoint(const char *names, const char *units, const double *ptr);
    void add_datapoint(const char *names, const char *units, const int *ptr);
    void add_datapoint(const char *names, const char *units, const bool *ptr);
    void add_quat(const char *names, Eigen::Quaterniond *q);

    
    ///////////////////////////////////////////
    // eric debugging tool
    static double tmpOut[20];
    static int errCode;
    static void setErrEW(int err);
    static void setTmpOut(int ind, double val);
    void addEWstatic();
    ///////////////////////////////////////////
    
    virtual void init(double timestep)=0;
    virtual void saveData()=0;

};



class BatchLogger : public Logger {
  public:
    BatchLogger() {;}
    ~BatchLogger() {;}
    void init(double timestep);
    void saveData();
    void writeToMRDPLOT();  
};


#endif
