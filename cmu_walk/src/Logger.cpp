/*
 * =====================================================================================
 *
 *       Filename:  Logger.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  03/13/2013 05:07:33 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */
#include <stdio.h>
#include <stdlib.h>
#include <string> 
#include "Logger.h"
#include "mrdplot.h"
#include "Eigen_utils.hpp"

#define LOGGER_MAX_N_POINTS 500000

#define LOGGER_DATA_TYPE_DOUBLE     0
#define LOGGER_DATA_TYPE_FLOAT      1
#define LOGGER_DATA_TYPE_INT        2
#define LOGGER_DATA_TYPE_BOOL		3

///////////////////////////////////////////
// eric debugging tool
double Logger::tmpOut[20] = {0.0};
int Logger::errCode = 0;

void Logger::setErrEW(int err) {
  errCode = err;
}

void Logger::setTmpOut(int ind, double val) {
  tmpOut[ind] = val;
}

void Logger::addEWstatic() {
  add_datapoint("errCode","-",&errCode);
  for(int i = 0; i < 20; i++) {
    char name[100];
    sprintf(name,"tmpOut[%d]",i);
    add_datapoint(name,"-",&(tmpOut[i]));
  }
}
///////////////////////////////////////////

Logger::Logger() {
  data = NULL;
  n_points = 0;
  myindex = 0;
  frequency = 0;
  n_channels = 0;
  _inited = false;

  nQuat = 0;
  recorded = false;
}

Logger::~Logger()
{
  if (data)
    delete []data;
  data = NULL;
}

void Logger::init_(double timestep)
{
  _inited = true;
  myindex = 0;
  n_points = 0;
  n_channels = 0;
  data = new float[LOGGER_MAX_N_POINTS*LOGGER_MAX_CHANNELS];


  frequency = 1. / timestep;
}

void Logger::add_quat(const char *names, Eigen::Quaterniond *q) {
  qPoint[nQuat] = q;
  char name[100];
  sprintf(name, "%s[R]",names);
  add_datapoint(name, "r", &(EAbuff[nQuat*3]));
  sprintf(name, "%s[P]",names);
  add_datapoint(name, "r", &(EAbuff[nQuat*3+1]));
  sprintf(name, "%s[Y]",names);
  add_datapoint(name, "r", &(EAbuff[nQuat*3+2]));
  nQuat++;
}


void Logger::add_datapoint(const char *names, const char *units, const bool *ptr){
  if (n_channels<LOGGER_MAX_CHANNELS) {
    DataPoint_t *dptr = &datapoints[n_channels];

    strncpy(dptr->names,names,LOGGER_MAX_CHARS-1);
    dptr->names[LOGGER_MAX_CHARS-1] = '\0';
    strncpy(dptr->units,units,LOGGER_MAX_CHARS-1);
    dptr->units[LOGGER_MAX_CHARS-1] = '\0';

    datapoints[n_channels].ptr = (const void *)ptr;
    datapoints[n_channels].data_type = LOGGER_DATA_TYPE_BOOL;
    n_channels++;
  }
}


void Logger::add_datapoint(const char *names, const char *units, const double *ptr){
  if (n_channels<LOGGER_MAX_CHANNELS) {
    DataPoint_t *dptr = &datapoints[n_channels];

    strncpy(dptr->names,names,LOGGER_MAX_CHARS-1);
    dptr->names[LOGGER_MAX_CHARS-1] = '\0';
    strncpy(dptr->units,units,LOGGER_MAX_CHARS-1);
    dptr->units[LOGGER_MAX_CHARS-1] = '\0';

    datapoints[n_channels].ptr = (const void *)ptr;
    datapoints[n_channels].data_type = LOGGER_DATA_TYPE_DOUBLE;
    n_channels++;
  }
}

void Logger::add_datapoint(const char *names, const char *units, const int *ptr){
  if (n_channels<LOGGER_MAX_CHANNELS){
    DataPoint_t *dptr = &datapoints[n_channels];

    strncpy(dptr->names,names,LOGGER_MAX_CHARS-1);
    dptr->names[LOGGER_MAX_CHARS-1] = '\0';
    strncpy(dptr->units,units,LOGGER_MAX_CHARS-1);
    dptr->units[LOGGER_MAX_CHARS-1] = '\0';

    datapoints[n_channels].ptr = (const void *)ptr;
    datapoints[n_channels].data_type = LOGGER_DATA_TYPE_INT;
    n_channels++;
  }
}

//////////////////////////////////////////////////////////////////////
void BatchLogger::init(double timestep)
{
  init_(timestep);
}

bool noInitSent = false;

void BatchLogger::saveData()
{
  static bool err = false;

  if(recorded)	return;		//stop wasting your time


  for(int i = 0; i < nQuat; i++) {
    quat2EA(*(qPoint[i]), &(EAbuff[i*3]));
  }

  const void *ptr;
  if ( n_points >= LOGGER_MAX_N_POINTS) {
    fprintf(stderr, "logger loop back.\n"); 
    n_points = 0;
  }

  if (!_inited) {
    if(!noInitSent) {
    	fprintf(stderr, "logger not inited.\n");
    	noInitSent = true;
    }
    return;
  }

  for(int i = 0; i < n_channels; i++) {
    ptr = datapoints[i].ptr;
    if (datapoints[i].data_type == LOGGER_DATA_TYPE_DOUBLE)
      data[myindex+i] = (float)(*((const double *)ptr));
    else if (datapoints[i].data_type == LOGGER_DATA_TYPE_INT)
      data[myindex+i] = (float)(*((const int *)ptr));
    else if (datapoints[i].data_type == LOGGER_DATA_TYPE_BOOL) {
      if(*((bool *)ptr))		data[myindex+i]	= 1;
      else					data[myindex+i]	= 0;
    }
    //fprintf(stderr, "%s \t\t\t %g\n", datapoints[i].names, data[myindex + i]);
  }
  myindex += n_channels;
  n_points++;
  //fprintf(stderr, "Finished save data \n\n");
} 

void BatchLogger::writeToMRDPLOT()
{
  if (!_inited)
    return;
  recorded = true;
  fprintf(stderr, "LOGGER SAVING DATA ..... \n");
  MRDPLOT_DATA *d;

  d = malloc_mrdplot_data( 0, 0 );
  d->filename = generate_file_name();
  d->n_channels = n_channels;
  d->n_points = n_points;
  d->total_n_numbers = d->n_channels*d->n_points;
  d->frequency = frequency;
  d->data = data;

  d->names = new char*[n_channels];
  d->units = new char*[n_channels];

  for (int i = 0; i < n_channels; i++) {
    d->names[i] = new char[LOGGER_MAX_CHARS];
    d->units[i] = new char[LOGGER_MAX_CHARS];

    strcpy(d->names[i], datapoints[i].names);
    strcpy(d->units[i], datapoints[i].units);
  }

  write_mrdplot_file( d );

  for (int i = 0; i < n_channels; i++) {
    delete []d->names[i];
    delete []d->units[i];
  }
  delete []d->names;
  delete []d->units;

  fprintf(stderr, "LOGGER SAVED DATA\n");
  free(d);
}


