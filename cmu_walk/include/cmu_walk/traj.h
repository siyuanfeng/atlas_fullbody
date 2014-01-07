/*
 * =====================================================================================
 *
 *       Filename:  traj.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  01/20/2013 09:15:21 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */


#ifndef __TRAJ_H
#define __TRAJ_H

#include <vector> 
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fstream>

#include "Utils.hpp"

//#include "gsl_utils.h"
//#define MAX_TRAJ_LENG   10000

/*
inline bool write_n_bin_double(const double *data, int len, std::ofstream &out)
{
  out.write((const char *)data, len*sizeof(double));
  return out.good();  
}

inline bool read_n_bin_double(double *data, int len, std::ifstream &in)
{
  in.read((char *)data, len*sizeof(double));
  return in.good();
}
*/

template <size_t pd, size_t ud> class TrajPoint {
  private:
    void init() 
    {
      time = 0;
      type = 0;
      pos = x;
      vel = x+1*pd;
      acc = x+2*pd;
      for (size_t i = 0; i < pd*3; i++)
        x[i] = 0;
      for (size_t i = 0; i < ud; i++)
        u[i] = 0; 
    }

  public:
    double time;
    int type;
    
    double x[3*pd];
    double u[ud];
    
    double *pos;
    double *vel;
    double *acc;

    TrajPoint() 
    {
      init();  
    }

    TrajPoint(double time, int type, const double p[pd], const double v[pd], const double a[pd], const double tau[ud]) 
    {
      init();
      this->time = time;
      this->type = type;
      if (p)
        memcpy(pos, p, sizeof(double)*pd);
      if (v)
        memcpy(vel, v, sizeof(double)*pd);
      if (a)
        memcpy(acc, a, sizeof(double)*pd);
      if (tau)
        memcpy(u, tau, sizeof(double)*ud);  
    }

    TrajPoint(const TrajPoint &other) 
    {
      init();
      time = other.time;
      type = other.type;
      for (size_t i = 0; i < pd*3; i++)
        x[i] = other.x[i];
      for (size_t i = 0; i < ud; i++)
        u[i] = other.u[i];  
    }

    ~TrajPoint() 
    {
      pos = NULL;
      vel = NULL;
      acc = NULL;
      time = -1;
      type = -1;
    }
 
    TrajPoint &operator= (const TrajPoint &other) 
    {
      if (&other == this)
        return *this;
      
      time = other.time;
      type = other.type;
      memcpy(x, other.x, sizeof(double)*3*pd); 
      memcpy(u, other.u, sizeof(double)*ud); 

      return *this;
    }
};

template <size_t pd, size_t ud> class Traj {
  private:
    std::vector <TrajPoint<pd, ud> > _traj;

  public:
    inline void reset() { _traj.clear(); }
    
    Traj() {}

    Traj &operator= (const Traj &other) {
      if (&other == this)
        return *this;
     
      _traj = other._traj;

      return *this;
    }

    ~Traj() { reset(); }

    size_t getIdx(double time) const
    {
      assert(_traj.size() > 1);
      size_t idx = 0; 
      double dt = time - _traj[0].time;
      if (dt < 0)
        idx = 0;
      else if (time > _traj[_traj.size()-1].time)
        idx = _traj.size()-1;
      else
        idx = (size_t)(dt / (_traj[1].time - _traj[0].time));

      return idx;  
    }
    
    /*
    inline TrajPoint <pd, ud> &operator[] (double t) { 
      return _traj[getIdx(t)]; 
    }

    inline const TrajPoint <pd, ud> &operator[] (double t) const {
      return _traj[getIdx(t)];
    }
    */

    inline TrajPoint <pd, ud> &operator[] (size_t i) { 
      assert (i < _traj.size());
      return _traj[i]; 
    }

    inline const TrajPoint <pd, ud> &operator[] (size_t i) const {
      assert (i < _traj.size());
      return _traj[i];
    }

    void append(double time, int type, const double *p, const double *v, const double *a, const double *t) 
    {
      _traj.push_back(TrajPoint <pd, ud>(time, type, p, v, a, t));
    }

    void computeVel() 
    {
      const double *pos0, *pos1;
      double dt;
      for (size_t t = 0; t < _traj.size()-1; t++) {
        dt = _traj[t+1].time - _traj[t].time;
        assert(dt >= 1e-6);
        pos1 = _traj[t+1].pos;
        pos0 = _traj[t].pos;

        for (size_t i = 0; i < pd; i++)
          _traj[t].vel[i] = (pos1[i]-pos0[i]) / dt;
      } 
    }

    void computeAcc() 
    {
      const double *vel0, *vel1;
      double dt;
      for (size_t t = 0; t < _traj.size()-1; t++) {
        dt = _traj[t+1].time - _traj[t].time;
        assert(dt >= 1e-6);
        vel1 = _traj[t+1].vel;
        vel0 = _traj[t].vel;

        for (size_t i = 0; i < pd; i++)
          _traj[t].acc[i] = (vel1[i]-vel0[i]) / dt;
      }
    }

    void shiftAcc(int offset, int i)
    {
      int t = 0;
      if (abs(offset) >= _traj.size() || offset == 0 || i >= pd || i < 0)
        return;
      else if (offset < 0) {
        for (t = 0; t < (int)_traj.size()-offset; t++) {
          _traj[t].acc[i] = _traj[t+offset].acc[i];
        }
        for ( ; t < (int)_traj.size(); t++) {
          _traj[t].acc[i] = 0;
        }
      }
      else if (offset > 0) {
        for (t = (int)_traj.size()-1; t >= 0 + offset; t--) {
          _traj[t].acc[i] = _traj[t-offset].acc[i];
        }
        for ( ; t >= 0; t--) {
          _traj[t].acc[i] = 0;
        }
      }
    }
    
    // only sets the pos and vel, not changing acc
    inline void setX(size_t i, const double *pv) 
    {
      assert(i < _traj.size()); 
      memcpy(_traj[i].x, pv, 2*pd);
    }

    inline void setU(size_t i, const double *u) 
    {
      assert(i < _traj.size()); 
      memcpy(_traj[i].u, u, ud);
    }
    
    inline size_t size() const { return _traj.size(); }
    
    void printTraj() const 
    {
      for (size_t i = 0; i < size(); i++) {
        const TrajPoint <pd, ud> &ptr = _traj[i];
        printf("t %g, type %d\n", ptr.time, ptr.type);
        printf("\t");
        for (size_t j = 0; j < pd; j++)
          printf("%g ", ptr.pos[j]);
        printf("\n\t");
        for (size_t j = 0; j < pd; j++)
          printf("%g ", ptr.vel[j]);
        printf("\n\t");
        for (size_t j = 0; j < pd; j++)
          printf("%g ", ptr.acc[j]);
        printf("\n\t");
        for (size_t j = 0; j < ud; j++)
          printf("%g ", ptr.u[j]);
        printf("\n");
      }
    }

    inline bool fromFileNoType(const char *name, bool ascii = false) 
    {
      std::ifstream in;
      in.open(name);
      bool ret = fromFile(in, false, ascii);
      in.close();
      return ret;
    }

    inline bool fromFileWithType(const char *name, bool ascii = false) 
    {
      std::ifstream in;
      in.open(name);
      bool ret = fromFile(in, true, ascii);
      in.close();
      return ret;
    }

    inline void toFile(const char *name, bool ascii = false, bool noHeader = false) const
    { 
      std::ofstream of;
      of.open(name);
      toFile(of, ascii, noHeader); 
      of.close();
    }
    
    bool fromFile(std::ifstream &in, bool typeFlag = true, bool ascii = false) 
    {
      double tmp[3];
      if (!in.is_open()) {
        printf("trajectory file not open!\n");
        return false;
      }

      size_t a, b, len;

      if (ascii) {
        in >> a >> b >> len;
        assert(a == pd && b == ud);
      }
      else {
        read_n_bin_double(tmp, 3, in);
        printf("%g %g %g\n", tmp[0], tmp[1], tmp[2]);
        //_p_dim = (int)tmp[0];
        //_u_dim = (int)tmp[1];
        len = (size_t)tmp[2];
      }
      printf("x dim %lu, u dim %lu, size %lu\n", pd, ud, len);

      if (_traj.size() != 0)
        reset();

      for (size_t i = 0; i < len; i++) {
        TrajPoint <pd, ud> ptr;
        if (ascii)
          in >> ptr.time;
        else
          read_n_bin_double(&(ptr.time), 1, in);

        if (typeFlag) {
          if (ascii)
            in >> ptr.type;
          else {
            read_n_bin_double(tmp, 1, in);
            ptr.type = (int)tmp[0];
          }
        }

        if (ascii) {
          for (size_t n = 0; n < pd; n++)
            in >> ptr.pos[n];
          for (size_t n = 0; n < pd; n++)
            in >> ptr.vel[n];
          for (size_t n = 0; n < pd; n++)
            in >> ptr.acc[n];

          for (size_t n = 0; n < ud; n++)
            in >> ptr.u[n];
        }
        else {
          read_n_bin_double(ptr.pos, pd, in);
          read_n_bin_double(ptr.vel, pd, in);
          read_n_bin_double(ptr.acc, pd, in);
          read_n_bin_double(ptr.u, ud, in);
        }

        _traj.push_back(ptr);
      }
      assert(in.good());
      return true;  
    }

    void toFile(std::ofstream &outFile, bool ascii = false, bool noHeader = false) const
    {
      double tmp[3];
      tmp[0] = pd;
      tmp[1] = ud;
      tmp[2] = _traj.size();
      
      if (!outFile.is_open()) {
        printf("trajectory file not open!\n");
        return;
      }

      if (!noHeader) {
        if (ascii)
          outFile << pd << " " << ud << " " << _traj.size() << std::endl;
        else
          write_n_bin_double(tmp, 3, outFile);
      }

      for (size_t i = 0; i < _traj.size(); i++) {
        const TrajPoint <pd, ud> &ptr = _traj[i];
        if (ascii) {
          outFile << ptr.time << " " << ptr.type << " ";
          for (size_t n = 0; n < pd; n++)
            outFile << ptr.pos[n] << " ";
          for (size_t n = 0; n < pd; n++)
            outFile << ptr.vel[n] << " ";
          for (size_t n = 0; n < pd; n++)
            outFile << ptr.acc[n] << " ";
          for (size_t n = 0; n < ud-1; n++)
            outFile << ptr.u[n] << " ";
          if (ud > 0)
            outFile << ptr.u[ud-1];
          outFile << std::endl; 
        }
        else {
          tmp[0] = ptr.time;
          tmp[1] = ptr.type;
          write_n_bin_double(tmp, 2, outFile);
          write_n_bin_double(ptr.pos, pd, outFile);
          write_n_bin_double(ptr.vel, pd, outFile);
          write_n_bin_double(ptr.acc, pd, outFile);
          write_n_bin_double(ptr.u, ud, outFile);
        }
      }
    }
};

#endif
