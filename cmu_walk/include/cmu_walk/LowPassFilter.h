#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

class LowPassFilter 
{
  private:
    double hist;

  public:
    LowPassFilter() 
    {
      hist = 0;  
    }

    double filt(double in, double alpha) 
    {
      if (alpha > 1)
        alpha = 1;
      else if (alpha < 0)
        alpha = 0;

      hist = hist * alpha + in * (1-alpha);
      return hist;
    }
};

#endif
