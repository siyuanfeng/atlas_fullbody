#ifndef BUTTER1_DEFINED
#define BUTTER1_DEFINED

class Butter1 {
public:
	Butter1(double frac) {
		a = frac/2.0;
		aa = frac;
		prevOut = 0;
		prevIn = 0;
		inited = false;
	}

	double filt(double input) {
		if(!inited) {
			inited = true;
			prevOut = input;
			prevIn = input;
		}
		double out = prevOut*(1-aa) + input*a + prevIn*a;
		prevIn = input;
		prevOut = out;
		return out;
	}

	double deriv(double input, double frequency) {
		if(!inited) {
			inited = true;
			prevOut = input;
			prevIn = input;
		}
		double out = prevOut*(1-aa) + input*a + prevIn*a;
		double deriv = (out-prevOut)*frequency;
		prevIn = input;
		prevOut = out;
		return deriv;
	}

	double a, aa;
	double prevOut;
	double prevIn;
	bool inited;
};


#endif
