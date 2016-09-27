/*
 * fft.h
 *
 *  Created on: Sep 24, 2016
 *      Author: Loc
 */

#ifndef DSPLIB_INC_FFT_H_
#define DSPLIB_INC_FFT_H_

#include <complex>

using namespace std;

class FFTc
{
public:
	static FFTc* GetInstance();
	int Setup(complex<float> *pTable, int NU);
	int GenerateTwiddleTable(complex<float> *pTable, int NU);
	int ConvertReal2Complex(complex<float> *pXc, int NFFTPoints, float *pXr, int DATAPoints);
	void FFT(complex<float> *pxc, complex<float> *pTwiddle, int NU, int NFFTPoints);
	int ConvertComplex2MagnitudedB(complex<float> *pFFTOut, signed char *pFFTMagdB, int NFFTPoints);

protected:
	FFTc(void);
	~FFTc(void);

private:
	void BitReverseArray(complex<float> *pXc, int NFFTPoints);
	void FFTRad2(complex<float> *pXc, complex<float> *pTwiddleTable, int NFFTPoints, int ePower);
};


#endif /* DSPLIB_INC_FFT_H_ */
