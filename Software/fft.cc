/*
 * fft.cc
 *
 *  Created on: Sep 24, 2016
 *      Author: Loc
 */
#include "math.h"
#include "fft.h"


#define PI				(float) 3.141592653589793
#define SWAP(a,b,t)		 t=(a);(a)=(b);(b)=t


int dataLen = 0;


/*****************************************************************************************
 *
 *****************************************************************************************/
FFTc::FFTc()
{

}

/***************************************************************************************
 *
 ***************************************************************************************/
FFTc::~FFTc()
{

}

/*************************************************************************************
 * return a handle.
 *
 ************************************************************************************/
FFTc* FFTc::GetInstance()
{
	static FFTc myInstance;

	return &myInstance;
}

/*************************************************************************************
 * setup the FFTc.  Basically, populate the twiddle table so we don't have to
 * make expensive (CPU resources) calculation during FFT calculation.
 *************************************************************************************/
int FFTc::Setup(complex<float> *pTable, int NU)
{
	FFTc::GenerateTwiddleTable(pTable,NU);

	return 1;
}

/****************************************************************************************
 * calculate Twiddle factors.
 *
 ****************************************************************************************/
int FFTc::GenerateTwiddleTable(complex<float> *pTable, int NU)
{
	int i;
	int e;

	for (i = 1; i <= NU; i++)
	{
		e = 1 << (i-1);
		*pTable++ = complex<float>(cos(PI / (float)e), -sin(PI / (float)e));
	}

	return 1;
}

/*********************************************************************************************
 * Convert real data point to complex data point for the FFT calculation.
 * NOTE, with a some changes to the FFT code, you can calculate FFT for real data points and
 * save lots of CPU cycles.
 *********************************************************************************************/
int FFTc::ConvertReal2Complex(complex<float> *pXc, int NFFTPoints, float *pXr, int DATAPoints)
{
	int i,j;

	for (i = 0; i < DATAPoints; i++)
	{
		*pXc++ = complex<float>(*pXr++, 0);
	}

	// padded with zero to make number of samples = nearest 2^N power.
	if (i < NFFTPoints)
	{
		i = i - 1;		// backoff index by one.
		pXc--;			// backoff pointer by one sample.
		for (j = i;j<NFFTPoints;j++)
		{
			*pXc++ = complex<float>(0,0);
		}
	}

	return 1;
}

/**************************************************************************************
 * FFT algorithm.
 *
 **************************************************************************************/
void FFTc::FFT(complex<float> *pxc, complex<float> *pTwiddle, int NU, int NFFTPoints)
{

	FFTc::BitReverseArray(pxc,NFFTPoints);

	FFTc::FFTRad2(pxc,pTwiddle,NFFTPoints,NU);

	return;
}

/***************************************************************************************************************
 * convert FFT complex output to dB.
 * Quantized the output dB to sign char (i.e. 8-bit signed data) so we can easily transmit
 * this 8-bit data through UART(LITE).  The FFT magnitude resolution is 1dB and range from -127 dB to +127 dB
 * you can introduce an offset (if you scale the time data appropriately) to cover a range of -255 dB to 0 dB.
 ***************************************************************************************************************/
int FFTc::ConvertComplex2MagnitudedB(complex<float> *pFFTOut, signed char *pFFTMagdB, int NFFTPoints)
{
	int i;

	for(i=0;i<NFFTPoints;i++)
	{
		*pFFTMagdB++ = (signed char) round(20*log10(abs(*pFFTOut++)));
	}

	return 1;
}

/**************************************************************************************************************
 * do bit reverse of the index of the FFT input sample array.
 *
 **************************************************************************************************************/
void FFTc::BitReverseArray(complex<float> *pXc, int NFFTPoints)
{
	complex<float> Temp;
	int N, NHALF, NMINUS1;
	int k,j, i;

	N = NFFTPoints;
	NHALF = N>>1;
	NMINUS1 = N - 1;

	j = 0;
	for (i = 0; i < NMINUS1; i++)
	{
		if (i < j)
		{
			SWAP(pXc[j],pXc[i],Temp);
		}
		k = NHALF;
		while (k >= 1 && j >= k)
		{
			j -= k;
			k  = k>>1;
		}
		j += k;
	}

	return;
}

/****************************************************************************************************************
 * Calculate the FFT base on Radix2
 * This FFT algorithm only take (NFFTPoints/2)*Log2(NFFTPoints) instead of NFFTPoints**2 complex multiplication
 ****************************************************************************************************************/
void FFTc::FFTRad2(complex<float> *pXc, complex<float> *pTwiddleTable, int NFFTPoints, int ePower)
{
	complex<float> F, W, T;
	int S,SDIV2,INDEX;
	int j, i,l;

	S = 2;
	for (l = 1; l <= ePower; l++)
	{
		SDIV2 = S >> 1;
		F = complex<float>(1.0, 0.0);
		W = pTwiddleTable[l - 1];
		for (j = 0; j < SDIV2; j++)
		{
			for (i = j; i < NFFTPoints;i=i+S)
			{
				INDEX = i + SDIV2;
				T = pXc[INDEX]*F;
				pXc[INDEX] = pXc[i] - T;
				pXc[i] = pXc[i] + T;
			}
			F = F*W;
		}
		S = S * 2;
	}

	return;
}

