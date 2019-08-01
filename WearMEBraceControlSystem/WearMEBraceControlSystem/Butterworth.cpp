//© 2019. Tyler Desplenter and Ana Luisa Trejos.

#include "Butterworth.h"

namespace WearMEBraceControlSystem
{
	///<summary>The default constructor of the Butterworth object.</summary>
	Butterworth::Butterworth()
	{
		order = 0;
	}

	///<summary>A contructor for the Butterworth object that accepts the filter coefficients.</summary>
	///<param name = "b"> A vector of numerator filter coefficients. </param>
	///<param name = "a"> A vector of denominator filter coefficients. </param>
	Butterworth::Butterworth(vector<double> b, vector<double> a)
	{
		order = a.size() - 1;
		B = b;
		A = a;
	}

	///<summary>A method for filtering a vector of samples based on the difference equation representation of the discrete filter.</summary>
	///<param name="x"> The vector of samples that need to be filtered. </param>
	///<param name="prev"> A vector of previous samples that is as long as the filter order. </param>
	///<param name="samplelength"> The number of samples that need to be processed and returned. </param>
	///<returns> A vector of filtered samples is returned. </returns>
	vector<double> Butterworth::process(vector<double> x, vector<double> prev, int samplelength)
	{
		vector<double> y(samplelength + order);
		for (int n = 0; n < samplelength + order; n++)
		{
			if (n < order)
				y[n] = prev[n];
			else if (n >= order)
			{
				y[n] = 0;
				for (int j = 0; j <= order; j++)
				{
					if (j == 0)
						y[n] += B[j] * x[n];
					else
						y[n] += (B[j] * x[n - j]) + (-A[j] * y[n - j]);
				}
			}
		}
		vector<double> temp(samplelength);
		for (int i = 0; i < samplelength; i++)
			temp[i] = y[i + order];
		return temp;
	}

	///<summary>A method for filtering a vector of samples based on the difference equation representation of the discrete filter.</summary>
	///<param name="x"> The pointer to the array of samples that need to be filtered. </param>
	///<param name="prev"> A pointer to the array of previous samples that is as long as the filter order. </param>
	///<param name="samplelength"> The number of samples that need to be processed and returned. </param>
	///<returns> A pointer to the array of filtered samples is returned. </returns>
	double * Butterworth::process(double * x, double * prev, int samplelength)
	{
		double * y = new double[samplelength + order];
		for (int n = 0; n < samplelength + order; n++)
		{
			if (n < order)
				y[n] = prev[n];
			else if (n >= order)
			{
				y[n] = 0;
				for (int j = 0; j <= order; j++)
				{
					if (j == 0)
						y[n] += B[j] * x[n];
					else
						y[n] += (B[j] * x[n - j]) + (-A[j] * y[n - j]);
				}
			}
		}
		double * temp = new double[samplelength];
		for (int i = 0; i < samplelength; i++)
			temp[i] = y[i + order];
		return temp;
	}
}