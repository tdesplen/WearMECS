//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace WearMEBenchTopControlSystem
{
    class Butterworth
    {
	    //private:
		List<double> B; //nominator coefficient
		List<double> A; //denominator coefficients
		int order; //filter order

        ///<summary>The default constructor of the Butterworth object.</summary>
        Butterworth()
        {
	        order = 0;
        }

        public int GetOrder()
        {
            return order;
        }

        ///<summary>A contructor for the Butterworth object that accepts the filter coefficients.</summary>
        ///<param name = "b"> A List of numerator filter coefficients. </param>
        ///<param name = "a"> A List of denominator filter coefficients. </param>
        public Butterworth(List<double> b, List<double> a)
        {
	        order = a.Count - 1;
	        B = b;
	        A = a;
        }

        ///<summary>A method for filtering a List of samples based on the difference equation representation of the discrete filter.</summary>
        ///<param name="x"> The List of samples that need to be filtered. </param>
        ///<param name="prev"> A List of previous samples that is as long as the filter order. </param>
        ///<param name="samplelength"> The number of samples that need to be processed and returned. </param>
        ///<returns> A List of filtered samples is returned. </returns>
        public List<double> process(ref List<double> x, ref List<double> prev, int samplelength)
        {
            List<double> y = new List<double>(samplelength);
            for (int n = 0; n < samplelength; n++)
            {
                y.Add(0);
                if (n < order)
                {
                    y[n] = prev[prev.Count - 1 - order + n];
                }
                else if (n >= order)
                {
                    for (int j = 0; j <= order; j++)
                    {
                        if (j == 0)
                            y[n] += B[j] * x[n];
                        else
                            y[n] += (B[j] * x[n - j]) + (-A[j] * y[n - j]);
                    }
                }
            }
            //List<double> temp = new List<double>(samplelength);
            //for (int i = 0; i < samplelength; i++)
            // temp.Add(y[i + order]);
            //return temp;
            return y;
        }

        //public List<double> process(ref List<double> x, ref List<double> prev, int samplelength)
        //{
        //    List<double> y = new List<double>(samplelength);
        //    for (int n = 0; n < samplelength; n++)
        //    {
        //        y.Add(0);
        //        if (n < order)
        //        {
        //            for (int j = 0; j <= n; j++)
        //            {
        //                if (j == 0)
        //                    y[n] += B[j] * x[n];
        //                else
        //                    y[n] += (B[j] * x[n - j]) + (-A[j] * y[n - j]);
        //            }
        //        }
        //        else if (n >= order)
        //        {
        //            for (int j = 0; j <= order; j++)
        //            {
        //                if (j == 0)
        //                    y[n] += B[j] * x[n];
        //                else
        //                    y[n] += (B[j] * x[n - j]) + (-A[j] * y[n - j]);
        //            }
        //        }
        //    }
        //    //List<double> temp = new List<double>(samplelength);
        //    //for (int i = 0; i < samplelength; i++)
        //    // temp.Add(y[i + order]);
        //    //return temp;
        //    return y;
        //}
    }
}
