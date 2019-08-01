//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WearMEBenchTopControlSystem
{
    class SignalProcessing
    {
        public Butterworth position_filter;
        public Butterworth emg_highpass_filter;
        public Butterworth emg_lowpass_filter;
        public Butterworth linear_envelope_filter;

        public SignalProcessing()
        {
            //Create Filters
            CreatePositionFilter();
            CreateEMGFilters();
            CreateLinearEnvelopeFilter();
        }

        ///<summary> A method for creating a Butterworth filter, based on the coefficents of the discrete filter, for filtering angle data. </summary>
        ///<returns> N/A </returns>
        void CreatePositionFilter()
        {
            //Create Butterworth Low Pass Filter for angle data - 6Hz, order 4
            int coefficients = 5;
            List<double> B = new List<double>(coefficients);
            List<double> A = new List<double>(coefficients);

            B.Add(Convert.ToSingle(7.69909892783804e-09));
            B.Add(Convert.ToSingle(3.07963957113522e-08));
            B.Add(Convert.ToSingle(4.61945935670283e-08));
            B.Add(Convert.ToSingle(3.07963957113522e-08));
            B.Add(Convert.ToSingle(7.69909892783804e-09));

            A.Add(Convert.ToSingle(1.000000000000000));
            A.Add(Convert.ToSingle(-3.95074409047721));
            A.Add(Convert.ToSingle(5.85344171948211));
            A.Add(Convert.ToSingle(-3.85463384437136));
            A.Add(Convert.ToSingle(0.951936338552047));

            position_filter = new Butterworth(B, A);
        }

        ///<summary> A method for creating a Butterworth filter, based on the coefficents of the discrete filter, for filtering EMG data. </summary>
        ///<returns> N/A </returns>
        void CreateEMGFilters()
        {
            int coefficients = 3;
            List<double> B1 = new List<double>(coefficients);
            List<double> A1 = new List<double>(coefficients);
                
            B1.Add(Convert.ToSingle(0.956543225556877));
            B1.Add(Convert.ToSingle(-1.91308645111375));
            B1.Add(Convert.ToSingle(0.956543225556877));
                
            A1.Add(Convert.ToSingle(1.000000000000000));
            A1.Add(Convert.ToSingle(-1.91119706742607));
            A1.Add(Convert.ToSingle(0.914975834801434));

            //Create Butterworth High Pass filter for EMG data - 20Hz, order 2
            emg_highpass_filter = new Butterworth(B1, A1);

            coefficients = 3;
            List<double> B2 = new List<double>(coefficients);
            List<double> A2 = new List<double>(coefficients);
                 
            B2.Add(Convert.ToSingle(0.292893218813452));
            B2.Add(Convert.ToSingle(0.585786437626905));
            B2.Add(Convert.ToSingle(0.292893218813452));
            
            A2.Add(Convert.ToSingle(1.000000000000000));
            A2.Add(Convert.ToSingle(-2.22044604925031e-16));
            A2.Add(Convert.ToSingle(0.171572875253810));

            //Create Butterworth Low Pass filter for EMG data - 500Hz, order 2
            emg_lowpass_filter = new Butterworth(B2, A2);
        }

        void CreateLinearEnvelopeFilter()
        {
            int coefficients = 5;
            List<double> B = new List<double>(coefficients);
            List<double> A = new List<double>(coefficients);
            
            B.Add(Convert.ToSingle(7.69909892783804e-09));
            B.Add(Convert.ToSingle(3.07963957113522e-08));
            B.Add(Convert.ToSingle(4.61945935670283e-08));
            B.Add(Convert.ToSingle(3.07963957113522e-08));
            B.Add(Convert.ToSingle(7.69909892783804e-09));

            A.Add(Convert.ToSingle(1.000000000000000));
            A.Add(Convert.ToSingle(-3.95074409047721));
            A.Add(Convert.ToSingle(5.85344171948211));
            A.Add(Convert.ToSingle(-3.85463384437136));
            A.Add(Convert.ToSingle(0.951936338552047));

            linear_envelope_filter = new Butterworth(B, A);
        }

        ///<summary> A method for using a Butterworth filter to filter the angle samples. </summary>
        ///<param name="angle_samples"> A pointer to an array of angle samples. </param>
        ///<param name="previous_angle"> A pointer to an array, with length of the filter order, of previous angle samples. </param>
        ///<returns> A pointer to an array of filtered angle data. </returns>
        public List<double> FilterPosition(ref List<double> angle_samples, ref List<double> previous_angle_samples)
        {
            List<double> filtered_angle = position_filter.process(ref angle_samples, ref previous_angle_samples, angle_samples.Count);
            return filtered_angle;
        }

        ///<summary> A method for using a Butterworth filter to filter the triceps EMG samples. </summary>
        ///<param name="triceps_samples"> A pointer to an array of triceps EMG samples. </param>
        ///<param name="previous_triceps_samples"> A pointer to an array, with length of the filter order, of previous triceps EMG samples. </param>
        ///<returns> A pointer to an array of filtered triceps EMG data. </returns>
        public List<double> FilterEMG(ref List<double> emg_samples, ref List<double> previous_emg_samples)
        {
            List<double> highpassed_emg = emg_highpass_filter.process(ref emg_samples, ref previous_emg_samples, emg_samples.Count);
            List<double> filtered_emg = emg_lowpass_filter.process(ref highpassed_emg, ref previous_emg_samples, emg_samples.Count);
            return filtered_emg;
        }

        public List<double> LinearEnvelope(ref List<double> emg_samples, ref List<double> previous_emg_samples)
        {
            List<double> filtered_emg = linear_envelope_filter.process(ref emg_samples, ref previous_emg_samples, emg_samples.Count);
            return filtered_emg;
        }



        ///<summary> A general purpose normalization function. </summary>
        ///<param name="values"> A pointer to an array of data values. </param>
        ///<param name="min"> The minimum value of the data values. </param>
        ///<param name="max"> The maximum value of the data values. </param>
        ///<returns> A pointer to an array of normalized values. </returns>
        public List<double> Normalize(ref List<double> values, double min, double max)
        {
            List<double> normalizedvalues = new List<double>(values.Count);
            for (int i = 0; i < values.Count; i++)
            {
                if (values[i] < min)
                    normalizedvalues.Add((min+(0.01*min) - min) / (max - min));
                else if (values[i] > max)
                    normalizedvalues.Add(1);
                else
                    normalizedvalues.Add((values[i] - min) / (max - min));
            }
            return normalizedvalues;
        }

        public List<double> Rectify(ref List<double> values)
        {
            List<double> rectified_values = new List<double>(values.Count);
            for (int i = 0; i < values.Count; i++)
                rectified_values.Add(Math.Abs(values[i]));
            return rectified_values;
        }

        ///<summary> A method used to compute the neural activation of a muscle. </summary>
        ///<param name="values"> A pointer to an array of EMG data values. </param>
        ///<returns> A pointer to an array of neural activation values. </returns>
        public List<double> NeuralActivation(ref List<double> values)
        {
            //Zajac Model
            double gamma1 = -0.99;
            double gamma2 = -0.79;
            double beta1 = gamma1 + gamma2;
            double beta2 = gamma1 * gamma2;
            double alpha = 1 + beta1 + beta2;
            List<double> neuralactivation = new List<double>(values.Count);

            neuralactivation.Add((double)(alpha * values[0] - beta1 * 0 - beta2 * 0));
            neuralactivation.Add((double)(alpha * values[1] - beta1 * neuralactivation[0] - beta2 * 0));
            neuralactivation.Add((double)(alpha * values[2] - beta1 * neuralactivation[1] - beta2 * neuralactivation[0]));

            for (int i = 3; i < values.Count; i++)
            {
                neuralactivation.Add((double)(alpha * values[i] - beta1 * neuralactivation[i - 1] - beta2 * neuralactivation[i - 2]));
            }

            return neuralactivation;
        }

        public List<double> RemoveOffset(ref List<double> values)
        {
            List<double> new_values = new List<double>(values.Count);
            double average = 0;
            for (int i = 0; i < values.Count; i++)
                average += values[i];
            average /= values.Count;
            for (int i = 0; i < values.Count; i++)
                new_values.Add(values[i] - average);
            return new_values;
        }
    }
}
