//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using WearMECSF;

namespace WearMEBenchTopControlSystem
{
    class KalmanFilterEstimationController : EstimationController
    {

        //private
        double R;  //The R coefficient of the Kalman filter
        double Q;  //The Q coefficient of the Kalman filter
        double muscle_amplification; //a constant to amplify the neural activity
        Body body;
        SignalProcessing signal_processing;
        List<double> optimization_parameters;
        public double previous_position;
        ///<summary>The default contructor initialzes internal variables to defaults.</summary>
	    public KalmanFilterEstimationController()
        {
            R = 0;
            Q = 0;
            muscle_amplification = 0;
            body = new Body();
            signal_processing = new SignalProcessing();
            optimization_parameters = new List<double>();
        }

        ///<summary>A constructor that initializes the model with the appropriate Kalman filter values. </summary>
        ///<param name="r_coefficient"> The R coefficient of the Kalman filter. </param>
        ///<param name="q_coefficient"> The Q coefficient of the Kalman filter. </param>
        public KalmanFilterEstimationController(double _R, double _Q, int _muscle_amplifcation, Body _body)
        {
            R = _R;
            Q = _Q;
            muscle_amplification = _muscle_amplifcation;
            body = _body;
            signal_processing = new SignalProcessing();
            optimization_parameters = new List<double>();
        }

        //parameters: List 1-> joint_angle, List 2-> biceps_emg, List 3-> triceps_emg, List 4-> prev_angle, List 5-> prev_biceps_emg, List 6-> previous_triceps_emg
        public List<double> EstimatePosition(List<List<double>> parameters)
        { 
            List<double> biceps_emg = parameters[0];
            List<double> triceps_emg = parameters[1];
            List<double> joint_angle = parameters[2];
            List<double> previous_biceps_emg = parameters[3];
            List<double> previous_triceps_emg = parameters[4];
            List<double> previous_joint_angle = parameters[5];

            //Sum the neural activity
            List<double> total_muscle_activity = SumNeuralActivity(ref biceps_emg, ref triceps_emg);

            //This is used in Stacey's model, otherwise neural activity is not between 0-2
            for (int i = 0; i < joint_angle.Count; i++)
                if (total_muscle_activity[i] < 0)
                    total_muscle_activity[i] = 0;

            //Amplify the neural activity
            total_muscle_activity = AmplifyNeuralActivity(ref total_muscle_activity);

            //Kalman Filter to estimate angle
            joint_angle = KalmanFilter(ref total_muscle_activity, ref joint_angle);

            return joint_angle;
        }

        List<double> EstimationController.EstimateVelocity(List<List<double>> parameters)
        {
            throw new NotImplementedException();
        }

        List<double> EstimationController.EstimateAcceleration(List<List<double>> parameters)
        {
            throw new NotImplementedException();
        }

        List<double> EstimationController.EstimateTorque(List<List<double>> parameters)
        {
            throw new NotImplementedException();
        }

        List<double> EstimationController.EstimateForce(List<List<double>> parameters)
        {
            throw new NotImplementedException();
        }

        ///<summary> A method used to compute the neural activation of a muscle. </summary>
        ///<param name="values"> A pointer to an array of EMG data values. </param>
        ///<returns> A pointer to an array of neural activation values. </returns>
        List<double> NeuralActivation(ref List<double> values)
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

        ///<summary> A method for the summation of the neural activations of the biceps and triceps muscles. </summary>
        ///<param name="biceps_activation"> A pointer to an array of neural activation values of the biceps muscle. </param>
        ///<param name="triceps_activation"> A pointer to an array of neural activation values of the triceps muscle. </param>
        ///<returns> A pointer to an array of summed neural activation values. </returns>
        List<double> SumNeuralActivity(ref List<double> biceps_activation, ref List<double> triceps_activation)
        {
            List<double> total_muscle_activity = new List<double>(biceps_activation.Count);
            for (int i = 0; i < biceps_activation.Count; i++)
                total_muscle_activity.Add(biceps_activation[i] + triceps_activation[i]);
            return total_muscle_activity;
        }

        ///<summary> A method used to amplify the neural activation values by a constant factor. </summary>
        ///<param name="neural_activitation"> A pointer to an array of EMG data values. </param>
        ///<returns> A pointer to an array of amplified neural activation values. </returns>
        List<double> AmplifyNeuralActivity(ref List<double> neural_activitation)
        {
            List<double> amplified_activtion = new List<double>(neural_activitation.Count);
            for (int i = 0; i < neural_activitation.Count; i++)
                amplified_activtion.Add(Math.Abs(neural_activitation[i] * muscle_amplification));
            return amplified_activtion;
        }

        ///<summary> A Kalman filter designed to estimate the elbow angle based on previous elbow angle position and the summed neural activation of the biceps and triceps muscles. </summary>
        ///<param name="neural_activation"> A pointer to an array of neural activation values. </param>
        ///<param name="filtered_angle"> A pointer to an array of filtered angle values. </param>
        ///<returns> A pointer to an array of estimated elbow angle values. </returns>
        List<double> KalmanFilter(ref List<double> neural_activation, ref List<double> filtered_angle)
        {
            List<double> xhat = new List<double>(neural_activation.Count); //estimated angle
            List<double> phat = new List<double>(neural_activation.Count);
            List<double> K = new List<double>(neural_activation.Count);
            xhat.Add(filtered_angle[0]);
            phat.Add(0);
            xhat.Add(filtered_angle[1]); //total_muscle_activity[0]
            phat.Add(1);
            K.Add(1);
            K.Add(1);
            for (int i = 2; i < neural_activation.Count; i++)
            {
                xhat.Add((xhat[i - 1] + filtered_angle[i - 2]) / 2);
                phat.Add(Convert.ToSingle(phat[i - 1] + Q));
                K.Add(Convert.ToSingle(phat[i] / (phat[i] + R)));
                xhat[i] = xhat[i] + (K[i] * (neural_activation[i] - xhat[i]));
                phat[i] = (1 - K[i]) * phat[i];
            }
            return xhat;
        }

        ///<summary> A method for setting the Kalman filter coefficients. </summary>
        ///<param name="r_coefficient"> The R coefficient of the Kalman filter. </param>
        ///<param name="q_coefficient"> The Q coefficient of the Kalman filter. </param>
        ///<returns> N/A </returns>
        void SetKalmanCoefficients(List<double> coefficients)
        {
            R = coefficients[0];
            Q = coefficients[1];
        }

        List<double> GetKalmanCoefficients()
        {
            List<double> coefficients = new List<double>(2);
            coefficients.Add(R);
            coefficients.Add(Q);
            return coefficients;
        }

        public void SetOptimizationParameters(List<double> optimized_parameters)
        {
            optimization_parameters = optimized_parameters;
            muscle_amplification = optimization_parameters[0];
            R = optimization_parameters[1];
            Q = optimization_parameters[2];
        }

        public List<double> GetOptimizationParameters()
        {
            return optimization_parameters;
        }

        public void SetBody(WearMECSF.Body _body)
        {
            body = (Body)_body;
        }

        public void SetPreviousPosition(double _previous_position)
        {
            previous_position = _previous_position;
        }
    }
}
