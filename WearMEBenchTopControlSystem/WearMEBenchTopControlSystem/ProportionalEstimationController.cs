//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using WearMECSF;
using System.IO;

namespace WearMEBenchTopControlSystem
{
    class ProportionalEstimationController : EstimationController
    {
        List<double> channel_gains;
        List<double> optimization_parameters;
        double time_step;
        public double previous_position;
        double previous_velocity;
        double previous_acceleration;
        Body body;
        private StreamWriter output_writer;
        private FileStream output_file;

        public ProportionalEstimationController(double _time_step)
        {
            channel_gains = new List<double>(2);
            for (int i = 0; i < channel_gains.Count; i++)
                channel_gains[i] = 0;
            optimization_parameters = new List<double>();
            time_step = _time_step;
            previous_position= 0;
            previous_velocity = 0;
            previous_acceleration = 0;
            body = new Body();
            output_file = new FileStream("motion data\\proportional.txt", FileMode.Create, FileAccess.Write);
            output_writer = new StreamWriter(output_file);
        }

        ~ProportionalEstimationController()
        {

        }

        public void SetChannelGains(List<double> _channel_gains)
        {
            for (int i = 0; i < _channel_gains.Count; i++)
                channel_gains[i] = _channel_gains[i];
        }

        public List<double> GetChannelGains()
        {
            return channel_gains;
        }

        public void SetPreviousPosition(double _previous_position)
        {
            previous_position = _previous_position;
        }
        /// <summary>
        /// The position of the elbow is estimated based on proportional gains applied to the EMG signals
        /// </summary>
        /// <param name="parameters">In this model, parameters is broken up in the following order: List 1->biceps_emg, List 2->triceps_emg, List 3->gains </param>
        /// <returns></returns>
        public List<double> EstimatePosition(List<List<double>> parameters)
        {
            List<double> position = new List<double>(parameters[0].Count);
            List<double> velocity = new List<double>(parameters[0].Count);
            List<double> acceleration = new List<double>(parameters[0].Count);

            int window_size = parameters[0].Count;
            List<double> biceps_average = parameters[0];
            List<double> triceps_average = parameters[1];
            List<double> subject_positions = parameters[2];

            acceleration.Add(0);
            velocity.Add(0);
            position.Add(subject_positions[0]);
            //acceleration.Add(previous_acceleration);
            //velocity.Add(previous_velocity);
            //position.Add(previous_position);
            output_writer.WriteLine("---");
            for (int i = 1; i < window_size; i++) //sum values over the window
            {
                double muscle_torque = biceps_average[i] * channel_gains[0] - triceps_average[i] * channel_gains[1]; //multiply each channel by the gain
                acceleration.Add(muscle_torque / body.lower_arm_inertia);
                velocity.Add(velocity[i - 1] + acceleration[i] * time_step);
                position.Add(position[i - 1] + velocity[i] * time_step);
                string line = subject_positions[i] + ", " + position[i].ToString() + ", " + velocity[i].ToString() + ", " + acceleration[i].ToString() + ", " + muscle_torque.ToString() + ", " + biceps_average[i].ToString() + ", " + triceps_average[i].ToString();
                output_writer.WriteLine(line);
            }

            previous_position = position[position.Count - 1];
            previous_velocity = velocity[velocity.Count - 1];
            previous_acceleration = acceleration[acceleration.Count - 1];

            return position;
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
            List<double> torque = new List<double>(parameters[0].Count);

            int window_size = parameters[0].Count;
            List<double> biceps_average = parameters[0];
            List<double> triceps_average = parameters[1];

            for (int i = 0; i < window_size; i++) //sum values over the window
            {
                torque.Add(biceps_average[i] * parameters[2][0] - triceps_average[i] * parameters[2][1]); //multiply each channel by the gain
            }

            return torque;
        }

        List<double> EstimationController.EstimateForce(List<List<double>> parameters)
        {
            throw new NotImplementedException();
        }

        public void SetOptimizationParameters(List<double> optimized_parameters)
        {
            optimization_parameters = optimized_parameters;
            channel_gains.Add(optimization_parameters[0]);
            channel_gains.Add(optimization_parameters[1]);
        }

        public List<double> GetOptimizationParameters()
        {
            return optimization_parameters;
        }

        public void SetBody(WearMECSF.Body _body)
        {
            body = (Body)_body;
            body.CalculateLowerArmInertia();
        }
    }
}
