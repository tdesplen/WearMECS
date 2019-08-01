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
    class PolynomialEstimationController : EstimationController
    {
        List<double> optimization_parameters;
        List<List<double>> coefficients;
        double time_step;
        public double previous_position;
        double previous_velocity;
        double previous_acceleration;
        Body body;
        private StreamWriter output_writer;
        private FileStream output_file;

        public PolynomialEstimationController(double _time_step, int _order)
        {
            time_step = _time_step;
            optimization_parameters = new List<double>();
            coefficients = new List<List<double>>();
            coefficients.Add(new List<double>(_order+1));
            coefficients.Add(new List<double>(_order+1));
            previous_position = 0;
            previous_velocity = 0;
            previous_acceleration = 0;
            body = new Body();
            output_file = new FileStream("motion data\\polynomial.txt", FileMode.Create, FileAccess.Write);
            output_writer = new StreamWriter(output_file);
        }

        public void SetPreviousPosition(double _previous_position)
        {
            previous_position = _previous_position;
        }

        List<double> EstimationController.EstimateAcceleration(List<List<double>> parameters)
        {
            throw new NotImplementedException();
        }

        List<double> EstimationController.EstimateForce(List<List<double>> parameters)
        {
            throw new NotImplementedException();
        }

        public List<double> EstimatePosition(List<List<double>> parameters)
        {
            List<double> position = new List<double>(parameters[0].Count);
            List<double> velocity = new List<double>(parameters[0].Count);
            List<double> acceleration = new List<double>(parameters[0].Count);

            int window_size = parameters[0].Count;
            List<double> biceps_emg = parameters[0];
            List<double> triceps_emg = parameters[1];
            List<double> subject_positions = parameters[2];

            //acceleration.Add(previous_acceleration);
            //position.Add(previous_position);
            //velocity.Add(previous_velocity);
            position.Add(subject_positions[0]);
            velocity.Add(0);
            acceleration.Add(0);
            output_writer.WriteLine("---");
            for (int i = 1; i < window_size; i++) //sum values over the window
            {
                double biceps_torque = (coefficients[0][0] * (biceps_emg[i] * biceps_emg[i] * biceps_emg[i])) + (coefficients[0][1] * (biceps_emg[i] * biceps_emg[i])) + (coefficients[0][2] * biceps_emg[i]) + coefficients[0][3];
                double triceps_torque = (coefficients[1][0] * (triceps_emg[i] * triceps_emg[i] * triceps_emg[i])) + (coefficients[1][1] * (triceps_emg[i] * triceps_emg[i])) + (coefficients[1][2] * triceps_emg[i]) + coefficients[1][3];
                double muscle_torque = biceps_torque - triceps_torque;
                acceleration.Add(muscle_torque / body.lower_arm_inertia);
                velocity.Add(velocity[i-1] + acceleration[i] * time_step);
                position.Add(position[i-1] + velocity[i] * time_step);
                string line = subject_positions[i] + ", " + position[i].ToString() + ", " + velocity[i].ToString() + ", " + acceleration[i].ToString() + ", " + biceps_torque.ToString() + ", " + triceps_torque.ToString() + ", " + biceps_emg[i].ToString() + ", " + triceps_emg[i].ToString();
                output_writer.WriteLine(line);
            }

            previous_position = position[position.Count - 1];
            previous_velocity = velocity[velocity.Count - 1];
            previous_acceleration = acceleration[acceleration.Count - 1];

            return position;
        }

        List<double> EstimationController.EstimateTorque(List<List<double>> parameters)
        {
            throw new NotImplementedException();
        }

        List<double> EstimationController.EstimateVelocity(List<List<double>> parameters)
        {
            throw new NotImplementedException();
        }

        List<double> EstimationController.GetOptimizationParameters()
        {
            throw new NotImplementedException();
        }

        public void SetOptimizationParameters(List<double> optimized_parameters)
        {
            optimization_parameters = optimized_parameters;
            for (int i = 0; i < 4; i++)
            {
                coefficients[0].Add(optimization_parameters[i]);
                coefficients[1].Add(optimization_parameters[i + 4]);
            }
        }

        public void SetBody(WearMECSF.Body _body)
        {
            body = (Body)_body;
            body.CalculateLowerArmInertia();
        }
    }
}
