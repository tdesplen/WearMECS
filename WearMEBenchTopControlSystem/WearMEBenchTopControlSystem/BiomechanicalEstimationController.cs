//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using WearMECSF;

namespace WearMEBenchTopControlSystem
{
    class BiomechanicalEstimationController : EstimationController
    {
        private double W; //length-force shaping factor
        private double A; //velocity-force shaping factor
        private double gmax; //maximum normalized eccentric muscle force
        private double timestep; //timestep of sampled data
        private List<List<double>> muscle_activation_coefficients;
        private List<double> optimization_parameters;

        private Body body;
        private SignalProcessing signal_processing;

        public BiomechanicalEstimationController()
        {
            W = 0.56;
            A = 0.25;
            gmax = 1.5;
            timestep = 0;
            muscle_activation_coefficients = new List<List<double>>();
            body = new Body();
            signal_processing = new SignalProcessing();
            optimization_parameters = new List<double>();
        }

        public BiomechanicalEstimationController(double _timestep, Body _body, List<List<double>> _muscle_activation_coefficients)
        {
            W = 0.56;
            A = 0.25;
            gmax = 1.5;
            timestep = _timestep;
            muscle_activation_coefficients = _muscle_activation_coefficients;
            body = _body;
            signal_processing = new SignalProcessing();
            optimization_parameters = new List<double>(2);
        }

        List<double> EstimationController.EstimatePosition(List<List<double>> parameters)
        {
            throw new NotImplementedException();
        }

        List<double> EstimationController.EstimateVelocity(List<List<double>> parameters)
        {
            throw new NotImplementedException();
        }

        List<double> EstimationController.EstimateAcceleration(List<List<double>> parameters)
        {
            throw new NotImplementedException();
        }

        //parameters: List 1-> joint_angle, List 2-> biceps_emg, List 3-> triceps_emg, List 4-> prev_angle, List 5-> prev_biceps_emg, List 6-> previous_triceps_emg
        List<double> EstimationController.EstimateTorque(List<List<double>> parameters)
        {
            List<double> joint_torque = new List<double>();

            List<double> joint_angle = parameters[0];
            List<double> biceps_emg = parameters[1];
            List<double> triceps_emg = parameters[2];
            List<double> previous_joint_angle = parameters[3];
            List<double> previous_biceps_emg = parameters[4];
            List<double> previous_triceps_emg = parameters[5];

            List<double> joint_velocity = GetJointVelocity(ref joint_angle, timestep);
            List<double> joint_acceleration = GetJointAcceleration(ref joint_velocity, timestep);

            //Process the EMG signals
            biceps_emg = signal_processing.FilterEMG(ref biceps_emg, ref previous_biceps_emg);
            triceps_emg = signal_processing.FilterEMG(ref triceps_emg, ref previous_triceps_emg);
            biceps_emg = signal_processing.Rectify(ref biceps_emg);
            triceps_emg = signal_processing.Rectify(ref triceps_emg);
            biceps_emg = signal_processing.Normalize(ref biceps_emg, body.biceps.minimum_activation, body.biceps.maximum_activation);
            triceps_emg = signal_processing.Normalize(ref triceps_emg, body.triceps.minimum_activation, body.triceps.maximum_activation);

            //Neural Activation
            List<double> biceps_neural_activation = signal_processing.LinearEnvelope(ref biceps_emg, ref previous_biceps_emg);
            List<double> triceps_neural_activation = signal_processing.LinearEnvelope(ref triceps_emg, ref previous_triceps_emg);

            //Muscle Activation
            List<double> biceps_muscle_activation = MuscleActivationModel1(biceps_neural_activation, muscle_activation_coefficients[0]);
            List<double> triceps_muscle_activation = MuscleActivationModel1(triceps_neural_activation, muscle_activation_coefficients[1]);

            //Muscle Torque
            List<double> biceps_torque = new List<double>();
            List<double> triceps_torque = new List<double>();
            for (int i = 0; i < joint_angle.Count - 1; i++)
            {
                biceps_torque.Add(EstimateMuscleTorque(biceps_muscle_activation[i + 1], joint_angle[i], joint_angle[i + 1], body.biceps, timestep));
                triceps_torque.Add(EstimateMuscleTorque(triceps_muscle_activation[i + 1], joint_angle[i], joint_angle[i + 1], body.triceps, timestep));
            }

            //Skeletal Model
            List<double> passive_torque = GetPassiveTorque(joint_velocity);
            List<double> gravitational_torque = GetGravitationalTorque(ref joint_angle, body.mass_hand, body.mass_forearm, body.moment_arm_hand, body.moment_arm_forearm);
            double inertial_mass = GetInertialMass(body.length_hand, body.length_forearm, body.circumference_forearm, body.mass_hand, body.mass_forearm);
            List<double> acceleration_torque = GetAccelerationTorque(ref joint_acceleration, inertial_mass);

            double temp = 0;
            for (int i = 0; i < joint_angle.Count; i++)
            {
                temp = biceps_torque[i] - triceps_torque[i] - gravitational_torque[i] + passive_torque[i] + acceleration_torque[i];
                joint_torque.Add(temp);
            }

            return joint_torque;
        }

        List<double> EstimationController.EstimateForce(List<List<double>> parameters)
        {
            throw new NotImplementedException();
        }

        private List<double> GetJointAcceleration(ref List<double> joint_velocity, double timestep)
        {
            List<double> joint_acceleration = new List<double>();
            for (int i = 0; i < joint_velocity.Count - 1; i++)
                joint_acceleration.Add((joint_velocity[i + 1] - joint_velocity[i]) / timestep);
            return joint_acceleration;
        }

        private List<double> GetJointVelocity(ref List<double> joint_angle, double timestep)
        {
            List<double> joint_velocity = new List<double>();
            for (int i = 0; i < joint_angle.Count - 1; i++)
                joint_velocity.Add((joint_angle[i + 1] - joint_angle[i]) / timestep);
            return joint_velocity;
        }

        private List<double> PreviousEMG(List<double> emg, int order)
        {
            List<double> prev_emg = new List<double>();
            for (int i = 0; i < order; i++)
                prev_emg.Add(emg[i]);
            return prev_emg;
        }

        //Estimate muscle torque using a Hill-based muscle contraction model with a rigid tendon element
        private double EstimateMuscleTorque(double activation, double q1, double q2, Muscle muscle, double timestep)
        {
            double muscle_torque = 0;
            double Ffl = 0; //Force due to the muscle length-force relationship
            double Lm_dot = 0; //muscle contraction velocity;
            double Vmax = 0; //maximum muscle contraction velocity
            double C3 = 0; //ensure continuous first derivative at Lm_dot = 0
            double Ffv = 0; //force due to the velocity-force relationship
            double Factive = 0; //active contraction force of the musculotendon unit
            double k3, k4 = 0; //spring coefficients
            double Fpee = 0; //force of the parallel elastic element
            double phi = 0; //pennation angle of the muscle fiber
            double Fsee = 0; //Force of the series elastic element (the output force of this model)
            double Lm1 = muscle.GetMuscleFiberLength(q1);
            double Lm2 = muscle.GetMuscleFiberLength(q2);

            // Muscle force-length model
            Ffl = Math.Exp(-Math.Pow((Lm2 - muscle.Lmo) / (W * muscle.Lmo), 2));

            //Muscle contraction velocity
            Lm_dot = Lm2 - Lm1 / timestep;

            //Muscle force-velocity model
            Vmax = 10 * muscle.Lmo;
            C3 = Vmax * A * (gmax - 1) / (A + 1);
            if (Lm_dot <= 0)
                Ffv = (Vmax + Lm_dot) / (Vmax - (Lm_dot / A));
            else
                Ffv = (gmax * Lm_dot + C3) / (Lm_dot + C3);

            //Active force contribution of the muscle
            Factive = activation * muscle.Fmax * Ffv * Ffl;

            //Passive force contribution of the muscle, modelled as a non-linear spring
            k3 = 10;
            double Lmo_p = muscle.Lmo_adj * muscle.Lmo; //adjusted optimla muscle fiber length for cases where passive forces were exceptionally high
            k4 = (muscle.Fmax - k3 * (W * Lmo_p - Lmo_p)) / Math.Pow((W * Lmo_p - Lmo_p), 2);
            if (Lm2 <= Lmo_p)
                Fpee = k3 * (Lm2 - Lmo_p);
            else
                Fpee = k3 * (Lm2 - Lmo_p) + k4 * Math.Pow((Lm2 - Lmo_p), 2);

            //Pennation angle
            phi = muscle.Lmo * Math.Sin(muscle.phi_opt) / Lm2;

            //Series Elastic Element - simply a rigid tendon element in this model
            Fsee = (Factive + Fpee) * Math.Cos(phi);

            //Muscle torque
            muscle_torque = Fsee * Math.Abs(muscle.GetMuscleMomentArm(q2));

            return muscle_torque;
        }

        private double RadianToDegree(double radian)
        {
            return Math.PI * radian / 180;
        }

        private double DegreeToRadian(double degree)
        {
            return degree * 180 / Math.PI;
        }

        private List<double> MuscleActivationModel1(List<double> neural_activation, List<double> coefficients)
        {
            List<double> muscle_activation = new List<double>();
            double temp = 0;
            for (int i = 0; i < neural_activation.Count; i++)
            {
                temp = (Math.Exp(coefficients[0] * neural_activation[i]) - 1) / (Math.Exp(coefficients[0]) - 1);
                muscle_activation.Add(temp);
            }
            return muscle_activation;
        }

        private List<double> MuscleActivationModel2(List<double> neural_activation, List<double> coefficients)
        {
            List<double> muscle_activation = new List<double>();
            double temp = 0;
            for (int i = 0; i < neural_activation.Count; i++)
            {
                temp = (Math.Pow(coefficients[0], neural_activation[i]) - 1) / (coefficients[0] - 1);
                muscle_activation.Add(temp);
            }
            return muscle_activation;
        }

        private List<double> MuscleActivationModel3(List<double> neural_activation, List<double> coefficients)
        {
            List<double> muscle_activation = new List<double>();
            for (int i = 0; i < neural_activation.Count; i++)
            {
                double u01 = 0.3085 - coefficients[0] * Math.Cos(DegreeToRadian(45));
                double a01 = 0.3085 - coefficients[0] * Math.Sin(DegreeToRadian(45));

                double m1 = (a01 - 1) / (u01 - 1);
                double c1 = 1 - m1;

                double a1 = coefficients[1];
                double b1 = (Math.Exp(a01 / a1) - 1) / u01;

                double temp = 0;
                if (neural_activation[i] >= 0 && neural_activation[i] < u01)
                    temp = a1 * Math.Log(b1 * neural_activation[i] + 1);
                else if (neural_activation[i] >= u01)
                    temp = m1 * neural_activation[i] + c1;

                muscle_activation.Add(temp);
            }
            return muscle_activation;
        }

        private List<double> MuscleActivationModel4(List<double> neural_activation, List<double> coefficients, double timestep)
        {
            List<double> muscle_activation = new List<double>();
            muscle_activation.Add(neural_activation[0]);
            double derivative, temp = 0;
            for (int i = 0; i < neural_activation.Count - 1; i++)
            {
                derivative = (neural_activation[i] / coefficients[0] + (1 - neural_activation[i]) / coefficients[1]) * (neural_activation[i] - muscle_activation[i]);
                temp = muscle_activation[i] + derivative * timestep;
                muscle_activation.Add(temp);
            }
            return muscle_activation;
        }

        private List<double> MuscleActivationModel5(List<double> neural_activation, List<double> coefficients, double timestep)
        {
            List<double> muscle_activation = new List<double>();
            muscle_activation.Add(neural_activation[0]);
            double derivative, temp = 0;
            for (int i = 0; i < neural_activation.Count - 1; i++)
            {
                if (neural_activation[i] >= muscle_activation[i])
                {
                    derivative = (-muscle_activation[i] / coefficients[0]) + (neural_activation[i] / coefficients[0]);
                    temp = muscle_activation[i] + derivative * timestep;
                }
                else
                {
                    derivative = (-muscle_activation[i] / coefficients[1]) + (neural_activation[i] / coefficients[1]);
                    temp = muscle_activation[i] + derivative * timestep;
                }
                muscle_activation.Add(temp);
            }
            return muscle_activation;
        }

        private List<double> MuscleActivationModel6(List<double> neural_activation, double timestep)
        {
            List<double> muscle_activation = new List<double>();
            muscle_activation.Add(neural_activation[0]);
            double derivative, temp = 0;
            for (int i = 0; i < neural_activation.Count - 1; i++)
            {
                derivative = (optimization_parameters[0] * neural_activation[i] + optimization_parameters[1]) * (neural_activation[i] - muscle_activation[i]);
                temp = muscle_activation[i] + derivative * timestep;
                muscle_activation.Add(temp);
            }
            return muscle_activation;
        }

        private List<double> MuscleActivationModel7(List<double> neural_activation, List<double> coefficients, double timestep)
        {
            List<double> muscle_activation = new List<double>();
            muscle_activation.Add(neural_activation[0]);
            double derivative, temp = 0;
            for (int i = 0; i < neural_activation.Count - 1; i++)
            {
                if (neural_activation[i] > muscle_activation[i])
                {
                    derivative = (neural_activation[i] - muscle_activation[i]) / coefficients[0] * (0.5 + 1.5 * muscle_activation[i]);
                    temp = muscle_activation[i] + derivative * timestep;
                }
                else
                {
                    derivative = (neural_activation[i] - muscle_activation[i]) / coefficients[1] * (0.5 + 1.5 * muscle_activation[i]);
                    temp = muscle_activation[i] + derivative * timestep;
                }
                muscle_activation.Add(temp);
            }
            return muscle_activation;
        }

        private List<double> GetPassiveTorque(List<double> joint_velocity)
        {
            double b = 1; //damping coefficient
            List<double> passive_torque = new List<double>();
            for (int i = 0; i < joint_velocity.Count; i++)
                passive_torque.Add(-b * joint_velocity[i]);
            return passive_torque;
        }

        private List<double> GetGravitationalTorque(ref List<double> joint_angle, double mass_hand, double mass_forearm, double moment_arm_hand, double moment_arm_forearm)
        {
            double g = 9.81; //gravitational constant
            double Fg_hand = 0;
            double Fg_forearm = 0;
            double temp = 0;
            List<double> gravitational_torque = new List<double>();

            for (int i = 0; i < joint_angle.Count; i++)
            {
                Fg_hand = mass_hand * g * Math.Sin(joint_angle[i]);
                Fg_forearm = mass_forearm * g * Math.Sin(joint_angle[i]);
                temp = Fg_hand * moment_arm_hand + Fg_forearm * moment_arm_forearm;
                gravitational_torque.Add(temp);
            }

            return gravitational_torque;
        }

        private double GetInertialMass(double length_hand, double length_forearm, double forearm_circumference, double mass_hand, double mass_forearm)
        {
            double height_cylinder = length_forearm + length_hand;
            double radius_cylinder = forearm_circumference / 2 * Math.PI;
            double mass = mass_hand + mass_forearm;
            return (mass / 12) * (3 * Math.Pow(radius_cylinder, 2) + Math.Pow(height_cylinder, 2));
        }

        private List<double> GetAccelerationTorque(ref List<double> joint_acceleration, double inertial_mass)
        {
            List<double> acceleration_torque = new List<double>();
            for (int i = 0; i < joint_acceleration.Count; i++)
                acceleration_torque.Add(joint_acceleration[i] * inertial_mass);
            return acceleration_torque;
        }

        public void SetTimestep(double _timestep)
        {
            timestep = _timestep;
        }

        public double GetTimestep()
        {
            return timestep;
        }

        public void SetBody(Body _body)
        {
            body = _body;
        }

        public Body GetBody()
        {
            return body;
        }

        public void SetOptimizationParameters(List<double> optimized_parameters)
        {
            optimization_parameters = optimized_parameters;
            for (int i = 0; i < body.muscles.Count; i=i+2)
            {
                body.muscles[i].muscle_activation_model_coefficients.Add(optimized_parameters[i]);
                body.muscles[i].muscle_activation_model_coefficients.Add(optimized_parameters[i+1]);
            }
        }

        public List<double> GetOptimizationParameters()
        {
            return optimization_parameters;
        }

        public void SetBody(WearMECSF.Body body)
        {
            throw new NotImplementedException();
        }

        public void SetPreviousPosition(double previous_position)
        {
            throw new NotImplementedException();
        }
    }
}
