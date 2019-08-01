//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WearMEBenchTopControlSystem
{
    abstract class Muscle : WearMECSF.Muscle
    {
        public double Lmo; //optimal muscle fiber length
        public double phi_opt; //pennation angle at the optimal muscle fiber length (radians)
        public double Fmax; //the maximum output force capacity of the muscle
        public double Lmo_adj; //adjustment factor for calculating passive muscle unit forces in cases where the passive forces were exceptionally higher
        public double maximum_activation; //maximum activation that produces the maximum amount of force
        public double minimum_activation; //baseline activation when muscle is at resting length/position

        public List<double> muscle_length_curve_coefficients;
        public List<double> tendon_length_curve_coefficients;
        public List<double> muscle_tendon_length_curve_coefficients;
        public List<double> moment_arm_curve_coefficients;

        public List<double> muscle_activation_model_coefficients;

        public Muscle()
        {
            Lmo = 0;
            phi_opt = 0;
            Fmax = 0;
            Lmo_adj = 0;
        }

        public Muscle(double _Lmo, double _phi_opt, double _Fmax, double _Lmo_adj)
        {
            Lmo = _Lmo;
            phi_opt = _phi_opt;
            Fmax = _Fmax;
            Lmo_adj = _Lmo_adj;
            muscle_length_curve_coefficients = new List<double>();
            tendon_length_curve_coefficients = new List<double>();
            muscle_tendon_length_curve_coefficients = new List<double>();
            moment_arm_curve_coefficients = new List<double>();
            muscle_activation_model_coefficients = new List<double>();
        }
    }
}
