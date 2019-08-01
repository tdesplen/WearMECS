//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WearMEBenchTopControlSystem
{
    class BicepsLH : Muscle
    {
        public override double GetMuscleFiberLength(double angle)
        {
            double muscle_length = 0;
            int curve_order = muscle_length_curve_coefficients.Count;
            for (int i = curve_order; i > 0; i--)
            {
                muscle_length += (Math.Pow(angle, i-1) * muscle_length_curve_coefficients[curve_order - i]); 
            }
            return muscle_length;
        }

        public override double GetMuscleMomentArm(double angle)
        {
            double moment_arm = 0;
            int curve_order = moment_arm_curve_coefficients.Count;
            for (int i = curve_order; i > 0; i--)
            {
                moment_arm += (Math.Pow(angle, i - 1) * moment_arm_curve_coefficients[curve_order - i]);
            }
            return Math.Abs(moment_arm);
        }

        public override double GetMusculoTendonLength(double angle)
        {
            double muscle_tendon_length = 0;
            int curve_order = muscle_tendon_length_curve_coefficients.Count;
            for (int i = curve_order; i > 0; i--)
            {
                muscle_tendon_length += (Math.Pow(angle, i - 1) * muscle_tendon_length_curve_coefficients[curve_order - i]);
            }
            return muscle_tendon_length;
        }

        public override double GetTendonLength(double angle)
        {
            double tendon_length = 0;
            int curve_order = tendon_length_curve_coefficients.Count;
            for (int i = curve_order; i > 0; i--)
            {
                tendon_length += (Math.Pow(angle, i - 1) * tendon_length_curve_coefficients[curve_order - i]);
            }
            return tendon_length;
        }
    }
}
