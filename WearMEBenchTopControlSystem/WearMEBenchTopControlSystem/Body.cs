//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WearMEBenchTopControlSystem
{
    class Body : WearMECSF.Body
    {
        public double mass_hand;
        public double mass_forearm;
        public double length_hand;
        public double length_forearm;
        public double circumference_forearm;
        public double moment_arm_hand;
        public double moment_arm_forearm;
        public double mass;
        public double height;
        public double lower_arm_inertia;

        public List<Muscle> muscles;
        public Muscle biceps;
        public Muscle triceps;

        public Body()
        {
            mass_hand = 0;
            mass_forearm = 0;
            length_hand = 0;
            length_forearm = 0;
            circumference_forearm = 0;
            moment_arm_hand = 0;
            moment_arm_forearm = 0;
            muscles = new List<Muscle>();
            biceps = new BicepsLH();
            triceps = new TricepsLH();
            muscles.Add(biceps);
            muscles.Add(triceps);
        }

        public Body(double _mass, double _height, double _forearm_circumference, double _forearm_length)
        {
            mass = _mass;
            mass_hand = 0.006 * mass;
            mass_forearm = 0.016 * mass;
            height = _height;
            length_hand = 0.108 * height;
            length_forearm = _forearm_length;
            circumference_forearm = _forearm_circumference;
            moment_arm_hand = 0.506 * length_hand + length_forearm;
            moment_arm_forearm = 0.430 * length_forearm;
            muscles = new List<Muscle>();
            biceps = new BicepsLH();
            triceps = new TricepsLH();
            muscles.Add(biceps);
            muscles.Add(triceps);
        }

        public void SetSkeletalParameters(double _mass, double _height, double _forearm_circumference)
        {
            mass = _mass;
            mass_hand = 0.006 * mass;
            mass_forearm = 0.016 * mass;
            height = _height;
            length_hand = 0.108 * height;
            length_forearm = 0.146 * height;
            circumference_forearm = _forearm_circumference;
            moment_arm_hand = 0.506 * length_hand + length_forearm;
            moment_arm_forearm = 0.436 * length_forearm;
        }

        internal void CalculateLowerArmInertia()
        {
            double hcylinder = length_hand + length_forearm;
            double rcylinder = circumference_forearm / 2 * Math.PI;
            double M = mass_hand + mass_forearm;
            lower_arm_inertia = (M / 12) * (3 * Math.Pow(rcylinder,2) + Math.Pow(hcylinder, 2));
        }
    }
}
