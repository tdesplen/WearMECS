//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WearMECSF
{
    public class Body
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
    }
}
