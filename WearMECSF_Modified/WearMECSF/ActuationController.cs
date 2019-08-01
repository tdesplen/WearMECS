//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WearMECSF
{
    public interface ActuationController
    {
        void MoveToPosition(double position);
        void MoveWithVelocity(double velocity);
        void MoveWithAcceleration(double acceleration);
        void MoveWithTorque(double torque);
        void MoveWithForce(double force);
    }
}
