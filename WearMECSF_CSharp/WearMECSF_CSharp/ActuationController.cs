//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WearMECSF_CSharp
{
    public interface ActuationController
    {
        /// <summary>
        /// A function template to command the position of the actuation system.
        /// </summary>
        /// <param name="position">The desired position</param>
        /// <returns>void</returns>
        void MoveToPosition(double position);

        /// <summary>
        /// A function template to command the velocity of the actuation system.
        /// </summary>
        /// <param name="velocity">The desired velocity</param>
        /// <returns>void</returns>
        void MoveWithVelocity(double velocity);

        /// <summary>
        /// A function template to command the acceleration of the actuation system.
        /// </summary>
        /// <param name="acceleration">The desired acceleration</param>
        /// <returns>void</returns>
        void MoveWithAcceleration(double acceleration);

        /// <summary>
        /// A function template to command the torque of the actuation system.
        /// </summary>
        /// <param name="torque">The desired torque</param>
        /// <returns>void</returns>
        void MoveWithTorque(double torque);

        /// <summary>
        /// A function template to command the force of the actuation system.
        /// </summary>
        /// <param name="force">The desired force</param>
        /// <returns>void</returns>
        void MoveWithForce(double force);
    }
}
