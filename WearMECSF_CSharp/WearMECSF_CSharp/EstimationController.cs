//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WearMECSF_CSharp
{
    public interface EstimationController
    {
        /// <summary>
        /// A function template for estimating the position(s) of a joint, componenet, or system.
        /// </summary>
        /// <param name="parameters">A list of parameters, each of which could itself be a list of data points</param>
        /// <returns>A list of doubles</returns>
        List<double> EstimatePosition(List<List<double>> parameters);

        /// <summary>
        /// A function template for estimating the velocity of a joint, component, or system.
        /// </summary>
        /// <param name="parameters">A list of parameters, each of which could itself be a list of data points</param>
        /// <returns>A list of doubles</returns>
        List<double> EstimateVelocity(List<List<double>> parameters);

        /// <summary>
        /// A function template for estimating the acceleration(s) of a joint, component, or system.
        /// </summary>
        /// <param name="parameters">A list of parameters, each of which could itself be a list of data points</param>
        /// <returns>A list of doubles</returns>
        List<double> EstimateAcceleration(List<List<double>> parameters);

        /// <summary>
        /// A function template for estimating a torque(s) of a joint, component, or system.
        /// </summary>
        /// <param name="parameters">A list of parameters, each of which could itself be a list of data points</param>
        /// <returns>A list of doubles</returns>
        List<double> EstimateTorque(List<List<double>> parameters);

        /// <summary>
        /// A function template for estimating a force(s) of a joint, component, or system.
        /// </summary>
        /// <param name="parameters">A list of parameters, each of which could itself be a list of data points</param>
        /// <returns>A list of doubles</returns>
        List<double> EstimateForce(List<List<double>> parameters);
    }
}
