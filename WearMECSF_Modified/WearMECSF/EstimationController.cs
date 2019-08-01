//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WearMECSF
{
    public interface EstimationController
    {
        List<double> EstimatePosition(List<List<double>> parameters);
        List<double> EstimateVelocity(List<List<double>> parameters);
        List<double> EstimateAcceleration(List<List<double>> parameters);
        List<double> EstimateTorque(List<List<double>> parameters);
        List<double> EstimateForce(List<List<double>> parameters);
        void SetOptimizationParameters(List<double> optimized_parameters);
        List<double> GetOptimizationParameters();
        void SetBody(Body body);
        void SetPreviousPosition(double previous_position);
    }
}
