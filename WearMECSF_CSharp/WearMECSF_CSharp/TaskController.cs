//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WearMECSF_CSharp
{
    public interface TaskController
    {
        /// <summary>
        /// A function template for any motion task that needs to be orchestrated by the control system.
        /// </summary>
        /// <param name="parameters">A list of parameters, each of which could itself be a list of data points</param>
        /// <returns>void</returns>
        void MotionTask(List<List<double>> parameters);
    }
}
