//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WearMECSF
{
    public interface TaskController
    {
        double MotionTask(List<List<double>> parameters);
    }
}
