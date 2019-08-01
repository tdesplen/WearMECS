//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using EposCmd.Net;

namespace WearMEBenchTopControlSystem
{
    class BracePositionThread
    {
        public Device epos;

        public double GetBracePosition()
        {
            return (double) epos.Operation.MotionInfo.GetPositionIs();
        }
    }
}
