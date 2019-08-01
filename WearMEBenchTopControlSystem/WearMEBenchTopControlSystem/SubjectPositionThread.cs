//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WearMEBenchTopControlSystem
{
    class SubjectPositionThread
    {
        public CollectionArmLibrary.CollectionArm collection_arm;

        public double GetArmPosition()
        {
            return collection_arm.ReadSerial();
        }
    }
}
