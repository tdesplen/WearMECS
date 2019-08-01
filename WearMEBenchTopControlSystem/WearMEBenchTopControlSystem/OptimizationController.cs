//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WearMEBenchTopControlSystem
{
    class OptimizationController
    {
        private List<double> biceps_emg;
        private List<double> triceps_emg;
        private List<double> subject_angle;

        String OptimizeEstimationModel(int model_number)
        {
            String optimziation_status = "Optimization failed. \r\n";

            //Check to make sure data is available for the optimization to be performed on
            if (biceps_emg.Count < 1 && triceps_emg.Count < 1 && subject_angle.Count < 1)
            {
                optimziation_status = "No data has been made available for the optimization.";
                return optimziation_status;
            }

            //We have data, now do the appropriate optimization
            if (model_number == 1) //proprtional model
            {

            }
            else if (model_number == 2) //Kalman Filter model
            {

            }
            else if (model_number == 3) //Bioemchanical model
            {

            }
            else //error with the model number
            {
                optimziation_status = "Invalid model number passed as a parameter.\r\n";
            }

            return optimziation_status;
        }
    }
}
