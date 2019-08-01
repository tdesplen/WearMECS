//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using EposCmd.Net;

namespace WearMEBenchTopControlSystem
{
    class BraceCommandThread
    {
        public Device epos;
        public int command;
        public int position;
        public int velocity;


        public int SendCommand()
        {
            if (command == 1) //position control
            {
                //epos.Operation.PositionMode.ActivatePositionMode();
                try
                {
                    epos.Operation.ProfilePositionMode.MoveToPosition(position, true, true);
                }
                catch (DeviceException e)
                {
                    System.Console.WriteLine(e.ErrorMessage +", "+ e.ErrorCode);
                }

            }
            else if (command == 2) //velocity control
            {
               //epos.Operation.VelocityMode.ActivateVelocityMode();
                epos.Operation.VelocityMode.SetVelocityMust(velocity);

            }
            return command;
        }
        
    }
}
