//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using WearMECSF;
using System.Threading;
using EposCmd.Net;
using System.ComponentModel;
using System.Diagnostics;

namespace WearMEBenchTopControlSystem
{
    class WearMEBraceController : ActuationController
    {

        //Function to implement:
        // -read encoder data
        // -command motor
        // -tranmission transformations

        //Variable Declarations

        BracePositionThread brace_position_thread;
        BraceCommandThread brace_command_thread;
        private BackgroundWorker brace_position_worker;
        private BackgroundWorker brace_command_worker;
        private Device epos;
        private ElbowMotionTrackingController motion_tracking_controller;
        private List<double> brace_positions;
        private Stopwatch stop_watch;
        private Stopwatch stop_watch2;
        private int gear_ratio;

        public WearMEBraceController(Device _epos, ElbowMotionTrackingController _motion_tracking_controller)
        {
            epos = _epos;
            motion_tracking_controller = _motion_tracking_controller;
            brace_positions = new List<double>();
            brace_position_thread = new BracePositionThread();
            brace_position_worker = new BackgroundWorker();
            brace_position_worker.DoWork += new DoWorkEventHandler(BracePositionWorker_DoWork);
            brace_position_worker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(BracePositionWorker_RunWorkerCompleted);
            brace_position_thread.epos = _epos;
            brace_command_thread = new BraceCommandThread();
            brace_command_worker = new BackgroundWorker();
            brace_command_worker.DoWork += new DoWorkEventHandler(BraceCommandWorker_DoWork);
            brace_command_worker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(BraceCommandWorker_RunWorkerCompleted);
            brace_command_thread.epos = _epos;
            brace_command_thread.command = 1; //position
            stop_watch = new Stopwatch();
            stop_watch2 = new Stopwatch();
            gear_ratio = 713;

            //initialize background workers/threads
        }

        public void MoveToPosition(double position)
        {
            brace_command_thread.command = 1;
            brace_command_thread.position = (int) DegreeToPulse(position);
            if (!brace_command_worker.IsBusy)
                stop_watch2.Start();
                brace_command_worker.RunWorkerAsync(brace_command_thread);
        }

        private int DegreeToQuadCounts(double position)
        {
            //1 quadcount = 4 * encoder count / revolution
            throw new NotImplementedException(); //TODO
        }

        void ActuationController.MoveWithAcceleration(double acceleration)
        {
            throw new NotImplementedException();
        }

        void ActuationController.MoveWithForce(double force)
        {
            throw new NotImplementedException();
        }

        void ActuationController.MoveWithTorque(double torque)
        {
            throw new NotImplementedException();
        }

        void ActuationController.MoveWithVelocity(double velocity)
        {
            brace_command_thread.command = 2; //velocity
            brace_command_thread.velocity = (int) DegreePerSecondToRPM(velocity);
            brace_command_worker.RunWorkerAsync(brace_command_thread);
        }

        ///<summary> A method for converting velocity of the motor to velocity of the elbow. </summary>
        ///<param name="vel"> Velocity of the motor with units: revolutions per minute. </param>
        ///<returns> Velocity of the elbow with units: degrees/second.  </returns>
        double RPMToDegreePerSecond(double vel)
        {
            double velocity = 0;
            velocity = vel * 6; //convert velocity from RPM to degree/second
            velocity = velocity / gear_ratio; //divide motor velocity by total gear ratio to get output velocity in degree/second
            return velocity;
        }

        ///<summary> A method for converting velocity of the elbow to velocity of the motor. </summary>
        ///<param name="vel"> Velocity of the elbow with units: degrees/second. </param>
        ///<returns> Velocity of the motor with units: revolutions per minute.</returns>
        double DegreePerSecondToRPM(double vel)
        {
            double velocity = 0;
            velocity = vel * 0.1667; // degree/second to RPM
            velocity = velocity * gear_ratio; //multiply to attain the RPM required at the motor
            return velocity;
        }

        ///<summary> A method for converting encoder pulses to degrees of elbow motion. </summary>
        ///<param name="pos"> The position of the motor with units: encoder pulses. </param>
        ///<returns> The position of the elbow as seen by the encoder with units: degrees. </returns>
        double PulseToDegree(double pos)
        {
            double position = 0;
            double encoder_counts = 512;
            double encoder_ratio = 4*encoder_counts / 360; // 4 * # of encoder counts / revolution
            double elbow_angle_degrees = ((pos / encoder_ratio) / gear_ratio);
            return elbow_angle_degrees;
        }

        ///<summary> A method for converting degrees of elbow motion to encoder pulses. </summary>
        ///<param name="pos"> The position of the motor with units: encoder pulses. </param>
        ///<returns> The position of the elbow as seen by the encoder with units: degrees. </returns>
        double DegreeToPulse(double pos)
        {
            double position = 0;
            double encoder_counts = 512;
            double encoder_ratio = 4* encoder_counts / 360; // 4 * # of encoder counts / revolution
            double motor_rotation = ((pos * encoder_ratio) * gear_ratio);
            return motor_rotation;
        }

        public void SetEPOS(Device _epos)
        {
            epos = _epos;
            brace_command_thread.epos = _epos;
            brace_position_thread.epos = _epos;
        }

        //<---------------------Thread Functions--------------------------------->
        public void GetBracePositionThreadCall()
        {
            if (!epos.Equals(null))
            {
                if (!brace_position_worker.IsBusy)
                {
                    stop_watch.Start();
                    brace_position_worker.RunWorkerAsync(brace_position_thread);
                }
            }
        }

        private void BracePositionWorker_DoWork(object sender, System.ComponentModel.DoWorkEventArgs e)
        {
            BracePositionThread thread_call = (BracePositionThread)e.Argument;
            e.Result = thread_call.GetBracePosition();
        }

        private void BracePositionWorker_RunWorkerCompleted(object sender, System.ComponentModel.RunWorkerCompletedEventArgs e)
        {
            double position = (double)e.Result;
            position = PulseToDegree(position);
            brace_positions.Add(position);
            motion_tracking_controller.SetBracePositions(brace_positions);
            stop_watch.Stop();
            motion_tracking_controller.SetBracePositionTimer(stop_watch.Elapsed.TotalMilliseconds);
            //System.Console.WriteLine("Brace Position: " + brace_positions[brace_positions.Count-1].ToString());
            //System.Console.WriteLine("Brace Position Thread Duration (ms): " + stop_watch.ElapsedMilliseconds);
            stop_watch.Reset();
        }

        //public void CommandBraceThreadCall(int _position, int _velocity, int _command)
        //{
        //    brace_command_thread.position = _position;
        //    brace_command_thread.velocity = _velocity;
        //    brace_command_thread.command = _command;
        //    if (!epos.Equals(null))
        //    {
        //        if (!brace_command_worker.IsBusy)
        //        {
        //            stop_watch2.Start();
        //            brace_command_worker.RunWorkerAsync(brace_command_thread);
        //        }
        //    }
        //}

        private void BraceCommandWorker_DoWork(object sender, System.ComponentModel.DoWorkEventArgs e)
        {
            BraceCommandThread thread_call = (BraceCommandThread)e.Argument;
            e.Result = thread_call.SendCommand();
        }

        private void BraceCommandWorker_RunWorkerCompleted(object sender, System.ComponentModel.RunWorkerCompletedEventArgs e)
        {
            stop_watch2.Stop();
            motion_tracking_controller.SetBraceCommandTimer(stop_watch2.Elapsed.TotalMilliseconds);
            stop_watch2.Reset();
        }

        public void ActivatePositionMode()
        {
            brace_command_thread.epos.Operation.PositionMode.ActivatePositionMode();
        }

        public void ActivateProfilePositionMode()
        {
            brace_command_thread.epos.Operation.ProfilePositionMode.ActivateProfilePositionMode();
        }

        public void ActivateVelocityMode()
        {
            brace_command_thread.epos.Operation.VelocityMode.ActivateVelocityMode();
        }

    }
}
