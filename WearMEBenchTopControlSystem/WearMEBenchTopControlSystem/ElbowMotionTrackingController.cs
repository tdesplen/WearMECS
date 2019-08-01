//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using WearMECSF;
using System.IO;
using System.Threading;
using EposCmd.Net;
using EposCmd.Net.DeviceCmdSet.Operation;
using System.ComponentModel;
using System.Diagnostics;

namespace WearMEBenchTopControlSystem
{
    class ElbowMotionTrackingController : TaskController
    {
        //Tasks to implement:
        // -previous samples
        // -control loops
        // -gather sensor data
        // -store sensor data
        // -optimization of coefficients
        // -read data from file


        //Complex Datatype Defintions
        EMGDataThread emg_data_thread;
        SubjectPositionThread subject_position_thread;
        LoggerThread logger_thread;
        private System.ComponentModel.BackgroundWorker emg_data_worker;
        private System.ComponentModel.BackgroundWorker subject_position_worker;
        private System.ComponentModel.BackgroundWorker logger_worker;
        Thread subject_position_thread2;

        Stopwatch stop_watch, stop_watch2, stop_watch3;
        List<EstimationController> estimation_controllers;
        KalmanFilterEstimationController kfmm;
        ProportionalEstimationController pmm;
        PolynomialEstimationController npmm;
        WearMEBraceController actuation_controller;
        DelsysTrignoCSharpLibrary.Driver trigno;
        CollectionArmLibrary.CollectionArm collection_arm;
        DeviceManager epos_manager;
        Device epos;
        private ExperimentInterface experiment_interface;
        private SignalProcessing signal_processing;

        private StreamWriter output_writer;
        private FileStream output_file;
        private StreamWriter timing_writer;
        private FileStream timing_file;
        private Body subject;
        private List<double> biceps_emg;
        private List<double> triceps_emg;
        private List<double> subject_positions;
        private List<double> brace_positions;
        private List<double> position_estimates;
        private List<double> previous_biceps_emg;
        private List<double> previous_triceps_emg;
        private List<double> previous_subject_positions;
        private List<double> previous_brace_positions;
        private List<List<double>> muscle_activation_parameters;
        private List<double> kalman_filter_parameters;
        private List<double> proportional_parameters;
        private List<List<double>> subject_motion_data;
        private List<double> command_positions;
        private List<double> previous_biceps_emg_filter1;
        private List<double> previous_triceps_emg_filter1;
        States trigno_state = States.Idle;

        //Simple Datatype Definitions
        public double brace_position;
        public double subject_position; // take joint_angle.Last(joint_angle[joint_angle.Count]) as most recent position
        public double command_position;
        private double time_step;
        private int window_size;
        private int biceps_emg_channel;
        private int triceps_emg_channel;
        private double emg_sampling_frequency;
        public double brace_encoder_offset;
        private int gear_ratio;
        private bool save_motion_data;
        private bool save_system_data;
        private bool track_motion_data;
        public int current_estimation_controller;
        public int upper_position_limit;
        public int lower_position_limit;
        public double command_frequency;
        private bool finished_logging;
        private int counter;
        private double tracking_loop_timer;
        private double data_log_timer;
        private double subject_position_timer;
        private double brace_position_timer;
        private double brace_command_timer;

        enum States
        {
            Idle = 0,
            Connected = 1,
            Streaming = 2,
            Stopped = 3
        };

        public ElbowMotionTrackingController(int _window_size, int _biceps_emg_channel, int _triceps_emg_channel, ExperimentInterface _experiment_interface) //add other arguments from startup
        {
            experiment_interface = _experiment_interface;

            subject = new Body();
            emg_sampling_frequency = 2000;
            time_step = 1 / emg_sampling_frequency;
            window_size = _window_size;
            brace_position = 0;
            subject_position = 0;
            biceps_emg_channel = _biceps_emg_channel;
            triceps_emg_channel = _triceps_emg_channel;
            brace_encoder_offset = 0;
            gear_ratio = 713; //Taylor's brace gear ratio
            save_motion_data = false;
            save_system_data = false;
            track_motion_data = false;
            current_estimation_controller = 0;
            upper_position_limit = 130;
            lower_position_limit = -20;
            command_position = 0;
            command_frequency = Convert.ToDouble(experiment_interface.textBox8.Text);
            counter = 0;

            //initialize complex data types
            estimation_controllers = new List<EstimationController>();
            kfmm = new KalmanFilterEstimationController();
            pmm = new ProportionalEstimationController(time_step);
            npmm = new PolynomialEstimationController(time_step, 3);
            estimation_controllers.Add(kfmm);
            estimation_controllers.Add(pmm);
            estimation_controllers.Add(npmm);
            actuation_controller = new WearMEBraceController(epos, this);
            epos_manager = new DeviceManager("EPOS", "MAXON_RS232", "RS232", experiment_interface.textBox9.Text);
            epos_manager.Baudrate = 115200;
            epos_manager.Timeout = 500;
            trigno = new DelsysTrignoCSharpLibrary.Driver();
            collection_arm = new CollectionArmLibrary.CollectionArm(experiment_interface.textBox13.Text, 38400);
            subject_positions = new List<double>(window_size);
            brace_positions = new List<double>(window_size);
            triceps_emg = new List<double>(window_size);
            biceps_emg = new List<double>(window_size);
            position_estimates = new List<double>(window_size);
            previous_biceps_emg = new List<double>(window_size);
            previous_triceps_emg = new List<double>(window_size);
            previous_subject_positions = new List<double>(window_size);
            previous_brace_positions = new List<double>(window_size);
            command_positions = new List<double>(window_size);
            previous_biceps_emg_filter1 = new List<double>(window_size);
            previous_triceps_emg_filter1 = new List<double>(window_size);
            for (int i =0; i < window_size; i++)
            {
                previous_biceps_emg.Add(0);
                previous_triceps_emg.Add(0);
                previous_subject_positions.Add(0);
                previous_brace_positions.Add(0);
                previous_biceps_emg_filter1.Add(0);
                previous_triceps_emg_filter1.Add(0);
                position_estimates.Add(0);
            }
            muscle_activation_parameters = new List<List<double>>();
            kalman_filter_parameters = new List<double>();
            proportional_parameters = new List<double>();
            subject_motion_data = new List<List<double>>();
            stop_watch = new Stopwatch();
            stop_watch2 = new Stopwatch();
            stop_watch3 = new Stopwatch();
            finished_logging = true;
            signal_processing = new SignalProcessing();

            //Thread initialization
            emg_data_thread = new EMGDataThread();
            emg_data_thread.window_size = window_size;
            emg_data_thread.trigno = trigno;
            emg_data_thread.biceps_emg_channel = biceps_emg_channel;
            emg_data_thread.triceps_emg_channel = triceps_emg_channel;
            emg_data_worker = new BackgroundWorker();
            emg_data_worker.DoWork += new DoWorkEventHandler(EMGDataWorker_DoWork);
            emg_data_worker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(EMGDataWorker_RunWorkerCompleted);
            subject_position_thread = new SubjectPositionThread();
            subject_position_thread.collection_arm = collection_arm;
            subject_position_worker = new BackgroundWorker();
            subject_position_worker.DoWork += new DoWorkEventHandler(SubjectPositionWorker_DoWork);
            subject_position_worker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(SubjectPositionWorker_RunWorkerCompleted);
            logger_thread = new LoggerThread();
            logger_worker = new BackgroundWorker();
            logger_worker.DoWork += new DoWorkEventHandler(LoggerWorker_DoWork);
            logger_worker.RunWorkerCompleted += new RunWorkerCompletedEventHandler(LoggerWorker_RunWorkerCompleted);

            tracking_loop_timer = 0;
            data_log_timer = 0;
            subject_position_timer = 0;
            brace_position_timer = 0;
            brace_command_timer = 0;
        }

        public void SetLoggerFilename(string motion_file_name, string timing_file_name)
        {
            output_file = new FileStream(motion_file_name, FileMode.Create, FileAccess.Write);
            output_writer = new StreamWriter(output_file);
            output_writer.WriteLine("biceps_emg, triceps_emg, subject_position, position_estimate, brace_position");

            timing_file = new FileStream(timing_file_name, FileMode.Create, FileAccess.Write);
            timing_writer = new StreamWriter(timing_file);
            timing_writer.WriteLine("tracking_loop_timer, data_log_timer, subject_position_timer, brace_position_timer, brace_command_timer");
        }

        public void WriteTimingData()
        {
            string line = tracking_loop_timer.ToString() + "," + data_log_timer.ToString() + "," + subject_position_timer.ToString() + "," + brace_position_timer.ToString() + "," + brace_command_timer.ToString();
            timing_writer.WriteLine(line);
        }

        public void SavingMotionDataEnable(bool v)
        {
            save_motion_data = v;
            if (v)
                experiment_interface.pictureBox6.BackColor = System.Drawing.Color.Green;
            else
                experiment_interface.pictureBox6.BackColor = System.Drawing.Color.Red;
        }

        public bool GetSavingMotionDataEnable()
        {
            return save_motion_data;
        }

        internal void SetBraceCommandTimer(double _brace_command_timer)
        {
            brace_command_timer = _brace_command_timer;
        }

        internal void SetBracePositionTimer(double _brace_position_timer)
        {
            brace_position_timer = _brace_position_timer;
        }

        internal void SetEMGChannels(int trigno_sensor_biceps, int trigno_sensor_triceps)
        {
            biceps_emg_channel = trigno_sensor_biceps - 1;
            triceps_emg_channel = trigno_sensor_triceps - 1;
            emg_data_thread.biceps_emg_channel = biceps_emg_channel;
            emg_data_thread.triceps_emg_channel = triceps_emg_channel;
        }

        internal void CloseLoggingFile()
        {
            while (!finished_logging) { }
            output_file.Close();
            timing_file.Close();
        }

        double TaskController.MotionTask(List<List<double>> parameters)
        {
            throw new NotImplementedException();
        }

        public void SetBracePositions(List<double> _brace_positions)
        {
            brace_positions = _brace_positions;
            brace_position = brace_positions[brace_positions.Count-1];
        }

        internal void SetBodyForEstimation()
        {
            for (int i = 0; i < estimation_controllers.Count; i++)
            {
                estimation_controllers[i].SetBody(subject);
            }
        }

        public double ConvertEncoderCountToDegrees(double encoder_value)
        {
            double encoder_counts = 512;
            double encoder_ratio = 4 * encoder_counts / 360; // 4 * # of encoder counts / revolution
            return ((encoder_value / encoder_ratio) / gear_ratio);
        }


        public void RotateMotorForward()
        {
            try
            {
                VelocityMode vm = epos.Operation.VelocityMode;

                //vm.ActivateVelocityMode();

                experiment_interface.textBox10.Text = epos.Operation.OperationMode.GetOperationModeAsString();

                experiment_interface.textBox2.Text = String.Format("{0}", epos.Operation.MotionInfo.GetPositionIs());

                vm.SetVelocityMust(1000);
            }
            catch (DeviceException e)
            {
                experiment_interface.ShowMessageBox(e.ErrorMessage, e.ErrorCode);
            }
            catch (Exception e)
            {
                System.Windows.Forms.MessageBox.Show(e.Message);
            }
        }

        internal bool EPOSIsEnabled()
        {
            return epos.Operation.StateMachine.GetEnableState();
        }

        internal void SetTrackingEnable(bool v)
        {
            track_motion_data = v;
            if (v)
                experiment_interface.pictureBox8.BackColor = System.Drawing.Color.Green;
            else
                experiment_interface.pictureBox8.BackColor = System.Drawing.Color.Red;
        }

        public bool GetTrackingEnable()
        {
            return track_motion_data;
        }

        public void RotateMotorBackwards()
        {
            try
            {
                VelocityMode vm = epos.Operation.VelocityMode;

                //vm.ActivateVelocityMode();

                experiment_interface.textBox10.Text = epos.Operation.OperationMode.GetOperationModeAsString();

                experiment_interface.textBox2.Text = String.Format("{0}", epos.Operation.MotionInfo.GetPositionIs());

                vm.SetVelocityMust(-1000);
            }
            catch (DeviceException e)
            {
                experiment_interface.ShowMessageBox(e.ErrorMessage, e.ErrorCode);
            }
            catch (Exception e)
            {
                System.Windows.Forms.MessageBox.Show(e.Message);
            }
        }

        public void StopMotor()
        {
            try
            {
                VelocityMode vm = epos.Operation.VelocityMode;

                //vm.ActivateVelocityMode();

                experiment_interface.textBox10.Text = epos.Operation.OperationMode.GetOperationModeAsString();

                experiment_interface.textBox2.Text = String.Format("{0}", epos.Operation.MotionInfo.GetPositionIs());
            
                vm.SetVelocityMust(0);
            }
            catch (DeviceException e)
            {
                experiment_interface.ShowMessageBox(e.ErrorMessage, e.ErrorCode);
            }
            catch (Exception e)
            {
                System.Windows.Forms.MessageBox.Show(e.Message);
            }
        }

        public void DisableEPOS()
        {
            try
            {
                StateMachine sm = epos.Operation.StateMachine;

                if (sm.GetFaultState())
                    sm.ClearFault();

                if (!sm.GetDisableState())
                    sm.SetDisableState();

                experiment_interface.textBox10.Text = epos.Operation.OperationMode.GetOperationModeAsString();

                experiment_interface.pictureBox1.BackColor = System.Drawing.Color.Red;
            }
            catch (DeviceException e)
            {
                experiment_interface.ShowMessageBox(e.ErrorMessage, e.ErrorCode);
            }
            catch (Exception e)
            {
                System.Windows.Forms.MessageBox.Show(e.Message);
            }
        }

        public void SetWindowSize(int _window_size)
        {
            window_size = _window_size;
        }

        public void DisconnectCollectionArm()
        {
            collection_arm.Disconnect();
            experiment_interface.pictureBox2.BackColor = System.Drawing.Color.Red;
        }

        public void StopTrignoSampling()
        {
            if (trigno.IsStreaming())
            {
                trigno.StopSampling();
                if (!trigno.IsStreaming())
                {
                    trigno_state = States.Stopped;
                    experiment_interface.UpdateTrignoStatus(trigno_state.ToString());
                    experiment_interface.EnableEMGTimer(false);
                    experiment_interface.pictureBox7.BackColor = System.Drawing.Color.Red;
                }
            }
        }

        public void StartTrignoSampling()
        {
            if (trigno.IsConnected())
                trigno.BeginSampling();
            if (trigno.IsStreaming())
            {
                trigno_state = States.Streaming;
                experiment_interface.UpdateTrignoStatus(trigno_state.ToString());
                experiment_interface.EnableEMGTimer(true);
                experiment_interface.pictureBox7.BackColor = System.Drawing.Color.Green;
            }
        }

        internal void SetMVCMinimums(string[] minimums)
        {
            int length = minimums.GetLength(1);
            for (int i = 0; i < length; i++)
            {
                subject.muscles[i].minimum_activation = Convert.ToDouble(minimums[i]);
            }
        }

        internal void SetMVCValues(string[] mvc_values)
        {
            subject.muscles[0].maximum_activation = Convert.ToDouble(mvc_values[0]);
            subject.muscles[0].minimum_activation = Convert.ToDouble(mvc_values[1]);
            subject.muscles[1].maximum_activation = Convert.ToDouble(mvc_values[2]);
            subject.muscles[1].minimum_activation = Convert.ToDouble(mvc_values[3]);
        }

        internal void SetMVCMaximums(string[] maximums)
        {
            int length = maximums.GetLength(1);
            for (int i = 0; i < length; i++)
            {
                subject.muscles[i].maximum_activation = Convert.ToDouble(maximums[i]);
            }
        }

        internal void SetSkeletalParameters(String[] values)
        {
            subject.mass = Convert.ToDouble(values[0]);
            subject.height = Convert.ToDouble(values[1]);
            subject.circumference_forearm = Convert.ToDouble(values[2]);
            subject.SetSkeletalParameters(subject.mass, subject.height, subject.circumference_forearm);
            subject.CalculateLowerArmInertia();
        }

        internal void SetMuscleParameters(List<string> data)
        {
            Char delimiter = ';';
            //Line Format: Lmo, Lmo_adj, phi_opt, Fmax
            for (int i = 0; i < data.Count; i++)
            {
                String[] values = data[i].Split(delimiter);
                subject.muscles[i].Lmo = Convert.ToDouble(values[0]);
                subject.muscles[i].Lmo_adj = Convert.ToDouble(values[1]);
                subject.muscles[i].phi_opt = Convert.ToDouble(values[2]);
                subject.muscles[i].Fmax = Convert.ToDouble(values[3]);
            }
        }

        internal void SetOptimizationParameters(List<string> data)
        {
            Char delimiter = ',';
            //Line Format: c1, .., cn (each line are the optimized parameters for each model) Note: each line length may vary
            for (int i = 0; i < estimation_controllers.Count; i++)
            {
                String[] values = data[i+1].Split(delimiter);
                int number_of_coefficients = values.Length;
                List<double> optimized_parameters = new List<double>(number_of_coefficients);
                for (int j = 0; j < number_of_coefficients; j++)
                    optimized_parameters.Add(Convert.ToDouble(values[j]));

                estimation_controllers[i].SetOptimizationParameters(optimized_parameters);
            }

        }

        internal void SetCurveParameters(List<string> data)
        {
            //Line Format: Lmt_p1, r_p1, Lm_p1, Lt_p1 (next lines are the p2 and so on)
            Char delimiter = ';';
            for (int i = 0; i < data.Count; i++)
            {
                String[] values = data[i].Split(delimiter);
                subject.muscles[i].muscle_tendon_length_curve_coefficients.Add(Convert.ToDouble(values[0]));
                subject.muscles[i].moment_arm_curve_coefficients.Add(Convert.ToDouble(values[1]));
                subject.muscles[i].muscle_length_curve_coefficients.Add(Convert.ToDouble(values[2]));
                subject.muscles[i].tendon_length_curve_coefficients.Add(Convert.ToDouble(values[3]));
            }
        }

        internal void MoveToPosition()
        {
            actuation_controller.MoveToPosition(command_position);
        }

        internal void GoToHomePosition()
        {
            epos.Operation.ProfilePositionMode.MoveToPosition(0, true, true);
        }

        internal void ActivateVelocityMode()
        {
            actuation_controller.ActivateVelocityMode();
            experiment_interface.textBox10.Text = epos.Operation.OperationMode.GetOperationModeAsString();
            ushort p = 0;
            ushort i = 0;
            ushort d = 0;
            epos.Configuration.Advanced.Regulator.GetPositionRegulatorGain(ref p, ref i, ref d);

            experiment_interface.textBox3.Text = Convert.ToString(p);
            experiment_interface.textBox4.Text = Convert.ToString(i);
            experiment_interface.textBox5.Text = Convert.ToString(d);
        }

        internal void ActivateProfilePositionMode()
        {
            actuation_controller.ActivateProfilePositionMode();
            experiment_interface.textBox10.Text = epos.Operation.OperationMode.GetOperationModeAsString();
            ushort p = 0;
            ushort i = 0;
            ushort d = 0;
            epos.Configuration.Advanced.Regulator.GetPositionRegulatorGain(ref p, ref i, ref d);

            experiment_interface.textBox3.Text = Convert.ToString(p);
            experiment_interface.textBox4.Text = Convert.ToString(i);
            experiment_interface.textBox5.Text = Convert.ToString(d);
        }

        private bool IsWithinPositionLimits(double average_position)
        {
            if (average_position <= upper_position_limit && average_position >= lower_position_limit)
                return true;
            else
                return false;
        }

        private void UpdateSubjectMotionData()
        {
            //make estimate and update next estimated position
            subject_motion_data.Clear();
            subject_motion_data.Add(biceps_emg);
            subject_motion_data.Add(triceps_emg);
            subject_motion_data.Add(subject_positions);
            subject_motion_data.Add(previous_biceps_emg);
            subject_motion_data.Add(previous_triceps_emg);
            subject_motion_data.Add(previous_subject_positions);
        }

        //<---------------------Thread Functions--------------------------------->

        //Begin execution of the thread for gathering EMG data from the Trigno. This will be called by a timer
        public void GetEMGDataThreadCall()
        {
            if (trigno.IsStreaming())
            {
                int available_samples = trigno.NumberOfEMGSamples(); //make sure trigno driver dumps old data, allow a buffer size to be set (5 s) gotta keep up to date: use a Queue
                if (available_samples >= window_size)
                {
                    try
                    {
                        if (!emg_data_worker.IsBusy)
                        {
                            stop_watch.Start();
                            emg_data_worker.RunWorkerAsync(emg_data_thread);
                        }
                        else
                            System.Console.WriteLine("Worker Busy");
                    }
                    catch (Exception ex)
                    {
                        System.Console.WriteLine(ex.Message);
                    }
                }
            }
        }

        private void EMGDataWorker_DoWork(object sender, System.ComponentModel.DoWorkEventArgs e)
        {
            EMGDataThread thread_class = (EMGDataThread)e.Argument;
            e.Result = thread_class.GetEMGData();
        }

        private void EMGDataWorker_RunWorkerCompleted(object sender, System.ComponentModel.RunWorkerCompletedEventArgs e)
        {
            List<List<double>> samples = (List<List<double>>) e.Result;
            List<List<double>> write_data;
            List<double> upsampled_motor_encoder = new List<double>(window_size); //in degrees
            List<double> upsampled_collection_arm_encoder = new List<double>(window_size); //in degrees
            List<double> biceps_emg_copy = new List<double>(samples[0]);
            List<double> triceps_emg_copy = new List<double>(samples[1]);
            double duration = window_size / emg_sampling_frequency; //in seconds
            double desired_frequency = emg_sampling_frequency; //match the EMG
            biceps_emg = samples[0];
            triceps_emg = samples[1];

            if (experiment_interface.display_emg_data)
                experiment_interface.GraphEMGData(biceps_emg, triceps_emg);

            if (biceps_emg.Count > 0 && triceps_emg.Count > 0 && subject_positions.Count > 0 && brace_positions.Count > 0)
            {
                if (save_motion_data || track_motion_data)
                {
                    //Update current parameters
                    subject_positions = Upsample(ref subject_positions, desired_frequency, duration);
                    previous_subject_positions = Upsample(ref previous_subject_positions, desired_frequency, duration);
                    upsampled_collection_arm_encoder = Upsample(ref subject_positions, desired_frequency, duration);
                    upsampled_motor_encoder = Upsample(ref brace_positions, desired_frequency, duration);
                    //Process data for estimation
                    GenerateFilterSamples(ref subject_positions, ref previous_subject_positions, signal_processing.position_filter.GetOrder());
                    subject_positions = signal_processing.FilterPosition(ref subject_positions, ref previous_subject_positions);

                    //biceps_emg = signal_processing.RemoveOffset(ref biceps_emg);
                    //triceps_emg = signal_processing.RemoveOffset(ref triceps_emg);
                    if (counter == 1)
                        System.Console.WriteLine(" ");

                    //Filter biceps and triceps EMG signals
                    GenerateFilterSamples(ref biceps_emg, ref previous_biceps_emg_filter1, signal_processing.emg_highpass_filter.GetOrder());
                    GenerateFilterSamples(ref triceps_emg, ref previous_triceps_emg_filter1, signal_processing.emg_highpass_filter.GetOrder());
                    biceps_emg = signal_processing.FilterEMG(ref biceps_emg, ref previous_biceps_emg_filter1);
                    triceps_emg = signal_processing.FilterEMG(ref triceps_emg, ref previous_triceps_emg_filter1);

                    previous_biceps_emg_filter1 = new List<double>(biceps_emg);
                    previous_triceps_emg_filter1 = new List<double>(triceps_emg);

                    //Rectify EMG signals
                    biceps_emg = signal_processing.Rectify(ref biceps_emg);
                    triceps_emg = signal_processing.Rectify(ref triceps_emg);

                    //Normalize signals
                    biceps_emg = signal_processing.Normalize(ref biceps_emg, subject.biceps.minimum_activation, subject.biceps.maximum_activation); //Set min and max values with main controller
                    triceps_emg = signal_processing.Normalize(ref triceps_emg, subject.triceps.minimum_activation, subject.triceps.maximum_activation); //Set min and max values with main controller

                    //Calculate Neural Activation
                    GenerateFilterSamples(ref biceps_emg, ref previous_biceps_emg, signal_processing.linear_envelope_filter.GetOrder());
                    GenerateFilterSamples(ref triceps_emg, ref previous_triceps_emg, signal_processing.linear_envelope_filter.GetOrder());
                    biceps_emg = signal_processing.LinearEnvelope(ref biceps_emg, ref previous_biceps_emg);
                    triceps_emg = signal_processing.LinearEnvelope(ref triceps_emg, ref previous_triceps_emg);

                    UpdateSubjectMotionData();

                    //Make copies of current data sources
                    previous_biceps_emg = new List<double>(biceps_emg);
                    previous_triceps_emg = new List<double>(triceps_emg);
                    previous_subject_positions = new List<double>(subject_positions);
                    previous_brace_positions = new List<double>(brace_positions);
                    counter++;
                }
            
                if (track_motion_data)
                {
                    //Get position estimates
                    if (counter == 1)
                        estimation_controllers[current_estimation_controller].SetPreviousPosition(subject_positions[subject_positions.Count-1]);
                    position_estimates = estimation_controllers[current_estimation_controller].EstimatePosition(subject_motion_data);

                    //Calculate the average position
                    double average_position = 0;
                    for (int i = 0; i < position_estimates.Count; i++)
                        average_position += position_estimates[i];
                    average_position /= position_estimates.Count;

                    //Position Limit Safety Check                    
                    if (IsWithinPositionLimits(average_position))
                    {
                        command_position = average_position;
                    }
                    else
                    {
                        command_position = brace_position;
                    }
                }

                if (save_motion_data)
                {
                    write_data = new List<List<double>>();
                    write_data.Add(biceps_emg_copy);
                    write_data.Add(triceps_emg_copy);
                    write_data.Add(upsampled_collection_arm_encoder); 
                    write_data.Add(position_estimates);
                    write_data.Add(upsampled_motor_encoder);
                    logger_thread.data = write_data;
                    logger_thread.writer = output_writer;
                    stop_watch3.Start();
                    logger_worker.RunWorkerAsync(logger_thread);
                }

                //Clear all data sources
                biceps_emg.Clear();
                triceps_emg.Clear();
                subject_positions.Clear();
                brace_positions.Clear();
            }

            stop_watch.Stop();
            tracking_loop_timer = stop_watch.Elapsed.TotalMilliseconds;
            //System.Console.WriteLine("EMG Thread Duration (ms): " + stop_watch.ElapsedMilliseconds);
            stop_watch.Reset();
        }

        //internal void StartSubjectPositionThread()
        //{
        //    subject_position_thread2 = new Thread(SubjectPositionThread2);
        //    subject_position_thread2.Start();
        //}

        //public void SubjectPositionThread2()
        //{
        //    while (collection_arm.serial_port.BytesToRead > 0 && experiment_interface.collection_arm_timer.Enabled)
        //    {
        //        double collected_position = collection_arm.ReadSerial();
        //        //subject_positions.Add(collected_position);
        //        subject_position = collected_position;
        //        //if (experiment_interface.display_subject_position_data)
        //        //experiment_interface.GraphSubjectPositionData(subject_position);
        //        stop_watch.Stop();
        //        stop_watch.Reset();
        //        //System.Console.WriteLine("Subject Position: " + subject_position.ToString());
        //        //System.Console.WriteLine("Subject Position Thread: " + stop_watch.ElapsedMilliseconds.ToString());
        //    }
        //}

        public void GetSubjectPositionThreadCall()
        {
            if (collection_arm.serial_port.BytesToRead > 0)
            {
                if (!subject_position_worker.IsBusy)
                {
                    stop_watch2.Start();
                    subject_position_worker.RunWorkerAsync(subject_position_thread);
                }
            }
        }

        private void SubjectPositionWorker_DoWork(object sender, System.ComponentModel.DoWorkEventArgs e)
        {
            SubjectPositionThread thread_call = (SubjectPositionThread)e.Argument;
            e.Result = thread_call.GetArmPosition();
        }

        private void SubjectPositionWorker_RunWorkerCompleted(object sender, System.ComponentModel.RunWorkerCompletedEventArgs e)
        {
            subject_positions.Add((double)e.Result);
            subject_position = (double)e.Result;
            if (experiment_interface.display_subject_position_data)
                experiment_interface.GraphSubjectPositionData(subject_position);
            stop_watch2.Stop();
            subject_position_timer = stop_watch2.Elapsed.TotalMilliseconds;
            stop_watch2.Reset();
            //System.Console.WriteLine("Subject Position Thread : " + stop_watch.ElapsedMilliseconds.ToString());
        }

        public void GetBracePositionThreadCall()
        {
            actuation_controller.GetBracePositionThreadCall();
        }

        private void LoggerWorker_DoWork(object sender, DoWorkEventArgs e)
        {
            LoggerThread thread_call = (LoggerThread)e.Argument;
            finished_logging = false;
            e.Result = thread_call.WriteToFile();
        }

        private void LoggerWorker_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            finished_logging = true;
            stop_watch3.Stop();
            data_log_timer = stop_watch3.Elapsed.TotalMilliseconds;
            stop_watch3.Reset();
        }

        //<---------------------External Device Functions--------------------------------->
        public void OpenEPOS()
        {
            epos_manager = new DeviceManager();

            //get baudrate info
            uint b = epos_manager.Baudrate;

            //set connection properties
            epos_manager.Baudrate = b;
            epos_manager.Timeout = 500;
        }

        public void ZeroEPOSEncoder()
        {
            brace_encoder_offset = brace_position;
        }

        internal void ConnectCollectionArm()
        {
            collection_arm.Connect();
            experiment_interface.textBox18.Text = "Connected";
            experiment_interface.pictureBox2.BackColor = System.Drawing.Color.Green;
        }

        public void DisconnectTrigno()
        {
            if (trigno.IsConnected())
                if (!trigno.IsStreaming())
                    trigno.Disconnect();
            if (!trigno.IsConnected())
            {
                trigno_state = States.Idle;
                experiment_interface.UpdateTrignoStatus(trigno_state.ToString());
            }
        }

        public void ConnectTrigno()
        {
            trigno.Connect("localhost");
            if (trigno.IsConnected())
            {
                trigno_state = States.Connected;
                experiment_interface.UpdateTrignoStatus(trigno_state.ToString());
            }
        }

        //Duration is in seconds
        public List<double> Upsample(ref List<double> samples, double desired_frequency, double duration)
        {
            double temp = (desired_frequency * duration); //the number of desired samples
            int desired_samples = Convert.ToInt32(Math.Round(temp));
            List<double> samples_us = new List<double>(desired_samples);
            int actual_samples = samples.Count;
            if (actual_samples > 1)
            {
                double offset = (double)desired_samples / actual_samples;
                List<double> positions = new List<double>(actual_samples + 1);

                positions.Add(1);
                for (int i = 1; i < actual_samples; i++)
                {
                    positions.Add(positions[i - 1] + offset);
                }
                positions.Add(desired_samples);

                int index = 1;
                samples_us.Add(samples[0]);
                for (int i = 1; i < actual_samples; i++)
                {
                    int inner_index = index;
                    int next_position = Convert.ToInt32(Math.Ceiling(positions[i]));
                    for (int j = 0; j < next_position - inner_index; j++)
                    {
                        samples_us.Add(samples[i - 1]);
                        index++;
                    }
                }
                int remaining = desired_samples - samples_us.Count;
                if (remaining > 0)
                {
                    for (int i = 0; i < remaining; i++)
                        samples_us.Add(samples[actual_samples - 1]);
                }
                else if (remaining < 0)
                {
                    for (int i = remaining; i > 0; i--)
                        samples_us.RemoveAt(samples_us.Count - 1);
                }
            }
            else
            {
                for (int i = 0; i < desired_samples; i++)
                    samples_us.Add(samples[0]);
            }
            return samples_us;
        }

        public void EnableEPOS()
        {
            try
            {
                epos = epos_manager.CreateDevice(Convert.ToUInt16(experiment_interface.textBox1.Text));
                actuation_controller.SetEPOS(epos);

                StateMachine sm = epos.Operation.StateMachine;
                sm.ClearFault();
                //if (sm.GetFaultState())
                //    sm.ClearFault();

                sm.SetDisableState();
                sm.SetEnableState();

                epos.Operation.ProfilePositionMode.ActivateProfilePositionMode();
                experiment_interface.textBox10.Text = epos.Operation.OperationMode.GetOperationModeAsString();                

                ushort p = 0;
                ushort i = 0;
                ushort d = 0;
                epos.Configuration.Advanced.Regulator.GetPositionRegulatorGain(ref p, ref i, ref d);

                experiment_interface.textBox3.Text = Convert.ToString(p);
                experiment_interface.textBox4.Text = Convert.ToString(i);
                experiment_interface.textBox5.Text = Convert.ToString(d);

                experiment_interface.textBox6.Text = Convert.ToString(epos_manager.Baudrate);
                experiment_interface.textBox7.Text = Convert.ToString(epos_manager.Timeout);

                experiment_interface.pictureBox1.BackColor = System.Drawing.Color.Green;
            }
            catch (DeviceException e)
            {
                experiment_interface.ShowMessageBox(e.ErrorMessage, e.ErrorCode);
                //add to logger
            }
            catch (Exception e)
            {
                System.Windows.Forms.MessageBox.Show(e.Message);
                //add to Logger
            }
        }

        public void SetCurrentEstimationController(int new_controller)
        {
            current_estimation_controller = new_controller;
        }

        public int GetCurrentEstimationController()
        {
            return current_estimation_controller;
        }

        public void SetCommandFrequency(double _command_frequency)
        {
            command_frequency = _command_frequency;
        }

        public List<double> GenerateFilterSamples(ref List<double> current, ref List<double> previous, int order)
        {
            List<double> mixed_samples = new List<double>(current.Count);
            for (int i = 0; i < order; i++)
            {
                mixed_samples.Add(previous[previous.Count - 1 - order + i]);
            }
            for (int i = 0; i < current.Count - order; i++)
            {
                mixed_samples.Add(current[i]);
            }
            return mixed_samples;
        }
    }
}
