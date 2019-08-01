//© 2019. Tyler Desplenter and Ana Luisa Trejos.

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace WearMEBenchTopControlSystem
{
    public partial class ExperimentInterface : Form
    {

        private ElbowMotionTrackingController motion_tracking_controller;
        private double collection_arm_sampling_frequency;
        private double motor_encoder_sampling_frequency;
        private double motor_command_frequency;

        public bool display_emg_data;
        public bool display_subject_position_data;

        private int trigno_sensor_biceps; //sensor number for biceps
        private int trigno_sensor_triceps; //sensor number for triceps
        private int window_size;
        private string optimization_motion_data_file;
        private string timing_file;
        private Stopwatch stop_watch;
        private int subject_data_loaded;

        public ExperimentInterface()
        {

            InitializeComponent();
            display_emg_data = checkBox1.Checked;
            display_subject_position_data = checkBox2.Checked;
            window_size = Convert.ToInt32(textBox16.Text);
            //Set combobox defaults
            comboBox1.SelectedIndex = 2;
            comboBox2.SelectedIndex = 3;
            comboBox9.SelectedIndex = 0;
            comboBox5.SelectedIndex = 0;
            comboBox8.SelectedIndex = 0;
            //Get the current sensors
            trigno_sensor_biceps = Convert.ToInt32(comboBox1.SelectedItem.ToString());
            trigno_sensor_triceps = Convert.ToInt32(comboBox2.SelectedItem.ToString());
            motion_tracking_controller = new ElbowMotionTrackingController(window_size, trigno_sensor_biceps, trigno_sensor_triceps, this);
            stop_watch = new Stopwatch();
            CheckControlSettings();
            subject_data_loaded = 0;
        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {

        }

        private void ExperimentInterface_Load(object sender, EventArgs e)
        {
            textBox19.Text = "motion data\\" + textBox27.Text + "-" + comboBox5.SelectedItem.ToString() + "-R" + comboBox9.SelectedItem.ToString() + "-M" + comboBox8.SelectedIndex + ".txt";
        }

        private void label27_Click(object sender, EventArgs e)
        {


        }

        private void label34_Click(object sender, EventArgs e)
        {

        }

        private void label31_Click(object sender, EventArgs e)
        {

        }

        private void button8_Click(object sender, EventArgs e)
        {
            stop_watch.Start();
            motion_tracking_controller.RotateMotorForward();
            stop_watch.Stop();
            System.Console.WriteLine("Rotate forward response time " + stop_watch.ElapsedMilliseconds);
            stop_watch.Reset();
        }

        private void button6_Click(object sender, EventArgs e)
        {
            motion_tracking_controller.DisableEPOS();
            brace_position_timer.Enabled = false;
        }

        private void label25_Click(object sender, EventArgs e)
        {

        }

        private void enableMotorButton_Click(object sender, EventArgs e)
        {

        }

        private void pictureBox8_Click(object sender, EventArgs e)
        {

        }

        private void textBox23_TextChanged(object sender, EventArgs e)
        {

        }

        private void label46_Click(object sender, EventArgs e)
        {

        }

        private void textBox27_TextChanged(object sender, EventArgs e)
        {
            textBox19.Text = "motion data\\" + textBox27.Text + "-" + comboBox5.SelectedItem.ToString() + "-R" + comboBox9.SelectedItem.ToString() + "-M" + comboBox8.SelectedIndex + ".txt";
        }

        private void textBox26_TextChanged(object sender, EventArgs e)
        {

        }

        private void button12_Click(object sender, EventArgs e)
        {
            optimization_motion_data_file = textBox19.Text;
            timing_file = "timing data\\" + textBox27.Text + "-" + comboBox5.SelectedItem.ToString() + "-R" + comboBox9.SelectedItem.ToString() + "-M" + comboBox8.SelectedIndex + ".txt";
            textBox21.AppendText("Recording motion trajectory. Data saved in: " + optimization_motion_data_file + "\r\n");
            motion_tracking_controller.SetLoggerFilename(optimization_motion_data_file, timing_file);
            motion_tracking_controller.SavingMotionDataEnable(true);
            
        }

        private void button14_Click(object sender, EventArgs e)
        {
            textBox21.AppendText("Recording stopped.\r\n");
            motion_tracking_controller.SavingMotionDataEnable(false);
            motion_tracking_controller.CloseLoggingFile();
        }

        public void UpdateTrignoStatus(String trigno_state)
        {
            textBox11.Text = trigno_state;
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            //check current positions from task controller
            textBox2.Text = Convert.ToString(motion_tracking_controller.brace_position - motion_tracking_controller.brace_encoder_offset);
            textBox14.Text = Convert.ToString(motion_tracking_controller.brace_position - motion_tracking_controller.brace_encoder_offset);
            textBox12.Text = Convert.ToString(motion_tracking_controller.subject_position);
            textBox17.Text = Convert.ToString(motion_tracking_controller.subject_position);
        }

        private void connect_button_Click(object sender, EventArgs e)
        {
            motion_tracking_controller.ConnectTrigno();
        }

        private void start_button_Click(object sender, EventArgs e)
        {
            //start updating
            motion_tracking_controller.StartTrignoSampling();
            trigno_sensor_biceps = Convert.ToInt32(comboBox1.SelectedItem.ToString());
            trigno_sensor_triceps = Convert.ToInt32(comboBox2.SelectedItem.ToString());
            motion_tracking_controller.SetEMGChannels(trigno_sensor_biceps, trigno_sensor_triceps);
        }

        private void button7_Click(object sender, EventArgs e)
        {
            //make sure to start a thread for updating the brace position at this point
            motion_tracking_controller.OpenEPOS();
        }

        private void button3_Click(object sender, EventArgs e)
        {
            //make sure to begin updating the subject position at this point
            collection_arm_timer.Enabled = true;
            textBox18.Text = "Sampling";
            position_update_timer.Enabled = true;
            //motion_tracking_controller.StartSubjectPositionThread();
        }

        private void stop_button_Click(object sender, EventArgs e)
        {
            motion_tracking_controller.StopTrignoSampling();
        }

        private void disconnect_button_Click(object sender, EventArgs e)
        {
            motion_tracking_controller.DisconnectTrigno();
        }

        private void timer2_Tick(object sender, EventArgs e)
        {
            //Gather Trigno data
            motion_tracking_controller.GetEMGDataThreadCall();
        }

        public void EnablePositionCheckTimer(bool v)
        {
            position_update_timer.Enabled = v;
        }

        public void GraphEMGData(List<double> biceps_emg, List<double> triceps_emg)
        {
            for (int i = 0; i < biceps_emg.Count; i++)
            {
                if (chart1.Series["EMG"].Points.Count < 4000) //if we are less than 2 seconds worth of data being displayed
                {
                    chart1.Series["EMG"].Points.Add(biceps_emg[i]); //keep adding data
                    chart2.Series["EMG"].Points.Add(triceps_emg[i]);
                }
                else //if we have 2 seconds worth already displayed
                {
                    chart1.Series["EMG"].Points.Add(biceps_emg[i]); //add a point and
                    chart1.Series["EMG"].Points.RemoveAt(0); //remove the oldest one
                    chart2.Series["EMG"].Points.Add(triceps_emg[i]);
                    chart2.Series["EMG"].Points.RemoveAt(0);
                }
            }
        }

        private void button10_Click(object sender, EventArgs e)
        {
            textBox21.AppendText("Tracking started.\r\n");
            motion_tracking_controller.SetCurrentEstimationController(comboBox8.SelectedIndex);
            //start logging motion data to file
            if (!motion_tracking_controller.GetSavingMotionDataEnable())
            {
                //set filename before tracking to make sure it is saving in the right location
                optimization_motion_data_file = "motion data\\" + textBox27.Text + "-" + comboBox5.SelectedItem.ToString() + "-R" + comboBox9.SelectedItem.ToString() + "-M" + comboBox8.SelectedIndex + ".txt";
                timing_file = "timing data\\" + textBox27.Text + "-" + comboBox5.SelectedItem.ToString() + "-R" + comboBox9.SelectedItem.ToString() + "-M" + comboBox8.SelectedIndex + ".txt";
                motion_tracking_controller.SetLoggerFilename(optimization_motion_data_file, timing_file);
                motion_tracking_controller.SavingMotionDataEnable(true);
            }

            if (!motion_tracking_controller.GetTrackingEnable())
                motion_tracking_controller.SetTrackingEnable(true);

            motion_tracking_controller.SetBodyForEstimation();

            //enable timers
            command_position_timer.Enabled = true;

            timing_log_timer.Enabled = true;         
        }

        internal void EnableEMGTimer(bool v)
        {
            trigno_timer.Enabled = v;
        }

        private void textBox30_TextChanged(object sender, EventArgs e)
        {
            collection_arm_sampling_frequency = Convert.ToDouble(textBox30.Text);
        }

        private void textBox29_TextChanged(object sender, EventArgs e)
        {
            motor_encoder_sampling_frequency = Convert.ToDouble(textBox29.Text);
        }

        private void checkBox1_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox1.Checked)
                display_emg_data = true;
            else
            {
                display_emg_data = false;
                chart1.Series["EMG"].Points.Clear();
                chart2.Series["EMG"].Points.Clear();
            }
        }

        private void comboBox1_SelectedIndexChanged(object sender, EventArgs e)
        {
            trigno_sensor_biceps = Convert.ToInt32(comboBox1.SelectedItem.ToString());
        }

        private void comboBox2_SelectedIndexChanged(object sender, EventArgs e)
        {
            trigno_sensor_triceps = Convert.ToInt32(comboBox2.SelectedItem.ToString());
        }

        private void button5_Click(object sender, EventArgs e)
        {
            motion_tracking_controller.EnableEPOS();
            if (motion_tracking_controller.EPOSIsEnabled())
                brace_position_timer.Enabled = true;
        }

        public void ShowMessageBox(string text, uint errorCode)
        {
            string msg;

            msg = String.Format("{0}\nErrorCode: {1:X8}", text, errorCode);
            MessageBox.Show(msg, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }

        private void button1_Click(object sender, EventArgs e)
        {
            stop_watch.Start();
            motion_tracking_controller.RotateMotorBackwards();
            stop_watch.Stop();
            System.Console.WriteLine("Rotate backward response time: " + stop_watch.ElapsedMilliseconds);
            stop_watch.Reset();
        }

        private void button9_Click(object sender, EventArgs e)
        {
            stop_watch.Start();
            motion_tracking_controller.StopMotor();
            stop_watch.Stop();
            System.Console.WriteLine("Stop moition response time: " + stop_watch.ElapsedMilliseconds);
            stop_watch.Reset();
        }

        private void button4_Click(object sender, EventArgs e)
        {
            motion_tracking_controller.ZeroEPOSEncoder();
        }

        private void button2_Click(object sender, EventArgs e)
        {
            motion_tracking_controller.ConnectCollectionArm();
        }

        private void timer1_Tick_1(object sender, EventArgs e)
        {
            stop_watch.Start();
            motion_tracking_controller.GetSubjectPositionThreadCall();
            textBox17.Text = Convert.ToString(motion_tracking_controller.subject_position);
        }

        private void brace_position_timer_Tick(object sender, EventArgs e)
        {
            motion_tracking_controller.GetBracePositionThreadCall();
        }

        private void button15_Click(object sender, EventArgs e)
        {
            collection_arm_timer.Enabled = false;
            motion_tracking_controller.DisconnectCollectionArm();
        }

        private void textBox16_TextChanged(object sender, EventArgs e)
        {
            CheckControlSettings();
        }

        private void CheckControlSettings()
        {
            if (textBox16.Text != String.Empty && textBox8.Text != String.Empty)
            {
                motion_tracking_controller.SetWindowSize(Convert.ToInt32(textBox16.Text));
                motion_tracking_controller.SetCommandFrequency(Convert.ToDouble(textBox8.Text));
                command_position_timer.Interval = Convert.ToInt32(Math.Round((1 / motion_tracking_controller.command_frequency) * 1000));
                pictureBox4.BackColor = System.Drawing.Color.Green;
            }
            else
            {
                pictureBox4.BackColor = System.Drawing.Color.Red;
            }
        }

        private void button13_Click(object sender, EventArgs e)
        {
            //Load all subjcet data - fill the Muscle objects with the data
            textBox21.AppendText("Loading subject MVC data.\r\n");
            subject_data_loaded = 0;
            //MVCs
            try
            {
                string filename = "motion data\\" + textBox27.Text + "-MVC.txt";
                StreamReader reader = new StreamReader(filename);
                Char delimiter = ',';

                //File Format: row 1-> header, row 2-> parameters
                String[] header = reader.ReadLine().Split(delimiter);                              
                String[] mvc_values = reader.ReadLine().Split(delimiter);
                motion_tracking_controller.SetMVCValues(mvc_values);
                subject_data_loaded++;

            }
            catch(Exception ex)
            {
                textBox21.AppendText("MVC data load: " + ex.Message + "\r\n");
            }

            textBox21.AppendText("Loading subject skeletal data.\r\n");
            //skeletal parameters - fill Body object with the data
            try
            {
                //File format: row 1-> parameter names, row 2 -> parameters of the subjects skeleton
                string filename = "motion data\\" + textBox27.Text + "-subject-parameters.txt";
                StreamReader reader = new StreamReader(filename);
                Char delimiter = ',';
                String[] header = reader.ReadLine().Split(delimiter);
                String[] values = reader.ReadLine().Split(delimiter);
                motion_tracking_controller.SetSkeletalParameters(values);
                subject_data_loaded++;

            }
            catch (Exception ex)
            {
                textBox21.AppendText("Skeletal data load: " + ex.Message + "\r\n");
            }

            textBox21.AppendText("Loading subject optimization data.\r\n");
            //optimization parameters - fill EstimationController objects with the data
            try
            {
                //File format: row 1-> parameter names, row 2 through n-> optimization parameters values for nth model
                string filename = "motion data\\" + textBox27.Text + "-optimization-parameters.txt";
                StreamReader reader = new StreamReader(filename);
                Char delimiter = ',';
                List<String> data = new List<String>();
                while (!reader.EndOfStream)
                    data.Add(reader.ReadLine());
                motion_tracking_controller.SetOptimizationParameters(data);
                subject_data_loaded++;
            }
            catch (Exception ex)
            {
                textBox21.AppendText("Optimization data load: " + ex.Message + "\r\n");
            }

            if (subject_data_loaded == 2)
            {
                pictureBox3.BackColor = System.Drawing.Color.Green;
                textBox21.AppendText("Subject data loading is complete.\r\n");
            }
            else
                pictureBox3.BackColor = System.Drawing.Color.Red;

        }

        private void comboBox8_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (motion_tracking_controller != null)
                motion_tracking_controller.SetCurrentEstimationController(comboBox8.SelectedIndex);
            textBox19.Text = "motion data\\" + textBox27.Text + "-" + comboBox5.SelectedItem.ToString() + "-R" + comboBox9.SelectedItem.ToString() + "-M" + comboBox8.SelectedIndex + ".txt";
        }

        private void button11_Click(object sender, EventArgs e)
        {
            command_position_timer.Enabled = false;
            textBox21.AppendText("Tracking stopped.\r\n");
            motion_tracking_controller.SetTrackingEnable(false);
            motion_tracking_controller.SavingMotionDataEnable(false);
            timing_log_timer.Enabled = false;
            motion_tracking_controller.CloseLoggingFile();
        }

        private void command_position_timer_Tick(object sender, EventArgs e)
        {
            if (motion_tracking_controller.GetTrackingEnable())
            {
                motion_tracking_controller.MoveToPosition();
            }
        }

        private void textBox8_TextChanged(object sender, EventArgs e)
        {
            //Change timer interval
            CheckControlSettings();            
        }

        private void checkBox2_CheckedChanged(object sender, EventArgs e)
        {
            if (checkBox2.Checked)
                display_subject_position_data = true;
            else
            {
                display_subject_position_data = false;
                chart3.Series["Position"].Points.Clear();
            }
        }

        public void GraphSubjectPositionData(double subject_positon)
        {
            if (chart3.Series["Position"].Points.Count < 100) //if we are less than 2 seconds worth of data being displayed
            {
                chart3.Series["Position"].Points.Add(subject_positon); //keep adding data
            }
            else //if we have 5 seconds worth already displayed
            {
                chart3.Series["Position"].Points.Add(subject_positon); //add a point and
                chart3.Series["Position"].Points.RemoveAt(0); //remove the oldest one
            }
        }

        private void chart3_Click(object sender, EventArgs e)
        {

        }
        private void UpdateFilename()
        {
            textBox19.Text = "motion data\\" + textBox27.Text + "-" + comboBox5.SelectedItem.ToString() + "-R" + comboBox9.SelectedItem.ToString() + "-M" + comboBox8.SelectedIndex + ".txt";
        }

        private void comboBox5_SelectedIndexChanged(object sender, EventArgs e)
        {
            UpdateFilename();
        }

        private void comboBox9_SelectedIndexChanged(object sender, EventArgs e)
        {
            UpdateFilename();
        }

        private void button16_Click(object sender, EventArgs e)
        {
            motion_tracking_controller.ActivateProfilePositionMode();
        }

        private void textBox9_TextChanged(object sender, EventArgs e)
        {

        }

        private void button17_Click(object sender, EventArgs e)
        {
            motion_tracking_controller.ActivateVelocityMode();
        }

        private void button18_Click(object sender, EventArgs e)
        {
            motion_tracking_controller.GoToHomePosition();
        }

        private void timing_log_timer_Tick(object sender, EventArgs e)
        {
            motion_tracking_controller.WriteTimingData();
        }

        private void textBox19_TextChanged(object sender, EventArgs e)
        {
            optimization_motion_data_file = "motion data\\" + textBox27.Text + "-" + comboBox5.SelectedItem.ToString() + "-R" + comboBox9.SelectedItem.ToString() + "-M" + comboBox8.SelectedIndex + ".txt";
        }
    }
}