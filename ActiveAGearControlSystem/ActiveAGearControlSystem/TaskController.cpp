//© 2019. Tyler Desplenter and Ana Luisa Trejos.

#include "EstimationController.h"
#include "ActuationController.h"
#include "EposController.h"
#include "FileParser.h"
#include <conio.h>
#include <ctime>
#include <Windows.h>
#include <process.h>
#include <exception>
#include <iostream>

using namespace std;
using namespace ActiveAGearControlSystem;

///------------------------------------------------------------------------------------------------------
///NOTE: This code is the implementation of the TaskController class, though it is explicitly not placed 
///into a class due the multithreading properties of C++
///------------------------------------------------------------------------------------------------------

#define NUM_OF_SAMPLES 89000 //Change the number of samples to process here
#define EPOS_COM	"COM4"
#define NODE_ID		8

//Create objects
EstimationController predictor(6396400000000000, 41378000); //change the R and Q kalman filter values here
ActuationController * tracker = new ActuationController(120,0,50,0.25);
EposController eposcontroller(EPOS_COM, NODE_ID);

//Create output file stream
ofstream outputstream;

//Thread variables
unsigned int ThreadID;
HANDLE * handle = new HANDLE[2]; //handle for TrackVelocity thread

//Loop function declaration
void mainloop();
void position_limit_loop();

//Thread function declaration
unsigned int _stdcall TrackVelocity(void* parameters);
unsigned int _stdcall GetVelocity(void* parameters);
unsigned int _stdcall GetPosition(void* parameters);

bool complete = false;

//This struct(ure) allows for objects to interact controlled in different threads
struct Parameters
{
	EposController* controller;
	ActuationController* tracker;
	long desiredvelocity;
};

//Only used to initialze the main control loop and open a file for placing the predicted velocity
int main()
{
	char * filename = "estimatedvelocity-controlsystem.txt";
	outputstream.open(filename, ios::out);

	mainloop(); //command the predicted motion
	//position_limit_loop();

	outputstream.close();
	return 0;
}

void mainloop()
{
	//Variable declaration
	FileParser * parser = new FileParser(); //used to parse the bioplux data files
	int filterorder = 2; //order of the filters, currently they are all second order -> **need to change filter code so data is filtered based on the order of each filter separately
	int windowsize = 250; //the windowsize over which to use the prediction model
	int windows = NUM_OF_SAMPLES/windowsize; //the number of windows of data available in the files
	int filtersize = windowsize + filterorder; //the number of samples needed for the filters to process one window worth of data
	double * angleset = new double[filtersize]; //holds one window worth of angle data 
	double * bicepsset = new double[filtersize]; //holds one window worth of biceps EMG data 
	double * tricepsset = new double[filtersize]; //holds one window worth of triceps EMG data 
	double * prevangle = new double[filterorder]; //holds the previous <filterorder> number of angle samples
	double * prevestimatedangle = new double[filterorder];
	double * prevbicepsemg = new double[filterorder]; //holds the previous <filterorder> number of biceps EMG samples
	double * prevtricepsemg = new double[filterorder]; //holds the previous <filterorder> number of triceps EMG samples
	double * anglesamples; //holds the entire array of angle samples collected from the data file
	double * acc1samples;
	double * acc2samples;
	double * bicepssamples; //holds the entire array of biceps EMG samples collected from the data file
	double * tricepssamples;//holds the entire array of triceps EMG samples collected from the data file
	double velocity_threshold = 0;
	Parameters parameters;
	//Timer variables
	clock_t start, loopstart;
	double duration = 0;
	complete = false;

	//Switch to velocity mode
	eposcontroller.SwitchMode(-2);

	//Set the parameters for thread functions
	parameters.controller = &eposcontroller;
	parameters.tracker = tracker;

	//Set the prediction controller sizes and create the filters
	predictor.SetWindowSize(windowsize); //must be set before predictor can predict
	predictor.SetFilterSize(filtersize); //must be set before predictor can predict
	predictor.CreateFilters();
	cout << "Filters created." << endl;

	//Get data samples and assign them local variables
	//parser->ParseDelsysEMGFile("0-90-rep-1.txt", NUM_OF_SAMPLES, 5); //change the file name of the bioplux data file here
	parser->ParseDelsysAccelerometerFile("acceleratometer1.txt", NUM_OF_SAMPLES, 25,2,4);
	parser->ParseDelsysEMGFile("emg1.txt", NUM_OF_SAMPLES, 9, 3, 4);
	acc1samples = parser->GetAccelerometer1Samples();
	acc2samples = parser->GetAccelerometer2Samples();
	bicepssamples = parser->GetBicepsSamples();
	tricepssamples = parser->GetTricepsSamples();
	cout << "Accelerometer and EMG data parsed." << endl;

	//Get voluntary contraction limits of each muscle and set them in the prediction controller
	parser->ParseVCFile("MVC1.txt", 4); //change the file name to that of the correct maximum voluntary contraction values of the subejct here
	double * vc_values = parser->GetVoluntaryContractionLimits();
	predictor.SetMaxMinEMGValues(vc_values[0], vc_values[1], vc_values[2], vc_values[3]); 
	cout << "Voluntary contraction values loaded." << endl;

	//Calculate Angle
	anglesamples = new double[NUM_OF_SAMPLES];
	for (int i = 0; i < NUM_OF_SAMPLES; i++)
	{
		double v1[2] = { acc1samples[0], acc2samples[0] };
		double v2[2] = { acc1samples[i], acc2samples[i] };
		double dot_product = (v1[0] * v2[0]) + (v1[1] * v2[1]);
		double norm1 = sqrt(pow(v1[0], 2) + pow(v1[1], 2));
		double norm2 = sqrt(pow(v2[0], 2) + pow(v2[1], 2));
		double temp = dot_product / (norm1 * norm2);
		if (temp > 1.0)
			temp = 1.0;
		anglesamples[i] = (57.2957795 * acos(temp)); //calculate angle in radians and convert to degrees
	}
	cout << "Angle calculated." << endl;

	//Rectify EMG samples
	for (int i = 0; i < NUM_OF_SAMPLES; i++)
	{
		bicepssamples[i] = abs(bicepssamples[i]);
		tricepssamples[i] = abs(tricepssamples[i]);
	}
	cout << "EMG signal rectified." << endl;

	//Zero out the data arrays used to hold the previous window's data values
	for (int i = 0; i < filterorder; i++)
	{
		prevangle[i] = 0;
		prevestimatedangle[i] = 0;
		prevbicepsemg[i] = 0;
		prevtricepsemg[i] = 0;
	}

	//Set the velocity threshold (in degrees per second of elbow speed) before commanding velocity
	velocity_threshold = 2.0;
	long command_velocity = 0;

	loopstart = clock();
	//For each window, read a window's worth of data and get the predicted velocity
	for (int i = 0; i < windows; i++)
	{
		start = clock();

		if (i == 0) //For the first window
		{
			for (int r = 0; r < filtersize - windowsize; r++) //fill the previous data samples with zeros
			{
				angleset[r] = anglesamples[0];
				bicepsset[r] = bicepssamples[0];
				tricepsset[r] = tricepssamples[0];
			}
		}
		else // For the rest of the windows
		{
			int start = (i * windowsize) - filterorder; // define the start of the samples from the previous window
			for (int r = 0; r < filterorder; r++) //fill the previous data samples with their values from the previous window
			{
				angleset[r] = anglesamples[start + r];
				bicepsset[r] = bicepssamples[start + r];
				tricepsset[r] = tricepssamples[start + r];
			}
		}
		int f = 0;
		for (int j = filterorder; j < filtersize; j++) //fill the rest of the window with new data
		{
			angleset[j] = anglesamples[i * windowsize + f];
			bicepsset[j] = bicepssamples[i * windowsize + f];
			tricepsset[j] = tricepssamples[i * windowsize + f];
			f++;
		}

		//Get the estimated angle from the prediction controller and use it to determine the average velocity for the window
		double * estimatedangle = predictor.GetEstimatedAngle(angleset, bicepsset, tricepsset,prevangle,prevbicepsemg,prevtricepsemg, prevestimatedangle);
		double * estimated_velocity = predictor.GetEstimatedVelocity(estimatedangle);
		double average_velocity = predictor.GetAverageVelocity(estimatedangle);

		//Get the filter real angle from the prediction controller
		double * filteredangle = predictor.GetRealAngle();

		//Calculate the root-mean squared error of the position for each window
		double  rms = predictor.GetRMSPositionError(filteredangle, estimatedangle);

		//Set the previous data arrays to the updated ones contained in the prediction controller
		prevangle = predictor.prevang;
		prevestimatedangle = predictor.prevestimatedang;
		prevbicepsemg = predictor.prevbiceps;
		prevtricepsemg = predictor.prevtriceps;

		//start thread for getting position
		handle[0] = (HANDLE)_beginthreadex(0, 0, GetPosition, &parameters, 0, 0);

		//Calculate the velocity in RPM to send to the motor
		long velocity_rpm2 = (long) tracker->DegreePerSecondToRPM(average_velocity);
		int * velocity_rpm = new int[windowsize];
		for (int i = 0; i < windowsize; i++)
			velocity_rpm[i] = (int) tracker->DegreePerSecondToRPM(estimated_velocity[i]);

		//test position
		/*position = estimatedangle[0];
		tracker->SetPosition(position);*/
		command_velocity = tracker->MakeMotionDecision((int) velocity_rpm2);

		//Command a velocity based on the threshold
		if (abs(average_velocity) > velocity_threshold && i > 0)
		{
			parameters.desiredvelocity = command_velocity;
			handle[1] = (HANDLE)_beginthreadex(0, 0, TrackVelocity, &parameters, 0, 0);
		}
		else
		{
			parameters.desiredvelocity = 0;
			handle[1] = (HANDLE)_beginthreadex(0, 0, TrackVelocity, &parameters, 0, 0);
		}

		//std::cout << i << std::endl; //print the window number, starting at 0
		for (int i = 0; i < windowsize; i++)
			outputstream << i << "," << estimated_velocity[i]  << "," << velocity_rpm[i] << "," << tracker->PulseToDegree(tracker->position) << "," << command_velocity << endl; //output the predicted velocity and RMSE for each window
		duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		DWORD sleep_time = (DWORD)((windowsize - 15) - (duration * 1000));
		if (sleep_time < windowsize) //ideally the control loop took less than 250ms - so sleep for the remainder of the time
			Sleep(sleep_time); //about 16 ms delay - total time interval should be 234 to compensate
		duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		//outputstream << duration << endl;
	}
	parameters.desiredvelocity = 0;
	handle[1] = (HANDLE)_beginthreadex(0, 0, TrackVelocity, &parameters, 0, 0);
	duration = (std::clock() - loopstart) / (double)CLOCKS_PER_SEC;
	complete = true;
	WaitForMultipleObjects(2, handle, true, INFINITE);
	try
	{
		CloseHandle(handle[0]);
		CloseHandle(handle[1]);
	}
	catch (exception ex)
	{
		cout << ex.what() << endl;
	}
}

void position_limit_loop()
{
	while (true)
	{
		//eposcontroller.DisableState();
		double sensor_position = (double) eposcontroller.GetPosition();
		tracker->SetPosition(sensor_position);
		double real_position = tracker->PulseToDegree(sensor_position);
		int velocity = tracker->MakeMotionDecision(1);
		cout << sensor_position << "\t" << real_position << "\t" << velocity << endl;
	}
}

unsigned int _stdcall TrackVelocity(void* parameters)
{
	/*while (!complete)
	{*/
		Parameters* param = (Parameters *)parameters;
		param->controller->TrackVelocity(param->desiredvelocity);
	//}
	return 0;
}

unsigned int _stdcall GetVelocity(void *parameters)
{
	//clock_t start;
	while (!complete)
	{
		//start = clock();
		Parameters* param = (Parameters *)parameters;
		long velocity = 0;
		velocity = param->controller->GetVelocity();
		param->tracker->SetVelocity(velocity);
		//double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		//cout << duration << endl;
	}
	return 0;
}

unsigned int _stdcall GetPosition(void *parameters)
{
	//Timer variables
	clock_t start;
	double duration = 0;

	//while (!complete)
	//{
		/*start = clock();*/
		Parameters* param = (Parameters *)parameters;
		long position = 0;
		position = param->controller->GetPosition();
		param->tracker->SetPosition(position);
		///duration = (clock() - start) / (double)CLOCKS_PER_SEC;
		double pos = (double)position;
		pos = tracker->PulseToDegree(pos);
		cout << position << "\t" << pos << endl;
	//}
	return 0;
}
