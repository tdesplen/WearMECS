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
using namespace WearMEBraceControlSystem;

///------------------------------------------------------------------------------------------------------
///NOTE: This code is the implementation of the TaskController class, though it is explicitly not placed 
///into a class due the multithreading properties of C++
///------------------------------------------------------------------------------------------------------

#define EPOS_COM	"COM6" //COM port
#define NODE_ID		1 //EPOS Node ID

//Create objects
EstimationController predictor(2593944260579450, 1277546); //change the R and Q kalman filter values here
ActuationController * tracker = new ActuationController(125,0,0,0);
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
unsigned int _stdcall TrackPosition(void* parameters);
unsigned int _stdcall GetVelocity(void* parameters);
unsigned int _stdcall GetPosition(void* parameters);

bool complete = false;

//This struct(ure) allows for objects to interact controlled in different threads
struct Parameters
{
	EposController* controller;
	ActuationController* tracker;
	long desiredvelocity;
	long desiredposition;
	long upperlimit;
	long lowerlimit;
};

//Only used to initialze the main control loop and open a file for placing the predicted velocity
int main()
{
	char * filename = "control_system_data.txt";
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
	int windowsize = 1000; //the windowsize over which to use the prediction model
	int number_of_samples = 0;
	int windows = 0; //the number of windows of data available in the files
	int filtersize = windowsize + filterorder; //the number of samples needed for the filters to process one window worth of data
	double * angleset = new double[filtersize]; //holds one window worth of angle data 
	double * bicepsset = new double[filtersize]; //holds one window worth of biceps EMG data 
	double * tricepsset = new double[filtersize]; //holds one window worth of triceps EMG data 
	double * prevangle = new double[filterorder]; //holds the previous <filterorder> number of angle samples
	double * prevestimatedangle = new double[filterorder];
	double * prevbicepsemg = new double[filterorder]; //holds the previous <filterorder> number of biceps EMG samples
	double * prevtricepsemg = new double[filterorder]; //holds the previous <filterorder> number of triceps EMG samples
	double * anglesamples; //holds the entire array of angle samples collected from the data file
	double * bicepssamples; //holds the entire array of biceps EMG samples collected from the data file
	double * tricepssamples;//holds the entire array of triceps EMG samples collected from the data file

	Parameters parameters;
	//Timer variables
	clock_t start, loopstart;
	double duration = 0;
	complete = false;

	//Switch to velocity mode
	//eposcontroller.SwitchMode(-2);

	//Set the parameters for thread functions
	parameters.controller = &eposcontroller;
	parameters.tracker = tracker;

	//Set the prediction controller sizes and create the filters
	predictor.SetWindowSize(windowsize); //must be set before predictor can predict
	predictor.SetFilterSize(filtersize); //must be set before predictor can predict
	predictor.CreateFilters();
	cout << "Filters created." << endl;

	eposcontroller.MoveTo(0, tracker->DegreeToPulse(tracker->upperpositionlimit), tracker->DegreeToPulse(tracker->lowerpositionlimit));

	//Get data samples and assign them local variables
	parser->ParseDataFile("motion1.txt",3,1,2,3);
	number_of_samples = parser->number_of_samples;
	windows = number_of_samples / windowsize;
	anglesamples = parser->GetAngleSamples();
	bicepssamples = parser->GetBicepsSamples();
	tricepssamples = parser->GetTricepsSamples();
	cout << "Accelerometer and EMG data parsed." << endl;

	//Get voluntary contraction limits of each muscle and set them in the prediction controller
	parser->ParseVCFile("MVC_motion1.txt", 4); //change the file name to that of the correct voluntary contraction values of the subejct here
	double * vc_values = parser->GetVoluntaryContractionLimits();
	predictor.SetMaxMinEMGValues(vc_values[0], vc_values[1], vc_values[2], vc_values[3]); 
	cout << "Voluntary contraction values loaded." << endl;

	cout << "Moving to starting position: " << anglesamples[0] << endl;
	eposcontroller.MoveTo(tracker->DegreeToPulse(anglesamples[0]), tracker->DegreeToPulse(tracker->upperpositionlimit), tracker->DegreeToPulse(tracker->lowerpositionlimit));
	_getch();
	//Zero out the data arrays used to hold the previous window's data values
	for (int i = 0; i < filterorder; i++)
	{
		prevangle[i] = 0;
		prevestimatedangle[i] = 0;
		prevbicepsemg[i] = 0;
		prevtricepsemg[i] = 0;
	}

	handle[0] = (HANDLE)_beginthreadex(0, 0, GetPosition, &parameters, 0, 0);

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
		double * estimatedangle = predictor.EstimationPosition(angleset, bicepsset, tricepsset,prevangle,prevbicepsemg,prevtricepsemg, prevestimatedangle);
		double average_angle = predictor.GetAverageAngle(estimatedangle);

		//Set the previous data arrays to the updated ones contained in the prediction controller
		prevangle = predictor.prevang;
		prevestimatedangle = predictor.prevestimatedang;
		prevbicepsemg = predictor.prevbiceps;
		prevtricepsemg = predictor.prevtriceps;

		parameters.desiredposition = tracker->DegreeToPulse(average_angle);
		parameters.lowerlimit = tracker->DegreeToPulse(tracker->lowerpositionlimit);
		parameters.upperlimit = tracker->DegreeToPulse(tracker->upperpositionlimit);
		handle[1] = (HANDLE)_beginthreadex(0, 0, TrackPosition, &parameters, 0, 0);

		outputstream << i << ", " << average_angle << endl;
		duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		DWORD sleep_time = (DWORD)250 - (duration * 1000);
		if (sleep_time < windowsize) //ideally the control loop took less than 250ms - so sleep for the remainder of the time
			Sleep(sleep_time); //about 16 ms delay - total time interval should be 234 to compensate
		duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		cout << duration << endl;
	}
	duration = (std::clock() - loopstart) / (double)CLOCKS_PER_SEC;
	complete = true;
	//comment for estimates
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

unsigned int _stdcall GetPosition(void *parameters)
{
	//Timer variables
	clock_t start;
	double duration = 0;

	while (!complete)
	{
		/*start = clock();*/
		Parameters* param = (Parameters *)parameters;
		long position = 0;
		position = param->controller->GetPosition();
		param->tracker->SetPosition(position);
		///duration = (clock() - start) / (double)CLOCKS_PER_SEC;
		double pos = (double)position;
		pos = tracker->PulseToDegree(pos);
		//cout << position << "\t" << pos << endl;
	}
	return 0;
}

unsigned int _stdcall TrackPosition(void* parameters)
{
	Parameters* param = (Parameters *)parameters;
	param->controller->MoveTo(param->desiredposition,param->upperlimit, param->lowerlimit);
	return 0;
}
