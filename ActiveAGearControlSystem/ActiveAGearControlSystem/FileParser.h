//© 2019. Tyler Desplenter and Ana Luisa Trejos.

#include <iostream>
#include <fstream>
#include <string>
using namespace std;

namespace ActiveAGearControlSystem
{
	class FileParser
	{
	public:
		FileParser();
		void ParseDelsysEMGFile(char * filename, int number_of_samples, int number_of_channels, int biceps_channel, int triceps_channel);
		void ParseDelsysAccelerometerFile(char * filename, int number_of_samples, int number_of_channels, int acc1_channel, int acc2_channel);
		void ParseVCFile(char * filename, int number_of_samples);
		double * GetAccelerometer1Samples();
		double * GetAccelerometer2Samples();
		double * GetBicepsSamples();
		double * GetTricepsSamples();
		double * GetVoluntaryContractionLimits();

	private:
		double * acc1data; //a pointer to an array of angle samples
		double * acc2data; //a pointer to an array of angle samples
		double * bicepsdata; //a pointer to an array of biceps emg samples
		double * tricepsdata; //a pointer to an array of triceps emg samples
		double * voluntary_contraction_limits; //a point to an array of voluntary muscle contraction values
	};
}