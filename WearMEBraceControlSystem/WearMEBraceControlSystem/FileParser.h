//© 2019. Tyler Desplenter and Ana Luisa Trejos.

#include <iostream>
#include <fstream>
#include <string>
#include <exception>
using namespace std;

namespace WearMEBraceControlSystem
{
	class FileParser
	{
	public:
		FileParser();
		void ParseDataFile(char * filename, int number_of_channels, int angle_channel, int biceps_channel, int triceps_channel);
		void ParseVCFile(char * filename, int number_of_samples);
		double * GetAngleSamples();
		double * GetBicepsSamples();
		double * GetTricepsSamples();
		double * GetVoluntaryContractionLimits();
		int number_of_samples;
	private:
		double * angledata; //a pointer to an array of angle samples
		double * bicepsdata; //a pointer to an array of biceps emg samples
		double * tricepsdata; //a pointer to an array of triceps emg samples
		double * voluntary_contraction_limits; //a point to an array of voluntary muscle contraction values
	};
}