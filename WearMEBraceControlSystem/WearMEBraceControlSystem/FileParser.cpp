//© 2019. Tyler Desplenter and Ana Luisa Trejos.

#include "FileParser.h"

namespace WearMEBraceControlSystem
{
	///<summary> The default contrusctor for the FileParser object sets private variables to 0. </summary>
	FileParser::FileParser()
	{
		angledata = 0;
		bicepsdata = 0;
		tricepsdata = 0;
		voluntary_contraction_limits = 0;
		number_of_samples = 0;
	}

	///<summary> A function for parsing Bioplux .txt files and gathering the values in double format. </summary>
	///<param name="filename"> The name of the Bioplux data file. </param>
	///<param name="number_of_samples"> The number of samples to read from the file. </param>
	///<param name="number_of_channels"> The number of channels of data contained in the file. </param>
	///<returns> N/A </returns>
	void FileParser::ParseDataFile(char * filename, int number_of_channels, int angle_channel, int biceps_channel, int triceps_channel)
	{
		ifstream data_stream;
		string line;
		string::size_type st;
		try {
			data_stream.open(filename, ios::in);
		}
		catch (exception& e)
		{
			cout << e.what() << endl;
		}
		//get number of lines
		int i = 0;
		char c = ' ';
		while (data_stream.get(c))
		{
			if (c == '\n')
				i++;
		}
		number_of_samples = i;
		data_stream.close();

		string delimeter = ",";
		int columns = number_of_channels;
		angledata = new double[number_of_samples];
		bicepsdata = new double[number_of_samples];
		tricepsdata = new double[number_of_samples];

		try {
			data_stream.open(filename, ios::in);
		}
		catch (exception& e)
		{
			cout << e.what() << endl;
		}
		
		for (int j = 0; j < number_of_samples; j++)
		{
			getline(data_stream, line);
			for (int n = 1; n <= columns; n++)
			{
				int end_position = line.find(delimeter);
				string sample = line.substr(0, end_position);
				if (n == angle_channel) //Angle samples
					angledata[j] = stod(sample, &st);
				else if (n == biceps_channel)// Biceps EMG samples
					bicepsdata[j] = stod(sample, &st);
				else if (n == triceps_channel) // Triceps EMG samples
					tricepsdata[j] = stod(sample, &st);
				line.erase(0, end_position + delimeter.length());
			}
		}
		data_stream.close();
	}

	///<summary> A function for parsing .txt files that contain voluntary contraction information. </summary>
	///<param name="filename"> The name of the Bioplux data file. </param>
	///<param name="number_of_samples"> The number of samples to read from the file. </param>
	///<returns> N/A </returns>
	void FileParser::ParseVCFile(char * filename, int number_of_samples)
	{
		ifstream mvc_stream;
		string line;
		string::size_type st;
		voluntary_contraction_limits = new double[number_of_samples];
		mvc_stream.open(filename, ios::in);
		for (int i = 0; i < number_of_samples; i++)
		{
			getline(mvc_stream, line);
			voluntary_contraction_limits[i] = stof(line, &st);
		}
		mvc_stream.close();
	}

	///<summary>A method getting the pointer to the angle samples gathered from the Bioplux data file. </summary>
	///<returns> A point to the array of angle samples. </returns>
	double * FileParser::GetAngleSamples()
	{
		return angledata;
	}

	///<summary>A method getting the pointer to the biceps EMG samples gathered from the Bioplux data file. </summary>
	///<returns> A point to the array of biceps EMG samples. </returns>
	double * FileParser::GetBicepsSamples()
	{
		return bicepsdata;
	}

	///<summary>A method getting the pointer to the triceps EMG samples gathered from the Bioplux data file. </summary>
	///<returns> A point to the array of triceps EMG samples. </returns>
	double * FileParser::GetTricepsSamples()
	{
		return tricepsdata;
	}

	///<summary>A method getting the pointer to the voluntary contraction limits gathered from the voluntary contractions data file. </summary>
	///<returns> A point to the array of voluntary contraction limits. </returns>
	double * FileParser::GetVoluntaryContractionLimits()
	{
		return voluntary_contraction_limits;
	}
}