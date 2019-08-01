//© 2019. Tyler Desplenter and Ana Luisa Trejos.

#include "EstimationController.h"
#include <ctime>
#include <cmath>
#include <math.h>

namespace WearMEBraceControlSystem
{
	///<summary>The default contructor initialzes internal variables to defaults.</summary>
	EstimationController::EstimationController()
	{
		windowsize = 0;
		firstiteration = true;
		filteredangle = 0;
	}

	///<summary>The destructor ensure the compare output stream is closed if it has been opened. </summary>
	EstimationController::~EstimationController()
	{
		if (compare.is_open())
			compare.close();
	}

	///<summary>A constructor that initializes the model with the appropriate Kalman filter values. </summary>
	///<param name="r_coefficient"> The R coefficient of the Kalman filter. </param>
	///<param name="q_coefficient"> The Q coefficient of the Kalman filter. </param>
	EstimationController::EstimationController(double r_coefficient, double q_coefficient)
	{
		R = r_coefficient;
		Q = q_coefficient;
		windowsize = 0;
		compare.open("angles-controlsystem.txt", ios::in | ios::trunc);
		firstiteration = true;
		filteredangle = 0;
	}

	///<summary> A method that initializes the creation of all of the Butterworth filters used in the model. </summary>
	///<returns> N/A </returns>
	void EstimationController::CreateFilters()
	{
		//Create Filters
		CreateButterworthAngleFilter();
		CreateButterworthVelocityFilter();
		CreateButterworthEMGFilters();
	}

	///<summary> A method for estimating the angle, based on the previous angle, biceps EMG and triceps EMG data, over one window worth of data. </summary>
	///<param name="anglesamples"> A pointer to a window worth of angle data. </param>
	///<param name="bicepssamples"> A pointer to a window worth of biceps EMG data. </param>
	///<param name="tricepssamples"> A pointer to a window worth of triceps EMG data. </param>
	///<param name="previous_angle"> A pointer to an array, with length of the filter order, of previous angle samples. </param>
	///<param name="previous_biceps_emg"> A pointer to an array, with length of the filter order, of previous biceps EMG samples. </param>
	///<param name="previous_triceps_emg"> A pointer to an array, with length of the filter order, of previous triceps EMG samples. </param>
	///<returns> A pointer to the array of estimated angles of the elbow with units: degrees. </returns>
	double * EstimationController::EstimationPosition(double * anglesamples, double * bicepssamples, double * tricepssamples, double * previous_angle, double * previous_biceps_emg, double * previous_triceps_emg, double * previous_estimated_angle)
	{
		//Variable declaration
		double * filteredangleradians;
		double * filteredbicepsemg;
		double * filteredtricepsemg;
		double * filteredvelocity;
		double * normalizedangle;
		double * normalizedbicepsemg;
		double * normalizedtricepsemg;
		double * bicepsactivation;
		double * tricepsactivation;
		double * totalmuscleactivity;
		double * realvelocity;
		double * estimatedvelocity;
		double * estimatedangle;
		double * filteredestimatedangle;
		double * realcommandvelocity;
		double * commandvelocity;
		int shifterror = 0;
		int filterorder = filtersize - windowsize;

		filteredangle = FilterAngle(anglesamples, previous_angle);

		prevang = new double[filterorder];
		for (int i = 0; i < filterorder; i++)
			prevang[i] = filteredangle[windowsize - filterorder + i];

		//Filter biceps and triceps EMG signals
		filteredbicepsemg = FilterBicepsEMG(bicepssamples, previous_biceps_emg);
		filteredtricepsemg = FilterTricepsEMG(tricepssamples, previous_triceps_emg);
		prevbiceps = new double[filtersize - windowsize];
		for (int i = 0; i < filtersize - windowsize; i++)
			prevbiceps[i] = filteredbicepsemg[windowsize - (filtersize - windowsize) + i];
		prevtriceps = new double[filtersize - windowsize];
		for (int i = 0; i < filtersize - windowsize; i++)
			prevtriceps[i] = filteredtricepsemg[windowsize - (filtersize - windowsize) + i];

		//Normalize signals
		normalizedbicepsemg = Normalize(filteredbicepsemg, bicepsmin, bicepsmax); //Set min and max values with main controller
		normalizedtricepsemg = Normalize(filteredtricepsemg, tricepsmin, tricepsmax); //Set min and max values with main controller

		//Calculate Neural Activation
		bicepsactivation = NeuralActivation(normalizedbicepsemg);
		tricepsactivation = NeuralActivation(normalizedtricepsemg);

		//Sum the neural activity
		totalmuscleactivity = SumNeuralActivity(bicepsactivation, tricepsactivation);

		//This is used in Stacey's model, otherwise neural activity is not between 0-2
		for (int i = 0; i < windowsize; i++)
			if (totalmuscleactivity[i] < 0)
				totalmuscleactivity[i] = 0;

		//Amplify the neural activity
		totalmuscleactivity = AmplifyNeuralActivity(totalmuscleactivity);

		//Kalman Filter to estimate torque
		estimatedangle = KalmanFilter(totalmuscleactivity, filteredangle);

		double * temp = new double[filtersize];
		for (int i = 0; i < filtersize; i++)
		{
			if (i < filtersize - windowsize)
				temp[i] = previous_estimated_angle[i];
			else
				temp[i] = estimatedangle[i - (filtersize - windowsize)];
		}

		filteredestimatedangle = FilterAngle(temp, previous_estimated_angle);

		prevestimatedang = new double[filtersize - windowsize];
		for (int i = 0; i < filtersize - windowsize; i++)
			prevestimatedang[i] = filteredestimatedangle[windowsize - (filtersize - windowsize) + i];

		OutputCompare(filteredangle, estimatedangle);

		return estimatedangle;

	}

	double EstimationController::GetAverageAngle(double * estimated_angle)
	{
		double average = 0;
		for (int i = 0; i < windowsize; i++)
			average += estimated_angle[i];
		average /= ((double)windowsize);
		return average;
	}

	double * EstimationController::GetEstimatedVelocity(double * estimated_angle)
	{
		double * velocity = new double[windowsize];
		double timestep = 0.001; // change this later based on sampling frequency
		for (int i = 0; i < windowsize - 1; i++)
			velocity[i] = (estimated_angle[i + 1] - estimated_angle[i]) / timestep;
		velocity[windowsize - 1] = velocity[windowsize - 2];
		return velocity;

	}

	///<summary> A method for calculating the average velocity of the elbow during a window. </summary>
	///<param name="angle"> A pointer to an array of elbow angle values with units: degrees. </param>
	///<returns> The average velocity of the elbow with units: degrees/second. </returns>
	double EstimationController::GetAverageVelocity(double * angle)
	{
		double * velocity = new double[windowsize];
		double timestep = 0.001; // change this later based on sampling frequency
		for (int i = 0; i < windowsize - 1; i++)
			velocity[i] = (angle[i + 1] - angle[i]) / timestep;
		double average = 0;
		for (int i = 0; i < windowsize - 1; i++)
			average += velocity[i];
		average /= ((double)windowsize - 1);
		return average;
	}

	///<summary> A method retrieving the real angle as calculated by the model. </summary>
	///<returns>A pointer to an array of real angle values with units: degrees. </returns>
	double * EstimationController::GetRealAngle()
	{
		return filteredangle;
	}

	///<summary> A method calculating the root-mean squared error between real and estimated position over one window. </summary>
	///<param name="filteredangle"> A pointer to an array of real elbow angle values with units: degrees. </param>
	///<param name="estimatedangle"> A pointer to an array of estimated elbow angle values with units: degrees. </param>
	///<returns> The root mean squared error with units: degrees. </returns>
	double EstimationController::GetRMSPositionError(double * filteredangle, double * estimatedangle)
	{
		double * temp = new double[windowsize];
		double error = 0;
		for (int i = 0; i < windowsize; i++)
		{
			temp[i] = (filteredangle[i] - estimatedangle[i]);
			temp[i] = powf(temp[i], 2.0);
			//temp[i] = abs(temp[i]);
			error += temp[i];
		}
		error /= windowsize;
		error = sqrtf(error);
		return error;
	}

	///<summary> A method calculating the offset of the angle samples to shift the data values to zero. </summary>
	///<param name="samples"> A pointer to an array of angle samples. </param>
	///<returns> The offset value. </returns>
	double EstimationController::CalculateOffset(double * samples)
	{
		double offset = 0;
		for (int i = 2; i < 12; i++)
			offset += samples[i];
		offset /= 10;
		return offset;
	}

	///<summary> A method for using a Butterworth filter to filter the angle samples. </summary>
	///<param name="angle_samples"> A pointer to an array of angle samples. </param>
	///<param name="previous_angle"> A pointer to an array, with length of the filter order, of previous angle samples. </param>
	///<returns> A pointer to an array of filtered angle data. </returns>
	double * EstimationController::FilterAngle(double * angle_samples, double * previous_angle)
	{
		double * filteredangle = anglefilter->process(angle_samples, previous_angle, windowsize);
		return filteredangle;
	}

	///<summary> A method for using a Butterworth filter to filter the biceps EMG samples. </summary>
	///<param name="biceps_samples"> A pointer to an array of biceps EMG samples. </param>
	///<param name="previous_biceps_samples"> A pointer to an array, with length of the filter order, of previous biceps EMG samples. </param>
	///<returns> A pointer to an array of filtered biceps EMG data. </returns>
	double * EstimationController::FilterBicepsEMG(double * biceps_samples, double * previous_biceps_samples)
	{
		double * highpassedemg = emghighpassfilter->process(biceps_samples, previous_biceps_samples, windowsize);
		double * temp = new double[filtersize];
		int order = filtersize - windowsize;
		for (int i = 0; i < filtersize; i++)
			if (i < order)
				temp[i] = previous_biceps_samples[i];
			else
				temp[i] = highpassedemg[i - order];
		double * filteredbicepsemg = emglowpassfilter->process(temp, previous_biceps_samples, windowsize);
		return filteredbicepsemg;
	}

	///<summary> A method for using a Butterworth filter to filter the triceps EMG samples. </summary>
	///<param name="triceps_samples"> A pointer to an array of triceps EMG samples. </param>
	///<param name="previous_triceps_samples"> A pointer to an array, with length of the filter order, of previous triceps EMG samples. </param>
	///<returns> A pointer to an array of filtered triceps EMG data. </returns>
	double * EstimationController::FilterTricepsEMG(double * triceps_samples, double * previous_triceps_samples)
	{
		double * highpassedemg = emghighpassfilter->process(triceps_samples, previous_triceps_samples, windowsize);
		double * temp = new double[filtersize];
		int order = filtersize - windowsize;
		for (int i = 0; i < filtersize; i++)
			if (i < order)
				temp[i] = previous_triceps_samples[i];
			else
				temp[i] = highpassedemg[i - order];
		double * filteredtricepsemg = emglowpassfilter->process(temp, previous_triceps_samples, windowsize);
		return filteredtricepsemg;
	}

	///<summary> A method for using a Butterworth filter to filter the velocity samples. </summary>
	///<param name="velocity_samples"> A pointer to an array of velocity samples. </param>
	///<param name="previous_velocity_samples"> A pointer to an array, with length of the filter order, of previous velocity samples. </param>
	///<returns> A pointer to an array of filtered velocity data. </returns>
	double * EstimationController::FilterVelocity(double * velocity_samples, double * previous_velocity_samples)
	{
		double * filteredvelocity = 0;
		velocityfilter->process(velocity_samples, previous_velocity_samples, windowsize);
		return filteredvelocity;
	}

	///<summary> A general purpose normalization function. </summary>
	///<param name="values"> A pointer to an array of data values. </param>
	///<param name="min"> The minimum value of the data values. </param>
	///<param name="max"> The maximum value of the data values. </param>
	///<returns> A pointer to an array of normalized values. </returns>
	double * EstimationController::Normalize(double * values, double min, double max)
	{
		double * normalizedvalues = new double[windowsize];
		for (int i = 0; i < windowsize; i++)
		{
			normalizedvalues[i] = (values[i] - min) / (max - min);
		}
		return normalizedvalues;
	}

	///<summary> A method used to compute the neural activation of a muscle. </summary>
	///<param name="values"> A pointer to an array of EMG data values. </param>
	///<returns> A pointer to an array of neural activation values. </returns>
	double * EstimationController::NeuralActivation(double * values)
	{
		//Zajac Model
		double gamma1 = -0.99;
		double gamma2 = -0.79;
		double beta1 = gamma1 + gamma2;
		double beta2 = gamma1 * gamma2;
		double alpha = 1 + beta1 + beta2;
		double * neuralactivation = new double[windowsize];

		neuralactivation[0] = (double)(alpha*values[0] - beta1 * 0 - beta2 * 0);
		neuralactivation[1] = (double)(alpha*values[1] - beta1 * neuralactivation[0] - beta2 * 0);
		neuralactivation[2] = (double)(alpha*values[2] - beta1 * neuralactivation[1] - beta2 * neuralactivation[0]);

		for (int i = 3; i < windowsize; i++)
		{
			neuralactivation[i] = (double)(alpha*values[i] - beta1 * neuralactivation[i - 1] - beta2 * neuralactivation[i - 2]);
		}

		return neuralactivation;
	}

	///<summary> A method for the summation of the neural activations of the biceps and triceps muscles. </summary>
	///<param name="biceps_activation"> A pointer to an array of neural activation values of the biceps muscle. </param>
	///<param name="triceps_activation"> A pointer to an array of neural activation values of the triceps muscle. </param>
	///<returns> A pointer to an array of summed neural activation values. </returns>
	double * EstimationController::SumNeuralActivity(double * biceps_activation, double * triceps_activation)
	{
		double * totalmuscleactivity = new double[windowsize];
		for (int i = 0; i < windowsize; i++)
			totalmuscleactivity[i] = biceps_activation[i] + triceps_activation[i];
		return totalmuscleactivity;
	}

	///<summary> A method used to amplify the neural activation values by a constant factor. </summary>
	///<param name="neural_activitation"> A pointer to an array of EMG data values. </param>
	///<returns> A pointer to an array of amplified neural activation values. </returns>
	double * EstimationController::AmplifyNeuralActivity(double * neural_activitation)
	{
		double * amplified_activtion = new double[windowsize];
		for (int i = 0; i < windowsize; i++)
			amplified_activtion[i] = abs(neural_activitation[i] * 10);
		return amplified_activtion;
	}

	///<summary> A Kalman filter designed to estimate the elbow angle based on previous elbow angle position and the summed neural activation of the biceps and triceps muscles. </summary>
	///<param name="neural_activation"> A pointer to an array of neural activation values. </param>
	///<param name="filtered_angle"> A pointer to an array of filtered angle values. </param>
	///<returns> A pointer to an array of estimated elbow angle values. </returns>
	double * EstimationController::KalmanFilter(double * neural_activation, double * filtered_angle)
	{
		double * estimatedangle = NULL;
		double * xhat = new double[windowsize];
		double * phat = new double[windowsize];
		double * K = new double[windowsize];
		xhat[0] = filtered_angle[0];
		phat[0] = 0;
		xhat[1] = filtered_angle[1]; //totalmuscleactivity[0]
		phat[1] = 1;
		for (int i = 2; i < windowsize; i++)
		{
			xhat[i] = (xhat[i - 1] + filtered_angle[i - 2]) / 2;
			phat[i] = phat[i - 1] + Q;
			K[i] = phat[i] / (phat[i] + R);
			xhat[i] = xhat[i] + (K[i] * (neural_activation[i] - xhat[i]));
			phat[i] = (1 - K[i]) * phat[i];
		}
		estimatedangle = xhat;
		return estimatedangle;
	}

	///<summary> A method for creating a Butterworth filter, based on the coefficents of the discrete filter, for filtering angle data. </summary>
	///<returns> N/A </returns>
	void EstimationController::CreateButterworthAngleFilter()
	{
		//Create Butterworth Low Pass Filter for angle data - 10Hz, order 2
		int coefficients = 3;
		vector<double> B(coefficients);
		vector<double> A(coefficients);

		B[0] = 0.000944691843840;
		B[1] = 0.001889383687680;
		B[2] = 0.000944691843840;
		A[0] = 1.000000000000000;
		A[1] = -1.911197067426073;
		A[2] = 0.914975834801434;

		anglefilter = new Butterworth(B, A);
	}

	///<summary> A method for creating a Butterworth filter, based on the coefficents of the discrete filter, for filtering velocity data. </summary>
	///<returns> N/A </returns>
	void EstimationController::CreateButterworthVelocityFilter()
	{
		//Create Butterworth Low Pass Filter for angle data - 2Hz, order 2
		int coefficients = 3;
		vector<double> B(coefficients);
		vector<double> A(coefficients);

		B[0] = 0.000039130205399;
		B[1] = 0.000078260410798;
		B[2] = 0.000039130205399;
		A[0] = 1.000000000000000;
		A[1] = -1.982228929792529;
		A[2] = 0.982385450614126;

		velocityfilter = new Butterworth(B, A);
	}

	///<summary> A method for creating a Butterworth filter, based on the coefficents of the discrete filter, for filtering EMG data. </summary>
	///<returns> N/A </returns>
	void EstimationController::CreateButterworthEMGFilters()
	{
		int coefficients = 3;
		vector<double> B1(coefficients);
		vector<double> A1(coefficients);

		B1[0] = 0.956543225556877;
		B1[1] = -1.913086451113754;
		B1[2] = 0.956543225556877;

		A1[0] = 1.000000000000000;
		A1[1] = -1.911197067426073;
		A1[2] = 0.914975834801434;

		//Create Butterworth High Pass filter for EMG data - 10Hz, order 2
		emghighpassfilter = new Butterworth(B1, A1);

		coefficients = 3;
		vector<double> B2(coefficients);
		vector<double> A2(coefficients);

		B2[0] = 0.391335772501769;
		B2[1] = 0.782671545003537;
		B2[2] = 0.391335772501769;

		A2[0] = 1.000000000000000;
		A2[1] = 0.369527377351241;
		A2[2] = 0.195815712655833;

		//Create Butterworth Low Pass filter for EMG data - 300Hz, order 2
		emglowpassfilter = new Butterworth(B2, A2);
	}

	///<summary> A method for setting the internal variables related to maximum and minimum EMG values during voluntary contaction. </summary>
	///<param name="biceps_max"> The maximum biceps EMG value during voluntary contaction. </param>
	///<param name="biceps_min"> The minimum biceps EMG value during rest. </param>
	///<param name="triceps_max"> The maximum triceps EMG value during voluntary contaction. </param>
	///<param name="triceps_min"> The minimum triceps EMG value during rest. </param>
	///<returns> N/A </returns>
	void EstimationController::SetMaxMinEMGValues(double biceps_max, double biceps_min, double triceps_max, double triceps_min)
	{
		bicepsmax = biceps_max;
		bicepsmin = biceps_min;
		tricepsmax = triceps_max;
		tricepsmin = triceps_min;
	}

	///<summary> A method for shifting the angle data by an offset. </summary>
	///<param name="angle_samples"> A pointer to an array of angle samples. </param>
	///<param name="offset"> The angle offset. </param>
	///<param name="number_of_samples"> The number of samples within the array that need to be shifted by the offset. </param>
	///<returns> N/A </returns>
	void EstimationController::ShiftAngleData(double * angle_samples, double offset, int number_of_samples)
	{
		for (int i = 0; i < number_of_samples; i++)
			angle_samples[i] -= offset;
	}

	///<summary> A method for writing two sets of data to a file for comparison. </summary>
	///<param name="value1"> A pointer to an array of values. </param>
	///<param name="value2"> A pointer to an array of values. </param>
	///<returns> N/A </returns>
	void EstimationController::OutputCompare(double * value1, double * value2)
	{
		for (int i = 0; i < windowsize; i++)
			compare << value1[i] << "," << value2[i] << endl;
			//compare << value1[i] << "\t\t" << value2[i] << endl;
		//compare <<  "  --------------------------------------------------------------------------" << endl;
	}

	///<summary> A method for setting the value of the 'windowsize' variable. </summary>
	///<param name="size"> The size of the window. </param>
	///<returns> N/A </returns>
	void EstimationController::SetWindowSize(int size)
	{
		windowsize = size;
	}

	///<summary> A method for setting the value of the 'filtersize' variable. </summary>
	///<param name="size"> The size of the filter. </param>
	///<returns> N/A </returns>
	void EstimationController::SetFilterSize(int size)
	{
		filtersize = size;
	}

	///<summary> A method for setting the Kalman filter coefficients. </summary>
	///<param name="r_coefficient"> The R coefficient of the Kalman filter. </param>
	///<param name="q_coefficient"> The Q coefficient of the Kalman filter. </param>
	///<returns> N/A </returns>
	void EstimationController::SetKalmanCoefficients(double r_coefficient, double q_coefficient)
	{
		R = r_coefficient;
		Q = q_coefficient;
	}
}