//© 2019. Tyler Desplenter and Ana Luisa Trejos.

#include "Butterworth.h"
#include <cmath>
#include <conio.h>
#include <fstream>

using namespace std;
namespace WearMEBraceControlSystem
{
	class EstimationController
	{
	public:
		EstimationController();
		EstimationController(double R, double Q);
		~EstimationController();
		double * EstimationPosition(double * anglesamples, double * bicepssamples, double * tricepssamples, double * prevangle, double * prevbemg, double * prevtemg, double * prevestimatedang);
		double GetAverageAngle(double * estimated_angle);
		double * GetEstimatedVelocity(double * estimated_angle);
		double GetAverageVelocity(double * angle);
		void SetMaxMinEMGValues(double bicepsmax, double bicepsmin, double tricepsmax, double tricepsmin);
		void SetWindowSize(int size);
		void SetFilterSize(int size);
		double * GetRealAngle();
		double GetRMSPositionError(double * filteredangle, double * estimatedangle);
		void CreateFilters();
		void SetKalmanCoefficients(double r_coefficient, double q_coefficient);

		double * prevang; //a pointer to an array of previous angle samples
		double * prevestimatedang;
		double * prevbiceps; //a pointer to an array of previous biceps EMG samples
		double * prevtriceps; //a pointer to an array of previous triceps EMG samples
		double * filteredangle; //a pointer to an array of real filtered elbow angles

	private:
		double * FilterAngle(double * anglesamples, double * prevangle);
		double * FilterBicepsEMG(double * bicepssamples, double * prevbemg);
		double * FilterTricepsEMG(double * bicepssamples, double * prevtemg);
		double * FilterVelocity(double * velocitysamples, double * previous_velocity);
		double * Normalize(double * values, double min, double max);
		double * NeuralActivation(double * values);
		double * SumNeuralActivity(double * bicepsactivation, double * tricepsactivation);
		double * AmplifyNeuralActivity(double * muscleactivity);
		double * KalmanFilter(double * totalmuscleactivity, double * realtorque);
		void CreateButterworthAngleFilter();
		void CreateButterworthVelocityFilter();
		void CreateButterworthEMGFilters();
		void ShiftAngleData(double * samples, double minangle, int size);
		void OutputCompare(double * value1, double * value2);
		double CalculateOffset(double * samples);

		double R;  //The R coefficient of the Kalman filter
		double Q;  //The Q coefficient of the Kalman filter
		int muscleamplification = 10; //a constant to amplify the neural activity by
		Butterworth * anglefilter; //a pointer to the Butterworth angle filter
		Butterworth * velocityfilter; //a pointer to the Butterworth velocity filter
		Butterworth * emglowpassfilter; //a pointer to the Butterworth EMG low pass filter
		Butterworth * emghighpassfilter; //a pointer to the Butterworth EMG high pass filter
		double bicepsmax; //max value of the biceps EMG during voluntary contraction
		double bicepsmin; //min value of the biceps EMG during rest
		double tricepsmax; //max value of the triceps EMG during voluntary contraction
		double tricepsmin; //min value of the triceps EMG during rest.
		int windowsize; //the window size for which data is estimated
		int filtersize; //the number of samples the filter needs to properly filter (windowsize + filter order)
		ofstream compare; //an output file stream for comparison of two arrays of values
		bool firstiteration; //a boolean that is true only when it is the first prediction iteration
		double minimumangle; //the offset value for angle data
	};
}