//© 2019. Tyler Desplenter and Ana Luisa Trejos.

#include <vector>
using namespace std;

namespace ActiveAGearControlSystem
{
	class Butterworth
	{
	public:
		Butterworth();
		Butterworth(vector<double> b, vector<double> a);
		vector<double> process(vector<double> x, vector<double> prev, int samplelength);
		double * process(double * x, double * prev, int samplelength);
	private:
		vector<double> B; //nominator coefficient
		vector<double> A; //denominator coefficients
		int order; //filter order
	};
}