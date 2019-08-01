//© 2019. Tyler Desplenter and Ana Luisa Trejos.

#include "EposController.h"
#include <ctime>
#include <iostream>

using namespace std;

namespace WearMEBraceControlSystem
{
	class ActuationController
	{
	public:
		ActuationController();
		ActuationController(double upperlimit, double lowerlimit, double velocitylimit, double timelimit);
		long MakeMotionDecision(int desiredvelocity);
		void MoveToLowerLimit();
		double RPMToDegreePerSecond(double velocity);
		double DegreePerSecondToRPM(double velocity);
		double PulseToDegree(double position);
		double DegreeToPulse(double position);
		void SetVelocity(long velocity);
		void SetPosition(long position);
		void SetController(EposController * ec);

		double timeremaining; //the amount of time remaining until the motor reaches the end limit moving at the current velocity
		double vel; //elbow velocity in degreed per second
		double spaceremaining; //the amount of space remaining, in degrees, before the system reaches the limit
		long velocity; //velocity of the motor in revolutions per minute
		long position; //position of the motor in terms of encoder pulses
		double upperpositionlimit; //upper position limit of the elbow
		double lowerpositionlimit; //lower position limit of the elbow
		double velocitylimit; //maximum velocity allowable
		double timelimit; //execution time of the control loop

	private:
		EposController * controller; //Epos controller to interact with the Epos device
		int direction; //current direction of motion
		bool started; //true if the brace is in motion

		int gear_ratio = 713;
	};
}