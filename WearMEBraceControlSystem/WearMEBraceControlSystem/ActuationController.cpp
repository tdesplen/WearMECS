//© 2019. Tyler Desplenter and Ana Luisa Trejos.

#include "ActuationController.h"

namespace WearMEBraceControlSystem
{
	///<summary> The default constructor of the MotionTracking object sets all internal variables to default values. </summary>
	ActuationController::ActuationController()
	{
		controller = NULL;
		upperpositionlimit = 0;
		lowerpositionlimit = 0;
		velocitylimit = 0;
		timelimit = 0;
		timeremaining = 0;
		direction = 0;
	}

	///<summary> A constructor that established the limits of the motion in order to make motion decisions. </summary>
	///<param name="upper_limit"> The elbow upper position limit with units: degrees. </param>
	///<param name="lower_limit"> The elbow lower position limit with units: degrees. </param>
	///<param name="velocity_limit"> The elbow velocity limit with units: degrees/second. </param>
	///<param name="time_limit"> The maximum control loop time limit with units: seconds. </param>
	///<param name="elbow_direction"> The direction of the elbow motion: 1 for flexion and -1 for extension. </param>
	ActuationController::ActuationController(double upper_limit, double lower_limit, double velocity_limit, double time_limit)
	{
		upperpositionlimit = upper_limit;
		lowerpositionlimit = lower_limit;
		velocitylimit = velocity_limit;
		timelimit = time_limit;
		velocity = 0;
		position = 0;
		started = false;
	}

	///<summary> A method that determines whether to send the desired velocity or zero velocity to the Epos device. </summary>
	///<param name="desiredvelocity"> The predicted velocity with units: revolutions per minute. </param>
	///<returns> The velocity to be sent to the Epos device with units: revolutions per minute. </returns>
	long ActuationController::MakeMotionDecision(int desiredvelocity)
	{
		double pos;
		vel = 0;
		timeremaining = 0;
		spaceremaining = 0;

		//Convert velocity and position to degree/s and degree
		vel = RPMToDegreePerSecond(desiredvelocity);
		pos = PulseToDegree(position);
		//pos = (double) position; -> for fake position testing

		if ((pos > upperpositionlimit && vel > 0) || (pos < lowerpositionlimit && vel < 0))
			return 0;

		if (vel > 0 && pos < upperpositionlimit) //Flexion
		{
			spaceremaining = upperpositionlimit - pos;
		}
		else if (vel < 0 && pos > lowerpositionlimit) //Extension
		{
			spaceremaining = pos - lowerpositionlimit;
		}
		else //No Movement
		{
			spaceremaining = 0;
		}

		timeremaining = abs(spaceremaining / vel); // degree / (degree/s) leaves seconds

		if (timeremaining <= timelimit) // if there is not enough time, stop motion (velocity = 0)
		{
			return 0;
		}
		else
		{
			if (vel > velocitylimit)
				return DegreePerSecondToRPM(velocitylimit);
			else if (vel < -velocitylimit)
				return DegreePerSecondToRPM(-velocitylimit);
			else
				return desiredvelocity;
		}
	}

	///<summary> A method for commanding the Epos to return the motor to the lower position limit. </summary>
	///<returns> N/A </returns>
	void ActuationController::MoveToLowerLimit()
	{
		//controller->MoveTo(lowerpositionlimit);
	}

	///<summary> A method for converting velocity of the motor to velocity of the elbow. </summary>
	///<param name="vel"> Velocity of the motor with units: revolutions per minute. </param>
	///<returns> Velocity of the elbow with units: degrees/second.  </returns>
	double ActuationController::RPMToDegreePerSecond(double vel)
	{
		double velocity = 0;
		velocity = vel * 6; //convert velocity from RPM to degree/second
		velocity = velocity / gear_ratio; //divide motor velocity by total gear ratio to get output velocity in degree/second
		return velocity;
	}

	///<summary> A method for converting velocity of the elbow to velocity of the motor. </summary>
	///<param name="vel"> Velocity of the elbow with units: degrees/second. </param>
	///<returns> Velocity of the motor with units: revolutions per minute.</returns>
	double ActuationController::DegreePerSecondToRPM(double vel)
	{
		double velocity = 0;
		velocity = vel * 0.1667; // degree/second to RPM
		velocity = velocity * gear_ratio; //multiply to attain the RPM required at the motor
		return velocity;
	}

	///<summary> A method for converting encoder pulses to degrees of elbow motion. </summary>
	///<param name="pos"> The position of the motor with units: encoder pulses. </param>
	///<returns> The position of the elbow as seen by the encoder with units: degrees. </returns>
	double ActuationController::PulseToDegree(double pos)
	{
		double position = 0;
		double encoder_counts = 512;
		double encoder_ratio = 4 * encoder_counts / 360; // 4 * # of encoder counts / revolution
		double elbow_angle_degrees = ((pos / encoder_ratio) / gear_ratio);
		return elbow_angle_degrees;
	}

	///<summary> A method for converting degrees of elbow motion to encoder pulses. </summary>
	///<param name="pos"> The position of the motor with units: encoder pulses. </param>
	///<returns> The position of the elbow as seen by the encoder with units: degrees. </returns>
	double ActuationController::DegreeToPulse(double pos)
	{
		double position = 0;
		double encoder_counts = 512;
		double encoder_ratio = 4 * encoder_counts / 360; // 4 * # of encoder counts / revolution
		double motor_rotation = ((pos * encoder_ratio) * gear_ratio);
		return motor_rotation;
	}

	///<summary> A method to set the internal velocity variable. </summary>
	///<param name="motor_velocity"> The velocity of the motor with units: revolutions per minute. </param>
	///<returns> N/A </returns>
	void ActuationController::SetVelocity(long motor_velocity)
	{
		velocity = motor_velocity;
	}

	///<summary> A method to set the internal position variable. </summary>
	///<param name="motor_position"> The position of the motor with units: encoder pulses. </param>
	///<returns> N/A </returns>
	void ActuationController::SetPosition(long motor_position)
	{
		position = motor_position;
	}

	///<summary> A method to set the EposController variable. </summary>
	///<param name="epos_controller"> A pointer to an EposController object. </param>
	///<returns> N/A </returns>
	void ActuationController::SetController(EposController * epos_controller)
	{
		controller = epos_controller;
	}
}