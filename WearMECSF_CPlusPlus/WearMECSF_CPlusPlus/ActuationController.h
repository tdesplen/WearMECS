#pragma once
//© 2019. Tyler Desplenter and Ana Luisa Trejos.

class ActuationController
{
public:
	// A function template to command the position of the actuation system.
	virtual void MoveToPosition(double position) = 0;

	// A function template to command the velocity of the actuation system.
	virtual void MoveWithVelocity(double velocity) = 0;

	// A function template to command the acceleration of the actuation system.
	virtual void MoveWithAcceleration(double acceleration) = 0;

	// A function template to command the force of the actuation system.
	virtual void MoveWithForce(double force) = 0;

	// A function template to command the torque of the actuation system.
	virtual void MoveWithTorque(double torque) = 0;
};