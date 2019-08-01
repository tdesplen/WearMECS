//© 2019. Tyler Desplenter and Ana Luisa Trejos.

#pragma once

class EstimationController
{
public:
	// A function template for estimating the position(s) of a joint, componenet, or system.
	virtual double* EstimatePosition(double **parameters, int rows, int columns) = 0;

	// A function template for estimating the velocity of a joint, component, or system.
	virtual double* EstimateVelocity(double **parameters, int rows, int columns) = 0;

	// A function template for estimating the acceleration(s) of a joint, component, or system.
	virtual double* EstimateAcceleration(double **parameters, int rows, int columns) = 0;

	// A function template for estimating a force(s) of a joint, component, or system.
	virtual double*	EstimateForce(double **parameters, int rows, int columns) = 0;

	// A function template for estimating a torque(s) of a joint, component, or system.
	virtual double* EstimateTorque(double **parameters, int rows, int columns) = 0;
};
