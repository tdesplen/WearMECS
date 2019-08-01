//© 2019. Tyler Desplenter and Ana Luisa Trejos.

#pragma once

class TaskController
{
public:
	//A function template for any motion task that needs to be orchestrated by the control system.
	virtual void MotionTask(double **parameters, int rows, int columns) = 0;
};
