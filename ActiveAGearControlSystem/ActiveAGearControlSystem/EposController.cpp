//© 2019. Tyler Desplenter, Abelardo Escoto, and Ana Luisa Trejos.

#include "EposController.h"
#include <iostream>

namespace ActiveAGearControlSystem
{
	///<summary> A contructor that is used to initialize the Epos device using a COM port. </summary>
	///<param name="pCommPort"> A string containing the name of the COM port. </param>
	///<param name="n_id"> The node id of the Epos device you are connected to. </param>
	EposController::EposController(char * pCommPort, int n_id) : _COM_PORT(pCommPort)
	{
		this->handle = NULL;
		// Opening device
		handle = VCS_OpenDevice("EPOS2","MAXON SERIAL V2","USB","USB0",&errorCode);
		//handle = VCS_OpenDevice("EPOS", "MAXON_RS232", "RS232", this->COM_PORT(), &errorCode);
		node_id = n_id;

		if (handle)
		{
			DWORD  Baudrate = 1000000;
			DWORD  Timeout = 500;
			VCS_SetProtocolStackSettings(handle, Baudrate, Timeout, &errorCode);
			this->ClearErrors(&handle, &errorCode);
			VCS_SetOperationMode(handle, node_id, -2, &errorCode);
			//VCS_SetCurrentMust(handle, node_id, 0, &errorCode);
			VCS_SetEnableState(handle, node_id, &errorCode);
			/*VCS_SetDisableState(handle, node_id, &errorCode);
			WORD state = 0;
			VCS_GetState(handle, node_id, &state, &errorCode);
			std::cout << state << std::endl;*/
		}
	}

	///<summary> The default constructor sets all internal variables to default values. </summary>
	EposController::EposController()
	{
		node_id = 0;
		_COM_PORT = "";
		handle = 0;
		errorCode = 0;
	}

	///<summary> The EposController desctructor ensures that the Epos device connection is closed. </summary>
	EposController::~EposController()
	{
		DisableState();
		VCS_CloseDevice(handle, &errorCode);
	}

	///<summary> A method for clearing the errors of the Epos device. </summary>
	///<param name="pHandle"> A pointer to the handle of the Epos device. </param>
	///<param name="pErrorCode"> A pointer to the error code of the Epos device. </param>
	///<returns> N/A </returns>
	void EposController::ClearErrors(HANDLE * pHandle, DWORD * pErrorCode)
	{
		for (int i = 0; i < 3; i++)
		{
			VCS_ClearFault(*pHandle, node_id, pErrorCode);
		}
		for (int i = 0; i < 3; i++)
		{
			WORD state = 0;
			VCS_GetState(handle, node_id, &state, &errorCode);
			if (state == 0)
				VCS_SetEnableState(*pHandle, node_id, pErrorCode);
		}
	}

	///<summary> A method to read the velocity from the Epos device. </summary>
	///<returns> The velocity of the motor with units: revolutions per minute. </returns>
	long EposController::GetVelocity()
	{
		long velocity = 0;
		if (handle != NULL)
		{
			ClearErrors(&handle, &errorCode);
			VCS_GetVelocityIs(handle, node_id, &velocity, &errorCode); //returns velocity with units - RPM
			return velocity;
		}
		else
		{
			return -1;
		}
	}

	///<summary> A method to read the value of the encoder from the Epos device. </summary>
	///<returns> The position of the motor with units: encoder pulses. </returns>
	long EposController::GetPosition()
	{
		long position = 0;
		DWORD functionErrorCode = 0;
		BYTE nbOfDeviceError = 0;
		if (handle != NULL)
		{
			ClearErrors(&handle, &errorCode);
			if (VCS_GetPositionIs(handle, node_id, &position, &errorCode))
			{
				return position;
			}
			else
			{
				std::cout << errorCode << std::endl;
			}
		}
		else
		{
			return -1;
		}
	}

	///<summary> A method for commanding a velocity to the motor attached to the Epos device. </summary>
	///<param name="desiredvelocity"> The velocity which is sent to the Epos device with units: revolutions per minute. </param>
	///<returns> A boolean value which is true if the velocity command was successful. </returns>
	bool EposController::TrackVelocity(long desiredvelocity)
	{
		bool complete = false;
		if (handle != NULL)
		{
			ClearErrors(&handle, &errorCode);
			complete = VCS_SetVelocityMust(handle, node_id, desiredvelocity, &errorCode); //returns velocity with units - RPM
		}
		return complete;
	}

	// Epos Operating Modes
	//	Position - -1
	//	Velocity - -2
	//	Current  - -3

	///<summary> A method for switching the operating mode of the Epos device. </summary>
	///<param name="mode"> The mode which is represented as an integer. </param>
	///<returns> A boolean value which is true if the mode switch was successful. </returns>
	bool EposController::SwitchMode(int mode)
	{
		bool complete = false;
		if (handle != NULL)
		{
			ClearErrors(&handle, &errorCode);
			if (mode < 0 && mode > -4) //make sure we can only choose modes that are needed
			{
				complete = VCS_SetOperationMode(handle, node_id, mode, &errorCode); //returns velocity with units - RPM
			}
		}
		return complete;
	}

	///<summary> A method for commanding the Epos device to move the motor to a specified position. </summary>
	///<param name="position"> The desired position to move to. </param>
	///<returns> A boolean value which is true if the position command was successful. </returns>
	bool EposController::MoveTo(long position)
	{
		bool complete = false;
		if (handle != NULL)
		{
			if (position <= 500000) //do not command an absolute position that is larger than the maximum absolute position, **TODO** determine the maximum position
			{
				ClearErrors(&handle, &errorCode);
				complete = VCS_MoveToPosition(handle, node_id, position, true, true, &errorCode);
			}
		}
		return complete;
	}

	void EposController::DisableState()
	{
		VCS_SetDisableState(handle, node_id, &errorCode);
		WORD state = 0;
		VCS_GetState(handle, node_id, &state, &errorCode);
		std::cout << state << std::endl;
	}
}