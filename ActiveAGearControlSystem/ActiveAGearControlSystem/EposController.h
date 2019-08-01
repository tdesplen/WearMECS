//© 2019. Tyler Desplenter and Ana Luisa Trejos.

#pragma once

#include <windows.h>
#include "Definitions.h"
#include <cstdio>

namespace ActiveAGearControlSystem
{
	class EposController
	{
	public:
		EposController();
		EposController(char * pCommPort, int node_id);
		~EposController();
		long GetVelocity();
		long GetPosition();
		bool TrackVelocity(long desiredvelocity);
		bool SwitchMode(int mode);
		bool MoveTo(long position);
		inline const char& GetCOM_PORT() const { return *(this->_COM_PORT); }
		void DisableState();

		int node_id; //holds the node id for this Epos device

	private:
		void ClearErrors(HANDLE * pHandle, DWORD * pErrorCode);
		static EposController * _USEpos;
		static EposController * _PlungerEpos;
		inline char * COM_PORT() { return this->_COM_PORT; }
		inline void SetCOM_PORT(char * pCOM_PORT) { this->_COM_PORT = pCOM_PORT; }

		char * _COM_PORT; //holds the COM port used between this system and the Epos device
		HANDLE handle; //holds the handle for making function calls to the Epos device
		DWORD errorCode; //holds the error code sent by the Epos device
	};
}