/*
 * ZazenSubSystemFactory.cpp
 *
 *  Created on: 7 Aug 2011
 *      Author: jonathan
 */

#include "ZazenSubSystemFactory.h"

#include <Windows.h>

#include <iostream>

using namespace std;

ZazenSubSystemFactory::ZazenSubSystemFactory()
{
}

ZazenSubSystemFactory::~ZazenSubSystemFactory()
{
}

ISubSystem*
ZazenSubSystemFactory::createSubSystem( const std::string& file, const std::string& type )
{
	cout << "INFO ... loading SubSystem-Type " << type << " from file " << file << endl;

	std::wstring fileWString = std::wstring( file.begin(), file.end() );

	HMODULE libHandle = LoadLibrary( fileWString.c_str() );
	if ( NULL == libHandle )
	{
		cout << "ERROR ... failed loading file for SubSystem with error: " << GetLastError() << endl;
		return 0;
	}

	ISubSystem::constructor_func constrFunc = ( ISubSystem::constructor_func ) GetProcAddress( libHandle, "createInstance" );
	if ( NULL == constrFunc )
	{
		cout << "ERROR ... failed getting ProcAddress for 'createInstance' with error: " << GetLastError() << endl;
		return 0;
	}

	ISubSystem::destructor_func destrFunc = ( ISubSystem::destructor_func ) GetProcAddress( libHandle, "deleteInstance" );
	if ( NULL == destrFunc )
	{
		cout << "ERROR ... failed getting ProcAddress for 'deleteInstance' with error: " << GetLastError() << endl;
		return 0;
	}

	ISubSystem* subSystemInstance = constrFunc( type.c_str(), NULL );
	if ( NULL == subSystemInstance )
	{
		cout << "ERROR ... constructor_func returned NULL-instance of SubSystem." << endl;
		return 0;
	}

	subSystemInstance->m_libStruct = new ISubSystem::LibStruct;
	subSystemInstance->m_libStruct->constructorFunc = constrFunc;
	subSystemInstance->m_libStruct->destructorFunc = destrFunc;
	subSystemInstance->m_libStruct->fileName = file;
	subSystemInstance->m_libStruct->libHandle = libHandle;

	// TODO set subsystem to LOADED-state

	return subSystemInstance;
}
