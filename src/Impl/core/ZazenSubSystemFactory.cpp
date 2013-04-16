/*
 * ZazenSubSystemFactory.cpp
 *
 *  Created on: 7 Aug 2011
 *      Author: jonathan
 */

#include "ZazenSubSystemFactory.h"
#include "Core.h"

#include <Windows.h>

#include <iostream>
#include <sstream>

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
	Core::getRef().getCoreLogger().logInfo() << "loading SubSystem-Type " << type << " from file " << file;

	std::wstring fileWString = std::wstring( file.begin(), file.end() );

	HMODULE libHandle = LoadLibrary( fileWString.c_str() );
	if ( NULL == libHandle )
	{
		Core::getRef().getCoreLogger().logError() << "Failed loading file for SubSystem with error: " << GetLastError();
		return NULL;
	}

	ISubSystem::constructor_func constrFunc = ( ISubSystem::constructor_func ) GetProcAddress( libHandle, "createInstance" );
	if ( NULL == constrFunc )
	{
		Core::getRef().getCoreLogger().logError() << "Failed getting ProcAddress for 'createInstance' with error: " << GetLastError();
		return NULL;
	}

	ISubSystem::destructor_func destrFunc = ( ISubSystem::destructor_func ) GetProcAddress( libHandle, "deleteInstance" );
	if ( NULL == destrFunc )
	{
		Core::getRef().getCoreLogger().logError() << "Failed getting ProcAddress for 'deleteInstance' with error: " << GetLastError();
		return 0;
	}

	ISubSystem* subSystemInstance = constrFunc( type.c_str(), Core::getInstance() );
	if ( NULL == subSystemInstance )
	{
		Core::getRef().getCoreLogger().logError( "Constructor_func returned NULL-instance of SubSystem." );
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
