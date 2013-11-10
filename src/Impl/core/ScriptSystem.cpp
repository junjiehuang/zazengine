#include "ScriptSystem.h"

#include <iostream>

using namespace std;
//using namespace luabind;

ScriptSystem* ScriptSystem::instance = 0;

bool
ScriptSystem::initialize()
{
	if ( 0 == ScriptSystem::instance )
	{
		new ScriptSystem();

		//ScriptSystem::instance->luaState = luaL_newstate();
		//open ( ScriptSystem::instance->luaState );
		//luaL_openlibs ( ScriptSystem::instance->luaState );
	}

	return true;
}

bool
ScriptSystem::shutdown()
{
	if ( ScriptSystem::instance )
	{
		//lua_close( ScriptSystem::instance->luaState );

		delete ScriptSystem::instance;
	}

	return true;
}

ScriptSystem::ScriptSystem()
{
	ScriptSystem::instance = this;

	this->luaState = 0;
}

ScriptSystem::~ScriptSystem()
{
	ScriptSystem::instance = 0;
}

bool
ScriptSystem::loadFile( const std::string& fileName )
{
	/*
	if ( 1 == luaL_dofile( this->luaState, fileName.c_str() ) )
	{
		cout << "ERROR ... failed loading script file \"" << fileName << "\"" << endl;
		return false;
	}

	cout << "SCRIPT: sucessfully loaded ScriptFile '" << fileName << "'" << endl;
	*/

	return true;
}

bool
ScriptSystem::callFunc( const std::string& funcName )
{
	/*
	try
	{
		call_function<int>( this->luaState, funcName.c_str() );
	} catch ( luabind::error e )
	{
		cout << "ERROR ... coulnd't call Script-Function '" << funcName << "' - reason: " << e.what() << endl;
		return false;
	}
	*/
	return true;
}
