/*
 * ScriptSystem.h
 *
 *  Created on: 24.07.2010
 *      Author: jonathan
 */

#ifndef SCRIPTSYSTEM_H_
#define SCRIPTSYSTEM_H_

extern "C"
{
	#include <lua/lua.h>
}

#include <string>

class ScriptSystem
{
	public:
		static bool initialize();
		static bool shutdown();
		static ScriptSystem& getInstance() { return *ScriptSystem::instance; };

		bool loadFile( const std::string& );
		bool callFunc( const std::string& );

	private:
		static ScriptSystem* instance;

		ScriptSystem();
		~ScriptSystem();

		lua_State* luaState;
};

#endif /* SCRIPTSYSTEM_H_ */
