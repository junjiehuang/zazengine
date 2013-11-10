#ifndef _SCRIPTSYSTEM_H_
#define _SCRIPTSYSTEM_H_

#include <lua/lua.hpp>

#include <string>

class ScriptSystem
{
	public:
		static bool initialize();
		static bool shutdown();
		static ScriptSystem& getInstance() { return *ScriptSystem::instance; };

		bool loadFile( const std::string& );
		bool callFunc( const std::string& );

		lua_State* getLuaState() { return this->luaState; };

	private:
		static ScriptSystem* instance;

		ScriptSystem();
		~ScriptSystem();

		lua_State* luaState;
};

#endif /* _SCRIPTSYSTEM_H_ */
