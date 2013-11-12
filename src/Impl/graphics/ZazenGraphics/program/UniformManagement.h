#ifndef _UNIFORM_MANAGEMENT_H_
#define _UNIFORM_MANAGEMENT_H_

#include "UniformBlock.h"

#include <map>
#include <string>

class Program;

class UniformManagement
{
	public:
		static bool freeAllBlocks();

		static UniformBlock* getBlock( const std::string& );

		static bool initUniforms( Program* );

	private:
		static std::map<std::string, UniformBlock*> m_uniformBlocks;

		static void loadUniformsActive( Program*, GLint );
		static bool loadUniformBlocks( Program*, GLint );
		static bool loadSubroutines( Program*, GLenum );

};

#endif /* _UNIFORM_MANAGEMENT_H_ */
