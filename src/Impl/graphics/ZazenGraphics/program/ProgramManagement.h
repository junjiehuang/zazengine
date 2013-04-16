/*
 * UniformBlock.h
 *
 *  Created on: 16. April 2013
 *      Author: jonathan
 */

#ifndef PROGRAMMANAGEMENT_H_
#define PROGRAMMANAGEMENT_H_

#include "Program.h"
#include "Shader.h"

#include <core/ICore.h>

#include <boost/filesystem.hpp>

#include <map>
#include <string>

class ProgramManagement
{
	public:
		static bool init( const boost::filesystem::path& );
		static bool freeAll();

		static Program* get( const std::string& );

	private:
		static boost::filesystem::path m_path;

		static std::map<std::string, Shader*> m_shaders;
		static std::map<std::string, Program*> m_programs;

		static bool parseShaders( TiXmlElement*, Program* );
		static bool parseFragDataLocations( TiXmlElement*, Program* );
		static bool parseAttribLocations( TiXmlElement*, Program* );
		static bool parseBoundUniforms( TiXmlElement*, Program* );

};

#endif /* UNIFORMBLOCKMANAGEMENT_H_ */
