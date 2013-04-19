/*
 * ProgramManagement.cpp
 *
 *  Created on: 16. April 2013
 *      Author: jonathan
 */

#include "ProgramManagement.h"

#include "UniformBlockManagement.h"

#include "../ZazenGraphics.h"

#include <GL/glew.h>

#include <iostream>

using namespace std;
using namespace boost;

map<string, Shader*> ProgramManagement::m_shaders;
map<string, Program*> ProgramManagement::m_programs;

boost::filesystem::path ProgramManagement::m_path;

bool
ProgramManagement::init( const boost::filesystem::path& path )
{
	ProgramManagement::m_path = path;

	string fullFileName = ProgramManagement::m_path.generic_string() + "programConfig.xml";

	TiXmlDocument doc( fullFileName.c_str() );

	if ( false == doc.LoadFile() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "could not load file " << fullFileName << " - reason = " << doc.ErrorDesc() << " at row = " << doc.ErrorRow() << " col = " << doc.ErrorCol();
		return false;
	}

	TiXmlElement* rootNode = doc.FirstChildElement( "programs" );
	if ( 0 == rootNode )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "root-node \"programs\" in " << fullFileName << " not found";
		return false;
	}

	for ( TiXmlElement* programNode = rootNode->FirstChildElement(); programNode != 0; programNode = programNode->NextSiblingElement() )
	{
		const char* str = programNode->Value();
		if ( NULL == str )
		{
			continue;
		}

		if ( 0 == strcmp( str, "program" ) )
		{
			std::string name;

			str = programNode->Attribute( "name" );
			if ( 0 == str )
			{
				ZazenGraphics::getInstance().getLogger().logWarning( "No name for program - will be ignored" );
				continue;
			}
			else
			{
				name = str;
			}

			Program* program = Program::createProgram( name );
			if ( NULL == program )
			{
				return false;
			}

			if ( false == ProgramManagement::parseShaders( programNode, program ) )
			{
				delete program;
				return false;
			}

			if ( false == ProgramManagement::parseFragDataLocations( programNode, program ) )
			{
				delete program;
				return false;
			}

			if ( false == ProgramManagement::parseAttribLocations( programNode, program ) )
			{
				delete program;
				return false;
			}

			if ( false == program->link() )
			{
				delete program;
				return false;
			}

			if ( false == ProgramManagement::parseBoundUniforms( programNode, program ) )
			{
				delete program;
				return false;
			}

			if ( false == UniformBlockManagement::initUniformBlocks( program ) )
			{
				delete program;
				return false;
			}

			ProgramManagement::m_programs[ name ] = program;
		}
	}

	return true;
}

bool
ProgramManagement::parseShaders( TiXmlElement* programNode, Program* program )
{
	TiXmlElement* shadersNode = programNode->FirstChildElement( "shaders" );
	if ( 0 == shadersNode )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "no shaders defined for program ";
		return false;
	}

	for ( TiXmlElement* shaderNode = shadersNode->FirstChildElement(); shaderNode != 0; shaderNode = shaderNode->NextSiblingElement() )
	{
		const char* str = shaderNode->Value();
		if ( NULL == str )
		{
			continue;
		}

		if ( 0 == strcmp( str, "shader" ) )
		{
			Shader* shader = NULL;
			std::string file = ProgramManagement::m_path.generic_string();
			Shader::ShaderType shaderType = Shader::VERTEX_SHADER;
			
			str = shaderNode->Attribute( "type" );
			if ( 0 == str )
			{
				ZazenGraphics::getInstance().getLogger().logWarning( "No type for shader - will be ignored" );
				continue;
			}
			else
			{
				string shaderTypeStr = str;

				if ( shaderTypeStr == "VERTEX" )
				{
					shaderType = Shader::VERTEX_SHADER;
				}
				else if ( shaderTypeStr == "FRAGMENT" )
				{
					shaderType = Shader::FRAGMENT_SHADER;
				}
				else
				{
					ZazenGraphics::getInstance().getLogger().logWarning( "unknown type in shader - will be ignored" );
					continue;
				}
			}

			str = shaderNode->Attribute( "file" );
			if ( 0 == str )
			{
				ZazenGraphics::getInstance().getLogger().logWarning( "No file for shader - will be ignored" );
				continue;
			}
			else
			{
				file += str;
			}

			map<string, Shader*>::iterator findIter = ProgramManagement::m_shaders.find( file );
			if ( findIter != ProgramManagement::m_shaders.end() )
			{
				shader = findIter->second;
			}
			else
			{
				shader = Shader::createShader( shaderType, file );
				if ( NULL == shader )
				{
					ZazenGraphics::getInstance().getLogger().logWarning( "Failed creating shader for program." );
					return false;
				}

				if ( false == shader->compile() )
				{
					ZazenGraphics::getInstance().getLogger().logWarning( "Failed compiling shader for program." );
					return false;
				}

				ProgramManagement::m_shaders[ file ] = shader;
			}

			if ( false == program->attachShader( shader ) )
			{
				return false;
			}
		}
	}

	return true;
}

bool
ProgramManagement::parseFragDataLocations( TiXmlElement* programNode, Program* program )
{
	TiXmlElement* fragDataLocationsNode = programNode->FirstChildElement( "fragDataLocations" );
	if ( 0 == fragDataLocationsNode )
	{
		ZazenGraphics::getInstance().getLogger().logWarning() << "no fragment-data locations defined for program ";
		return true;
	}

	for ( TiXmlElement* fragDataLocNode = fragDataLocationsNode->FirstChildElement(); fragDataLocNode != 0; fragDataLocNode = fragDataLocNode->NextSiblingElement() )
	{
		const char* str = fragDataLocNode->Value();
		if ( NULL == str )
		{
			continue;
		}

		if ( 0 == strcmp( str, "fragDataLocation" ) )
		{
			string name;
			unsigned int colorNumber = 0;
			
			str = fragDataLocNode->Attribute( "colorNumber" );
			if ( 0 == str )
			{
				ZazenGraphics::getInstance().getLogger().logWarning( "No colorNumber in fragdatalocation - will be ignored" );
				continue;
			}
			else
			{
				colorNumber = atoi( str );
			}

			str = fragDataLocNode->Attribute( "name" );
			if ( 0 == str )
			{
				ZazenGraphics::getInstance().getLogger().logWarning( "No name in fragdatalocation - will be ignored" );
				continue;
			}
			else
			{
				name = str;
			}

			if ( false == program->bindFragDataLocation( colorNumber, name ) )
			{
				return false;
			}
		}
	}

	return true;
}

bool
ProgramManagement::parseAttribLocations( TiXmlElement* programNode, Program* program )
{
	TiXmlElement* attribLocationsNode = programNode->FirstChildElement( "attribLocations" );
	if ( 0 == attribLocationsNode )
	{
		ZazenGraphics::getInstance().getLogger().logWarning() << "no attribLocations defined for program ";
		return true;
	}

	for ( TiXmlElement* attLocNode = attribLocationsNode->FirstChildElement(); attribLocationsNode != 0; attribLocationsNode = attribLocationsNode->NextSiblingElement() )
	{
		const char* str = attLocNode->Value();
		if ( NULL == str )
		{
			continue;
		}

		if ( 0 == strcmp( str, "attribLocation" ) )
		{
			string name;
			unsigned int index = 0;
			
			str = attLocNode->Attribute( "index" );
			if ( 0 == str )
			{
				ZazenGraphics::getInstance().getLogger().logWarning( "No index in attrib-location - will be ignored" );
				continue;
			}
			else
			{
				index = atoi( str );
			}

			str = attLocNode->Attribute( "name" );
			if ( 0 == str )
			{
				ZazenGraphics::getInstance().getLogger().logWarning( "No name in attrib-location - will be ignored" );
				continue;
			}
			else
			{
				name = str;
			}

			if ( false == program->bindAttribLocation( index, name ) )
			{
				return false;
			}
		}
	}

	return true;
}

bool
ProgramManagement::parseBoundUniforms( TiXmlElement* programNode, Program* program )
{
	TiXmlElement* boundUniformBlockParentNode = programNode->FirstChildElement( "boundUniformBlocks" );
	if ( 0 == boundUniformBlockParentNode )
	{
		ZazenGraphics::getInstance().getLogger().logWarning() << "no bound uniform blocks defined for program ";
		return true;
	}

	for ( TiXmlElement* boundUniformBlockChildNode = boundUniformBlockParentNode->FirstChildElement(); boundUniformBlockParentNode != 0; boundUniformBlockParentNode = boundUniformBlockParentNode->NextSiblingElement() )
	{
		const char* str = boundUniformBlockChildNode->Value();
		if ( NULL == str )
		{
			continue;
		}

		if ( 0 == strcmp( str, "boundUniformBlock" ) )
		{
			string name;

			str = boundUniformBlockChildNode->Attribute( "name" );
			if ( 0 == str )
			{
				ZazenGraphics::getInstance().getLogger().logWarning( "No name in boundUniformBlock - will be ignored" );
				continue;
			}
			else
			{
				name = str;
			}

			UniformBlock* uniformBlock = UniformBlockManagement::get( name );
			if ( NULL == uniformBlock )
			{
				ZazenGraphics::getInstance().getLogger().logWarning( "Uniformblock for program not found - will be ignored" );
				continue;
			}

			if ( false == program->bindUniformBlock( uniformBlock ) )
			{
				return false;
			}
		}
	}

	return true;
}

bool
ProgramManagement::freeAll()
{
	map<string, Shader*>::iterator shadersIter = ProgramManagement::m_shaders.begin();
	while ( shadersIter != ProgramManagement::m_shaders.end() )
	{
		delete shadersIter->second;

		shadersIter++;
	}

	map<string, Program*>::iterator programsIter = ProgramManagement::m_programs.begin();
	while ( programsIter != ProgramManagement::m_programs.end() )
	{
		delete programsIter->second;

		programsIter++;
	}

	ProgramManagement::m_programs.clear();
	ProgramManagement::m_shaders.clear();

	return true;
}

Program*
ProgramManagement::get( const std::string& name )
{
	map<string, Program*>::iterator findIter = ProgramManagement::m_programs.find( name );
	if ( ProgramManagement::m_programs.end() != findIter )
	{
		return findIter->second;
	}

	return NULL;
}
