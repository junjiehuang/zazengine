/*
 * UniformBlockManagement.cpp
 *
 *  Created on: 16. April 2013
 *      Author: jonathan
 */

#include "UniformBlockManagement.h"

#include "../ZazenGraphics.h"

#include <iostream>

using namespace std;
using namespace boost;

map<string, UniformBlock*> UniformBlockManagement::m_uniformBlocks;

bool
UniformBlockManagement::init( const boost::filesystem::path& path )
{
	string fullFileName = path.generic_string() + "uniformBlockConfig.xml";

	TiXmlDocument doc( fullFileName.c_str() );

	if ( false == doc.LoadFile() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "could not load file " << fullFileName << " - reason = " << doc.ErrorDesc() << " at row = " << doc.ErrorRow() << " col = " << doc.ErrorCol();
		return false;
	}

	TiXmlElement* rootNode = doc.FirstChildElement( "uniformBlocks" );
	if ( 0 == rootNode )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "root-node \"uniformBlocks\" in " << fullFileName << " not found";
		return false;
	}

	for ( TiXmlElement* uniformBlockNode = rootNode->FirstChildElement(); uniformBlockNode != 0; uniformBlockNode = uniformBlockNode->NextSiblingElement() )
	{
		const char* str = uniformBlockNode->Value();
		if ( NULL == str )
		{
			continue;
		}

		if ( 0 == strcmp( str, "uniformBlock" ) )
		{
			std::string name;

			str = uniformBlockNode->Attribute( "name" );
			if ( 0 == str )
			{
				ZazenGraphics::getInstance().getLogger().logWarning( "No name for uniform-block - will be ignored" );
				continue;
			}
			else
			{
				name = str;
			}

			UniformBlock* uniformBlock = UniformBlock::createBlock( name );
			if ( NULL == uniformBlock )
			{
				ZazenGraphics::getInstance().getLogger().logError() << "Failed creating uniform-block " << name;
				return false;
			}

			UniformBlockManagement::m_uniformBlocks[ name ] = uniformBlock;
		}
	}

	return true;
}

bool
UniformBlockManagement::freeAll()
{
	map<string, UniformBlock*>::iterator iter = UniformBlockManagement::m_uniformBlocks.begin();
	while ( iter != UniformBlockManagement::m_uniformBlocks.end() )
	{
		delete iter->second;

		iter++;
	}

	UniformBlockManagement::m_uniformBlocks.clear();

	return true;
}

UniformBlock*
UniformBlockManagement::get( const std::string& name )
{
	map<string, UniformBlock*>::iterator findIter = UniformBlockManagement::m_uniformBlocks.find( name );
	if ( UniformBlockManagement::m_uniformBlocks.end() != findIter )
	{
		return findIter->second;
	}

	return NULL;
}

// TODO check for opengl-errors
bool
UniformBlockManagement::initUniformBlocks( Program* program )
{
	GLint uniformsCount = 0;
	GLint uniformBlocksCount = 0;

	glGetProgramiv( program->getId(), GL_ACTIVE_UNIFORMS, &uniformsCount );
	glGetProgramiv( program->getId(), GL_ACTIVE_UNIFORM_BLOCKS, &uniformBlocksCount );

	for ( int i = 0; i < uniformBlocksCount; i++ )
	{
		GLsizei nameLength = 0;
		vector<GLchar> nameBuffer( 1024 );

		glGetActiveUniformBlockName( program->getId(), i, nameBuffer.size() - 1, &nameLength, &nameBuffer[ 0 ] );
		// TODO check for opengl-errors

		string uniformName( &nameBuffer[ 0 ] );

		UniformBlock* uniformBlock = UniformBlockManagement::get( uniformName );
		if ( NULL == uniformBlock )
		{
			uniformBlock = UniformBlock::createBlock( uniformName );
			if ( NULL == uniformBlock )
			{
				
			}
		}

		for ( int j = 0; j < uniformsCount; j++ )
		{
			GLint uniformBlockIndex = -1;
			GLuint uniformIndex = j;
			
			glGetActiveUniformsiv( program->getId(), 1, &uniformIndex, GL_UNIFORM_BLOCK_INDEX, &uniformBlockIndex );
			
			if ( uniformBlockIndex == i )
			{
				GLint uniformSize = 0;
				GLenum uniformType = 0;

				GLint uniformBlockOffset = -1;
				glGetActiveUniform( program->getId(), j, nameBuffer.size() - 1, &nameLength, &uniformSize, &uniformType, &nameBuffer[ 0 ] );
				glGetActiveUniformsiv( program->getId(), 1, &uniformIndex, GL_UNIFORM_OFFSET, &uniformBlockOffset );

				string subUniformName( &nameBuffer[ 0 ] );
			}
		}
	}

	return true;
}
