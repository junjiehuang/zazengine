#include "UniformManagement.h"

#include "Program.h"

#include "../ZazenGraphics.h"
#include "../util/GLUtils.h"

#include <iostream>

using namespace std;
using namespace boost;

map<string, UniformBlock*> UniformManagement::m_uniformBlocks;

bool
UniformManagement::freeAllBlocks()
{
	map<string, UniformBlock*>::iterator iter = UniformManagement::m_uniformBlocks.begin();
	while ( iter != UniformManagement::m_uniformBlocks.end() )
	{
		delete iter->second;

		iter++;
	}

	UniformManagement::m_uniformBlocks.clear();

	return true;
}

UniformBlock*
UniformManagement::getBlock( const std::string& name )
{
	map<string, UniformBlock*>::iterator findIter = UniformManagement::m_uniformBlocks.find( name );
	if ( UniformManagement::m_uniformBlocks.end() != findIter )
	{
		return findIter->second;
	}

	return NULL;
}

// TODO check for opengl-errors
bool
UniformManagement::initUniforms( Program* program )
{
	GLint uniformsCount = 0;
	GLint uniformBlocksCount = 0;

	glGetProgramiv( program->getId(), GL_ACTIVE_UNIFORMS, &uniformsCount );
	glGetProgramiv( program->getId(), GL_ACTIVE_UNIFORM_BLOCKS, &uniformBlocksCount );

	for ( int i = 0; i < uniformsCount; i++ )
	{
		GLint uniformBlockIndex = -1;
		GLuint uniformIndex = i;
			
		glGetActiveUniformsiv( program->getId(), 1, &uniformIndex, GL_UNIFORM_BLOCK_INDEX, &uniformBlockIndex );
		
		if ( uniformBlockIndex == -1 )
		{
			GLsizei nameLength = 0;
			vector<GLchar> nameBuffer( 1024 );

			GLint uniformSize = 0;
			GLenum uniformType = 0;

			GLint uniformBlockOffset = -1;
			glGetActiveUniform( program->getId(), i, nameBuffer.size() - 1, &nameLength, &uniformSize, &uniformType, &nameBuffer[ 0 ] );
			glGetActiveUniformsiv( program->getId(), 1, &uniformIndex, GL_UNIFORM_OFFSET, &uniformBlockOffset );
			// TODO check for opengl-errors

			string uniformFieldName( &nameBuffer[ 0 ] );

			Program::UniformField* uniformField = new Program::UniformField();
			uniformField->m_index = uniformIndex;
			uniformField->m_name = uniformFieldName;
			uniformField->m_size = uniformSize;
			uniformField->m_type = uniformType;

			program->m_uniforms[ uniformFieldName ] = uniformField;
		}
	}

	for ( int i = 0; i < uniformBlocksCount; i++ )
	{
		GLsizei nameLength = 0;
		vector<GLchar> nameBuffer( 1024 );

		glGetActiveUniformBlockName( program->getId(), i, nameBuffer.size() - 1, &nameLength, &nameBuffer[ 0 ] );
		// TODO check for opengl-errors

		string uniformBlockName( &nameBuffer[ 0 ] );

		UniformBlock* uniformBlock = UniformManagement::getBlock( uniformBlockName );
		// not yet defined, create and initialize new
		if ( NULL == uniformBlock )
		{
			uniformBlock = UniformBlock::createBlock( uniformBlockName );
			// creation failed, leave
			if ( NULL == uniformBlock )
			{
				return false;
			}

			UniformManagement::m_uniformBlocks[ uniformBlockName ] = uniformBlock;
		}

		// we got a valid uniformblock, could exist already, maybe there are additional fields in this program which are removed by compiler in earlier program
		// search for all affected uniforms within this block
		for ( int j = 0; j < uniformsCount; j++ )
		{
			GLint uniformBlockIndex = -1;
			GLuint uniformIndex = j;
			
			glGetActiveUniformsiv( program->getId(), 1, &uniformIndex, GL_UNIFORM_BLOCK_INDEX, &uniformBlockIndex );
			// TODO check for opengl-errors

			// found one
			if ( uniformBlockIndex == i )
			{
				GLint uniformSize = 0;
				GLenum uniformType = 0;

				GLint uniformBlockOffset = -1;
				glGetActiveUniform( program->getId(), j, nameBuffer.size() - 1, &nameLength, &uniformSize, &uniformType, &nameBuffer[ 0 ] );
				glGetActiveUniformsiv( program->getId(), 1, &uniformIndex, GL_UNIFORM_OFFSET, &uniformBlockOffset );
				// TODO check for opengl-errors

				string uniformFieldName( &nameBuffer[ 0 ] );

				UniformBlock::UniformField* uniformField = uniformBlock->getUniformField( uniformFieldName );
				// not yet existent in this programs version of this uniform block, add this field
				if ( NULL == uniformField )
				{
					uniformField = new UniformBlock::UniformField();
					uniformField->m_index = uniformIndex;
					uniformField->m_name = uniformFieldName;
					uniformField->m_offset = uniformBlockOffset;
					uniformField->m_size = uniformSize;
					uniformField->m_type = uniformType;

					uniformBlock->m_fields[ uniformFieldName ] = uniformField;
				}
			}	
		}

		program->bindUniformBlock( uniformBlock );
	}

	return true;
}
