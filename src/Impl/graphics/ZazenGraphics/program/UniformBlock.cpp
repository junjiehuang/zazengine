/*
 * UniformBlock.cpp
 *
 *  Created on: Jan 31, 2011
 *      Author: jonathan
 */

#include "UniformBlock.h"

#include "../ZazenGraphics.h"

#include <glm/gtc/type_ptr.hpp>

#include <iostream>

using namespace std;

// the index used to bind the uniformblock with glBindBufferBase
// this is the index like the activetexture in texturing. its different because
// an arbitrary number of uniformblocks can be active
GLuint UniformBlock::m_nextBinding = 0;

int UniformBlock::m_currentBoundId = -1;

UniformBlock*
UniformBlock::createBlock( const std::string& name )
{
	GLuint id;
	GLint status;

	glGenBuffers( 1, &id );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "UniformBlock::createBlock: glGenBuffers failed for name \"" << name << "\": " << gluErrorString( status );
		return 0;
	}

	UniformBlock* block = new UniformBlock( name );
	block->m_id = id;
	block->m_binding = UniformBlock::m_nextBinding;

	UniformBlock::m_nextBinding++;

	return block;
}

UniformBlock::UniformBlock( const std::string& name )
	: m_name ( name )
{
	this->m_id = 0;
	this->m_binding = 0;
}

UniformBlock::~UniformBlock()
{
	if ( this->m_id )
	{
		glDeleteBuffers( 1, &this->m_id );
	}
}

bool
UniformBlock::bindBase()
{
	glBindBufferBase( GL_UNIFORM_BUFFER, this->m_binding, this->m_id );
	
#ifdef CHECK_GL_ERRORS
	GLint status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "UniformBlock::bindBase: glBindBufferBase failed for name \"" << this->m_name << "\": " << gluErrorString( status );
		return false;
	}
#endif

	return true;
}

bool
UniformBlock::bindBuffer()
{
	if ( UniformBlock::m_currentBoundId != this->m_id )
	{
		glBindBuffer( GL_UNIFORM_BUFFER, this->m_id );

#ifdef CHECK_GL_ERRORS
		GLint status = glGetError();
		if ( GL_NO_ERROR != status )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "UniformBlock::bindBuffer: glBindBuffer failed for name \"" << this->m_name << "\": " << gluErrorString( status );
			return false;
		}
#endif

		UniformBlock::m_currentBoundId = this->m_id;
	}

	return true;
}

bool
UniformBlock::updateData( const void* data, int offset, int size )
{
	glBufferSubData( GL_UNIFORM_BUFFER, offset, size, data );

#ifdef CHECK_GL_ERRORS
	GLint status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "UniformBlock::updateData: glBufferData failed for name \"" << this->m_name << "\": " << gluErrorString( status );
		return false;
	}
#endif

	return true;
}

bool
UniformBlock::updateData( const void* data, int size )
{
	glBufferData( GL_UNIFORM_BUFFER, size, data, GL_DYNAMIC_DRAW );

#ifdef CHECK_GL_ERRORS
	GLint status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "UniformBlock::updateData: glBufferData failed for name \"" << this->m_name << "\": " << gluErrorString( status );
		return false;
	}
#endif

	return true;
}

bool
UniformBlock::updateMat4( const glm::mat4& matrix, int offset )
{
	return this->updateData( glm::value_ptr( matrix ), offset, 64 );
}

bool
UniformBlock::updateVec4( const glm::vec4& vec, int offset )
{
	return this->updateData( glm::value_ptr( vec ), offset, 16 );
}
