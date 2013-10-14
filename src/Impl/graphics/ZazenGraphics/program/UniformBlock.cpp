/*
 * UniformBlock.cpp
 *
 *  Created on: Jan 31, 2011
 *      Author: jonathan
 */

#include "UniformBlock.h"

#include "../ZazenGraphics.h"
#include "../util/GLUtils.h"

#include <glm/gtc/type_ptr.hpp>

#include <iostream>

using namespace std;

// the index used to bind the uniformblock with glBindBufferBase
// this is the index like the activetexture in texturing. its different because
// an arbitrary number of uniformblocks can be active
GLuint UniformBlock::m_nextBinding = 0;

// no more needed, see bindBuffer for info
//int UniformBlock::m_currentBoundId = -1;

UniformBlock*
UniformBlock::createBlock( const std::string& name )
{
	GLuint id;

	glGenBuffers( 1, &id );
	if ( GL_PEEK_ERRORS )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "UniformBlock::createBlock: glGenBuffers failed for name \"" << name << "\"";
		return 0;
	}

	UniformBlock* block = new UniformBlock( id, UniformBlock::m_nextBinding, name );

	UniformBlock::m_nextBinding++;

	return block;
}

UniformBlock::UniformBlock( GLuint id, GLuint binding, const std::string& name )
	: m_name ( name )
{
	this->m_id = id;
	this->m_binding = binding;
}

UniformBlock::~UniformBlock()
{
	glDeleteBuffers( 1, &this->m_id );

	map<string, UniformField*>::iterator deleteIter = this->m_fields.begin();
	while ( deleteIter != this->m_fields.end() )
	{
		delete deleteIter->second;

		deleteIter++;
	}
}

bool
UniformBlock::bindBase()
{
	glBindBufferBase( GL_UNIFORM_BUFFER, this->m_binding, this->m_id );
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}

bool
UniformBlock::bindBuffer()
{
	// WARNING: should not do this because it seems that this won't work internally
	// because there are multiple buffer-types so we need to set it always although it
	// seems this UNIFORM_BUFFER with the same id is already bound
	//if ( UniformBlock::m_currentBoundId != this->m_id )
	//{
		glBindBuffer( GL_UNIFORM_BUFFER, this->m_id );
		GL_PEEK_ERRORS_AT_DEBUG

		//UniformBlock::m_currentBoundId = this->m_id;
	//}

	return true;
}

bool
UniformBlock::updateData( const void* data, int offset, int size )
{
	glBufferSubData( GL_UNIFORM_BUFFER, offset, size, data );
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}

bool
UniformBlock::updateData( const void* data, int size )
{
	glBufferData( GL_UNIFORM_BUFFER, size, data, GL_DYNAMIC_DRAW );
	GL_PEEK_ERRORS_AT_DEBUG

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

bool
UniformBlock::updateField( const std::string& fieldName, const glm::mat4& data )
{
	UniformField* field = this->getUniformField( fieldName );
	if ( NULL == field )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "UniformBlock::updateField: attempted to update unknown field \"" << fieldName << "\"";
		return false;
	}

	return this->updateMat4( data, field->m_offset );
}

bool
UniformBlock::updateField( const std::string& fieldName, const glm::vec4& data )
{
	UniformField* field = this->getUniformField( fieldName );
	if ( NULL == field )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "UniformBlock::updateField: attempted to update unknown field \"" << fieldName << "\"";
		return false;
	}

	return this->updateVec4( data, field->m_offset );
}

UniformBlock::UniformField*
UniformBlock::getUniformField( const std::string& name )
{
	map<string, UniformField*>::iterator findIter = this->m_fields.find( name );
	if ( findIter != this->m_fields.end() )
	{
		return findIter->second;
	}

	return NULL;
}
