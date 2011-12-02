/*
 * UniformBlock.cpp
 *
 *  Created on: Jan 31, 2011
 *      Author: jonathan
 */

#include "UniformBlock.h"

#include <iostream>

using namespace std;

// the index used to bind the uniformblock with glBindBufferBase
// this is the index like the activetexture in texturing. its different because
// an arbitrary number of uniformblocks can be active
GLuint UniformBlock::bindIndexer = 0;

UniformBlock*
UniformBlock::createBlock( const std::string& name )
{
	GLuint id;
	GLint status;

	glGenBuffers( 1, &id );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "UniformBlock::createBlock: glGenBuffers failed for name \"" << name << "\": " << gluErrorString( status )  << endl;
		return 0;
	}

	UniformBlock* block = new UniformBlock( name );
	block->m_id = id;
	block->m_bindIndex = UniformBlock::bindIndexer;

	UniformBlock::bindIndexer++;

	return block;
}

UniformBlock::UniformBlock( const std::string& name )
	: m_name ( name )
{
	this->m_id = 0;
	this->m_bindIndex = 0;
}

UniformBlock::~UniformBlock()
{
	if ( this->m_id )
		glDeleteBuffers( 1, &this->m_id );
}

bool
UniformBlock::bind()
{
	GLint status;

	glBindBufferBase( GL_UNIFORM_BUFFER, this->m_bindIndex, this->m_id );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "UniformBlock::bind: glBindBufferBase failed for name \"" << this->m_name << "\": " << gluErrorString( status )  << endl;
		return false;
	}

	return true;
}

bool
UniformBlock::updateData( const void* data, int offset, int size )
{
	GLint status;

	if ( false == this->bindBuffer() )
		return false;

	glBufferSubData( GL_UNIFORM_BUFFER, offset, size, data );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "UniformBlock::updateData: glBufferData failed for name \"" << this->m_name << "\": " << gluErrorString( status )  << endl;
		return false;
	}

	if ( false == this->unbindBuffer() )
		return false;

	return true;
}

bool
UniformBlock::updateData( const void* data, int size )
{
	GLint status;

	if ( false == this->bindBuffer() )
		return false;

	glBufferData( GL_UNIFORM_BUFFER, size, data, GL_DYNAMIC_DRAW );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "UniformBlock::updateData: glBufferData failed for name \"" << this->m_name << "\": " << gluErrorString( status )  << endl;
		return false;
	}

	if ( false == this->unbindBuffer() )
		return false;

	return true;
}

bool
UniformBlock::bindBuffer()
{
	GLint status;

	glBindBuffer( GL_UNIFORM_BUFFER, this->m_id );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "UniformBlock::bindBuffer: glBindBuffer failed for name \"" << this->m_name << "\": " << gluErrorString( status )  << endl;
		return false;
	}

	return true;
}

bool
UniformBlock::unbindBuffer()
{
	GLint status;

	glBindBuffer( GL_UNIFORM_BUFFER, 0 );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "UniformBlock::unbindBuffer: glBindBuffer( 0 ) failed for name \"" << this->m_name << "\": " << gluErrorString( status )  << endl;
		return false;
	}

	return true;
}
