/*
 * UniformBlock.cpp
 *
 *  Created on: Jan 31, 2011
 *      Author: jonathan
 */

#include "UniformBlock.h"

#include <iostream>

using namespace std;

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
	block->id = id;

	return block;
}

UniformBlock::UniformBlock( const std::string& name )
	: name ( name )
{
	this->id = 0;
}

UniformBlock::~UniformBlock()
{
}

bool
UniformBlock::updateData( void* data, int offset, int size )
{
	return true;
}

bool
UniformBlock::updateData( void* data, int size )
{
	GLint status;

	glBindBuffer( GL_UNIFORM_BUFFER, this->id );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "UniformBlock::updateData: glBindBuffer failed for name \"" << this->name << "\": " << gluErrorString( status )  << endl;
		return false;
	}

	glBufferData( GL_UNIFORM_BUFFER, size, data, GL_DYNAMIC_DRAW );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "UniformBlock::updateData: glBufferData failed for name \"" << this->name << "\": " << gluErrorString( status )  << endl;
		return false;
	}

	glBindBuffer( GL_UNIFORM_BUFFER, 0 );
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "UniformBlock::updateData: glBindBuffer( 0 ) failed for name \"" << this->name << "\": " << gluErrorString( status )  << endl;
		return false;
	}

	return true;
}
