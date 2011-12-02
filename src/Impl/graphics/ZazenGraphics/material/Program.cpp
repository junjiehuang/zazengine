/*
 * Program.cpp
 *
 *  Created on: Jan 16, 2011
 *      Author: jonathan
 */

#include "Program.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <iostream>

using namespace std;

Program*
Program::createProgram()
{
	GLint status;
	GLuint programObject = 0;

	programObject = glCreateProgram();
	if ( 0 == programObject )
	{
		if ( GL_NO_ERROR != ( status = glGetError() ) )
			cout << "glCreateProgram failed with " << gluErrorString( status ) << endl;

		return 0;
	}

	return new Program( programObject );
}

Program::Program( GLuint programObject )
{
	this->programObject = programObject;
}

Program::~Program()
{
	glDeleteProgram( this->programObject );
}

void
Program::printInfoLog()
{
	GLchar* infoLog = 0;
	GLint infoLogLen = 0;
	GLint charsWritten  = 0;

	glGetProgramiv( this->programObject , GL_INFO_LOG_LENGTH, &infoLogLen );
	if (infoLogLen > 0)
	{
		infoLog = (GLchar*) malloc( ( infoLogLen + 1 ) * sizeof( GLchar ) );
		memset( infoLog, 0, infoLogLen + 1 );

		glGetProgramInfoLog( this->programObject, infoLogLen, &charsWritten, infoLog );

	    if ( charsWritten )
			cout << infoLog << endl;

	    free( infoLog );
	}
}

bool
Program::attachShader( Shader* shader )
{
	GLint status;

	glAttachShader( this->programObject, shader->getObject() );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "glAttachShader failed with " << gluErrorString( status ) << endl;
		return false;
	}

	return true;
}


bool
Program::detachShader( Shader* shader )
{
	GLint status;

	glDetachShader( this->programObject, shader->getObject() );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "glDetachShader failed with " << gluErrorString( status ) << endl;
		return false;
	}

	return true;
}

bool
Program::link()
{
	GLint status;

	glLinkProgram( this->programObject );
	glGetProgramiv( this->programObject, GL_LINK_STATUS, &status);
	if ( GL_TRUE != status )
	{
		cout << "ERROR linking of program failed" << endl;
		this->printInfoLog();
		return false;
	}

	return true;
}

bool
Program::bindUniformBlock( UniformBlock* block )
{
	GLint status;
	GLint uniformBlockSize;

	GLuint index = this->getUniformBlockIndex( block->getName() );
	if ( GL_INVALID_INDEX == index )
		return false;

	glGetActiveUniformBlockiv( this->programObject, index, GL_UNIFORM_BLOCK_DATA_SIZE, &uniformBlockSize );

	// create the UBO
	if ( false == block->updateData( 0, uniformBlockSize ) )
		return false;

	//if ( false == block->bind( index ) )
	//	return false;

	glUniformBlockBinding( this->programObject, index,  block->getBindIndex() );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "glUniformBlockBinding failed for name \"" << block->getName() << "\": " << gluErrorString( status )  << endl;
		return false;
	}

	return true;
}

bool
Program::bindAttribLocation( GLuint index, const std::string& name )
{
	GLint status;

	glBindAttribLocation( this->programObject, index, name.c_str() );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "glBindAttribLocation failed for name \"" << name << "\": " << gluErrorString( status )  << endl;
		return false;
	}

	return true;
}

GLint
Program::getAttribLocation( const std::string& name )
{
	GLint location = 0;

	location = glGetAttribLocation( this->programObject, name.c_str() );
	if ( -1 == location )
		cout << "Coulnd't glGetAttribLocation location for name \"" << name << "\". OpenGL-Error: " << gluErrorString(glGetError())  << endl;

	return location;
}

bool
Program::bindFragDataLocation( GLuint index, const std::string& name )
{
	GLint status;

	glBindFragDataLocation( this->programObject, index, name.c_str() );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "glBindFragDataLocation failed for name \"" << name << "\": " << gluErrorString( status )  << endl;
		return false;
	}

	return true;
}

GLint
Program::getFragDataLocation( const std::string& name )
{
	GLint location = 0;

	location = glGetFragDataLocation( this->programObject, name.c_str() );
	if ( -1 == location )
		cout << "Coulnd't glGetFragDataLocation location for name \"" << name << "\". OpenGL-Error: " << gluErrorString(glGetError())  << endl;

	return location;
}

bool
Program::use()
{
	GLint status;

	glUseProgram( this->programObject );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "glUseProgram failed: " << gluErrorString( status )  << endl;
		return false;
	}

	return true;
}

bool
Program::setUniform4( const std::string& name, const float* value )
{
	GLint status;
	GLint location = this->getUniformLocation( name );
	if ( -1 == location )
		return false;

	glUniform4fv( location, 4, value );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "glUniform4fv failed for " << name << ": " << gluErrorString( status )  << endl;
		return false;
	}

	return true;
}

bool
Program::setUniformInt( const std::string& name, int value )
{
	GLint status;
	GLint location = this->getUniformLocation( name );
	if ( -1 == location )
		return false;

	glUniform1i( location, value );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "glUniform1i failed for " << name << ": " << gluErrorString( status )  << endl;
		return false;
	}

	return true;
}

bool
Program::setUniformMatrix4( const std::string& name, const float* value )
{
	GLint status;

	GLint location = this->getUniformLocation( name );
	if ( -1 == location )
		return false;

	glUniformMatrix4fv( location, 1, GL_FALSE, value );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		cout << "glUniformMatrix4fv failed for " << name << ": " << gluErrorString( status )  << endl;
		return false;
	}

	return true;
}

GLuint
Program::getUniformBlockIndex( const std::string& name )
{
	GLuint index = 0;

	index = glGetUniformBlockIndex( this->programObject, name.c_str() );
	if ( GL_INVALID_INDEX == index )
		cout << "glGetUniformBlockIndex failed for name \"" << name << "\". OpenGL-Error: GL_INVALID_INDEX" << endl;

	return index;
}

GLint
Program::getUniformLocation( const std::string& name )
{
	GLint location = 0;

	location = glGetUniformLocation( this->programObject, name.c_str() );
	if ( -1 == location )
		cout << "Coulnd't get Uniform Location for name \"" << name << "\". OpenGL-Error: " << gluErrorString(glGetError())  << endl;

	return location;
}
