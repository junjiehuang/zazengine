/*
 * Program.cpp
 *
 *  Created on: Jan 16, 2011
 *      Author: jonathan
 */

#include "Program.h"

#include "../ZazenGraphics.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <iostream>

using namespace std;

Program*
Program::createProgram( const std::string& programName )
{
	GLint status;
	GLuint programObject = 0;

	programObject = glCreateProgram();
	if ( 0 == programObject )
	{
		if ( GL_NO_ERROR != ( status = glGetError() ) )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "Program::createProgram for programm " << programName << ": glCreateProgram failed with " << gluErrorString( status );
		}

		return NULL;
	}

	return new Program( programObject, programName );
}

Program::Program( GLuint programObject, const std::string& programName )
{
	this->m_programObject = programObject;
	this->m_programName = programName;
}

Program::~Program()
{
	glDeleteProgram( this->m_programObject );

	map<string, UniformField*>::iterator deleteIter = this->m_uniforms.begin();
	while ( deleteIter != this->m_uniforms.end() )
	{
		delete deleteIter->second;

		deleteIter++;
	}
}

void
Program::printInfoLog()
{
	GLchar* infoLog = 0;
	GLint infoLogLen = 0;
	GLint charsWritten  = 0;

	glGetProgramiv( this->m_programObject , GL_INFO_LOG_LENGTH, &infoLogLen );
	if (infoLogLen > 0)
	{
		infoLog = (GLchar*) malloc( ( infoLogLen + 1 ) * sizeof( GLchar ) );
		memset( infoLog, 0, infoLogLen + 1 );

		glGetProgramInfoLog( this->m_programObject, infoLogLen, &charsWritten, infoLog );

	    if ( charsWritten )
		{
			ZazenGraphics::getInstance().getLogger().logError( infoLog );
		}

	    free( infoLog );
	}
}

bool
Program::attachShader( Shader* shader )
{
	GLint status;

	glAttachShader( this->m_programObject, shader->getObject() );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::attachShader for programm " << this->m_programName << ": glAttachShader failed with " << gluErrorString( status );
		return false;
	}

	return true;
}


bool
Program::detachShader( Shader* shader )
{
	GLint status;

	glDetachShader( this->m_programObject, shader->getObject() );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::detachShader for programm " << this->m_programName << ": glDetachShader failed with " << gluErrorString( status );
		return false;
	}

	return true;
}

bool
Program::link()
{
	GLint status;

	glLinkProgram( this->m_programObject );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::link for programm " << this->m_programName << ": calling glLinkProgram failed.";
		return false;
	}

	glGetProgramiv( this->m_programObject, GL_LINK_STATUS, &status );
	if ( GL_TRUE != status )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::link for programm " << this->m_programName << ": linking of program failed. Error-Log:";
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
	{
		return false;
	}

	glGetActiveUniformBlockiv( this->m_programObject, index, GL_UNIFORM_BLOCK_DATA_SIZE, &uniformBlockSize );

	if ( false == block->bindBuffer() )
	{
		return false;
	}

	// create the UBO
	if ( false == block->updateData( 0, uniformBlockSize ) )
	{
		return false;
	}

	glUniformBlockBinding( this->m_programObject, index, block->getBinding() );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::bindUniformBlock for programm " << this->m_programName << ": glUniformBlockBinding failed for name \"" << block->getName() << "\": " << gluErrorString( status );
		return false;
	}

	return true;
}

bool
Program::bindAttribLocation( GLuint index, const std::string& name )
{
	GLint status;

	glBindAttribLocation( this->m_programObject, index, name.c_str() );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::bindAttribLocation for programm " << this->m_programName << ": glBindAttribLocation failed for name \"" << name << "\": " << gluErrorString( status );
		return false;
	}

	return true;
}

bool
Program::bindFragDataLocation( GLuint index, const std::string& name )
{
	GLint status;

	glBindFragDataLocation( this->m_programObject, index, name.c_str() );
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::bindFragDataLocation for programm " << this->m_programName << ": glBindFragDataLocation failed for name \"" << name << "\": " << gluErrorString( status );
		return false;
	}

	return true;
}

bool
Program::use()
{
	glUseProgram( this->m_programObject );

#ifdef CHECK_GL_ERRORS
	GLint status;

	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::use for programm " << this->m_programName << ": glUseProgram failed: " << gluErrorString( status );
		return false;
	}
#endif

	return true;
}

bool
Program::unuse()
{
	glUseProgram( 0 );

#ifdef CHECK_GL_ERRORS
	GLint status;

	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::unuse: glUseProgram( 0 ) failed: " << gluErrorString( status );
		return false;
	}
#endif

	return true;
}

bool
Program::setUniformInt( const std::string& name, int value )
{
	UniformField* field = this->getUniformField( name );
	if ( NULL == field )
	{
		return false;
	}

	glUniform1i( field->m_index, value );

#ifdef CHECK_GL_ERRORS
	GLint status;
	if ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::setUniformInt for programm " << this->m_programName << ": glUniform1i failed for " << name << ": " << gluErrorString( status );
		return false;
	}
#endif

	return true;
}

GLuint
Program::getUniformBlockIndex( const std::string& name )
{
	GLuint index = 0;

	index = glGetUniformBlockIndex( this->m_programObject, name.c_str() );
	if ( GL_INVALID_INDEX == index )
	{
		ZazenGraphics::getInstance().getLogger().logWarning() << "Program::getUniformBlockIndex for programm " << this->m_programName << ": glGetUniformBlockIndex failed for name \"" << name << "\". OpenGL-Error: GL_INVALID_INDEX";
	}

	return index;
}

Program::UniformField*
Program::getUniformField( const std::string& name )
{
	map<string, UniformField*>::iterator findIter = this->m_uniforms.find( name );
	if ( findIter != this->m_uniforms.end() )
	{
		return findIter->second;
	}

	return NULL;
}
