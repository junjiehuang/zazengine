#include "Program.h"

#include "../ZazenGraphics.h"
#include "../Util/GLUtils.h"

#include <glm/gtc/type_ptr.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <iostream>

using namespace std;

Program*
Program::createProgram( const std::string& programName )
{
	GLuint programObject = 0;

	programObject = glCreateProgram();
	if ( 0 == programObject )
	{
		if ( GL_PEEK_ERRORS )
		{
			ZazenGraphics::getInstance().getLogger().logError() << "Program::createProgram for programm " << programName << ": glCreateProgram failed";
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
}

void
Program::printInfoLog()
{
	GLint infoLogLen = 0;
	
	glGetProgramiv( this->m_programObject , GL_INFO_LOG_LENGTH, &infoLogLen );
	if ( 0 < infoLogLen )
	{
		GLint charsWritten  = 0;
		vector<GLchar> buffer( infoLogLen + 1 );

		glGetProgramInfoLog( this->m_programObject, infoLogLen, &charsWritten, &buffer[ 0 ] );

	    if ( charsWritten )
		{
			ZazenGraphics::getInstance().getLogger().logError( string( &buffer[ 0 ] ) );
		}
	}
}

bool
Program::attachShader( Shader* shader )
{
	glAttachShader( this->m_programObject, shader->getObject() );
	if ( GL_PEEK_ERRORS )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::attachShader for programm " << this->m_programName << ": glAttachShader";
		return false;
	}

	return true;
}


bool
Program::detachShader( Shader* shader )
{
	glDetachShader( this->m_programObject, shader->getObject() );
	if ( GL_PEEK_ERRORS )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::detachShader for programm " << this->m_programName << ": glDetachShader failed";
		return false;
	}

	return true;
}

bool
Program::link()
{
	GLint status;

	glLinkProgram( this->m_programObject );
	if ( GL_PEEK_ERRORS )
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
Program::activateSubroutine( const std::string& subroutineName, Shader::ShaderType shaderType )
{
	GLenum shaderTypeGL = GL_VERTEX_SHADER;
	
	if ( Shader::ShaderType::FRAGMENT_SHADER == shaderType )
	{
		shaderTypeGL = GL_FRAGMENT_SHADER;
	}
	
	vector<GLuint>& subroutineConfig = this->m_activeSubroutineConfig[ shaderTypeGL ];
	vector<Subroutine>& activeSubroutinesTyped = this->m_activeSubroutines[ shaderTypeGL ];
	map<string, Subroutine>& allSubroutinesTyped = this->m_allSubroutines[ shaderTypeGL ];
	
	// no subroutines for this kind of shader-type - no need to check the other vectors because if no subroutines for this
	// given shader-type the vectors will be empty too
	if ( allSubroutinesTyped.empty() )
	{
		return false;
	}

	map<string, Subroutine>::iterator findIter = allSubroutinesTyped.find( subroutineName );
	if ( allSubroutinesTyped.end() != findIter )
	{
		Subroutine& newSubroutine = findIter->second;

		// found the according subroutine, replace it in the active ones
		for ( unsigned int i = 0; i < activeSubroutinesTyped.size(); i++ )
		{
			Subroutine& activeSubroutine = activeSubroutinesTyped[ i ];

			// only one subroutine can be active with same index, so thats the way we find them
			// the m_uniformLocation correspond to the index i
			if ( activeSubroutine.m_uniformIndex == newSubroutine.m_uniformIndex )
			{
				// no change, no update
				if ( activeSubroutine.m_index == newSubroutine.m_index )
				{
					return true;
				}

				subroutineConfig[ i ] = newSubroutine.m_index;
				activeSubroutinesTyped[ i ] = newSubroutine;

				break;
			}
		}
	}

	// need to pass exactly GL_ACTIVE_SUBROUTINE_UNIFORM_LOCATIONS items in range 0 to GL_ACTIVE_SUBROUTINES - 1
	// this implies subroutinesConfig at index 0 is responsible for location 0 and specifies the index of the subroutine
	glUniformSubroutinesuiv( shaderTypeGL, subroutineConfig.size(), &subroutineConfig[ 0 ] );
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}

bool
Program::bindUniformBlock( UniformBlock* block )
{
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
	if ( GL_PEEK_ERRORS )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::bindUniformBlock for programm " << this->m_programName << ": glUniformBlockBinding failed for name \"" << block->getName() << "\"";
		return false;
	}

	return true;
}

bool
Program::bindAttribLocation( GLuint index, const std::string& name )
{
	glBindAttribLocation( this->m_programObject, index, name.c_str() );
	if ( GL_PEEK_ERRORS )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::bindAttribLocation for programm " << this->m_programName << ": glBindAttribLocation failed for name \"" << name << "\"";
		return false;
	}

	return true;
}

bool
Program::bindFragDataLocation( GLuint index, const std::string& name )
{
	glBindFragDataLocation( this->m_programObject, index, name.c_str() );
	if ( GL_PEEK_ERRORS )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Program::bindFragDataLocation for programm " << this->m_programName << ": glBindFragDataLocation failed for name \"" << name << "\"";
		return false;
	}

	return true;
}

bool
Program::use()
{
	glUseProgram( this->m_programObject );
	GL_PEEK_ERRORS_AT_DEBUG

	map<GLenum, vector<GLuint>>::iterator iter = this->m_activeSubroutineConfig.begin();
	while ( this->m_activeSubroutineConfig.end() != iter )
	{
		glUniformSubroutinesuiv( iter->first, iter->second.size(), &iter->second[ 0 ] );
		GL_PEEK_ERRORS_AT_DEBUG

		iter++;
	}

	return true;
}

bool
Program::unuse()
{
	glUseProgram( 0 );
	GL_PEEK_ERRORS_AT_DEBUG

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
	GL_PEEK_ERRORS_AT_DEBUG

	return true;
}

bool
Program::setUniformMatrices( const std::string& name, const std::vector<glm::mat4>& matrices )
{
	UniformField* field = this->getUniformField( name );
	if ( NULL == field )
	{
		return false;
	}

	glUniformMatrix4fv( field->m_index, matrices.size(), GL_FALSE, glm::value_ptr( matrices[ 0 ] ) );
	GL_PEEK_ERRORS_AT_DEBUG

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
	map<string, UniformField>::iterator findIter = this->m_uniforms.find( name );
	if ( findIter != this->m_uniforms.end() )
	{
		return &findIter->second;
	}

	return NULL;
}
