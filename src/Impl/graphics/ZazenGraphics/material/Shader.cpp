/*
 * Shader.cpp
 *
 *  Created on: Jan 23, 2011
 *      Author: jonathan
 */

#include "Shader.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <iostream>

using namespace std;

Shader*
Shader::createShader( Shader::ShaderType type, const std::string& file )
{
	GLuint shaderObject = 0;
	GLint status;
	std::string source;
	const char* sourcePtr = NULL;

	if (Shader::readShaderSource( file, source ) == false)
		return 0;

	if ( Shader::VERTEX_SHADER == type )
	{
		shaderObject = glCreateShader( GL_VERTEX_SHADER );
		if (0 == shaderObject )
		{
			cout << "ERROR ... in Shader::createShader: glCreateShader for GL_VERTEX_SHADER \"" << file << "\" failed with " << gluErrorString( glGetError() ) << endl;
			return 0;
		}
	}
	else if ( Shader::FRAGMENT_SHADER == type )
	{
		shaderObject = glCreateShader( GL_FRAGMENT_SHADER );
		if ( 0 == shaderObject )
		{
			cout << "ERROR ... in Shader::createShader: glCreateShader for GL_FRAGMENT_SHADER \"" << file << "\" failed with " << gluErrorString( glGetError() ) << endl;
			return 0;
		}
	}
	else if ( Shader::GEOMETRY_SHADER == type )
	{
		shaderObject = glCreateShader( GL_GEOMETRY_SHADER_EXT );
		if ( 0 == shaderObject )
		{
			cout << "ERROR ... in Shader::createShader: glCreateShader for GL_GEOMETRY_SHADER_EXT \"" << file << "\" failed with " << gluErrorString( glGetError() ) << endl;
			return 0;
		}
	}

	sourcePtr = source.c_str();
	glShaderSource( shaderObject, 1, ( const GLchar** ) &sourcePtr, NULL);
	status = glGetError();
	if ( GL_NO_ERROR != status )
	{
		cout << "ERROR ... Shader::createShader: glShaderSource for \"" << file << "\" failed with " << gluErrorString( status ) << endl;
		return 0;
	}


	return new Shader( shaderObject, file );
}

Shader::Shader( GLuint shaderObject, const std::string& fileName )
{
	this->m_shaderObject = shaderObject;
	this->m_fileName = fileName;
}

Shader::~Shader()
{
	glDeleteShader( this->m_shaderObject );
}

void
Shader::printInfoLog()
{
	GLchar* infoLog = 0;
	GLint infoLogLen = 0;
	GLint charsWritten  = 0;

	glGetShaderiv( this->m_shaderObject  , GL_INFO_LOG_LENGTH, &infoLogLen );
	if (infoLogLen > 0)
	{
		infoLog = ( GLchar* ) malloc( ( infoLogLen + 1 ) * sizeof( GLchar ) );
		memset( infoLog, 0, infoLogLen + 1 );

		glGetShaderInfoLog( this->m_shaderObject , infoLogLen, &charsWritten, infoLog );

	    if ( charsWritten )
	    	cout << infoLog << endl;

	    free( infoLog );
	}
}

bool
Shader::compile()
{
	GLint status;

	glCompileShader( this->m_shaderObject );
	glGetShaderiv( this->m_shaderObject, GL_COMPILE_STATUS, &status );
	if ( GL_TRUE != status )
	{
		cout << "ERROR ... in Shader::compile for shader " << this->m_fileName << ": failed compilation of shader. Error-Log:" << endl;
		this->printInfoLog();
		return false;
	}

	return true;
}

bool
Shader::readShaderSource( const string& file, string& shaderSource )
{
	string fullFileName = file;
	FILE* shaderSourceFile = NULL;

	if ( 0 != fopen_s( &shaderSourceFile, fullFileName.c_str(), "r" ) )
	{
		cout << "ERROR ... in Shader::readShaderSource: couldn't open Shadersource-File " << fullFileName << endl;
		return false;
	}

	char c;
	while ( EOF != ( c = fgetc( shaderSourceFile ) ) )
		shaderSource += c;

	fclose( shaderSourceFile );

	return true;
}
