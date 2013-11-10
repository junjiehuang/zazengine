#include "Shader.h"

#include "../ZazenGraphics.h"
#include "../util/GLUtils.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <iostream>

using namespace std;

Shader*
Shader::createShader( Shader::ShaderType type, const std::string& file )
{
	GLuint shaderObject = 0;
	std::string source;
	const char* sourcePtr = NULL;

	if ( false == Shader::readShaderSource( file, source ) )
	{
		return NULL;
	}

	if ( Shader::VERTEX_SHADER == type )
	{
		shaderObject = glCreateShader( GL_VERTEX_SHADER );
		if ( 0 == shaderObject )
		{
			GL_PEEK_ERRORS_AT
			ZazenGraphics::getInstance().getLogger().logError() << "Shader::createShader: glCreateShader for GL_VERTEX_SHADER \"" << file << "\" failed";
			return NULL;
		}
	}
	else if ( Shader::FRAGMENT_SHADER == type )
	{
		shaderObject = glCreateShader( GL_FRAGMENT_SHADER );
		if ( 0 == shaderObject )
		{
			GL_PEEK_ERRORS_AT
			ZazenGraphics::getInstance().getLogger().logError() << "Shader::createShader: glCreateShader for GL_FRAGMENT_SHADER \"" << file << "\" failed";
			return NULL;
		}
	}

	sourcePtr = source.c_str();
	glShaderSource( shaderObject, 1, ( const GLchar** ) &sourcePtr, NULL);
	if ( GL_PEEK_ERRORS )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Shader::createShader: glShaderSource for \"" << file << "\" failed";
		return NULL;
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
	GLint infoLogLen = 0;

	glGetShaderiv( this->m_shaderObject , GL_INFO_LOG_LENGTH, &infoLogLen );
	if ( 0 < infoLogLen )
	{
		GLint charsWritten  = 0;
		vector<GLchar> buffer( infoLogLen + 1 );
		glGetShaderInfoLog( this->m_shaderObject, infoLogLen, &charsWritten, &buffer[ 0 ] );
	    if ( charsWritten )
		{
			ZazenGraphics::getInstance().getLogger().logError( string( &buffer[ 0 ] ) );
		}
	}
}

bool
Shader::compile()
{
	GLint status;

	glCompileShader( this->m_shaderObject );
	GL_PEEK_ERRORS_AT

	glGetShaderiv( this->m_shaderObject, GL_COMPILE_STATUS, &status );
	if ( GL_TRUE != status )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "Shader::compile for shader " << this->m_fileName << ": failed compilation of shader. Error-Log:";
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
		ZazenGraphics::getInstance().getLogger().logError() << "Shader::readShaderSource: couldn't open Shadersource-File " << fullFileName;
		return false;
	}

	char c;
	while ( EOF != ( c = fgetc( shaderSourceFile ) ) )
	{
		shaderSource += c;
	}

	fclose( shaderSourceFile );

	return true;
}
