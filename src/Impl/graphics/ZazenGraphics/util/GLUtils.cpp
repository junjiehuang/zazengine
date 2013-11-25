#include "GLUtils.h"

#include "../ZazenGraphics.h"

#include <GL/glew.h>

#include <Windows.h>

#include <sstream>

using namespace std;

// copied and adjusted from ogl demo
void APIENTRY debugOutput ( GLenum source,
								GLenum type,
								GLuint id,
								GLenum severity,
								GLsizei length,
								const GLchar* message,
								GLvoid* userParam
)
{
	string debSource;
	string debType;
	string debSev;

	if(source == GL_DEBUG_SOURCE_API_ARB)
		debSource = "OpenGL";
	else if(source == GL_DEBUG_SOURCE_WINDOW_SYSTEM_ARB)
		debSource = "Windows";
	else if(source == GL_DEBUG_SOURCE_SHADER_COMPILER_ARB)
		debSource = "Shader Compiler";
	else if(source == GL_DEBUG_SOURCE_THIRD_PARTY_ARB)
		debSource = "Third Party";
	else if(source == GL_DEBUG_SOURCE_APPLICATION_ARB)
		debSource = "Application";
	else if (source == GL_DEBUG_SOURCE_OTHER_ARB)
		debSource = "Other";
	else
		assert(0);
 
	if(type == GL_DEBUG_TYPE_ERROR)
		debType = "error";
	else if(type == GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR)
		debType = "deprecated behavior";
	else if(type == GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR)
		debType = "undefined behavior";
	else if(type == GL_DEBUG_TYPE_PORTABILITY)
		debType = "portability";
	else if(type == GL_DEBUG_TYPE_PERFORMANCE)
		debType = "performance";
	else if(type == GL_DEBUG_TYPE_OTHER)
		debType = "message";
	else if(type == GL_DEBUG_TYPE_MARKER)
		debType = "marker";
	else if(type == GL_DEBUG_TYPE_PUSH_GROUP)
		debType = "push group";
	else if(type == GL_DEBUG_TYPE_POP_GROUP)
		debType = "pop group";
	else
		assert(0);
 
	if(severity == GL_DEBUG_SEVERITY_HIGH_ARB)
		debSev = "high";
	else if(severity == GL_DEBUG_SEVERITY_MEDIUM_ARB)
		debSev = "medium";
	else if(severity == GL_DEBUG_SEVERITY_LOW_ARB)
		debSev = "low";
	else if (severity == GL_DEBUG_SEVERITY_NOTIFICATION)
		debSev = "notification";
	else
		assert(0);

	ZazenGraphics::getInstance().getLogger().logError() << "OpenGL Debug-Context Output:\n    " << debSource << ": " << debType << "(" << debSev << ")" << id << ": " << message << "\n";
}

bool
GLUtils::peekErrorsSilent()
{
	GLenum status;
	bool hasErrors = false;

	while ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		hasErrors = true;
	}

	return hasErrors;
}

bool
GLUtils::peekErrors( const std::string& locationInfo )
{
	GLenum status;
	ostringstream errors;
	unsigned int errorCount = 0;

	errors << endl;

	while ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		errors << "    " << errorCount + 1;
		errors << ": \'";
		errors << ( const char* ) gluErrorString( status );
		errors << "\'";
		errors << endl;

		errorCount++;
	}

	if ( errorCount )
	{
		errors << "    @ " << locationInfo.c_str() << endl;
		ZazenGraphics::getInstance().getLogger().logError() << "Detected " << errorCount << " OpenGL-Error(s) "  << errors.str();

		return true;
	}

	return false;
}

void
GLUtils::enableDebugOutput()
{
#ifdef _DEBUG
	glEnable( GL_DEBUG_OUTPUT_SYNCHRONOUS );
	glDebugMessageControl( GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, GL_TRUE );
	glDebugMessageCallback( &debugOutput, NULL );
#endif
}
