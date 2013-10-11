#include "GLUtils.h"

#include "../ZazenGraphics.h"

#include <GL/glew.h>

#include <Windows.h>

#include <sstream>

using namespace std;

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
