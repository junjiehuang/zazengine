#include "GLUtils.h"

#include "../ZazenGraphics.h"

#include <GL/glew.h>

#include <Windows.h>

#include <string>
#include <sstream>

using namespace std;

bool
GLUtils::peekErrors()
{
	GLenum status;
	ostringstream errors;

	while ( GL_NO_ERROR != ( status = glGetError() ) )
	{
		errors << ( const char* ) gluErrorString( status ); 
		errors << endl;
	}

	if ( errors.rdbuf()->in_avail() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "GLUtils::peekErrors detected OpenGL-Errors: " << errors.str();
		return false;
	}

	return true;
}
