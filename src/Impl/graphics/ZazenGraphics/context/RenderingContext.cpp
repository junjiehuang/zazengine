#include "RenderingContext.h"

#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL

#include <GLFW/glfw3native.h>

#include "../ZazenGraphics.h"
#include "../util/GLUtils.h"
#include "../texture/TextureFactory.h"

using namespace std;
using namespace boost::filesystem;

#define REQUIRED_MAJOR_OPENGL_VER 4
#define REQUIRED_MINOR_OPENGL_VER 2

#define WINDOW_BITS_PER_PIXEL 32

RenderingContext* RenderingContext::instance = NULL;

bool
RenderingContext::initialize( const std::string& title, int width, int height, bool fullScreenFlag, path screenShotPath )
{
	new RenderingContext( title, width, height, fullScreenFlag, screenShotPath );

	if ( false == RenderingContext::initGLFW() )
	{
		RenderingContext::shutdown();
        return false;
	}

	if ( false == RenderingContext::initGlew() )
	{
		RenderingContext::shutdown();
        return false;
	}

	// always enable debugging, this is the way to detect errors in release-build
	// calling glGetError every frame is too costly (about 33us)
	if ( false == RenderingContext::initDebugging() )
	{
		RenderingContext::shutdown();
        return false;
	}

	return true;
}

bool
RenderingContext::initGLFW()
{
	/* Initialize the library */
    if ( false == glfwInit() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "RenderingContext::initGLFW: init GLFW failed";
        return false;
	}

	glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );

	glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, REQUIRED_MAJOR_OPENGL_VER );
	glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, REQUIRED_MINOR_OPENGL_VER );

#ifdef _DEBUG
	// create a debug-context for debug-build
	glfwWindowHint( GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE );
#endif

    /* Create a windowed mode window and its OpenGL context */
	RenderingContext::instance->m_window = glfwCreateWindow( RenderingContext::instance->m_windowWidth, RenderingContext::instance->m_windowHeight, 
		RenderingContext::instance->m_windowTitle.c_str(), NULL, NULL );
    if ( NULL == RenderingContext::instance->m_window )
    {
		ZazenGraphics::getInstance().getLogger().logError() << "RenderingContext::initGLFW: creating window failed";
        return false;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent( RenderingContext::instance->m_window );

	RenderingContext::instance->m_hWnd = glfwGetWin32Window( RenderingContext::instance->m_window );
	if ( NULL == RenderingContext::instance->m_hWnd )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "RenderingContext::initGLFW: retrieving HWND failed";
		return false;
	}

	return true;
}

bool
RenderingContext::initGlew()
{
	GLenum err = 0;

	glewExperimental = GL_TRUE;
	err = glewInit();
	if ( GLEW_OK != err )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "RenderingContext::initGlew: GLEW failed with " << glewGetErrorString( err );
		return false;
	}

	// need to peek GL-Errors because a bug in glew will cause INVALID_ENUM when a profile > 3.2 is requested
	GL_PEEK_ERRORS_SILENT

	return true;
}

bool
RenderingContext::initDebugging()
{
	glEnable( GL_DEBUG_OUTPUT_SYNCHRONOUS_ARB );
	glDebugMessageControlARB( GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, GL_TRUE );
	glDebugMessageCallbackARB( &RenderingContext::debugOutput, NULL );

	return true;
}

bool
RenderingContext::shutdown()
{
	// not initialize, silent fail
	if ( NULL == RenderingContext::instance )
	{
		return true;
	}

	if ( NULL != RenderingContext::instance->m_window )
	{
		glfwTerminate();
	}

	// delete singleton instance 
	delete RenderingContext::instance;

	return true;
}

RenderingContext::RenderingContext( const std::string& title, int width, int height, bool fullScreenFlag, path screenShotPath )
{
	this->m_windowTitle = title;
	this->m_windowWidth = width;
	this->m_windowHeight = height;
	this->m_fullScreen = fullScreenFlag;

	this->m_window = NULL;
	this->m_hWnd = NULL;

	this->m_activeFlag = true;

	this->m_screnShotFlag = false;
	this->m_screenShotPath = screenShotPath;

	RenderingContext::instance = this;
}

RenderingContext::~RenderingContext()
{
	RenderingContext::instance = NULL;
}

bool
RenderingContext::toggleFullscreen()
{
	// TODO implement

	return true;
}

void
RenderingContext::resize( int width, int height )
{
	// special case for height: prevent divide by 0 in OpenGL
	if ( 0 == height )
	{
		height = 1;
	}

	glViewport( 0,0, width, height );

	// TODO resize camera
	// TODO reinit renderer

	// TODO refactor: need to do a callback
}

bool
RenderingContext::swapBuffers()
{
	glfwSwapBuffers( RenderingContext::instance->m_window );

	// take screenshot after buffer swapped
	if ( this->m_screnShotFlag )
	{
		unsigned int index = 0;
		stringstream fileName;
		
		fileName << this->m_screenShotPath.generic_string() << "screenshot_";

		for ( directory_iterator iter( this->m_screenShotPath ); iter != directory_iterator(); ++iter )
		{
		  if ( is_regular_file( iter->status() ) )
		  {
			  index++;
		  }
		}

		fileName << index;
		fileName << ".png";

		TextureFactory::captureScreen( fileName.str().c_str() );

		this->m_screnShotFlag = false;
	}

	return true;
}

bool
RenderingContext::takeScreenShot()
{
	// post-pone screenshot after swapping of buffers
	this->m_screnShotFlag = true;

	return true;
}

void APIENTRY 
RenderingContext::debugOutput ( GLenum source,
								GLenum type,
								GLuint id,
								GLenum severity,
								GLsizei length,
								const GLchar* message,
								GLvoid* userParam )
{
	std::ostringstream str;

	str << "OpenGL Debug-Context Output:\n    ";

	if( GL_DEBUG_SOURCE_API_ARB == source )
		str << "OpenGL";
	else if( GL_DEBUG_SOURCE_WINDOW_SYSTEM_ARB == source )
		str << "Windows";
	else if( GL_DEBUG_SOURCE_SHADER_COMPILER_ARB == source )
		str << "Shader Compiler";
	else if( GL_DEBUG_SOURCE_THIRD_PARTY_ARB == source )
		str << "Third Party";
	else if( GL_DEBUG_SOURCE_APPLICATION_ARB == source )
		str << "Application";
	else if ( GL_DEBUG_SOURCE_OTHER_ARB == source )
		str << "Other";
	else
		str << "N/A";
 
	str << ": ";

	if( GL_DEBUG_TYPE_ERROR == type )
		str << "error";
	else if( GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR == type )
		str << "deprecated behavior";
	else if( GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR == type )
		str << "undefined behavior";
	else if( GL_DEBUG_TYPE_PORTABILITY == type )
		str << "portability";
	else if( GL_DEBUG_TYPE_PERFORMANCE == type )
		str << "performance";
	else if( GL_DEBUG_TYPE_OTHER == type )
		str << "message";
	else if( GL_DEBUG_TYPE_MARKER == type )
		str << "marker";
	else if( GL_DEBUG_TYPE_PUSH_GROUP == type )
		str << "push group";
	else if( GL_DEBUG_TYPE_POP_GROUP == type )
		str << "pop group";
	else
		str << "N/A";
 
	str << "(";

	if( GL_DEBUG_SEVERITY_HIGH_ARB == severity )
		str << "high";
	else if( GL_DEBUG_SEVERITY_MEDIUM_ARB == severity )
		str << "medium";
	else if( GL_DEBUG_SEVERITY_LOW_ARB == severity )
		str << "low";
	else if ( GL_DEBUG_SEVERITY_NOTIFICATION == severity )
		str << "notification";
	else
		str << "N/A";

	str << ") ";
	str << id;
	str << ": ";
	str << message;

	if ( message[ length - 1 ] != '\n' )
	{
		str << "\n";
	}
	
	if ( GL_DEBUG_TYPE_ERROR == type )
	{
		ZazenGraphics::getInstance().getLogger().logError() << str.str();
	}
	else if ( GL_DEBUG_SEVERITY_MEDIUM_ARB == type )
	{
		ZazenGraphics::getInstance().getLogger().logWarning() << str.str();
	}
	else if ( GL_DEBUG_SEVERITY_LOW_ARB == type )
	{
		ZazenGraphics::getInstance().getLogger().logDebug() << str.str();
	}
	else if ( GL_DEBUG_SEVERITY_NOTIFICATION == type )
	{
		ZazenGraphics::getInstance().getLogger().logDebug() << str.str();
	}
}
