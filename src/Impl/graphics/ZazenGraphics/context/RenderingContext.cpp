#include "RenderingContext.h"

#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL

#include <GLFW/glfw3native.h>

#include "../ZazenGraphics.h"
#include "../util/GLUtils.h"
#include "../texture/TextureFactory.h"

using namespace std;
using namespace boost::filesystem;

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

	/*
	glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 4 );
	glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 3 );
	glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_DEBUG_CONTEXT );
	*/

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
		fileName << ".tiff";

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

/*
LRESULT CALLBACK
RenderingContext::WndProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	switch ( uMsg ) 
	{
		case WM_ACTIVATE:
		{
			if ( 0 == HIWORD( wParam ) )
			{
				RenderingContext::instance->m_activeFlag = true;
			}
			else
			{
				RenderingContext::instance->m_activeFlag = false;
			}

			return 0;
		}

		case WM_SYSCOMMAND:
		{
			switch ( wParam )
			{
				case SC_SCREENSAVE:
				case SC_MONITORPOWER:
				return 0;
			}

			break;
		}

		case WM_CLOSE:
		{
			PostQuitMessage( 0 );
			return 0;
		}

		case WM_SIZE:
		{
			RenderingContext::instance->resize( LOWORD( lParam ), HIWORD( lParam ) );
			return 0;
		}
	}

	// Pass All Unhandled Messages To DefWindowProc
	return DefWindowProc( hWnd, uMsg, wParam, lParam );
}
*/