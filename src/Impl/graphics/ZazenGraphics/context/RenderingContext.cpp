#include "RenderingContext.h"

#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL

#include <GLFW/glfw3native.h>

#include "../ZazenGraphics.h"

using namespace std;

#define WINDOW_BITS_PER_PIXEL 32

RenderingContext* RenderingContext::instance = NULL;

bool
RenderingContext::initialize( const std::string& title, int width, int height, bool fullScreenFlag )
{
	new RenderingContext( title, width, height, fullScreenFlag );

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
	glewExperimental = TRUE;
	GLenum err = glewInit();
	if ( GLEW_OK != err )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "RenderingContext::initGlew: GLEW failed with " << glewGetErrorString( err );
		return false;
	}

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

RenderingContext::RenderingContext( const std::string& title, int width, int height, bool fullScreenFlag )
{
	this->m_windowTitle = title;
	this->m_windowWidth = width;
	this->m_windowHeight = height;
	this->m_fullScreen = fullScreenFlag;

	this->m_window = NULL;
	this->m_hWnd = NULL;

	this->m_activeFlag = true;

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