#include "RenderingWindow.h"

#include <GL/glew.h>
#include <GL/wglew.h>

#include <iostream>

using namespace std;

#define WINDOW_BITS_PER_PIXEL 32

RenderingWindow* RenderingWindow::instance = NULL;

bool
RenderingWindow::initialize( const std::string& title, int width, int height, bool fullScreenFlag )
{
	new RenderingWindow();
	
	// register new OpenGL window class
	if ( false == RenderingWindow::registerClass( GetModuleHandle( NULL ) ) )
	{
		RenderingWindow::shutdown();
		return false;
	}
	
	// create window (with according device context & pixelformat )
	if ( false == RenderingWindow::createWindow( width, height, fullScreenFlag, title ) )
	{
		RenderingWindow::shutdown();
		return false;
	}
	
	// create basic OpenGL compatibility rendering-context & initialize GLEW => fetched function pointers for createing core rendering-context
	if ( false == RenderingWindow::createCompatibilityRenderingContext() )
	{
		RenderingWindow::shutdown();
		return false;
	}

	/* uncomment for core-context
	// destroy rendering context because we will set up a new ( core ) one
	if ( false == RenderingWindow::destroyRenderingContext() )
	{
		RenderingWindow::shutdown();
		return false;
	}
	
	// destroy the created window
	if ( false == RenderingWindow::destroyWindow() )
	{
		RenderingWindow::shutdown();
		return false;
	}
	
	// create window again with then new core rendering-context
	if ( false == RenderingWindow::createWindow( width, height, fullScreenFlag, title ) )
	{
		RenderingWindow::shutdown();
		return false;
	}

	// create core rendering context
	if ( false == RenderingWindow::createCoreRenderingContext() )
	{
		RenderingWindow::shutdown();
		return false;
	}
	*/

	// show and size window
	ShowWindow( RenderingWindow::instance->hWnd, SW_SHOW );					
	SetForegroundWindow( RenderingWindow::instance->hWnd );
	SetFocus( RenderingWindow::instance->hWnd );
	RenderingWindow::instance->resize( width, height );

	return true;
}

bool
RenderingWindow::shutdown()
{
	// not initialize, silent fail
	if ( NULL == RenderingWindow::instance )
	{
		return true;
	}

	// clean-up in reversed order
	// destroy OpenGL rendering-context
	RenderingWindow::destroyRenderingContext();
	// destroy window (& device context)
	RenderingWindow::destroyWindow();
	// unregister OpenGL window-class
	RenderingWindow::unregisterClass();

	// delete singleton instance 
	delete RenderingWindow::instance;

	return true;
}

bool
RenderingWindow::registerClass( HINSTANCE hInstance )
{
	WNDCLASS wc;						
	wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;		// Redraw On Size, And Own DC For Window.
	wc.lpfnWndProc = (WNDPROC) WndProc;					// WndProc Handles Messages
	wc.cbClsExtra = 0;									// No Extra Window Data
	wc.cbWndExtra = 0;									// No Extra Window Data
	wc.hInstance = hInstance;							// Set The Instance
	wc.hIcon = LoadIcon( NULL, IDI_WINLOGO );			// Load The Default Icon
	wc.hCursor = LoadCursor( NULL, IDC_ARROW );			// Load The Arrow Pointer
	wc.hbrBackground = NULL;							// No Background Required For GL
	wc.lpszMenuName	= NULL;								// We Don't Want A Menu
	wc.lpszClassName = "OpenGL";						// Set The Class Name

	if ( 0 == RegisterClass( &wc ) )
	{
		cout << "ERROR ... in RenderingWindow::registerClass: Failed To Register The Window Class." << endl;
		return false;
	}

	// successful in registering the class, set hInstance
	RenderingWindow::instance->hInstance = hInstance;

	return true;
}

bool
RenderingWindow::createWindow( int width, int height, bool fullScreenFlag, const std::string& title )
{
	HWND hWnd = NULL;
	HDC hDC = NULL;
	DWORD dwExStyle;		
	DWORD dwStyle;	
	RECT WindowRect;

	WindowRect.left = ( long ) 0;			
	WindowRect.right = ( long ) width;		
	WindowRect.top = ( long ) 0;				
	WindowRect.bottom = ( long ) height;

	if ( fullScreenFlag )
	{
		DEVMODE dmScreenSettings;
		memset( &dmScreenSettings, 0, sizeof( dmScreenSettings ) );
		dmScreenSettings.dmSize = sizeof( dmScreenSettings );
		dmScreenSettings.dmPelsWidth = width;
		dmScreenSettings.dmPelsHeight = height;
		dmScreenSettings.dmBitsPerPel = WINDOW_BITS_PER_PIXEL;
		dmScreenSettings.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT;

		// Try To Set Selected Mode And Get Results.  NOTE: CDS_FULLSCREEN Gets Rid Of Start Bar.
		if ( ChangeDisplaySettings( &dmScreenSettings, CDS_FULLSCREEN ) != DISP_CHANGE_SUCCESSFUL )
		{
			cout << "WARNING ... in RenderingWindow::createWindow: The Requested Fullscreen Mode Is Not Supported By Your Video Card, fallback to windowed mode." << endl;
			fullScreenFlag = false;
		}
	}

	if ( fullScreenFlag )
	{
		dwExStyle = WS_EX_APPWINDOW;					
		dwStyle = WS_POPUP;	
		ShowCursor( FALSE );
	}
	else
	{
		dwExStyle = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;	
		dwStyle = WS_OVERLAPPEDWINDOW;
	}

	// Adjust Window To True Requested Size
	AdjustWindowRectEx( &WindowRect, dwStyle, FALSE, dwExStyle );

	// create the window
	hWnd = CreateWindowEx( dwExStyle,
								"OpenGL",
								title.c_str(),
								dwStyle |
								WS_CLIPSIBLINGS |
								WS_CLIPCHILDREN,
								0, 0,
								WindowRect.right-WindowRect.left,
								WindowRect.bottom-WindowRect.top,
								NULL,
								NULL,
								RenderingWindow::instance->hInstance,
								NULL ); 
	if ( NULL == hWnd )
	{
		cout << "ERROR ... in RenderingWindow::createWindow: Failed To Register The Window Class." << endl;
		return false;
	}

	// creating the window succeeded
	RenderingWindow::instance->hWnd = hWnd;

	hDC = GetDC( RenderingWindow::instance->hWnd );
	if ( NULL == hDC )
	{
		cout << "ERROR ... in RenderingWindow::createWindow: Can't Create A GL Device Context." << endl;
		return false;
	}

	RenderingWindow::instance->hDC = hDC;

	RenderingWindow::instance->m_windowWidth = width;
	RenderingWindow::instance->m_windowHeight = height;
	RenderingWindow::instance->m_fullScreen = fullScreenFlag;

	return true;
}

bool
RenderingWindow::createCompatibilityRenderingContext()
{
	GLuint pixelFormat;

	static PIXELFORMATDESCRIPTOR pfd =	
	{
		sizeof( PIXELFORMATDESCRIPTOR ),			// Size Of This Pixel Format Descriptor
		1,											// Version Number
		PFD_DRAW_TO_WINDOW |						// Format Must Support Window
		PFD_SUPPORT_OPENGL |						// Format Must Support OpenGL
		PFD_DOUBLEBUFFER,							// Must Support Double Buffering
		PFD_TYPE_RGBA,								// Request An RGBA Format
		WINDOW_BITS_PER_PIXEL,						// Select Our Color Depth
		0, 0, 0, 0, 0, 0,							// Color Bits Ignored
		0,											// No Alpha Buffer
		0,											// Shift Bit Ignored
		0,											// No Accumulation Buffer
		0, 0, 0, 0,									// Accumulation Bits Ignored
		32,											// 32Bit Z-Buffer (Depth Buffer)  
		8,											// 8 bit Stencil Buffer
		0,											// No Auxiliary Buffer
		PFD_MAIN_PLANE,								// Main Drawing Layer
		0,											// Reserved
		0, 0, 0										// Layer Masks Ignored
	};

	pixelFormat = ChoosePixelFormat( RenderingWindow::instance->hDC, &pfd );
	if ( 0 == pixelFormat )
	{
		cout << "ERROR ... in RenderingWindow::createWindow: Can't Find A Suitable PixelFormat." << endl;
		return false;
	}

	if ( false == SetPixelFormat( RenderingWindow::instance->hDC, pixelFormat, &pfd ) )
	{
		cout << "ERROR ... in RenderingWindow::createWindow: Can't Set The PixelFormat." << endl;
		return false;
	}

	HGLRC hRC = wglCreateContext( RenderingWindow::instance->hDC );
	if ( NULL == hRC )
	{
		cout << "ERROR ... in RenderingWindow::createCompatibilityRenderingContext: Can't Create A GL Rendering Context." << endl;
		return false;
	}

	RenderingWindow::instance->hRC = hRC;

	if( 0 == wglMakeCurrent( RenderingWindow::instance->hDC, RenderingWindow::instance->hRC ) )
	{
		cout << "ERROR ... in RenderingWindow::createCompatibilityRenderingContext: Can't Activate The GL Rendering Context." << endl;
		return false;
	}

	GLenum err = glewInit();
	if ( GLEW_OK != err )
	{
		cout << "ERROR ... in RenderingWindow::createCompatibilityRenderingContext: GLEW failed with " <<  glewGetErrorString( err ) << endl;
		return false;
	}

	return true;
}

bool
RenderingWindow::createCoreRenderingContext()
{
	HGLRC hRC = NULL;
	int majorVersion = 3;
	int minorVersion = 3;
	PIXELFORMATDESCRIPTOR pfd;
	int iPixelFormat, iNumFormats;

	const int iPixelFormatAttribList[] =
	{
		WGL_DRAW_TO_WINDOW_ARB, GL_TRUE,
		WGL_SUPPORT_OPENGL_ARB, GL_TRUE,
		WGL_DOUBLE_BUFFER_ARB, GL_TRUE,
		WGL_PIXEL_TYPE_ARB, WGL_TYPE_RGBA_ARB,
		WGL_COLOR_BITS_ARB, 32,
		WGL_DEPTH_BITS_ARB, 24,
		WGL_STENCIL_BITS_ARB, 8,
		0 // End of attributes list
	};

	int iContextAttribs[] =
	{
		WGL_CONTEXT_MAJOR_VERSION_ARB, majorVersion,
		WGL_CONTEXT_MINOR_VERSION_ARB, minorVersion,
		WGL_CONTEXT_FLAGS_ARB, WGL_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB, 
		//WGL_CONTEXT_FLAGS_ARB, WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
		0 // End of attributes list
	};

	if ( 0 == wglChoosePixelFormatARB( RenderingWindow::instance->hDC, iPixelFormatAttribList, NULL, 
		1, &iPixelFormat, ( UINT* ) &iNumFormats ) )
	{
		cout << "ERROR ... in RenderingWindow::createCoreRenderingContext: Failed choosing pixel format." << endl;
		return false;
	}

	if ( false == SetPixelFormat( RenderingWindow::instance->hDC, iPixelFormat, &pfd ) )
	{
		cout << "ERROR ... in RenderingWindow::createWindow: Can't Set The PixelFormat." << endl;
		return false;
	}

	hRC = wglCreateContextAttribsARB( RenderingWindow::instance->hDC, 0, iContextAttribs );
	if ( NULL == hRC )
	{
		cout << "ERROR ... in RenderingWindow::createCoreRenderingContext: Can't Create A GL Rendering Context." << endl;
		return false;
	}

	RenderingWindow::instance->hRC = hRC;

	if( 0 == wglMakeCurrent( RenderingWindow::instance->hDC, RenderingWindow::instance->hRC ) )
	{
		cout << "ERROR ... in RenderingWindow::createCoreRenderingContext: Can't Activate The GL Rendering Context." << endl;
		return false;
	}

	/*
	// re-init glew: refetch function pointers because of new context
	GLenum err = glewInit();
	if ( GLEW_OK != err )
	{
		cout << "ERROR ... in RenderingWindow::createCoreRenderingContext: GLEW failed with " <<  glewGetErrorString( err ) << endl;
		return false;
	}
	*/

	return true;
}

bool
RenderingWindow::unregisterClass()
{
	if ( NULL != RenderingWindow::instance->hInstance )
	{
		if ( 0 == UnregisterClass( "OpenGL", RenderingWindow::instance->hInstance ) )
		{
			cout << "WARNING ... in RenderingWindow::unregisterClass: Could Not Unregister Class." << endl;		
		}

		RenderingWindow::instance->hInstance = NULL;
	}

	return true;
}

bool
RenderingWindow::destroyRenderingContext()
{
	if ( NULL != RenderingWindow::instance->hRC )
	{
		if ( 0 == wglMakeCurrent( NULL, NULL ) )	
		{
			cout << "WARNING ... in RenderingWindow::unregisterClass: Release Of DC And RC Failed." << endl;
		}

		if ( 0 == wglDeleteContext( RenderingWindow::instance->hRC ) )
		{
			cout << "WARNING ... in RenderingWindow::unregisterClass: Release Rendering Context Failed." << endl;
		}

		RenderingWindow::instance->hRC = NULL;
	}

	return true;
}

bool
RenderingWindow::destroyWindow()
{
	if ( NULL != RenderingWindow::instance->hDC && NULL != RenderingWindow::instance->hWnd )
	{
		if ( 0 == ReleaseDC( RenderingWindow::instance->hWnd,RenderingWindow::instance->hDC ) )
		{
			cout << "WARNING ... in RenderingWindow::destroyWindow: Release Device Context Failed." << endl;
		}

		RenderingWindow::instance->hDC = NULL;
	}

	if ( NULL != RenderingWindow::instance->hWnd )
	{
		// when in fullscreen, un-fullscreen the window
		if ( RenderingWindow::instance->m_fullScreen )	
		{
			ChangeDisplaySettings( NULL, 0 );
			ShowCursor( TRUE );
		}

		if ( 0 == DestroyWindow( RenderingWindow::instance->hWnd ) )
		{
			cout << "WARNING ... in RenderingWindow::destroyWindow: Destroy Window Failed." << endl;
		}

		RenderingWindow::instance->hWnd = NULL;
	}

	return true;
}

RenderingWindow::RenderingWindow()
{
	this->m_windowWidth = 800;
	this->m_windowHeight = 600;
	this->m_fullScreen = false;

	this->hDC = NULL;
	this->hRC = NULL;
	this->hWnd = NULL;
	this->hInstance = NULL;

	this->m_activeFlag = true;

	RenderingWindow::instance = this;
}

RenderingWindow::~RenderingWindow()
{
	RenderingWindow::instance = NULL;
}

bool
RenderingWindow::toggleFullscreen()
{
	// TODO implement

	return true;
}

void
RenderingWindow::resize( int width, int height )
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
RenderingWindow::swapBuffers()
{
	SwapBuffers( RenderingWindow::instance->hDC );

	return true;
}

LRESULT CALLBACK
RenderingWindow::WndProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	switch ( uMsg ) 
	{
		case WM_ACTIVATE:
		{
			if ( 0 == HIWORD( wParam ) )
			{
				RenderingWindow::instance->m_activeFlag = true;
			}
			else
			{
				RenderingWindow::instance->m_activeFlag = false;
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
			RenderingWindow::instance->resize( LOWORD( lParam ), HIWORD( lParam ) );
			return 0;
		}
	}

	// Pass All Unhandled Messages To DefWindowProc
	return DefWindowProc( hWnd, uMsg, wParam, lParam );
}
