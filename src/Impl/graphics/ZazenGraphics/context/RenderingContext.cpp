#include "RenderingContext.h"

#include <GL/glew.h>
#include <GL/wglew.h>

#include <iostream>
#include <vector>

using namespace std;

#define WINDOW_BITS_PER_PIXEL 32

RenderingContext* RenderingContext::instance = NULL;

bool
RenderingContext::initialize( const std::string& title, int width, int height, bool fullScreenFlag )
{
	new RenderingContext();

	// register new OpenGL window class
	if ( false == RenderingContext::registerClass( GetModuleHandle( NULL ) ) )
	{
		RenderingContext::shutdown();
		return false;
	}
	
	// create window (with according device context & pixelformat )
	if ( false == RenderingContext::createWindow( width, height, fullScreenFlag, title ) )
	{
		RenderingContext::shutdown();
		return false;
	}
	
	// create basic OpenGL compatibility rendering-context & initialize GLEW => fetched function pointers for creating core rendering-context
	if ( false == RenderingContext::createBaseRenderingContext() )
	{
		RenderingContext::shutdown();
		return false;
	}

	/* WARNING: seems to crash on my ATI Radeon HD 6650M at SOME point...
	// create core rendering context
	if ( false == RenderingContext::createCoreRenderingContext() )
	{
		RenderingContext::shutdown();
		return false;
	}
	*/

	// show and size window
	ShowWindow( RenderingContext::instance->hWnd, SW_SHOW );					
	SetForegroundWindow( RenderingContext::instance->hWnd );
	SetFocus( RenderingContext::instance->hWnd );
	RenderingContext::instance->resize( width, height );

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

	// clean-up in reversed order
	// destroy OpenGL rendering-context
	RenderingContext::destroyRenderingContext();
	// destroy window (& device context)
	RenderingContext::destroyWindow();
	// unregister OpenGL window-class
	RenderingContext::unregisterClass();

	// delete singleton instance 
	delete RenderingContext::instance;

	return true;
}

bool
RenderingContext::registerClass( HINSTANCE hInstance )
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
		cout << "ERROR ... in RenderingContext::registerClass: Failed To Register The Window Class." << endl;
		return false;
	}

	// successful in registering the class, set hInstance
	RenderingContext::instance->hInstance = hInstance;

	return true;
}

bool
RenderingContext::createWindow( int width, int height, bool fullScreenFlag, const std::string& title )
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
			cout << "WARNING ... in RenderingContext::createWindow: The Requested Fullscreen Mode Is Not Supported By Your Video Card, fallback to windowed mode." << endl;
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
								RenderingContext::instance->hInstance,
								NULL ); 
	if ( NULL == hWnd )
	{
		cout << "ERROR ... in RenderingContext::createWindow: Failed To Register The Window Class." << endl;
		return false;
	}

	// creating the window succeeded
	RenderingContext::instance->hWnd = hWnd;

	hDC = GetDC( RenderingContext::instance->hWnd );
	if ( NULL == hDC )
	{
		cout << "ERROR ... in RenderingContext::createWindow: Can't Create A GL Device Context." << endl;
		return false;
	}

	RenderingContext::instance->hDC = hDC;

	RenderingContext::instance->m_windowWidth = width;
	RenderingContext::instance->m_windowHeight = height;
	RenderingContext::instance->m_fullScreen = fullScreenFlag;

	return true;
}

bool
RenderingContext::createBaseRenderingContext()
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

	pixelFormat = ChoosePixelFormat( RenderingContext::instance->hDC, &pfd );
	if ( 0 == pixelFormat )
	{
		cout << "ERROR ... in RenderingContext::createBaseRenderingContext: Can't Find A Suitable PixelFormat." << endl;
		return false;
	}

	if ( false == SetPixelFormat( RenderingContext::instance->hDC, pixelFormat, &pfd ) )
	{
		cout << "ERROR ... in RenderingContext::createBaseRenderingContext: Can't Set The PixelFormat." << endl;
		return false;
	}

	HGLRC hRC = wglCreateContext( RenderingContext::instance->hDC );
	if ( NULL == hRC )
	{
		cout << "ERROR ... in RenderingContext::createBaseRenderingContext: Can't Create A GL Rendering Context." << endl;
		return false;
	}

	RenderingContext::instance->hRC = hRC;

	if( 0 == wglMakeCurrent( RenderingContext::instance->hDC, RenderingContext::instance->hRC ) )
	{
		cout << "ERROR ... in RenderingContext::createBaseRenderingContext: Can't Activate The GL Rendering Context." << endl;
		return false;
	}

	glewExperimental = TRUE;
	GLenum err = glewInit();
	if ( GLEW_OK != err )
	{
		cout << "ERROR ... in RenderingContext::createBaseRenderingContext: GLEW failed with " <<  glewGetErrorString( err ) << endl;
		return false;
	}

	return true;
}

bool
RenderingContext::createCoreRenderingContext()
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

	// WARNING: need to set far higher than really necessary because it seems
	// that wglCreateContextAttribsARB is manipulating it somehow inside
	// which leads to a stack-corruption and a crash of the program if not done
	// found after exhausting search in forums and stackoverflow: this was a bug in GLFW too
	std::vector<int> attribs( 20 );
	attribs[ 0 ] = WGL_CONTEXT_MAJOR_VERSION_ARB;
	attribs[ 1 ] = majorVersion;
	attribs[ 2 ] = WGL_CONTEXT_MINOR_VERSION_ARB;
	attribs[ 3 ] = minorVersion;
	attribs[ 4 ] = WGL_CONTEXT_PROFILE_MASK_ARB;
	attribs[ 5 ] = WGL_CONTEXT_CORE_PROFILE_BIT_ARB;
	
	if ( 0 == wglChoosePixelFormatARB( RenderingContext::instance->hDC, iPixelFormatAttribList, NULL, 
		1, &iPixelFormat, ( UINT* ) &iNumFormats ) )
	{
		cout << "ERROR ... in RenderingContext::createCoreRenderingContext: Failed choosing pixel format." << endl;
		return false;
	}

	if ( false == SetPixelFormat( RenderingContext::instance->hDC, iPixelFormat, &pfd ) )
	{
		cout << "ERROR ... in RenderingContext::createWindow: Can't Set The PixelFormat." << endl;
		return false;
	}

	hRC = wglCreateContextAttribsARB( RenderingContext::instance->hDC, 0, static_cast<int*>( &attribs[0] ) );
	if ( NULL == hRC )
	{
		cout << "ERROR ... in RenderingContext::createCoreRenderingContext: Can't Create A GL Rendering Context." << endl;
		return false;
	}

	// destroy base-context before switching to new core-context
	if ( false == RenderingContext::destroyRenderingContext() )
	{
		cout << "ERROR ... in RenderingContext::createCoreRenderingContext: Couldn't delete base-context." << endl;
		return false;
	}

	RenderingContext::instance->hRC = hRC;

	if( 0 == wglMakeCurrent( RenderingContext::instance->hDC, RenderingContext::instance->hRC ) )
	{
		cout << "ERROR ... in RenderingContext::createCoreRenderingContext: Can't Activate The GL Rendering Context." << endl;
		return false;
	}
	
	glewExperimental = TRUE;
	GLenum err = glewInit();
	if ( GLEW_OK != err )
	{
		cout << "ERROR ... in RenderingContext::createCoreRenderingContext: GLEW failed with " <<  glewGetErrorString( err ) << endl;
		return false;
	}

	return true;
}

bool
RenderingContext::unregisterClass()
{
	if ( NULL != RenderingContext::instance->hInstance )
	{
		if ( 0 == UnregisterClass( "OpenGL", RenderingContext::instance->hInstance ) )
		{
			cout << "WARNING ... in RenderingContext::unregisterClass: Could Not Unregister Class." << endl;		
		}

		RenderingContext::instance->hInstance = NULL;
	}

	return true;
}

bool
RenderingContext::destroyRenderingContext()
{
	if ( NULL != RenderingContext::instance->hRC )
	{
		if ( 0 == wglMakeCurrent( NULL, NULL ) )	
		{
			cout << "WARNING ... in RenderingContext::destroyRenderingContext: wglMakeCurrent failed." << endl;
		}

		if ( 0 == wglDeleteContext( RenderingContext::instance->hRC ) )
		{
			cout << "WARNING ... in RenderingContext::destroyRenderingContext: wglDeleteContext failed." << endl;
		}

		RenderingContext::instance->hRC = NULL;
	}

	return true;
}

bool
RenderingContext::destroyWindow()
{
	if ( NULL != RenderingContext::instance->hDC && NULL != RenderingContext::instance->hWnd )
	{
		if ( 0 == ReleaseDC( RenderingContext::instance->hWnd,RenderingContext::instance->hDC ) )
		{
			cout << "WARNING ... in RenderingContext::destroyWindow: Release Device Context Failed." << endl;
		}

		RenderingContext::instance->hDC = NULL;
	}

	if ( NULL != RenderingContext::instance->hWnd )
	{
		// when in fullscreen, un-fullscreen the window
		if ( RenderingContext::instance->m_fullScreen )	
		{
			ChangeDisplaySettings( NULL, 0 );
			ShowCursor( TRUE );
		}

		if ( 0 == DestroyWindow( RenderingContext::instance->hWnd ) )
		{
			cout << "WARNING ... in RenderingContext::destroyWindow: Destroy Window Failed." << endl;
		}

		RenderingContext::instance->hWnd = NULL;
	}

	return true;
}

RenderingContext::RenderingContext()
{
	this->m_windowWidth = 800;
	this->m_windowHeight = 600;
	this->m_fullScreen = false;

	this->hDC = NULL;
	this->hRC = NULL;
	this->hWnd = NULL;
	this->hInstance = NULL;

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
	SwapBuffers( RenderingContext::instance->hDC );

	return true;
}

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
