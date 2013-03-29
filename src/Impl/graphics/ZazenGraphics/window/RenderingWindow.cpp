#include "RenderingWindow.h"

#include <GL/glew.h>

#define WINDOW_BITS_PER_PIXEL 32

RenderingWindow* RenderingWindow::instance = NULL;

bool
RenderingWindow::createRenderingWindow( const std::string& title, int width, int height, bool fullScreenFlag )
{
	new RenderingWindow();
	RenderingWindow::instance->m_windowWidth = width;
	RenderingWindow::instance->m_windowHeight = height;

	GLuint PixelFormat;			// Holds The Results After Searching For A Match
	WNDCLASS wc;						// Windows Class Structure
	DWORD dwExStyle;				// Window Extended Style
	DWORD dwStyle;				// Window Style
	RECT WindowRect;				// Grabs Rectangle Upper Left / Lower Right Values
	WindowRect.left = ( long ) 0;			// Set Left Value To 0
	WindowRect.right = ( long ) width;		// Set Right Value To Requested Width
	WindowRect.top = ( long ) 0;				// Set Top Value To 0
	WindowRect.bottom = ( long ) height;		// Set Bottom Value To Requested Height

	RenderingWindow::instance->m_fullScreen = fullScreenFlag;

	RenderingWindow::instance->hInstance			= GetModuleHandle(NULL);				// Grab An Instance For Our Window
	wc.style			= CS_HREDRAW | CS_VREDRAW | CS_OWNDC;	// Redraw On Size, And Own DC For Window.
	wc.lpfnWndProc		= (WNDPROC) WndProc;					// WndProc Handles Messages
	wc.cbClsExtra		= 0;									// No Extra Window Data
	wc.cbWndExtra		= 0;									// No Extra Window Data
	wc.hInstance		= RenderingWindow::instance->hInstance;							// Set The Instance
	wc.hIcon			= LoadIcon(NULL, IDI_WINLOGO);			// Load The Default Icon
	wc.hCursor			= LoadCursor(NULL, IDC_ARROW);			// Load The Arrow Pointer
	wc.hbrBackground	= NULL;									// No Background Required For GL
	wc.lpszMenuName		= NULL;									// We Don't Want A Menu
	wc.lpszClassName	= "OpenGL";								// Set The Class Name

	if (!RegisterClass(&wc))									// Attempt To Register The Window Class
	{
		MessageBox(NULL,"Failed To Register The Window Class.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return false;											// Return FALSE
	}
	
	if ( RenderingWindow::instance->m_fullScreen )												// Attempt Fullscreen Mode?
	{
		DEVMODE dmScreenSettings;								// Device Mode
		memset(&dmScreenSettings,0,sizeof(dmScreenSettings));	// Makes Sure Memory's Cleared
		dmScreenSettings.dmSize=sizeof(dmScreenSettings);		// Size Of The Devmode Structure
		dmScreenSettings.dmPelsWidth	= width;				// Selected Screen Width
		dmScreenSettings.dmPelsHeight	= height;				// Selected Screen Height
		dmScreenSettings.dmBitsPerPel	= WINDOW_BITS_PER_PIXEL;					// Selected Bits Per Pixel
		dmScreenSettings.dmFields=DM_BITSPERPEL|DM_PELSWIDTH|DM_PELSHEIGHT;

		// Try To Set Selected Mode And Get Results.  NOTE: CDS_FULLSCREEN Gets Rid Of Start Bar.
		if (ChangeDisplaySettings(&dmScreenSettings,CDS_FULLSCREEN)!=DISP_CHANGE_SUCCESSFUL)
		{
			// If The Mode Fails, Offer Two Options.  Quit Or Use Windowed Mode.
			if (MessageBox(NULL,"The Requested Fullscreen Mode Is Not Supported By\nYour Video Card. Use Windowed Mode Instead?","NeHe GL",MB_YESNO|MB_ICONEXCLAMATION)==IDYES)
			{
				RenderingWindow::instance->m_fullScreen=FALSE;		// Windowed Mode Selected.  Fullscreen = FALSE
			}
			else
			{
				// Pop Up A Message Box Letting User Know The Program Is Closing.
				MessageBox(NULL,"Program Will Now Close.","ERROR",MB_OK|MB_ICONSTOP);
				return false;									// Return FALSE
			}
		}
	}

	if ( RenderingWindow::instance->m_fullScreen )												// Are We Still In Fullscreen Mode?
	{
		dwExStyle=WS_EX_APPWINDOW;								// Window Extended Style
		dwStyle=WS_POPUP;										// Windows Style
		ShowCursor(FALSE);										// Hide Mouse Pointer
	}
	else
	{
		dwExStyle=WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;			// Window Extended Style
		dwStyle=WS_OVERLAPPEDWINDOW;							// Windows Style
	}

	AdjustWindowRectEx(&WindowRect, dwStyle, FALSE, dwExStyle);		// Adjust Window To True Requested Size

	// Create The Window
	if (!(RenderingWindow::instance->hWnd=CreateWindowEx(	dwExStyle,							// Extended Style For The Window
								"OpenGL",							// Class Name
								title.c_str(),						// Window Title
								dwStyle |							// Defined Window Style
								WS_CLIPSIBLINGS |					// Required Window Style
								WS_CLIPCHILDREN,					// Required Window Style
								0, 0,								// Window Position
								WindowRect.right-WindowRect.left,	// Calculate Window Width
								WindowRect.bottom-WindowRect.top,	// Calculate Window Height
								NULL,								// No Parent Window
								NULL,								// No Menu
								RenderingWindow::instance->hInstance,							// Instance
								NULL)))								// Dont Pass Anything To WM_CREATE
	{
		RenderingWindow::destroyWindow();
		MessageBox(NULL,"Window Creation Error.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return false;								// Return FALSE
	}

	static	PIXELFORMATDESCRIPTOR pfd=				// pfd Tells Windows How We Want Things To Be
	{
		sizeof(PIXELFORMATDESCRIPTOR),				// Size Of This Pixel Format Descriptor
		1,											// Version Number
		PFD_DRAW_TO_WINDOW |						// Format Must Support Window
		PFD_SUPPORT_OPENGL |						// Format Must Support OpenGL
		PFD_DOUBLEBUFFER,							// Must Support Double Buffering
		PFD_TYPE_RGBA,								// Request An RGBA Format
		WINDOW_BITS_PER_PIXEL,										// Select Our Color Depth
		0, 0, 0, 0, 0, 0,							// Color Bits Ignored
		0,											// No Alpha Buffer
		0,											// Shift Bit Ignored
		0,											// No Accumulation Buffer
		0, 0, 0, 0,									// Accumulation Bits Ignored
		16,											// 16Bit Z-Buffer (Depth Buffer)  
		0,											// No Stencil Buffer
		0,											// No Auxiliary Buffer
		PFD_MAIN_PLANE,								// Main Drawing Layer
		0,											// Reserved
		0, 0, 0										// Layer Masks Ignored
	};
	
	if (!(RenderingWindow::instance->hDC=GetDC(RenderingWindow::instance->hWnd)))							// Did We Get A Device Context?
	{
		RenderingWindow::destroyWindow();
		MessageBox(NULL,"Can't Create A GL Device Context.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return false;								// Return FALSE
	}

	if (!(PixelFormat=ChoosePixelFormat(RenderingWindow::instance->hDC,&pfd)))	// Did Windows Find A Matching Pixel Format?
	{
		RenderingWindow::destroyWindow();
		MessageBox(NULL,"Can't Find A Suitable PixelFormat.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return false;								// Return FALSE
	}

	if(!SetPixelFormat(RenderingWindow::instance->hDC,PixelFormat,&pfd))		// Are We Able To Set The Pixel Format?
	{
		RenderingWindow::destroyWindow();
		MessageBox(NULL,"Can't Set The PixelFormat.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return false;								// Return FALSE
	}

	if (!(RenderingWindow::instance->hRC=wglCreateContext(RenderingWindow::instance->hDC)))				// Are We Able To Get A Rendering Context?
	{
		RenderingWindow::destroyWindow();
		MessageBox(NULL,"Can't Create A GL Rendering Context.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return false;								// Return FALSE
	}

	if(!wglMakeCurrent(RenderingWindow::instance->hDC,RenderingWindow::instance->hRC))					// Try To Activate The Rendering Context
	{
		RenderingWindow::destroyWindow();
		MessageBox(NULL,"Can't Activate The GL Rendering Context.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return false;								// Return FALSE
	}

	ShowWindow(RenderingWindow::instance->hWnd,SW_SHOW);						// Show The Window
	SetForegroundWindow(RenderingWindow::instance->hWnd);						// Slightly Higher Priority
	SetFocus(RenderingWindow::instance->hWnd);									// Sets Keyboard Focus To The Window
	RenderingWindow::instance->resize( width, height );

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
	return true;
}

void
RenderingWindow::resize( int width, int height )		// Resize And Initialize The GL Window
{
	if (height==0)										// Prevent A Divide By Zero By
	{
		height=1;										// Making Height Equal One
	}

	glViewport( 0,0, width, height );						// Reset The Current Viewport

	// TODO resize camera
	// TODO reinit renderer
}

bool
RenderingWindow::swapBuffers()
{
	SwapBuffers( hDC );				// Swap Buffers (Double Buffering)

	return true;
}

bool
RenderingWindow::destroyWindow()								// Properly Kill The Window
{
	if ( NULL == RenderingWindow::instance )
	{
		return true;
	}

	if ( RenderingWindow::instance->m_fullScreen )										// Are We In Fullscreen Mode?
	{
		ChangeDisplaySettings(NULL,0);					// If So Switch Back To The Desktop
		ShowCursor(TRUE);								// Show Mouse Pointer
	}

	if ( RenderingWindow::instance->hRC )											// Do We Have A Rendering Context?
	{
		if (!wglMakeCurrent(NULL,NULL))					// Are We Able To Release The DC And RC Contexts?
		{
			MessageBox(NULL,"Release Of DC And RC Failed.","SHUTDOWN ERROR",MB_OK | MB_ICONINFORMATION);
		}

		if (!wglDeleteContext(RenderingWindow::instance->hRC))						// Are We Able To Delete The RC?
		{
			MessageBox(NULL,"Release Rendering Context Failed.","SHUTDOWN ERROR",MB_OK | MB_ICONINFORMATION);
		}
		RenderingWindow::instance->hRC=NULL;										// Set RC To NULL
	}

	if (RenderingWindow::instance->hDC && !ReleaseDC(RenderingWindow::instance->hWnd,RenderingWindow::instance->hDC))					// Are We Able To Release The DC
	{
		MessageBox(NULL,"Release Device Context Failed.","SHUTDOWN ERROR",MB_OK | MB_ICONINFORMATION);
		RenderingWindow::instance->hDC=NULL;										// Set DC To NULL
	}

	if (RenderingWindow::instance->hWnd && !DestroyWindow(RenderingWindow::instance->hWnd))					// Are We Able To Destroy The Window?
	{
		MessageBox(NULL,"Could Not Release hWnd.","SHUTDOWN ERROR",MB_OK | MB_ICONINFORMATION);
		RenderingWindow::instance->hWnd=NULL;										// Set hWnd To NULL
	}

	if (!UnregisterClass("OpenGL",RenderingWindow::instance->hInstance))			// Are We Able To Unregister Class
	{
		MessageBox(NULL,"Could Not Unregister Class.","SHUTDOWN ERROR",MB_OK | MB_ICONINFORMATION);
		RenderingWindow::instance->hInstance=NULL;									// Set hInstance To NULL
	}

	delete RenderingWindow::instance;

	return true;
}

LRESULT CALLBACK
RenderingWindow::WndProc(	HWND	hWnd,			// Handle For This Window
							UINT	uMsg,			// Message For This Window
							WPARAM	wParam,			// Additional Message Information
							LPARAM	lParam)			// Additional Message Information
{
	switch (uMsg)									// Check For Windows Messages
	{
		case WM_ACTIVATE:							// Watch For Window Activate Message
		{
			if (!HIWORD(wParam))					// Check Minimization State
			{
				//active=TRUE;						// Program Is Active
				RenderingWindow::instance->m_activeFlag = true;
			}
			else
			{
				//active=FALSE;						// Program Is No Longer Active
				RenderingWindow::instance->m_activeFlag = false;
			}

			return 0;								// Return To The Message Loop
		}

		case WM_SYSCOMMAND:							// Intercept System Commands
		{
			switch (wParam)							// Check System Calls
			{
				case SC_SCREENSAVE:					// Screensaver Trying To Start?
				case SC_MONITORPOWER:				// Monitor Trying To Enter Powersave?
				return 0;							// Prevent From Happening
			}
			break;									// Exit
		}

		case WM_CLOSE:								// Did We Receive A Close Message?
		{
			PostQuitMessage(0);						// Send A Quit Message
			return 0;								// Jump Back
		}

		case WM_SIZE:								// Resize The OpenGL Window
		{
			RenderingWindow::instance->resize( LOWORD(lParam),HIWORD(lParam) );
			return 0;								// Jump Back
		}
	}

	// Pass All Unhandled Messages To DefWindowProc
	return DefWindowProc(hWnd,uMsg,wParam,lParam);
}
