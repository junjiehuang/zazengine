/*
 * ZazenGraphics.cpp
 *
 *  Created on: 06.07.2010
 *      Author: Jonathan Thaler
 */

#include "ZazenGraphics.h"

#include "Geometry/GeometryFactory.h"
#include "Material/Material.h"

#include "Renderer/DRRenderer.h"

#include <GL/glew.h>

#include <glm/gtc/matrix_transform.hpp>

#include <Windows.h>

#include <iostream>

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define WINDOW_BITS_PER_PIXEL 32

#define REQUIRED_MAJOR_OPENGL_VER 3
#define REQUIRED_MINOR_OPENGL_VER 1

using namespace std;

HDC			hDC=NULL;		// Private GDI Device Context
HGLRC		hRC=NULL;		// Permanent Rendering Context
HWND		hWnd=NULL;		// Holds Our Window Handle
HINSTANCE	hInstance;		// Holds The Instance Of The Application

bool	active=TRUE;		// Window Active Flag Set To TRUE By Default
bool	fullscreen=TRUE;	// Fullscreen Flag Set To Fullscreen Mode By Default

LRESULT	CALLBACK WndProc( HWND, UINT, WPARAM, LPARAM );	// Declaration For WndProc
GLvoid KillGLWindow( GLvoid );
BOOL CreateGLWindow( char* title, int width, int height, int bits, bool fullscreenflag );
GLvoid ReSizeGLScene( GLsizei width, GLsizei height );	

ZazenGraphics* ZazenGraphics::instance = NULL;

ZazenGraphics::ZazenGraphics( const std::string& id, ICore* core )
	: m_id ( id ),
	  m_type ( "graphics" ),
	  m_core( core )
{
	ZazenGraphics::instance = this;
}

ZazenGraphics::~ZazenGraphics()
{
	ZazenGraphics::instance = NULL;
}

bool
ZazenGraphics::initialize( TiXmlElement* configNode )
{
	cout << endl << "=============== ZazenGraphics initializing... ===============" << endl;

	// important: need to initialize window first because only then we have a valid opengl-context
	if ( false == this->createWindow( configNode ) )
	{
		return false;
	}

	if ( false == this->initGL( configNode ) )
	{
		return false;
	}

	// cannot initialize renderer now because camera not yet loaded
	this->m_renderer = new DRRenderer();

	if ( false == Material::loadAll() )
	{
		cout << "Coulnd't load materials - exit" << endl;
		return false;
	}

	this->m_core->getEventManager().registerForEvent( "TOGGLE_FULLSCREEN", this );

	cout << "================ ZazenGraphics initialized =================" << endl;
	
	return true;
}

bool
ZazenGraphics::shutdown()
{
	cout << endl << "=============== ZazenGraphics shutting down... ===============" << endl;

	this->m_core->getEventManager().unregisterForEvent( "TOGGLE_FULLSCREEN", this );

	std::list<ZazenGraphicsEntity*>::iterator iter = this->m_entities.begin();
	while ( iter != this->m_entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;
		delete entity;
	}

	this->m_entities.clear();

	Material::freeAll();
	Texture::freeAll();
	GeometryFactory::freeAll();
	
	KillGLWindow();

	cout << "================ ZazenGraphics shutdown =================" << endl;

	return true;
}

bool
ZazenGraphics::start()
{
	if ( 0 == this->m_camera )
	{
		cout << "ERROR ... missing camera in ZazenGraphics - exit" << endl;
		return false;
	}

	this->m_renderer->setCamera( this->m_camera );
	if ( false == this->m_renderer->initialize() )
	{
		cout << "ERROR ... initializing renderer failed - exit" << endl;
		return false;
	}

	return true;
}

bool
ZazenGraphics::stop()
{
	return true;
}

bool
ZazenGraphics::pause()
{
	return true;
}

bool
ZazenGraphics::process( double iterationFactor )
{
	bool flag = true;
	//cout << "ZazenGraphics::process enter" << endl;

	// process events of m_entities
	std::list<ZazenGraphicsEntity*>::iterator iter = this->m_entities.begin();
	while ( iter != this->m_entities.end() )
	{
		ZazenGraphicsEntity* entity = *iter++;
	
		if ( entity->isAnimated() )
		{
			entity->doAnimation();
		}

		entity->queuedEvents.clear();
	}

	MSG		msg;

	if ( PeekMessage( &msg, NULL, 0, 0, PM_REMOVE ) )	// Is There A Message Waiting?
	{
		TranslateMessage( &msg );				// Translate The Message
		DispatchMessage( &msg );				// Dispatch The Message
	}
	else										// If There Are No Messages
	{
		// Draw The Scene.  Watch For ESC Key And Quit Messages From DrawGLScene()
		if ( active )								// Program Active?
		{
			flag = this->m_renderer->renderFrame( this->m_instances, this->m_lights );
			SwapBuffers( hDC );				// Swap Buffers (Double Buffering)
		}
	}

	//cout << "ZazenGraphics::process leave" << endl;

	return flag;
}

bool
ZazenGraphics::finalizeProcess()
{
	//cout << "ZazenGraphics::finalizeProcess" << endl;

	return true;
}

bool
ZazenGraphics::sendEvent( Event& e )
{
	if  ( e == "TOGGLE_FULLSCREEN" )
	{
		this->toggleFullscreen();
	}

	return true;
}

ZazenGraphicsEntity*
ZazenGraphics::createEntity( TiXmlElement* objectNode, IGameObject* parent )
{
	ZazenGraphicsEntity* entity = new ZazenGraphicsEntity( parent );

	TiXmlElement* instanceNode = objectNode->FirstChildElement( "instance" );
	if ( instanceNode )
	{
		Instance* instance = new Instance();

		const char* str = instanceNode->Attribute( "mesh" );
		if ( 0 != str )
		{
			instance->geom = GeometryFactory::get( str );
			if ( NULL == instance->geom )
			{
				// TODO ignore
			}
		}

		str = instanceNode->Attribute( "material" );
		if ( 0 != str )
		{
			instance->material = Material::get( str );
			if ( NULL == instance->material )
			{
				// TODO ignore
			}
		}

		if ( NULL != instance->geom && NULL != instance->material )
		{
			entity->m_orientation = instance;

			this->m_instances.push_back( instance );
		}
	}

	if ( 0 == entity->m_orientation )
	{
		TiXmlElement* lightNode = objectNode->FirstChildElement( "light" );
		if ( lightNode )
		{
			std::string lightType = "SPOT";

			const char* str = lightNode->Attribute( "type" );
			if ( 0 != str )
			{
				lightType = str;
			}

			Light* light = 0;

			if ( lightType == "DIRECTIONAL" )
			{
				light = Light::createDirectionalLight( WINDOW_WIDTH, WINDOW_HEIGHT );

			}
			else if ( lightType == "POINT" )
			{
				light = Light::createPointLight( WINDOW_HEIGHT );
			}
			// default is spot
			else
			{
				light = Light::createSpoptLight( 90, WINDOW_WIDTH, WINDOW_HEIGHT );
			}

			entity->m_orientation = light;
			this->m_lights.push_back( light );
		}
	}

	if ( 0 == entity->m_orientation )
	{
		TiXmlElement* cameraNode = objectNode->FirstChildElement( "camera" );
		if ( cameraNode )
		{
			float fov = 90.0f;
			std::string mode = "PROJ";

			const char* str = cameraNode->Attribute( "fov" );
			if ( 0 == str )
			{
				cout << "INFO ... fov attribute missing in cameraNode - use default " << endl;
			}
			else
			{
				fov = ( float ) atof( str );
			}

			str = cameraNode->Attribute( "view" );
			if ( 0 == str )
			{
				cout << "INFO ... view attribute missing in cameraNode - use default " << endl;
			}
			else
			{
				mode = str;
			}

			Viewer* camera = new Viewer( WINDOW_WIDTH, WINDOW_HEIGHT );
			if ( mode == "PROJ" )
			{
				camera->setFov( fov );
				camera->setupPerspective();
			}
			else if ( mode == "ORTHO" )
			{
				camera->setupOrtho();
			}

			entity->m_orientation = camera;
			this->m_camera = camera;
		}
	}

	if ( 0 == entity->m_orientation )
	{
		cout << "No valid entity defined in ZazenGraphics for Object \"" << parent->getName() << "\" - error " << endl;
		// TODO cleanup memory!
		return 0;
	}

	TiXmlElement* orientationNode = objectNode->FirstChildElement( "orientation" );
	if ( orientationNode )
	{
		glm::vec3 v;
		float roll = 0.0f;
		float pitch = 0.0f;
		float heading = 0.0f;

		const char* str = orientationNode->Attribute( "x" );
		if ( 0 != str )
		{
			v[ 0 ] = ( float ) atof( str );
		}

		str = orientationNode->Attribute( "y" );
		if ( 0 != str )
		{
			v[ 1 ] = ( float ) atof( str );
		}

		str = orientationNode->Attribute( "z" );
		if ( 0 != str )
		{
			v[ 2 ] = ( float ) atof( str );
		}

		str = orientationNode->Attribute( "heading" );
		if ( 0 == str )
		{
			cout << "INFO ... heading attribute missing in orientation - use default " << endl;
		}
		else
		{
			heading = ( float ) atof( str );
		}

		str = orientationNode->Attribute( "roll" );
		if ( 0 == str )
		{
			cout << "INFO ... roll attribute missing in orientation - use default " << endl;
		}
		else
		{
			roll = ( float ) atof( str );
		}

		str = orientationNode->Attribute( "pitch" );
		if ( 0 == str )
		{
			cout << "INFO ... pitch attribute missing in orientation - use default " << endl;
		}
		else
		{
			pitch = ( float ) atof( str );
		}

		entity->m_orientation->set( v, pitch, heading, roll );
	}

	TiXmlElement* animationNode = objectNode->FirstChildElement( "animation" );
	if ( animationNode )
	{
		glm::vec3 animRot;

		const char* str = animationNode->Attribute( "heading" );
		if ( 0 == str )
		{
			cout << "INFO ... heading attribute missing in animation - use default " << endl;
		}
		else
		{
			animRot[ 0 ] = ( float ) atof( str );
		}

		str = animationNode->Attribute( "roll" );
		if ( 0 == str )
		{
			cout << "INFO ... roll attribute missing in animation - use default " << endl;
		}
		else
		{
			animRot[ 1 ] = ( float ) atof( str );
		}

		str = animationNode->Attribute( "pitch" );
		if ( 0 == str )
		{
			cout << "INFO ... pitch attribute missing in animation - use default " << endl;
		}
		else
		{
			animRot[ 2 ] = ( float ) atof( str );
		}

		entity->setAnimation( animRot[ 0 ], animRot[ 1 ], animRot[ 2 ] ); 
	}

	this->m_entities.push_back( entity );

	return entity;
}

void*
ZazenGraphics::getWindowHandle()
{
	return hWnd;
}

bool
ZazenGraphics::toggleFullscreen()
{
	KillGLWindow();
	fullscreen=!fullscreen;
	// Recreate Our OpenGL Window
	if ( !CreateGLWindow( "zaZengine", WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_BITS_PER_PIXEL, fullscreen ) )
	{
		return false;
	}

	/* TODO fix it, not yet working
	this->m_renderer->shutdown();
	this->m_renderer->initialize();
	*/

	return true;
}

bool
ZazenGraphics::createWindow( TiXmlElement* configNode )
{
	if ( ! CreateGLWindow( "zaZengine", WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_BITS_PER_PIXEL, false ) )
	{
		cout << "ERROR ... in ZazenGraphics::createWindow: failed creating window" << endl;
		return false;
	}

	return true;
}

BOOL CreateGLWindow( char* title, int width, int height, int bits, bool fullscreenflag )
{
	GLuint		PixelFormat;			// Holds The Results After Searching For A Match
	WNDCLASS	wc;						// Windows Class Structure
	DWORD		dwExStyle;				// Window Extended Style
	DWORD		dwStyle;				// Window Style
	RECT		WindowRect;				// Grabs Rectangle Upper Left / Lower Right Values
	WindowRect.left=(long)0;			// Set Left Value To 0
	WindowRect.right=(long)width;		// Set Right Value To Requested Width
	WindowRect.top=(long)0;				// Set Top Value To 0
	WindowRect.bottom=(long)height;		// Set Bottom Value To Requested Height

	fullscreen=fullscreenflag;			// Set The Global Fullscreen Flag

	hInstance			= GetModuleHandle(NULL);				// Grab An Instance For Our Window
	wc.style			= CS_HREDRAW | CS_VREDRAW | CS_OWNDC;	// Redraw On Size, And Own DC For Window.
	wc.lpfnWndProc		= (WNDPROC) WndProc;					// WndProc Handles Messages
	wc.cbClsExtra		= 0;									// No Extra Window Data
	wc.cbWndExtra		= 0;									// No Extra Window Data
	wc.hInstance		= hInstance;							// Set The Instance
	wc.hIcon			= LoadIcon(NULL, IDI_WINLOGO);			// Load The Default Icon
	wc.hCursor			= LoadCursor(NULL, IDC_ARROW);			// Load The Arrow Pointer
	wc.hbrBackground	= NULL;									// No Background Required For GL
	wc.lpszMenuName		= NULL;									// We Don't Want A Menu
	wc.lpszClassName	= "OpenGL";								// Set The Class Name

	if (!RegisterClass(&wc))									// Attempt To Register The Window Class
	{
		MessageBox(NULL,"Failed To Register The Window Class.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return FALSE;											// Return FALSE
	}
	
	if (fullscreen)												// Attempt Fullscreen Mode?
	{
		DEVMODE dmScreenSettings;								// Device Mode
		memset(&dmScreenSettings,0,sizeof(dmScreenSettings));	// Makes Sure Memory's Cleared
		dmScreenSettings.dmSize=sizeof(dmScreenSettings);		// Size Of The Devmode Structure
		dmScreenSettings.dmPelsWidth	= width;				// Selected Screen Width
		dmScreenSettings.dmPelsHeight	= height;				// Selected Screen Height
		dmScreenSettings.dmBitsPerPel	= bits;					// Selected Bits Per Pixel
		dmScreenSettings.dmFields=DM_BITSPERPEL|DM_PELSWIDTH|DM_PELSHEIGHT;

		// Try To Set Selected Mode And Get Results.  NOTE: CDS_FULLSCREEN Gets Rid Of Start Bar.
		if (ChangeDisplaySettings(&dmScreenSettings,CDS_FULLSCREEN)!=DISP_CHANGE_SUCCESSFUL)
		{
			// If The Mode Fails, Offer Two Options.  Quit Or Use Windowed Mode.
			if (MessageBox(NULL,"The Requested Fullscreen Mode Is Not Supported By\nYour Video Card. Use Windowed Mode Instead?","NeHe GL",MB_YESNO|MB_ICONEXCLAMATION)==IDYES)
			{
				fullscreen=FALSE;		// Windowed Mode Selected.  Fullscreen = FALSE
			}
			else
			{
				// Pop Up A Message Box Letting User Know The Program Is Closing.
				MessageBox(NULL,"Program Will Now Close.","ERROR",MB_OK|MB_ICONSTOP);
				return FALSE;									// Return FALSE
			}
		}
	}

	if (fullscreen)												// Are We Still In Fullscreen Mode?
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
	if (!(hWnd=CreateWindowEx(	dwExStyle,							// Extended Style For The Window
								"OpenGL",							// Class Name
								title,								// Window Title
								dwStyle |							// Defined Window Style
								WS_CLIPSIBLINGS |					// Required Window Style
								WS_CLIPCHILDREN,					// Required Window Style
								0, 0,								// Window Position
								WindowRect.right-WindowRect.left,	// Calculate Window Width
								WindowRect.bottom-WindowRect.top,	// Calculate Window Height
								NULL,								// No Parent Window
								NULL,								// No Menu
								hInstance,							// Instance
								NULL)))								// Dont Pass Anything To WM_CREATE
	{
		KillGLWindow();								// Reset The Display
		MessageBox(NULL,"Window Creation Error.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return FALSE;								// Return FALSE
	}

	static	PIXELFORMATDESCRIPTOR pfd=				// pfd Tells Windows How We Want Things To Be
	{
		sizeof(PIXELFORMATDESCRIPTOR),				// Size Of This Pixel Format Descriptor
		1,											// Version Number
		PFD_DRAW_TO_WINDOW |						// Format Must Support Window
		PFD_SUPPORT_OPENGL |						// Format Must Support OpenGL
		PFD_DOUBLEBUFFER,							// Must Support Double Buffering
		PFD_TYPE_RGBA,								// Request An RGBA Format
		bits,										// Select Our Color Depth
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
	
	if (!(hDC=GetDC(hWnd)))							// Did We Get A Device Context?
	{
		KillGLWindow();								// Reset The Display
		MessageBox(NULL,"Can't Create A GL Device Context.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return FALSE;								// Return FALSE
	}

	if (!(PixelFormat=ChoosePixelFormat(hDC,&pfd)))	// Did Windows Find A Matching Pixel Format?
	{
		KillGLWindow();								// Reset The Display
		MessageBox(NULL,"Can't Find A Suitable PixelFormat.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return FALSE;								// Return FALSE
	}

	if(!SetPixelFormat(hDC,PixelFormat,&pfd))		// Are We Able To Set The Pixel Format?
	{
		KillGLWindow();								// Reset The Display
		MessageBox(NULL,"Can't Set The PixelFormat.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return FALSE;								// Return FALSE
	}

	if (!(hRC=wglCreateContext(hDC)))				// Are We Able To Get A Rendering Context?
	{
		KillGLWindow();								// Reset The Display
		MessageBox(NULL,"Can't Create A GL Rendering Context.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return FALSE;								// Return FALSE
	}

	if(!wglMakeCurrent(hDC,hRC))					// Try To Activate The Rendering Context
	{
		KillGLWindow();								// Reset The Display
		MessageBox(NULL,"Can't Activate The GL Rendering Context.","ERROR",MB_OK|MB_ICONEXCLAMATION);
		return FALSE;								// Return FALSE
	}

	ShowWindow(hWnd,SW_SHOW);						// Show The Window
	SetForegroundWindow(hWnd);						// Slightly Higher Priority
	SetFocus(hWnd);									// Sets Keyboard Focus To The Window
	ReSizeGLScene(width, height);					// Set Up Our Perspective GL Screen

	return TRUE;									// Success
}

GLvoid ReSizeGLScene(GLsizei width, GLsizei height)		// Resize And Initialize The GL Window
{
	if (height==0)										// Prevent A Divide By Zero By
	{
		height=1;										// Making Height Equal One
	}

	glViewport(0,0,width,height);						// Reset The Current Viewport

	// TODO resize viewer
	// TODO reinit renderer
}

GLvoid KillGLWindow(GLvoid)								// Properly Kill The Window
{
	if (fullscreen)										// Are We In Fullscreen Mode?
	{
		ChangeDisplaySettings(NULL,0);					// If So Switch Back To The Desktop
		ShowCursor(TRUE);								// Show Mouse Pointer
	}

	if (hRC)											// Do We Have A Rendering Context?
	{
		if (!wglMakeCurrent(NULL,NULL))					// Are We Able To Release The DC And RC Contexts?
		{
			MessageBox(NULL,"Release Of DC And RC Failed.","SHUTDOWN ERROR",MB_OK | MB_ICONINFORMATION);
		}

		if (!wglDeleteContext(hRC))						// Are We Able To Delete The RC?
		{
			MessageBox(NULL,"Release Rendering Context Failed.","SHUTDOWN ERROR",MB_OK | MB_ICONINFORMATION);
		}
		hRC=NULL;										// Set RC To NULL
	}

	if (hDC && !ReleaseDC(hWnd,hDC))					// Are We Able To Release The DC
	{
		MessageBox(NULL,"Release Device Context Failed.","SHUTDOWN ERROR",MB_OK | MB_ICONINFORMATION);
		hDC=NULL;										// Set DC To NULL
	}

	if (hWnd && !DestroyWindow(hWnd))					// Are We Able To Destroy The Window?
	{
		MessageBox(NULL,"Could Not Release hWnd.","SHUTDOWN ERROR",MB_OK | MB_ICONINFORMATION);
		hWnd=NULL;										// Set hWnd To NULL
	}

	if (!UnregisterClass("OpenGL",hInstance))			// Are We Able To Unregister Class
	{
		MessageBox(NULL,"Could Not Unregister Class.","SHUTDOWN ERROR",MB_OK | MB_ICONINFORMATION);
		hInstance=NULL;									// Set hInstance To NULL
	}
}

LRESULT CALLBACK WndProc(	HWND	hWnd,			// Handle For This Window
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
				active=TRUE;						// Program Is Active
			}
			else
			{
				active=FALSE;						// Program Is No Longer Active
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
			ReSizeGLScene(LOWORD(lParam),HIWORD(lParam));  // LoWord=Width, HiWord=Height
			return 0;								// Jump Back
		}
	}

	// Pass All Unhandled Messages To DefWindowProc
	return DefWindowProc(hWnd,uMsg,wParam,lParam);
}

bool
ZazenGraphics::initGL( TiXmlElement* configNode )
{
	int major, minor;

	GLenum err = glewInit();
	if ( GLEW_OK != err )
	{
		cout << "ERROR ... GLEW failed with " <<  glewGetErrorString(err) << endl;
		return false;
	}
	else
	{
		cout << "OK ... GLEW " << glewGetString(GLEW_VERSION) << " initialized " << endl;
	}

	cout << "OpenGL Version: " << glGetString(GL_VERSION) << endl;

	glGetIntegerv( GL_MAJOR_VERSION, &major );
	glGetIntegerv( GL_MINOR_VERSION, &minor );

	if ( REQUIRED_MAJOR_OPENGL_VER > major || 
		 ( REQUIRED_MAJOR_OPENGL_VER == major && REQUIRED_MINOR_OPENGL_VER > minor ) )
	{
		cout << "ERROR ... OpenGL " << REQUIRED_MAJOR_OPENGL_VER << "." << REQUIRED_MINOR_OPENGL_VER << " or above required" << endl;
		return false;
	}
	
	return true;
}

extern "C"
{	
	__declspec( dllexport ) ISubSystem*
	createInstance ( const char* id, ICore* core )
	{
		return new ZazenGraphics( id, core );
	}

	__declspec( dllexport ) void
	deleteInstance ( ISubSystem* subSys )
	{
		if ( 0 == subSys )
			return;

		if ( 0 == dynamic_cast<ZazenGraphics*>( subSys ) )
			return;

		delete subSys;
	}
}