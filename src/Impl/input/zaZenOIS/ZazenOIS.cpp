/*
 *  ZazenOIS.cpp
 *  Zazengine
 *
 *  Created by Jonathan Thaler on 09.03.13
 *
 */

#include "ZazenOIS.h"

#include <core/ICore.h>
#include <graphics/IGraphics.h>

#include <windows.h>

#include <iostream>
#include <vector>
#include <sstream>

using namespace std;
using namespace OIS;

ZazenOIS::ZazenOIS( const std::string& id, ICore* core )
	: id ( id ),
	type ( "input" ),
	m_core( core )
{
}

ZazenOIS::~ZazenOIS()
{
}

bool
ZazenOIS::initialize( TiXmlElement* element )
{
	cout << endl << "=============== ZazenOIS initializing... ===============" << endl;

	if ( false == this->initOIS( element ) )
	{
		return false;
	}

	cout << "================ ZazenOIS initialized =================" << endl;

	return true;
}

bool
ZazenOIS::shutdown()
{
	cout << endl << "=============== ZazenOIS shutting down... ===============" << endl;

	if( m_inputManager )
	{
		InputManager::destroyInputSystem( m_inputManager );
	}

	cout << "================ ZazenOIS shutdown =================" << endl;

	return true;
}

bool
ZazenOIS::start()
{
	return true;
}

bool
ZazenOIS::stop()
{
	return true;
}

bool
ZazenOIS::pause()
{
	return true;
}

bool
ZazenOIS::process( double factor )
{
	if ( this->m_keyBoard )
	{
		this->m_keyBoard->capture();
	}

	if ( this->m_mouse )
	{
		this->m_mouse->capture();
	}

	for ( int i = 0; i < 4 ; ++i )
	{
		if ( this->m_joys[i] )
		{
			this->m_joys[i]->capture();
		}
	}
	
	return true;
}

bool
ZazenOIS::finalizeProcess()
{
	return true;
}

bool
ZazenOIS::sendEvent( Event& e )
{
	return true;
}

ZazenOISEntity*
ZazenOIS::createEntity( TiXmlElement* cfgNode, IGameObject* parent )
{
	return 0;
}

bool
ZazenOIS::keyPressed( const KeyEvent &arg )
{
	Event e( "KEY_PRESSED" );
	e.addValue( "key", ( int ) arg.key );

	this->m_core->getEventManager().postEvent( e );

	return true;
}

bool
ZazenOIS::keyReleased( const KeyEvent &arg )
{
	if( arg.key == KC_ESCAPE )
	{
		this->m_core->stop();
	}
	else
	{
		Event e( "KEY_RELEASED" );
		e.addValue( "key", ( int ) arg.key );

		this->m_core->getEventManager().postEvent( e );
	}
	
	return true;
}

bool
ZazenOIS::mouseMoved( const MouseEvent &arg )
{
	const OIS::MouseState& s = arg.state;

	Event e( "MOUSE_MOVED" );
	e.addValue( "x", s.X.rel );
	e.addValue( "y", s.Y.rel );

	this->m_core->getEventManager().postEvent( e );

	return true;
}

bool
ZazenOIS::mousePressed( const MouseEvent &arg, MouseButtonID id )
{
	return true;
}

bool
ZazenOIS::mouseReleased( const MouseEvent &arg, MouseButtonID id )
{
	return true;
}

bool
ZazenOIS::buttonPressed( const JoyStickEvent &arg, int button )
{
	return true;
}

bool
ZazenOIS::buttonReleased( const JoyStickEvent &arg, int button )
{
	return true;
}

bool
ZazenOIS::axisMoved( const JoyStickEvent &arg, int axis )
{
	return true;
}

bool
ZazenOIS::povMoved( const JoyStickEvent &arg, int pov )
{
	return true;
}

bool
ZazenOIS::vector3Moved( const JoyStickEvent &arg, int index)
{
	return true;
}

bool
ZazenOIS::initOIS( TiXmlElement* element )
{
	ParamList pl;

	HWND hWnd = NULL;

	IGraphics* graphics = this->m_core->getGraphics();
	if ( NULL == graphics )
	{
		cout << "ERROR ... in ZazenOIS::initOIS: could not get access to IGraphics but is mandatory to aquire window-handle" << endl;
		return false;
	}

	hWnd = ( HWND ) graphics->getWindowHandle();

	std::ostringstream wnd;
	wnd << ( size_t ) hWnd;

	pl.insert( std::make_pair( std::string( "WINDOW" ), wnd.str() ) );

		//This never returns null.. it will raise an exception on errors
	this->m_inputManager = InputManager::createInputSystem( pl );

	//Lets enable all addons that were compiled in:
	this->m_inputManager->enableAddOnFactory( InputManager::AddOn_All );

	//Print debugging information
	unsigned int v = m_inputManager->getVersionNumber();
	std::cout << "OIS Version: " << (v>>16 ) << "." << ((v>>8) & 0x000000FF) << "." << (v & 0x000000FF)
		<< "\nRelease Name: " << m_inputManager->getVersionName()
		<< "\nManager: " << m_inputManager->inputSystemName()
		<< "\nTotal Keyboards: " << m_inputManager->getNumberOfDevices(OISKeyboard)
		<< "\nTotal Mice: " << m_inputManager->getNumberOfDevices(OISMouse)
		<< "\nTotal JoySticks: " << m_inputManager->getNumberOfDevices(OISJoyStick);

	const char *g_DeviceType[6] = { "OISUnknown", "OISKeyboard", "OISMouse", "OISJoyStick", "OISTablet", "OISOther" };
	//List all devices
	DeviceList list = m_inputManager->listFreeDevices();
	for( DeviceList::iterator i = list.begin(); i != list.end(); ++i )
		std::cout << "\n\tDevice: " << g_DeviceType[ i->first ] << " Vendor: " << i->second << endl;

	m_keyBoard = ( Keyboard* ) m_inputManager->createInputObject( OISKeyboard, true );
	m_keyBoard->setEventCallback( ( KeyListener* ) this );

	m_mouse = ( Mouse* ) m_inputManager->createInputObject( OISMouse, true );
	m_mouse->setEventCallback( this );
	const MouseState &ms = m_mouse->getMouseState();
	ms.width = 100;
	ms.height = 100;

	try
	{
		//This demo uses at most 4 joysticks - use old way to create (i.e. disregard vendor)
		int numSticks = m_inputManager->getNumberOfDevices( OISJoyStick );
		if ( numSticks >= 4 )
			numSticks = 4;

		memset( this->m_joys, 0, sizeof( this->m_joys ) ); 

		for( int i = 0; i < numSticks; ++i )
		{
			m_joys[ i ] = ( JoyStick* )m_inputManager->createInputObject( OISJoyStick, true );
			m_joys[ i ]->setEventCallback( this );
			std::cout << "\n\nCreating Joystick " << ( i + 1 )
				<< "\n\tAxes: " << m_joys[ i ]->getNumberOfComponents( OIS_Axis )
				<< "\n\tSliders: " << m_joys[ i ]->getNumberOfComponents( OIS_Slider )
				<< "\n\tPOV/HATs: " << m_joys[ i ]->getNumberOfComponents( OIS_POV )
				<< "\n\tButtons: " << m_joys[ i ]->getNumberOfComponents( OIS_Button )
				<< "\n\tVector3: " << m_joys[ i] ->getNumberOfComponents( OIS_Vector3 )
				<< endl;
		}
	}
	catch(OIS::Exception &ex)
	{
		std::cout << "\nException raised on joystick creation: " << ex.eText << std::endl;
	}

	return true;
}

extern "C"
{	
	__declspec( dllexport ) ISubSystem*
	createInstance ( const char* id, ICore* core )
	{
		return new ZazenOIS( id, core );
	}

	__declspec( dllexport ) void
	deleteInstance ( ISubSystem* subSys )
	{
		if ( 0 == subSys )
			return;

		if ( 0 == dynamic_cast<ZazenOIS*>( subSys ) )
			return;

		delete subSys;
	}
}
