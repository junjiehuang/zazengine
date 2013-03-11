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
	if( m_keyBoard )
	{
		m_keyBoard->capture();
		if( !m_keyBoard->buffered() )
			handleNonBufferedKeys();
	}

	if( m_mouse )
	{
		m_mouse->capture();
		if( !m_mouse->buffered() )
			handleNonBufferedMouse();
	}

	for( int i = 0; i < 4 ; ++i )
	{
		if( m_joys[i] )
		{
			m_joys[i]->capture();
			if( !m_joys[i]->buffered() )
				handleNonBufferedJoy( m_joys[i] );
		}
	}

	list<int>::iterator pressedKeysIter = pressedKeys.begin();
	while ( pressedKeysIter != pressedKeys.end() )
	{
		int key = *pressedKeysIter++;
		
		switch( key )
		{
			case KC_Q:
				this->m_core->getEventManager().postEvent( Event( "KEY_PRE_Q" ) );
				break;

			case KC_E:
				this->m_core->getEventManager().postEvent( Event( "KEY_PRE_E" ) );
				break;

			case KC_W:
				this->m_core->getEventManager().postEvent( Event( "KEY_PRE_W" ) );
				break;
				
			case KC_S:
				this->m_core->getEventManager().postEvent( Event( "KEY_PRE_S" ) );
				break;
				
			case KC_A:
				this->m_core->getEventManager().postEvent( Event( "KEY_PRE_A" ) );
				break;
				
			case KC_D:
				this->m_core->getEventManager().postEvent( Event( "KEY_PRE_D" ) );
				break;

			default:
				break;
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
	pressedKeys.push_back( arg.key );
	/*
	std::cout << " KeyPressed {" << arg.key
		<< ", " << ((Keyboard*)(arg.device))->getAsString(arg.key)
		<< "} || Character (" << (char)arg.text << ")" << std::endl;
		*/
	return true;
}

bool
ZazenOIS::keyReleased( const KeyEvent &arg )
{
	if( arg.key == KC_ESCAPE )
	{
		this->m_core->stop();
	}
	else if (  arg.key == KC_F1 )
	{
		this->m_core->getEventManager().postEvent( Event( "TOGGLE_FULLSCREEN" ) );
	}
	else
	{
		pressedKeys.remove( arg.key );
	}
	
	//std::cout << "KeyReleased {" << ((Keyboard*)(arg.device))->getAsString(arg.key) << "}\n";
	
	return true;
}

bool
ZazenOIS::mouseMoved( const MouseEvent &arg )
{
	/*
	const OIS::MouseState& s = arg.state;
	std::cout << "\nMouseMoved: Abs("
				<< s.X.abs << ", " << s.Y.abs << ", " << s.Z.abs << ") Rel("
				<< s.X.rel << ", " << s.Y.rel << ", " << s.Z.rel << ")";
				*/
	const OIS::MouseState& s = arg.state;

	Event e( "MOUSE_MOVE" );
	e.addValue( "x", s.X.rel );
	e.addValue( "y", s.Y.rel );

	this->m_core->getEventManager().postEvent( e );

	return true;
}

bool
ZazenOIS::mousePressed( const MouseEvent &arg, MouseButtonID id )
{
	/*
	const OIS::MouseState& s = arg.state;
	std::cout << "\nMouse button #" << id << " pressed. Abs("
				<< s.X.abs << ", " << s.Y.abs << ", " << s.Z.abs << ") Rel("
				<< s.X.rel << ", " << s.Y.rel << ", " << s.Z.rel << ")";
				*/
	return true;
}

bool
ZazenOIS::mouseReleased( const MouseEvent &arg, MouseButtonID id )
{
	/*
	const OIS::MouseState& s = arg.state;
	std::cout << "\nMouse button #" << id << " released. Abs("
				<< s.X.abs << ", " << s.Y.abs << ", " << s.Z.abs << ") Rel("
				<< s.X.rel << ", " << s.Y.rel << ", " << s.Z.rel << ")";
				*/
	return true;
}

bool
ZazenOIS::buttonPressed( const JoyStickEvent &arg, int button )
{
	//std::cout << std::endl << arg.device->vendor() << ". Button Pressed # " << button;
	return true;
}

bool
ZazenOIS::buttonReleased( const JoyStickEvent &arg, int button )
{
	//std::cout << std::endl << arg.device->vendor() << ". Button Released # " << button;
	return true;
}

bool
ZazenOIS::axisMoved( const JoyStickEvent &arg, int axis )
{
	//Provide a little dead zone
	/*if( arg.state.mAxes[axis].abs > 2500 || arg.state.mAxes[axis].abs < -2500 )
		std::cout << std::endl << arg.device->vendor() << ". Axis # " << axis << " Value: " << arg.state.mAxes[axis].abs
		*/
	return true;
}

bool
ZazenOIS::povMoved( const JoyStickEvent &arg, int pov )
{
	/*
	std::cout << std::endl << arg.device->vendor() << ". POV" << pov << " ";

	if( arg.state.mPOV[pov].direction & Pov::North ) //Going up
		std::cout << "North";
	else if( arg.state.mPOV[pov].direction & Pov::South ) //Going down
		std::cout << "South";

	if( arg.state.mPOV[pov].direction & Pov::East ) //Going right
		std::cout << "East";
	else if( arg.state.mPOV[pov].direction & Pov::West ) //Going left
		std::cout << "West";

	if( arg.state.mPOV[pov].direction == Pov::Centered ) //stopped/centered out
		std::cout << "Centered";
		*/
	return true;
}

bool
ZazenOIS::vector3Moved( const JoyStickEvent &arg, int index)
{
	/*
	std::cout.precision(2);
	std::cout.flags(std::ios::fixed | std::ios::right);
	std::cout << std::endl << arg.device->vendor() << ". Orientation # " << index 
		<< " X Value: " << arg.state.mVectors[index].x
		<< " Y Value: " << arg.state.mVectors[index].y
		<< " Z Value: " << arg.state.mVectors[index].z;
	std::cout.precision();
	std::cout.flags();
	*/
	return true;
}

void
ZazenOIS::handleNonBufferedKeys()
{
	if( m_keyBoard->isKeyDown( KC_ESCAPE ) )
	{
		this->m_core->stop();
	}

	if( m_keyBoard->isModifierDown(Keyboard::Shift) )
		std::cout << "Shift is down..\n";
	if( m_keyBoard->isModifierDown(Keyboard::Alt) )
		std::cout << "Alt is down..\n";
	if( m_keyBoard->isModifierDown(Keyboard::Ctrl) )
		std::cout << "Ctrl is down..\n";
}

void
ZazenOIS::handleNonBufferedMouse()
{
	/*
	//Just dump the current mouse state
	const MouseState &ms = m_mouse->getMouseState();
	std::cout << "\nMouse: Abs(" << ms.X.abs << " " << ms.Y.abs << " " << ms.Z.abs
		<< ") B: " << ms.buttons << " Rel(" << ms.X.rel << " " << ms.Y.rel << " " << ms.Z.rel << ")";
		*/
}

void
ZazenOIS::handleNonBufferedJoy( JoyStick* js )
{
	/*
	//Just dump the current joy state
	const JoyStickState &joy = js->getJoyStickState();
	for( unsigned int i = 0; i < joy.mAxes.size(); ++i )
		std::cout << "\nAxis " << i << " X: " << joy.mAxes[i].abs;
		*/
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
		int numSticks = m_inputManager->getNumberOfDevices(OISJoyStick);
		if ( numSticks >= 4 )
			numSticks = 4;

		memset( this->m_joys, 0, sizeof( this->m_joys ) ); 

		for( int i = 0; i < numSticks; ++i )
		{
			m_joys[i] = (JoyStick*)m_inputManager->createInputObject( OISJoyStick, true );
			m_joys[i]->setEventCallback( this );
			std::cout << "\n\nCreating Joystick " << (i + 1)
				<< "\n\tAxes: " << m_joys[i]->getNumberOfComponents(OIS_Axis)
				<< "\n\tSliders: " << m_joys[i]->getNumberOfComponents(OIS_Slider)
				<< "\n\tPOV/HATs: " << m_joys[i]->getNumberOfComponents(OIS_POV)
				<< "\n\tButtons: " << m_joys[i]->getNumberOfComponents(OIS_Button)
				<< "\n\tVector3: " << m_joys[i]->getNumberOfComponents(OIS_Vector3)
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
