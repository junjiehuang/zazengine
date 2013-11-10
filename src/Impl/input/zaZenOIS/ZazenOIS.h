#ifndef _ZAZENOIS_H_
#define _ZAZENOIS_H_

#include <input/IInput.h>

#include <core/ILogger.h>

#include "ZazenOISEntity.h"

#include <OIS.h>

#include <list>
#include <string>

class ZazenOIS : public IInput, public OIS::KeyListener, public OIS::MouseListener, public OIS::JoyStickListener
{
	public:
		ZazenOIS( const std::string&, ICore* );
		virtual ~ZazenOIS();
		
		const std::string& getID() const { return this->id; };
		const std::string& getType() const { return this->type; };
		
		bool isAsync() const { return false; };
		
		bool initialize( TiXmlElement* );
		bool shutdown();
		
		bool start();
		bool stop();
		bool pause();
		
		bool process( double );
		bool finalizeProcess();
		
		bool sendEvent( Event& );
		
		ZazenOISEntity* createEntity( TiXmlElement*, IGameObject* parent);
		
		bool keyPressed( const OIS::KeyEvent &arg );
		bool keyReleased( const OIS::KeyEvent &arg );

		bool mouseMoved( const OIS::MouseEvent &arg );
		bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
		bool mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
	
		bool buttonPressed( const OIS::JoyStickEvent &arg, int button );
		bool buttonReleased( const OIS::JoyStickEvent &arg, int button );

		bool axisMoved( const OIS::JoyStickEvent &arg, int axis );
		bool povMoved( const OIS::JoyStickEvent &arg, int pov );
		bool vector3Moved( const OIS::JoyStickEvent &arg, int index );

	private:
		std::string id;
		std::string type;

		ICore* m_core;
		ILogger* m_logger;

		OIS::InputManager* m_inputManager;
		OIS::Keyboard* m_keyBoard;
		OIS::Mouse* m_mouse;
		OIS::JoyStick* m_joys[4];

		bool initOIS( TiXmlElement* );
};

#endif /* _ZAZENOIS_H_ */
