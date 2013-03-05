/*
 *  SDLInput.h
 *  Zazengine
 *
 *  Created by Jonathan Thaler on 17.07.10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef SDLINPUT_H_
#define SDLINPUT_H_

#include "../../../Core/SubSystems/IFaces/IInput.h"

#include <list>

class SDLInput : public IInput
{
	public:
		SDLInput();
		virtual ~SDLInput();
		
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
		
		ISubSystemEntity* createEntity( TiXmlElement*, IGameObject* parent);
		
	private:
		std::string id;
		std::string type;
	
		std::list<int> pressedKeys;
		
};

#endif /* SDLINPUT_H_ */
