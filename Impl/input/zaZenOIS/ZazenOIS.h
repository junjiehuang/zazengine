/*
 *  ZazenOIS.h
 *  Zazengine
 *
 *  Created by Jonathan Thaler on 09.03.13
 *
 */

#ifndef ZAZENOIS_H_
#define ZAZENOIS_H_

#include <input/IInput.h>

#include "ZazenOISEntity.h"

#include <list>
#include <string>

class ZazenOIS : public IInput
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
		
	private:
		std::string id;
		std::string type;
	
		std::list<int> pressedKeys;

		ICore* m_core;
		
};

#endif /* SDLINPUT_H_ */
