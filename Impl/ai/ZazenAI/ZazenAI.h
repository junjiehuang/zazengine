/*
 * PlaygroundAI.h
 *
 *  Created on: 06.07.2010
 *      Author: joni
 */

#ifndef ZAZENAI_H_
#define ZAZENAI_H_

#include "../../../Core/SubSystems/IFaces/IAi.h"

#include "ZazenAIEntity.h"

class ZazenAI : public IAi
{
	public:
		ZazenAI();
		virtual ~ZazenAI();

		const std::string& getID() const { return this->id; };
		const std::string& getType() const { return this->type; };

		bool isAsync() const { return false; };

		bool start();
		bool stop();
		bool pause();

		bool initialize( TiXmlElement* );
		bool shutdown();

		bool process( double );
		bool finalizeProcess();

		bool sendEvent( Event& e );

		ZazenAIEntity* createEntity( TiXmlElement*, IGameObject* parent );

	private:
		std::string id;
		std::string type;

};

#endif /* ZAZENAI_H_ */
