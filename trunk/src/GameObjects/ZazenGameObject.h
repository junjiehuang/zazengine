/*
 * ZazenGameObject.h
 *
 *  Created on: 27.06.2010
 *      Author: joni
 */

#ifndef ZAZENGAMEOBJECT_H_
#define ZAZENGAMEOBJECT_H_

#include "../Core/ObjectModel/IFaces/IGameObject.h"

#include "../Core/EventSystem/Event.h"
#include "../Core/EventSystem/IEventListener.h"

class ZazenGameObject : public IGameObject
{
	public:
		ZazenGameObject();
		virtual ~ZazenGameObject();

		GameObjectID getID() const { return id; };
		const std::string& getName() const { return this->name; };

		bool sendEvent(const Event&);

		IGameObject* clone() { return 0; };

		bool initialize( TiXmlElement* );

	private:
		static GameObjectID nextID;

		GameObjectID id;
		std::string name;

		std::map<std::string, ISubSystemEntity*> subSystemEntities;
};

#endif /* ZAZENGAMEOBJECT_H_ */
