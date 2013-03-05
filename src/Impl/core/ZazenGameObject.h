/*
 * ZazenGameObject.h
 *
 *  Created on: 27.06.2010
 *      Author: joni
 */

#ifndef ZAZENGAMEOBJECT_H_
#define ZAZENGAMEOBJECT_H_

#include <core/IGameObject.h>

class ZazenGameObject : public IGameObject
{
	public:
		ZazenGameObject();
		virtual ~ZazenGameObject();

		GameObjectID getID() const { return id; };
		const std::string& getName() const { return this->name; };

		bool sendEvent( Event& );

		IGameObject* clone() { return 0; };

		bool initialize( TiXmlElement* );

		boost::any getProperty( const std::string& id )  { return this->properties[ id ]; };

	private:
		static GameObjectID nextID;

		GameObjectID id;
		std::string name;

		std::map<std::string, ISubSystemEntity*> subSystemEntities;
		std::map<std::string, boost::any> properties;
};

#endif /* ZAZENGAMEOBJECT_H_ */
