/*
 * ZazenGameObject.h
 *
 *  Created on: 27.06.2010
 *      Author: Jonathan Thaler
 */

#ifndef ZAZENGAMEOBJECT_H_
#define ZAZENGAMEOBJECT_H_

#include <core/DllExport.h>
#include <core/IGameObject.h>

class DLL_API ZazenGameObject : public IGameObject
{
	public:
		ZazenGameObject( const std::string& );
		virtual ~ZazenGameObject();

		GameObjectID getID() const { return m_id; };
		const std::string& getName() const { return this->m_name; };
		const std::string& getObjectClass() const { return this->m_objectClass; };

		bool sendEvent( Event& );

		IGameObject* clone() { return 0; };

		bool initialize( TiXmlElement* );

		boost::any getProperty( const std::string& id )  { return this->m_properties[ id ]; };

	private:
		static GameObjectID nextID;

		GameObjectID m_id;
		std::string m_name;
		std::string m_objectClass;

		std::map<std::string, ISubSystemEntity*> m_subSystemEntities;
		std::map<std::string, boost::any> m_properties;
};

#endif /* ZAZENGAMEOBJECT_H_ */
