#ifndef _IGAMEOBJECT_H_
#define _IGAMEOBJECT_H_

#include "IEventListener.h"

#include "XML/tinyxml.h"

class ISubSystemEntity;

class IGameObject : public IEventListener
{
	public:
		typedef long long GameObjectID;

		virtual ~IGameObject() {};

		virtual GameObjectID getID() const = 0;
		virtual const std::string& getName() const = 0;
		virtual const std::string& getObjectClass() const = 0;

		virtual boost::any getProperty( const std::string& id ) = 0;

		virtual IGameObject* clone() = 0;

		virtual bool initialize( TiXmlElement* ) = 0;
};

#endif /* _IGAMEOBJECT_H_ */
