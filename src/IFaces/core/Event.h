#ifndef _EVENT_H_
#define _EVENT_H_

#include <boost/any.hpp>

#include <map>
#include <string>

class IEventListener;

typedef std::string EventID;
//typedef long long EventID;

class Event
{
	public:
		Event( const std::string& id ) 	
			: id (id)
		{
			this->target = 0;
			this->source = 0;
		};

		Event( const Event& e )
			: id ( e.id )
		{
			this->source = e.source;
			this->target = e.target;

			this->values = e.values;
		};

		~Event() {};

		const EventID getID() const { return this->id; };

		void setTarget( IEventListener* t ) { this->target = t; };
		IEventListener* getTarget() { return this->target; };

		bool operator== ( const Event& e ) const { return this->id == e.id; };
		bool operator== ( const std::string& str ) const { return this->id == str; };
		bool operator== ( const char* str ) const { return this->id == str; };

		void
		addValue( const std::string& id, const boost::any& value )
		{
			this->values[ id ] = value;

			std::map<std::string, boost::any>::iterator findIter = this->values.find( "" );
			if ( findIter != this->values.end() )
			{

			}
		};

		boost::any&
		getValue( const std::string& id )
		{
			return this->values[ id ];
		};

	private:
		const EventID id;
		IEventListener* source;
		IEventListener* target;

		std::map<std::string, boost::any> values;
};

#endif /* _EVENT_H_ */
