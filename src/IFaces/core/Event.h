/*
 * Event.h
 *
 *  Created on: 27.06.2010
 *      Author: Jonathan Thaler
 */

#ifndef EVENT_H_
#define EVENT_H_

#include <boost/any.hpp>

#include <map>
#include <string>

class IEventListener;

typedef std::string EventID;
//typedef long long EventID;

class Event
{
	public:
		Event( const std::string& id );
		Event( const Event& e );
		~Event();

		const EventID getID() const { return this->id; };

		void setTarget( IEventListener* t ) { this->target = t; };
		IEventListener* getTarget() { return this->target; };

		bool operator== ( const Event& e ) const { return this->id == e.id; };
		bool operator== ( const std::string& str ) const { return this->id == str; };
		bool operator== ( const char* str ) const { return this->id == str; };

		void addValue( const std::string& id, const boost::any& );
		boost::any& getValue( const std::string& id );

	private:
		const EventID id;
		IEventListener* source;
		IEventListener* target;

		std::map<std::string, boost::any> values;
};

#endif /* EVENT_H_ */
