/*
 * Event.cpp
 *
 *  Created on: Oct 1, 2010
 *      Author: jonathan
 */

#include "Event.h"

#include <algorithm>

Event::Event( const std::string& id )
	: id (id)
{
	this->target = 0;
	this->source = 0;
}

Event::Event(const Event& e)
	: id ( e.id )
{
	this->source = e.source;
	this->target = e.target;

	this->values = e.values;
}

Event::~Event()
{
}

void
Event::addValue( const std::string& id, const boost::any& v )
{
	this->values[ id ] = v;

	std::map<std::string, boost::any>::iterator findIter = this->values.find( "" );
	if ( findIter != this->values.end() )
	{

	}
}

boost::any&
Event::getValue( const std::string& id )
{
	return this->values[ id ];
}
