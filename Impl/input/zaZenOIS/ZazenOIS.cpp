/*
 *  ZazenOIS.cpp
 *  Zazengine
 *
 *  Created by Jonathan Thaler on 09.03.13
 *
 */

#include "ZazenOIS.h"

#include <iostream>

using namespace std;

ZazenOIS::ZazenOIS( const std::string& id, ICore* core )
	: id ( id ),
	type ( "input" ),
	m_core( core )
{
	
}

ZazenOIS::~ZazenOIS()
{
	
}

bool
ZazenOIS::initialize( TiXmlElement* element )
{
	cout << endl << "=============== ZazenOIS initializing... ===============" << endl;

	cout << "================ ZazenOIS initialized =================" << endl;

	return true;
}

bool
ZazenOIS::shutdown()
{
	cout << endl << "=============== ZazenOIS shutting down... ===============" << endl;

	cout << "================ ZazenOIS shutdown =================" << endl;

	return true;
}

bool
ZazenOIS::start()
{
	return true;
}

bool
ZazenOIS::stop()
{
	return true;
}

bool
ZazenOIS::pause()
{
	return true;
}

bool
ZazenOIS::process( double factor )
{
	// TODO implement using OIS

	return true;
}

bool
ZazenOIS::finalizeProcess()
{
	return true;
}

bool
ZazenOIS::sendEvent( Event& e )
{
	return true;
}

ZazenOISEntity*
ZazenOIS::createEntity( TiXmlElement* cfgNode, IGameObject* parent )
{
	// TODO implement when necessary
	return 0;
}

extern "C"
{	
	__declspec( dllexport ) ISubSystem*
	createInstance ( const char* id, ICore* core )
	{
		return new ZazenOIS( id, core );
	}

	__declspec( dllexport ) void
	deleteInstance ( ISubSystem* subSys )
	{
		if ( 0 == subSys )
			return;

		if ( 0 == dynamic_cast<ZazenOIS*>( subSys ) )
			return;

		delete subSys;
	}
}
