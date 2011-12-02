/*
 * ZazenAI.cpp
 *
 *  Created on: 06.07.2010
 *      Author: joni
 */

#include "ZazenAI.h"

#include <iostream>

using namespace std;

ZazenAI::ZazenAI()
	: id ( "ZazenAI" ),
	type ("ai")
{

}

ZazenAI::~ZazenAI()
{

}

bool
ZazenAI::initialize( TiXmlElement* )
{
	cout << endl << "=============== ZazenAI initializing... ===============" << endl;

	cout << "================ ZazenAI initialized =================" << endl;

	return true;
}

bool
ZazenAI::shutdown()
{
	cout << endl << "=============== ZazenAI shutting down... ===============" << endl;

	cout << "================ ZazenAI shutdown =================" << endl;

	return true;
}

bool
ZazenAI::start()
{
	return true;
}

bool
ZazenAI::stop()
{
	return true;
}

bool
ZazenAI::pause()
{
	return true;
}

bool
ZazenAI::process(double factor)
{
	//cout << "ZazenAI::process()" << endl;

	return true;
}

bool
ZazenAI::finalizeProcess()
{
	//cout << "ZazenAI::finalizeProcess()" << endl;

	return true;
}

bool
ZazenAI::sendEvent( Event& e )
{
	return true;
}

ZazenAIEntity*
ZazenAI::createEntity( TiXmlElement* node, IGameObject* parent )
{
	ZazenAIEntity* entity = new ZazenAIEntity( parent );

	return entity;
}
