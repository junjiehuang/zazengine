#include "Instance.h"

Instance::Instance( )
	: Orientation( m_modelMatrix )
{
	this->visible = true;
	this->distance = 0;
	this->lastFrame = 0;
	
	this->geom = 0;
	this->material = 0;
}

Instance::~Instance()
{
}

float
Instance::recalculateDistance()
{
	// TODO: implement

	//Vector wTranfCenter( this->geom->getCenter() );
	//this->transform.matrix.transform( wTranfCenter );
	
	//this->distance = wTranfCenter.length();
	this->distance = 0;
	
	return this->distance;
}
