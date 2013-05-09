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

void
Instance::recalculateDistance( const glm::mat4& viewMatrix )
{
	// calculate model-view for this instance
	glm::mat4 modelView = viewMatrix * this->m_modelMatrix;
	// calculate center of instance in view-space
	glm::vec4 vsCenter = modelView * glm::vec4( this->geom->getCenter(), 1.0 );

	// distance from the viewer is the z-component of the center
	this->distance = vsCenter.z;
}
