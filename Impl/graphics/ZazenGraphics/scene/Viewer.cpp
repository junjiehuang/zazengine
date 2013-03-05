#include "Viewer.h"

#include <GL/glew.h>

//#include <glm/gtc/matrix_projection.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>

using namespace std;

Viewer::Viewer( int width, int height )
	: Orientation( m_modelMatrix )
{
	this->width = width;
	this->height = height;

	this->nearDist = 1;
	this->farDist = 1000;

	this->fov = 90.0f;
}

Viewer::~Viewer()
{
}

void
Viewer::setupPerspective()
{
	this->m_projectionMatrix = glm::perspective( this->fov, ( float ) this->width / ( float ) this->height, this->nearDist, this->farDist );

	// TODO: remove this fixed-function stuff
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();

	glLoadMatrixf( glm::value_ptr( this->m_projectionMatrix ) );

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void
Viewer::setupOrtho()
{
	this->m_projectionMatrix = glm::ortho( 0.0f, this->width, this->height, 0.0f, -1.0f, 1.0f );

	// TODO: remove this fixed-function stuff
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();

	glLoadMatrixf( glm::value_ptr( this->m_projectionMatrix ) );

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void
Viewer::resize( int width, int height )
{
	this->width = (float) width;
	this->height = (float) height;
}

/* Not correctly working yet ( and pretty slow )
 * with this method an object will be clipped although one is inside it
 */
Viewer::CullResult
Viewer::cullBB( const glm::vec3& bbMin, const glm::vec3& bbMax )
{
	int counter = 0;
	glm::vec4 clippingCoordsMin = this->m_PVMatrix * glm::vec4( bbMin, 1.0 );
	glm::vec4 clippingCoordsMax = this->m_PVMatrix * glm::vec4( bbMax, 1.0 );

	// perspective division to NormalizedDeviceCoords
	clippingCoordsMin /= clippingCoordsMin[ 3 ];
	clippingCoordsMax /= clippingCoordsMax[ 3 ];

	if ( clippingCoordsMin[ 0 ] < -1 || clippingCoordsMin[ 0 ] > 1 )
		counter++;

	if ( clippingCoordsMin[ 1 ] < -1 || clippingCoordsMin[ 1 ] > 1 )
		counter++;

	if ( clippingCoordsMin[ 2 ] < -1 || clippingCoordsMin[ 2 ] > 1 )
		counter++;


	if ( clippingCoordsMax[ 0 ] < -1 || clippingCoordsMax[ 0 ] > 1 )
		counter++;

	if ( clippingCoordsMax[ 1 ] < -1 || clippingCoordsMax[ 1 ] > 1 )
		counter++;

	if ( clippingCoordsMax[ 2 ] < -1 || clippingCoordsMax[ 2 ] > 1 )
		counter++;

	/*
	if ( counter > 0 && counter < 6 )
		return INTERSECTING;

	if ( counter == 0 )
		return INSIDE;
	*/

	return INSIDE;
}

void
Viewer::matrixChanged()
{
	// viewing-matrix is the inverse of the model-matrix
	// in opengl the camera (or a viewer) is positioned in the origin 0,0,0 and points down the -Z achsis
	// not the camera is moved, the objecs are moved with the inverse of the cameras modeling
	// viewingMatrix is changed by Orientation -> recalculate ProjectionView-Matrix
	this->m_viewMatrix = glm::inverse( this->m_modelMatrix );
	this->m_PVMatrix = this->m_projectionMatrix * this->m_viewMatrix;
}
