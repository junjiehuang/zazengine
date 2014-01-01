#include "Viewer.h"

#include <GL/glew.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>

using namespace std;

Viewer::Viewer( int width, int height )
	: Orientation( m_modelMatrix )
{
	this->m_width = width;
	this->m_height = height;

	this->m_nearDist = 0.1f;
	this->m_farDist = 1000.0f;

	this->m_rightFrustum = 0.0;
	this->m_topFrustum = 0.0;

	this->m_fov = 90.0f;
}

Viewer::~Viewer()
{
}

void
Viewer::restoreViewport()
{
	// TODO: implement a guard: only do api-call when width or height changes

	glViewport( 0, 0, this->getWidth(), this->getHeight() );
}

glm::mat4
Viewer::createPerspProj() const
{
	return glm::perspective( this->m_fov, ( float ) this->m_width / ( float ) this->m_height, this->m_nearDist, this->m_farDist );
}

glm::mat4
Viewer::createOrthoProj( bool centered, bool normalizeZ ) const
{
	float left = 0.0f;
	float right = ( float ) this->m_width;
	float bottom = ( float ) this->m_height;
	float top = 0.0f;
	float zNear = this->m_nearDist;
	float zFar = this->m_farDist;

	if ( centered ) 
	{
		float halfWidth = ( float ) this->m_width / 2.0f;
		float halfHeight = ( float ) this->m_height / 2.0f;

		left = -halfWidth;
		right = halfWidth;
		bottom = -halfHeight;
		top = halfHeight;
	}

	if ( normalizeZ )
	{
		zNear = -1.0;
		zFar = 1.0;
	}

	// TODO: need to set m_topFrustum and m_rightFrustum

	return glm::ortho( left, right, bottom, top, zNear, zFar );
}

void
Viewer::setupPerspective()
{
	float aspect = ( float ) this->m_width / ( float ) this->m_height;
	float const rad = glm::radians( this->m_fov );
	float tanHalfFovy = tan( rad / 2.0f );
	
	this->m_topFrustum = this->m_nearDist * tanHalfFovy;
	this->m_rightFrustum = this->m_topFrustum * aspect;

	this->m_projectionMatrix = this->createPerspProj();
}

void
Viewer::setupOrtho()
{
	this->m_projectionMatrix = this->createOrthoProj( true, false );
}

void
Viewer::resize( int width, int height )
{
	this->m_width = width;
	this->m_height = height;
}

/* Not correctly working yet ( and pretty slow )
 * with this method an object will be clipped although one is inside it
 *
 * TODO fix it: bbMin & bbMax need to be in correct space to apply clippingcoords
 */
Viewer::CullResult
Viewer::cullBB( const glm::vec3& bbMin, const glm::vec3& bbMax )
{
	/*
	int counter = 0;
	glm::vec4 clippingCoordsMin = this->m_VPMatrix * glm::vec4( bbMin, 1.0 );
	glm::vec4 clippingCoordsMax = this->m_VPMatrix * glm::vec4( bbMax, 1.0 );

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
	this->m_VPMatrix = this->m_projectionMatrix * this->m_viewMatrix;
}
