/*
 * Viewer.h
 *
 *  Created on: Feb 9, 2011
 *      Author: jonathan
 */


#ifndef VIEWER_H_
#define VIEWER_H_

#include "../Orientation/Orientation.h"

/* Can be used as the view-point of a camera or light
 *
 * in opengl the camera is always in the 0,0,0 origin and points down
 * the -Z achsis. the camera itself is not modified but instead one has
 * to apply the inverse operation on the viewing matrix to achive a
 * camera/viewer transformation
 *
 * IMPLEMENTED: it is nice to use only modelMatrix for the orientation-manipulation
 * and derive viewMatrix as the inverse of the modelMatrix.
 * The pro of this approach is, that we, we can easily switch between modeling an object in world-space and
 * switch to its point-of-view. this becomes very important for lights because they need to be
 * placed within the scene with modeling-transformations to which camera-viewing must be applied
 * but it must also be able to render the scene from the point-of-view of the light for shadowmap rendering.
 */

class Viewer : public Orientation
{
 public:
	enum CullResult {
		INSIDE = 0,
		OUTSIDE,
		INTERSECTING
	};

	Viewer( int, int );
	~Viewer();

	glm::mat4 m_modelMatrix;
	glm::mat4 m_viewMatrix;
	glm::mat4 m_projectionMatrix;

	glm::mat4 m_PVMatrix;

    void setupPerspective();
    void setupOrtho();

    int getHeight() { return this->height; };
    int getWidth() { return this->width; };

    void setFov( float fov ) { this->fov = fov; };

	void setNear( float near ) { this->nearDist = near; };
	void setFar( float far ) { this->farDist = far; };
	
	void resize( int, int );

	CullResult cullBB( const glm::vec3&, const glm::vec3& );

 protected:
	// overridden from Orientation
	virtual void matrixChanged();
	
 private:
	int width;
	int height;

	float fov;

	float nearDist;
	float farDist;
	
};

#endif /*VIEWER_H_*/
