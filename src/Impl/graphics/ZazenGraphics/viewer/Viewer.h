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

		void restoreViewport();

		void setupPerspective();
		void setupOrtho();

		const int& getHeight() const { return this->m_height; };
		const int& getWidth() const { return this->m_width; };

		void setFov( float fov ) { this->m_fov = fov; };

		void setNear( float nearDist ) { this->m_nearDist = nearDist; };
		void setFar( float farDist ) { this->m_farDist = farDist; };
	
		void resize( int, int );

		glm::mat4 createPerspProj() const;
		glm::mat4 createOrthoProj( bool, bool ) const;

		const glm::mat4& getViewMatrix() const { return this->m_viewMatrix; };
		const glm::mat4& getProjMatrix() const { return this->m_projectionMatrix; };
		const glm::mat4& getVPMatrix() const { return this->m_VPMatrix; };

		CullResult cullBB( const glm::vec3&, const glm::vec3& );

	protected:
		// overridden from Orientation
		virtual void matrixChanged();
	
		private:
		int m_width;
		int m_height;

		float m_fov;

		float m_nearDist;
		float m_farDist;
	
		glm::mat4 m_modelMatrix;
		glm::mat4 m_viewMatrix;
		glm::mat4 m_projectionMatrix;
		glm::mat4 m_VPMatrix; // combination of projection and view: projection * view

};

#endif /*VIEWER_H_*/
