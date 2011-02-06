#ifndef CAMERA_H_
#define CAMERA_H_

#include <glm/glm.hpp>

enum CullResult {
	INSIDE = 0,
	OUTSIDE,
	INTERSECTING
};

class Camera
{
 public:
	Camera(float, int, int);
	~Camera();

	glm::mat4 m_viewingMatrix;
	glm::mat4 m_projectionMatrix;

    void setupPerspective();
    void setupOrtho();

    glm::vec3 getPosition();

    int getHeight() { return this->height; };
    int getWidth() { return this->width; };


	void changeNearClip(float);
	void changeFarClip(float);
	
	void resize(int, int);
	void changeFov(float);
	
	/*
	void setPosition(float, float, float);

	void changeHeading(float);
	void changePitch(float);
	void changeRoll(float);
	void strafeForward(float);
	void strafeRight(float);
	void strafeUp(float);
	void strafe(float*, float);

	CullResult cullBB( const glm::vec3&, const glm::vec3& );
	CullResult cullSphere( const glm::vec3&, float );
	*/
	
 private:
	float width;
	float height;

	float angle;
	float ratio;

	float nearDist;
	float farDist;
	
	float frustum[6][4];
	
	void recalculateFrustum();
	
};

#endif /*CAMERA_H_*/
