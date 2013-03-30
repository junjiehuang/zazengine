/*
 * Light.h
 *
 *  Created on: Feb 4, 2011
 *      Author: jonathan
 */

#ifndef LIGHT_H_
#define LIGHT_H_

#include "../Scene/Viewer.h"

#include "../renderer/RenderTarget.h"

#include <GL/glew.h>

class Light : public Viewer
{
 public:
	enum LightType {
		SPOT = 0,
		DIRECTIONAL,
		POINT
	};

	static Light* createSpotLight( float, int, int, bool );
	static Light* createDirectionalLight( int, int, bool );
	static Light* createPointLight( int, bool );

	virtual ~Light();

	LightType getType() const { return this->m_type; };

	float getFalloff() const { return this->m_falloff; };

	bool isShadowCaster() const { return this->m_shadowCaster; };

	glm::vec4 getPosition();
	glm::vec4 getDirection();

	void setColor( const glm::vec4& color ) { this->m_color = color; };
	const glm::vec4& getColor() const { return this->m_color; };

	RenderTarget* getShadowMap() { return this->m_shadowMap; };
	GLuint* getShadowCubeMap() { return this->m_cubeShadowMap; };

 private:
	Light( int, int, LightType, bool );

	float m_falloff;
	bool m_shadowCaster;

	LightType m_type;
	glm::vec4 m_color;

	RenderTarget* m_shadowMap;
	GLuint m_cubeShadowMap[ 6 ];

	bool createShadowMap( int width, int height );
	bool createShadowCubeMap( int side );

};

#endif /* LIGHT_H_ */
