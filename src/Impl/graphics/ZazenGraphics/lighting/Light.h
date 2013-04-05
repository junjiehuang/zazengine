/*
 * Light.h
 *
 *  Created on: Feb 4, 2011
 *      Author: jonathan
 */

#ifndef LIGHT_H_
#define LIGHT_H_

#include "../Geometry/GeomType.h"

#include "../Scene/Viewer.h"

#include "../Framebuffer/RenderTarget.h"

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

	virtual ~Light();

	LightType getType() const { return this->m_type; };

	float getFalloff() const { return this->m_falloff; };

	bool isShadowCaster() const { return this->m_shadowCaster; };

	void setColor( const glm::vec4& color ) { this->m_color = color; };
	const glm::vec4& getColor() const { return this->m_color; };

	void setBoundingGeometry( GeomType* boundingGeom ) { this->m_boundingGeom = boundingGeom; };
	GeomType* getBoundingGeometry() { return this->m_boundingGeom; };

	RenderTarget* getShadowMap() { return this->m_shadowMap; };

 private:
	Light( int, int, LightType, bool );

	float m_falloff;
	bool m_shadowCaster;

	LightType m_type;
	glm::vec4 m_color;

	GeomType* m_boundingGeom;

	RenderTarget* m_shadowMap;

	bool createShadowMap( int width, int height );

};

#endif /* LIGHT_H_ */
