#ifndef _LIGHT_H_
#define _LIGHT_H_

#include "../Geometry/Mesh.h"

#include "../Viewer/Viewer.h"

#include "../Framebuffer/RenderTarget.h"

#include <GL/glew.h>

class Light : public Viewer
{
	public:
		friend class ZazenGraphics;

		enum LightType {
			SPOT = 0,
			DIRECTIONAL,
			POINT
		};

		static Light* createSpot( float, int, int, bool );
		static Light* createDirectional( int, int, bool );
		static Light* createPoint( int, bool );

		virtual ~Light();

		LightType getType() const { return this->m_type; };

		float getFalloff() const { return this->m_falloff; };

		bool isShadowCaster() const { return this->m_shadowCaster; };

		const glm::vec3& getColor() const { return this->m_color; };
		const glm::vec3& getAttenuation() const { return this->m_attenuation; };
		const glm::vec2& getSpecular() const { return this->m_specular; };

		Mesh* getBoundingMesh() { return this->m_boundingMesh; };

		RenderTarget* getShadowMap() { return this->m_shadowMap; };

	private:
		Light( int, int, LightType, bool );

		float m_falloff;
		bool m_shadowCaster;

		LightType m_type;
		glm::vec3 m_color;
		glm::vec3 m_attenuation;
		glm::vec2 m_specular;

		Mesh* m_boundingMesh;

		RenderTarget* m_shadowMap;

		bool createShadowMap();
};

#endif /* _LIGHT_H_ */
