#ifndef _LIGHT_H_
#define _LIGHT_H_

#include "../Geometry/MeshStatic.h"

#include "../Viewer/Viewer.h"

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

		static Light* createSpot( float, int, int, bool );
		static Light* createDirectional( int, int, bool );
		static Light* createPoint( int, bool );

		virtual ~Light();

		LightType getType() const { return this->m_type; };

		float getFalloff() const { return this->m_falloff; };

		bool isShadowCaster() const { return this->m_shadowCaster; };

		void setColor( const glm::vec4& color ) { this->m_color = color; };
		const glm::vec4& getColor() const { return this->m_color; };

		void setBoundingMesh( MeshStatic* boundingGeom ) { this->m_boundingMesh = boundingGeom; };
		MeshStatic* getBoundingMesh() { return this->m_boundingMesh; };

		RenderTarget* getShadowMap() { return this->m_shadowMap; };

	private:
		Light( int, int, LightType, bool );

		float m_falloff;
		bool m_shadowCaster;

		LightType m_type;
		glm::vec4 m_color;

		MeshStatic* m_boundingMesh;

		RenderTarget* m_shadowMap;

		bool createShadowMap();
};

#endif /* _LIGHT_H_ */
