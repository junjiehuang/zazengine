/*
 * ZazenGraphicsEntity.h
 *
 *  Created on: 08.07.2010
 *      Author: Jonathan Thaler
 */

#ifndef ZAZENGRAPHICSENTITY_H_
#define ZAZENGRAPHICSENTITY_H_

#include <graphics/IGraphicsEntity.h>

#include "Orientation/Orientation.h"
#include "Material/Material.h"
#include "Geometry/MeshNode.h"
#include "Animation/Animation.h"
#include "Lighting/Light.h"

#include <vector>

class ZazenGraphicsEntity : public IGraphicsEntity, public Orientation
{
	public:
		friend class ZazenGraphics;

		ZazenGraphicsEntity( IGameObject* p );
		virtual ~ZazenGraphicsEntity();

		const std::string& getType() const { return this->m_type; };

		virtual void setOrientation( const float* pos, const float* rot );
		virtual void setAnimation( float heading, float roll, float pitch );

		virtual void update();

		virtual bool sendEvent( Event& e );

		Material* getMaterial() const { return this->m_material; };
		Animation* getAnimation() const { return this->m_activeAnimation; };
		MeshNode* getMeshNode() const { return this->m_rootMeshNode; };
		Viewer* getCamera() const { return this->m_camera; };
		Light* getLight() const { return this->m_light; };

		float getDistance() const { return this->m_distance; };

		void recalculateDistance( const glm::mat4& viewMatrix );

	private:
		std::string m_type;

		std::list<int> m_keyDown;

		bool m_isAnimated;
		float m_animRoll;
		float m_animPitch;
		float m_heading;

		bool m_visible;
		float m_distance;
		long m_lastFrame;

		std::vector<Animation*> m_allAnimations;
		Animation* m_activeAnimation;

		Material* m_material;
		MeshNode* m_rootMeshNode;

		Light* m_light;
		Viewer* m_camera;

		glm::mat4 m_modelMatrix;
		
		void matrixChanged();
};

#endif /* ZAZENGRAPHICSENTITY_H_ */
