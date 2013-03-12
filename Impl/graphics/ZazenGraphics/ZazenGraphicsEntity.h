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

class ZazenGraphicsEntity : public IGraphicsEntity
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

	private:
		std::string m_type;

		std::list<int> m_keyDown;

		bool m_isAnimated;
		float m_animRoll;
		float m_animPitch;
		float m_heading;

		Orientation* m_orientation;
};

#endif /* ZAZENGRAPHICSENTITY_H_ */
