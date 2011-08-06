/*
 * ZazenGraphicsEntity.h
 *
 *  Created on: 08.07.2010
 *      Author: joni
 */

#ifndef ZAZENGRAPHICSENTITY_H_
#define ZAZENGRAPHICSENTITY_H_

#include "../../../Core/SubSystems/IFaces/IGraphicsEntity.h"
#include "Orientation/Orientation.h"

class ZazenGraphicsEntity : public IGraphicsEntity
{
	public:
		friend class ZazenGraphics;

		ZazenGraphicsEntity( IGameObject* p );
		virtual ~ZazenGraphicsEntity();

		const std::string& getType() const { return this->m_type; };

		virtual void setOrientation( const float* pos, const float* rot);

		virtual bool sendEvent( Event& e );

	private:
		std::string m_type;

		Orientation* m_orientation;
};

#endif /* ZAZENGRAPHICSENTITY_H_ */
