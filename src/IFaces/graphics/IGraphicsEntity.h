#ifndef _IGRAPHICSENTITY_H_
#define _IGRAPHICSENTITY_H_

#include <core/ISubSystemEntity.h>

class IGraphicsEntity : public ISubSystemEntity
{
	public:
		IGraphicsEntity( IGameObject* p ) : ISubSystemEntity( p ) {};
		virtual ~IGraphicsEntity() {};

		virtual void setOrientation( const float* pos, const float* rot) = 0;
		
		virtual void setAnimation( float heading, float roll, float pitch ) = 0;
};

#endif /* _IGRAPHICSENTITY_H_ */
