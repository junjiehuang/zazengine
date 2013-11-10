#ifndef _ZAZENOISENTITY_H_
#define _ZAZENOISENTITY_H_

#include <input/IInputEntity.h>

class ZazenOISEntity : public IInputEntity
{
	public:
		friend class ZazenGraphics;

		ZazenOISEntity( IGameObject* p );
		virtual ~ZazenOISEntity();

		const std::string& getType() const { return this->m_type; };

		virtual bool sendEvent( Event& e );

	private:
		std::string m_type;
};

#endif /* _ZAZENOISENTITY_H_ */
