/*
 * ZazenOISEntity.h
 *
 *  Created on: 09.03.2013
 *      Author: Jonathan Thaler
 */

#ifndef ZAZENOISENTITY_H_
#define ZAZENOISENTITY_H_

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

#endif /* ZAZENOISENTITY_H_ */
