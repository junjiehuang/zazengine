/*
 * IAi.h
 *
 *  Created on: 05.07.2010
 *      Author: joni
 */

#ifndef IAI_H_
#define IAI_H_

#include "ISubSystem.h"
#include "IAIEntity.h"

class IAi : public ISubSystem
{
	public:
		virtual ~IAi() {};

		virtual IAIEntity* createEntity( TiXmlElement*, IGameObject* parent ) = 0;

};

#endif /* IAI_H_ */
