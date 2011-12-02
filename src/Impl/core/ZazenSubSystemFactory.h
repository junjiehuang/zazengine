/*
 * ZazenSubSystemFactory.h
 *
 *  Created on: 7 Aug 2011
 *      Author: jonathan
 */

#ifndef ZAZENSUBSYSTEMFACTORY_H_
#define ZAZENSUBSYSTEMFACTORY_H_

#include <string>
#include "IFaces/ISubSystem.h"

class ZazenSubSystemFactory
{
	public:
		ISubSystem* createSubSystem( const std::string& file );

		ZazenSubSystemFactory();
		~ZazenSubSystemFactory();

	private:
};

#endif /* ZAZENSUBSYSTEMFACTORY_H_ */
