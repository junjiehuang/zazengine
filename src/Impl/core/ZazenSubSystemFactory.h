/*
 * ZazenSubSystemFactory.h
 *
 *  Created on: 7 Aug 2011
 *      Author: jonathan
 */

#ifndef ZAZENSUBSYSTEMFACTORY_H_
#define ZAZENSUBSYSTEMFACTORY_H_

#include <core/ISubSystem.h>

class ZazenSubSystemFactory
{
	public:
		ZazenSubSystemFactory();
		~ZazenSubSystemFactory();

		ISubSystem* createSubSystem( const std::string& file, const std::string& type );
};

#endif /* ZAZENSUBSYSTEMFACTORY_H_ */
