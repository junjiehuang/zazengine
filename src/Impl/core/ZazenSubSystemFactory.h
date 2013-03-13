/*
 * ZazenSubSystemFactory.h
 *
 *  Created on: 7 Aug 2011
 *      Author: jonathan
 */

#ifndef ZAZENSUBSYSTEMFACTORY_H_
#define ZAZENSUBSYSTEMFACTORY_H_

#include <string>
#include <core/ISubSystem.h>

class ZazenSubSystemFactory
{
	public:
		ISubSystem* createSubSystem( const std::string& file, const std::string& type );

		ZazenSubSystemFactory();
		~ZazenSubSystemFactory();

	private:
};

#endif /* ZAZENSUBSYSTEMFACTORY_H_ */
