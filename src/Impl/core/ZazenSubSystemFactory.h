#ifndef _ZAZENSUBSYSTEMFACTORY_H_
#define _ZAZENSUBSYSTEMFACTORY_H_

#include <core/ISubSystem.h>

class ZazenSubSystemFactory
{
	public:
		ZazenSubSystemFactory();
		~ZazenSubSystemFactory();

		ISubSystem* createSubSystem( const std::string& file, const std::string& type );
};

#endif /* _ZAZENSUBSYSTEMFACTORY_H_ */
