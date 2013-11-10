#ifndef _ISUBSYSTEM_H_
#define _ISUBSYSTEM_H_

#include "ISubSystemEntity.h"
#include "IEventListener.h"

#include "XML/tinyxml.h"

#include <string>

class ICore;

class ISubSystem : public IEventListener
{
	public:
		typedef ISubSystem* ( *constructor_func ) ( const char* id, ICore* core );
		typedef void ( *destructor_func ) ( ISubSystem* comp );

		typedef struct _LibStruct
		{
			std::string fileName;
			void* libHandle;
			constructor_func constructorFunc;
			destructor_func destructorFunc;
		} LibStruct;

		enum SubSystemState {
			SUBSYSTEM_LOADED = 0,
			SUBSYSTEM_DEINITIALIZED,
			SUBSYSTEM_INITIALIZING,
			SUBSYSTEM_INITIALIZED,
			SUBSYSTEM_DEINITIALIZING
		};

		virtual ~ISubSystem() {};

		virtual const std::string& getID() const = 0;
		virtual const std::string& getType() const = 0;

		virtual bool isAsync() const = 0;

		virtual bool start() = 0;
		virtual bool stop() = 0;
		virtual bool pause() = 0;

		virtual bool initialize( TiXmlElement* ) = 0;
		virtual bool shutdown() = 0;

		virtual bool process( double ) = 0;		// runs simulations/calculations for this frame: in this step all calculations are done either synced or threaded
		virtual bool finalizeProcess() = 0;		// finishes processing and must block until threads are finished: in this step all calculations are finished and maybe exchanged through consumer/producer

		virtual ISubSystemEntity* createEntity( TiXmlElement*, IGameObject* parent ) = 0;

		LibStruct* m_libStruct;
};
#endif /* _ISUBSYSTEM_H_ */
