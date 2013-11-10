#ifndef _ILOGGER_H_
#define _ILOGGER_H_

#include "ILoggerTarget.h"

#include <string>

class ILogger : public ILogReceiver
{
	public:
		virtual ~ILogger() {};

		virtual void logTrace( const std::string& ) const = 0;
		virtual void logDebug( const std::string& ) const = 0;
		virtual void logInfo( const std::string& ) const = 0;
		virtual void logWarning( const std::string& ) const = 0;
		virtual void logError( const std::string& ) const = 0;
		virtual void logFatal( const std::string& ) const = 0;

		ILoggerTarget logTrace() { return ILoggerTarget( this, ILoggerTarget::TARGET_TRACE ); }; 
		ILoggerTarget logDebug() { return ILoggerTarget( this, ILoggerTarget::TARGET_DEBUG ); }; 
		ILoggerTarget logInfo() { return ILoggerTarget( this, ILoggerTarget::TARGET_INFO ); }; 
		ILoggerTarget logWarning() { return ILoggerTarget( this, ILoggerTarget::TARGET_WARNING ); }; 
		ILoggerTarget logError() { return ILoggerTarget( this, ILoggerTarget::TARGET_ERROR ); }; 
		ILoggerTarget logFatal() { return ILoggerTarget( this, ILoggerTarget::TARGET_FATAL ); }; 

};

#endif /* _ILOGGER_H_ */
