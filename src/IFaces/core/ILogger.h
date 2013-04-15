/*
 * ILogger.h
 *
 *  Created on: 11 April 2013
 *      Author: jonathan
 */

#ifndef ILOGGER_H_
#define ILOGGER_H_

#include "ILoggerTarget.h"

#include <string>

class ILogger
{
	public:
		virtual ~ILogger() {};

		virtual void logTrace( const std::string& ) const = 0;
		virtual ILoggerTarget logTargetTrace() = 0; 

		virtual void logTrace( const std::ostream& ) const = 0;

		virtual void logDebug( const std::string& ) const = 0;
		virtual void logDebug( const std::ostream& ) const = 0;

		virtual void logInfo( const std::string& ) const = 0;
		virtual void logInfo( const std::ostream& ) const = 0;

		virtual void logWarning( const std::string& ) const = 0;
		virtual void logWarning( const std::ostream& ) const = 0;

		virtual void logError( const std::string& ) const = 0;
		virtual void logError( const std::ostream& ) const = 0;

		virtual void logFatal( const std::string& ) const = 0;
		virtual void logFatal( const std::ostream& ) const = 0;

};

#endif /* ILOGGER_H_ */
