/*
 * ILogger.h
 *
 *  Created on: 11 April 2013
 *      Author: jonathan
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <core/ILogger.h>
#include <core/ILogReceiver.h>

#include <log4cplus/logger.h>

class Logger : public ILogger, public ILogReceiver
{
	public:
		Logger( const std::string& );
		~Logger();

		void sendLog( const std::string& ) const;

		ILoggerTarget logTargetTrace(); 

		void logTrace( const std::string& ) const;
		void logTrace( const std::ostream& ) const;

		void logDebug( const std::string& ) const;
		void logDebug( const std::ostream& ) const;

		void logInfo( const std::string& ) const;
		void logInfo( const std::ostream& ) const;

		void logWarning( const std::string& ) const;
		void logWarning( const std::ostream& ) const;

		void logError( const std::string& ) const;
		void logError( const std::ostream& ) const;

		void logFatal( const std::string& ) const;
		void logFatal( const std::ostream& ) const;

	private:
		log4cplus::Logger m_logger;

};

#endif /* ILOGGER_H_ */
