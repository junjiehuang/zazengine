/*
 * ILogger.h
 *
 *  Created on: 11 April 2013
 *      Author: jonathan
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <core/ILogger.h>

#include <log4cplus/logger.h>

class Logger : public ILogger
{
	public:
		Logger( const std::string& );
		~Logger();

		void logTrace( const std::string& ) const;
		void logDebug( const std::string& ) const;
		void logInfo( const std::string& ) const;
		void logWarning( const std::string& ) const;
		void logError( const std::string& ) const;
		void logFatal( const std::string& ) const;

	private:
		log4cplus::Logger m_logger;

};

#endif /* ILOGGER_H_ */
