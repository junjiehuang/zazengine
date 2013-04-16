#include "Logger.h"

#include <sstream>

#include <log4cplus/loggingmacros.h>
#include <log4cplus/configurator.h>
#include <iomanip>

using namespace std;

Logger::Logger( const std::string& name )
{
	this->m_logger = log4cplus::Logger::getInstance( log4cplus::helpers::towstring( name.c_str() ) );
}

Logger::~Logger()
{
}

void
Logger::logTrace( const std::string& text ) const
{
	LOG4CPLUS_TRACE( m_logger, text.c_str() );
}

void
Logger::logDebug( const std::string& text ) const
{
	LOG4CPLUS_DEBUG( m_logger, text.c_str() );
}

void
Logger::logInfo( const std::string& text ) const
{
	LOG4CPLUS_INFO( m_logger, text.c_str() );
}

void
Logger::logWarning( const std::string& text ) const
{
	LOG4CPLUS_WARN( m_logger, text.c_str() );
}

void
Logger::logError( const std::string& text ) const
{
	LOG4CPLUS_ERROR( m_logger, text.c_str() );
}

void
Logger::logFatal( const std::string& text ) const
{
	LOG4CPLUS_FATAL( m_logger, text.c_str() );
}
