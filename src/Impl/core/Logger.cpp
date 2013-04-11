#include "Logger.h"

#include <sstream>

#include <log4cplus/loggingmacros.h>
#include <log4cplus/configurator.h>
#include <iomanip>

using namespace std;

Logger::Logger( const std::string& name )
{
	// TODO correct name
	this->m_logger = log4cplus::Logger::getInstance( LOG4CPLUS_TEXT( "name" ) );
}

Logger::~Logger()
{
}

void
Logger::logTrace( const std::string& text ) const
{
	LOG4CPLUS_TRACE( m_logger, stringstream( text ) );
}

void
Logger::logTrace( const std::ostream& os ) const
{
	LOG4CPLUS_TRACE( m_logger, os );
}

void
Logger::logDebug( const std::string& text ) const
{
	LOG4CPLUS_DEBUG( m_logger, stringstream( text ) );
}

void
Logger::logDebug( const std::ostream& os ) const
{
	LOG4CPLUS_DEBUG( m_logger, os );
}

void
Logger::logInfo( const std::string& text ) const
{
	LOG4CPLUS_INFO( m_logger, stringstream( text ) );
}

void
Logger::logInfo( const std::ostream& os ) const
{
	LOG4CPLUS_INFO( m_logger, os );
}

void
Logger::logWarning( const std::string& text ) const
{
	LOG4CPLUS_WARN( m_logger, stringstream( text ) );
}

void
Logger::logWarning( const std::ostream& os ) const
{
	LOG4CPLUS_WARN( m_logger, os );
}

void
Logger::logError( const std::string& text ) const
{
	LOG4CPLUS_ERROR( m_logger, stringstream( text ) );
}

void
Logger::logError( const std::ostream& os ) const
{
	LOG4CPLUS_ERROR( m_logger, os );
}

void
Logger::logFatal( const std::string& text ) const
{
	LOG4CPLUS_FATAL( m_logger, stringstream( text ) );
}

void
Logger::logFatal( const std::ostream& os ) const
{
	LOG4CPLUS_FATAL( m_logger, os );
}
