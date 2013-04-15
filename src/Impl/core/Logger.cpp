#include "Logger.h"

#include "LoggerTarget.h"

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

ILoggerTarget
Logger::logTargetTrace()
{
	return ILoggerTarget( this );
}

void
Logger::sendLog( const std::string& text ) const
{
	LOG4CPLUS_TRACE( m_logger, text.c_str() );
}


void
Logger::logTrace( const std::string& text ) const
{
	LOG4CPLUS_TRACE( m_logger, text.c_str() );
}

void
Logger::logTrace( const std::ostream& os ) const
{
	stringstream ss;
	ss << os;

	LOG4CPLUS_TRACE( m_logger, ss.str().c_str() );
}

void
Logger::logDebug( const std::string& text ) const
{
	LOG4CPLUS_DEBUG( m_logger, text.c_str() );
}

void
Logger::logDebug( const std::ostream& os ) const
{
	stringstream ss;
	ss << os;

	LOG4CPLUS_DEBUG( m_logger, ss.str().c_str() );
}

void
Logger::logInfo( const std::string& text ) const
{
	LOG4CPLUS_INFO( m_logger, text.c_str() );
}

void
Logger::logInfo( const std::ostream& os ) const
{
	const stringstream& ss = dynamic_cast<const stringstream&>( os );
	std::string str = ss.str();



	LOG4CPLUS_INFO( m_logger, str.c_str() );
}

void
Logger::logWarning( const std::string& text ) const
{
	LOG4CPLUS_WARN( m_logger, text.c_str() );
}

void
Logger::logWarning( const std::ostream& os ) const
{
	stringstream ss;
	ss << os;

	LOG4CPLUS_WARN( m_logger, ss.str().c_str() );
}

void
Logger::logError( const std::string& text ) const
{
	LOG4CPLUS_ERROR( m_logger, text.c_str() );
}

void
Logger::logError( const std::ostream& os ) const
{
	stringstream ss;
	ss << os;

	LOG4CPLUS_ERROR( m_logger, ss.str().c_str() );
}

void
Logger::logFatal( const std::string& text ) const
{
	LOG4CPLUS_FATAL( m_logger, text.c_str() );
}

void
Logger::logFatal( const std::ostream& os ) const
{
	stringstream ss;
	ss << os;

	LOG4CPLUS_FATAL( m_logger, ss.str().c_str() );
}
