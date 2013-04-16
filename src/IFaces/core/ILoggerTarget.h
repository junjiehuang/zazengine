/*
 * ILogger.h
 *
 *  Created on: 15 April 2013
 *      Author: jonathan
 */

#ifndef ILOGGERTARGET_H_
#define ILOGGERTARGET_H_

#include "ILogReceiver.h"

#include <sstream>

class ILoggerTarget
{
	public:
		enum LOG_LEVEL
		{
			TARGET_TRACE,
			TARGET_DEBUG,
			TARGET_INFO,
			TARGET_WARNING,
			TARGET_ERROR,
			TARGET_FATAL
		};

		ILoggerTarget( ILogReceiver* receiver, LOG_LEVEL level )
		{
			this->m_level = level;
			this->m_logReceiver = receiver;
			this->m_stream = new std::ostringstream();
		}

		virtual ~ILoggerTarget()
		{
			if ( TARGET_TRACE == this->m_level )
			{
				this->m_logReceiver->logTrace( this->m_stream->str() );
			}
			else if ( TARGET_DEBUG == this->m_level )
			{
				this->m_logReceiver->logDebug( this->m_stream->str() );
			}
			else if ( TARGET_INFO == this->m_level )
			{
				this->m_logReceiver->logInfo( this->m_stream->str() );
			}
			else if ( TARGET_WARNING == this->m_level )
			{
				this->m_logReceiver->logWarning( this->m_stream->str() );
			}
			else if ( TARGET_ERROR == this->m_level )
			{
				this->m_logReceiver->logError( this->m_stream->str() );
			}
			else if ( TARGET_FATAL == this->m_level )
			{
				this->m_logReceiver->logFatal( this->m_stream->str() );
			}

			delete this->m_stream;
		}

		template<typename T> ILoggerTarget& operator<<( T const& value )
		{
			*m_stream << value;
			return *this;
		}

	protected:
		std::ostringstream* m_stream;

	private:
		ILogReceiver* m_logReceiver;
		LOG_LEVEL m_level;

};

#endif /* ILOGGER_H_ */
