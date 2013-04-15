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
		ILoggerTarget( ILogReceiver* receiver )
		{
			this->m_logReceiver = receiver;
			this->m_stream = new std::ostringstream();
		}

		virtual ~ILoggerTarget()
		{
			this->m_logReceiver->sendLog( this->m_stream->str() );
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

};

#endif /* ILOGGER_H_ */
