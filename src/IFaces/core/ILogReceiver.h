#ifndef _ILOGRECEIVER_H_
#define _ILOGRECEIVER_H_

#include <string>

class ILogReceiver
{
	public:
		virtual ~ILogReceiver() {};

		virtual void logTrace( const std::string& ) const = 0;
		virtual void logDebug( const std::string& ) const = 0;
		virtual void logInfo( const std::string& ) const = 0;
		virtual void logWarning( const std::string& ) const = 0;
		virtual void logError( const std::string& ) const = 0;
		virtual void logFatal( const std::string& ) const = 0;

};

#endif /* _ILOGRECEIVER_H_ */
