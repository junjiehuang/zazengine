/*
 * ILogger.h
 *
 *  Created on: 11 April 2013
 *      Author: jonathan
 */

#ifndef ILOGRECEIVER_H_
#define ILOGRECEIVER_H_

#include <string>

class ILogReceiver
{
	public:
		virtual ~ILogReceiver() {};

		virtual void sendLog( const std::string& ) const = 0;

};

#endif /* ILOGRECEIVER_H_ */
