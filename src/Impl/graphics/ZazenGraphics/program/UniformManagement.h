/*
 * UniformManagement.h
 *
 *  Created on: 16. April 2013
 *      Author: jonathan
 */

#ifndef UNIFORMMANAGEMENT_H_
#define UNIFORMMANAGEMENT_H_

#include "UniformBlock.h"

#include <boost/filesystem.hpp>

#include <map>
#include <string>

class Program;

class UniformManagement
{
	public:
		static bool freeAllBlocks();

		static UniformBlock* getBlock( const std::string& );

		static bool initUniforms( Program* );

	private:
		static std::map<std::string, UniformBlock*> m_uniformBlocks;

};

#endif /* UNIFORMBLOCKMANAGEMENT_H_ */
