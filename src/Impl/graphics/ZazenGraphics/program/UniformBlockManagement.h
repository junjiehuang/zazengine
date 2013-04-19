/*
 * UniformBlockManagement.h
 *
 *  Created on: 16. April 2013
 *      Author: jonathan
 */

#ifndef UNIFORMBLOCKMANAGEMENT_H_
#define UNIFORMBLOCKMANAGEMENT_H_

#include "UniformBlock.h"

#include <boost/filesystem.hpp>

#include <map>
#include <string>

class Program;

class UniformBlockManagement
{
	public:
		static bool init( const boost::filesystem::path& );
		static bool freeAll();

		static UniformBlock* get( const std::string& );

		static bool initUniformBlocks( Program* );

	private:
		static std::map<std::string, UniformBlock*> m_uniformBlocks;

};

#endif /* UNIFORMBLOCKMANAGEMENT_H_ */
