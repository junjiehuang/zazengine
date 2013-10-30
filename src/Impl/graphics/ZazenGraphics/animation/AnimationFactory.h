/*
 *  AnimationFactory.cpp
 *  zaZengine
 *
 *  Created by Jonathan Thaler on 30.10.13.
 *
 */

#ifndef _ANIMATION_FACTORY_H_
#define _ANIMATION_FACTORY_H_

#include "Animation.h"

#include <assimp/scene.h>

#include <boost/filesystem.hpp>

#include <map>
#include <string>

class AnimationFactory
{
	public:
		static void setDataPath( const boost::filesystem::path& );
		static void freeAll();
		static void free( Animation* );

		static std::vector<Animation*> loadDir( const std::string& directory, const std::string& extension );
		static Animation* get( const std::string& fileName );

	private:
		static boost::filesystem::path animationDataPath;
		static std::map<std::string, Animation*> allAnimations;

		static Animation* loadFile( const boost::filesystem::path& );

};

#endif
