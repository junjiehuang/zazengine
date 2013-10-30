#include "AnimationFactory.h"

#include "../ZazenGraphics.h"

#include <assimp/cimport.h>
#include <assimp/postprocess.h>

#include <glm/gtc/type_ptr.hpp>

#include <iostream>

using namespace std;
using namespace boost;

map<string, Animation*> AnimationFactory::allAnimations;
boost::filesystem::path AnimationFactory::animationDataPath;

void
AnimationFactory::setDataPath( const boost::filesystem::path& animationDataPath )
{
	AnimationFactory::animationDataPath = animationDataPath;
}

Animation*
AnimationFactory::get( const std::string& fileName )
{
	map<std::string, Animation*>::iterator findIter = AnimationFactory::allAnimations.find( filePath.generic_string().c_str() );
	if ( findIter != AnimationFactory::allAnimations.end() )
	{
		return findIter->second;
	}

	filesystem::path fullFileName( AnimationFactory::animationDataPath.generic_string() + fileName );
	if ( ! filesystem::exists( fullFileName ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "AnimationFactory::get: file \"" << fullFileName << "\" does not exist";
		return 0;	
	}

	if ( filesystem::is_directory( fullFileName ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "AnimationFactory::get: file \"" << fullFileName << "\" is a directory";
		return 0;
	}
	
	Animation* animation = AnimationFactory::loadFile( fullFileName );

	if ( NULL != animation )
	{
		AnimationFactory::allAnimations[ fileName ] = animation;
	}

	return animation;
}

vector<Animation*>
AnimationFactory::loadDir( const std::string& directory, const std::string& extension )
{
	vector<Animation*> rootAnimations;

	filesystem::path folderPath( AnimationFactory::animationDataPath.generic_string() + directory );
	if ( ! filesystem::exists( folderPath ) )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "AnimationFactory::loadDir: directory \"" << directory << "\" does not exist";
		return rootAnimations;	
	}

	filesystem::directory_iterator iter( folderPath );
	filesystem::directory_iterator endIter;

	// iterate through directory
	while ( iter != endIter )
	{
		filesystem::directory_entry entry = *iter++;
		
		if ( filesystem::is_directory( entry.path() ) )
		{
			vector<Animation*> subDirAnimations = AnimationFactory::loadDir( entry.path().generic_string().c_str(), extension );
			rootAnimations.insert( rootAnimations.begin(), subDirAnimations.begin(), subDirAnimations.end() ); 
		}
		else
		{
			string pathExtension = filesystem::extension( entry.path() );
			if ( pathExtension == extension )
			{
				Animation* animation = AnimationFactory::loadFile( entry.path().generic_string().c_str() );
				if ( 0 != animation )
				{
					rootAnimations.push_back( animation );
				}
			}
		}
	}

	return rootAnimations;
}

void
AnimationFactory::freeAll()
{
	map<std::string, Animation*>::iterator iter = AnimationFactory::allAnimations.begin();
	while ( iter != AnimationFactory::allAnimations.end() )
	{
		Animation* anim = iter->second;
		delete anim;

		iter++;
	}

	AnimationFactory::allAnimations.clear();
}

Animation*
AnimationFactory::loadFile( const filesystem::path& filePath )
{
	ZazenGraphics::getInstance().getLogger().logInfo() << "LOADING ... " << filePath;

	map<std::string, Animation*>::iterator findIter = AnimationFactory::allAnimations.find( filePath.generic_string().c_str() );
	if ( findIter != AnimationFactory::allAnimations.end() )
	{
		return findIter->second;
	}

	const std::string& fileName = filePath.generic_string();

	const struct aiScene* scene = aiImportFile( fileName.c_str(), 0 );
	if ( NULL == scene )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "AnimationFactory::loadFile: AssetImporter failed loading the file with error: " << aiGetErrorString();
		return 0;
	}

	Animation* animation = 0;
	
	if ( scene->HasAnimations() )
	{
		animation = new Animation();

		// TODO: how-to handle multiple animations in one file??
		for ( unsigned int i = 0; i < scene->mNumAnimations; i++ )
		{
			aiAnimation* anim = scene->mAnimations[ i ];
		}
		
		AnimationFactory::allAnimations[ filePath.generic_string().c_str() ] = animation; 

		ZazenGraphics::getInstance().getLogger().logInfo() << "LOADED ... " << filePath;
	}
	else 
	{
		ZazenGraphics::getInstance().getLogger().logError() << "AnimationFactory::loadFile: tried to load animation-file without animations - ignoring this file.";
	}

	aiReleaseImport( scene );

    return animation;
}
