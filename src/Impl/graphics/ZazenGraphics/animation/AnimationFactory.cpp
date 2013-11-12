#include "AnimationFactory.h"

#include "../ZazenGraphics.h"

#include "../Util/AssImpUtils.h"

#include <assimp/cimport.h>
#include <assimp/postprocess.h>

#include <glm/gtc/type_ptr.hpp>

#include <iostream>

using namespace std;
using namespace boost;

map<string, Animation*> AnimationFactory::m_animationPrototypes;
boost::filesystem::path AnimationFactory::m_animationDataPath;

void
AnimationFactory::setDataPath( const boost::filesystem::path& animationDataPath )
{
	AnimationFactory::m_animationDataPath = animationDataPath;
}

Animation*
AnimationFactory::get( const std::string& fileName )
{
	map<std::string, Animation*>::iterator findIter = AnimationFactory::m_animationPrototypes.find( fileName );
	if ( findIter != AnimationFactory::m_animationPrototypes.end() )
	{
		// create a clone of this found prototype because we hand out unique instances and not shared instances like meshes (animations are unique for each instance)
		return AnimationFactory::cloneAnimation( findIter->second );
	}

	filesystem::path fullFileName( AnimationFactory::m_animationDataPath.generic_string() + fileName );
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
	
	// loadFile returns already a clone of the prototype
	return AnimationFactory::loadFile( fullFileName );
}

vector<Animation*>
AnimationFactory::loadDir( const std::string& directory, const std::string& extension )
{
	vector<Animation*> rootAnimations;

	filesystem::path folderPath( AnimationFactory::m_animationDataPath.generic_string() + directory );
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
		
		// recursively go down into sub-directories
		if ( filesystem::is_directory( entry.path() ) )
		{
			vector<Animation*> subDirAnimations = AnimationFactory::loadDir( entry.path().generic_string().c_str(), extension );
			rootAnimations.insert( rootAnimations.begin(), subDirAnimations.begin(), subDirAnimations.end() ); 
		}
		else
		{
			// check if extension is the one we want
			string pathExtension = filesystem::extension( entry.path() );
			if ( pathExtension == extension )
			{
				// check if this file is already loaded
				map<std::string, Animation*>::iterator findIter = AnimationFactory::m_animationPrototypes.find( entry.path().generic_string().c_str() );
				if ( findIter != AnimationFactory::m_animationPrototypes.end() )
				{
					// create a clone of this found prototype because we hand out unique instances and not shared instances like meshes (animations are unique for each instance)
					rootAnimations.push_back( AnimationFactory::cloneAnimation( findIter->second ) );
				}
				// not yet loaded => load it fresh
				else
				{
					// load animation, will return a clone of the prototype
					Animation* animation = AnimationFactory::loadFile( entry.path().generic_string().c_str() );
					if ( NULL != animation )
					{
						rootAnimations.push_back( animation );
					}
				}
			}
		}
	}

	return rootAnimations;
}

void
AnimationFactory::freeAll()
{
	// this will clean-up only the prototypes, AnimationFactory hands out clones of those
	// the client is responsible for cleaning up the clones!
	map<std::string, Animation*>::iterator iter = AnimationFactory::m_animationPrototypes.begin();
	while ( iter != AnimationFactory::m_animationPrototypes.end() )
	{
		Animation* anim = iter->second;
		// the animation-channels are owned only by AnimationFactory and thus need to be deleted separately
		// clones will hold a pointer to the channels of their prototypes
		delete anim->m_animationChannels;
		delete anim;

		iter++;
	}

	AnimationFactory::m_animationPrototypes.clear();
}

Animation*
AnimationFactory::loadFile( const filesystem::path& filePath )
{
	ZazenGraphics::getInstance().getLogger().logInfo() << "LOADING ... " << filePath;

	const std::string& fileName = filePath.generic_string();

	const struct aiScene* scene = aiImportFile( fileName.c_str(), 0 );
	if ( NULL == scene )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "AnimationFactory::loadFile: AssetImporter failed loading the file with error: " << aiGetErrorString();
		return 0;
	}

	if ( ! scene->HasAnimations() )
	{
		ZazenGraphics::getInstance().getLogger().logError() << "AnimationFactory::loadFile: tried to load animation-file without animations - ignoring this file.";
		
		aiReleaseImport( scene );
		return 0;
	}

	Animation* animation = NULL;
	aiNode* rootNode = scene->mRootNode;

	// NOTE: for now only first animation is considered, should be enough for doom3
	aiAnimation* assImpAnim = scene->mAnimations[ 0 ];

	map<string, Animation::AnimationChannel>* animationChannels = new map<string, Animation::AnimationChannel>();
	animation = new Animation( assImpAnim->mDuration, assImpAnim->mTicksPerSecond, animationChannels );
	
	// extract data from all channels
	for ( unsigned int i = 0; i < assImpAnim->mNumChannels; i++ )
	{
		aiNodeAnim* assImpChannel = assImpAnim->mChannels[ i ];
		Animation::AnimationChannel animChannel;

		// extract all position-keys
		for ( unsigned int j = 0; j < assImpChannel->mNumPositionKeys; j++ )
		{
			aiVectorKey assImpPosKey = assImpChannel->mPositionKeys[ j ];

			Animation::AnimationKey<glm::vec3> posKey;
			posKey.m_time = assImpPosKey.mTime;
	
			AssImpUtils::assimpVecToGlm( assImpPosKey.mValue, posKey.m_value );

			animChannel.m_positionKeys.push_back( posKey );
		}

		// extract all rotation-keys
		for ( unsigned int j = 0; j < assImpChannel->mNumRotationKeys; j++ )
		{
			aiQuatKey assImpRotKey = assImpChannel->mRotationKeys[ j ];

			Animation::AnimationKey<glm::quat> rotKey;
			rotKey.m_time = assImpRotKey.mTime;
	
			// assimp provides the rotations by quaternions, get matrix from it
			AssImpUtils::assimpQuatToGlm( assImpRotKey.mValue, rotKey.m_value );

			animChannel.m_rotationKeys.push_back( rotKey );
		}

		// extract all scaling-keys
		for ( unsigned int j = 0; j < assImpChannel->mNumScalingKeys; j++ )
		{
			aiVectorKey assImpScaleKey = assImpChannel->mScalingKeys[ j ];

			Animation::AnimationKey<glm::vec3> scaleKey;
			scaleKey.m_time = assImpScaleKey.mTime;
	
			AssImpUtils::assimpVecToGlm( assImpScaleKey.mValue, scaleKey.m_value );

			animChannel.m_scalingKeys.push_back( scaleKey );
		}

		animationChannels->insert( std::make_pair( assImpChannel->mNodeName.C_Str(), animChannel ) );
	}
		
	// store this new prototype in our known prototypes
	AnimationFactory::m_animationPrototypes[ filePath.generic_string().c_str() ] = animation; 

	ZazenGraphics::getInstance().getLogger().logInfo() << "LOADED ... " << filePath;

	// return a clone of this new prototype
    return AnimationFactory::cloneAnimation( animation );
}

Animation*
AnimationFactory::cloneAnimation( Animation* prototype )
{
	return new Animation( prototype->m_durationTicks, prototype->m_ticksPerSecond, prototype->m_animationChannels );
}