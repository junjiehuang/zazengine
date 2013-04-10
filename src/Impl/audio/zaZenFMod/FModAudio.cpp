/*
 * FModAudio.cpp
 *
 *  Created on: 06.07.2010
 *      Author: Jonathan Thaler
 */

#include "FModAudio.h"

#include <core/ICore.h>

#include <fmodex/fmod_errors.h>

#include <iostream>
#include <sstream>

#include <stdio.h>

using namespace std;
using namespace boost;

FModAudio* FModAudio::instance = NULL;

FModAudio::FModAudio( const std::string& id, ICore* core )
	: id ( id ),
	type ( "audio" ),
	m_core( core )
{
	this->m_system = 0;
	this->m_bgMusic = 0;
	this->m_bgMusicCh = 0;

	FModAudio::instance = this;
}

FModAudio::~FModAudio()
{
	FModAudio::instance = NULL;
}

bool
FModAudio::initialize( TiXmlElement* configElem )
{
	this->m_core->logInfo( "=============== FModAudio initializing... ===============" );

	if ( false == this->initAudioDataPath( configElem ) )
	{
		return false;
	}

	if ( false == this->initFMod( configElem ) )
	{
		return false;
	}

	if ( false == this->init3dSettings( configElem ) )
	{
		return false;
	}

	if ( false == this->loadBackgroundMusic( configElem ) )
	{
		return false;
	}

	this->m_core->logInfo( "================ FModAudio initialized =================" );

	return true;
}

bool
FModAudio::shutdown()
{
	this->m_core->logInfo( "=============== FModAudio shutting down... ===============" );

	FMOD_RESULT result;

	std::list<FModAudioEntity*>::iterator iter = this->entities.begin();
	while ( iter != this->entities.end() )
	{
		FModAudioEntity* entity = *iter++;
		delete entity;
	}

	this->entities.clear();

	if ( this->m_bgMusic )
	{
		result = this->m_bgMusic->release();
		if ( FMOD_OK != result )
		{
			this->m_core->logError( stringstream( "FModAudio::shutdown: bgMusic release failed: " ) << FMOD_ErrorString( result ) );
		}
	}

	if ( this->m_system )
	{
		result = this->m_system->close();
		if ( FMOD_OK != result )
		{
			this->m_core->logError( stringstream( "FModAudio::shutdown: system close failed: " ) << FMOD_ErrorString( result ) );
		}

		result = this->m_system->release();
		if ( FMOD_OK != result )
		{
			this->m_core->logError( stringstream( "FModAudio::shutdown: system release failed: " ) << FMOD_ErrorString( result ) );
		}
	}

	this->m_core->logInfo( "================ FModAudio shutdown =================" );

	return true;
}

bool
FModAudio::start()
{
	FMOD_RESULT result;

	if ( this->m_bgMusic )
	{
		result = this->m_system->playSound( FMOD_CHANNEL_FREE, this->m_bgMusic, false, &this->m_bgMusicCh );
		if ( FMOD_OK != result )
		{
			this->m_core->logError( stringstream( "FModAudio::start: system playSound failed: " ) << FMOD_ErrorString( result ) );
			return false;
		}
	}

	std::list<FModAudioEntity*>::iterator iter = this->entities.begin();
	while ( iter != this->entities.end() )
	{
		FModAudioEntity* entity = *iter++;
		if ( entity->isImmediatePlayback() )
		{
			if ( false == entity->playSound() )
			{
				return false;
			}
		}
	}

	return true;
}

bool
FModAudio::stop()
{
	return true;
}

bool
FModAudio::pause()
{
	return true;
}

bool
FModAudio::process( double factor )
{
	FMOD_RESULT result = this->m_system->update();
	if ( FMOD_OK != result )
	{
		this->m_core->logError( stringstream( "FModAudio::process: system update failed: " ) << FMOD_ErrorString( result ) );
		return false;
	}

	return true;
}

bool
FModAudio::finalizeProcess()
{
	return true;
}

bool
FModAudio::sendEvent( Event& e )
{
	return true;
}

FModAudioEntity*
FModAudio::createEntity( TiXmlElement* objectNode, IGameObject* parent )
{
	FModAudioEntity* entity = 0;

	for ( TiXmlElement* soundNode = objectNode->FirstChildElement(); soundNode != 0; soundNode = soundNode->NextSiblingElement() )
	{
		const char* str = soundNode->Value();
		if ( 0 == str )
		{
			continue;
		}

		if ( 0 == strcmp( str, "listener" ) )
		{
			float posX = 0.0f;
			float posY = 0.0f;
			float posZ = 0.0f;

			str = soundNode->Attribute( "x" );
			if ( 0 != str )
			{
				posX = ( float ) atof( str );
			}

			str = soundNode->Attribute( "y" );
			if ( 0 != str )
			{
				posY = ( float ) atof( str );
			}

			str = soundNode->Attribute( "z" );
			if ( 0 != str )
			{
				posZ = ( float ) atof( str );
			}

			entity = new FModAudioEntity( parent );
		}
		else if ( 0 == strcmp( str, "sound" ) )
		{
			bool loop = false;
			bool immediatePlayback = true;
			float posX = 0.0f;
			float posY = 0.0f;
			float posZ = 0.0f;
			float minDist = 1.0f;
			float maxDist = 1.0f;
			std::string fileName;
			
			str = soundNode->Attribute( "file" );
			if ( 0 == str )
			{
				continue;
			}

			fileName = this->m_audioDataPath.generic_string() + str;

			str = soundNode->Attribute( "x" );
			if ( 0 != str )
			{
				posX = ( float ) atof( str );
			}

			str = soundNode->Attribute( "y" );
			if ( 0 != str )
			{
				posY = ( float ) atof( str );
			}

			str = soundNode->Attribute( "z" );
			if ( 0 != str )
			{
				posZ = ( float ) atof( str );
			}

			str = soundNode->Attribute( "minDist" );
			if ( 0 != str )
			{
				minDist = ( float ) atof( str );
			}

			str = soundNode->Attribute( "maxDist" );
			if ( 0 != str )
			{
				maxDist = ( float ) atof( str );
			}

			str = soundNode->Attribute( "loop" );
			if ( 0 != str )
			{
				string caseStr = str;
				std::transform( caseStr.begin(), caseStr.end(), caseStr.begin(), tolower );
				if ( caseStr == "true" )
				{
					loop = true;
				}
			}

			str = soundNode->Attribute( "immediatePlayback" );
			if ( 0 != str )
			{
				string caseStr = str;
				std::transform( caseStr.begin(), caseStr.end(), caseStr.begin(), tolower );
				if ( caseStr == "false" )
				{
					immediatePlayback = false;
				}
			}

			FMOD::Sound* sound = 0;
			FMOD_RESULT result = this->m_system->createSound( fileName.c_str(), FMOD_SOFTWARE | FMOD_3D, 0, &sound );
		    if ( FMOD_OK != result )
		    {
				this->m_core->logError( stringstream( "loading sound from file \"" ) << fileName << ": " << FMOD_ErrorString( result ) );
		    	return 0;
		    }

			result = sound->set3DMinMaxDistance( minDist, maxDist );
			if ( FMOD_OK != result )
			{
				this->m_core->logError( stringstream( "setting min-max distance for sound \""  ) << fileName << ": " << FMOD_ErrorString( result ) );
		    	return 0;
			}

			if ( loop )
			{
				result = sound->setMode( FMOD_LOOP_NORMAL );
				if ( FMOD_OK != result )
				{
					this->m_core->logError( stringstream( "setting loop for sound \"" ) << fileName << ": " << FMOD_ErrorString( result ) );
		    		return 0;
				}
			}

			entity = new FModAudioEntity( parent );
			entity->setSound( sound );
			entity->setPosition( posX, posY, posZ );
			entity->setImmediatePlayback( immediatePlayback );
		}
	}

	if ( NULL != entity )
	{
		this->entities.push_back( entity );
	}

	return entity;
}


bool
FModAudio::initFMod( TiXmlElement* configElem )
{
	FMOD_RESULT result;
	unsigned int version;

	result = FMOD::System_Create( &this->m_system );
    if ( FMOD_OK != result )
	{
		this->m_core->logError( stringstream( "FModAudio::initFMod: System_Create failed: " ) << FMOD_ErrorString( result ) );
		return false;
	}

    result = this->m_system->getVersion( &version );
    if ( FMOD_OK != result )
	{
		this->m_core->logError( stringstream( "FModAudio::initFMod: getVersion failed: " ) << FMOD_ErrorString( result ) );
		return false;
	}

    if ( FMOD_VERSION > version )
    {
		this->m_core->logError( stringstream( "FModAudio::initFMod: You are using an old version of FMOD " ) << version << ". This program requires" << FMOD_VERSION );
		return false;
    }

	int numDrivers = 0;
	FMOD_SPEAKERMODE speakermode;
    FMOD_CAPS caps;

	result = this->m_system->getNumDrivers( &numDrivers);
    if ( FMOD_OK != result )
	{
		this->m_core->logError( stringstream( "FModAudio::initFMod: getNumDrivers failed: " ) << FMOD_ErrorString( result ) );
		return false;
	}

    if ( 0 == numDrivers )
    {
        result = this->m_system->setOutput( FMOD_OUTPUTTYPE_NOSOUND );
        if ( FMOD_OK != result )
		{
			this->m_core->logError( stringstream( "FModAudio::initFMod: setOutput failed: " ) << FMOD_ErrorString( result ) );
			return false;
		}
    }
    else
    {
        result = this->m_system->getDriverCaps( 0, &caps, 0, &speakermode );
        if ( FMOD_OK != result )
		{
			this->m_core->logError( stringstream( "FModAudio::initFMod: getDriverCaps failed: " ) << FMOD_ErrorString( result ) );
			return false;
		}

        result = this->m_system->setSpeakerMode( speakermode ); 
        if ( FMOD_OK != result )
		{
			this->m_core->logError( stringstream( "" ) << FMOD_ErrorString( result ) );
			cout << "ERROR ... in FModAudio::initFMod: setSpeakerMode failed: " << FMOD_ErrorString( result ) << endl;
			return false;
		}

        if ( caps & FMOD_CAPS_HARDWARE_EMULATED )
        { 
            result = this->m_system->setDSPBufferSize( 1024, 10 );
            if ( FMOD_OK != result )
			{
				this->m_core->logError( stringstream( "" ) << FMOD_ErrorString( result ) );
				cout << "ERROR ... in FModAudio::initFMod: setDSPBufferSize failed: " << FMOD_ErrorString( result ) << endl;
				return false;
			}
        }
    }

    result = this->m_system->init( 100, FMOD_INIT_NORMAL, 0 );
    if ( FMOD_OK != result )
	{
		this->m_core->logError( stringstream( "" ) << FMOD_ErrorString( result ) );
		cout << "ERROR ... in FModAudio::initFMod: system-init failed: " << FMOD_ErrorString( result ) << endl;
		return false;
	}

	return true;
}

bool
FModAudio::loadBackgroundMusic( TiXmlElement* configElem )
{
	TiXmlElement* musicNode = configElem->FirstChildElement( "backgroundMusic" );
	if ( 0 != musicNode )
	{
		const char* str = musicNode->Attribute( "file" );
		if ( 0 != str )
		{
			FMOD_RESULT result = m_system->createSound( str, FMOD_SOFTWARE | FMOD_2D, 0, &this->m_bgMusic );
			if ( FMOD_OK != result )
			{
				this->m_core->logError( stringstream( "" ) << FMOD_ErrorString( result ) );
				cout << "ERROR ... in FModAudio::loadBackgroundMusic: createSound failed: " << FMOD_ErrorString( result ) << endl;
				return false;
			}
		}
	}

	return true;
}

bool
FModAudio::initAudioDataPath( TiXmlElement* configElem )
{
	TiXmlElement* audioDataNode = configElem->FirstChildElement( "audioData" );
	if ( 0 == audioDataNode )
	{
		this->m_core->logError( stringstream( "missing audioData-config in audio-config" ) );
		return false;
	}

	const char* str = audioDataNode->Attribute( "path" );
	if ( 0 == str )
	{
		this->m_core->logError( stringstream( "missing audioData-path in audio-config" ) );
		return false;
	}

	this->m_audioDataPath = filesystem::path( str );
	if ( ! filesystem::exists( this->m_audioDataPath ) )
	{
		this->m_core->logError( stringstream( "audioData-path " ) << this->m_audioDataPath << " does not exist" );
		return false;
	}

	if ( false == filesystem::is_directory( this->m_audioDataPath ) )
	{
		this->m_core->logError( stringstream( "audioData-path " ) << this->m_audioDataPath << " is not a directory" );
		return false;
	}

	return true;
}

bool
FModAudio::init3dSettings( TiXmlElement* configElem )
{
	TiXmlElement* settings3dNode = configElem->FirstChildElement( "settings3d" );
	if ( 0 != settings3dNode )
	{
		float dopplerScale = 1.0f;
		float distanceFactor = 1.0f;
		float rollOffScale = 1.0f;

		const char* str = settings3dNode->Attribute( "dopplerScale" );
		if ( 0 != str )
		{
			dopplerScale = ( float ) atof( str );
		}

		str = settings3dNode->Attribute( "distanceFactor" );
		if ( 0 != str )
		{
			distanceFactor = ( float ) atof( str );
		}

		str = settings3dNode->Attribute( "rollOffScale" );
		if ( 0 != str )
		{
			rollOffScale = ( float ) atof( str );
		}

		FMOD_RESULT result = this->m_system->set3DSettings( dopplerScale, distanceFactor, rollOffScale );
		if ( FMOD_OK != result )
		{
			this->m_core->logError( stringstream( "" ) << FMOD_ErrorString( result ) );
			cout << "ERROR ... in FModAudio::init3dSettings: set3DSettings failed: " << FMOD_ErrorString( result ) << endl;
			return false;
		}
	}

	return true;
}

extern "C"
{	
	__declspec( dllexport ) ISubSystem*
	createInstance ( const char* id, ICore* core )
	{
		return new FModAudio( id, core );
	}

	__declspec( dllexport ) void
	deleteInstance ( ISubSystem* subSys )
	{
		if ( 0 == subSys )
		{
			return;
		}

		if ( 0 == dynamic_cast<FModAudio*>( subSys ) )
		{
			return;
		}

		delete subSys;
	}
}
