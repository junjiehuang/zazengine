/*
 * ZazenGraphics.h
 *
 *  Created on: 06.07.2010
 *      Author: Jonathan Thaler
 */

#ifndef ZAZENGRAPHICS_H_
#define ZAZENGRAPHICS_H_

#include <core/ICore.h>
#include <graphics/IGraphics.h>
#include <core/ILogger.h>

#include "ZazenGraphicsEntity.h"

#include "Viewer/Viewer.h"

#include "Lighting/Light.h"

#include "Renderer/DRRenderer.h"

#include <boost/filesystem.hpp>

class ZazenGraphics : public IGraphics
{
	public:
		static ZazenGraphics& getInstance() { return *ZazenGraphics::instance; };
		
		ZazenGraphics( const std::string& id, ICore* core );
		virtual ~ZazenGraphics();

		const std::string& getID() const { return this->m_id; };
		const std::string& getType() const { return this->m_type; };

		bool isAsync() const { return false; };

		bool initialize( TiXmlElement* );
		bool shutdown();

		bool start();
		bool stop();
		bool pause();

		bool process( double );
		bool finalizeProcess();

		bool sendEvent( Event& e );

		ZazenGraphicsEntity* createEntity( TiXmlElement*, IGameObject* parent );

		ICore& getCore() { return *this->m_core; };
		ILogger& getLogger() const { return *this->m_logger; };

		void* getWindowHandle();
		bool toggleFullscreen();

	private:
		static ZazenGraphics* instance;

		std::string m_id;
		std::string m_type;

		boost::filesystem::path m_pipelinePath;
		boost::filesystem::path m_modelDataPath;
		boost::filesystem::path m_animationDataPath;
		boost::filesystem::path m_textureDataPath;
		boost::filesystem::path m_materialDataPath;
		boost::filesystem::path m_screenShotPath;

		ICore* m_core;
		ILogger* m_logger;

		DRRenderer* m_renderer;

		std::list<ZazenGraphicsEntity*> m_entities;

		bool initPipelinePath( TiXmlElement* );
		bool initModelDataPath( TiXmlElement* );
		bool initAnimationDataPath( TiXmlElement* );
		bool initTextureDataPath( TiXmlElement* );
		bool initMaterialDataPath( TiXmlElement* );
		bool initSkyBoxFolderPath( TiXmlElement* );
		void initScreenShotPath( TiXmlElement* );

		bool createWindow( TiXmlElement* );
};

#endif /* ZAZENGRAPHICS_H_ */
