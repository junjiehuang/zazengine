#ifndef _ZAZENGRAPHICS_H_
#define _ZAZENGRAPHICS_H_

// core API includes
#include <core/ICore.h>
#include <core/ILogger.h>

// subsystem-specific API
#include <graphics/IGraphics.h>

// local includes
#include "ZazenGraphicsEntity.h"

// local subdirectories includes
#include "Renderer/DRRenderer.h"

// libs/system includes
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

#endif /* _ZAZENGRAPHICS_H_ */
