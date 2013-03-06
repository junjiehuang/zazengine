/*
 * ZazenGraphics.h
 *
 *  Created on: 06.07.2010
 *      Author: joni
 */

#ifndef ZAZENGRAPHICS_H_
#define ZAZENGRAPHICS_H_

#include <core/ICore.h>
#include <graphics/IGraphics.h>

#include <SDL/SDL.h>

#include "ZazenGraphicsEntity.h"

#include "Scene/Viewer.h"
#include "Scene/Instance.h"

#include "Lighting/Light.h"

#include "Renderer/Renderer.h"

class ZazenGraphics : public IGraphics
{
	public:
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

	private:
		std::string m_id;
		std::string m_type;

		ICore* m_core;

		double m_lastItFact;

		SDL_Surface* m_drawContext;

		Viewer* m_camera;
		Renderer* m_renderer;

		std::list<Light*> m_lights;
		std::list<Instance*> m_instances;

		std::list<ZazenGraphicsEntity*> m_entities;

		bool initSDL();
		bool initGL();
};

#endif /* ZAZENGRAPHICS_H_ */
