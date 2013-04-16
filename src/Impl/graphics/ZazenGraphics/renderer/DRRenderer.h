/*
 * DRRenderer.h
 *
 *  Created on: Jan 16, 2011
 *      Author: jonathan
 */

#ifndef DRRENDERER_H_
#define DRRENDERER_H_

#include "Renderer.h"

#include "../Framebuffer/FrameBufferObject.h"
#include "../Framebuffer/RenderTarget.h"

#include "../Program/UniformBlock.h"
#include "../Program/Program.h"
#include "../Program/Shader.h"

class DRRenderer : public Renderer
{
	public:
		DRRenderer();
		virtual ~DRRenderer();

		virtual bool initialize( const boost::filesystem::path& );
		virtual bool shutdown();

		// renders this list of geominstances which must be in front-to-back order
		bool renderFrame( std::list<Instance*>& instances, std::list<Light*>& lights );

	private:
		// Multiple-Render-Targes & Framebuffer for Deferred Rendering
		FrameBufferObject* m_gBufferFbo;
		////////////////////////////////////////

		// Program and shaders for geometry-stage
		Program* m_progGeomStage;
		Program* m_progSkyBox;
		////////////////////////////////////////

		// Program and shaders for lighting-stage with shadowing
		Program* m_progLightingStage;
		////////////////////////////////////////

		// Program and shaders for lighting-stage withOUT shadowing
		Program* m_progLightingNoShadowStage;
		////////////////////////////////////////

		// Program and shaders for shadow-mapping
		Program* m_progShadowMapping;

		FrameBufferObject* m_shadowMappingFB;
		////////////////////////////////////////

		// Uniform-Blocks
		UniformBlock* m_transformsBlock;
		UniformBlock* m_cameraBlock;
		UniformBlock* m_lightBlock;
		UniformBlock* m_materialBlock;
		////////////////////////////////////////

		// utils matrix
		glm::mat4 m_unitCubeMatrix;

		bool initGeomStage( const boost::filesystem::path& );
		bool initGBuffer();
		bool initLightingStage( const boost::filesystem::path& );
		bool initShadowMapping( const boost::filesystem::path& );
		bool initUniformBlocks();

		bool createMrtBuffer( RenderTarget::RenderTargetType );

		bool doGeometryStage( std::list<Instance*>&, std::list<Light*>& );
		bool doLightingStage( std::list<Instance*>&, std::list<Light*>& );

		bool renderSkyBox();
		bool renderLight( std::list<Instance*>&, Light* );
		bool renderShadowMap( std::list<Instance*>&, Light* );

		bool renderInstances( Viewer*, std::list<Instance*>&, Program*, bool );
		bool renderGeom( Viewer*, GeomType*, const glm::mat4& );

		bool peekOpenglErrors();

};

#endif /* DRRENDERER_H_ */
