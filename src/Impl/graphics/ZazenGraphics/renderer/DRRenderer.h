/*
 * DRRenderer.h
 *
 *  Created on: Jan 16, 2011
 *      Author: jonathan
 */

#ifndef DRRENDERER_H_
#define DRRENDERER_H_

#include "../Framebuffer/FrameBufferObject.h"
#include "../Framebuffer/RenderTarget.h"

#include "../Program/UniformBlock.h"
#include "../Program/Program.h"
#include "../Program/Shader.h"

#include "../ZazenGraphicsEntity.h"

#include <list>

class DRRenderer
{
	public:
		DRRenderer();
		~DRRenderer();

		bool initialize();
		bool shutdown();

		bool renderFrame( std::list<ZazenGraphicsEntity*>& instances );

	private:
		// Multiple-Render-Targes & Framebuffer for Deferred Rendering
		FrameBufferObject* m_gBufferFbo;
		FrameBufferObject* m_intermediateDepthFB;

		RenderTarget* m_depthCopy;
		////////////////////////////////////////

		MeshStatic* m_fullScreenQuad;

		// Program and shaders for geometry-stage
		Program* m_progGeomStaticStage;
		Program* m_progGeomAnimStage;
		Program* m_progSkyBox;
		////////////////////////////////////////

		// Program and shaders for lighting-stage with shadowing
		Program* m_progLightingStage;
		////////////////////////////////////////

		// Program and shaders for lighting-stage withOUT shadowing
		Program* m_progLightingNoShadowStage;
		////////////////////////////////////////

		Program* m_progTransparency;
		Program* m_progBlendTransparency;

		// Program and shaders for shadow-mapping
		Program* m_progShadowMapping;
		////////////////////////////////////////

		// Uniform-Blocks
		UniformBlock* m_transformsBlock;
		UniformBlock* m_cameraBlock;
		UniformBlock* m_lightBlock;
		UniformBlock* m_materialBlock;
		UniformBlock* m_transparentMaterialBlock;
		////////////////////////////////////////

		// utils matrix
		glm::mat4 m_unitCubeMatrix;

		Viewer* m_mainCamera;
		std::vector<ZazenGraphicsEntity*> m_transparentEntities;
		std::vector<ZazenGraphicsEntity*> m_lightEntities;

		bool initFBOs();
		bool initGBuffer();
		bool initIntermediateDepthBuffer();

		bool initGeomStage();
		bool initLightingStage();
		bool initShadowMapping();
		bool initTransparency();
		bool initUniformBlocks();

		bool createMrtBuffer( RenderTarget::RenderTargetType, FrameBufferObject* );

		bool separateEntities( std::list<ZazenGraphicsEntity*>& );
		void preProcessTransparency( std::list<ZazenGraphicsEntity*>& );
		bool doGeometryStage( std::list<ZazenGraphicsEntity*>& );
		bool doLightingStage( std::list<ZazenGraphicsEntity*>& );
		bool doTransparencyStage( std::list<ZazenGraphicsEntity*>& );

		bool renderSkyBox();
		bool renderGeometry( std::list<ZazenGraphicsEntity*>&, Program* );

		bool renderLight( std::list<ZazenGraphicsEntity*>&, Light* );
		bool renderShadowMap( std::list<ZazenGraphicsEntity*>&, Light* );

		bool renderTransparentInstance( ZazenGraphicsEntity*, unsigned int, unsigned int, bool );

		bool renderEntities( Viewer*, std::list<ZazenGraphicsEntity*>&, Program*, bool, bool );
		bool renderEntity( Viewer*, ZazenGraphicsEntity*, Program* );
		bool renderMesh( Viewer*, Mesh*, const glm::mat4& );

		static bool depthSortingFunc( ZazenGraphicsEntity* a, ZazenGraphicsEntity* b );

};

#endif /* DRRENDERER_H_ */
