#ifndef _DRRENDERER_H_
#define _DRRENDERER_H_

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

		Mesh* m_fullScreenQuad;

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

		Program* m_progTransparency;
		Program* m_progBlendTransparency;

		// Program and shaders for shadow-mapping
		Program* m_progShadowMappingPlanar;
		Program* m_progShadowMappingCubeSinglePass;
		Program* m_progShadowMappingCubeMultiPass;
		////////////////////////////////////////

		// Uniform-Blocks
		UniformBlock* m_cameraBlock;
		UniformBlock* m_transformsBlock;
		UniformBlock* m_screenRenderingBoundaryBlock;
		UniformBlock* m_lightBlock;
		UniformBlock* m_materialBlock;
		UniformBlock* m_transparentMaterialBlock;
		////////////////////////////////////////

		// static data which will not change every frame
		// unit-cube matrix for directional- and spot-light shadow-mapping
		glm::mat4 m_unitCubeMatrix;
		// view-matrices for cube-map rendering 
		std::vector<glm::mat4> m_cubeViewDirections;
		// the indices of the g-buffer targets
		std::vector<unsigned int> m_gBufferDrawBufferIndices;
		////////////////////////////////////////

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

		void initializeStaticData();

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
		bool renderShadowPlanar( std::list<ZazenGraphicsEntity*>&, Light* );
		bool renderShadowCube( std::list<ZazenGraphicsEntity*>&, Light* );
		bool renderShadowCubeSinglePass( std::list<ZazenGraphicsEntity*>&, Light* );
		bool renderShadowCubeMultiPass( std::list<ZazenGraphicsEntity*>&, Light* );
		bool renderShadowPass( std::list<ZazenGraphicsEntity*>&, Light*, Program* );

		bool renderTransparentInstance( ZazenGraphicsEntity*, unsigned int, unsigned int, bool );
		bool renderTransparentEntity( Viewer*, ZazenGraphicsEntity*, Program* );
		
		bool renderEntities( Viewer*, std::list<ZazenGraphicsEntity*>&, Program*, bool, bool );
		bool renderMeshNode( MeshNode*, const glm::mat4&, const glm::mat4& );

		bool updateCameraBlock( Viewer* );
		bool updateLightBlock( Light* );
		bool updateScreenRenderingBlock( const glm::mat4& );

		static bool depthSortingFunc( ZazenGraphicsEntity* a, ZazenGraphicsEntity* b );

};

#endif /* _DRRENDERER_H_ */
