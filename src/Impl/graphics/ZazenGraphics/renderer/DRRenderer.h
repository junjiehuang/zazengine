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
		FrameBufferObject* m_fbo;
		FrameBufferObject* m_helperFbo;
		////////////////////////////////////////

		// full-screen-quad (FSQ) needed to perform final lighting- and post-processing passes 
		Mesh* m_fsq;
		////////////////////////////////////////

		// Programs for geometry-stage
		Program* m_progGeomStage;
		Program* m_progSkyBox;
		////////////////////////////////////////

		// Programs for lighting-stage with shadowing
		Program* m_progLightingStage;
		Program* m_progLightingStageStencilVolume;
		////////////////////////////////////////

		// Programs for shadow-mapping
		Program* m_progShadowMappingPlanar;
		Program* m_progShadowMappingCubeSinglePass;
		Program* m_progShadowMappingCubeMultiPass;
		////////////////////////////////////////

		// Programs for Post-Processing
		// Program for refractive-transparency rendering
		Program* m_progTranspRefract;
		// Program for classic-transparency rendering
		Program* m_progTranspClassic;
		// Program for cube-maped environmental rendering
		Program* m_progCubeEnv;
		// Program for planar environmental rendering
		Program* m_progPlanarEnv;
		////////////////////////////////////////

		// Uniform-Blocks
		UniformBlock* m_cameraBlock;
		UniformBlock* m_transformsBlock;
		UniformBlock* m_lightBlock;
		UniformBlock* m_gStageMaterialBlock;

		UniformBlock* m_materialTranspClassicBlock;
		UniformBlock* m_materialTranspRefractBlock;
		UniformBlock* m_materialEnvCubeBlock;
		UniformBlock* m_materialEnvPlanarBlock;
		////////////////////////////////////////

		// the camera for the current frame 
		Viewer* m_currentCamera;
		////////////////////////////////////////

		// helper rendering-targets
		RenderTarget* m_envPlanarTarget;
		RenderTarget* m_envCubeTarget;
		////////////////////////////////////////

		// all entities for the current frame
		std::list<ZazenGraphicsEntity*>* m_currentEntities;
		////////////////////////////////////////

		// static data which will not change every frame
		// unit-cube matrix for directional- and spot-light shadow-mapping
		glm::mat4 m_unitCubeMatrix;
		// view-matrices for cube-map rendering 
		std::vector<glm::mat4> m_cubeViewDirections;
		// model-matrices for cube-maped enviornment rendering
		std::vector<glm::mat4> m_viewerModelMatrices;
		// the indices of the g-buffer targets for drawing to during geometry-stage
		std::vector<unsigned int> m_gBufferDrawBufferIndices;
		// the indices of the g-buffer targets to be bound during lighting-stage
		std::vector<unsigned int> m_gBufferBindTargetIndices;
		////////////////////////////////////////

		bool initFBOs();
		bool initGBuffer();
		bool initIntermediateDepthBuffer();

		bool initGeomStage();
		bool initLightingStage();
		bool initShadowMapping();
		bool initPostProcessing();
		bool initUniformBlocks();

		void initializeStaticData();

		bool createMrtBuffer( RenderTarget::RenderTargetType, FrameBufferObject* );

		bool renderFrameInternal( Viewer* );
		bool doGeometryStage();
		bool doLightingStage();
		bool doScreenSpaceStage();

		bool doPostProcessing();

		bool renderSkyBox();
		bool renderGeometry( std::list<ZazenGraphicsEntity*>&, Program* );

		bool processLight( Light*, unsigned int );
		bool markLightVolume( Light*, unsigned int );
		bool renderLight( Light*, unsigned int );

		bool renderShadowMap( Light* );
		bool renderShadowPlanar( Light* );
		bool renderShadowCube( Light* );
		bool renderShadowCubeSinglePass( Light* );
		bool renderShadowCubeMultiPass( Light* );
		bool renderShadowPass( Light*, Program* );

		void filterPostProcessEntities( std::vector<ZazenGraphicsEntity*>& );

		bool renderTransparentClassicInstance( ZazenGraphicsEntity*, unsigned int );
		bool renderTransparentRefractiveInstance( ZazenGraphicsEntity*, unsigned int, unsigned int, bool );
		bool renderEnvironmentCubeInstance( ZazenGraphicsEntity* );
		bool renderEnvironmentPlanarInstance( ZazenGraphicsEntity* );
		bool renderEnvironmentInstance( ZazenGraphicsEntity* );

		bool renderPostProcessEntity( Viewer*, ZazenGraphicsEntity*, Program* );
		
		bool renderEntities( Viewer*, std::list<ZazenGraphicsEntity*>&, Program*, bool, bool );
		bool renderMeshNode( MeshNode*, const glm::mat4&, const glm::mat4&, const glm::mat4& );

		bool updateCameraBlock( Viewer* );
		bool updateLightBlock( Light*, Viewer* );

		static bool depthSortingFunc( ZazenGraphicsEntity* a, ZazenGraphicsEntity* b );

};

#endif /* _DRRENDERER_H_ */
