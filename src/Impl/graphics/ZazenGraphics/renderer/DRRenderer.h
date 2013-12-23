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

		// Programs for transparency rendering
		Program* m_progTransparency;
		////////////////////////////////////////

		// Programs for environmental rendering
		Program* m_progCubeEnv;
		////////////////////////////////////////

		// Uniform-Blocks
		UniformBlock* m_cameraBlock;
		UniformBlock* m_transformsBlock;
		UniformBlock* m_lightBlock;
		UniformBlock* m_materialBlock;
		UniformBlock* m_transparentMaterialBlock;
		////////////////////////////////////////

		// the camera for the current frame 
		Viewer* m_currentCamera;
		////////////////////////////////////////

		// helper rendering-targets
		RenderTarget* m_planarHelperTarget;
		RenderTarget* m_environmentHelperTarget;
		////////////////////////////////////////

		// all entities for the current frame
		std::list<ZazenGraphicsEntity*>* m_currentEntities;
		////////////////////////////////////////

		// static data which will not change every frame
		// unit-cube matrix for directional- and spot-light shadow-mapping
		glm::mat4 m_unitCubeMatrix;
		// view-matrices for cube-map rendering 
		std::vector<glm::mat4> m_cubeViewDirections;
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
		bool initTransparency();
		bool initUniformBlocks();

		void initializeStaticData();

		bool createMrtBuffer( RenderTarget::RenderTargetType, FrameBufferObject* );

		bool renderInternalFrame( Viewer* );

		void preProcessTransparency();
		bool doGeometryStage();
		bool doLightingStage();
		bool doPostProcessing();
		bool doTransparencyStage();

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

		void filterTransparentEntities( std::vector<ZazenGraphicsEntity*>& );
		bool processTransparentEntities( std::vector<ZazenGraphicsEntity*>&, unsigned int& );
		bool renderTransparentInstance( ZazenGraphicsEntity*, unsigned int, unsigned int, bool );
		bool renderEnvironmentalInstance( ZazenGraphicsEntity* );
		bool renderTransparentEntity( Viewer*, ZazenGraphicsEntity*, Program* );
		
		bool renderEntities( Viewer*, std::list<ZazenGraphicsEntity*>&, Program*, bool, bool );
		bool renderMeshNode( MeshNode*, const glm::mat4&, const glm::mat4&, const glm::mat4& );

		bool updateCameraBlock( Viewer* );
		bool updateLightBlock( Light*, Viewer* );

		static bool depthSortingFunc( ZazenGraphicsEntity* a, ZazenGraphicsEntity* b );

};

#endif /* _DRRENDERER_H_ */
