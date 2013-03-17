/*
 * DRRenderer.h
 *
 *  Created on: Jan 16, 2011
 *      Author: jonathan
 */

#ifndef DRRENDERER_H_
#define DRRENDERER_H_

// number of generic color-attachment rendering targets for deferred renderer
// 1st = diffuse color
// 2nd = normals
// 3rd = genericParams 1
// 4th = genericParams 2
#define MRT_COUNT 4

#include "Renderer.h"

#include "../Material/UniformBlock.h"
#include "../Material/Program.h"
#include "../Material/Shader.h"


/* Deferred Rendering
 * Geometry-Stage:
 * Renders diffuse color ( texturing will be applied here ), depth and normals ( could also come from a normalmap )
 * into 3 rendering target. Those targets will be used by the LightingStage to produce the final result.
 * A DEPTH_ATTACHMENT is used as the depth-buffer and to get correct depth-visibility ( http://www.gamedev.net/topic/578084-depth-buffer-and-deferred-rendering/ )
 *
 * Lighting-Stage:
 * Uses the rendering targets to calculate the final color of each fragment. See Shadowing how shadowing is applied
 * Iterate over each light and apply it through the rendering of a quad with the size of the screen.
 * Additively blend the results of each light together.
 *
 * Shadowing
 * This renderer utilizes ShadowMaps as shadow-algorithm. The scene is rendererd from the viewpoint of
 * the light into a depth-texture ( managed by the light ). Shadowing itself happens in the lighting-stage.
 * The screen-space coordinate is transformed back into world-space (using the depth) and then transformed
 * into light-space. Furthermore the light-space coordinate is transformed into 0-1 to be able to access
 * the shadowmap like a texture ( using textureProj ).
 * Deferred shadowing:
 * first with the screen-coords x,y in the range 0-1 and the depth in the range 0-1 the frag-position
 * can be optained: transform x and y back in NDC ( after clipping and perspective division ), values
 * are in the range of -1 to +1. then the inverse PROJECTION matrix of the camera with which
 * the g-buffer was rendered is applied. then a division by w must happen to bring it back from projective
 * to the clipping space. finally the inverse viewin-matrix of the camera with which the g-buffer was
 * rendered is applied to bring the fragment into model-space. the model-space is the same for lighting
 * and thus we can start from this coordinate to apply the transformation into light-space.
 * -> we asume that each light has a different viewing and projection than the camera. the projection and thus
 *    the projection-matrix ( projection-space ) changes when setting a different FOV or viewport size or
 *    near and far or having an orthogonal projectione ( e.g. directional light ).
 *    the viewing-transformation is different too when the light should not stay with the camera. then
 *    the viewing-matrix ( view-space) is different from the one of the camera too (it could be the same
 *    but this is a special case and is handeled too with this approach ).
 *    the model-space stays the same because the model-space is local to each model and places it in the
 *    world which should be independent wheter it's being rendered from the point of view of a light or
 *    a camera.
 *    so clip-space (this is: model-view-projection space) is different between the camera and light.
 *    as previously was shown that view and projection differ but the model-space stays the same, the
 *    smallest denominator is the model-space. from there it must be possible to apply the projection-view
 *    of the camera to come into camera-space and to apply the projection-view of the light to come into
 *    the light-space.
 *    when doing deferred rendering, we don't have the information about individual objects in the light-pass,
 *    e.g. the model-matrix is no more available. so we cannot go back into object-space but rather in model-space.
 *    to go back into object-space one must apply the inverse model-matrix on the modeling-coordinates but
 *    that matrix is no more available - to be more precise, we operate in screen-space and so cannot know
 *    which pixel applies to which object ( it should be of no importance anymore ).
 *    what we can do is to get the model-space from screen-coordinates + depth as described above:
 *    1. tansform point( screenX, screenY, depth ) (all values 0-1) to NDC ( -1 to +1 )
 *    2. transform NDC to projection space: apply the inverse projection-matrix of the camera at the time of the g-buffer
 *    3. transform the projection-space to clip-space: divide by w
 *    4. transform the clip-space to model-space: apply inverse viewing-matrix of the camera at the time of the g-buffer
 *
 *    spaces (each stacks upon the previous):
 *    -> object-space: the object-local coordinate system - no matrix applied
 *    -> model-space: the object placed in the world - model-matix applied
 *    -> view-space: the object seen from e.g. the camera or a light - viewing-matrix applied
 *    -> clip-space: the object is transformed into the clipspace of the view-frustum - projection-matrix applied
 *    -> normalized-device-coordinates NDC: the coordinates of the clipspace are projected by dividing by w
 *    										coordinates between -1 and +1 in all axes, z is 0-1
 *    -> viewport-space: coordinates are transformed into x and y dimensions of the screen, z stays 0-1
 *
 * Lighting
 * because this is a deferred renderer, lighting occurs in screen-space. for this the normals must be
 * in screen-space and the lights position and direction too.
 * Very imporant is that light-direction and position must be transformed with the camera too because
 * each light is also part of the scene and must be placed in world-coordinates with its modeling-transformations.
 * During the lighting-pass each light is transformed into its world-coordinates by multiplying its
 * modeling-matrix with the viewing matrix of the camera. If one doesn't do this, the light sticks with the
 * camera (maybe this is desired...)
 *
 * something not right yet with light-space calculations.
 */

/* Errors:
 * - Shadowmap display: glTexParameteri(texture_target, GL_DEPTH_TEXTURE_MODE, GL_LUMINANCE);
 *
 * - Shadowing artefacts:
 *		-> employ Face-Culling: cull front-faces during shadowmap generation
 *
 * - Lighting and Normals:
 * 		-> check space of normals and camera: normals must be provided in screen-space and camera position
 * 			and direction must be transformed to screen-space too. LIGHTING MUST ALWAYS HAPPEN IN THE SAME
 * 		    SPACE.
 * 		-> something wrong with normals transformation ( maybe they're not correctly loaded in geometryfactory )
 *		-> lights must be transformed by camera-viewingMatrix to world-coordinates otherwise they stick with camera
 *		-> LIGHTING IS DONE IN WORLD-SPACE
 *
 * - Correct Deferred Shadowing:
 * 		-> something still wrong with the shadowing transformation in lighting Fragment-shader
 *
 * Questions:
 * - am i in the right space?
 * - do i mix up spaces (e.g. screen-space and view-space or clipping space (after applying mvp matrix ) with projection space ( after perspective div to NDC ) )
 * - is the data correct i receive?
 * - where does the data i receive come from?
 * - what is the data i receive?
 * - where does the data go to i transform?
 * - what happens with the data i transformed, send away and received somewhere else in between?
 */

/* TODO:
 * - Enhance Deferred Shadowing (reduce artifacts and implement soft-shadows )
 * - Introduce multiple lights: each light contributes ADDITIVELY to the framebuffe, solve this
 * - Reflections: would be nice to have planar reflections in this renderer too
 */
class DRRenderer : public Renderer
{
 public:
	DRRenderer();
	virtual ~DRRenderer();

	virtual bool initialize( const boost::filesystem::path& );
	virtual bool shutdown();

	virtual bool toggleDisplay();

	// renders this list of geominstances which must be in front-to-back order
	bool renderFrame( std::list<Instance*>& instances, std::list<Light*>& lights );

 private:
	// Multiple-Render-Targes & Framebuffer for Deferred Rendering
	GLuint m_fbo;
	GLuint m_geometryDepth;						// the id of the depth-buffer
	GLuint m_colorBuffers[ MRT_COUNT ];			// the generic color attachments for MRTs
	GLenum m_buffers[ MRT_COUNT ];
	////////////////////////////////////////

	// Program and shaders for geometry-stage
	Program* m_progGeomStage;
	Shader* m_vertGeomStage;
	Shader* m_fragGeomStage;
	////////////////////////////////////////

	// Program and shaders for lighting-stage
	Program* m_progLightingStage;
	Shader* m_vertLightingStage;
	Shader* m_fragLightingStage;
	////////////////////////////////////////

	// Program and shaders for shadow-mapping
	Program* m_progShadowMapping;
	Shader* m_vertShadowMapping;
	Shader* m_fragShadowMapping;

	GLuint m_shadowMappingFB;
	////////////////////////////////////////

	// Uniform-Blocks
	UniformBlock* m_transformsBlock;
	UniformBlock* m_lightBlock;
	UniformBlock* m_materialBlock;
	////////////////////////////////////////

	// utils matrix
	glm::mat4 m_unitCubeMatrix;

	bool m_displayMRT;

	bool initFBO();
	bool initGeomStage( const boost::filesystem::path& );
	bool initLightingStage( const boost::filesystem::path& );
	bool initShadowMapping( const boost::filesystem::path& );
	bool initUniformBlocks();

	bool initDepthBuffer();
	bool initMrtBuffer( unsigned int );

	bool renderShadowMap( std::list<Instance*>&, std::list<Light*>& );
	bool renderSkyBox();
	bool renderGeometryStage( std::list<Instance*>&, std::list<Light*>& );
	bool renderLightingStage( std::list<Instance*>&, std::list<Light*>& );

	bool renderInstances( Viewer*, std::list<Instance*>&, Program*, bool, bool );
	bool renderGeom( Viewer*, Instance*, GeomType* );

	bool showTexture( GLuint texID, int quarter );

};

#endif /* DRRENDERER_H_ */
