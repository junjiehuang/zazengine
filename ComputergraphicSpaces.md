CARSTEN WENZEL CRYTEC CRYSIS worldspace

objectspace; the coordinates of the vertices when loaded from geom file (independent of viewer)
modelspace / worldspace: coordinates of the vertices when placed in the scene (independent of viewer)
eyespace / viewspace: coordinates of the vertices when viewed through camera (dependent on viewer)
clipspace: coordinates of the vertices when projection matrix applied
projectionspace / NDC: coordinates of the vertices when projective division is applied (-1 to +1)
screenspace: coordinates of the vertex/fragment when viewport transformation is applied (0 to resolutionX and 0 to resolutionY)

vertex transformations:
objectspace -> MODEL -> worldspace -> VIEW -> viewspace -> PROJECTION -> clipspace -> PERSP. DIVIDE -> projection space NDC -> VIEWPORT TRANSF. -> screenspace

normal transformations:
objectspace -> TRANSPOSE(INVERSE(MODEL)) -> worldspace -> TRANSPOSE(INVERSE(VIEW)) -> viewspace. will then be perspective interpolated between vertices

lighting
normals are stored in view-space and are available in the lighting-stage in view-space. lighting must be applied in the same spaces => light position and direction must be available transformed to screen space.

shadowmap rendering
the objects are transformed from world-coordinates (model-matrix applied, which are independent of camera or light) to the lights view-space because the scene is rendered from the point of view of the light.

shadowing
the shadowmap is available in the light-space ( which includes projection, persp. division and so on because we render the scene in the standard way to a texture ). the fragments x and y screen-space coordinates and the fragments depth are available.
first the fragments world-space coordinates must be calculated. then the world-space coordinates are transformed to the light-space and finally multiplied with a matrix which transforms the values from NDC to 0-1 to access the shadow map.
so then it should be possible to calculate the world-space coordinate for the shadow-coord in the geometry-stage vertex-shader and transform it to the light-space, but this seems not to work. why?

- when and how does the light have to be transformed when lighting is applied
the normals are available in view-space in the deferred rendering lighting stage and so must be the lights position and direction because lighting has to be carried out always in the same space.
NOT IMPLEMENTED YET.

- when and how does the light have to be transformed when shadow maps are rendered
the lights modelmatrix is transformed to become a viewing matrix which is just the inverse of the modelmatrix. this is necessary because the light becomes the camera from which the scene is rendered.
IMPLEMENTED.

- when and how does the light have to be transformed when shadowing is applied
the light is not transformed during applying shadowing for each fragment because the fragments world-space coordinate, the depth-map and the lightspace matrix is enough for this process.
IMPLEMENTED.

- when and how do the objects have to be transformed when lighting is applied
the objects aren't anymore transformed, instead their view-space normals are used which are already in view-space when reading them in the lighting-stage. the lights position and direction must also be transformed to view-space. when the view-vector is needed just a normalization of the xyz vector of the fragments world-space is needed.
IMPLEMENTED.

- when and how do the objects have to be transformed when shadow maps are rendered
the objects are transformed with their model-matrix to world-space, then lights viewing matrix is applied, then lights projection matrix and the rest is taken care by opengl, so models are just rendered from a different point of view - the world-space stays the same.
IMPLEMENTED.

- when and how do the objects have to be transformed when shadowing is applied
each fragment of each object must be transformed from world-space (which is independent of viewers and though a good starting point, it could be even the object space but the inverse modelmatrix is no more available in the lighting-stage and we would have to apply it again because the world.space is the single space to start from in this case) to light-space and then further on into the unitcube 0-1.
IMPLEMENTED BUT SOMETHINGS WRONG.