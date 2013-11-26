#ifndef _SKYBOX_H_
#define _SKYBOX_H_

#include "../Geometry/Mesh.h"
#include "../Texture/Texture.h"
#include "../Program/UniformBlock.h"
#include "../Viewer/Viewer.h"

#include <boost/filesystem.hpp>

class SkyBox
{
	public:
		static bool initialize( const boost::filesystem::path&, const std::string& );
		static bool shutdown();
		static bool isPresent() { return SkyBox::instance != NULL; };

		static SkyBox& getRef() { return *SkyBox::instance; };

		bool render( const Viewer&, UniformBlock*, UniformBlock* );

	private:
		static SkyBox* instance;

		Mesh* m_cubeMesh;
		Texture* m_cubeMap;

		SkyBox();
		~SkyBox();
};

#endif /* _SKYBOX_H_ */
