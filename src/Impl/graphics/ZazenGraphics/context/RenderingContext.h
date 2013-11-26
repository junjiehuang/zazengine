#ifndef _RENDERING_CONTEXT_H_
#define _RENDERING_CONTEXT_H_

#include <boost/filesystem.hpp>

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <string>

class RenderingContext
{
	public:
		static bool initialize( const std::string&, int, int, bool, boost::filesystem::path );
		static bool shutdown();
		static RenderingContext& getRef() { return *RenderingContext::instance; };

		bool toggleFullscreen();

		void* getHandle() { return this->m_hWnd; };

		int getWidth() { return this->m_windowWidth; };
		int getHeight() { return this->m_windowHeight; };

		bool isActive() { return this->m_activeFlag; };
		
		bool swapBuffers();

		bool takeScreenShot();

	private:
		static RenderingContext* instance;

		RenderingContext( const std::string& title, int width, int height, bool fullScreenFlag, boost::filesystem::path screenShotPath );
		~RenderingContext();

		std::string m_windowTitle;

		bool m_fullScreen;
		
		int m_windowWidth;
		int m_windowHeight;

		bool m_activeFlag;

		void* m_hWnd;
		GLFWwindow* m_window;

		bool m_screnShotFlag;
		boost::filesystem::path m_screenShotPath;

		void resize( int, int );

		static bool initGLFW();
		static bool initGlew();
		static bool initDebugging();

		static void APIENTRY debugOutput( GLenum, GLenum,GLuint, GLenum, GLsizei, const GLchar*, GLvoid* );
};

#endif /* _RENDERING_CONTEXT_H_ */