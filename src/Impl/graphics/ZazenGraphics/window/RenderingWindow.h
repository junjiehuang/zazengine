/*
 * Window.h
 *
 *  Created on: 13.03.2013
 *      Author: Jonathan Thaler
 */

#ifndef RENDERING_WINDOW_H_
#define RENDERING_WINDOW_H_

#include <Windows.h>

#include <string>

class RenderingWindow
{
	public:
		static bool initialize( const std::string&, int, int, bool );
		static bool shutdown();
		static RenderingWindow& getRef() { return *RenderingWindow::instance; };

		bool toggleFullscreen();

		HWND getHandle() { return this->hWnd; };

		int getWidth() { return this->m_windowWidth; };
		int getHeight() { return this->m_windowHeight; };

		bool isActive() { return this->m_activeFlag; };
		
		bool swapBuffers();

	private:
		static RenderingWindow* instance;

		RenderingWindow();
		~RenderingWindow();

		std::string m_windowTitle;

		bool m_fullScreen;
		
		int m_windowWidth;
		int m_windowHeight;

		bool m_activeFlag;

		HDC hDC;
		HGLRC hRC;
		HWND hWnd;
		HINSTANCE hInstance;

		void resize( int, int );

		static bool registerClass( HINSTANCE );
		static bool createWindow( int, int, bool, const std::string& );
		static bool createCompatibilityRenderingContext();
		static bool createCoreRenderingContext();

		static bool unregisterClass();
		static bool destroyWindow();
		static bool destroyRenderingContext();

		static LRESULT CALLBACK WndProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam );

};

#endif /* WINDOW_H_ */