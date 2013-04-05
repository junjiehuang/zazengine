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
		static bool createRenderingWindow( const std::string&, int, int, bool );
		static bool destroyWindow();
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

		HDC hDC; // Private GDI Device Context
		HGLRC hRC; // Permanent Rendering Context
		HWND hWnd;	// Holds Our Window Handle
		HINSTANCE hInstance; // Holds The Instance Of The Application

		void resize( int, int );

		static LRESULT CALLBACK WndProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam );

};

#endif /* WINDOW_H_ */