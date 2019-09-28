#include "Renderer.h"
#include <iostream>


//checking errors, you can check errors wherever you want by applying GLCall() on them
void GlClearError()
{
	while (glGetError() != GL_NO_ERROR);

}

bool GLLOGCall(const char* function, const char* file, int line)
{
	while (GLenum error = glGetError())
	{
		std::cout << "[OpenGL_Error] (" << error << ")" << function << " "
			<< file << " : " << line << std::endl;
		return false;
	}
	return true;
}

//draw
void Renderer::Draw(const int type,const VertexArray& vao, const IndexBuffer& ebo, const Shader& shader)const
{
	shader.Bind();
	vao.Bind();
	ebo.Bind();
	GLCall(glDrawElements(type, ebo.GetCount(), GL_UNSIGNED_INT, 0));

}

//clear the frame
void Renderer::Clear()
{
	glClearColor(0.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);

}