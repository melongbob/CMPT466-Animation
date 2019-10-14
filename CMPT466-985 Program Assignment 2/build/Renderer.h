#pragma once
#include <GL/glew.h>
#include "VertexArray.h"
#include "IndexBuffer.h"
#include "Shader.h"



//detecting errors
#define ASSERT(x) if (!(x)) __debugbreak();
#define GLCall(x) GlClearError();\
x;\
ASSERT(GLLOGCall(#x,__FILE__,__LINE__))

void GlClearError();

bool GLLOGCall(const char* function, const char* file, int line);

class Renderer
{
public:

	//clear the fram
	void Clear();
	//draw
	void Draw(const int type,const VertexArray& vao, const IndexBuffer& ebo, const Shader& shader)const;


};


