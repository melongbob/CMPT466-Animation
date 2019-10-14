#pragma once

#include"VertexBuffer.h"


class VertexBufferLayout;

class VertexArray
{
private:
	unsigned int m_RendererID;


public:
	//generating vertex array
	VertexArray() ;
	~VertexArray();
	//parsing data
	void AddBuffer( const VertexBufferLayout& layout);

		void Bind() const;
		void Unbind() const;
};

