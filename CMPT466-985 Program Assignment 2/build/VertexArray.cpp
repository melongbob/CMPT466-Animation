#include "VertexArray.h"
#include "VertexBufferLayout.h"



//generating vertex array
VertexArray::VertexArray()
{
	glGenVertexArrays(1, &m_RendererID);
	glBindVertexArray(m_RendererID);
}


VertexArray::~VertexArray()
{
	glDeleteVertexArrays(1, &m_RendererID);
}

//parsing the data
void VertexArray:: AddBuffer( const VertexBufferLayout& layout)
{
	//vb.Bind();
	const auto& elements = layout.GetlElements();
	unsigned int offset = 0;
	for (unsigned int i{ 0 }; i < elements.size(); i++)
	{
		const auto& element = elements[i];
		GLCall(glVertexAttribPointer(i, element.count, element.type, element.normalized, layout.GetStride(), (const void*)offset));
		glEnableVertexAttribArray(i);
		offset += element.count* VertexBufferElement::GetSizeOfType(element.type);
	}
}


void VertexArray::Bind() const
{
	glBindVertexArray(m_RendererID);
}

void VertexArray::Unbind() const
{
	glBindVertexArray(0);
}