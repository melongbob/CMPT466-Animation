#pragma once

class IndexBuffer
{
private:

	unsigned int m_RendererID;
	unsigned int m_count;
public:
	//generating index buffer
	IndexBuffer(const void* data, unsigned int count);
	~IndexBuffer();

	void Bind() const;
	void Unbind() const;


	inline unsigned int GetCount() const { return m_count; }
};