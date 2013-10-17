#include "GLBuffer.h"

GLBuffer::GLBuffer(GLenum type) : type_(type), buffer_(0) { }

void GLBuffer::create() {
	glGenBuffers(1, &buffer_);
	checkError();
}

void GLBuffer::bind() {
	glBindBuffer(type_, buffer_);
	checkError();
}

void GLBuffer::allocate(const void* data, int count, GLenum pattern) {
	glBufferData(type_, count, data, pattern);
	checkError();
}

void GLBuffer::release() {
	glBindBuffer(type_, 0);
	checkError();
}

int GLBuffer::size() const {
	GLint value = -1;
	glGetBufferParameteriv(type_, GL_BUFFER_SIZE, &value);
	checkError();
	return value;
}

void* GLBuffer::map(GLuint access) {
	void* ptr = glMapBufferARB(type_, access);
	checkError();
	return ptr;
}

bool GLBuffer::unmap() {
	bool result = glUnmapBufferARB(type_) == GL_TRUE;
	checkError();
	return result;
}

bool GLBuffer::isCreated() const {
	return buffer_ != 0;
}

void GLBuffer::destroy() {
	glDeleteBuffers(1, &buffer_);
}

void GLBuffer::checkError() {
	GLenum err = glGetError();
	if (err != GL_NO_ERROR) {
		std::cout << "GL Error code: " << err << std::endl;
	}
}
