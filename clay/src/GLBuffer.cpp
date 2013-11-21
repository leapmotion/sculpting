#include "StdAfx.h"
#include "GLBuffer.h"
#include "Common.h"

GLBuffer::GLBuffer(GLenum type) : type_(type), buffer_(0) { }

void GLBuffer::create() {
  glGenBuffers(1, &buffer_);
  checkError("Create");
}

void GLBuffer::bind() {
  glBindBuffer(type_, buffer_);
  checkError("Bind");
}

void GLBuffer::allocate(const void* data, int count, GLenum pattern) {
  glBufferData(type_, count, data, pattern);
  checkError("Allocate");
}

void GLBuffer::release() {
  glBindBuffer(type_, 0);
  checkError("Release");
}

int GLBuffer::size() const {
  GLint value = -1;
  glGetBufferParameteriv(type_, GL_BUFFER_SIZE, &value);
  checkError("Size");
  return value;
}

void* GLBuffer::map(GLuint access) {
  void* ptr = glMapBufferARB(type_, access);
  checkError("Map");
  return ptr;
}

bool GLBuffer::unmap() {
  bool result = glUnmapBufferARB(type_) == GL_TRUE;
  checkError("Unmap");
  return result;
}

bool GLBuffer::isCreated() const {
  return buffer_ != 0;
}

void GLBuffer::destroy() {
  glDeleteBuffers(1, &buffer_);
  buffer_ = 0;
}

void GLBuffer::checkError(const std::string& loc) {
#if !LM_PRODUCTION_BUILD
  GLenum err = glGetError();
  if (err != GL_NO_ERROR) {
    if (!loc.empty()) {
      std::cout << "At " << loc << ": ";
    }
    std::cout << "GL Error code: " << err << std::endl;
  }
#endif
}
