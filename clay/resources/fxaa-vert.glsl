varying vec2 vertTexcoord;

void main() {
  vertTexcoord = gl_MultiTexCoord0;
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
