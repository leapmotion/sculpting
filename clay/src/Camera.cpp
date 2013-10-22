#include "StdAfx.h"
#include "Camera.h"

/** Constructor */
Camera::Camera() : mode_(SPHERICAL), rot_(Quaternion::Identity()), viewMatrix_(Matrix4x4::Zero()), projectionMatrix_(Matrix4x4::Zero()), normalizedMouseXY_(Vector2::Zero()),
    viewport_(Vector2::Zero()), transX_(0), transY_(0), zoom_(20), moveX_(0), moveZ_(0), position_(0,0,20),
    center_(Vector3::Zero()), up_(0,1,0), theta_(-90), phi_(0), globalScale_(1)
{}

/** Destructor */
Camera::~Camera()
{}

/** Setters/Getters */
void Camera::setViewport(const Vector2& viewport) { viewport_ = viewport; glViewport(0, 0, static_cast<GLsizei>(viewport_.x()), static_cast<GLsizei>(viewport_.y()));}
float Camera::getZoom() const { return zoom_; }
const Vector2& Camera::getViewport() const { return viewport_; }
const Matrix4x4& Camera::getViewMatrix() const { return viewMatrix_; }
const Matrix4x4& Camera::getProjectionMatrix() const { return projectionMatrix_; }
void Camera::setMoveX(int direction) { moveX_ = direction; }
void Camera::setMoveZ(int direction) { moveZ_ = direction; }
void Camera::setGlobalScale(float scale) { globalScale_ = scale; }
bool Camera::isMoving() const { return moveX_!=0 || moveZ_!=0; }
void Camera::setZoom(float zoom) { position_.z() = zoom; zoom_ = zoom; }
Camera::CameraMode Camera::getMode() const { return mode_; }
void Camera::setMode(CameraMode mode)
{
    rot_ = Quaternion::Identity();
    mode_ = mode;
    float global = globalScale_;
    reset();
    globalScale_ = global;
    zoom(-0.4f);
}

/** Start camera (store mouse coordinates) */
void Camera::start(const Vector2& mouseXY)
{
    normalizedMouseXY_=Geometry::normalizedMouse(mouseXY, viewport_);
}

/** Compute rotation values (by updating the quaternion) */
void Camera::rotate(const Vector2& mouseXY)
{
#if 0
    Vector2 normalizedMouseXY = Geometry::normalizedMouse(mouseXY, viewport_);
    if (mode_==PLANE)
    {
        QLineF diff(normalizedMouseXY_, normalizedMouseXY);
        Vector3 axe = Vector3(-diff.dy(), diff.dx(), 0.0f).normalized();
				AngleAxis aa(diff.length()*180/M_PI, axe);
				rot_ = Quaternion(aa) * rot_;
    }
    else if (mode_==SPHERICAL)
    {
        Vector3 mouseOnSphereBefore = Geometry::mouseOnUnitSphere(normalizedMouseXY_);
        Vector3 mouseOnSphereAfter = Geometry::mouseOnUnitSphere(normalizedMouseXY);
        float angle=acosf(mouseOnSphereBefore.dot(mouseOnSphereAfter));
        Vector3 axe = mouseOnSphereBefore.cross(mouseOnSphereAfter);
				AngleAxis aa(angle*180/M_PI, axe);
				rot_ = Quaternion(aa) * rot_;
    }
    else if (mode_==FREELOOK)
    {
        QLineF diff(normalizedMouseXY_, normalizedMouseXY);
        theta_ += diff.dx()*50.f;
        phi_ += diff.dy()*50.f;
        if(phi_>89.f)
            phi_=89.f;
        else if (phi_<-89.f)
            phi_=-89.f;
        float theta=theta_*M_PI/180.f;
        float phi = phi_*M_PI/180.f;
        Vector3 view(cos(phi)*cos(theta),sin(phi),cos(phi)*sin(theta));
        center_ = position_ + view.normalized();
    }
    normalizedMouseXY_ = normalizedMouseXY;
#endif
}

/** Compute translation values */
void Camera::translate(const Vector2& mouseDiff)
{
    transX_-=mouseDiff.x()*globalScale_;
    transY_+=mouseDiff.y()*globalScale_;
}

/** Update model view matrices */
void Camera::updateView()
{
#if 0
    viewMatrix_.setIdentity();
    if(mode_==FREELOOK)
        viewMatrix_.lookAt(Vector3(position_.x(), position_.y(), position_.z()),
                           Vector3(center_.x(), center_.y(), center_.z()),Vector3( 0, 1, 0));
    else
    {
        viewMatrix_.lookAt(Vector3(transX_, transY_, zoom_),Vector3(transX_, transY_, 0),Vector3( 0, 1, 0));
        viewMatrix_.rotate(rot_);
    }
#endif
}

/** Update projection matrix */
void Camera::updateProjection()
{
#if 0
    projectionMatrix_.setIdentity();
    projectionMatrix_.perspective(70, (GLdouble)viewport_.width()/viewport_.height(), 0.01, 100000);
#endif
}

/** Zoom */
void Camera::zoom(float delta)
{
    if(mode_==FREELOOK)
    {
        Vector3 view = (center_-position_).normalized();
        Vector3 trans = view*delta*globalScale_;
        position_+=trans;
        center_+=trans;
    }
    else
        zoom_ = std::max(0.00001f, zoom_-delta*globalScale_);
}

/** Update translation */
void Camera::updateTranslation()
{
    if(mode_==FREELOOK)
    {
        Vector3 view = (center_-position_).normalized();
        Vector3 ortho = (view.cross(up_)).normalized();
        Vector3 trans = (ortho*static_cast<float>(moveX_) - view*static_cast<float>(moveZ_))*globalScale_/400.f ;
        position_+=trans;
        center_+=trans;
    }
    else
    {
        transX_+=moveX_*globalScale_/400.f;
        zoom_=std::max(0.00001f, zoom_+moveZ_*globalScale_/400.f);
    }
}
/** Load projection matrix (only for immediate mode...) */
void Camera::loadProjectionMatrix()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float *dataMat = projectionMatrix_.data();
    GLfloat matriceArray[16];
    for (int i= 0; i < 16; ++i)
        matriceArray[i] = dataMat[i];
    glMultMatrixf(matriceArray);
}

/** Load view matrix (only for immediate mode...) */
void Camera::loadViewMatrix()
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    float *dataMat = viewMatrix_.data();
    GLfloat matriceArray[16];
    for (int i= 0; i < 16; ++i)
        matriceArray[i] = dataMat[i];
    glMultMatrixf(matriceArray);
}

/** Reset camera */
void Camera::reset()
{
    rot_ = Quaternion();
    transX_ = 0;
    transY_ = 0;
    zoom_ = 0;
    globalScale_ = 1;
    position_ = Vector3(0,0,1);
    center_ = Vector3::Zero();
    theta_ = -90;
    phi_ = 0;
}
