#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <cinder/gl/gl.h>
#include <Geometry.h>

/**
 * Camera
 * @author Stéphane GINIER
 */
class Camera
{

public:
    enum CameraMode{SPHERICAL,PLANE,FREELOOK};
    Camera();
    ~Camera();
    void setViewport(const Vector2& viewport);
    float getZoom() const;
    const Vector2& getViewport() const;
    const Matrix4x4& getViewMatrix() const;
    const Matrix4x4& getProjectionMatrix() const;
    void setMoveX(int direction);
    void setMoveZ(int direction);
    void setGlobalScale(float scale);
    bool isMoving() const;
    void setZoom(float zoom);
    CameraMode getMode() const;
    void setMode(CameraMode mode);

    void start(const Vector2& mouseXY);
    void rotate(const Vector2& mouseXY);
    void translate(const Vector2& mouseDiff);
    void updateView();
    void updateProjection();
    void zoom(float delta);
    void updateTranslation();
    void loadProjectionMatrix();
    void loadViewMatrix();
    void reset();

private:
    CameraMode mode_; //camera mode
    Quaternion rot_; //quaternion
    Matrix4x4 viewMatrix_; //view matrix
    Matrix4x4 projectionMatrix_; //projection matrix
    Vector2 normalizedMouseXY_; //mouse position (between 0 and 1)
    Vector2 viewport_; //viewport size
    float transX_; //x translation
    float transY_; //y translation
    float zoom_; //zoom value
    int moveX_; //free look (strafe)
    int moveZ_; //free look (strafe)
    Vector3 position_; //position (freelook)
    Vector3 center_;  //point where we are looking (freelook)
    Vector3 up_;  //up vector (freelook)
    float theta_; //x mouse angle (freelook)
    float phi_; //y mouse angle (freelook)
    float globalScale_; //solve scale issue
};

#endif /*__CAMERA_H__*/
