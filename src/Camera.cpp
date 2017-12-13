// **************************************************
// GLRK : the OpenGL Refinement Kernel.
// Camera.
// This file is a part of the GLRK library.
// Author : Tamy Boubekeur (Tamy.Boubekeur@labri.fr).
// Copyright (C) 2006 Tamy Boubekeur.
// All rights reserved.
// **************************************************

#include "Camera.h"
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
//#include <GL/gl.h>
//#include <GL/glu.h>
#include <iostream>

using namespace std;

static int _spinning, _moving;
static int _beginu, _beginv;
static float _curquat[4];
static float _x, _y, _z;
static float __zoom;
static bool ini = false;

Camera::Camera () {
  fovAngle = 45.0;
  aspectRatio = 1.0;
  nearPlane = 0.1;
  farPlane = 10000.0;
  
  spinning = 0;
  moving = 0;
  beginu = 0;
  beginv = 0;
  
  trackball (curquat, 0.0, 0.0, 0.0, 0.0);
  x = y = z = 0.0;
  _zoom = 3.0;
}


void Camera::resize (int _W, int _H) {
  H = _H;
  W = _W;
  glViewport (0, 0, (GLint)W, (GLint)H);
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  aspectRatio = static_cast<float>(W)/static_cast<float>(H);
  gluPerspective (fovAngle, aspectRatio, nearPlane, farPlane);
  glMatrixMode (GL_MODELVIEW);
}


void Camera::initPos () {
  if (!ini) {
  _spinning = spinning;
  _moving = moving;;
  _beginu = beginu;
  _beginv = beginv;
  _curquat[0] = curquat[0];
  _curquat[1] = curquat[1];
  _curquat[2] = curquat[2];
  _curquat[3] = curquat[3];
  _x = x;
  _y = y;
  _z = z;;
  __zoom = _zoom;
  ini = true;
  } else {
    spinning = _spinning;
    moving = _moving;;
    beginu = _beginu;
    beginv = _beginv;
    curquat[0] = _curquat[0];
    curquat[1] = _curquat[1];
    curquat[2] = _curquat[2];
    curquat[3] = _curquat[3];
    x = _x;
    y = _y;
    z = _z;;
    _zoom = __zoom;
  } 
}


void Camera::move (float dx, float dy, float dz) {
  x += dx;
  y += dy;
  z += dz;
}


void Camera::beginRotate (int u, int v) {
  beginu = u; 
  beginv = v;
  moving = 1;
  spinning = 0;
}


void Camera::rotate (int u, int v) {
  if (moving) {
    trackball(lastquat,
        (2.0 * beginu - W) / W,
        (H - 2.0 * beginv) / H,
        (2.0 * u - W) / W,
        (H - 2.0 * v) / H);
    beginu = u;
    beginv = v;
    spinning = 1;
    add_quats (lastquat, curquat, curquat);
  }
}


void Camera::endRotate () {
  moving = false;
}


void Camera::zoom (float z) {
  _zoom += z;
}


void Camera::apply () {
  glLoadIdentity();
  glTranslatef (x, y, z);
  GLfloat m[4][4]; 
  build_rotmatrix(m, curquat);
  glTranslatef (0.0, 0.0, -_zoom);
  glMultMatrixf(&m[0][0]);
}


void Camera::getPos (float & X, float & Y, float & Z) {
  GLfloat m[4][4]; 
  build_rotmatrix(m, curquat);
  float _x = -x;
  float _y = -y;
  float _z = -z + _zoom;
  X = m[0][0] * _x +  m[0][1] * _y +  m[0][2] * _z;
  Y = m[1][0] * _x +  m[1][1] * _y +  m[1][2] * _z;
  Z = m[2][0] * _x +  m[2][1] * _y +  m[2][2] * _z;

}

