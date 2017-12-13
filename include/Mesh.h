// --------------------------------------------------------------------------
// Copyright(C) 2009-2016
// Tamy Boubekeur
//
// Permission granted to use this code only for teaching projects and
// private practice.
//
// Do not distribute this code outside the teaching assignements.
// All rights reserved.
// --------------------------------------------------------------------------
#ifndef MESH_H
#define MESH_H

#include <cmath>
#include <vector>
#include <string>
#include "Vec3.h"

class Vertex {
public:
    inline Vertex () {}
    inline Vertex (const Vec3f & p, const Vec3f & n) : p (p), n (n) {}
    inline Vertex (const Vertex & v) : p (v.p), n (v.n) {}
    inline virtual ~Vertex () {}
    inline Vertex & operator= (const Vertex & v) {
        p = v.p;
        n = v.n;
        return (*this);
    }
    Vec3f p;
    Vec3f n;
};

class Triangle {
public:
    inline Triangle () {
        v[0] = v[1] = v[2] = 0;
    }
    inline Triangle (const Triangle & t) {
        v[0] = t.v[0];
        v[1] = t.v[1];
        v[2] = t.v[2];
    }
    inline Triangle (unsigned int v0, unsigned int v1, unsigned int v2) {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
    }
    inline virtual ~Triangle () {}
    inline Triangle & operator= (const Triangle & t) {
        v[0] = t.v[0];
        v[1] = t.v[1];
        v[2] = t.v[2];
        return (*this);
    }
    unsigned int v[3];
};

class Mesh {
public:
    inline Mesh () {}
    inline virtual ~Mesh () {}
    std::vector<Vertex> V;
    std::vector<Triangle> T;

    void loadOFF (const std::string & filename);
    void recomputeNormals ();
    void centerAndScaleToUnit ();
    // void scaleUnit ();
    void clear ();
};

#endif // MESH_H
