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
#include "Vec3.h"
#include "Triangle.h"

/// A Mesh class, storing a list of vertices and a list of triangles indexed over it.
class Mesh {
public:
    inline Mesh () {}
    inline virtual ~Mesh () {}

    /*inline std::vector<Vec3f> & vertices () { return _vertices; }
    inline const std::vector<Vec3f> & vertices () const { return _vertices; }
    inline  std::vector<Vec3f> & normals () { return _normals; }
    inline const std::vector<Vec3f> & normals () const { return _normals; }
    inline std::vector<Triangle> triangles () { return _triangles; }
    inline const std::vector<Triangle> & triangles () const { return _triangles; }*/

    /// Empty the positions, normals and triangles arrays.
    void clear ();

	/// Loads the mesh from a <file>.off or <file>.obj
	void loadMesh (const std::string & filename);

    
    /// Compute smooth per-vertex normals
    void recomputeNormals ();

    /// scale to the unit cube and center at original
    void centerAndScaleToUnit ();

    std::vector<Vec3f> vertices;
    std::vector<Vec3f> normals;
    std::vector<Triangle> triangles;
};

#endif // MESH_H
