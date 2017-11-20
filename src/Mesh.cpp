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

#include "Mesh.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>

using namespace std;

void Mesh::clear () {
    vertices.clear ();
    normals.clear ();
    triangles.clear ();
}

void Mesh::loadMesh (const std::string & filename) {
    vertices.clear ();
    triangles.clear();
	ifstream in (filename.c_str ());
    if (!in)
        exit (1);

    unsigned int sizeV, sizeT, tmp1, tmp2, tmp3; //because 0 at end of each lines
    in >> sizeV >> sizeT;
    cout << "size V: " << sizeV <<", sizeT: "<<sizeT << std::endl;
    vertices.resize (sizeV);
    triangles.resize (sizeT);
    float x, y, z;
    char feature; // f or v
    for (unsigned int i = 0; i < sizeV; i++){
        in >> feature >> x >> y >> z;// >> tmp1 >> tmp2 >> tmp3 ;
        //cout << "x: " << x <<" y: " <<y << " z: "<< z << std::endl;
        vertices[i] = Vec3f(x,y,z);
    }
    char feature2;
    for (unsigned int i = 0; i < sizeT; i++) {
         in >> feature2;
        for (unsigned int j = 0; j < 3; j++){
            in >> triangles[i][j];
        }
        //cout << "x: " << triangles[i][0] <<" y: " <<triangles[i][1] << " z: "<< triangles[i][2] << std::endl;
    }

    in.close ();
    centerAndScaleToUnit ();
    recomputeNormals ();
}

void Mesh::recomputeNormals () {
    normals.clear ();
    normals.resize (vertices.size (), Vec3f (0.f, 0.f, 0.f));
    for (unsigned int i = 0; i < triangles.size (); i++) {
        Vec3f e01 = vertices[triangles[i][1]] -  vertices[triangles[i][0]];
        Vec3f e02 = vertices[triangles[i][2]] -  vertices[triangles[i][0]];
        Vec3f n = cross (e01, e02);
        n.normalize ();
        for (unsigned int j = 0; j < 3; j++)
            normals[triangles[i][j]] += n;
    }
    for (unsigned int i = 0; i < normals.size (); i++)
        normals[i].normalize ();
}

void Mesh::centerAndScaleToUnit () {
    Vec3f c;
    for  (unsigned int i = 0; i < vertices.size (); i++)
        c += vertices[i];
    c /= vertices.size ();
    float maxD = dist (vertices[0], c);
    for (unsigned int i = 0; i < vertices.size (); i++){
        float m = dist (vertices[i], c);
        if (m > maxD)
            maxD = m;
    }
    for  (unsigned int i = 0; i < vertices.size (); i++)
        vertices[i] = (vertices[i] - c) / maxD;
}
