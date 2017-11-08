// -------------------------------------------
// gMini : a minimal OpenGL/GLUT application
// for 3D graphics.
// Copyright (C) 2006-2008 Tamy Boubekeur
// All rights reserved.
// -------------------------------------------

// -------------------------------------------
// Disclaimer: this code is dirty in the
// meaning that there is no attention paid to
// proper class attribute access, memory
// management or optimisation of any kind. It
// is designed for quick-and-dirty testing
// purpose.
// -------------------------------------------

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>

#include <algorithm>
#include <GLUT/glut.h>
#include "Camera.h"
//#include "Mesh.h"
#include "tiny_obj_loader.h"

using namespace std;

// -------------------------------------------
// OpenGL/GLUT application code.
// -------------------------------------------

static GLint window;
static unsigned int SCREENWIDTH = 640;
static unsigned int SCREENHEIGHT = 480;
static Camera camera;
static bool mouseRotatePressed = false;
static bool mouseMovePressed = false;
static bool mouseZoomPressed = false;
static int lastX=0, lastY=0, lastZoom=0;
static unsigned int FPS = 0;
static bool fullScreen = false;

static vector<tinyobj::shape_t> shapes;
static vector<tinyobj::material_t> materials;
static Vec3f sceneCenter = Vec3f (0.f, 0.f, 0.f);
static float sceneRadius = 1.f;
static Vec3f lightPos = Vec3f (1.f, 1.f, 1.f);
static Vec3f lightColor = Vec3f (1.f, 1.f, 1.f);


//Mesh mesh;

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

void printUsage () {
    cerr << endl 
         << "gMini: a minimal OpenGL/GLUT application" << endl
         << "for 3D graphics." << endl 
         << "Author : Tamy Boubekeur (http://www.labri.fr/~boubek)" << endl << endl
         << "Usage : ./gmini [<file.off>]" << endl
         << "Keyboard commands" << endl 
         << "------------------" << endl
         << " ?: Print help" << endl 
         << " w: Toggle Wireframe Mode" << endl 
         << " g: Toggle Gouraud Shading Mode" << endl 
         << " f: Toggle full screen mode" << endl 
         << " <drag>+<left button>: rotate model" << endl 
         << " <drag>+<right button>: move model" << endl
         << " <drag>+<middle button>: zoom" << endl
         << " q, <esc>: Quit" << endl << endl; 
}

void usage () {
    //printUsage ();
    exit (EXIT_FAILURE);
}



// ------------------------------------

void initLight () {
    // GLfloat light_position1[4] = {22.0f, 16.0f, 50.0f, 0.0f};
    // GLfloat direction1[3] = {-52.0f,-16.0f,-50.0f};
    // GLfloat color1[4] = {0.5f, 1.0f, 0.5f, 1.0f};
    // GLfloat ambient[4] = {0.3f, 0.3f, 0.3f, 0.5f};
  
    // glLightfv (GL_LIGHT1, GL_POSITION, light_position1);
    // glLightfv (GL_LIGHT1, GL_SPOT_DIRECTION, direction1);
    // glLightfv (GL_LIGHT1, GL_DIFFUSE, color1);
    // glLightfv (GL_LIGHT1, GL_SPECULAR, color1);
    // glLightModelfv (GL_LIGHT_MODEL_AMBIENT, ambient);
    // glEnable (GL_LIGHT1);
    // glEnable (GL_LIGHTING);
    lightPos = Vec3f(0.f,sceneRadius/2.f,sceneRadius) ;
      glEnable (GL_LIGHTING);
      GLfloat position[4] = {lightPos[0], lightPos[1], lightPos[2], 1.0f};
      GLfloat color[4] = {lightColor[0], lightColor[1], lightColor[2], 1.0f};
      glLightfv (GL_LIGHT0, GL_POSITION, position);
      glLightfv (GL_LIGHT0, GL_DIFFUSE, color);
      glLightfv (GL_LIGHT0, GL_SPECULAR, color);
      glEnable (GL_LIGHT0);
}


// ------------------------------------
// Replace the code of this 
// functions for cleaning memory, 
// closing sockets, etc.
// ------------------------------------

void clear () {
  
}

// ------------------------------------
// Replace the code of this 
// functions for alternative rendering.
// ------------------------------------

void draw () {
    glBegin(GL_TRIANGLES);
    
    for (size_t s = 0; s < shapes.size (); s++){
        for (size_t f = 0; f < shapes[s].mesh.indices.size() / 3; f++) {
            // if (!materials.empty ()) {
            //     unsigned int i = shapes[s].mesh.material_ids[f];
            //     glColor3f (materials[i].diffuse[0], materials[i].diffuse[1], materials[i].diffuse[2]);
            // }
            for (size_t v = 0; v  < 3; v++) {
                unsigned int index = 3*shapes[s].mesh.indices[3*f+v];
                glColor3f (1.f, 0.f, 0.f);
                glNormal3f (shapes[s].mesh.normals[index],
                            shapes[s].mesh.normals[index+1],
                            shapes[s].mesh.normals[index+2]);
                glVertex3f (shapes[s].mesh.positions[index],
                            shapes[s].mesh.positions[index+1],
                            shapes[s].mesh.positions[index+2]);
            }
        }
    }
    glEnd ();
    glFlush (); // Ensures any previous OpenGL call has been executed
    glutSwapBuffers ();  // swap the render buffer and the displayed (screen) one
}

void display () {
    glLoadIdentity ();
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera.apply ();
    draw ();
    //glFlush ();
    //glutSwapBuffers ();
}



void idle () {
    static float lastTime = glutGet ((GLenum)GLUT_ELAPSED_TIME);
    static unsigned int counter = 0;
    counter++;
    float currentTime = glutGet ((GLenum)GLUT_ELAPSED_TIME);
    if (currentTime - lastTime >= 1000.0f) {
        FPS = counter;
        counter = 0;
        static char winTitle [64];
        // unsigned int numOfTriangles = mesh.triangles.size ();
        // sprintf (winTitle, "gmini - Num. Of Tri.: %d - FPS: %d", numOfTriangles, FPS);
        glutSetWindowTitle (winTitle);
        lastTime = currentTime;
    }
    glutPostRedisplay ();
}

void key (unsigned char keyPressed, int x, int y) {
    Vec3f l = Vec3f (50.0, 0.0, 0.0);
    switch (keyPressed) {
    case 'f':
        if (fullScreen == true) {
            glutReshapeWindow (SCREENWIDTH, SCREENHEIGHT);
            fullScreen = false;
        } else {
            glutFullScreen ();
            fullScreen = true;
        }      
        break;
    case 'q':
    case 27:
        clear ();
        exit (0);
        break;
    case 'w':
        glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
        break;
    case 'g':
        glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
        break;
    default:
        printUsage ();
        break;
    }
    idle ();
}

void mouse (int button, int state, int x, int y) {
    if (state == GLUT_UP) {
        mouseMovePressed = false;
        mouseRotatePressed = false;
        mouseZoomPressed = false;
    } else {
        if (button == GLUT_LEFT_BUTTON) {
            camera.beginRotate (x, y);
            mouseMovePressed = false;
            mouseRotatePressed = true;
            mouseZoomPressed = false;
        } else if (button == GLUT_RIGHT_BUTTON) {
            lastX = x;
            lastY = y;
            mouseMovePressed = true;
            mouseRotatePressed = false;
            mouseZoomPressed = false;
        } else if (button == GLUT_MIDDLE_BUTTON) {
            if (mouseZoomPressed == false) {
                lastZoom = y;
                mouseMovePressed = false;
                mouseRotatePressed = false;
                mouseZoomPressed = true;
            }
        }
    }
    idle ();
}

void motion (int x, int y) {
    if (mouseRotatePressed == true) {
        camera.rotate (x, y);
    }
    else if (mouseMovePressed == true) {
        camera.move ((x-lastX)/static_cast<float>(SCREENWIDTH), (lastY-y)/static_cast<float>(SCREENHEIGHT), 0.0);
        lastX = x;
        lastY = y;
    }
    else if (mouseZoomPressed == true) {
        camera.zoom (float (y-lastZoom)/SCREENHEIGHT);
        lastZoom = y;
    }
}


void reshape(int w, int h) {
    camera.resize (w, h);
}

void computeSceneNormals () {
  for (unsigned int s = 0; s < shapes.size (); s++)
    if (shapes[s].mesh.normals.empty ()) {
      shapes[s].mesh.normals.resize (shapes[s].mesh.positions.size (), 0.f);
      for (size_t f = 0; f < shapes[s].mesh.indices.size() / 3; f++) {
        Vec3f q[3];
        for (size_t v = 0; v < 3; v++) {
          unsigned int index = 3*shapes[s].mesh.indices[3*f+v];
          for (unsigned int i = 0; i < 3; i++)
            q[v][i] = shapes[s].mesh.positions[index+i];
        }
        Vec3f e01 = q[1] - q[0];
        Vec3f e02 = q[2] - q[0];
        Vec3f nf = normalize (cross (e01, e02));
        for (size_t v = 0; v < 3; v++) {
          unsigned int index = 3*shapes[s].mesh.indices[3*f+v];
          for (unsigned int i = 0; i < 3; i++)
            shapes[s].mesh.normals[index+i] += nf[i];
        }
      }
      for (unsigned int i = 0; i < shapes[s].mesh.normals.size () / 3; i++) {
        Vec3f n;
        for (unsigned int j = 0; j < 3; j++)
          n[j] = shapes[s].mesh.normals[3*i+j];
        n.normalize ();
        for (unsigned int j = 0; j < 3; j++)
          shapes[s].mesh.normals[3*i+j] = n[j];
      }
    }
}


void computeSceneBoundingSphere () {
    sceneCenter = Vec3f (0.f, 0.f, 0.f);
    unsigned int count = 0;
    for (unsigned int s = 0; s < shapes.size (); s++){
        for (unsigned int p = 0; p < shapes[s].mesh.positions.size () / 3; p++) {
            sceneCenter += Vec3f (shapes[s].mesh.positions[3*p],
                            shapes[s].mesh.positions[3*p+1],
                            shapes[s].mesh.positions[3*p+2]);
            count++;
        }
    }
    sceneCenter /= count;

    for (unsigned int s = 0; s < shapes.size (); s++){
        for (unsigned int p = 0; p < shapes[s].mesh.positions.size () / 3; p++)
        {
          shapes[s].mesh.positions[3*p] -= sceneCenter[0];
          shapes[s].mesh.positions[3*p+1] -= sceneCenter[1];
          shapes[s].mesh.positions[3*p+2] -= sceneCenter[2];
        }
    }
  sceneCenter = Vec3f (0.f, 0.f, 0.f);

  sceneRadius = 0.f;
    for (unsigned int s = 0; s < shapes.size (); s++){
        for (unsigned int p = 0; p < shapes[s].mesh.positions.size () / 3; p++) {
          float d = dist (sceneCenter, Vec3f (shapes[s].mesh.positions[3*p],
                                              shapes[s].mesh.positions[3*p+1],
                                              shapes[s].mesh.positions[3*p+2]));
            if (d > sceneRadius){
                sceneRadius = d;
            }
        }
    }
}

// Loads an OBJ file using tinyOBJ (http://syoyo.github.io/tinyobjloader/)
bool loadScene(const string & filename, const string & basepath = "") {
    shapes.clear ();
    materials.clear ();
    std::cout << "Loading " << filename << std::endl;
    std::string err = tinyobj::LoadObj(shapes, materials, filename.c_str (), basepath.c_str ());
    if (!err.empty()) {
        std::cerr << err << std::endl;
        return false;
    }
    std::cout << "shapes size: " << shapes.size() << std::endl;
    std::cout << "positions size: " << shapes[0].mesh.positions.size() << std::endl;
    std::cout << "normals size: " << shapes[0].mesh.normals.size() << std::endl;
    //std::cout << "shapes size: " << shpaes.size() << std::endl;

    computeSceneNormals ();
    computeSceneBoundingSphere ();
    return true;
}

void init (const char* _filename) {
    camera.resize (SCREENWIDTH, SCREENHEIGHT);
    //mesh.loadMesh (modelFilename);
    string filename = _filename;
    unsigned int i = filename.find_last_of ("/");
    loadScene (filename, filename.substr (0, i+1));
    initLight ();
    glCullFace (GL_BACK);
    glEnable (GL_CULL_FACE);
    glDepthFunc (GL_LESS);
    glEnable (GL_DEPTH_TEST);
    //glClearColor (0.2f, 0.2f, 0.3f, 1.0f);
    glClearColor (0.f, 0.f, 0.f, 1.0f);
}

int main (int argc, char ** argv) {
    if (argc > 2) {
        printUsage ();
        exit (EXIT_FAILURE);
    }
    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize (SCREENWIDTH, SCREENHEIGHT);
    window = glutCreateWindow ("Obj Viewer");  
    init(argc == 2 ? argv[1] : "cube.obj");
    glutIdleFunc (idle);
    glutDisplayFunc (display);
    glutKeyboardFunc (key);
    glutReshapeFunc (reshape);
    glutMotionFunc (motion);
    glutMouseFunc (mouse);
    key ('?', 0, 0);   
    glutMainLoop ();
    return EXIT_SUCCESS;
}

