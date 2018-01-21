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
//#include <GL/glut.h> //not good for mac
#include "Vec3.h"
#include "Camera.h"
#include "Mesh.h"
#include "Shape.h"
#include "Utils.h"

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

Mesh mesh;
std::vector<CGAL_Mesh> meshes;
std::vector<Point_3> points;
std::vector<std::vector<Vec3d>> lines;
Shape* shape;

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
    printUsage ();
    exit (EXIT_FAILURE);
}



// ------------------------------------

void initLight () {
    GLfloat light_position1[4] = {22.0f, 16.0f, 50.0f, 0.0f};
    GLfloat direction1[3] = {-52.0f,-16.0f,-50.0f};
    // GLfloat color1[4] = {0.5f, 1.0f, 0.5f, 1.0f};
    // GLfloat ambient[4] = {0.3f, 0.3f, 0.3f, 0.5f};
    GLfloat color1[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    glLightfv (GL_LIGHT1, GL_POSITION, light_position1);
    glLightfv (GL_LIGHT1, GL_SPOT_DIRECTION, direction1);
    glLightfv (GL_LIGHT1, GL_DIFFUSE, color1);
    glLightfv (GL_LIGHT1, GL_SPECULAR, color1);
    // glLightModelfv (GL_LIGHT_MODEL_AMBIENT, ambient);
    glEnable (GL_LIGHT1);
    glEnable (GL_LIGHTING);
}

void init (const char * modelFilename) {
    camera.resize (SCREENWIDTH, SCREENHEIGHT);
    mesh.loadOFF (modelFilename);
    initLight ();
    glCullFace (GL_BACK);
    glEnable (GL_CULL_FACE);
    glDepthFunc (GL_LESS);
    glEnable (GL_DEPTH_TEST);
//    glClearColor (0.2f, 0.2f, 0.3f, 1.0f);
     glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glEnable (GL_COLOR_MATERIAL); // Dont forget this if you want to use glColor3f
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

    glBegin (GL_TRIANGLES);

    // Hand mesh
    for (unsigned int i = 0; i < mesh.T.size(); i++){
        for (unsigned int j = 0; j < 3; j++) {
            const Vertex & v = mesh.V[mesh.T[i].v[j]];
            // glColor3f(0.6f, 0.6f, 1.0f);
            glColor4f(0.8f, 0.8f, 0.8f, 0.3f);
            glNormal3f (v.n[0], v.n[1], v.n[2]);
            glVertex3f (v.p[0], v.p[1], v.p[2]);
        }
    }
    glEnd ();
    /*for(int index = 0; index < meshes.size(); index++){
        CGAL_Mesh gc = meshes[index];
        BOOST_FOREACH(CGAL_Mesh::Vertex_index vd, gc.vertices()){
            Point_3 point =  gc.point(vd);
            glColor3f(0.6f, 0.0f, 0.4f);
            glVertex3f(point.x(), point.y(), point.z());
        }
    }*/

    // Printing axis as line [ps,pe]
    glBegin(GL_LINES);

    for(int i = 0; i < shape->localGCs.size(); i++){

        GC gc = shape->localGCs[i];
        assert(gc.axis.size() == 1);
        for(int j = 0; j < gc.axis.size(); j++){
            Vec3d p0 = gc.axis[j].interpolate(0.0);
            Vec3d p1 = gc.axis[j].interpolate(1.0);
            glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(p0[0], p0[1], p0[2]);
            glVertex3f(p1[0], p1[1], p1[2]);

        }
    }
    glEnd();

    // Printing profile curves
   glBegin(GL_POINTS);

   for(int i = 0; i < shape->localGCs.size(); i++){

       GC gc = shape->localGCs[i];
       for(int j = 0; j < gc.rotatedProfiles.size(); j++){

           std::vector<Point_3> profile = gc.rotatedProfiles[j];
           for(int k = 0; k < profile.size(); k++){

               Point_3 p = profile[k];
               glColor3f(0.6f, 0.0f, 0.4f);
               glVertex3f(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
           }

       }
   }
   glEnd();
}

void display () {
    glLoadIdentity ();
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera.apply ();
    draw();
    glFlush ();
    glutSwapBuffers ();
}

void idle () {
    static float lastTime = glutGet ((GLenum)GLUT_ELAPSED_TIME);
    static unsigned int counter = 0;
    counter++;;
    float currentTime = glutGet ((GLenum)GLUT_ELAPSED_TIME);
    if (currentTime - lastTime >= 1000.0f) {
        FPS = counter;
        counter = 0;
        static char winTitle [64];
        unsigned int numOfTriangles = mesh.T.size ();
        sprintf (winTitle, "gmini - Num. Of Tri.: %d - FPS: %d", numOfTriangles, FPS);
        glutSetWindowTitle (winTitle);
        lastTime = currentTime;
    }
    glutPostRedisplay ();
}

void key (unsigned char keyPressed, int x, int y) {
    Vec3d l = Vec3d (50.0, 0.0, 0.0);
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
    case 's':

        //vector<float> rad (mesh.V.size (), 0.0);
        for (unsigned int i = 0; i < mesh.V.size (); i++) {
            Vec3d pl = l - mesh.V[i].p;
            pl.normalize ();
            // float rad/*[i]*/ = max (dot(mesh.V[i].n, pl), 0.0f);
            double rad/*[i]*/ = max (dot(mesh.V[i].n, pl), 0.0);
            mesh.V[i].n *= rad/*[i]*/;
        }
        break;
    case 'a':
        if (mouseZoomPressed == false) {
                lastZoom = y;
                mouseMovePressed = false;
                mouseRotatePressed = false;
                mouseZoomPressed = true;
        }
        break;
    default:
        // printUsage ();
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
        } else if (button == GLUT_KEY_F1) {
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

/*void initLines(int argc, char **argv, const char *filename){

    Shape *shape = new Shape();
    std::ifstream input(filename);
    input >> shape->mesh;
    shape->initLocalGCs(argv[1], argv[2], 0.1);
    int nbLocalGCs = shape->localGCs.size();

    for(int index = 0; index < nbLocalGCs; index++){

        GC gc = shape->localGCs[index];
        HermiteCurve axis = gc.axis[0];
        std::vector<Vec3d> points;
        for(int t = 0; t <= 3; t++){
            points.push_back(axis.interpolate(t/3.0));
        }
        lines.push_back(points);
    }
    std::cout << "There is " << lines.size() << " local GCs." << std::endl;
}*/

void initMeshes(int argc, char **argv, const char *filename){

    shape = new Shape();
    std::ifstream input(filename);
    input >> shape->mesh;
    shape->initLocalGCs(argv[1], argv[2], 0.001);

    // shape->mergeLocalGCs();

    // int nbLocalGCs = shape->localGCs.size();

    // for(int index = 0; index < nbLocalGCs; index++){

    //     std::vector<Point_3> GC_points = shape->localGCs[index].getAllPoints();
    //     meshes.push_back(Utils::generateMesh(GC_points));
    // }
}

void initGlut(int argc, char **argv){
    glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize (SCREENWIDTH, SCREENHEIGHT);
    window = glutCreateWindow ("GCD Viewer");

    const char *filename = argc == 4 ? argv[1] : "../hand_mesh.off";

    initMeshes(argc, argv, filename);
    // initLines(argc, argv, filename);
    init(filename);

    glutIdleFunc (idle);
    glutDisplayFunc (display);
    glutKeyboardFunc (key);
    glutReshapeFunc (reshape);
    glutMotionFunc (motion);
    glutMouseFunc (mouse);
    key ('?', 0, 0);
    glutMainLoop ();
}

int main (int argc, char ** argv) {
    if (argc != 3) {
        // printUsage ();
        std::cout << "Wrong number of inputs" << std::endl;
        exit (EXIT_FAILURE);
    }
    initGlut(argc, argv);

    return EXIT_SUCCESS;
}

