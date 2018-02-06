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
#include <chrono>

using namespace std;
typedef std::chrono::high_resolution_clock Clock;
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
GLUquadricObj *quadratic;
GLfloat width[2];

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


string getStringFromTime(float i_timeInSeconds){
    ostringstream oss;

    float hms; // for hour minute second
    oss << i_timeInSeconds*1000.f << " ms";
    if(i_timeInSeconds >= 1.f)
    {
        oss << " ( ";
        if(i_timeInSeconds >= 3600.f)
        {
            hms = floor(i_timeInSeconds/3600.f);
            oss << hms << "h ";
            i_timeInSeconds -= hms * 3600.f;
        }
        if(i_timeInSeconds >= 60.f)
        {
            hms = floor(i_timeInSeconds/60.f);
            oss << hms << "min ";
            i_timeInSeconds -= hms * 60.f;
        }
        hms = floor(i_timeInSeconds);
        oss << hms << "s ";
        i_timeInSeconds -= hms;
        hms = floor(i_timeInSeconds*1000.f);
        oss << hms << "ms )";
    }

    return oss.str();
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
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND); // to use transparency
//    glClearColor (0.2f, 0.2f, 0.3f, 1.0f);
     glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glEnable (GL_COLOR_MATERIAL); // Dont forget this if you want to use glColor3f
    glGetFloatv(GL_LINE_WIDTH_RANGE, width);
    std::cout << "width between " << width[0] << " and " << width[1] << std::endl;
}


// ------------------------------------
// Replace the code of this
// functions for cleaning memory,
// closing sockets, etc.
// ------------------------------------

void clear () {

}

void drawCylinder(Vec3d startPoint, Vec3d endPoint, float length){

    Vec3d dir = endPoint - startPoint;
    // float length = (float)dir.length();
    // float length = 0.01;
    // std::cout << "length: " << dir.length() << std::endl;
    dir.normalize();
    Vec3d ez = Vec3d(0.0, 0.0, 1.0);
    Vec3d middlePoint = startPoint*0.5 + endPoint*0.5;
    float c = (float)dot(ez, dir);
    Vec3d v = cross(ez, dir);
    float v1 = (float)v[0];
    float v2 = (float)v[1];
    float v3 = (float)v[2];

    glPushMatrix();
    glTranslatef((float)middlePoint[0], (float)middlePoint[1], (float)middlePoint[2]-0.5*length);
    if( fabs(1.0f+c) < 0.0000001f || fabs(1.0f-c) < 0.0000001f){
        GLfloat rotation[16] = {1.0f, 0.0f, 0.0f, 0.0f,
                                0.0f, 1.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 1.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 1.0f
                                };
        // glLoadMatrixf(rotation);
        glMultMatrixf(rotation);
    }else{
        float inv = 1.0f/(1.0f+c);
        // GLfloat rotation[16] = { 1.0f-(v2*v2+v3*v3)*inv, v3+(v1*v2)*inv, -1.0f*v2+(v1*v3)*inv, 0.0f,
        //                         -1.0f*v3+(v1*v2)*inv, 1.0f-(v1*v1+v3*v3)*inv, v1+(v2*v3)*inv, 0.0f,
        //                         v2+(v1*v3)*inv, -1.0f*v1+(v2*v3)*inv, 1.0f-(v1*v1+v2*v2)*inv, 0.0f,
        //                         (float)startPoint[0], (float)startPoint[1], (float)startPoint[2], 1.0f
        //                         };
        GLfloat rotation[16] = {1.0f-(v2*v2+v3*v3)*inv, -1.0f*v3+(v1*v2)*inv, v2+(v1*v3)*inv, 0.0f,
                                v3+(v1*v2)*inv, 1.0f-(v1*v1+v3*v3)*inv, -1.0f*v1+(v2*v3)*inv, 0.0f,
                                -1.0f*v2+(v1*v3)*inv, v1+(v2*v3)*inv, 1.0f-(v1*v1+v2*v2)*inv, 0.0f,
                                0.0f, 0.0f, 0.0f, 1.0f
                                };
        // glLoadMatrixf(rotation);
        glMultMatrixf(rotation);
    }
    gluCylinder(quadratic, 0.002f, 0.002f, length, 16, 16);
    glPopMatrix();
}

void draw () {

    // Hand mesh
    // glPushMatrix();
    // glScalef(10.0f,10.0f,10.0f);
    glBegin (GL_TRIANGLES);
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

    // gluCylinder(quadratic, 0.5f, 0.5f, 5.0f, 16, 16);

    // Printing axis of local gcs
    // glBegin(GL_LINES);
    for(int i = 0; i < shape->localGCs.size(); i++){

        GC gc = shape->localGCs[i];
        assert(gc.axis.size() == 1);
//        glLineWidth(10.0);
        for(int j = 0; j < gc.axis.size(); j++){
            Vec3d p0 = gc.axis[j].interpolate(0.0);
            Vec3d p1 = gc.axis[j].interpolate(1.0);
            glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
            drawCylinder(p0, p1, 0.01f);
            // glVertex3f(p0[0], p0[1], p0[2]);
            // glVertex3f(p1[0], p1[1], p1[2]);

        }
    }
    // glEnd();
    // glPopMatrix();
    // Printing aligned profiles
    // glBegin(GL_LINES);
    // for(int i = 0; i < shape->localGCs.size(); i++){

    //     GC gc = shape->localGCs[i];
    //     for(int j = 0; j < gc.alignedProfiles.size(); j++){
    //         std::vector<Point_3> profile = gc.alignedProfiles[j];
    //         for(int k = 0; k < profile.size(); k++){
    //             Point_3 p = profile[k];
    //             glColor3f(0.0f, 0.0f, 1.0f);
    //             glVertex3f(p.x(), p.y(), p.z());
    //        }
    //     }
    // }
    // glEnd();
    // for(int i = 0; i < shape->localGCs.size(); i++){

    //     GC gc = shape->localGCs[i];
    //     assert(gc.axis.size() == 1);
    //     for(int j = 0; j < gc.axis.size(); j++){
    //         Vec3d p0 = gc.axis[j].interpolate(0.0);
    //         Vec3d p1 = gc.axis[j].interpolate(1.0);
    //         glColor3f(0.0f, 0.0f, 1.0f);
    //         drawCylinder(p0, p1);

    //     }
    // }
    // Printing axis of nonlocal gcs as line [ps,pe]
    glBegin(GL_LINES);
    for(int i = 0; i < shape->nonLocalGCs.size(); i++){

        float prop;
        if(i < (shape->nonLocalGCs.size()-1)){
            prop = (float)i/(i - shape->nonLocalGCs.size() + 1.f);
        }else{
            prop = 1.0f;
        }
        float gval = 1.0f - prop;
        float bval = prop;
        GC gc = shape->nonLocalGCs[i];
        glColor4f(0.0f, gval, bval, 1.0f);
        // glLineWidth(5.0);
        for(int j = 0; j < gc.axis.size(); j++){
            Vec3d p0 = gc.axis[j].interpolate(0.0);
            Vec3d p1 = gc.axis[j].interpolate(1.0);
            // glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
            glVertex3f(p0[0], p0[1], p0[2]);
            glVertex3f(p1[0], p1[1], p1[2]);
            // float length = (p1 - p0).normalize();
            // drawCylinder(p0, p1, length);
        }
    }
    glEnd();

    // Printing profile curves
   /*glBegin(GL_POINTS);

   for(int i = 0; i < shape->localGCs.size(); i++){

       GC gc = shape->localGCs[i];
       for(int j = 0; j < gc.alignedProfiles.size(); j++){

           std::vector<Point_3> profile = gc.alignedProfiles[j];
           for(int k = 0; k < profile.size(); k++){

               Point_3 p = profile[k];
               glColor3f(0.6f, 0.0f, 0.4f);
               glVertex3f(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
           }

       }
   }
   glEnd();*/
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
        sprintf (winTitle, "Num. Of Tri.: %d - FPS: %d", numOfTriangles, FPS);
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

    double tau = std::atof(argv[3]);
    double epsilon = std::atof(argv[4]);
    double dn = std::atof(argv[5]);
    std::cout << "tau: " << tau <<", epsilon: " << epsilon << ", dn: " << dn << std::endl;
    shape = new Shape(0.1, 1.0, tau, epsilon, dn);
    std::ifstream input(filename);
    input >> shape->mesh;
    shape->initLocalGCs(argv[1], argv[2]);
    std::cout << "shape local size: " << shape->localGCs.size() << std::endl;

    shape->mergeLocalGCs();
    std::cout << "shape non local size: " << shape->nonLocalGCs.size() << std::endl;

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
    quadratic = gluNewQuadric();
    const char *filename = argc == 4 ? argv[1] : "../hand_mesh.off";

    // argv 1: points 2: rosa normals 3:tau 4: epsilon 5: dn
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
    if (argc != 6) {
        // printUsage ();
        std::cout << "Wrong number of inputs" << std::endl;
        exit (EXIT_FAILURE);
    }

    initGlut(argc, argv);

    return EXIT_SUCCESS;
}

