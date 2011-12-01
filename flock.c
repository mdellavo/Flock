/*
** flock
** 
** Made by Marc DellaVolpe(marc.dellavolpe@gmail.com)
**
** gcc -Wall -std=c99 -O3 -o flock flock.c `sdl-config --cflags` `sdl-config --libs` -lGL -lGLU 
*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

#include "SDL.h"

#include <GL/gl.h>
#include <GL/glu.h>

typedef uint8_t bool;
#define true  0x1
#define false 0x0

#define PI 3.1415926535
#define RAND_RANGE(MIN, MAX) (MIN + ((MAX-MIN) * (rand() / (RAND_MAX + 1.0))))

#define WIDTH      800
#define HEIGHT     600
#define VIEW_ANGLE 90.0
#define FLAGS      (SDL_OPENGL | SDL_ANYFORMAT )

#define FPS               20
#define MS_BETWEEN_FRAMES (int)round(1000.0/FPS)

#define MAX_DEPTH 1000.0

#define X_MIN -150.0
#define Y_MIN -150.0
#define Z_MIN -100.0
#define X_MAX 150.0
#define Y_MAX 150.0
#define Z_MAX 150.0

#define CAMERA_X 0.0
#define CAMERA_Y 0.0
#define CAMERA_Z 250.0

#define DRAW_BOUNDS false
#define BOID_SIZE   5.0
#define HISTORY     100

#define FLOCK_SIZE 200

#define SCAN_RADIUS     40.0
#define CENTER_FACTOR   0.01
#define AVOID_RADIUS    20.0
#define MATCH_FACTOR    0.75
#define MAX_FLOCK_SPEED 2.0
#define MAX_FLOCK_ACCEL 0.25
#define BIND_FACTOR     1.0

static GLubyte red[]    = { 255,   0,   0, 255 };
static GLubyte green[]  = {   0, 255,   0, 255 };
static GLubyte blue[]   = {   0,   0, 255, 255 };
static GLubyte white[]  = { 255, 255, 255, 255 };
static GLubyte yellow[] = {   0, 255, 255, 255 };
static GLubyte black[]  = {   0,   0,   0, 255 };
static GLubyte orange[] = { 255, 255,   0, 255 };
static GLubyte purple[] = { 255,   0, 255,   0 };

typedef GLfloat vector_t[3];

typedef struct {
  vector_t position;
  vector_t velocity;
  vector_t last[HISTORY];
  GLubyte color[4];
} boid_t;
boid_t flock[FLOCK_SIZE];

inline void vector_add(vector_t v1, vector_t v2)
{
  v1[0] += v2[0];
  v1[1] += v2[1];
  v1[2] += v2[2];
}

inline void vector_sub(vector_t v1, vector_t v2)
{
  v1[0] -= v2[0];
  v1[1] -= v2[1];
  v1[2] -= v2[2];
}

inline void vector_scale(vector_t v, double i)
{
  v[0] *= i;
  v[1] *= i;
  v[2] *= i;
}

inline double vector_length(vector_t v)
{
  return sqrt( pow(v[0],2) + pow(v[1],2) + pow(v[2],2));
}

inline double vector_distance(vector_t a, vector_t b)
{
  vector_t v = { b[0]-a[0], b[1]-a[1], b[2]-a[2] };
  return vector_length(v);
}

inline void flock_tick(void) 
{
  for(int i=0; i<FLOCK_SIZE; i++) {

    double len;
    int interactions = 0;

    vector_t v1  = { 0.0, 0.0, 0.0 };
    vector_t v2  = { 0.0, 0.0, 0.0 };
    vector_t v3  = { 0.0, 0.0, 0.0 };
    vector_t tmp = { 0.0, 0.0, 0.0 };

    for(int j=0; j<FLOCK_SIZE; j++) {
      if( i != j && 
	  vector_distance(flock[i].position, flock[j].position) < SCAN_RADIUS) {

	interactions++;
     	
	// Rule 1 - sum up positions of boids near current
	vector_add(v1, flock[j].position);
      
	// Rule 2 - avoid other boids
	if(vector_distance(flock[i].position, flock[j].position) < AVOID_RADIUS) {
	  vector_t tmp = { 0.0, 0.0, 0.0 };
	  vector_add(tmp, flock[j].position);
	  vector_sub(tmp, flock[i].position);
	  vector_sub(v2, tmp);
	}   

	// Rule 3 - sum up velocities of boids near current 
	vector_add(v3, flock[j].velocity);
      }
    }

    // Rule 1
    // multiply by 1/(N-1)
    vector_scale(v1, 1.0 / (double)interactions);
    vector_sub(v1, flock[i].position);
    vector_scale(v1, CENTER_FACTOR);

    // Rule 3
    // multiply by 1/(N-1)
    vector_scale(v3, 1.0 / (double)interactions);
    vector_sub(v3, flock[i].velocity);
    vector_scale(v3, MATCH_FACTOR);

    // Combine Rules
    vector_add(tmp, v1);
    vector_add(tmp, v2);
    vector_add(tmp, v3);

    if(tmp[0] > MAX_FLOCK_ACCEL)
      tmp[0] = MAX_FLOCK_ACCEL;
    else if(tmp[0] < -MAX_FLOCK_ACCEL)
      tmp[0] = -MAX_FLOCK_ACCEL;
      
    if(tmp[1] > MAX_FLOCK_ACCEL)
      tmp[1] = MAX_FLOCK_ACCEL;
    else if(tmp[1] < -MAX_FLOCK_ACCEL)
      tmp[1] = -MAX_FLOCK_ACCEL;

    if(tmp[2] > MAX_FLOCK_ACCEL)
      tmp[2] = MAX_FLOCK_ACCEL;
    else if(tmp[2] < -MAX_FLOCK_ACCEL)
      tmp[2] = -MAX_FLOCK_ACCEL;
    
    vector_add(flock[i].velocity, tmp);

    // Clamp speed
    len = vector_length(flock[i].velocity);
    if(len > MAX_FLOCK_SPEED)
      vector_scale(flock[i].velocity, MAX_FLOCK_SPEED / len);

    // Add new velocity to position
    vector_add(flock[i].position, flock[i].velocity);      

    // Bind to box
    if(flock[i].position[0] < X_MIN)
      flock[i].position[0] += BIND_FACTOR;
    else if(flock[i].position[0] > X_MAX)
      flock[i].position[0] -= BIND_FACTOR;
    
    if(flock[i].position[1] < Y_MIN)
      flock[i].position[1] += BIND_FACTOR;
    else if(flock[i].position[1] > Y_MAX)
      flock[i].position[1] -= BIND_FACTOR;

    if(flock[i].position[2] < Z_MIN)
      flock[i].position[2] += BIND_FACTOR;
    else if(flock[i].position[2] > Z_MAX)
      flock[i].position[2] -= BIND_FACTOR;
  }
}

void gfx_init(int width, int height)
{
  int                 bpp;
  SDL_Surface         *screen;
  const SDL_VideoInfo *info = NULL;
  int                 flags = FLAGS;
    
  if(SDL_Init(SDL_INIT_VIDEO| SDL_INIT_TIMER)<0) {
    fprintf(stderr, "Unable to init SDL: %s\n", SDL_GetError());
    exit(1);
  }
  atexit(SDL_Quit);

  SDL_ShowCursor(SDL_DISABLE);

  info = SDL_GetVideoInfo();

  if(info == NULL) {
    fprintf(stderr, "Unable to get display information: %s\n", SDL_GetError());
    exit(1);
  }

  bpp = info->vfmt->BitsPerPixel;

  // Extra opengl stuff, i took it from some example
  SDL_GL_SetAttribute( SDL_GL_RED_SIZE,     5);
  SDL_GL_SetAttribute( SDL_GL_GREEN_SIZE,   5);
  SDL_GL_SetAttribute( SDL_GL_BLUE_SIZE,    5);
  SDL_GL_SetAttribute( SDL_GL_DEPTH_SIZE,   16);
  SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1);
  
  screen = SDL_SetVideoMode(width, height, bpp, flags);
  if (screen == NULL) {
    fprintf(stderr, "Unable to initialize display: %s\n", SDL_GetError());
    exit(1);
  }

  fprintf(stderr, "Initialized screen at %dx%d@%d\n", width, height, bpp);

  // more stuff i ripped off from sdl guide
  glShadeModel( GL_SMOOTH );
  glCullFace( GL_BACK );
  glFrontFace( GL_CCW );
  glEnable( GL_CULL_FACE );

  // AA Lines
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
  glHint(GL_POINT_SMOOTH_HINT, GL_DONT_CARE);

  glClearColor( 0, 0, 0, 0 );
  glViewport( 0, 0, width, height );

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  gluPerspective(VIEW_ANGLE, (double)width/(double)height, 1, MAX_DEPTH);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(CAMERA_X, CAMERA_Y, CAMERA_Z,
	    0.0, 0.0, 0.0,
	    0.0, 1.0, 0.0);

  glPointSize(5.0);
}

void gfx_render(void)
{  
  glClear(GL_COLOR_BUFFER_BIT);
  
  if(DRAW_BOUNDS) {
    glLineWidth(2.0);

    glColor4ubv(white);

    glBegin(GL_LINES);

    // Front face
    glVertex3f( X_MIN, Y_MIN, Z_MIN );
    glVertex3f( X_MAX, Y_MIN, Z_MIN );

    glVertex3f( X_MAX, Y_MIN, Z_MIN );
    glVertex3f( X_MAX, Y_MAX, Z_MIN );

    glVertex3f( X_MAX, Y_MAX, Z_MIN );
    glVertex3f( X_MIN, Y_MAX, Z_MIN );

    glVertex3f( X_MIN, Y_MAX, Z_MIN );
    glVertex3f( X_MIN, Y_MIN, Z_MIN );

    // Back face
    glVertex3f( X_MIN, Y_MIN, Z_MAX );
    glVertex3f( X_MAX, Y_MIN, Z_MAX );

    glVertex3f( X_MAX, Y_MIN, Z_MAX );
    glVertex3f( X_MAX, Y_MAX, Z_MAX );

    glVertex3f( X_MAX, Y_MAX, Z_MAX );
    glVertex3f( X_MIN, Y_MAX, Z_MAX );

    glVertex3f( X_MIN, Y_MAX, Z_MAX );
    glVertex3f( X_MIN, Y_MIN, Z_MAX );

    // Connecting front and back
    glVertex3f( X_MIN, Y_MIN, Z_MIN );
    glVertex3f( X_MIN, Y_MIN, Z_MAX );

    glVertex3f( X_MAX, Y_MIN, Z_MIN );
    glVertex3f( X_MAX, Y_MIN, Z_MAX );

    glVertex3f( X_MAX, Y_MAX, Z_MIN );
    glVertex3f( X_MAX, Y_MAX, Z_MAX );

    glVertex3f( X_MIN, Y_MAX, Z_MIN );
    glVertex3f( X_MIN, Y_MAX, Z_MAX );
  
    glEnd();
  }
  
  glLineWidth(1.0);

  glBegin(GL_LINES);

  for(int i=0; i<FLOCK_SIZE; i++) {
    double length;
    
    length = vector_length(flock[i].velocity);

    glColor4ubv(flock[i].color);

    glVertex3f(flock[i].position[0] - BOID_SIZE * flock[i].velocity[0]/length,
	       flock[i].position[1] - BOID_SIZE * flock[i].velocity[1]/length,
	       flock[i].position[2] - BOID_SIZE * flock[i].velocity[2]/length );

    glVertex3f(flock[i].position[0] + BOID_SIZE * flock[i].velocity[0]/length,
	       flock[i].position[1] + BOID_SIZE * flock[i].velocity[1]/length,
	       flock[i].position[2] + BOID_SIZE * flock[i].velocity[2]/length );

  }
  
  glEnd();
  glFlush();
  
  SDL_GL_SwapBuffers();
}

int main(int argc, char *argv[])
{
  int       width   = WIDTH;
  int       height  = HEIGHT;

  bool      running = true;
  int       fps     = 0;

  SDL_Event event;
  int now, last, last_fps;

  srand(time(NULL));

  for(int i=0; i<FLOCK_SIZE; i++) {  
    flock[i].color[0] = (int)RAND_RANGE(50, 255);
    flock[i].color[1] = (int)RAND_RANGE(50, 255);
    flock[i].color[2] = (int)RAND_RANGE(50, 255);
    flock[i].color[3] = 220;

    flock[i].position[0] = (double)RAND_RANGE(X_MIN, X_MAX);
    flock[i].position[1] = (double)RAND_RANGE(Y_MIN, Y_MAX);
    flock[i].position[2] = (double)RAND_RANGE(Z_MIN, Z_MAX);

    flock[i].velocity[0] = (double)RAND_RANGE(1.0, MAX_FLOCK_SPEED-.10); 
    flock[i].velocity[1] = (double)RAND_RANGE(1.0, MAX_FLOCK_SPEED-.10); 
    flock[i].velocity[2] = (double)RAND_RANGE(1.0, MAX_FLOCK_SPEED-.10); 
  }

  gfx_init(width, height);

  now = last = last_fps = 0;
  while(running) {

    while( SDL_PollEvent( &event ) ) {	
      switch( event.type ) {
	
      case SDL_KEYDOWN:

	switch( event.key.keysym.sym ) {
	case SDLK_q:
	case SDLK_ESCAPE:
	  running = false;
	  break;
	default:
	  break;
	}

	break;
	
      case SDL_QUIT:
	running = false;
	break;

      default:
	break;

      }  
    }

    flock_tick();
    gfx_render();
    fps++;

    last = now;
    now = SDL_GetTicks();

    if(now - last_fps > 1000) {
      fprintf(stderr, "%d ms,  %d fps\n", now-last, fps);
      last_fps = now;
      fps=0;
    }

    if( now-last < MS_BETWEEN_FRAMES )
      SDL_Delay(MS_BETWEEN_FRAMES - (now-last));    
  }

  exit(0);
}
