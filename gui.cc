
#include "antix.h"
using namespace Antix;

#if GRAPHICS
#include <GLUT/glut.h> // OS X users need <glut/glut.h> instead

int Robot::winsize( 700 );

// GLUT callback functions ---------------------------------------------------

// update the world - this is called whenever GLUT runs out of events
// to process
static void idle_func( void )
{
  Robot::UpdateAll();
}

static void timer_func( int dummy )
{
  glutPostRedisplay(); // force redraw
}

// draw the world - this is called whenever the window needs redrawn
static void display_func( void ) 
{  
  Robot::winsize = glutGet( GLUT_WINDOW_WIDTH );
  glClear( GL_COLOR_BUFFER_BIT );  
  Robot::DrawAll();
  glutSwapBuffers();
	
  // run this function again in about 50 msec
  glutTimerFunc( Robot::gui_interval, timer_func, 0 );
}

static void mouse_func(int button, int state, int x, int y) 
{  
  if( (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN ) )
	 {
		Robot::paused = !Robot::paused;
	 }
}


// utility
void GlDrawCircle( double x, double y, double r, double count )
{
	glBegin(GL_LINE_LOOP);
	for( float a=0; a<(M_PI*2.0); a+=M_PI/count )
		glVertex2f( x + sin(a) * r, y + cos(a) * r );
	glEnd();
}


//
// Robot static member methods ---------------------------------------------

void Robot::InitGraphics( int argc, char* argv[] )
{
  // initialize opengl graphics
  glutInit( &argc, argv );
  glutInitWindowSize( winsize, winsize );
  glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA );
  glutCreateWindow( argv[0] ); // program name
  glClearColor( 0.1,0.1,0.1,1 ); // dark grey
  glutDisplayFunc( display_func );
  glutTimerFunc( gui_interval, timer_func, 0 );
  glutMouseFunc( mouse_func );
  glutIdleFunc( idle_func );
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  glEnable( GL_BLEND );
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  gluOrtho2D( 0,1,0,1 );
  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();
  glScalef( 1.0/Robot::worldsize, 1.0/Robot::worldsize, 1 ); 
	glPointSize( 2.0 );
}

void Robot::UpdateGui()
{
	glutMainLoop();
}

// render all robots in OpenGL
void Robot::DrawAll()
{		
#if 1
	// draw the matrix 
	double d = worldsize / (double)(matrixwidth);

	glColor3f( 0.15,0.15,0.15 );

	glBegin( GL_LINES );
	for( unsigned int x(0); x < matrixwidth; x++ )
	  {
		 glVertex2f( x*d, 0 );
		 glVertex2f( x*d, worldsize );
	  }
	for( unsigned int y(0); y < matrixwidth; y++ )
	  {
		 glVertex2f( 0, y*d );
		 glVertex2f( worldsize, y*d );
	  }
	glEnd();
#endif

	FOR_EACH( r, population )
		(*r)->Draw();
	
	FOR_EACH( it, homes )
		{
			Home* h = *it;
			
			glColor3f( h->color.r, 
								 h->color.g,
								 h->color.b );
			
			GlDrawCircle( h->x, h->y, h->r, 12 );
			GlDrawCircle( h->x+worldsize, h->y, h->r, 12 );
			GlDrawCircle( h->x-worldsize, h->y, h->r, 12 );
			GlDrawCircle( h->x, h->y+worldsize, h->r, 12 );
			GlDrawCircle( h->x, h->y-worldsize, h->r, 12 );
		}
	
	glColor3f( 1,1,1 ); // green

	glPointSize( 1.0 );
	glBegin( GL_POINTS );
	FOR_EACH( p, pucks )
		glVertex2f( (*p)->x, (*p)->y );
	glEnd();
	glPointSize( 2.0 );

#if 0
	// draw the sorted lists
	glColor3f( 1,0,0 );
	glBegin( GL_LINE_STRIP );
	for( Robot* r = Robot::leftmost; r; r=r->right )
	  {
		 //printf( "%.2f %.2f\n", r->pose.x, r->pose.y );
		 glVertex2f( r->pose.x, r->pose.y );
	  }
	glEnd();

	// draw the sorted lists
	glColor3f( 0,1,0 );
	glBegin( GL_LINE_STRIP );
	for( Robot* r = Robot::downmost; r; r=r->up )
	  {
		 //printf( "%.2f %.2f\n", r->pose.x, r->pose.y );
		 glVertex2f( r->pose.x, r->pose.y );
	  }
	glEnd();
#endif

}

// draw a robot
void Robot::Draw()
{
  glPushMatrix();

	// shift into this robot's local coordinate frame
  glTranslatef( pose.x, pose.y, 0 );
  glRotatef( rtod(pose.a), 0,0,1 );
  
	glColor3f( home->color.r, home->color.g, home->color.b ); 
	
	double radius = Robot::radius;
	
	// if robots are smaller than 4 pixels across, draw them as points
	if( (radius * (double)winsize/(double)worldsize) < 2.0 )
	  {
		 glBegin( GL_POINTS );
		 glVertex2f( 0,0 );
		 glEnd();
	  }
	else
	  {
		 // draw a circular body
			GlDrawCircle( 0,0, radius, 12 );

		 // draw a nose indicating forward direction
		 glBegin(GL_LINES);
		 glVertex2f( 0, 0 );
		 glVertex2f( Robot::radius, 0 );
		 glEnd();
	  }

  if( Robot::show_data )
	 {
		glColor3f( 0,0,1 ); // blue
		
		FOR_EACH( it, see_robots )
		  {
				float dx = it->range * cos(it->bearing);
				float dy = it->range * sin(it->bearing);
				
				glBegin( GL_LINES );
				glVertex2f( 0,0 );
				glVertex2f( dx, dy );
				glEnd();
		  }
		

#if 1		
		glColor3f( 0.3,0.8,0.3 ); // light green
		FOR_EACH( it, see_pucks )
		  {
				float dx = it->range * cos(it->bearing);
				float dy = it->range * sin(it->bearing);
				
				glBegin( GL_LINES );
				glVertex2f( 0,0 );
				glVertex2f( dx, dy );
				glEnd();
		  }
#endif
		
		glColor3f( 0.4,0.4,0.4 ); // grey

		// draw the sensor FOV
		glBegin(GL_LINE_LOOP);
		
		glVertex2f( 0, 0 );
		
		double right = -fov/2.0;
		double left = +fov/2.0;// + M_PI;
		double incr = fov/32.0;
		for( float a=right; a<left; a+=incr)
		  glVertex2f( cos(a) * range, 
						  sin(a) * range );

		glVertex2f( cos(left) * range, 
						sin(left) * range );
		
		glEnd();		

		
	 }
	
	// shift out of local coordinate frame
  glPopMatrix();

  if( first == this )
	 {
		
		glColor3f( 1,1,0 );

		//		GlDrawCircle( pose.x, pose.y, Robot::range, 32 );

		double ep = Robot::range;

		glBegin( GL_LINE_LOOP );
		glVertex2f( pose.x+ep, pose.y+ep );
		glVertex2f( pose.x-ep, pose.y+ep );
		glVertex2f( pose.x-ep, pose.y-ep );
		glVertex2f( pose.x+ep, pose.y-ep );
		glEnd();  

		ep = 0.02;

		glBegin( GL_LINE_LOOP );
		glVertex2f( pose.x+ep, pose.y+ep );
		glVertex2f( pose.x-ep, pose.y+ep );
		glVertex2f( pose.x-ep, pose.y-ep );
		glVertex2f( pose.x+ep, pose.y-ep );
		glEnd();  
		
		glColor3f( 1,0,1 );
		FOR_EACH( it, neighbors )
		  {
			 glBegin( GL_LINE_LOOP );
			 glVertex2f( (*it)->pose.x+ep, (*it)->pose.y+ep );
			 glVertex2f( (*it)->pose.x-ep, (*it)->pose.y+ep );
			 glVertex2f( (*it)->pose.x-ep, (*it)->pose.y-ep );
			 glVertex2f( (*it)->pose.x+ep, (*it)->pose.y-ep );
			 glEnd();  
		  }

		ep = 0.01;
		glColor3f( 0,1,1 );
		FOR_EACH( it, neighbor_pucks )
		  {
			 glBegin( GL_LINE_LOOP );
			 glVertex2f( (*it)->x+ep, (*it)->y+ep );
			 glVertex2f( (*it)->x-ep, (*it)->y+ep );
			 glVertex2f( (*it)->x-ep, (*it)->y-ep );
			 glVertex2f( (*it)->x+ep, (*it)->y-ep );
			 glEnd();  
		  }
		
		glColor3f( 0.5,0.1,0.1 );

		double dx = worldsize / (double)matrixwidth;
		
		FOR_EACH( it, neighbor_cells )
		  {
				unsigned int index = *it;
				unsigned int y = index / matrixwidth;
				unsigned int x = index % matrixwidth;

				GlDrawCircle( x*dx+dx/2.0, y*dx+dx/2.0, dx/2.0, 8 );
		  }		
	 }
}

#endif // GRAPHICS
