
#include "antix.h"
using namespace Antix;

#if GRAPHICS
#include <GLUT/glut.h> // OS X users need <glut/glut.h> instead

int Robot::winsize( 700 );

static double zoom(1.0);
static double panx(0.0);
static double pany(0.0);
static bool zooming(false);
//static int clickx(0), clicky(0);

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
  
  if( (button == GLUT_RIGHT_BUTTON) )
	 {
		zooming = (state == GLUT_DOWN);
		//clickx = x;
		//clicky = y;
	 }
}

static void motion_func( int x, int y) 
{  
  if( zooming )
	 {
		static int lasty(0);		
		double scale = y > lasty ? 0.95 : 1.05;
		glScalef( scale, scale, 1 );
		lasty = y;
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
  zoom = 1.0 / Robot::worldsize;

  // initialize opengl graphics
  glutInit( &argc, argv );
  glutInitWindowSize( winsize, winsize );
  glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA );
  glutCreateWindow( argv[0] ); // program name
  glClearColor( 0.1,0.1,0.1,1 ); // dark grey
  glutDisplayFunc( display_func );
  glutTimerFunc( gui_interval, timer_func, 0 );
  glutMouseFunc( mouse_func );
  glutMotionFunc( motion_func );
  glutIdleFunc( idle_func );
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  glEnable( GL_BLEND );
  glEnableClientState( GL_VERTEX_ARRAY );
	//  glEnableClientState( GL_COLOR_ARRAY );
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  gluOrtho2D( 0,1,0,1 );
  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();
  glScalef( zoom, zoom, 1 ); 
  glPointSize( 2.0 );
}

void Robot::UpdateGui()
{
	glutMainLoop();
}

// render all robots in OpenGL
void Robot::DrawAll()
{		
#if DEBUGVIS
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
	
	// if robots are smaller than 4 pixels across, draw them as points
	if( (radius * (double)winsize/(double)worldsize) < 2.0 )
		{
			const size_t len( population.size() );
			// keep this buffer around between calls for speed
			static std::vector<GLfloat> pts;	
			static std::vector<GLfloat> colors;	
			pts.resize( len * 2 );
			colors.resize( len * 3 );
			
			glVertexPointer( 2, GL_FLOAT, 0, &pts[0] );       

			glEnableClientState( GL_COLOR_ARRAY );
			glColorPointer( 3, GL_FLOAT, 0, &colors[0] );       
			
			for( unsigned int i(0); i<len; ++i )
				{
					pts[2*i+0] = population[i]->pose.x;
					pts[2*i+1] = population[i]->pose.y;

					Home::Color& col = population[i]->home->color;
					colors[3*i+0] = col.r;
					colors[3*i+1] = col.g;
					colors[3*i+2] = col.b;
				}
			
			glDrawArrays( GL_POINTS, 0, len );
			glDisableClientState( GL_COLOR_ARRAY );
		}
	else // more detailed drawing
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

	// pack the puck points into a vertex array for fast rendering
	const size_t len( pucks.size() );
	
	// keep this buffer around between calls for speed
	static std::vector<GLfloat> pts;	
	pts.resize( len * 2 );
	glVertexPointer( 2, GL_FLOAT, 0, &pts[0] );       
	
	for( unsigned int i(0); i<len; ++i )
		{
			pts[2*i+0] = pucks[i].x;
			pts[2*i+1] = pucks[i].y;
		}

	glDrawArrays( GL_POINTS, 0, len );

	glPointSize( 2.0 );
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
	
	// draw a circular body
	GlDrawCircle( 0,0, radius, 12 );
	
	// draw a nose indicating forward direction
	glBegin(GL_LINES);
	glVertex2f( 0, 0 );
	glVertex2f( Robot::radius, 0 );
	glEnd();

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

#if DEBUGVIS

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
		
		ep = Robot::radius;

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
#endif
}

#endif // GRAPHICS
