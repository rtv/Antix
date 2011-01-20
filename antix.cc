/****
   antix.cc
   version 1
   Richard Vaughan  
	 Clone this package from git://github.com/rtv/Antix.git
****/

#include <assert.h>
#include <unistd.h>

#include "antix.h"
using namespace Antix;

const char* PROGNAME = "antix";

#if GRAPHICS
#include <GLUT/glut.h> // OS X users need <glut/glut.h> instead
#endif

// initialize static members
bool Robot::paused( false );
bool Robot::show_data( false );
double Robot::fov(  dtor(90.0) );
double Robot::pickup_range( Robot::range/5.0 );
double Robot::radius(0.01);
double Robot::range( 0.1 );
double Robot::worldsize(1.0);
int Robot::winsize( 600 );
std::vector<Home*> Robot::homes;
std::vector<Robot*> Robot::population;
std::vector<Robot::Puck> Robot::pucks;
uint64_t Robot::updates(0);
uint64_t Robot::updates_max( 0.0 ); 
unsigned int Robot::home_count(1);
unsigned int Robot::population_size( 20 );
unsigned int Robot::puck_count(100);
unsigned int Robot::sleep_msec( 10 );

const char usage[] = "Antix understands these command line arguments:\n"
	"  -? : Prints this helpful message.\n"
	"  -c <int> : sets the number of pixels in the robots' sensor.\n"
	"  -d  Enables drawing the sensor field of view. Speeds things up a bit.\n"
	"  -f <float> : sets the sensor field of view angle in degrees.\n"
	"  -p <int> : set the size of the robot population.\n"
	"  -r <float> : sets the sensor field of view range.\n"
	"  -s <float> : sets the side length of the (square) world.\n"
	"  -u <int> : sets the number of updates to run before quitting.\n"
	"  -w <int> : sets the initial size of the window, in pixels.\n"
	"  -z <int> : sets the number of milliseconds to sleep between updates.\n";

#if GRAPHICS
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
  glutTimerFunc( 20, timer_func, 0 );
}

static void mouse_func(int button, int state, int x, int y) 
{  
  if( (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN ) )
	 {
		Robot::paused = !Robot::paused;
	 }
}


void GlDrawCircle( double x, double y, double r, double count )
{
	glBegin(GL_LINE_LOOP);
	for( float a=0; a<(M_PI*2.0); a+=M_PI/count )
		glVertex2f( x + sin(a) * r, y + cos(a) * r );
	glEnd();
}

// render all robots in OpenGL
void Robot::DrawAll()
{		
	FOR_EACH( r, population )
		(*r)->Draw();
	
	FOR_EACH( it, homes )
		{
			Home* h = *it;
			
			glColor3f( h->color.r, 
								 h->color.g,
								 h->color.b );
			
			GlDrawCircle( h->x, h->y, h->r, 16 );
			GlDrawCircle( h->x+worldsize, h->y, h->r, 16 );
			GlDrawCircle( h->x-worldsize, h->y, h->r, 16 );
			GlDrawCircle( h->x, h->y+worldsize, h->r, 16 );
			GlDrawCircle( h->x, h->y-worldsize, h->r, 16 );
		}
	
	glColor3f( 1,1,1 ); // green
	glBegin( GL_POINTS );
	FOR_EACH( p, pucks )
		glVertex2f( p->x, p->y );
	glEnd();
}

#endif // GRAPHICS

Robot::Robot( Home* home,
							const Pose& pose )
  : home(home),
	  pose(pose),
		speed(),
		see_robots(),
		see_pucks(),
		puck_held(NULL)
{
  // add myself to the static vector of all robots
  population.push_back( this );
}

void Robot::Init( int argc, char** argv )
{
  // seed the random number generator with the current time
  srand48(time(NULL));
	
  // parse arguments to configure Robot static members
	int c;
	while( ( c = getopt( argc, argv, "?dh:a:p:s:f:r:c:u:z:w:")) != -1 )
		switch( c )
			{
			case 'h':
			  home_count = atoi( optarg );
			  printf( "[Antix] home count: %d\n", home_count );
			  break;

			case 'a':
			  puck_count = atoi( optarg );
			  printf( "[Antix] puck count: %d\n", puck_count );
			  break;
			  
			case 'p': 
				population_size = atoi( optarg );
				printf( "[Antix] population_size: %d\n", population_size );
				break;
				
			case 's': 
				worldsize = atof( optarg );
				printf( "[Antix] worldsize: %.2f\n", worldsize );
				break;
				
			case 'f': 
				fov = dtor(atof( optarg )); // degrees to radians
				printf( "[Antix] fov: %.2f\n", fov );
				break;
				
			case 'r': 
				range = atof( optarg );
				printf( "[Antix] range: %.2f\n", range );
				break;
								
      case 'u':
				updates_max = atol( optarg );
				printf( "[Antix] updates_max: %lu\n", (long unsigned)updates_max );
				break;
				
			case 'z':
				sleep_msec = atoi( optarg );
				printf( "[Antix] sleep_msec: %d\n", sleep_msec );
				break;
				
#if GRAPHICS
			case 'w': winsize = atoi( optarg );
				printf( "[Antix] winsize: %d\n", winsize );
				break;

			case 'd': show_data=true;
			  puts( "[Antix] show data" );
			  break;
#endif			
			case '?':
			  puts( usage );
			  exit(0); // ok
			  break;

			default:
				fprintf( stderr, "[Antix] Option parse error.\n" );
				puts( usage );
				exit(-1); // error
			}
	
	for( unsigned int i=0; i<puck_count; i++ )
	  pucks.push_back( Puck() );	
	
#if GRAPHICS
  // initialize opengl graphics
  glutInit( &argc, argv );
  glutInitWindowSize( winsize, winsize );
  glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA );
  glutCreateWindow( PROGNAME );
  //glClearColor( 0.8,0.8,1.0,1.0 ); // pale blue
  // glClearColor( 0,0,0,1 ); // black
  //glClearColor( 0.2,0,0,1 ); // dark red
  glClearColor( 0.1,0.1,0.1,1 ); // dark grey
  glutDisplayFunc( display_func );
  glutTimerFunc( 50, timer_func, 0 );
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
  
//   // define a display list for a robot body
//   double h = 0.01;
//   double w = 0.01;

	glPointSize( 4.0 );

//   displaylist = glGenLists(1);
//   glNewList( displaylist, GL_COMPILE );

//   glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

//   glBegin( GL_POLYGON );
//   glVertex2f( h/2.0, 0 );
//   glVertex2f( -h/2.0,  w/2.0 );
//   glVertex2f( -h/2.0, -w/2.0 );
//   glEnd();

//   glEndList();

#endif // GRAPHICS
}

void Robot::UpdateSensors()
{
  see_robots.clear();
  see_pucks.clear();
  
	// note: the following two large Fsensing operations could safely be
	// done in parallel since they do not modify any common data
	
  // first fill the robot sensor
  // check every robot in the world to see if it is detected
  FOR_EACH( it, population )
    {
      Robot* other = *it;
			
      // discard if it's the same robot
      if( other == this )
				continue;
			
      // discard if it's out of range. We put off computing the
      // hypotenuse as long as we can, as it's relatively expensive.
		
      double dx( WrapDistance( other->pose.x - pose.x ) );
			if( fabs(dx) > Robot::range )
				continue; // out of range
			
      double dy( WrapDistance( other->pose.y - pose.y ) );		
			if( fabs(dy) > Robot::range )
				continue; // out of range
			
      double range = hypot( dx, dy );
      if( range > Robot::range ) 
				continue; 
			
      // discard if it's out of field of view 
      double absolute_heading = atan2( dy, dx );
      double relative_heading = AngleNormalize((absolute_heading - pose.a) );
      if( fabs(relative_heading) > fov/2.0   ) 
				continue; 
			
			see_robots.push_back( SeeRobot( other->home,
																			other->pose, 
																			other->speed, 
																			range, 
																			relative_heading,
																			false ) );			
    }	
	
  // next fill the puck sensor
  // check every puck in the world to see if it is detected
  FOR_EACH( it, pucks )
    {      
      // discard if it's out of range. We put off computing the
      // hypotenuse as long as we can, as it's relatively expensive.
		
      double dx( WrapDistance( it->x - pose.x ) );
		if( fabs(dx) > Robot::range )
		  continue; // out of range
		
      double dy( WrapDistance( it->y - pose.y ) );		
		if( fabs(dy) > Robot::range )
		  continue; // out of range
		
      double range = hypot( dx, dy );
      if( range > Robot::range ) 
		  continue; 
		
      // discard if it's out of field of view 
      double absolute_heading = atan2( dy, dx );
      double relative_heading = AngleNormalize((absolute_heading - pose.a) );
      if( fabs(relative_heading) > fov/2.0   ) 
		  continue; 
		
			// passes all the tests, so we record a puck detection in the
			// vector
			see_pucks.push_back( SeePuck( &(*it), range, 
																		relative_heading,
																		it->held));
		}		
}

bool Robot::Pickup()
{
  if( ! puck_held ) 
		FOR_EACH( it, see_pucks )
			{
				if( (it->range < pickup_range) && !it->puck->held)
					{				
						puck_held = it->puck;
						puck_held->held = true;
						return true;
					}		  		  
			}
	
  return false; // already holding or nothing close enough
}

bool Robot::Holding()
{
  return (bool)puck_held;
}

bool Robot::Drop()
{
  if( puck_held )
	 {
		puck_held->held = false;
		puck_held = NULL;		
		return true; // dropped successfully
	 }
  return false; // nothing to drop  
}

void Robot::UpdatePose()
{
  // move according to the current speed 
  double dx = speed.v * cos(pose.a);
  double dy = speed.v * sin(pose.a);; 
  double da = speed.w;
  
  pose.x = DistanceNormalize( pose.x + dx );
  pose.y = DistanceNormalize( pose.y + dy );
  pose.a = AngleNormalize( pose.a + da );

  // if we're carrying a puck, update it's position
  if( puck_held )
	 {
		puck_held->x = pose.x;
		puck_held->y = pose.y;
	 }
}

void Robot::UpdateAll()
{
  // if we've done enough updates, exit the program
  if( updates_max > 0 && updates > updates_max )
	 exit(1);
  
  if( ! Robot::paused )
		{
			FOR_EACH( r, population )
				(*r)->UpdatePose();

			FOR_EACH( r, population )
				(*r)->UpdateSensors();

			FOR_EACH( r, population )
				(*r)->Controller();
		}

  ++updates;
  
  // possibly snooze to save CPU and slow things down
  if( sleep_msec > 0 )
	 usleep( sleep_msec * 1e3 );
}

// draw a robot
void Robot::Draw()
{
#if GRAPHICS
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
		 glBegin(GL_LINE_LOOP);
		 for( float a=0; a<(M_PI*2.0); a+=M_PI/16 )
			glVertex2f( sin(a) * radius, 
							cos(a) * radius );
		 glEnd();
		 
		 // draw a nose indicating forward direction
		 glBegin(GL_LINES);
		 glVertex2f( 0, 0 );
		 glVertex2f( Robot::radius, 0 );
		 glEnd();
	  }

  if( Robot::show_data )
	 {
		glColor3f( 1,0,0 ); // red
		
		FOR_EACH( it, see_robots )
		  {
				float dx = it->range * cos(it->bearing);
				float dy = it->range * sin(it->bearing);
				
				glBegin( GL_LINES );
				glVertex2f( 0,0 );
				glVertex2f( dx, dy );
				glEnd();
		  }
		
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
#endif // GRAPHICS
}


void Robot::Run()
{
#if GRAPHICS
  glutMainLoop();
#else
  while( 1 )
    UpdateAll();
#endif
}

// wrap around torus
double Robot::WrapDistance( double d )
{
  const double halfworld( worldsize * 0.5 );
  
  if( d > halfworld )
	 d -= worldsize;
  else if( d < -halfworld )
	 d += worldsize;

  return d;
}

/** Normalize a length to within 0 to worldsize. */
double Robot::DistanceNormalize( double d )
{
	while( d < 0 ) d += worldsize;
	while( d > worldsize ) d -= worldsize;
	return d; 
} 

/** Normalize an angle to within +/_ M_PI. */
double Robot::AngleNormalize( double a )
{
	while( a < -M_PI ) a += 2.0*M_PI;
	while( a >  M_PI ) a -= 2.0*M_PI;	 
	return a;
}	 
