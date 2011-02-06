/****
   antix.cc
   version 1
   Richard Vaughan  
	 Clone this package from git://github.com/rtv/Antix.git
****/

#include <assert.h>
#include <unistd.h>
#include <algorithm>
#include "antix.h"
using namespace Antix;


// initialize static members
bool Robot::paused( false );
bool Robot::show_data( false );
double Robot::fov(  dtor(90.0) );
double Robot::pickup_range( Robot::range/5.0 );
double Robot::radius(0.01);
double Robot::range( 0.1 );
double Robot::worldsize(1.0);
std::vector<Home*> Robot::homes;
std::vector<Robot*> Robot::population;
std::vector<Robot::Puck> Robot::pucks;
uint64_t Robot::updates(0);
uint64_t Robot::updates_max( 0.0 ); 
unsigned int Robot::home_count(1);
unsigned int Robot::home_population( 20 );
unsigned int Robot::puck_count(100);
unsigned int Robot::sleep_msec( 10 );
std::vector<Robot::MatrixCell> Robot::matrix;

unsigned int Robot::gui_interval(100);
Robot* Robot::first(NULL);

unsigned int Robot::matrixwidth( Robot::worldsize / Robot::range );

const char usage[] = "Antix understands these command line arguments:\n"
  "  -? : Prints this helpful message.\n"
  "  -a <int> : sets the number of pucks in the world.\n"
  "  -c <int> : sets the number of pixels in the robots' sensor.\n"
  "  -d  Enables drawing the sensor field of view. Speeds things up a bit.\n"
  "  -f <float> : sets the sensor field of view angle in degrees.\n"
  "  -g <int> : sets the interval between GUI redraws in milliseconds.\n"
  "  -p <int> : set the size of the robot population.\n"
  "  -r <float> : sets the sensor field of view range.\n"
  "  -s <float> : sets the side length of the (square) world.\n"
  "  -u <int> : sets the number of updates to run before quitting.\n"
  "  -w <int> : sets the initial size of the window, in pixels.\n"
  "  -z <int> : sets the number of milliseconds to sleep between updates.\n";

Home::Home( const Color& color, double x, double y, double r ) 
	: color(color), x(x), y(y), r(r) 
{
	Robot::homes.push_back(this);
}


Robot::Robot( Home* home,
				  const Pose& pose )
  : index(0),
	 home(home),
	 pose(pose),
	 speed(),
	 see_robots(),
	 see_pucks(),
	 puck_held(NULL)
{
  // add myself to the static vector of all robots
  population.push_back( this );
  
  if( ! first )
	 first = this;
}

void Robot::Init( int argc, char** argv )
{
  // seed the random number generator with the current time
  srand48(time(NULL));
  //srand48(0); // for debugging - start the same every time
	
  // parse arguments to configure Robot static members
	int c;
	while( ( c = getopt( argc, argv, "?dh:a:p:s:f:g:r:c:u:z:w:")) != -1 )
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
				home_population = atoi( optarg );
				printf( "[Antix] home_population: %d\n", home_population );
				break;
				
			case 's': 
				worldsize = atof( optarg );
				printf( "[Antix] worldsize: %.2f\n", worldsize );
				break;
				
			case 'f': 
				fov = dtor(atof( optarg )); // degrees to radians
				printf( "[Antix] fov: %.2f\n", fov );
				break;
				
      case 'g':
				gui_interval = atol( optarg );
				printf( "[Antix] gui_interval: %lu\n", (long unsigned)gui_interval );
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

	Robot::matrixwidth = floor( Robot::worldsize / Robot::range );
	Robot::matrix.resize( Robot::matrixwidth * Robot::matrixwidth );
	
	pucks.resize(puck_count);
	FOR_EACH( p, pucks )
		{
			p->x = drand48() * worldsize;
			p->y = drand48() * worldsize;
			matrix[Cell(p->x,p->y)].pucks.insert( &(*p) );
		}
	
	
#if GRAPHICS
	InitGraphics( argc, argv );
#endif // GRAPHICS
}

void Robot::TestRobotsInCell( const MatrixCell& cell )
{
	FOR_EACH( it, cell.robots )
		{
      Robot* other = *it;
			
      // discard if it's the same robot
      if( other == this )
				continue;
		
#if DEBUGVIS
		neighbors.push_back( other );
#endif
			
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
																			other->Holding() ) );						
    }
}	

void Robot::TestPucksInCell( const MatrixCell& cell )
{

  FOR_EACH( it, cell.pucks )
    {      
      Puck* puck = *it;

#if DEBUGVIS
		neighbor_pucks.push_back( puck );
#endif
      // discard if it's out of range. We put off computing the
      // hypotenuse as long as we can, as it's relatively expensive.
		
      const double dx( WrapDistance( puck->x - pose.x ) );
			if( fabs(dx) > Robot::range )
				continue; // out of range
			
      const double dy( WrapDistance( puck->y - pose.y ) );		
			if( fabs(dy) > Robot::range )
				continue; // out of range
			
      const double range( hypot( dx, dy ) );
      if( range > Robot::range ) 
				continue; 
			
      // discard if it's out of field of view 
      const double absolute_heading( atan2( dy, dx ) );
      const double relative_heading( AngleNormalize((absolute_heading - pose.a)));
      if( fabs(relative_heading) > fov/2.0   ) 
				continue; 
		
			// passes all the tests, so we record a puck detection in the
			// vector
			see_pucks.push_back( SeePuck( puck, range, 
																		relative_heading,
																		puck->held));
		}		
}

void Robot::UpdateSensors()
{
  see_robots.clear();
  see_pucks.clear();
  
  // note: the following two large sensing operations could safely be
  // done in parallel since they do not modify any common data

#if DEBUGVIS
  // debug visualization  
     neighbors.clear();
     neighbor_pucks.clear();
     neighbor_cells.clear();
#endif  
  
  int x( Cell( pose.x ));
  int y( Cell( pose.y ));
  
  // check the 3x3 cells around my position
  UpdateSensorsCell( x-1, y-1 );
  UpdateSensorsCell( x+0, y-1 );
  UpdateSensorsCell( x+1, y-1 );
  UpdateSensorsCell( x-1, y+0 );
  UpdateSensorsCell( x+0, y+0 );
  UpdateSensorsCell( x+1, y+0 );
  UpdateSensorsCell( x-1, y+1 );
  UpdateSensorsCell( x+0, y+1 );
  UpdateSensorsCell( x+1, y+1 );
}

bool Robot::Pickup()
{
  if( ! puck_held ) 
		FOR_EACH( it, see_pucks )
			{
				// is the puck close enough and is it not held already?
				if( (it->range < pickup_range) && !it->puck->held)
					{				
						// pick it up
						puck_held = it->puck;
						puck_held->held = true;
						return true;
					}		  		  
			}
	
	// already holding or nothing close enough
  return false; 
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
    
  unsigned int newindex = Cell( pose.x, pose.y );
 
 // if we're carrying a puck, update it's position
  if( puck_held )
	 {
		puck_held->x = pose.x;
		puck_held->y = pose.y;
	 }
	
  if( newindex != index )
		{
			matrix[index].robots.erase( this );
			matrix[newindex].robots.insert( this );		
			
			if( puck_held )
				{
					matrix[index].pucks.erase( puck_held );
					matrix[newindex].pucks.insert( puck_held );		
				}
			
			index = newindex;
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

void Robot::Run()
{
#if GRAPHICS
  UpdateGui();
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
