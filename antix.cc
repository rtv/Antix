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

unsigned int Robot::mbits( 3 );

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
Robot* Robot::leftmost(NULL);
Robot* Robot::downmost(NULL);
uint64_t Robot::updates(0);
uint64_t Robot::updates_max( 0.0 ); 
unsigned int Robot::home_count(1);
unsigned int Robot::home_population( 20 );
unsigned int Robot::puck_count(100);
unsigned int Robot::sleep_msec( 10 );
std::vector<std::set<Robot*> > Robot::matrix( (1<<Robot::mbits) * (1<<Robot::mbits) );
std::vector<Robot*> RobotsByX;
std::vector<Robot*> RobotsByY;

Robot* Robot::first(NULL);

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


Home::Home( const Color& color, double x, double y, double r ) 
	: color(color), x(x), y(y), r(r) 
{
	Robot::homes.push_back(this);
}


Robot::Robot( Home* home,
				  const Pose& pose )
  : home(home),
	 pose(pose),
	 speed(),
	 see_robots(),
	 see_pucks(),
	 puck_held(NULL),
	 index(0)
{
  // add myself to the static vector of all robots
  population.push_back( this );
  
  // add myself to the left of the X list
  left = NULL;
  right = Robot::leftmost;
  if( right ) right->left = this;
  Robot::leftmost = this;
  
  // and the downmost of the Y list
  down = NULL;
  up = Robot::downmost;
  if( up) up->down = this;
  Robot::downmost = this;

  if( ! first )
	 first = this;
}

void Robot::Init( int argc, char** argv )
{
  // seed the random number generator with the current time
  //srand48(time(NULL));
  srand48(0); // for debugging - start the same every time
	
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
	InitGraphics( argc, argv );
#endif // GRAPHICS
}

inline unsigned int Cell( double x, double y )
{
  double d = Robot::worldsize / (double)(1<<Robot::mbits);
  
  unsigned int cx( floor( x / d ));
  unsigned int cy( floor( y / d ));
  
  const unsigned int i(cx + (cy << Robot::mbits) );
  
  //printf( "(%.2f %.2f) [%u %u]=> %u\n", x, y, cx, cy, i );	 

  assert( i < ((1<<Robot::mbits) * (1<<Robot::mbits)));

  return i;
}


void Robot::UpdateSensors()
{
  see_robots.clear();
  see_pucks.clear();
  
	// note: the following two large Fsensing operations could safely be
	// done in parallel since they do not modify any common data

#if 0
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
																			other->Holding() ) );			
    }	
#else  
  
  neighbors.clear();
 // check every robot in the world to see if it is detected
  FOR_EACH( it, matrix[Cell(pose.x, pose.y)] )
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
												  other->Holding() ) );			

		neighbors.push_back( other );
    }	
#endif      
  
  // next fill the puck sensor
  // check every puck in the world to see if it is detected
#if 1
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
#endif
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
  
  if( newindex != index )
	 {
		matrix[index].erase( this );
		matrix[newindex].insert( this );		
		index = newindex;
	 }
  
  // if we're carrying a puck, update it's position
  if( puck_held )
	 {
		puck_held->x = pose.x;
		puck_held->y = pose.y;
	 }
}


// in-place insertion sort
inline void Robot::SortX()
{  
  // iterate along the list to the right
  for( Robot* a(Robot::leftmost); a->right;  )
	 {
		Robot* b = a->right;
		Robot* c = a;
		
		// 		printf( "before shift\n" );
		// 		// iterate along the list to the right
		// 		for( Robot* z(Robot::leftmost); z; z = z->right )
		// 		  {
		// 			 printf( "X:%.2f %p (l:%p r:%p) %s%s%s\n", z->pose.x, z, z->left, z->right, 
		// 						z == a ? "a" : "", 
		// 						z == b ? "b" : "",
		// 						z == c ? "c" : "" );
		// 		  }		
		
		// slide left until we find the right place for b 
		while( c && (c->pose.x > b->pose.x) )
		  c = c->left;			 
		
		if( c != a ) // ie. b is not in the correct place
		  {			 
			 // remove b from current slot
			 a->right = b->right;			 
			 if( b->right )
				b->right->left = a;
			 
			 // insert it to the right of c
			 b->left = c;
			 
			 if( c )
				{
				  b->right = c->right;
				  b->right->left = b;
				  c->right = b;				  
				}			
			 else
				{
				  b->right = leftmost;				
				  leftmost->left = b;
				  leftmost = b;
				}			 
		  }
		else
		  a = a->right;
	 }
}

// in-place insertion sort
inline void Robot::SortY()
{  
  // iterate along the list to the up
  for( Robot* a(Robot::downmost); a->up;  )
	 {
		Robot* b = a->up;
		Robot* c = a;
		
		// 		printf( "before shift\n" );
		// 		// iterate along the list to the up
		// 		for( Robot* z(Robot::downmost); z; z = z->up )
		// 		  {
		// 			 printf( "X:%.2f %p (l:%p r:%p) %s%s%s\n", z->pose.x, z, z->down, z->up, 
		// 						z == a ? "a" : "", 
		// 						z == b ? "b" : "",
		// 						z == c ? "c" : "" );
		// 		  }		
		
		// slide down until we find the up place for b 
		while( c && (c->pose.y > b->pose.y) )
		  c = c->down;			 
		
		if( c != a ) // ie. b is not in the correct place
		  {			 
			 // remove b from current slot
			 a->up = b->up;			 
			 if( b->up )
				b->up->down = a;
			 
			 // insert it to the up of c
			 b->down = c;
			 
			 if( c )
				{
				  b->up = c->up;
				  b->up->down = b;
				  c->up = b;				  
				}			
			 else
				{
				  b->up = downmost;				
				  downmost->down = b;
				  downmost = b;
				}			 
		  }
		else
		  a = a->up;
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

			SortX();
			SortY();

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
