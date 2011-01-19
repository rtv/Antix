/*****
		controller.cc
		version 1
		Copyright Richard Vaughan, 2009.09.09
****/

#include "antix.h"
using namespace Uni;

Home::Color colors[] = { Home::Color(1,0,0), 
												 Home::Color(0,0.5,0), // darker green 
												 Home::Color(0,0,1), 
												 Home::Color(1,1,0), 
												 Home::Color(1,0,1), 
												 Home::Color(0,1,1), 
												 Home::Color(1,0,1) };

size_t color_count = 7;

// this is the robot controller code
class Swarmer : public Robot
{
public:
  
  static bool invert;
  
  Swarmer( Home* h ) : Robot( h, Pose::Random() )
  {}
  
  // must implement this method. Examine the pixels vector and set the
  // speed sensibly.
  virtual void Controller()
  {
	 speed.v = 0.005;   // constant forward speed 
	 speed.w = 0.0;     // no turning. we may change this below
	 
	 double halfworld = Robot::worldsize * 0.5;
	 
	 double dx = home->x - pose.x;
	 
	 // wrap around torus
	 if( dx > halfworld )
		 dx -= worldsize;
	 else if( dx < -halfworld )
		 dx += worldsize;
	 
	 double dy = home->y - pose.y;
	 
		// wrap around torus
		if( dy > halfworld )
		  dy -= worldsize;
		else if( dy < -halfworld )
		  dy += worldsize;

	 double angle_home = atan2( dy, dx );
	 if( normalize( pose.a - angle_home ) < 0 )
		 speed.w = 0.05;
	 else
		 speed.w = -0.05;


	 if( hypot( dx, dy ) < home->r )
		 speed.v = 0.002;

	 return;

	 // steer away from the closest roboot
	 double dist = Robot::range; // max sensor range
	 double bearing = 0.0;
	 
	 for( std::vector<SeeRobot>::const_iterator it = see_robots.begin();
				it != see_robots.end();
				++it )
		 {
			 if( it->range < dist )
				 {
					 dist = it->range;
					 bearing =it->bearing;
					 
					 speed.w = (bearing < 0.0) ? 0.03 : -0.03;
					 
					 if( invert )
						 speed.w *= -1.0; // invert turn direction
				 }		 
		 }
	}
};

// static members
bool Swarmer::invert( false );

int main( int argc, char* argv[] )
{
	// configure global robot settings
	Robot::Init( argc, argv );
	
  // parse remaining cmdline arguments to configure swarmer
	int c;
	while( ( c = getopt( argc, argv, "i")) != -1 )
	 	switch( c )
	 		{
	 		case 'i': Swarmer::invert = true;
	 			break;				
			}
	
	for( unsigned int i=0; i<Robot::home_count; i++ )
		{
			Home* h = new Home( i < color_count ? colors[i] : Home::Color::Random(), 
													drand48(),
													drand48(),													
													Robot::worldsize / 10.0 );
			
			Robot::homes.insert(h);

			for( unsigned int i=0; i<Robot::population_size; i++ )
				new Swarmer( h );
		}		
	// and start the simulation running
	Robot::Run();
	
	// we'll probably never get here, but this keeps the compiler happy.
	return 0;
 }
