/*****
		controller.cc
		version 1
		Copyright Richard Vaughan, 2009.09.09
****/

#include <math.h>
#include "antix.h"
using namespace Antix;

Home::Color colors[] = { Home::Color(1,0,0), 
												 Home::Color(0,0.5,0), // darker green 
												 Home::Color(0.3,0.3,1),  // lighter blue
												 Home::Color(1,1,0), 
												 Home::Color(1,0,1), 
												 Home::Color(0,1,1), 
												 Home::Color(1,0,1) };

size_t color_count = 7;

// this is the robot controller code
class Forager : public Robot
{
public:  
  double lastx, lasty;  
  
  Forager( Home* h ) 
		: Robot( h, Pose::Random() ), 
			lastx(home->x), // initial search location is close to my home
			lasty(home->y)
  {}
  
  // must implement this method. Examine the pixels vector and set the
  // speed sensibly.
  virtual void Controller()
  {		
		double heading_error(0.0);
		
		// distance and angle to home
		double dx( WrapDistance( home->x - pose.x ));	 	 
		double dy( WrapDistance( home->y - pose.y ));		  
		double da( atan2( dy, dx ));
		double dist( hypot( dx, dy ));
		
		if( Holding() )
			{ // drive home		  
				// turn towards home		  
				heading_error = AngleNormalize( da - pose.a );// < 0  ? 0.05 : -0.05;
				
				// if we're some random distance inside the home radius
				if( dist < drand48() * home->r )
					Drop(); // release the puck (implies we won't be holding
				// next time round)
			}
		else // not holding
			{
				// if I see any pucks and I'm away from home
				if( see_pucks.size() > 0 && dist > home->r )
					{
						// find the angle to the closest puck that is not being carried
						double closest_range(1e9); //BIG				
						FOR_EACH( it, see_pucks )
							{
								if( it->range < closest_range && !it->held)						 
									{
										heading_error = it->bearing;
										closest_range = it->range; // remember the closest range so far
									}
							}
						
						// and attempt to pick something up
						if( Pickup() )
							{
								// got one! remember where it was
								lastx = pose.x;
								lasty = pose.y;
							}
					}
				else
					{
						double lx( WrapDistance( lastx - pose.x ));	 	 
						double ly( WrapDistance( lasty - pose.y ));		  
						
						// go towards the last place I picked up a puck
						heading_error = AngleNormalize( atan2(ly, lx) - pose.a );
						
						// if I've arrived at the last place and not yet found a
						// puck, choose another place 
						if( hypot( lx,ly ) < 0.05 )
							{
								lastx += drand48() * 0.4 - 0.2;
								lasty += drand48() * 0.4 - 0.2;
								
								DistanceNormalize( lastx );
								DistanceNormalize( lasty );
							}
					}
			}
		
		// if I'm pointing in about the right direction
		if( fabs( heading_error ) < 0.1 )
			{
				speed.v = 0.005; // drive fast
				speed.w = 0.0; // don't turn
			}
		else
			{
				speed.v = 0.001; // drive slowly
				speed.w = 0.2 * (heading_error); // turn to reduce the error
			}		
  }
};

int main( int argc, char* argv[] )
{
  // configure global robot settings
  Robot::Init( argc, argv );
	
  for( unsigned int i=0; i<Robot::home_count; i++ )
		{
			Home* h = new Home( i < color_count ? colors[i] : Home::Color::Random(), 
													i ? drand48() * Robot::worldsize : Robot::worldsize/2.0,
													i ? drand48() * Robot::worldsize : Robot::worldsize/2.0,													
													0.1 );
			
			Robot::homes.push_back( h );			
			
			for( unsigned int i=0; i<Robot::population_size; i++ )
				new Forager( h );
		}		
  // and start the simulation running
  Robot::Run();
  
  // we'll probably never get here, but this keeps the compiler happy.
  return 0;
}
