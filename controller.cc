/*****
		controller.cc
		version 1
		Copyright Richard Vaughan, 2009.09.09
****/

#include <math.h>
#include "controller.h"
using namespace Antix;


Forager::Forager( Antix::Home* h ) 
  : Robot( h, Pose() ), 
    lastx(home->x), // initial search location is close to my home
    lasty(home->y)
{
  double delta( 4.0 );
  DistanceNormalize( pose.x = delta * drand48() -delta/2.0 + home->x );
  DistanceNormalize( pose.y = delta * drand48() -delta/2.0 + home->y );
  
//   static bool startup( true );

//   if( startup )
//     {
//       RVOsim->setAgentDefaults(0.1,10,10,2,1.3*Robot::radius, 0.005);
//       RVOsim->setTimeStep(1);
//       startup = false;
//     }

//   this->RVOid = Robot::RVOsim->addAgent(RVO::Vector2(pose.x,pose.y));  
}

void Forager::Controller()
{		
  double heading_error(0.0);
  
  // distance and angle to home
  double dx( WrapDistance( home->x - pose.x ));	 	 
  double dy( WrapDistance( home->y - pose.y ));		  
  double da( fast_atan2( dy, dx ));
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
	      if( it->range < closest_range && !it->held  )						 
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
	      heading_error = AngleNormalize( fast_atan2(ly, lx) - pose.a );
	      
	      // if I've arrived at the last place and not yet found a
	      // puck, choose another place 
	      if( hypot( lx,ly ) < 0.05 )
		{
		  lastx += drand48() * 1.0 - 0.5;
		  lasty += drand48() * 1.0 - 0.5;
		  
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
  
