
#include "antix.h"

// this is the robot controller code
class Forager : public Antix::Robot
{
 public:  
  double lastx, lasty;  
  
 Forager( Antix::Home* h ) 
	: Robot( h, Pose() ), 
		lastx(home->x), // initial search location is close to my home
		lasty(home->y)
			{
			  double delta( 4.0 );
			  DistanceNormalize( pose.x = delta * drand48() -delta/2.0 + home->x );
			  DistanceNormalize( pose.y = delta * drand48() -delta/2.0 + home->y );
			}
  
  // must implement this method. Examine the pixels vector and set the
  // speed sensibly.
  virtual void Controller();
};
