
#include "antix.h"


// this is the robot controller code
class Forager : public Antix::Robot
{
 public:  
  double lastx, lasty;  
  
  Forager( Antix::Home* h );
  
  // must implement this method. Examine the pixels vector and set the
  // speed sensibly.
  virtual void Controller();
};
