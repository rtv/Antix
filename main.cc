
#include "controller.h"
using namespace Antix;

Home::Color colors[] = { Home::Color(1,0,0), 
												 Home::Color(0,0.5,0), // darker green 
												 Home::Color(0.3,0.3,1),  // lighter blue
												 Home::Color(1,1,0), 
												 Home::Color(1,0,1), 
												 Home::Color(0,1,1), 
												 Home::Color(1,0,1) };

size_t color_count = 7;


int main( int argc, char* argv[] )
{
  // configure global robot settings
  Robot::Init( argc, argv );
	
	// create each home, and each robot within each home
  for( unsigned int i=0; i<Robot::home_count; i++ )
		{
			Home* h = new Home( i < color_count ? colors[i] : Home::Color::Random(), 
													i ? drand48() * Robot::worldsize : Robot::worldsize/2.0,
													i ? drand48() * Robot::worldsize : Robot::worldsize/2.0,													
													0.1 );
			
			for( unsigned int i=0; i<Robot::home_population; i++ )
				new Forager( h );
		}		
  // and start the simulation running
  Robot::Run();
  
  // we'll probably never get here, but this keeps the compiler happy.
  return 0;
}
