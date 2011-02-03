/****
	  antix.h
		Clone this package from git://github.com/rtv/Antix.git
	  version 2
	  Richard Vaughan  
****/

#include <vector>
#include <set>
#include <list>
#include <math.h> 
#include <stdio.h>
#include <stdlib.h>

#define GRAPHICS 1

// handy STL iterator macro pair. Use FOR_EACH(I,C){ } to get an iterator I to
// each item in a collection C.
#define VAR(V,init) __typeof(init) V=(init)
#define FOR_EACH(I,C) for(VAR(I,(C).begin());I!=(C).end();I++)

namespace Antix
{
  /** Convert radians to degrees. */
  inline double rtod( double r ){ return( r * 180.0 / M_PI ); }
  /** Convert degrees to radians */
  inline double dtor( double d){ return( d * M_PI / 180.0 ); }
  	
	class Home
	{
	public:
		
		class Color
		{
		public:
			double r, g, b;
			
		Color( double r, double g, double b ) :	r(r), g(g), b(b) {}
			
			// get a random color
			static Color Random()
			{
				return Color( drand48(), drand48(), drand48() );
			}
			
		} color; 
		
		double x, y, r;
		
		Home( const Color& color, double x, double y, double r );
	};
	
  class Robot
  {
  public:

	 /** initialization: call this before using any other calls. */	
	 static void Init( int argc, char** argv );

	 static void UpdateGui();

	 /** update all robots */
	 static void UpdateAll();

	 /** Normalize a length to within 0 to worldsize. */
	 static double DistanceNormalize( double d );

	 /** Normalize an angle to within +/_ M_PI. */
	 static double AngleNormalize( double a );
	 
	 /** Wrap distances around the torus */
	 static double WrapDistance( double d );

	 /** Start running the simulation. Does not return. */
	 static void Run();

	 static bool paused; // runs only when this is false
	 static bool show_data; // controls visualization of pixel data
	 static double fov;      // sensor detects objects within this angular field-of-view about the current heading
	 static double pickup_range;
	 static double radius; // radius of all robot's bodies
	 static double range;    // sensor detects objects up tp this maximum distance
	 static double worldsize; // side length of the toroidal world
	 

	 static std::vector<Home*> homes;
	 static std::vector<Robot*> population;
	 
	 static Robot* leftmost;
	 static Robot* bottom;

	 static uint64_t updates; // number of simulation steps so far	 
	 static uint64_t updates_max; // number of simulation steps to run before quitting (0 means infinity)
	 static unsigned int home_count; // number of home zones
	 static unsigned int home_population; // number of robots
	 static unsigned int puck_count; // number of pucks that exist in the world
	 static unsigned int sleep_msec; // number of milliseconds to sleep at each update

#if GRAPHICS
	 static int winsize; // initial size of the window in pixels

	 /** initialization: call this before using any other calls. */	
	 static void InitGraphics( int argc, char* argv[] );

	 /** render all robots in OpenGL */
	 static void DrawAll();

	 // render the robot in OpenGL
	 void Draw();	 
#endif
	 
	 // pointers to next robots sorted by axis
	 Robot *left, *right, *up, *down;
	 
	 // deliver pucks to this location
	 Home* home;
	 
	 static void SortX();
	 static void SortY();

		class Pose
		{
		public:
			double x,y,a; // 2d position and orientation
			
		Pose( double x, double y, double a ) : x(x), y(y), a(a) {}
		Pose() : x(0.0), y(0.0), a(0.0) {}
			
			// get a random pose 
			static Pose Random()
			{
			  return Pose( drand48() * Robot::worldsize, 
								drand48() * Robot::worldsize, 
								Robot::AngleNormalize( drand48() * (M_PI*2.0)));
			}
		} pose; // instance: robot is located at this pose
		
		class Speed
		{		
		public:
			double v; // forward speed
			double w; // turn speed
	  	
			// constructor sets speeds to zero
		Speed() : v(0.0), w(0.0) {}		
		} speed; // instance: robot is moving this fast
		
		class SeeRobot
		{
		public:
		  const Home* home;
		  Pose pose;
		  Speed speed;
		  double range;
		  double bearing;
		  bool haspuck;
			
		SeeRobot( const Home* home, const Pose& p, const Speed& s, const double range, const double bearing, const bool haspuck )
		  : home(home), pose(p), speed(s), range(range), bearing(bearing), haspuck(haspuck)
			{ /* empty */}
	 };
	 
	 /** A sense vector containing information about all the robots
			 detected in my field of view */
	 std::vector<SeeRobot> see_robots;

	public: class Puck
	 {
	 public:
		 double x, y;
		 bool held;

		 /** constructor places a puck at specified pose */
		 //Puck( double x, double y ) : x(x), y(y), held(false) {}
		 
		 /** default constructor places puck at random pose */
	 Puck() 
		 : x(drand48()*worldsize), y(drand48()*worldsize), held(false) 
			 { /* empty */ }
		 
	 };		 
	 
	 static std::vector<Puck> pucks;

	 class SeePuck
	 {
	 public:
		 Puck* puck;
		 double range;
		 double bearing;		 
		 bool held;
		 
	 SeePuck( Puck* puck,  const double range, const double bearing, const bool held )
		 : puck(puck), range(range), bearing(bearing), held(held)
		 { /* empty */}
	 };
	 
	 /** A sense vector containing information about all the pucks
			 detected in my field of view */
	 std::vector<SeePuck> see_pucks;	 
	 
	 	 
	 // constructor
	 Robot( Home* home, const Pose& pose );
	 
	 // destructor
	 virtual ~Robot() {}
	 
	 /** Attempt to pick up a puck. Returns true if one was picked up,
			 else false. */
	 bool Pickup(); 
	 
	 /** Attempt to drop a puck. Returns true if one was dropped, else
			 false. */
	 bool Drop();
	 
	 /** Returns true if we are currently holding a puck. */
	 bool Holding();
	  
	 /** pure virtual - subclasses must implement this method  */
	 virtual void Controller() = 0;

	private:
	 Puck* puck_held;
	 
	 // move the robot
	 void UpdatePose();
	 
	 // update
	 void UpdateSensors();
  };	
}; // namespace Antix
