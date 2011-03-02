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
#include <assert.h>

#define GRAPHICS 1
#define DEBUGVIS 0

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
  	
  // bounds type - specifies a range of values
  typedef struct
  {
    double min, max;
  } bounds_t;

  // bounding box type - specifies a 2d range of values
  typedef struct
  {
    bounds_t x, y;
  } bbox_t;
  
  class Home;

  class Puck
  {
  public:
    bool held; // true iff carried by a robot 
    Home* home;
    unsigned int index; // the matrix cell that currently contains this puck
    unsigned int lifetime;
    double x,y; // location
    //unsigned int count; // number of puck units here 
    
    /** constructor places a puck at specified pose */
    Puck( double x, double y ); 
    ~Puck();
    
    void Replace();

    //Puck() 
    //:  held(false), index(0), lifetime(10), x(0.0), y(0.0)//, count(1) 
	   // {  /* do nothing */ }		 
    
    void Pickup();
    void Drop();
  };		 
  
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
    
    std::set<Puck*> pucks;

    unsigned int score;

    double x, y, r;

    Home( const Color& color, double x, double y, double r );

    void UpdatePucks();
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
	 
	 static uint64_t updates; // number of simulation steps so far	 
	 static uint64_t updates_max; // number of simulation steps to run before quitting (0 means infinity)
	 static unsigned int home_count; // number of home zones
	 static unsigned int home_population; // number of robots
	 static unsigned int puck_count; // number of pucks that exist in the world
	 static unsigned int sleep_msec; // number of milliseconds to sleep at each update

	 static unsigned int gui_interval; // number of milliseconds between window redraws
	 static Robot* first;
	 

	 class MatrixCell
	 {
	 public:
		 std::set<Robot*> robots;
		 std::set<Puck*> pucks;
	 };

	 static std::vector<Robot::MatrixCell> matrix;
	 static unsigned int matrixwidth;

	 void TestPucksInCell( const MatrixCell& cell );
	 void TestRobotsInCell( const MatrixCell& cell );

	 unsigned int index; // the matrix cell that currently holds this robot

#if GRAPHICS
	 static int winsize; // initial size of the window in pixels

	 /** initialization: call this before using any other calls. */	
	 static void InitGraphics( int argc, char* argv[] );

	 /** render all robots in OpenGL */
	 static void DrawAll();

	 // render the robot in OpenGL
	 void Draw();	 
#endif
	
	 bbox_t sensor_bbox; 
	 void FovBBox( bbox_t& box );

	 // deliver pucks to this location
	 Home* home;
	 
		class Pose
		{
		public:
			double x,y,a; // 2d position and orientation
			
		Pose( double x, double y, double a ) : x(x), y(y), a(a) {}
		Pose() : x(0.0), y(0.0), a(0.0) {}

			//Pose( const Pose &p ) : x(p.x), y(p.y), a(p.a) {}	
			
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
	 
	 static inline unsigned int Cell( double x )
	 {
		const double d = Robot::worldsize / (double)Robot::matrixwidth;
		
		while( x > worldsize ) // wraparound
		  x -= worldsize;
		
		while( x < 0 ) // wraparound
		  x += worldsize;
		
		 return floor( x / d );
	 }
	 
	 static inline int CellNoWrap( double x )
	 {
	   const double d = Robot::worldsize / (double)Robot::matrixwidth;	      		
	   return floor( x / d );
	 }
	 
	 static inline unsigned int CellWrap( int x )
	 {
		while( x >= (int)matrixwidth ) // wraparound
		  x -= matrixwidth;
		
		while( x < 0 ) // wraparound
		  x += matrixwidth;
		
		return x;
	 }
	 
	 static inline unsigned int Cell( double x, double y ) 
	 {
		return (Cell(x) + (Cell(y) * Robot::matrixwidth) );		 
	 }
	 
	 inline void UpdateSensorsCell( unsigned int x, unsigned int y )
	 {
		unsigned int index( CellWrap(x) + ( CellWrap(y) * matrixwidth ));
		TestRobotsInCell( matrix[index] );
		TestPucksInCell( matrix[index] );
		
#if DEBUGVIS		
		neighbor_cells.insert( index );
#endif
	 }
	 	 
	 static std::vector<Puck> pucks;

	 class SeePuck
	 {
	 public:
		 Puck* puck;
		 bool held;
		 double bearing;		 
		 double range;
		 
	 SeePuck( Puck* puck,  const double range, const double bearing, const bool held )
		: puck(puck), held(held), bearing(bearing), range(range) 
		 { /* empty */}
	 };
	 
	 /** A sense vector containing information about all the pucks
			 detected in my field of view */
	 std::vector<SeePuck> see_pucks;	 	 
#if DEBUGVIS
	 std::vector<Robot*> neighbors;
	 std::vector<Puck*> neighbor_pucks;
	 std::set<unsigned int> neighbor_cells;
#endif

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
	 bool Holding() const;
	  
	 /** pure virtual - subclasses must implement this method  */
	 virtual void Controller() = 0;

	private:
	 Puck* puck_held;
	 
	 // move the robot
	 void UpdatePose();
	 
	 // update
	 void UpdateSensors();
  };	

  // fast approximation to atan2
  inline double fast_atan2( double y, double x )
  {
    const double piD2( M_PI/2.0 );
    double atan;
    double z = y/x;
    
    if ( x == 0.0 ){
      if ( y > 0.0 ) return piD2;
      if ( y == 0.0 ) return 0.0;
      return -piD2;
    }
    
    if ( fabs( z ) < 1.0 ){
      atan = z/(1.0 + 0.28*z*z);
      if ( x < 0.0 ){
	if ( y < 0.0 ) return atan - M_PI;
	return atan + M_PI;
      }
    }
    else{
      atan = piD2 - z/(z*z + 0.28f);
      if ( y < 0.0f ) return atan - M_PI;
    }
    return atan;
  }
  
  inline double fast_sin(double x)
  {
    const double B = 4/M_PI;
    const double C = -4/(M_PI*M_PI);
    const double P = 0.225;
    const double y = B * x + C * x * fabs(x);  
    return(  P * (y * fabs(y) - y) + y );   
  }
  
  inline double fast_cos(double x)
  {
    const double B = 4/M_PI;
    const double C = -4/(M_PI*M_PI);
    const double P = 0.225;
    
    x = x + M_PI/2;
    if(x > M_PI){   // Original x > M_PI/2
      x -= 2 * M_PI;   // Wrap: cos(x) = cos(x - 2 M_PI)
    }
    
    double y = B * x + C * x * fabs(x);  //fast, inprecise
    return( P * (y * fabs(y) - y) + y );  
  }
}; // namespace Antix
