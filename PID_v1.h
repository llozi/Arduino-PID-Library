#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.1

class PID
{


  public:

    //Constants used in some of the functions below
    #define AUTOMATIC	1
    #define MANUAL	0
    #define DIRECT  0
    #define REVERSE  1
    #define P_ON_M 0
    #define P_ON_E 1

    //commonly used functions **************************************************************************
    #if defined(USE_FLOAT)
    PID(float*, float*, float*,          // * constructor.  links the PID to the Input, Output, and 
        float, float, float, int, int);   //   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    PID(float*, float*, float*,           // * constructor.  links the PID to the Input, Output, and 
        float, float, float, int);        //   Setpoint.  Initial tuning parameters are also set here
	
    void SetOutputLimits(float, float); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
    #else
    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double, int, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here
	
    void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
    #endif

    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively
	


    //available but not commonly used functions ********************************************************
    #if defined(USE_FLOAT)
    void SetTunings(float, float,         // * While most users will set the tunings once in the 
                    float);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(float, float,         // * overload for specifying proportional mode
                    float, int);
    #else
    void SetTunings(double, double,       // * While most users will set the tunings once in the 
                    double);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(double, double,       // * overload for specifying proportional mode
                    double, int);
    #endif

	  void SetControllerDirection(int);	    // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
										  
										  
    //Display functions ****************************************************************
    #if defined(USE_FLOAT)
    float GetKp();						  // These functions query the pid for interal values.
    float GetKi();						  //  they were created mainly for the pid front-end,
    float GetKd();						  // where it's important to know what is actually
    #else
    double GetKp();						  // These functions query the pid for interal values.
    double GetKi();						  //  they were created mainly for the pid front-end,
    double GetKd();						  // where it's important to know what is actually
    #endif
    int GetMode();					    //  inside the PID.
    int GetDirection();				  //
    
  private:
    void Initialize();
    
    #if defined(USE_FLOAT)
    float dispKp;				// * we'll hold on to the tuning parameters in user-entered 
    float dispKi;				//   format for display purposes
    float dispKd;				//
      
    float kp;                  // * (P)roportional Tuning Parameter
    float ki;                  // * (I)ntegral Tuning Parameter
    float kd;                  // * (D)erivative Tuning Parameter

    float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    float *myOutput;             //   This creates a hard link between the variables and the 
    float *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
    float outputSum, lastInput;
    float outMin, outMax;
    #else
    double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
    double dispKi;				//   format for display purposes
    double dispKd;				//
      
    double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the 
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
    double outputSum, lastInput;
    double outMin, outMax;
    #endif
          
    int controllerDirection;
    int pOn;

    unsigned long lastTime;
    unsigned long SampleTime;
    bool inAuto, pOnE;
};
#endif

