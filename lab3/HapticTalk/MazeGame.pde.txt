 /**
 **********************************************************************************************************************
 * @file       MazeGame.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V3.0.0
 * @date       27-September-2018
 * @brief      Base project template for use with pantograph 2-DOF device and 2-D physics engine
 *             creates a blank world ready for creation
 * @modified  Bereket Guta: Jan 30 2023 to create a maze game
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */
 
 
 
 /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 3;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           rendering_force                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           pos_ee                              = new PVector(0, 0);
PVector           f_ee                                = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

/* Initialization of virtual tool */
HVirtualCoupling  s;

/* Hello Maze*/
MazeGrid maze;

int w = 160; // width in pixels
int cols;
int rows;
int seed = -1; // seed for maze generation

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = w*0.2;


float boxW = w/pixelsPerCentimeter; // in centimeter
float boxH = 1; // in centimeter

/* helper to start sim */
boolean simStart = false;
boolean startGame = false;
boolean finishGame = false;

boolean startTimer = false;
boolean stopTimer = false;

/* end elements definition *********************************************************************************************/

/* Background image definition*/
PImage backImg;

/* Game Timer */
int startTime;
int stopTime;


/* define start and stop buttons (adopted from Maze example) */
FCircle           start_b;
FCircle           stop_b;

/* Game text */
PFont             f;

/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(640, 640);
  
  backImg = loadImage("Sierra_Madre.jpg");
  backImg.resize(width, height);

  cols = floor(width / w);
  rows = floor(height / w);

  /* Maze definition */
  maze = new MazeGrid(cols, rows, w, boxW, boxH, seed);
  
  /* set font type and size */
  f                   = createFont("Georgia", 32, true);
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[0], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();

  widgetOne.set_mechanism(pantograph);
  
  /**
  // commented out to fix inverse motion of the ball
  widgetOne.add_actuator(1, CW, 1);
  widgetOne.add_actuator(2, CW, 2);
 
  widgetOne.add_encoder(1, CW, 180, 13824, 1);
  widgetOne.add_encoder(2, CW, 0, 13824, 2);
  */
  
  //start: added to fix inverse motion of the ball
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);

  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  //end: added to fix inverse motion of the ball
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((2)); 
  s.h_avatar.setDensity(1); 
  s.h_avatar.setFill(0,255,0); 
  s.init(world, boxW*0.5 + cols*0.5*boxW, boxW*0.5); 
  
  // turn into a ghost until the game starts
  s.h_avatar.setSensor(true);
  
  /* World conditions setup */
  world.setGravity((0.0), (300.0)); //1000 cm/(s^2)
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  
  /* Start Button */
  start_b                  = new FCircle(2.0); // diameter is 2
  start_b.setPosition(boxW*0.5, boxW*0.5);
  start_b.setFill(0, 255, 0, 50);
  start_b.setStaticBody(true);
  world.add(start_b);
  
  /* Finish Button */
  stop_b                  = new FCircle(2.0);
  stop_b.setPosition(boxW*0.5 + (cols - 1)*boxW, boxW*0.5 + (rows-1)*boxW);
  stop_b.setFill(200,0,0, 50);
  stop_b.setStaticBody(true);
  stop_b.setSensor(true);
  world.add(stop_b);
 
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
}



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  background(backImg); 
  maze.show();
  maze.stepMaze();
  
  if(maze.finished){
     maze.drawWalls(world); 
     world.draw();
     
     
     textFont(f, 32);
 
    if(startGame && !finishGame){
      
      if(!startTimer){ // start the timer if it has not been started
        startTime = millis();
        startTimer = true;
        stopTimer = false;
      }
      
      
      int totalTime = millis() - startTime;
      fill(0, 255, 0);
      textAlign(CENTER);
      text("Get to the red circle!" + "\nElapsed Time(ms): " + totalTime, width/2, 60);
    } else if (!startGame && !finishGame) {
      fill(0, 0, 0);
      textAlign(CENTER);
      text("Touch the green circle to start the maze!", width/2, 60);
    }else{
      
      if(!stopTimer){ // stop the timer if it has not been started
        stopTime = millis()  - startTime;
        stopTimer = true;
        startTimer = false;
      }
      
      fill(0, 0, 0);
      textAlign(CENTER);
      text("You finished the maze!"  + "\nElapsed Time(ms): " + stopTime + "\nTouch the green circle to restart!", width/2, 60);
    }
     
     if(maze.walls_done && !simStart){
       /* setup simulation thread to run at 1kHz */ 
       SimulationThread st = new SimulationThread();
       scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
       
       // start the simulation
       simStart = true;
     }
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    rendering_force = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      pos_ee.set(widgetOne.get_device_position(angles.array()));
      pos_ee.set(pos_ee.copy().mult(200));  
    }
    
    s.setToolPosition(boxW * 0.5 + cols*0.5*boxW - (pos_ee).x+2, boxW * 0.5 +(pos_ee).y-7); 
    s.updateCouplingForce();
    f_ee.set(-s.getVCforceX(), s.getVCforceY());
    f_ee.div(20000); //
    
    torques.set(widgetOne.set_device_torques(f_ee.array()));
    widgetOne.device_write_torques();
    
    if (s.h_avatar.isTouchingBody(start_b)){
      startGame = true;
      s.h_avatar.setSensor(false);
      start_b.setSensor(true);
      stop_b.setSensor(false);
      finishGame = false;
    }
  
    if (startGame && s.h_avatar.isTouchingBody(stop_b)){
      stop_b.setSensor(true);
      finishGame = true;
      startGame = false;
    }
    world.step(1.0f/1000.0f);
  
    rendering_force = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
