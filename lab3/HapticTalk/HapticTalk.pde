/**
 **********************************************************************************************************************
 * @file       Drag_n_Drop.pde
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher
 * @version    V3.0.0
 * @date       21-September-2018
 * @brief      Drag and Drop haptic example using 2D physics engine 
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

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           rendering_force                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           pos_ee                              = new PVector(0, 0);
PVector           f_ee                                = new PVector(0, 0); 

/* World boundaries in centimeters*/
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0;

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

/* joints to be created for avatar */
boolean           jointCreated                        = false;
FDistanceJoint    d1;

/* Initialization of virtual tool */
HVirtualCoupling  s;
FCircle           h1; // grab radius

/* define region seperator boxes */
FBox              b1;
FBox              b2;
FBox              b3;

/* define water trap */
FBox              f1;

/* define goalie variables */
FBox              g1;
FBox              g2;
FPrismaticJoint   p1;
boolean           upOrDown                            = false;
float             gTime;
float             gTimeLast;
float             gTimeLimit                          = 1500;

/* define beads total */
int               beads                               = 45;

/* define linkage state */
boolean           canLink                             = false;

/* Sensor data */
float[]           sensorData;

/* region information */
int region = -1;

/* angle for region 2 */
float theta = 0;
float radius = 1.0;

/* end elements definition *********************************************************************************************/  



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 400);
  
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
  
  //start: added to fix inverse motion of the ball
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);

  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  //end: added to fix inverse motion of the ball
  
  //widgetOne.add_analog_sensor("A1");
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  
  /* Barrier box for word 1 | 2 */
  b1                  = new FBox(1, 8);
  b1.setPosition(worldWidth/3, 7);
  b1.setFill(0, 0, 0);
  b1.setStatic(true);
  world.add(b1);
  
  
  /* Barrier box for word 2 | 3 */
  b2                  = new FBox(1, 8);
  b2.setPosition(2*worldWidth/3, 7);
  b2.setFill(0, 0, 0);
  b2.setStatic(true);
  world.add(b2);
  
  
  
  
  /* Water Bucket */
   f1                  = new FBox(worldWidth/3 -1, 8);
   f1.setPosition(worldWidth/3 + worldWidth/(3*2), 7);
   f1.setFill(150, 150, 255, 80);
   f1.setDensity(5);
   f1.setSensor(true);
   f1.setNoStroke();
   f1.setStatic(true);
   world.add(f1);
  
  
  /* Goalie box definition */
  g1                  = new FBox(2, 5);
  g1.setFill(150, 0, 0);
  g1.setPosition(2*worldWidth/3 + 0.5 + 0.5*2, 5 + 1.5);
  g1.setDensity(10);
  g1.setStatic(false);
  world.add(g1);

  g2                  = new FBox(2, 5);
  g2.setFill(150, 0, 0);
  g2.setPosition(worldWidth - 0.5*2 - 0.75, 5 + 1.5);
  g2.setDensity(10);
  g2.setStatic(false);
  world.add(g2);
  // p1 = new FPrismaticJoint(g1, b2);
  // p1.setNoStroke();
  // world.add(p1);
  
  

  
  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((0.25)); 
  s.h_avatar.setDensity(10);
  s.h_avatar.setFill(255,0,0); 
  //s.h_avatar.setSensor(true);
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2);
  
  //h1                  = new FCircle(0.75);
  //h1.setDensity(0);
  //h1.setNoStroke();
  //h1.setSensor(true);
  //h1.setFill(0, 0, 0, 50);
  //h1.setPosition(3, 3);
  //world.add(h1);
  
  
  /* world conditions setup */ 
  world.setGravity((0.0), (300.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  background(255);
  

  checkRegion();
  println(region);
  gTime = millis();
  if (gTime - gTimeLast > gTimeLimit){
    if (abs(g1.getVelocityX())      < 5){
      upOrDown =! upOrDown;
      gTimeLast = gTime;
    }
  }
  if(upOrDown == false){
    g1.setVelocity(5,0);
    g2.setVelocity(-5,0);
  }
  else{
    g1.setVelocity(-5,0);
    g2.setVelocity(5,0);
  }
  
  theta += 0.01;
  radius = cos(theta); 
  world.draw();  
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
    


    if(region == 0){

      pos_ee.add(getRandomVector(1));

    } else if(region == 1){
        var y = radius*cos(theta);
        var x = radius*sin(theta);
        var v = new PVector(x, y);
        pos_ee.add(v);
    } else if (region == 2){
        var y = radius*cos(theta*10)*2;
        var v = new PVector(0, y);
        //pos_ee.add(v);
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+2, edgeTopLeftY+(pos_ee).y-7); 
    s.updateCouplingForce();
    f_ee.set(-s.getVCforceX(), s.getVCforceY());
 
    println(s.getVCforceY());
    f_ee.div(20000); //
    torques.set(widgetOne.set_device_torques(f_ee.array()));
    widgetOne.device_write_torques();
  
    //h1.setPosition(s.h_avatar.getX(), s.h_avatar.getY());

    if (s.h_avatar.isTouchingBody(f1)){
      s.h_avatar.setDamping(700);
    }
    else{
      s.h_avatar.setDamping(40); 
    }
 
  
    world.step(1.0f/1000.0f);
  
    rendering_force = false;
  }
}
/* end simulation section **********************************************************************************************/



/* end helper functions section ****************************************************************************************/


/*
 return a random vector magnitude equal to scale
*/
PVector getRandomVector(float scale){
  var v = PVector.random2D();
  v.setMag(scale);
  return v;
}

void checkRegion(){

  float x = s.getAvatarPositionX();
  float y = s.getAvatarPositionY();

  if(y < 3){
    region = -1;
    return;
  }

  if(x < worldWidth/3 - 0.5){
    region = 0;
  }else if(x < 2*worldWidth/3 - 0.5){
    region = 1;
  }else{
    region = 2;
  }

}
