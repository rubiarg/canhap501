/**
 **********************************************************************************************************************
 * @file       sketch_4_Wall_Physics.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V4.1.0
 * @date       08-January-2021
 * @brief      wall haptic example using 2D physics engine 
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
boolean           renderingForce                      = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                = new PVector(0, 0); 

/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 15; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;


/* Initialization of wall */
FBox              wall;

/* Initialization of finish sign */
FBox           c1;


/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar;

/* end elements definition *********************************************************************************************/ 



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 600);
  
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
  haplyBoard          = new Board(this, Serial.list()[1], 1);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  

  /* hardcoded maze */
  // row 8
  wall                   = new FBox(1.8, 0.3);
  wall.setPosition(edgeTopLeftX + 2.75, edgeBottomRightY - 2);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(3.3, 0.3);
  wall.setPosition(edgeTopLeftX + 8, edgeBottomRightY - 2);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(1.8, 0.3);
  wall.setPosition(edgeTopLeftX + 13.25, edgeBottomRightY - 2);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(6, 0.3);
  wall.setPosition(edgeTopLeftX + 18.4, edgeBottomRightY - 2);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // row 7
  wall                   = new FBox(1.8, 0.3);
  wall.setPosition(edgeTopLeftX + 2.75, edgeBottomRightY - 3.5);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(7.8, 0.3);
  wall.setPosition(edgeTopLeftX + 10.25, edgeBottomRightY - 3.5);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // row 6
  wall                   = new FBox(4.8, 0.3);
  wall.setPosition(edgeTopLeftX + 8.75, edgeBottomRightY - 5);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(3.3, 0.3);
  wall.setPosition(edgeTopLeftX + 21.5, edgeBottomRightY - 5);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // row 5
  wall                   = new FBox(1.5, 0.3);
  wall.setPosition(edgeTopLeftX + 1.4, edgeBottomRightY - 6.5);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(1.8, 0.3);
  wall.setPosition(edgeTopLeftX + 4.25, edgeBottomRightY - 6.5);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(4.8, 0.3);
  wall.setPosition(edgeTopLeftX + 8.75, edgeBottomRightY - 6.5);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(1.8, 0.3);
  wall.setPosition(edgeTopLeftX + 17.75, edgeBottomRightY - 6.5);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
    
  // row 4
  wall                   = new FBox(3.3, 0.3);
  wall.setPosition(edgeTopLeftX + 6.5, edgeBottomRightY-8);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(1.8, 0.3);
  wall.setPosition(edgeTopLeftX + 14.75, edgeBottomRightY-8);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(1.8, 0.3);
  wall.setPosition(edgeTopLeftX + 19.25, edgeBottomRightY-8);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // row 3
  wall                   = new FBox(3.3, 0.3);
  wall.setPosition(edgeTopLeftX + 5.0, edgeBottomRightY - 9.5);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(1.8, 0.3);
  wall.setPosition(edgeTopLeftX + 8.75, edgeBottomRightY - 9.5);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(3.3, 0.3);
  wall.setPosition(edgeTopLeftX + 15.5, edgeBottomRightY - 9.5);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(3.3, 0.3);
  wall.setPosition(edgeTopLeftX + 20.0, edgeBottomRightY - 9.5);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // row 2
  wall                   = new FBox(1.5, 0.3);
  wall.setPosition(edgeTopLeftX + 5.9, edgeBottomRightY - 11);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(3, 0.3);
  wall.setPosition(edgeTopLeftX + 9.65, edgeBottomRightY - 11);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(4.8, 0.3);
  wall.setPosition(edgeTopLeftX + 16.25, edgeBottomRightY - 11);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // row 1
  wall                   = new FBox(1.5, 0.3);
  wall.setPosition(edgeTopLeftX + 1.4, edgeBottomRightY - 12.5);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(21, 0.3);
  wall.setPosition(edgeBottomRightX - 9.35, edgeBottomRightY - 12.5);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
    
  // column 15
  wall                   = new FBox(0.3, 9.0);
  wall.setPosition(edgeBottomRightX - 2, edgeTopLeftY + 8.55);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // column 14
  wall                   = new FBox(0.3, 3);
  wall.setPosition(edgeBottomRightX - 3.5, edgeTopLeftY + 4.1);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(0.3, 4.5);
  wall.setPosition(edgeBottomRightX - 3.5, edgeTopLeftY + 7.6);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(0.3, 3);
  wall.setPosition(edgeBottomRightX - 3.5, edgeBottomRightY - 2);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // column 13
  wall                   = new FBox(0.3, 1.5);
  wall.setPosition(edgeBottomRightX - 5, edgeTopLeftY + 6.3);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // column 12
  wall                   = new FBox(0.3, 1.5);
  wall.setPosition(edgeBottomRightX - 6.5, edgeTopLeftY + 4.8);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(0.3, 4.5);
  wall.setPosition(edgeBottomRightX - 6.5, edgeBottomRightY - 5.8);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // column 11
  wall                   = new FBox(0.3, 1.5);
  wall.setPosition(edgeBottomRightX - 8, edgeTopLeftY + 6.3);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(0.3, 3);
  wall.setPosition(edgeBottomRightX - 8, edgeTopLeftY + 10);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // column 10
  wall                   = new FBox(0.3, 4.5);
  wall.setPosition(edgeBottomRightX - 9.5, edgeBottomRightY - 5.8);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // column 9
  wall                   = new FBox(0.3, 1.5);
  wall.setPosition(edgeBottomRightX - 11, edgeTopLeftY + 4.8);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(0.3, 6.0);
  wall.setPosition(edgeBottomRightX - 11, edgeTopLeftY + 9.9);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // column 8
  wall                   = new FBox(0.3, 7.5);
  wall.setPosition(edgeBottomRightX - 12.5, edgeTopLeftY + 6.3);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
 
  // column 7
  wall                   = new FBox(0.3, 3);
  wall.setPosition(edgeBottomRightX - 14, edgeBottomRightY - 2);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(0.3, 4.5);
  wall.setPosition(edgeBottomRightX - 14, edgeTopLeftY + 6.3);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
 
  // column 6
  wall                   = new FBox(0.3, 1.5);
  wall.setPosition(edgeBottomRightX - 15.5, edgeTopLeftY + 6.3);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
    
  // column 5
  wall                   = new FBox(0.3, 1.5);
  wall.setPosition(edgeBottomRightX - 17, edgeTopLeftY + 6.3);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // column 4
  wall                   = new FBox(0.3, 1.5);
  wall.setPosition(edgeBottomRightX - 18.5, edgeTopLeftY + 4.8);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(0.3, 1.5);
  wall.setPosition(edgeBottomRightX - 18.5, edgeBottomRightY - 5.7);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(0.3, 1.5);
  wall.setPosition(edgeBottomRightX - 18.5, edgeBottomRightY - 2.7);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // column 3
  wall                   = new FBox(0.3, 4.5);
  wall.setPosition(edgeBottomRightX - 20, edgeBottomRightY - 4.2);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // column 2
  wall                   = new FBox(0.3, 1.5);
  wall.setPosition(edgeBottomRightX - 21.5, edgeBottomRightY - 1.2);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(0.3, 4.5);
  wall.setPosition(edgeBottomRightX - 21.5, edgeBottomRightY - 5.7);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(0.3, 3);
  wall.setPosition(edgeBottomRightX - 21.5, edgeBottomRightY - 11.1);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  // column 1
  wall                   = new FBox(0.3, 1.5);
  wall.setPosition(edgeBottomRightX - 23, edgeTopLeftY + 9.3);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
  
  wall                   = new FBox(0.3, 4.5);
  wall.setPosition(edgeBottomRightX - 23, edgeTopLeftY + 4.8);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall);
 
  /* Finish sign */
  c1                   = new FBox(1.1, 0.3);
  c1.setPosition(edgeTopLeftX + 23.7, edgeBottomRightY - 6.5);
  c1.setStatic(true);
  c1.setFill(200, 0, 0);
  world.add(c1);
    
  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4);  
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
 
  
  /* If you are developing on a Mac users must update the path below 
   * from "../img/Haply_avatar.png" to "./img/Haply_avatar.png" 
   */
  haplyAvatar = loadImage("./img/Haply_avatar.png"); 
  haplyAvatar.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1)));
  s.h_avatar.attachImage(haplyAvatar); 


  /* world conditions setup */
  world.setGravity((0.0), (1000.0)); //1000 cm/(s^2)
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
  if(renderingForce == false){
    background(255);
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/

/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    
    
    s.updateCouplingForce();
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
