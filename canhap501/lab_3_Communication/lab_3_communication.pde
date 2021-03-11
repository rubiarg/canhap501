/**
 **********************************************************************************************************************
 * @file       lab_3_layers.pde
 * @author     Hannah E., Rubia G.
 * @version    V4.0.0
 * @date       10-March-2021
 * @brief      Lab 3
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
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/

/* text font */
PFont             f;


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
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 30.0;  
float             worldHeight                         = 20.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2

/* Initialization of virtual tool */
HVirtualCoupling  s;

/* define maze blocks */
FBox                l1;
FBox                l2;
FBox                l3;
FCircle             c1;
FCircle             c2;
FCircle             c3;
FCircle             c4;
FCircle             c5;
FCircle             c6;
FCircle             c7;
FCircle             b1;
FBox                l4;


PVector posEELast = new PVector(0, 0);

String k = "off";

float threshold = 0.02;


/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */

  /* screen size definition */
  size(1200, 800);

  /* set font type and size */
  f                   = createFont("Arial", 30, true);

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


  l1                  = new FBox(15, 7.5);
  l1.setPosition(15/2, 8);
  l1.setFill(0);
  l1.setDensity(800);
  l1.setSensor(true);
  l1.setNoStroke();
  l1.setStatic(true);
  l1.setName("mode_1");

  l2                  = new FBox(15, 7.5);
  l2.setPosition(15/2, 15.5);
  l2.setFill(0);
  l2.setDensity(800);
  l2.setSensor(true);
  l2.setNoStroke();
  l2.setStatic(true);
  l2.setName("mode_1");

  l3                  = new FBox(15, 15);
  l3.setPosition(15 + 15/2, 11.75);
  l3.setFill(0);
  l3.setDensity(800);
  l3.setSensor(true);
  l3.setNoStroke();
  l3.setStatic(true);
  l3.setName("mode_1");

  c1                  = new FCircle(10);
  c1.setPosition(2, 12);
  c1.setFill(0);
  c1.setDensity(500);
  c1.setSensor(false);
  c1.setNoStroke();
  c1.setStatic(true);
  c1.setName("mode_2");

  c2                  = new FCircle(8);
  c2.setPosition(8, 14);
  c2.setFill(0);
  c2.setDensity(500);
  c2.setSensor(false);
  c2.setNoStroke();
  c2.setStatic(true);
  c2.setName("mode_2");

  c3                  = new FCircle(6);
  c3.setPosition(14, 12);
  c3.setFill(0);
  c3.setDensity(500);
  c3.setSensor(false);
  c3.setNoStroke();
  c3.setStatic(true);
  c3.setName("mode_2");

  c4                  = new FCircle(6);
  c4.setPosition(18, 13);
  c4.setFill(0);
  c4.setDensity(500);
  c4.setSensor(false);
  c4.setNoStroke();
  c4.setStatic(true);
  c4.setName("mode_2");

  c5                  = new FCircle(4);
  c5.setPosition(22, 14);
  c5.setFill(0);
  c5.setDensity(500);
  c5.setSensor(false);
  c5.setNoStroke();
  c5.setStatic(true);
  c5.setName("mode_2");

  c6                  = new FCircle(4);
  c6.setPosition(25, 12);
  c6.setFill(0);
  c6.setDensity(500);
  c6.setSensor(false);
  c6.setNoStroke();
  c6.setStatic(true);
  c6.setName("mode_2");

  c7                  = new FCircle(6);
  c7.setPosition(29, 14);
  c7.setFill(0);
  c7.setDensity(500);
  c7.setSensor(false);
  c7.setNoStroke();
  c7.setStatic(true);
  c7.setName("mode_2");

  l4                  = new FBox(15, 15);
  l4.setPosition(worldWidth/2, worldHeight/2);
  l4.setFill(0);
  l4.setDensity(800);
  l4.setSensor(true);
  l4.setNoStroke();
  l4.setStatic(true);
  l4.setName("mode_4");


  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(0); 
  s.h_avatar.setSensor(false);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)

  world.draw();

  /* setup framerate speed */
  frameRate(baseFrameRate);


  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw() {
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if (renderingForce == false) {
    background(0);
    world.draw();

    textAlign(CENTER);
    text("Press keys 1, 2, 3, or 4 to change mode.", width/2, 60);
    textAlign(CENTER);
    text("Before changing modes, return to callibration point to make sure you start in the same position.", width/2, 90);
    textAlign(LEFT);
    text("MODE: " + k, 15, 60);
  }
}
/* end draw section ****************************************************************************************************/

/* keyboard inputs ********************************************************************************************************/
void keyPressed() {
  /*reset*/
  if (key == '1') {
    if (!bodyExists(l1)) {
      removeBodyByName("mode_2");
      removeBodyByName("mode_3");
      removeBodyByName("mode_4");
      world.add(l1);
      world.add(l2);
      world.add(l3);
      k = "1";
    }
  }
  if (key == '2') {
    if (!bodyExists(c1)) {
      removeBodyByName("mode_1");
      removeBodyByName("mode_3");
      removeBodyByName("mode_4");
      world.add(c1);
      world.add(c2);
      world.add(c3);
      world.add(c4);
      world.add(c5);
      world.add(c6);
      world.add(c7);
      k = "2";
    }
  }
  if (key == '3') {
    if (!bodyExists(b1)) {
      removeBodyByName("mode_1");
      removeBodyByName("mode_2");
      removeBodyByName("mode_4");
      k = "3";
      for (int x = -10; x < worldWidth + 10; x++) {
        b1                  = new FCircle(0.5);
        b1.setPosition(x, 10);
        b1.setFill(0);
        b1.setDensity(500);
        b1.setSensor(false);
        b1.setNoStroke();
        b1.setStatic(true);
        b1.setName("mode_3");
        world.add(b1);
      }
    }
  }
  if (key == '4') {
    if (!bodyExists(l4)) {
      removeBodyByName("mode_1");
      removeBodyByName("mode_2");
      removeBodyByName("mode_3");
      world.add(l4);
      k = "4";
    }
  }
}

Boolean bodyExists(FBody body) {
  ArrayList<FBody> bodies = world.getBodies();
  for (FBody b : bodies) {
    try {
      if (b == body) {
        return true;
      }
    } 
    catch(NullPointerException e) {
      // do nothing
    }
  }
  return false;
}
void removeBodyByName(String bodyName) {
  ArrayList<FBody> bodies = world.getBodies();
  for (FBody b : bodies) {
    try {
      if (b.getName().equals(bodyName)) {
        world.remove(b);
      }
    } 
    catch(NullPointerException e) {
      // do nothing
    }
  }
}



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */

    renderingForce = true;

    if (haplyBoard.data_available()) {
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



    /* Viscous layer codes */
    if (s.h_avatar.isTouchingBody(l1)) {
      s.h_avatar.setDamping(700);
    } else if (s.h_avatar.isTouchingBody(l2)) {
      s.h_avatar.setDamping(800);
    } else if (s.h_avatar.isTouchingBody(l3)) {
      s.h_avatar.setDamping(950);
    } else if (s.h_avatar.isTouchingBody(l4)) {
      PVector xDiff = (posEE.copy()).sub(posEELast);
      posEELast.set(posEE);
      if ((xDiff.mag()) < threshold) { 
        s.h_avatar.setDamping(700);
        fEE.x = random(-1, 1);
        fEE.y = random(-1, 1);
      }
    } else {
      s.h_avatar.setDamping(4);
    }

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();

    world.step(1.0f/1000.0f);

    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/

/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
