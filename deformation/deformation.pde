import java.util.Random;
//use preprocessing algorithms (find contours, threshold)
import gab.opencv.*;
import java.awt.Rectangle;

//-----------------------------------------------
//Proscene
//Use InteractiveModelFrame and override actions
import remixlab.bias.core.*;
import remixlab.bias.event.*;
import remixlab.proscene.*;
import remixlab.dandelion.core.Constants.*;
import remixlab.dandelion.geom.*;
import remixlab.dandelion.core.*;
//-----------------------------------------------
/*Sebastian Chaparro
William Rpdriguez   
April 2 2015
*/  
//First Step get the image, and it contour
//For this purpose we're gonna use OpenCV
PImage source_image;
PImage destination_image;

//VARS TO FIND CONTOURS------------------------
OpenCV opencv;
float approximation = 0.1;
boolean invert = false; //change to true if the information of the image is in black 
PVector centroid;
PShape figure;
PShape deformed_figure;
ArrayList<Contour> contours        = new ArrayList<Contour>();
Contour contour;
ArrayList<PVector> edges           = new ArrayList<PVector>();  
ArrayList<PVector> deformed_edges  = new ArrayList<PVector>();  
//---------------------------------------------

//TEXTURE VARS IF IS REQUIRED------------------
PImage last_texture = null;
//---------------------------------------------

//CONTROL POINTS-------------------------------
ArrayList<PVector> control_points     = new ArrayList<PVector>(); 
ArrayList<PVector> control_points_out = new ArrayList<PVector>(); 
//---------------------------------------------

//PGRAPHICS------------------------------------
/*we're gonna use 3: the first one for display the original
object and it's deformation, the second one is used to GUI
options, the third one is used with proscene to show the 
usefulness of InteractiveObject Classes*/
PGraphics main_graphics;
PGraphics aux_graphics;
PGraphics gui_graphics;
int all_width  = 600;
int all_heigth = 620;
//---------------------------------------------

//SCENES---------------------------------------
/*Basically 2 scenes are required: one to draw the main Object,
the other one to control the world as an sphere */
Scene main_scene;
Scene aux_scene;
boolean show_control = true;
int aux_pos_x =  width-all_width/4;
int aux_pos_y =  height-all_heigth/3;
//---------------------------------------------
//Some models
InteractiveModelFrame original_fig;
InteractiveModelFrame deformed_fig;
ModelContour control_shape;

//Class that overrides iModelFrame to customize actions
color colour = color(255,0,0,100);
public class ModelContour extends InteractiveModelFrame{
  public ModelContour(Scene sc){
    super(sc);
    update();
  }
  @Override
  public void performCustomAction(DOF1Event event) {
    colour = color(color(random(0, 255), random(0, 255), random(0, 255), 125));
    update();
  }
  @Override
  public void performCustomAction(DOF2Event event) {
    //radiusX += event.dx();
    //radiusY += event.dy();
    update();
  }
  @Override
  public void performCustomAction(ClickEvent event) {
    colour = color(color(random(0, 255), random(0, 255), random(0, 255), 125));
    update();
  }
  //*/
  void update() {
    //setShape(createShape(ELLIPSE, -radiusX, -radiusY, 2*radiusX, 2*radiusY));
    shape().setFill(color(colour));
  }
}



void setup(){
  //load the image
  source_image = loadImage("/home/sebchap/Processing/Programs/Deformation/deformation/data/human.png");  
  source_image.resize(0,150);  
  size(all_width, all_heigth, P2D);
  main_graphics = createGraphics(all_width,2*all_heigth/3,P2D);
  main_scene = new Scene(this, main_graphics);
  aux_graphics = createGraphics(all_width/4,all_heigth/3,P2D);
  aux_scene = new Scene(this, aux_graphics, aux_pos_x, aux_pos_y);    
  main_scene.setAxesVisualHint(false);
  main_scene.setGridVisualHint(false);
  aux_scene.setAxesVisualHint(false);
  aux_scene.setGridVisualHint(false);
  main_scene.setRadius(50);
  //get the countours
  figure = getCountours();
  //associate the shape with the original shape frame
  original_fig = new InteractiveModelFrame(main_scene, figure);
  original_fig.translate(-centroid.x,-centroid.y);
  //initial deformed shape without modifications
  deformed_fig = new InteractiveModelFrame(main_scene);
  deformed_fig.translate(centroid.x, centroid.y);
}


void draw(){
  handleAgents(); 
  if (original_fig.grabsInput(main_scene.motionAgent()))
    original_fig.shape().setFill(color(255, 0, 0, 100));
  else {
    original_fig.shape().setFill(color(0, 0, 255, 100));
  }
  main_graphics.beginDraw();
  main_scene.beginDraw();
  main_graphics.background(0);
  original_fig.draw();
  deformed_fig.draw();
  // Save the current model view matrix
  pushMatrix();
  // Multiply matrix to get in the frame coordinate system.
  // applyMatrix(Scene.toPMatrix(iFrame.matrix())); //is possible but inefficient
  original_fig.applyTransformation();//very efficient  
  drawControlPoints(control_points, color(0,0,255));
  drawControlPoints(control_points,control_points_out);  
  popMatrix();
  main_scene.endDraw();
  main_graphics.endDraw();  
  image(main_graphics, main_scene.originCorner().x(), main_scene.originCorner().y());
  //if (showAid) {
    /*
    aux_graphics.beginDraw();
    aux_scene.beginDraw();
    aux_graphics.background(125, 125, 125, 125);
    aux_scene.drawModels();
    aux_scene.endDraw();
    aux_graphics.endDraw();    
    image(aux_graphics, aux_scene.originCorner().x(), aux_scene.originCorner().y());
    */
  //}
}

void handleAgents() {
  aux_scene.disableMotionAgent();
  aux_scene.disableKeyboardAgent();
  main_scene.disableMotionAgent();
  main_scene.disableKeyboardAgent();
  if(main_scene.height() > mouseY && drag_mode == -1) {
    main_scene.enableMotionAgent();
    main_scene.enableKeyboardAgent();
  }
  if ((mouseX >= all_width - aux_scene.width()) && mouseY >= all_heigth - aux_scene.height()) {
    aux_scene.enableMotionAgent();
    aux_scene.enableKeyboardAgent();
  }
}

  
  
  
