import java.util.Random;
//use preprocessing algorithms (find contours, threshold)
import gab.opencv.*;

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
PShape figure;
PShape deformed_figure;
PShape deformed_world_figure;
Rectangle r_figure;
Rectangle r_deformed_figure;
Rectangle r_deformed_world_figure;
PShape world_shape;
PShape deformed_world_shape;
ArrayList<Contour> contours        = new ArrayList<Contour>();
Contour contour;
ArrayList<PVector> edges           = new ArrayList<PVector>();  
ArrayList<PVector> deformed_edges  = new ArrayList<PVector>();  
ArrayList<PVector> deformed_world  = null;
//---------------------------------------------

//TEXTURE VARS IF IS REQUIRED------------------
PImage last_texture = null;
//---------------------------------------------

//CONTROL POINTS-------------------------------
ArrayList<PVector> control_points     = new ArrayList<PVector>(); 
ArrayList<PVector> control_points_out = new ArrayList<PVector>(); 
ArrayList<PVector> world_control_points     = new ArrayList<PVector>(); 
ArrayList<PVector> world_control_points_out = new ArrayList<PVector>(); 
boolean world_modified = true;
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
InteractiveModelFrame deformed_world_fig;
InteractiveModelFrame deformed_fig;
ModelContour control_shape;
boolean showAid = true;

//Class that overrides iModelFrame to customize actions
//left click & drag to control the shape
//right click remove all
//scroll more or less detail
public class ModelContour extends InteractiveModelFrame{
  float radius = 40;
  float detail = 3;
  ArrayList<PVector> points = new ArrayList<PVector>();

  public ModelContour(Scene sc){
    super(sc);
    detail = 3;
    points = new ArrayList<PVector>();
    points = setInitialPoints(radius);
    update();
  }

  //scroll action will increase or decrease the detail of the shape
  @Override
  public void performCustomAction(DOF1Event event) {
    println("cambio : " + radius);
    detail += event.x();       
    if(detail < 3){ 
      detail=3;
    }else{
      points = setInitialPoints(radius);
      update();
    }
  }
  @Override
  public void performCustomAction(DOF2Event event) {    
    if( event.id() != MotionEvent.NO_ID ) {
      Vec point_world = aux_scene.eye().unprojectedCoordinatesOf(new Vec(event.x(), event.y()));
      Vec point_shape = coordinatesOf(point_world);
      Vec change = transformOf(aux_scene.eye().frame().inverseTransformOf(new Vec(event.dx(), event.dy())));
      PVector point = getPoint(point_shape.x(), point_shape.y()); 
      point.x += change.x(); 
      point.y += change.y();
      update();
    }
  }
  @Override
  public void performCustomAction(ClickEvent event) {
    /*draw the shape in the main scene*/
    Rectangle r = getBoundingBox(edges);
    PVector centroid = new PVector(r.getCenterX(), r.getCenterY()); 
    float prev_radius = radius;    
    radius = max(r.w, r.h)/2.0;
    world_control_points = setInitialPoints(radius,centroid, true);
    world_control_points_out.clear();
    for(PVector p : points){
      Vec v = new Vec(p.x*radius/prev_radius,p.y*radius/prev_radius);
      v = inverseTransformOf(v);
      v = new Vec(v.x() + centroid.x,v.y() + centroid.y); 
      world_control_points_out.add(new PVector(v.x(),v.y()));
    }
    this.radius = prev_radius;
    world_modified = true;   
    morphTransformationAction();

    //update();
  }
  void update(){
    PShape shape_temp = createShape();
    shape_temp.beginShape();    
    shape_temp.stroke(255,255,255); 
    for(PVector v : points){
      shape_temp.vertex(v.x,v.y);
    }
    shape_temp.endShape(CLOSE);    
    setShape(shape_temp);
    updateColor();
  }

  void updateColor() {
    shape().setFill(color(255,0,0,100));
  }
  
  ArrayList<PVector> setInitialPoints(float radius){
    return setInitialPoints(radius,new PVector(0,0), false);
  }

  ArrayList<PVector> setInitialPoints(float radius, PVector centroid, boolean check_rot){
    ArrayList<PVector> points = new ArrayList<PVector>();  
    float step = 360/detail;
    float theta = 0;
    float poly_angle = radians(step/2.0);
    float phi = 90;
    float poly_rad = sqrt(radius*radius + radius*tan(poly_angle)*radius*tan(poly_angle));
    while(theta < 360){
      Vec v = new Vec(poly_rad*cos(radians(theta - phi)) + centroid.x,poly_rad*sin(radians(theta - phi))+ centroid.y);
      if(check_rot){
        v = new Vec(poly_rad*cos(radians(theta - phi)),poly_rad*sin(radians(theta - phi)));
        v = inverseTransformOf(v);
        v = new Vec(v.x() + centroid.x,v.y() + centroid.y); 
      }
      points.add(new PVector(v.x(),v.y()));    
      theta += step;
    }
    return points;
  }

  PVector getPoint(float x, float y){
    PVector min = new PVector(9999,9999);
    PVector ref = new PVector(x,y);
    for(int i = 0; i < points.size(); i++){
      PVector p = points.get(i);
      if(PVector.dist(min,ref) > PVector.dist(p,ref)) min = p;
    }
    return min;
  }
}



void setup(){
  //load the image
  source_image = loadImage("test2.png");  
  source_image.resize(0,150);   
  size(all_width, all_heigth, P2D);
  aux_pos_x =  width-all_width/4;
  aux_pos_y =  height-all_heigth/3;
  
  main_graphics = createGraphics(all_width,all_heigth,P2D);
  main_scene = new Scene(this, main_graphics);
  aux_graphics = createGraphics(all_width/4,all_heigth/3,P2D);
  aux_scene = new Scene(this, aux_graphics, aux_pos_x, aux_pos_y);    
  main_scene.setAxesVisualHint(false);
  main_scene.setGridVisualHint(false);
  aux_scene.setAxesVisualHint(true);
  aux_scene.setGridVisualHint(true);
  main_scene.setRadius(50);
  //get the countours
  figure = getCountours();
  deformed_world = new ArrayList<PVector>();
  deformed_world.addAll(edges);
  //save util info (bounding box) in a rectangle class
  r_figure = getBoundingBox(edges);
  r_deformed_figure = getBoundingBox(edges);
  r_deformed_world_figure = getBoundingBox(edges);
  //println(r_figure);
  //associate the shape with the original shape frame
  original_fig = new InteractiveModelFrame(main_scene, figure);
  original_fig.translate(-2*r_figure.getCenterX(),-r_figure.getCenterY());
  original_fig.scale(0.5);
  //initial deformed shape without modifications
  deformed_world_fig = new InteractiveModelFrame(main_scene, figure);
  deformed_world_fig.translate(0,-r_figure.getCenterY());
  deformed_world_fig.scale(0.5);
  //initial deformed shape without modifications
  deformed_fig = new InteractiveModelFrame(main_scene, figure);
  deformed_fig.translate(2*r_figure.getCenterX(),-r_figure.getCenterY());
  deformed_fig.scale(0.5);
  //control window
  morphTransformationAction();
  
  control_shape = new ModelContour(aux_scene);
  aux_scene.mouseAgent().setButtonBinding(Target.FRAME, LEFT, DOF2Action.CUSTOM);
  aux_scene.mouseAgent().setButtonBinding(Target.FRAME, RIGHT, DOF2Action.ROTATE);
  aux_scene.mouseAgent().setButtonBinding(Target.FRAME, CENTER, DOF2Action.CUSTOM);
  aux_scene.mouseAgent().setWheelBinding(Target.FRAME, DOF1Action.CUSTOM);
  aux_scene.mouseAgent().setClickBinding(Target.FRAME, RIGHT, ClickAction.CUSTOM);
  aux_scene.mouseAgent().setClickBinding(Target.FRAME, LEFT, ClickAction.CUSTOM);
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
  deformed_world_fig.draw();
  deformed_fig.draw();

  // Save the current model view matrix
  main_graphics.pushMatrix();
  // Multiply matrix to get in the frame coordinate system.
  // applyMatrix(Scene.toPMatrix(iFrame.matrix())); //is possible but inefficient
  original_fig.applyTransformation();//very efficient  
  if(world_shape != null)main_graphics.shape(world_shape);
  if(deformed_world_shape != null)main_graphics.shape(deformed_world_shape);
  //drawControlPoints(control_points, color(0,0,255));
  //drawControlPoints(control_points,control_points_out);  
  main_graphics.popMatrix();
  main_graphics.pushMatrix();
  deformed_world_fig.applyTransformation();//very efficient  
  drawControlPoints(control_points, color(0,0,255));
  drawControlPoints(control_points,control_points_out);  
  main_graphics.popMatrix();

  main_scene.endDraw();
  main_graphics.endDraw();  


  image(main_graphics, main_scene.originCorner().x(), main_scene.originCorner().y());
  if (showAid) {
    aux_graphics.beginDraw();
    aux_scene.beginDraw();
    aux_graphics.background(125, 125, 125, 125);
    aux_scene.drawModels();
    //draw the points
    aux_graphics.pushStyle();
    aux_graphics.strokeWeight(5);
    aux_graphics.stroke(color(0,255,0));  
    aux_graphics.pushMatrix();
    control_shape.applyTransformation();//very efficient  
    for(PVector point : control_shape.points){
      aux_graphics.point(point.x,point.y);
    }
    aux_graphics.popMatrix();
    aux_graphics.popStyle();
    aux_scene.endDraw();
    aux_graphics.endDraw();    
    image(aux_graphics, aux_scene.originCorner().x(), aux_scene.originCorner().y());
  }
}

void handleAgents() {
  aux_scene.disableMotionAgent();
  aux_scene.disableKeyboardAgent();
  main_scene.disableMotionAgent();
  main_scene.disableKeyboardAgent();
  if ((mouseX >= all_width - aux_scene.width()) && mouseY >= all_heigth - aux_scene.height()) {
    aux_scene.enableMotionAgent();
    aux_scene.enableKeyboardAgent();
  }else if(drag_mode == -1) {
    main_scene.enableMotionAgent();
    main_scene.enableKeyboardAgent();
  }
}

  
  
  
