import papaya.*;
import processing.core.*;

import java.util.*;

//use preprocessing algorithms (find contours, threshold)
import gab.opencv.*;
//-----------------------------------------------
//Proscene
//Use InteractiveModelFrame and override actions
import remixlab.bias.core.*;
import remixlab.bias.event.*;
import remixlab.proscene.*;
import remixlab.dandelion.geom.*;
import remixlab.dandelion.core.*;
//-----------------------------------------------
/*Sebastian Chaparro
William Rpdriguez   
April 2 2015
*/  
//First Step get the image, and it contour
//For this purpose we're gonna use OpenCV

public class Deformation extends PApplet{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	public static PImage source_image;
	public static PImage destination_image;
	
	//VARS TO FIND CONTOURS------------------------
	public static PShape figure;
	public static PShape deformed_figure;
	public static PShape deformed_world_figure;
	public static Utilities.Rectangle r_figure;
	public static Utilities.Rectangle r_deformed_figure;
	public static Utilities.Rectangle r_deformed_world_figure;
	public static PShape world_shape;
	public static PShape deformed_world_shape;
	public static ArrayList<PVector> edges           = new ArrayList<PVector>();  
	public static ArrayList<PVector> deformed_edges  = new ArrayList<PVector>();  
	public static ArrayList<PVector> deformed_world  = null;
	//---------------------------------------------
	//CONTROL POINTS-------------------------------
	public static ArrayList<PVector> control_points     = new ArrayList<PVector>(); 
	public static ArrayList<PVector> control_points_out = new ArrayList<PVector>(); 
	public static ArrayList<PVector> laplacian_control_points     = new ArrayList<PVector>(); 
	public static ArrayList<PVector> laplacian_control_points_out = new ArrayList<PVector>(); 
	public static ArrayList<PVector> world_control_points     = new ArrayList<PVector>(); 
	public static ArrayList<PVector> world_control_points_out = new ArrayList<PVector>(); 
	public static boolean world_modified = true;	
	//---------------------------------------------
	
	//PGRAPHICS------------------------------------
	/*we're gonna use 3: the first one for display the original
	object and it's deformation, the second one is used to GUI
	options, the third one is used with proscene to show the 
	usefulness of InteractiveObject Classes*/
	PGraphics main_graphics;
	PGraphics aux_graphics;
	PGraphics gui_graphics;
	public static int all_width  = 600;
	public static int all_heigth = 620;
	public static int radius = 10;
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
	public static InteractiveFrame original_fig;
	public static InteractiveFrame deformed_world_fig;
	public static InteractiveFrame deformed_fig;
	public static ModelContour control_shape;
	boolean showAid = true;
	
	//Change between Laplacian Deformation or MLS
	public static boolean laplacian_mode = false;
	
	
	//Class that overrides iModelFrame to customize actions
	//left click & drag to control the shape
	//right click remove all
	//scroll more or less detail
	public class ModelContour extends InteractiveFrame{
	  float radius = 40;
	  float detail = 3;
	  ArrayList<PVector> points = new ArrayList<PVector>();
	
	  public ModelContour(Scene sc){
	    super(sc);
	    detail = 3;
	    points = new ArrayList<PVector>();
	    points = setInitialPoints(radius);
	    updateShape();
	  }
	
	  //scroll action will increase or decrease the detail of the shape
	  public void increaseDetail(DOF1Event event) {
	    System.out.println("cambio : " + radius);
	    detail += event.dx();       
	    if(detail < 3){ 
	      detail=3;
	    }else{
	      points = setInitialPoints(radius);
	      updateShape();
	    }
	  }
	  public void translatePoint(DOF2Event event) {    
	    if( event.id() != MotionEvent.NO_ID ) {
	      Vec point_world = aux_scene.eye().unprojectedCoordinatesOf(new Vec(event.x(), event.y()));
	      Vec point_shape = coordinatesOf(point_world);
	      Vec change = transformOf(aux_scene.eye().frame().inverseTransformOf(new Vec(event.dx(), event.dy())));
	      PVector point = getPoint(point_shape.x(), point_shape.y()); 
	      point.x += change.x(); 
	      point.y += change.y();
	      updateShape();
	    }
	  }
	  public void resetPoints(ClickEvent event) {
	    /*draw the shape in the main scene*/
		Utilities.Rectangle r = Utilities.getBoundingBox(edges);
	    PVector centroid = new PVector(r.getCenterX(), r.getCenterY()); 
	    float prev_radius = radius;    
	    radius = (float) (max(r.w, r.h)/2.0);
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
	    morphTransformationAction(laplacian_mode);
	
	    //update();
	  }
	  void updateShape(){
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
	    float poly_angle = radians((float) (step/2.0));
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
	
	/*Each time the deformed figure is modified so the laplacian deformation attributes*/
	public static void setDeformedFigure(PShape shape){
		deformed_figure = shape;
	    LaplacianDeformation.setup(deformed_figure);
	    LaplacianDeformation.calculateLaplacian();		  
	}
	
	public static ArrayList<PVector> getControlPoints(){
		return laplacian_mode == true ? laplacian_control_points : control_points; 
	}

	public static ArrayList<PVector> getControlPointsOut(){
		return laplacian_mode == true ? laplacian_control_points_out : control_points_out; 		
	}
	
	static public void main(String[] args) {
		PApplet.main(Deformation.class.getName());
	}	

	public void settings(){
		size(600, 620, P2D);		
	}
	
	public void setup(){
	  //load the image
	  source_image = loadImage("human.png");  
	  source_image.resize(0,150);   
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
	  Utilities.p = this;
	  figure = Utilities.getCountours();
	  deformed_world = new ArrayList<PVector>();
	  deformed_world.addAll(edges);
	  //save util info (bounding box) in a rectangle class
	  r_figure = Utilities.getBoundingBox(edges);
	  r_deformed_figure = Utilities.getBoundingBox(edges);
	  r_deformed_world_figure = Utilities.getBoundingBox(edges);
	  //System.out.println(r_figure);
	  //associate the shape with the original shape frame
	  original_fig = new InteractiveFrame(main_scene, figure);
	  original_fig.translate(-2*r_figure.getCenterX(),-r_figure.getCenterY());
	  original_fig.scale((float) 0.5);
	  //initial deformed shape without modifications
	  deformed_world_fig = new InteractiveFrame(main_scene, figure);
	  deformed_world_fig.translate(0,-r_figure.getCenterY());
	  deformed_world_fig.scale((float) 0.5);
	  //initial deformed shape without modifications
	  deformed_fig = new InteractiveFrame(main_scene, figure);
	  deformed_fig.translate(2*r_figure.getCenterX(),-r_figure.getCenterY());
	  deformed_fig.scale((float) 0.5);
	  //control window
	  morphTransformationAction(laplacian_mode);
	  
	  control_shape = new ModelContour(aux_scene);
	  control_shape.setClickBinding(LEFT, 1, "resetPoints");
	  control_shape.setMotionBinding(MouseAgent.WHEEL_ID, "increaseDetail");
	  control_shape.setMotionBinding(RIGHT, "translatePoint");
	}
	
	
	public void draw(){
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
	  InteractiveFrame model = laplacian_mode == true ? deformed_fig : deformed_world_fig;	  
	  model.applyTransformation();//very efficient  
	  drawControlPoints(getControlPoints(), color(0,0,255));
	  drawControlPoints(getControlPoints(),getControlPointsOut());  
	  main_graphics.popMatrix();
	
	  main_scene.endDraw();
	  main_graphics.endDraw();  
	
	
	  image(main_graphics, main_scene.originCorner().x(), main_scene.originCorner().y());
	  if (showAid) {
	    aux_graphics.beginDraw();
	    aux_scene.beginDraw();
	    aux_graphics.background(125, 125, 125, 125);
	    aux_scene.drawFrames();
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

	//----------------------------------------------------------------
	//GUI METHODS-----------------------------------------------------
	//----------------------------------------------------------------
	//----------------------------------------------------------------
	//PARAMS--------------------------
	public static int ML = 0;
	public static int SPLINES = 1;
	public static int morph_method = 0;
	public static float lambda = 0;
	public static boolean change_points = true;
	public static int step_per_point = 10;
	public static boolean clear_points = true;
	//--------------------------------------------------
	//HANDLE MOUSE EVENTS & CONTROL POINTS--------------
	public static void morphTransformationAction(boolean laplacian_mode){
	  if(laplacian_mode == true){
		  LaplacianDeformation.addAnchors(laplacian_control_points,true);
		  ArrayList<PVector> new_img = LaplacianDeformation.solveLaplacian();
		  //modify the shape
		  Deformation.deformed_edges = new_img;
		  Deformation.deformed_figure = Utilities.getContours(Deformation.deformed_edges, 217,245,12,100);
		  //join to the model
		  Deformation.deformed_fig.setShape(Deformation.deformed_figure);		  
		  //draw natural spline
	      //curve_natural = drawCurve(control_points_out, lambda, 0.1);
	      //change_points = true;
	  }else if(morph_method == ML){
	      //first modify the world
	      if(world_modified){
	        deformed_world = new ArrayList<PVector>();
	        deformed_world.addAll(edges);
	        MLS.getA(deformed_world, world_control_points);
	        deformed_world = MLS.calculateNewImage(edges,world_control_points_out);
	        //modify the shape
	        MLS.updateControlPoints(deformed_world);
	        //create the world shape in the main scene
	        world_shape = Utilities.getContours(world_control_points, 0,255,0,150);
	        deformed_world_shape = Utilities.getContours(world_control_points_out, 255,0,0,100);
	        world_modified = false;
	        //modify the shape
	        deformed_world_figure = Utilities.getContours(deformed_world, 21,245,217,100);
	        r_deformed_world_figure = Utilities.getBoundingBox(deformed_world);
	        deformed_world_fig.setShape(deformed_world_figure);
	        //System.out.println(r_deformed_world_figure);
	      }
	      deformed_edges = MLS.calculateNewImage(deformed_world,control_points_out);
	      //modify the shape
	      setDeformedFigure(Utilities.getContours(deformed_edges, 217,245,12,100));
	      r_deformed_figure = Utilities.getBoundingBox(deformed_edges);
	      //join to the model
	      deformed_fig.setShape(deformed_figure);
	  }
	}


	public static int drag_mode = -1;

	void addPoint(PVector v){
	  ArrayList<PVector> control_points = getControlPoints();
	  ArrayList<PVector> control_points_out = getControlPointsOut();
	  if(control_points.size() == 0){
	    control_points.add(v);
	    control_points_out.add(v);
	    return;
	  }
	  float min_dist_l = 9999;
	  int best_pos = control_points.size();
	  for(int i = 0; i < control_points.size(); i++){
	      PVector left = control_points.get(i);
	      if(v.dist(left) < min_dist_l){
	        min_dist_l = v.dist(left);
	        best_pos = i+1;        
	      }      
	  }
	  control_points.add(best_pos,v);
	  control_points_out.add(best_pos,v);
	  //update A
	  if(laplacian_mode == false)MLS.updateControlPoints();
	}

	public void mousePressed( ){
	  System.out.println("coord : " + mouseX + " Y: " + mouseY);
	  ArrayList<PVector> control_points = getControlPoints();
	  ArrayList<PVector> control_points_out = getControlPointsOut();	  
	  if((mouseX >= all_width - aux_scene.width()) && mouseY >= all_heigth - aux_scene.height()) return;
	  //get coordinates in world
	  Vec point_world = main_scene.eye().unprojectedCoordinatesOf(new Vec(mouseX, mouseY));
	  //get corrdinates in local frame
	  InteractiveFrame model = laplacian_mode == true ? deformed_fig : deformed_world_fig;
	  Vec point_shape = model.coordinatesOf(point_world);
	  //int point = getPoint(v.x, v.y);
	  if(mouseButton == LEFT){
	    int pos = getControlPoint(point_shape.x(),point_shape.y());
	    if(pos == -1){
	      //calculate the point in the screen 
	      //just in case do a method to not allow near points
	      //add the point between the 2 nearest neightbors
	      //control_points.add(v);
	      //control_points_out.add(v);
	      addPoint(new PVector(point_shape.x(), point_shape.y()));
	      morphTransformationAction(laplacian_mode);
	    }else{
	      drag_mode = pos;
	    }
	  }
	  else if(mouseButton == RIGHT){
	    //remove a point
	    //if(point != -1){
	      int pos = getControlPoint(point_shape.x(), point_shape.y());
	      if(pos != -1){
	        control_points.remove(pos);
	        control_points_out.remove(pos);
	        //update A
	        if(laplacian_mode == false)MLS.updateControlPoints();
	        morphTransformationAction(laplacian_mode);
	      }
	    //}
	  }
	}

	public void mouseReleased(){
	  drag_mode = -1;
	}

	public void mouseDragged(){
	  ArrayList<PVector> control_points = getControlPoints();
	  ArrayList<PVector> control_points_out = getControlPointsOut();
	  //get coordinates in world
	  Vec point_world = main_scene.eye().unprojectedCoordinatesOf(new Vec(mouseX, mouseY));
	  //get corrdinates in local frame
	  InteractiveFrame model = laplacian_mode == true ? deformed_fig : deformed_world_fig;	  
	  Vec point_shape = model.coordinatesOf(point_world);
	  //System.out.println("drag : " + drag_mode);
	  if(drag_mode != -1){
	    control_points_out.set(drag_mode, new PVector(point_shape.x(), point_shape.y()));
	    morphTransformationAction(laplacian_mode);
	  }
	}

	int getPoint(float x, float y){
	  for(int i = 0; i < edges.size(); i++){
	    PVector p = edges.get(i);
	    if(isInside(p,x,y)) return i;
	  }
	  return -1;
	}

	boolean isInside(PVector p, float x, float y){
	  if(Math.abs( p.x - x ) <= radius){
	    if(Math.abs( p.y - y ) <= radius){
	      return true;
	    }
	  }
	  return false;
	}

	int getControlPoint(float x, float y){
	  ArrayList<PVector> control_points = getControlPoints();
	  for(int i = 0; i < control_points.size(); i++){
	    PVector p = control_points.get(i);
	    if(isInside(p,x,y)) return i;
	  }
	  return -1;
	}

	void drawControlPoints(ArrayList<PVector> control_points){
	  drawControlPoints(control_points, color(0,255,0));
	}

	void drawControlPoints(ArrayList<PVector> control_points, int col){
	  PGraphics p = main_scene.pg();
	  p.pushStyle();
	  p.strokeWeight(5);
	  p.stroke(0,255,0);
	  //get coordinates in local frame
	  p.point(r_deformed_world_figure.getCenterX(),r_deformed_world_figure.getCenterY());
	  p.stroke(col);
	  for(PVector point : control_points){
	    p.point(point.x,point.y);
	  }  
	  p.popStyle();
	  change_points = false;
	}

	void drawControlPoints(ArrayList<PVector> control_points, ArrayList<PVector> control_points_out){
	  PGraphics p = main_scene.pg();
	  p.pushStyle();
	  p.stroke(0,200,140);
	  p.strokeWeight(radius);  
	  //get coordinates in local frame
	  //System.out.println(r_deformed_world_figure);
	  //p.point(r_deformed_world_figure.getCenterX(),r_deformed_world_figure.getCenterY());
	  for(int i = 0; i < control_points.size(); i++){
	    p.strokeWeight(1);
	    p.stroke(255,255,255);
	    p.line(control_points_out.get(i).x,control_points_out.get(i).y,control_points.get(i).x,control_points.get(i).y);
	    p.strokeWeight(5);
	    p.stroke(0,0,255);
	    p.point(control_points.get(i).x,control_points.get(i).y);
	    p.stroke(0,255,0);
	    p.point(control_points_out.get(i).x,control_points_out.get(i).y);
	  }  
	  p.popStyle();
	  change_points = false;
	}


	//KEYBOARD HANDLING EVENTS
	boolean reflex = false;
	public void keyPressed(){
	  if (key == ' '){
	    showAid = !showAid;
	  }
	  if(key == 'r' || key == 'R'){
	    reflex = !reflex;
	  }
	  if(key == 'c' || key == 'C'){
		ArrayList<PVector> control_points = getControlPoints();
		ArrayList<PVector> control_points_out = getControlPointsOut();		  
	    clear_points = !clear_points;
	    control_points.clear();
	    control_points_out.clear();   
	  }
	  if(key == 'v' || key == 'V'){
	    step_per_point = edges.size()/((int)random(15,30) + 1);
	    MLS.addControlPointsAuto(true);
	    MLS.updateControlPoints();
	    morphTransformationAction(false);
	  }
	  if(key == 'x' || key == 'X'){
	    step_per_point = edges.size()/((int)random(15,30) + 1);
	    MLS.addControlPointsAuto(false);
	    MLS.updateControlPoints();
	    morphTransformationAction(false);
	  }

	  //predefined transformation
	  //scale w
	  if(key=='1'){
		MLS.scaleW(clear_points);
		MLS.updateControlPoints();
	    morphTransformationAction(false);
	  }
	  //scale h  
	  if(key=='2'){
		MLS.scaleH(clear_points);
		MLS.updateControlPoints();
	    morphTransformationAction(false);
	  }
	  //spline horizontal t
	  if(key=='3'){
		MLS.applyHorizontalSpline(0,clear_points, reflex);
		MLS.updateControlPoints();
	    morphTransformationAction(false);
	  }
	  //spline horizontal m
	  if(key=='4'){
		MLS.applyHorizontalSpline(2,clear_points, false);
		MLS.updateControlPoints();
	    morphTransformationAction(false);
	  }
	  //spline horizontal b
	  if(key=='5'){
		MLS.applyHorizontalSpline(1,clear_points, reflex);
		MLS.updateControlPoints();
	    morphTransformationAction(false);
	  }
	  //spline vertical l
	  if(key=='6'){
		MLS.applyVerticalSpline(0,clear_points,reflex);
		MLS.updateControlPoints();
	    morphTransformationAction(false);
	  }
	  //spline vertical m
	  if(key=='7'){
		MLS.applyVerticalSpline(2,clear_points, false);
		MLS.updateControlPoints();
	    morphTransformationAction(false);
	  }
	  //spline vertical r
	  if(key=='8'){
		MLS.applyVerticalSpline(1,clear_points, reflex);
		MLS.updateControlPoints();
	    morphTransformationAction(false);
	  }
	  //combination
	  if(key=='9'){
		MLS.combination(reflex);
	  }
	  //combination r
	  if(key=='0'){
		MLS.combination(reflex);
	  }
	  
	  if(key=='l' || key=='L'){
		  laplacian_mode = !laplacian_mode;
	  }
	}
}