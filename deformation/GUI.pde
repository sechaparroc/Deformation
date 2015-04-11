//PARAMS--------------------------
int MLS = 0;
int SPLINES = 1;
int morph_method = 0;
float lambda = 0;
boolean change_points = true;
int step_per_point = 10;
boolean clear_points = true;
//--------------------------------------------------
//HANDLE MOUSE EVENTS & CONTROL POINTS--------------
void morphTransformationAction(){
  if(morph_method == SPLINES){
      //draw natural spline
      //curve_natural = drawCurve(control_points_out, lambda, 0.1);
      change_points = true;
  }else if(morph_method == MLS){
      //first modify the world
      if(world_modified){
        deformed_world = new ArrayList<PVector>();
        deformed_world.addAll(edges);
        getA(deformed_world, world_control_points);
        deformed_world = calculateNewImage(edges,world_control_points_out);
        //modify the shape
        updateControlPoints(deformed_world);
        //create the world shape in the main scene
        world_shape = getContours(world_control_points, color(0,255,0,150));
        deformed_world_shape = getContours(world_control_points_out, color(255,0,0,100));
        world_modified = false;
        //modify the shape
        deformed_world_figure = getContours(deformed_world, color(21,245,217,100));
        r_deformed_world_figure = getBoundingBox(deformed_world);
        deformed_world_fig.setShape(deformed_world_figure);
        //println(r_deformed_world_figure);
      }
      deformed_edges = calculateNewImage(deformed_world,control_points_out);
      //modify the shape
      deformed_figure = getContours(deformed_edges, color(217,245,12,100));
      r_deformed_figure = getBoundingBox(deformed_edges);
      //join to the model
      deformed_fig.setShape(deformed_figure);
  }
}


int drag_mode = -1;

void addPoint(PVector v){
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
  updateControlPoints();
}

void mousePressed( ){
  println("coord : " + mouseX + " Y: " + mouseY);
  if((mouseX >= all_width - aux_scene.width()) && mouseY >= all_heigth - aux_scene.height()) return;
  //get coordinates in world
  Vec point_world = main_scene.eye().unprojectedCoordinatesOf(new Vec(mouseX, mouseY));
  //get corrdinates in local frame
  Vec point_shape = deformed_world_fig.coordinatesOf(point_world);
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
      morphTransformationAction();
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
        updateControlPoints();
        morphTransformationAction();
      }
    //}
  }
}

void mouseReleased(){
  drag_mode = -1;
}

void mouseDragged(){
  //get coordinates in world
  Vec point_world = main_scene.eye().unprojectedCoordinatesOf(new Vec(mouseX, mouseY));
  //get corrdinates in local frame
  Vec point_shape = deformed_world_fig.coordinatesOf(point_world);
  //println("drag : " + drag_mode);
  if(drag_mode != -1){
    control_points_out.set(drag_mode, new PVector(point_shape.x(), point_shape.y()));
    morphTransformationAction();
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
  if(Math.abs( p.x - x ) <= 5){
    if(Math.abs( p.y - y ) <= 5){
      return true;
    }
  }
  return false;
}

int getControlPoint(float x, float y){
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
  p.strokeWeight(5);  
  //get coordinates in local frame
  //println(r_deformed_world_figure);
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
void keyPressed(){
  if (key == ' '){
    showAid = !showAid;
  }
  if(key == 'r' || key == 'R'){
    reflex = !reflex;
  }
  if(key == 'c' || key == 'C'){
    clear_points = !clear_points;
  }
  if(key == 'v' || key == 'V'){
    step_per_point = edges.size()/((int)random(15,30) + 1);
    addControlPointsAuto(true);
    updateControlPoints();
    morphTransformationAction();
  }
  //predefined transformation
  //scale w
  if(key=='1'){
    scaleW(clear_points);
    updateControlPoints();
    morphTransformationAction();
  }
  //scale h  
  if(key=='2'){
    scaleH(clear_points);
    updateControlPoints();
    morphTransformationAction();
  }
  //spline horizontal t
  if(key=='3'){
    applyHorizontalSpline(0,clear_points, reflex);
    updateControlPoints();
    morphTransformationAction();
  }
  //spline horizontal m
  if(key=='4'){
    applyHorizontalSpline(2,clear_points, false);
    updateControlPoints();
    morphTransformationAction();
  }
  //spline horizontal b
  if(key=='5'){
    applyHorizontalSpline(1,clear_points, reflex);
    updateControlPoints();
    morphTransformationAction();
  }
  //spline vertical l
  if(key=='6'){
    applyVerticalSpline(0,clear_points,reflex);
    updateControlPoints();
    morphTransformationAction();
  }
  //spline vertical m
  if(key=='7'){
    applyVerticalSpline(2,clear_points, false);
    updateControlPoints();
    morphTransformationAction();
  }
  //spline vertical r
  if(key=='8'){
    applyVerticalSpline(1,clear_points, reflex);
    updateControlPoints();
    morphTransformationAction();
  }
  //combination
  if(key=='9'){
    combination(reflex);
  }
  //combination r
  if(key=='0'){
    combination(reflex);
  }
}
