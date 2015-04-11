//Based on [Schaefer] : Image Deformation Using Moving Least Squares
double[][] A;
double[][] w;
//only use contour
void getA(ArrayList<PVector> img, ArrayList<PVector> control){
  A = new double[img.size()][control.size()];
  w = new double[img.size()][control.size()];
  int counter = 0;
  for(PVector v : img){
    double sum_weights = 0;
    PVector sum_weights_per_p = new PVector(0,0);
    PVector p_star;
    for(int k = 0; k < control.size(); k++){
        PVector pk = control.get(k); 
        double den = (PVector.dist(v, pk)*PVector.dist(v, pk));
        den = den < 0.00000000001 ? 0.00000000001 : den;
        w[counter][k] = 1/den;
        sum_weights += w[counter][k]; 
        sum_weights_per_p.x = sum_weights_per_p.x  + (float)w[counter][k]*pk.x;  
        sum_weights_per_p.y = sum_weights_per_p.y  + (float)w[counter][k]*pk.y;  
    }
    p_star = PVector.mult(sum_weights_per_p, 1.0/(float)sum_weights);
    PVector v_minus_p_s = PVector.sub(v,p_star);    
    for(int i = 0; i < control.size(); i++){
      double[][] pt_per_wp = new double[2][2]; 
      for(int j = 0; j < control.size(); j++){
        PVector pj = control.get(j); 
        PVector p_hat_j = PVector.sub(pj,p_star);
        pt_per_wp[0][0] += w[counter][j]*p_hat_j.x*p_hat_j.x;            
        pt_per_wp[0][1] += w[counter][j]*p_hat_j.x*p_hat_j.y;            
        pt_per_wp[1][0]  = pt_per_wp[0][1];            
        pt_per_wp[1][1] += w[counter][j]*p_hat_j.y*p_hat_j.y;            
      }   
      PVector pi = control.get(i); 
      PVector p_hat_i = PVector.sub(pi,p_star);
      //inverse
      double a = pt_per_wp[0][0];
      double b = pt_per_wp[0][1];
      double c = pt_per_wp[1][0];
      double d = pt_per_wp[1][1];
      double det_inv = 1/(a*d - b*c); 
      pt_per_wp[0][0] = det_inv* d;
      pt_per_wp[0][1] = -1*det_inv* b;
      pt_per_wp[1][0] = -1*det_inv* c;
      pt_per_wp[1][1] = det_inv* a;
      double[] Ai_1 = new double[2];
      Ai_1[0]= (v_minus_p_s.x * pt_per_wp[0][0]) + (v_minus_p_s.y * pt_per_wp[0][1]); 
      Ai_1[1]= (v_minus_p_s.x * pt_per_wp[1][0]) + (v_minus_p_s.y * pt_per_wp[1][1]); 
      A[counter][i] = Ai_1[0] * p_hat_i.x * w[counter][i] + Ai_1[1] * p_hat_i.y * w[counter][i];    
    }
    counter++;
  }
}

ArrayList<PVector> calculateNewImage(ArrayList<PVector> img, ArrayList<PVector> out_control){
  if(out_control.size() < 3) return img;
  /*for(int i = 0; i < A.length; i++){
    for(int j = 0; j < A[0].length; j++){
        print(A[i][j] + "  ");
    }
    println();
  }*/
  ArrayList<PVector> dest = new ArrayList<PVector>();
  int counter = 0;
  for(PVector v : img){
    double sum_weights = 0;
    PVector sum_weights_per_q = new PVector(0,0);
    PVector q_star;
    for(int k = 0; k < out_control.size(); k++){
        PVector qk = out_control.get(k); 
        sum_weights += w[counter][k]; 
        sum_weights_per_q.x = sum_weights_per_q.x  + (float)w[counter][k]*qk.x;  
        sum_weights_per_q.y = sum_weights_per_q.y  + (float)w[counter][k]*qk.y;  
    }
    q_star = PVector.mult(sum_weights_per_q, 1.0/(float)sum_weights);
    PVector sum_A_q_j = new PVector (0,0);
    for(int j = 0; j < out_control.size(); j++){
        PVector qj = out_control.get(j); 
        PVector q_hat_j = PVector.sub(qj,q_star);
        sum_A_q_j.x += A[counter][j]*q_hat_j.x;  
        sum_A_q_j.y += A[counter][j]*q_hat_j.y;  
    }
    PVector f_a_v = PVector.add(sum_A_q_j, q_star);
    dest.add(f_a_v);
    counter++;
  }
  return dest;
}

void updateControlPoints(){
  getA(deformed_world,control_points);
}

void updateControlPoints(ArrayList<PVector> img){
  getA(img,control_points);
}

//SOME PREDEFINED DEFORMATIONS
void addControlPointsAuto(boolean rand){
  //clear
  control_points.clear();
  control_points_out.clear();
  for(int i = 0; i < edges.size(); i+=step_per_point){
    //get coordinates in local frame
    //control_points.add(edges.get(i));
    PVector v = edges.get(i);
    PVector new_v = new PVector(v.x - centroid.x, v.y - centroid.y);                                          
    new_v.mult(random(1,2));
    new_v.add(v);
    control_points.add(new_v);
    if(!rand)control_points_out.add(edges.get(i));
    else{
      control_points_out.add(PVector.add(control_points.get(control_points.size()-1), 
                                            new PVector(random(-20,20),random(-20,20))));
    }
  }
}

void scaleW(boolean clear){
  //clear
  if(clear){
    control_points.clear();
    control_points_out.clear();
  }
  Rectangle r = contour.getBoundingBox();
  PVector top_left = new PVector(r.x, r.y); 
  PVector bottom_left = new PVector(r.x, r.y + r.height);
  PVector mid_right = new PVector(r.x + r.width, (float)r.getCenterY()); 
  PVector movement = new PVector((r.width/8)*randomGaussian(), 0);
  PVector mid_right_im = PVector.add(mid_right, movement);
  control_points.add(top_left);
  control_points.add(bottom_left);
  control_points.add(mid_right);
  control_points_out.add(top_left);
  control_points_out.add(bottom_left);
  control_points_out.add(mid_right_im);
}

void scaleH(boolean clear){
  //clear
  if(clear){
    control_points.clear();
    control_points_out.clear();
  }
  Rectangle r = contour.getBoundingBox();
  PVector top_left = new PVector(r.x, r.y); 
  PVector top_rigth = new PVector(r.x + r.width, r.y);
  PVector mid_bottom = new PVector((float)r.getCenterX(), r.y + r.height); 
  PVector movement = new PVector(0, (r.height/8)*randomGaussian());
  PVector mid_bottom_im = PVector.add(mid_bottom, movement);
  control_points.add(top_left);
  control_points.add(top_rigth);
  control_points.add(mid_bottom);
  control_points_out.add(top_left);
  control_points_out.add(top_rigth);
  control_points_out.add(mid_bottom_im);
}

//mode: 0 up, 1 bottom, 2 middle
void applyHorizontalSpline(int mode, boolean clear, boolean reflexive){
  //clear
  if(clear){
    control_points.clear();
    control_points_out.clear();
  }
  Rectangle r = contour.getBoundingBox();
  ArrayList<PVector> spline_control = new ArrayList<PVector>();
  int quantity = 16;
  float e = random(0.3,0.5); //get a new point for each 2 control points
  float t = 0;
  for(int i = 0; i < quantity; i++){
      float x_pos = r.x + i*(r.width/(quantity-1));
      float y_mode = mode == 0 ? r.y : mode == 1 ? r.y + r.height : (float) r.getCenterY(); 
      float y_pos = y_mode + random(-30, 30); 
      spline_control.add(new PVector(x_pos, y_pos));
  }
  spline_control = drawCurve(spline_control, t, e, false);
  //apply the same transformation to all the points
  float y_mode = mode == 0 ? r.y : mode == 1 ? r.y + r.height : (float) r.getCenterY(); 
  float y_pos = y_mode + random(-20, 20); 
  for(PVector point : spline_control){
    control_points.add(new PVector(point.x, y_pos));
  }
  control_points_out.addAll(spline_control);  
  if(!reflexive || mode == 2){
    //put a control point in the oposite side
    float anchor = mode == 0 ? r.y + r.height : r.y - 20;
    PVector mid = new PVector((float)r.getCenterX(), anchor); 
    control_points.add(mid);  
    control_points_out.add(mid);  
  }else{
    //put the same calculated points in the oposite place
    //apply the same transformation to all the points
    float inv_y_mode = mode == 1 ? r.y : mode == 0 ? r.y + r.height : (float) r.getCenterY();
    y_pos = -(y_pos - y_mode) + inv_y_mode; 
    for(int i = 0; i < spline_control.size(); i++){
      PVector point = new PVector(spline_control.get(i).x, spline_control.get(i).y);
      control_points.add(new PVector(point.x, y_pos));
      point.y = -(point.y - y_mode) + inv_y_mode;
      control_points_out.add(point);
    }
  }
}

//mode: 0 left, 1 right, 2 middle
void applyVerticalSpline(int mode, boolean clear, boolean reflexive){
  //clear
  if(clear){
    control_points.clear();
    control_points_out.clear();
  }
  Rectangle r = contour.getBoundingBox();
  ArrayList<PVector> spline_control = new ArrayList<PVector>();
  int quantity = 16;
  float e = random(0.3,0.5); //get a new point for each 2 control points
  float t = 0;
  for(int i = 0; i < quantity; i++){
      float y_pos = r.y + i*(r.height/(quantity-1));
      float x_mode = mode == 0 ? r.x : mode == 1 ? r.y + r.width : (float) r.getCenterX(); 
      float x_pos = x_mode + random(-30, 30);
      
      spline_control.add(new PVector(x_pos, y_pos));
  }
  spline_control = drawCurve(spline_control, t, e, false);
  //apply the same transformation to all the points
  float x_mode = mode == 0 ? r.x : mode == 1 ? r.x + r.width : (float) r.getCenterX(); 
  float x_pos = x_mode + random(-20, 20); 
  for(PVector point : spline_control){
    control_points.add(new PVector(x_pos, point.y));
  }
  control_points_out.addAll(spline_control);
  if(!reflexive || mode == 2){
    //put a control point in the oposite side
    float anchor = mode == 0 ? r.x + r.width : r.x - 20;
    PVector mid = new PVector(anchor,(float)r.getCenterY()); 
    control_points.add(mid);  
    control_points_out.add(mid);  
  }else{
    //put the same calculated points in the oposite place
    //apply the same transformation to all the points
    float inv_x_mode = mode == 1 ? r.x : mode == 0 ? r.x + r.width : (float) r.getCenterX();
    x_pos = -(x_pos - x_mode) + inv_x_mode; 
    for(int i = 0; i < spline_control.size(); i++){
      PVector point = new PVector(spline_control.get(i).x, spline_control.get(i).y);
      control_points.add(new PVector(x_pos, point.y));
      point.x = -(point.x - x_mode) + inv_x_mode;
      control_points_out.add(point);
    }
  }
}

void combination(boolean reflex){
  ArrayList<PVector> new_img = new ArrayList<PVector>();
  new_img.addAll(edges);
  //splineht
  applyHorizontalSpline(1,true, reflex);
  updateControlPoints(new_img);
  new_img = calculateNewImage(new_img,control_points_out);
  //scalehb
  /*scaleH(true);
  applyHorizontalSpline(1,true);
  updateControlPoints(new_img);
  new_img = calculateNewImage(new_img,control_points_out);*/
  //splinevl
  applyVerticalSpline(1,true,reflex);
  updateControlPoints(new_img);
  new_img = calculateNewImage(new_img,control_points_out);
  //scalevr
  /*scaleH(true);
  applyVerticalSpline(1,true);
  updateControlPoints(new_img);*/

  //scalew
  scaleW(true);
  updateControlPoints(new_img);
  new_img = calculateNewImage(new_img,control_points_out);
  //scaleh
  scaleH(true);
  updateControlPoints(new_img);
  new_img = calculateNewImage(new_img,control_points_out);

  new_img = calculateNewImage(new_img,control_points_out);
  //modify the shape
  deformed_edges = new_img;
  deformed_figure = getContours(deformed_edges, color(217,245,12,100));
  //join to the model
  deformed_fig.setShape(deformed_figure);
}

void combinationR(boolean reflex){
  ArrayList<PVector> new_img = new ArrayList<PVector>();
  new_img.addAll(edges);
  //splineht
  applyHorizontalSpline(0,true, reflex);
  updateControlPoints(new_img);
  new_img = calculateNewImage(new_img,control_points_out);
  //scalehb
  /*scaleH(true);
  applyHorizontalSpline(1,true);
  updateControlPoints(new_img);
  new_img = calculateNewImage(new_img,control_points_out);*/
  //splinevl
  applyVerticalSpline(0,true,reflex);
  updateControlPoints(new_img);
  new_img = calculateNewImage(new_img,control_points_out);
  //scalevr
  /*scaleH(true);
  applyVerticalSpline(1,true);
  updateControlPoints(new_img);*/

  //scalew
  scaleW(true);
  updateControlPoints(new_img);
  new_img = calculateNewImage(new_img,control_points_out);
  //scaleh
  scaleH(true);
  updateControlPoints(new_img);
  new_img = calculateNewImage(new_img,control_points_out);

  new_img = calculateNewImage(new_img,control_points_out);
  //modify the shape
  deformed_edges = new_img;
  deformed_figure = getContours(deformed_edges, color(217,245,12,100));
  //join to the model
  deformed_fig.setShape(deformed_figure);
}
