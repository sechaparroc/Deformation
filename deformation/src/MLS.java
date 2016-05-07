import java.util.*;
import papaya.*;
import processing.core.*;


public class MLS {
	//Based on [Schaefer] : Image Deformation Using Moving Least Squares
	public static double[][] A;
	public static double[][] w;
	public static Random rand = new Random();

	//only use contour
	public static void getA(ArrayList<PVector> img, ArrayList<PVector> control){
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
	    p_star = PVector.mult(sum_weights_per_p, (float) (1.0/(float)sum_weights));
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
	      /*
	      double a = pt_per_wp[0][0];
	      double b = pt_per_wp[0][1];
	      double c = pt_per_wp[1][0];
	      double d = pt_per_wp[1][1];
	      double det_inv = 1/(a*d - b*c); 
	      pt_per_wp[0][0] = det_inv* d;
	      pt_per_wp[0][1] = -1*det_inv* b;
	      pt_per_wp[1][0] = -1*det_inv* c;
	      pt_per_wp[1][1] = det_inv* a;
	      */
	      float[][] pt_per_wp_inv = papaya.Mat.inverse(Cast.doubleToFloat(pt_per_wp));     

	      double[] Ai_1 = new double[2];
	      Ai_1[0]= (v_minus_p_s.x * pt_per_wp_inv[0][0]) + (v_minus_p_s.y * pt_per_wp_inv[0][1]); 
	      Ai_1[1]= (v_minus_p_s.x * pt_per_wp_inv[1][0]) + (v_minus_p_s.y * pt_per_wp_inv[1][1]); 
	      A[counter][i] = Ai_1[0] * p_hat_i.x * w[counter][i] + Ai_1[1] * p_hat_i.y * w[counter][i];    
	    }
	    counter++;
	  }
	}

	public static ArrayList<PVector> calculateNewImage(ArrayList<PVector> img, ArrayList<PVector> out_control){
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
	    q_star = PVector.mult(sum_weights_per_q, (float) (1.0/(float)sum_weights));
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

	public static void updateControlPoints(){
	  if(Deformation.control_points.size() < 3) return;  
	  getA(Deformation.deformed_world,Deformation.control_points);
	}

	public static void updateControlPoints(ArrayList<PVector> img){
	  if(Deformation.control_points.size() < 3) return;  
	  getA(img,Deformation.control_points);
	}

	//SOME PREDEFINED DEFORMATIONS
	public static void addControlPointsAuto(boolean rand){
	  //clear
	  Deformation.control_points.clear();
	  Deformation.control_points_out.clear();
	  for(int i = 0; i < Deformation.deformed_world.size(); i+=Deformation.step_per_point){
	    //get coordinates in local frame
	    //control_points.add(edges.get(i));
	    if(!rand){
	    	Deformation.control_points.add(Deformation.deformed_world.get(i));
	    	Deformation.control_points_out.add(Deformation.deformed_world.get(i));
	    }else{
	      PVector v = Deformation.deformed_world.get(i);
	      PVector new_v = new PVector(v.x - Deformation.r_deformed_world_figure.getCenterX(),
	    		  v.y - Deformation.r_deformed_world_figure.getCenterY());                                          
	      new_v.mult((float) (Math.random() + 1));
	      new_v.add(v);
	      Deformation.control_points.add(new_v);
	      float r1 = (float) (40*Math.random() - 20);
	      float r2 = (float) (40*Math.random() - 20);
	      Deformation.control_points_out.add(
	    		  PVector.add(Deformation.control_points.get(Deformation.control_points.size()-1), 
	    				  new PVector(r1,r2)));
	    }
	  }
	}

	public static void scaleW(boolean clear){
	  //clear
	  if(clear){
		  Deformation.control_points.clear();
		  Deformation.control_points_out.clear();
	  }
	  Utilities.Rectangle r = Utilities.getBoundingBox(Deformation.deformed_world);
	  PVector top_left = new PVector(r.x, r.y); 
	  PVector bottom_left = new PVector(r.x, r.y + r.h);
	  PVector mid_right = new PVector(r.x + r.w, (float)r.getCenterY()); 
	  PVector movement = new PVector((float) ((r.w/8)*rand.nextGaussian()), 0);
	  PVector mid_right_im = PVector.add(mid_right, movement);
	  Deformation.control_points.add(top_left);
	  Deformation.control_points.add(bottom_left);
	  Deformation.control_points.add(mid_right);
	  Deformation.control_points_out.add(top_left);
	  Deformation.control_points_out.add(bottom_left);
	  Deformation.control_points_out.add(mid_right_im);
	}

	public static void scaleH(boolean clear){
	  //clear
	  if(clear){
		  Deformation.control_points.clear();
		  Deformation.control_points_out.clear();
	  }
	  Utilities.Rectangle r = Utilities.getBoundingBox(Deformation.deformed_world);
	  PVector top_left = new PVector(r.x, r.y); 
	  PVector top_rigth = new PVector(r.x + r.w, r.y);
	  PVector mid_bottom = new PVector((float)r.getCenterX(), r.y + r.h); 
	  PVector movement = new PVector(0, (float) ((r.h/8)*rand.nextGaussian()));
	  PVector mid_bottom_im = PVector.add(mid_bottom, movement);
	  Deformation.control_points.add(top_left);
	  Deformation.control_points.add(top_rigth);
	  Deformation.control_points.add(mid_bottom);
	  Deformation.control_points_out.add(top_left);
	  Deformation.control_points_out.add(top_rigth);
	  Deformation.control_points_out.add(mid_bottom_im);
	}

	//mode: 0 up, 1 bottom, 2 middle
	public static void applyHorizontalSpline(int mode, boolean clear, boolean reflexive){
	  //clear
	  if(clear){
	    Deformation.control_points.clear();
	    Deformation.control_points_out.clear();
	  }
	  Utilities.Rectangle r = Utilities.getBoundingBox(Deformation.deformed_world);
	  ArrayList<PVector> spline_control = new ArrayList<PVector>();
	  int quantity = 16;
	  float e = (float) ((0.5 - 0.3)*Math.random() + 0.3);//get a new point for each 2 control points
	  float t = 0;
	  for(int i = 0; i < quantity; i++){
	      float x_pos = r.x + i*(r.w/(quantity-1));
	      float y_mode = mode == 0 ? r.y : mode == 1 ? r.y + r.h : (float) r.getCenterY(); 
	      float y_pos = y_mode + (float) ((30 - -30)*Math.random() + -30); 
	      spline_control.add(new PVector(x_pos, y_pos));
	  }
	  spline_control = Splines.drawCurve(spline_control, t, e, false);
	  //apply the same transformation to all the points
	  float y_mode = mode == 0 ? r.y : mode == 1 ? r.y + r.h : (float) r.getCenterY(); 
	  float y_pos = y_mode + (float) ((20 - -20)*Math.random() + -20); 
	  for(PVector point : spline_control){
	    Deformation.control_points.add(new PVector(point.x, y_pos));
	  }
	  Deformation.control_points_out.addAll(spline_control);  
	  if(!reflexive || mode == 2){
	    //put a control point in the oposite side
	    float anchor = mode == 0 ? r.y + r.h : r.y - 20;
	    PVector mid = new PVector((float)r.getCenterX(), anchor); 
	    Deformation.control_points.add(mid);  
	    Deformation.control_points_out.add(mid);  
	  }else{
	    //put the same calculated points in the oposite place
	    //apply the same transformation to all the points
	    float inv_y_mode = mode == 1 ? r.y : mode == 0 ? r.y + r.h : (float) r.getCenterY();
	    y_pos = -(y_pos - y_mode) + inv_y_mode; 
	    for(int i = 0; i < spline_control.size(); i++){
	      PVector point = new PVector(spline_control.get(i).x, spline_control.get(i).y);
	      Deformation.control_points.add(new PVector(point.x, y_pos));
	      point.y = -(point.y - y_mode) + inv_y_mode;
	      Deformation.control_points_out.add(point);
	    }
	  }
	}

	//mode: 0 left, 1 right, 2 middle
	public static void applyVerticalSpline(int mode, boolean clear, boolean reflexive){
	  //clear
	  if(clear){
	    Deformation.control_points.clear();
	    Deformation.control_points_out.clear();
	  }
	  Utilities.Rectangle r = Utilities.getBoundingBox(Deformation.deformed_world);
	  ArrayList<PVector> spline_control = new ArrayList<PVector>();
	  int quantity = 16;
	  float e = (float) ((0.5 - 0.3)*Math.random() + 0.3); //get a new point for each 2 control points
	  float t = 0;
	  for(int i = 0; i < quantity; i++){
	      float y_pos = r.y + i*(r.h/(quantity-1));
	      float x_mode = mode == 0 ? r.x : mode == 1 ? r.x + r.w : (float) r.getCenterX(); 
	      float x_pos = x_mode + (float) ((20 - -20)*Math.random() + -20);
	      
	      spline_control.add(new PVector(x_pos, y_pos));
	  }
	  spline_control = Splines.drawCurve(spline_control, t, e, false);
	  //apply the same transformation to all the points
	  float x_mode = mode == 0 ? r.x : mode == 1 ? r.x + r.w : (float) r.getCenterX(); 
	  float x_pos = x_mode + (float) ((20 - -20)*Math.random() + -20); 
	  for(PVector point : spline_control){
	    Deformation.control_points.add(new PVector(x_pos, point.y));
	  }
	  Deformation.control_points_out.addAll(spline_control);
	  if(!reflexive || mode == 2){
	    //put a control point in the oposite side
	    float anchor = mode == 0 ? r.x + r.w : r.x - 20;
	    PVector mid = new PVector(anchor,(float)r.getCenterY()); 
	    Deformation.control_points.add(mid);  
	    Deformation.control_points_out.add(mid);  
	  }else{
	    //put the same calculated points in the oposite place
	    //apply the same transformation to all the points
	    float inv_x_mode = mode == 1 ? r.x : mode == 0 ? r.x + r.w : (float) r.getCenterX();
	    x_pos = -(x_pos - x_mode) + inv_x_mode; 
	    for(int i = 0; i < spline_control.size(); i++){
	      PVector point = new PVector(spline_control.get(i).x, spline_control.get(i).y);
	      Deformation.control_points.add(new PVector(x_pos, point.y));
	      point.x = -(point.x - x_mode) + inv_x_mode;
	      Deformation.control_points_out.add(point);
	    }
	  }
	}

	public static void combination(boolean reflex){
	  ArrayList<PVector> new_img = new ArrayList<PVector>();
	  new_img.addAll(Deformation.deformed_world);
	  //splineht
	  applyHorizontalSpline(1,true, reflex);
	  updateControlPoints(new_img);
	  new_img = calculateNewImage(new_img,Deformation.control_points_out);
	  //scalehb
	  /*scaleH(true);
	  applyHorizontalSpline(1,true);
	  updateControlPoints(new_img);
	  new_img = calculateNewImage(new_img,Deformation.control_points_out);*/
	  //splinevl
	  applyVerticalSpline(1,true,reflex);
	  updateControlPoints(new_img);
	  new_img = calculateNewImage(new_img,Deformation.control_points_out);
	  //scalevr
	  /*scaleH(true);
	  applyVerticalSpline(1,true);
	  updateControlPoints(new_img);*/

	  //scalew
	  scaleW(true);
	  updateControlPoints(new_img);
	  new_img = calculateNewImage(new_img,Deformation.control_points_out);
	  //scaleh
	  scaleH(true);
	  updateControlPoints(new_img);
	  new_img = calculateNewImage(new_img,Deformation.control_points_out);

	  new_img = calculateNewImage(new_img,Deformation.control_points_out);
	  //modify the shape
	  Deformation.deformed_edges = new_img;
	  Deformation.setDeformedFigure(Utilities.getContours(Deformation.deformed_edges, 217,245,12,100));
	  //join to the model
	  Deformation.deformed_fig.setShape(Deformation.deformed_figure);
	}

	public static void combinationR(boolean reflex){
	  ArrayList<PVector> new_img = new ArrayList<PVector>();
	  new_img.addAll(Deformation.deformed_world);
	  //splineht
	  applyHorizontalSpline(0,true, reflex);
	  updateControlPoints(new_img);
	  new_img = calculateNewImage(new_img,Deformation.control_points_out);
	  //scalehb
	  /*scaleH(true);
	  applyHorizontalSpline(1,true);
	  updateControlPoints(new_img);
	  new_img = calculateNewImage(new_img,Deformation.control_points_out);*/
	  //splinevl
	  applyVerticalSpline(0,true,reflex);
	  updateControlPoints(new_img);
	  new_img = calculateNewImage(new_img,Deformation.control_points_out);
	  //scalevr
	  /*scaleH(true);
	  applyVerticalSpline(1,true);
	  updateControlPoints(new_img);*/

	  //scalew
	  scaleW(true);
	  updateControlPoints(new_img);
	  new_img = calculateNewImage(new_img,Deformation.control_points_out);
	  //scaleh
	  scaleH(true);
	  updateControlPoints(new_img);
	  new_img = calculateNewImage(new_img,Deformation.control_points_out);

	  new_img = calculateNewImage(new_img,Deformation.control_points_out);
	  //modify the shape
	  Deformation.deformed_edges = new_img;
	  Deformation.setDeformedFigure(Utilities.getContours(Deformation.deformed_edges, 217,245,12,100));
	  //join to the model
	  Deformation.deformed_fig.setShape(Deformation.deformed_figure);
	}
}
