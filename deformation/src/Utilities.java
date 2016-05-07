import java.util.ArrayList;
import java.util.Random;

import gab.opencv.Contour;
import gab.opencv.OpenCV;
import processing.core.*;


public class Utilities {
	public static OpenCV opencv;
	public static boolean invert = false; //change to true if the information of the image is in black
	public static Contour contour;
	public static ArrayList<Contour> contours        = new ArrayList<Contour>();
	public static float approximation = (float) 0.001;
	public static PApplet p;
	//TEXTURE VARS IF IS REQUIRED------------------
	public static PImage last_texture = null;
	//---------------------------------------------	
	
	
	public static class Rectangle{
		  float x,y,w,h;  
		  public Rectangle(float xx, float yy, float ww, float hh){
		    x = xx;
		    y = yy;
		    w = ww; 
		    h= hh; 
		  }
		  float getCenterX(){
		    return x + w/2;
		  }
		  float getCenterY(){
		    return y + h/2;
		  }
		  @Override
		  public String toString(){
		    String s = "Rectangle: \n";
		    s += "UL : x = " + x + ", y = " + y; 
		    s += "width = " + w + ", height = " + h; 
		    s += "centerX = " + getCenterX() + ", centerY = " + getCenterY(); 
		    return s;
		  }
		}

		//COUNTOUR METHODS-------------------------------
		public static PShape getCountours(){
		  PShape img_contours;
		  opencv = new OpenCV(p, Deformation.source_image);
		  //convert to gray
		  opencv.gray();
		  //apply basic threshold
		  if(invert) opencv.invert();
		  opencv.threshold(10);
		  Deformation.source_image = opencv.getOutput();
		  contours = opencv.findContours();  
		  //save just the external countour
		  contour = contours.get(0);
		  for (Contour c : contours){
		    contour = contour.numPoints() < c.numPoints() ? c : contour;
		  }
		  contour.setPolygonApproximationFactor(approximation);
		  contour = contour.getPolygonApproximation();

		  System.out.println("founded a contour with" + contour.numPoints() + " points");  
		  //save the points
		  Deformation.edges = contour.getPoints();
		  img_contours = getCountoursShape(last_texture);
		  return img_contours;
		}

		public static void getCountoursShape(PShape img_contours){
		  getCountoursShape((PImage)null);
		}

		public static PShape getCountoursShape(PImage text){
		  PShape figure = p.createShape();
		  figure.beginShape();
		  if(text != null){
		    text.resize(Deformation.all_width,2*Deformation.all_heigth/3);
		    figure.textureMode(PConstants.IMAGE);    
		    figure.texture(text);
		  }
		  
		  for(int k = 0; k < Deformation.edges.size();k++){
		    figure.stroke(255,255,255); 
		    figure.strokeWeight(1); 
		    figure.setFill(p.color(0,0,255,100));
		    figure.vertex(Deformation.edges.get(k).x, Deformation.edges.get(k).y,
		    		Deformation.edges.get(k).x, Deformation.edges.get(k).y);
		  }
		  figure.endShape(PConstants.CLOSE);
		  return figure;
		}

		public static void getContours(PShape s, ArrayList<PVector> points){
		  s = getContours(points, 0,255,0,100);
		}

		public static PShape getContours(ArrayList<PVector> points, int c1, int c2, int c3, int a){
		  int col = p.color(c1, c2, c3, a);
		  PShape s = p.createShape();
		  s.beginShape();
		  for(int k = 0; k < points.size();k++){
		    s.stroke(255,255,255); 
		    s.strokeWeight(2); 
		    s.fill(col);
		    s.vertex(points.get(k).x, points.get(k).y );
		  }
		  s.endShape(PConstants.CLOSE);
		  return s;
		}
		//END COUNTOUR METHODS---------------------------


		//UTIL ALGORITHMS--------------------------------
		public static ArrayList<PVector> quickSort(ArrayList<PVector> list, PVector comp, int size){
		  if(size < 2) return list;
		  Random rand = new Random();
		  int pivot = rand.nextInt(size);
		  int p1 = 0,p2 = 0;
		  ArrayList<PVector>list1 = new ArrayList<PVector>();
		  ArrayList<PVector>list2 = new ArrayList<PVector>();  
		  //reorganize list
		  for(int k = 0; k < size; k++){
		    if(list.get(k).dist(comp) < list.get(pivot).dist(comp)){
		      list1.add(list.get(k));
		      p1++;
		    }else{
		      if(k != pivot){
		        list2.add(list.get(k));
		        p2++;
		      }
		    }
		  }
		  //recursion
		  list1 = quickSort(list1, comp, p1);
		  list2 = quickSort(list2, comp, p2);
		  PVector num_pivot = list.get(pivot);
		  //return the list in the right order
		  for(int k = 0; k < p1; k++){
		    list.set(k,list1.get(k));
		  }
		  list.set(p1, num_pivot);
		  for(int k = 0; k < p2; k++){
		    list.set(p1 + k + 1, list2.get(k));
		  }
		  return list;
		}

		public static Rectangle getBoundingBox(ArrayList<PVector> points){
		  PVector top = new PVector(9999,9999);
		  PVector bottom = new PVector(-9999,-9999);
		  for(PVector p : points){
		    if(p.x < top.x) top.x = p.x;  
		    if(p.y < top.y) top.y = p.y;  
		    if(p.x > bottom.x) bottom.x = p.x;  
		    if(p.y > bottom.y) bottom.y = p.y;  
		  }
		  return new Rectangle(top.x, top.y , bottom.x - top.x, bottom.y - top.y);
		} 
		//-----------------------------------------------
}
