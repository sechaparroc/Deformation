public class Rectangle{
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
PShape getCountours(){
  PShape img_contours;
  opencv = new OpenCV(this, source_image);
  //convert to gray
  opencv.gray();
  //apply basic threshold
  if(invert) opencv.invert();
  opencv.threshold(10);
  source_image = opencv.getOutput();
  contours = opencv.findContours();  
  //save just the external countour
  contour = contours.get(0);
  for (Contour c : contours){
    contour = contour.numPoints() < c.numPoints() ? c : contour;
  }
  contour.setPolygonApproximationFactor(approximation);
  contour = contour.getPolygonApproximation();

  println("founded a contour with" + contour.numPoints() + " points");  
  //save the points
  edges = contour.getPoints();
  img_contours = getCountoursShape(last_texture);
  return img_contours;
}

void getCountoursShape(PShape img_contours){
  getCountoursShape((PImage)null);
}

PShape getCountoursShape(PImage text){
  PShape figure = createShape();
  figure.beginShape();
  if(text != null){
    text.resize(all_width,2*all_heigth/3);
    figure.textureMode(IMAGE);    
    figure.texture(text);
  }
  
  for(int k = 0; k < edges.size();k++){
    figure.stroke(255,255,255); 
    figure.strokeWeight(1); 
    figure.setFill(color(0,0,255,100));
    figure.vertex(edges.get(k).x, edges.get(k).y,edges.get(k).x, edges.get(k).y);
  }
  figure.endShape(CLOSE);
  return figure;
}

void getContours(PShape s, ArrayList<PVector> points){
  s = getContours(points, color(0,255,0,100));
}

PShape getContours(ArrayList<PVector> points, int col){
  PShape s = createShape();
  s.beginShape();
  for(int k = 0; k < points.size();k++){
    s.stroke(255,255,255); 
    s.strokeWeight(2); 
    s.fill(col);
    s.vertex(points.get(k).x, points.get(k).y );
  }
  s.endShape(CLOSE);
  return s;
}
//END COUNTOUR METHODS---------------------------


//UTIL ALGORITHMS--------------------------------
ArrayList<PVector> quickSort(ArrayList<PVector> list, PVector comp, int size){
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

public Rectangle getBoundingBox(ArrayList<PVector> points){
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
