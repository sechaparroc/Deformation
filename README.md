# Deformation
Affine Tranformations using MLS

This is an implementation in Processing of affine transformations using the method proposed on:
http://faculty.cs.tamu.edu/schaefer/research/mls.pdf

So, the idea is that an image is loaded to the program and there will be some preprocessing to consider just the outter contour of the image. Then, basically you can add, remove and traslate control points and drag them to generate their new positions deformating the shape of the image as Schaefer proposed.

There are some predefined deformations than can be used by pressing the numbers.

This predifened deformations are:
- Random
- Scaling Width
- Scaling Height
- Horizontal splines in three different positions: Top, middle and Bottom. Here, basically some control points are
added in a horizontal line, at the top, the middle or the left of the shape. Then there are randomly added other       points that will be the images of the control points and then an interpolation of that points using hermite            splines generates some other images of the control points to make a smooth change.
An additional option can be used pressing the key 'r' and in this case top and bottom methods will create a            reflexion of the calculated spline at the top (if the method is bottom) or the bottom (if the option is top) of        the shape.
- Vertical spline: same idea as before but modifing the width of the shape.
- Combination: Mix scaling and splines methods.

We also consider some deformations that we call "world deformations", so the idea here is to use a regular polygon that encloses the shape, and that can be manipulated changing the position of its vertices (control points). In this case, for example, a Triangle will deform the shape scaling, rotating, traslating or reflexing it.

After the world deformations are applied, then you can use the proposed predefined deformations or add and modify control points over the deformed figure as result by the world deformation.

It's important to know that the proposed method, do a preprocessing of the image (Basically a threshold and then find the contour by 8-connectivity) to consider only the outter contour of it. So for this purpose we use very basic and useful algorithms provided by OpenCV. 

There is also another dependency library very useful to draw complex scenes: ProScene. So the idea here was to use two scenes:
- Main Scene: is the scene where you can see the changes produced to the shape, and here you can see three models of    your shape. The first one shows you the original shape enclosed by the shape that represents the original world (a    regular polygon) and the deformed world. The second one shows you the shape as result of world deformations, so is    according to this method where the control points and their images are going to be located to deform the image. The   latest one shows you the final deformed image.
- Auxiliar Scene: is a scene located in the bottom left of the screen. here you can manipulate the world represented    as a regular polygon. You can modify the number of edges of the polygon, rotate it in a way that it will change the    world deformation and modify the vertex of the polygon for more freedom modifications.
