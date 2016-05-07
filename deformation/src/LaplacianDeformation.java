import java.util.ArrayList;
import java.util.HashMap;

import processing.core.PShape;
import processing.core.PVector;
import remixlab.dandelion.core.MatrixHelper;
import smile.data.Dataset;
import smile.data.SparseDataset;
import smile.data.parser.SparseDatasetParser;
import smile.math.Math;
import smile.math.SparseArray;
import smile.math.matrix.CholeskyDecomposition;
import smile.math.matrix.IMatrix;
import smile.math.matrix.LUDecomposition;
import smile.math.matrix.Matrix;
import smile.math.matrix.QRDecomposition;
import smile.math.matrix.SparseMatrix;
import smile.util.SmileUtils;

/*
 * November 19 2015
 * Sebastian Chaparro
 * 
 * This is an implementation of Laplacian Surface Deformation 
 * The paper can be found at: 
 * http://igl.ethz.ch/projects/Laplacian-mesh-processing/Laplacian-mesh-editing/laplacian-mesh-editing.pdf
 * 
 * */

public class LaplacianDeformation {
	static ArrayList<Vertex> vertices;
	static HashMap<Vertex,Anchor> anchors;
	static SparseDataset A, L, M; 
	
	public static class Anchor{
		Vertex vertex;
		PVector pos;
		int idx;//id of the control point
		
		public Anchor(Vertex vv, int ii){
			vertex = vv;
			idx = ii;
		}
		
		public void updatePosition(){
			PVector i = Deformation.laplacian_control_points.get(idx);
			PVector f = Deformation.laplacian_control_points_out.get(idx);
			pos = PVector.sub(f, i);
			//pos = new PVector(0,0);
			pos.add(vertex.v);
		}
	}
	
	public static class Vertex{
		PVector v;
		PVector d;
		int idx;
		ArrayList<Vertex> neighbors;		

		public Vertex(PVector vv, int i){
			v = vv;
			idx = i;
			neighbors = new ArrayList<Vertex>();
		}
		
		public void addNeighbor(Vertex n){
			neighbors.add(n);
			n.neighbors.add(this);
		}
	}
	
	public static void addEdge(SparseDataset A, Vertex v1, Vertex v2){
		//The whole vetex is used as arg if its desired to use other weight scheme
		A.set(v1.idx, v2.idx, 1);
		A.set(v2.idx, v1.idx, 1);		
	}
	
	public static void setup(PShape shape){
		getNeighbors(shape);
		getLaplacian();
		anchors = new HashMap<Vertex,Anchor>();
	}
	
	public static void getNeighbors(PShape shape){
		A = new SparseDataset();
		vertices = new ArrayList<Vertex>();
		Vertex prev = new Vertex(shape.getVertex(0),0);
		vertices.add(prev);
		Vertex v0 = prev;
		for(int i = 1; i < shape.getVertexCount(); i++){
			PVector vec = shape.getVertex(i);
			Vertex v = new Vertex(vec,i);
			v.addNeighbor(prev);
			addEdge(A,v,prev);
			prev = v;
			vertices.add(prev);
		}
		addEdge(A,v0,prev);
		prev.addNeighbor(v0);
	}

	public static void getLaplacian(){
		int n = vertices.size();
		//M is used as the matrix to get the new positions of the vertices
		M = new SparseDataset();
		L = new SparseDataset();
		for(Vertex v_i : vertices){
			double dx = v_i.v.x;
			double dy = v_i.v.y;
			L.set(v_i.idx, v_i.idx, 1);
			M.set(v_i.idx, v_i.idx, 1);
			M.set(v_i.idx + n, v_i.idx + n, 1);
			int degree = v_i.neighbors.size();
			for(Vertex v_j : v_i.neighbors){
				L.set(v_i.idx, v_j.idx, -1./degree);
				dx += -(1./degree) * v_j.v.x;
				dy += -(1./degree) * v_j.v.y;
				M.set(v_i.idx, v_j.idx, -1./degree);
				M.set(v_i.idx + n, v_j.idx + n, -1./degree);				
			}
			v_i.d = new PVector((float)dx, (float)dy);
		}
		//printMat("Laplacian", L.toArray());
		//printMat("Initial M", M.toArray());		
	}
	
	public static Vertex getNearest(PVector p){
		float min_dist = 99999;
		Vertex min = null;
		for(Vertex v : vertices){
			if(PVector.dist(v.v, p) < min_dist){
				min = v;
				min_dist = PVector.dist(v.v, p);
			}
		}
		return min;
	}
	
	public static void addAnchors(ArrayList<PVector> vecs){
		addAnchors(vecs, false);
	}
	public static void addAnchors(ArrayList<PVector> vecs, boolean reset){
		if(reset) anchors.clear();
		int i = 0;
		for(PVector v : vecs){
			//System.out.println("cp : ->" + v);
			addAnchor(v,i++);			
		}
	}
	
	public static void addAnchor(PVector p, int i){
		//get the nearest point
		Vertex v = getNearest(p);
		Anchor anchor = new Anchor(v, i);
		if(anchors.containsKey(v) == false)anchors.put(v, anchor);
		for(Vertex v_j : anchor.vertex.neighbors){
			if(anchors.containsKey(v_j) == false) anchors.put(v_j,new Anchor(v_j, i));
		}
	}
	
	public static void calculateLaplacian(){
		int n = vertices.size();	
		//printMat("laplacian",L.toArray(),30,30);
		for(Vertex v_i : vertices){
			int num_n = v_i.neighbors.size();			
			double[][] T_data = new double[6][4];
			int idx = 0;
			T_data[idx] = new double[]{v_i.v.x,  v_i.v.y, 1, 0};
			T_data[idx + num_n + 1] = new double[]{v_i.v.y, -v_i.v.x, 0, 1};			
			idx++;
			for(Vertex v_j : v_i.neighbors){
				T_data[idx] = new double[]{v_j.v.x,  v_j.v.y, 1, 0};
				T_data[idx + num_n + 1] = new double[]{v_j.v.y, -v_j.v.x, 0, 1};			
				idx++;
			}
			
			QRDecomposition qr = new QRDecomposition(T_data);
			//Matrix T = new Matrix(T_data);
			//qr.inverse();
			double[][] T_inv = new double[4][6];
			qr.solve(Math.eye(6, 6), T_inv);
			//printMat("inverse T implicit",T_inv);
			
			//get the linear transformation coefficients
			double[] s = T_inv[0];
			double[] a = T_inv[1];
			//s = new double[]{-1,0,0,-1,0,0};
			//a = new double[]{0,-1,0,0,-1,0};

			//apply the transformation to laplacian coords
			double[][] T_delta = new double[2][6];
			for(int i = 0; i < T_delta[0].length; i++){
				T_delta[0][i] =  s[i]*v_i.d.x + a[i]*v_i.d.y;
				T_delta[1][i] = -a[i]*v_i.d.x + s[i]*v_i.d.y;				
			}
			//Update values on M
			idx = 0;
			M.set(v_i.idx    , v_i.idx    , M.get(v_i.idx    , v_i.idx    ) - T_delta[0][idx]);
			M.set(v_i.idx + n, v_i.idx    , M.get(v_i.idx + n, v_i.idx    ) - T_delta[1][idx]);
			M.set(v_i.idx    , v_i.idx + n, M.get(v_i.idx    , v_i.idx + n) - T_delta[0][idx + num_n + 1]);
			M.set(v_i.idx + n, v_i.idx + n, M.get(v_i.idx + n, v_i.idx + n) - T_delta[1][idx + num_n + 1]);
			idx++;
			for(Vertex v_j : v_i.neighbors){
				M.set(v_i.idx    , v_j.idx    , M.get(v_i.idx    , v_j.idx    ) - T_delta[0][idx]);
				M.set(v_i.idx + n, v_j.idx    , M.get(v_i.idx + n, v_j.idx    ) - T_delta[1][idx]);
				M.set(v_i.idx    , v_j.idx + n, M.get(v_i.idx    , v_j.idx + n) - T_delta[0][idx + num_n + 1]);
				M.set(v_i.idx + n, v_j.idx + n, M.get(v_i.idx + n, v_j.idx + n) - T_delta[1][idx + num_n + 1]);
				idx++;
			}
		}
	}		
	
	public static ArrayList<PVector> solveLaplacian(){
		int n = vertices.size();			
		SparseDataset M = new SparseDataset();
		SparseDataset M_T = new SparseDataset();
		for(int i = 0; i < LaplacianDeformation.M.size(); i++){
			for(int j = 0; j < LaplacianDeformation.M.ncols(); j++){
				double val = LaplacianDeformation.M.get(i, j);
				M.set(i,j,val);
				M_T.set(j,i,val);					
			}
		}
		int m_dim = M.size();
		double weight = 1;
		double[] RHS = new double[m_dim + 2*anchors.size()];		
		
		for(Anchor anchor : anchors.values()){
			anchor.updatePosition();
			M.set(m_dim, anchor.vertex.idx, weight);
			M_T.set(anchor.vertex.idx, m_dim, weight);
			RHS[m_dim++] = weight*anchor.pos.x;
			//System.out.println("--> RHS : " + "( " + RHS[m_dim-1] + ", " + (int)(m_dim-1) + " ) ");
			M.set(m_dim, anchor.vertex.idx + n, weight);
			M_T.set(anchor.vertex.idx + n, m_dim, weight);
			RHS[m_dim++] = weight*anchor.pos.y;
			//System.out.println("--> RHS : " + "( " + RHS[m_dim-1] + ", " + (int)(m_dim-1) + " ) ");
		}
		//Solve
		SparseMatrix MMT = M.toSparseMatrix().transpose().times(M.toSparseMatrix()); 
//				SparseMatrix.AAT(M.toSparseMatrix(), M.toSparseMatrix().transpose());
		Matrix LHS = new Matrix(matrixToArray(MMT), true, true);
		Matrix M_aux = new Matrix(M_T.toArray());		
		//double 
		double[] RHSS = new double[M_aux.nrows()];
		M_aux.ax(RHS, RHSS);
		//printArr("rhs " + RHS.length, RHS);
		//printArr("new rhs", RHSS);
		//printMat("m cond", M.toArray());
		double[] new_coords = new double[LHS.ncols()];	
		CholeskyDecomposition ch = LHS.cholesky();
		//QRDecomposition ch = new QRDecomposition(M.toArray());
		ch.solve(RHSS, new_coords);		
		ArrayList<PVector> new_img = new ArrayList<PVector>();
		for(int i = 0; i < n; i++){
			//System.out.println("--> prev_coord : " + "( " + vertices.get(i).v.x + ", " + vertices.get(i).v.y + " ) ");
			//System.out.println("--> coord : " + "( " + new_coords[i] + ", " + new_coords[i+n] + " ) ");
			new_img.add(new PVector((float)new_coords[i], (float)new_coords[i+n]));
		}
		/*for(int i = 0; i < RHS.length; i++){
			System.out.println("--> RHS : " + "( " + RHS[i] + ", " + i + " ) ");
		}*/

		//printArr("RHS", RHS);
		//printArr("coords", new_coords);
		//System.out.println("previous coords");
		//for(int i = 0; i < n; i++){
		//	System.out.print("(" + vertices.get(i).v.x + ", " + vertices.get(i).v.y + "), ");
		//}		
		//for(int i = 0; i < n; i++){
			//System.out.print(vertices.get(i).v.y + ", ");
		//}		
		return new_img;		
	}
	
	static void printMat(String name, double[][] m){
		System.out.println("---------------------");
		System.out.println(name);		
		System.out.println("---------------------");		
		for(int i = 0; i < m.length; i++){
			for(int j = 0; j < m[0].length; j++){
				System.out.printf("%.2f" + ", " , m[i][j]);
			}
			System.out.println();
		}
		System.out.println("---------------------");		
		System.out.println("---------------------");		
	}

	static void printMat(String name, double[][] m, int r, int c){
		System.out.println("---------------------");
		System.out.println(name);		
		System.out.println("---------------------");
		int rr = Math.min(r, m.length);
		int cc = Math.min(c, m[0].length);		
		for(int i = 0; i < rr; i++){
			for(int j = 0; j < cc; j++){
				System.out.printf("%.2f" + ", \t" , m[i][j]);
			}
			System.out.println();
		}
		System.out.println("---------------------");		
		System.out.println("---------------------");		
	}
	
	
	static void printArr(String name, double[] m){
		System.out.println("---------------------");
		System.out.println(name);		
		System.out.println("---------------------");		
		for(int i = 0; i < m.length; i++){
				System.out.printf("%.2f" + ", " , m[i]);
		}
		System.out.println("---------------------");		
		System.out.println("---------------------");		
	}
	
	static double[][] matrixMult(double[][] m1, double[][] m2){
		int n = m1.length;
		int m = m1[0].length;
		int l = m2[0].length;
		double[][] result = new double[n][l];
		for(int i = 0; i < n; i++){
			for(int k = 0; k < m; k++){
				for(int j = 0; j < l; j++){
					result[i][j] += m1[i][j] * m2[j][k]; 
				}			
			}
		}
		return result;
	}
	
	static double[][] matrixToArray(IMatrix m){
		double[][] result = new double[m.nrows()][m.ncols()];
		for(int i = 0; i < m.nrows(); i++){
			for(int j = 0; j < m.ncols(); j++){
				result[i][j] = m.get(i, j);	
			}
		}
		return result;
	}
}
