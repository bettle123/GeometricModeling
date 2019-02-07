//
//coder: Su Ming Yi, date: 01/09/2018
//5543 lab1
//Implement the following curve generation procedures in Lab 1. 
//Allow the user to select one of the techniques 
//and generate a new wire from an existing wire 
//by selecting the number of points(on a slider) 
//and a curve generation technique(radio buttons.) 
//My initial set of points may be input as a set of ordered points
//in plane.
//
//Bezier curve - one curve for all the points
//Cubic B - spline with uniform knot vector.
//Subdivision curves using repeated de Casteljau method.
//Subdivision Quadric B - spline with uniform knot vector.
//The first two should be implemented with the direct method, 
//that is, by sampling the parameter range of u
//Allow the user to generate a curve, go back 
//and edit a point on the control wire, 
//and then regenerate a new curve replacing the original curve.
//
// Reference:
// Bezier curve:
// https://www.cs.uaf.edu/2011/fall/cs381/code/bezier.cpp
// Opengl guide:
// http://www.opengl.org/wiki/Tutorial:_OpenGL_3.1_The_First_Triangle_%28C%2B%2B/Win%29
// B-Spline curve:
// http://www2.cs.uregina.ca/~anima/408/Notes/Interpolation/UniformBSpline.htm

///////////////////////////////////////////////////////////////////////////
//coder: Su Ming Yi, date: 02/26/2018
//5543 lab2
//Write a program that can generate basic geometric models. 
//My program should provide reasonable user interaction.  
//Using Lab1 generate surfaces implementing the following techniques:

// surfaces of revolution
// The user should draw an open or closed curve.
// The curve is drawn relative to an axis in the display image;
// you can use either a default axis or a user - specified one).
// The user should be able to set the number of slices.
// check for overlap of contour and axis of revolution 
// and issue a warning to the user.

// extrusion
// The user should draw one open or closed wire(curve) 
// and be able to set the z - depth of the extrusion.
// the wire should be considered to lie in the x - y plane 
// and the extrusion to occur in the z direction.
// Optional extension : you can generalize this to include a user 
// - specified extrusion direction.
// Optional extension : you can generate multiple extruded wires.

// sweep operators
// User input of an open or closed generator wire 
// and an open or closed trajectory wire.
// Consider the generator wire to lie in the x - y plane.
// Consider the trajectory wire to lie in the y - z plane.

// output ASCII vertex - face file
// Use counterclockwise face definitions and use the following format :


////////////////////////////////////////////////////////////////////////////
// 03/24
// Generate control polyhedron using Lab 2. 
// I should generate surfaces with boundary and without boundary.
// Do the following surface types :
// 1. (50 % )
// Bezier surface enforcing G1 continuity in the closed direction
// Cubic B - spline surface - use uniform knot vector
// I must generate .off files as final output 
// by polygonizing the surfaces using equally spaced u, v coordinates.


// 2. (50 % )
// (subdivision surfaces.
// Use some examples where the initial control polyhedron
// is not a regular rectangular mesh)
// Doo - Sabin surface
// Catmull - Clark surface
// Loop surface
// You will be graded on the design of the user interface 
// and the generation power of your program.

// take reference by 
// https://github.com/rajaditya-m/geometric-modeling


// 04/15

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_walk_along_line_point_location.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Point_2<Kernel> Point_2;
typedef CGAL::Vector_2<Kernel> Vector_2;
typedef CGAL::Point_3<Kernel> Point_3;
typedef CGAL::Vector_3<Kernel> Vector_3;
typedef CGAL::Segment_2<Kernel> Edge_Segment_2;
typedef CGAL::Delaunay_triangulation_2<Kernel> Delaunay;
typedef Delaunay::Face_iterator face_iter;
typedef Delaunay::Edge_iterator edge_iter;
typedef Delaunay::Vertex_iterator vertex_iter;
typedef CGAL::Arr_segment_traits_2<Kernel> Traits_2;
typedef CGAL::Arrangement_2<Traits_2> Arrangement_2;
typedef CGAL::Arr_walk_along_line_point_location<Arrangement_2> Walk_pl;
typedef Traits_2::X_monotone_curve_2 Segment_2;







#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

// 03/01
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtx/transform2.hpp>
#include <gtc/type_ptr.hpp>
#include <math.h>
#include <string>
#include <sstream>
namespace patch
{
	template < typename T > std::string to_string(const T& n)
	{
		std::ostringstream stm;
		stm << n;
		return stm.str();
	}
}




using namespace std;
/////////////////////////////////////////////////////////////
// 03/26
// ------------------------------------------------------------
// Mesh.h
// ------------------------------------------------------------
//
// ------------------------------------------------------------

#define	MESHCLASS
#define TIMEOUTCNTR 100


std::vector<std::pair <glm::vec2, glm::vec2> > render_recons;
int always_open_recons = 0;


#include <stdio.h>
#include <vector>
#include <set>
// Change to double for more precision
typedef		double	datatype;
using namespace std;
// ------------------------------------------------------------
// GeomVert: this class holds the geometric coordinates of a vertex
// ------------------------------------------------------------
// 02/18
// This class will have the position (x,y,z) of the point
// the function GetCo will return the x,y,z coordinate of point
// ex. GetCo(0) will return x coordinate
class GeomVert {
public:
	GeomVert(datatype x, datatype y, datatype z) { mCo[0] = x; mCo[1] = y; mCo[2] = z; }

	datatype      GetCo(int axis) { return mCo[axis]; }

	bool operator == (GeomVert &A) {
		return ((mCo[0] == A.GetCo(0)) && (mCo[1] == A.GetCo(1)) && (mCo[2] == A.GetCo(2)));
	}

private:
	datatype	mCo[3];
};
// ------------------------------------------------------------
// ------------------------------------------------------------
// TopoVert: this class holds all of a vertex's topological (connectivity) information
// ------------------------------------------------------------
// This class TopoVert will store all of a vertex's topological (connectivity information)
// Use some "Add" functions to add vertices, edges, and facets
// Use some "GetNumber" functions to get the size of  V, E, F
// Use some "Get" functions to get the value of the index V, E, F
class TopoVert {
public:
	TopoVert() { };
	~TopoVert() { mIncVerts.clear(); mIncEdges.clear(); mIncFacets.clear(); }
	void AddIncVert(int vert_ind) { mIncVerts.insert(vert_ind); }
	//void AddIncVert (int vert_ind ) { mIncVerts.push_back( vert_ind ); }
	void AddIncEdge(int edge_ind) { mIncEdges.push_back(edge_ind); }
	void AddIncFacet(int facet_ind) { mIncFacets.push_back(facet_ind); }

	int GetNumberIncVertices() { return mIncVerts.size(); }
	int GetIncVertex(int vert_ind) {
		set<int>::iterator sit = mIncVerts.begin(); for (int i = 0; i < vert_ind; i++) sit++;
		return *sit;
	}
	int GetNumberIncEdges() { return mIncEdges.size(); }
	int GetIncEdge(int edge_ind) { return mIncEdges[edge_ind]; }
	int GetNumberIncFacets() { return mIncFacets.size(); }
	int GetIncFacet(int facet_ind) { return mIncFacets[facet_ind]; }

private:
	set<int>    mIncVerts;
	vector<int> mIncEdges;
	vector<int> mIncFacets;

};
// ------------------------------------------------------------
// ------------------------------------------------------------
// TopoEdge
// ------------------------------------------------------------
class TopoEdge {
public:
	TopoEdge() { v1 = v2 = -1; }
	~TopoEdge() { mIncFacets.clear(); }

	bool operator == (TopoEdge &A) {
		return (((v1 == A.GetVertex(0)) && (v2 == A.GetVertex(1))) || ((v2 == A.GetVertex(0)) && (v1 == A.GetVertex(1))));
	}

	int  GetVertex(int ind) { if (ind == 0) return v1;  return v2; }
	void SetVertex(int ind, int v) { if (ind == 0) { v1 = v; } else { v2 = v; } }

	void AddIncFacet(int facet_ind) { mIncFacets.push_back(facet_ind); }
	int  GetNumberIncFacets() { return mIncFacets.size(); }
	int  GetIncFacet(int facet_ind) { return mIncFacets[facet_ind]; }

private:
	int v1, v2;
	vector<int> mIncFacets;
};
// ------------------------------------------------------------
// ------------------------------------------------------------
// TopoFacet:  this class holds a facet's topological connectivity) information
//             Facets are represented as a list of vertex indices
// ------------------------------------------------------------
class TopoFacet {
public:
	TopoFacet() { };
	~TopoFacet() { mIncVerts.clear(); mIncEdges.clear();  mIncFacets.clear(); }
	void AddIncVertex(int v_ind) { mIncVerts.push_back(v_ind); }
	void AddIncEdge(int e_ind) { mIncEdges.push_back(e_ind); }
	void AddIncFacet(int f_ind) { mIncFacets.insert(f_ind); }
	int  GetNumberVertices() { return mIncVerts.size(); }
	int  GetVertexInd(int vert_ind) { return mIncVerts[vert_ind]; }
	int  GetNumberEdges() { return mIncEdges.size(); }
	int  GetIncEdge(int edge_ind) { return mIncEdges[edge_ind]; }
	int  GetNumberFacets() { return mIncFacets.size(); }
	int  GetIncFacet(int facet_ind) {
		set<int>::iterator sit = mIncFacets.begin(); for (int i = 0; i < facet_ind; i++) sit++;
		return *sit;
	}



private:
	vector<int> mIncVerts;
	vector<int> mIncEdges;
	set<int>    mIncFacets;
};
// ------------------------------------------------------------
// ------------------------------------------------------------
// Mesh:  This class uses all the preceding classes to represent a mesh with
//        adjacency.connectivity information
// ------------------------------------------------------------
// 02/18
// Mesh is the class that includes everything.
class Mesh {
public:
	Mesh() { };
	~Mesh() { Erase(); };

	void      AddFacet(datatype x1, datatype y1, datatype z1, datatype x2, datatype y2, datatype z2, datatype x3, datatype y3, datatype z3);
	void      AddFacet(vector<GeomVert> geomfacet);

	int		  GetNumberVertices() { return mGeomVerts.size(); }
	int		  GetNumberEdges() { return mTopoEdges.size(); }
	int       GetNumberFacets() { return mTopoFacets.size(); }

	TopoVert  GetVertex(int vert_ind) { return mTopoVerts[vert_ind]; }
	TopoEdge  GetEdge(int edge_ind) { return mTopoEdges[edge_ind]; }
	TopoFacet GetFacet(int facet_ind) { return mTopoFacets[facet_ind]; }

	GeomVert  GetGeomVertex(int vert_ind) { return mGeomVerts[vert_ind]; }



private:
	int       FindGeomVertex(GeomVert v);
	int		  FindTopoEdge(TopoEdge e);
	void      Erase();

	vector<GeomVert>  mGeomVerts;
	vector<TopoVert>  mTopoVerts;
	vector<TopoEdge>  mTopoEdges;
	vector<TopoFacet> mTopoFacets;
};
// ------------------------------------------------------------
// ------------------------------------------------------------
// Mesh.cpp:  Implementation of mesh class
// ------------------------------------------------------------
// ------------------------------------------------------------
// AddFacet:  Adds a triangle to the mesh.
//            This is one of 2 functions that can be used to build a mesh
// ------------------------------------------------------------
void Mesh::AddFacet(datatype x1, datatype y1, datatype z1, datatype x2, datatype y2, datatype z2, datatype x3, datatype y3, datatype z3) {

	vector<GeomVert> geomfacet;
	geomfacet.push_back(GeomVert(x1, y1, z1));
	geomfacet.push_back(GeomVert(x2, y2, z2));
	geomfacet.push_back(GeomVert(x3, y3, z3));

	AddFacet(geomfacet);
}
// ------------------------------------------------------------
// ------------------------------------------------------------
// AddFacet:  Adds a facet with arbitrary number of vertices to mesh
//            This is one of 2 functions that can be used to build a mesh
// ------------------------------------------------------------
void Mesh::AddFacet(vector<GeomVert> geomfacet) {
	int i;

	// --------------
	// Create topo facet (list of geom vertex indices)
	TopoFacet topofacet;
	// Look for facet vertices in mesh - if they don't already exist in mesh then add them
	for (i = 0; i < geomfacet.size(); i++) {
		int v_ind = FindGeomVertex(geomfacet[i]);
		if (v_ind == -1) {
			// New vertex:  add geomtric vertex
			v_ind = mGeomVerts.size();
			mGeomVerts.push_back(geomfacet[i]);

			// Add topo vertex
			TopoVert topovert;
			mTopoVerts.push_back(topovert);
		}

		// Add vertex indice to topo facet
		topofacet.AddIncVertex(v_ind);
	}

	// Add this new topo facet to mesh	
	int facet_ind = mTopoFacets.size();
	mTopoFacets.push_back(topofacet);


	// Add edges of facet to mesh, again checking if they already exist
	for (i = 0; i < topofacet.GetNumberVertices(); i++) {
		int prev = (i == 0) ? topofacet.GetNumberVertices() - 1 : i - 1;

		// Create edge
		TopoEdge e;
		e.SetVertex(0, topofacet.GetVertexInd(prev));
		e.SetVertex(1, topofacet.GetVertexInd(i));

		// Check if exists
		int e_ind = FindTopoEdge(e);

		if (e_ind == -1) {
			// Didn't exist, add to mesh
			e_ind = mTopoEdges.size();
			mTopoVerts[e.GetVertex(0)].AddIncEdge(e_ind);
			mTopoVerts[e.GetVertex(1)].AddIncEdge(e_ind);
			mTopoEdges.push_back(e);
		}

		// Point edge to this facet
		mTopoEdges[e_ind].AddIncFacet(facet_ind);

		// Point facet to this edge
		mTopoFacets[facet_ind].AddIncEdge(e_ind);
	}
	// --------------



	// Compute other connectivity
	for (i = 0; i < topofacet.GetNumberVertices(); i++) {
		// Add vertex-facet topology
		mTopoVerts[topofacet.GetVertexInd(i)].AddIncFacet(facet_ind);

		// Add vertex-vertex (edge) topology
		int prev = (i == 0) ? topofacet.GetNumberVertices() - 1 : i - 1;
		int next = (i == topofacet.GetNumberVertices() - 1) ? 0 : i + 1;

		mTopoVerts[topofacet.GetVertexInd(i)].AddIncVert(topofacet.GetVertexInd(prev));
		mTopoVerts[topofacet.GetVertexInd(i)].AddIncVert(topofacet.GetVertexInd(next));
	}

	// Facet-facet adjacency...
	for (i = 0; i < mTopoFacets[facet_ind].GetNumberEdges(); i++) {
		TopoEdge edge = mTopoEdges[mTopoFacets[facet_ind].GetIncEdge(i)];
		for (int j = 0; j < edge.GetNumberIncFacets(); j++) {
			if (edge.GetIncFacet(j) != facet_ind) {
				mTopoFacets[facet_ind].AddIncFacet(edge.GetIncFacet(j));
				mTopoFacets[edge.GetIncFacet(j)].AddIncFacet(facet_ind);
			}
		}
	}
}
// ------------------------------------------------------------
// ------------------------------------------------------------
// Erase:  Releases all memory used by object
// ------------------------------------------------------------
void Mesh::Erase() {
	mGeomVerts.clear();
	mTopoVerts.clear();
	mTopoEdges.clear();
	mTopoFacets.clear();
}
// ------------------------------------------------------------
// ------------------------------------------------------------
// FindGeomVertex:  Searches for a geometric vertex in the mesh,
//                  returning its indice if found, -1 otherwise
// ------------------------------------------------------------
int Mesh::FindGeomVertex(GeomVert v) {
	for (int i = 0; i < mGeomVerts.size(); i++) {
		if (mGeomVerts[i] == v) return i;
	}
	return -1;
}
// ------------------------------------------------------------
// ------------------------------------------------------------
// FindTopoEdge:  Searches for an edge in the mesh, returing 
//                its indice if found, -1 otherwise
// ------------------------------------------------------------
int	Mesh::FindTopoEdge(TopoEdge e) {
	for (int i = 0; i < mTopoEdges.size(); i++) {
		if (mTopoEdges[i] == e) return i;
	}
	return -1;
}
// ------------------------------------------------------------

///////////////////////////////////////////////////////////
// 03/26
static const double cube_data[] = {
	-1.0f,-1.0f,-1.0f, // triangle 1 : begin
	-1.0f,-1.0f, 1.0f,
	-1.0f, 1.0f, 1.0f, // triangle 1 : end
	1.0f, 1.0f,-1.0f, // triangle 2 : begin
	-1.0f,-1.0f,-1.0f,
	-1.0f, 1.0f,-1.0f, // triangle 2 : end
	1.0f,-1.0f, 1.0f,
	-1.0f,-1.0f,-1.0f,
	1.0f,-1.0f,-1.0f,
	1.0f, 1.0f,-1.0f,
	1.0f,-1.0f,-1.0f,
	-1.0f,-1.0f,-1.0f,
	-1.0f,-1.0f,-1.0f,
	-1.0f, 1.0f, 1.0f,
	-1.0f, 1.0f,-1.0f,
	1.0f,-1.0f, 1.0f,
	-1.0f,-1.0f, 1.0f,
	-1.0f,-1.0f,-1.0f,
	-1.0f, 1.0f, 1.0f,
	-1.0f,-1.0f, 1.0f,
	1.0f,-1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f,-1.0f,-1.0f,
	1.0f, 1.0f,-1.0f,
	1.0f,-1.0f,-1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f,-1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f, 1.0f,-1.0f,
	-1.0f, 1.0f,-1.0f,
	1.0f, 1.0f, 1.0f,
	-1.0f, 1.0f,-1.0f,
	-1.0f, 1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	-1.0f, 1.0f, 1.0f,
	1.0f,-1.0f, 1.0f
};

double const cube_data2[] = {
	1, 1, 1,
	-1, 1, 1,
	-1, -1, 1,
	1, -1, 1,
	1, 1, -1,
	-1, 1, -1,
	-1, -1, -1,
	1, -1, -1
};

unsigned int vao_cube;
unsigned int vbo_cube;

unsigned int vao_bcS;
unsigned int vbo_bcS;
vector<GLdouble> bc_surface_vertices;

unsigned int vao_SBS;
unsigned int vbo_SBS;
vector<GLdouble> SB_surface_vertices;




// BitmapPrinter: For drawing text using GLUT Bitmap facility
// Usage: Bitmap t(-0.9, 0.9, 0.1)
// start postion of text (x, y, height)
// t.print("Hello"): texting on (-0.9, 0.9) first line
// t.print("World"): texting on (-0.9, 0.8) second line
class BitmapPrinter
{
private:
	// cursx, cursy: initial raster position for text line
	// lineh: the reduce amount of each line
	double cursx, cursy;
	double lineh;
public:
	BitmapPrinter(double Cursx = 0.0, double Cursy = 0.0, double lineH = 0.1) {
		setup(Cursx, Cursy, lineH);
	}
	void setup(double Cursx, double Cursy, double lineH = 0.1) {
		cursx = Cursx;
		cursy = Cursy;
		lineh = lineH;
	}
	// print 
	// Draw the given string using glutBitmapCharacter,
	// with GLUT's 9x15 font, at (cursx, cursy)
	// and reduce cursy by lineh (move to next line)
	// the model/view transformation should be identity
	// (or just translations) when calling this function
	void print(const string &message) {
		glRasterPos2d(cursx, cursy);
		for (string::const_iterator iter = message.begin();
			iter != message.end(); iter++)
		{
			glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *iter);
		}
		cursy = cursy - lineh;
	}
};

// A vertex array object for the points
// A vertex buffer object for the vertices
// A Shader program
unsigned int vao_verts;
unsigned int vbo_verts;
unsigned int BCProgramID;
unsigned int L_BCProgramID;


int width, height;
const int Max_Points = 512;
vector<GLdouble> vertices;

// 01/27
unsigned int vao_bcverts;
unsigned int vbo_bcverts;
vector<GLdouble> bc_vertices;
bool draw_bc;

unsigned int vao_spverts;
unsigned int vbo_spverts;
vector<GLdouble> sp_vertices;
bool draw_sp;

// 01/29
double sub_u = 0.5;
vector<GLdouble> poly1_vertices;
vector<GLdouble> poly2_vertices;
vector<GLdouble> poly_bs_vertices;

// 02/03
bool currentmoving;
int select_vert;


// 03/01
unsigned int vao_ob1verts;
unsigned int vbo_ob1verts;
vector<double> object1_vertices;

unsigned int vao_extrusion;
unsigned int vbo_extrusion;
vector<double> extrusion_vertices;

// 03/02
unsigned int lab2_ProgramID;
unsigned int lab2_LProgramID;


// 03/04
vector<GLdouble> SOR_vertices;
int SOR_number = 10;
double Extru_height = 1.0;

vector<GLdouble> Sweep_vertices;
vector<GLdouble> SBC_vertices;
unsigned int vao_sweep;
unsigned int vbo_sweep;

// 03/05
vector<GLdouble> vertices2;
unsigned int vao_bcverts2;
unsigned int vbo_bcverts2;
vector<GLdouble> bc_vertices2;



vector<GLdouble> f1vertices;
vector<GLdouble> f2vertices;
vector<GLdouble> f3vertices;




// 03/26
bool bcS1 = true;
bool bc_withboundary = false;
bool spS1 = true;
bool sp_withouboundary = true;


unsigned int lab3_Program_V;
unsigned int lab3_Program_Surface;

// 03/27

Mesh SOR_bc;
Mesh SOR_bc_NB;
Mesh SOR_Sp;
Mesh SOR_SP_NB;


// 03/29
Mesh hole;
Mesh Doo_Sabin_hole;
Mesh Catmull_Clark_hole;
Mesh Loop_hole;
unsigned int vao_hole;
unsigned int vbo_hole;
vector<glm::vec3> hole_vertices;

// 04/18
unsigned int vao_crust;
unsigned int vbo_crust;
vector<glm::vec3> crust_vertices;

unsigned int vao_NNcrust;
unsigned int vbo_NNcrust;
vector<glm::vec3> NNcrust_vertices;
bool reconstruction;

bool open_curve;

void outputOFF(Mesh *new_cone, char* filename);


vector<GLdouble> de_Casteljau(const vector<GLdouble> &v, double u) {
	if (v.size() == 3) {
		return v;
	}
	vector<GLdouble> new_v;
	for (int i = 0; i < v.size() / 3 - 1; i++)
	{

		GLdouble x = u * v[i * 3] + (1 - u)*v[i * 3 + 3];
		GLdouble y = u * v[i * 3 + 1] + (1 - u)*v[i * 3 + 4];
		GLdouble z = u * v[i * 3 + 2] + (1 - u)*v[i * 3 + 5];
		new_v.push_back(x);
		new_v.push_back(y);
		new_v.push_back(z);
	}
	return de_Casteljau(new_v, u);

}

void drawSOR(vector<GLdouble> &v);
void drawExtrusion(vector<GLdouble> &v, double z);
void drawSweep(vector<GLdouble> &v);

// drawBezierCurve (function)
void drawBezierCurve(const vector<GLdouble> &v, int segs)
{
	if (v.size() == 0 || v.size() == 3 || v.size() / 3 > 512) {
		return;
	}
	for (double u = 0.0; u <= 1; u = u + 1.0 / segs)
	{
		vector<GLdouble> tmp_v = de_Casteljau(v, u);
		bc_vertices.push_back(tmp_v[0]);
		bc_vertices.push_back(tmp_v[1]);
		bc_vertices.push_back(tmp_v[2]);
	}
	glUseProgram(L_BCProgramID);
	glBindVertexArray(vao_bcverts);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_bcverts);
	glBufferSubData(GL_ARRAY_BUFFER, 0, bc_vertices.size() * sizeof(GLdouble), &bc_vertices[0]);
	glDrawArrays(GL_LINE_STRIP, 0, bc_vertices.size() / 3);


	// 03/04
	for (int i = 0; i < bc_vertices.size(); i++) {
		SOR_vertices.push_back(bc_vertices[i]);
	}
	/*
	cout << "Now Drawing SOR" << endl;
	drawSOR(bc_vertices);
	cout << "Finish Drawing SOR" << endl;
	cout << "Now Drawing Extrusion" << endl;
	drawExtrusion(bc_vertices, 5.0);
	cout << "Finish Drawing Extrusion" << endl;
	drawSweep(bc_vertices);
	*/

	bc_vertices.clear();
};


// drawBezierCurve (function)
void drawBezierCurve2(const vector<GLdouble> &v, int segs)
{
	if (v.size() == 0 || v.size() == 3 || v.size() / 3 > 512) {
		return;
	}
	for (double u = 0.0; u <= 1; u = u + 1.0 / segs)
	{
		vector<GLdouble> tmp_v = de_Casteljau(v, u);
		bc_vertices2.push_back(tmp_v[0]);
		bc_vertices2.push_back(tmp_v[1]);
		bc_vertices2.push_back(tmp_v[2]);
	}
	glUseProgram(L_BCProgramID);
	glBindVertexArray(vao_bcverts2);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_bcverts2);
	glBufferSubData(GL_ARRAY_BUFFER, 0, bc_vertices2.size() * sizeof(GLdouble), &bc_vertices2[0]);
	glDrawArrays(GL_LINE_STRIP, 0, bc_vertices2.size() / 3);


	// 03/04
	for (int i = 0; i < bc_vertices2.size(); i++) {
		SBC_vertices.push_back(bc_vertices2[i]);
	}
	bc_vertices2.clear();
};


// 01/28
// functions for B-spline
double B0(double u) {
	double x = (1 - u)*(1 - u)*(1 - u) / 6;
	return x;
}
double B1(double u) {
	double x = (3 * u*u*u - 6 * u*u + 4) / 6;
	return x;
}
double B2(double u) {
	double x = (-3 * u*u*u + 3 * u*u + 3 * u + 1) / 6;
	return x;
}
double B3(double u) {
	double x = u * u*u / 6;
	return x;
}


// 01/27
// draw B-spline (function)
void drawBsplineCurve(const vector<GLdouble> &v, int segs)
{
	if (v.size() < 12 || v.size() / 3 > 512) {
		return;
	}
	// B-spline
	for (int i = 0; i < (v.size() / 3) - 3; i++)
	{
		GLdouble x1, x2, x3, x4;
		GLdouble y1, y2, y3, y4;

		x1 = v[i * 3];
		x2 = v[i * 3 + 3];
		x3 = v[i * 3 + 6];
		x4 = v[i * 3 + 9];

		y1 = v[i * 3 + 1];
		y2 = v[i * 3 + 4];
		y3 = v[i * 3 + 7];
		y4 = v[i * 3 + 10];
		for (double u = 0.0; u < 1.0; u = u + 1.0 / segs) {
			double x = B0(u)*x1 + B1(u)*x2
				+ B2(u)*x3 + B3(u)*x4;
			double y = B0(u)*y1 + B1(u)*y2
				+ B2(u)*y3 + B3(u)*y4;
			sp_vertices.push_back(x);
			sp_vertices.push_back(y);
			sp_vertices.push_back(0.0);
		}
	}

	glUseProgram(L_BCProgramID);
	glBindVertexArray(vao_spverts);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_spverts);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sp_vertices.size() * sizeof(GLdouble), &sp_vertices[0]);
	glDrawArrays(GL_LINE_STRIP, 0, sp_vertices.size() / 3);
	sp_vertices.clear();

};



// 01/29
void sub_dC(vector<GLdouble> &v, double u) {
	// if n = 0, poly1 add p0, and poly2 add p0
	if (v.size() == 0) {
		return;
	}
	if (v.size() == 3) {
		poly1_vertices.push_back(v[0]);
		poly1_vertices.push_back(v[1]);
		poly1_vertices.push_back(v[2]);
		poly2_vertices.push_back(v[0]);
		poly2_vertices.push_back(v[1]);
		poly2_vertices.push_back(v[2]);
	}

	else {
		poly1_vertices.push_back(v[0]);
		poly1_vertices.push_back(v[1]);
		poly1_vertices.push_back(v[2]);
		int size = v.size();
		poly2_vertices.push_back(v[size - 3]);
		poly2_vertices.push_back(v[size - 2]);
		poly2_vertices.push_back(v[size - 1]);

		vector<GLdouble> new_v;
		for (int i = 0; i < v.size() / 3 - 1; i++) {
			double x = (1 - u)*v[3 * i] + u * v[3 * i + 3];
			double y = (1 - u)*v[3 * i + 1] + u * v[3 * i + 4];
			new_v.push_back(x);
			new_v.push_back(y);
			new_v.push_back(0.0);
		}
		sub_dC(new_v, sub_u);

	}

}


void sub_dC_function() {
	if (vertices.size() <= 3) {
		return;
	}
	poly1_vertices.clear();
	poly2_vertices.clear();
	sub_dC(vertices, sub_u);

	vertices.clear();
	for (int i = 0; i < poly1_vertices.size() / 3; i++) {
		vertices.push_back(poly1_vertices[i * 3]);
		vertices.push_back(poly1_vertices[i * 3 + 1]);
		vertices.push_back(0.0);
	}
	for (int i = poly2_vertices.size() / 3 - 1; i >= 0; i--) {
		vertices.push_back(poly2_vertices[i * 3]);
		vertices.push_back(poly2_vertices[i * 3 + 1]);
		vertices.push_back(0.0);
	}
}


void sub_Bs(vector<GLdouble> &v) {
	if (v.size() <= 6) {
		return;
	}
	for (int i = 0; i < v.size() / 3 - 2; i++) {
		double x1 = (3 * v[3 * i] + v[3 * i + 3]) / 4;
		double y1 = (3 * v[3 * i + 1] + v[3 * i + 4]) / 4;

		double x2 = (v[3 * i] + 3 * v[3 * i + 3]) / 4;
		double y2 = (v[3 * i + 1] + 3 * v[3 * i + 4]) / 4;

		double x3 = (3 * v[3 * i + 3] + v[3 * i + 6]) / 4;
		double y3 = (3 * v[3 * i + 4] + v[3 * i + 7]) / 4;

		double x4 = (v[3 * i + 3] + 3 * v[3 * i + 6]) / 4;
		double y4 = (v[3 * i + 4] + 3 * v[3 * i + 7]) / 4;
		poly_bs_vertices.push_back(x1);
		poly_bs_vertices.push_back(y1);
		poly_bs_vertices.push_back(0.0);
		poly_bs_vertices.push_back(x2);
		poly_bs_vertices.push_back(y2);
		poly_bs_vertices.push_back(0.0);
		poly_bs_vertices.push_back(x3);
		poly_bs_vertices.push_back(y3);
		poly_bs_vertices.push_back(0.0);
		poly_bs_vertices.push_back(x4);
		poly_bs_vertices.push_back(y4);
		poly_bs_vertices.push_back(0.0);
	}
	v.clear();
	v = poly_bs_vertices;
	poly_bs_vertices.clear();
}


// Bspline surface

// 03/01
void drawSOR(vector<GLdouble> &v)
{
	if (v.size() == 0 || v.size() == 3) {
		return;
	}

	for (int i = 0; i < v.size() / 3; i++) {
		double r = v[3 * i];
		double z = v[3 * i + 1];
		if (r < 0) { r = -r; }
		for (double j = 0; j < 4; j++) {
			double x = r * sin(j*1.57);
			double y = r * cos(j*1.57);
			object1_vertices.push_back(x);
			object1_vertices.push_back(y);
			object1_vertices.push_back(z);
		}
	}
	////////////////////////////////////////////////////////
	// 03/28
	// output SOR Mesh
	Mesh SOR;
	for (int i = 0; i < object1_vertices.size() / 3 / 4; i++) {
		vector<GeomVert> geomfacet;
		for (int j = 0; j < 4; j++) {
			GeomVert t1(object1_vertices[12 * i + 3 * j],
				object1_vertices[12 * i + 3 * j + 1],
				object1_vertices[12 * i + 3 * j + 2]);
			geomfacet.push_back(t1);
		}
		SOR.AddFacet(geomfacet);
	}
	Mesh *SOR_ptr = &SOR;
	outputOFF(SOR_ptr, "SOR.off");
	SOR.~Mesh();
	/*
	//////////////////////////////////////
	glUseProgram(lab3_Program_V);
	// draw surface of revolution
	//  set up modelview and projection matrix
	glm::mat4 projectionmatrix = glm::perspective(glm::radians(45.0f), (float)width / (float)height, 0.1f, 100.0f);
	glm::mat4 viewmatrix = glm::lookAt(
		glm::vec3(1, 1, 1), // Camera is at (4,3,3), in World Space
		glm::vec3(0, 0, 0), // and looks at the origin
		glm::vec3(0, -1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
	);
	glm::mat4 modelmatrix = glm::mat4(10.0f);
	glm::mat4 mvp = projectionmatrix * viewmatrix * modelmatrix;
	GLuint mvpIdx = glGetUniformLocation(lab3_Program_V, "MVP");
	glUniformMatrix4fv(mvpIdx, 1, GL_FALSE, &mvp[0][0]);
	glBindVertexArray(vao_ob1verts);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_ob1verts);
	glBufferSubData(GL_ARRAY_BUFFER, 0, object1_vertices.size() * sizeof(GLdouble), &object1_vertices[0]);
	glDrawArrays(GL_POINTS, 0, object1_vertices.size() / 3);
	*/
	// draw the Bezier Surface
	bcS1 = true;
	if (bcS1) {
		for (double v = 0.0; v < 1.0; v = v + 0.1) {
			for (double u = 0.0; u < 1.0; u = u + 0.1) {
				vector<GLdouble> vertical;
				for (int i = 0; i < object1_vertices.size() / 3 / 4; i = i++) {
					vector<GLdouble> horizon;
					for (int j = i * 3 * 4; j < (i + 1) * 3 * 4; j++) {
						horizon.push_back(object1_vertices[j]);
					}
					// without boundary
					if (bc_withboundary) {
						horizon.push_back(object1_vertices[i * 3 * 4]);
						horizon.push_back(object1_vertices[i * 3 * 4 + 1]);
						horizon.push_back(object1_vertices[i * 3 * 4 + 2]);
					}

					vector<GLdouble> tmp = de_Casteljau(horizon, u);
					vertical.push_back(tmp[0]);
					vertical.push_back(tmp[1]);
					vertical.push_back(tmp[2]);
				}
				vector<GLdouble> result = de_Casteljau(vertical, v);

				bc_surface_vertices.push_back(result[0]);
				bc_surface_vertices.push_back(result[1]);
				bc_surface_vertices.push_back(result[2]);

			}
		}
		for (int i = 0; i < bc_surface_vertices.size() / 3 / 11; i++) {
			vector<GeomVert> geomfacet;
			for (int j = 0; j < 11; j++) {
				GeomVert t1(bc_surface_vertices[33 * i + 3 * j],
					bc_surface_vertices[33 * i + 3 * j + 1],
					bc_surface_vertices[33 * i + 3 * j + 2]);
				geomfacet.push_back(t1);
			}
			SOR_bc.AddFacet(geomfacet);
		}
		Mesh *SOR_bc_ptr = &SOR_bc;
		outputOFF(SOR_bc_ptr, "SOR_bc.off");
	}
	spS1 = true;
	if (spS1) {
		vector<GLdouble> Sp_surface;
		for (int i = 0; i < 97; i++) {
			vector<GLdouble> tmp_vertices;
			for (int j = 12 * i; j < 12 * i + 48; j++) {
				tmp_vertices.push_back(object1_vertices[j]);
			}
			for (double v = 0; v <= 1.0; v = v + 1) {
				for (double u = 0; u <= 1.0; u = u + 1) {
					double u_vector[4] = { 1, u, u*u, u*u*u };
					double v_vector[4] = { 1, v, v*v, v*v*v };
					for (int k = 0; k < 3; k++) {
						double P[4][4] = {
							tmp_vertices[0 + k],tmp_vertices[3 + k],tmp_vertices[6 + k],tmp_vertices[9 + k],
							tmp_vertices[12 + k], tmp_vertices[15 + k], tmp_vertices[18 + k], tmp_vertices[21 + k],
							tmp_vertices[24 + k], tmp_vertices[27 + k], tmp_vertices[30 + k], tmp_vertices[33 + k],
							tmp_vertices[36 + k], tmp_vertices[39 + k], tmp_vertices[42 + k], tmp_vertices[45 + k] };
						double M[4][4] = {
							1, 4, 1, 0,
							-3, 0, 3, 0,
							3, -6, 3, 0,
							-1, 3, -3, 1
						};
						double t_M[4][4] = {
							1, -3, 3, -1,
							4, 0,-6, 3,
							1, 3, 3, -3,
							0, 0, 0, 1
						};
						double u_M[4];
						for (int i = 0; i < 4; i++) {
							u_M[i] = u_vector[0] * M[0][i]
								+ u_vector[1] * M[1][i]
								+ u_vector[2] * M[2][i]
								+ u_vector[3] * M[3][i];
						}
						double u_M_P[4];
						for (int i = 0; i < 4; i++) {
							u_M_P[i] = u_M[0] * P[0][i]
								+ u_M[1] * P[1][i]
								+ u_M[2] * P[2][i]
								+ u_M[3] * P[3][i];
						}
						double u_M_P_t_M[4];
						for (int i = 0; i < 4; i++) {
							u_M_P_t_M[i] = u_M_P[0] * t_M[0][i]
								+ u_M_P[1] * t_M[1][i]
								+ u_M_P[2] * t_M[2][i]
								+ u_M_P[3] * t_M[3][i];
						}
						double result = u_M_P_t_M[0] * v_vector[0]
							+ u_M_P_t_M[1] * v_vector[1]
							+ u_M_P_t_M[2] * v_vector[2]
							+ u_M_P_t_M[3] * v_vector[3];
						Sp_surface.push_back(result / 36);
					}
				}
			}
		}
		Mesh SOR_sp;
		for (int i = 0; i < Sp_surface.size() / 3 / 4; i++) {
			vector<GeomVert> geomfacet;
			for (int j = 0; j < 4; j++) {
				GeomVert t1(Sp_surface[12 * i + 3 * j],
					Sp_surface[12 * i + 3 * j + 1],
					Sp_surface[12 * i + 3 * j + 2]);
				geomfacet.push_back(t1);
			}
			SOR_sp.AddFacet(geomfacet);
		}
		Mesh *SOR_sp_ptr = &SOR_sp;
		outputOFF( SOR_sp_ptr, "SOR_sp.off");
		SOR_sp.~Mesh();



	}




	for (int i = 0; i < object1_vertices.size(); i++) {
		f1vertices.push_back(object1_vertices[i]);
	}
	object1_vertices.clear();
}

void drawExtrusion(vector<GLdouble> &v, double z) {

	if (v.size() <= 6 || v.size() / 3 > 512) {
		return;
	}
	if (z <= 0) {
		return;
	}
	Mesh Extrusion;
	for (double j = 0.0; j <= z; j = j + 0.1) {
		vector<GeomVert> tmp_vertex;
		for (int i = 0; i < v.size() / 3; i++) {
			double x = v[3 * i];
			double y = v[3 * i + 1];
			extrusion_vertices.push_back(x);
			extrusion_vertices.push_back(y);
			extrusion_vertices.push_back(j);
			GeomVert tmp(x, y, j);
			tmp_vertex.push_back(tmp);
		}
		Extrusion.AddFacet(tmp_vertex);
	}
	Mesh *Extrusion_ptr = &Extrusion;
	outputOFF(Extrusion_ptr, "Extrusion.off");
	/*
	//glUseProgram(lab2_ProgramID);
	glUseProgram(lab2_LProgramID);
	//  set up modelview and projection matrix
	glm::mat4 projectionmatrix = glm::ortho(-5.0f, 5.0f, -10.0f, 10.0f, 0.0f, 100.0f);
	glm::mat4 viewmatrix = glm::lookAt(
		glm::vec3(0, 50, 50), // Camera is at (4,3,3), in World Space
		glm::vec3(0, 0, 0), // and looks at the origin
		glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
	);
	glm::mat4 modelmatrix = glm::mat4(1.0f);
	glm::mat4 mvp = projectionmatrix * viewmatrix * modelmatrix;
	GLuint mvpIdx = glGetUniformLocation(lab2_LProgramID, "MVP");
	//GLuint mvpIdx = glGetUniformLocation(lab2_ProgramID, "MVP");
	glUniformMatrix4fv(mvpIdx, 1, GL_FALSE, &mvp[0][0]);

	glBindVertexArray(vao_extrusion);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_extrusion);
	glBufferSubData(GL_ARRAY_BUFFER, 0, extrusion_vertices.size() * sizeof(GLdouble), &extrusion_vertices[0]);
	//glDrawArrays(GL_POINTS, 0, extrusion_vertices.size()/3);

	glDrawArrays(GL_TRIANGLE_FAN, 0, extrusion_vertices.size() / 3);
	*/


	for (int i = 0; i < extrusion_vertices.size(); i++) {
		f2vertices.push_back(extrusion_vertices[i]);
	}

	extrusion_vertices.clear();

}


void drawSweep(vector<GLdouble> &v) {

	if (v.size() <= 6 || v.size() / 3 > 512) {
		return;
	}
	if (SBC_vertices.size() <= 6) {
		return;
	}
	Mesh Sweep;
	for (int j = 0.0; j < SBC_vertices.size() / 3; j = j + 10) {
		double curve_x = SBC_vertices[3 * j] + 1;
		double curve_y = SBC_vertices[3 * j + 1];
		vector<GeomVert> tmp_vertex;
		if (curve_y < 0) {
			curve_y = -curve_y;
		}
		for (int i = 0; i < v.size() / 3; i++) {
			double x = v[3 * i];
			double y = v[3 * i + 1];
			x = x * curve_y;
			y = y * curve_y;
			Sweep_vertices.push_back(x);
			Sweep_vertices.push_back(y);
			Sweep_vertices.push_back(curve_x);
			GeomVert tmp(x, y, curve_x);
			tmp_vertex.push_back(tmp);
		}
		Sweep.AddFacet(tmp_vertex);
	}
	Mesh *Sweep_ptr = &Sweep;
	outputOFF(Sweep_ptr, "Sweep.off");
	/*
	//glUseProgram(lab2_ProgramID);
	glUseProgram(lab2_LProgramID);
	//  set up modelview and projection matrix
	glm::mat4 projectionmatrix = glm::ortho(-5.0f, 5.0f, -10.0f, 10.0f, 0.0f, 100.0f);
	glm::mat4 viewmatrix = glm::lookAt(
		glm::vec3(0, 1, 5), // Camera is at (4,3,3), in World Space
		glm::vec3(0, 0, 0), // and looks at the origin
		glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
	);
	glm::mat4 modelmatrix = glm::mat4(1.0f);
	glm::mat4 mvp = projectionmatrix * viewmatrix * modelmatrix;
	GLuint mvpIdx = glGetUniformLocation(lab2_LProgramID, "MVP");
	//GLuint mvpIdx = glGetUniformLocation(lab2_ProgramID, "MVP");
	glUniformMatrix4fv(mvpIdx, 1, GL_FALSE, &mvp[0][0]);

	glBindVertexArray(vao_sweep);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_sweep);
	glBufferSubData(GL_ARRAY_BUFFER, 0, Sweep_vertices.size() * sizeof(GLdouble), &Sweep_vertices[0]);
	glDrawArrays(GL_TRIANGLE_FAN, 0, Sweep_vertices.size() / 3);

	*/

	for (int i = 0; i < Sweep_vertices.size(); i++) {
		f3vertices.push_back(Sweep_vertices[i]);
	}

	Sweep_vertices.clear();

}

// loadFile - loads text file into char* 
// allocates memory - so need to delete after use
// size of file returned in fSize
char* loadFile(const char *fname, GLint &fSize)
{
	ifstream::pos_type size;
	char * memblock;
	string text;

	// file read based on example in cplusplus.com tutorial
	ifstream file(fname, ios::in | ios::binary | ios::ate);
	if (file.is_open())
	{
		size = file.tellg();
		fSize = (GLuint)size;
		memblock = new char[size];
		file.seekg(0, ios::beg);
		file.read(memblock, size);
		file.close();
		cout << "file " << fname << " loaded" << endl;
		text.assign(memblock);
	}
	else
	{
		cout << "Unable to open file " << fname << endl;
		exit(1);
	}
	return memblock;
}


// printShaderInfoLog
// From OpenGL Shading Language 3rd Edition, p215-216
// Display (hopefully) useful error messages if shader fails to compile
void printShaderInfoLog(GLint shader)
{
	int infoLogLen = 0;
	int charsWritten = 0;
	GLchar *infoLog;

	glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogLen);

	if (infoLogLen > 0)
	{
		infoLog = new GLchar[infoLogLen];
		// error check for fail to allocate memory omitted
		glGetShaderInfoLog(shader, infoLogLen, &charsWritten, infoLog);
		cout << "InfoLog:" << endl << infoLog << endl;
		delete[] infoLog;
	}
}

void printProgramInfoLog(GLint program)
{
	int infoLogLen = 0;
	int charsWritten = 0;
	GLchar *infoLog;

	glGetProgramiv(program, GL_INFO_LOG_LENGTH, &infoLogLen);

	if (infoLogLen > 0)
	{
		infoLog = new GLchar[infoLogLen];
		// error check for fail to allocate memory omitted
		glGetProgramInfoLog(program, infoLogLen, &charsWritten, infoLog);
		cout << "InfoLog:" << endl << infoLog << endl;
		delete[] infoLog;
	}
}

unsigned int createShader(const char* vertexShader, const char* fragmentShader, const char* geometryShader = 0)
{
	GLuint f, g, v;

	char *vs, *gs, *fs;
	GLint compiled;

	unsigned int programID = glCreateProgram();


	// load shaders & get length of each
	GLint vlen;
	v = glCreateShader(GL_VERTEX_SHADER);
	vs = loadFile(vertexShader, vlen);
	const char * vv = vs;
	glShaderSource(v, 1, &vv, &vlen);
	glCompileShader(v);
	glGetShaderiv(v, GL_COMPILE_STATUS, &compiled);
	if (!compiled)
	{
		cout << "Vertex shader (" << vertexShader << ") did not compiled." << endl;
		printShaderInfoLog(v);
		return 0;
	}
	glAttachShader(programID, v);

	if (geometryShader != 0)
	{
		GLint glen;
		g = glCreateShader(GL_GEOMETRY_SHADER);
		gs = loadFile(geometryShader, glen);
		const char * gg = gs;
		glShaderSource(g, 1, &gg, &glen);
		glCompileShader(g);
		glGetShaderiv(g, GL_COMPILE_STATUS, &compiled);
		if (!compiled)
		{
			cout << "Geometry shader  (" << geometryShader << ") did not compiled." << endl;
			printShaderInfoLog(g);
			return 0;
		}
		glAttachShader(programID, g);
	}

	GLint flen;
	f = glCreateShader(GL_FRAGMENT_SHADER);
	fs = loadFile(fragmentShader, flen);
	const char * ff = fs;
	glShaderSource(f, 1, &ff, &flen);
	glCompileShader(f);
	glGetShaderiv(f, GL_COMPILE_STATUS, &compiled);
	if (!compiled)
	{
		cout << "Fragment shader  (" << fragmentShader << ") did not compiled." << endl;
		printShaderInfoLog(f);
	}
	glAttachShader(programID, f);


	glBindAttribLocation(programID, 0, "in_Position");
	glBindFragDataLocation(programID, 0, "out_Color");

	GLint linked;
	glLinkProgram(programID);
	glGetProgramiv(programID, GL_LINK_STATUS, &linked);
	if (!linked)
	{
		cout << "Shader Program did not link." << endl;
		printShaderInfoLog(f);
	}

	delete[] vs; // dont forget to free allocated memory
	delete[] fs; // we allocated this in the loadFile function...
	if (geometryShader != 0)
		//delete [] gs; // we allocated this in the loadFile function...

		return programID;
}

void initOpenGlWindow(int argc, char* argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB);
	glutInitWindowSize(800, 800);
	int window1 = glutCreateWindow("5543 Su lab4");
	glewInit();
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		/* Problem: glewInit failed, something is seriously wrong. */
		cout << "glewInit failed, aborting." << endl;
		exit(1);
	}
	cout << "Status: Using GLEW " << glewGetString(GLEW_VERSION) << endl;
	cout << "OpenGL version " << glGetString(GL_VERSION) << " supported" << endl;

}

// 03/01
void initOpenGlWindow2()
{
	glutInitDisplayMode(GLUT_RGB);
	glutInitWindowSize(800, 800);
	glutCreateWindow("5543 Su lab2 instruction");
	glewInit();
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		/* Problem: glewInit failed, something is seriously wrong. */
		cout << "glewInit failed, aborting." << endl;
		exit(1);
	}
	cout << "Status: Using GLEW " << glewGetString(GLEW_VERSION) << endl;
	cout << "OpenGL version " << glGetString(GL_VERSION) << " supported" << endl;

}





void initModels(void)
{

	glGenVertexArrays(1, &vao_verts);
	glBindVertexArray(vao_verts);
	glGenBuffers(1, &vbo_verts);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_verts);
	glBufferData(GL_ARRAY_BUFFER, Max_Points * 500 * sizeof(GLdouble), 0, GL_DYNAMIC_DRAW);
	glVertexAttribPointer((GLuint)0, 3, GL_DOUBLE, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);
	// Reset
	glBindVertexArray(0);
	// 01/27
	// bezier curve
	glGenVertexArrays(1, &vao_bcverts);
	glBindVertexArray(vao_bcverts);
	glGenBuffers(1, &vbo_bcverts);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_bcverts);
	glBufferData(GL_ARRAY_BUFFER, Max_Points * 500 * sizeof(GLdouble), 0, GL_DYNAMIC_DRAW);
	glVertexAttribPointer((GLuint)0, 3, GL_DOUBLE, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	// B-spline
	glGenVertexArrays(1, &vao_spverts);
	glBindVertexArray(vao_spverts);
	glGenBuffers(1, &vbo_spverts);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_spverts);
	glBufferData(GL_ARRAY_BUFFER, Max_Points * 500 * sizeof(GLdouble), 0, GL_DYNAMIC_DRAW);
	glVertexAttribPointer((GLuint)0, 3, GL_DOUBLE, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);


	// Object1
	glGenVertexArrays(1, &vao_ob1verts);
	glBindVertexArray(vao_ob1verts);
	glGenBuffers(1, &vbo_ob1verts);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_ob1verts);
	glBufferData(GL_ARRAY_BUFFER, Max_Points * 1000 * sizeof(GLdouble), 0, GL_DYNAMIC_DRAW);
	glVertexAttribPointer((GLuint)0, 3, GL_DOUBLE, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);


	// Extrusion
	glGenVertexArrays(1, &vao_extrusion);
	glBindVertexArray(vao_extrusion);
	glGenBuffers(1, &vbo_extrusion);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_extrusion);
	glBufferData(GL_ARRAY_BUFFER, Max_Points * 500 * sizeof(GLdouble), 0, GL_DYNAMIC_DRAW);
	glVertexAttribPointer((GLuint)0, 3, GL_DOUBLE, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	// bezier curve
	glGenVertexArrays(1, &vao_bcverts2);
	glBindVertexArray(vao_bcverts2);
	glGenBuffers(1, &vbo_bcverts2);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_bcverts2);
	glBufferData(GL_ARRAY_BUFFER, Max_Points * 500 * sizeof(GLdouble), 0, GL_DYNAMIC_DRAW);
	glVertexAttribPointer((GLuint)0, 3, GL_DOUBLE, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);




	// Sweep
	glGenVertexArrays(1, &vao_sweep);
	glBindVertexArray(vao_sweep);
	glGenBuffers(1, &vbo_sweep);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_sweep);
	glBufferData(GL_ARRAY_BUFFER, Max_Points * 500 * sizeof(GLdouble), 0, GL_DYNAMIC_DRAW);
	glVertexAttribPointer((GLuint)0, 3, GL_DOUBLE, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	//////////////////////////////////////////////////////////////
	// 03/26
	// BC_surface
	glGenVertexArrays(1, &vao_bcS);
	glBindVertexArray(vao_bcS);
	glGenBuffers(1, &vbo_bcS);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_bcS);
	glBufferData(GL_ARRAY_BUFFER, Max_Points * 500 * sizeof(GLdouble), 0, GL_DYNAMIC_DRAW);
	glVertexAttribPointer((GLuint)0, 3, GL_DOUBLE, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);


	// 04/18
	// crust
	glGenVertexArrays(1, &vao_crust);
	glBindVertexArray(vao_crust);
	glGenBuffers(1, &vbo_crust);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_crust);
	glBufferData(GL_ARRAY_BUFFER, Max_Points * 500 * sizeof(glm::vec3), 0, GL_DYNAMIC_DRAW);
	glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);
	// NN-crust
	glGenVertexArrays(1, &vao_NNcrust);
	glBindVertexArray(vao_NNcrust);
	glGenBuffers(1, &vbo_NNcrust);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_NNcrust);
	glBufferData(GL_ARRAY_BUFFER, Max_Points * 500 * sizeof(glm::vec3), 0, GL_DYNAMIC_DRAW);
	glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);



	// Reset 
	glBindVertexArray(0);
}

void initShaders()
{
	BCProgramID = createShader("bc.vert", "bc.frag", "bc.geom");
	L_BCProgramID = createShader("bc_line.vert", "bc_line.frag");

	// 03/01
	lab2_ProgramID = createShader("lab2.vert", "lab2.frag", "lab2.geom");
	lab2_LProgramID = createShader("lab2_line.vert", "lab2_line.frag", "lab2_line.geom");
	lab3_Program_V = createShader("lab3.vert", "lab3.frag", "lab3.geom");
	lab3_Program_Surface = createShader("lab3S.vert", "lab3S.frag", "lab3S.geom");
}


void initBlending()
{
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void initScene(void)
{
	glClearColor(0.5f, 0.5f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	initShaders();
	initModels();
	initBlending();
	// 01/27
	draw_bc = false;
	//draw_sp = true;
	// 02/03
	currentmoving = false;
	reconstruction = false;
	open_curve = true;
}
// 04/17

void clean_circularity(std::vector<Point_2> &ordered_set)
{
	Point_2 first = ordered_set.front();
	Point_2 last = ordered_set.back();
	if (first == last)
	{
		//Delete the last point to clear circularity 
		ordered_set.pop_back();
	}

}
void curve_reconstruction_crust()
{
	crust_vertices.clear();
	// define two triangulation first
	Delaunay vor_dual;
	Delaunay del_fin;
	vector<Point_2> samples;
	// First construct the delaunay triangulation of the curve
	// put the points in voronoi diagram and delaunay triangulation
	for (int i = 0; i < vertices.size() / 3; i++)
	{
		glm::vec2 p;
		p.x = vertices[3 * i];
		p.y = vertices[3 * i + 1];
		Point_2 cgal_p(p.x, p.y);
		vor_dual.insert(cgal_p);
		del_fin.insert(cgal_p);
		samples.push_back(cgal_p);
	}
	// Go through all the voronoi vertices
	face_iter fit = vor_dual.faces_begin();
	for (; fit != vor_dual.faces_end(); fit++)
	{
		Point_2 vor_vertex = vor_dual.dual(fit);
		del_fin.insert(vor_vertex);
	}
	// go through all the egdes which are sample points in the delaunay triangulation
	std::deque<std::pair<Point_2, Point_2>> residues;
	vector<Point_2> ordered_samples;
	edge_iter eit = del_fin.edges_begin();
	int counter = 0;
	bool first_edge = true;
	for (; eit != del_fin.edges_end(); eit++)
	{
		Delaunay::Edge e = *eit;
		Delaunay::Vertex_handle v1 = e.first->vertex((e.second + 1) % 3);
		Delaunay::Vertex_handle v2 = e.first->vertex((e.second + 2) % 3);
		Point_2 p1 = v1->point();
		Point_2 p2 = v2->point();
		vector<Point_2>::iterator it;
		bool match_1_found = false; bool match_2_found = false;
		for (it = samples.begin(); it != samples.end(); it++)
		{
			Point_2 ref = *it;
			if (ref == p1) { match_1_found = true; }
			else if (ref == p2) { match_2_found = true; }
			if (match_1_found &&match_2_found) { break; }
		}
		if (match_1_found&&match_2_found)
		{
			if (first_edge)
			{
				ordered_samples.push_back(p1);
				ordered_samples.push_back(p2);
				first_edge = false;
			}
			else
			{
				vector<Point_2>::iterator my_iter;
				// Look for point 1
				my_iter = find(ordered_samples.begin(), ordered_samples.end(), p1);
				if (my_iter != ordered_samples.end())
				{
					// Check if it is the last point
					vector<Point_2>::iterator chk_iter = my_iter + 1;
					if (chk_iter == ordered_samples.end())
					{
						// It is the last point insert p2
						ordered_samples.push_back(p2);
					}
					else if (my_iter == ordered_samples.begin())
					{
						ordered_samples.insert(ordered_samples.begin(), p2);
					}
				}
				else
				{
					my_iter = find(ordered_samples.begin(), ordered_samples.end(), p2);
					if (my_iter != ordered_samples.end())
					{
						vector<Point_2>::iterator chk_iter = my_iter + 1;
						if (chk_iter == ordered_samples.end())
						{
							// it is the last point insert p1
							ordered_samples.push_back(p1);
						}
						else if (my_iter == ordered_samples.begin())
						{
							ordered_samples.insert(ordered_samples.begin(), p1);
						}
					}
					else
					{
						residues.push_back(std::make_pair(p1, p2));
					}
				}
			}
		}
		counter++;
	}
	// Now we take care of residues
	int time_out = 0;
	while (!residues.empty())
	{
		std::pair<Point_2, Point_2> cur_res = residues.front();
		residues.pop_front();
		Point_2 p1 = cur_res.first;
		Point_2 p2 = cur_res.second;
		Point_2 head = ordered_samples.front();
		Point_2 tail = ordered_samples.back();
		if (head == p1)
		{
			ordered_samples.insert(ordered_samples.begin(), p2);
		}
		else if (tail == p1)
		{
			ordered_samples.push_back(p2);
		}
		else if (head == p2)
		{
			ordered_samples.insert(ordered_samples.begin(), p1);
		}
		else if (tail == p2)
		{
			ordered_samples.push_back(p1);
		}
		else
		{
			residues.push_back(cur_res);
		}
		time_out++;
		if (time_out == TIMEOUTCNTR) { break; }
	}
	if (always_open_recons)
	{
		clean_circularity(ordered_samples);
	}
	std::vector<Point_2>::iterator it;
	for (it = ordered_samples.begin(); it != ordered_samples.end(); ++it)
	{
		glm::vec3 tmp(it->x(), it->y(), 0.0);
		crust_vertices.push_back(tmp);
	}


}

void curve_reconstruction_NN_crust()
{
	NNcrust_vertices.clear();
	// if there is only one vertex, there is no reason to reconstruct
	if (vertices.size() <= 3) { return; }
	vector<std::pair<int, int>> vertex_pairs;
	// vertex_pairs: unsorted vertex pair
	// transfer the data type into glm::vec3
	vector<glm::vec3> samples;
	for (int i = 0; i < vertices.size() / 3; i++)
	{
		glm::vec3 tmp;
		tmp.x = vertices[3 * i];
		tmp.y = vertices[3 * i + 1];
		tmp.z = 0;
		samples.push_back(tmp);
	}
	for (int i = 0; i < samples.size(); i++)
	{
		double shortest_distance = 1000.0;
		int index1 = i; int index2;
		// find the shortest edge: pq (i, j)
		for (int j = 0; j < samples.size(); j++)
		{
			if (i != j)
			{
				double x_distance, y_distance;
				x_distance = (samples[i].x - samples[j].x);
				y_distance = (samples[i].y - samples[j].y);
				double distance = sqrt(x_distance*x_distance + y_distance * y_distance);
				if (distance < shortest_distance)
				{
					shortest_distance = distance;
					index2 = j;
				}
			}
		}
		std::pair<int, int> tmp;
		tmp.first = index1;
		tmp.second = index2;
		vertex_pairs.push_back(tmp);
		// find the second shortest edge ps: (i, j)
		if (samples.size() >= 3) {
			int index_p = index1; int index_q = index2; int index_s = 10000;
			shortest_distance = 100.0f;
			for (int k = 0; k < samples.size(); k++) {
				if (k != i && k != index_q)
				{
					double x_distance, y_distance;
					x_distance = (samples[i].x - samples[k].x);
					y_distance = (samples[i].y - samples[k].y);
					double distance = sqrt(x_distance*x_distance + y_distance * y_distance);
					if (distance < shortest_distance)
					{
						shortest_distance = distance;
						index_s = k;
					}
				}
			}

			// if angle pqs >90 degree, add edge ps
			double pq, qs, ps;

			glm::vec3 pq_d = samples[index_p] - samples[index_q];
			pq = pq_d.x*pq_d.x + pq_d.y*pq_d.y;
			glm::vec3 qs_d = samples[index_q] - samples[index_s];
			qs = qs_d.x*qs_d.x + qs_d.y*qs_d.y;
			glm::vec3 ps_d = samples[index_p] - samples[index_s];
			ps = ps_d.x*ps_d.x + ps_d.y*ps_d.y;

			if (qs > (pq + ps))
			{
				std::pair<int, int> tmp;
				tmp.first = index_p;
				tmp.second = index_s;
				vertex_pairs.push_back(tmp);
			}


		}
	}
	// delete the same edge such like pair(0, 1), pair(1, 0)
	vector<std::pair<int, int>> order_pairs;
	for (int i = 0; i < vertex_pairs.size(); i++)
	{
		int index1 = vertex_pairs[i].first;
		int index2 = vertex_pairs[i].second;
		bool repeat = false;
		for (int j = 0; j < order_pairs.size(); j++)
		{
			int new_index1 = vertex_pairs[j].first;
			int new_index2 = vertex_pairs[j].second;
			if (index1 == new_index2 && index2 == new_index1)
			{
				repeat = true;
			}
			if (index1 == new_index1 && index2 == new_index2)
			{
				repeat = true;
			}

		}
		if (!repeat) {
			std::pair<int, int> tmp;
			tmp.first = vertex_pairs[i].first;
			tmp.second = vertex_pairs[i].second;
			order_pairs.push_back(tmp);
		}
	}
	for (int i = 0; i<order_pairs.size(); i++)
	{
		int index1 = order_pairs[i].first;
		int index2 = order_pairs[i].second;
		for (int j = 0; j < order_pairs.size(); j++)
		{
			if (i != j)
			{
				int new_index_1 = order_pairs[j].first;
				int new_index_2 = order_pairs[j].second;
				if ((index1 == new_index_1) && (index2 == new_index_2))
				{
					order_pairs.erase(order_pairs.begin() + j);
				}
				if ((index1 == new_index_2) && (index2 == new_index_1))
				{
					order_pairs.erase(order_pairs.begin() + j);
				}
			}
		}
	}

	// let the vertex in correct order
	vector<int> ordered_index;
	int index1 = order_pairs[0].first;
	int index2 = order_pairs[0].second;
	order_pairs.erase(order_pairs.begin());
	ordered_index.push_back(index1);
	ordered_index.push_back(index2);

	int count = 0;
	while (order_pairs.size() != 0 && count<100) {
		int length = ordered_index.size();
		int first = ordered_index[0];
		int last = ordered_index[length - 1];
		int pair_index;
		for (int i = 0; i < order_pairs.size(); i++)
		{
			index1 = order_pairs[i].first;
			index2 = order_pairs[i].second;

			if (index1 == last) {
				ordered_index.push_back(index2);
				pair_index = i;
				order_pairs.erase(order_pairs.begin() + pair_index);
				break;
			}
			if (index2 == last)
			{
				ordered_index.push_back(index1);
				pair_index = i;
				order_pairs.erase(order_pairs.begin() + pair_index);
				break;
			}
			if (index1 == first)
			{
				ordered_index.insert(ordered_index.begin(), index2);
				pair_index = i;
				order_pairs.erase(order_pairs.begin() + pair_index);
				break;
			}
			if (index2 == first)
			{
				ordered_index.insert(ordered_index.begin(), index1);
				pair_index = i;
				order_pairs.erase(order_pairs.begin() + pair_index);
				break;
			}

		}
		count++;
	}
	for (int i = 0; i < ordered_index.size(); i++)
	{
		int index = ordered_index[i];
		glm::vec3 tmp = samples[index];
		NNcrust_vertices.push_back(tmp);
	}
}

// display only 2D plane first
/*
void display(void)
{
glEnable(GL_SCISSOR_TEST);
// 03/04
// left up scene
glViewport(0, 400, 400, 400);
glScissor(0, 400, 400, 400);
glClear(GL_COLOR_BUFFER_BIT);
glClearColor(0.3, 0.4, 0.3, 1.0);
glColor3d(0.0, 0.0, 0.0);
BitmapPrinter t(-0.9, 0.9, 0.1);
t.print("y-z plane for Surface of revolution");
t.print("x-y plane for Extrusion");
t.print("x-y plane for Sweep Operators");
// Draw bezier curve
if (draw_bc) {
glColor3d(0.0, 0.0, 1.0);
t.print("Now Drawing Bezier Curve");
drawBezierCurve(vertices, 100);
}
// Draw B-spline curve
if (draw_sp) {
glColor3d(0.5, 0.0, 0.0);
t.print("Now Drawing B-Spline Curve");
drawBsplineCurve(vertices, 100);
}
if (vertices.size() <= 0)
return;

// draw points
glUseProgram(BCProgramID);
glBindVertexArray(vao_verts);
glBindBuffer(GL_ARRAY_BUFFER, vbo_verts);
glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(GLdouble), &vertices[0]);
glDrawArrays(GL_POINTS, 0, vertices.size() / 3);
// draw lines
glColor3d(0.0, 0.0, 0.0);
glUseProgram(L_BCProgramID);
glBindVertexArray(vao_verts);
glBindBuffer(GL_ARRAY_BUFFER, vbo_verts);
glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(GLdouble), &vertices[0]);
glDrawArrays(GL_LINE_STRIP, 0, vertices.size() / 3);
glBindVertexArray(0);
glUseProgram(0);
///////////////////////////////////////////////////////////////
// right down scene surface of revolution
int viewportSize = 400;
glViewport(0, 0, viewportSize, viewportSize);
glScissor(0, 0, viewportSize, viewportSize);
glClear(GL_COLOR_BUFFER_BIT);
glClearColor(0.3, 0.4, 0.0, 1.0);
///////////////////////////////////////////////////////////
// draw surface of revolution
glColor3d(0.0, 0.0, 0.0);
BitmapPrinter k(-0.9, 0.9, 0.1);
k.print("Now Drawing Surface of Revolution");
string x = patch::to_string(SOR_number);
x = "the number of slices : 4";
k.print(x);
//drawSOR(SOR_vertices, SOR_number, bcS1);
//SOR_vertices.clear();
/////////////////////////////////////////////////////////////////
glViewport(400, 0, viewportSize, viewportSize);
glScissor(400, 0, viewportSize, viewportSize);
glClear(GL_COLOR_BUFFER_BIT);
glClearColor(0.5, 0.4, 0.6, 1.0);
glColor3d(0.0, 0.0, 0.0);
BitmapPrinter g(-0.9, 0.9, 0.1);
g.print("Please click here and input .off file");

/*
// middle down scene Extrusion
glViewport(400, 0, viewportSize, viewportSize);
glScissor(400, 0, viewportSize, viewportSize);
glClear(GL_COLOR_BUFFER_BIT);
glClearColor(0.5, 0.4, 0.6, 1.0);
glColor3d(0.0, 0.0, 0.0);
BitmapPrinter g(-0.9, 0.9, 0.1);
g.print("Now Drawing Extrusion");
string y = patch::to_string(Extru_height);
y = " z-depth of the extrusion: " + y;
g.print(y);
drawExtrusion(vertices, Extru_height);
////////////////////////////////////////////////
// middle up scene
// another Bezier Curve
glViewport(400, 400, viewportSize, viewportSize);
glScissor(400, 400, viewportSize, viewportSize);
glClear(GL_COLOR_BUFFER_BIT);
glClearColor(0.4, 0.0, 0.1, 1.0);
glColor3d(0.0, 0.0, 0.0);
BitmapPrinter s1(-0.9, 0.9, 0.1);
s1.print("Now Drawing Bezier Curve");
s1.print("y-z plane for Sweep Operators");
glColor3d(0.0, 0.0, 1.0);
t.print("Now Drawing Bezier Curve");
drawBezierCurve2(vertices2, 100);
/////////////////////////////////////////////////////////
// right down scene Sweep Operators
glViewport(800, 0, viewportSize, viewportSize);
glScissor(800, 0, viewportSize, viewportSize);
glClear(GL_COLOR_BUFFER_BIT);
glClearColor(0.3, 0.3, 0.3, 1.0);
glColor3d(0.0, 0.0, 0.0);
BitmapPrinter s(-0.9, 0.9, 0.1);
s.print("Now Drawing sweep operators");
drawSweep(vertices);
// Be nice and reset things back to the default.
/////////////////////////////////////////////
// click for output the file
// Draw BC surface Here
glViewport(800, 400, viewportSize, viewportSize);
glScissor(800, 400, viewportSize, viewportSize);
glClear(GL_COLOR_BUFFER_BIT);
glClearColor(0.0, 1.0, 0.0, 1.0);
glColor3d(0.0, 0.0, 0.0);

//BitmapPrinter s2(-1.0, 0.0, 0.1);
//s2.print("Click here will output txt file.");
//s2.print("BC surface");

////////////////////////////////////
// drawBCsurface();
glBindVertexArray(0);
glUseProgram(0);
glFinish();

}
*/

void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glClearColor(0.5f, 0.5f, 0.0f, 1.0f);
	BitmapPrinter t(-0.9, 0.9, 0.1);
	// Draw bezier curve
	/*if (draw_bc) {
	glColor3d(0.0, 0.0, 1.0);
	t.print("Now Drawing Bezier Curve");
	drawBezierCurve(vertices, 100);
	}*/
	// Draw B-spline curve
	/*if (draw_sp) {
	glColor3d(0.5, 0.0, 0.0);
	t.print("Now Drawing B-Spline Curve");
	drawBsplineCurve(vertices, 100);
	}*/
	if (vertices.size() <= 0)
		return;
	// draw points
	glUseProgram(BCProgramID);
	glBindVertexArray(vao_verts);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_verts);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(GLdouble), &vertices[0]);
	glDrawArrays(GL_POINTS, 0, vertices.size() / 3);

	if (reconstruction) {
		glColor3d(0.0, 0.0, 0.0);
		t.print("Now Using Crust algorithm to reconstruct");
		t.print("Press keyboard N to change to NN-Crust");
		curve_reconstruction_crust();
		if (crust_vertices.size() > 0) {
			// draw Crust
			glColor3d(0.0, 0.0, 0.0);
			glUseProgram(L_BCProgramID);
			glBindVertexArray(vao_crust);
			glBindBuffer(GL_ARRAY_BUFFER, vbo_crust);
			glBufferSubData(GL_ARRAY_BUFFER, 0, crust_vertices.size() * sizeof(glm::vec3), &crust_vertices[0]);
			glDrawArrays(GL_LINE_STRIP, 0, crust_vertices.size());
			// Draw bezier curve
			if (draw_bc) {
				glColor3d(0.5, 0.0, 0.0);
				t.print("Now Drawing Bezier Curve for reconstruction");
				vector<GLdouble> crust_b;
				for (int i = 0; i < crust_vertices.size(); i++)
				{
					glm::vec3 tmp = crust_vertices[i];
					crust_b.push_back(tmp.x);
					crust_b.push_back(tmp.y);
					crust_b.push_back(tmp.z);
				}
				drawBezierCurve(crust_b, 100);
				
			}
			// Draw B-spline curve
			if (draw_sp) {
				glColor3d(0.0, 0.0, 0.5);
				t.print("Now Drawing B-Spline Curve for reconstruction");
				vector<GLdouble> crust_bs;
				for (int i = 0; i < crust_vertices.size(); i++)
				{
					glm::vec3 tmp = crust_vertices[i];
					crust_bs.push_back(tmp.x);
					crust_bs.push_back(tmp.y);
					crust_bs.push_back(tmp.z);
				}
				drawBsplineCurve(crust_bs, 100);
			}
		}
	}
	if (!reconstruction) {
		glColor3d(0.0, 0.0, 0.0);
		t.print("Now Using NN-Crust algorithm to reconstruct");
		t.print("Press keyboard N to change to Crust");
		curve_reconstruction_NN_crust();
		if (NNcrust_vertices.size() > 0) {
			glColor3d(0.0, 0.0, 0.0);
			glUseProgram(L_BCProgramID);
			glBindVertexArray(vao_NNcrust);
			glBindBuffer(GL_ARRAY_BUFFER, vbo_NNcrust);
			glBufferSubData(GL_ARRAY_BUFFER, 0, NNcrust_vertices.size() * sizeof(glm::vec3), &NNcrust_vertices[0]);
			glDrawArrays(GL_LINE_STRIP, 0, NNcrust_vertices.size());
			// Draw bezier curve
			if (draw_bc) {
				glColor3d(0.5, 0.0, 0.0);
				t.print("Now Drawing Bezier Curve for reconstruction");
				vector<GLdouble> NN_b;
				for (int i = 0; i < NNcrust_vertices.size(); i++)
				{
					glm::vec3 tmp = NNcrust_vertices[i];
					NN_b.push_back(tmp.x);
					NN_b.push_back(tmp.y);
					NN_b.push_back(tmp.z);
				}
				drawBezierCurve(NN_b, 100);
			}
			// Draw B-spline curve
			if (draw_sp) {
				glColor3d(0.0, 0.0, 0.5);
				t.print("Now Drawing B-Spline Curve for reconstruction");
				vector<GLdouble> NN_bs;
				for (int i = 0; i < NNcrust_vertices.size(); i++)
				{
					glm::vec3 tmp = NNcrust_vertices[i];
					NN_bs.push_back(tmp.x);
					NN_bs.push_back(tmp.y);
					NN_bs.push_back(tmp.z);
				}
				drawBsplineCurve(NN_bs, 100);
			}
		}
	}

	// draw lines
	/*
	glColor3d(0.0, 0.0, 0.0);
	glUseProgram(L_BCProgramID);
	glBindVertexArray(vao_verts);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_verts);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(GLdouble), &vertices[0]);
	glDrawArrays(GL_LINE_STRIP, 0, vertices.size() / 3);
	*/
	glBindVertexArray(0);
	glUseProgram(0);
	glFinish();
}

// 03/01
void display_2(void)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glClearColor(0.9f, 0.9f, 0.1f, 1.0f);
	glColor3d(0.5, 0., 0.0);
	BitmapPrinter t(-0.9, 0.9, 0.1);
	t.print("This is Su's 5543 lab2");
	t.print("Click to create new control point");
	t.print("Click on control point to move");
	t.print("B control Bezier Curve");
	t.print("S control B-Spline Curve");
	t.print("D control Subdivision curves (de Casteljau method)");
	t.print("Q control Subdivision Quadric B-spline");
	t.print("Space   Delete last control points");
	t.print("Click the subwindow to input z-depth of the extrusion");
	t.print("Or use keyboard 3 or 4 to increase/decrease it");

}



void reshape(int w, int h)
{
	width = w;
	height = h;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	// For debugging and a cleaner look
	glClear(GL_COLOR_BUFFER_BIT);
	glFinish();
	glutPostRedisplay();
}


/////////////////////////////////////////////////////////////
// function readOFF: .off file and store it into Mesh
Mesh readOFF(const char* filename) {
	int v_number; int f_number; int e_number;
	vector<glm::vec3> tmp_vertices;
	vector< unsigned int > vertexIndices;
	Mesh object;
	ifstream file;
	file.open(filename);
	string x;
	file >> x;
	if (x == "OFF") {
		file >> v_number; file >> f_number; file >> e_number;
	}
	for (int i = 0; i < v_number; i++) {
		glm::vec3 vertex;
		file >> vertex.x; file >> vertex.y; file >> vertex.z;
		tmp_vertices.push_back(vertex);
	}
	for (int i = 0; i < f_number; i++) {
		int number_F_vertex;
		// number_F_vertex: number of vertices in this face
		file >> number_F_vertex;
		if (number_F_vertex == 3) {
			int vertexIndex[3];
			file >> vertexIndex[0];
			file >> vertexIndex[1];
			file >> vertexIndex[2];
			glm::vec3 v1 = tmp_vertices[vertexIndex[0]];
			glm::vec3 v2 = tmp_vertices[vertexIndex[1]];
			glm::vec3 v3 = tmp_vertices[vertexIndex[2]];
			object.AddFacet(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, v3.x, v3.y, v3.z);
		}
		else {
			int vertexIndex[100];
			for (int i = 0; i < number_F_vertex; i++) {
				file >> vertexIndex[i];
			}
			vector<GeomVert> geomfacet;
			for (int i = 0; i < number_F_vertex; i++) {
				glm::vec3 v = tmp_vertices[vertexIndex[i]];
				GeomVert tmp_v(v.x, v.y, v.z);
				geomfacet.push_back(tmp_v);
			}
			object.AddFacet(geomfacet);
		}
	}
	return object;
}

/////////////////////////////////////////////////////////////
// function outputOFF: output Mesh to .off file
void outputOFF(Mesh *new_cone, char* filename) {
	ofstream conefile;
	int cone_v_number = new_cone->GetNumberVertices();
	int cone_f_number = new_cone->GetNumberFacets();
	conefile.open(filename);
	conefile << "OFF" << '\n';
	conefile << cone_v_number << " " << cone_f_number << " " << 0 << "\n";

	for (int i = 0; i < cone_v_number; i++) {
		GeomVert tmp = new_cone->GetGeomVertex(i);
		conefile << tmp.GetCo(0)
			<< " " << tmp.GetCo(1)
			<< " " << tmp.GetCo(2) << "\n";
	}

	for (int i = 0; i < cone_f_number; i++) {
		conefile << new_cone->GetFacet(i).GetNumberVertices() << " ";
		for (int j = 0; j < new_cone->GetFacet(i).GetNumberVertices(); j++) {
			conefile << new_cone->GetFacet(i).GetVertexInd(j) << " ";
		}
		conefile << "\n";
	}
	conefile.close();
}


// 03/30
////////////////////////////////////////////////////////////
// Function: doo_sabine_vf 
// Input: vertex_index, facet_index, Mesh
// Output: new vertex x, y, z
glm::vec3 doo_sabine_vf(int vertex_id, int facet_id, Mesh *mesh)
{
	// the new vertex = 
	// (the centroid of face (x)
	// + edge_v1 + edge_v2
	// + old_vertex) / 4
	// compute the centroid of face (x)
	glm::vec3 x(0.0, 0.0, 0.0);
	TopoFacet current_face = mesh->GetFacet(facet_id);
	int number_vertices = current_face.GetNumberVertices();
	for (int i = 0; i < number_vertices; i++) {
		int current_vertex_index = current_face.GetVertexInd(i);
		GeomVert current_vertex = mesh->GetGeomVertex(current_vertex_index);
		glm::vec3 p(current_vertex.GetCo(0), current_vertex.GetCo(1), current_vertex.GetCo(1));
		x = x + p;
	}
	x = x / (float)number_vertices;
	//Get the ve1 and ve2
	////////////////////////////////////////////////

	vector<int>edge_index;
	int number_edges = current_face.GetNumberEdges();
	for (int i = 0; i < number_edges; i++) {
		int current_edge_index = current_face.GetIncEdge(i);
		TopoEdge current_edge = mesh->GetEdge(current_edge_index);
		int edge_vertex_1 = current_edge.GetVertex(0);
		int edge_vertex_2 = current_edge.GetVertex(1);
		if (edge_vertex_1 == vertex_id || edge_vertex_2 == vertex_id)
		{
			edge_index.push_back(current_edge_index);
		}
	}

	////////////////////////////////////////////////
	TopoEdge e1 = mesh->GetEdge(edge_index[0]);
	TopoEdge e2 = mesh->GetEdge(edge_index[1]);
	GeomVert e1_1 = mesh->GetGeomVertex(e1.GetVertex(0));
	GeomVert e1_2 = mesh->GetGeomVertex(e1.GetVertex(1));
	GeomVert e2_1 = mesh->GetGeomVertex(e2.GetVertex(0));
	GeomVert e2_2 = mesh->GetGeomVertex(e2.GetVertex(1));
	glm::vec3 e11(e1_1.GetCo(0), e1_1.GetCo(1), e1_1.GetCo(2));
	glm::vec3 e12(e1_2.GetCo(0), e1_2.GetCo(1), e1_2.GetCo(2));
	glm::vec3 e21(e2_1.GetCo(0), e2_1.GetCo(1), e2_1.GetCo(2));
	glm::vec3 e22(e2_2.GetCo(0), e2_2.GetCo(1), e2_2.GetCo(2));
	glm::vec3 ve1 = 0.5f*(e11 + e12);
	glm::vec3 ve2 = 0.5f*(e21 + e22);
	GeomVert v_origin = mesh->GetGeomVertex(vertex_id);

	glm::vec3 v(v_origin.GetCo(0), v_origin.GetCo(1), v_origin.GetCo(2));

	glm::vec3 result = (x + v + ve1 + ve2) / (float)4;
	return result;
}

// Function: doo_sabine
// Input: old_Mesh, new_Mesh

void doo_sabine(Mesh* input, Mesh** output)
{
	//Create face-face
	int n_faces = input->GetNumberFacets();
	for (int i = 0; i<n_faces; i++)
	{
		TopoFacet face = input->GetFacet(i);
		std::vector<GeomVert> temp_face_buffer;
		int n_vertices_in_face = face.GetNumberVertices();
		for (int j = 0; j<n_vertices_in_face; j++)
		{
			int vert_id = face.GetVertexInd(j);
			glm::vec3 pnt = doo_sabine_vf(vert_id, i, input);
			temp_face_buffer.push_back(GeomVert(pnt.x, pnt.y, pnt.z));
		}
		(*output)->AddFacet(temp_face_buffer);
	}
	//Create edge-face

	int n_edges = input->GetNumberEdges();
	for (int i = 0; i<n_edges; i++)
	{
		TopoEdge edge = input->GetEdge(i);
		std::vector<GeomVert> temp_face_buffer;
		int n_faces_surrounding = edge.GetNumberIncFacets();
		if (n_faces_surrounding == 1)
			continue;
		else if (n_faces_surrounding == 2)
		{
			int face_id_1 = edge.GetIncFacet(0);
			int face_id_2 = edge.GetIncFacet(1);
			int vert_id_1 = edge.GetVertex(0);
			int vert_id_2 = edge.GetVertex(1);
			glm::vec3 v1f1 = doo_sabine_vf(vert_id_1, face_id_1, input);
			temp_face_buffer.push_back(GeomVert(v1f1.x, v1f1.y, v1f1.z));
			glm::vec3 v1f2 = doo_sabine_vf(vert_id_1, face_id_2, input);
			temp_face_buffer.push_back(GeomVert(v1f2.x, v1f2.y, v1f2.z));
			glm::vec3 v2f2 = doo_sabine_vf(vert_id_2, face_id_2, input);
			temp_face_buffer.push_back(GeomVert(v2f2.x, v2f2.y, v2f2.z));
			glm::vec3 v2f1 = doo_sabine_vf(vert_id_2, face_id_1, input);
			temp_face_buffer.push_back(GeomVert(v2f1.x, v2f1.y, v2f1.z));
		}
		else
		{
			std::cout << "Something is wrong here.";
		}
		(*output)->AddFacet(temp_face_buffer);
	}

	//Create vertex-face

	int n_vertices = input->GetNumberVertices();
	for (int i = 0; i<n_vertices; i++)
	{
		std::set<int> already_processed;
		TopoVert ver = input->GetVertex(i);
		std::vector<GeomVert> temp_face_buffer;
		int n_faces_surrounding = ver.GetNumberIncFacets();
		int seed = ver.GetIncFacet(0);
		bool next_seed_found = false;
		for (int j = 0; j<n_faces_surrounding; j++)
		{
			already_processed.insert(seed);
			// std::cout <<  seed << " ";
			next_seed_found = false;
			// int face_id = ver.GetIncFacet(seed);
			glm::vec3 pnt = doo_sabine_vf(i, seed, input);
			temp_face_buffer.push_back(GeomVert(pnt.x, pnt.y, pnt.z));
			//Generate the next seed
			//Look at all the surrounding faces of this face
			TopoFacet cur_seed = input->GetFacet(seed);
			int n_sur_face_cur_face = cur_seed.GetNumberFacets();
			for (int k = 0; k<n_sur_face_cur_face; k++)
			{
				int idx_facet = cur_seed.GetIncFacet(k);
				std::set<int>::iterator it;
				it = already_processed.find(idx_facet);
				if (it != already_processed.end())
					continue;
				//Now search of idx_facet in the this list
				for (int p = 0; p<n_faces_surrounding; p++)
				{
					int face_id_search = ver.GetIncFacet(p);
					if (face_id_search == idx_facet)
					{
						seed = idx_facet;
						next_seed_found = true;
						break;
						//p loop breaks
					}
				}
				if (next_seed_found)
					break;
				//k loop breaks
			}
		}
		(*output)->AddFacet(temp_face_buffer);
	}

}


glm::vec3 cc_vf(int facet_id, Mesh *mesh)
{
	glm::vec3 vf(0.0, 0.0, 0.0);
	TopoFacet current_face = mesh->GetFacet(facet_id);
	int number_vertex_face = current_face.GetNumberVertices();
	for (int i = 0; i < number_vertex_face; i++) {
		int vertex_index = current_face.GetVertexInd(i);
		GeomVert point = mesh->GetGeomVertex(vertex_index);
		glm::vec3 v(point.GetCo(0), point.GetCo(1), point.GetCo(2));
		vf = v + vf;
	}
	vf = vf / (float)number_vertex_face;
	return vf;
}
glm::vec3 cc_ve(int edge_id, Mesh *mesh)
{
	glm::vec3 ve(0.0, 0.0, 0.0);

	TopoEdge current_edge = mesh->GetEdge(edge_id);
	if (current_edge.GetNumberIncFacets() == 1)
	{
		int edge_v1_index = current_edge.GetVertex(0);
		int edge_v2_index = current_edge.GetVertex(1);
		int edge_f1_index = current_edge.GetIncFacet(0);
		GeomVert edge_v1 = mesh->GetGeomVertex(edge_v1_index);
		GeomVert edge_v2 = mesh->GetGeomVertex(edge_v2_index);
		glm::vec3 v_f1 = cc_vf(edge_f1_index, mesh);
		glm::vec3 e_v1(edge_v1.GetCo(0), edge_v1.GetCo(1), edge_v1.GetCo(2));
		glm::vec3 e_v2(edge_v2.GetCo(0), edge_v2.GetCo(1), edge_v2.GetCo(2));

		ve = (v_f1 + e_v1 + e_v2) / (float)4;
		return ve;
	}

	int edge_v1_index = current_edge.GetVertex(0);
	int edge_v2_index = current_edge.GetVertex(1);
	int edge_f1_index = current_edge.GetIncFacet(0);
	int edge_f2_index = current_edge.GetIncFacet(1);
	GeomVert edge_v1 = mesh->GetGeomVertex(edge_v1_index);
	GeomVert edge_v2 = mesh->GetGeomVertex(edge_v2_index);
	glm::vec3 v_f1 = cc_vf(edge_f1_index, mesh);
	glm::vec3 v_f2 = cc_vf(edge_f2_index, mesh);
	glm::vec3 e_v1(edge_v1.GetCo(0), edge_v1.GetCo(1), edge_v1.GetCo(2));
	glm::vec3 e_v2(edge_v2.GetCo(0), edge_v2.GetCo(1), edge_v2.GetCo(2));

	ve = (v_f1 + v_f2 + e_v1 + e_v2) / (float)4;
	return ve;
}
glm::vec3 cc_midpoint(int edge_id, Mesh *mesh)
{
	glm::vec3 mid(0.0, 0.0, 0.0);
	TopoEdge current_edge = mesh->GetEdge(edge_id);
	int edge_v1_index = current_edge.GetVertex(0);
	int edge_v2_index = current_edge.GetVertex(1);
	GeomVert edge_v1 = mesh->GetGeomVertex(edge_v1_index);
	GeomVert edge_v2 = mesh->GetGeomVertex(edge_v2_index);
	glm::vec3 v1(edge_v1.GetCo(0), edge_v1.GetCo(1), edge_v1.GetCo(2));
	glm::vec3 v2(edge_v2.GetCo(0), edge_v2.GetCo(1), edge_v2.GetCo(2));
	mid = (v1 + v2) / (float)2;
	return mid;
}
glm::vec3 cc_vv(int vertex_id, Mesh *mesh)
{
	glm::vec3 new_v(0.0, 0.0, 0.0);
	TopoVert current_vertex = mesh->GetVertex(vertex_id);
	glm::vec3 Q(0.0, 0.0, 0.0);
	int number_face = current_vertex.GetNumberIncFacets();
	for (int i = 0; i < number_face; i++) {
		int facet_index = current_vertex.GetIncFacet(i);
		glm::vec3 vf = cc_vf(facet_index, mesh);
		Q = Q + vf;
	}
	Q = Q / (float)number_face;

	glm::vec3 R(0.0, 0.0, 0.0);
	int number_edge = current_vertex.GetNumberIncEdges();
	for (int i = 0; i < number_edge; i++) {
		int edge_index = current_vertex.GetIncEdge(i);
		glm::vec3 ve = cc_midpoint(edge_index, mesh);
		R = R + ve;
	}
	R = R / (float)number_edge;

	GeomVert old_v = mesh->GetGeomVertex(vertex_id);
	glm::vec3 v(old_v.GetCo(0), old_v.GetCo(1), old_v.GetCo(2));

	new_v = ((float)1 * Q + (float)2 * R + (float)(number_edge - 3)*v) / (float)number_edge;
	return new_v;
}
void Catmull_Clark(Mesh* input, Mesh **output)
{

	int number_vertex = input->GetNumberVertices();
	for (int i = 0; i < number_vertex; i++)
	{
		TopoVert current_vertex = input->GetVertex(i);
		glm::vec3 new_vv = cc_vv(i, input);
		int number_v_face = current_vertex.GetNumberIncFacets();
		int number_v_edge = current_vertex.GetNumberIncEdges();
		for (int j = 0; j < number_v_face; j++)
		{
			int face_index = current_vertex.GetIncFacet(j);
			glm::vec3 new_vf = cc_vf(face_index, input);
			TopoFacet current_face = input->GetFacet(face_index);
			int number_facet_edges = current_face.GetNumberEdges();
			vector<int> edge_index;
			for (int k = 0; k < number_facet_edges; k++)
			{
				int current_edge_index = current_face.GetIncEdge(k);
				TopoEdge current_edge = input->GetEdge(current_edge_index);
				int edge_vertex_1 = current_edge.GetVertex(0);
				int edge_vertex_2 = current_edge.GetVertex(1);
				if (edge_vertex_1 == i || edge_vertex_2 == i)
				{
					edge_index.push_back(current_edge_index);
				}
			}
			int edge1_index;
			int edge2_index;
			edge1_index = edge_index[0];
			edge2_index = edge_index[1];
			glm::vec3 new_ve1 = cc_ve(edge1_index, input);
			glm::vec3 new_ve2 = cc_ve(edge2_index, input);
			vector<GeomVert> new_face;
			new_face.push_back(GeomVert(new_vf.x, new_vf.y, new_vf.z));
			new_face.push_back(GeomVert(new_ve1.x, new_ve1.y, new_ve1.z));
			new_face.push_back(GeomVert(new_vv.x, new_vv.y, new_vv.z));
			new_face.push_back(GeomVert(new_ve2.x, new_ve2.y, new_ve2.z));
			(*output)->AddFacet(new_face);
		}
	}
}


int third_vertex_index(int facet_id, int v1, int v2, Mesh *mesh)
{
	TopoFacet current_face = mesh->GetFacet(facet_id);
	int f_vertex1 = current_face.GetVertexInd(0);
	int f_vertex2 = current_face.GetVertexInd(1);
	int f_vertex3 = current_face.GetVertexInd(2);
	if (f_vertex1 == v1 && f_vertex2 == v2)
	{
		return f_vertex3;
	}
	if (f_vertex1 == v2 && f_vertex2 == v1)
	{
		return f_vertex3;
	}
	if (f_vertex1 == v1 && f_vertex3 == v2)
	{
		return f_vertex2;
	}
	if (f_vertex1 == v2 && f_vertex3 == v1)
	{
		return f_vertex2;
	}
	if (f_vertex2 == v1 && f_vertex3 == v2)
	{
		return f_vertex1;
	}
	if (f_vertex2 == v2 && f_vertex3 == v1)
	{
		return f_vertex1;
	}

}
glm::vec3 loop_ve(int edge_id, Mesh *mesh)
{
	glm::vec3 ve(0.0, 0.0, 0.0);
	TopoEdge current_edge = mesh->GetEdge(edge_id);
	int r_index = current_edge.GetVertex(0);
	int s_index = current_edge.GetVertex(1);
	int face_prs_index = current_edge.GetIncFacet(0);
	int face_qrs_index = current_edge.GetIncFacet(1);
	int p_index = third_vertex_index(face_prs_index, r_index, s_index, mesh);
	int q_index = third_vertex_index(face_qrs_index, r_index, s_index, mesh);

	GeomVert vertex_r = mesh->GetGeomVertex(r_index);
	GeomVert vertex_s = mesh->GetGeomVertex(s_index);
	GeomVert vertex_p = mesh->GetGeomVertex(p_index);
	GeomVert vertex_q = mesh->GetGeomVertex(q_index);
	glm::vec3 r(vertex_r.GetCo(0), vertex_r.GetCo(1), vertex_r.GetCo(2));
	glm::vec3 s(vertex_s.GetCo(0), vertex_s.GetCo(1), vertex_s.GetCo(2));
	glm::vec3 p(vertex_p.GetCo(0), vertex_p.GetCo(1), vertex_p.GetCo(2));
	glm::vec3 q(vertex_q.GetCo(0), vertex_q.GetCo(1), vertex_q.GetCo(2));

	ve = (p + (float)3 * r + (float)3 * s + q) / (float)8;
	return ve;
}
float ALPHA = 5.0 / 8.0;
glm::vec3 loop_vv(int vertex_id, Mesh *mesh)
{
	glm::vec3 new_v(0.0, 0.0, 0.0);
	TopoVert current_vertex = mesh->GetVertex(vertex_id);
	glm::vec3 p(0.0, 0.0, 0.0);
	int number_v_edge = current_vertex.GetNumberIncEdges();
	for (int i = 0; i < number_v_edge; i++)
	{
		int edge_index = current_vertex.GetIncEdge(i);
		TopoEdge current_edge = mesh->GetEdge(edge_index);
		int edge_v1_index = current_edge.GetVertex(0);
		int edge_v2_index = current_edge.GetVertex(1);
		if (edge_v1_index == vertex_id)
		{
			GeomVert endpoint = mesh->GetGeomVertex(edge_v2_index);
			glm::vec3 ptn(endpoint.GetCo(0), endpoint.GetCo(1), endpoint.GetCo(2));
			p = p + ptn;
		}
		if (edge_v2_index == vertex_id)
		{
			GeomVert endpoint = mesh->GetGeomVertex(edge_v1_index);
			glm::vec3 ptn(endpoint.GetCo(0), endpoint.GetCo(1), endpoint.GetCo(2));
			p = p + ptn;
		}
	}
	p = p / (float)number_v_edge;
	GeomVert old_v = mesh->GetGeomVertex(vertex_id);
	glm::vec3 v(old_v.GetCo(0), old_v.GetCo(1), old_v.GetCo(2));
	new_v = (1 - ALPHA)*p + ALPHA * v;
	return new_v;
}
void Loop(Mesh *input, Mesh **output)
{

	int number_face = input->GetNumberFacets();
	for (int i = 0; i < number_face; i++)
	{
		TopoFacet current_face = input->GetFacet(i);
		// let the vertices of the triangle be p, q, r;
		int p_index = current_face.GetVertexInd(0);
		int q_index = current_face.GetVertexInd(1);
		int r_index = current_face.GetVertexInd(2);
		// allocate edges pq, qr, pr
		int edge_pq_index = -1;
		int edge_qr_index = -1;
		int edge_pr_index = -1;
		int edge1_index = current_face.GetIncEdge(0);
		int edge2_index = current_face.GetIncEdge(1);
		int edge3_index = current_face.GetIncEdge(2);
		TopoEdge edge1 = input->GetEdge(edge1_index);
		TopoEdge edge2 = input->GetEdge(edge2_index);
		TopoEdge edge3 = input->GetEdge(edge3_index);


		// now calculate the edge_pq_index, edge_qr_index, edge_pr_index
		// By the method 
		// if the point r is not on edge1, edge1 = pq
		// if the point p is not on edge1, edge1 = qr
		// if the point q is not on edge1, edge1 = pr
		int res1 = third_vertex_index(i, edge1.GetVertex(0), edge1.GetVertex(1), input);
		if (res1 == p_index)
		{
			edge_qr_index = edge1_index;
		}
		if (res1 == q_index)
		{
			edge_pr_index = edge1_index;
		}
		if (res1 == r_index) {
			edge_pq_index = edge1_index;
		}
		int res2 = third_vertex_index(i, edge2.GetVertex(0), edge2.GetVertex(1), input);
		if (res2 == p_index)
		{
			edge_qr_index = edge2_index;
		}
		if (res2 == q_index)
		{
			edge_pr_index = edge2_index;
		}
		if (res2 == r_index) {
			edge_pq_index = edge2_index;
		}
		int res3 = third_vertex_index(i, edge3.GetVertex(0), edge3.GetVertex(1), input);
		if (res3 == p_index)
		{
			edge_qr_index = edge3_index;
		}
		if (res3 == q_index)
		{
			edge_pr_index = edge3_index;
		}
		if (res3 == r_index) {
			edge_pq_index = edge3_index;
		}
		if (edge_pq_index == -1 || edge_pr_index == -1 || edge_qr_index == -1)
		{
			cout << "Loop Subdivision algorithm is wrong now." << endl;
		}
		glm::vec3 v_p = loop_vv(p_index, input);
		glm::vec3 v_q = loop_vv(q_index, input);
		glm::vec3 v_r = loop_vv(r_index, input);
		glm::vec3 v_pq = loop_ve(edge_pq_index, input);
		glm::vec3 v_pr = loop_ve(edge_pr_index, input);
		glm::vec3 v_qr = loop_ve(edge_qr_index, input);

		(*output)->AddFacet(v_p.x, v_p.y, v_p.z, v_pq.x, v_pq.y, v_pq.z, v_pr.x, v_pr.y, v_pr.z);
		(*output)->AddFacet(v_q.x, v_q.y, v_q.z, v_pq.x, v_pq.y, v_pq.z, v_qr.x, v_qr.y, v_qr.z);
		(*output)->AddFacet(v_r.x, v_r.y, v_r.z, v_pr.x, v_pr.y, v_pr.z, v_qr.x, v_qr.y, v_qr.z);
		(*output)->AddFacet(v_pr.x, v_pr.y, v_pr.z, v_pq.x, v_pq.y, v_pq.z, v_qr.x, v_qr.y, v_qr.z);
	}
}

// 
void mouseEvent(int button, int state, int ix, int iy)
{
	if (state == GLUT_UP)
	{
		currentmoving = false;
		return;
	}
	double x = 2.0f* ix / (float)(width - 1) - 1.0f;
	double y = 2.0f* (height - 1 - iy) / (float)(height - 1) - 1.0f;
	// Could require left mouse down only, but ...
	if (state == GLUT_DOWN)
	{
		if (vertices.size() > Max_Points)
			vertices.clear();
		// 02/03 check if click on a vertex
		double tol = 0.1;
		int i;
		for (i = 0; i < vertices.size() / 3; i++)
		{
			if (x >= vertices[3 * i] - tol &&
				x <= vertices[3 * i] + tol &&
				y >= vertices[3 * i + 1] - tol &&
				y <= vertices[3 * i + 1] + tol) break;
		}


		if (i == vertices.size() / 3)
		{
			vertices.push_back(x);
			vertices.push_back(y);
			vertices.push_back(0.0);
		}
		if (i != vertices.size() / 3) {
			select_vert = i;
		}
		currentmoving = true;
		glutPostRedisplay();
	}
}
void mouseMove(int ix, int iy)
{
	if (!currentmoving) {
		return;
	}
	double x = 2.0f* ix / (float)(width - 1) - 1.0f;
	double y = 2.0f* (height - 1 - iy) / (float)(height - 1) - 1.0f;
	vertices[3 * select_vert] = x;
	vertices[3 * select_vert + 1] = y;
	glutPostRedisplay();
}


// 04/14
// let the screen become 800x800 again first

/*
void mouseEvent(int button, int state, int ix, int iy)
{
if (state == GLUT_UP)
{
currentmoving = false;
return;
}
//cout << "ix:" << ix << " iy: " << iy << endl;

if (ix <= 400 && iy <= 400) {
double x = ix / 200.0 - 1.0f;
double y = -iy / 200.0 + 1.0f;
// Could require left mouse down only, but ...
if (state == GLUT_DOWN)
{
if (vertices.size() > Max_Points)
vertices.clear();
// 02/03 check if click on a vertex
double tol = 0.1;
int i;
for (i = 0; i < vertices.size() / 3; i++)
{
if (x >= vertices[3 * i] - tol &&
x <= vertices[3 * i] + tol &&
y >= vertices[3 * i + 1] - tol &&
y <= vertices[3 * i + 1] + tol) break;
}


if (i == vertices.size() / 3)
{
vertices.push_back(x);
vertices.push_back(y);
vertices.push_back(0.0);
}
if (i != vertices.size() / 3) {
select_vert = i;
}
currentmoving = true;
glutPostRedisplay();
}
}
// 03/05
if (ix>=400 && ix <= 800 && iy <= 400) {
double x = ix / 200.0 - 3.0f;
double y = -iy / 200.0 + 1.0f;
//cout << "x: " << x << " y: " << y << endl;
// Could require left mouse down only, but ...
if (state == GLUT_DOWN)
{
if (vertices2.size() > Max_Points) {
vertices2.clear();
}
vertices2.push_back(x);
vertices2.push_back(y);
vertices2.push_back(0.0);
glutPostRedisplay();
}
}
if (ix >= 400 && ix <= 800 && iy <= 800 && iy >= 400)
{
cout << "Please input the .off file" << endl;
string filename;
cin >> filename;
//filename = "cone.off";
int choice;
cout << "Please select the subdivision method"
<< "input 1: doo_sabine 2: Catmull-Clark 3: Loop" << endl;
cin >> choice;
//choice = 3;
if (choice == 1) {
Mesh cone = readOFF(filename.c_str());
Mesh *cone_ptr = &cone;
Mesh *new_cone;
new_cone = new Mesh();
doo_sabine(cone_ptr, &new_cone);
outputOFF(new_cone, "new_object_ds.off");
cout << "finish the Doo-Sabine subdivision.\n"
<< "The new filename is new_object.off" << endl;
}
if (choice == 2) {
Mesh cone = readOFF(filename.c_str());
Mesh *cone_ptr = &cone;
Mesh *new_cone;
new_cone = new Mesh();
Catmull_Clark(cone_ptr, &new_cone);
outputOFF(new_cone, "new_object_CC.off");
cout << "finish the Catmull-Clark subdivision.\n"
<< "The new filename is new_object_CC.off" << endl;
}
if (choice == 3) {
Mesh cone = readOFF(filename.c_str());
Mesh *cone_ptr = &cone;
Mesh *new_cone;
new_cone = new Mesh();
Loop(cone_ptr, &new_cone);
outputOFF(new_cone, "new_object_loop.off");
cout << "finish the loop subdivision.\n"
<< "The new filename is new_object_loop.off" << endl;
}
}

if (ix >= 400 && ix <= 800 && iy <= 800 && iy >= 400 && vertices.size() >= 9) {
double x;
cout << "Please input the z-depth for Extrusion." << endl;
cin >> x;
Extru_height = x;
}
if (ix > 800 && ix < 1200 && iy < 400&& iy>0) {

}

}
void mouseMove(int ix, int iy)
{
if (!currentmoving) {
return;
}
double x = ix / 200.0 - 1.0f;
double y = -iy / 200.0 + 1.0f;
vertices[3 * select_vert] = x;
vertices[3 * select_vert + 1] = y;
glutPostRedisplay();
}
*/
void Suddivision()
{
	cout << "Please input the .off file" << endl;
	string filename;
	cin >> filename;
	//filename = "cone.off";
	int choice;
	cout << "Please select the subdivision method"
		<< "input 1: doo_sabine 2: Catmull-Clark 3: Loop" << endl;
	cin >> choice;
	//choice = 3;
	if (choice == 1) {
		Mesh cone = readOFF(filename.c_str());
		Mesh *cone_ptr = &cone;
		Mesh *new_cone;
		new_cone = new Mesh();
		doo_sabine(cone_ptr, &new_cone);
		outputOFF(new_cone, "new_object_ds.off");
		cout << "finish the Doo-Sabine subdivision.\n"
			<< "The new filename is new_object.off" << endl;
	}
	if (choice == 2) {
		Mesh cone = readOFF(filename.c_str());
		Mesh *cone_ptr = &cone;
		Mesh *new_cone;
		new_cone = new Mesh();
		Catmull_Clark(cone_ptr, &new_cone);
		outputOFF(new_cone, "new_object_CC.off");
		cout << "finish the Catmull-Clark subdivision.\n"
			<< "The new filename is new_object_CC.off" << endl;
	}
	if (choice == 3) {
		Mesh cone = readOFF(filename.c_str());
		Mesh *cone_ptr = &cone;
		Mesh *new_cone;
		new_cone = new Mesh();
		Loop(cone_ptr, &new_cone);
		outputOFF(new_cone, "new_object_loop.off");
		cout << "finish the loop subdivision.\n"
			<< "The new filename is new_object_loop.off" << endl;
	}
}
void keyboard(unsigned char key, int x, int y)
{
	switch (key) {
	case ' ':
		if (vertices.size() > 0) {
			vertices.pop_back();
			vertices.pop_back();
			vertices.pop_back();
			glutPostRedisplay();
		}
		break;
	case 'b':
	case 'B':
		draw_bc = !draw_bc;
		glutPostRedisplay();
		break;
	case 's':
	case 'S':
		draw_sp = !draw_sp;
		glutPostRedisplay();
		break;
	case 'd':
	case 'D':
		sub_dC_function();
		glutPostRedisplay();
		break;
	case 'q':
	case 'Q':
		sub_Bs(vertices);
		glutPostRedisplay();
		break;
	case '1':
		SOR_number = SOR_number + 1;
		glutPostRedisplay();
		break;
	case '2':
		SOR_number--;
		glutPostRedisplay();
		break;
	case '3':
		Extru_height = Extru_height + 0.1;
		glutPostRedisplay();
		break;
	case '4':
		Extru_height = Extru_height - 0.1;
		glutPostRedisplay();
		break;
	case 'n':
	case 'N':
		reconstruction = !reconstruction;
		glutPostRedisplay();
		break;
	case 'P':
	case 'p':
		Suddivision();
		glutPostRedisplay();
	}
}


////////////////////////////////////////////////


int main(int argc, char* argv[])
{

	initOpenGlWindow(argc, argv);
	// 03/01
	initScene();
	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutReshapeFunc(reshape);
	glutMouseFunc(mouseEvent);
	glutMotionFunc(mouseMove);
	// 03/01
	//initOpenGlWindow2();
	//glutDisplayFunc(display_2);
	glutMainLoop();

	return 0;
}




