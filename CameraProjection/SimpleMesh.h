/*
Copyright (c) 2018, Michael Kazhdan and Fabian Prada
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution.

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/


#ifndef SIMPLE_MESH_INCLUDED
#define SIMPLE_MESH_INCLUDED

#include <Misha/Ply.h>
#include <Misha/Image.h>

class SimpleMesh{
public:
	std::vector<Point3D<double>> vertices;
	std::vector<Point3D<double>> normals;
	std::vector<TriangleIndex> triangles;
};

class ColoredMesh : public SimpleMesh{
public:
	std::vector<Point3D<double>> colors;
};

void UpdateNormals(SimpleMesh & mesh){
	mesh.normals.clear();
	mesh.normals.resize(mesh.vertices.size(), Point3D<double>(0.0, 0.0, 0.0));
	for (int t = 0; t < mesh.triangles.size(); t++){
		Point3D<double> d01 = mesh.vertices[mesh.triangles[t][1]] - mesh.vertices[mesh.triangles[t][0]];
		Point3D<double> d02 = mesh.vertices[mesh.triangles[t][2]] - mesh.vertices[mesh.triangles[t][0]];
		Point3D<double> n = Point3D<double>::CrossProduct(d01,d02);
		for (int v = 0; v < 3; v++) mesh.normals[mesh.triangles[t][v]] += n;
	}

	for (int i = 0; i < mesh.normals.size(); i++){
		if (Point3D<double>::Length(mesh.normals[i])> 0) mesh.normals[i] /= Point3D<double>::Length(mesh.normals[i]);
	}
}

int ReadSimpleMesh(SimpleMesh & mesh, const char * fileName){
	mesh.vertices.clear();
	mesh.triangles.clear();
	int file_type;
	std::vector< PlyVertex< double > > ply_vertices;
	bool readFlags[PlyVertex< double >::ReadComponents];
	if (!PlyReadTriangles(fileName, ply_vertices, mesh.triangles, PlyVertex< double >::ReadProperties, readFlags, PlyVertex< double >::ReadComponents, file_type)) return 0;
	mesh.vertices.resize(ply_vertices.size());
	for (int i = 0; i < ply_vertices.size(); i++) mesh.vertices[i] = Point3D<double>(ply_vertices[i].point[0], ply_vertices[i].point[1], ply_vertices[i].point[2]);
	UpdateNormals(mesh);
	return 1;
}

void WriteSimpleMesh(SimpleMesh & mesh, const char * fileName){
	std::vector< PlyVertex< float > > ply_vertices(mesh.vertices.size());
	for (int i = 0; i<mesh.vertices.size(); i++) ply_vertices[i].point = Point3D<float>(mesh.vertices[i][0], mesh.vertices[i][1], mesh.vertices[i][2]);
	PlyWriteTriangles(fileName, ply_vertices, mesh.triangles, PlyVertex< float >::WriteProperties, PlyVertex< float >::WriteComponents, PLY_BINARY_NATIVE);
}

void WriteColoredMesh(ColoredMesh & mesh, const char * fileName){
	std::vector< PlyColorVertex< float > > ply_vertices(mesh.vertices.size());
	for (int i = 0; i<mesh.vertices.size(); i++) ply_vertices[i].point = Point3D<float>(mesh.vertices[i][0], mesh.vertices[i][1], mesh.vertices[i][2]), ply_vertices[i].color = Point3D<float>(mesh.colors[i][0], mesh.colors[i][1], mesh.colors[i][2]);
	PlyWriteTriangles(fileName, ply_vertices, mesh.triangles, PlyColorVertex< float >::WriteProperties, PlyColorVertex< float >::WriteComponents, PLY_BINARY_NATIVE);
}

#endif//SIMPLE_MESH_INCLUDED
