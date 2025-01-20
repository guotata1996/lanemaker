/************************************************************************************

OpenGL with Qt - Tutorial
-------------------------
Autor      : Andreas Nicolai <andreas.nicolai@gmx.net>
Repository : https://github.com/ghorwin/OpenGLWithQt-Tutorial
License    : BSD License,
			 see https://github.com/ghorwin/OpenGLWithQt-Tutorial/blob/master/LICENSE

************************************************************************************/

#ifndef VERTEX_H
#define VERTEX_H

#include <QVector2D>
#include <QVector3D>
#include <QColor>

/*! A container class to store data (coordinates, normals, textures, colors) of a vertex, used for interleaved
	storage. Expand this class as needed.

	Memory layout (each char is a byte): xxxxyyyyzzzzrrrrggggbbbb = 6*4 = 24 Bytes

	You can define a vector<Vertex> and use this directly as input to the vertex buffer.

	Mind implicit padding by compiler! Hence, for allocation use:
	- sizeof(Vertex) as stride
	- offsetof(Vertex, r) as start offset for the color

	This will only become important, if mixed data types are used in the struct.
	Read http://www.catb.org/esr/structure-packing/ for an in-depth explanation.
*/
struct Vertex {
	Vertex() {}
	Vertex(const QVector3D& coords, const QColor& col, const unsigned int objID) :
		x(float(coords.x())),
		y(float(coords.y())),
		z(float(coords.z())),
		r(float(col.redF())),
		g(float(col.greenF())),
		b(float(col.blueF())),
		objectID(objID)
	{
	}

	float x, y, z;
	float r, g, b;
	unsigned int objectID;
};

struct VertexInstanced {
	VertexInstanced() = default;

	VertexInstanced(const QVector3D& coords, const QVector2D& uv, const unsigned int objID):
		x(float(coords.x())),
		y(float(coords.y())),
		z(float(coords.z())),
		u(uv.x()),
		v(uv.y()),
		objectID(objectID)
	{}

	float x, y, z;
	float u, v;
	unsigned int objectID;
};

struct Pose
{
	Pose() = default;

	float m00, m01, m02, m03;
	float m10, m11, m12, m13;
	float m20, m21, m22, m23;
	float m30, m31, m32, m33;

	float r, g, b;
	unsigned int objectID;
};

#endif // VERTEX_H
