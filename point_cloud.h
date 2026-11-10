#ifndef _POINT_CLOUD_PARSER_H_
#define _POINT_CLOUD_PARSER_H_

#include <vector>
#include <iostream>

#include <cyPointCloud.h>
#include <Eigen/SVD>

#include "helper.h"

extern "C" {
	#define REAL float
	#define TRILIBRARY
	#define ANSI_DECLARATORS
	#include "triangle.h"
}

//extern "C" void triangulate(char*, struct triangulateio*, struct triangulateio*, 
//	struct triangulateio*);
//extern "C" void trifree(void* ptr);

void Init(triangulateio& io) {
	io.pointlist = nullptr;
	io.pointattributelist = nullptr;
	io.pointmarkerlist = nullptr;
	io.numberofpoints = 0;
	io.numberofpointattributes = 0;

	io.trianglelist = nullptr;
	io.triangleattributelist = nullptr;
	io.trianglearealist = nullptr;
	io.neighborlist = nullptr;
	io.numberoftriangles = 0;
	io.numberofcorners = 0;
	io.numberoftriangleattributes = 0;

	io.segmentlist = nullptr;
	io.segmentmarkerlist = nullptr;
	io.numberofsegments = 0;

	io.holelist = nullptr;
	io.numberofholes = 0;

	io.regionlist = nullptr;
	io.numberofregions = 0;

	io.edgelist = nullptr;
	io.edgemarkerlist = nullptr;
	io.normlist = nullptr;
	io.numberofedges = 0;
}

void Free(triangulateio& io) {
	if (io.pointlist)				trifree(io.pointlist);
	if (io.pointattributelist)		trifree(io.pointattributelist);
	if (io.pointmarkerlist)			trifree(io.pointmarkerlist);

	if (io.trianglelist)			trifree(io.trianglelist);
	if (io.triangleattributelist)	trifree(io.triangleattributelist);
	if (io.trianglearealist)		trifree(io.trianglearealist);
	if (io.neighborlist)			trifree(io.neighborlist);

	if (io.segmentlist)				trifree(io.segmentlist);
	if (io.segmentmarkerlist)		trifree(io.segmentmarkerlist);

	if (io.holelist)				trifree(io.holelist);

	if (io.regionlist)				trifree(io.regionlist);

	if (io.edgelist)				trifree(io.edgelist);
	if (io.edgemarkerlist)			trifree(io.edgemarkerlist);
	if (io.normlist)				trifree(io.normlist);
}

inline Vec3f toCyVec(const Eigen::Ref<Eigen::Vector3f>& in) { return Vec3f(in.data()); }

typedef cy::PointCloud<Vec3f, float, 3u> PtCloud3f;
class CustomPtCloud : protected PtCloud3f {
	std::vector<Vec3f> pos;
	std::vector<Vec3f> norm;
	std::vector<float> area;

	// Incomplete
	void ApproxAreas() {
		const uint k = 20;

		for (uint i = 0; i < uint(pos.size()); ++i) {
			PointInfo pts[k];
			uint count = GetPoints(i, k, pts);

			Vec3f avg(0.0f);
			for (uint j = 0; j < count; ++j) { avg += pts[j].pos; }
			avg /= count;

			Eigen::Matrix<float, 3, k> mat;
			for (uint j = 0; j < count; ++j) {
				uint colStart = j * 3;
				for (uint k = 0; k < 3; ++k) {
					mat[colStart + k] = pts[j].pos[k] - avg[k];
				}
			}

			Eigen::JacobiSVD<Eigen::Matrix<float, 3, k>> svd(mat.leftCols(count), Eigen::ComputeFullV);
			Vec3f fit = toCyVec(svd.matrixV().col(count - 1));

			Vec2f pts2D[k];
			for (uint j = 0; j < count; ++j) {
				pts2D[j] = projectToLocalPlane(pts[j].pos, fit);
			}

			triangulateio in, out, vorout;
			Init(in); Init(out); Init(vorout);

			in.numberofpoints = count;
			in.pointlist = &pts2D->x;

			char cmd[] = "zevQ";

			triangulate(cmd, &in, &out, &vorout);

			// Now just need to compute voronoi cell area

			Free(out); Free(vorout);
		}
	}

public:

	// Modified version of implementation from cyCodeBase's cy::PointCloud
	int GetPoints(uint pt, float radius, uint maxCount, PointInfo* closestPoints) const
	{
		Vec3f position = pos[pt];
		Vec3f normal = norm[pt];
		int pointsFound = 0;
		PtCloud3f::GetPoints(position, radius, [&](uint i, Vec3f const& p, float d2, float& r2) {
			if (norm[i].Dot(normal) <= 0.0f) { return; }
			if (pointsFound == maxCount) {
				std::pop_heap(closestPoints, closestPoints + maxCount);
				closestPoints[maxCount - 1].index = i;
				closestPoints[maxCount - 1].pos = p;
				closestPoints[maxCount - 1].distanceSquared = d2;
				std::push_heap(closestPoints, closestPoints + maxCount);
				r2 = closestPoints[0].distanceSquared;
			}
			else {
				closestPoints[pointsFound].index = i;
				closestPoints[pointsFound].pos = p;
				closestPoints[pointsFound].distanceSquared = d2;
				pointsFound++;
				if (pointsFound == maxCount) {
					std::make_heap(closestPoints, closestPoints + maxCount);
					r2 = closestPoints[0].distanceSquared;
				}
			}
			});
		return pointsFound;
	}

	int GetPoints(uint pt, uint maxCount, PointInfo* closestPoints) const
	{
		return GetPoints(pt, std::numeric_limits<float>::max(), maxCount, closestPoints);
	}

	// Modified version of cyCodeBase's cy::TriMesh::LoadFromFileObj
	inline bool LoadFromFileObj(char const* filename, std::ostream* outStream = &std::cout)
	{
		FILE* fp = fopen(filename, "r");
		if (!fp) {
			if (outStream) *outStream << "ERROR: Cannot open file " << filename << std::endl;
			return false;
		}

		pos.clear();
		norm.clear();
		area.clear();

		//! Simple character string
		struct Str
		{
			char* data;	//!< String data
			Str() : data(nullptr) {}							//!< Constructor
			Str(Str const& s) : data(nullptr) { *this = s; }	//!< Copy constructor
			~Str() { if (data) delete[] data; }				//!< Destructor
			operator char const* () { return data; }			//!< Implicit conversion to const char
			void operator = (Str  const& s) { *this = s.data; }	//!< Assignment operator
			void operator = (char const* s) { if (s) { size_t n = strlen(s); if (data) delete[] data; data = new char[n + 1]; strncpy(data, s, n); data[n] = '\0'; } else if (data) { delete[] data; data = nullptr; } }	//!< Assignment operator
		};

		class Buffer
		{
			char data[1024];
			int readLine;
		public:
			int ReadLine(FILE* fp)
			{
				char c = fgetc(fp);
				while (!feof(fp)) {
					while (isspace(c) && (!feof(fp) || c != '\0')) c = fgetc(fp);	// skip empty space
					if (c == '#') while (!feof(fp) && c != '\n' && c != '\r' && c != '\0') c = fgetc(fp);	// skip comment line
					else break;
				}
				int i = 0;
				bool inspace = false;
				while (i < 1024 - 1) {
					if (feof(fp) || c == '\n' || c == '\r' || c == '\0') break;
					if (isspace(c)) {	// only use a single space as the space character
						inspace = true;
					}
					else {
						if (inspace) data[i++] = ' ';
						inspace = false;
						data[i++] = c;
					}
					c = fgetc(fp);
				}
				data[i] = '\0';
				readLine = i;
				return i;
			}
			char& operator[](int i) { return data[i]; }
			void ReadVertex(Vec3f& v) const { v.Zero(); sscanf(data + 2, "%f %f %f", &v.x, &v.y, &v.z); }
			void ReadFloat3(float f[3]) const { f[2] = f[1] = f[0] = 0; int n = sscanf(data + 2, "%f %f %f", &f[0], &f[1], &f[2]); if (n == 1) f[2] = f[1] = f[0]; }
			void ReadFloat(float* f) const { sscanf(data + 2, "%f", f); }
			void ReadInt(int* i, int start) const { sscanf(data + start, "%d", i); }
			bool IsCommand(char const* cmd) const {
				int i = 0;
				while (cmd[i] != '\0') {
					if (cmd[i] != data[i]) return false;
					i++;
				}
				return (data[i] == '\0' || data[i] == ' ');
			}
			char const* Data(int start = 0) { return data + start; }
			void Copy(Str& str, int start = 0)
			{
				while (data[start] != '\0' && data[start] <= ' ') start++;
				str = Data(start);
			}
		};
		Buffer buffer;

		while (int rb = buffer.ReadLine(fp)) {
			if (buffer.IsCommand("v")) {
				Vec3f vertex;
				buffer.ReadVertex(vertex);
				pos.push_back(vertex);
			}
			/*else if (buffer.IsCommand("vt")) {
				Vec3f texVert;
				buffer.ReadVertex(texVert);
				tex.push_back(texVert);
			}*/
			else if (buffer.IsCommand("vn")) {
				Vec3f normal;
				buffer.ReadVertex(normal);
				norm.push_back(normal);
			}
			if (feof(fp)) break;
		}

		fclose(fp);

		PtCloud3f::Build(uint32_t(pos.size()), pos.data());

		// ApproxAreas();

		return true;
	}

	const std::vector<Vec3f>& Positions() { return pos; }
	const std::vector<Vec3f>& Normals() { return norm; }
	const std::vector<float>& Areas() { return area; }
};

#endif