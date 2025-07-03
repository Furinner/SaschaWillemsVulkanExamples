/*
* Vulkan Example - Deferred shading with multiple render targets (aka G-Buffer) example
*
* This samples shows how to do deferred rendering. Unlike forward rendering, different components like
* albedo, normals, world positions are rendered to offscreen images which are then put together and lit
* in a composition pass
* Use the dropdown in the ui to switch between the final composition pass or the separate components
*
*
* Copyright (C) 2016-2023 by Sascha Willems - www.saschawillems.de
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

#include "vulkanexamplebase.h"
#include "VulkanglTFModel.h"
#include <algorithm>
#include <vector>
#include <memory>
#include <limits>
#include <unordered_set>
#include <optional>
#include <STEPControl_Reader.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopLoc_Location.hxx>
#include <Geom_Surface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Line.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom_Curve.hxx>                 // 曲线基类
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2dAPI_InterCurveCurve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomAPI.hxx>  
#include <GeomLProp_SLProps.hxx>
#include <Geom2dLProp_CLProps2d.hxx>
#include <gp_Pnt.hxx>                     // 点类
#include <Standard_Real.hxx>             // 实数类型
#include <TColgp_Array1OfPnt.hxx>        // 点数组（可选）
#include <BRepLib.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <ShapeFix_Edge.hxx>
#include <Poly_Triangulation.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <HLRBRep_Algo.hxx>
#include <HLRAlgo_Projector.hxx>
#include <HLRBRep_HLRToShape.hxx>
#include <iostream>


#define DLF 1

#define MAX_NEIGHBOR_FACE_COUNT 50

#define uPtr std::unique_ptr
#define mkU std::make_unique

class VulkanExample : public VulkanExampleBase
{
	//template<typename T>
	//static inline bool fequal(T a, T b, T epsilon = std::numeric_limits<float>::min()) {
	//	if (a == b) {
	//		// Shortcut
	//		return true;
	//	}

	//	const T diff = std::abs(a - b);
	//	if (a * b == 0) {
	//		// a or b or both are zero; relative error is not meaningful here
	//		return diff < (epsilon* epsilon);
	//	}

	//	return diff / (std::abs(a) + std::abs(b)) < epsilon;
	//}
	template<typename T>
	static inline bool fequal(T a, T b, T epsilon = 1e-3) {
		if (a == b) {
			// Shortcut
			return true;
		}

		//self-added
		const T diff = std::abs(a - b);
		if (diff < epsilon) {
			return true;
		}

		if (a * b == 0) {
			// a or b or both are zero; relative error is not meaningful here
			return diff < (epsilon * epsilon);
		}

		return diff / (std::abs(a) + std::abs(b)) < epsilon;
	};

public:
	vkglTF::Model model;

	//self-added
	struct Vertex;
	struct Mesh;
	struct HalfEdge;
	struct Face;
	class VertexHash {
	public:
		size_t operator()(const Vertex* v) const {
			size_t seed = 5381;
			const auto hasher = std::hash<float>{};
			seed ^= hasher(v->position.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
			seed ^= hasher(v->position.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
			seed ^= hasher(v->position.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
			return seed;
		}

	};

	struct VertexEqual {
		bool operator()(Vertex* const& v1, Vertex* const& v2) const {
			return fequal(v1->position.x, v2->position.x) && (fequal(v1->position.y, v2->position.y) && (fequal(v1->position.z, v2->position.z)));
		};
	};

	struct Vertex {
		//alignas means some memory before the variable behind
		//for example, glm::vec3 pos should align to 16 bytes (meaning the start point of pos in memory should be a multiple of 16 byte)
		//if my first variable is glm::vec2 uv, then glm::vec3 pos, now the start point of pos is 8 byte, which is not a multiple of 16 byte.
		//thus we change it to alignas(16) glm::vec3 pos, it will fill the empty 8 bytes and make the start point of pos be 16 byte.
		//thus, alignas(16) automatically fill the bytes to make the variable behind has a start memory point be multiple of 16.
		alignas(16) glm::vec3 position;
		alignas(16) glm::vec3 normal;
		alignas(16) glm::vec3 faceNor = glm::vec3(0);
		alignas(16) glm::vec3 symFaceNor = glm::vec3(0);
		alignas(8) glm::vec2 uv;
		alignas(4) int objectID;
		alignas(4) int faceID;
		alignas(4) int border = 0;
		alignas(4) int heID = -1;
		alignas(4) int uniqueID; //unique id in obj

		Vertex(glm::vec3 position, glm::vec3 normal, glm::vec2 uv, int objectID) :position(position), normal(normal), uv(uv), objectID(objectID) {};
		Vertex(glm::vec3 position, glm::vec3 normal, glm::vec2 uv, int objectID, int faceID, int border) :position(position), normal(normal), uv(uv), objectID(objectID), faceID(faceID), border(border) {};
		Vertex(glm::vec3 position, glm::vec3 normal, glm::vec2 uv, int objectID, int faceID) :position(position), normal(normal), uv(uv), objectID(objectID), faceID(faceID) {};
		Vertex(Vertex* ver, int faceID) : position(ver->position), normal(ver->normal), faceNor(ver->faceNor), symFaceNor(ver->symFaceNor), uv(ver->uv), objectID(ver->objectID), uniqueID(ver->uniqueID), faceID(faceID) {};
		Vertex(glm::vec3 position) :position(position) {};
		Vertex(Vertex* ver) : position(ver->position), normal(ver->normal), faceNor(ver->faceNor), symFaceNor(ver->symFaceNor), uv(ver->uv), objectID(ver->objectID), faceID(ver->faceID), border(ver->border), uniqueID(ver->uniqueID) {};

		bool operator==(const Vertex& other) const {
			return fequal(position.x, other.position.x) && (fequal(position.y, other.position.y) && (fequal(position.z, other.position.z)));
		}

		static VkVertexInputBindingDescription getBindingDescription() {
			VkVertexInputBindingDescription bindingDescription{};
			bindingDescription.binding = 0;
			bindingDescription.stride = sizeof(Vertex);
			bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
			return bindingDescription;
		}

		static std::array<VkVertexInputAttributeDescription, 9> getAttributeDescriptions() {
			std::array<VkVertexInputAttributeDescription, 9> attributeDescriptions{};
			attributeDescriptions[0].binding = 0;
			attributeDescriptions[0].location = 0;
			attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
			attributeDescriptions[0].offset = offsetof(Vertex, position); //0

			attributeDescriptions[1].binding = 0;
			attributeDescriptions[1].location = 1;
			attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
			attributeDescriptions[1].offset = offsetof(Vertex, normal);

			attributeDescriptions[2].binding = 0;
			attributeDescriptions[2].location = 2;
			attributeDescriptions[2].format = VK_FORMAT_R32G32B32_SFLOAT;
			attributeDescriptions[2].offset = offsetof(Vertex, faceNor);

			attributeDescriptions[3].binding = 0;
			attributeDescriptions[3].location = 3;
			attributeDescriptions[3].format = VK_FORMAT_R32G32B32_SFLOAT;
			attributeDescriptions[3].offset = offsetof(Vertex, symFaceNor);

			attributeDescriptions[4].binding = 0;
			attributeDescriptions[4].location = 4;
			attributeDescriptions[4].format = VK_FORMAT_R32G32_SFLOAT;
			attributeDescriptions[4].offset = offsetof(Vertex, uv);

			attributeDescriptions[5].binding = 0;
			attributeDescriptions[5].location = 5;
			attributeDescriptions[5].format = VK_FORMAT_R32_SINT;
			attributeDescriptions[5].offset = offsetof(Vertex, objectID);

			attributeDescriptions[6].binding = 0;
			attributeDescriptions[6].location = 6;
			attributeDescriptions[6].format = VK_FORMAT_R32_SINT;
			attributeDescriptions[6].offset = offsetof(Vertex, faceID);

			attributeDescriptions[7].binding = 0;
			attributeDescriptions[7].location = 7;
			attributeDescriptions[7].format = VK_FORMAT_R32_SINT;
			attributeDescriptions[7].offset = offsetof(Vertex, border);

			attributeDescriptions[8].binding = 0;
			attributeDescriptions[8].location = 8;
			attributeDescriptions[8].format = VK_FORMAT_R32_SINT;
			attributeDescriptions[8].offset = offsetof(Vertex, heID);

			return attributeDescriptions;
		}
	};

	struct FaceConnectToVertex {
		int neighborFaceCnt = 0;
		std::vector<int> neighborFaceIDs = {};
		void addFace(int id) {
			if (neighborFaceCnt < MAX_NEIGHBOR_FACE_COUNT) {
				neighborFaceIDs.push_back(id);
				++neighborFaceCnt;
			}
		}
	};

	struct HalfEdge {
		int id;
		Vertex* prevVer;
		Vertex* nextVer;
		HalfEdge* sym = nullptr;
		HalfEdge(int id, Vertex* prevVer, Vertex* nextVer) : id(id), prevVer(prevVer), nextVer(nextVer) {};
		void setSym(HalfEdge* sym) {
			this->sym = sym;
			sym->sym = this;
		};
	};

	struct Face {
		int objectID;
		int id;
		glm::vec3 normal;
		FaceConnectToVertex* v1;
		FaceConnectToVertex* v2;
		FaceConnectToVertex* v3;
		HalfEdge* he1;
		HalfEdge* he2;
		HalfEdge* he3;
		int neighborFaceCnt = 0;
		std::vector<int> neighborFaces = {};
		Face(int objectID, int id, FaceConnectToVertex* v1, FaceConnectToVertex* v2, FaceConnectToVertex* v3, HalfEdge* he1, HalfEdge* he2, HalfEdge* he3)
			:objectID(objectID), id(id), v1(v1), v2(v2), v3(v3), he1(he1), he2(he2), he3(he3) {};

		void getAllNeighborFaces() {
			for (auto v1tmp : v1->neighborFaceIDs) {
				if (v1tmp != id) {
					auto it = std::find(neighborFaces.begin(), neighborFaces.end(), v1tmp);
					if (it == neighborFaces.end()) {
						neighborFaces.push_back(v1tmp);
						++neighborFaceCnt;
					}
				}
			}
			for (auto v2tmp : v2->neighborFaceIDs) {
				if (v2tmp != id) {
					auto it = std::find(neighborFaces.begin(), neighborFaces.end(), v2tmp);
					if (it == neighborFaces.end()) {
						neighborFaces.push_back(v2tmp);
						++neighborFaceCnt;
					}
				}
			}
			for (auto v3tmp : v3->neighborFaceIDs) {
				if (v3tmp != id) {
					auto it = std::find(neighborFaces.begin(), neighborFaces.end(), v3tmp);
					if (it == neighborFaces.end()) {
						neighborFaces.push_back(v3tmp);
						++neighborFaceCnt;
					}
				}
			}
			if (neighborFaceCnt < MAX_NEIGHBOR_FACE_COUNT) {
				for (int i = neighborFaceCnt; i < MAX_NEIGHBOR_FACE_COUNT; ++i) {
					neighborFaces.push_back(-1);
				}
			}
			if (neighborFaceCnt > MAX_NEIGHBOR_FACE_COUNT) {
				for (int i = neighborFaceCnt; i > MAX_NEIGHBOR_FACE_COUNT; --i) {
					neighborFaces.pop_back();
				}
			}
		}

		void getAllBorderEdges(std::vector<int>& edgeIdx, std::vector<Vertex>& edgeVert) {
			if (he1->sym == nullptr) {
				int newIdx = edgeIdx.size();
				edgeIdx.push_back(newIdx);
				edgeIdx.push_back(newIdx + 1);
				Vertex tmpV1 = Vertex(he1->prevVer, id);
				Vertex tmpV2 = Vertex(he1->nextVer, id);
				edgeVert.push_back(tmpV1);
				edgeVert.push_back(tmpV2);
			}
			if (he2->sym == nullptr) {
				int newIdx = edgeIdx.size();
				edgeIdx.push_back(newIdx);
				edgeIdx.push_back(newIdx + 1);
				Vertex tmpV1 = Vertex(he2->prevVer, id);
				Vertex tmpV2 = Vertex(he2->nextVer, id);
				edgeVert.push_back(tmpV1);
				edgeVert.push_back(tmpV2);
			}
			if (he3->sym == nullptr) {
				int newIdx = edgeIdx.size();
				edgeIdx.push_back(newIdx);
				edgeIdx.push_back(newIdx + 1);
				Vertex tmpV1 = Vertex(he3->prevVer, id);
				Vertex tmpV2 = Vertex(he3->nextVer, id);
				edgeVert.push_back(tmpV1);
				edgeVert.push_back(tmpV2);
			}
		};

		void getAllEdges(std::vector<int>& edgeIdx, std::vector<Vertex>& edgeVert) {
			if (he1 != nullptr) {
				int newIdx = edgeIdx.size();
				edgeIdx.push_back(newIdx);
				edgeIdx.push_back(newIdx + 1);
				Vertex tmpV1 = Vertex(he1->prevVer, id);
				Vertex tmpV2 = Vertex(he1->nextVer, id);
				if (he1->sym != nullptr) {
					tmpV1.symFaceNor = he1->sym->prevVer->faceNor;
					tmpV2.symFaceNor = tmpV1.symFaceNor;
				}
				else {
					tmpV1.border = 1;
					tmpV2.border = 1;
				}
				tmpV1.heID = he1->id;
				tmpV2.heID = he1->id;
				edgeVert.push_back(tmpV1);
				edgeVert.push_back(tmpV2);
			}
			if (he2 != nullptr) {
				int newIdx = edgeIdx.size();
				edgeIdx.push_back(newIdx);
				edgeIdx.push_back(newIdx + 1);
				Vertex tmpV1 = Vertex(he2->prevVer, id);
				Vertex tmpV2 = Vertex(he2->nextVer, id);
				if (he2->sym != nullptr) {
					tmpV1.symFaceNor = he2->sym->prevVer->faceNor;
					tmpV2.symFaceNor = tmpV1.symFaceNor;
				}
				else {
					tmpV1.border = 1;
					tmpV2.border = 1;
				}
				tmpV1.heID = he2->id;
				tmpV2.heID = he2->id;
				edgeVert.push_back(tmpV1);
				edgeVert.push_back(tmpV2);
			}
			if (he3 != nullptr) {
				int newIdx = edgeIdx.size();
				edgeIdx.push_back(newIdx);
				edgeIdx.push_back(newIdx + 1);
				Vertex tmpV1 = Vertex(he3->prevVer, id);
				Vertex tmpV2 = Vertex(he3->nextVer, id);
				if (he3->sym != nullptr) {
					tmpV1.symFaceNor = he3->sym->prevVer->faceNor;
					tmpV2.symFaceNor = tmpV1.symFaceNor;
				}
				else {
					tmpV1.border = 1;
					tmpV2.border = 1;
				}
				tmpV1.heID = he3->id;
				tmpV2.heID = he3->id;
				edgeVert.push_back(tmpV1);
				edgeVert.push_back(tmpV2);
			}
		};
	};

	struct Quad {
		enum QuadType {
			PP,
			PE,
			EP,
			EE
		};
		bool line1 = false;
		bool line2 = false;
		bool line1Type = PP;
		bool line2Type = PP;
		int line1v1;
		int line1v2;
		int line2v1;
		int line2v2;
		std::tuple<int, int> line1v11;
		std::tuple<int, int> line1v22;
		std::tuple<int, int> line2v11;
		std::tuple<int, int> line2v22;
		double line1t1;
		double line1t2;
		double line2t1;
		double line2t2;

		Quad() : line1(false), line2(false) {};
		Quad(int line, int v1, int v2) {
			if (line == 1) {
				line1 = true;
				line1Type = PP;
				line1v1 = v1;
				line1v2 = v2;
			}
			else {
				line2 = true;
				line2Type = PP;
				line2v1 = v1;
				line2v2 = v2;
			}
		}
		Quad(int line, int v1, std::tuple<int, int> v22, double t2) {
			if (line == 1) {
				line1 = true;
				line1Type = PE;
				line1v1 = v1;
				line1v22 = v22;
				line1t2 = t2;
			}
			else {
				line2 = true;
				line2Type = PE;
				line2v1 = v1;
				line2v22 = v22;
				line2t2 = t2;
			}
		}
		Quad(int line, std::tuple<int, int> v11, int v2, double t1) {
			if (line == 1) {
				line1 = true;
				line1Type = EP;
				line1v11 = v11;
				line1v2 = v2;
				line1t1 = t1;
			}
			else {
				line2 = true;
				line2Type = EP;
				line2v11 = v11;
				line2v2 = v2;
				line2t1 = t1;
			}
		}
		Quad(int line, std::tuple<int, int> v11, std::tuple<int, int> v22, double t1, double t2) {
			if (line == 1) {
				line1 = true;
				line1Type = EE;
				line1v11 = v11;
				line1v22 = v22;
				line1t1 = t1;
				line1t2 = t2;
			}
			else {
				line2 = true;
				line2Type = EE;
				line2v11 = v11;
				line2v22 = v22;
				line2t1 = t1;
				line2t2 = t2;
			}
		}
	};

	struct Mesh {
		std::vector<uint32_t> index{};
		std::vector<uPtr<Vertex>> vertices1{};  //simple vertices, merged using position
		std::vector<uPtr<Vertex>> vertices2{};  //complex vertices, used for final rendering
		std::vector<uPtr<FaceConnectToVertex>> facesConnectToVertex{};
		std::vector<std::vector<uPtr<HalfEdge>>> halfEdges{};
		std::vector<uPtr<Face>> faces{};
		std::vector<int> neighborFacesData{};
		std::vector<int> objFaceCnt{};  //the starting face idx of each obj
		std::vector<glm::vec4> faceNors{};
		std::vector<int> edgeIdx{};
		std::vector<Vertex> edgeVert{};
		std::vector<int> edgeIdxHidden{};
		std::vector<Vertex> edgeVertHidden{};
		std::vector<int> unchangedEdgeIdx{};
		std::vector<Vertex> unchangedEdgeVert{};
		std::vector<glm::vec3> silhouettePoints;
		std::vector<glm::vec3> silhouettePointsDebug;
		std::vector<std::tuple<Standard_Real, Standard_Real>> silhouette2dPnt;
		std::vector<std::vector<glm::dvec2>> silhouetteBoundings;  //half length of silhouette2dPnt
		std::vector<int> silhouetteOri;
		std::vector<std::vector<glm::vec3>> uvLinePointsHidden;
		std::vector<std::tuple<double, double>> silhouette2dPntInside;
		std::vector<std::vector<Handle(Geom2d_BSplineCurve)>> outwireCurves2d;
		std::vector<std::vector<glm::vec3>> outedgePoints;
		std::vector<std::vector<glm::vec3>> occPoints;
		std::vector<Vertex> finEdgeVer;
		std::vector<int> finEdgeIdx;
		vks::Buffer verticesBuffer;
		vks::Buffer idxBuffer;
		vks::Buffer faceInfoBuffer;
		vks::Buffer faceDataBuffer;
		vks::Buffer faceNorBuffer;
		vks::Buffer edgeIdxBuffer;
		vks::Buffer edgeVertBuffer;
		vks::Buffer cpuImageBuffer;
		vks::Buffer lockedEdgeVertBuffer;
		vks::Buffer lockedEdgeIdxBuffer;
		vks::Buffer hiddenLineVertBuffer;
		vks::Buffer hiddenLineIdxBuffer;
		size_t lockedEdgeIdxCnt;
		size_t hiddenLineIdxCnt;
		void* edgePixelsRawData;
		int32_t* edgePixels;
		size_t vertexBuffersSize1 = 0;
		size_t vertexBuffersSize2 = 0;
		size_t indexBuffersSize = 0;
		size_t facesSize = 0;

		void create(std::vector<std::vector<uint32_t>>& indexBuffers, std::vector<std::vector<vkglTF::Vertex>>& vertexBuffers, vks::VulkanDevice* device, VkQueue transferQueue) {
			int objectID = 0;
			for (auto& vertexBuffer : vertexBuffers) {
				//find all unique vertices, store them into vertices1 and store verIDtoUniqueID
				int uniqueVerId = 0;
				std::unordered_map<Vertex*, int, VertexHash, VertexEqual> uniqueVers;
				std::unordered_map<int, int> verIDtoUniqueID;
				std::unordered_map<std::string, HalfEdge*> symHEs;
				std::vector<std::vector<int>> neighborFaces;
				for (int i = 0; i < vertexBuffer.size(); ++i) {
					vertexBuffer[i].pos *= 20.f;
					glm::vec3 pos = vertexBuffer[i].pos;
					glm::vec3 nor = vertexBuffer[i].normal;
					glm::vec2 uv = vertexBuffer[i].uv;
					uPtr<Vertex> tmpVer = mkU<Vertex>(pos, nor, uv, objectID);
					Vertex* tmpVerPtr = tmpVer.get();
					auto it = uniqueVers.find(tmpVerPtr);
					if (it != uniqueVers.end()) {
						//tmpVerPtr->uniqueID = it->second;
						verIDtoUniqueID[i] = it->second;
					}
					else {
						uniqueVers[tmpVerPtr] = uniqueVerId;
						if (i != uniqueVerId) {
							verIDtoUniqueID[i] = uniqueVerId;
						}
						tmpVerPtr->uniqueID = uniqueVerId++;
						vertices1.push_back(std::move(tmpVer));
						facesConnectToVertex.push_back(std::move(mkU<FaceConnectToVertex>()));
					}
				}
				int faceID = 0;
				int indexCnt = 0;
				int heID = 0;
				objFaceCnt.push_back(faceNors.size());
				halfEdges.push_back(std::vector<uPtr<HalfEdge>>{});
				for (int i = 0; i < indexBuffers[objectID].size(); i += 3) {
					int idx1 = indexBuffers[objectID][i];
					auto it = verIDtoUniqueID.find(idx1);
					if (it != verIDtoUniqueID.end()) {
						idx1 = it->second;
					}
					int idx2 = indexBuffers[objectID][i + 1];
					it = verIDtoUniqueID.find(idx2);
					if (it != verIDtoUniqueID.end()) {
						idx2 = it->second;
					}
					int idx3 = indexBuffers[objectID][i + 2];
					it = verIDtoUniqueID.find(idx3);
					if (it != verIDtoUniqueID.end()) {
						idx3 = it->second;
					}
					/*index.push_back(idx1 + vertexBuffersSize);
					index.push_back(idx2 + vertexBuffersSize);
					index.push_back(idx3 + vertexBuffersSize);*/
					Vertex* v1 = vertices1[idx1 + vertexBuffersSize1].get();
					Vertex* v2 = vertices1[idx2 + vertexBuffersSize1].get();
					Vertex* v3 = vertices1[idx3 + vertexBuffersSize1].get();
					FaceConnectToVertex* ftv1 = facesConnectToVertex[idx1 + vertexBuffersSize1].get();
					FaceConnectToVertex* ftv2 = facesConnectToVertex[idx2 + vertexBuffersSize1].get();
					FaceConnectToVertex* ftv3 = facesConnectToVertex[idx3 + vertexBuffersSize1].get();
					uPtr<Vertex> v1fin = mkU<Vertex>(v1, faceID);
					uPtr<Vertex> v2fin = mkU<Vertex>(v2, faceID);
					uPtr<Vertex> v3fin = mkU<Vertex>(v3, faceID);
					uPtr<HalfEdge> he1 = mkU<HalfEdge>(heID++, v1fin.get(), v2fin.get());
					uPtr<HalfEdge> he2 = mkU<HalfEdge>(heID++, v2fin.get(), v3fin.get());
					uPtr<HalfEdge> he3 = mkU<HalfEdge>(heID++, v3fin.get(), v1fin.get());
					ftv1->addFace(faceID);
					ftv2->addFace(faceID);
					ftv3->addFace(faceID);
					uPtr<Face> f = mkU<Face>(objectID, faceID, ftv1, ftv2, ftv3, he1.get(), he2.get(), he3.get());
					glm::vec3 faceNor = glm::normalize(glm::cross(v3->position - v2->position, v1->position - v2->position));
					//glm::vec3 faceNor = glm::normalize((v1->normal + v2->normal + v3->normal) / 3.f);
					v1fin->faceNor = faceNor;
					v2fin->faceNor = faceNor;
					v3fin->faceNor = faceNor;
					glm::vec2 uv1 = vertexBuffer[indexBuffers[objectID][i]].uv;
					glm::vec2 uv2 = vertexBuffer[indexBuffers[objectID][i + 1]].uv;
					glm::vec2 uv3 = vertexBuffer[indexBuffers[objectID][i + 2]].uv;
					v1fin->uv = uv1;
					v2fin->uv = uv2;
					v3fin->uv = uv3;
					faceNors.push_back(glm::vec4(faceNor, 1));
					index.push_back(vertexBuffersSize2 + indexCnt++);
					index.push_back(vertexBuffersSize2 + indexCnt++);
					index.push_back(vertexBuffersSize2 + indexCnt++);
					vertices2.push_back(std::move(v1fin));
					vertices2.push_back(std::move(v2fin));
					vertices2.push_back(std::move(v3fin));
					f->normal = faceNor;
					faces.push_back(std::move(f));

					std::array<HalfEdge*, 3> hes = { he1.get(), he2.get(), he3.get() };
					for (auto currHe : hes) {
						Vertex* ver1 = currHe->prevVer;
						Vertex* ver2 = currHe->nextVer;
						std::string key1 = std::to_string(ver1->uniqueID) + "#" + std::to_string(ver2->uniqueID);
						std::string key2 = std::to_string(ver2->uniqueID) + "#" + std::to_string(ver1->uniqueID);
						auto it = symHEs.find(key2);
						if (it != symHEs.end()) {
							currHe->setSym(it->second);
						}
						else {
							symHEs[key1] = currHe;
						}
					}
					halfEdges[objectID].push_back(std::move(he1));
					halfEdges[objectID].push_back(std::move(he2));
					halfEdges[objectID].push_back(std::move(he3));
					/*halfEdges.push_back(std::move(mkU<HalfEdge>(heID++, v1, v2)));
					halfEdges.push_back(std::move(mkU<HalfEdge>(heID++, v2, v3)));
					halfEdges.push_back(std::move(mkU<HalfEdge>(heID++, v3, v1)));
					HalfEdge* he1 = halfEdges[halfEdges.size() - 3].get();
					HalfEdge* he2 = halfEdges[halfEdges.size() - 2].get();
					HalfEdge* he3 = halfEdges[halfEdges.size() - 1].get();
					he1->prev = he3;
					he1->next = he2;
					he2->prev = he1;
					he2->next = he3;
					he3->prev = he2;
					he3->next = he1;
					std::array<HalfEdge*, 3> hes = { he1, he2, he3 };
					for (auto currHe : hes) {
						Vertex* ver1 = currHe->prevVer;
						Vertex* ver2 = currHe->nextVer;
						std::string key1 = std::to_string(ver1->uniqueID) + "#" + std::to_string(ver2->uniqueID);
						std::string key2 = std::to_string(ver2->uniqueID) + "#" + std::to_string(ver1->uniqueID);
						auto it = symHEs.find(key2);
						if (it != symHEs.end()) {
							currHe->setSym(it->second);
						}
						else {
							symHEs[key1] = currHe;
						}
					}
					uPtr<Face> f = mkU<Face>(faceID, he1, he2, he3);
					faces.push_back(std::move(f));*/
					++faceID;
				}
				for (int i = facesSize; i < faces.size(); ++i) {
					faces[i]->getAllNeighborFaces();
					//faces[i]->getAllBorderEdges(edgeIdx, edgeVert);
					faces[i]->getAllEdges(edgeIdx, edgeVert);
					neighborFacesData.insert(neighborFacesData.end(), faces[i]->neighborFaces.begin(), faces[i]->neighborFaces.end());
				}
				facesSize = faces.size();
				vertexBuffersSize1 = vertices1.size();
				vertexBuffersSize2 = vertices2.size();
				indexBuffersSize = index.size();
				++objectID;

			}

			//store vertices2's data
			std::vector<Vertex> vertices2Data;
			for (auto& ver : vertices2) {
				Vertex tmpVer(ver.get());
				vertices2Data.push_back(tmpVer);
			}

			size_t vertexBufferSize = vertexBuffersSize2 * sizeof(Vertex);
			//size_t vertexBufferSize = vertexBuffersSize * (2 * sizeof(glm::vec3) + sizeof(int));
			size_t indexBufferSize = indexBuffersSize * sizeof(uint32_t);
			size_t faceInfoSize = objFaceCnt.size() * sizeof(int);
			size_t faceDataSize = neighborFacesData.size() * sizeof(int);
			size_t faceNorSize = faceNors.size() * sizeof(glm::vec4);
			size_t edgeVertSize = edgeVert.size() * sizeof(Vertex);
			size_t edgeIdxSize = edgeIdx.size() * sizeof(int);
			struct StagingBuffer {
				VkBuffer buffer;
				VkDeviceMemory memory;
			} vertexStaging, indexStaging, faceInfoStaging, faceDataStaging, faceNorStaging, edgeVertStaging, edgeIdxStaging;

			// Create staging buffers
			// Vertex data
			//has to transfer from std::vector<uPtr<Vertex>> to std::vector<Vertex>, since uPtr vector is not contiguous like Vertex vector.
			/*std::vector<Vertex> verticesData{};
			for (auto& verData : vertices) {
				verticesData.push_back(Vertex(verData->position, verData->normal, verData->objectID));
			}*/
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				vertexBufferSize,
				&vertexStaging.buffer,
				&vertexStaging.memory,
				vertices2Data.data()));
			// Index data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				indexBufferSize,
				&indexStaging.buffer,
				&indexStaging.memory,
				index.data()));
			// Neighbor Face Info
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				faceInfoSize,
				&faceInfoStaging.buffer,
				&faceInfoStaging.memory,
				objFaceCnt.data()));
			// Neighbor Face Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				faceDataSize,
				&faceDataStaging.buffer,
				&faceDataStaging.memory,
				neighborFacesData.data()));
			//Face Nor Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				faceNorSize,
				&faceNorStaging.buffer,
				&faceNorStaging.memory,
				faceNors.data()));
			//Edge Vert Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				edgeVertSize,
				&edgeVertStaging.buffer,
				&edgeVertStaging.memory,
				edgeVert.data()));
			//Edge Idx Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				edgeIdxSize,
				&edgeIdxStaging.buffer,
				&edgeIdxStaging.memory,
				edgeIdx.data()));
			//
			// Create device local buffers
			// Vertex buffer
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				vertexBufferSize,
				&verticesBuffer.buffer,
				&verticesBuffer.memory));
			// Index buffer
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				indexBufferSize,
				&idxBuffer.buffer,
				&idxBuffer.memory));
			// Face Info
			//this buffer creation method is different from the one above
			//this one will create a default descriptor for you
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				&faceInfoBuffer,
				faceInfoSize));
			// Face Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				&faceDataBuffer,
				faceDataSize));
			//Face Nor
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				&faceNorBuffer,
				faceNorSize));
			// Edge Vert Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				edgeVertSize,
				&edgeVertBuffer.buffer,
				&edgeVertBuffer.memory));
			// Edge Idx Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				edgeIdxSize,
				&edgeIdxBuffer.buffer,
				&edgeIdxBuffer.memory));
			// Copy from staging buffers
			VkCommandBuffer copyCmd = device->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

			VkBufferCopy copyRegion = {};

			copyRegion.size = vertexBufferSize;
			vkCmdCopyBuffer(copyCmd, vertexStaging.buffer, verticesBuffer.buffer, 1, &copyRegion);

			copyRegion.size = indexBufferSize;
			vkCmdCopyBuffer(copyCmd, indexStaging.buffer, idxBuffer.buffer, 1, &copyRegion);

			copyRegion.size = faceInfoSize;
			vkCmdCopyBuffer(copyCmd, faceInfoStaging.buffer, faceInfoBuffer.buffer, 1, &copyRegion);

			copyRegion.size = faceDataSize;
			vkCmdCopyBuffer(copyCmd, faceDataStaging.buffer, faceDataBuffer.buffer, 1, &copyRegion);

			copyRegion.size = faceNorSize;
			vkCmdCopyBuffer(copyCmd, faceNorStaging.buffer, faceNorBuffer.buffer, 1, &copyRegion);

			copyRegion.size = edgeVertSize;
			vkCmdCopyBuffer(copyCmd, edgeVertStaging.buffer, edgeVertBuffer.buffer, 1, &copyRegion);

			copyRegion.size = edgeIdxSize;
			vkCmdCopyBuffer(copyCmd, edgeIdxStaging.buffer, edgeIdxBuffer.buffer, 1, &copyRegion);

			device->flushCommandBuffer(copyCmd, transferQueue, true);

			vkDestroyBuffer(device->logicalDevice, vertexStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, vertexStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, indexStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, indexStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, faceInfoStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, faceInfoStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, faceDataStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, faceDataStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, faceNorStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, faceNorStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, edgeVertStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, edgeVertStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, edgeIdxStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, edgeIdxStaging.memory, nullptr);
		}

		void setBuffers(vks::VulkanDevice* device, VkQueue transferQueue, std::vector<Vertex>& verticesData, std::vector<int>& idxData, std::vector<std::vector<glm::vec3>>& boundaryPoints, std::vector<std::vector<glm::vec3>>& uvParametricPoints, std::vector<glm::vec3>& silhouettePoints, std::vector<glm::vec3>& silhouettePointsDebug, std::vector<glm::vec3>& debugLines) {
			for (auto& idx : idxData) {
				index.push_back((uint32_t)idx);
			}
			size_t vertexBufferSize = verticesData.size() * sizeof(Vertex);
			//size_t vertexBufferSize = vertexBuffersSize * (2 * sizeof(glm::vec3) + sizeof(int));
			size_t indexBufferSize = idxData.size() * sizeof(uint32_t);
			size_t faceInfoSize = verticesData.size() * sizeof(int);
			size_t faceDataSize = verticesData.size() * sizeof(int);
			size_t faceNorSize = verticesData.size() * sizeof(glm::vec4);
			
			std::vector<int> empty_vec1(verticesData.size(), 0);
			std::vector<glm::vec4> empty_vec2(verticesData.size(), glm::vec4(0));
			objFaceCnt = empty_vec1;
			neighborFacesData = empty_vec1;
			faceNors = empty_vec2;
			edgeVert.clear();
			edgeIdx.clear();
			int edgeIdxCnt = 0;
			for (auto& boundaryPointsPerEdge : boundaryPoints) {
				for (int i = 0; i < boundaryPointsPerEdge.size(); ++i) {
					float t = (float)i / (float)boundaryPointsPerEdge.size();
					glm::vec3 color = glm::mix(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), t);
					//glm::vec3 color = glm::vec3(0, 0, 0); //red color for boundary edges
					if (i == 0) {
						edgeVert.push_back(Vertex(boundaryPointsPerEdge[i], color, glm::vec2(0), 0, 0, 1));
						edgeIdx.push_back(edgeIdxCnt++);
					}
					else if (i == boundaryPointsPerEdge.size() - 1) {
						edgeVert.push_back(Vertex(boundaryPointsPerEdge[i], color, glm::vec2(0), 0, 0, 1));
						edgeIdx.push_back(edgeIdxCnt++);
					}
					else {
						edgeVert.push_back(Vertex(boundaryPointsPerEdge[i], color, glm::vec2(0), 0, 0, 1));
						edgeVert.push_back(Vertex(boundaryPointsPerEdge[i], color, glm::vec2(0), 0, 0, 1));
						edgeIdx.push_back(edgeIdxCnt++);
						edgeIdx.push_back(edgeIdxCnt++);
					}
				}
			}
			for (auto& uvPointsPerEdge : uvParametricPoints) {
				if (uvPointsPerEdge.size() <= 1) {
					continue;
				}
				float colorMin = 0.3;
				float colorStep = (1 - colorMin) / (uvPointsPerEdge.size() - 1);
				for (int i = 0; i < uvPointsPerEdge.size(); ++i) {
					if (i == 0) {
						edgeVert.push_back(Vertex(uvPointsPerEdge[i], glm::vec3(colorMin), glm::vec2(0), 0, 0, 1));
						edgeIdx.push_back(edgeIdxCnt++);
					}
					else if (i == uvPointsPerEdge.size() - 1) {
						edgeVert.push_back(Vertex(uvPointsPerEdge[i], glm::vec3(1), glm::vec2(0), 0, 0, 1));
						edgeIdx.push_back(edgeIdxCnt++);
					}
					else {
						float color1 = colorMin + colorStep * (i - 1);
						float color2 = colorMin + colorStep * i;
						edgeVert.push_back(Vertex(uvPointsPerEdge[i], glm::vec3(color1), glm::vec2(0), 0, 0, 1));
						edgeVert.push_back(Vertex(uvPointsPerEdge[i], glm::vec3(color2), glm::vec2(0), 0, 0, 1));
						edgeIdx.push_back(edgeIdxCnt++);
						edgeIdx.push_back(edgeIdxCnt++);
					}
				}
				/*for (int i = 0; i < uvPointsPerEdge.size(); ++i) {
					edgeVert.push_back(Vertex(uvPointsPerEdge[i], glm::vec3(1), glm::vec2(0), 0));
					edgeIdx.push_back(edgeIdxCnt++);
				}*/
			}

			unchangedEdgeVert = edgeVert;
			unchangedEdgeIdx = edgeIdx;

			for(auto& silhouettePoint : silhouettePoints) {
				edgeVert.push_back(Vertex(silhouettePoint, glm::vec3(0,0,1), glm::vec2(0), 0, 0, 1));
				edgeIdx.push_back(edgeIdxCnt++);
			}
			for (auto& silhouettePoint : silhouettePointsDebug) {
				edgeVert.push_back(Vertex(silhouettePoint, glm::vec3(1, 0, 1), glm::vec2(0), 1, 0, 1));
				edgeIdx.push_back(edgeIdxCnt++);
			}
			for(auto& debugPoint : debugLines) {
				edgeVert.push_back(Vertex(debugPoint, glm::vec3(1, 1, 0), glm::vec2(0), 0, 0, 1));
				edgeIdx.push_back(edgeIdxCnt++);
			}



			//set hidden line points
			int edgeIdxHiddenCnt = 0;
			uvLinePointsHidden = occPoints;
			for (auto& uvPoint : uvLinePointsHidden) {
				if (uvPoint.size() <= 1) {
					continue;
				}
				for (int i = 0; i < uvPoint.size(); ++i) {
					glm::vec3 color = glm::vec3(1);
					//glm::vec3 color = glm::vec3(0, 0, 0); //red color for boundary edges
					if (i == 0) {
						edgeVertHidden.push_back(Vertex(uvPoint[i], color, glm::vec2(0), 0, 0, 1));
						edgeIdxHidden.push_back(edgeIdxHiddenCnt++);
					}
					else if (i == uvPoint.size() - 1) {
						edgeVertHidden.push_back(Vertex(uvPoint[i], color, glm::vec2(0), 0, 0, 1));
						edgeIdxHidden.push_back(edgeIdxHiddenCnt++);
					}
					else {
						edgeVertHidden.push_back(Vertex(uvPoint[i], color, glm::vec2(0), 0, 0, 1));
						edgeVertHidden.push_back(Vertex(uvPoint[i], color, glm::vec2(0), 0, 0, 1));
						edgeIdxHidden.push_back(edgeIdxHiddenCnt++);
						edgeIdxHidden.push_back(edgeIdxHiddenCnt++);
					}
				}
			}
			/*for (auto& uvPoint : uvLinePointsHidden) {
				for (auto& p : uvPoint) {
					edgeVertHidden.push_back(Vertex(p, glm::vec3(1), glm::vec2(0), 0, 0, 1));
					edgeIdxHidden.push_back(edgeIdxHiddenCnt++);
				}
			}*/
			/*for (auto& uvPoint : uvLinePointsHidden) {
				for (int i = 0; i < uvPoint.size(); i+=2) {
					edgeVertHidden.push_back(Vertex(uvPoint[i], glm::vec3(1,0,0), glm::vec2(0), 0, 0, 1));
					edgeVertHidden.push_back(Vertex(uvPoint[i + 1], glm::vec3(0, 1, 0), glm::vec2(0), 0, 0, 1));
					edgeIdxHidden.push_back(edgeIdxHiddenCnt++);
					edgeIdxHidden.push_back(edgeIdxHiddenCnt++);
				}
			}*/

			size_t edgeVertSize = edgeVert.size() * sizeof(Vertex);
			size_t edgeIdxSize = edgeIdx.size() * sizeof(int);

			size_t edgeVertHiddenSize = edgeVertHidden.size() * sizeof(Vertex);
			size_t edgeIdxHiddenSize = edgeIdxHidden.size() * sizeof(int);

			struct StagingBuffer {
				VkBuffer buffer;
				VkDeviceMemory memory;
			} vertexStaging, indexStaging, faceInfoStaging, faceDataStaging, faceNorStaging, edgeVertStaging, edgeIdxStaging, lockedEdgeVertStaging, lockedEdgeIdxStaging, hiddenLineVertStaging, hiddenLineIdxStaging;

			// Create staging buffers
			// Vertex data
			//has to transfer from std::vector<uPtr<Vertex>> to std::vector<Vertex>, since uPtr vector is not contiguous like Vertex vector.
			/*std::vector<Vertex> verticesData{};
			for (auto& verData : vertices) {
				verticesData.push_back(Vertex(verData->position, verData->normal, verData->objectID));
			}*/
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				vertexBufferSize,
				&vertexStaging.buffer,
				&vertexStaging.memory,
				verticesData.data()));
			// Index data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				indexBufferSize,
				&indexStaging.buffer,
				&indexStaging.memory,
				idxData.data()));
			// Neighbor Face Info
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				faceInfoSize,
				&faceInfoStaging.buffer,
				&faceInfoStaging.memory,
				objFaceCnt.data()));
			// Neighbor Face Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				faceDataSize,
				&faceDataStaging.buffer,
				&faceDataStaging.memory,
				neighborFacesData.data()));
			//Face Nor Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				faceNorSize,
				&faceNorStaging.buffer,
				&faceNorStaging.memory,
				faceNors.data()));
			//Edge Vert Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				edgeVertSize,
				&edgeVertStaging.buffer,
				&edgeVertStaging.memory,
				edgeVert.data()));
			//Edge Idx Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				edgeIdxSize,
				&edgeIdxStaging.buffer,
				&edgeIdxStaging.memory,
				edgeIdx.data()));
			//L Edge Vert Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				edgeVertSize,
				&lockedEdgeVertStaging.buffer,
				&lockedEdgeVertStaging.memory,
				edgeVert.data()));
			//L Edge Idx Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				edgeIdxSize,
				&lockedEdgeIdxStaging.buffer,
				&lockedEdgeIdxStaging.memory,
				edgeIdx.data()));
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				edgeVertHiddenSize,
				&hiddenLineVertStaging.buffer,
				&hiddenLineVertStaging.memory,
				edgeVertHidden.data()));
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				edgeIdxHiddenSize,
				&hiddenLineIdxStaging.buffer,
				&hiddenLineIdxStaging.memory,
				edgeIdxHidden.data()));
			//
			// Create device local buffers
			// Vertex buffer
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				vertexBufferSize,
				&verticesBuffer.buffer,
				&verticesBuffer.memory));
			// Index buffer
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				indexBufferSize,
				&idxBuffer.buffer,
				&idxBuffer.memory));
			// Face Info
			//this buffer creation method is different from the one above
			//this one will create a default descriptor for you
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				&faceInfoBuffer,
				faceInfoSize));
			// Face Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				&faceDataBuffer,
				faceDataSize));
			//Face Nor
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				&faceNorBuffer,
				faceNorSize));
			// Edge Vert Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				edgeVertSize,
				&edgeVertBuffer.buffer,
				&edgeVertBuffer.memory));
			// Edge Idx Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				edgeIdxSize,
				&edgeIdxBuffer.buffer,
				&edgeIdxBuffer.memory));
			// Edge Vert Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				edgeVertSize,
				&lockedEdgeVertBuffer.buffer,
				&lockedEdgeVertBuffer.memory));
			// Edge Idx Data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				edgeIdxSize,
				&lockedEdgeIdxBuffer.buffer,
				&lockedEdgeIdxBuffer.memory));
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				edgeVertHiddenSize,
				&hiddenLineVertBuffer.buffer,
				&hiddenLineVertBuffer.memory));
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				edgeIdxHiddenSize,
				&hiddenLineIdxBuffer.buffer,
				&hiddenLineIdxBuffer.memory));
			// Copy from staging buffers
			VkCommandBuffer copyCmd = device->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

			VkBufferCopy copyRegion = {};

			copyRegion.size = vertexBufferSize;
			vkCmdCopyBuffer(copyCmd, vertexStaging.buffer, verticesBuffer.buffer, 1, &copyRegion);

			copyRegion.size = indexBufferSize;
			vkCmdCopyBuffer(copyCmd, indexStaging.buffer, idxBuffer.buffer, 1, &copyRegion);

			copyRegion.size = faceInfoSize;
			vkCmdCopyBuffer(copyCmd, faceInfoStaging.buffer, faceInfoBuffer.buffer, 1, &copyRegion);

			copyRegion.size = faceDataSize;
			vkCmdCopyBuffer(copyCmd, faceDataStaging.buffer, faceDataBuffer.buffer, 1, &copyRegion);

			copyRegion.size = faceNorSize;
			vkCmdCopyBuffer(copyCmd, faceNorStaging.buffer, faceNorBuffer.buffer, 1, &copyRegion);

			copyRegion.size = edgeVertSize;
			vkCmdCopyBuffer(copyCmd, edgeVertStaging.buffer, edgeVertBuffer.buffer, 1, &copyRegion);

			copyRegion.size = edgeIdxSize;
			vkCmdCopyBuffer(copyCmd, edgeIdxStaging.buffer, edgeIdxBuffer.buffer, 1, &copyRegion);

			copyRegion.size = edgeVertSize;
			vkCmdCopyBuffer(copyCmd, lockedEdgeVertStaging.buffer, lockedEdgeVertBuffer.buffer, 1, &copyRegion);

			copyRegion.size = edgeIdxSize;
			vkCmdCopyBuffer(copyCmd, lockedEdgeIdxStaging.buffer, lockedEdgeIdxBuffer.buffer, 1, &copyRegion);

			copyRegion.size = edgeVertHiddenSize;
			vkCmdCopyBuffer(copyCmd, hiddenLineVertStaging.buffer, hiddenLineVertBuffer.buffer, 1, &copyRegion);
			copyRegion.size = edgeIdxHiddenSize;
			vkCmdCopyBuffer(copyCmd, hiddenLineIdxStaging.buffer, hiddenLineIdxBuffer.buffer, 1, &copyRegion);

			device->flushCommandBuffer(copyCmd, transferQueue, true);
			lockedEdgeIdxCnt = edgeIdx.size();
			hiddenLineIdxCnt = edgeIdxHidden.size();
			vkDestroyBuffer(device->logicalDevice, vertexStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, vertexStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, indexStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, indexStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, faceInfoStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, faceInfoStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, faceDataStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, faceDataStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, faceNorStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, faceNorStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, edgeVertStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, edgeVertStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, edgeIdxStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, edgeIdxStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, lockedEdgeVertStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, lockedEdgeVertStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, lockedEdgeIdxStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, lockedEdgeIdxStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, hiddenLineVertStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, hiddenLineVertStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, hiddenLineIdxStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, hiddenLineIdxStaging.memory, nullptr);
		}

		void bindBuffers(VkCommandBuffer commandBuffer) {
			const VkDeviceSize offsets[1] = { 0 };
			vkCmdBindVertexBuffers(commandBuffer, 0, 1, &verticesBuffer.buffer, offsets);
			vkCmdBindIndexBuffer(commandBuffer, idxBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);

		}

		void bindLineBuffers(VkCommandBuffer commandBuffer) {
			const VkDeviceSize offsets[1] = { 0 };
			vkCmdBindVertexBuffers(commandBuffer, 0, 1, &edgeVertBuffer.buffer, offsets);
			vkCmdBindIndexBuffer(commandBuffer, edgeIdxBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);
		}

		void bindLockedEdgeBuffers(VkCommandBuffer commandBuffer) {
			const VkDeviceSize offsets[1] = { 0 };
			vkCmdBindVertexBuffers(commandBuffer, 0, 1, &lockedEdgeVertBuffer.buffer, offsets);
			vkCmdBindIndexBuffer(commandBuffer, lockedEdgeIdxBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);
		}

		void bindHiddenLineBuffers(VkCommandBuffer commandBuffer) {
			const VkDeviceSize offsets[1] = { 0 };
			vkCmdBindVertexBuffers(commandBuffer, 0, 1, &hiddenLineVertBuffer.buffer, offsets);
			vkCmdBindIndexBuffer(commandBuffer, hiddenLineIdxBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);
		}

		void analyzeEdgePixels(vks::VulkanDevice* device, int32_t height, int32_t width, VkQueue transferQueue) {
			std::unordered_map<int32_t, std::unordered_set<int32_t>> edgeList;
			edgePixels = static_cast<int32_t*>(edgePixelsRawData);
			for (int y = 0; y < height; ++y) {
				for (int x = 0; x < width; ++x) {
					int index = (y * width + x) * 4;

					int32_t objID = edgePixels[index];
					int32_t heID = edgePixels[index + 1];
					int32_t color = edgePixels[index + 2];
					int32_t a = edgePixels[index + 3];
					if (color == 1) {
						auto it = edgeList.find(objID);
						if (it == edgeList.end()) {
							edgeList[objID] = std::unordered_set<int32_t>{ heID };
						}
						else {
							edgeList[objID].insert(heID);
						}
					}
				}
			}
			std::vector<Vertex> finVer;
			std::vector<int> finIdx;
			int idx = 0;
			struct StagingBuffer {
				VkBuffer buffer;
				VkDeviceMemory memory;
			} vertexStaging, indexStaging;
			for (const auto& pair : edgeList) {
				for (const auto& heID : pair.second) {
					finVer.push_back(Vertex(halfEdges[pair.first][heID]->prevVer));
					finVer.push_back(Vertex(halfEdges[pair.first][heID]->nextVer));
					finIdx.push_back(idx++);
					finIdx.push_back(idx++);
				}
			}
			lockedEdgeIdxCnt = idx;
			vkDestroyBuffer(device->logicalDevice, lockedEdgeVertBuffer.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, lockedEdgeVertBuffer.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, lockedEdgeIdxBuffer.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, lockedEdgeIdxBuffer.memory, nullptr);
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				finVer.size() * sizeof(Vertex),
				&vertexStaging.buffer,
				&vertexStaging.memory,
				finVer.data()));
			// Index data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				finIdx.size() * sizeof(int),
				&indexStaging.buffer,
				&indexStaging.memory,
				finIdx.data()));
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				finVer.size() * sizeof(Vertex),
				&lockedEdgeVertBuffer.buffer,
				&lockedEdgeVertBuffer.memory));
			// Index buffer
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				finIdx.size() * sizeof(int),
				&lockedEdgeIdxBuffer.buffer,
				&lockedEdgeIdxBuffer.memory));

			VkCommandBuffer copyCmd = device->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);
			VkBufferCopy copyRegion = {};

			copyRegion.size = finVer.size() * sizeof(Vertex);
			vkCmdCopyBuffer(copyCmd, vertexStaging.buffer, lockedEdgeVertBuffer.buffer, 1, &copyRegion);

			copyRegion.size = finIdx.size() * sizeof(int);
			vkCmdCopyBuffer(copyCmd, indexStaging.buffer, lockedEdgeIdxBuffer.buffer, 1, &copyRegion);
			device->flushCommandBuffer(copyCmd, transferQueue, true);

			vkDestroyBuffer(device->logicalDevice, vertexStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, vertexStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, indexStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, indexStaging.memory, nullptr);
		}

		void analyzeEdgePixels2(vks::VulkanDevice* device, int32_t height, int32_t width, VkQueue transferQueue, const Camera& camera) {
			/*std::unordered_map<int32_t, std::unordered_set<int32_t>> edgeList;
			edgePixels = static_cast<int32_t*>(edgePixelsRawData);
			for (int y = 0; y < height; ++y) {
				for (int x = 0; x < width; ++x) {
					int index = (y * width + x) * 4;

					int32_t objID = edgePixels[index];
					int32_t heID = edgePixels[index + 1];
					int32_t color = edgePixels[index + 2];
					int32_t a = edgePixels[index + 3];
					if (color == 1) {
						auto it = edgeList.find(objID);
						if (it == edgeList.end()) {
							edgeList[objID] = std::unordered_set<int32_t>{ heID };
						}
						else {
							edgeList[objID].insert(heID);
						}
					}
				}
			}*/
			//int idx = 0;
			struct StagingBuffer {
				VkBuffer buffer;
				VkDeviceMemory memory;
			} vertexStaging, indexStaging, vertexHiddenStaging, indexHiddenStaging;
			//scan-convert
			/*glm::mat4 persp = glm::ortho(camera.orthoLeft, camera.orthoRight, camera.orthoBottom, camera.orthoTop, camera.znear, camera.zfar);
			glm::mat4 view = camera.matrices.view;
			for (const auto& pair : edgeList) {
				for (const auto& heID : pair.second) {
					Vertex v1 = halfEdges[pair.first][heID]->prevVer;
					Vertex v2 = halfEdges[pair.first][heID]->nextVer;
					glm::vec4 v1Per = persp * view * glm::vec4(v1.position, 1);
					glm::vec4 v2Per = persp * view * glm::vec4(v2.position, 1);
					glm::vec3 v1Scr = glm::vec3(v1Per.x / v1Per.w, v1Per.y / v1Per.w, v1Per.z / v1Per.w);
					glm::vec3 v2Scr = glm::vec3(v2Per.x / v2Per.w, v2Per.y / v2Per.w, v2Per.z / v2Per.w);
					std::vector<Vertex> currSegments = scanConvert(v1Scr, v2Scr, height, width, pair.first, heID);
					for (auto& ver : currSegments) {
						finVer.push_back(ver);
						finIdx.push_back(idx++);
					}
				}
			}*/
			//glm::mat4 view = camera.matrices.view;
			//for (auto& halfEdge : halfEdges) {
			//	for (auto& he : halfEdge) {
			//		//boundary edge
			//		if (he->sym == nullptr) {
			//			finVer.push_back(Vertex(he->prevVer));
			//			finVer.push_back(Vertex(he->nextVer));
			//			finIdx.push_back(idx++);
			//			finIdx.push_back(idx++);
			//			continue;
			//		}
			//		//silhouette
			//		float faceNorView = glm::dot(view * glm::vec4(he->prevVer->faceNor,0), glm::vec4(0,0,-1,0));
			//		float symFaceNorView = glm::dot(view * glm::vec4(he->sym->prevVer->faceNor, 0), glm::vec4(0,0,-1,0));
			//		if ((faceNorView * symFaceNorView) <= 0.f) {
			//			finVer.push_back(Vertex(he->prevVer));
			//			finVer.push_back(Vertex(he->nextVer));
			//			finIdx.push_back(idx++);
			//			finIdx.push_back(idx++);
			//		}
			//	}
			//}
			//lockedEdgeIdxCnt = idx;
			vkDestroyBuffer(device->logicalDevice, lockedEdgeVertBuffer.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, lockedEdgeVertBuffer.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, lockedEdgeIdxBuffer.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, lockedEdgeIdxBuffer.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, edgeVertBuffer.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, edgeVertBuffer.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, edgeIdxBuffer.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, edgeIdxBuffer.memory, nullptr);
			finEdgeVer.clear();
			finEdgeIdx.clear();
			edgeVertHidden.clear();
			edgeIdxHidden.clear();
			finEdgeVer = unchangedEdgeVert;
			finEdgeIdx = unchangedEdgeIdx;
			for(int i = 0; i < silhouettePoints.size(); ++i) {
				finEdgeVer.push_back(Vertex(silhouettePoints[i], glm::vec3(0, 0, 1), glm::vec2(0), 0, 0, 1));
				finEdgeIdx.push_back(i + unchangedEdgeIdx.size());
			}
			for (int i = 0; i < silhouettePointsDebug.size(); ++i) {
				finEdgeVer.push_back(Vertex(silhouettePointsDebug[i], glm::vec3(1, 0, 1), glm::vec2(0), 1, 0, 1));
				finEdgeIdx.push_back(finEdgeIdx.size());
			}
			
			for (auto& occP : occPoints) {
				for (int i = 0; i < occP.size(); ++i) {
					if (i == 0) {
						edgeVertHidden.push_back(Vertex(occP[i], glm::vec3(1), glm::vec2(0), 0, 0, 1));
						edgeIdxHidden.push_back(edgeIdxHidden.size());
					}
					else if (i == occP.size() - 1) {
						edgeVertHidden.push_back(Vertex(occP[i], glm::vec3(1), glm::vec2(0), 0, 0, 1));
						edgeIdxHidden.push_back(edgeIdxHidden.size());
					}
					else {
						edgeVertHidden.push_back(Vertex(occP[i], glm::vec3(1), glm::vec2(0), 0, 0, 1));
						edgeVertHidden.push_back(Vertex(occP[i], glm::vec3(1), glm::vec2(0), 0, 0, 1));
						edgeIdxHidden.push_back(edgeIdxHidden.size());
						edgeIdxHidden.push_back(edgeIdxHidden.size());
					}
				}
			}
			
			int edgeVertHiddenSize = edgeVertHidden.size() * sizeof(Vertex);
			int edgeIdxHiddenSize = edgeIdxHidden.size() * sizeof(int);
			hiddenLineIdxCnt = edgeIdxHidden.size();
			lockedEdgeIdxCnt = finEdgeIdx.size();
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				finEdgeVer.size() * sizeof(Vertex),
				&vertexStaging.buffer,
				&vertexStaging.memory,
				finEdgeVer.data()));
			// Index data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				finEdgeIdx.size() * sizeof(int),
				&indexStaging.buffer,
				&indexStaging.memory,
				finEdgeIdx.data()));
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				edgeVertHidden.size() * sizeof(Vertex),
				&vertexHiddenStaging.buffer,
				&vertexHiddenStaging.memory,
				edgeVertHidden.data()));
			// Index data
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
				VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
				edgeIdxHidden.size() * sizeof(int),
				&indexHiddenStaging.buffer,
				&indexHiddenStaging.memory,
				edgeIdxHidden.data()));

			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				finEdgeVer.size() * sizeof(Vertex),
				&lockedEdgeVertBuffer.buffer,
				&lockedEdgeVertBuffer.memory));
			// Index buffer
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				finEdgeIdx.size() * sizeof(int),
				&lockedEdgeIdxBuffer.buffer,
				&lockedEdgeIdxBuffer.memory));
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				finEdgeVer.size() * sizeof(Vertex),
				&edgeVertBuffer.buffer,
				&edgeVertBuffer.memory));
			// Index buffer
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				finEdgeIdx.size() * sizeof(int),
				&edgeIdxBuffer.buffer,
				&edgeIdxBuffer.memory));
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				edgeVertHiddenSize,
				&hiddenLineVertBuffer.buffer,
				&hiddenLineVertBuffer.memory));
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				edgeIdxHiddenSize,
				&hiddenLineIdxBuffer.buffer,
				&hiddenLineIdxBuffer.memory));

			VkCommandBuffer copyCmd = device->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);
			VkBufferCopy copyRegion = {};

			copyRegion.size = finEdgeVer.size() * sizeof(Vertex);
			vkCmdCopyBuffer(copyCmd, vertexStaging.buffer, lockedEdgeVertBuffer.buffer, 1, &copyRegion);
			vkCmdCopyBuffer(copyCmd, vertexStaging.buffer, edgeVertBuffer.buffer, 1, &copyRegion);

			copyRegion.size = finEdgeIdx.size() * sizeof(int);
			vkCmdCopyBuffer(copyCmd, indexStaging.buffer, lockedEdgeIdxBuffer.buffer, 1, &copyRegion);
			vkCmdCopyBuffer(copyCmd, indexStaging.buffer, edgeIdxBuffer.buffer, 1, &copyRegion);

			copyRegion.size = edgeVertHiddenSize;
			vkCmdCopyBuffer(copyCmd, vertexHiddenStaging.buffer, hiddenLineVertBuffer.buffer, 1, &copyRegion);

			copyRegion.size = edgeIdxHiddenSize;
			vkCmdCopyBuffer(copyCmd, indexHiddenStaging.buffer, hiddenLineIdxBuffer.buffer, 1, &copyRegion);

			device->flushCommandBuffer(copyCmd, transferQueue, true);

			vkDestroyBuffer(device->logicalDevice, vertexStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, vertexStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, indexStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, indexStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, vertexHiddenStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, vertexHiddenStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, indexHiddenStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, indexHiddenStaging.memory, nullptr);
		}

		std::vector<Vertex> scanConvert(glm::vec3 v1Scr, glm::vec3 v2Scr, int32_t height, int32_t width, int objID, int heID) {
			glm::vec3 v1Pix = glm::vec3(((v1Scr.x * 0.5f) + 0.5f) * width, ((v1Scr.y * 0.5f) + 0.5f) * height, v1Scr.z);
			glm::vec3 v1Pix0 = glm::vec3(v1Pix.x, v1Pix.y, 0);
			glm::vec3 v2Pix = glm::vec3(((v2Scr.x * 0.5f) + 0.5f) * width, ((v2Scr.y * 0.5f) + 0.5f) * height, v2Scr.z);
			glm::vec3 v2Pix0 = glm::vec3(v2Pix.x, v2Pix.y, 0);
			glm::vec3 v13D = halfEdges[objID][heID]->prevVer->position;
			glm::vec3 v23D = halfEdges[objID][heID]->nextVer->position;
			int x0 = std::floor(v1Pix.x);
			int x1 = std::floor(v2Pix.x);
			int y0 = std::floor(v1Pix.y);
			int y1 = std::floor(v2Pix.y);

			int dx = abs(x1 - x0);
			int dy = abs(y1 - y0);
			int sx = (x0 < x1) ? 1 : -1;
			int sy = (y0 < y1) ? 1 : -1;

			int err = dx - dy;
			glm::vec3 offset = glm::normalize(glm::cross(glm::vec3(0, 0, 1), v2Pix0 - v1Pix0));
			int segment = 0;
			bool preVis = false;
			bool pass = false;
			std::vector<Vertex> segments = {};
			glm::vec2 orig = glm::vec2(x0, y0);
			glm::vec2 whole = glm::vec2(x1 - x0, y1 - y0);
			glm::vec3 start;
			glm::vec3 end;
			int stride = 1;

			while (true) {
				if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height) {
					int index = ((height - y0 - 1) * width + x0) * 4;
					pass = false;
					if ((index >= 0) && (index < 4 * width * height)) {
						int32_t checkObjID = edgePixels[index];
						int32_t checkHeID = edgePixels[index + 1];
						int32_t checkColor = edgePixels[index + 2];
						if ((checkObjID == objID) && (checkHeID == heID)) {
							pass = true;
						}
					}
					if (!pass) {
						for (int i = -stride; i <= stride; ++i) {
							for (int j = -stride; j <= stride; ++j) {
								if ((i == 0) && (j == 0)) {
									continue;
								}
								int tempx0 = x0 + i;
								int tempy0 = y0 + j;
								int tempIndex = ((height - tempy0 - 1) * width + tempx0) * 4;
								if ((tempIndex < 0) || (tempIndex >= 4 * width * height)) {
									continue;
								}
								
								int32_t checkObjID_temp = edgePixels[tempIndex];
								int32_t checkHeID_temp = edgePixels[tempIndex + 1];
								int32_t checkColor_temp = edgePixels[tempIndex + 2];
								if ((checkObjID_temp == objID) && (checkHeID_temp == heID)) {
									pass = true;
									break;
								}
							}
						};
					}
				}
				if (preVis != pass) {
					int debug = 1;
					if (preVis == false) {
						//starting point
						float t = glm::dot(glm::vec2(x0, y0) - orig, glm::normalize(whole)) / glm::length(whole);
						start = glm::mix(v13D, v23D, t);
						segments.push_back(Vertex(start));
					}
					else {
						//end point
						float t = glm::dot(glm::vec2(x0, y0) - orig, glm::normalize(whole)) / glm::length(whole);
						end = glm::mix(v13D, v23D, t);
						segments.push_back(Vertex(end));
					}
					preVis = pass;
				}

				if ((x0 == x1) && (y0 == y1)) {
					if (preVis == true) {
						//float t = glm::dot(glm::vec2(x0, y0) - orig, whole) / glm::length(whole);
						end = glm::mix(v13D, v23D, 1.f);
						segments.push_back(Vertex(end));
					}
					break;
				}
				int e2 = 2 * err;
				if (e2 > -dy) {
					err -= dy;
					x0 += sx;
				}
				if (e2 < dx) {
					err += dx;
					y0 += sy;
				}
			}
			return segments;
		}

		void calculateSilhouette(TopoDS_Shape& shape, Camera& camera) {
			TopExp_Explorer faceExp;
			silhouettePoints.clear();
			silhouette2dPnt.clear();
			silhouette2dPntInside.clear();
			for (faceExp.Init(shape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
				TopoDS_Face face = TopoDS::Face(faceExp.Current());
				Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
				//silhouette
				if (surface->IsKind(STANDARD_TYPE(Geom_BSplineSurface))) {
					Handle(Geom_BSplineSurface) bsplineSurf = Handle(Geom_BSplineSurface)::DownCast(surface);
					/*int bsplineU = bsplineSurf->UDegree();
					int bsplineV = bsplineSurf->VDegree();
					int bsplineUKnots = bsplineSurf->NbUKnots();
					int bsplineVKnots = bsplineSurf->NbVKnots();
					std::cout << "  UDegree: " << bsplineSurf->UDegree() << std::endl;
					std::cout << "  VDegree: " << bsplineSurf->VDegree() << std::endl;
					std::cout << "  Num UKnots: " << bsplineSurf->NbUKnots() << std::endl;
					std::cout << "  Num VKnots: " << bsplineSurf->NbVKnots() << std::endl;
					std::cout << "  Num Control Points: ("
						<< bsplineSurf->NbUPoles() << ", "
						<< bsplineSurf->NbVPoles() << ")" << std::endl;*/
					Standard_Real u1, u2, v1, v2;
					bsplineSurf->Bounds(u1, u2, v1, v2);
					if ((bsplineSurf->IsUPeriodic()) || (bsplineSurf->IsVPeriodic())) {
						continue;
					}
					const int divide = 40;
					double uStep = (u2 - u1) / ((double)divide);
					double vStep = (v2 - v1) / ((double)divide);
					std::array<std::array<float, (divide + 1)>, (divide + 1)> silhouetteNorsArray;
					std::array<std::array<bool, (divide + 1)>, (divide + 1)> silhouetteBoolsArray;
					std::array<std::array<glm::vec3, (divide + 1)>, (divide + 1)> silhouettePointsArray;
					glm::vec3 cameraNor = glm::vec3(0.f, 0.f, -1.f);
					for (int i = 0; i < divide; ++i) {
						Standard_Real su1 = u1 + double(i) * uStep;
						Standard_Real su2 = u1 + double(i + 1) * uStep;
						for (int j = 0; j < divide; ++j) {
							Standard_Real sv1 = v1 + double(j) * vStep;
							Standard_Real sv2 = v1 + double(j + 1) * vStep;

							GeomLProp_SLProps props(bsplineSurf, su1, sv1, 1, Precision::Confusion());
							gp_Dir p1ngp = props.Normal();
							glm::vec3 p1n(p1ngp.X(), p1ngp.Y(), p1ngp.Z());
							float p1s = glm::dot(glm::mat3(camera.matrices.view) * p1n, glm::vec3(0, 0, 1));

							props = GeomLProp_SLProps(bsplineSurf, su2, sv1, 1, Precision::Confusion());
							gp_Dir p2ngp = props.Normal();
							glm::vec3 p2n(p2ngp.X(), p2ngp.Y(), p2ngp.Z());
							float p2s = glm::dot(glm::mat3(camera.matrices.view) * p2n, glm::vec3(0, 0, 1));

							props = GeomLProp_SLProps(bsplineSurf, su1, sv2, 1, Precision::Confusion());
							gp_Dir p3ngp = props.Normal();
							glm::vec3 p3n(p3ngp.X(), p3ngp.Y(), p3ngp.Z());
							float p3s = glm::dot(glm::mat3(camera.matrices.view) * p3n, glm::vec3(0, 0, 1));

							props = GeomLProp_SLProps(bsplineSurf, su2, sv2, 1, Precision::Confusion());
							gp_Dir p4ngp = props.Normal();
							glm::vec3 p4n(p4ngp.X(), p4ngp.Y(), p4ngp.Z());
							float p4s = glm::dot(glm::mat3(camera.matrices.view) * p4n, glm::vec3(0, 0, 1));

							silhouetteNorsArray[i][j] = p1s;
							silhouetteNorsArray[i + 1][j] = p2s;
							silhouetteNorsArray[i][j + 1] = p3s;
							silhouetteNorsArray[i + 1][j + 1] = p4s;
						}

					}
					for (int i = 0; i < divide; ++i) {
						Standard_Real su1 = u1 + double(i) * uStep;
						Standard_Real su2 = u1 + double(i + 1) * uStep;
						for (int j = 0; j < divide; ++j) {
							Standard_Real sv1 = v1 + double(j) * vStep;
							Standard_Real sv2 = v1 + double(j + 1) * vStep;
							glm::vec3 p1 = silhouettePointsArray[i][j];
							glm::vec3 p2 = silhouettePointsArray[i + 1][j];
							glm::vec3 p3 = silhouettePointsArray[i][j + 1];
							glm::vec3 p4 = silhouettePointsArray[i + 1][j + 1];
							float n1 = silhouetteNorsArray[i][j];
							float n2 = silhouetteNorsArray[i + 1][j];
							float n3 = silhouetteNorsArray[i][j + 1];
							float n4 = silhouetteNorsArray[i + 1][j + 1];
							glm::dvec2 p1d(su1, sv1);
							glm::dvec2 p2d(su2, sv1);
							glm::dvec2 p3d(su1, sv2);
							glm::dvec2 p4d(su2, sv2);
							/*bool b1 = silhouetteBoolsArray[i][j];
							bool b2 = silhouetteBoolsArray[i + 1][j];
							bool b3 = silhouetteBoolsArray[i][j + 1];
							bool b4 = silhouetteBoolsArray[i + 1][j + 1];
							if (!b1 || !b2 || !b3 || !b4) {
								continue;
							}*/
							//tri 1
							if ((n1 == 0.f) && (n2 == 0.f)) {
								silhouettePoints.push_back(p1);
								silhouettePoints.push_back(p2);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 1);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								if (j == 0) {
									silhouetteOri.push_back(1);
									std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
									silhouetteBoundings.push_back(boundings);
								}
								else {
									glm::dvec2 ptd(su2, sv1 - vStep);
									std::vector<glm::dvec2> l1 = { p2d, p1d, ptd };
									std::vector<glm::dvec2> l2 = { p1d, p2d, p3d };
									silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
									std::vector<glm::dvec2> boundings = { p1d, ptd, p2d, p3d };
									silhouetteBoundings.push_back(boundings);
								}
							}
							if ((n2 == 0.f) && (n3 == 0.f)) {
								silhouettePoints.push_back(p2);
								silhouettePoints.push_back(p3);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								std::vector<glm::dvec2> l1 = { p2d, p3d, p1d };
								std::vector<glm::dvec2> l2 = { p3d, p2d, p4d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							if ((n1 == 0.f) && (n3 == 0.f)) {
								silhouettePoints.push_back(p1);
								silhouettePoints.push_back(p3);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 1);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								if (i == 0) {
									silhouetteOri.push_back(1);
									std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
									silhouetteBoundings.push_back(boundings);
								}
								else {
									glm::dvec2 ptd(su1 - uStep, sv2);
									std::vector<glm::dvec2> l1 = { p1d, p3d, ptd };
									std::vector<glm::dvec2> l2 = { p3d, p1d, p2d };
									silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
									std::vector<glm::dvec2> boundings = { p1d, p2d, p3d, ptd };
									silhouetteBoundings.push_back(boundings);
								}
							}
							//0zf
							if ((n1 == 0.f) && (n2 > 0.f) && (n3 < 0.f)) {
								double t23 = n2 / (n2 - n3);
								silhouettePoints.push_back(p1);
								silhouettePoints.push_back(glm::mix(p2, p3, t23));
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 1);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 3, t23);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p23d = glm::mix(p2d, p3d, t23);
								std::vector<glm::dvec2> l1 = { p23d, p1d, p2d };
								std::vector<glm::dvec2> l2 = { p1d, p23d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//0fz
							if ((n1 == 0.f) && (n2 < 0.f) && (n3 > 0.f)) {
								double t32 = n3 / (n3 - n2);
								silhouettePoints.push_back(p1);
								silhouettePoints.push_back(glm::mix(p3, p2, t32));
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 1);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 2, t32);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p32d = glm::mix(p3d, p2d, t32);
								std::vector<glm::dvec2> l1 = { p32d, p1d, p2d };
								std::vector<glm::dvec2> l2 = { p1d, p32d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//z0f
							if ((n1 > 0.f) && (n2 == 0.f) && (n3 < 0.f)) {
								float t13 = n1 / (n1 - n3);
								silhouettePoints.push_back(glm::mix(p1, p3, t13));
								silhouettePoints.push_back(p2);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 1, 3, t13);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p13d = glm::mix(p1d, p3d, t13);
								std::vector<glm::dvec2> l1 = { p13d, p2d, p3d };
								std::vector<glm::dvec2> l2 = { p2d, p13d, p1d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//f0z
							if ((n1 < 0.f) && (n2 == 0.f) && (n3 > 0.f)) {
								float t31 = n3 / (n3 - n1);
								silhouettePoints.push_back(glm::mix(p3, p1, t31));
								silhouettePoints.push_back(p2);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 1, t31);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p31d = glm::mix(p3d, p1d, t31);
								std::vector<glm::dvec2> l1 = { p31d, p2d, p3d };
								std::vector<glm::dvec2> l2 = { p2d, p31d, p1d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zf0
							if ((n1 > 0.f) && (n2 < 0.f) && (n3 == 0.f)) {
								float t12 = n1 / (n1 - n2);
								silhouettePoints.push_back(p3);
								silhouettePoints.push_back(glm::mix(p1, p2, t12));
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 1, 2, t12);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p12d = glm::mix(p1d, p2d, t12);
								std::vector<glm::dvec2> l1 = { p12d, p3d, p1d };
								std::vector<glm::dvec2> l2 = { p3d, p12d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//fz0
							if ((n1 < 0.f) && (n2 > 0.f) && (n3 == 0.f)) {
								float t21 = n2 / (n2 - n1);
								silhouettePoints.push_back(p3);
								silhouettePoints.push_back(glm::mix(p2, p1, t21));
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 1, t21);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p21d = glm::mix(p2d, p1d, t21);
								std::vector<glm::dvec2> l1 = { p21d, p3d, p1d };
								std::vector<glm::dvec2> l2 = { p3d, p21d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zff
							if ((n1 > 0.f) && (n2 < 0.f) && (n3 < 0.f)) {
								float t12 = n1 / (n1 - n2);
								float t13 = n1 / (n1 - n3);
								silhouettePoints.push_back(glm::mix(p1, p2, t12));
								silhouettePoints.push_back(glm::mix(p1, p3, t13));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 1, 2, t12);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 1, 3, t13);
								
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p12d = glm::mix(p1d, p2d, t12);
								glm::dvec2 p13d = glm::mix(p1d, p3d, t13);
								std::vector<glm::dvec2> l1 = { p12d, p13d, p1d };
								std::vector<glm::dvec2> l2 = { p13d, p12d, p2d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//fzf
							if ((n1 < 0.f) && (n2 > 0.f) && (n3 < 0.f)) {
								float t21 = n2 / (n2 - n1);
								float t23 = n2 / (n2 - n3);
								silhouettePoints.push_back(glm::mix(p2, p1, t21));
								silhouettePoints.push_back(glm::mix(p2, p3, t23));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 1, t21);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 3, t23);
								
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p21d = glm::mix(p2d, p1d, t21);
								glm::dvec2 p23d = glm::mix(p2d, p3d, t23);
								std::vector<glm::dvec2> l1 = { p23d, p21d, p2d };
								std::vector<glm::dvec2> l2 = { p21d, p23d, p3d, p1d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//ffz
							if ((n1 < 0.f) && (n2 < 0.f) && (n3 > 0.f)) {
								float t31 = n3 / (n3 - n1);
								float t32 = n3 / (n3 - n2);
								silhouettePoints.push_back(glm::mix(p3, p1, t31));
								silhouettePoints.push_back(glm::mix(p3, p2, t32));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 1, t31);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 2, t32);
								
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p31d = glm::mix(p3d, p1d, t31);
								glm::dvec2 p32d = glm::mix(p3d, p2d, t32);
								std::vector<glm::dvec2> l1 = { p31d, p32d, p3d };
								std::vector<glm::dvec2> l2 = { p32d, p31d, p1d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zzf
							if ((n1 > 0.f) && (n2 > 0.f) && (n3 < 0.f)) {
								float t13 = n1 / (n1 - n3);
								float t23 = n2 / (n2 - n3);
								silhouettePoints.push_back(glm::mix(p1, p3, t13));
								silhouettePoints.push_back(glm::mix(p2, p3, t23));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 1, 3, t13);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 3, t23);
								
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p13d = glm::mix(p1d, p3d, t13);
								glm::dvec2 p23d = glm::mix(p2d, p3d, t23);
								std::vector<glm::dvec2> l1 = { p13d, p23d, p3d };
								std::vector<glm::dvec2> l2 = { p23d, p13d, p1d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zfz
							if ((n1 > 0.f) && (n2 < 0.f) && (n3 > 0.f)) {
								float t12 = n1 / (n1 - n2);
								float t32 = n3 / (n3 - n2);
								silhouettePoints.push_back(glm::mix(p1, p2, t12));
								silhouettePoints.push_back(glm::mix(p3, p2, t32));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 1, 2, t12);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 2, t32);
								
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p12d = glm::mix(p1d, p2d, t12);
								glm::dvec2 p32d = glm::mix(p3d, p2d, t32);
								std::vector<glm::dvec2> l1 = { p32d, p12d, p2d };
								std::vector<glm::dvec2> l2 = { p12d, p32d, p3d, p1d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//fzz
							if ((n1 < 0.f) && (n2 > 0.f) && (n3 > 0.f)) {
								float t21 = n2 / (n2 - n1);
								float t31 = n3 / (n3 - n1);
								silhouettePoints.push_back(glm::mix(p2, p1, t21));
								silhouettePoints.push_back(glm::mix(p3, p1, t31));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 1, t21);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 1, t31);
								
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p21d = glm::mix(p2d, p1d, t21);
								glm::dvec2 p31d = glm::mix(p3d, p1d, t31);
								std::vector<glm::dvec2> l1 = { p21d, p31d, p1d };
								std::vector<glm::dvec2> l2 = { p31d, p21d, p2d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//tri 2
							if ((n2 == 0.f) && (n3 == 0.f)) {
								silhouettePoints.push_back(p2);
								silhouettePoints.push_back(p3);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								
								std::vector<glm::dvec2> boundings = { p1d, p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								std::vector<glm::dvec2> l1 = { p3d, p2d, p4d };
								std::vector<glm::dvec2> l2 = { p2d, p3d, p1d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							if ((n3 == 0.f) && (n4 == 0.f)) {
								silhouettePoints.push_back(p3);
								silhouettePoints.push_back(p4);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 4);
								
								if (j == (divide - 1)) {
									silhouetteOri.push_back(1);
									std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
									silhouetteBoundings.push_back(boundings);
								}
								else {
									glm::dvec2 ptd(su1, sv2 + vStep);
									std::vector<glm::dvec2> l1 = { p3d, p4d, ptd };
									std::vector<glm::dvec2> l2 = { p4d, p3d, p2d };
									silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
									std::vector<glm::dvec2> boundings = { p2d, p4d, ptd, p3d };
									silhouetteBoundings.push_back(boundings);
								}
							}
							if ((n4 == 0.f) && (n2 == 0.f)) {
								silhouettePoints.push_back(p2);
								silhouettePoints.push_back(p4);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 4);
								
								if (i == (divide - 1)) {
									silhouetteOri.push_back(1);
									std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
									silhouetteBoundings.push_back(boundings);
								}
								else {
									glm::dvec2 ptd(su2 + uStep, sv1);
									std::vector<glm::dvec2> l1 = { p4d, p2d, ptd };
									std::vector<glm::dvec2> l2 = { p2d, p4d, p3d };
									silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
									std::vector<glm::dvec2> boundings = { p2d, ptd, p4d, p3d };
									silhouetteBoundings.push_back(boundings);
								}
							}
							//0zf
							if ((n2 == 0.f) && (n3 > 0.f) && (n4 < 0.f)) {
								float t34 = n3 / (n3 - n4);
								silhouettePoints.push_back(glm::mix(p3, p4, t34));
								silhouettePoints.push_back(p2);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 4, t34);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p34d = glm::mix(p3d, p4d, t34);
								std::vector<glm::dvec2> l1 = { p34d, p2d, p4d };
								std::vector<glm::dvec2> l2 = { p2d, p34d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//0fz
							if ((n2 == 0.f) && (n3 < 0.f) && (n4 > 0.f)) {
								float t43 = n4 / (n4 - n3);
								silhouettePoints.push_back(glm::mix(p4, p3, t43));
								silhouettePoints.push_back(p2);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 4, 3, t43);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p43d = glm::mix(p4d, p3d, t43);
								std::vector<glm::dvec2> l1 = { p43d, p2d, p4d };
								std::vector<glm::dvec2> l2 = { p2d, p43d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//z0f
							if ((n2 > 0.f) && (n3 == 0.f) && (n4 < 0.f)) {
								float t24 = n2 / (n2 - n4);
								silhouettePoints.push_back(glm::mix(p2, p4, t24));
								silhouettePoints.push_back(p3);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 4, t24);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p24d = glm::mix(p2d, p4d, t24);
								std::vector<glm::dvec2> l1 = { p24d, p3d, p2d };
								std::vector<glm::dvec2> l2 = { p3d, p24d, p4d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//f0z
							if ((n2 < 0.f) && (n3 == 0.f) && (n4 > 0.f)) {
								float t42 = n4 / (n4 - n2);
								silhouettePoints.push_back(glm::mix(p4, p2, t42));
								silhouettePoints.push_back(p3);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 4, 2, t42);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p42d = glm::mix(p4d, p2d, t42);
								std::vector<glm::dvec2> l1 = { p42d, p3d, p2d };
								std::vector<glm::dvec2> l2 = { p3d, p42d, p4d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zf0
							if ((n2 > 0.f) && (n3 < 0.f) && (n4 == 0.f)) {
								float t23 = n2 / (n2 - n3);
								silhouettePoints.push_back(p4);
								silhouettePoints.push_back(glm::mix(p2, p3, t23));
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 4);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 3, t23);
								
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p23d = glm::mix(p2d, p3d, t23);
								std::vector<glm::dvec2> l1 = { p23d, p4d, p3d };
								std::vector<glm::dvec2> l2 = { p4d, p23d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//fz0
							if ((n2 < 0.f) && (n3 > 0.f) && (n4 == 0.f)) {
								float t32 = n3 / (n3 - n2);
								silhouettePoints.push_back(p4);
								silhouettePoints.push_back(glm::mix(p3, p2, t32));
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 4);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 2, t32);
								
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p32d = glm::mix(p3d, p2d, t32);
								std::vector<glm::dvec2> l1 = { p32d, p4d, p3d };
								std::vector<glm::dvec2> l2 = { p4d, p32d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zff
							if ((n2 > 0.f) && (n3 < 0.f) && (n4 < 0.f)) {
								float t23 = n2 / (n2 - n3);
								float t24 = n2 / (n2 - n4);
								silhouettePoints.push_back(glm::mix(p2, p3, t23));
								silhouettePoints.push_back(glm::mix(p2, p4, t24));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 3, t23);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 4, t24);
								
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p23d = glm::mix(p2d, p3d, t23);
								glm::dvec2 p24d = glm::mix(p2d, p4d, t24);
								std::vector<glm::dvec2> l1 = { p24d, p23d, p2d };
								std::vector<glm::dvec2> l2 = { p23d, p24d, p4d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//fzf
							if ((n2 < 0.f) && (n3 > 0.f) && (n4 < 0.f)) {
								float t32 = n3 / (n3 - n2);
								float t34 = n3 / (n3 - n4);
								silhouettePoints.push_back(glm::mix(p3, p2, t32));
								silhouettePoints.push_back(glm::mix(p3, p4, t34));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 2, t32);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 4, t34);
								
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p32d = glm::mix(p3d, p2d, t32);
								glm::dvec2 p34d = glm::mix(p3d, p4d, t34);
								std::vector<glm::dvec2> l1 = { p32d, p34d, p3d };
								std::vector<glm::dvec2> l2 = { p34d, p32d, p2d, p4d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//ffz
							if ((n2 < 0.f) && (n3 < 0.f) && (n4 > 0.f)) {
								float t42 = n4 / (n4 - n2);
								float t43 = n4 / (n4 - n3);
								silhouettePoints.push_back(glm::mix(p4, p2, t42));
								silhouettePoints.push_back(glm::mix(p4, p3, t43));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 4, 2, t42);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 4, 3, t43);
								
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p43d = glm::mix(p4d, p3d, t43);
								glm::dvec2 p42d = glm::mix(p4d, p2d, t42);
								std::vector<glm::dvec2> l1 = { p43d, p42d, p4d };
								std::vector<glm::dvec2> l2 = { p42d, p43d, p3d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zzf
							if ((n2 > 0.f) && (n3 > 0.f) && (n4 < 0.f)) {
								float t24 = n2 / (n2 - n4);
								float t34 = n3 / (n3 - n4);
								silhouettePoints.push_back(glm::mix(p2, p4, t24));
								silhouettePoints.push_back(glm::mix(p3, p4, t34));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 4, t24);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 4, t34);
								
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p24d = glm::mix(p2d, p4d, t24);
								glm::dvec2 p34d = glm::mix(p3d, p4d, t34);
								std::vector<glm::dvec2> l1 = { p34d, p24d, p4d };
								std::vector<glm::dvec2> l2 = { p24d, p34d, p3d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zfz
							if ((n2 > 0.f) && (n3 < 0.f) && (n4 > 0.f)) {
								float t23 = n2 / (n2 - n3);
								float t43 = n4 / (n4 - n3);
								silhouettePoints.push_back(glm::mix(p2, p3, t23));
								silhouettePoints.push_back(glm::mix(p4, p3, t43));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 3, t23);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 4, 3, t43);
								
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p23d = glm::mix(p2d, p3d, t23);
								glm::dvec2 p43d = glm::mix(p4d, p3d, t43);
								std::vector<glm::dvec2> l1 = { p23d, p43d, p3d };
								std::vector<glm::dvec2> l2 = { p43d, p23d, p2d, p4d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//fzz
							if ((n2 < 0.f) && (n3 > 0.f) && (n4 > 0.f)) {
								float t32 = n3 / (n3 - n2);
								float t42 = n4 / (n4 - n2);
								silhouettePoints.push_back(glm::mix(p3, p2, t32));
								silhouettePoints.push_back(glm::mix(p4, p2, t42));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 2, t32);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 4, 2, t42);
								
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p32d = glm::mix(p3d, p2d, t32);
								glm::dvec2 p42d = glm::mix(p4d, p2d, t42);
								std::vector<glm::dvec2> l1 = { p42d, p32d, p2d };
								std::vector<glm::dvec2> l2 = { p32d, p42d, p4d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
						}

						/*for (int i = 1; i < divide - 1; ++i) {
							for (int j = 1; j < divide - 1; ++j) {
								glm::vec3 p1 = silhouettePointsArray[i][j];
								glm::vec3 p2 = silhouettePointsArray[i + 1][j];
								glm::vec3 p3 = silhouettePointsArray[i][j + 1];
								glm::vec3 p4 = silhouettePointsArray[i + 1][j + 1];
								silhouettePointsDebug.push_back(p1);
								silhouettePointsDebug.push_back(p2);
								silhouettePointsDebug.push_back(p2);
								silhouettePointsDebug.push_back(p3);
								silhouettePointsDebug.push_back(p3);
								silhouettePointsDebug.push_back(p1);
								silhouettePointsDebug.push_back(p3);
								silhouettePointsDebug.push_back(p4);
								silhouettePointsDebug.push_back(p4);
								silhouettePointsDebug.push_back(p2);
							}
						}*/
					}
					silhouette2dPntInside.clear();
					for (int i = 0; i < silhouette2dPnt.size(); i += 2) {
						double su1 = std::get<0>(silhouette2dPnt[i]);
						double su2 = std::get<0>(silhouette2dPnt[i + 1]);
						double sv1 = std::get<1>(silhouette2dPnt[i]);
						double sv2 = std::get<1>(silhouette2dPnt[i + 1]);
						gp_Pnt2d P1(su1, sv1), P2(su2, sv2);
						Handle(Geom2d_Line) sline = new Geom2d_Line(P1, gp_Dir2d(P2.XY() - P1.XY()));
						Standard_Real len1 = 0.0, len2 = P1.Distance(P2);
						Handle(Geom2d_TrimmedCurve) ssegment =
							new Geom2d_TrimmedCurve(sline, len1, len2, Standard_True, Standard_True);
						std::vector<std::tuple<Standard_Real, glm::dvec2>> inters;
						int nullInters = 0;
						int wireId = 0;
						for (auto& outwire : outwireCurves2d) {
							int edgeId = 0;
							for (auto& outedge : outwire) {
								gp_Pnt2d prevPt;
								Standard_Real prevt;
								Geom2dAPI_InterCurveCurve intersector(sline, outedge, 1.0e-4);
								const Geom2dInt_GInter& ginter = intersector.Intersector();
								if (intersector.NbPoints() > 0) {
									int interCnt = intersector.NbPoints();
									for (Standard_Integer i = 1; i <= intersector.NbPoints(); ++i) {
										gp_Pnt2d pt = intersector.Point(i);
										const IntRes2d_IntersectionPoint& ip = ginter.Point(i);
										Standard_Real t1 = ip.ParamOnFirst();
										Standard_Real t2 = ip.ParamOnSecond();
										//only compare with previous
										if (i != 1) {
											bool b1 = fequal(t1, prevt);
											bool b2 = fequal(pt.X(), prevPt.X());
											bool b3 = fequal(pt.Y(), prevPt.Y());
											if (b1 && b2 && b3) {
												continue;
											}
										}
										if ((t1 >= ssegment->FirstParameter()) && (t1 <= ssegment->LastParameter())) {
											gp_Dir2d outedgeTan;
											Geom2dLProp_CLProps2d props(outedge, t2, 1, 0.1);
											props.Tangent(outedgeTan);
											glm::dvec2 outedgeT(outedgeTan.X(), outedgeTan.Y());
											outedgeT = glm::normalize(outedgeT);
											inters.size();
											inters.emplace_back(t1, outedgeT);
										}
										else if (t1 > ssegment->LastParameter()) {
											++nullInters;
										}
										prevPt = pt;
										prevt = t1;
										//debug
										/*gp_Pnt2d test2d1 = ssegment->Value(t1);
										gp_Pnt2d test2d = outedge->Value(t2);
										gp_Vec2d tangentDir(outedgeT.X() * 0.0005, outedgeT.Y() * 0.0005);
										gp_Pnt2d testt = test2d.Translated(tangentDir);
										gp_Pnt test3dpnt1 = bsplineSurf->Value(test2d.X(), test2d.Y());
										gp_Pnt test3dpnt2 = bsplineSurf->Value(testt.X(), testt.Y());
										debugLines.push_back(glm::vec3(test3dpnt1.X(), test3dpnt1.Y(), test3dpnt1.Z()));
										debugLines.push_back(glm::vec3(test3dpnt2.X(), test3dpnt2.Y(), test3dpnt2.Z()));*/
									}
								}
								++edgeId;
							}
							++wireId;
						}
						if (inters.size() != 0) {
							//有交一定在内
							double para1 = ssegment->FirstParameter();
							double para2 = ssegment->LastParameter();
							glm::dvec3 sdir(su2 - su1, sv2 - sv1, 0);
							sdir = glm::normalize(sdir);
							for (auto& inter : inters) {
								double t = std::get<0>(inter);
								glm::dvec2 tant = std::get<1>(inter);
								glm::dvec3 tan(tant.x, tant.y, 0);
								double result = glm::cross(sdir, tan).z;
								if (result < 0) {
									if (t < para1) {
										continue;
									}
									gp_Pnt2d val = sline->Value(t);
									su1 = val.X();
									sv1 = val.Y();
									para1 = t;
								}
								else {
									if (t > para2) {
										continue;
									}
									gp_Pnt2d val = sline->Value(t);
									su2 = val.X();
									sv2 = val.Y();
									para2 = t;

								}
							}
							silhouette2dPntInside.emplace_back(su1, sv1);
							silhouette2dPntInside.emplace_back(su2, sv2);

						}
						else {
							if (nullInters % 2 == 0) {
								//外

							}
							else {
								//内
								silhouette2dPntInside.emplace_back(su1, sv1);
								silhouette2dPntInside.emplace_back(su2, sv2);
							}
						}
					}
					silhouettePoints.clear();
					for (auto& uv : silhouette2dPntInside) {

						gp_Pnt p = bsplineSurf->Value(std::get<0>(uv), std::get<1>(uv));
						silhouettePoints.push_back(glm::vec3(p.X(), p.Y(), p.Z()));
					}
				}
			}
			occPoints.clear();
			// 1. 构建投射器
			glm::vec3 cameraDir = glm::inverse(glm::mat3(camera.matrices.view)) * glm::vec3(0, 0, 1);
			glm::vec4 tmp = glm::inverse(glm::mat4(camera.matrices.view))[3];
			glm::vec3 cameraPos = glm::vec3(tmp);
			gp_Ax2 ax2(gp_Pnt(cameraPos.x, cameraPos.y, cameraPos.z), gp_Dir(cameraPos.x + cameraDir.x, cameraPos.y + cameraDir.y, cameraPos.z + cameraDir.z));
			HLRAlgo_Projector projector(ax2); // 纯正交

			// 2. HLR 算法设置
			Handle(HLRBRep_Algo) algo = new HLRBRep_Algo();
			Standard_Integer nbIso = 2;
			algo->Add(shape, nbIso);
			algo->Projector(projector);
			algo->Update();
			algo->Hide();
			// 3. 提取结果
			HLRBRep_HLRToShape extractor(algo);
			//silhouette
			TopoDS_Shape proj2d_comp = extractor.OutLineVCompound();
			addOCCHLResults(proj2d_comp);

			//边界边
			proj2d_comp = extractor.VCompound();
			addOCCHLResults(proj2d_comp);

			//等参数线
			proj2d_comp = extractor.IsoLineVCompound();
			addOCCHLResults(proj2d_comp);
		}

		void addOCCHLResults(TopoDS_Shape& proj2d_comp) {
			int sampleNum = 40;
			// 4. 遍历 compound 中的每个 edge
			for (TopExp_Explorer exp(proj2d_comp, TopAbs_EDGE); exp.More(); exp.Next()) {
				TopoDS_Edge e2d = TopoDS::Edge(exp.Current());
				BRepAdaptor_Curve curveAdaptor(e2d);
				double u1 = curveAdaptor.FirstParameter();
				double u2 = curveAdaptor.LastParameter();
				gp_Pnt P;
				double step = (u2 - u1) / sampleNum;
				std::vector<glm::vec3> tempVec;
				for (int i = 0; i <= sampleNum; ++i) {
					double u = u1 + i * step;
					P = curveAdaptor.Value(u);
					gp_Pnt test;
					curveAdaptor.D0(u, test);
					//将3d点转换为屏幕坐标
					glm::vec3 screenP = glm::vec3(P.X(), P.Y(), 10);
					tempVec.push_back(screenP);
				}
				occPoints.push_back(tempVec);
			}
		}
	};

	struct PushValue {
		int max_neighbor_cnt = MAX_NEIGHBOR_FACE_COUNT;
		float screenHalfLengthX;
		float screenHalfLengthY;

	};

	struct PushValue2 {
		float color = 0.2;
	};

	Mesh mesh;
	TopoDS_Shape shape;
	PushValue pushVal{};
	PushValue2 pushVal2{};
	//
	//self-added
	float fov = 60.f;
	float oldFov = fov;
	/*float pixelLengthY = (glm::tan(glm::radians(fov / 2.f)) / (height / 2.f));
	float pixelLengthX = pixelLengthY * ((float)width / (float)height);*/
	float halfScreenHeightY = glm::tan(glm::radians(fov / 2.f));
	float halfScreenHeightX = halfScreenHeightY * ((float)width / (float)height);
	int32_t debugDisplayTarget = 0;
	int singleStride = 1;
	float depthFactor = 1.f;
	int uFactor = 1;
	int vFactor = 1;
	float triColor = 0.2f;
	bool lockedView = false;
	bool prevLockedView = false;
	

	struct {
		struct {
			vks::Texture2D colorMap;
			vks::Texture2D normalMap;
		} model;
		struct {
			vks::Texture2D colorMap;
			vks::Texture2D normalMap;
		} floor;
	} textures;

	struct {
		vkglTF::Model model;
		vkglTF::Model floor;
	} models;

	struct UniformDataOffscreen {
		glm::mat4 projection;
		glm::mat4 model;
		glm::mat4 view;
		glm::vec4 instancePos[3];
	} uniformDataOffscreen;

	struct Light {
		glm::vec4 position;
		glm::vec3 color;
		float radius;
	};

	struct UniformDataComposition {
		Light lights[6];
		glm::vec4 viewPos;
		glm::mat4 camView;
		glm::mat4 camViewTr;
		int debugDisplayTarget = 0;
		int singleStride = 1;
		float depthFactor = 1.f;
		int uFactor = 1;
		int vFactor = 1;
		int orthographic = false;
	} uniformDataComposition;

	struct UniformDataEdge {
		glm::mat4 projection;
		glm::mat4 model;
		glm::mat4 view;
		//using bool here need to alignas
		int orthographic = false;
	} uniformDataEdge;

	struct UniformDataLockedEdge {
		glm::mat4 projection;
		glm::mat4 model;
		glm::mat4 view;
	} uniformDataLockedEdge;

	struct UniformDataHiddenLine {
		glm::mat4 projection;
		glm::mat4 model;
		glm::mat4 view;
	} uniformDataHiddenLine;

	struct UniformDataEdgeFrag {
		float color = 0.2f;
	} uniformDataEdgeFrag;

	struct UniformDataLockedEdgeFrag {
		float color = 0.2f;
	} uniformDataLockedEdgeFrag;

	struct {
		vks::Buffer offscreen{ VK_NULL_HANDLE };
		vks::Buffer composition{ VK_NULL_HANDLE };
		vks::Buffer edge{ VK_NULL_HANDLE };
		vks::Buffer lockedEdge{ VK_NULL_HANDLE };
		vks::Buffer edgeFrag{ VK_NULL_HANDLE };
		vks::Buffer lockedEdgeFrag{ VK_NULL_HANDLE };
		vks::Buffer hiddenLine{ VK_NULL_HANDLE };
	} uniformBuffers;


	struct {
		VkPipeline offscreen{ VK_NULL_HANDLE };
		VkPipeline composition{ VK_NULL_HANDLE };
		VkPipeline edge{ VK_NULL_HANDLE };
		VkPipeline lockedEdge{ VK_NULL_HANDLE };
		VkPipeline hiddenLine{ VK_NULL_HANDLE };
	} pipelines;
	VkPipelineLayout pipelineLayout{ VK_NULL_HANDLE };
	VkPipelineLayout pipelineLayoutedge{ VK_NULL_HANDLE };

	struct {
		VkDescriptorSet model{ VK_NULL_HANDLE };
		VkDescriptorSet floor{ VK_NULL_HANDLE };
		VkDescriptorSet composition{ VK_NULL_HANDLE };
		VkDescriptorSet edge{ VK_NULL_HANDLE };
		VkDescriptorSet lockedEdge{ VK_NULL_HANDLE };
		VkDescriptorSet hiddenLine{ VK_NULL_HANDLE };
	} descriptorSets;

	VkDescriptorSetLayout descriptorSetLayout{ VK_NULL_HANDLE };

	// Framebuffers holding the deferred attachments
	struct FrameBufferAttachment {
		VkImage image;
		VkDeviceMemory mem;
		VkImageView view;
		VkFormat format;
	};
	struct FrameBuffer {
		int32_t width, height;
		VkFramebuffer frameBuffer;
		// One attachment for every component required for a deferred rendering setup
		FrameBufferAttachment position, normal, albedo;
		FrameBufferAttachment depth;
		VkRenderPass renderPass;
	} offScreenFrameBuf{};

	FrameBuffer edgeFrameBuf{};
	FrameBuffer lockedEdgeFrameBuf{};
	FrameBuffer hiddenLineFrameBuf{};


	// One sampler for the frame buffer color attachments
	VkSampler colorSampler{ VK_NULL_HANDLE };

	VkCommandBuffer offScreenCmdBuffer{ VK_NULL_HANDLE };
	VkCommandBuffer edgeCmdBuffer{ VK_NULL_HANDLE };
	VkCommandBuffer lockedEdgeCmdBuffer{ VK_NULL_HANDLE };
	VkCommandBuffer hiddenLineCmdBuffer{ VK_NULL_HANDLE };

	// Semaphore used to synchronize between offscreen and final scene rendering
	VkSemaphore offscreenSemaphore{ VK_NULL_HANDLE };
	VkSemaphore edgeSemaphore{ VK_NULL_HANDLE };
	VkSemaphore lockedEdgeSemaphore{ VK_NULL_HANDLE };
	VkSemaphore hiddenLineSemaphore{ VK_NULL_HANDLE };

	VulkanExample() : VulkanExampleBase()
	{
		title = "OCC";
		camera.type = Camera::CameraType::firstperson;
		camera.movementSpeed = 5.0f;
#ifndef __ANDROID__
		camera.rotationSpeed = 0.25f;
#endif
		/*camera.position = { 2.15f, 0.3f, -8.75f };
		camera.setRotation(glm::vec3(-0.75f, 12.5f, 0.0f));
		camera.setPerspective(60.0f, (float)width / (float)height, 0.1f, 256.0f);*/
		camera.movementSpeed = 3.f;
		camera.position = { 0.f, 0.f, 20.f };
		//perspective camera parameter
		camera.fov = 60.f;
		camera.znear = 0.01f;
		camera.zfar = 256.0f;
		camera.aspect = (float)width / (float)height;
		//orthographic camera parameter
		camera.orthoLeft = -4.f;
		camera.orthoRight = 4.f;
		camera.orthoBottom = camera.orthoLeft * height / width;
		camera.orthoTop = camera.orthoRight * height / width;
		camera.orthographic = false;
		camera.setRotation(glm::vec3(-0.f, 0.f, 0.0f));
		camera.setPerspective(60.f, (float)width / (float)height, 0.01f, 256.0f);
	}

	~VulkanExample()
	{
		if (device) {
			vkDestroySampler(device, colorSampler, nullptr);

			// Frame buffer

			// Color attachments
			vkDestroyImageView(device, offScreenFrameBuf.position.view, nullptr);
			vkDestroyImage(device, offScreenFrameBuf.position.image, nullptr);
			vkFreeMemory(device, offScreenFrameBuf.position.mem, nullptr);
			vkDestroyImageView(device, edgeFrameBuf.position.view, nullptr);
			vkDestroyImage(device, edgeFrameBuf.position.image, nullptr);
			vkFreeMemory(device, edgeFrameBuf.position.mem, nullptr);
			vkDestroyImageView(device, lockedEdgeFrameBuf.position.view, nullptr);
			vkDestroyImage(device, lockedEdgeFrameBuf.position.image, nullptr);
			vkFreeMemory(device, lockedEdgeFrameBuf.position.mem, nullptr);
			vkDestroyImageView(device, hiddenLineFrameBuf.position.view, nullptr);
			vkDestroyImage(device, hiddenLineFrameBuf.position.image, nullptr);
			vkFreeMemory(device, hiddenLineFrameBuf.position.mem, nullptr);

			vkDestroyImageView(device, offScreenFrameBuf.normal.view, nullptr);
			vkDestroyImage(device, offScreenFrameBuf.normal.image, nullptr);
			vkFreeMemory(device, offScreenFrameBuf.normal.mem, nullptr);
			vkDestroyImageView(device, edgeFrameBuf.normal.view, nullptr);
			vkDestroyImage(device, edgeFrameBuf.normal.image, nullptr);
			vkFreeMemory(device, edgeFrameBuf.normal.mem, nullptr);
			vkDestroyImageView(device, lockedEdgeFrameBuf.normal.view, nullptr);
			vkDestroyImage(device, lockedEdgeFrameBuf.normal.image, nullptr);
			vkFreeMemory(device, lockedEdgeFrameBuf.normal.mem, nullptr);

			vkDestroyImageView(device, offScreenFrameBuf.albedo.view, nullptr);
			vkDestroyImage(device, offScreenFrameBuf.albedo.image, nullptr);
			vkFreeMemory(device, offScreenFrameBuf.albedo.mem, nullptr);
			vkDestroyImageView(device, edgeFrameBuf.albedo.view, nullptr);
			vkDestroyImage(device, edgeFrameBuf.albedo.image, nullptr);
			vkFreeMemory(device, edgeFrameBuf.albedo.mem, nullptr);

			// Depth attachment
			vkDestroyImageView(device, offScreenFrameBuf.depth.view, nullptr);
			vkDestroyImage(device, offScreenFrameBuf.depth.image, nullptr);
			vkFreeMemory(device, offScreenFrameBuf.depth.mem, nullptr);
			vkDestroyImageView(device, edgeFrameBuf.depth.view, nullptr);
			vkDestroyImage(device, edgeFrameBuf.depth.image, nullptr);
			vkFreeMemory(device, edgeFrameBuf.depth.mem, nullptr);
			vkDestroyImageView(device, lockedEdgeFrameBuf.depth.view, nullptr);
			vkDestroyImage(device, lockedEdgeFrameBuf.depth.image, nullptr);
			vkFreeMemory(device, lockedEdgeFrameBuf.depth.mem, nullptr);
			vkDestroyImageView(device, hiddenLineFrameBuf.depth.view, nullptr);
			vkDestroyImage(device, hiddenLineFrameBuf.depth.image, nullptr);
			vkFreeMemory(device, hiddenLineFrameBuf.depth.mem, nullptr);

			vkDestroyFramebuffer(device, offScreenFrameBuf.frameBuffer, nullptr);
			vkDestroyFramebuffer(device, edgeFrameBuf.frameBuffer, nullptr);
			vkDestroyFramebuffer(device, hiddenLineFrameBuf.frameBuffer, nullptr);
			vkDestroyFramebuffer(device, lockedEdgeFrameBuf.frameBuffer, nullptr);

			vkDestroyPipeline(device, pipelines.composition, nullptr);
			vkDestroyPipeline(device, pipelines.offscreen, nullptr);
			vkDestroyPipeline(device, pipelines.edge, nullptr);
			vkDestroyPipeline(device, pipelines.lockedEdge, nullptr);
			vkDestroyPipeline(device, pipelines.hiddenLine, nullptr);

			vkDestroyPipelineLayout(device, pipelineLayout, nullptr);

			vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);

			// Uniform buffers
			uniformBuffers.offscreen.destroy();
			uniformBuffers.composition.destroy();
			uniformBuffers.edge.destroy();
			uniformBuffers.lockedEdge.destroy();
			uniformBuffers.edgeFrag.destroy();
			uniformBuffers.lockedEdgeFrag.destroy();
			uniformBuffers.hiddenLine.destroy();

			vkDestroyRenderPass(device, offScreenFrameBuf.renderPass, nullptr);
			vkDestroyRenderPass(device, edgeFrameBuf.renderPass, nullptr);
			vkDestroyRenderPass(device, lockedEdgeFrameBuf.renderPass, nullptr);
			vkDestroyRenderPass(device, hiddenLineFrameBuf.renderPass, nullptr);

			textures.model.colorMap.destroy();
			textures.model.normalMap.destroy();
			textures.floor.colorMap.destroy();
			textures.floor.normalMap.destroy();

			vkDestroySemaphore(device, offscreenSemaphore, nullptr);
		}
	}

	// Enable physical device features required for this example
	virtual void getEnabledFeatures()
	{
		// Enable anisotropic filtering if supported
		if (deviceFeatures.samplerAnisotropy) {
			enabledFeatures.samplerAnisotropy = VK_TRUE;
		}
	};

	// Create a frame buffer attachment
	void createAttachment(
		VkFormat format,
		VkImageUsageFlagBits usage,
		FrameBufferAttachment* attachment)
	{
		VkImageAspectFlags aspectMask = 0;
		VkImageLayout imageLayout;

		attachment->format = format;

		if (usage & VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT)
		{
			aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
			imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		}
		if (usage & VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT)
		{
			aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
			if (format >= VK_FORMAT_D16_UNORM_S8_UINT)
				aspectMask |= VK_IMAGE_ASPECT_STENCIL_BIT;
			imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
		}

		assert(aspectMask > 0);

		VkImageCreateInfo image = vks::initializers::imageCreateInfo();
		image.imageType = VK_IMAGE_TYPE_2D;
		image.format = format;
		image.extent.width = offScreenFrameBuf.width;
		image.extent.height = offScreenFrameBuf.height;
		image.extent.depth = 1;
		image.mipLevels = 1;
		image.arrayLayers = 1;
		image.samples = VK_SAMPLE_COUNT_1_BIT;
		image.tiling = VK_IMAGE_TILING_OPTIMAL;
		image.usage = usage | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

		VkMemoryAllocateInfo memAlloc = vks::initializers::memoryAllocateInfo();
		VkMemoryRequirements memReqs;

		VK_CHECK_RESULT(vkCreateImage(device, &image, nullptr, &attachment->image));
		vkGetImageMemoryRequirements(device, attachment->image, &memReqs);
		memAlloc.allocationSize = memReqs.size;
		memAlloc.memoryTypeIndex = vulkanDevice->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
		VK_CHECK_RESULT(vkAllocateMemory(device, &memAlloc, nullptr, &attachment->mem));
		VK_CHECK_RESULT(vkBindImageMemory(device, attachment->image, attachment->mem, 0));

		VkImageViewCreateInfo imageView = vks::initializers::imageViewCreateInfo();
		imageView.viewType = VK_IMAGE_VIEW_TYPE_2D;
		imageView.format = format;
		imageView.subresourceRange = {};
		imageView.subresourceRange.aspectMask = aspectMask;
		imageView.subresourceRange.baseMipLevel = 0;
		imageView.subresourceRange.levelCount = 1;
		imageView.subresourceRange.baseArrayLayer = 0;
		imageView.subresourceRange.layerCount = 1;
		imageView.image = attachment->image;
		VK_CHECK_RESULT(vkCreateImageView(device, &imageView, nullptr, &attachment->view));
	}

	// Prepare a new framebuffer and attachments for offscreen rendering (G-Buffer)
	void prepareOffscreenFramebuffer()
	{
		// Note: Instead of using fixed sizes, one could also match the window size and recreate the attachments on resize
		//offScreenFrameBuf.width = 1920;
		//offScreenFrameBuf.height = 1080;
		offScreenFrameBuf.width = width * 2;
		offScreenFrameBuf.height = height * 2;

		//self-added
		/*VK_CHECK_RESULT(vulkanDevice->createBuffer(
			VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
			&offScreenFrameBuf.linkedList,
			sizeof(Node) * offScreenFrameBuf.width * offScreenFrameBuf.height));*/
			// 

			// Color attachments

			// (World space) Positions
		createAttachment(
			VK_FORMAT_R16G16B16A16_SFLOAT,
			VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
			&offScreenFrameBuf.position);

		// (World space) Normals
		createAttachment(
			VK_FORMAT_R16G16B16A16_SFLOAT,
			VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
			&offScreenFrameBuf.normal);

		// Albedo (color)
		createAttachment(
			VK_FORMAT_R32G32_SINT,
			VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
			&offScreenFrameBuf.albedo);

		// Depth attachment

		// Find a suitable depth format
		VkFormat attDepthFormat;
		VkBool32 validDepthFormat = vks::tools::getSupportedDepthFormat(physicalDevice, &attDepthFormat);
		assert(validDepthFormat);

		createAttachment(
			attDepthFormat,
			VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
			&offScreenFrameBuf.depth);

		// Set up separate renderpass with references to the color and depth attachments
		std::array<VkAttachmentDescription, 4> attachmentDescs = {};

		// Init attachment properties
		for (uint32_t i = 0; i < 4; ++i)
		{
			attachmentDescs[i].samples = VK_SAMPLE_COUNT_1_BIT;
			attachmentDescs[i].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
			attachmentDescs[i].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
			attachmentDescs[i].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
			attachmentDescs[i].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
			if (i == 3)
			{
				attachmentDescs[i].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
				attachmentDescs[i].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
			}
			else
			{
				attachmentDescs[i].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
				attachmentDescs[i].finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
			}
		}

		// Formats
		attachmentDescs[0].format = offScreenFrameBuf.position.format;
		attachmentDescs[1].format = offScreenFrameBuf.normal.format;
		attachmentDescs[2].format = offScreenFrameBuf.albedo.format;
		attachmentDescs[3].format = offScreenFrameBuf.depth.format;

		std::vector<VkAttachmentReference> colorReferences;
		colorReferences.push_back({ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL });
		colorReferences.push_back({ 1, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL });
		colorReferences.push_back({ 2, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL });

		VkAttachmentReference depthReference = {};
		depthReference.attachment = 3;
		depthReference.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

		VkSubpassDescription subpass = {};
		subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
		subpass.pColorAttachments = colorReferences.data();
		subpass.colorAttachmentCount = static_cast<uint32_t>(colorReferences.size());
		subpass.pDepthStencilAttachment = &depthReference;

		// Use subpass dependencies for attachment layout transitions
		std::array<VkSubpassDependency, 2> dependencies;

		dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL; //outside current render pass
		dependencies[0].dstSubpass = 0;
		dependencies[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
		dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependencies[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

		dependencies[1].srcSubpass = 0;
		dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
		dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
		dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

		VkRenderPassCreateInfo renderPassInfo = {};
		renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
		renderPassInfo.pAttachments = attachmentDescs.data();
		renderPassInfo.attachmentCount = static_cast<uint32_t>(attachmentDescs.size());
		renderPassInfo.subpassCount = 1;
		renderPassInfo.pSubpasses = &subpass;
		renderPassInfo.dependencyCount = dependencies.size();
		renderPassInfo.pDependencies = dependencies.data();

		VK_CHECK_RESULT(vkCreateRenderPass(device, &renderPassInfo, nullptr, &offScreenFrameBuf.renderPass));

		std::array<VkImageView, 4> attachments;
		attachments[0] = offScreenFrameBuf.position.view;
		attachments[1] = offScreenFrameBuf.normal.view;
		attachments[2] = offScreenFrameBuf.albedo.view;
		attachments[3] = offScreenFrameBuf.depth.view;

		VkFramebufferCreateInfo fbufCreateInfo = {};
		fbufCreateInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		fbufCreateInfo.pNext = NULL;
		fbufCreateInfo.renderPass = offScreenFrameBuf.renderPass;
		fbufCreateInfo.pAttachments = attachments.data();
		fbufCreateInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
		fbufCreateInfo.width = offScreenFrameBuf.width;
		fbufCreateInfo.height = offScreenFrameBuf.height;
		fbufCreateInfo.layers = 1;
		VK_CHECK_RESULT(vkCreateFramebuffer(device, &fbufCreateInfo, nullptr, &offScreenFrameBuf.frameBuffer));

		// Create sampler to sample from the color attachments
		VkSamplerCreateInfo sampler = vks::initializers::samplerCreateInfo();
		sampler.magFilter = VK_FILTER_NEAREST;
		sampler.minFilter = VK_FILTER_NEAREST;
		sampler.anisotropyEnable = VK_FALSE;
		sampler.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
		sampler.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		sampler.addressModeV = sampler.addressModeU;
		sampler.addressModeW = sampler.addressModeU;
		sampler.mipLodBias = 0.0f;
		sampler.maxAnisotropy = 1.0f;
		sampler.minLod = 0.0f;
		sampler.maxLod = 1.0f;
		sampler.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
		VK_CHECK_RESULT(vkCreateSampler(device, &sampler, nullptr, &colorSampler));
	}


	void prepareEdgeFramebuffer()
	{
		// Note: Instead of using fixed sizes, one could also match the window size and recreate the attachments on resize
		//offScreenFrameBuf.width = 1920;
		//offScreenFrameBuf.height = 1080;
		edgeFrameBuf.width = width * 2;
		edgeFrameBuf.height = height * 2;

		//self-added
		/*VK_CHECK_RESULT(vulkanDevice->createBuffer(
			VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
			&offScreenFrameBuf.linkedList,
			sizeof(Node) * offScreenFrameBuf.width * offScreenFrameBuf.height));*/
			// 

			// Color attachments

			// (World space) Positions
		createAttachment(
			VK_FORMAT_R32G32B32A32_SINT,
			VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
			&edgeFrameBuf.position);

		createAttachment(
			VK_FORMAT_R32G32B32A32_SFLOAT,
			VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
			&edgeFrameBuf.normal);

		// Depth attachment

		// Find a suitable depth format
		VkFormat attDepthFormat;
		VkBool32 validDepthFormat = vks::tools::getSupportedDepthFormat(physicalDevice, &attDepthFormat);
		assert(validDepthFormat);


		// Set up separate renderpass with references to the color and depth attachments
		std::array<VkAttachmentDescription, 3> attachmentDescs = {};

		// Init attachment properties
		for (uint32_t i = 0; i < 3; ++i)
		{
			//diff: edge should change loadOp to VK_ATTACHMENT_LOAD_OP_LOAD
			attachmentDescs[i].samples = VK_SAMPLE_COUNT_1_BIT;
			attachmentDescs[i].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
			attachmentDescs[i].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
			attachmentDescs[i].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
			if (i == 2)
			{
				//diff: if share depth attachmentm, its initial layout should match the final layout
				//of last depth attachment
				attachmentDescs[i].loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
				attachmentDescs[i].initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
				attachmentDescs[i].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
			}
			else
			{
				attachmentDescs[i].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
				attachmentDescs[i].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
				attachmentDescs[i].finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
			}
		}

		// Formats
		attachmentDescs[0].format = edgeFrameBuf.position.format;
		attachmentDescs[1].format = edgeFrameBuf.normal.format;
		attachmentDescs[2].format = offScreenFrameBuf.depth.format;

		std::vector<VkAttachmentReference> colorReferences;
		colorReferences.push_back({ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL });
		colorReferences.push_back({ 1, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL });

		VkAttachmentReference depthReference = {};
		depthReference.attachment = 2;
		depthReference.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

		VkSubpassDescription subpass = {};
		subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
		subpass.pColorAttachments = colorReferences.data();
		subpass.colorAttachmentCount = static_cast<uint32_t>(colorReferences.size());
		subpass.pDepthStencilAttachment = &depthReference;

		// Use subpass dependencies for attachment layout transitions
		std::array<VkSubpassDependency, 3> dependencies;

		dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
		dependencies[0].dstSubpass = 0;
		dependencies[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
		dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependencies[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

		dependencies[2].srcSubpass = VK_SUBPASS_EXTERNAL;
		dependencies[2].dstSubpass = 0;
		dependencies[2].srcStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
		dependencies[2].dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
		dependencies[2].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		dependencies[2].dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;
		dependencies[2].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

		dependencies[1].srcSubpass = 0;
		dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
		dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
		dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

		VkRenderPassCreateInfo renderPassInfo = {};
		renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
		renderPassInfo.pAttachments = attachmentDescs.data();
		renderPassInfo.attachmentCount = static_cast<uint32_t>(attachmentDescs.size());
		renderPassInfo.subpassCount = 1;
		renderPassInfo.pSubpasses = &subpass;
		renderPassInfo.dependencyCount = dependencies.size();
		renderPassInfo.pDependencies = dependencies.data();

		VK_CHECK_RESULT(vkCreateRenderPass(device, &renderPassInfo, nullptr, &edgeFrameBuf.renderPass));

		std::array<VkImageView, 3> attachments;
		attachments[0] = edgeFrameBuf.position.view;
		attachments[1] = edgeFrameBuf.normal.view;
		attachments[2] = offScreenFrameBuf.depth.view;

		VkFramebufferCreateInfo fbufCreateInfo = {};
		fbufCreateInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		fbufCreateInfo.pNext = NULL;
		fbufCreateInfo.renderPass = edgeFrameBuf.renderPass;
		fbufCreateInfo.pAttachments = attachments.data();
		fbufCreateInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
		fbufCreateInfo.width = edgeFrameBuf.width;
		fbufCreateInfo.height = edgeFrameBuf.height;
		fbufCreateInfo.layers = 1;
		VK_CHECK_RESULT(vkCreateFramebuffer(device, &fbufCreateInfo, nullptr, &edgeFrameBuf.frameBuffer));
	}

	void prepareLockedEdgeFramebuffer(vks::VulkanDevice* vulkanDevice)
	{
		// Note: Instead of using fixed sizes, one could also match the window size and recreate the attachments on resize
		//offScreenFrameBuf.width = 1920;
		//offScreenFrameBuf.height = 1080;
		lockedEdgeFrameBuf.width = width * 2;
		lockedEdgeFrameBuf.height = height * 2;

		//self-added
		/*VK_CHECK_RESULT(vulkanDevice->createBuffer(
			VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
			&offScreenFrameBuf.linkedList,
			sizeof(Node) * offScreenFrameBuf.width * offScreenFrameBuf.height));*/
			// 

			// Color attachments

			// (World space) Positions
		createAttachment(
			VK_FORMAT_R32G32B32A32_SFLOAT,
			VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
			&lockedEdgeFrameBuf.position);

		VK_CHECK_RESULT(vulkanDevice->createBuffer(
			VK_BUFFER_USAGE_TRANSFER_DST_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
			edgeFrameBuf.width * edgeFrameBuf.height * sizeof(glm::vec4),
			&mesh.cpuImageBuffer.buffer,
			&mesh.cpuImageBuffer.memory));
		// Depth attachment

		// Find a suitable depth format
		VkFormat attDepthFormat;
		VkBool32 validDepthFormat = vks::tools::getSupportedDepthFormat(physicalDevice, &attDepthFormat);
		assert(validDepthFormat);

		createAttachment(
			attDepthFormat,
			VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
			&lockedEdgeFrameBuf.depth);

		// Set up separate renderpass with references to the color and depth attachments
		std::array<VkAttachmentDescription, 2> attachmentDescs = {};

		// Init attachment properties
		for (uint32_t i = 0; i < 2; ++i)
		{
			//diff: edge should change loadOp to VK_ATTACHMENT_LOAD_OP_LOAD
			attachmentDescs[i].samples = VK_SAMPLE_COUNT_1_BIT;
			attachmentDescs[i].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
			attachmentDescs[i].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
			attachmentDescs[i].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
			if (i == 1)
			{
				//diff: if share depth attachmentm, its initial layout should match the final layout
				//of last depth attachment
				attachmentDescs[i].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
				attachmentDescs[i].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
				attachmentDescs[i].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
			}
			else
			{
				attachmentDescs[i].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
				attachmentDescs[i].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
				attachmentDescs[i].finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
			}
		}

		// Formats
		attachmentDescs[0].format = lockedEdgeFrameBuf.position.format;
		attachmentDescs[1].format = lockedEdgeFrameBuf.depth.format;

		std::vector<VkAttachmentReference> colorReferences;
		colorReferences.push_back({ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL });

		VkAttachmentReference depthReference = {};
		depthReference.attachment = 1;
		depthReference.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

		VkSubpassDescription subpass = {};
		subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
		subpass.pColorAttachments = colorReferences.data();
		subpass.colorAttachmentCount = static_cast<uint32_t>(colorReferences.size());
		subpass.pDepthStencilAttachment = &depthReference;

		// Use subpass dependencies for attachment layout transitions
		std::array<VkSubpassDependency, 2> dependencies;

		dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
		dependencies[0].dstSubpass = 0;
		dependencies[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
		dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependencies[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

		dependencies[1].srcSubpass = 0;
		dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
		dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
		dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

		VkRenderPassCreateInfo renderPassInfo = {};
		renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
		renderPassInfo.pAttachments = attachmentDescs.data();
		renderPassInfo.attachmentCount = static_cast<uint32_t>(attachmentDescs.size());
		renderPassInfo.subpassCount = 1;
		renderPassInfo.pSubpasses = &subpass;
		renderPassInfo.dependencyCount = dependencies.size();
		renderPassInfo.pDependencies = dependencies.data();

		VK_CHECK_RESULT(vkCreateRenderPass(device, &renderPassInfo, nullptr, &lockedEdgeFrameBuf.renderPass));

		std::array<VkImageView, 2> attachments;
		attachments[0] = lockedEdgeFrameBuf.position.view;
		attachments[1] = lockedEdgeFrameBuf.depth.view;

		VkFramebufferCreateInfo fbufCreateInfo = {};
		fbufCreateInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		fbufCreateInfo.pNext = NULL;
		fbufCreateInfo.renderPass = lockedEdgeFrameBuf.renderPass;
		fbufCreateInfo.pAttachments = attachments.data();
		fbufCreateInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
		fbufCreateInfo.width = lockedEdgeFrameBuf.width;
		fbufCreateInfo.height = lockedEdgeFrameBuf.height;
		fbufCreateInfo.layers = 1;
		VK_CHECK_RESULT(vkCreateFramebuffer(device, &fbufCreateInfo, nullptr, &lockedEdgeFrameBuf.frameBuffer));
	}

	void prepareHiddenLineFramebuffer()
	{
		// Note: Instead of using fixed sizes, one could also match the window size and recreate the attachments on resize
		//offScreenFrameBuf.width = 1920;
		//offScreenFrameBuf.height = 1080;
		hiddenLineFrameBuf.width = width * 2;
		hiddenLineFrameBuf.height = height * 2;

		//self-added
		/*VK_CHECK_RESULT(vulkanDevice->createBuffer(
			VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
			&offScreenFrameBuf.linkedList,
			sizeof(Node) * offScreenFrameBuf.width * offScreenFrameBuf.height));*/
			// 

			// Color attachments

			// (World space) Positions
		createAttachment(
			VK_FORMAT_R32G32B32A32_SFLOAT,
			VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
			&hiddenLineFrameBuf.position);

		/*VK_CHECK_RESULT(vulkanDevice->createBuffer(
			VK_BUFFER_USAGE_TRANSFER_DST_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
			edgeFrameBuf.width * edgeFrameBuf.height * sizeof(glm::vec4),
			&mesh.cpuImageBuffer.buffer,
			&mesh.cpuImageBuffer.memory));*/
		// Depth attachment

		// Find a suitable depth format
		VkFormat attDepthFormat;
		VkBool32 validDepthFormat = vks::tools::getSupportedDepthFormat(physicalDevice, &attDepthFormat);
		assert(validDepthFormat);

		createAttachment(
			attDepthFormat,
			VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
			&hiddenLineFrameBuf.depth);

		// Set up separate renderpass with references to the color and depth attachments
		std::array<VkAttachmentDescription, 2> attachmentDescs = {};

		// Init attachment properties
		for (uint32_t i = 0; i < 2; ++i)
		{
			//diff: edge should change loadOp to VK_ATTACHMENT_LOAD_OP_LOAD
			attachmentDescs[i].samples = VK_SAMPLE_COUNT_1_BIT;
			attachmentDescs[i].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
			attachmentDescs[i].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
			attachmentDescs[i].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
			if (i == 1)
			{
				//diff: if share depth attachmentm, its initial layout should match the final layout
				//of last depth attachment
				attachmentDescs[i].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
				attachmentDescs[i].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
				attachmentDescs[i].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
			}
			else
			{
				attachmentDescs[i].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
				attachmentDescs[i].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
				attachmentDescs[i].finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
			}
		}

		// Formats
		attachmentDescs[0].format = hiddenLineFrameBuf.position.format;
		attachmentDescs[1].format = hiddenLineFrameBuf.depth.format;

		std::vector<VkAttachmentReference> colorReferences;
		colorReferences.push_back({ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL });

		VkAttachmentReference depthReference = {};
		depthReference.attachment = 1;
		depthReference.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

		VkSubpassDescription subpass = {};
		subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
		subpass.pColorAttachments = colorReferences.data();
		subpass.colorAttachmentCount = static_cast<uint32_t>(colorReferences.size());
		subpass.pDepthStencilAttachment = &depthReference;

		// Use subpass dependencies for attachment layout transitions
		std::array<VkSubpassDependency, 2> dependencies;

		dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
		dependencies[0].dstSubpass = 0;
		dependencies[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
		dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependencies[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

		dependencies[1].srcSubpass = 0;
		dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
		dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
		dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

		VkRenderPassCreateInfo renderPassInfo = {};
		renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
		renderPassInfo.pAttachments = attachmentDescs.data();
		renderPassInfo.attachmentCount = static_cast<uint32_t>(attachmentDescs.size());
		renderPassInfo.subpassCount = 1;
		renderPassInfo.pSubpasses = &subpass;
		renderPassInfo.dependencyCount = dependencies.size();
		renderPassInfo.pDependencies = dependencies.data();

		VK_CHECK_RESULT(vkCreateRenderPass(device, &renderPassInfo, nullptr, &hiddenLineFrameBuf.renderPass));

		std::array<VkImageView, 2> attachments;
		attachments[0] = hiddenLineFrameBuf.position.view;
		attachments[1] = hiddenLineFrameBuf.depth.view;

		VkFramebufferCreateInfo fbufCreateInfo = {};
		fbufCreateInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		fbufCreateInfo.pNext = NULL;
		fbufCreateInfo.renderPass = hiddenLineFrameBuf.renderPass;
		fbufCreateInfo.pAttachments = attachments.data();
		fbufCreateInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
		fbufCreateInfo.width = hiddenLineFrameBuf.width;
		fbufCreateInfo.height = hiddenLineFrameBuf.height;
		fbufCreateInfo.layers = 1;
		VK_CHECK_RESULT(vkCreateFramebuffer(device, &fbufCreateInfo, nullptr, &hiddenLineFrameBuf.frameBuffer));
	}

	// Build command buffer for rendering the scene to the offscreen frame buffer attachments (to G-buffer)
	void buildDeferredCommandBuffer()
	{
		if (offScreenCmdBuffer == VK_NULL_HANDLE) {
			offScreenCmdBuffer = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, false);
		}

		// Create a semaphore used to synchronize offscreen rendering and usage
		VkSemaphoreCreateInfo semaphoreCreateInfo = vks::initializers::semaphoreCreateInfo();
		VK_CHECK_RESULT(vkCreateSemaphore(device, &semaphoreCreateInfo, nullptr, &offscreenSemaphore));

		VkCommandBufferBeginInfo cmdBufInfo = vks::initializers::commandBufferBeginInfo();

		// Clear values for all attachments written in the fragment shader
		std::array<VkClearValue, 4> clearValues;
		clearValues[0].color = { { 0.0f, 0.0f, 0.0f, 0.0f } };
		clearValues[1].color = { { 0.0f, 0.0f, 0.0f, 0.0f } };
		clearValues[2].color.int32[0] = -1;
		clearValues[2].color.int32[1] = -1;
		clearValues[3].depthStencil = { 1.0f, 0 };

		VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
		renderPassBeginInfo.renderPass = offScreenFrameBuf.renderPass;
		renderPassBeginInfo.framebuffer = offScreenFrameBuf.frameBuffer;
		renderPassBeginInfo.renderArea.extent.width = offScreenFrameBuf.width;
		renderPassBeginInfo.renderArea.extent.height = offScreenFrameBuf.height;
		renderPassBeginInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
		renderPassBeginInfo.pClearValues = clearValues.data();

		VK_CHECK_RESULT(vkBeginCommandBuffer(offScreenCmdBuffer, &cmdBufInfo));

		vkCmdBeginRenderPass(offScreenCmdBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

		VkViewport viewport = vks::initializers::viewport((float)offScreenFrameBuf.width, -(float)offScreenFrameBuf.height, 0.0f, 1.0f);
		viewport.x = 0;
		viewport.y = (float)offScreenFrameBuf.height;
		vkCmdSetViewport(offScreenCmdBuffer, 0, 1, &viewport);

		VkRect2D scissor = vks::initializers::rect2D(offScreenFrameBuf.width, offScreenFrameBuf.height, 0, 0);
		vkCmdSetScissor(offScreenCmdBuffer, 0, 1, &scissor);

		vkCmdBindPipeline(offScreenCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.offscreen);

		pushVal.screenHalfLengthX = halfScreenHeightX;
		pushVal.screenHalfLengthY = halfScreenHeightY;
		vkCmdPushConstants(offScreenCmdBuffer, pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushValue), &pushVal);
		//// Floor
		//vkCmdBindDescriptorSets(offScreenCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets.floor, 0, nullptr);
		//models.floor.draw(offScreenCmdBuffer);

		//// We render multiple instances of a model
		//vkCmdBindDescriptorSets(offScreenCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets.model, 0, nullptr);
		//models.model.bindBuffers(offScreenCmdBuffer);
		//vkCmdDrawIndexed(offScreenCmdBuffer, models.model.indices.count, 3, 0, 0, 0);
		vkCmdBindDescriptorSets(offScreenCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets.floor, 0, nullptr);
		mesh.bindBuffers(offScreenCmdBuffer);
		vkCmdDrawIndexed(offScreenCmdBuffer, mesh.index.size(), 1, 0, 0, 0);

		vkCmdEndRenderPass(offScreenCmdBuffer);

		VK_CHECK_RESULT(vkEndCommandBuffer(offScreenCmdBuffer));
	}

	void buildEdgeCommandBuffer()
	{
		if (edgeCmdBuffer == VK_NULL_HANDLE) {
			edgeCmdBuffer = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, false);
		}

		// Create a semaphore used to synchronize offscreen rendering and usage
		VkSemaphoreCreateInfo semaphoreCreateInfo = vks::initializers::semaphoreCreateInfo();
		VK_CHECK_RESULT(vkCreateSemaphore(device, &semaphoreCreateInfo, nullptr, &edgeSemaphore));

		VkCommandBufferBeginInfo cmdBufInfo = vks::initializers::commandBufferBeginInfo();

		std::array<VkClearValue, 2> clearValues;
		clearValues[0].color.int32[0] = -1;
		clearValues[0].color.int32[1] = -1;
		clearValues[0].color.int32[2] = 0;
		clearValues[0].color.int32[3] = 0;
		clearValues[1].color = { {0.0f, 0.0f, 0.0f, 0.0f} };

		VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
		renderPassBeginInfo.renderPass = edgeFrameBuf.renderPass;
		renderPassBeginInfo.framebuffer = edgeFrameBuf.frameBuffer;
		renderPassBeginInfo.renderArea.extent.width = edgeFrameBuf.width;
		renderPassBeginInfo.renderArea.extent.height = edgeFrameBuf.height;
		//diff: render pass don't have any clear values
		/*renderPassBeginInfo.clearValueCount = 1;
		renderPassBeginInfo.pClearValues = clearValues;*/
		renderPassBeginInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
		renderPassBeginInfo.pClearValues = clearValues.data();

		// Add memory barrier between render passes
		VkImageMemoryBarrier colorBarrier = {};
		colorBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		colorBarrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		colorBarrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;
		colorBarrier.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		colorBarrier.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		colorBarrier.image = offScreenFrameBuf.albedo.image;
		colorBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		colorBarrier.subresourceRange.baseMipLevel = 0;
		colorBarrier.subresourceRange.levelCount = 1;
		colorBarrier.subresourceRange.baseArrayLayer = 0;
		colorBarrier.subresourceRange.layerCount = 1;

		VkImageMemoryBarrier positionBarrier = {};
		positionBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		positionBarrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		positionBarrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;
		positionBarrier.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		positionBarrier.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		positionBarrier.image = offScreenFrameBuf.position.image;
		positionBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		positionBarrier.subresourceRange.baseMipLevel = 0;
		positionBarrier.subresourceRange.levelCount = 1;
		positionBarrier.subresourceRange.baseArrayLayer = 0;
		positionBarrier.subresourceRange.layerCount = 1;

		VkImageMemoryBarrier barriers[] = { colorBarrier, positionBarrier };

		VK_CHECK_RESULT(vkBeginCommandBuffer(edgeCmdBuffer, &cmdBufInfo));

		vkCmdPipelineBarrier(
			edgeCmdBuffer,
			VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
			VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
			0,
			0, nullptr,
			0, nullptr,
			2, barriers
		);

		vkCmdBeginRenderPass(edgeCmdBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

		VkViewport viewport = vks::initializers::viewport((float)edgeFrameBuf.width, -(float)edgeFrameBuf.height, 0.0f, 1.0f);
		viewport.x = 0;
		viewport.y = (float)edgeFrameBuf.height;
		vkCmdSetViewport(edgeCmdBuffer, 0, 1, &viewport);

		VkRect2D scissor = vks::initializers::rect2D(edgeFrameBuf.width, edgeFrameBuf.height, 0, 0);
		vkCmdSetScissor(edgeCmdBuffer, 0, 1, &scissor);

		vkCmdBindPipeline(edgeCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.edge);


		vkCmdPushConstants(edgeCmdBuffer, pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushValue2), &pushVal2);

		vkCmdBindDescriptorSets(edgeCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets.edge, 0, nullptr);
		mesh.bindLineBuffers(edgeCmdBuffer);
		vkCmdDrawIndexed(edgeCmdBuffer, mesh.edgeIdx.size(), 1, 0, 0, 0);
		//vkCmdDraw(edgeCmdBuffer, 2, 1, 0, 0);

		vkCmdEndRenderPass(edgeCmdBuffer);

		VK_CHECK_RESULT(vkEndCommandBuffer(edgeCmdBuffer));
	}

	void rebuildEdgeCommandBuffer()
	{
		vkResetCommandBuffer(edgeCmdBuffer, 0);

		VkCommandBufferBeginInfo cmdBufInfo = vks::initializers::commandBufferBeginInfo();

		VkClearValue clearValues[1];
		clearValues[0].color.int32[0] = -1;
		clearValues[0].color.int32[1] = -1;
		clearValues[0].color.int32[2] = 0;
		clearValues[0].color.int32[3] = 0;

		VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
		renderPassBeginInfo.renderPass = edgeFrameBuf.renderPass;
		renderPassBeginInfo.framebuffer = edgeFrameBuf.frameBuffer;
		renderPassBeginInfo.renderArea.extent.width = edgeFrameBuf.width;
		renderPassBeginInfo.renderArea.extent.height = edgeFrameBuf.height;
		//diff: render pass don't have any clear values
		renderPassBeginInfo.clearValueCount = 1;
		renderPassBeginInfo.pClearValues = clearValues;

		// Add memory barrier between render passes
		VkImageMemoryBarrier colorBarrier = {};
		colorBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		colorBarrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		colorBarrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;
		colorBarrier.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		colorBarrier.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		colorBarrier.image = offScreenFrameBuf.albedo.image;
		colorBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		colorBarrier.subresourceRange.baseMipLevel = 0;
		colorBarrier.subresourceRange.levelCount = 1;
		colorBarrier.subresourceRange.baseArrayLayer = 0;
		colorBarrier.subresourceRange.layerCount = 1;

		VkImageMemoryBarrier positionBarrier = {};
		positionBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		positionBarrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		positionBarrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;
		positionBarrier.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		positionBarrier.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		positionBarrier.image = offScreenFrameBuf.position.image;
		positionBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		positionBarrier.subresourceRange.baseMipLevel = 0;
		positionBarrier.subresourceRange.levelCount = 1;
		positionBarrier.subresourceRange.baseArrayLayer = 0;
		positionBarrier.subresourceRange.layerCount = 1;

		VkImageMemoryBarrier barriers[] = { colorBarrier, positionBarrier };

		VK_CHECK_RESULT(vkBeginCommandBuffer(edgeCmdBuffer, &cmdBufInfo));

		vkCmdPipelineBarrier(
			edgeCmdBuffer,
			VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
			VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
			0,
			0, nullptr,
			0, nullptr,
			2, barriers
		);

		vkCmdBeginRenderPass(edgeCmdBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

		VkViewport viewport = vks::initializers::viewport((float)edgeFrameBuf.width, -(float)edgeFrameBuf.height, 0.0f, 1.0f);
		viewport.x = 0;
		viewport.y = (float)edgeFrameBuf.height;
		vkCmdSetViewport(edgeCmdBuffer, 0, 1, &viewport);

		VkRect2D scissor = vks::initializers::rect2D(edgeFrameBuf.width, edgeFrameBuf.height, 0, 0);
		vkCmdSetScissor(edgeCmdBuffer, 0, 1, &scissor);

		vkCmdBindPipeline(edgeCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.edge);

		vkCmdPushConstants(edgeCmdBuffer, pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushValue2), &pushVal2);

		vkCmdBindDescriptorSets(edgeCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets.edge, 0, nullptr);
		mesh.bindLineBuffers(edgeCmdBuffer);
		vkCmdDrawIndexed(edgeCmdBuffer, mesh.edgeIdx.size(), 1, 0, 0, 0);
		//vkCmdDraw(edgeCmdBuffer, 2, 1, 0, 0);

		vkCmdEndRenderPass(edgeCmdBuffer);

		VK_CHECK_RESULT(vkEndCommandBuffer(edgeCmdBuffer));
	}

	void buildLockedEdgeCommandBuffer() {
		if (lockedEdgeCmdBuffer == VK_NULL_HANDLE) {
			lockedEdgeCmdBuffer = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, false);
		}

		VkSemaphoreCreateInfo semaphoreCreateInfo = vks::initializers::semaphoreCreateInfo();
		VK_CHECK_RESULT(vkCreateSemaphore(device, &semaphoreCreateInfo, nullptr, &lockedEdgeSemaphore));

		VkCommandBufferBeginInfo cmdBufInfo = vks::initializers::commandBufferBeginInfo();

		std::array<VkClearValue, 2> clearValues;
		clearValues[0].color = { { 0.0f, 0.0f, 0.0f, 0.0f } };
		clearValues[1].depthStencil = { 1.0f, 0 };

		VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
		renderPassBeginInfo.renderPass = lockedEdgeFrameBuf.renderPass;
		renderPassBeginInfo.framebuffer = lockedEdgeFrameBuf.frameBuffer;
		renderPassBeginInfo.renderArea.extent.width = lockedEdgeFrameBuf.width;
		renderPassBeginInfo.renderArea.extent.height = lockedEdgeFrameBuf.height;
		//diff: render pass don't have any clear values
		renderPassBeginInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
		renderPassBeginInfo.pClearValues = clearValues.data();

		VK_CHECK_RESULT(vkBeginCommandBuffer(lockedEdgeCmdBuffer, &cmdBufInfo));

		vkCmdBeginRenderPass(lockedEdgeCmdBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

		VkViewport viewport = vks::initializers::viewport((float)lockedEdgeFrameBuf.width, -(float)lockedEdgeFrameBuf.height, 0.0f, 1.0f);
		viewport.x = 0;
		viewport.y = (float)lockedEdgeFrameBuf.height;
		vkCmdSetViewport(lockedEdgeCmdBuffer, 0, 1, &viewport);

		VkRect2D scissor = vks::initializers::rect2D(lockedEdgeFrameBuf.width, lockedEdgeFrameBuf.height, 0, 0);
		vkCmdSetScissor(lockedEdgeCmdBuffer, 0, 1, &scissor);

		vkCmdBindPipeline(lockedEdgeCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.lockedEdge);

		vkCmdPushConstants(edgeCmdBuffer, pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushValue2), &pushVal2);

		vkCmdBindDescriptorSets(lockedEdgeCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets.lockedEdge, 0, nullptr);
		mesh.bindLockedEdgeBuffers(lockedEdgeCmdBuffer);
		vkCmdDrawIndexed(lockedEdgeCmdBuffer, mesh.lockedEdgeIdxCnt, 1, 0, 0, 0);

		vkCmdEndRenderPass(lockedEdgeCmdBuffer);


		VK_CHECK_RESULT(vkEndCommandBuffer(lockedEdgeCmdBuffer));
	}

	void buildHiddenLineCommandBuffer() {

		if (hiddenLineCmdBuffer == VK_NULL_HANDLE) {
			hiddenLineCmdBuffer = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, false);
		}

		// Create a semaphore used to synchronize offscreen rendering and usage
		VkSemaphoreCreateInfo semaphoreCreateInfo = vks::initializers::semaphoreCreateInfo();
		VK_CHECK_RESULT(vkCreateSemaphore(device, &semaphoreCreateInfo, nullptr, &hiddenLineSemaphore));

		VkCommandBufferBeginInfo cmdBufInfo = vks::initializers::commandBufferBeginInfo();

		std::array<VkClearValue, 2> clearValues;
		clearValues[0].color = { { 0.0f, 0.0f, 0.0f, 0.0f } };
		clearValues[1].depthStencil = { 1.0f, 0 };

		VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
		renderPassBeginInfo.renderPass = hiddenLineFrameBuf.renderPass;
		renderPassBeginInfo.framebuffer = hiddenLineFrameBuf.frameBuffer;
		renderPassBeginInfo.renderArea.extent.width = hiddenLineFrameBuf.width;
		renderPassBeginInfo.renderArea.extent.height = hiddenLineFrameBuf.height;
		//diff: render pass don't have any clear values
		renderPassBeginInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
		renderPassBeginInfo.pClearValues = clearValues.data();

		VK_CHECK_RESULT(vkBeginCommandBuffer(hiddenLineCmdBuffer, &cmdBufInfo));

		vkCmdBeginRenderPass(hiddenLineCmdBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

		VkViewport viewport = vks::initializers::viewport((float)hiddenLineFrameBuf.width, -(float)hiddenLineFrameBuf.height, 0.0f, 1.0f);
		viewport.x = 0;
		viewport.y = (float)hiddenLineFrameBuf.height;
		vkCmdSetViewport(hiddenLineCmdBuffer, 0, 1, &viewport);

		VkRect2D scissor = vks::initializers::rect2D(hiddenLineFrameBuf.width, hiddenLineFrameBuf.height, 0, 0);
		vkCmdSetScissor(hiddenLineCmdBuffer, 0, 1, &scissor);

		vkCmdBindPipeline(hiddenLineCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.hiddenLine);

		//vkCmdPushConstants(edgeCmdBuffer, pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushValue2), &pushVal2);

		vkCmdBindDescriptorSets(hiddenLineCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets.hiddenLine, 0, nullptr);
		mesh.bindHiddenLineBuffers(hiddenLineCmdBuffer);
		vkCmdDrawIndexed(hiddenLineCmdBuffer, mesh.hiddenLineIdxCnt, 1, 0, 0, 0);
		//vkCmdDraw(hiddenLineCmdBuffer, mesh.hiddenLineIdxCnt, 1, 0, 0);
		vkCmdEndRenderPass(hiddenLineCmdBuffer);


		VK_CHECK_RESULT(vkEndCommandBuffer(hiddenLineCmdBuffer));
	}

	static bool compareByU(const gp_Pnt2d& a, const gp_Pnt2d& b) {
		return a.X() < b.X();
	};

	static bool compareByV(const gp_Pnt2d& a, const gp_Pnt2d& b) {
		return a.Y() < b.Y();
	};

	static bool compareByTupleFirst(const std::tuple<Standard_Real, int>& a, const std::tuple<Standard_Real, int>& b) {
		return std::get<0>(a) < std::get<0>(b);
	}

	static void silhouettePnt2dEmplace1(std::vector<std::tuple<Standard_Real, Standard_Real>>& silhouette2dPnt, double u1, double u2, double v1, double v2, int p) {
		if (p == 1) {
			silhouette2dPnt.emplace_back(u1, v1);
		}
		else if (p == 2) {
			silhouette2dPnt.emplace_back(u2, v1);
		}
		else if (p == 3) {
			silhouette2dPnt.emplace_back(u1, v2);
		}
		else if(p == 4){
			silhouette2dPnt.emplace_back(u2, v2);
		}
	}

	static void silhouettePnt2dEmplace2(std::vector<std::tuple<Standard_Real, Standard_Real>>& silhouette2dPnt, double u1, double u2, double v1, double v2, int p1, int p2, double t) {
		if ((p1 == 1) && (p2 == 2)) {
			silhouette2dPnt.emplace_back(glm::mix(u1, u2, t), v1);
		}
		else if ((p1 == 1) && (p2 == 3)) {
			silhouette2dPnt.emplace_back(u1, glm::mix(v1, v2, t));
		}
		else if ((p1 == 1) && (p2 == 4)) {
			silhouette2dPnt.emplace_back(glm::mix(u1, u2, t), glm::mix(v1, v2, t));
		}
		else if ((p1 == 2) && (p2 == 1)) {
			silhouette2dPnt.emplace_back(glm::mix(u2, u1, t), v1);
		}
		else if ((p1 == 2) && (p2 == 3)) {
			silhouette2dPnt.emplace_back(glm::mix(u2, u1, t), glm::mix(v1, v2, t));
		}
		else if ((p1 == 2) && (p2 == 4)) {
			silhouette2dPnt.emplace_back(u2, glm::mix(v1, v2, t));
		}
		else if ((p1 == 3) && (p2 == 1)) {
			silhouette2dPnt.emplace_back(u1, glm::mix(v2, v1, t));
		}
		else if ((p1 == 3) && (p2 == 2)) {
			silhouette2dPnt.emplace_back(glm::mix(u1, u2, t), glm::mix(v2, v1, t));
		}
		else if ((p1 == 3) && (p2 == 4)) {
			silhouette2dPnt.emplace_back(glm::mix(u1, u2, t), v2);
		}
		else if ((p1 == 4) && (p2 == 1)) {
			silhouette2dPnt.emplace_back(glm::mix(u2, u1, t), glm::mix(v2, v1, t));
		}
		else if ((p1 == 4) && (p2 == 2)) {
			silhouette2dPnt.emplace_back(u2, glm::mix(v2, v1, t));
		}
		else if ((p1 == 4) && (p2 == 3)) {
			silhouette2dPnt.emplace_back(glm::mix(u2, u1, t), v2);
		}

	}

	static int calculateSilhouetteOri(std::vector<glm::dvec2> l1, std::vector<glm::dvec2> l2, Handle(Geom_BSplineSurface) bsplineSurf, Camera& camera) {
		glm::dvec2 l1c(0);
		glm::dvec2 l2c(0);
		for (auto& l1p : l1) {
			l1c += l1p;
		}
		for(auto& l2p : l2) {
			l2c += l2p;
		}
		l1c /= l1.size();
		l2c /= l2.size();
		
		GeomLProp_SLProps props(bsplineSurf, l1c.x, l1c.y, 1, Precision::Confusion());
		gp_Dir p1ngp = props.Normal();
		glm::vec3 p1n(p1ngp.X(), p1ngp.Y(), p1ngp.Z());
		p1n = glm::mat3(camera.matrices.view) * p1n;
		float l1n = glm::dot(p1n, glm::vec3(0, 0, 1));

		props = GeomLProp_SLProps(bsplineSurf, l2c.x, l2c.y, 1, Precision::Confusion());
		gp_Dir p2ngp = props.Normal();
		glm::vec3 p2n(p2ngp.X(), p2ngp.Y(), p2ngp.Z());
		p2n = glm::mat3(camera.matrices.view) * p2n;
		float l2n = glm::dot(p2n, glm::vec3(0, 0, 1));

		glm::vec3 np;
		glm::vec3 nn;
		gp_Pnt nne1;
		gp_Pnt nne2;
		if (l1n < 0) {
			//l1反向，l2为正向
			np = p2n;
			nn = p1n;
			nne1 = bsplineSurf->Value(l1[0].x, l1[0].y);
			nne2 = bsplineSurf->Value(l1[1].x, l1[1].y);
		}
		else {
			//l1正向，l2为反向
			np = p1n;
			nn = p2n;
			nne1 = bsplineSurf->Value(l2[0].x, l2[0].y);
			nne2 = bsplineSurf->Value(l2[1].x, l2[1].y);
		}
		gp_Vec nndirtmp(nne1, nne2);
		glm::vec3 nndir = glm::vec3(nndirtmp.X(), nndirtmp.Y(), nndirtmp.Z());
		float res = glm::dot(glm::cross(np, nn), nndir);
		if (res < 0) {
			//正silhouette
			return 0;
		}
		else if(res == 0){
			return 1;
		}
		else {
			//反silhouette
			return 2;
		}
	}

	void loadAssets()
	{
		const uint32_t glTFLoadingFlags = vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::PreMultiplyVertexColors;
		models.model.loadFromFile(getAssetPath() + "models/armor/armor.gltf", vulkanDevice, queue, glTFLoadingFlags);
		models.floor.loadFromFile(getAssetPath() + "models/deferred_floor.gltf", vulkanDevice, queue, glTFLoadingFlags);
		textures.model.colorMap.loadFromFile(getAssetPath() + "models/armor/colormap_rgba.ktx", VK_FORMAT_R8G8B8A8_UNORM, vulkanDevice, queue);
		textures.model.normalMap.loadFromFile(getAssetPath() + "models/armor/normalmap_rgba.ktx", VK_FORMAT_R8G8B8A8_UNORM, vulkanDevice, queue);
		textures.floor.colorMap.loadFromFile(getAssetPath() + "textures/stonefloor01_color_rgba.ktx", VK_FORMAT_R8G8B8A8_UNORM, vulkanDevice, queue);
		textures.floor.normalMap.loadFromFile(getAssetPath() + "textures/stonefloor01_normal_rgba.ktx", VK_FORMAT_R8G8B8A8_UNORM, vulkanDevice, queue);
		std::vector<std::vector<uint32_t>> indexBuffers;
		std::vector<std::vector<vkglTF::Vertex>> vertexBuffers;
		//model.loadFromFileWithVertIdxMultipleMesh(indexBuffers, vertexBuffers, { getAssetPath() + "models/armor/armor.gltf", getAssetPath() + "models/test/12_quad_far.gltf", getAssetPath() + "models/cerberus/cerberus.gltf" }, vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/cylinder", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/car", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/car_smooth_normal", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/car_uv", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/navy", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/navy_lowpoly", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/navy_debug", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/navy_debug2", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/navy_debug3", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/pixels_debug", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/torus", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/car_screen_debug", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/car_brake", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/car_debug", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/5f_cube_smooth", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/test", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/two_tri", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/three_tri", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/quad", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/cube", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/282", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/wrong_linewidth", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/sphere_bump", vulkanDevice, queue, glTFLoadingFlags);
		model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/two_cube", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/finger", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/patch_bump", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/classic_patch1", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/patch_bump2", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/singular_patch1", vulkanDevice, queue, glTFLoadingFlags);
		//mesh.create(indexBuffers, vertexBuffers, vulkanDevice, queue);
		//view independent
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/usb-charger-with-detachable-us-plug-1.snapshot.37/3D CAD Files/apple_12w_usb_charger_us_plug.stp";
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/soft-drink-glass-cup-300ml-10oz-1.snapshot.16/3D CAD Files/casablanca_water_glass_300ml.stp";
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/agv-robot-arm-1.snapshot.3/AGV Robot Arm.stp";
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/013-housing-013-hsg-1.snapshot.10/013-hsg_thread.stp";
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/kibri-37033-chalet-beckenried-1.snapshot.1/Kibri 37033 Chalet Beckenried/Kibri 37033 Chalet Beckenried.stp";
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/piper-j-3-cub-editable-and-scalable-kit-123-parts-1.snapshot.7/Piper J3-CUB WWII_5.STEP";
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/sheet-metal-tool-holder-ac-apm-v4-1.snapshot.5/AC-APM-V4.stp";
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/yak-3-rc-1.snapshot.6/YAK 3 SKRZYDLO L.stp";
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/electric-plane-airbus-is-nervous-1.snapshot.1/204-00-00.stp";
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/plastic-part-00-1.snapshot.2/Plastic Part 00.stp";
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/e-bobber-v2-concept-1.snapshot.15/E-Bobber V2 concept.stp";
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/forked-drop-down-longboard-1.snapshot.3/Forked Drop-Down Longboard.stp";
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/mesa-table-2.snapshot.1/MESA.stp";
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/escritorio-desk-2.snapshot.1/ESCRITORIO.stp";
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/Jeep sheet metal.STEP";
		//用于测试trim curve等等
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/ff1-flying-formula-1-1.snapshot.1/3.stp";
		//大部分完美，有些curve没trim掉
		//const char* filename = (getAssetPath() + "models/stp/Extra-230-50ccm/Extra-230-50ccm_5.stp").c_str();
		std::string filenameTmp = getAssetPath() + "models/stp/Extra-230-50ccm/head2.stp";
		const char* filename = filenameTmp.c_str();
		// 轮椅
		//const char* filename = "D:/Program Files/Google/Chrome/Downloads/COMPLETA.STEP";
		STEPControl_Reader reader;
		IFSelect_ReturnStatus status = reader.ReadFile(filename);
		if (status == IFSelect_RetDone) {
			// 转换所有根，nbr为Number of roots transferred。
			Standard_Integer nbr = reader.TransferRoots();

			// 获取转换后的形状
			shape = reader.OneShape();
			//为整个形状或单个边构建 3D 几何
			BRepLib::BuildCurves3d(shape);
			
			// Traverse all faces
			//int nurbsCount = 0;
			//for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
			//	TopoDS_Face face = TopoDS::Face(exp.Current());
			//	for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next()) {
			//		TopoDS_Edge edge = TopoDS::Edge(exp.Current());

			//		Standard_Real first, last;
			//		Handle(Geom2d_Curve) c2d = BRep_Tool::CurveOnSurface(edge, face, first, last);

			//		// c2d 是参数空间中的边界线，可以用来分析或绘制UV图
			//	}


			//	// Get the geometric surface of the face
			//	Handle(Geom_Surface) surface = BRep_Tool::Surface(face);

			//	// Check if it's a BSpline surface (NURBS)
			//	Handle(Geom_BSplineSurface) bspline = Handle(Geom_BSplineSurface)::DownCast(surface);
			//	if (!bspline.IsNull()) {
			//		nurbsCount++;

			//		int bsplineU = bspline->UDegree();
			//		int bsplineV = bspline->VDegree();
			//		int bsplineUKnots = bspline->NbUKnots();
			//		int bsplineVKnots = bspline->NbVKnots();
			//		std::cout << "Found NURBS surface #" << nurbsCount << std::endl;
			//		std::cout << "  UDegree: " << bspline->UDegree() << std::endl;
			//		std::cout << "  VDegree: " << bspline->VDegree() << std::endl;
			//		std::cout << "  Num UKnots: " << bspline->NbUKnots() << std::endl;
			//		std::cout << "  Num VKnots: " << bspline->NbVKnots() << std::endl;
			//		std::cout << "  Num Control Points: ("
			//			<< bspline->NbUPoles() << ", "
			//			<< bspline->NbVPoles() << ")" << std::endl;


			//	}
			//}

			//// 创建面与边之间的反向索引：Edge → [Faces]
			//TopTools_IndexedDataMapOfShapeListOfShape edgeToFacesMap;
			//TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeToFacesMap);

			//// 遍历边，找出哪些边被多个面共享
			//for (int i = 1; i <= edgeToFacesMap.Extent(); ++i) {
			//	const TopoDS_Edge& edge = TopoDS::Edge(edgeToFacesMap.FindKey(i));
			//	const TopTools_ListOfShape& faceList = edgeToFacesMap.FindFromIndex(i);

			//	if (faceList.Extent() > 1) {
			//		std::cout << "Edge is shared by " << faceList.Extent() << " faces." << std::endl;

			//		int nurbsFaceCount = 0;
			//		for (TopTools_ListIteratorOfListOfShape it(faceList); it.More(); it.Next()) {
			//			TopoDS_Face face = TopoDS::Face(it.Value());
			//			Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
			//			if (!surf.IsNull()) {
			//				Handle(Geom_BSplineSurface) bs = Handle(Geom_BSplineSurface)::DownCast(surf);
			//				if (!bs.IsNull()) {
			//					nurbsFaceCount++;
			//				}
			//			}
			//		}

			//		if (nurbsFaceCount >= 2) {
			//			std::cout << "  ⮑ At least two of them are NURBS surfaces." << std::endl;
			//		}
			//	}
			//}

			//Rendering
			// 对整个模型进行离散化（细分）
			BRepMesh_IncrementalMesh mesher(shape, 1e-1); // 0.001是线性公差（越小越精细）

			// 遍历所有的面并访问三角剖分
			int objID = 0;
			int idx = 0;
			std::vector<Vertex> verticesData;
			std::vector<int> idxData;
			for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
				TopoDS_Face face = TopoDS::Face(faceExp.Current());
				TopLoc_Location loc;
				Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);
				int faceID = 0;
				if (!triangulation.IsNull()) {
					const Poly_Array1OfTriangle& triangles = triangulation->Triangles();
					std::vector<glm::vec3> positions;
					std::vector<glm::vec3> normals;
					std::vector<glm::vec2> uvs;
					std::vector<int> indices;
					for (int i = 1; i <= triangulation->NbNodes(); ++i) {
						positions.push_back(glm::vec3(triangulation->Node(i).X(), triangulation->Node(i).Y(), triangulation->Node(i).Z()));
						if (triangulation->HasNormals()) {
							normals.push_back(glm::vec3(triangulation->Normal(i).X(), triangulation->Normal(i).Y(), triangulation->Normal(i).Z()));
						}
						else {
							normals.push_back(glm::vec3(0));
						}
						uvs.push_back(glm::vec2(triangulation->UVNode(i).X(), triangulation->UVNode(i).Y()));
					}
					for (int i = triangles.Lower(); i <= triangles.Upper(); ++i) {
						int n1, n2, n3; //node indices
						triangles(i).Get(n1, n2, n3);
						--n1;
						--n2;
						--n3;
						if (triangulation->HasNormals()) {
							normals[n1] = glm::normalize(normals[n1]);
							normals[n2] = glm::normalize(normals[n2]);
							normals[n3] = glm::normalize(normals[n3]);
						}
						else {
							glm::vec3 v1 = positions[n2] - positions[n1];
							glm::vec3 v2 = positions[n3] - positions[n1];
							glm::vec3 normal = glm::normalize(glm::cross(v1, v2));
							normals[n1] = normal;
							normals[n2] = normal;
							normals[n3] = normal;
						}
						Vertex ver1(positions[n1], normals[n1], uvs[n1], objID, faceID);
						Vertex ver2(positions[n2], normals[n2], uvs[n2], objID, faceID);
						Vertex ver3(positions[n3], normals[n3], uvs[n3], objID, faceID);
						verticesData.push_back(ver1);
						verticesData.push_back(ver2);
						verticesData.push_back(ver3);
						idxData.push_back(idx++);
						idxData.push_back(idx++);
						idxData.push_back(idx++);
						++faceID;
					}
					++objID;
				}
			}

			int nullCurve3d = 0;
			int nullCurve2d = 0;
			int build3dCurveFailed = 0;
			int fixAndAdd3dCurve = 0;

			int sampleNum = 20; //每条线的采样点数
			std::vector<std::vector<glm::vec3>> boundaryPoints;
			std::vector<std::vector<glm::vec3>> uvParametricPoints;
			std::vector<glm::vec3> silhouettePoints;
			std::vector<glm::vec3> silhouettePointsDebug;
			std::vector<glm::vec3> debugLines;
			std::vector<std::tuple<Handle(Geom_BSplineSurface), std::vector<std::tuple<Handle(Geom2d_TrimmedCurve), std::vector<std::tuple<Standard_Real, Standard_Real, int>>>>>> surfLinesHidden;
			

			TopExp_Explorer faceExp;
			for (faceExp.Init(shape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
				TopoDS_Face face = TopoDS::Face(faceExp.Current());
				Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
				Handle(Geom_BSplineSurface) bsplineSurf;
				if (surface->IsKind(STANDARD_TYPE(Geom_BSplineSurface))) {
					bsplineSurf = Handle(Geom_BSplineSurface)::DownCast(surface);
				}
				else {
					continue;
				}
				std::vector<std::vector<Handle(Geom2d_BSplineCurve)>> outwireCurves2d;
				//原来edge的t1,t2。该line的长度（新line参数就是0到长度）。隶属于哪一个edge
				std::vector<std::tuple<Handle(Geom2d_TrimmedCurve), Standard_Real, Standard_Real, double, int>> outwirePolylines2d;
				//z1，z2。该line长度。隶属于哪一个edge
				std::vector<std::tuple<Handle(Geom2d_TrimmedCurve), Standard_Real, Standard_Real, Standard_Real, int>> outwirePolylines3d;
				std::vector<std::vector<Handle(Geom2d_BSplineCurve)>> inwireCurves2d;
				int curve3dDowncastFailed = 0;
				int curve2dDowncastFailed = 0;

				//boundary curves
				TopExp_Explorer wireExp;
				for (wireExp.Init(face, TopAbs_WIRE); wireExp.More(); wireExp.Next()) {
					TopoDS_Wire wire = TopoDS::Wire(wireExp.Current());
					std::vector<Handle(Geom2d_BSplineCurve)> currWireCurves2d;

					//使用WireExplorer会正确按照拓扑顺序遍历wire的edge。
					BRepTools_WireExplorer exp(wire);
					int edgeCnt = 0;
					for (; exp.More(); exp.Next()) {
						// e 是接续关系正确的下一条边
						const TopoDS_Edge& edgeTmp = exp.Current();
						TopoDS_Edge edge = TopoDS::Edge(edgeTmp);

						Standard_Real first, last;
						Standard_Real first2d, last2d;
						Handle(Geom_Curve) curve3d = BRep_Tool::Curve(edge, first, last);
						Handle(Geom2d_Curve) curve2d = BRep_Tool::CurveOnSurface(edge, face, first2d, last2d);
						if (curve3d.IsNull()) {
							if (curve2d.IsNull()) {
								continue;
							}
							if (!BRepLib::BuildCurves3d(edge)) {
								ShapeFix_Edge fixEdge;
								if (!fixEdge.FixAddCurve3d(edge)) {
									continue;
								}
							}
							curve3d = BRep_Tool::Curve(edge, first, last);
							/*if (curve3d.IsNull()) {
								continue;
							}*/
						}
						
						//ShapeAnalysis_Edge用于调整每条边的参数走向，使其符合拓扑定义的方向
						ShapeAnalysis_Edge saEdge;
						Standard_Real realFirst, realLast;
						saEdge.Curve3d(edge, curve3d, realFirst, realLast, Standard_True);
						/*if (realFirst > realLast) {
							edge.Reverse();
							saEdge.Curve3d(edge, curve3d, realFirst, realLast, Standard_True);
							curve3d->Reverse();
						};*/
						if (edge.Orientation() == TopAbs_REVERSED) {
							edge.Reverse();
							saEdge.Curve3d(edge, curve3d, realFirst, realLast, Standard_True);
							curve3d->Reverse();
							curve2d->Reverse();
						};
						Standard_Real deflection = 1e-3; // 控制细分精度

						//Handle(Geom_Surface) surface = BRep_Tool::Surface(face);

						//TopoDS_Edge edge2dOnSurf = BRepBuilderAPI_MakeEdge(curve2d, surface);
						/*if (!BRepLib::BuildCurves3d(edge)) {
							continue;
						}*/


						//拿到NURBS
						Handle(Geom_BSplineCurve) bs;
						Handle(Geom_Line) bsline;
						Handle(Geom_BezierCurve) bsbezier;
						Handle(Geom_Circle) bscircle;
						//if (bs = Handle(Geom_BSplineCurve)::DownCast(curve3d)) {
						//	// 直接使用 bs
						//	/*TColgp_Array1OfPnt poles(1, bs->NbPoles());
						//	TColStd_Array1OfReal w(1, bs->NbPoles());
						//	TColStd_Array1OfReal k(1, bs->NbKnots());
						//	TColStd_Array1OfInteger m(1, bs->NbKnots());

						//	bs->Poles(poles);
						//	bs->Weights(w);
						//	bs->Knots(k);
						//	bs->Multiplicities(m);

						//	std::vector<glm::vec3> ctrlPts;
						//	for (Standard_Integer i = poles.Lower(); i <= poles.Upper(); ++i) {
						//		gp_Pnt p = poles(i);
						//		ctrlPts.push_back(glm::vec3(p.X(), p.Y(), p.Z()));
						//	}

						//	std::vector<double> weights;
						//	for (Standard_Integer i = w.Lower(); i <= w.Upper(); ++i) {
						//		weights.push_back(double(w(i)));
						//	}

						//	std::vector<double> knots;
						//	for (Standard_Integer i = k.Lower(); i <= k.Upper(); ++i) {
						//		knots.push_back(double(k(i)));
						//	}

						//	std::vector<double> mults;
						//	for (Standard_Integer i = m.Lower(); i <= m.Upper(); ++i) {
						//		mults.push_back(double(m(i)));
						//	}

						//	int degree;
						//	degree = bs->Degree();

						//	gp_Pnt start = bs->Value(realFirst);
						//	gp_Pnt end = bs->Value(realLast);*/

						//	//GeomAdaptor_Curve不允许uFirst大于uLast。
						//	GeomAdaptor_Curve adaptor(bs, realFirst, realLast);
						//	GCPnts_QuasiUniformDeflection discretizer(adaptor, deflection);
						//	std::vector<glm::vec3> boundaryPointsForCurrEdge;
						//	for (Standard_Integer i = 1; i <= discretizer.NbPoints(); ++i) {
						//		boundaryPointsForCurrEdge.push_back(glm::vec3(discretizer.Value(i).X(), discretizer.Value(i).Y(), discretizer.Value(i).Z()));
						//	}
						//	boundaryPoints.push_back(boundaryPointsForCurrEdge);

						//}
						//else if (bsline = Handle(Geom_Line)::DownCast(curve3d)) {
						//	continue;
						//}
						//else if (bsbezier = Handle(Geom_BezierCurve)::DownCast(curve3d)) {
						//	continue;
						//}
						//else if (bscircle = Handle(Geom_Circle)::DownCast(curve3d)) {
						//	continue;
						//}
						//else {
						//	++curve3dDowncastFailed;
						//	if (Handle(Geom2d_BSplineCurve)::DownCast(curve2d)) {
						//		continue;
						//	}
						//}


						//拿到2d NURBS
						Handle(Geom2d_BSplineCurve) bs2d;
						if (bs2d = Handle(Geom2d_BSplineCurve)::DownCast(curve2d)) {
							/*TColgp_Array1OfPnt2d poles(1, bs2d->NbPoles());
							TColStd_Array1OfReal w(1, bs2d->NbPoles());
							TColStd_Array1OfReal k(1, bs2d->NbKnots());
							TColStd_Array1OfInteger m(1, bs2d->NbKnots());

							bs2d->Poles(poles);
							bs2d->Weights(w);
							bs2d->Knots(k);
							bs2d->Multiplicities(m);

							std::vector<glm::vec2> ctrlPts;
							for (Standard_Integer i = poles.Lower(); i <= poles.Upper(); ++i) {
								gp_Pnt2d p = poles(i);
								ctrlPts.push_back(glm::vec2(p.X(), p.Y()));
							}

							std::vector<double> weights;
							for (Standard_Integer i = w.Lower(); i <= w.Upper(); ++i) {
								weights.push_back(double(w(i)));
							}

							std::vector<double> knots;
							for (Standard_Integer i = k.Lower(); i <= k.Upper(); ++i) {
								knots.push_back(double(k(i)));
							}

							std::vector<double> mults;
							for (Standard_Integer i = m.Lower(); i <= m.Upper(); ++i) {
								mults.push_back(double(m(i)));
							}

							int degree;
							degree = bs2d->Degree();

							gp_Pnt2d start = bs2d->Value(first2d);
							gp_Pnt2d end = bs2d->Value(last2d);*/

							/*GeomAdaptor_Curve adaptor2d(bs2d);
							if (adaptor2d.GetType() == GeomAbs_Line) {
								bs2d = bs2d;
							}*/
							std::vector<glm::vec3> boundaryPointsForCurrEdge;
							Standard_Real step = (last2d - first2d) / sampleNum;
							for (int i = 0; i <= sampleNum; ++i) {
								gp_Pnt2d uv = bs2d->Value(first2d + step * i);
								gp_Pnt p3d = surface->Value(uv.X(), uv.Y());
								boundaryPointsForCurrEdge.push_back(glm::vec3(p3d.X(), p3d.Y(), p3d.Z()));
							}
							boundaryPoints.push_back(boundaryPointsForCurrEdge);
						}
						else {
							++curve2dDowncastFailed;
						}
						++edgeCnt;
						currWireCurves2d.push_back(bs2d);
					}
					if (wire == BRepTools::OuterWire(face)) {
						outwireCurves2d.push_back(currWireCurves2d);
					}
					else {
						inwireCurves2d.push_back(currWireCurves2d);
					}
				}
				mesh.outwireCurves2d = outwireCurves2d;
				//uv parametric points
				int uNum = 2; //num of u curves in middle
				int vNum = 2;

				//if (surface->IsKind(STANDARD_TYPE(Geom_BSplineSurface))) {

				//	Standard_Real u1, u2, v1, v2;
				//	bsplineSurf->Bounds(u1, u2, v1, v2);
				//	if ((bsplineSurf->IsUPeriodic()) || (bsplineSurf->IsVPeriodic())) {
				//		continue;
				//	}

				//	double uStep = (u2 - u1) / (uNum + 1);
				//	double vStep = (v2 - v1) / (vNum + 1);
				//	Standard_Real deflection = 1e-3;

				//	for (int i = 1; i <= vNum; ++i) {
				//		Handle(Geom_Curve) vIsoCurve = bsplineSurf->VIso(v1 + vStep * i);
				//		Handle(Geom_BSplineCurve) vNurbsCurve = Handle(Geom_BSplineCurve)::DownCast(vIsoCurve);
				//		if (vNurbsCurve.IsNull()) {
				//			continue;
				//		};
				//		GeomAdaptor_Curve adaptor(vNurbsCurve);
				//		assert(vNurbsCurve->FirstParameter() < vNurbsCurve->LastParameter());
				//		GCPnts_QuasiUniformDeflection discretizer(adaptor, deflection, vNurbsCurve->FirstParameter(), vNurbsCurve->LastParameter());
				//		if (!discretizer.IsDone()) {
				//			continue;
				//		}
				//		std::vector<glm::vec3> uvPointsForCurrEdge;
				//		for (Standard_Integer i = 1; i <= discretizer.NbPoints(); ++i) {
				//			const gp_Pnt& pt = discretizer.Value(i);
				//			BRepClass_FaceClassifier classifier(face, pt, 1e-6);
				//			if ((classifier.State() == TopAbs_IN) || (classifier.State() == TopAbs_ON)) {
				//				uvPointsForCurrEdge.push_back(glm::vec3(discretizer.Value(i).X(), discretizer.Value(i).Y(), discretizer.Value(i).Z()));
				//			}
				//			//uvPointsForCurrEdge.push_back(glm::vec3(discretizer.Value(i).X(), discretizer.Value(i).Y(), discretizer.Value(i).Z()));
				//		}
				//		uvParametricPoints.push_back(uvPointsForCurrEdge);
				//	}

				//	for (int i = 1; i <= uNum; ++i) {
				//		Handle(Geom_Curve) uIsoCurve = bsplineSurf->UIso(u1 + uStep * i);
				//		Handle(Geom_BSplineCurve) uNurbsCurve = Handle(Geom_BSplineCurve)::DownCast(uIsoCurve);
				//		if (uNurbsCurve.IsNull()) {
				//			continue;
				//		};
				//		GeomAdaptor_Curve adaptor(uNurbsCurve);
				//		assert(uNurbsCurve->FirstParameter() < uNurbsCurve->LastParameter());
				//		GCPnts_QuasiUniformDeflection discretizer(adaptor, deflection, uNurbsCurve->FirstParameter(), uNurbsCurve->LastParameter());
				//		if (!discretizer.IsDone()) {
				//			continue;
				//		}
				//		std::vector<glm::vec3> uvPointsForCurrEdge;
				//		for (Standard_Integer i = 1; i <= discretizer.NbPoints(); ++i) {
				//			//Standard_Real test = discretizer.Parameter(i);
				//			const gp_Pnt& pt = discretizer.Value(i);
				//			BRepClass_FaceClassifier classifier(face, pt, 1e-6);
				//			if ((classifier.State() == TopAbs_IN) || (classifier.State() == TopAbs_ON)) {
				//				uvPointsForCurrEdge.push_back(glm::vec3(discretizer.Value(i).X(), discretizer.Value(i).Y(), discretizer.Value(i).Z()));
				//			}
				//			//uvPointsForCurrEdge.push_back(glm::vec3(discretizer.Value(i).X(), discretizer.Value(i).Y(), discretizer.Value(i).Z()));
				//		}
				//		uvParametricPoints.push_back(uvPointsForCurrEdge);
				//	}
				//}

				//2d uv lines
				uvParametricPoints.clear();
				//最终的等参数线
				std::vector<std::tuple<Handle(Geom2d_Line), std::vector<Standard_Real>>> uvLines;
				if (surface->IsKind(STANDARD_TYPE(Geom_BSplineSurface))) {
					std::vector<glm::vec3> uvPointsForCurrEdge;
					Handle(Geom_BSplineSurface) bsplineSurf = Handle(Geom_BSplineSurface)::DownCast(surface);

					Standard_Real u1, u2, v1, v2;
					bsplineSurf->Bounds(u1, u2, v1, v2);
					Standard_Real deflection = 1e-3;
					if ((bsplineSurf->IsUPeriodic()) || (bsplineSurf->IsVPeriodic())) {
						continue;
					}

					double uStep = (u2 - u1) / (uNum + 1);
					double vStep = (v2 - v1) / (vNum + 1);

					for (int i = 1; i <= vNum; ++i) {
						std::vector<Standard_Real> linet;
						gp_Pnt2d P1(u1 - 0.01, v1 + vStep * i), P2(u2 + 0.01, v1 + vStep * i);
						Handle(Geom2d_Line) vLine = new Geom2d_Line(P1, gp_Dir2d(P2.XY() - P1.XY()));
						for (auto& outwire : outwireCurves2d) {
							for (auto& outedge : outwire) {
								gp_Pnt2d prevPt;
								Standard_Real prevt;
								Geom2dAPI_InterCurveCurve intersector(vLine, outedge, 1.0e-4);
								const Geom2dInt_GInter& ginter = intersector.Intersector();
								if (intersector.NbPoints() > 0) {
									for (Standard_Integer i = 1; i <= intersector.NbPoints(); ++i) {
										gp_Pnt2d pt = intersector.Point(i);
										const IntRes2d_IntersectionPoint& ip = ginter.Point(i);
										Standard_Real t = ip.ParamOnFirst();
										//only compare with previous
										if (i != 1) {
											bool b1 = fequal(t, prevt);
											bool b2 = fequal(pt.X(), prevPt.X());
											bool b3 = fequal(pt.Y(), prevPt.Y());
											if (b1 && b2 && b3) {
												continue;
											}
										}
										linet.push_back(t);
										prevPt = pt;
										prevt = t;
									}
								}
							}
						}
						for (auto& inwire : inwireCurves2d) {
							for (auto& inedge : inwire) {
								gp_Pnt2d prevPt;
								Standard_Real prevt;
								Geom2dAPI_InterCurveCurve intersector(vLine, inedge, 1.0e-4);
								const Geom2dInt_GInter& ginter = intersector.Intersector();
								if (intersector.NbPoints() > 0) {
									for (Standard_Integer i = 1; i <= intersector.NbPoints(); ++i) {
										gp_Pnt2d pt = intersector.Point(i);
										const IntRes2d_IntersectionPoint& ip = ginter.Point(i);
										Standard_Real t = ip.ParamOnFirst();
										//only compare with previous
										if (i != 1) {
											bool b1 = fequal(t, prevt);
											bool b2 = fequal(pt.X(), prevPt.X());
											bool b3 = fequal(pt.Y(), prevPt.Y());
											if (b1 && b2 && b3) {
												continue;
											}
										}
										linet.push_back(t);
										prevPt = pt;
										prevt = t;
									}
								}
							}
						}
						//std::sort(linet.begin(), linet.end(), &VulkanExample::compareByU);
						std::sort(linet.begin(), linet.end());
						uvLines.push_back(std::make_tuple(vLine, linet));
						for (int i = 0; i < linet.size(); i+=2) {
							Standard_Real t1 = linet[i];
							Standard_Real t2 = linet[i + 1];
							Standard_Real step = (t2 - t1) / Standard_Real(sampleNum);
							for (int j = 0; j <= sampleNum; ++j) {
								Standard_Real t = t1 + Standard_Real(j) * step;
								gp_Pnt2d uv2d = vLine->Value(t);
								gp_Pnt uv3d = bsplineSurf->Value(uv2d.X(), uv2d.Y());
								uvPointsForCurrEdge.push_back(glm::vec3(uv3d.X(), uv3d.Y(), uv3d.Z()));
							}
						}
						uvParametricPoints.push_back(uvPointsForCurrEdge);
						uvPointsForCurrEdge.clear();
					}
					for (int i = 1; i <= uNum; ++i) {
						std::vector<Standard_Real> linet;
						gp_Pnt2d P1(u1 + uStep * i, v1 - 0.01), P2(u1 + uStep * i, v2 + 0.01);
						Handle(Geom2d_Line) uLine = new Geom2d_Line(P1, gp_Dir2d(P2.XY() - P1.XY()));
						for (auto& outwire : outwireCurves2d) {
							int edgeCnt = 0;
							for (auto& outedge : outwire) {
								++edgeCnt;
								gp_Pnt2d prevPt;
								Standard_Real prevt;
								Geom2dAPI_InterCurveCurve intersector(uLine, outedge, 1.0e-4);
								const Geom2dInt_GInter& ginter = intersector.Intersector();
								if (intersector.NbPoints() > 0) {
									for (Standard_Integer i = 1; i <= intersector.NbPoints(); ++i) {
										gp_Pnt2d pt = intersector.Point(i);
										const IntRes2d_IntersectionPoint& ip = ginter.Point(i);
										Standard_Real t = ip.ParamOnFirst();
										//only compare with previous
										if (i != 1) {
											bool b1 = fequal(t, prevt);
											bool b2 = fequal(pt.X(), prevPt.X());
											bool b3 = fequal(pt.Y(), prevPt.Y());
											if (b1 && b2 && b3) {
												continue;
											}
										}
										linet.push_back(t);
										prevPt = pt;
										prevt = t;
									}
								}
							}
						}
						for (auto& inwire : inwireCurves2d) {
							for (auto& inedge : inwire) {
								gp_Pnt2d prevPt;
								Standard_Real prevt;
								Geom2dAPI_InterCurveCurve intersector(uLine, inedge, 1.0e-4);
								const Geom2dInt_GInter& ginter = intersector.Intersector();
								if (intersector.NbPoints() > 0) {
									for (Standard_Integer i = 1; i <= intersector.NbPoints(); ++i) {
										gp_Pnt2d pt = intersector.Point(i);
										const IntRes2d_IntersectionPoint& ip = ginter.Point(i);
										Standard_Real t = ip.ParamOnFirst();
										//only compare with previous
										if (i != 1) {
											bool b1 = fequal(t, prevt);
											bool b2 = fequal(pt.X(), prevPt.X());
											bool b3 = fequal(pt.Y(), prevPt.Y());
											if (b1 && b2 && b3) {
												continue;
											}
										}
										linet.push_back(t);
										prevPt = pt;
										prevt = t;
									}
								}
							}
						}
						std::sort(linet.begin(), linet.end());
						uvLines.push_back(std::make_tuple(uLine, linet));
						for (int i = 0; i < linet.size(); i += 2) {
							Standard_Real t1 = linet[i];
							Standard_Real t2 = linet[i + 1];
							Standard_Real step = (t2 - t1) / Standard_Real(sampleNum);
							for (int j = 0; j <= sampleNum; ++j) {
								Standard_Real t = t1 + Standard_Real(j) * step;
								gp_Pnt2d uv2d = uLine->Value(t);
								gp_Pnt uv3d = bsplineSurf->Value(uv2d.X(), uv2d.Y());
								uvPointsForCurrEdge.push_back(glm::vec3(uv3d.X(), uv3d.Y(), uv3d.Z()));
							}
						}
						uvParametricPoints.push_back(uvPointsForCurrEdge);
						uvPointsForCurrEdge.clear();
					}
					
					uvParametricPoints.push_back(uvPointsForCurrEdge);
				}

				//purely uv parametric points
				/*uvParametricPoints.clear();
				if (surface->IsKind(STANDARD_TYPE(Geom_BSplineSurface))) {
					std::vector<glm::vec3> uvPointsForCurrEdge;
					Handle(Geom_BSplineSurface) bsplineSurf = Handle(Geom_BSplineSurface)::DownCast(surface);

					Standard_Real u1, u2, v1, v2;
					bsplineSurf->Bounds(u1, u2, v1, v2);
					Standard_Real deflection = 1e-3;
					if ((bsplineSurf->IsUPeriodic()) || (bsplineSurf->IsVPeriodic())) {
						continue;
					}

					double uStep = (u2 - u1) / (uNum + 1);
					double vStep = (v2 - v1) / (vNum + 1);

					for (int i = 1; i <= vNum; ++i) {
						std::vector<Standard_Real> linet;
						gp_Pnt2d P1(u1, v1 + vStep * i), P2(u2, v1 + vStep * i);
						Standard_Real step = (u2 - u1) / Standard_Real(sampleNum);
						for (int j = 0; j <= sampleNum; ++j) {
							Standard_Real ut = u1 + Standard_Real(j) * step;
							gp_Pnt uv3d = bsplineSurf->Value(ut, v1 + vStep * i);
							uvPointsForCurrEdge.push_back(glm::vec3(uv3d.X(), uv3d.Y(), uv3d.Z()));
						}
						uvParametricPoints.push_back(uvPointsForCurrEdge);
						uvPointsForCurrEdge.clear();
					}
					for (int i = 1; i <= uNum; ++i) {
						std::vector<Standard_Real> linet;
						gp_Pnt2d P1(u1 + uStep * i, v1), P2(u1 + uStep * i, v2);
						Standard_Real ut = u1 + uStep * i;
						Standard_Real step = (v2 - v1) / Standard_Real(sampleNum);
						for (int j = 0; j <= sampleNum; ++j) {
							Standard_Real vt = v1 + Standard_Real(j) * step;
							gp_Pnt uv3d = bsplineSurf->Value(ut, vt);
							uvPointsForCurrEdge.push_back(glm::vec3(uv3d.X(), uv3d.Y(), uv3d.Z()));
						}
						uvParametricPoints.push_back(uvPointsForCurrEdge);
						uvPointsForCurrEdge.clear();
					}
				}*/

				//silhouette
				if (surface->IsKind(STANDARD_TYPE(Geom_BSplineSurface))) {
					Handle(Geom_BSplineSurface) bsplineSurf = Handle(Geom_BSplineSurface)::DownCast(surface);
					/*int bsplineU = bsplineSurf->UDegree();
					int bsplineV = bsplineSurf->VDegree();
					int bsplineUKnots = bsplineSurf->NbUKnots();
					int bsplineVKnots = bsplineSurf->NbVKnots();
					std::cout << "  UDegree: " << bsplineSurf->UDegree() << std::endl;
					std::cout << "  VDegree: " << bsplineSurf->VDegree() << std::endl;
					std::cout << "  Num UKnots: " << bsplineSurf->NbUKnots() << std::endl;
					std::cout << "  Num VKnots: " << bsplineSurf->NbVKnots() << std::endl;
					std::cout << "  Num Control Points: ("
						<< bsplineSurf->NbUPoles() << ", "
						<< bsplineSurf->NbVPoles() << ")" << std::endl;*/
					Standard_Real u1, u2, v1, v2;
					bsplineSurf->Bounds(u1, u2, v1, v2);
					if ((bsplineSurf->IsUPeriodic()) || (bsplineSurf->IsVPeriodic())) {
						continue;
					}
					const int divide = 40;
					double uStep = (u2 - u1) / ((double)divide);
					double vStep = (v2 - v1) / ((double)divide);
					std::array<std::array<float, (divide + 1)>, (divide + 1)> silhouetteNorsArray;
					std::array<std::array<bool, (divide + 1)>, (divide + 1)> silhouetteBoolsArray;
					std::array<std::array<glm::vec3, (divide + 1)>, (divide + 1)> silhouettePointsArray;
					std::vector<std::tuple<Standard_Real, Standard_Real>> silhouette2dPnt;
					std::vector<std::tuple<Standard_Real, Standard_Real>> silhouette2dPntInside;
					std::vector<Handle(Geom2d_TrimmedCurve)> silhouetteGeoms;
					std::vector<std::vector<glm::dvec2>> silhouetteBoundings;  //half length of silhouette2dPnt
					std::vector<int> silhouetteOri;


					std::array<std::array<Quad, divide>, divide> quads;
					glm::vec3 cameraNor = glm::vec3(0.f, 0.f, -1.f);
					//for (int i = 0; i <= divide; ++i) {
					//	Standard_Real u = u1 + double(i) * uStep;
					//	for(int j = 0; j <= divide; ++j) {
					//		Standard_Real v = v1 + double(j) * vStep;
					//		gp_Pnt pt{};
					//		gp_Vec pu{};
					//		gp_Vec pv{};
					//		bsplineSurf->D1(u, v, pt, pu, pv);
					//		BRepClass_FaceClassifier classifier(face, pt, 1e-6);
					//		if ((classifier.State() == TopAbs_IN) || (classifier.State() == TopAbs_ON)) {
					//			gp_Vec norGp = pu.Crossed(pv);
					//			glm::vec3 nor = glm::normalize(glm::vec3(norGp.X(),norGp.Y(),norGp.Z()));
					//			silhouetteNorsArray[i][j] = glm::dot(glm::mat3(camera.matrices.view)*nor, glm::vec3(0, 0, -1));
					//			//silhouettePointsArray[i][j] = glm::vec3(pt.X(), pt.Y(), pt.Z());
					//			silhouetteBoolsArray[i][j] = true;
					//		}
					//		else {
					//			silhouetteBoolsArray[i][j] = false;
					//		}
					//	}
					//}
					for (int i = 0; i < divide; ++i) {
						Standard_Real su1 = u1 + double(i) * uStep;
						Standard_Real su2 = u1 + double(i + 1) * uStep;
						for (int j = 0; j < divide; ++j) {
							Standard_Real sv1 = v1 + double(j) * vStep;
							Standard_Real sv2 = v1 + double(j + 1) * vStep;
							gp_Pnt p1gp = bsplineSurf->Value(su1, sv1);
							GeomLProp_SLProps props(bsplineSurf, su1, sv1, 1, Precision::Confusion());
							gp_Dir p1ngp = props.Normal();
							glm::vec3 p1n(p1ngp.X(), p1ngp.Y(), p1ngp.Z());
							float p1s = glm::dot(glm::mat3(camera.matrices.view) * p1n, glm::vec3(0, 0, 1));

							gp_Pnt p2gp = bsplineSurf->Value(su2, sv1);
							props = GeomLProp_SLProps(bsplineSurf, su2, sv1, 1, Precision::Confusion());
							gp_Dir p2ngp = props.Normal();
							glm::vec3 p2n(p2ngp.X(), p2ngp.Y(), p2ngp.Z());
							float p2s = glm::dot(glm::mat3(camera.matrices.view) * p2n, glm::vec3(0, 0, 1));

							gp_Pnt p3gp = bsplineSurf->Value(su1, sv2);
							props = GeomLProp_SLProps(bsplineSurf, su1, sv2, 1, Precision::Confusion());
							gp_Dir p3ngp = props.Normal();
							glm::vec3 p3n(p3ngp.X(), p3ngp.Y(), p3ngp.Z());
							float p3s = glm::dot(glm::mat3(camera.matrices.view) * p3n, glm::vec3(0, 0, 1));

							gp_Pnt p4gp = bsplineSurf->Value(su2, sv2);
							props = GeomLProp_SLProps(bsplineSurf, su2, sv2, 1, Precision::Confusion());
							gp_Dir p4ngp = props.Normal();
							glm::vec3 p4n(p4ngp.X(), p4ngp.Y(), p4ngp.Z());
							float p4s = glm::dot(glm::mat3(camera.matrices.view) * p4n, glm::vec3(0, 0, 1));

							glm::vec3 p1(p1gp.X(), p1gp.Y(), p1gp.Z());
							glm::vec3 p2(p2gp.X(), p2gp.Y(), p2gp.Z());
							glm::vec3 p3(p3gp.X(), p3gp.Y(), p3gp.Z());
							glm::vec3 p4(p4gp.X(), p4gp.Y(), p4gp.Z());

							silhouettePointsArray[i][j] = p1;
							silhouettePointsArray[i + 1][j] = p2;
							silhouettePointsArray[i][j + 1] = p3;
							silhouettePointsArray[i + 1][j + 1] = p4;
							silhouetteNorsArray[i][j] = p1s;
							silhouetteNorsArray[i + 1][j] = p2s;
							silhouetteNorsArray[i][j + 1] = p3s;
							silhouetteNorsArray[i + 1][j + 1] = p4s;
							silhouettePointsDebug.push_back(p1);
							silhouettePointsDebug.push_back(p2);
							silhouettePointsDebug.push_back(p2);
							silhouettePointsDebug.push_back(p3);
							silhouettePointsDebug.push_back(p3);
							silhouettePointsDebug.push_back(p1);
							silhouettePointsDebug.push_back(p3);
							silhouettePointsDebug.push_back(p4);
							silhouettePointsDebug.push_back(p4);
							silhouettePointsDebug.push_back(p2);
						}

					}
					//marching squares
					for(int i = 0; i < divide; ++i) {
						Standard_Real su1 = u1 + double(i) * uStep;
						Standard_Real su2 = u1 + double(i + 1) * uStep;
						for(int j = 0; j < divide; ++j) {
							Standard_Real sv1 = v1 + double(j) * vStep;
							Standard_Real sv2 = v1 + double(j + 1) * vStep;
							glm::vec3 p1 = silhouettePointsArray[i][j];
							glm::vec3 p2 = silhouettePointsArray[i + 1][j];
							glm::vec3 p3 = silhouettePointsArray[i][j + 1];
							glm::vec3 p4 = silhouettePointsArray[i + 1][j + 1];
							float n1 = silhouetteNorsArray[i][j];
							float n2 = silhouetteNorsArray[i + 1][j];
							float n3 = silhouetteNorsArray[i][j + 1];
							float n4 = silhouetteNorsArray[i + 1][j + 1];
							glm::dvec2 p1d(su1, sv1);
							glm::dvec2 p2d(su2, sv1);
							glm::dvec2 p3d(su1, sv2);
							glm::dvec2 p4d(su2, sv2);
							/*bool b1 = silhouetteBoolsArray[i][j];
							bool b2 = silhouetteBoolsArray[i + 1][j];
							bool b3 = silhouetteBoolsArray[i][j + 1];
							bool b4 = silhouetteBoolsArray[i + 1][j + 1];
							if (!b1 || !b2 || !b3 || !b4) {
								continue;
							}*/
							//tri 1
							if ((n1 == 0.f) && (n2 == 0.f)) {
								silhouettePoints.push_back(p1);
								silhouettePoints.push_back(p2);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 1);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								quads[i][j] = Quad(1, 1, 2);
								if (j == 0) {
									silhouetteOri.push_back(1);
									std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
									silhouetteBoundings.push_back(boundings);
								}
								else {
									glm::dvec2 ptd(su2, sv1 - vStep);
									std::vector<glm::dvec2> l1 = { p2d, p1d, ptd };
									std::vector<glm::dvec2> l2 = { p1d, p2d, p3d };
									silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
									std::vector<glm::dvec2> boundings = { p1d, ptd, p2d, p3d };
									silhouetteBoundings.push_back(boundings);
								}
							}
							if ((n2 == 0.f) && (n3 == 0.f)) {
								silhouettePoints.push_back(p2);
								silhouettePoints.push_back(p3);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								quads[i][j] = Quad(1, 2, 3);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								std::vector<glm::dvec2> l1 = { p2d, p3d, p1d };
								std::vector<glm::dvec2> l2 = { p3d, p2d, p4d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							if ((n1 == 0.f) && (n3 == 0.f)) {
								silhouettePoints.push_back(p1);
								silhouettePoints.push_back(p3);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 1);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								quads[i][j] = Quad(1, 1, 3);
								if (i == 0) {
									silhouetteOri.push_back(1);
									std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
									silhouetteBoundings.push_back(boundings);
								}
								else {
									glm::dvec2 ptd(su1 - uStep, sv2);
									std::vector<glm::dvec2> l1 = { p1d, p3d, ptd };
									std::vector<glm::dvec2> l2 = { p3d, p1d, p2d };
									silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
									std::vector<glm::dvec2> boundings = { p1d, p2d, p3d, ptd };
									silhouetteBoundings.push_back(boundings);
								}
							}
							//0zf
							if ((n1 == 0.f) && (n2 > 0.f) && (n3 < 0.f)) {
								double t23 = n2 / (n2 - n3);
								silhouettePoints.push_back(p1);
								silhouettePoints.push_back(glm::mix(p2, p3, t23));
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 1);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 3, t23);
								quads[i][j] = Quad(1, 1, std::tuple<int, int>(2, 3), t23);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d};
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p23d = glm::mix(p2d, p3d, t23);
								std::vector<glm::dvec2> l1 = { p23d, p1d, p2d };
								std::vector<glm::dvec2> l2 = { p1d, p23d, p3d};
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//0fz
							if((n1 == 0.f) && (n2 < 0.f) && (n3 > 0.f)) {
								double t32 = n3 / (n3 - n2);
								silhouettePoints.push_back(p1);
								silhouettePoints.push_back(glm::mix(p3, p2, t32));
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 1);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 2, t32);
								quads[i][j] = Quad(1, 1, std::tuple<int, int>(3, 2), t32);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p32d = glm::mix(p3d, p2d, t32);
								std::vector<glm::dvec2> l1 = { p32d, p1d, p2d };
								std::vector<glm::dvec2> l2 = { p1d, p32d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//z0f
							if((n1 > 0.f) && (n2 == 0.f) && (n3 < 0.f)) {
								float t13 = n1 / (n1 - n3);
								silhouettePoints.push_back(glm::mix(p1, p3, t13));
								silhouettePoints.push_back(p2);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 1, 3, t13);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								quads[i][j] = Quad(1, std::tuple<int, int>(1, 3), 2, t13);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p13d = glm::mix(p1d, p3d, t13);
								std::vector<glm::dvec2> l1 = { p13d, p2d, p3d };
								std::vector<glm::dvec2> l2 = { p2d, p13d, p1d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//f0z
							if ((n1 < 0.f) && (n2 == 0.f) && (n3 > 0.f)) {
								float t31 = n3 / (n3 - n1);
								silhouettePoints.push_back(glm::mix(p3, p1, t31));
								silhouettePoints.push_back(p2);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 1, t31);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								quads[i][j] = Quad(1, std::tuple<int, int>(3, 1), 2, t31);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p31d = glm::mix(p3d, p1d, t31);
								std::vector<glm::dvec2> l1 = { p31d, p2d, p3d };
								std::vector<glm::dvec2> l2 = { p2d, p31d, p1d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zf0
							if((n1 > 0.f) && (n2 < 0.f) && (n3 == 0.f)) {
								float t12 = n1 / (n1 - n2);
								silhouettePoints.push_back(p3);
								silhouettePoints.push_back(glm::mix(p1, p2, t12));
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 1, 2, t12);
								quads[i][j] = Quad(1, 3, std::tuple<int, int>(1, 2), t12);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p12d = glm::mix(p1d, p2d, t12);
								std::vector<glm::dvec2> l1 = { p12d, p3d, p1d };
								std::vector<glm::dvec2> l2 = { p3d, p12d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//fz0
							if((n1 < 0.f) && (n2 > 0.f) && (n3 == 0.f)) {
								float t21 = n2 / (n2 - n1);
								silhouettePoints.push_back(p3);
								silhouettePoints.push_back(glm::mix(p2, p1, t21));
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 1, t21);
								quads[i][j] = Quad(1, 3, std::tuple<int, int>(2, 1), t21);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p21d = glm::mix(p2d, p1d, t21);
								std::vector<glm::dvec2> l1 = { p21d, p3d, p1d };
								std::vector<glm::dvec2> l2 = { p3d, p21d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zff
							if ((n1 > 0.f) && (n2 < 0.f) && (n3 < 0.f)) {
								float t12 = n1 / (n1 - n2);
								float t13 = n1 / (n1 - n3);
								silhouettePoints.push_back(glm::mix(p1, p2, t12));
								silhouettePoints.push_back(glm::mix(p1, p3, t13));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 1, 2, t12);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 1, 3, t13);
								quads[i][j] = Quad(1, std::tuple<int, int>(1, 2), std::tuple<int, int>(1, 3), t12, t13);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p12d = glm::mix(p1d, p2d, t12);
								glm::dvec2 p13d = glm::mix(p1d, p3d, t13);
								std::vector<glm::dvec2> l1 = { p12d, p13d, p1d };
								std::vector<glm::dvec2> l2 = { p13d, p12d, p2d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//fzf
							if ((n1 < 0.f) && (n2 > 0.f) && (n3 < 0.f)) {
								float t21 = n2 / (n2 - n1);
								float t23 = n2 / (n2 - n3);
								silhouettePoints.push_back(glm::mix(p2, p1, t21));
								silhouettePoints.push_back(glm::mix(p2, p3, t23));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 1, t21);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 3, t23);
								quads[i][j] = Quad(1, std::tuple<int, int>(2, 1), std::tuple<int, int>(2, 3), t21, t23);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p21d = glm::mix(p2d, p1d, t21);
								glm::dvec2 p23d = glm::mix(p2d, p3d, t23);
								std::vector<glm::dvec2> l1 = { p23d, p21d, p2d };
								std::vector<glm::dvec2> l2 = { p21d, p23d, p3d, p1d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//ffz
							if ((n1 < 0.f) && (n2 < 0.f) && (n3 > 0.f)) {
								float t31 = n3 / (n3 - n1);
								float t32 = n3 / (n3 - n2);
								silhouettePoints.push_back(glm::mix(p3, p1, t31));
								silhouettePoints.push_back(glm::mix(p3, p2, t32));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 1, t31);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 2, t32);
								quads[i][j] = Quad(1, std::tuple<int, int>(3, 1), std::tuple<int, int>(3, 2), t31, t32);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p31d = glm::mix(p3d, p1d, t31);
								glm::dvec2 p32d = glm::mix(p3d, p2d, t32);
								std::vector<glm::dvec2> l1 = { p31d, p32d, p3d };
								std::vector<glm::dvec2> l2 = { p32d, p31d, p1d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zzf
							if ((n1 > 0.f) && (n2 > 0.f) && (n3 < 0.f)) {
								float t13 = n1 / (n1 - n3);
								float t23 = n2 / (n2 - n3);
								silhouettePoints.push_back(glm::mix(p1, p3, t13));
								silhouettePoints.push_back(glm::mix(p2, p3, t23));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 1, 3, t13);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 3, t23);
								quads[i][j] = Quad(1, std::tuple<int, int>(1, 3), std::tuple<int, int>(2, 3), t13, t23);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p13d = glm::mix(p1d, p3d, t13);
								glm::dvec2 p23d = glm::mix(p2d, p3d, t23);
								std::vector<glm::dvec2> l1 = { p13d, p23d, p3d };
								std::vector<glm::dvec2> l2 = { p23d, p13d, p1d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zfz
							if ((n1 > 0.f) && (n2 < 0.f) && (n3 > 0.f)) {
								float t12 = n1 / (n1 - n2);
								float t32 = n3 / (n3 - n2);
								silhouettePoints.push_back(glm::mix(p1, p2, t12));
								silhouettePoints.push_back(glm::mix(p3, p2, t32));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 1, 2, t12);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 2, t32);
								quads[i][j] = Quad(1, std::tuple<int, int>(1, 2), std::tuple<int, int>(3, 2), t12, t32);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p12d = glm::mix(p1d, p2d, t12);
								glm::dvec2 p32d = glm::mix(p3d, p2d, t32);
								std::vector<glm::dvec2> l1 = { p32d, p12d, p2d };
								std::vector<glm::dvec2> l2 = { p12d, p32d, p3d, p1d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//fzz
							if ((n1 < 0.f) && (n2 > 0.f) && (n3 > 0.f)) {
								float t21 = n2 / (n2 - n1);
								float t31 = n3 / (n3 - n1);
								silhouettePoints.push_back(glm::mix(p2, p1, t21));
								silhouettePoints.push_back(glm::mix(p3, p1, t31));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 1, t21);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 1, t31);
								quads[i][j] = Quad(1, std::tuple<int, int>(2, 1), std::tuple<int, int>(3, 1), t21, t31);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p21d = glm::mix(p2d, p1d, t21);
								glm::dvec2 p31d = glm::mix(p3d, p1d, t31);
								std::vector<glm::dvec2> l1 = { p21d, p31d, p1d };
								std::vector<glm::dvec2> l2 = { p31d, p21d, p2d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//tri 2
							if ((n2 == 0.f) && (n3 == 0.f)) {
								silhouettePoints.push_back(p2);
								silhouettePoints.push_back(p3);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								quads[i][j] = Quad(2, 2, 3);
								std::vector<glm::dvec2> boundings = { p1d, p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								std::vector<glm::dvec2> l1 = { p3d, p2d, p4d };
								std::vector<glm::dvec2> l2 = { p2d, p3d, p1d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							if ((n3 == 0.f) && (n4 == 0.f)) {
								silhouettePoints.push_back(p3);
								silhouettePoints.push_back(p4);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 4);
								quads[i][j] = Quad(2, 3, 4);
								if (j == (divide - 1)) {
									silhouetteOri.push_back(1);
									std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
									silhouetteBoundings.push_back(boundings);
								}
								else {
									glm::dvec2 ptd(su1, sv2 + vStep);
									std::vector<glm::dvec2> l1 = { p3d, p4d, ptd };
									std::vector<glm::dvec2> l2 = { p4d, p3d, p2d };
									silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
									std::vector<glm::dvec2> boundings = { p2d, p4d, ptd, p3d };
									silhouetteBoundings.push_back(boundings);
								}
							}
							if ((n4 == 0.f) && (n2 == 0.f)) {
								silhouettePoints.push_back(p2);
								silhouettePoints.push_back(p4);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 4);
								quads[i][j] = Quad(2, 2, 4);
								if (i == (divide - 1)) {
									silhouetteOri.push_back(1);
									std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
									silhouetteBoundings.push_back(boundings);
								}
								else {
									glm::dvec2 ptd(su2 + uStep, sv1);
									std::vector<glm::dvec2> l1 = { p4d, p2d, ptd };
									std::vector<glm::dvec2> l2 = { p2d, p4d, p3d };
									silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
									std::vector<glm::dvec2> boundings = { p2d, ptd, p4d, p3d };
									silhouetteBoundings.push_back(boundings);
								}
							}
							//0zf
							if((n2 == 0.f) && (n3 > 0.f) && (n4 < 0.f)) {
								float t34 = n3 / (n3 - n4);
								silhouettePoints.push_back(glm::mix(p3, p4, t34));
								silhouettePoints.push_back(p2);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 4, t34);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								quads[i][j] = Quad(2, std::tuple<int, int>(3, 4), 2, t34);
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p34d = glm::mix(p3d, p4d, t34);
								std::vector<glm::dvec2> l1 = { p34d, p2d, p4d };
								std::vector<glm::dvec2> l2 = { p2d, p34d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//0fz
							if((n2 == 0.f) && (n3 < 0.f) && (n4 > 0.f)) {
								float t43 = n4 / (n4 - n3);
								silhouettePoints.push_back(glm::mix(p4, p3, t43));
								silhouettePoints.push_back(p2);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 4, 3, t43);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 2);
								quads[i][j] = Quad(2, std::tuple<int, int>(4, 3), 2, t43);
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p43d = glm::mix(p4d, p3d, t43);
								std::vector<glm::dvec2> l1 = { p43d, p2d, p4d };
								std::vector<glm::dvec2> l2 = { p2d, p43d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//z0f
							if((n2 > 0.f) && (n3 == 0.f) && (n4 < 0.f)) {
								float t24 = n2 / (n2 - n4);
								silhouettePoints.push_back(glm::mix(p2, p4, t24));
								silhouettePoints.push_back(p3);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 4, t24);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								quads[i][j] = Quad(2, std::tuple<int, int>(2, 4), 3, t24);
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p24d = glm::mix(p2d, p4d, t24);
								std::vector<glm::dvec2> l1 = { p24d, p3d, p2d };
								std::vector<glm::dvec2> l2 = { p3d, p24d, p4d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//f0z
							if((n2 < 0.f) && (n3 == 0.f) && (n4 > 0.f)) {
								float t42 = n4 / (n4 - n2);
								silhouettePoints.push_back(glm::mix(p4, p2, t42));
								silhouettePoints.push_back(p3);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 4, 2, t42);
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 3);
								quads[i][j] = Quad(2, std::tuple<int, int>(4, 2), 3, t42);
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p42d = glm::mix(p4d, p2d, t42);
								std::vector<glm::dvec2> l1 = { p42d, p3d, p2d };
								std::vector<glm::dvec2> l2 = { p3d, p42d, p4d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zf0
							if((n2 > 0.f) && (n3 < 0.f) && (n4 == 0.f)) {
								float t23 = n2 / (n2 - n3);
								silhouettePoints.push_back(p4);
								silhouettePoints.push_back(glm::mix(p2, p3, t23));
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 4);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 3, t23);
								quads[i][j] = Quad(2, 4, std::tuple<int, int>(2, 3), t23);
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p23d = glm::mix(p2d, p3d, t23);
								std::vector<glm::dvec2> l1 = { p23d, p4d, p3d };
								std::vector<glm::dvec2> l2 = { p4d, p23d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//fz0
							if((n2 < 0.f) && (n3 > 0.f) && (n4 == 0.f)) {
								float t32 = n3 / (n3 - n2);
								silhouettePoints.push_back(p4);
								silhouettePoints.push_back(glm::mix(p3, p2, t32));
								silhouettePnt2dEmplace1(silhouette2dPnt, su1, su2, sv1, sv2, 4);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 2, t32);
								quads[i][j] = Quad(2, 4, std::tuple<int, int>(3, 2), t32);
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p32d = glm::mix(p3d, p2d, t32);
								std::vector<glm::dvec2> l1 = { p32d, p4d, p3d };
								std::vector<glm::dvec2> l2 = { p4d, p32d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zff
							if ((n2 > 0.f) && (n3 < 0.f) && (n4 < 0.f)) {
								float t23 = n2 / (n2 - n3);
								float t24 = n2 / (n2 - n4);
								silhouettePoints.push_back(glm::mix(p2, p3, t23));
								silhouettePoints.push_back(glm::mix(p2, p4, t24));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 3, t23);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 4, t24);
								quads[i][j] = Quad(2, std::tuple<int, int>(2, 3), std::tuple<int, int>(2, 4), t23, t24);
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p23d = glm::mix(p2d, p3d, t23);
								glm::dvec2 p24d = glm::mix(p2d, p4d, t24);
								std::vector<glm::dvec2> l1 = { p24d, p23d, p2d };
								std::vector<glm::dvec2> l2 = { p23d, p24d, p4d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//fzf
							if ((n2 < 0.f) && (n3 > 0.f) && (n4 < 0.f)) {
								float t32 = n3 / (n3 - n2);
								float t34 = n3 / (n3 - n4);
								silhouettePoints.push_back(glm::mix(p3, p2, t32));
								silhouettePoints.push_back(glm::mix(p3, p4, t34));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 2, t32);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 4, t34);
								quads[i][j] = Quad(2, std::tuple<int, int>(3, 2), std::tuple<int, int>(3, 4), t32, t34);
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p32d = glm::mix(p3d, p2d, t32);
								glm::dvec2 p34d = glm::mix(p3d, p4d, t34);
								std::vector<glm::dvec2> l1 = { p32d, p34d, p3d };
								std::vector<glm::dvec2> l2 = { p34d, p32d, p2d, p4d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//ffz
							if ((n2 < 0.f) && (n3 < 0.f) && (n4 > 0.f)) {
								float t42 = n4 / (n4 - n2);
								float t43 = n4 / (n4 - n3);
								silhouettePoints.push_back(glm::mix(p4, p2, t42));
								silhouettePoints.push_back(glm::mix(p4, p3, t43));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 4, 2, t42);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 4, 3, t43);
								quads[i][j] = Quad(2, std::tuple<int, int>(4, 2), std::tuple<int, int>(4, 3), t42, t43);
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p43d = glm::mix(p4d, p3d, t43);
								glm::dvec2 p42d = glm::mix(p4d, p2d, t42);
								std::vector<glm::dvec2> l1 = { p43d, p42d, p4d };
								std::vector<glm::dvec2> l2 = { p42d, p43d, p3d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zzf
							if ((n2 > 0.f) && (n3 > 0.f) && (n4 < 0.f)) {
								float t24 = n2 / (n2 - n4);
								float t34 = n3 / (n3 - n4);
								silhouettePoints.push_back(glm::mix(p2, p4, t24));
								silhouettePoints.push_back(glm::mix(p3, p4, t34));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 4, t24);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 4, t34);
								quads[i][j] = Quad(2, std::tuple<int, int>(2, 4), std::tuple<int, int>(3, 4), t24, t34);
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p24d = glm::mix(p2d, p4d, t24);
								glm::dvec2 p34d = glm::mix(p3d, p4d, t34);
								std::vector<glm::dvec2> l1 = { p34d, p24d, p4d };
								std::vector<glm::dvec2> l2 = { p24d, p34d, p3d, p2d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//zfz
							if ((n2 > 0.f) && (n3 < 0.f) && (n4 > 0.f)) {
								float t23 = n2 / (n2 - n3);
								float t43 = n4 / (n4 - n3);
								silhouettePoints.push_back(glm::mix(p2, p3, t23));
								silhouettePoints.push_back(glm::mix(p4, p3, t43));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 2, 3, t23);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 4, 3, t43);
								quads[i][j] = Quad(2, std::tuple<int, int>(2, 3), std::tuple<int, int>(4, 3), t23, t43);
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p23d = glm::mix(p2d, p3d, t23);
								glm::dvec2 p43d = glm::mix(p4d, p3d, t43);
								std::vector<glm::dvec2> l1 = { p23d, p43d, p3d };
								std::vector<glm::dvec2> l2 = { p43d, p23d, p2d, p4d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
							//fzz
							if ((n2 < 0.f) && (n3 > 0.f) && (n4 > 0.f)) {
								float t32 = n3 / (n3 - n2);
								float t42 = n4 / (n4 - n2);
								silhouettePoints.push_back(glm::mix(p3, p2, t32));
								silhouettePoints.push_back(glm::mix(p4, p2, t42));
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 3, 2, t32);
								silhouettePnt2dEmplace2(silhouette2dPnt, su1, su2, sv1, sv2, 4, 2, t42);
								quads[i][j] = Quad(2, std::tuple<int, int>(3, 2), std::tuple<int, int>(4, 2), t32, t42);
								std::vector<glm::dvec2> boundings = { p2d, p4d, p3d };
								silhouetteBoundings.push_back(boundings);
								glm::dvec2 p32d = glm::mix(p3d, p2d, t32);
								glm::dvec2 p42d = glm::mix(p4d, p2d, t42);
								std::vector<glm::dvec2> l1 = { p42d, p32d, p2d };
								std::vector<glm::dvec2> l2 = { p32d, p42d, p4d, p3d };
								silhouetteOri.push_back(calculateSilhouetteOri(l1, l2, bsplineSurf, camera));
							}
						}
						
						/*for (int i = 1; i < divide - 1; ++i) {
							for (int j = 1; j < divide - 1; ++j) {
								glm::vec3 p1 = silhouettePointsArray[i][j];
								glm::vec3 p2 = silhouettePointsArray[i + 1][j];
								glm::vec3 p3 = silhouettePointsArray[i][j + 1];
								glm::vec3 p4 = silhouettePointsArray[i + 1][j + 1];
								silhouettePointsDebug.push_back(p1);
								silhouettePointsDebug.push_back(p2);
								silhouettePointsDebug.push_back(p2);
								silhouettePointsDebug.push_back(p3);
								silhouettePointsDebug.push_back(p3);
								silhouettePointsDebug.push_back(p1);
								silhouettePointsDebug.push_back(p3);
								silhouettePointsDebug.push_back(p4);
								silhouettePointsDebug.push_back(p4);
								silhouettePointsDebug.push_back(p2);
							}
						}*/
					}
					silhouettePoints.clear();
					/*for (auto& uv : silhouette2dPnt) {
						
						gp_Pnt p = bsplineSurf->Value(std::get<0>(uv), std::get<1>(uv));
						silhouettePoints.push_back(glm::vec3(p.X(), p.Y(), p.Z()));
					}*/
					//silhouette与边界边在uv参数域上的交点
					std::vector<Geom2d_Line> silhouetteTrimmedCurves;
					//Geom2d, silhouetteidVec, para_on_silhouette_vec, para_on_outEdge_vec
					std::vector<std::tuple<Handle(Geom2d_BSplineCurve), int, Standard_Real, Standard_Real>> outEdgesInters;
					int silhouetteSegId = 0;
					std::vector<std::tuple<int, int>> osInters;
					for (int i = 0; i < silhouette2dPnt.size(); i+=2) {
						double su1 = std::get<0>(silhouette2dPnt[i]);
						double su2 = std::get<0>(silhouette2dPnt[i + 1]);
						double sv1 = std::get<1>(silhouette2dPnt[i]);
						double sv2 = std::get<1>(silhouette2dPnt[i + 1]);
						gp_Pnt2d P1(su1, sv1), P2(su2, sv2);
						Handle(Geom2d_Line) sline = new Geom2d_Line(P1, gp_Dir2d(P2.XY() - P1.XY()));
						Standard_Real len1 = 0.0, len2 = P1.Distance(P2);
						Handle(Geom2d_TrimmedCurve) ssegment =
							new Geom2d_TrimmedCurve(sline, len1, len2, Standard_True, Standard_True);
						silhouetteGeoms.push_back(ssegment);
						std::vector<std::tuple<Standard_Real, glm::dvec2>> inters;
						int nullInters = 0;
						int wireId = 0;
						for (auto& outwire : outwireCurves2d) {
							int edgeId = 0;
							for (auto& outedge : outwire) {
								gp_Pnt2d prevPt;
								Standard_Real prevt;
								Geom2dAPI_InterCurveCurve intersector(sline, outedge, 1.0e-4);
								const Geom2dInt_GInter& ginter = intersector.Intersector();
								if (intersector.NbPoints() > 0) {
									int interCnt = intersector.NbPoints();
									
									for (Standard_Integer i = 1; i <= intersector.NbPoints(); ++i) {
										gp_Pnt2d pt = intersector.Point(i);
										const IntRes2d_IntersectionPoint& ip = ginter.Point(i);
										Standard_Real t1 = ip.ParamOnFirst();
										Standard_Real t2 = ip.ParamOnSecond();
										//only compare with previous
										if (i != 1) {
											bool b1 = fequal(t1, prevt);
											bool b2 = fequal(pt.X(), prevPt.X());
											bool b3 = fequal(pt.Y(), prevPt.Y());
											if (b1 && b2 && b3) {
												continue;
											}
										}
										if ((t1 >= ssegment->FirstParameter()) && (t1 <= ssegment->LastParameter())) {
											//计算边界边在该点处的tangent
											gp_Dir2d outedgeTan;
											Geom2dLProp_CLProps2d props(outedge, t2, 1, 0.1);
											props.Tangent(outedgeTan);
											glm::dvec2 outedgeT(outedgeTan.X(), outedgeTan.Y());
											outedgeT = glm::normalize(outedgeT);
											inters.emplace_back(t1, outedgeT);
											outEdgesInters.emplace_back(outedge, silhouetteSegId, t1, t2);
										}
										else if(t1 > ssegment->LastParameter()){
											++nullInters;
										}
										prevPt = pt;
										prevt = t1;
										//debug
										/*gp_Pnt2d test2d1 = ssegment->Value(t1);
										gp_Pnt2d test2d = outedge->Value(t2);
										gp_Vec2d tangentDir(outedgeT.X() * 0.0005, outedgeT.Y() * 0.0005);
										gp_Pnt2d testt = test2d.Translated(tangentDir);
										gp_Pnt test3dpnt1 = bsplineSurf->Value(test2d.X(), test2d.Y());
										gp_Pnt test3dpnt2 = bsplineSurf->Value(testt.X(), testt.Y());
										debugLines.push_back(glm::vec3(test3dpnt1.X(), test3dpnt1.Y(), test3dpnt1.Z()));
										debugLines.push_back(glm::vec3(test3dpnt2.X(), test3dpnt2.Y(), test3dpnt2.Z()));*/
									}
								}
								++edgeId;
							}
							++wireId;
						}
						if (inters.size() != 0) {
							//有交一定在内
							double para1 = ssegment->FirstParameter();
							double para2 = ssegment->LastParameter();
							glm::dvec3 sdir(su2 - su1, sv2 - sv1, 0);
							sdir = glm::normalize(sdir);
							for (auto& inter : inters) {
								double t = std::get<0>(inter);
								glm::dvec2 tant = std::get<1>(inter);
								glm::dvec3 tan(tant.x, tant.y, 0);
								double result = glm::cross(sdir, tan).z;
								if (result < 0) {
									if (t < para1) {
										continue;
									}
									gp_Pnt2d val = sline->Value(t);
									su1 = val.X();
									sv1 = val.Y();
									para1 = t;
								}
								else {
									if (t > para2) {
										continue;
									}
									gp_Pnt2d val = sline->Value(t);
									su2 = val.X();
									sv2 = val.Y();
									para2 = t;

								}
							}
							silhouette2dPntInside.emplace_back(su1, sv1);
							silhouette2dPntInside.emplace_back(su2, sv2);
						}
						else {
							if (nullInters % 2 == 0) {
								//外

							}
							else {
								//内
								silhouette2dPntInside.emplace_back(su1, sv1);
								silhouette2dPntInside.emplace_back(su2, sv2);
							}
						}
						++silhouetteSegId;
					}
					for (auto& uv : silhouette2dPntInside) {

						gp_Pnt p = bsplineSurf->Value(std::get<0>(uv), std::get<1>(uv));
						silhouettePoints.push_back(glm::vec3(p.X(), p.Y(), p.Z()));
					}
					//提前记录好silhouette与边界边在uv参数域上的交点
					//edge id, para, visChange
					std::vector<std::tuple<Handle(Geom2d_BSplineCurve), Standard_Real, int>> preComputedOutedgeVisChanges;
					for (auto& outedgeInter : outEdgesInters) {
						Handle(Geom2d_BSplineCurve) outedge = std::get<0>(outedgeInter);
						int silhouetteId = std::get<1>(outedgeInter);
						Standard_Real st = std::get<2>(outedgeInter);
						Standard_Real et = std::get<3>(outedgeInter);
						std::vector<Standard_Real> boundingInters;
						Standard_Real boundingInterParaFirst;
						Standard_Real boundingInterParaLast;
						//边界边与该silhouette的bounding的交点数
						std::vector<glm::dvec2> currSilBounding = silhouetteBoundings[silhouetteId];
						for (int k = 0; k < currSilBounding.size(); ++k) {
							Handle(Geom2d_Line) currBoundLine;
							gp_Pnt2d P1b, P2b;
							if (k == (currSilBounding.size() - 1)) {
								P1b = gp_Pnt2d(currSilBounding[k].x, currSilBounding[k].y);
								P2b = gp_Pnt2d(currSilBounding[0].x, currSilBounding[0].y);
							}
							else {
								P1b = gp_Pnt2d(currSilBounding[k].x, currSilBounding[k].y);
								P2b = gp_Pnt2d(currSilBounding[k + 1].x, currSilBounding[k + 1].y);
							}
							currBoundLine = new Geom2d_Line(P1b, gp_Dir2d(P2b.XY() - P1b.XY()));
							Handle(Geom2d_TrimmedCurve) currBoundLineTrim =
								new Geom2d_TrimmedCurve(currBoundLine, 0, P1b.Distance(P2b), Standard_True, Standard_True);
							//该bounding line与等参数线求交
							Geom2dAPI_InterCurveCurve intersector2(outedge, currBoundLineTrim, 1.0e-4);
							const Geom2dInt_GInter& ginter2 = intersector2.Intersector();
							if (intersector2.NbPoints() > 0) {
								const IntRes2d_IntersectionPoint& ip2 = ginter2.Point(1);
								boundingInters.push_back(ip2.ParamOnFirst());
							}
						}
						std::sort(boundingInters.begin(), boundingInters.end());
						for (auto boundingPara : boundingInters) {
							if (boundingPara < et) {
								boundingInterParaFirst = boundingPara;
							}
							else if (boundingPara > et) {
								boundingInterParaLast = boundingPara;
							}
						}
						//现在知道了边界边和bounding的两个交点（boundingInterParaFirst & boundingInterParaLast），以及和silhouette的交点（st）。
						//利用silhouette的正反信息，判断是否隐藏该等参数线。
						gp_Pnt2d paraFirst = outedge->Value(boundingInterParaFirst);
						gp_Pnt2d paraLast = outedge->Value(boundingInterParaLast);
						gp_Pnt2d pt = outedge->Value(et);
						gp_Pnt2d mid1 = gp_Pnt2d((paraFirst.X() + pt.X()) / 2.0, (paraFirst.Y() + pt.Y()) / 2.0); //
						gp_Pnt2d mid2 = gp_Pnt2d((paraLast.X() + pt.X()) / 2.0, (paraLast.Y() + pt.Y()) / 2.0);

						//mid1的normal
						GeomLProp_SLProps props(bsplineSurf, mid1.X(), mid1.Y(), 1, Precision::Confusion());
						gp_Dir p1ngp = props.Normal();
						glm::vec3 p1n(p1ngp.X(), p1ngp.Y(), p1ngp.Z());
						glm::vec3 test = glm::mat3(camera.matrices.view) * p1n;
						float p1s = glm::dot(glm::mat3(camera.matrices.view) * p1n, glm::vec3(0, 0, 1));
						//mid2的normal
						props = GeomLProp_SLProps(bsplineSurf, mid2.X(), mid2.X(), 1, Precision::Confusion());
						p1ngp = props.Normal();
						glm::vec3 p2n(p1ngp.X(), p1ngp.Y(), p1ngp.Z());
						glm::vec3 test2 = glm::mat3(camera.matrices.view) * p2n;
						float p2s = glm::dot(glm::mat3(camera.matrices.view) * p2n, glm::vec3(0, 0, 1));

						int sOri = silhouetteOri[silhouetteId];
						if (p1s > 0) {
							//mid1在正，mid2在反
							if (sOri == 0) {
								//silhouette为正silhouette
								//mid1可见，mid2不可见
								//visibility我改成负数吧，直观点。
								preComputedOutedgeVisChanges.emplace_back(outedge, et, -1);
							}
							else if (sOri == 2) {
								//silhouette为反silhouette
								//mid1不可见，mid2可见
								preComputedOutedgeVisChanges.emplace_back(outedge, et, 1);
							}
						}
						else {
							//mid1在反，mid2在正
							if (sOri == 0) {
								//silhouette为正silhouette
								//mid1不可见，mid2可见
								preComputedOutedgeVisChanges.emplace_back(outedge, et, 1);
							}
							else if (sOri == 2) {
								//silhouette为反silhouette
								//mid1可见，mid2不可见
								preComputedOutedgeVisChanges.emplace_back(outedge, et, -1);
							}
						}
					}
					//每一个边界边细分为polyline
					outwirePolylines3d;
					for (int i = 0; i < outwireCurves2d[0].size(); ++i) {
						Handle(Geom2d_BSplineCurve) outedge = outwireCurves2d[0][i];
						std::vector<std::tuple<Handle(Geom2d_TrimmedCurve), Standard_Real, Standard_Real, Standard_Real, int>> c2d;
						std::vector<std::tuple<Handle(Geom2d_TrimmedCurve), Standard_Real, Standard_Real, Standard_Real, int>> c3d;
						double firstt = outedge->FirstParameter();
						double lastt = outedge->LastParameter();
						double step = (lastt - firstt) / sampleNum;
						for (int j = 0; j < sampleNum; ++j) {
							double t1 = firstt + (step * j);
							double t2 = firstt + (step * (j + 1));
							gp_Pnt2d P1 = outedge->Value(t1);
							gp_Pnt2d P2 = outedge->Value(t2);
							Handle(Geom2d_Line) sline = new Geom2d_Line(P1, gp_Dir2d(P2.XY() - P1.XY()));
							Standard_Real len1 = 0.0, len2 = P1.Distance(P2);
							Handle(Geom2d_TrimmedCurve) ssegment2d =
								new Geom2d_TrimmedCurve(sline, len1, len2, Standard_True, Standard_True);
							c2d.emplace_back(ssegment2d, t1, t2, len2, i);
							gp_Pnt P1_3d = bsplineSurf->Value(P1.X(), P1.Y());
							gp_Pnt P2_3d = bsplineSurf->Value(P2.X(), P2.Y());
							glm::vec3 p1tmp = glm::mat3(camera.matrices.view) * glm::vec3(P1_3d.X(), P1_3d.Y(), P1_3d.Z());
							glm::vec3 p2tmp = glm::mat3(camera.matrices.view) * glm::vec3(P2_3d.X(), P2_3d.Y(), P2_3d.Z());
							gp_Pnt2d P1View(p1tmp.x, p1tmp.y);
							gp_Pnt2d P2View(p2tmp.x, p2tmp.y);
							double z1 = p1tmp.z;
							double z2 = p2tmp.z;
							Handle(Geom2d_Line) sline2 = new Geom2d_Line(P1View, gp_Dir2d(P2View.XY() - P1View.XY()));
							len2 = P1View.Distance(P2View);
							Handle(Geom2d_TrimmedCurve) ssegment3d =
								new Geom2d_TrimmedCurve(sline2, len1, len2, Standard_True, Standard_True);
							c3d.emplace_back(ssegment3d, z1, z2, len2, i);
						}
						outwirePolylines2d.insert(outwirePolylines2d.end(), c2d.begin(), c2d.end());
						outwirePolylines3d.insert(outwirePolylines3d.end(), c3d.begin(), c3d.end());
					}
					//边界边和silhouette在正交投影中求交
					//

					
					mesh.uvLinePointsHidden.clear();
					std::vector<glm::vec3> tempVec;
					//边界边和边界边在正交投影中求交
					//edge id, para, visChange
					std::vector<std::tuple<int, double, int>> edgeEdgeInters;
					for (int i = 0; i < outwirePolylines3d.size(); ++i) {
						for (int j = i; j < outwirePolylines3d.size(); ++j) {
							if ((j == i) || (j == i - 1) || (j == i + 1) || ((i == 0) && (j == outwirePolylines3d.size() - 1)) || ((i == outwirePolylines3d.size() - 1) && j == 0)) {
								continue;
							}
							else {
								Handle(Geom2d_TrimmedCurve) c1 = std::get<0>(outwirePolylines3d[i]);
								Handle(Geom2d_TrimmedCurve) c2 = std::get<0>(outwirePolylines3d[j]);
								Handle(Geom2d_TrimmedCurve) c1_2d = std::get<0>(outwirePolylines2d[i]);
								Handle(Geom2d_TrimmedCurve) c2_2d = std::get<0>(outwirePolylines2d[j]);
								int id1 = std::get<4>(outwirePolylines3d[i]);
								int id2 = std::get<4>(outwirePolylines3d[j]);
								Geom2dAPI_InterCurveCurve intersector(c1, c2, 1.0e-8);
								const Geom2dInt_GInter& ginter = intersector.Intersector();
								if (intersector.NbPoints() > 0) {
									//默认只有一个交点
									gp_Pnt2d pt = intersector.Point(1);
									const IntRes2d_IntersectionPoint& ip = ginter.Point(1);
									Standard_Real t1 = ip.ParamOnFirst();
									Standard_Real t2 = ip.ParamOnSecond();
									gp_Pnt2d p11p = c1->Value(c1->FirstParameter());
									gp_Pnt2d p12p = c1->Value(c1->LastParameter());
									gp_Pnt2d p21p = c2->Value(c2->FirstParameter());
									gp_Pnt2d p22p = c2->Value(c2->LastParameter());
									glm::dvec2 p11(p11p.X(), p11p.Y());
									glm::dvec2 p12(p12p.X(), p12p.Y());
									glm::dvec2 p21(p21p.X(), p21p.Y());
									glm::dvec2 p22(p22p.X(), p22p.Y());
									double z11 = std::get<1>(outwirePolylines3d[i]);
									double z12 = std::get<2>(outwirePolylines3d[i]);
									double z21 = std::get<1>(outwirePolylines3d[j]);
									double z22 = std::get<2>(outwirePolylines3d[j]);
									double z1 = glm::mix(z11, z12, t1 / (c1->FirstParameter() - c1->LastParameter()));
									double z2 = glm::mix(z21, z22, t2 / (c2->FirstParameter() - c2->LastParameter()));
									double te1 = c1->FirstParameter();
									double te11 = c1->LastParameter();
									double te2 = c2->FirstParameter();
									double te22 = c2->LastParameter();
									double t1orig = glm::mix(std::get<1>(outwirePolylines2d[i]), std::get<2>(outwirePolylines2d[i]), t1 / (c1->FirstParameter() - c1->LastParameter()));
									double t2orig = glm::mix(std::get<1>(outwirePolylines2d[j]), std::get<2>(outwirePolylines2d[j]), t2 / (c2->FirstParameter() - c2->LastParameter()));
									glm::dvec3 c2dir = glm::dvec3(p22, 0) - glm::dvec3(p21, 0);
									glm::dvec3 c1dir = glm::dvec3(p12, 0) - glm::dvec3(p11, 0);
									if (z1 < z2) {
										//z1在后，即c1在后
										//计算c2的normal
										gp_Pnt2d uvorig = c2_2d->Value(t1orig);
										GeomLProp_SLProps props(bsplineSurf, uvorig.X(), uvorig.Y(), 1, Precision::Confusion());
										gp_Dir p1ngp = props.Normal();
										glm::vec3 p1n(p1ngp.X(), p1ngp.Y(), p1ngp.Z());
										float p1s = glm::dot(glm::mat3(camera.matrices.view) * p1n, glm::vec3(0, 0, 1));
										double crossResult = glm::cross(c1dir, c2dir).z;
										if (p1s > 0) {
											//c2正面
											if (crossResult < 0) {
												//c1进入
												edgeEdgeInters.emplace_back(id1, t1orig, -1);
											}
											else {
												//c1走出
												edgeEdgeInters.emplace_back(id1, t1orig, 1);
											}
										}
										else {
											//c2反面
											if (crossResult < 0) {
												//c1走出
												edgeEdgeInters.emplace_back(id1, t1orig, 1);
											}
											else {
												//c1进入
												edgeEdgeInters.emplace_back(id1, t1orig, -1);
											}
											
										}
									}
									else {
										//c2在后
										gp_Pnt2d uvorig = c1_2d->Value(t2orig);
										GeomLProp_SLProps props(bsplineSurf, uvorig.X(), uvorig.Y(), 1, Precision::Confusion());
										gp_Dir p1ngp = props.Normal();
										glm::vec3 p1n(p1ngp.X(), p1ngp.Y(), p1ngp.Z());
										float p1s = glm::dot(glm::mat3(camera.matrices.view) * p1n, glm::vec3(0, 0, 1));
										double crossResult = glm::cross(c2dir, c1dir).z;
										if (p1s > 0) {
											//c1正面
											if (crossResult < 0) {
												//c2进入
												edgeEdgeInters.emplace_back(id2, t2orig, -1);
											}
											else {
												//c2走出
												edgeEdgeInters.emplace_back(id2, t2orig, 1);
											}
										}
										else {
											//c1反面
											if (crossResult < 0) {
												//c2走出
												edgeEdgeInters.emplace_back(id2, t2orig, 1);
											}
											else {
												//c2进入
												edgeEdgeInters.emplace_back(id2, t2orig, -1);
											}

										}
									}
								}
							}
						}
					}
					mesh.uvLinePointsHidden.push_back(tempVec);
					//vis压制到0
					int edgeId = 0;
					int maxVis = 0;
					int startVis = 0;
					std::vector<std::tuple<Handle(Geom2d_BSplineCurve), std::vector<std::tuple<double, double>>, std::vector<int>>> edgeVis;
					for (auto& outwire : outwireCurves2d[0]) {
						std::vector<double> inters;
						std::vector<int> visChange;
						for (auto& inter : edgeEdgeInters) {
							if (std::get<0>(inter) == edgeId) {
								inters.push_back(std::get<1>(inter));
								visChange.push_back(std::get<2>(inter));
							}
						}
						if (inters.size() == 0) {
							std::vector<std::tuple<double, double>> paras = {std::tuple<double, double>(outwire->FirstParameter(), outwire->LastParameter())};
							std::vector<int> vis = {startVis};
							edgeVis.emplace_back(outwire, paras, vis);
						}
						else {
							std::vector<std::tuple<double, double>> paras;
							std::vector<int> vis;
							double lastPara = outwire->FirstParameter();
							for (int i = 0; i < inters.size(); ++i) {
								paras.emplace_back(lastPara, inters[i]);
								vis.push_back(startVis);
								startVis += visChange[i];
								lastPara = inters[i];
							}
							if (startVis > maxVis) {
								maxVis = startVis;
							}
							paras.emplace_back(lastPara, outwire->LastParameter());
							vis.push_back(startVis);
							edgeVis.emplace_back(outwire, paras, vis);
						}
						++edgeId;
					}
					for (auto& edge : edgeVis) {
						for (auto& vis : std::get<2>(edge)) {
							vis -= maxVis;
						}
					}
					//装入vector中
					for (auto& edge : edgeVis) {
						std::vector<int> vis = std::get<2>(edge);
						for (int i = 0; i < vis.size(); ++i) {
							if (vis[i] == 0) {
								std::vector<glm::vec3> tempVec;
								Handle(Geom2d_BSplineCurve) b2d = std::get<0>(edge);
								//区间
								std::tuple<double, double> paras = std::get<1>(edge)[i];
								double t1 = std::get<0>(paras);
								double t2 = std::get<1>(paras);
								double step = (t2 - t1) / sampleNum;
								for (int j = 0; j <= sampleNum; ++j) {
									double t = t1 + step * j;
									gp_Pnt2d p2d = b2d->Value(t);
									gp_Pnt p3d = bsplineSurf->Value(p2d.X(), p2d.Y());
									tempVec.push_back(glm::vec3(p3d.X(), p3d.Y(), p3d.Z()));
								}
								mesh.outedgePoints.push_back(tempVec);
							}
						}
					}

					
					//等参数线和silhouette在uv参数域中求交
					std::vector<std::tuple<Handle(Geom2d_TrimmedCurve), std::vector<std::tuple<Standard_Real, Standard_Real, int>>>> uvLinesHidden;
					for (auto& uvLine : uvLines) {
						std::vector<Standard_Real> linet = std::get<1>(uvLine);
						for (int i = 0; i < linet.size(); i += 2) {
							Standard_Real t1 = linet[i];
							Standard_Real t2 = linet[i + 1];
							std::vector<std::tuple<Standard_Real, int>> visChange;
							Handle(Geom2d_TrimmedCurve) trimmedLine = new Geom2d_TrimmedCurve(std::get<0>(uvLine), t1, t2, Standard_True, Standard_True);
							for (int j = 0; j < silhouette2dPnt.size(); j += 2) {
								double su1 = std::get<0>(silhouette2dPnt[j]);
								double su2 = std::get<0>(silhouette2dPnt[j + 1]);
								double sv1 = std::get<1>(silhouette2dPnt[j]);
								double sv2 = std::get<1>(silhouette2dPnt[j + 1]);
								gp_Pnt2d P1(su1, sv1), P2(su2, sv2);
								Handle(Geom2d_Line) sline = new Geom2d_Line(P1, gp_Dir2d(P2.XY() - P1.XY()));
								Standard_Real len1 = 0.0, len2 = P1.Distance(P2);
								Handle(Geom2d_TrimmedCurve) ssegment =
									new Geom2d_TrimmedCurve(sline, len1, len2, Standard_True, Standard_True);
								//等参数线和每个silhouette segment求交
								Geom2dAPI_InterCurveCurve intersector(trimmedLine, ssegment, 1.0e-4);
								const Geom2dInt_GInter& ginter = intersector.Intersector();
								if (intersector.NbPoints() > 0) {
									int interCnt = intersector.NbPoints();
									gp_Pnt2d pt = intersector.Point(1);
									const IntRes2d_IntersectionPoint& ip = ginter.Point(1);
									Standard_Real it1 = ip.ParamOnFirst();
									Standard_Real it2 = ip.ParamOnSecond();

									std::vector<Standard_Real> boundingInters;
									Standard_Real boundingInterParaFirst, boundingInterParaLast;
									//等参数线与该silhouette的bounding的交点数
									std::vector<glm::dvec2> currSilBounding = silhouetteBoundings[j / 2];
									for (int k = 0; k < currSilBounding.size(); ++k) {
										Handle(Geom2d_Line) currBoundLine;
										gp_Pnt2d P1b, P2b;
										if (k == (currSilBounding.size() - 1)) {
											P1b = gp_Pnt2d(currSilBounding[k].x, currSilBounding[k].y);
											P2b = gp_Pnt2d(currSilBounding[0].x, currSilBounding[0].y);
										}
										else {
											P1b = gp_Pnt2d(currSilBounding[k].x, currSilBounding[k].y);
											P2b = gp_Pnt2d(currSilBounding[k + 1].x, currSilBounding[k + 1].y);
										}
										currBoundLine = new Geom2d_Line(P1b, gp_Dir2d(P2b.XY() - P1b.XY()));
										Handle(Geom2d_TrimmedCurve) currBoundLineTrim =
											new Geom2d_TrimmedCurve(currBoundLine, 0, P1b.Distance(P2b), Standard_True, Standard_True);
										//该bounding line与等参数线求交
										Geom2dAPI_InterCurveCurve intersector2(trimmedLine, currBoundLineTrim, 1.0e-4);
										const Geom2dInt_GInter& ginter2 = intersector2.Intersector();
										if (intersector2.NbPoints() > 0) {
											const IntRes2d_IntersectionPoint& ip2 = ginter2.Point(1);
											boundingInters.push_back(ip2.ParamOnFirst());
										}
									}
									std::sort(boundingInters.begin(), boundingInters.end());
									for (auto boundingPara : boundingInters) {
										if (boundingPara < it1) {
											boundingInterParaFirst = boundingPara;
										}
										else if (boundingPara > it1) {
											boundingInterParaLast = boundingPara;
										}
									}
									//现在知道了等参数线和bounding的两个交点（boundingInterParaFirst & boundingInterParaLast），以及和silhouette的交点（it1）。
									//利用silhouette的正反信息，判断是否隐藏该等参数线。
									gp_Pnt2d paraFirst = trimmedLine->Value(boundingInterParaFirst);
									gp_Pnt2d paraLast = trimmedLine->Value(boundingInterParaLast);
									gp_Pnt2d mid1 = gp_Pnt2d((paraFirst.X() + pt.X()) / 2.0, (paraFirst.Y() + pt.Y()) / 2.0); //
									gp_Pnt2d mid2 = gp_Pnt2d((paraLast.X() + pt.X()) / 2.0, (paraLast.Y() + pt.Y()) / 2.0);

									//mid1的normal
									GeomLProp_SLProps props(bsplineSurf, mid1.X(), mid1.Y(), 1, Precision::Confusion());
									gp_Dir p1ngp = props.Normal();
									glm::vec3 p1n(p1ngp.X(), p1ngp.Y(), p1ngp.Z());
									glm::vec3 test = glm::mat3(camera.matrices.view)* p1n;
									float p1s = glm::dot(glm::mat3(camera.matrices.view) * p1n, glm::vec3(0, 0, 1));
									//mid2的normal
									props = GeomLProp_SLProps(bsplineSurf, mid2.X(), mid2.X(), 1, Precision::Confusion());
									p1ngp = props.Normal();
									glm::vec3 p2n(p1ngp.X(), p1ngp.Y(), p1ngp.Z());
									glm::vec3 test2 = glm::mat3(camera.matrices.view) * p2n;
									float p2s = glm::dot(glm::mat3(camera.matrices.view) * p2n, glm::vec3(0, 0, 1));

									int sOri = silhouetteOri[j / 2];
									if (p1s > 0) {
										//mid1在正，mid2在反
										if (sOri == 0) {
											//silhouette为正silhouette
											//mid1可见，mid2不可见
											//visibility我改成负数吧，直观点。
											visChange.emplace_back(it1, -1);
										}
										else if (sOri == 2) {
											//silhouette为反silhouette
											//mid1不可见，mid2可见
											visChange.emplace_back(it1, 1);
										}
									}
									else {
										//mid1在反，mid2在正
										if (sOri == 0) {
											//silhouette为正silhouette
											//mid1不可见，mid2可见
											visChange.emplace_back(it1, 1);
										}
										else if (sOri == 2) {
											//silhouette为反silhouette
											//mid1可见，mid2不可见
											visChange.emplace_back(it1, -1);
										}
									}
								}
							}
							//算出每个segment的可见性度
							std::vector<std::tuple<Standard_Real, Standard_Real, int>> segVis;
							//sort visChange
							std::sort(visChange.begin(), visChange.end(), compareByTupleFirst);
							if (visChange.size() == 0) {
								segVis.emplace_back(t1, t2, 0);
							}
							else {
								segVis.emplace_back(t1, std::get<0>(visChange[0]), 0);
								int segVisCnt = 0;
								for (int k = 0; k < (visChange.size() - 1); ++k) {
									segVis.emplace_back(std::get<0>(visChange[k]), std::get<0>(visChange[k + 1]), std::get<1>(visChange[k]) + std::get<2>(segVis[segVisCnt]));
									++segVisCnt;
								}
								segVis.emplace_back(std::get<0>(visChange[visChange.size() - 1]), t2, std::get<1>(visChange[visChange.size() - 1]) + std::get<2>(segVis[segVisCnt]));
							}
							//可见度矫正，最可见的设置为0。
							int max = 0;
							for (auto& seg : segVis) {
								if (std::get<2>(seg) > max) {
									max = std::get<2>(seg);
								}
							}
							if (max > 0) {
								for (auto& seg : segVis) {
									std::get<2>(seg) -= max;
								}
							}
							uvLinesHidden.emplace_back(trimmedLine, segVis);
							
						}
					}
					surfLinesHidden.emplace_back(bsplineSurf, uvLinesHidden);
				}
			}
			
			//mesh.uvLinePointsHidden.clear();
			//for(auto& surfline : surfLinesHidden) {
			//	Handle(Geom_Surface) bsplineSurf = std::get<0>(surfline);
			//	std::vector<std::tuple<Handle(Geom2d_TrimmedCurve), std::vector<std::tuple<Standard_Real, Standard_Real, int>>>> linesHidden = std::get<1>(surfline);
			//	for (auto& line : linesHidden) {
			//		Handle(Geom2d_TrimmedCurve) trimmedLine = std::get<0>(line);
			//		std::vector<std::tuple<Standard_Real, Standard_Real, int>> segVis = std::get<1>(line);
			//		for (auto& seg : segVis) {
			//			Standard_Real t1 = std::get<0>(seg);
			//			Standard_Real t2 = std::get<1>(seg);
			//			int vis = std::get<2>(seg);
			//			if (vis == 0) {
			//				//可见
			//				std::vector<glm::vec3> hiddenLinePointsTemp;
			//				double step = (t2 - t1) / sampleNum;
			//				for(int i = 0; i <= sampleNum; ++i) {
			//					double t = t1 + i * step;
			//					gp_Pnt2d p2d = trimmedLine->Value(t);
			//					gp_Pnt p = bsplineSurf->Value(p2d.X(), p2d.Y());
			//					hiddenLinePointsTemp.push_back(glm::vec3(p.X(), p.Y(), p.Z()));
			//				}
			//				mesh.uvLinePointsHidden.push_back(hiddenLinePointsTemp);
			//			}
			//		}
			//	}
			//}

			// 1. 构建投射器
			//可以理解为从(0, 0, 1)映射到(0, 0, 0)的这个方向。
			// 摄像机在从(0,0,0)往(0,0,1)去的无限远点，摄向(0,0,0)
			gp_Ax2 ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
			HLRAlgo_Projector projector(ax2); // 纯正交

			// 2. HLR 算法设置
			Handle(HLRBRep_Algo) algo = new HLRBRep_Algo();
			Standard_Integer nbIso = 2;
			algo->Add(shape, nbIso);
			algo->Projector(projector);
			algo->Update();
			algo->Hide();
			// 3. 提取结果
			HLRBRep_HLRToShape extractor(algo);
			// 可见轮廓
			mesh.uvLinePointsHidden.clear();

			//silhouette
			TopoDS_Shape proj2d_comp = extractor.OutLineVCompound();
			mesh.addOCCHLResults(proj2d_comp);
			
			//边界边
			proj2d_comp = extractor.VCompound();
			mesh.addOCCHLResults(proj2d_comp);

			proj2d_comp = extractor.IsoLineVCompound();
			mesh.addOCCHLResults(proj2d_comp);

			proj2d_comp = extractor.RgNLineVCompound();
			//mesh.addOCCHLResults(proj2d_comp);

			mesh.silhouettePointsDebug = silhouettePointsDebug;
			mesh.setBuffers(vulkanDevice, queue, verticesData, idxData, boundaryPoints, uvParametricPoints, silhouettePoints, silhouettePointsDebug, debugLines);

		};
	};

	//Final Composition Command Buffer
	void buildCommandBuffers()
	{
		VkCommandBufferBeginInfo cmdBufInfo = vks::initializers::commandBufferBeginInfo();

		VkClearValue clearValues[2];
		clearValues[0].color = { { 0.0f, 0.0f, 0.2f, 0.0f } };
		clearValues[1].depthStencil = { 1.0f, 0 };

		VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
		renderPassBeginInfo.renderPass = renderPass;
		renderPassBeginInfo.renderArea.offset.x = 0;
		renderPassBeginInfo.renderArea.offset.y = 0;
		renderPassBeginInfo.renderArea.extent.width = width;
		renderPassBeginInfo.renderArea.extent.height = height;
		renderPassBeginInfo.clearValueCount = 2;
		renderPassBeginInfo.pClearValues = clearValues;

		for (int32_t i = 0; i < drawCmdBuffers.size(); ++i)
		{
			renderPassBeginInfo.framebuffer = frameBuffers[i];

			VK_CHECK_RESULT(vkBeginCommandBuffer(drawCmdBuffers[i], &cmdBufInfo));

			vkCmdBeginRenderPass(drawCmdBuffers[i], &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

			VkViewport viewport = vks::initializers::viewport((float)width, -(float)height, 0.0f, 1.0f);
			viewport.y = (float)height;
			vkCmdSetViewport(drawCmdBuffers[i], 0, 1, &viewport);

			VkRect2D scissor = vks::initializers::rect2D(width, height, 0, 0);
			vkCmdSetScissor(drawCmdBuffers[i], 0, 1, &scissor);

			vkCmdBindDescriptorSets(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets.composition, 0, nullptr);

			vkCmdBindPipeline(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.composition);

			// Final composition
			// This is done by simply drawing a full screen quad
			// The fragment shader then combines the deferred attachments into the final image
			// Note: Also used for debug display if debugDisplayTarget > 0
			vkCmdDraw(drawCmdBuffers[i], 3, 1, 0, 0);

			drawUI(drawCmdBuffers[i]);

			vkCmdEndRenderPass(drawCmdBuffers[i]);

			VK_CHECK_RESULT(vkEndCommandBuffer(drawCmdBuffers[i]));
		}
	}

	void setupDescriptors()
	{
		// Pool
		std::vector<VkDescriptorPoolSize> poolSizes = {
			vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 8),
			vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 11),
			vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 3)
		};
		VkDescriptorPoolCreateInfo descriptorPoolInfo = vks::initializers::descriptorPoolCreateInfo(poolSizes, 6);
		VK_CHECK_RESULT(vkCreateDescriptorPool(device, &descriptorPoolInfo, nullptr, &descriptorPool));

		// Layouts
		std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {
			// Binding 0 : Vertex shader uniform buffer
			vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_VERTEX_BIT, 0),
			// Binding 1 : Position texture target / Scene colormap
			vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 1),
			// Binding 2 : Normals texture target
			vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 2),
			// Binding 3 : Albedo texture target
			vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 3),
			// Binding 4 : Fragment shader uniform buffer
			vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_FRAGMENT_BIT, 4),
			// Binding 5 : Storage buffer face info
			vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_FRAGMENT_BIT, 5),
			// Binding 6 : Storage buffer face data
			vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_FRAGMENT_BIT, 6),
			// Binding 7 : Storage buffer face nor
			vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_FRAGMENT_BIT, 7),
			// Binding 8 : Edge texture target
			vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 8),
			// Binding 9 : Locked Edge texture target
			vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 9),
			// Binding 10: Edge texture target 2
			vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 10),
			// Binding 11: Hidden line texture target
			vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 11)
		};
		VkDescriptorSetLayoutCreateInfo descriptorLayout = vks::initializers::descriptorSetLayoutCreateInfo(setLayoutBindings);
		VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorLayout, nullptr, &descriptorSetLayout));

		// Sets
		std::vector<VkWriteDescriptorSet> writeDescriptorSets;
		VkDescriptorSetAllocateInfo allocInfo = vks::initializers::descriptorSetAllocateInfo(descriptorPool, &descriptorSetLayout, 1);

		// Image descriptors for the offscreen color attachments
		VkDescriptorImageInfo texDescriptorPosition =
			vks::initializers::descriptorImageInfo(
				colorSampler,
				offScreenFrameBuf.position.view,
				VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

		VkDescriptorImageInfo texDescriptorNormal =
			vks::initializers::descriptorImageInfo(
				colorSampler,
				offScreenFrameBuf.normal.view,
				VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

		VkDescriptorImageInfo texDescriptorAlbedo =
			vks::initializers::descriptorImageInfo(
				colorSampler,
				offScreenFrameBuf.albedo.view,
				VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

		VkDescriptorImageInfo texDescriptorEdge =
			vks::initializers::descriptorImageInfo(
				colorSampler,
				edgeFrameBuf.position.view,
				VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

		VkDescriptorImageInfo texDescriptorEdge2 =
			vks::initializers::descriptorImageInfo(
				colorSampler,
				edgeFrameBuf.normal.view,
				VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

		VkDescriptorImageInfo texDescriptorLockedEdge =
			vks::initializers::descriptorImageInfo(
				colorSampler,
				lockedEdgeFrameBuf.position.view,
				VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

		VkDescriptorImageInfo texDescriptorHiddenLine =
			vks::initializers::descriptorImageInfo(
				colorSampler,
				hiddenLineFrameBuf.position.view,
				VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

		// Deferred composition
		VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &descriptorSets.composition));
		writeDescriptorSets = {
			// Binding 1 : Position texture target
			vks::initializers::writeDescriptorSet(descriptorSets.composition, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, &texDescriptorPosition),
			// Binding 2 : Normals texture target
			vks::initializers::writeDescriptorSet(descriptorSets.composition, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 2, &texDescriptorNormal),
			// Binding 3 : Albedo texture target
			vks::initializers::writeDescriptorSet(descriptorSets.composition, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 3, &texDescriptorAlbedo),
			// Binding 4 : Fragment shader uniform buffer
			vks::initializers::writeDescriptorSet(descriptorSets.composition, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 4, &uniformBuffers.composition.descriptor),
			// Binding 5 : Neighbor Face Info Buffer
			vks::initializers::writeDescriptorSet(descriptorSets.composition, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 5, &mesh.faceInfoBuffer.descriptor),
			// Binding 6 : Neighbor Face Data Buffer
			vks::initializers::writeDescriptorSet(descriptorSets.composition, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 6, &mesh.faceDataBuffer.descriptor),
			// Binding 7 : Neighbor Face Nor Buffer
			vks::initializers::writeDescriptorSet(descriptorSets.composition, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 7, &mesh.faceNorBuffer.descriptor),
			// Binding 8 : Extra texture target
			vks::initializers::writeDescriptorSet(descriptorSets.composition, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 8, &texDescriptorEdge),
			// Binding 9 : Extra locked texture target
			vks::initializers::writeDescriptorSet(descriptorSets.composition, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 9, &texDescriptorLockedEdge),
			// Binding 10: Extra edge texture target 2
			vks::initializers::writeDescriptorSet(descriptorSets.composition, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 10, &texDescriptorEdge2),
			// Binding 11: Hidden line texture target
			vks::initializers::writeDescriptorSet(descriptorSets.composition, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 11, &texDescriptorHiddenLine)
		};
		vkUpdateDescriptorSets(device, static_cast<uint32_t>(writeDescriptorSets.size()), writeDescriptorSets.data(), 0, nullptr);

		// Offscreen (scene)

		// Model
		VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &descriptorSets.model));
		writeDescriptorSets = {
			// Binding 0: Vertex shader uniform buffer
			vks::initializers::writeDescriptorSet(descriptorSets.model, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 0, &uniformBuffers.offscreen.descriptor),
			// Binding 1: Color map
			vks::initializers::writeDescriptorSet(descriptorSets.model, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, &textures.model.colorMap.descriptor),
			// Binding 2: Normal map
			vks::initializers::writeDescriptorSet(descriptorSets.model, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 2, &textures.model.normalMap.descriptor),
		};
		vkUpdateDescriptorSets(device, static_cast<uint32_t>(writeDescriptorSets.size()), writeDescriptorSets.data(), 0, nullptr);

		// Background
		VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &descriptorSets.floor));
		writeDescriptorSets = {
			// Binding 0: Vertex shader uniform buffer
			vks::initializers::writeDescriptorSet(descriptorSets.floor, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 0, &uniformBuffers.offscreen.descriptor),
			// Binding 1: Color map
			vks::initializers::writeDescriptorSet(descriptorSets.floor, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, &textures.floor.colorMap.descriptor),
			// Binding 2: Normal map
			vks::initializers::writeDescriptorSet(descriptorSets.floor, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 2, &textures.floor.normalMap.descriptor)
		};
		vkUpdateDescriptorSets(device, static_cast<uint32_t>(writeDescriptorSets.size()), writeDescriptorSets.data(), 0, nullptr);

		// Edge
		VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &descriptorSets.edge));
		writeDescriptorSets = {
			// Binding 0: Vertex shader uniform buffer
			vks::initializers::writeDescriptorSet(descriptorSets.edge, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 0, &uniformBuffers.edge.descriptor),
			// Binding 4: Fragment shader uniform buffer
			vks::initializers::writeDescriptorSet(descriptorSets.edge, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 4, &uniformBuffers.edgeFrag.descriptor),
		};
		vkUpdateDescriptorSets(device, static_cast<uint32_t>(writeDescriptorSets.size()), writeDescriptorSets.data(), 0, nullptr);

		//LockedEdge
		VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &descriptorSets.lockedEdge));
		writeDescriptorSets = {
			// Binding 0: Vertex shader uniform buffer
			vks::initializers::writeDescriptorSet(descriptorSets.lockedEdge, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 0, &uniformBuffers.lockedEdge.descriptor),
			// Binding 4: Fragment shader uniform buffer
			vks::initializers::writeDescriptorSet(descriptorSets.lockedEdge, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 4, &uniformBuffers.lockedEdgeFrag.descriptor),
		};
		vkUpdateDescriptorSets(device, static_cast<uint32_t>(writeDescriptorSets.size()), writeDescriptorSets.data(), 0, nullptr);

		//HiddenLine
		VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &descriptorSets.hiddenLine));
		writeDescriptorSets = {
			// Binding 0: Vertex shader uniform buffer
			vks::initializers::writeDescriptorSet(descriptorSets.hiddenLine, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 0, &uniformBuffers.hiddenLine.descriptor),
		};
		vkUpdateDescriptorSets(device, static_cast<uint32_t>(writeDescriptorSets.size()), writeDescriptorSets.data(), 0, nullptr);
	}

	void preparePipelines()
	{
		// Pipeline layout
		VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = vks::initializers::pipelineLayoutCreateInfo(&descriptorSetLayout, 1);
		VkPushConstantRange pushConstantRange = vks::initializers::pushConstantRange(VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, sizeof(PushValue), 0);
		pipelineLayoutCreateInfo.pushConstantRangeCount = 1;
		pipelineLayoutCreateInfo.pPushConstantRanges = &pushConstantRange;
		VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pipelineLayoutCreateInfo, nullptr, &pipelineLayout));

		// Pipelines
		VkPipelineInputAssemblyStateCreateInfo inputAssemblyState = vks::initializers::pipelineInputAssemblyStateCreateInfo(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, 0, VK_FALSE);
		VkPipelineRasterizationStateCreateInfo rasterizationState = vks::initializers::pipelineRasterizationStateCreateInfo(VK_POLYGON_MODE_FILL, VK_CULL_MODE_BACK_BIT, VK_FRONT_FACE_COUNTER_CLOCKWISE, 0);
		VkPipelineColorBlendAttachmentState blendAttachmentState = vks::initializers::pipelineColorBlendAttachmentState(0xf, VK_FALSE);
		VkPipelineColorBlendStateCreateInfo colorBlendState = vks::initializers::pipelineColorBlendStateCreateInfo(1, &blendAttachmentState);
		VkPipelineDepthStencilStateCreateInfo depthStencilState = vks::initializers::pipelineDepthStencilStateCreateInfo(VK_TRUE, VK_TRUE, VK_COMPARE_OP_LESS_OR_EQUAL);
		VkPipelineViewportStateCreateInfo viewportState = vks::initializers::pipelineViewportStateCreateInfo(1, 1, 0);
		VkPipelineMultisampleStateCreateInfo multisampleState = vks::initializers::pipelineMultisampleStateCreateInfo(VK_SAMPLE_COUNT_1_BIT, 0);
		std::vector<VkDynamicState> dynamicStateEnables = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
		VkPipelineDynamicStateCreateInfo dynamicState = vks::initializers::pipelineDynamicStateCreateInfo(dynamicStateEnables);
		std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;

		VkGraphicsPipelineCreateInfo pipelineCI = vks::initializers::pipelineCreateInfo(pipelineLayout, renderPass);
		pipelineCI.pInputAssemblyState = &inputAssemblyState;
		pipelineCI.pRasterizationState = &rasterizationState;
		pipelineCI.pColorBlendState = &colorBlendState;
		pipelineCI.pMultisampleState = &multisampleState;
		pipelineCI.pViewportState = &viewportState;
		pipelineCI.pDepthStencilState = &depthStencilState;
		pipelineCI.pDynamicState = &dynamicState;
		pipelineCI.stageCount = static_cast<uint32_t>(shaderStages.size());
		pipelineCI.pStages = shaderStages.data();

		// Final fullscreen composition pass pipeline
		rasterizationState.cullMode = VK_CULL_MODE_BACK_BIT;
		shaderStages[0] = loadShader(getShadersPath() + "occ/deferred.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getShadersPath() + "occ/deferred.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
		// Empty vertex input state, vertices are generated by the vertex shader
		VkPipelineVertexInputStateCreateInfo emptyInputState = vks::initializers::pipelineVertexInputStateCreateInfo();
		pipelineCI.pVertexInputState = &emptyInputState;
		VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipelines.composition));

		//self-added
		VkPipelineVertexInputStateCreateInfo pipelineVertexInputStateCreateInfo{};
		auto bindingDescription = Vertex::getBindingDescription();
		auto attributeDescriptions = Vertex::getAttributeDescriptions();
		pipelineVertexInputStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
		pipelineVertexInputStateCreateInfo.vertexBindingDescriptionCount = 1;
		pipelineVertexInputStateCreateInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescriptions.size()); //2
		pipelineVertexInputStateCreateInfo.pVertexBindingDescriptions = &bindingDescription;
		pipelineVertexInputStateCreateInfo.pVertexAttributeDescriptions = attributeDescriptions.data();
		pipelineCI.pVertexInputState = &pipelineVertexInputStateCreateInfo;
		// Vertex input state from glTF model for pipeline rendering models
		//pipelineCI.pVertexInputState = vkglTF::Vertex::getPipelineVertexInputState({ vkglTF::VertexComponent::Position, vkglTF::VertexComponent::UV, vkglTF::VertexComponent::Color, vkglTF::VertexComponent::Normal, vkglTF::VertexComponent::Tangent });

		//cullMode should change to no culling, so back face can also get into depth attachment.
		rasterizationState.cullMode = VK_CULL_MODE_NONE;

		// Offscreen pipeline
		shaderStages[0] = loadShader(getShadersPath() + "occ/mrt.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getShadersPath() + "occ/mrt.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);

		// Separate render pass
		pipelineCI.renderPass = offScreenFrameBuf.renderPass;

		// Blend attachment states required for all color attachments
		// This is important, as color write mask will otherwise be 0x0 and you
		// won't see anything rendered to the attachment
		std::array<VkPipelineColorBlendAttachmentState, 3> blendAttachmentStates = {
			vks::initializers::pipelineColorBlendAttachmentState(0xf, VK_FALSE),
			vks::initializers::pipelineColorBlendAttachmentState(0xf, VK_FALSE),
			vks::initializers::pipelineColorBlendAttachmentState(0xf, VK_FALSE)
		};

		colorBlendState.attachmentCount = static_cast<uint32_t>(blendAttachmentStates.size());
		colorBlendState.pAttachments = blendAttachmentStates.data();

		VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipelines.offscreen));


		std::array<VkPipelineColorBlendAttachmentState, 2> blendAttachmentStates2 = {
			vks::initializers::pipelineColorBlendAttachmentState(0xf, VK_FALSE),
			vks::initializers::pipelineColorBlendAttachmentState(0xf, VK_FALSE)
		};
		colorBlendState.attachmentCount = static_cast<uint32_t>(blendAttachmentStates2.size());
		colorBlendState.pAttachments = blendAttachmentStates2.data();
		inputAssemblyState.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
		rasterizationState.lineWidth = 5.f;
		//inputAssemblyState.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
		shaderStages[0] = loadShader(getShadersPath() + "occ/edge.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getShadersPath() + "occ/edge.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
		pipelineCI.renderPass = edgeFrameBuf.renderPass;
		VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipelines.edge));


		colorBlendState = vks::initializers::pipelineColorBlendStateCreateInfo(1, &blendAttachmentState);
		rasterizationState.lineWidth = 5.f;
		depthStencilState.depthWriteEnable = VK_TRUE;
		depthStencilState.depthTestEnable = VK_TRUE;
		shaderStages[0] = loadShader(getShadersPath() + "occ/lockededge.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getShadersPath() + "occ/lockededge.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
		pipelineCI.renderPass = lockedEdgeFrameBuf.renderPass;
		VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipelines.lockedEdge));
		
		rasterizationState.lineWidth = 5.f;
		shaderStages[0] = loadShader(getShadersPath() + "occ/hiddenline.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getShadersPath() + "occ/hiddenline.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
		pipelineCI.renderPass = hiddenLineFrameBuf.renderPass;
		VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipelines.hiddenLine));
	}

	// Prepare and initialize uniform buffer containing shader uniforms
	void prepareUniformBuffers()
	{
		// Offscreen vertex shader
		VK_CHECK_RESULT(vulkanDevice->createBuffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &uniformBuffers.offscreen, sizeof(UniformDataOffscreen)));

		// Edge vertex shader
		VK_CHECK_RESULT(vulkanDevice->createBuffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &uniformBuffers.edge, sizeof(UniformDataEdge)))

		// Locked Edge vertex shader
		VK_CHECK_RESULT(vulkanDevice->createBuffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &uniformBuffers.lockedEdge, sizeof(UniformDataLockedEdge)))

		// Deferred fragment shader
		VK_CHECK_RESULT(vulkanDevice->createBuffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &uniformBuffers.composition, sizeof(UniformDataComposition)));

		// Edge frag shader
		VK_CHECK_RESULT(vulkanDevice->createBuffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &uniformBuffers.edgeFrag, sizeof(UniformDataEdgeFrag)));

		// Locked Edge frag shader
		VK_CHECK_RESULT(vulkanDevice->createBuffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &uniformBuffers.lockedEdgeFrag, sizeof(UniformDataLockedEdgeFrag)));
		
		// Hidden Line vertex shader
		VK_CHECK_RESULT(vulkanDevice->createBuffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &uniformBuffers.hiddenLine, sizeof(UniformDataHiddenLine)));

		// Map persistent
		VK_CHECK_RESULT(uniformBuffers.offscreen.map());
		VK_CHECK_RESULT(uniformBuffers.edge.map());
		VK_CHECK_RESULT(uniformBuffers.lockedEdge.map());
		VK_CHECK_RESULT(uniformBuffers.composition.map());
		VK_CHECK_RESULT(uniformBuffers.edgeFrag.map());
		VK_CHECK_RESULT(uniformBuffers.lockedEdgeFrag.map());
		VK_CHECK_RESULT(uniformBuffers.hiddenLine.map());

		// Setup instanced model positions
		uniformDataOffscreen.instancePos[0] = glm::vec4(0.0f);
		uniformDataOffscreen.instancePos[1] = glm::vec4(-4.0f, 0.0, -4.0f, 0.0f);
		uniformDataOffscreen.instancePos[2] = glm::vec4(4.0f, 0.0, -4.0f, 0.0f);

		// Update
		updateUniformBufferOffscreen();
		updateUniformBufferEdge();
		updateUniformBufferLockedEdge();
		updateUniformBufferComposition();
		updateUniformBufferHiddenLine();
	}

	// Update matrices used for the offscreen rendering of the scene
	void updateUniformBufferOffscreen()
	{
		if (camera.orthographic) {
			uniformDataOffscreen.projection = glm::ortho(camera.orthoLeft, camera.orthoRight, camera.orthoBottom, camera.orthoTop, camera.znear, camera.zfar);
		}
		else {
			uniformDataOffscreen.projection = glm::perspective(glm::radians(camera.fov), (float)width / (float)height, camera.znear, camera.zfar);
		}
		uniformDataOffscreen.view = camera.matrices.view;
		uniformDataOffscreen.model = glm::mat4(1.0f);
		memcpy(uniformBuffers.offscreen.mapped, &uniformDataOffscreen, sizeof(UniformDataOffscreen));
	}

	void updateUniformBufferEdge() {
		if (camera.orthographic) {
			uniformDataEdge.projection = glm::ortho(camera.orthoLeft, camera.orthoRight, camera.orthoBottom, camera.orthoTop, camera.znear, camera.zfar);
		}
		else {
			uniformDataEdge.projection = glm::perspective(glm::radians(camera.fov), (float)width / (float)height, camera.znear, camera.zfar);
			//uniformDataEdge.projection = camera.matrices.perspective;
		}
		uniformDataEdge.view = camera.matrices.view;
		uniformDataEdge.model = glm::mat4(1.0f);
		uniformDataEdge.orthographic = camera.orthographic;
		uniformDataEdgeFrag.color = triColor;
		memcpy(uniformBuffers.edge.mapped, &uniformDataEdge, sizeof(UniformDataEdge));
		memcpy(uniformBuffers.edgeFrag.mapped, &uniformDataEdgeFrag, sizeof(UniformDataEdgeFrag));
	}

	void updateUniformBufferLockedEdge() {
		if (camera.orthographic) {
			uniformDataLockedEdge.projection = glm::ortho(camera.orthoLeft, camera.orthoRight, camera.orthoBottom, camera.orthoTop, camera.znear, camera.zfar);
		}
		else {
			uniformDataLockedEdge.projection = glm::perspective(glm::radians(camera.fov), (float)width / (float)height, camera.znear, camera.zfar);
			//uniformDataLockedEdge.projection = camera.matrices.perspective;
		}
		uniformDataLockedEdge.view = camera.matrices.view;
		uniformDataLockedEdge.model = glm::mat4(1.0f);
		uniformDataLockedEdgeFrag.color = triColor;
		memcpy(uniformBuffers.lockedEdge.mapped, &uniformDataLockedEdge, sizeof(UniformDataLockedEdge));
		memcpy(uniformBuffers.lockedEdgeFrag.mapped, &uniformDataLockedEdgeFrag, sizeof(UniformDataLockedEdgeFrag));
	}

	void updateUniformBufferHiddenLine() {
		if (camera.orthographic) {
			uniformDataHiddenLine.projection = glm::ortho(camera.orthoLeft, camera.orthoRight, camera.orthoBottom, camera.orthoTop, camera.znear, camera.zfar);
		}
		else {
			uniformDataHiddenLine.projection = glm::perspective(glm::radians(camera.fov), (float)width / (float)height, camera.znear, camera.zfar);
			//uniformDataLockedEdge.projection = camera.matrices.perspective;
		}
		uniformDataHiddenLine.view = camera.matrices.view;
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				if (i == j) {
					uniformDataHiddenLine.view[i][j] = 1;
				}
				else {
					uniformDataHiddenLine.view[i][j] = 0;
				}
			}
		}
		uniformDataHiddenLine.model = glm::mat4(1.0f);
		memcpy(uniformBuffers.hiddenLine.mapped, &uniformDataHiddenLine, sizeof(UniformDataHiddenLine));
	}

	// Update lights and parameters passed to the composition shaders
	void updateUniformBufferComposition()
	{
		// White
		uniformDataComposition.lights[0].position = glm::vec4(0.0f, 0.0f, 1.0f, 0.0f);
		uniformDataComposition.lights[0].color = glm::vec3(1.5f);
		uniformDataComposition.lights[0].radius = 15.0f * 0.25f;
		// Red
		uniformDataComposition.lights[1].position = glm::vec4(-2.0f, 0.0f, 0.0f, 0.0f);
		uniformDataComposition.lights[1].color = glm::vec3(1.0f, 0.0f, 0.0f);
		uniformDataComposition.lights[1].radius = 15.0f;
		// Blue
		uniformDataComposition.lights[2].position = glm::vec4(2.0f, -1.0f, 0.0f, 0.0f);
		uniformDataComposition.lights[2].color = glm::vec3(0.0f, 0.0f, 2.5f);
		uniformDataComposition.lights[2].radius = 5.0f;
		// Yellow
		uniformDataComposition.lights[3].position = glm::vec4(0.0f, -0.9f, 0.5f, 0.0f);
		uniformDataComposition.lights[3].color = glm::vec3(1.0f, 1.0f, 0.0f);
		uniformDataComposition.lights[3].radius = 2.0f;
		// Green
		uniformDataComposition.lights[4].position = glm::vec4(0.0f, -0.5f, 0.0f, 0.0f);
		uniformDataComposition.lights[4].color = glm::vec3(0.0f, 1.0f, 0.2f);
		uniformDataComposition.lights[4].radius = 5.0f;
		// Yellow
		uniformDataComposition.lights[5].position = glm::vec4(0.0f, -1.0f, 0.0f, 0.0f);
		uniformDataComposition.lights[5].color = glm::vec3(1.0f, 0.7f, 0.3f);
		uniformDataComposition.lights[5].radius = 25.0f;

		// Animate the lights
		if (!paused) {
			uniformDataComposition.lights[0].position.x = sin(glm::radians(360.0f * timer)) * 5.0f;
			uniformDataComposition.lights[0].position.z = cos(glm::radians(360.0f * timer)) * 5.0f;

			uniformDataComposition.lights[1].position.x = -4.0f + sin(glm::radians(360.0f * timer) + 45.0f) * 2.0f;
			uniformDataComposition.lights[1].position.z = 0.0f + cos(glm::radians(360.0f * timer) + 45.0f) * 2.0f;

			uniformDataComposition.lights[2].position.x = 4.0f + sin(glm::radians(360.0f * timer)) * 2.0f;
			uniformDataComposition.lights[2].position.z = 0.0f + cos(glm::radians(360.0f * timer)) * 2.0f;

			uniformDataComposition.lights[4].position.x = 0.0f + sin(glm::radians(360.0f * timer + 90.0f)) * 5.0f;
			uniformDataComposition.lights[4].position.z = 0.0f - cos(glm::radians(360.0f * timer + 45.0f)) * 5.0f;

			uniformDataComposition.lights[5].position.x = 0.0f + sin(glm::radians(-360.0f * timer + 135.0f)) * 10.0f;
			uniformDataComposition.lights[5].position.z = 0.0f - cos(glm::radians(-360.0f * timer - 45.0f)) * 10.0f;
		}

		// Current view position
		//uniformDataComposition.viewPos = glm::vec4(camera.position, 0.0f) * glm::vec4(-1.0f, 1.0f, -1.0f, 1.0f);

		uniformDataComposition.viewPos = -glm::vec4(glm::vec3(camera.matrices.view[3]), 0.0f);
		uniformDataComposition.debugDisplayTarget = debugDisplayTarget;
		uniformDataComposition.singleStride = singleStride;
		uniformDataComposition.camView = camera.matrices.view;
		uniformDataComposition.camViewTr = glm::transpose(camera.matrices.view);
		uniformDataComposition.depthFactor = depthFactor;
		uniformDataComposition.uFactor = uFactor;
		uniformDataComposition.vFactor = vFactor;
		uniformDataComposition.orthographic = camera.orthographic;
		memcpy(uniformBuffers.composition.mapped, &uniformDataComposition, sizeof(UniformDataComposition));
	}

	void changeImageLayoutOneTime(VkImageLayout oldLayout, VkImageLayout newLayout, VkImage image) {
		VkCommandBuffer tmpCmdBuf = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);
		VkImageMemoryBarrier barrier = {};
		barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		barrier.oldLayout = oldLayout;
		barrier.newLayout = newLayout;
		barrier.image = image;
		barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		barrier.subresourceRange.baseMipLevel = 0;
		barrier.subresourceRange.levelCount = 1;
		barrier.subresourceRange.baseArrayLayer = 0;
		barrier.subresourceRange.layerCount = 1;

		vkCmdPipelineBarrier(
			tmpCmdBuf,
			VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
			0,
			0, nullptr,
			0, nullptr,
			1, &barrier
		);
		vulkanDevice->flushCommandBuffer(tmpCmdBuf, queue, true);
	}

	void copyGPUtoCPU(uint32_t width, uint32_t height, VkImage image, VkBuffer dstBuffer) {
		VkCommandBuffer tmpCmdBuf = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

		VkBufferImageCopy region = {};
		region.bufferOffset = 0;
		region.bufferRowLength = 0;
		region.bufferImageHeight = 0;
		region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		region.imageSubresource.mipLevel = 0;
		region.imageSubresource.baseArrayLayer = 0;
		region.imageSubresource.layerCount = 1;
		region.imageOffset = { 0, 0, 0 };
		region.imageExtent = { width, height, 1 };

		vkCmdCopyImageToBuffer(tmpCmdBuf, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, dstBuffer, 1, &region);
		vkEndCommandBuffer(tmpCmdBuf);

		VkSubmitInfo submitInfo{};
		submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
		submitInfo.commandBufferCount = 1;
		submitInfo.pCommandBuffers = &tmpCmdBuf;
		VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));
		vkQueueWaitIdle(queue);
		vkFreeCommandBuffers(device, cmdPool, 1, &tmpCmdBuf);
	}

	void rebuildLockedEdgeCommandBuffer() {
		vkResetCommandBuffer(lockedEdgeCmdBuffer, 0);
		VkCommandBufferBeginInfo cmdBufInfo = vks::initializers::commandBufferBeginInfo();

		std::array<VkClearValue, 2> clearValues;
		clearValues[0].color = { { 0.0f, 0.0f, 0.0f, 0.0f } };
		clearValues[1].depthStencil = { 1.0f, 0 };

		VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
		renderPassBeginInfo.renderPass = lockedEdgeFrameBuf.renderPass;
		renderPassBeginInfo.framebuffer = lockedEdgeFrameBuf.frameBuffer;
		renderPassBeginInfo.renderArea.extent.width = lockedEdgeFrameBuf.width;
		renderPassBeginInfo.renderArea.extent.height = lockedEdgeFrameBuf.height;
		//diff: render pass don't have any clear values
		renderPassBeginInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
		renderPassBeginInfo.pClearValues = clearValues.data();

		VK_CHECK_RESULT(vkBeginCommandBuffer(lockedEdgeCmdBuffer, &cmdBufInfo));

		vkCmdBeginRenderPass(lockedEdgeCmdBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

		VkViewport viewport = vks::initializers::viewport((float)lockedEdgeFrameBuf.width, -(float)lockedEdgeFrameBuf.height, 0.0f, 1.0f);
		viewport.x = 0;
		viewport.y = (float)lockedEdgeFrameBuf.height;
		vkCmdSetViewport(lockedEdgeCmdBuffer, 0, 1, &viewport);

		VkRect2D scissor = vks::initializers::rect2D(lockedEdgeFrameBuf.width, lockedEdgeFrameBuf.height, 0, 0);
		vkCmdSetScissor(lockedEdgeCmdBuffer, 0, 1, &scissor);

		vkCmdBindPipeline(lockedEdgeCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.lockedEdge);

		vkCmdPushConstants(edgeCmdBuffer, pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushValue2), &pushVal2);

		vkCmdBindDescriptorSets(lockedEdgeCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets.lockedEdge, 0, nullptr);
		mesh.bindLockedEdgeBuffers(lockedEdgeCmdBuffer);
		vkCmdDrawIndexed(lockedEdgeCmdBuffer, mesh.lockedEdgeIdxCnt, 1, 0, 0, 0);

		vkCmdEndRenderPass(lockedEdgeCmdBuffer);


		VK_CHECK_RESULT(vkEndCommandBuffer(lockedEdgeCmdBuffer));
	}

	void rebuildHiddenLineCommandBuffer() {
		vkResetCommandBuffer(hiddenLineCmdBuffer, 0);
		VkCommandBufferBeginInfo cmdBufInfo = vks::initializers::commandBufferBeginInfo();

		std::array<VkClearValue, 2> clearValues;
		clearValues[0].color = { { 0.0f, 0.0f, 0.0f, 0.0f } };
		clearValues[1].depthStencil = { 1.0f, 0 };

		VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
		renderPassBeginInfo.renderPass = hiddenLineFrameBuf.renderPass;
		renderPassBeginInfo.framebuffer = hiddenLineFrameBuf.frameBuffer;
		renderPassBeginInfo.renderArea.extent.width = hiddenLineFrameBuf.width;
		renderPassBeginInfo.renderArea.extent.height = hiddenLineFrameBuf.height;
		//diff: render pass don't have any clear values
		renderPassBeginInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
		renderPassBeginInfo.pClearValues = clearValues.data();

		VK_CHECK_RESULT(vkBeginCommandBuffer(hiddenLineCmdBuffer, &cmdBufInfo));

		vkCmdBeginRenderPass(hiddenLineCmdBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

		VkViewport viewport = vks::initializers::viewport((float)hiddenLineFrameBuf.width, -(float)hiddenLineFrameBuf.height, 0.0f, 1.0f);
		viewport.x = 0;
		viewport.y = (float)hiddenLineFrameBuf.height;
		vkCmdSetViewport(hiddenLineCmdBuffer, 0, 1, &viewport);

		VkRect2D scissor = vks::initializers::rect2D(hiddenLineFrameBuf.width, hiddenLineFrameBuf.height, 0, 0);
		vkCmdSetScissor(hiddenLineCmdBuffer, 0, 1, &scissor);

		vkCmdBindPipeline(hiddenLineCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.hiddenLine);

		

		vkCmdBindDescriptorSets(hiddenLineCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets.hiddenLine, 0, nullptr);
		mesh.bindHiddenLineBuffers(hiddenLineCmdBuffer);
		vkCmdDrawIndexed(hiddenLineCmdBuffer, mesh.hiddenLineIdxCnt, 1, 0, 0, 0);

		vkCmdEndRenderPass(hiddenLineCmdBuffer);


		VK_CHECK_RESULT(vkEndCommandBuffer(hiddenLineCmdBuffer));
	}

	void prepare()
	{
#if DLF
		std::string glslToSpvBat = getShadersPath() + "occ/glsltospv.bat " + getShadersPath() + "occ";
		system(glslToSpvBat.c_str());
		camera.flipY = false;
#endif
		VulkanExampleBase::prepare();
		loadAssets();
		prepareOffscreenFramebuffer();
		prepareEdgeFramebuffer();
		prepareLockedEdgeFramebuffer(vulkanDevice);
		prepareHiddenLineFramebuffer();
		prepareUniformBuffers();
		setupDescriptors();
		preparePipelines();
		buildCommandBuffers();
		buildDeferredCommandBuffer();
		buildEdgeCommandBuffer();
		buildLockedEdgeCommandBuffer();
		buildHiddenLineCommandBuffer();
		prepared = true;
	}

	void draw()
	{
		VulkanExampleBase::prepareFrame();
		// The scene render command buffer has to wait for the offscreen
		// rendering to be finished before we can use the framebuffer
		// color image for sampling during final rendering
		// To ensure this we use a dedicated offscreen synchronization
		// semaphore that will be signaled when offscreen rendering
		// has been finished
		// This is necessary as an implementation may start both
		// command buffers at the same time, there is no guarantee
		// that command buffers will be executed in the order they
		// have been submitted by the application

		// Offscreen rendering
		// Wait for swap chain presentation to finish
		submitInfo.pWaitSemaphores = &semaphores.presentComplete;
		// Signal ready with offscreen semaphore
		submitInfo.pSignalSemaphores = &offscreenSemaphore;
		// Submit work
		submitInfo.commandBufferCount = 1;
		submitInfo.pCommandBuffers = &offScreenCmdBuffer;
		VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));


		// Edge rendering
		// Wait for offscreen semaphore
		submitInfo.pWaitSemaphores = &offscreenSemaphore;
		submitInfo.pSignalSemaphores = &edgeSemaphore;
		// Submit work
		submitInfo.commandBufferCount = 1;
		submitInfo.pCommandBuffers = &edgeCmdBuffer;
		VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));

		//if (prevLockedView != lockedView) {
		//	if (prevLockedView == false) {
		//		// locked view opened
		//		vkDeviceWaitIdle(device);
		//		changeImageLayoutOneTime(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, edgeFrameBuf.position.image);
		//		copyGPUtoCPU(edgeFrameBuf.width, edgeFrameBuf.height, edgeFrameBuf.position.image, mesh.cpuImageBuffer.buffer);
		//		vkMapMemory(device, mesh.cpuImageBuffer.memory, 0, edgeFrameBuf.width * edgeFrameBuf.height * sizeof(glm::vec4), 0, &mesh.edgePixelsRawData);
		//		mesh.analyzeEdgePixels(vulkanDevice, edgeFrameBuf.height, edgeFrameBuf.width, queue);
		//		rebuildLockedEdgeCommandBuffer();
		//	}
		//	else {
		//		// locked view closed

		//	}
		//	prevLockedView = lockedView;
		//}
		if (lockedView) {
			vkDeviceWaitIdle(device);
			//changeImageLayoutOneTime(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, edgeFrameBuf.position.image);
			//copyGPUtoCPU(edgeFrameBuf.width, edgeFrameBuf.height, edgeFrameBuf.position.image, mesh.cpuImageBuffer.buffer);
			//vkMapMemory(device, mesh.cpuImageBuffer.memory, 0, edgeFrameBuf.width * edgeFrameBuf.height * sizeof(glm::vec4), 0, &mesh.edgePixelsRawData);
			mesh.calculateSilhouette(shape, camera);
			mesh.analyzeEdgePixels2(vulkanDevice, edgeFrameBuf.height, edgeFrameBuf.width, queue, camera);
			rebuildEdgeCommandBuffer();
			rebuildLockedEdgeCommandBuffer();
			rebuildHiddenLineCommandBuffer();
			lockedView = false;
		}

		submitInfo.pWaitSemaphores = &semaphores.presentComplete;
		//submitInfo.pWaitSemaphores = &edgeSemaphore;
		// Signal ready with render complete semaphore
		submitInfo.pSignalSemaphores = &lockedEdgeSemaphore;
		// Submit work
		submitInfo.pCommandBuffers = &lockedEdgeCmdBuffer;
		VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));

		submitInfo.pWaitSemaphores = &semaphores.presentComplete;
		// Signal ready with render complete semaphore
		submitInfo.pSignalSemaphores = &hiddenLineSemaphore;
		// Submit work
		submitInfo.pCommandBuffers = &hiddenLineCmdBuffer;
		VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));

		std::array<VkSemaphore, 3> waitSemaphores = { edgeSemaphore, lockedEdgeSemaphore, hiddenLineSemaphore };
		// Scene rendering
		// Wait for offscreen semaphore
		submitInfo.pWaitSemaphores = waitSemaphores.data();
		// Signal ready with render complete semaphore
		submitInfo.pSignalSemaphores = &semaphores.renderComplete;
		// Submit work
		submitInfo.pCommandBuffers = &drawCmdBuffers[currentBuffer];
		VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));

		VulkanExampleBase::submitFrame();
	}


	virtual void render()
	{
		if (!prepared)
			return;
		updateUniformBufferComposition();
		updateUniformBufferOffscreen();
		updateUniformBufferEdge();
		updateUniformBufferLockedEdge();
		updateUniformBufferHiddenLine();
		draw();
	}

	virtual void OnUpdateUIOverlay(vks::UIOverlay* overlay)
	{
		if (overlay->header("Settings")) {
			overlay->comboBox("Display", &debugDisplayTarget, { "Final composition", "Position", "Normals", "LineWire", "LineObj", "LineFace", "LineFaceNor", "PureNor", "DepthNor","IsoparametricLine", "Edge", "EdgeUV", "EdgePure", "LockedEdge", "LockedEdge2", "HiddenLine", "Test1", "Test2" });
			ImGui::InputInt("Stride", &singleStride);
			ImGui::DragFloat("DepthFactor", &depthFactor, 0.1f, 0.f, 100.f);
			overlay->sliderInt("U", &uFactor, 1, 100);
			overlay->sliderInt("V", &vFactor, 1, 100);
			overlay->sliderFloat("triColor", &triColor, 0.0, 1.0);
			overlay->text("Camera Parameters");
			overlay->sliderFloat("fov", &camera.fov, 10.f, 80.f);
			overlay->sliderFloat("zNear", &camera.znear, 0.000001f, 0.5f);
			overlay->sliderFloat("zFar", &camera.zfar, 50.f, 1000.f);
			overlay->sliderFloat("left", &camera.orthoLeft, -10.f, -0.5f);
			overlay->sliderFloat("right", &camera.orthoRight, 0.5f, 10.f);
			overlay->sliderFloat("bottom", &camera.orthoBottom, -10.f, -0.5f);
			overlay->sliderFloat("top", &camera.orthoTop, 0.5f, 10.f);
			overlay->checkBox("Orthographic", &camera.orthographic);
			lockedView = overlay->button("LockedView");
			//overlay->text("CamPos: (%.3f, %.3f, %.3f)", camera.position.x, camera.position.y, camera.position.z);
			//overlay->text("CamFront: (%.3f, %.3f, %.3f)", camera.getCamFront().x, camera.getCamFront().y, camera.getCamFront().z);
			//overlay->text("CamUp: (%.3f, %.3f, %.3f)", camera.getCamUp().x, camera.getCamUp().y, camera.getCamUp().z);
			//overlay->text("CamRight: (%.3f, %.3f, %.3f)", camera.getCamRight().x, camera.getCamRight().y, camera.getCamRight().z);
			//glm::vec3 camRightCross = glm::normalize(glm::cross(camera.getCamFront(), glm::vec3(0.0f, 1.0f, 0.0f)));
			//overlay->text("CamRightCross: (%.3f, %.3f, %.3f)", camRightCross.x, camRightCross.y, camRightCross.z);
		}
	}
};

VULKAN_EXAMPLE_MAIN()