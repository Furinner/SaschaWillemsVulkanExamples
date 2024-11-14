/*
* Vulkan Example - Rendering outlines using the stencil buffer
*
* Copyright (C) 2016-2023 by Sascha Willems - www.saschawillems.de
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

#include "vulkanexamplebase.h"
#include "VulkanglTFModel.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#define DLF 1

#define ALL_LINE 0
#define WIREFRAME 1
#define HIDDEN_LINE 0

#define uPtr std::unique_ptr
#define mkU std::make_unique

class VulkanExample : public VulkanExampleBase
{
public:
	vkglTF::Model model;

	//self-added
	struct Vertex;
	struct HalfEdge;
	struct Face;
	struct Mesh;
	struct BoundingBox;
	

	struct BoundingBox {
		float xmin = 0.0;
		float xmax = 0.0;
		float ymin = 0.0;
		float ymax = 0.0;
		float zmin = 0.0;
		float zmax = 0.0;
	};

	struct Vertex {
		glm::vec3 position;
		glm::vec3 normal;
		HalfEdge* he;
		glm::vec3 origPos;
		glm::vec3 origNor;
		glm::vec3 posProj;
		glm::vec3 posDiv;
		int id;

		class VertexHash {
		public:
			size_t operator()(const Vertex& v) const {
				size_t seed = 5381;
				const auto hasher = std::hash<float>{};
				seed ^= hasher(v.position[0]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
				seed ^= hasher(v.position[1]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
				seed ^= hasher(v.position[2]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
				return seed;
			}
		};

		Vertex(int id, glm::vec3 pos, glm::vec3 normal, glm::vec3 origPos, glm::vec3 origNor, glm::vec3 posProj) : id(id), position(pos), normal(normal), origPos(origPos), origNor(origNor), posProj(posProj) { posDiv = perspectiveDivide(position, true); }
		
		bool operator==(const Vertex& other) const {
			return (position == other.position) && (normal == other.normal) && (he == other.he);
		}

		void setHE(HalfEdge* he) {
			this->he = he;
		}

		static VkVertexInputBindingDescription getBindingDescription() {
			VkVertexInputBindingDescription bindingDescription{};
			bindingDescription.binding = 0;
			bindingDescription.stride = 2 * sizeof(glm::vec3);
			bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
			return bindingDescription;
		}

		static std::array<VkVertexInputAttributeDescription, 2> getAttributeDescriptions() {
			std::array<VkVertexInputAttributeDescription, 2> attributeDescriptions{};
			attributeDescriptions[0].binding = 0;
			attributeDescriptions[0].location = 0;
			attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
			attributeDescriptions[0].offset = offsetof(Vertex, position); //0

			attributeDescriptions[1].binding = 0;
			attributeDescriptions[1].location = 1;
			attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
			attributeDescriptions[1].offset = offsetof(Vertex, normal);

			return attributeDescriptions;
		}

		template<typename T>
		static inline bool fequal(T a, T b, T epsilon = 0.0001) {
			if (a == b) {
				// Shortcut
				return true;
			}

			const T diff = std::abs(a - b);
			if (a * b == 0) {
				// a or b or both are zero; relative error is not meaningful here
				return diff < (epsilon * epsilon);
			}

			return diff / (std::abs(a) + std::abs(b)) < epsilon;
		}

		static bool inFace(glm::vec3 pos, glm::vec3 pos1, glm::vec3 pos2, glm::vec3 pos3) {
			float triArea = glm::length(glm::cross((pos2 - pos1), (pos3 - pos1))) * 0.5f;
			float s1 = glm::length(glm::cross((pos1 - pos), (pos2 - pos))) * 0.5f / triArea;
			float s2 = glm::length(glm::cross((pos2 - pos), (pos3 - pos))) * 0.5f / triArea;
			float s3 = glm::length(glm::cross((pos3 - pos), (pos1 - pos))) * 0.5f / triArea;
			float sum = s1 + s2 + s3;
			if (s1 >= 0 && s1 <= 1 && s2 >= 0 && s2 <= 1 && s3 >= 0 && s3 <= 1 && fequal(sum, 1.0f)) {
				return true;
			}
			return false;
		}

		static glm::vec3 inline perspectiveDivide(glm::vec3 pos, bool negZ) {
			if (negZ) {
				pos /= (-pos.z);
			}
			else {
				pos /= pos.z;
			}
			return pos;
		}

		static float inline perspectiveCorrect(glm::vec3 q1, glm::vec3 q2, float x) {
			//give me original q1 and q2, not q1 and q2 after perspective divide
			glm::vec3 qx = glm::mix(q1, q2, x);
			float t = x * qx.z / q2.z;
			return t;
		}
	};

	struct HalfEdge {
		HalfEdge* next;
		HalfEdge* prev;
		HalfEdge* sym;
		Face* face;
		Vertex* vertex;
		BoundingBox box;
		float slope;
		std::vector<std::pair<float, float>> visRanges = { {0.f, 1.f} };
		int id;

		HalfEdge(int id) :id(id) { sym = nullptr; };
		void setSym(HalfEdge* sym) {
			this->sym = sym;
			sym->sym = this;
		};
		void setFace(Face* face) {
			this->face = face;
		};
		void setVertex(Vertex* ver) {
			this->vertex = ver;
			ver->he = this;
		};
		bool inFace(Face* f) {
			HalfEdge* origHe = f->he;
			HalfEdge* currHe = origHe;
			do {
				if (currHe == this) {
					return true;
				}
				currHe = currHe->next;
			} while (currHe != origHe);
			return false;
		}

		void calculateSlope() {
			glm::vec3 v1Pos = prev->vertex->posDiv;
			glm::vec3 v2Pos = vertex->posDiv;
			slope = (v2Pos.y - v1Pos.y) / (v2Pos.x - v1Pos.x);
		}

		void calculateBox() {
			glm::vec3 v1Pos = prev->vertex->position;
			glm::vec3 v2Pos = vertex->position;
			box.xmax = std::max(v1Pos.x, v2Pos.x);
			box.xmin = std::min(v1Pos.x, v2Pos.x);
			box.ymax = std::max(v1Pos.y, v2Pos.y);
			box.ymin = std::min(v1Pos.y, v2Pos.y);
			box.zmax = std::max(v1Pos.z, v2Pos.z);
			box.zmin = std::min(v1Pos.z, v2Pos.z);
		}

		bool contourEdge() {
			glm::vec3 negZ = glm::vec3(0, 0, -1);
			glm::vec3 currP1 = vertex->position;
			glm::vec3 currP2 = prev->vertex->position;
			glm::vec3 currP3 = next->vertex->position;
			glm::vec3 currFaceNor = glm::normalize(glm::cross(currP3 - currP2, currP1 - currP2));
			glm::vec3 otherP1 = sym->vertex->position;
			glm::vec3 otherP2 = prev->vertex->position;
			glm::vec3 otherP3 = next->vertex->position;
			glm::vec3 otherFaceNor = glm::normalize(glm::cross(otherP3 - otherP2, otherP1 - otherP2));
			float currFaceDot = glm::dot(currFaceNor, negZ);
			float otherFaceDot = glm::dot(otherFaceNor, negZ);
			if ((currFaceDot * otherFaceDot) <= 0.f) {
				return true;
			}
			return false;
		}

		void calculateVisRangeWithFace(Face* f) {
			if (visRanges.empty()) {
				return;
			}
			Vertex* p1Ver = prev->vertex;
			Vertex* p2Ver = vertex;
			//check if p1Ver and p2Ver are behind camera
			if ( (p1Ver->position.z > 0) || (p2Ver->position.z > 0)) {
				//note, this may not be correct
				//maybe allow line to intersect camera?
				visRanges.clear();
				return;
			}
			std::vector<std::pair<float, float>> visVec{};
			//case 1, line won't intersect face.
			//line's box is behind face's box.
			if (f->box.zmin > box.zmax) {
				//in 2D plane
				glm::vec3 p1 = p1Ver->posDiv;
				glm::vec3 p2 = p2Ver->posDiv;
				HalfEdge* lineSeg1 = f->he->prev;
				HalfEdge* lineSeg2 = f->he;
				HalfEdge* lineSeg3 = f->he->next;
				std::vector<std::pair<int, float>> inters{};
				std::vector<std::pair<int, float>> activeInters{};
				inters.push_back(intersectLine(p1Ver, p2Ver, lineSeg1));
				inters.push_back(intersectLine(p1Ver, p2Ver, lineSeg2));
				inters.push_back(intersectLine(p1Ver, p2Ver, lineSeg3));
				//line 1 (p1,p2)
				//line 2 (lineSeg)
				for (int i = 0; i < inters.size(); ++i) {
					if (inters[i].first == 1) {
						activeInters.push_back(inters[i]);
					}
					else {
						if (inters[i].first == 3) {
							//line1 parallel to line2
							std::pair<float, float> vis{ 0.f, 1.f };
							visVec.push_back(vis);
							mergeVisRange(visVec);
							return;
						}
					}
				}
				//0 intersection
				if (activeInters.size() == 0) {
					//check if outside face or inside face
					if (Vertex::inFace(p1, lineSeg1->vertex->posDiv, lineSeg2->vertex->posDiv, lineSeg3->vertex->posDiv) || Vertex::inFace(p2, lineSeg1->vertex->posDiv, lineSeg2->vertex->posDiv, lineSeg3->vertex->posDiv)) {
						// if inside face, totally invisible
						visRanges.clear();
						return;
					}
					else {
						std::pair<float, float> vis{ 0.f, 1.f };
						visVec.push_back(vis);
						mergeVisRange(visVec);
					}
				}
				//1 intersection 
				else if (activeInters.size() == 1) {
					std::pair<float, float> vis{ 0.f, 1.f };
					//get the active inter
					float x = 0.f;
					bool p1InFace = Vertex::inFace(p1, lineSeg1->vertex->posDiv, lineSeg2->vertex->posDiv, lineSeg3->vertex->posDiv);
					bool p2InFace = Vertex::inFace(p2, lineSeg1->vertex->posDiv, lineSeg2->vertex->posDiv, lineSeg3->vertex->posDiv);

					//test which endpoint is in face
					if (p1InFace && (!p2InFace)) {
						//p1 in tri
						vis.first = activeInters[0].second;
					}
					else if((!p1InFace) && p2InFace){
						//p2 in tri
						vis.second = activeInters[0].second;
					}
					else if (p1InFace && p2InFace) {
						visRanges.clear();
						return;
					}
					visVec.push_back(vis);
					mergeVisRange(visVec);
				}
				//2 or 3 intersections, the segment of the line between these points is visible
				else {
					float smallx = 1.f;
					float bigx = 0.f;
					for (auto& inter : activeInters) {
						if (inter.first) {
							if (inter.second < smallx) {
								smallx = inter.second;
							}
							if (inter.second > bigx) {
								bigx = inter.second;
							}
						}
					}
					if (smallx != 0.f) {
						std::pair<float, float> vis{ 0.f, smallx };
						visVec.push_back(vis);
					}
					if (bigx != 1.f) {
						std::pair<float, float> vis{ bigx, 1.f };
						visVec.push_back(vis);
					}
					mergeVisRange(visVec);
				}
			}
			else {
				//case 2, can't hide line
				if (box.zmin > f->box.zmax) {
					return;
				}
				else {
					//case 3, could intersect face
					HalfEdge* lineSeg1 = f->he->prev;
					HalfEdge* lineSeg2 = f->he;
					HalfEdge* lineSeg3 = f->he->next;
					std::pair<int, float> interFaceResult = intersectFace(p1Ver, p2Ver, lineSeg1, lineSeg2, lineSeg3);
					
				}
			}
		}

		std::pair<int, float> intersectFace(Vertex* p1Ver, Vertex* p2Ver, HalfEdge* lineSeg1, HalfEdge* lineSeg2, HalfEdge* lineSeg3) {
			std::pair<int, float> result;
			glm::vec3 r0 = p1Ver->position;
			glm::vec3 p2 = p2Ver->position;
			glm::vec3 q1 = lineSeg1->vertex->position;
			glm::vec3 q2 = lineSeg2->vertex->position;
			glm::vec3 q3 = lineSeg3->vertex->position;
			glm::vec3 rd = glm::normalize(p2 - r0);
			glm::vec3 n = glm::normalize(glm::cross(q3 - q2, q1 - q2));


			//1. Ray-plane intersection
			float t_tmp = glm::dot(n, (q1 - r0)) / glm::dot(n, rd);
			glm::vec3 P = r0 + t_tmp * rd;
			float t = (P.y - r0.y) / (p2.y - r0.y);
			if (isinf(t)) {
				t = (P.x - r0.x) / (p2.x - r0.x);
				if (isinf(t)) {
					t = (P.z - r0.z) / (p2.z - r0.z);
				}
			}

			result.second = t;
			if ((t < 0) || (t > 1)) {
				result.first = 0;
				return result;
			}
			
			//2. Barycentric test
			if (Vertex::inFace(P, q1, q2, q3)) {
				if (r0.z < p2.z) {
					//result.first = 1, for r0 to P behind face.
					result.first = 1;
					return result;
				}
				else {
					//result.first = 2, for P to p2 behind face.
					result.first = 2;
					return result;
				}
				return result;
			}
			result.first = 0;
			return result;

		}



		std::pair<int, float> intersectLine(Vertex* p1Ver, Vertex* p2Ver, HalfEdge* he) {
			std::pair<int, float> result;
			result.second = 0.f;
			glm::vec3 p1 = p1Ver->posDiv;
			glm::vec3 p2 = p2Ver->posDiv;
			float m1 = slope;
			float m2 = he->slope;
			//if slope is the same
			if (m1 == m2 || (isinf(m1) && isinf(m2))) {
				return intersectLineSame(p1Ver, p2Ver, he);
			}
			//handle inf
			if (isinf(m1) || isinf(m2)){
				return intersectLineInf(p1Ver, p2Ver, he);
			}
			//line 2, other line
			Vertex* q1Ver = he->prev->vertex;
			Vertex* q2Ver = he->vertex;
			glm::vec3 q1 = q1Ver->posDiv;
			glm::vec3 q2 = q2Ver->posDiv;
			//intersection x
			float x = (m1 * p1.x - p1.y - m2 * q1.x + q1.y) / (m1 - m2);
			//perspective correct t
			float t1 = (x - p1.x) / (p2.x - p1.x);
			float t2 = (x - q1.x) / (q2.x - q1.x);
			//check if x is in the range of line 2
			if ((t1 > 1) || (t1 < 0) || (t2 > 1) || (t2 < 0)) {
				result.first = 0;
			}
			else {
				result.first = 1;
				float t1_correct = Vertex::perspectiveCorrect(p1Ver->position, p2Ver->position, t1);
				result.second = t1_correct;
			}
			return result;
		}

		
		std::pair<int, float> intersectLineInf(Vertex* p1Ver, Vertex* p2Ver, HalfEdge* he) {
			//here handle inf but not same slope
			std::pair<int, float> result;
			result.second = 0.f;
			glm::vec3 p1 = p1Ver->posDiv;
			glm::vec3 p2 = p2Ver->posDiv;
			float m1 = slope;
			float m2 = he->slope;
			Vertex* q1Ver = he->prev->vertex;
			Vertex* q2Ver = he->vertex;
			glm::vec3 q1 = q1Ver->posDiv;
			glm::vec3 q2 = q2Ver->posDiv;
			if (isinf(m1)) {
				float x = p1.x;
				float y = m2 * x - m2 * q1.x + q1.y;
				float t1 = (y - p1.y) / (p2.y - p1.y);
				float t2 = (x - q1.x) / (q2.x - q1.x);
				if ((t1 > 1) || (t1 < 0) || (t2 > 1) || (t2 < 0)) {
					result.first = 0;
				}
				else {
					float t_correct = Vertex::perspectiveCorrect(p1Ver->position, p2Ver->position, t1);
					result.first = 1;
					result.second = t_correct;
				}
			}
			else {
				float x = q1.x;
				float y = m1 * x - m1 * p1.x + p1.y;
				float t1 = (x - p1.x) / (p2.x - p1.x);
				float t2 = (y - q1.y) / (q2.y - q1.y);
				if ((t1 > 1) || (t1 < 0) || (t2 > 1) || (t2 < 0)) {
					result.first = 0;
				}
				else {
					result.first = 1;
					float t_correct = Vertex::perspectiveCorrect(p1Ver->position, p2Ver->position, t1);
					result.second = t_correct;
				}
			}
			return result;
		}

		std::pair<int, float> intersectLineSame(Vertex* p1Ver, Vertex* p2Ver, HalfEdge* he) {
			//here handle same slope
			std::pair<int, float> result;
			glm::vec3 p1 = p1Ver->posDiv;
			glm::vec3 p2 = p2Ver->posDiv;
			float m1 = slope;
			float m2 = he->slope;
			glm::vec3 q1 = he->prev->vertex->posDiv;
			float m3 = (q1.y - p1.y) / (q1.x - p1.x);
			if (m3 == m1 || isnan(m3) || (isinf(m3) && isinf(m1))) {
				//intersect!
				result.first = 3;
			}
			else {
				result.first = 0;
			}
			return result;
		}

		void mergeVisRange(std::vector<std::pair<float, float>>& visVec) {
			if (!visVec.empty()) {
				visRanges = intersectionOper(visRanges, visVec);
			}
		}

	private:
		std::vector<int> intersectionHelper(std::pair<float, float>& vec1, std::pair<float, float>& vec2) {
			std::vector<int> results{};
			//case 1, no intersection
			if (vec1.second < vec2.first) {
				results.push_back(1);
				results.push_back(0);
				return results;
			}
			if (vec2.second < vec1.first) {
				results.push_back(1);
				results.push_back(1);
				return results;
			}
			//case 2, fully bound
			if ((vec2.first < vec1.first) && (vec1.second < vec2.second)) {
				results.push_back(2);
				results.push_back(0);
				return results;
			}
			if ((vec1.first < vec2.first) && (vec2.second < vec1.second)) {
				results.push_back(2);
				results.push_back(1);
				return results;
			}
			//case 3, intersection
			if ((vec1.first <= vec2.first) && (vec1.second <= vec2.second)) {
				results.push_back(3);
				results.push_back(0);
				return results;
			}
			if ((vec2.first <= vec1.first) && (vec2.second <= vec1.second)) {
				results.push_back(3);
				results.push_back(1);
				return results;
			}

		}

		std::vector<std::pair<float, float>> intersectionOper(std::vector<std::pair<float, float>>& vec1, std::vector<std::pair<float, float>>& vec2) {
			int i = 0;
			int j = 0;
			std::vector<std::pair<float, float>> results;
			while (true) {
				std::vector<int> tmpResult = intersectionHelper(vec1[i], vec2[j]);
				int cas = tmpResult[0];
				int fir = tmpResult[1];
				if (cas == 1) {
					//case 1
					if (fir == 0) {
						++i;
					}
					else if (fir == 1) {
						++j;
					}
				}
				else if (cas == 2) {
					//case 2
					if (fir == 0) {
						results.push_back(vec1[i]);
						++i;

					}
					else if (fir == 1){
						results.push_back(vec2[j]);
						++j;
					}
				}
				else {
					//case 3
					if (fir == 0) {
						std::pair<float, float> tmpPair{vec2[j].first, vec1[i].second};
						results.push_back(tmpPair);
						++i;

					}
					else if (fir == 1) {
						std::pair<float, float> tmpPair{ vec1[i].first, vec2[j].second };
						results.push_back(tmpPair);
						++j;
					}
				}
				//ij
				if ((i == vec1.size()) || (j == vec2.size())) {
					break;
				}
			}
			return results;
		}
	};

	struct Face {
		HalfEdge* he;
		BoundingBox box;
		int id;

		Face(int id) :id(id) {};
		void setHalfEdge(HalfEdge* he) {
			this->he = he;
		};

		void calculateBox() {
			glm::vec3 v1Pos = he->prev->vertex->position;
			glm::vec3 v2Pos = he->vertex->position;
			glm::vec3 v3Pos = he->next->vertex->position;
			box.xmax = std::max(std::max(v1Pos.x, v2Pos.x), v3Pos.x);
			box.xmin = std::min(std::min(v1Pos.x, v2Pos.x), v3Pos.x);
			box.ymax = std::max(std::max(v1Pos.y, v2Pos.y), v3Pos.y);
			box.ymin = std::min(std::min(v1Pos.y, v2Pos.y), v3Pos.y);
			box.zmax = std::max(std::max(v1Pos.z, v2Pos.z), v3Pos.z);
			box.zmin = std::min(std::min(v1Pos.z, v2Pos.z), v3Pos.z);
		}
	};


	struct Mesh {
		std::vector<uint32_t> index{};
		std::vector<uPtr<Face>> faces{};
		std::vector<uPtr<HalfEdge>> halfEdges{};  //get()
		std::vector<uPtr<Vertex>> vertices{}; //record only all original vertices

		std::vector<HalfEdge*> lineSegs{}; //all original lineSegs (keep only 1 sym)
		std::vector<uint32_t> lineIdx{};
		std::vector<glm::vec3> verticesData{};

#if ALL_LINE
		VkBuffer lineIdxBuffer;
		VkDeviceMemory lineIdxMemory;
		VkBuffer verticesBuffer;
		VkDeviceMemory verticesMemory;
#else
		vks::Buffer verticesBuffer;
		vks::Buffer lineIdxBuffer;
#endif

		void create(std::vector<uint32_t>& indexBuffer, std::vector<vkglTF::Vertex>& vertexBuffer, vks::VulkanDevice* device, VkQueue transferQueue, glm::mat4 view, glm::mat4 proj) {
			std::unordered_map<std::string, HalfEdge*> symHEs;
			for (int i = 0; i < vertexBuffer.size(); ++i) {
#if ALL_LINE
				glm::vec3 pos = vertexBuffer[i].pos;
				glm::vec3 nor = vertexBuffer[i].normal;
#else
				glm::vec3 pos = glm::vec3(view * glm::vec4(vertexBuffer[i].pos, 1.f));
				glm::vec3 nor = glm::mat3(view) * vertexBuffer[i].normal;
				glm::vec3 posProj = glm::vec3(proj * glm::vec4(pos, 1));
#endif
				verticesData.push_back(pos);
				verticesData.push_back(nor);
				uPtr<Vertex> vert = mkU<Vertex>(i, pos, nor, vertexBuffer[i].pos, vertexBuffer[i].normal, posProj);
				vertices.push_back(std::move(vert));
			}
			for (int i = 0; i < indexBuffer.size(); i += 3) {
				this->index.push_back(indexBuffer[i]);
				this->index.push_back(indexBuffer[i + 1]);
				this->index.push_back(indexBuffer[i + 2]);
				uPtr<HalfEdge> he1 = mkU<HalfEdge>(halfEdges.size());
				he1->setVertex(vertices[indexBuffer[i + 1]].get());
				uPtr<HalfEdge> he2 = mkU<HalfEdge>(halfEdges.size() + 1);
				he2->setVertex(vertices[indexBuffer[i + 2]].get());
				uPtr<HalfEdge> he3 = mkU<HalfEdge>(halfEdges.size() + 2);
				he3->setVertex(vertices[indexBuffer[i]].get());
				he1->next = he2.get();
				he1->prev = he3.get();
				he2->next = he3.get();
				he2->prev = he1.get();
				he3->next = he1.get();
				he3->prev = he2.get();
				uPtr<Face> f = mkU<Face>(faces.size());
				f->setHalfEdge(he1.get());
				he1->setFace(f.get());
				he2->setFace(f.get());
				he3->setFace(f.get());
				halfEdges.push_back(std::move(he1));
				halfEdges.push_back(std::move(he2));
				halfEdges.push_back(std::move(he3));
				faces.push_back(std::move(f));

				//set sym
				for (int j = 0; j < 3; ++j) {
					HalfEdge* currHe = halfEdges[halfEdges.size() - 1 - j].get();
					Vertex* ver1 = currHe->prev->vertex;
					Vertex* ver2 = currHe->vertex;
					std::string key1 = std::to_string(ver1->id) + "#" + std::to_string(ver2->id);
					std::string key2 = std::to_string(ver2->id) + "#" + std::to_string(ver1->id);
					/*if (symHEs[key2] != 0) {
						currHe->setSym(symHEs[key2]);
					}
					else {
						symHEs[key1] = currHe;
					}*/
					auto it = symHEs.find(key2);
					if (it != symHEs.end()) {
						currHe->setSym(it->second);
					}
					else {
						symHEs[key1] = currHe;
						lineSegs.push_back(currHe);
					}
				}
			}

			lineIdx.clear();
			for (auto& line : lineSegs) {
				lineIdx.push_back(line->prev->vertex->id);
				lineIdx.push_back(line->vertex->id);
			}
#if ALL_LINE
			// Create staging buffers
			// Vertex data
			struct StagingBuffer {
				VkBuffer buffer;
				VkDeviceMemory memory;
			} vertexStaging, indexStaging;

			size_t vertexBufferSize = verticesData.size() * sizeof(glm::vec3);
			size_t indexBufferSize = lineIdx.size() * sizeof(uint32_t);
			// Create staging buffers
			// Vertex data
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
				lineIdx.data()));

			// Create device local buffers
			// Vertex buffer
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | 0,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				vertexBufferSize,
				&verticesBuffer,
				&verticesMemory));
			// Index buffer
			VK_CHECK_RESULT(device->createBuffer(
				VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | 0,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
				indexBufferSize,
				&lineIdxBuffer,
				&lineIdxMemory));

			// Copy from staging buffers
			VkCommandBuffer copyCmd = device->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

			VkBufferCopy copyRegion = {};

			copyRegion.size = vertexBufferSize;
			vkCmdCopyBuffer(copyCmd, vertexStaging.buffer, verticesBuffer, 1, &copyRegion);

			copyRegion.size = indexBufferSize;
			vkCmdCopyBuffer(copyCmd, indexStaging.buffer, lineIdxBuffer, 1, &copyRegion);

			device->flushCommandBuffer(copyCmd, transferQueue, true);

			vkDestroyBuffer(device->logicalDevice, vertexStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, vertexStaging.memory, nullptr);
			vkDestroyBuffer(device->logicalDevice, indexStaging.buffer, nullptr);
			vkFreeMemory(device->logicalDevice, indexStaging.memory, nullptr);
#else
			size_t vertexBufferSize = verticesData.size() * sizeof(glm::vec3) * 5;
			size_t indexBufferSize = lineIdx.size() * sizeof(uint32_t) * 5;

			device->createBuffer(VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &verticesBuffer, vertexBufferSize);
			device->createBuffer(VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &lineIdxBuffer, indexBufferSize);
			verticesBuffer.map();
			lineIdxBuffer.map();
			memcpy(verticesBuffer.mapped, verticesData.data(), verticesData.size() * sizeof(glm::vec3));
			memcpy(lineIdxBuffer.mapped, lineIdx.data(), lineIdx.size() * sizeof(uint32_t));
#endif
		}

		void updateVertices(glm::mat4 view, glm::mat4 proj) {
			verticesData.clear();
			for (auto& ver : vertices) {
				ver->position = glm::vec3(view * glm::vec4(ver->origPos, 1));
				ver->normal = glm::mat3(view) * ver->origNor;
				ver->posProj = glm::vec3(proj * glm::vec4(ver->position, 1));
				ver->posDiv = Vertex::perspectiveDivide(ver->position, true);
				glm::vec4 test4 = proj * glm::vec4(ver->position, 1);
				verticesData.push_back(ver->position);
				verticesData.push_back(ver->normal);
			}
			for (auto& he : halfEdges) {
				std::vector<std::pair<float, float>> tmpVisRange{ {0,1} };
				he->visRanges = tmpVisRange;
			}
		}

		void calculateVisibleLineSegs(glm::mat4 view) {
			//after view and vertices' position are updated
			//calculate box & slope
			for (auto& he : halfEdges) {
				he->calculateBox();
				he->calculateSlope();
			}
			for (auto& f : faces) {
				f->calculateBox();
			}
#if !ALL_LINE
			//use current lineSegs
			for (auto& lineSeg : lineSegs) {
				for (auto& f : faces) {
					if (!lineSeg->inFace(f.get())) {
						lineSeg->calculateVisRangeWithFace(f.get());
					}
				}
			}

#endif
		}

		void updateBuffer(vks::VulkanDevice* device, VkQueue transferQueue) {
#if WIREFRAME
			lineIdx.clear();
			verticesData.clear();
			int lineIdxCnt = 0;
			for (auto& line : lineSegs) {
				if (!(line->visRanges.empty())) {
					glm::vec3 v1 = line->prev->vertex->position;
					glm::vec3 n1 = line->prev->vertex->normal;
					glm::vec3 v2 = line->vertex->position;
					glm::vec3 n2 = line->vertex->normal;
					for (auto& vis : line->visRanges) {
						verticesData.push_back(glm::mix(v1, v2, vis.first));
						verticesData.push_back(glm::mix(n1, n2, vis.first));
						verticesData.push_back(glm::mix(v1, v2, vis.second));
						verticesData.push_back(glm::mix(n1, n2, vis.second));
						lineIdx.push_back(lineIdxCnt++);
						lineIdx.push_back(lineIdxCnt++);
					}
				}
			}
#elif HIDDEN_LINE
			lineIdx.clear();
			verticesData.clear();
			int lineIdxCnt = 0;
			for (auto& line : lineSegs) {
				if (line->sym != nullptr) {
					if (!line->contourEdge()) {
						continue;
					}
				}
				if (!(line->visRanges.empty())) {
					glm::vec3 v1 = line->prev->vertex->position;
					glm::vec3 n1 = line->prev->vertex->normal;
					glm::vec3 v2 = line->vertex->position;
					glm::vec3 n2 = line->vertex->normal;
					for (auto& vis : line->visRanges) {
						verticesData.push_back(glm::mix(v1, v2, vis.first));
						verticesData.push_back(glm::mix(n1, n2, vis.first));
						verticesData.push_back(glm::mix(v1, v2, vis.second));
						verticesData.push_back(glm::mix(n1, n2, vis.second));
						lineIdx.push_back(lineIdxCnt++);
						lineIdx.push_back(lineIdxCnt++);
					}
				}
			}
#endif
#if !ALL_LINE
			size_t vertexBufferSize = verticesData.size() * sizeof(glm::vec3);
			size_t indexBufferSize = lineIdx.size() * sizeof(uint32_t);
			memcpy(verticesBuffer.mapped, verticesData.data(), vertexBufferSize);
			memcpy(lineIdxBuffer.mapped, lineIdx.data(), indexBufferSize);
#endif
		}
	};

	Mesh mesh;

	struct UniformData {
		glm::mat4 projection;
		glm::mat4 model;
		glm::vec4 lightPos = glm::vec4(0.0f, -2.0f, 1.0f, 0.0f);
		// Vertex shader extrudes model by this value along normals for outlining
		float outlineWidth = 0.025f;
	} uniformData;
	vks::Buffer uniformBuffer;

	struct {
		VkPipeline stencil{ VK_NULL_HANDLE };
		VkPipeline outline{ VK_NULL_HANDLE };
	} pipelines;

	VkPipelineLayout pipelineLayout{ VK_NULL_HANDLE };
	VkDescriptorSet descriptorSet{ VK_NULL_HANDLE };
	VkDescriptorSetLayout descriptorSetLayout{ VK_NULL_HANDLE };

	VulkanExample() : VulkanExampleBase()
	{
		title = "Stencil buffer outlines";
		timerSpeed *= 0.25f;
		camera.type = Camera::CameraType::lookat;
		camera.setPerspective(60.0f, (float)width / (float)height, 0.1f, 512.0f);
		//camera.setRotation(glm::vec3(2.5f, -35.0f, 0.0f));
		camera.setRotation(glm::vec3(0.f, 0.0f, 0.0f));
		camera.setTranslation(glm::vec3(0.0f, 0.0f, -2.0f));
		// This samples requires a format that supports depth AND stencil, which will be handled by the base class
		requiresStencil = true;
	}

	~VulkanExample()
	{
		if (device) {
			vkDestroyPipeline(device, pipelines.stencil, nullptr);
			vkDestroyPipeline(device, pipelines.outline, nullptr);
			vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
			vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);
			uniformBuffer.destroy();
		}
	}

	void buildCommandBuffers()
	{
		VkCommandBufferBeginInfo cmdBufInfo = vks::initializers::commandBufferBeginInfo();

		VkClearValue clearValues[2];
		clearValues[0].color = defaultClearColor;
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

			VkViewport viewport = vks::initializers::viewport((float)width, (float)height, 0.0f, 1.0f);
			vkCmdSetViewport(drawCmdBuffers[i], 0, 1, &viewport);

			VkRect2D scissor = vks::initializers::rect2D(width, height,	0, 0);
			vkCmdSetScissor(drawCmdBuffers[i], 0, 1, &scissor);

			VkDeviceSize offsets[1] = { 0 };

			/*vkCmdBindVertexBuffers(drawCmdBuffers[i], 0, 1, &model.vertices.buffer, offsets);
			vkCmdBindIndexBuffer(drawCmdBuffers[i], model.indices.buffer, 0, VK_INDEX_TYPE_UINT32);*/
#if ALL_LINE
			vkCmdBindVertexBuffers(drawCmdBuffers[i], 0, 1, &mesh.verticesBuffer, offsets);
			vkCmdBindIndexBuffer(drawCmdBuffers[i], mesh.lineIdxBuffer, 0, VK_INDEX_TYPE_UINT32);
#else
			vkCmdBindVertexBuffers(drawCmdBuffers[i], 0, 1, &mesh.verticesBuffer.buffer, offsets);
			vkCmdBindIndexBuffer(drawCmdBuffers[i], mesh.lineIdxBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);
#endif

			vkCmdBindDescriptorSets(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSet, 0, NULL);

			// First pass renders object (toon shaded) and fills stencil buffer
			vkCmdBindPipeline(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.stencil);
			//model.draw(drawCmdBuffers[i]);
			vkCmdDrawIndexed(drawCmdBuffers[i], mesh.lineIdx.size(), 1, 0, 0, 0);

			// Second pass renders scaled object only where stencil was not set by first pass
			/*vkCmdBindPipeline(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.outline);
			model.draw(drawCmdBuffers[i]);*/

			drawUI(drawCmdBuffers[i]);

			vkCmdEndRenderPass(drawCmdBuffers[i]);

			VK_CHECK_RESULT(vkEndCommandBuffer(drawCmdBuffers[i]));
		}
	}


	void loadAssets()
	{
		//model.loadFromFile(getAssetPath() + "models/venus.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		std::vector<uint32_t> indexBuffer;
		std::vector<vkglTF::Vertex> vertexBuffer;
		model.loadFromFileWithVertIdx(indexBuffer, vertexBuffer, getAssetPath() + "models/test/torus.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		//model.loadFromFileWithVertIdx(indexBuffer, vertexBuffer, getAssetPath() + "models/test/quad.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		//model.loadFromFileWithVertIdx(indexBuffer, vertexBuffer, getAssetPath() + "models/test/cube.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		//model.loadFromFileWithVertIdx(indexBuffer, vertexBuffer, getAssetPath() + "models/test/two_quad.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		//model.loadFromFileWithVertIdx(indexBuffer, vertexBuffer, getAssetPath() + "models/test/two_tri_inter.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		//model.loadFromFileWithVertIdx(indexBuffer, vertexBuffer, getAssetPath() + "models/test/two_tri_far.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		//model.loadFromFileWithVertIdx(indexBuffer, vertexBuffer, getAssetPath() + "models/test/12_quad_far.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		//model.loadFromFileWithVertIdx(indexBuffer, vertexBuffer, getAssetPath() + "models/test/two_quad_far.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		//model.loadFromFileWithVertIdx(indexBuffer, vertexBuffer, getAssetPath() + "models/test/four_quad.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		//model.loadFromFileWithVertIdx(indexBuffer, vertexBuffer, getAssetPath() + "models/test/two_quad_far3.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		//model.loadFromFileWithVertIdx(indexBuffer, vertexBuffer, getAssetPath() + "models/test/two_tri_far2.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		//model.loadFromFileWithVertIdx(indexBuffer, vertexBuffer, getAssetPath() + "models/test/two_quad_far2.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		//model.loadFromFileWithVertIdx(indexBuffer, vertexBuffer, getAssetPath() + "models/test/3_tri.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		//model.loadFromFileWithVertIdx(indexBuffer, vertexBuffer, getAssetPath() + "models/test/18_quad.gltf", vulkanDevice, queue, vkglTF::FileLoadingFlags::PreTransformVertices | vkglTF::FileLoadingFlags::FlipY);
		mesh.create(indexBuffer, vertexBuffer, vulkanDevice, queue, camera.matrices.view, camera.matrices.perspective);
	}

	void setupDescriptors()
	{
		// Pool
		std::vector<VkDescriptorPoolSize> poolSizes = {
			vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1),
		};
		VkDescriptorPoolCreateInfo descriptorPoolInfo = vks::initializers::descriptorPoolCreateInfo(static_cast<uint32_t>(poolSizes.size()), poolSizes.data(), 1);
		VK_CHECK_RESULT(vkCreateDescriptorPool(device, &descriptorPoolInfo, nullptr, &descriptorPool));
		
		// Layout
		std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {
			vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_VERTEX_BIT, 0)
		};

		VkDescriptorSetLayoutCreateInfo descriptorLayoutInfo = vks::initializers::descriptorSetLayoutCreateInfo(setLayoutBindings.data(), 1);
		VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorLayoutInfo, nullptr, &descriptorSetLayout));

		// Set
		VkDescriptorSetAllocateInfo allocInfo = vks::initializers::descriptorSetAllocateInfo(descriptorPool, &descriptorSetLayout, 1);
		VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &descriptorSet));
		std::vector<VkWriteDescriptorSet> modelWriteDescriptorSets = {
			vks::initializers::writeDescriptorSet(descriptorSet, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 0, &uniformBuffer.descriptor)
		};
		vkUpdateDescriptorSets(device, static_cast<uint32_t>(modelWriteDescriptorSets.size()), modelWriteDescriptorSets.data(), 0, NULL);
	}

	void preparePipelines()
	{
		// Layout
		VkPipelineLayoutCreateInfo pipelineLayoutInfo = vks::initializers::pipelineLayoutCreateInfo(&descriptorSetLayout, 1);
		VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &pipelineLayout));

		// Pipeline
		//VkPipelineInputAssemblyStateCreateInfo inputAssemblyState = vks::initializers::pipelineInputAssemblyStateCreateInfo(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, 0, VK_FALSE);
		VkPipelineInputAssemblyStateCreateInfo inputAssemblyState = vks::initializers::pipelineInputAssemblyStateCreateInfo(VK_PRIMITIVE_TOPOLOGY_LINE_LIST, 0, VK_FALSE);
		VkPipelineRasterizationStateCreateInfo rasterizationState = vks::initializers::pipelineRasterizationStateCreateInfo(VK_POLYGON_MODE_FILL, VK_CULL_MODE_FRONT_BIT, VK_FRONT_FACE_CLOCKWISE, 0);
		rasterizationState.lineWidth = 2.f;
		VkPipelineColorBlendAttachmentState blendAttachmentState = vks::initializers::pipelineColorBlendAttachmentState(0xf, VK_FALSE);
		VkPipelineColorBlendStateCreateInfo colorBlendState = vks::initializers::pipelineColorBlendStateCreateInfo(1, &blendAttachmentState);
		VkPipelineDepthStencilStateCreateInfo depthStencilState = vks::initializers::pipelineDepthStencilStateCreateInfo(VK_TRUE, VK_TRUE, VK_COMPARE_OP_LESS_OR_EQUAL);
		VkPipelineViewportStateCreateInfo viewportState = vks::initializers::pipelineViewportStateCreateInfo(1, 1, 0);
		VkPipelineMultisampleStateCreateInfo multisampleState = vks::initializers::pipelineMultisampleStateCreateInfo(VK_SAMPLE_COUNT_1_BIT, 0);
		std::vector<VkDynamicState> dynamicStateEnables = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
		VkPipelineDynamicStateCreateInfo dynamicState = vks::initializers::pipelineDynamicStateCreateInfo(dynamicStateEnables);
		std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;

		VkGraphicsPipelineCreateInfo pipelineCI = vks::initializers::pipelineCreateInfo(pipelineLayout, renderPass, 0);
		pipelineCI.pInputAssemblyState = &inputAssemblyState;
		pipelineCI.pRasterizationState = &rasterizationState;
		pipelineCI.pColorBlendState = &colorBlendState;
		pipelineCI.pMultisampleState = &multisampleState;
		pipelineCI.pViewportState = &viewportState;
		pipelineCI.pDepthStencilState = &depthStencilState;
		pipelineCI.pDynamicState = &dynamicState;
		pipelineCI.stageCount = static_cast<uint32_t>(shaderStages.size());
		pipelineCI.pStages = shaderStages.data();
		
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
		//pipelineCI.pVertexInputState = vkglTF::Vertex::getPipelineVertexInputState({ vkglTF::VertexComponent::Position, vkglTF::VertexComponent::Color, vkglTF::VertexComponent::Normal });
		
		// Toon render and stencil fill pass
#if ALL_LINE
		shaderStages[0] = loadShader(getShadersPath() + "robertshiddenline/toon1.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getShadersPath() + "robertshiddenline/toon1.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
#else
		shaderStages[0] = loadShader(getShadersPath() + "robertshiddenline/toon.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getShadersPath() + "robertshiddenline/toon.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
#endif
		rasterizationState.cullMode = VK_CULL_MODE_NONE;
		depthStencilState.stencilTestEnable = VK_TRUE;
		depthStencilState.back.compareOp = VK_COMPARE_OP_ALWAYS;
		depthStencilState.back.failOp = VK_STENCIL_OP_REPLACE;
		depthStencilState.back.depthFailOp = VK_STENCIL_OP_REPLACE;
		depthStencilState.back.passOp = VK_STENCIL_OP_REPLACE;
		depthStencilState.back.compareMask = 0xff;
		depthStencilState.back.writeMask = 0xff;
		depthStencilState.back.reference = 1;
		depthStencilState.front = depthStencilState.back;
		VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipelines.stencil));
		// Outline pass
		depthStencilState.back.compareOp = VK_COMPARE_OP_NOT_EQUAL;
		depthStencilState.back.failOp = VK_STENCIL_OP_KEEP;
		depthStencilState.back.depthFailOp = VK_STENCIL_OP_KEEP;
		depthStencilState.back.passOp = VK_STENCIL_OP_REPLACE;
		depthStencilState.front = depthStencilState.back;
		depthStencilState.depthTestEnable = VK_FALSE;
		shaderStages[0] = loadShader(getShadersPath() + "robertshiddenline/outline.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getShadersPath() + "robertshiddenline/outline.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
		VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipelines.outline));
	}

	// Prepare and initialize uniform buffer containing shader uniforms
	void prepareUniformBuffers()
	{
		// Mesh vertex shader uniform buffer block
		VK_CHECK_RESULT(vulkanDevice->createBuffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &uniformBuffer, sizeof(UniformData), &uniformData));
		VK_CHECK_RESULT(uniformBuffer.map());
	}

	void updateUniformBuffers()
	{
		uniformData.projection = camera.matrices.perspective;
		uniformData.model = camera.matrices.view;
		memcpy(uniformBuffer.mapped, &uniformData, sizeof(UniformData));
	}

	void draw()
	{
		VulkanExampleBase::prepareFrame();
#if !ALL_LINE
		//re-record cmd buffers
		reRecordCurrentCommandBuffer();
#endif
		submitInfo.commandBufferCount = 1;
		submitInfo.pCommandBuffers = &drawCmdBuffers[currentBuffer];
		VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));

		VulkanExampleBase::submitFrame();
	}

	void reRecordCurrentCommandBuffer() {
		VkCommandBufferBeginInfo cmdBufInfo = vks::initializers::commandBufferBeginInfo();

		VkClearValue clearValues[2];
		clearValues[0].color = defaultClearColor;
		clearValues[1].depthStencil = { 1.0f, 0 };

		VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
		renderPassBeginInfo.renderPass = renderPass;
		renderPassBeginInfo.renderArea.offset.x = 0;
		renderPassBeginInfo.renderArea.offset.y = 0;
		renderPassBeginInfo.renderArea.extent.width = width;
		renderPassBeginInfo.renderArea.extent.height = height;
		renderPassBeginInfo.clearValueCount = 2;
		renderPassBeginInfo.pClearValues = clearValues;

		renderPassBeginInfo.framebuffer = frameBuffers[currentBuffer];

		VK_CHECK_RESULT(vkBeginCommandBuffer(drawCmdBuffers[currentBuffer], &cmdBufInfo));

		vkCmdBeginRenderPass(drawCmdBuffers[currentBuffer], &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

		VkViewport viewport = vks::initializers::viewport((float)width, (float)height, 0.0f, 1.0f);
		vkCmdSetViewport(drawCmdBuffers[currentBuffer], 0, 1, &viewport);

		VkRect2D scissor = vks::initializers::rect2D(width, height, 0, 0);
		vkCmdSetScissor(drawCmdBuffers[currentBuffer], 0, 1, &scissor);

		VkDeviceSize offsets[1] = { 0 };

#if !ALL_LINE
		vkCmdBindVertexBuffers(drawCmdBuffers[currentBuffer], 0, 1, &mesh.verticesBuffer.buffer, offsets);
		vkCmdBindIndexBuffer(drawCmdBuffers[currentBuffer], mesh.lineIdxBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);
#endif

		vkCmdBindDescriptorSets(drawCmdBuffers[currentBuffer], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSet, 0, NULL);

		// First pass renders object (toon shaded) and fills stencil buffer
		vkCmdBindPipeline(drawCmdBuffers[currentBuffer], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.stencil);
		//model.draw(drawCmdBuffers[i]);
		vkCmdDrawIndexed(drawCmdBuffers[currentBuffer], mesh.lineIdx.size(), 1, 0, 0, 0);

		// Second pass renders scaled object only where stencil was not set by first pass
		/*vkCmdBindPipeline(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.outline);
		model.draw(drawCmdBuffers[i]);*/

		drawUI(drawCmdBuffers[currentBuffer]);

		vkCmdEndRenderPass(drawCmdBuffers[currentBuffer]);

		VK_CHECK_RESULT(vkEndCommandBuffer(drawCmdBuffers[currentBuffer]));
	}

	void prepare()
	{
#if DLF
		std::string glslToSpvBat = getShadersPath() + "robertshiddenline/glsltospv.bat " + getShadersPath() + "robertshiddenline";
		system(glslToSpvBat.c_str());
#endif
		VulkanExampleBase::prepare();
		loadAssets();
		prepareUniformBuffers();
		setupDescriptors();
		preparePipelines();
		buildCommandBuffers();
		prepared = true;
	}

	virtual void render()
	{
		if (!prepared)
			return;
		updateUniformBuffers();
#if !ALL_LINE
		glm::mat4 rot = glm::mat4(1);
		rot = glm::rotate(rot, glm::radians(180.f), glm::vec3(0, 1, 0));
		glm::mat4 cameraRot = camera.matrices.view;
		glm::mat4 cameraPer = camera.matrices.perspective;
		if (camera.updated) {
			mesh.updateVertices(camera.matrices.view, camera.matrices.perspective);
			mesh.calculateVisibleLineSegs(camera.matrices.view);
			mesh.updateBuffer(vulkanDevice, queue);
		}
#endif
		draw();
	}

	virtual void OnUpdateUIOverlay(vks::UIOverlay *overlay)
	{
		if (overlay->header("Settings")) {
			if (overlay->inputFloat("Outline width", &uniformData.outlineWidth, 0.01f, 2)) {
				updateUniformBuffers();
			}
		}
	}

};

VULKAN_EXAMPLE_MAIN()
