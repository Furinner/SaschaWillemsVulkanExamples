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
#include <memory>
#include <limits>
#include <unordered_set>
#define DLF 1

#define MAX_NEIGHBOR_FACE_COUNT 50

#define uPtr std::unique_ptr
#define mkU std::make_unique

class VulkanExample : public VulkanExampleBase
{
	template<typename T>
	static inline bool fequal(T a, T b, T epsilon = std::numeric_limits<float>::min()) {
		if (a == b) {
			// Shortcut
			return true;
		}

		const T diff = std::abs(a - b);
		if (a * b == 0) {
			// a or b or both are zero; relative error is not meaningful here
			return diff < (epsilon* epsilon);
		}

		return diff / (std::abs(a) + std::abs(b)) < epsilon;
	}

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
		size_t lockedEdgeIdxCnt;
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
			std::vector<Vertex> finVer;
			std::vector<int> finIdx;
			int idx = 0;
			struct StagingBuffer {
				VkBuffer buffer;
				VkDeviceMemory memory;
			} vertexStaging, indexStaging;
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
			glm::mat4 view = camera.matrices.view;
			for (auto& halfEdge : halfEdges) {
				for (auto& he : halfEdge) {
					//boundary edge
					if (he->sym == nullptr) {
						finVer.push_back(Vertex(he->prevVer));
						finVer.push_back(Vertex(he->nextVer));
						finIdx.push_back(idx++);
						finIdx.push_back(idx++);
						continue;
					}
					//silhouette
					float faceNorView = glm::dot(view * glm::vec4(he->prevVer->faceNor,0), glm::vec4(0,0,-1,0));
					float symFaceNorView = glm::dot(view * glm::vec4(he->sym->prevVer->faceNor, 0), glm::vec4(0,0,-1,0));
					if ((faceNorView * symFaceNorView) <= 0.f) {
						finVer.push_back(Vertex(he->prevVer));
						finVer.push_back(Vertex(he->nextVer));
						finIdx.push_back(idx++);
						finIdx.push_back(idx++);
					}
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
	};

	struct PushValue {
		int max_neighbor_cnt = MAX_NEIGHBOR_FACE_COUNT;
		float screenHalfLengthX;
		float screenHalfLengthY;

	};
	Mesh mesh;
	PushValue pushVal{};
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

	struct {
		vks::Buffer offscreen{ VK_NULL_HANDLE };
		vks::Buffer composition{ VK_NULL_HANDLE };
		vks::Buffer edge{ VK_NULL_HANDLE };
		vks::Buffer lockedEdge{ VK_NULL_HANDLE };
	} uniformBuffers;

	struct {
		VkPipeline offscreen{ VK_NULL_HANDLE };
		VkPipeline composition{ VK_NULL_HANDLE };
		VkPipeline edge{ VK_NULL_HANDLE };
		VkPipeline lockedEdge{ VK_NULL_HANDLE };
	} pipelines;
	VkPipelineLayout pipelineLayout{ VK_NULL_HANDLE };

	struct {
		VkDescriptorSet model{ VK_NULL_HANDLE };
		VkDescriptorSet floor{ VK_NULL_HANDLE };
		VkDescriptorSet composition{ VK_NULL_HANDLE };
		VkDescriptorSet edge{ VK_NULL_HANDLE };
		VkDescriptorSet lockedEdge{ VK_NULL_HANDLE };
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


	// One sampler for the frame buffer color attachments
	VkSampler colorSampler{ VK_NULL_HANDLE };

	VkCommandBuffer offScreenCmdBuffer{ VK_NULL_HANDLE };
	VkCommandBuffer edgeCmdBuffer{ VK_NULL_HANDLE };
	VkCommandBuffer lockedEdgeCmdBuffer{ VK_NULL_HANDLE };

	// Semaphore used to synchronize between offscreen and final scene rendering
	VkSemaphore offscreenSemaphore{ VK_NULL_HANDLE };
	VkSemaphore edgeSemaphore{ VK_NULL_HANDLE };
	VkSemaphore lockedEdgeSemaphore{ VK_NULL_HANDLE };

	VulkanExample() : VulkanExampleBase()
	{
		title = "Deferred shading";
		camera.type = Camera::CameraType::firstperson;
		camera.movementSpeed = 5.0f;
#ifndef __ANDROID__
		camera.rotationSpeed = 0.25f;
#endif
		/*camera.position = { 2.15f, 0.3f, -8.75f };
		camera.setRotation(glm::vec3(-0.75f, 12.5f, 0.0f));
		camera.setPerspective(60.0f, (float)width / (float)height, 0.1f, 256.0f);*/
		camera.movementSpeed = 3.f;
		camera.position = { 0.f, 3.f, -15.f };
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

			vkDestroyImageView(device, offScreenFrameBuf.normal.view, nullptr);
			vkDestroyImage(device, offScreenFrameBuf.normal.image, nullptr);
			vkFreeMemory(device, offScreenFrameBuf.normal.mem, nullptr);
			vkDestroyImageView(device, edgeFrameBuf.normal.view, nullptr);
			vkDestroyImage(device, edgeFrameBuf.normal.image, nullptr);
			vkFreeMemory(device, edgeFrameBuf.normal.mem, nullptr);

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

			vkDestroyFramebuffer(device, offScreenFrameBuf.frameBuffer, nullptr);
			vkDestroyFramebuffer(device, edgeFrameBuf.frameBuffer, nullptr);

			vkDestroyPipeline(device, pipelines.composition, nullptr);
			vkDestroyPipeline(device, pipelines.offscreen, nullptr);

			vkDestroyPipelineLayout(device, pipelineLayout, nullptr);

			vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);

			// Uniform buffers
			uniformBuffers.offscreen.destroy();
			uniformBuffers.composition.destroy();

			vkDestroyRenderPass(device, offScreenFrameBuf.renderPass, nullptr);
			vkDestroyRenderPass(device, edgeFrameBuf.renderPass, nullptr);

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

		// Depth attachment

		// Find a suitable depth format
		VkFormat attDepthFormat;
		VkBool32 validDepthFormat = vks::tools::getSupportedDepthFormat(physicalDevice, &attDepthFormat);
		assert(validDepthFormat);


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
		attachmentDescs[1].format = offScreenFrameBuf.depth.format;

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

		std::array<VkImageView, 2> attachments;
		attachments[0] = edgeFrameBuf.position.view;
		attachments[1] = offScreenFrameBuf.depth.view;

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

		vkCmdBindDescriptorSets(lockedEdgeCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets.lockedEdge, 0, nullptr);

		vkCmdDraw(lockedEdgeCmdBuffer, 0, 1, 0, 0);

		vkCmdEndRenderPass(lockedEdgeCmdBuffer);


		VK_CHECK_RESULT(vkEndCommandBuffer(lockedEdgeCmdBuffer));
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
		model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/torus", vulkanDevice, queue, glTFLoadingFlags);
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
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/two_cube", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/finger", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/patch_bump", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/classic_patch1", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/patch_bump2", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/singular_patch1", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/patch_bigbump", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/vischange", vulkanDevice, queue, glTFLoadingFlags);
		mesh.create(indexBuffers, vertexBuffers, vulkanDevice, queue);
		//view independent
		
	}

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
			vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 5),
			vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 9),
			vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 3)
		};
		VkDescriptorPoolCreateInfo descriptorPoolInfo = vks::initializers::descriptorPoolCreateInfo(poolSizes, 5);
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
			vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 9)
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

		VkDescriptorImageInfo texDescriptorLockedEdge =
			vks::initializers::descriptorImageInfo(
				colorSampler,
				lockedEdgeFrameBuf.position.view,
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
		};
		vkUpdateDescriptorSets(device, static_cast<uint32_t>(writeDescriptorSets.size()), writeDescriptorSets.data(), 0, nullptr);

		//LockedEdge
		VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &descriptorSets.lockedEdge));
		writeDescriptorSets = {
			// Binding 0: Vertex shader uniform buffer
			vks::initializers::writeDescriptorSet(descriptorSets.lockedEdge, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 0, &uniformBuffers.lockedEdge.descriptor),
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
		shaderStages[0] = loadShader(getShadersPath() + "gbufferhiddenline/deferred.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getShadersPath() + "gbufferhiddenline/deferred.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
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
		shaderStages[0] = loadShader(getShadersPath() + "gbufferhiddenline/mrt.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getShadersPath() + "gbufferhiddenline/mrt.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);

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


		std::array<VkPipelineColorBlendAttachmentState, 1> blendAttachmentStates2 = {
			vks::initializers::pipelineColorBlendAttachmentState(0xf, VK_FALSE)
		};
		colorBlendState.attachmentCount = static_cast<uint32_t>(blendAttachmentStates2.size());
		colorBlendState.pAttachments = blendAttachmentStates2.data();
		inputAssemblyState.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
		rasterizationState.lineWidth = 3.f;
		//inputAssemblyState.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
		shaderStages[0] = loadShader(getShadersPath() + "gbufferhiddenline/edge.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getShadersPath() + "gbufferhiddenline/edge.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
		pipelineCI.renderPass = edgeFrameBuf.renderPass;
		VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipelines.edge));


		colorBlendState = vks::initializers::pipelineColorBlendStateCreateInfo(1, &blendAttachmentState);
		rasterizationState.lineWidth = 3.f;
		depthStencilState.depthWriteEnable = VK_FALSE;
		depthStencilState.depthTestEnable = VK_FALSE;
		shaderStages[0] = loadShader(getShadersPath() + "gbufferhiddenline/lockededge.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getShadersPath() + "gbufferhiddenline/lockededge.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
		pipelineCI.renderPass = lockedEdgeFrameBuf.renderPass;
		VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipelines.lockedEdge));
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

		// Map persistent
		VK_CHECK_RESULT(uniformBuffers.offscreen.map());
		VK_CHECK_RESULT(uniformBuffers.edge.map());
		VK_CHECK_RESULT(uniformBuffers.lockedEdge.map());
		VK_CHECK_RESULT(uniformBuffers.composition.map());

		// Setup instanced model positions
		uniformDataOffscreen.instancePos[0] = glm::vec4(0.0f);
		uniformDataOffscreen.instancePos[1] = glm::vec4(-4.0f, 0.0, -4.0f, 0.0f);
		uniformDataOffscreen.instancePos[2] = glm::vec4(4.0f, 0.0, -4.0f, 0.0f);

		// Update
		updateUniformBufferOffscreen();
		updateUniformBufferEdge();
		updateUniformBufferLockedEdge();
		updateUniformBufferComposition();
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
		memcpy(uniformBuffers.edge.mapped, &uniformDataEdge, sizeof(UniformDataEdge));
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
		memcpy(uniformBuffers.lockedEdge.mapped, &uniformDataLockedEdge, sizeof(UniformDataLockedEdge));
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

		vkCmdBindDescriptorSets(lockedEdgeCmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets.lockedEdge, 0, nullptr);
		mesh.bindLockedEdgeBuffers(lockedEdgeCmdBuffer);
		vkCmdDrawIndexed(lockedEdgeCmdBuffer, mesh.lockedEdgeIdxCnt, 1, 0, 0, 0);

		vkCmdEndRenderPass(lockedEdgeCmdBuffer);


		VK_CHECK_RESULT(vkEndCommandBuffer(lockedEdgeCmdBuffer));
	}

	void prepare()
	{
#if DLF
		std::string glslToSpvBat = getShadersPath() + "gbufferhiddenline/glsltospv.bat " + getShadersPath() + "gbufferhiddenline";
		system(glslToSpvBat.c_str());
		camera.flipY = true;
#endif
		VulkanExampleBase::prepare();
		loadAssets();
		prepareOffscreenFramebuffer();
		prepareEdgeFramebuffer();
		prepareLockedEdgeFramebuffer(vulkanDevice);
		prepareUniformBuffers();
		setupDescriptors();
		preparePipelines();
		buildCommandBuffers();
		buildDeferredCommandBuffer();
		buildEdgeCommandBuffer();
		buildLockedEdgeCommandBuffer();
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
			changeImageLayoutOneTime(VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, edgeFrameBuf.position.image);
			copyGPUtoCPU(edgeFrameBuf.width, edgeFrameBuf.height, edgeFrameBuf.position.image, mesh.cpuImageBuffer.buffer);
			vkMapMemory(device, mesh.cpuImageBuffer.memory, 0, edgeFrameBuf.width * edgeFrameBuf.height * sizeof(glm::vec4), 0, &mesh.edgePixelsRawData);
			mesh.analyzeEdgePixels2(vulkanDevice, edgeFrameBuf.height, edgeFrameBuf.width, queue, camera);
			rebuildLockedEdgeCommandBuffer();
			lockedView = false;
		}

		submitInfo.pWaitSemaphores = &edgeSemaphore;
		// Signal ready with render complete semaphore
		submitInfo.pSignalSemaphores = &semaphores.renderComplete;
		// Submit work
		submitInfo.pCommandBuffers = &lockedEdgeCmdBuffer;
		VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));


		// Scene rendering
		// Wait for offscreen semaphore
		submitInfo.pWaitSemaphores = &edgeSemaphore;
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
		draw();
	}

	virtual void OnUpdateUIOverlay(vks::UIOverlay* overlay)
	{
		if (overlay->header("Settings")) {
			overlay->comboBox("Display", &debugDisplayTarget, { "Final composition", "Position", "Normals", "LineWire", "LineObj", "LineFace", "LineFaceNor", "PureNor", "DepthNor","IsoparametricLine", "Edge", "EdgeUV", "EdgePure", "LockedEdge","Test1", "Test2" });
			ImGui::InputInt("Stride", &singleStride);
			ImGui::DragFloat("DepthFactor", &depthFactor, 0.1f, 0.f, 100.f);
			overlay->sliderInt("U", &uFactor, 1, 100);
			overlay->sliderInt("V", &vFactor, 1, 100);
			overlay->text("Camera Parameters");
			overlay->sliderFloat("fov", &camera.fov, 10.f, 80.f);
			overlay->sliderFloat("zNear", &camera.znear, 0.000001f, 0.5f);
			overlay->sliderFloat("zFar", &camera.zfar, 50.f, 500.f);
			overlay->sliderFloat("left", &camera.orthoLeft, -10.f, -0.5f);
			overlay->sliderFloat("right", &camera.orthoRight, 0.5f, 10.f);
			overlay->sliderFloat("bottom", &camera.orthoBottom, -10.f, -0.5f);
			overlay->sliderFloat("top", &camera.orthoTop, 0.5f, 10.f);
			overlay->checkBox("Orthographic", &camera.orthographic);
			lockedView = overlay->button("LockedView");
		}
	}
};

VULKAN_EXAMPLE_MAIN()