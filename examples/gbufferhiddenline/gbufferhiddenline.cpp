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
#define DLF 1

#define MAX_NEIGHBOR_FACE_COUNT 50

#define uPtr std::unique_ptr
#define mkU std::make_unique

class VulkanExample : public VulkanExampleBase
{

	template<typename T>
	static inline bool fequal(T a, T b, T epsilon = 0.001) {
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
		glm::vec3 position;
		glm::vec3 normal;
		int objectID;
		int faceID;
		int uniqueID; //unique id in obj

		Vertex(glm::vec3 position, glm::vec3 normal, int objectID) :position(position), normal(normal), objectID(objectID) {};
		Vertex(Vertex* ver, int faceID) : position(ver->position), normal(ver->normal), objectID(ver->objectID), faceID(faceID) {};

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

		static std::array<VkVertexInputAttributeDescription, 4> getAttributeDescriptions() {
			std::array<VkVertexInputAttributeDescription, 4> attributeDescriptions{};
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
			attributeDescriptions[2].format = VK_FORMAT_R32_UINT;
			attributeDescriptions[2].offset = offsetof(Vertex, objectID);

			attributeDescriptions[3].binding = 0;
			attributeDescriptions[3].location = 3;
			attributeDescriptions[3].format = VK_FORMAT_R32_UINT;
			attributeDescriptions[3].offset = offsetof(Vertex, faceID);

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
		Vertex* prevVer;
		Vertex* nextVer;
		HalfEdge* sym = nullptr;
		HalfEdge( Vertex* prevVer, Vertex* nextVer) : prevVer(prevVer), nextVer(nextVer) {};
		void setSym(HalfEdge* sym) {
			this->sym = sym;
			sym->sym = this;
		};
	};

	struct Face {
		int objectID;
		int id;
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

		void getAllEdges(std::vector<int>& edgeIdx, std::vector<Vertex>& edgeVert) {
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
		}
	};

	struct Mesh {
		std::vector<uint32_t> index{};
		std::vector<uPtr<Vertex>> vertices1{};  //simple vertices, merged using position
		std::vector<Vertex> vertices2{};  //complex vertices, used for final rendering
		std::vector<uPtr<FaceConnectToVertex>> facesConnectToVertex{};
		std::vector<uPtr<HalfEdge>> halfEdges{};
		std::vector<uPtr<Face>> faces{};
		std::vector<int> neighborFacesData{};
		std::vector<int> objFaceCnt{};
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
		size_t vertexBuffersSize1 = 0;
		size_t vertexBuffersSize2 = 0;
		size_t indexBuffersSize = 0;
		size_t facesSize = 0;

		void create(std::vector<std::vector<uint32_t>>& indexBuffers, std::vector<std::vector<vkglTF::Vertex>>& vertexBuffers, vks::VulkanDevice* device, VkQueue transferQueue) {
			int objectID = 0;
			for (auto& vertexBuffer : vertexBuffers) {
				int uniqueVerId = 0;
				std::unordered_map<Vertex*, int, VertexHash, VertexEqual> uniqueVers;
				std::unordered_map<int, int> verIDtoUniqueID;
				std::unordered_map<std::string, HalfEdge*> symHEs;
				std::vector<std::vector<int>> neighborFaces;
				for (int i = 0; i < vertexBuffer.size(); ++i) {
					glm::vec3 pos = vertexBuffer[i].pos;
					glm::vec3 nor = vertexBuffer[i].normal;
					uPtr<Vertex> tmpVer = mkU<Vertex>(pos, nor, objectID);
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
				objFaceCnt.push_back(faceNors.size());
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
					uPtr<HalfEdge> he1 = mkU<HalfEdge>(v1, v2);
					uPtr<HalfEdge> he2 = mkU<HalfEdge>(v2, v3);
					uPtr<HalfEdge> he3 = mkU<HalfEdge>(v3, v1);
					FaceConnectToVertex* ftv1 = facesConnectToVertex[idx1 + vertexBuffersSize1].get();
					FaceConnectToVertex* ftv2 = facesConnectToVertex[idx2 + vertexBuffersSize1].get();
					FaceConnectToVertex* ftv3 = facesConnectToVertex[idx3 + vertexBuffersSize1].get();
					Vertex v1fin(v1, faceID);
					Vertex v2fin(v2, faceID);
					Vertex v3fin(v3, faceID);
					ftv1->addFace(faceID);
					ftv2->addFace(faceID);
					ftv3->addFace(faceID);
					uPtr<Face> f = mkU<Face>(objectID, faceID, ftv1, ftv2, ftv3, he1.get(), he2.get(), he3.get());
					//glm::vec3 faceNor = glm::normalize(glm::cross(v3->position - v2->position, v1->position - v2->position));
					glm::vec3 faceNor = glm::normalize((v1->normal + v2->normal + v3->normal) / 3.f);
					v1fin.normal = v1->normal;
					v2fin.normal = v2->normal;
					v3fin.normal = v3->normal;
					faceNors.push_back(glm::vec4(faceNor, 1));
					index.push_back(vertexBuffersSize2 + indexCnt++);
					index.push_back(vertexBuffersSize2 + indexCnt++);
					index.push_back(vertexBuffersSize2 + indexCnt++);
					vertices2.push_back(v1fin);
					vertices2.push_back(v2fin);
					vertices2.push_back(v3fin);
					faces.push_back(std::move(f));

					std::array<HalfEdge*, 3> hes = { he1.get(), he2.get(), he3.get()};
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
					halfEdges.push_back(std::move(he1));
					halfEdges.push_back(std::move(he2));
					halfEdges.push_back(std::move(he3));
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
					faces[i]->getAllEdges(edgeIdx, edgeVert);
					neighborFacesData.insert(neighborFacesData.end(), faces[i]->neighborFaces.begin(), faces[i]->neighborFaces.end());
				}
				facesSize = faces.size();
				vertexBuffersSize1 = vertices1.size();
				vertexBuffersSize2 = vertices2.size();
				indexBuffersSize = index.size();
				++objectID;

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
				vertices2.data()));
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
	/*float pixelLengthY = (glm::tan(glm::radians(fov / 2.f)) / (height / 2.f));
	float pixelLengthX = pixelLengthY * ((float)width / (float)height);*/
	float halfScreenHeightY = glm::tan(glm::radians(fov / 2.f));
	float halfScreenHeightX = halfScreenHeightY * ((float)width / (float)height);
	int32_t debugDisplayTarget = 0;
	int singleStride = 1;

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
	} uniformDataComposition;

	struct UniformDataEdge {
		glm::mat4 projection;
		glm::mat4 model;
		glm::mat4 view;
	} uniformDataEdge;

	struct {
		vks::Buffer offscreen{ VK_NULL_HANDLE };
		vks::Buffer composition{ VK_NULL_HANDLE };
		vks::Buffer edge{ VK_NULL_HANDLE };
	} uniformBuffers;

	struct {
		VkPipeline offscreen{ VK_NULL_HANDLE };
		VkPipeline composition{ VK_NULL_HANDLE };
		VkPipeline edge{ VK_NULL_HANDLE };
	} pipelines;
	VkPipelineLayout pipelineLayout{ VK_NULL_HANDLE };

	struct {
		VkDescriptorSet model{ VK_NULL_HANDLE };
		VkDescriptorSet floor{ VK_NULL_HANDLE };
		VkDescriptorSet composition{ VK_NULL_HANDLE };
		VkDescriptorSet edge{ VK_NULL_HANDLE };
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


	// One sampler for the frame buffer color attachments
	VkSampler colorSampler{ VK_NULL_HANDLE };

	VkCommandBuffer offScreenCmdBuffer{ VK_NULL_HANDLE };
	VkCommandBuffer edgeCmdBuffer{ VK_NULL_HANDLE };

	// Semaphore used to synchronize between offscreen and final scene rendering
	VkSemaphore offscreenSemaphore{ VK_NULL_HANDLE };
	VkSemaphore edgeSemaphore{ VK_NULL_HANDLE };

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
		camera.position = { 0.f, 0.f, -8.75f };
		camera.setRotation(glm::vec3(-0.f, 0.f, 0.0f));
		camera.setPerspective(60.0f, (float)width / (float)height, 0.1f, 256.0f);
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
		image.usage = usage | VK_IMAGE_USAGE_SAMPLED_BIT;

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
				attachmentDescs[i].finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
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
		std::array<VkSubpassDependency, 3> dependencies;

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

		dependencies[2].srcSubpass = VK_SUBPASS_EXTERNAL;
		dependencies[2].dstSubpass = 0;
		dependencies[2].srcStageMask = VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
		dependencies[2].dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
		dependencies[2].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		dependencies[2].dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;
		dependencies[2].dependencyFlags = 0;

		VkRenderPassCreateInfo renderPassInfo = {};
		renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
		renderPassInfo.pAttachments = attachmentDescs.data();
		renderPassInfo.attachmentCount = static_cast<uint32_t>(attachmentDescs.size());
		renderPassInfo.subpassCount = 1;
		renderPassInfo.pSubpasses = &subpass;
		renderPassInfo.dependencyCount = 3;
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
			VK_FORMAT_R16G16B16A16_SFLOAT,
			VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
			&edgeFrameBuf.position);

		// (World space) Normals
		createAttachment(
			VK_FORMAT_R16G16B16A16_SFLOAT,
			VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
			&edgeFrameBuf.normal);

		// Albedo (color)
		createAttachment(
			VK_FORMAT_R32G32_SINT,
			VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
			&edgeFrameBuf.albedo);

		// Depth attachment

		// Find a suitable depth format
		VkFormat attDepthFormat;
		VkBool32 validDepthFormat = vks::tools::getSupportedDepthFormat(physicalDevice, &attDepthFormat);
		assert(validDepthFormat);

		createAttachment(
			attDepthFormat,
			VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
			&edgeFrameBuf.depth);

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
				//diff: if share depth attachmentm, its initial layout should match the initial layout
				//of last depth attachment
				attachmentDescs[i].initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
				attachmentDescs[i].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
			}
			else
			{
				attachmentDescs[i].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
				attachmentDescs[i].finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
			}
		}

		// Formats
		attachmentDescs[0].format = edgeFrameBuf.position.format;
		attachmentDescs[1].format = edgeFrameBuf.normal.format;
		attachmentDescs[2].format = edgeFrameBuf.albedo.format;
		attachmentDescs[3].format = edgeFrameBuf.depth.format;

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
		std::array<VkSubpassDependency, 3> dependencies;

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

		dependencies[2].srcSubpass = VK_SUBPASS_EXTERNAL;
		dependencies[2].dstSubpass = 0;
		dependencies[2].srcStageMask = VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
		dependencies[2].dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
		dependencies[2].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		dependencies[2].dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT;
		dependencies[2].dependencyFlags = 0;

		VkRenderPassCreateInfo renderPassInfo = {};
		renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
		renderPassInfo.pAttachments = attachmentDescs.data();
		renderPassInfo.attachmentCount = static_cast<uint32_t>(attachmentDescs.size());
		renderPassInfo.subpassCount = 1;
		renderPassInfo.pSubpasses = &subpass;
		renderPassInfo.dependencyCount = 3;
		renderPassInfo.pDependencies = dependencies.data();

		VK_CHECK_RESULT(vkCreateRenderPass(device, &renderPassInfo, nullptr, &edgeFrameBuf.renderPass));

		std::array<VkImageView, 4> attachments;
		attachments[0] = edgeFrameBuf.position.view;
		attachments[1] = edgeFrameBuf.normal.view;
		attachments[2] = edgeFrameBuf.albedo.view;
		attachments[3] = offScreenFrameBuf.depth.view;

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

	// Build command buffer for rendering the scene to the offscreen frame buffer attachments
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

		// Clear values for all attachments written in the fragment shader
		std::array<VkClearValue, 4> clearValues;
		clearValues[0].color = { { 0.0f, 0.0f, 0.0f, 0.0f } };
		clearValues[1].color = { { 0.0f, 0.0f, 0.0f, 0.0f } };
		clearValues[2].color.int32[0] = -1;
		clearValues[2].color.int32[1] = -1;
		clearValues[3].depthStencil = { 1.0f, 0 };

		VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
		renderPassBeginInfo.renderPass = edgeFrameBuf.renderPass;
		renderPassBeginInfo.framebuffer = edgeFrameBuf.frameBuffer;
		renderPassBeginInfo.renderArea.extent.width = edgeFrameBuf.width;
		renderPassBeginInfo.renderArea.extent.height = edgeFrameBuf.height;
		renderPassBeginInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
		renderPassBeginInfo.pClearValues = clearValues.data();

		VK_CHECK_RESULT(vkBeginCommandBuffer(edgeCmdBuffer, &cmdBufInfo));

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

		vkCmdEndRenderPass(edgeCmdBuffer);

		VK_CHECK_RESULT(vkEndCommandBuffer(edgeCmdBuffer));
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
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/car_debug", vulkanDevice, queue, glTFLoadingFlags);
		model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/5f_cube_smooth", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/test", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/two_tri", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/three_tri", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/quad", vulkanDevice, queue, glTFLoadingFlags);
		//model.loadFromFolder(indexBuffers, vertexBuffers, getAssetPath() + "models/test/combined/cube", vulkanDevice, queue, glTFLoadingFlags);
		mesh.create(indexBuffers, vertexBuffers, vulkanDevice, queue);
	}

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
			vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 3),
			vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 7),
			vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 3)
		};
		VkDescriptorPoolCreateInfo descriptorPoolInfo = vks::initializers::descriptorPoolCreateInfo(poolSizes, 4);
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
		rasterizationState.cullMode = VK_CULL_MODE_BACK_BIT;

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

		inputAssemblyState.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
		shaderStages[0] = loadShader(getShadersPath() + "gbufferhiddenline/edge.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
		shaderStages[1] = loadShader(getShadersPath() + "gbufferhiddenline/edge.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
		pipelineCI.renderPass = edgeFrameBuf.renderPass;
		VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipelines.edge));
	}

	// Prepare and initialize uniform buffer containing shader uniforms
	void prepareUniformBuffers()
	{
		// Offscreen vertex shader
		VK_CHECK_RESULT(vulkanDevice->createBuffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &uniformBuffers.offscreen, sizeof(UniformDataOffscreen)));

		// Edge vertex shader
		VK_CHECK_RESULT(vulkanDevice->createBuffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &uniformBuffers.edge, sizeof(UniformDataEdge)))

		// Deferred fragment shader
		VK_CHECK_RESULT(vulkanDevice->createBuffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &uniformBuffers.composition, sizeof(UniformDataComposition)));

		// Map persistent
		VK_CHECK_RESULT(uniformBuffers.offscreen.map());
		VK_CHECK_RESULT(uniformBuffers.edge.map());
		VK_CHECK_RESULT(uniformBuffers.composition.map());

		// Setup instanced model positions
		uniformDataOffscreen.instancePos[0] = glm::vec4(0.0f);
		uniformDataOffscreen.instancePos[1] = glm::vec4(-4.0f, 0.0, -4.0f, 0.0f);
		uniformDataOffscreen.instancePos[2] = glm::vec4(4.0f, 0.0, -4.0f, 0.0f);

		// Update
		updateUniformBufferOffscreen();
		updateUniformBufferEdge();
		updateUniformBufferComposition();
	}

	// Update matrices used for the offscreen rendering of the scene
	void updateUniformBufferOffscreen()
	{
		uniformDataOffscreen.projection = camera.matrices.perspective;
		uniformDataOffscreen.view = camera.matrices.view;
		uniformDataOffscreen.model = glm::mat4(1.0f);
		memcpy(uniformBuffers.offscreen.mapped, &uniformDataOffscreen, sizeof(UniformDataOffscreen));
	}

	void updateUniformBufferEdge() {
		uniformDataEdge.projection = camera.matrices.perspective;
		uniformDataEdge.view = camera.matrices.view;
		uniformDataEdge.model = glm::mat4(1.0f);
		memcpy(uniformBuffers.edge.mapped, &uniformDataEdge, sizeof(UniformDataEdge));
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
		memcpy(uniformBuffers.composition.mapped, &uniformDataComposition, sizeof(UniformDataComposition));
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
		prepareUniformBuffers();
		setupDescriptors();
		preparePipelines();
		buildCommandBuffers();
		buildDeferredCommandBuffer();
		buildEdgeCommandBuffer();
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

		// Scene rendering

		// Wait for offscreen semaphore
		submitInfo.pWaitSemaphores = &offscreenSemaphore;
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
		draw();
	}

	virtual void OnUpdateUIOverlay(vks::UIOverlay* overlay)
	{
		if (overlay->header("Settings")) {
			overlay->comboBox("Display", &debugDisplayTarget, { "Final composition", "Position", "Normals", "LineWire", "LineObj", "LineFace", "LineFaceNor", "DepthNormal", "Test1", "Test2"});
			ImGui::InputInt("Stride", &singleStride);
		}
	}
};

VULKAN_EXAMPLE_MAIN()