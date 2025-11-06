/*
* Vulkan buffer class
*
* Encapsulates a Vulkan buffer
*
* Copyright (C) 2016 by Sascha Willems - www.saschawillems.de
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

#include "VulkanBuffer.h"

namespace vks
{	
	/** 
	* Map a memory range of this buffer. If successful, mapped points to the specified buffer range.
	* 
	* @param size (Optional) Size of the memory range to map. Pass VK_WHOLE_SIZE to map the complete buffer range.
	* @param offset (Optional) Byte offset from beginning
	* 
	* @return VkResult of the buffer mapping call
	*/
	VkResult Buffer::map(VkDeviceSize size, VkDeviceSize offset)
	{
		return vkMapMemory(device, memory, offset, size, 0, &mapped);
	}

	/**
	* Unmap a mapped memory range
	*
	* @note Does not return a result as vkUnmapMemory can't fail
	*/
	void Buffer::unmap()
	{
		if (mapped)
		{
			vkUnmapMemory(device, memory);
			mapped = nullptr;
		}
	}

	/** 
	* Attach the allocated memory block to the buffer
	* 
	* @param offset (Optional) Byte offset (from the beginning) for the memory region to bind
	* 
	* @return VkResult of the bindBufferMemory call
	*/
	VkResult Buffer::bind(VkDeviceSize offset)
	{
		return vkBindBufferMemory(device, buffer, memory, offset);
	}

	/**
	* Setup the default descriptor for this buffer
	*
	* @param size (Optional) Size of the memory range of the descriptor
	* @param offset (Optional) Byte offset from beginning
	*
	*/
	void Buffer::setupDescriptor(VkDeviceSize size, VkDeviceSize offset)
	{
		descriptor.offset = offset;
		descriptor.buffer = buffer;
		descriptor.range = size;
	}

	/**
	* Copies the specified data to the mapped buffer
	* 
	* @param data Pointer to the data to copy
	* @param size Size of the data to copy in machine units
	*
	*/
	void Buffer::copyTo(void* data, VkDeviceSize size)
	{
		assert(mapped);
		memcpy(mapped, data, size);
	}

	/** 
	* Flush a memory range of the buffer to make it visible to the device
	*
	* @note Only required for non-coherent memory
	*
	* @param size (Optional) Size of the memory range to flush. Pass VK_WHOLE_SIZE to flush the complete buffer range.
	* @param offset (Optional) Byte offset from beginning
	*
	* @return VkResult of the flush call
	*/
	VkResult Buffer::flush(VkDeviceSize size, VkDeviceSize offset)
	{
		VkMappedMemoryRange mappedRange = {};
		mappedRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
		mappedRange.memory = memory;
		mappedRange.offset = offset;
		mappedRange.size = size;
		return vkFlushMappedMemoryRanges(device, 1, &mappedRange);
	}

	/**
	* Invalidate a memory range of the buffer to make it visible to the host
	*
	* @note Only required for non-coherent memory
	*
	* @param size (Optional) Size of the memory range to invalidate. Pass VK_WHOLE_SIZE to invalidate the complete buffer range.
	* @param offset (Optional) Byte offset from beginning
	*
	* @return VkResult of the invalidate call
	*/
	VkResult Buffer::invalidate(VkDeviceSize size, VkDeviceSize offset)
	{
		VkMappedMemoryRange mappedRange = {};
		mappedRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
		mappedRange.memory = memory;
		mappedRange.offset = offset;
		mappedRange.size = size;
		return vkInvalidateMappedMemoryRanges(device, 1, &mappedRange);
	}

	/** 
	* Release all Vulkan resources held by this buffer
	*/
	void Buffer::destroy()
	{
		if (buffer)
		{
			vkDestroyBuffer(device, buffer, nullptr);
		}
		if (memory)
		{
			vkFreeMemory(device, memory, nullptr);
		}
	}

	void Buffer::releaseBuffers(VkCommandBuffer cb, const std::vector<VkBuffer>& buffers,
		uint32_t srcFamily, uint32_t dstFamily, VkPipelineStageFlags srcStage,
		VkAccessFlags srcAccess, bool sameFamily) {
		std::vector<VkBufferMemoryBarrier> barriers(buffers.size());
		for (size_t i = 0; i < buffers.size(); ++i) {
			barriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
			barriers[i].pNext = nullptr;
			barriers[i].srcAccessMask = srcAccess;
			barriers[i].dstAccessMask = 0;
			barriers[i].srcQueueFamilyIndex = sameFamily ? VK_QUEUE_FAMILY_IGNORED : srcFamily;
			barriers[i].dstQueueFamilyIndex = sameFamily ? VK_QUEUE_FAMILY_IGNORED : dstFamily;
			barriers[i].buffer = buffers[i];
			barriers[i].offset = 0;
			barriers[i].size = VK_WHOLE_SIZE;
		}
		vkCmdPipelineBarrier(
			cb,
			srcStage,                                       // release: 源阶段
			VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,           // 无需后续阶段
			0,
			0, nullptr,
			static_cast<uint32_t>(barriers.size()),
			barriers.data(),
			0, nullptr);
		
	}

	void Buffer::acquireBuffers(VkCommandBuffer cb, const std::vector<VkBuffer>& buffers,
		uint32_t srcFamily, uint32_t dstFamily, VkPipelineStageFlags dstStage,
		VkAccessFlags dstAccess, bool sameFamily) {
		std::vector<VkBufferMemoryBarrier> barriers(buffers.size());
		for (size_t i = 0; i < buffers.size(); ++i) {
			barriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
			barriers[i].pNext = nullptr;
			barriers[i].srcAccessMask = 0;
			barriers[i].dstAccessMask = dstAccess;
			barriers[i].srcQueueFamilyIndex = sameFamily ? VK_QUEUE_FAMILY_IGNORED : srcFamily;
			barriers[i].dstQueueFamilyIndex = sameFamily ? VK_QUEUE_FAMILY_IGNORED : dstFamily;
			barriers[i].buffer = buffers[i];
			barriers[i].offset = 0;
			barriers[i].size = VK_WHOLE_SIZE;
		}

		vkCmdPipelineBarrier(
			cb,
			VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,              // acquire: 无前序阶段
			dstStage,                                       // 目标访问阶段
			0,                                              // dependencyFlags
			0, nullptr,                                     // memory barrier
			static_cast<uint32_t>(barriers.size()),         // buffer barrier 数量
			barriers.data(),                                // barrier 数组
			0, nullptr);                                    // image barrier
	}


	void Buffer::bufferBarrier(VkCommandBuffer cb, const std::vector<VkBuffer>& buffers,
		VkPipelineStageFlags srcStage, VkAccessFlags srcAccess, 
		VkPipelineStageFlags dstStage, VkAccessFlags dstAccess) {
		std::vector<VkBufferMemoryBarrier> barriers(buffers.size());
		for (size_t i = 0; i < buffers.size(); ++i) {
			barriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
			barriers[i].pNext = nullptr;
			barriers[i].srcAccessMask = srcAccess;
			barriers[i].dstAccessMask = dstAccess;
			barriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
			barriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
			barriers[i].buffer = buffers[i];
			barriers[i].offset = 0;
			barriers[i].size = VK_WHOLE_SIZE;
		}
		vkCmdPipelineBarrier(
			cb,
			srcStage,                                       // fill buffer 所属阶段
			dstStage,                                       // 目标访问阶段
			0,                                              
			0, nullptr,                                     
			static_cast<uint32_t>(barriers.size()),         // buffer barrier 数量
			barriers.data(),                                // barrier 数组
			0, nullptr);
	}
};
