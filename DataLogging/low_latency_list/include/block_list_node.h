/**
 * @file block_list_node.h
 * @brief A general purpose node for a forward linked list of blocks.
 *
 * This header defines the block_list_node::BlockListNode class, which is a
 * minimal node for a forward-linked list, as well as the
 * block_list_node::BlockListNodeAllocator class, which which
 * manages allocation and recycling of these nodes by grouping them into
 * super-blocks.
 */
#ifndef BLOCK_LIST_NODE_H
#define BLOCK_LIST_NODE_H

#include <cstddef>
#ifdef __linux__
#include <sys/mman.h>
#include <unistd.h>
#endif
namespace block_list_node {

/// Default size (in bytes) of a super-block when allocating memory (ideally
/// system's page size).
constexpr size_t default_super_block_size = 1 << 12;

/**
 * @struct BlockListNode
 * @brief Represents a single node in a forward-linked list of blocks.
 *
 * Holds a pointer to the next node, the size of its backing memory,
 * and the raw address of that memory.
 */
struct BlockListNode {
	BlockListNode* next; ///< Pointer to the next node in the list (or nullptr).
	size_t size;         ///< Size in bytes of the allocated block.
	void* addr;          ///< Pointer to the start of the allocated memory.
};

/**
 * @class BlockListNodeAllocator
 * @brief Allocates and recycles BlockListNode instances in super-blocks.
 *
 * This allocator pre-allocates large contiguous regions (super-blocks) and
 * allocates BlockListNode objects in them. It keeps track of unused nodes for
 * quick reuse and can optionally avoid costly system calls.
 */
class BlockListNodeAllocator {
  public:
	/**
	 * @struct AllocateResult
	 * @brief Result of an allocation request.
	 *
	 * Contains the allocated node pointer and a count of costly operations
	 * (e.g., mmap).
	 */
	struct AllocateResult {
		BlockListNode* allocated; ///< The allocated node (nullptr on failure).
		size_t costly_ops; ///< Number of expensive system calls performed.
	};

  private:
	size_t super_block_size; ///< Size of each super-block in bytes.
	size_t nodes_in_block;   ///< Number of nodes that fit in one super-block.

	BlockListNode* unused;  ///< Head of the free list for recycled nodes.
	BlockListNode* first;   ///< First node of the super-block list.
	BlockListNode* last;    ///< First node of the super-block list.
	BlockListNode* current; ///< Super-block we're currently allocating in.
	size_t current_idx;     ///< Next free node index within `current`.

	/**
	 * @brief Allocates a new super-block and returns a node describing it.
	 *
	 * Under Linux uses mmap, otherwise uses malloc. Returns nullptr on failure.
	 * @return a node describing the allocated super_block
	 */
	BlockListNode* new_super_block();

	/**
	 * @brief Appends a fresh super-block after `last` and updates `last`.
	 *
	 * @return true on success, false on failure to allocate.
	 */
	bool append_super_block();

	/**
	 * @brief Releases all super-blocks backing allocated by this allocator.
	 *
	 * Frees or unmaps each block from `first` onward.
	 */
	void release_all();

  public:
	/**
	 * @brief Constructs an allocator with a specified super-block size.
	 *
	 * Ensures the block size is at least twice the size of a node.
	 * @param super_block_size Desired size of each super-block in bytes.
	 */
	BlockListNodeAllocator(size_t super_block_size);

	/**
	 * @brief Constructs an allocator using the default page-aligned size.
	 */
	BlockListNodeAllocator();

	// Disable copy semantics
	BlockListNodeAllocator(const BlockListNodeAllocator&)            = delete;
	BlockListNodeAllocator& operator=(const BlockListNodeAllocator&) = delete;

	/**
	 * @brief Move-construct a BlockListNodeAllocator, clearing the other
	 * allocator.
	 */
	BlockListNodeAllocator(BlockListNodeAllocator&&) noexcept;
	/**
	 * @brief Move-construct a BlockListNodeAllocator, releasing allocated
	 * memory.
	 */
	BlockListNodeAllocator& operator=(BlockListNodeAllocator&&) noexcept;

	/**
	 * @brief Destroys the allocator and releases all allocated memory.
	 */
	~BlockListNodeAllocator();

	/**
	 * @brief Allocates or recycles a node, optionally avoiding costly
	 * operations.
	 *
	 * @param avoid_costly_ops If true, postpones super-block growth unless
	 * necessary.
	 * @return AllocateResult containing the node and count of expensive calls.
	 */
	AllocateResult allocate(bool avoid_costly_ops);

	/**
	 * @brief Allocates or recycles a node with no avoidance of costly ops.
	 * @return AllocateResult with allocated node and costly op count.
	 */
	AllocateResult allocate();

	/**
	 * @brief Returns a node to the free list for future reuse.
	 *
	 * @param node Pointer to the node being deallocated.
	 */
	void deallocate(BlockListNode* node);
};
} // namespace block_list_node
#endif /* BLOCK_LIST_NODE_H */
