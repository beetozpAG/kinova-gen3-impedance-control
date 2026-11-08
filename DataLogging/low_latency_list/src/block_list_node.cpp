#include <algorithm>
#include <block_list_node.h>
#include <cstdlib>
#include <new>
#ifdef __linux__
#include <sys/mman.h>
#include <unistd.h>
#endif
#include <utility>

namespace block_list_node {
BlockListNode* BlockListNodeAllocator::new_super_block() {
#ifdef __linux__
	auto addr = mmap(nullptr, super_block_size, PROT_READ | PROT_WRITE,
	                 MAP_ANONYMOUS | MAP_PRIVATE | MAP_POPULATE, -1, 0);
	if (addr == MAP_FAILED)
		return nullptr;
#else
	auto addr = malloc(super_block_size);
	if (!addr)
		return nullptr;
#endif
	auto node  = new (static_cast<BlockListNode*>(addr)) BlockListNode();
	node->addr = addr;
	node->next = nullptr;
	node->size = super_block_size;
	return node;
}

bool BlockListNodeAllocator::append_super_block() {
	auto next = new_super_block();
	if (next) {
		last->next = next;
		last       = next;
	}
	return next;
}

BlockListNodeAllocator::BlockListNodeAllocator(size_t super_block_size)
    : super_block_size(std::max(super_block_size, 2 * sizeof(BlockListNode))),
      nodes_in_block(super_block_size / sizeof(BlockListNode)), unused(nullptr),
      first(new_super_block()), last(first), current(first),
      current_idx(first ? 1 : 0) {
}
BlockListNodeAllocator::BlockListNodeAllocator()
    : BlockListNodeAllocator(

#ifdef __linux__
          std::max(default_super_block_size, static_cast<size_t>(getpagesize()))
#else
          default_super_block_size
#endif
      ) {
}

BlockListNodeAllocator::BlockListNodeAllocator(
    BlockListNodeAllocator&& other) noexcept
    : super_block_size(std::exchange(other.super_block_size, 0)),
      nodes_in_block(std::exchange(other.nodes_in_block, 0)),
      unused(std::exchange(other.unused, nullptr)),
      first(std::exchange(other.first, nullptr)),
      last(std::exchange(other.last, nullptr)),
      current(std::exchange(other.current, nullptr)),
      current_idx(std::exchange(other.current_idx, 0)) {
}

BlockListNodeAllocator&
BlockListNodeAllocator::operator=(BlockListNodeAllocator&& other) noexcept {
	if (this != &other) {
		release_all();
		super_block_size = std::exchange(other.super_block_size, 0);
		nodes_in_block   = std::exchange(other.nodes_in_block, 0);
		unused           = std::exchange(other.unused, nullptr);
		first            = std::exchange(other.first, nullptr);
		last             = std::exchange(other.last, nullptr);
		current          = std::exchange(other.current, nullptr);
		current_idx      = std::exchange(other.current_idx, 0);
	}
	return *this;
}

void BlockListNodeAllocator::release_all() {
	BlockListNode* node = first;
	while (node) {
		BlockListNode* next = node->next;
#ifdef __linux__
		munmap(node->addr, node->size);
#else
		free(node->addr);
#endif
		node = next;
	}
}

BlockListNodeAllocator::~BlockListNodeAllocator() {
	release_all();
	unused  = nullptr;
	first   = nullptr;
	last    = nullptr;
	current = nullptr;
}

BlockListNodeAllocator::AllocateResult
BlockListNodeAllocator::allocate(bool avoid_costly_ops) {
	size_t costly_ops        = 0;
	BlockListNode* allocated = nullptr;
	if (unused) {
		allocated = unused;
		unused    = unused->next;
		return {.allocated = allocated, .costly_ops = 0};
	}
	if (!current) {
		++costly_ops;
		if (!append_super_block()) {
			return {.allocated = nullptr, .costly_ops = costly_ops};
		}
		current = last;
	}
	allocated = static_cast<BlockListNode*>(current->addr) + current_idx++;
	new (allocated) BlockListNode{};
	if (!avoid_costly_ops && current == last &&
	    current_idx >= nodes_in_block / 2) {
		++costly_ops;
		append_super_block();
	}
	if (current_idx >= nodes_in_block) {
		current     = current->next;
		current_idx = 1;
	}
	return {.allocated = allocated, .costly_ops = costly_ops};
}
BlockListNodeAllocator::AllocateResult BlockListNodeAllocator::allocate() {
	return allocate(false);
}

void BlockListNodeAllocator::deallocate(BlockListNode* node) {
	node->next = unused;
	unused     = node;
}
} // namespace block_list_node
