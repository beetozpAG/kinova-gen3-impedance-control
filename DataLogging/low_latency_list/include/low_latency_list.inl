#ifndef LOW_LATENCY_LIST_INL
#define LOW_LATENCY_LIST_INL
#include "low_latency_list.h"
#include <algorithm>
#include <bit>
#include <cstdint>
#include <new>
#include <utility>
#ifdef __linux__
#include <sys/mman.h>
#include <unistd.h>
#endif
namespace low_latency_list {
constexpr uintptr_t align_up(uintptr_t ptr, uintptr_t alignment) noexcept {
	assert(std::has_single_bit(alignment));
	return ((ptr) + (alignment - 1)) & ~(alignment - 1);
}
constexpr void* align_up(void* ptr, uintptr_t alignment) noexcept {
	return reinterpret_cast<void*>(
	    align_up(reinterpret_cast<uintptr_t>(ptr), alignment));
}

constexpr const void* align_up(const void* ptr, uintptr_t alignment) noexcept {
	return reinterpret_cast<const void*>(
	    align_up(reinterpret_cast<uintptr_t>(ptr), alignment));
}
template <typename T>
List<T>::List(size_t elems_per_block, size_t prefault_stride,
              size_t prefault_every, BlockListNodeAllocator allocator) noexcept
    : elems_per_block{std::max(elems_per_block, size_t{1})},
      prefault_stride{
#ifdef __linux__
          std::max(size_t(getpagesize()), prefault_stride)
#else
          prefault_stride
#endif
      },
      prefault_every{prefault_every},
      first{nullptr}, last{nullptr}, insert_block{nullptr}, next_block{nullptr},
      next_insert{nullptr}, prefault_block{nullptr}, prefault_addr{nullptr},
      since_prefault{0}, allocator{std::move(allocator)}, elems{0} {
	allocate_new_block();
}
template <typename T>
List<T>::List(size_t elems_per_block, size_t prefault_stride,
              size_t prefault_every) noexcept
    : List(elems_per_block, prefault_stride, prefault_every,
           BlockListNodeAllocator()) {
}
template <typename T>
List<T>::List(size_t elems_per_block, size_t prefault_stride) noexcept
    : List(elems_per_block, prefault_stride,
           (prefault_stride >= elem_stride ? prefault_stride / elem_stride
                                           : 0)) {
}

template <typename T>
List<T>::List(size_t elems_per_block) noexcept
    : List(elems_per_block,
#ifdef __linux__
           std::max(default_prefault_stride, static_cast<size_t>(getpagesize()))
#else
           default_prefault_stride
#endif
      ) {
}
template <typename T> List<T>::List() noexcept : List(default_elems_per_block) {
}

template <typename T>
List<T>::List(List&& other) noexcept
    : elems_per_block{std::exchange(other.elems_per_block, 0)},
      prefault_stride{std::exchange(other.prefault_stride, 0)},
      prefault_every{std::exchange(other.prefault_every, 0)},
      first{std::exchange(other.first, nullptr)},
      last{std::exchange(other.last, nullptr)},
      insert_block{std::exchange(other.insert_block, nullptr)},
      next_block{std::exchange(other.next_block, nullptr)},
      next_insert{std::exchange(other.next_insert, nullptr)},
      prefault_block{std::exchange(other.prefault_block, nullptr)},
      prefault_addr{std::exchange(other.prefault_addr, nullptr)},
      since_prefault{std::exchange(other.since_prefault, 0)},
      allocator{std::move(other.allocator)},
      elems{std::exchange(other.elems, 0)} {
}

template <typename T> List<T>& List<T>::operator=(List&& other) noexcept {
	if (this != &other) {
		destruct_all();
		release_all();
		elems_per_block = std::exchange(other.elems_per_block, 0);
		prefault_stride = std::exchange(other.prefault_stride, 0);
		prefault_every  = std::exchange(other.prefault_every, 0);
		first           = std::exchange(other.first, nullptr);
		last            = std::exchange(other.last, nullptr);
		insert_block    = std::exchange(other.insert_block, nullptr);
		next_block      = std::exchange(other.next_block, nullptr);
		next_insert     = std::exchange(other.next_insert, nullptr);
		prefault_block  = std::exchange(other.prefault_block, nullptr);
		prefault_addr   = std::exchange(other.prefault_addr, nullptr);
		since_prefault  = std::exchange(other.since_prefault, 0);
		allocator       = std::move(other.allocator);
		elems           = std::exchange(other.elems, 0);
	}
	return *this;
}
template <typename T>
List<T>::BlockResult List<T>::allocate_new_block() noexcept {
	static const size_t alloc_alignment =
#ifdef __linux__
	    getpagesize();
#else
	    alignof(std::max_align_t);
#endif
	size_t extra_bytes =
	    (alignof(T) > alloc_alignment) ? alignof(T) - alloc_alignment : 0;
	size_t costly_ops = 0;
	auto block_size   = align_up(elems_per_block * elem_stride + extra_bytes,
	                             std::max(alloc_alignment, alignof(T)));
#ifdef __linux__
	block_size = align_up(block_size, alloc_alignment);
#endif
	BlockListNode* next = nullptr;
	if (next_block) {
		next = std::exchange(next_block, nullptr);
	} else {
		auto allocRes = allocator.allocate(true);
		next          = allocRes.allocated;
		costly_ops += allocRes.costly_ops;
		if (!next) {
			return {.success = false, .costly_ops = costly_ops};
		}
	}
	++costly_ops;
#ifdef __linux__
	void* addr = mmap(nullptr, block_size, PROT_READ | PROT_WRITE,
	                  MAP_ANONYMOUS | MAP_PRIVATE, -1, 0);
	if (addr == MAP_FAILED) {
		allocator.deallocate(next);
		return {.success = false, .costly_ops = costly_ops};
	}
	madvise(addr, block_size, MADV_NOHUGEPAGE);
#else
	void* addr = malloc(block_size);
	if (!addr) {
		allocator.deallocate(next);
		return {.success = false, .costly_ops = costly_ops};
	}
#endif
	next->addr = addr;
	next->size = block_size;
	next->next = nullptr;
	if (last) {
		last->next = next;
	} else {
		first = next;
	}
	last = next;
	if (!insert_block) {
		insert_from(next);
	}
	return {.success = true, .costly_ops = costly_ops};
}

template <typename T>
size_t List<T>::check_current_block(bool avoid_costly_ops) noexcept {
	size_t costly_ops = 0;
	if (next_insert &&
	    static_cast<std::byte*>(next_insert) + sizeof(T) >
	        static_cast<std::byte*>(insert_block->addr) + insert_block->size) {
		insert_block = insert_block->next;
		if (insert_block) {
			next_insert = align_up(insert_block->addr, alignof(T));
		} else {
			next_insert = nullptr;
		}
	}
	if (!avoid_costly_ops && !next_block) {
		auto allocRes = allocator.allocate(avoid_costly_ops);
		next_block    = allocRes.allocated;
		avoid_costly_ops |= allocRes.costly_ops;
		costly_ops += allocRes.costly_ops;
	}
	if (!avoid_costly_ops && insert_block == last &&
	    next_insert >= static_cast<std::byte*>(insert_block->addr) +
	                       insert_block->size / 2) {
		auto allocRes = allocate_new_block();
		costly_ops += allocRes.costly_ops;
	}
	return costly_ops;
}

template <typename T>
List<T>::BlockResult List<T>::ensure_current_block() noexcept {
	if (!insert_block) {
		return allocate_new_block();
	}
	return {.success = true, .costly_ops = 0};
}
template <typename T> void List<T>::check_prefault_block() noexcept {
	if (prefault_addr && static_cast<std::byte*>(prefault_addr) >=
	                         static_cast<std::byte*>(prefault_block->addr) +
	                             prefault_block->size) {
		prefault_block = prefault_block->next;
		if (prefault_block) {
			prefault_addr = prefault_block->addr;
		} else {
			prefault_addr = nullptr;
		}
	}
}
template <typename T>
void List<T>::prefault_from(BlockListNode* block) noexcept {
	prefault_block = block;
	prefault_addr  = block->addr;
	check_prefault_block();
}
template <typename T> void List<T>::insert_from(BlockListNode* block) noexcept {
	insert_block = block;
	next_insert  = align_up(block->addr, alignof(T));
	if (!prefault_block)
		prefault_from(block);
}
template <typename T>
size_t List<T>::update_next(bool avoid_costly_ops) noexcept {
	next_insert = static_cast<std::byte*>(next_insert) + elem_stride;
	return check_current_block(avoid_costly_ops);
}
template <typename T> void List<T>::update_prefault() noexcept {
	prefault_addr = static_cast<std::byte*>(prefault_addr) + prefault_stride;
	check_prefault_block();
}
template <typename T> void List<T>::destruct_all() noexcept {
	if constexpr (NeedsDestructor_v<T>) {
		BlockListNode* node = first;
		while (node) {
			void* ptr = node->addr;
			void* end = node == insert_block
			                ? next_insert
			                : static_cast<std::byte*>(node->addr) + node->size;
			while (ptr < end) {
				std::destroy_at(static_cast<T*>(ptr));
				ptr = static_cast<std::byte*>(ptr) + elem_stride;
			}
			node = node == insert_block ? nullptr : node->next;
		}
	}
	elems = 0;
}
template <typename T> void List<T>::release_all() noexcept {
	BlockListNode* node = first;
	while (node) {
		BlockListNode* next = node->next;
#ifdef __linux__
		munmap(node->addr, node->size);
#else
		free(node->addr);
#endif
		allocator.deallocate(node);
		node = next;
	}
	if (next_block)
		allocator.deallocate(next_block);
}

template <typename T> size_t List<T>::do_prefault() noexcept {
	if (prefault_block) {
		*static_cast<volatile std::byte*>(prefault_addr) = std::byte(0);
		update_prefault();
		return 1;
	}
	return 0;
}

template <typename T>
template <bool isConst>
List<T>::iteratorBase<isConst>::iteratorBase() noexcept
    : list{nullptr}, current{nullptr}, cur_ptr{nullptr} {
}
template <typename T>
template <bool isConst>
List<T>::iteratorBase<isConst>::iteratorBase(const List* l,
                                             const BlockListNode* blk,
                                             void_ptr_type ptr) noexcept
    : list{l}, current{blk}, cur_ptr{ptr} {
}
template <typename T>
template <bool isConst>
List<T>::iteratorBase<isConst>&
List<T>::iteratorBase<isConst>::operator++() noexcept {
	cur_ptr = static_cast<byte_ptr_type>(cur_ptr) + elem_stride;
	if (current == list->insert_block) {
		if (cur_ptr >= list->next_insert) {
			current = nullptr;
			cur_ptr = nullptr;
			list = nullptr;
		}
	} else {
		if (static_cast<byte_ptr_type>(cur_ptr) + sizeof(T) >
		    static_cast<byte_ptr_type>(current->addr) + current->size) {
			current = current->next;
			if (current) {
				cur_ptr = align_up(current->addr, alignof(T));
			} else {
				cur_ptr = nullptr;
				list = nullptr;
			}
		}
	}
	return *this;
}
template <typename T>
template <bool isConst>
List<T>::iteratorBase<isConst>
List<T>::iteratorBase<isConst>::operator++(int) noexcept {
	auto tmp = *this;
	++*this;
	return tmp;
}
template <typename T>
template <bool isConst>
List<T>::iteratorBase<isConst>::reference
List<T>::iteratorBase<isConst>::operator*() const noexcept {
	return *static_cast<pointer>(cur_ptr);
}
template <typename T>
template <bool isConst>
List<T>::iteratorBase<isConst>::pointer
List<T>::iteratorBase<isConst>::operator->() const noexcept {
	return static_cast<pointer>(cur_ptr);
}
template <typename T>
template <bool isConst>
bool List<T>::iteratorBase<isConst>::operator==(
    const iteratorBase& rhs) const noexcept {
	return list == rhs.list && current == rhs.current && cur_ptr == rhs.cur_ptr;
}

template <typename T>
template <bool isConst>
bool List<T>::iteratorBase<isConst>::operator!=(
    const iteratorBase& rhs) const noexcept {
	return !(*this == rhs);
}

template <typename T> List<T>::iterator List<T>::begin() noexcept {
	auto begin_addr = first ? align_up(first->addr, alignof(T)) : nullptr;
	if (!first || (first == insert_block && begin_addr >= next_insert)) {
		return {};
	} else {
		return {this, first, begin_addr};
	}
}
template <typename T> List<T>::const_iterator List<T>::begin() const noexcept {
	auto begin_addr = first ? align_up(first->addr, alignof(T)) : nullptr;
	if (!first || (first == insert_block && begin_addr >= next_insert)) {
		return {};
	} else {
		return {this, first, begin_addr};
	}
}
template <typename T> List<T>::const_iterator List<T>::cbegin() const noexcept {
	auto begin_addr = first ? align_up(first->addr, alignof(T)) : nullptr;
	if (!first || (first == insert_block && begin_addr >= next_insert)) {
		return {};
	} else {
		return {this, first, begin_addr};
	}
}
template <typename T> List<T>::iterator List<T>::end() noexcept {
	return {};
}
template <typename T> List<T>::const_iterator List<T>::end() const noexcept {
	return {};
}
template <typename T> List<T>::const_iterator List<T>::cend() const noexcept {
	return {};
}
template <typename T> void List<T>::reserve_more(size_t n) noexcept {
	size_t reserved = 0;
	while (reserved < n) {
		auto res = allocate_new_block();
		if (!res.success)
			break;
		reserved += last->size / elem_stride;
	}
}
template <typename T>
template <typename... Args>
List<T>::AppendResult List<T>::append(Args&&... args) noexcept(
    noexcept(T(std::forward<Args>(args)...))) {
	size_t costly_ops = 0;
	auto nextRes      = ensure_current_block();
	costly_ops += nextRes.costly_ops;
	if (!nextRes.success)
		return {.appended = {this, nullptr, nullptr}, .costly_ops = costly_ops};
	new (static_cast<T*>(next_insert)) T(std::forward<Args>(args)...);
	elems++;
	while (insert_block == prefault_block && prefault_block &&
	       prefault_addr < static_cast<std::byte*>(next_insert) + sizeof(T)) {
		update_prefault();
		costly_ops += 1;
	}
	iterator appended = {this, insert_block, next_insert};
	costly_ops += update_next(costly_ops > 0);
	if (costly_ops == 0 && prefault_every && since_prefault >= prefault_every) {
		costly_ops += do_prefault();
		since_prefault = 0;
	}
	++since_prefault;
	return {.appended = appended, .costly_ops = costly_ops};
}

template <typename T> void List<T>::clear() noexcept {
	destruct_all();
	insert_block   = first;
	next_insert    = first ? align_up(first->addr, alignof(T)) : nullptr;
	prefault_block = first;
	prefault_addr  = first ? first->addr : nullptr;
	since_prefault = 0;
}

template <typename T> size_t List<T>::size() const noexcept {
	return elems;
}

template <typename T> bool List<T>::empty() const noexcept {
	return !elems;
}

template <typename T> bool List<T>::is_valid() const noexcept {
	return last;
}

template <typename T> List<T>::~List() noexcept {
	destruct_all();
	release_all();

	first          = nullptr;
	last           = nullptr;
	insert_block   = nullptr;
	next_insert    = nullptr;
	prefault_block = nullptr;
	prefault_addr  = nullptr;
	since_prefault = 0;
	next_block     = nullptr;
}

} // namespace low_latency_list
#endif /* LOW_LATENCY_LIST_INL */
