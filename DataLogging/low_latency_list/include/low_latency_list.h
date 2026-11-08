/**
 * @file low_latency_list.h
 * @brief A low-latency block list container implementing efficient append and
 * traversal.
 *
 * This header defines the low_latency_list::List template, which allocates
 * elements in large blocks to minimize allocations and try to provide a low
 * worst-case latency.
 *
 * Under typical settings, append will cause at most one allocation or one new
 * page touch.
 */
#ifndef LOW_LATENCY_LIST_H
#define LOW_LATENCY_LIST_H
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <type_traits>

#include "block_list_node.h"

namespace low_latency_list {

/**
 * @brief Default number of bytes between prefaults (ideally system's page
 * size).
 */
constexpr size_t default_prefault_stride = 1 << 12;

/**
 * @brief Default number of elements per allocated block.
 */
constexpr size_t default_elems_per_block = 1 << 20;

/**
 * @brief Align a pointer value upwards to the given alignment.
 *
 * @param ptr The pointer value as an unsigned integer.
 * @param alignment The desired alignment (must be power of two).
 * @return The aligned pointer value.
 */
constexpr uintptr_t align_up(uintptr_t ptr, uintptr_t alignment) noexcept;

/**
 * @brief Align a pointer value upwards to the given alignment.
 *
 * @param ptr The pointer to align.
 * @param alignment The desired alignment (must be power of two).
 * @return The aligned pointer.
 */
constexpr void* align_up(void* ptr, uintptr_t alignment) noexcept;

/**
 * @brief Align a const pointer value upwards to the given alignment.
 *
 * @param ptr The const pointer to align.
 * @param alignment The desired alignment (must be power of two).
 * @return The aligned const pointer.
 */
constexpr const void* align_up(const void* ptr, uintptr_t alignment) noexcept;

using block_list_node::BlockListNode;
using block_list_node::BlockListNodeAllocator;

/**
 * @brief Trait to determine if a type requires explicit destruction.
 *
 * Defaults to `!std::is_trivially_destructible_v<T>`. Can be specialized
 * for types that do not need destructor calls.
 */
template <typename T> struct NeedsDestructor {
	static constexpr bool value =
	    !std::is_trivially_destructible_v<T>; /**< true if T needs its
	                                           destructor called */
};

/**
 * @brief Helper variable template for NeedsDestructor.
 */
template <typename T>
constexpr bool NeedsDestructor_v = NeedsDestructor<T>::value;

/**
 * @brief A list container that allocates elements in large blocks to provide
 * low worst-case latency.
 *
 * @tparam T The element type to store.
 *
 * This container is optimized for low worst-case latency append operations and
 * sequential iteration. Elements are allocated in contiguous blocks of
 * configurable size to reduce the number of system allocations and improve
 * cache performance. Optionally, memory can be prefaulted to amortize the cost
 * of page faults during insertion.
 *
 * Not thread safe.
 */
template <typename T> class List {
	static constexpr size_t elem_stride = align_up(
	    sizeof(T),
	    alignof(T));        /**< Number of bytes between consecutive elements */
	size_t elems_per_block; /**< Number of elements per block */
	size_t prefault_stride; /**< Bytes between prefaults (ideally system's
	                           page size) */
	size_t prefault_every;  /**< Number of insertions between prefaults */
	BlockListNode* first;   /**< First allocated block */
	BlockListNode* last;    /**< Last allocated block */
	BlockListNode* insert_block;   /**< Block for the next insertion */
	BlockListNode* next_block;     /**< Eagerly allocated block */
	void* next_insert;             /**< Address for the next insertion */
	BlockListNode* prefault_block; /**< Block for the next prefault */
	void* prefault_addr;           /**< Address for the next prefault */
	size_t since_prefault; /**< Number of insertions since last prefault */
	BlockListNodeAllocator allocator; /**< Allocator for block headers */
	size_t elems;                     /**< Number of stored elements */

  public:
	/**
	 * @brief Construct a List with custom block size, prefault stride, and
	 * prefault frequency, using a given allocator.
	 * @param elems_per_block Number of elements per allocated block.
	 * @param prefault_stride Number of bytes between prefaults.
	 * @param prefault_every Number of insertions between prefaults.
	 * @param allocator Allocator to use.
	 */
	explicit List(size_t elems_per_block, size_t prefault_stride,
	              size_t prefault_every,
	              BlockListNodeAllocator allocator) noexcept;
	/**
	 * @brief Construct a List with custom block size, prefault stride, and
	 * prefault frequency.
	 * @param elems_per_block Number of elements per allocated block.
	 * @param prefault_stride Number of bytes between prefaults.
	 * @param prefault_every Number of insertions between prefaults.
	 */
	explicit List(size_t elems_per_block, size_t prefault_stride,
	              size_t prefault_every) noexcept;

	/**
	 * @brief Construct a List with custom block size and prefault stride.
	 * @param elems_per_block Number of elements per allocated block.
	 * @param prefault_stride Number of bytes between prefaults.
	 */
	explicit List(size_t elems_per_block, size_t prefault_stride) noexcept;

	/**
	 * @brief Construct a List with custom block size.
	 * @param elems_per_block Number of elements per allocated block.
	 */
	explicit List(size_t elems_per_block) noexcept;

	/**
	 * @brief Default-construct a List with library defaults.
	 */
	List() noexcept;

	// Disable copy semantics
	List(const List&)            = delete;
	List& operator=(const List&) = delete;

	/**
	 * @brief Move-construct a List, Takes ownership of the other list's memory
	 * and leaves it in a destructible but unsafe to append state. All iterators
	 * into it become invalid.
	 */
	List(List&&) noexcept;

	/**
	 * @brief Move-assign a List, cleaning up existing data. Leaves the moved
	 * from list in a destructible but unsafe to append state. All iterators
	 * into it become invalid.
	 */
	List& operator=(List&&) noexcept;

  private:
	/**
	 * @brief Internal result of block allocation.
	 */
	struct BlockResult {
		bool success;      /**< Whether allocation succeeded */
		size_t costly_ops; /**< Number of costly operations performed */
	};

	/**
	 * @brief Allocate and link a new memory block.
	 *
	 * @return BlockResult indicating success and operation cost.
	 */
	BlockResult allocate_new_block() noexcept;

	/**
	 * @brief Bookkeep insertion block, possibly allocating a new block.
	 * @param avoid_costly_ops if true, don't perform any allocation.
	 *
	 * @return The number of costly operation done.
	 */
	size_t check_current_block(bool avoid_costly_ops) noexcept;

	/**
	 * @brief Bookkeep prefault block.
	 */
	void check_prefault_block() noexcept;

	/**
	 * @brief Begin prefaulting from a given block.
	 */
	void prefault_from(BlockListNode* block) noexcept;
	/**
	 * @brief Begin inserting from a given block.
	 */
	void insert_from(BlockListNode* block) noexcept;

	/**
	 * @brief Allocate a new block if there's no current block where to insert.
	 *
	 * @return BlockResult indicating success and operation cost.
	 */
	BlockResult ensure_current_block() noexcept;

	/**
	 * @brief Bookkeep insertion pointer, bookkeep insertion block if needed.
	 * @param avoid_costly_ops if true, don't perform any allocation.
	 *
	 * @return The number of costly operation done.
	 */
	size_t update_next(bool avoid_costly_ops) noexcept;

	/**
	 * @brief Bookkeep prefault pointer.
	 */
	void update_prefault() noexcept;

	/**
	 * @brief Destruct all elements in the list.
	 */
	void destruct_all() noexcept;

	/**
	 * @brief Release all memory owned by the list.
	 */
	void release_all() noexcept;

  public:
	/**
	 * @struct iteratorBase
	 * @brief Iterator base for List.
	 * @warning Invalidated on list move.
	 * @tparam isConst true for const_iterator, false for iterator.
	 */
	template <bool isConst> struct iteratorBase {
		using difference_type =
		    std::ptrdiff_t; /**< Type for representing differences between
		                       iterators */
		using value_type =
		    std::conditional_t<isConst, const T,
		                       T>; /**< Element type (either const T or T) */
		using reference = value_type& /**< reference to element type */;
		using pointer   = value_type* /**< pointer to element type */;
		using iterator_category =
		    std::forward_iterator_tag; /**< Iterator category for STL
		                                  compatibility */
		using void_ptr_type =
		    std::conditional_t<isConst, const void,
		                       void>*; /**< Pointer to void, respecting const */
		using byte_ptr_type =
		    std::conditional_t<isConst, const std::byte,
		                       std::byte>*; /**< Pointer to std::byte,
		                                       respecting const */
		const List* list;                   /**< Pointer to owning List */
		const BlockListNode* current;       /**< Current block pointer */
		void_ptr_type cur_ptr;              /**< Current element address */

		/**
		 * @brief Default constructor for an iteratorBase.
		 * Constructs an iterator pointing past the end of an empty list
		 */
		iteratorBase() noexcept;

		/**
		 * @brief Construct an iteratorBase with specified list, block, and
		 * pointer.
		 * @param l Pointer to the owning List
		 * @param blk Pointer to the current BlockListNode
		 * @param ptr Pointer to the current element
		 */
		iteratorBase(const List* l, const BlockListNode* blk,
		             void_ptr_type ptr) noexcept;

		/**
		 * @brief Pre-increment operator (move to next element).
		 * @return Reference to the incremented iterator.
		 */
		iteratorBase& operator++() noexcept;

		/**
		 * @brief Post-increment operator (move to next element).
		 * @return Iterator before incrementing.
		 */
		iteratorBase operator++(int) noexcept;

		/**
		 * @brief Dereference operator.
		 * @return Reference to the current element.
		 */
		reference operator*() const noexcept;

		/**
		 * @brief Arrow operator.
		 * @return Pointer to the current element.
		 */
		pointer operator->() const noexcept;

		/**
		 * @brief Equality comparison.
		 * @param rhs Other iterator to compare.
		 * @return true if both iterators are equal (point to the same element).
		 */
		bool operator==(const iteratorBase& rhs) const noexcept;

		/**
		 * @brief Inequality comparison.
		 * @param rhs Other iterator to compare.
		 * @return true if iterators are not equal.
		 */
		bool operator!=(const iteratorBase& rhs) const noexcept;
	};

	using iterator       = iteratorBase<false>; /**< Mutable iterator type */
	using const_iterator = iteratorBase<true>;  /**< Const iterator type */

	/**
	 * @brief Return iterator to beginning.
	 */
	iterator begin() noexcept;
	/**
	 * @brief Return const_iterator to beginning.
	 */
	const_iterator begin() const noexcept;
	/**
	 * @brief Return const_iterator to beginning.
	 */
	const_iterator cbegin() const noexcept;
	/**
	 * @brief Return past the end iterator.
	 */
	iterator end() noexcept;
	/**
	 * @brief Return past the end const_iterator.
	 */
	const_iterator end() const noexcept;
	/**
	 * @brief Return past the end const_iterator.
	 */
	const_iterator cend() const noexcept;

	/**
	 * @brief Pre-allocate additional elements.
	 *
	 * Allocate blocks until at least n more elements worth of space is
	 * available.
	 *
	 * All iterators remain valid after calling this function.
	 *
	 * @param n Number of extra elements to reserve.
	 */
	void reserve_more(size_t n) noexcept;

	/**
	 * @brief Result of append operation.
	 */
	struct AppendResult {
		iterator appended; /**< Iterator to newly appended element */
		size_t costly_ops; /**< Costly operations performed */
	};

	/**
	 * @brief Append a new element in-place. Amortized O(1). Will try to perform
	 * at most a single "costly" operation (allocating memory or touching a new
	 * page).
	 *
	 * Constructs `T(args...)` at the next available slot.
	 * All iterators remain valid after calling this function.
	 * If an exception is thrown when calling T's constructor, the exception is
	 * thrown and no element is appended. Possibly allocated memory is not
	 * released, but the external state of the list remains the same as before
	 * this function was called.
	 * Never throws unless T's constructor does, on any
	 * other internal error this function returns an end() iterator.
	 *
	 * @tparam Args Parameter pack for T constructor.
	 * @param args Arguments forwarded to T constructor.
	 * @return AppendResult with iterator to inserted element or end() if
	 * insertion failed, and the number of costly operations performed.
	 */
	template <typename... Args>
	AppendResult
	append(Args&&... args) noexcept(noexcept(T(std::forward<Args>(args)...)));

	/**
	 * @brief Perform one prefault touch if enabled.
	 *
	 * @return 1 if prefaulted, 0 otherwise.
	 */
	size_t do_prefault() noexcept;

	/**
	 * @brief Get number of elements stored.
	 * @return Current size.
	 */
	[[nodiscard]] size_t size() const noexcept;

	/**
	 * @brief Check if the list is empty.
	 * @return true if the list has no elements.
	 */
	[[nodiscard]] bool empty() const noexcept;

	/**
	 * @brief Validate internal state.
	 * @return true if container has been initialized.
	 */
	[[nodiscard]] bool is_valid() const noexcept;

	/**
	 * @brief Clear the container, calling the destructor for all inserted
	 * elements if NeedsDestructor_v<T> is true. Does not release memory.
	 */
	void clear() noexcept;

	/**
	 * @brief Destructor, calls the destructor for all inserted
	 * elements if NeedsDestructor_v<T> is true. Releases all memory
	 */
	~List() noexcept;
};
} // namespace low_latency_list
#ifndef LOW_LATENCY_LIST_INL
#include "low_latency_list.inl"
#endif
#endif /* LOW_LATENCY_LIST_H */
