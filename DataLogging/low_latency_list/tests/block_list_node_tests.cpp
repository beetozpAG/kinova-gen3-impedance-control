#include "block_list_node.h"
#include <gtest/gtest.h>
#include <set>

using namespace block_list_node;

using Allocator = BlockListNodeAllocator;

// Test that single allocation returns non-null and unique pointer
TEST(AllocatorTest, SingleAllocate) {
	Allocator alloc;
	auto res = alloc.allocate();
	EXPECT_NE(res.allocated, nullptr);
	EXPECT_EQ(res.costly_ops, 0);
}

// Test that allocate and deallocate reuses nodes
TEST(AllocatorTest, AllocateDeallocateReuse) {
	Allocator alloc;
	auto r1 = alloc.allocate();
	auto p1 = r1.allocated;
	alloc.deallocate(p1);
	auto r2 = alloc.allocate();
	EXPECT_EQ(r2.allocated, p1);
}

// Test enforcing minimum super-block size (at least 2 nodes)
TEST(AllocatorTest, MinimumBlockSize) {
	// Request smaller than two nodes worth of space
	size_t smallSize = sizeof(BlockListNode);
	Allocator alloc(smallSize);
	// Should at least allow one allocation (header already constructed)
	auto r1 = alloc.allocate();
	EXPECT_NE(r1.allocated, nullptr);
}

// Test expensive counter increments when superblocks are appended
TEST(AllocatorTest, ExpensiveCountIncrements) {
	// Force small block to trigger super block allocations
	size_t blockSize = sizeof(BlockListNode) * 2;
	Allocator alloc(blockSize);
	// First allocate causes a new super block allocation
	auto r1 = alloc.allocate(); // idx=1
	EXPECT_GE(r1.costly_ops, 1);
}

// Test that moving allocator transfers pool
TEST(AllocatorTest, MoveConstructorTransfersState) {
	Allocator a1;
	auto p = a1.allocate().allocated;
	Allocator a2(std::move(a1));
	auto r = a2.allocate();
	EXPECT_NE(r.allocated, nullptr);
	// a1.allocate should now allocate on a2's state
	EXPECT_EQ(a1.allocate().allocated, nullptr);
}

// Stress test: allocate many nodes and ensure uniqueness
TEST(AllocatorTest, ManyAllocationsUnique) {
	const size_t N = 10000;
	Allocator alloc;
	std::set<BlockListNode*> seen;
	for (size_t i = 0; i < N; ++i) {
		auto node = alloc.allocate().allocated;
		ASSERT_NE(node, nullptr);
		seen.emplace(node);
	}
	EXPECT_EQ(seen.size(), N);
}
