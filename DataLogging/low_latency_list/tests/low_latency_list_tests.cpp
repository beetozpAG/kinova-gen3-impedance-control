#include "low_latency_list.h"
#include <gtest/gtest.h>
#include <type_traits>
#include <utility>

using namespace low_latency_list;

TEST(ListTest, AppendAndIterateIntegers) {
	List<int> list;

	const int N = 1000;
	for (int i = 0; i < N; ++i) {
		list.append(i);
	}

	int count = 0;
	for (int val : list) {
		EXPECT_EQ(val, count);
		count++;
	}

	EXPECT_EQ(count, N);
	EXPECT_EQ(list.size(), N);
}

struct DestructorTest {
	size_t* cnt;
	DestructorTest(size_t* cnt) : cnt(cnt) {
	}
	DestructorTest(const DestructorTest&) = delete;
	DestructorTest(DestructorTest&& other) noexcept
			: cnt{std::exchange(other.cnt, nullptr)} {
	}
	DestructorTest& operator=(const DestructorTest&) = delete;
	DestructorTest& operator=(DestructorTest&& other) noexcept {
		if (this != &other) {
			cnt = std::exchange(other.cnt, nullptr);
		}
		return *this;
	}

	~DestructorTest() {
		++*cnt;
	}
};

TEST(ListTest, DestroyElements) {
	size_t destroyed = 0;
	const int N = 1000;
	{
		List<DestructorTest> list;

		for (int i = 0; i < N; ++i) {
			list.append(&destroyed);
		}
	}


	EXPECT_EQ(destroyed, N);
}

struct NotDestroyTest : DestructorTest {
	using DestructorTest::DestructorTest;
};

template<>
struct low_latency_list::NeedsDestructor<NotDestroyTest> : std::false_type {};

TEST(ListTest, NotDestroyElements) {
	size_t destroyed = 0;
	const int N = 1000;
	{
		List<NotDestroyTest> list;

		for (int i = 0; i < N; ++i) {
			list.append(&destroyed);
		}
	}
	EXPECT_EQ(destroyed, 0);
}

TEST(ListTest, BeginIsEndIfEmpty) {
	List<int> list;
	EXPECT_EQ(list.begin(), list.end());
}

TEST(ListTest, MoveConstructor) {
		const int N = 1000;
    List<int> a;
    for (int i = 0; i < N; ++i) a.append(i);

    List<int> b = std::move(a);

    EXPECT_FALSE(a.is_valid());

		int count = 0;
		for (int val : b) {
			EXPECT_EQ(val, count);
			count++;
		}

		EXPECT_EQ(count, N);
}

TEST(ListTest, ClearTest) {
		const int N = 1000;
    List<int> list;
    for (int i = 0; i < N; ++i) list.append(i);
		EXPECT_EQ(list.size(), N);
		list.clear();
		EXPECT_EQ(list.size(), 0);
    for (int i = 0; i < N; ++i) list.append(i);
		EXPECT_EQ(list.size(), N);
}

