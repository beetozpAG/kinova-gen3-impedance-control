#include "cmp_tme_str.h"
#include "cmp_tme_str_utils.h"
#include <iostream>
#include <named_tuple.h>
#include <string_view>
#include <type_traits>
#include <vector>

using namespace cmp_tme_str;

constexpr CTStr k("k");

constexpr auto b = tstr<decltype(k)>;

constexpr auto s = join("a", 'b', k, nstr<9000>, b);

#define debx(...)                                                              \
	std::cout << #__VA_ARGS__ << " -> " << tstr<decltype(__VA_ARGS__)>.data()  \
	          << " : " << (__VA_ARGS__) << std::endl;

// #define debx(...)
//#define NO_LOW_LATENCY_LIST
#include "data_logger.h"
template <CTStr s, typename T> using n_leaf = named_tuple::leaf<s, T>;

template <named_tuple::Leaf... L> using n_tup = named_tuple::named_tuple<L...>;

int main() {
	debx((std::string_view)s);

	using A = n_tup<n_leaf<"a", int>, n_leaf<"bb", std::vector<int>>>;
	A ab{1, {1, 2, 3, 4}};

	auto& [a, b] = ab;
	debx(a);
	debx(b[0]);
	debx(A::has_name<"a">);
	debx(named_tuple::traits::has_leaf_with_name_v<A, "a">);
	debx(get<"a">(ab));
	debx(get<int>(ab));

	using named_tuple::operator""_nm;

	get<"bb">(ab) = {1, 3, 4};
	debx(ab != make_named_tuple("a"_nm(1), "bb"_nm(std::vector<int>{1, 3, 4})));
	for (auto& x : get<"bb">(ab))
		debx(x);
	auto x =
	    n_tup<n_leaf<"a", std::string>, n_leaf<"b", int&>, n_leaf<"c", int>>(
	        {1, 3, 4}, get<"a">(ab), get<"a">(ab));
	debx(get<"a">(x));
	auto z = std::tuple<std::string, int&>(std::string{1, 3, 4}, get<"a">(ab));
	debx(get<"a">(std::move(ab)));
	auto y = get<1>(std::make_tuple(1, 2));

	std::string s = "hello";
	auto nt =
	    make_named_tuple("a"_nm(42), "b"_nm(std::string("temp")), "c"_nm(s));
	debx(std::is_same_v<decltype(nt),
	                    n_tup<n_leaf<"a", int>, n_leaf<"b", std::string>,
	                          n_leaf<"c", std::string>>>);
	using namespace data_logger;

	DataLogger<field<float, "x">, field<array<int, 2>, "y">> logger;

	logger.log("x"_nm = 1.0, "y"_nm = std::array<int, 2>{2, 3});
	logger.log("y"_nm = std::array<int, 2>{}, "x"_nm = 4.0f);
	int numLogs = 1000000;
	for (int i = 0; i < numLogs; i++) {
		logger.log("y"_nm = std::array<int, 2>{i, i * i}, "x"_nm = i);
	}
	logger.reset();

	logger.save_csv(std::filesystem::path("data_test.csv"));

	constexpr auto test0 = cmp_tme_str::join(cmp_tme_str::concat("a"), "b");
	CTStr_list{join("a", "b", "k")};
	auto headers = csv::get_headers_list<
	    n_tup<n_leaf<"a", std::pair<int, float>>,
	          n_leaf<"b", std::array<std::array<int, 4>, 4>>>,
	    "test">();
	auto testFlat = join(",", headers);
	debx(string_view(testFlat));
	// auto headers = csv::get_headers_list<std::tuple<int>, "test">();
}
