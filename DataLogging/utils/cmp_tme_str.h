#ifndef CMP_TME_STR_H
#define CMP_TME_STR_H
#include <algorithm>
#include <array>
#include <compare>
#include <concepts>
#include <cstddef>
#include <cstring>
#include <string_view>
#include <tuple>

namespace cmp_tme_str {
using std::array;
using std::integral;
using std::size_t;
using std::string_view;
using std::strong_ordering;

namespace adaptor {

template <typename T> struct CTStrAdaptor {
	static constexpr bool adaptable = false;
};

} // namespace adaptor

using adaptor::CTStrAdaptor;

template <typename T>
concept CTStrAdaptable = CTStrAdaptor<T>::adaptable;

template <CTStrAdaptable T> constexpr auto get_ctstr_data(const T& t) {
	return CTStrAdaptor<T>::data(t);
}

template <size_t N>
    requires(N > 0)
struct CTStr : array<char, N> {
	static constexpr size_t len = N - 1;
	using array<char, N>::data;
	static constexpr size_t size() {
		return len;
	}
	constexpr const char* c_str() {
		return data();
	}
	constexpr CTStr() {
		data()[len] = '\0';
	};
	constexpr CTStr(const char (&str)[N]) {
		std::copy_n(str, len, data());
		data()[len] = '\0';
	}
	constexpr CTStr(const array<char, N>& str) {
		std::copy_n(str.data(), len, data());
		data()[len] = '\0';
	}
	constexpr CTStr(const char& c)
	    requires(N == 2)
	    : array<char, N>{c, '\0'} {
	}
	constexpr operator array<char, N>&() {
		return *this;
	}
	constexpr operator const array<char, N>&() const {
		return *this;
	}

	constexpr operator string_view() const {
		return {data(), len};
	}
};

CTStr() -> CTStr<1>;
CTStr(const char& c) -> CTStr<2>;

template <CTStrAdaptable LHS, CTStrAdaptable RHS>
constexpr strong_ordering operator<=>(const LHS& lhs, const RHS& rhs) {
	return std::lexicographical_compare_three_way(
	    get_ctstr_data(lhs), get_ctstr_data(lhs) + CTStrAdaptor<LHS>::len,
	    get_ctstr_data(rhs), get_ctstr_data(rhs) + CTStrAdaptor<RHS>::len);
}
template <CTStrAdaptable LHS, CTStrAdaptable RHS>
constexpr bool operator==(const LHS& lhs, const RHS& rhs) {
	if constexpr (CTStrAdaptor<LHS>::len == CTStrAdaptor<RHS>::len) {
		return (lhs <=> rhs) == std::strong_ordering::equal;
	} else {
		return false;
	}
}

namespace adaptor {

template <size_t N> struct CTStrAdaptor<char[N]> {
	static constexpr bool adaptable = true;
	static constexpr size_t len     = N - 1;
	static constexpr const char* data(const char (&adapted)[N]) {
		return adapted;
	}
};
template <size_t N> struct CTStrAdaptor<array<char, N>> {
	static constexpr bool adaptable = true;
	static constexpr size_t len     = N - 1;
	static constexpr const char* data(const array<char, N>& adapted) {
		return adapted.data();
	}
};
template <size_t N> struct CTStrAdaptor<CTStr<N>> {
	static constexpr bool adaptable = true;
	static constexpr size_t len     = N - 1;
	static constexpr const char* data(const CTStr<N>& adapted) {
		return adapted.data();
	}
};

template <> struct CTStrAdaptor<char> {
	static constexpr bool adaptable = true;
	static constexpr size_t len     = 1;
	static constexpr const char* data(const char& adapted) {
		return &adapted;
	}
};
} // namespace adaptor

template <CTStrAdaptable... T>
constexpr auto total_len = (0 + ... + CTStrAdaptor<T>::len);

template <CTStrAdaptable... T> constexpr auto concat(const T&... strs) {
	CTStr<total_len<T...> + 1> res;

	size_t index = 0;
	((std::copy_n(get_ctstr_data(strs), CTStrAdaptor<T>::len,
	              res.data() + index),
	  index += CTStrAdaptor<T>::len),
	 ...);
	return res;
}

template <CTStrAdaptable J, CTStrAdaptable... Ts>
constexpr auto join(const J& sep, const Ts&... strs) {
	if constexpr (sizeof...(strs) == 0) {
		return CTStr();
	} else {
		constexpr auto pat = []() {
			array<size_t, 2 * sizeof...(strs) - 1> p{};
			for (size_t i = 0; i < sizeof...(strs); ++i) {
				p[2 * i] = i + 1;
			}
			return p;
		}();
		auto tup = std::tie(sep, strs...);
		return [pat, &tup]<size_t... I>(std::index_sequence<I...>) {
			return concat(get<pat[I]>(tup)...);
		}(std::make_index_sequence<pat.size()>());
	}
}

} // namespace cmp_tme_str
#endif /* ifndef CMP_TME_STR_H */
