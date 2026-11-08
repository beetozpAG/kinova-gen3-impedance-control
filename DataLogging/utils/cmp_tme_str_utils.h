#ifndef CMP_TME_STR_UTILS_H
#define CMP_TME_STR_UTILS_H
#include <source_location>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

#include "cmp_tme_str.h"

namespace cmp_tme_str {

template <integral auto n, bool leading = false> consteval auto nstr_impl() {
	if constexpr (n < 0) {
		return concat("-", nstr_impl<-(n / 10), true>(),
		              nstr_impl<-(n % 10)>());
	} else if constexpr (n == 0) {
		if constexpr (leading) {
			return CTStr("");
		} else {
			return CTStr('0');
		}
	} else {
		return concat(nstr_impl<n / 10, true>(), CTStr(char(n % 10 + '0')));
	}
}

template <integral auto n> constexpr auto nstr = nstr_impl<n>();

} // namespace cmp_tme_str
template <typename T> consteval const char* func_name() {
	return std::source_location::current().function_name();
}
namespace cmp_tme_str {

consteval string_view extract_between(string_view str, string_view beg,
                                      string_view end) {
	auto b       = str.find(beg);
	auto e       = str.rfind(end);
	auto beg_len = beg.size();
	if (b == std::string_view::npos || e == std::string_view::npos ||
	    b + beg_len > e) {
		return {};
	}
	return str.substr(b + beg_len, e - (b + beg_len));
}

template <typename T> consteval auto tstr_impl() {
#if __cpp_lib_source_location >= 201907L
	constexpr string_view f_name = func_name<T>();
#if defined(__clang__)
	// const char *cmp_tme_str::func_name() [T = <type>]
	constexpr auto begin_pat = "= ";
	constexpr auto end_pat   = "]";
#elif defined(__GNUC__) || defined(__GNUG__)
	// consteval const char* cmp_tme_str::func_name() [with T = <type>]
	constexpr auto begin_pat = "= ";
	constexpr auto end_pat   = "]";
#elif defined(_MSC_VER)
	// const char *__cdecl cmp_tme_str::func_name<<type>>(void)
	constexpr auto begin_pat = "<";
	constexpr auto end_pat   = ">";
#endif
	constexpr auto name = extract_between(f_name, begin_pat, end_pat);
	array<char, name.size() + 1> res = {};
	std::copy_n(name.data(), name.size(), res.begin());
	return CTStr(res);
#else
	return CTStr();
#endif
}
template <typename T> constexpr auto tstr = tstr_impl<T>();

template <size_t... N> struct CTStr_list {
	static constexpr size_t size = sizeof...(N);
	std::tuple<CTStr<N>...> strings;
	template <CTStrAdaptable... Strs>
	constexpr CTStr_list(const Strs&... strs) noexcept : strings(strs...){}
};

template <CTStrAdaptable... Strs>
CTStr_list(const Strs&...) -> CTStr_list<(CTStrAdaptor<Strs>::len + 1)...>;

template <typename T> struct is_ctstr_list : std::false_type {};
template <size_t... N>
struct is_ctstr_list<CTStr_list<N...>> : std::true_type {};

template <typename T> constexpr bool is_ctstr_list_v = is_ctstr_list<T>::value;

template <typename T>
concept CTStrList = is_ctstr_list_v<T>;

template <CTStrList... Ls> constexpr auto concat_lists(const Ls&... lists) {
	using std::get;
	constexpr size_t tot_size = (Ls::size + ... + 0);
	constexpr auto starts     = []() {
        array<size_t, sizeof...(Ls)> res;
        size_t sum = 0;
        size_t i   = 0;
        ((res[i] = sum, ++i, sum += Ls::size), ...);
        return res;
	}();
	auto l             = std::tie(lists...);
	constexpr auto src = [starts]<size_t... I>(std::index_sequence<I...>) {
		array<std::pair<size_t, size_t>, tot_size> res;
		(
		    [&res, starts]<size_t... J>(std::index_sequence<J...>) {
			    size_t lI = I;
			    ((res[starts[lI] + J] = {lI, J}), ...);
		    }(std::make_index_sequence<Ls::size>()),
		    ...);
		return res;
	}(std::index_sequence_for<Ls...>());
	return [src, l]<size_t... I>(std::index_sequence<I...>) {
		return CTStr_list{get<get<1>(src[I])>(get<get<0>(src[I])>(l).strings)...};
	}(std::make_index_sequence<tot_size>());
}

template <CTStrList List> constexpr auto concat(const List& list) {
	return [&list]<size_t... I>(std::index_sequence<I...>) {
		return concat(get<I>(list.strings)...);
	}(std::make_index_sequence<List::size>());
}

template <CTStrAdaptable J, CTStrList List>
constexpr auto join(const J& sep, const List& list) {
	return [&list, &sep]<size_t... I>(std::index_sequence<I...>) {
		return join(sep, get<I>(list.strings)...);
	}(std::make_index_sequence<List::size>());
}

template <CTStr S, CTStr... Ts>
constexpr std::size_t count_str_v = (std::size_t(Ts == S) + ... + 0);

template <CTStr S, CTStr... Ts>
constexpr bool contains_str_v = ((Ts == S) || ...);

template <CTStr... Ts>
constexpr bool all_unique_str_v = ((count_str_v<Ts, Ts...> == 1) && ...);

} // namespace cmp_tme_str
#endif /* ifndef CMP_TME_STR_UTILS_H */
