#ifndef UTILS_H
#define UTILS_H

#include <cstddef>
#include <type_traits>

namespace utils {

template <typename T> using tag = std::type_identity<T>;

template <typename... Types> struct has_duplicates;

template <typename... T>
static constexpr bool has_duplicates_v = has_duplicates<T...>::value;

template <typename To, typename From> struct copy_cvref;

template <typename To, typename From>
using copy_cvref_t = typename copy_cvref<To, From>::type;

template <typename T, typename... U> struct count;

template <typename T, typename... Ts>
static constexpr std::size_t count_v = count<T, Ts...>::value;

template <typename T, typename... U> struct contains;

template <typename T, typename... Ts>
static constexpr bool contains_v = contains<T, Ts...>::value;

template <typename T, std::size_t N, typename... U> struct appears_times;

template <typename T, std::size_t N, typename... U>
static constexpr bool appears_times_v = appears_times<T, N, U...>::value;

template <typename T, typename... U>
using appears_once = appears_times<T, 1, U...>;

template <typename T, typename... U>
static constexpr bool appears_once_v = appears_once<T, U...>::value;
} // namespace utils

// Implementation //
namespace utils {

template <typename...> struct has_duplicates : std::false_type {};
template <typename T, typename... Us>
struct has_duplicates<T, Us...>
    : std::disjunction<std::is_same<T, Us>..., has_duplicates<Us...>> {};
template <typename To, typename From> struct copy_cvref {
	using NoRef = std::remove_reference_t<From>;
	using WithConst =
	    std::conditional_t<std::is_const_v<NoRef>, std::add_const_t<To>, To>;
	using WithCV =
	    std::conditional_t<std::is_volatile_v<NoRef>,
	                       std::add_volatile_t<WithConst>, WithConst>;
	using WithLRef =
	    std::conditional_t<std::is_lvalue_reference_v<From>, WithCV&, WithCV>;
	using type =
	    std::conditional_t<std::is_rvalue_reference_v<From>, WithLRef&&, WithLRef>;
};
template <typename T, typename... U>
struct count
    : std::integral_constant<std::size_t,
                             (static_cast<std::size_t>(std::is_same_v<T, U>) +
                              ... + 0)> {};
template <typename T, typename... U>
struct contains : std::bool_constant<(count_v<T, U...> > 0)> {};
template <typename T, std::size_t N, typename... U>
struct appears_times : std::bool_constant<(count_v<T, U...> == N)> {};

template <template <typename> typename Cond, typename... Ts>
consteval size_t find_type() {
	size_t idx = -1;
	size_t i   = 0;
	((idx == size_t(-1) && Cond<Ts>::value ? idx = i : ++i), ...);
	return idx;
}

template <template <typename> typename Cond, typename... Ts>
constexpr size_t find_type_v = find_type<Cond, Ts...>();

} // namespace utils

#endif /* UTILS_H */
