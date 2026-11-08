#ifndef NAMED_TUPLE_H
#define NAMED_TUPLE_H
#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>

#include "cmp_tme_str.h"
#include "cmp_tme_str_utils.h"
#include "utils.h"

namespace named_tuple {
using cmp_tme_str::CTStr;

namespace traits {
using std::remove_cvref_t;
template <typename Type> struct is_leaf;
template <typename T>
concept Leaf = is_leaf<T>::value;

template <typename Type> struct is_named_tuple;
template <typename T>
constexpr bool is_named_tuple_v = is_named_tuple<T>::value;
template <typename T>
concept NamedTuple = is_named_tuple_v<T>;
template <typename T>
concept CvrefNamedTuple = is_named_tuple_v<remove_cvref_t<T>>;

template <NamedTuple, Leaf> struct has_leaf;
template <NamedTuple Tup, Leaf L>
constexpr bool has_leaf_v = has_leaf<Tup, L>::value;
template <typename Tup, typename T>
concept CvrefHasLeaf = has_leaf_v<remove_cvref_t<Tup>, T>;

template <NamedTuple, CTStr> struct has_leaf_with_name;
template <NamedTuple Tup, CTStr S>
constexpr bool has_leaf_with_name_v = has_leaf_with_name<Tup, S>::value;
template <typename Tup, CTStr S>
concept CvrefHasLeafWithName = has_leaf_with_name_v<remove_cvref_t<Tup>, S>;

template <NamedTuple, typename> struct has_leaf_with_type;
template <NamedTuple Tup, typename T>
constexpr bool has_leaf_with_type_v = has_leaf_with_type<Tup, T>::value;
template <typename Tup, typename T>
concept CvrefHasLeafWithType = has_leaf_with_type_v<remove_cvref_t<Tup>, T>;

template <NamedTuple Tup, size_t I> struct has_leaf_with_idx;
template <NamedTuple Tup, size_t I>
constexpr bool has_leaf_with_idx_v = has_leaf_with_idx<Tup, I>::value;
template <typename Tup, size_t I>
concept CvrefHasLeafWithIdx = has_leaf_with_idx_v<remove_cvref_t<Tup>, I>;

template <typename> struct is_named_ref;

template <typename T> constexpr bool is_named_ref_v = is_named_ref<T>::value;
template <typename T>
concept NamedRef = is_named_ref_v<T>;
template <typename T>
concept CvrefNamedRef = is_named_ref_v<remove_cvref_t<T>>;

template <CTStr s> struct LeafNamed;
template <typename T> struct LeafTyped;
} // namespace traits
using traits::CvrefHasLeaf;
using traits::CvrefHasLeafWithIdx;
using traits::CvrefHasLeafWithName;
using traits::CvrefHasLeafWithType;
using traits::CvrefNamedRef;
using traits::Leaf;
using traits::NamedTuple;

template <CTStr str, typename T> struct leaf {
	static constexpr auto name = str;
	using type                 = T;
	T data;
};
template <Leaf... Leaves>
    requires(cmp_tme_str::all_unique_str_v<Leaves::name...>)
struct named_tuple : Leaves... {
	static constexpr size_t N   = sizeof...(Leaves);
	using leaves                = std::tuple<Leaves...>;
	static constexpr auto names = std::make_tuple(Leaves::name...);
	template <template <typename> typename Cond>
	using find_leaf_t =
	    std::tuple_element_t<utils::find_type_v<Cond, Leaves...>, leaves>;
	template <Leaf L>
	static constexpr bool is_leaf = utils::contains_v<L, Leaves...>;
	template <CTStr S>
	static constexpr bool has_name =
	    cmp_tme_str::contains_str_v<S, Leaves::name...>;
	template <typename T>
	static constexpr bool has_type =
	    utils::appears_once_v<T, typename Leaves::type...>;
	using leaf_types = std::tuple<typename Leaves::type...>;
	template <CTStr name>
	    requires(has_name<name>)
	static constexpr size_t idx_by_name =
	    utils::find_type_v<traits::LeafNamed<name>::template cond, Leaves...>;
	template <typename T>
	    requires(has_type<T>)
	static constexpr size_t idx_by_type =
	    utils::find_type_v<traits::LeafTyped<T>::template cond, Leaves...>;
	template <size_t I> using leaf_by_idx = std::tuple_element_t<I, leaves>;
	template <CTStr name>
	    requires(has_name<name>)
	using leaf_by_name = leaf_by_idx<idx_by_name<name>>;
	template <typename T>
	    requires(has_type<T>)
	using leaf_by_type = leaf_by_idx<idx_by_type<T>>;

	constexpr named_tuple() noexcept(
	    (std::is_nothrow_default_constructible_v<typename Leaves::type> && ...))
	    requires((std::default_initializable<typename Leaves::type>) && ...)
	= default;
	explicit constexpr named_tuple(const Leaves::type&... fields) noexcept(
	    (std::is_nothrow_constructible_v<typename Leaves::type,
	                                     const typename Leaves::type&> &&
	     ...))
	    : Leaves(fields)... {
	}
	explicit constexpr named_tuple(Leaves::type&&... fields) noexcept(
	    (std::is_nothrow_constructible_v<typename Leaves::type,
	                                     typename Leaves::type&&> &&
	     ...))
	    requires(!(std::is_same_v < typename Leaves::type &&,
	               const typename Leaves::type& > && ...))

	    : Leaves(std::forward<typename Leaves::type>(fields))... {
	}
	template <typename... Us>
	    requires(sizeof...(Us) == N &&
	             (std::constructible_from<typename Leaves::type, Us> && ...))
	explicit constexpr named_tuple(Us&&... us) noexcept(
	    (std::is_nothrow_constructible_v<typename Leaves::type, Us> && ...))
	    : Leaves(std::forward<Us>(us))... {
	}
	friend constexpr bool operator==(named_tuple const& a,
	                                 named_tuple const& b) noexcept {
		return std::tie(static_cast<Leaves const&>(a).data...) ==
		       std::tie(static_cast<Leaves const&>(b).data...);
	}
	friend constexpr bool operator<=>(named_tuple const& a,
	                                  named_tuple const& b) noexcept {
		return std::tie(static_cast<Leaves const&>(a).data...) <=>
		       std::tie(static_cast<Leaves const&>(b).data...);
	}
};

namespace detail {
template <Leaf L, CvrefHasLeaf<L> Tup>
constexpr decltype(auto) get_leaf_data(Tup&& tup) noexcept {
	using LeafQ = utils::copy_cvref_t<L, Tup&&>;
	return (static_cast<LeafQ>(std::forward<Tup>(tup)).data);
}
} // namespace detail

template <CTStr n, CvrefHasLeafWithName<n> Tup>
constexpr decltype(auto) get(Tup&& tup) noexcept {
	return detail::get_leaf_data<
	    typename std::remove_cvref_t<Tup>::template leaf_by_name<n>>(
	    std::forward<Tup>(tup));
}

template <typename T, CvrefHasLeafWithType<T> Tup>
constexpr decltype(auto) get(Tup&& tup) noexcept {
	return detail::get_leaf_data<
	    typename std::remove_cvref_t<Tup>::template leaf_by_type<T>>(
	    std::forward<Tup>(tup));
}

template <size_t I, CvrefHasLeafWithIdx<I> Tup>
constexpr decltype(auto) get(Tup&& tup) noexcept {
	return detail::get_leaf_data<
	    typename std::remove_cvref_t<Tup>::template leaf_by_idx<I>>(
	    std::forward<Tup>(tup));
}

template <CTStr S, typename T> struct named_ref {
	static constexpr auto name = S;
	using type                 = T;
	T&& ref;
};

template <CTStr name> struct named_member {
	template <typename T> constexpr named_ref<name, T> operator()(T&& t) {
		return {std::forward<T>(t)};
	}
	template <typename T> constexpr named_ref<name, T> operator=(T&& t) {
		return {std::forward<T>(t)};
	}
};

template <CTStr name> named_member<name> consteval operator""_nm() {
	return {};
}

template <CvrefNamedRef... Refs>
    requires(
        cmp_tme_str::all_unique_str_v<std::remove_cvref_t<Refs>::name...> &&
        (std::constructible_from<
             std::unwrap_ref_decay_t<typename std::remove_cvref_t<Refs>::type>,
             decltype((std::declval<Refs &&>().ref))> &&
         ...))
constexpr auto make_named_tuple(Refs&&... refs) noexcept(
    (std::is_nothrow_constructible_v<
         std::unwrap_ref_decay_t<typename std::remove_cvref_t<Refs>::type>,
         decltype((std::forward<Refs>(refs).ref))> &&
     ...)) {
	return named_tuple<leaf<
	    std::remove_cvref_t<Refs>::name,
	    std::unwrap_ref_decay_t<typename std::remove_cvref_t<Refs>::type>>...>(
	    std::forward<Refs>(refs).ref...);
}

template <CvrefNamedRef... Refs>
    requires(
        cmp_tme_str::all_unique_str_v<std::remove_cvref_t<Refs>::name...> &&
        (std::constructible_from<typename std::remove_cvref_t<Refs>::type &&,
                                 decltype((std::declval<Refs &&>().ref))> &&
         ...))
auto forward_as_named_tuple(Refs&&... refs) noexcept((
    std::is_nothrow_constructible_v<typename std::remove_cvref_t<Refs>::type&&,
                                    decltype((std::forward<Refs>(refs).ref))> &&
    ...))

{
	return named_tuple<leaf<std::remove_cvref_t<Refs>::name,
	                        typename std::remove_cvref_t<Refs>::type&&>...>(
	    std::forward<Refs>(refs).ref...);
}
} // namespace named_tuple


namespace named_tuple::traits {
using std::false_type;
using std::true_type;

template <typename Type> struct is_leaf : false_type {};
template <CTStr name, typename Type>
struct is_leaf<leaf<name, Type>> : true_type {};

template <typename Type> struct is_named_tuple : false_type {};
template <Leaf... Leaves>
struct is_named_tuple<named_tuple<Leaves...>> : true_type {};

template <NamedTuple Tup, Leaf L>
struct has_leaf : std::bool_constant<Tup::template is_leaf<L>> {};
template <NamedTuple Tup, CTStr S>
struct has_leaf_with_name : std::bool_constant<Tup::template has_name<S>> {};
template <NamedTuple Tup, typename T>
struct has_leaf_with_type : std::bool_constant<Tup::template has_type<T>> {};
template <NamedTuple Tup, size_t I>
struct has_leaf_with_idx : std::bool_constant<(I < Tup::N)> {};
template <CTStr s> struct LeafNamed {
	template <Leaf L> using cond = std::bool_constant<s == L::name>;
};
template <typename T> struct LeafTyped {
	template <Leaf L> using cond = std::is_same<T, typename L::type>;
};
template <typename> struct is_named_ref : false_type {};
template <CTStr name, typename T>
struct is_named_ref<named_ref<name, T>> : true_type {};
} // namespace named_tuple::traits

namespace std {
template <named_tuple::NamedTuple Tup>
struct tuple_size<Tup> : integral_constant<size_t, Tup::N> {};
template <size_t I, named_tuple::NamedTuple Tup>
struct tuple_element<I, Tup> : tuple_element<I, typename Tup::leaf_types> {};
} // namespace std


#endif /* NAMED_TUPLE_H */
