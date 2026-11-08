#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H
#include "utils/cmp_tme_str.h"
#include "utils/cmp_tme_str_utils.h"
#include "utils/named_tuple.h"
#include <concepts>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <ios>
#include <list>
#ifndef NO_LOW_LATENCY_LIST
#include <low_latency_list.h>
#endif
#include <ostream>
#include <sstream>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

namespace data_logger {
using cmp_tme_str::CTStr;
using named_tuple::traits::CvrefNamedRef;
using named_tuple::traits::NamedRef;
using std::false_type;
using std::true_type;

template <typename Container> struct append_list_traits;

template <typename T, typename Alloc>
struct append_list_traits<std::list<T, Alloc>> {
	using container      = std::list<T, Alloc>;
	using append_result  = container::iterator;
	using iterator       = container::iterator;
	using const_iterator = container::const_iterator;
	template <typename... Args>
	    requires(std::constructible_from<T, Args && ...>)
	static append_result append(container& list, Args&&... args) {
		try {
			list.emplace_back(std::forward<Args>(args)...);
			return --list.end();
		} catch (...) {
			return list.end();
		}
	}
	static void clear(container& list) {
		list.clear();
	}

};

#ifndef NO_LOW_LATENCY_LIST
template <typename T> struct append_list_traits<low_latency_list::List<T>> {
	using container      = low_latency_list::List<T>;
	using append_result  = container::iterator;
	using iterator       = container::iterator;
	using const_iterator = container::const_iterator;
	template <typename... Args>
	    requires(std::constructible_from<T, Args && ...>)
	static append_result append(container& list, Args&&... args) {
		return list.append(std::forward<Args>(args)...).appended;
	}
	static void clear(container& list) {
		list.clear();
	}
};
#endif

namespace csv {
namespace utils {
using cmp_tme_str::concat;
using cmp_tme_str::CTStr_list;
using cmp_tme_str::CTStrAdaptable;
using cmp_tme_str::CTStrList;
template <CTStrAdaptable Pref, CTStrList List>
constexpr auto prepend_to_all(const Pref& pref, const List& list) {
	return [&pref, &list]<size_t... I>(std::index_sequence<I...>) {
		return CTStr_list(concat(pref, get<I>(list.strings))...);
	}(std::make_index_sequence<List::size>());
}

} // namespace utils
using named_tuple::get;
using named_tuple::Leaf;

using std::get;
using std::tuple;

template <typename T> struct csv_traits {
	static constexpr bool introspect = false;
};
template <Leaf... Leaves>
struct csv_traits<named_tuple::named_tuple<Leaves...>> {
	static constexpr bool introspect = true;
	static constexpr size_t num_elems =
	    std::tuple_size_v<named_tuple::named_tuple<Leaves...>>;
	template <size_t I>
	    requires(I < num_elems)
	static constexpr auto suffix_for =
	    concat('.', get<I>(named_tuple::named_tuple<Leaves...>::names));
	template <size_t I>
	    requires(I < num_elems)
	using type_of =
	    std::tuple_element_t<I, named_tuple::named_tuple<Leaves...>>;
	template <size_t I>
	    requires(I < num_elems)
	static constexpr decltype(auto)
	get(const named_tuple::named_tuple<Leaves...>& tup) {
		return named_tuple::get<I>(tup);
	}
};

template <typename... Types> struct csv_traits<tuple<Types...>> {
	static constexpr bool introspect  = true;
	static constexpr size_t num_elems = std::tuple_size_v<tuple<Types...>>;
	template <size_t I>
	    requires(I < num_elems)
	static constexpr auto suffix_for = concat('_', cmp_tme_str::nstr<I>);
	template <size_t I>
	    requires(I < num_elems)
	using type_of = std::tuple_element_t<I, tuple<Types...>>;
	template <size_t I>
	    requires(I < num_elems)
	static constexpr decltype(auto) get(const tuple<Types...>& tup) {
		return std::get<I>(tup);
	}
};

template <typename T, size_t N> struct csv_traits<std::array<T, N>> {
	static constexpr bool introspect  = true;
	static constexpr size_t num_elems = N;
	template <size_t I>
	    requires(I < num_elems)
	static constexpr auto suffix_for = concat('[', cmp_tme_str::nstr<I>, ']');
	template <size_t I>
	    requires(I < num_elems)
	using type_of = T;
	template <size_t I>
	    requires(I < num_elems)
	static constexpr decltype(auto) get(const std::array<T, N>& arr) {
		return std::get<I>(arr);
	}
};

template <typename A, typename B> struct csv_traits<std::pair<A, B>> {
	static constexpr bool introspect  = true;
	static constexpr size_t num_elems = 2;
	template <size_t I>
	    requires(I <= 2)
	static constexpr auto suffix_for =
	    get<I>(std::make_tuple(CTStr(".first"), CTStr(".second")));
	template <size_t I>
	    requires(I < num_elems)
	using type_of = std::tuple_element_t<I, std::pair<A, B>>;
	template <size_t I>
	    requires(I < num_elems)
	static constexpr decltype(auto) get(const std::pair<A, B>& arr) {
		return std::get<I>(arr);
	}
};

template <typename T, CTStr name> consteval auto get_headers_list() {
	using cmp_tme_str::CTStr_list;
	using traits = csv_traits<T>;
	if constexpr (traits::introspect) {
		return []<size_t... I>(std::index_sequence<I...>) {
			return utils::prepend_to_all(
			    name,
			    cmp_tme_str::concat_lists(
			        get_headers_list<typename traits::template type_of<I>,
			                         traits::template suffix_for<I>>()...));
		}(std::make_index_sequence<traits::num_elems>());
	} else {
		return CTStr_list(name);
	}
}

template <typename T> std::string to_csv(const T& t) {
	using traits = csv_traits<T>;
	std::stringstream res;
	if constexpr (traits::introspect) {
		[&res, &t]<size_t... I>(std::index_sequence<I...>) {
			(((I ? res << "," : res)
			  << to_csv(csv_traits<T>::template get<I>(t))),
			 ...);
		}(std::make_index_sequence<traits::num_elems>());
	} else {
		res << t;
	}
	return res.str();
}
} // namespace csv
template <typename T, CTStr Name> struct field {
	static constexpr auto name = Name;
	using type                 = T;
};

template <typename T> struct is_field : false_type {};
template <typename T, CTStr name>
struct is_field<field<T, name>> : true_type {};

template <typename T> constexpr bool is_field_v = is_field<T>::value;

template <typename T>
concept Field = is_field_v<T>;

template <Field F>
using leaf_for = named_tuple::leaf<F::name, typename F::type>;

template <Field... Fields>
    requires(cmp_tme_str::all_unique_str_v<Fields::name...>)
struct field_spec {
	static constexpr auto names = cmp_tme_str::CTStr_list{Fields::name...};
	static constexpr size_t num_fields = sizeof...(Fields);
	using sample_type = named_tuple::named_tuple<leaf_for<Fields>...>;
	using source_tuple =
	    named_tuple::named_tuple<named_tuple::leaf<Fields::name, size_t>...>;
	static constexpr auto header = cmp_tme_str::join(
	    ",",
	    cmp_tme_str::concat_lists(
	        csv::get_headers_list<typename Fields::type, Fields::name>()...));
	template <CvrefNamedRef... Refs>
	static constexpr bool check_refs =
	    sizeof...(Refs) == num_fields &&
	    cmp_tme_str::all_unique_str_v<std::remove_cvref_t<Refs>::name...> &&
	    (cmp_tme_str::contains_str_v<std::remove_cvref_t<Refs>::name,
	                                 Fields::name...> &&
	     ...) &&
	    (std::constructible_from<typename sample_type::template leaf_by_name<
	                                 std::remove_cvref_t<Refs>::name>::type,
	                             typename std::remove_cvref_t<Refs>::type&&> &&
	     ...);
};

template <typename T> struct is_field_spec : false_type {};

template <Field... Fields>
struct is_field_spec<field_spec<Fields...>> : true_type {};

template <typename T> constexpr bool is_field_spec_v = is_field_spec<T>::value;

template <typename T>
concept FieldSpec = is_field_spec_v<T>;

template <FieldSpec Spec,
          typename Container =
#ifndef NO_LOW_LATENCY_LIST
              low_latency_list::List<typename Spec::sample_type>
#else
              std::list<typename Spec::sample_type>
#endif
          ,
          typename Traits = append_list_traits<Container>>
struct data_logger {
	using sample_type = Spec::sample_type;
	Container samples;

	template <typename... Args>
	    requires(std::constructible_from<Container, Args && ...>)
	data_logger(Args&&... args) : samples(std::forward<Args>(args)...) {
	}

	template <CvrefNamedRef... Refs>
	    requires(Spec::template check_refs<Refs...>)
	void log(Refs&&... refs) {
		constexpr auto source = []() {
			typename Spec::source_tuple res;
			size_t i = 0;
			((named_tuple::get<std::remove_cvref_t<Refs>::name>(res) = i, ++i),
			 ...);
			return res;
		}();
		auto fw = std::forward_as_tuple(std::forward<Refs>(refs).ref...);
		[&fw, &source, this]<size_t... I>(std::index_sequence<I...>) {
			Traits::append(
			    samples,
			    get<named_tuple::get<get<I>(Spec::names.strings)>(source)>(
			        fw)...);
		}(std::make_index_sequence<Spec::num_fields>());
	}

	std::ostream& save_csv(std::ostream& os, bool include_header = true) const {
		if (include_header) {
			os << std::string_view(Spec::header) << '\n';
		}
		for (const auto& sample : samples) {
			os << csv::to_csv(sample) << '\n';
		}
		return os;
	}

	bool save_csv(const std::filesystem::path& path, bool include_header = true,
	              std::ios_base::openmode mode = std::ios_base::out |
	                                             std::ios_base::trunc) const {
		std::ofstream stream(path, mode);
		if (!stream.good())
			return false;
		save_csv(stream, include_header);
		return stream.good();
	}

	void reset() {
		Traits::clear(samples);
	}
};

template <Field... Fields>
using DataLogger = data_logger<field_spec<Fields...>>;

} // namespace data_logger

#endif /* ifndef DATA_LOGGER_H */
