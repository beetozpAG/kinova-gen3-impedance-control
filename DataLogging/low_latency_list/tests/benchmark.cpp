#include <algorithm>
#include <forward_list>
#include <iostream>
#include <chrono>
#include <vector>
#include <numeric>
#include "low_latency_list.h"

using namespace std;
using namespace std::chrono;
using low_latency_list::List;

constexpr size_t NUM_BATCHES = 100'000;
constexpr size_t ELEMS_PER_BATCH = 500;
    constexpr size_t TOTAL = NUM_BATCHES * ELEMS_PER_BATCH;

#include <x86intrin.h>
#include <cstdint>

static inline uint64_t rdtscp() {
    unsigned aux;
    return __rdtscp(&aux);
}

static inline uint64_t rdtsc_start() {
    _mm_lfence();                 // serialize before rdtsc
    return __rdtsc();
}

static inline uint64_t rdtsc_end() {
    unsigned aux;
    uint64_t r = __rdtscp(&aux);
    _mm_lfence();                 // serialize after
    return r;
}

struct TestType {
	size_t val;
	std::array<char, 250> x;
	operator size_t() {return val;}
};

template<typename F>
long long time_it(F&& func) {
    auto start = high_resolution_clock::now();
    func();
    auto end = high_resolution_clock::now();
    return duration_cast<nanoseconds>(end - start).count();
}

template<typename T>
void benchmark_low_latency_list() {
    List<T> l;
	std::vector<uint64_t> cycles(TOTAL);

		    size_t idx = 0;
    auto total_time = time_it([&] {
			for (size_t b = 0; b < NUM_BATCHES; ++b) {
					for (size_t j = 0; j < ELEMS_PER_BATCH; ++j) {
							uint64_t t0 = rdtsc_start();
							l.append(idx);
							uint64_t t1 = rdtsc_end();
							cycles[idx] = t1 - t0;
							idx++;
					}
			}
    });

    auto [min_it, max_it] = std::minmax_element(cycles.begin(), cycles.end());

    long double sum = 0;
    for (uint64_t c : cycles) sum += c;

    long double avg = sum / (long double)TOTAL;

    cout << "[LowLatencyList]\n";
    std::cout << "Min latency:  " << *min_it << " cycles\n";
    std::cout << "Avg latency:  " << (uint64_t)avg << " cycles\n";
    std::cout << "Max latency:  " << *max_it << " cycles\n\n";
    cout << "Total time: " << total_time / 1e6 << " ms\n";

    // Measure iteration
    auto iter_time = time_it([&]() {
        volatile int sink = 0;
        for (auto& val : l) {
            sink += val;
        }
    });
    cout << "Iteration time: " << iter_time / 1e6 << " ms\n\n";
}

template<typename T>
void benchmark_std_forward_list() {
    std::forward_list<T> l;
    auto it = l.before_begin();

	std::vector<uint64_t> cycles(TOTAL);

		    size_t idx = 0;
    auto total_time = time_it([&] {
			for (size_t b = 0; b < NUM_BATCHES; ++b) {
					for (size_t j = 0; j < ELEMS_PER_BATCH; ++j) {
							uint64_t t0 = rdtsc_start();
								it = l.emplace_after(it, idx);
							uint64_t t1 = rdtsc_end();
							cycles[idx] = t1 - t0;
							idx++;
					}
			}
    });

    auto [min_it, max_it] = std::minmax_element(cycles.begin(), cycles.end());

    long double sum = 0;
    for (uint64_t c : cycles) sum += c;

    long double avg = sum / (long double)TOTAL;

    cout << "[std::forward_list]\n";
    std::cout << "Min latency:  " << *min_it << " cycles\n";
    std::cout << "Avg latency:  " << (uint64_t)avg << " cycles\n";
    std::cout << "Max latency:  " << *max_it << " cycles\n\n";
    cout << "Total time: " << total_time / 1e6 << " ms\n";

    // Measure iteration
    auto iter_time = time_it([&]() {
        volatile int sink = 0;
        for (auto& val : l) {
            sink += val;
        }
    });
    cout << "Iteration time: " << iter_time / 1e6 << " ms\n\n";
}

template<typename T>
void benchmark_std_vector_reserve() {
    std::vector<T> l;
		l.reserve(TOTAL);
		std::vector<uint64_t> cycles(TOTAL);

		    size_t idx = 0;
    auto total_time = time_it([&] {
			for (size_t b = 0; b < NUM_BATCHES; ++b) {
					for (size_t j = 0; j < ELEMS_PER_BATCH; ++j) {
							uint64_t t0 = rdtsc_start();
							l.emplace_back(idx);
							uint64_t t1 = rdtsc_end();
							cycles[idx] = t1 - t0;
							idx++;
					}
			}
    });

    auto [min_it, max_it] = std::minmax_element(cycles.begin(), cycles.end());

    long double sum = 0;
    for (uint64_t c : cycles) sum += c;

    long double avg = sum / (long double)TOTAL;

    cout << "[std::vector(reserve)]\n";
    std::cout << "Min latency:  " << *min_it << " cycles\n";
    std::cout << "Avg latency:  " << (uint64_t)avg << " cycles\n";
    std::cout << "Max latency:  " << *max_it << " cycles\n\n";

    cout << "Total time: " << total_time / 1e6 << " ms\n";

    // Measure iteration
    auto iter_time = time_it([&]() {
        volatile int sink = 0;
        for (auto& val : l) {
            sink += val;
        }
    });
    cout << "Iteration time: " << iter_time / 1e6 << " ms\n\n";
}





int main() {
    benchmark_low_latency_list<int>();
    benchmark_std_forward_list<int>();
		benchmark_std_vector_reserve<int>();
    benchmark_low_latency_list<TestType>();
    benchmark_std_forward_list<TestType>();
		benchmark_std_vector_reserve<TestType>();
    return 0;
}

