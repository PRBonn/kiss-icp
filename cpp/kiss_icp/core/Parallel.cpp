#include "Parallel.hpp"

namespace kiss_icp::parallel {
NParallel::NParallel(int num_threads) : number_of_threads_(num_threads), arena_(number_of_threads_) {}

auto NParallel::get_max_threads() -> int {
    return number_of_threads_;
}

}
