#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>

namespace kiss_icp::parallel {

class NParallel{
public:
    NParallel(int n);
    auto get_max_threads() -> int;

    template <typename Index, typename Function>
    void n_for(Index begin, Index end, const Function& func);

private:
    int number_of_threads_;
    tbb::task_arena arena_;
};

template <typename Index, typename Function>
void NParallel::n_for(Index begin, Index end, const Function& func) {
    arena_.execute([&]() {
        tbb::parallel_for(begin, end, func);
    });
}

}  // namespace kiss_icp::parallel
