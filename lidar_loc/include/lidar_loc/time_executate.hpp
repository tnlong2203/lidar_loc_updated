#ifndef TIME_EXECUATE_HPP
#define TIME_EXECUATE_HPP
#include "chrono"
#include "string"
class ExecutionTimer{
    public:
        explicit ExecutionTimer(const std::string& name): name_(name), start_time_(std::chrono::high_resolution_clock::now()) {}
        ~ExecutionTimer() {
            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed_time = std::chrono::duration<double>(end_time - start_time_).count();
            printf("Excutable time of %s: %.6f seconds\n", name_.c_str(), elapsed_time);
        }
    private:
        std::chrono::high_resolution_clock::time_point start_time_; 
        std::string name_;
};
#endif //TIME_EXECUATE_HPP