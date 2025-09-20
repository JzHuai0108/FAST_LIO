// omp_compat.h — drop-in replacements when OpenMP is not available.
#pragma once

#ifdef _OPENMP
  #include <omp.h>
#else
  #include <chrono>

  // Wall clock time in seconds since first call (monotonic).
  inline double omp_get_wtime() {
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now();
    const auto now = clock::now();
    return std::chrono::duration<double>(now - t0).count();
  }

  // Timer granularity in seconds.
  inline double omp_get_wtick() {
    using clock = std::chrono::steady_clock;
    return static_cast<double>(clock::period::num) /
           static_cast<double>(clock::period::den);
  }

  // No-OpenMP fallbacks so code compiles unmodified.
  inline int  omp_get_max_threads()   { return 1; }
  inline int  omp_get_thread_num()    { return 0; }
  inline void omp_set_num_threads(int) { /* no-op */ }
  inline int  omp_in_parallel()       { return 0; }
#endif
