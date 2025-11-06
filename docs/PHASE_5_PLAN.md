# Phase 5: Algorithm and Computational Optimizations - PLANNING

**Status**: 📋 Planning  
**Branch**: develop  
**Prerequisites**: Phases 1-4 complete

## Overview

Phase 5 focuses on algorithmic and computational optimizations that go beyond ROS 2 infrastructure improvements. These are deeper optimizations targeting the core estimation algorithms and data processing pipelines.

## Planned Tasks

### 1. GPU Acceleration with CUDA

**Goal**: Offload point-to-plane correspondence and Jacobian computation to GPU.

**Benefits**:
- 5-10x speedup for correspondence finding
- Parallel processing of 10,000+ points
- Frees CPU for other tasks

**Implementation**:
```cpp
// CUDA kernel for point transformation
__global__ void transformPoints(
    const float* points_in,
    const float* q_itp,
    const float* t_itp,
    float* points_out,
    int num_points)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < num_points) {
        // Transform point using quaternion and translation
        // ... CUDA optimized quaternion rotation ...
    }
}

// Wrapper function
void Association::findCorrespGPU(
    Eigen::aligned_deque<PointData>& pt_meas,
    SplineState* spl,
    KD_TREE<pcl::PointXYZINormal>* ikdtree)
{
    // Copy to GPU
    cudaMemcpy(d_points, h_points, size, cudaMemcpyHostToDevice);
    
    // Launch kernel
    transformPoints<<<blocks, threads>>>(d_points, d_q, d_t, d_output, num_points);
    
    // Copy back
    cudaMemcpy(h_output, d_output, size, cudaMemcpyDeviceToHost);
}
```

**Requirements**:
- CUDA toolkit installed
- NVIDIA GPU with compute capability 6.0+
- CMake CUDA support

**Challenges**:
- ikd-tree is CPU-based (may need GPU k-d tree implementation)
- Memory transfer overhead (mitigate with pinned memory)
- Maintaining Eigen compatibility

**Estimated Impact**: 5-10x speedup in correspondence finding

---

### 2. SIMD Vectorization (AVX2/AVX512)

**Goal**: Explicit SIMD vectorization of critical loops beyond what Eigen provides.

**Benefits**:
- 4-8x speedup for vector operations
- Better cache utilization
- Works on CPUs without GPU

**Implementation**:
```cpp
#include <immintrin.h>

// AVX2 vectorized point transformation
void transformPointsAVX2(
    const Eigen::Vector3d* points_in,
    const Eigen::Quaterniond& q,
    const Eigen::Vector3d& t,
    Eigen::Vector3d* points_out,
    size_t num_points)
{
    // Process 4 points at a time with AVX2
    __m256d q_vec = _mm256_set_pd(q.w(), q.z(), q.y(), q.x());
    
    for (size_t i = 0; i < num_points; i += 4) {
        // Load 4 points
        __m256d px = _mm256_load_pd(&points_in[i].x());
        __m256d py = _mm256_load_pd(&points_in[i].y());
        __m256d pz = _mm256_load_pd(&points_in[i].z());
        
        // Quaternion rotation (vectorized)
        // ... optimized quaternion math ...
        
        // Store result
        _mm256_store_pd(&points_out[i].x(), result_x);
    }
}
```

**Target Loops**:
- `Association::findCorresp()` point transformations
- `Estimator::updateLiDAR()` Jacobian computations
- `transformCloud()` in Mapping

**Estimated Impact**: 2-4x speedup in point processing

---

### 3. Adaptive Thread Scaling

**Goal**: Dynamically adjust OpenMP thread count based on workload.

**Benefits**:
- Optimal CPU utilization
- Reduces over-subscription on light loads
- Better power efficiency

**Implementation**:
```cpp
class AdaptiveThreading {
    int computeOptimalThreads(size_t num_points) {
        // Heuristic: more threads for larger point clouds
        if (num_points < 1000) return 2;
        if (num_points < 5000) return 4;
        if (num_points < 10000) return 8;
        return num_threads_max_;
    }
    
    void updateThreadCount(size_t workload) {
        int optimal = computeOptimalThreads(workload);
        omp_set_num_threads(optimal);
    }
};

// In updateIEKFLiDAR:
adaptive_threading_.updateThreadCount(pt_meas.size());
estimator.updateIEKFLiDAR(pt_meas, ...);
```

**Estimated Impact**: 10-20% CPU efficiency improvement

---

### 4. Covariance Matrix Caching

**Goal**: Cache and reuse covariance matrix computations across iterations.

**Benefits**:
- Reduces redundant linear algebra operations
- Faster IEKF convergence checks
- Lower memory bandwidth

**Implementation**:
```cpp
class CovarianceCache {
    std::unordered_map<int64_t, Eigen::MatrixXd> cache_;
    
    const Eigen::MatrixXd& getOrCompute(
        int64_t timestamp,
        std::function<Eigen::MatrixXd()> compute_fn)
    {
        auto it = cache_.find(timestamp);
        if (it != cache_.end()) {
            return it->second;  // Cache hit
        }
        // Cache miss - compute and store
        cache_[timestamp] = compute_fn();
        return cache_[timestamp];
    }
};
```

**Cache Strategy**:
- LRU eviction when cache size exceeds threshold
- Clear cache after spline window update
- Key by timestamp + control point ID

**Estimated Impact**: 15-25% reduction in linear algebra operations

---

### 5. Sparse Matrix Optimizations

**Goal**: Exploit sparsity in Jacobian matrices for faster computation.

**Benefits**:
- Reduced memory usage
- Faster matrix operations
- Better cache performance

**Implementation**:
```cpp
#include <Eigen/Sparse>

// Use sparse matrices for large Jacobians
Eigen::SparseMatrix<double> H_sparse(num_measurements, STATE_SIZE);
std::vector<Eigen::Triplet<double>> triplets;

// Build sparse matrix efficiently
for (size_t i = 0; i < pt_meas.size(); i++) {
    const PointData& pt = pt_meas[i];
    // Only non-zero entries
    for (int j = 0; j < 24; j++) {
        if (std::abs(pt.H(j)) > 1e-12) {
            triplets.emplace_back(i, j, pt.H(j));
        }
    }
}
H_sparse.setFromTriplets(triplets.begin(), triplets.end());

// Sparse linear solve
Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
solver.compute(H_sparse.transpose() * H_sparse);
Eigen::VectorXd x = solver.solve(H_sparse.transpose() * innov);
```

**Estimated Impact**: 30-50% speedup for large measurement sets

---

### 6. Lock-Free Data Structures

**Goal**: Replace mutex-protected buffers with lock-free queues.

**Benefits**:
- Eliminates lock contention
- Better multi-LiDAR scalability
- Lower latency

**Implementation**:
```cpp
#include <boost/lockfree/queue.hpp>

class LidarData {
    // Lock-free queue instead of mutex + deque
    boost::lockfree::queue<PointCloud> pc_buff{100};
    boost::lockfree::queue<int64_t> t_buff{100};
    
    // No mutex needed
    void push(const PointCloud& pc, int64_t t) {
        pc_buff.push(pc);
        t_buff.push(t);
    }
    
    bool pop(PointCloud& pc, int64_t& t) {
        return pc_buff.pop(pc) && t_buff.pop(t);
    }
};
```

**Target Buffers**:
- `LidarData::pc_buff` and `t_buff`
- `imu_int_buff` in RESPLE

**Estimated Impact**: 20-30% reduction in lock contention overhead

---

### 7. Precomputed B-Spline Basis Functions

**Goal**: Precompute and cache B-spline basis functions for common time offsets.

**Benefits**:
- Avoids repeated basis function evaluation
- Faster spline interpolation
- Lower CPU usage

**Implementation**:
```cpp
class BasisCache {
    struct CacheEntry {
        double u;  // Normalized time
        std::array<double, 4> basis;  // B-spline basis values
    };
    
    std::vector<CacheEntry> cache_;
    
    const std::array<double, 4>& getBasis(double u) {
        // Quantize u to cache granularity
        int idx = static_cast<int>(u * cache_resolution_);
        if (idx < cache_.size()) {
            return cache_[idx].basis;
        }
        // Compute and cache
        return computeAndCache(u);
    }
};
```

**Estimated Impact**: 10-15% speedup in spline operations

---

## Implementation Priority

### High Priority (Significant Impact)
1. **Sparse Matrix Optimizations** - Large impact, moderate effort
2. **SIMD Vectorization** - Good ROI, works on all platforms
3. **Covariance Matrix Caching** - Easy win, clear benefit

### Medium Priority
4. **Adaptive Thread Scaling** - Nice optimization, low risk
5. **GPU Acceleration** - High impact but requires hardware
6. **Precomputed B-Spline Basis** - Moderate impact

### Low Priority (Specialized)
7. **Lock-Free Data Structures** - Mainly for multi-LiDAR edge cases

---

## Dependencies

### CUDA (Optional, for GPU acceleration)
```bash
# NVIDIA CUDA Toolkit
sudo apt install nvidia-cuda-toolkit

# CMake CUDA support
cmake -DCUDA_SUPPORT=ON ..
```

### SIMD Intrinsics
```cpp
// Compile with AVX2 support
add_compile_options(-mavx2 -mfma)
```

### Sparse Matrix Library
```cmake
# Eigen SparseCore (included in Eigen)
find_package(Eigen3 REQUIRED)
```

### Lock-Free Queues
```bash
sudo apt install libboost-lockfree-dev
```

---

## Performance Benchmarking

### Metrics to Track
- **Processing Rate**: Frames per second
- **Latency**: Time from sensor input to odometry output
- **CPU Usage**: Per-core utilization
- **Memory Bandwidth**: GB/s transferred
- **Cache Performance**: L1/L2/L3 hit rates

### Profiling Tools
```bash
# CPU profiling
perf record -F 99 -g -p $(pgrep resple) -- sleep 30
perf report

# Memory bandwidth
perf stat -e cache-references,cache-misses -p $(pgrep resple)

# GPU profiling (if using CUDA)
nvprof ros2 run resple resple
```

---

## Risks and Mitigations

### Risk: SIMD/GPU breaks portability
**Mitigation**: Keep CPU fallback paths, use `#ifdef` guards

### Risk: Lock-free adds complexity
**Mitigation**: Extensive testing, start with single LiDAR

### Risk: Sparse matrices slower for small problems
**Mitigation**: Adaptive switching based on problem size

### Risk: Numerical precision issues with SIMD
**Mitigation**: Validation tests against reference implementation

---

## Success Criteria

- [ ] 2x overall speedup in estimation loop
- [ ] CPU usage reduced by 30%
- [ ] Maintains numerical accuracy (< 1e-6 difference from baseline)
- [ ] All optimizations have CPU fallback paths
- [ ] Passes all existing unit and integration tests

---

## Timeline Estimate

- **Sparse Matrix Optimizations**: 2-3 days
- **SIMD Vectorization**: 3-4 days (with testing)
- **Covariance Caching**: 1-2 days
- **Adaptive Threading**: 1 day
- **GPU Acceleration**: 4-5 days (if pursued)
- **B-Spline Basis Cache**: 1-2 days
- **Lock-Free Structures**: 2-3 days

**Total**: ~14-20 days for complete Phase 5

---

## References

- [Eigen Sparse Matrix Documentation](https://eigen.tuxfamily.org/dox/group__Sparse__chapter.html)
- [Intel Intrinsics Guide](https://www.intel.com/content/www/us/en/docs/intrinsics-guide/index.html)
- [CUDA Programming Guide](https://docs.nvidia.com/cuda/cuda-c-programming-guide/)
- [OpenMP Performance Tuning](https://www.openmp.org/wp-content/uploads/openmp-webinar-Performance-Tuning.pdf)
- [Lock-Free Programming](https://preshing.com/20120612/an-introduction-to-lock-free-programming/)
