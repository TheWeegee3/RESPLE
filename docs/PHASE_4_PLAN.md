# Phase 4: Advanced ROS 2 Features - PLANNING

**Status**: 📋 Planning  
**Branch**: develop  
**Prerequisites**: Phases 1-3 complete

## Overview

Phase 4 focuses on advanced ROS 2 features for improved performance, diagnostics, and system integration. These optimizations leverage ROS 2 Jazzy's latest capabilities.

## Planned Tasks

### 1. Intra-process Communication (Zero-Copy)

**Goal**: Enable zero-copy communication between RESPLE and Mapping nodes when running in the same process.

**Benefits**:
- Eliminates serialization/deserialization overhead
- Reduces memory copies between nodes
- Lower latency for estimate and spline messages

**Implementation**:
- Use `rclcpp::intra_process_comms` for `/est_window` topic
- Requires both nodes to be in same composable container
- Update launch files to use component composition

**Estimated Impact**: 30-50% reduction in message passing latency

**Challenges**:
- Requires both nodes to support shared_ptr message semantics
- May affect external subscribers (need to verify)
- Thread safety considerations with shared memory

---

### 2. Lifecycle Nodes

**Goal**: Add proper initialization, activation, and shutdown sequences using ROS 2 Lifecycle.

**Benefits**:
- Graceful startup/shutdown
- Better error recovery
- State management for sensor calibration
- Controlled activation after initialization

**Implementation**:
```cpp
class RESPLE : public rclcpp_lifecycle::LifecycleNode {
    // States: unconfigured -> inactive -> active -> finalized
    on_configure() { /* load params, setup */ }
    on_activate()  { /* start processing */ }
    on_deactivate() { /* pause processing */ }
    on_cleanup()   { /* cleanup resources */ }
    on_shutdown()  { /* final cleanup */ }
};
```

**Key Transitions**:
- `unconfigured → inactive`: Load parameters, setup buffers
- `inactive → active`: Start sensor callbacks and processing
- `active → inactive`: Pause (keep state)
- `inactive → finalized`: Cleanup and exit

**Estimated Impact**: Better reliability, easier debugging

---

### 3. Diagnostics Integration

**Goal**: Add ROS 2 diagnostics for monitoring system health.

**Benefits**:
- Real-time performance monitoring
- Early warning for degraded performance
- Integration with ROS 2 diagnostic tools

**Implementation**:
```cpp
#include <diagnostic_updater/diagnostic_updater.hpp>

diagnostic_updater::Updater diagnostics_;

void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    // Report frequency, latency, dropped frames
    if (processing_rate < expected_rate * 0.8) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, 
                     "Processing rate low");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, 
                     "System healthy");
    }
    stat.add("Processing Rate (Hz)", processing_rate);
    stat.add("IMU Buffer Size", imu_buff.size());
    stat.add("LiDAR Buffer Size", pt_meas.size());
}
```

**Metrics to Monitor**:
- Processing rate (target: 20 Hz)
- Sensor data rates
- Buffer sizes
- Computation time per frame
- Memory usage
- IEKF iteration count

**Estimated Impact**: Better observability, easier debugging in production

---

### 4. Action Server for Save Map

**Goal**: Replace service-based map saving with ROS 2 Action for long-running operations.

**Benefits**:
- Progress feedback during map saving
- Cancellation support
- Better for long-running operations

**Implementation**:
```cpp
#include <rclcpp_action/rclcpp_action.hpp>

// Define action: SaveMap.action
// ---
// request:
//   string filename
// ---
// result:
//   bool success
//   string message
// ---
// feedback:
//   float progress

class SaveMapAction : public rclcpp_action::Server<SaveMap> {
    void execute(const std::shared_ptr<GoalHandle> goal_handle) {
        // Save map with progress updates
        for (int i = 0; i < num_chunks; i++) {
            if (goal_handle->is_canceling()) {
                return; // Handle cancellation
            }
            // Save chunk
            feedback->progress = float(i) / num_chunks;
            goal_handle->publish_feedback(feedback);
        }
        result->success = true;
        goal_handle->succeed(result);
    }
};
```

**Estimated Impact**: Better UX for long map saves

---

### 5. Parameters with Validation

**Goal**: Add parameter constraints and dynamic reconfigure support.

**Benefits**:
- Prevent invalid parameter values
- Runtime parameter tuning with validation
- Better error messages

**Implementation**:
```cpp
// Declare with constraints
auto num_threads_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
num_threads_descriptor.description = "Number of OpenMP threads";
num_threads_descriptor.integer_range.resize(1);
num_threads_descriptor.integer_range[0].from_value = 1;
num_threads_descriptor.integer_range[0].to_value = 16;
num_threads_descriptor.integer_range[0].step = 1;

this->declare_parameter("num_threads", 5, num_threads_descriptor);

// Add dynamic reconfigure callback
auto param_callback = [this](const std::vector<rclcpp::Parameter>& params) {
    for (const auto& param : params) {
        if (param.get_name() == "num_threads") {
            num_threads_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Updated num_threads to %d", num_threads_);
        }
    }
    return rcl_interfaces::msg::SetParametersResult{}.set__successful(true);
};
param_callback_handle_ = this->add_on_set_parameters_callback(param_callback);
```

**Parameters to Validate**:
- `num_threads`: [1, 16]
- `num_match_points`: [3, 10]
- `nn_thresh`: [0.1, 5.0]
- `ds_scan_voxel`: [0.01, 1.0]

**Estimated Impact**: Fewer runtime errors, better tunability

---

### 6. ROS 2 Bag Recording Integration

**Goal**: Add service to start/stop bag recording programmatically.

**Benefits**:
- Selective data recording
- Triggered by system events (e.g., high IMU variance)
- Integration with autonomous workflows

**Implementation**:
```cpp
#include <rosbag2_cpp/writer.hpp>

class BagRecorder {
    rosbag2_cpp::Writer writer_;
    
    void startRecording(const std::string& filename) {
        writer_.open(filename);
        // Subscribe to topics and write to bag
    }
    
    void stopRecording() {
        writer_.close();
    }
};
```

**Estimated Impact**: Better data collection workflows

---

## Implementation Priority

### High Priority (Do First)
1. **Intra-process Communication** - Significant performance gain
2. **Diagnostics Integration** - Critical for production deployment
3. **Parameters with Validation** - Improves reliability

### Medium Priority
4. **Lifecycle Nodes** - Nice to have for production systems
5. **Action Server for Save Map** - UX improvement

### Low Priority (Future)
6. **ROS 2 Bag Recording Integration** - Workflow improvement

---

## Dependencies

### Required Packages
```bash
sudo apt install ros-jazzy-diagnostic-updater
sudo apt install ros-jazzy-rclcpp-action
sudo apt install ros-jazzy-rclcpp-lifecycle
sudo apt install ros-jazzy-rosbag2-cpp
```

### CMakeLists.txt Additions
```cmake
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosbag2_cpp REQUIRED)
```

### package.xml Additions
```xml
<depend>diagnostic_updater</depend>
<depend>rclcpp_action</depend>
<depend>rclcpp_lifecycle</depend>
<depend>rosbag2_cpp</depend>
```

---

## Testing Strategy

### For Each Feature
1. Unit tests for new components
2. Integration tests with existing system
3. Performance benchmarks (before/after)
4. Backward compatibility verification

### Test Scenarios
- Single node vs. composed nodes (intra-process)
- Lifecycle state transitions
- Diagnostic threshold triggers
- Parameter validation edge cases
- Action cancellation mid-operation

---

## Risks and Mitigations

### Risk: Intra-process breaks external subscribers
**Mitigation**: Keep inter-process topics available, test with external nodes

### Risk: Lifecycle adds complexity
**Mitigation**: Start with RESPLE only, make Mapping optional

### Risk: Diagnostics overhead
**Mitigation**: Update rate tunable, use lightweight metrics

---

## Success Criteria

- [ ] Intra-process communication working with measurable latency reduction
- [ ] Diagnostics visible in `ros2 topic echo /diagnostics`
- [ ] Lifecycle transitions tested in all states
- [ ] Parameter validation prevents invalid configs
- [ ] Action server provides progress feedback
- [ ] All features backward compatible with Phase 3

---

## Timeline Estimate

- **Intra-process Communication**: 2-3 days
- **Diagnostics Integration**: 1-2 days
- **Parameters with Validation**: 1 day
- **Lifecycle Nodes**: 2-3 days
- **Action Server**: 1-2 days
- **Bag Recording Integration**: 1 day

**Total**: ~8-12 days for complete Phase 4

---

## References

- [ROS 2 Intra-process Communications](https://docs.ros.org/en/jazzy/Tutorials/Demos/Intra-Process-Communication.html)
- [ROS 2 Managed Nodes (Lifecycle)](https://design.ros2.org/articles/node_lifecycle.html)
- [ROS 2 Diagnostic Updater](https://github.com/ros/diagnostics/tree/ros2)
- [ROS 2 Actions](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- [ROS 2 Parameter Callbacks](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
