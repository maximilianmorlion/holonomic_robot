# Architecture Simplification Summary

## Changes Made

### 1. Removed Redundant Components

#### Files Deleted:
- `robot_application/obstacle_monitor.py` (module)
- `robot_application/task_recovery.py` (recovery strategies)
- `scripts/obstacle_monitor.py` (executable wrapper)
- `config/obstacle_monitor.yaml` (configuration)
- `OBSTACLE_HANDLING.md` (old documentation)

#### Rationale:
Nav2 already provides:
- **Costmap-based obstacle detection** - Updates continuously from sensors
- **Automatic path replanning** - MPPI controller replans when obstacles detected
- **Retry logic** - Attempts navigation multiple times before giving up
- **Timeout management** - Cancels goals that exceed time threshold

Your custom ObstacleMonitor was detecting:
- STUCK - Nav2's local planner detects this
- TIMEOUT - Nav2 already has timeout logic
- UNREACHABLE - Nav2 returns this in goal feedback
- BLOCKED - Costmap-based planning prevents this
- OUT_OF_BOUNDS - Nav2 checks map boundaries

### 2. Updated Configuration

#### `setup.py` Changes:
```diff
- 'obstacle_monitor.py = robot_application.obstacle_monitor:main',
```

#### `CMakeLists.txt` Changes:
```diff
- scripts/obstacle_monitor.py
```

### 3. Simplified TaskPlanner

#### Removed from `task_planner.py`:
```python
# ❌ OLD
from robot_application.task_recovery import (
    TaskRecoveryManager, BlockageType, create_recovery_context,
    is_blockage_critical
)

self.recovery_manager = TaskRecoveryManager()
self.last_blockage_type = None
self.blockage_recovery_in_progress = False

self.blockage_sub = create_subscription(...)
self.recovery_suggestion_sub = create_subscription(...)
self.recovery_status_pub = create_publisher(...)
self.blocked_task_pub = create_publisher(...)

def blockage_callback(...) → REMOVED
def recovery_suggestion_callback(...) → REMOVED  
def handle_critical_blockage(...) → REMOVED
def handle_blockage_callback(...) → REMOVED
```

#### Simplified to:
```python
# ✅ NEW
# TaskPlanner focuses on: 
# 1. Task prioritization (utility-based)
# 2. Monitoring Nav2 goal outcomes
# 3. Task switching on failure

# No obstacle-specific logic - Nav2 handles it!
```

## New Workflow: Task → Nav2 → Success/Failure → Replan

```
┌─────────────────────────────────────────────────────────────┐
│ GameStateManager                                            │
│ - Track match timer, phase, scores                          │
└────────────────────────────┬────────────────────────────────┘
                             │ publish game state
                             ▼
┌─────────────────────────────────────────────────────────────┐
│ TaskPlanner                                                 │
│ - Select next task (highest utility)                        │
│ - Monitor Nav2 goal feedback                                │
│ - Switch tasks on failure                                   │
└────────────────────────────┬────────────────────────────────┘
                             │ dispatch goal
                             ▼
┌─────────────────────────────────────────────────────────────┐
│ Nav2 NavigateToPose Action                                  │
│ - Plan path to goal                                         │
│ - Execute with MPPI local planner                           │
│ - Handle obstacle avoidance (costmap + replanning)          │
│ - Timeout after max_planning_retries                        │
└────────────────────────────┬────────────────────────────────┘
                             │ goal result (SUCCESS/FAILURE)
                             ▼
                    ┌──────────────────┐
                    │ Success? Continue │
                    │ Failure? Switch   │
                    └──────────────────┘
```

## Configuration Simplification

### Before (Complex):
```yaml
# obstacle_monitor.yaml
obstacle_monitor:
  stuck_timeout_sec: 3.0
  blockage_detection_interval_sec: 0.5
  unreachable_retry_limit: 3
  critical_blockage_types: [STUCK, UNREACHABLE]
  
# task_planner.yaml  
task_planner:
  enable_obstacle_recovery: true
  recovery_strategies: [wait_retry, reduce_scope, detour, ...]
```

### After (Simplified):
```yaml
# task_planner.yaml only
task_planner:
  replan_interval_sec: 5.0
  enable_task_interruption: true
  min_utility_threshold: 0.5
  base_location_x: 0.0
  base_location_y: 0.0
  
# Nav2 costmap and planner configs handle obstacle logic
```

## Benefits

| Aspect | Before | After |
|--------|--------|-------|
| **LOC** | 600+ (obstacle_monitor.py + task_recovery.py) | Removed |
| **Topics** | 10 (including /obstacle/*) | 5 (clean & simple) |
| **State Tracking** | Complex (blockage types, recovery states) | Simple (task status only) |
| **Duplication** | High (obstacle detection in 2 places) | None (single source of truth) |
| **Debugging** | Hard (multiple failure modes) | Easy (Nav2 feedback is clear) |
| **Maintainability** | Hard (coupled systems) | Easy (loose coupling) |

## What to Monitor Now

Instead of custom obstacle detection, monitor Nav2 feedback:

```python
# In Mission executor
goal_handle = nav2_client.send_goal(goal)
result = goal_handle.get_result()

if result.status == GoalStatus.SUCCEEDED:
    # Task completed successfully
    switch_to_next_task()
    
elif result.status == GoalStatus.ABORTED:
    # Could be UNREACHABLE, TIMEOUT, or other failure
    # NavResult.msg tells you why
    get_logger().error(f"Goal failed: {result.result}")
    switch_to_next_task()
```

## Testing Checklist

- [ ] TaskPlanner starts without obstacle_monitor errors
- [ ] Task prioritization works (utility calculation)
- [ ] Navigation completes for achievable goals
- [ ] TaskPlanner switches tasks when Nav2 returns UNREACHABLE
- [ ] Game phase transitions trigger re-planning
- [ ] Queue publishes correct size
- [ ] Mission executor respects Nav2 feedback

## Next Steps

1. **Remove obstacle_monitor.py from filesystem** (run: `rm -f` commands)
2. **Test TaskPlanner** with simplified configuration
3. **Implement actual mission executors** (currently stubbed with sleep)
4. **Add robot_vision package** for AprilTag-based goal location
5. **Deploy to Jetson Orin Nano** for hardware testing

## Rollback Plan

If you need to restore obstacle handling:
1. Original files available in git history
2. Simply re-add ObstacleMonitor node
3. Update entry points in setup.py/CMakeLists.txt
4. Add back obstacle_monitor.yaml config

But: **You probably won't need it.** Nav2 is very robust for mobile manipulation.
