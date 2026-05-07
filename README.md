# How to Run Codes

## Run Launch File from Both Leader and Follower Side

---

### 1. For Normal Teleoperation

**Leader:**
```bash
roslaunch leader_ws version2.launch
```

Codes in launch file:
- `leader.py`
- `camera_leader.py`

**Follower:**
```bash
roslaunch follower_ws version2.launch
```

Codes in launch file:
- `follower_force.py`
- `follower_pose.py`
- `camera_follower.py`

---

### 2. For 2AFC Experiments

**Leader:**
```bash
roslaunch leader_ws version1.launch
```

Codes in launch file:
- `leader.py`
- For **delay experiment**: `test_leader_delay.py`
- For **packet loss experiment**: `test_leader_packetloss.py`

**Follower:**
```bash
roslaunch follower_ws version1.launch
```

Codes in launch file:
- `follower_force.py`
- `follower_pose.py`
- `camera_follower.py`
- For **delay experiment**: `test_slave_delay.py`
- For **packet loss experiment**: `test_slave_packetloss.py`
