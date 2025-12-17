---
sidebar_position: 5
---

# Manipulation and Grasping in Humanoid Robots

## Dexterous Interaction with the Physical World

Manipulation and grasping represent critical capabilities for humanoid robots, enabling them to interact with objects in human environments. This section explores the principles of dexterous manipulation, grasp planning, and the integration of manipulation with whole-body control in humanoid systems.

### Introduction to Robotic Manipulation

Robotic manipulation involves the controlled interaction with objects using robotic end-effectors. For humanoid robots, manipulation is particularly challenging due to the need to coordinate multiple degrees of freedom while maintaining balance and considering the whole-body dynamics.

#### Manipulation vs. Grasping

- **Grasping**: The process of securely holding an object
- **Manipulation**: The broader concept of moving and interacting with objects
- **Dexterous Manipulation**: Fine motor control similar to human hand capabilities

#### Humanoid Manipulation Challenges

Humanoid robots face unique manipulation challenges:
- **Balance Constraints**: Manipulation affects whole-body stability
- **Limited Dexterity**: Fewer fingers than human hands
- **Force Control**: Managing interaction forces while standing
- **Workspace Limitations**: Human-like workspace constraints

### Grasp Planning and Synthesis

#### Grasp Types and Classification

##### Power Grasps vs. Precision Grasps

**Power Grasps:**
- Maximize grasp force and stability
- Use palm and multiple fingers
- Suitable for heavy or large objects
- Examples: Cylindrical grasp, spherical grasp

**Precision Grasps:**
- Maximize dexterity and control
- Use fingertips primarily
- Suitable for small or delicate objects
- Examples: Pinch grasp, tip pinch

##### Grasp Taxonomy

The Feix taxonomy provides a systematic classification:

```python
class GraspType(Enum):
    # Power grasps
    PALM_GRASP = 1
    CYLINDRICAL_GRASP = 2
    SPHERICAL_GRASP = 3
    LATERAL_GRASP = 4

    # Precision grasps
    PINCH_GRASP = 5
    TIP_PINCH = 6
    TERRACE_GRASP = 7
    INTERDIGITAL_GRASP = 8

def classify_grasp(fingertip_positions, object_shape):
    """Classify grasp based on fingertip configuration"""
    # Calculate geometric relationships between fingertips
    distances = calculate_fingertip_distances(fingertip_positions)
    angles = calculate_fingertip_angles(fingertip_positions)

    # Determine grasp type based on configuration
    if len(fingertip_positions) >= 3 and min(distances) < 0.02:  # Close fingertips
        if object_shape == 'cylindrical':
            return GraspType.CYLINDRICAL_GRASP
        elif object_shape == 'spherical':
            return GraspType.SPHERICAL_GRASP
        else:
            return GraspType.PALM_GRASP
    elif len(fingertip_positions) >= 2 and max(distances) > 0.05:  # Spread fingertips
        return GraspType.PINCH_GRASP
    else:
        return GraspType.PALM_GRASP
```

#### Grasp Stability Analysis

##### Force Closure

Force closure ensures the grasp can resist any external wrench:

```python
def check_force_closure(fingertip_forces, object_mass, gravity=9.81):
    """Check if grasp provides force closure"""
    # Calculate gravitational force on object
    weight = object_mass * gravity

    # Sum of normal forces must exceed weight
    normal_forces = [f[2] for f in fingertip_forces]  # Assuming z is upward
    total_normal_force = sum(normal_forces)

    # Check force closure condition
    if total_normal_force > weight:
        # Check moment equilibrium
        moments = calculate_moments(fingertip_forces, object_com)
        if np.allclose(moments, 0, atol=0.01):
            return True

    return False

def calculate_moments(forces, object_com):
    """Calculate moments around object center of mass"""
    moments = np.zeros(3)
    for force, position in forces:
        moment_arm = position - object_com
        moment = np.cross(moment_arm, force)
        moments += moment
    return moments
```

##### Form Closure

Form closure is a geometric condition independent of friction:

```python
def check_form_closure(fingertip_positions, contact_normals):
    """Check form closure for frictionless contacts"""
    # For 2D: Need at least 4 contacts not all on one side
    # For 3D: Need at least 7 contacts not all on one side

    n_contacts = len(fingertip_positions)

    if len(fingertip_positions[0]) == 2:  # 2D case
        if n_contacts < 4:
            return False
        # Additional geometric checks for 2D form closure
    else:  # 3D case
        if n_contacts < 7:
            return False
        # Additional geometric checks for 3D form closure

    # Check that contact normals span the space of possible motions
    grasp_matrix = construct_grasp_matrix(fingertip_positions, contact_normals)
    return has_full_rank(grasp_matrix)

def construct_grasp_matrix(positions, normals):
    """Construct grasp matrix for form closure analysis"""
    # Grasp matrix maps joint torques to object wrenches
    # Implementation depends on contact model and robot kinematics
    pass
```

#### Grasp Planning Algorithms

##### Sampling-Based Approaches

```python
class GraspPlanner:
    def __init__(self, robot_hand, object_mesh):
        self.hand = robot_hand
        self.object = object_mesh
        self.safety_margin = 0.005  # 5mm safety margin

    def plan_grasp(self, target_object_pose, num_samples=1000):
        """Plan stable grasp using sampling approach"""
        valid_grasps = []

        for _ in range(num_samples):
            # Sample random grasp pose
            grasp_pose = self.sample_grasp_pose(target_object_pose)

            # Check collision-free
            if not self.check_collision_free(grasp_pose):
                continue

            # Check grasp quality
            grasp_quality = self.evaluate_grasp(grasp_pose)
            if grasp_quality > 0.5:  # Threshold for good grasp
                valid_grasps.append((grasp_pose, grasp_quality))

        # Return best grasp
        if valid_grasps:
            return max(valid_grasps, key=lambda x: x[1])[0]
        else:
            return None

    def sample_grasp_pose(self, object_pose):
        """Sample potential grasp pose"""
        # Sample approach direction
        approach_dir = np.random.randn(3)
        approach_dir = approach_dir / np.linalg.norm(approach_dir)

        # Sample grasp position on object surface
        surface_point = self.sample_surface_point(object_pose)

        # Construct grasp pose
        grasp_pose = self.construct_grasp_frame(surface_point, approach_dir)
        return grasp_pose

    def evaluate_grasp(self, grasp_pose):
        """Evaluate grasp quality"""
        # Check force closure
        force_closure = self.check_force_closure(grasp_pose)

        # Check grasp wrench space
        wrench_space = self.calculate_grasp_wrench_space(grasp_pose)

        # Check accessibility
        accessibility = self.check_accessibility(grasp_pose)

        # Combine metrics
        quality = (force_closure * 0.4 +
                  wrench_space * 0.4 +
                  accessibility * 0.2)

        return quality

    def calculate_grasp_wrench_space(self, grasp_pose):
        """Calculate grasp wrench space quality"""
        # The grasp wrench space represents all wrenches the grasp can resist
        # Implementation involves calculating the convex hull of possible wrenches
        pass
```

##### Optimization-Based Approaches

```python
import cvxpy as cp

class OptimizationBasedGraspPlanner:
    def __init__(self, robot_hand, object_model):
        self.hand = robot_hand
        self.object = object_model

    def plan_grasp_optimization(self, target_object):
        """Plan grasp using optimization approach"""
        # Define optimization variables
        # Contact positions, orientations, forces
        contact_positions = cp.Variable((self.hand.n_fingers, 3))
        contact_forces = cp.Variable((self.hand.n_fingers, 3))

        # Objective: minimize grasp force while maintaining stability
        objective = cp.Minimize(cp.sum_squares(contact_forces))

        # Constraints
        constraints = []

        # Force equilibrium constraint
        constraints.append(
            cp.sum(contact_forces, axis=0) == np.array([0, 0, target_object.mass * 9.81])
        )

        # Moment equilibrium constraint
        moments = []
        for i in range(self.hand.n_fingers):
            moment_arm = contact_positions[i, :] - target_object.com
            moment = cp.cross(moment_arm, contact_forces[i, :])
            moments.append(moment)

        constraints.append(cp.sum(moments, axis=0) == np.zeros(3))

        # Friction cone constraints
        for i in range(self.hand.n_fingers):
            normal_force = contact_forces[i, 2]  # Assuming z is normal direction
            tangential_force = cp.norm(contact_forces[i, :2])
            friction_coeff = 0.8  # Typical friction coefficient
            constraints.append(tangential_force <= friction_coeff * normal_force)

        # Object boundary constraints
        for i in range(self.hand.n_fingers):
            constraints.append(self.is_on_object_surface(contact_positions[i, :]))

        # Solve optimization problem
        problem = cp.Problem(objective, constraints)
        problem.solve()

        if problem.status == cp.OPTIMAL:
            return contact_positions.value, contact_forces.value
        else:
            return None, None
```

### Dexterous Manipulation

#### Hand Design for Dexterity

##### Anthropomorphic vs. Simplified Hands

**Anthropomorphic Hands:**
- Multiple joints per finger
- Opposable thumb
- Sensory feedback
- High dexterity but complex

**Simplified Hands:**
- Fewer degrees of freedom
- Easier control
- Lower cost
- Reduced dexterity

##### Underactuated Hands

Underactuated hands use fewer actuators than degrees of freedom:

```python
class UnderactuatedHand:
    def __init__(self, n_fingers=5, n_links_per_finger=3):
        self.n_fingers = n_fingers
        self.n_links_per_finger = n_links_per_finger
        self.n_actuators = n_fingers * 2  # 2 actuators per finger
        self.n_dof = n_fingers * n_links_per_finger  # 3 DOF per finger

    def map_actuator_to_joints(self, actuator_commands):
        """Map actuator commands to individual joint angles"""
        joint_angles = np.zeros(self.n_dof)

        for i in range(self.n_fingers):
            # Each finger has coupled joints
            base_actuator = actuator_commands[i * 2]
            coupling_actuator = actuator_commands[i * 2 + 1]

            # Apply coupling mechanism
            joint_angles[i * 3] = base_actuator  # MCP joint
            joint_angles[i * 3 + 1] = coupling_actuator * 0.7  # PIP joint
            joint_angles[i * 3 + 2] = coupling_actuator * 0.3  # DIP joint

        return joint_angles

    def grasp_object(self, object_shape, required_force):
        """Execute grasp with underactuated hand"""
        # Determine required actuator commands
        grasp_config = self.plan_grasp_configuration(object_shape)

        # Execute grasp with force control
        self.execute_grasp_with_force_control(grasp_config, required_force)

    def plan_grasp_configuration(self, object_shape):
        """Plan finger configuration based on object shape"""
        if object_shape == 'cylindrical':
            return self.plan_cylindrical_grasp()
        elif object_shape == 'spherical':
            return self.plan_spherical_grasp()
        elif object_shape == 'rectangular':
            return self.plan_box_grasp()
        else:
            return self.plan_power_grasp()
```

#### Tactile Sensing and Feedback

##### Tactile Sensor Integration

```python
class TactileFeedbackController:
    def __init__(self, hand):
        self.hand = hand
        self.tactile_sensors = hand.tactile_sensors
        self.slip_detection_threshold = 0.1
        self.force_control_gains = {'p': 10.0, 'i': 1.0, 'd': 0.1}

    def execute_grasp_with_feedback(self, target_force_profile):
        """Execute grasp with tactile feedback control"""
        integral_error = 0
        prev_error = 0
        dt = 0.01  # 100 Hz control rate

        while not self.is_grasp_stable():
            # Read tactile sensors
            tactile_data = self.tactile_sensors.read_all()

            # Detect slip
            slip_detected = self.detect_slip(tactile_data)

            if slip_detected:
                # Increase grasp force to prevent slip
                target_force_profile *= 1.2

            # Calculate force error
            current_forces = self.get_current_grasp_forces()
            error = target_force_profile - current_forces

            # PID control
            integral_error += error * dt
            derivative_error = (error - prev_error) / dt

            control_output = (self.force_control_gains['p'] * error +
                            self.force_control_gains['i'] * integral_error +
                            self.force_control_gains['d'] * derivative_error)

            # Apply control
            self.adjust_grasp_force(control_output)

            prev_error = error
            time.sleep(dt)

    def detect_slip(self, tactile_data):
        """Detect slip using tactile sensors"""
        # Analyze frequency content of tactile signals
        # High frequency components often indicate slip
        for sensor_data in tactile_data:
            if self.has_high_freq_content(sensor_data):
                return True
        return False

    def has_high_freq_content(self, signal):
        """Check if tactile signal has high frequency content indicating slip"""
        # Implementation would involve FFT or other frequency analysis
        pass
```

### Whole-Body Manipulation

#### Integration with Locomotion

Humanoid robots must coordinate manipulation with balance:

##### Whole-Body Control Framework

```python
class WholeBodyManipulationController:
    def __init__(self, robot):
        self.robot = robot
        self.manipulation_controller = ManipulationController(robot.arm)
        self.balance_controller = BalanceController(robot)
        self.task_priority = {
            'balance': 1,      # Highest priority
            'manipulation': 2,
            'posture': 3       # Lowest priority
        }

    def execute_manipulation_with_balance(self, manipulation_task, balance_config):
        """Execute manipulation while maintaining balance"""
        # Calculate required CoM adjustments for manipulation
        com_adjustment = self.calculate_manipulation_com_adjustment(manipulation_task)

        # Modify balance controller reference
        modified_balance_config = self.modify_balance_for_manipulation(
            balance_config, com_adjustment
        )

        # Execute coordinated control
        manipulation_cmd = self.manipulation_controller.plan(manipulation_task)
        balance_cmd = self.balance_controller.plan(modified_balance_config)

        # Combine commands using priority-based framework
        final_commands = self.combine_commands(
            manipulation_cmd, balance_cmd, self.task_priority
        )

        return final_commands

    def calculate_manipulation_com_adjustment(self, task):
        """Calculate CoM adjustment needed for manipulation task"""
        # Estimate the effect of manipulation forces on balance
        # This might involve dynamics simulation or learning-based approach
        pass

    def modify_balance_for_manipulation(self, balance_config, com_adjustment):
        """Modify balance configuration to accommodate manipulation"""
        modified_config = balance_config.copy()
        modified_config['com_reference'] += com_adjustment
        return modified_config

    def combine_commands(self, manip_cmd, balance_cmd, priorities):
        """Combine manipulation and balance commands respecting priorities"""
        # Use null-space projection to combine commands
        # Higher priority tasks are executed first
        # Lower priority tasks are projected to null space of higher priority tasks

        # Pseudo-inverse with null-space projection
        combined_cmd = self.prioritized_nullspace_projection(
            [balance_cmd, manip_cmd],  # Order by priority
            priorities
        )

        return combined_cmd

    def prioritized_nullspace_projection(self, commands, priorities):
        """Project commands using prioritized null-space approach"""
        # Implementation of prioritized task control
        # Balance (priority 1) is satisfied first
        # Manipulation (priority 2) is projected to null space of balance
        # Posture (priority 3) is projected to null space of both balance and manipulation
        pass
```

##### Center of Mass Compensation

```python
class COMCompensationController:
    def __init__(self, robot_mass, com_limits):
        self.robot_mass = robot_mass
        self.com_limits = com_limits
        self.gravity = 9.81

    def calculate_compensation(self, manipulation_force, manipulation_point):
        """Calculate CoM compensation for manipulation force"""
        # Calculate moment created by manipulation force
        moment_arm = manipulation_point - self.robot.com_position
        manipulation_moment = np.cross(manipulation_force, moment_arm)

        # Calculate required CoM shift to counteract moment
        required_com_shift = self.calculate_required_com_shift(manipulation_moment)

        return required_com_shift

    def calculate_required_com_shift(self, external_moment):
        """Calculate CoM shift to counteract external moment"""
        # Moment = F * d = mg * d (where d is CoM shift)
        # For moment balance: external_moment + mg * com_shift = 0
        com_shift = -external_moment / (self.robot_mass * self.gravity)
        return np.clip(com_shift, self.com_limits[0], self.com_limits[1])
```

### Manipulation Strategies

#### Prehensile vs. Non-Prehensile Manipulation

##### Prehensile Manipulation

Prehensile manipulation involves grasping objects:

```python
class PrehensileManipulation:
    def __init__(self, robot_hand):
        self.hand = robot_hand
        self.grasp_database = self.load_grasp_database()

    def grasp_object(self, object_info):
        """Execute prehensile grasp of object"""
        # Select appropriate grasp from database
        selected_grasp = self.select_best_grasp(object_info)

        # Execute approach motion
        self.execute_approach_motion(selected_grasp)

        # Close hand with force control
        self.execute_grasp_with_force_control(object_info.mass)

        # Verify grasp success
        return self.verify_grasp_success()

    def select_best_grasp(self, object_info):
        """Select best grasp from database based on object properties"""
        candidate_grasps = self.find_candidate_grasps(object_info)

        # Evaluate grasps based on stability, accessibility, etc.
        best_grasp = max(candidate_grasps, key=self.evaluate_grasp_quality)
        return best_grasp

    def execute_approach_motion(self, grasp_pose):
        """Execute motion to reach grasp position"""
        # Plan trajectory avoiding collisions
        approach_trajectory = self.plan_approach_trajectory(grasp_pose)

        # Execute with appropriate approach speed
        self.follow_trajectory(approach_trajectory, approach_speed=0.1)

    def execute_grasp_with_force_control(self, object_mass):
        """Execute grasp with controlled force"""
        target_force = self.calculate_grasp_force(object_mass)

        # Close fingers while monitoring force
        self.close_hand_with_force_control(target_force)

    def calculate_grasp_force(self, object_mass):
        """Calculate required grasp force"""
        # Safety factor to prevent slip
        safety_factor = 2.0
        weight = object_mass * 9.81
        friction_coeff = 0.8

        # Minimum force to prevent slip
        min_grasp_force = weight / (2 * friction_coeff)  # Assuming 2 contact points
        target_force = min_grasp_force * safety_factor

        return target_force
```

##### Non-Prehensile Manipulation

Non-prehensile manipulation uses pushing, sliding, and other contact methods:

```python
class NonPrehensileManipulation:
    def __init__(self, robot_arm):
        self.arm = robot_arm

    def push_object(self, object_pose, target_pose, push_direction):
        """Push object to target position"""
        # Plan push trajectory
        push_trajectory = self.plan_push_trajectory(
            object_pose, target_pose, push_direction
        )

        # Execute push with force control
        self.execute_push_with_force_control(push_trajectory)

    def plan_push_trajectory(self, object_pose, target_pose, push_direction):
        """Plan trajectory for pushing object"""
        # Calculate push points on object
        contact_points = self.calculate_push_contact_points(
            object_pose, push_direction
        )

        # Plan approach, contact, and follow-through motions
        approach_pose = self.calculate_approach_pose(contact_points[0])
        contact_pose = self.calculate_contact_pose(contact_points[0])
        follow_through_pose = self.calculate_follow_through_pose(contact_points[0])

        return [approach_pose, contact_pose, follow_through_pose]

    def execute_push_with_force_control(self, trajectory):
        """Execute push with controlled force"""
        # Follow trajectory with force control
        for waypoint in trajectory:
            self.move_to_with_force_control(waypoint)

    def calculate_push_contact_points(self, object_pose, push_direction):
        """Calculate optimal contact points for pushing"""
        # Find surface points where push direction aligns with surface normal
        surface_points = self.object_model.get_surface_points()
        valid_contacts = []

        for point in surface_points:
            surface_normal = self.object_model.get_surface_normal(point)

            # Check if push direction is suitable (not opposing normal)
            if np.dot(push_direction, surface_normal) > 0.5:  # At least 60 degrees
                valid_contacts.append(point)

        return valid_contacts
```

### Advanced Manipulation Techniques

#### Learning-Based Manipulation

##### Imitation Learning

```python
class ImitationLearningManipulation:
    def __init__(self, robot, demonstration_data):
        self.robot = robot
        self.demonstration_data = demonstration_data
        self.policy_network = self.build_policy_network()

    def learn_manipulation_skill(self, skill_name):
        """Learn manipulation skill from demonstrations"""
        # Extract features from demonstrations
        states, actions = self.extract_state_action_pairs(skill_name)

        # Train policy network
        self.train_policy_network(states, actions)

        # Validate learned policy
        success_rate = self.validate_policy(skill_name)
        return success_rate

    def extract_state_action_pairs(self, skill_name):
        """Extract state-action pairs from demonstrations"""
        skill_demonstrations = self.get_skill_demonstrations(skill_name)
        states = []
        actions = []

        for demo in skill_demonstrations:
            for timestep in demo:
                state = self.extract_state_features(timestep)
                action = timestep['action']
                states.append(state)
                actions.append(action)

        return np.array(states), np.array(actions)

    def extract_state_features(self, timestep):
        """Extract relevant features from robot state"""
        # Robot joint positions and velocities
        joint_pos = timestep['joint_positions']
        joint_vel = timestep['joint_velocities']

        # Object state
        obj_pos = timestep['object_position']
        obj_orient = timestep['object_orientation']

        # End-effector state
        ee_pos = timestep['end_effector_position']
        ee_orient = timestep['end_effector_orientation']

        # Combine all features
        features = np.concatenate([joint_pos, joint_vel, obj_pos, obj_orient, ee_pos, ee_orient])
        return features

    def execute_learned_skill(self, initial_state, target_state):
        """Execute learned manipulation skill"""
        current_state = initial_state
        trajectory = [initial_state]

        while not self.is_target_reached(current_state, target_state):
            # Get action from learned policy
            action = self.policy_network.predict(current_state)

            # Execute action
            next_state = self.execute_action(action, current_state)
            trajectory.append(next_state)

            current_state = next_state

        return trajectory
```

#### Reinforcement Learning for Manipulation

```python
class RLManipulationAgent:
    def __init__(self, state_dim, action_dim, learning_rate=1e-4):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.learning_rate = learning_rate

        # Actor-Critic networks
        self.actor = self.build_actor_network()
        self.critic = self.build_critic_network()

        # Replay buffer
        self.replay_buffer = ReplayBuffer(capacity=100000)

    def build_actor_network(self):
        """Build neural network for policy (actor)"""
        import torch
        import torch.nn as nn

        class Actor(nn.Module):
            def __init__(self, state_dim, action_dim):
                super(Actor, self).__init__()
                self.network = nn.Sequential(
                    nn.Linear(state_dim, 400),
                    nn.ReLU(),
                    nn.Linear(400, 300),
                    nn.ReLU(),
                    nn.Linear(300, action_dim),
                    nn.Tanh()  # Actions in [-1, 1]
                )

            def forward(self, state):
                return self.network(state)

        return Actor(self.state_dim, self.action_dim)

    def build_critic_network(self):
        """Build neural network for value function (critic)"""
        import torch
        import torch.nn as nn

        class Critic(nn.Module):
            def __init__(self, state_dim, action_dim):
                super(Critic, self).__init__()
                self.q_network = nn.Sequential(
                    nn.Linear(state_dim + action_dim, 400),
                    nn.ReLU(),
                    nn.Linear(400, 300),
                    nn.ReLU(),
                    nn.Linear(300, 1)
                )

            def forward(self, state, action):
                sa = torch.cat([state, action], dim=1)
                return self.q_network(sa)

        return Critic(self.state_dim, self.action_dim)

    def compute_reward(self, state, action, next_state, goal):
        """Compute reward for manipulation task"""
        # Distance to goal
        current_pos = self.get_end_effector_pos(next_state)
        goal_dist = np.linalg.norm(current_pos - goal)

        # Success bonus
        if goal_dist < 0.01:  # Within 1cm of goal
            return 100.0

        # Negative distance reward
        distance_reward = -goal_dist * 10.0

        # Penalty for excessive joint velocities
        joint_vel_penalty = -np.sum(np.abs(next_state['joint_velocities'])) * 0.1

        # Penalty for collisions
        collision_penalty = -10.0 if self.check_collision(next_state) else 0.0

        total_reward = distance_reward + joint_vel_penalty + collision_penalty
        return max(total_reward, -10.0)  # Clamp minimum reward

    def train_step(self, batch):
        """Perform one training step"""
        states, actions, rewards, next_states, dones = batch

        # Convert to tensors
        states = torch.FloatTensor(states)
        actions = torch.FloatTensor(actions)
        rewards = torch.FloatTensor(rewards).unsqueeze(1)
        next_states = torch.FloatTensor(next_states)
        dones = torch.BoolTensor(dones).unsqueeze(1)

        # Critic update
        with torch.no_grad():
            next_actions = self.actor(next_states)
            next_q = self.critic(next_states, next_actions)
            target_q = rewards + (0.99 * next_q * ~dones)

        current_q = self.critic(states, actions)
        critic_loss = nn.MSELoss()(current_q, target_q)

        # Actor update
        predicted_actions = self.actor(states)
        actor_loss = -self.critic(states, predicted_actions).mean()

        # Update networks
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()
```

### Implementation Considerations

#### Real-time Control Requirements

Manipulation requires precise timing and coordination:

##### Multi-rate Control Architecture

```python
class MultiRateManipulationController:
    def __init__(self):
        self.high_level_planner = HighLevelPlanner(dt=0.1)    # 10 Hz
        self.mid_level_controller = MidLevelController(dt=0.01)  # 100 Hz
        self.low_level_controller = LowLevelController(dt=0.001) # 1000 Hz

    def execute_manipulation_task(self, task):
        """Execute manipulation with multi-rate control"""
        while not task_completed(task):
            # High-level planning (trajectory generation)
            if self.high_level_planner.is_update_time():
                planned_trajectory = self.high_level_planner.plan(task)
                self.mid_level_controller.set_reference_trajectory(planned_trajectory)

            # Mid-level control (trajectory following)
            if self.mid_level_controller.is_update_time():
                joint_commands = self.mid_level_controller.compute_commands()
                self.low_level_controller.set_joint_targets(joint_commands)

            # Low-level control (joint servoing)
            if self.low_level_controller.is_update_time():
                self.low_level_controller.execute_servo_loop()

            time.sleep(min(0.001, self.get_next_update_time()))
```

#### Safety and Compliance

##### Force Control and Compliance

```python
class CompliantManipulationController:
    def __init__(self, robot):
        self.robot = robot
        self.max_force_threshold = 50.0  # Newtons
        self.compliance_matrix = np.eye(6) * 0.001  # Small compliance

    def execute_compliant_manipulation(self, task, compliance_config):
        """Execute manipulation with compliance control"""
        # Set compliance parameters
        self.set_compliance(compliance_config)

        # Execute task with force monitoring
        while not task_completed(task):
            # Monitor interaction forces
            forces = self.get_interaction_forces()

            # Check for excessive forces
            if np.any(np.abs(forces) > self.max_force_threshold):
                self.execute_safety_procedure()
                return False

            # Continue with compliant control
            self.execute_compliant_step(task)

        return True

    def set_compliance(self, config):
        """Set compliance parameters"""
        self.compliance_matrix = config['compliance_matrix']
        self.damping_matrix = config['damping_matrix']

    def get_interaction_forces(self):
        """Get estimated interaction forces"""
        # Use force/torque sensors or current-based estimation
        if hasattr(self.robot, 'ft_sensors'):
            return self.robot.ft_sensors.get_wrench()
        else:
            # Estimate from motor currents
            return self.estimate_contact_forces_from_currents()

    def execute_safety_procedure(self):
        """Execute safety procedure when forces exceed limits"""
        # Reduce stiffness
        self.set_compliance({'compliance_matrix': np.eye(6) * 0.01})

        # Move away from contact gently
        self.execute_safe_withdrawal()

        # Report error
        self.report_error("Excessive interaction force detected")
```

### Performance Evaluation

#### Manipulation Quality Metrics

```python
class ManipulationPerformanceEvaluator:
    def __init__(self):
        self.metrics = {
            'success_rate': 0,
            'execution_time': 0,
            'energy_efficiency': 0,
            'dexterity': 0,
            'safety': 0
        }

    def evaluate_grasp_success(self, grasp_attempt):
        """Evaluate grasp success"""
        # Check if object was successfully picked up
        object_lifted = self.check_object_lifted(grasp_attempt)

        # Check grasp stability during manipulation
        grasp_stable = self.check_grasp_stability(grasp_attempt)

        # Check for damage to object
        object_intact = self.check_object_integrity(grasp_attempt)

        return object_lifted and grasp_stable and object_intact

    def evaluate_manipulation_dexterity(self, manipulation_task):
        """Evaluate dexterity of manipulation"""
        # Measure precision in positioning
        positioning_precision = self.calculate_positioning_error(manipulation_task)

        # Measure smoothness of motion
        motion_smoothness = self.calculate_motion_jerk(manipulation_task)

        # Measure adaptability to disturbances
        disturbance_robustness = self.test_disturbance_robustness(manipulation_task)

        # Combine metrics
        dexterity_score = (0.4 * (1.0 / (1.0 + positioning_precision)) +
                          0.3 * motion_smoothness +
                          0.3 * disturbance_robustness)

        return dexterity_score

    def calculate_positioning_error(self, task):
        """Calculate positioning accuracy"""
        target_pos = task['target_position']
        actual_pos = task['actual_position']
        error = np.linalg.norm(target_pos - actual_pos)
        return error

    def calculate_motion_jerk(self, task):
        """Calculate motion smoothness (lower jerk is better)"""
        # Calculate third derivative of position
        positions = task['trajectory_positions']
        velocities = np.gradient(positions, axis=0)
        accelerations = np.gradient(velocities, axis=0)
        jerks = np.gradient(accelerations, axis=0)

        # Average jerk magnitude
        avg_jerk = np.mean(np.abs(jerks))

        # Convert to smoothness score (higher is better)
        smoothness = 1.0 / (1.0 + avg_jerk)
        return smoothness
```

### Best Practices

#### Design Principles

1. **Modular Architecture**: Separate perception, planning, and control
2. **Robust Grasp Planning**: Handle uncertainty in object properties
3. **Safety First**: Prioritize safe operation over task success
4. **Human-like Motion**: Create natural, predictable movements

#### Testing Strategies

1. **Progressive Complexity**: Start with simple objects, increase complexity
2. **Failure Mode Testing**: Test boundary conditions and error recovery
3. **Human Interaction Safety**: Ensure safe interaction with humans
4. **Long-term Reliability**: Test sustained operation over time

### Future Developments

#### Emerging Technologies

- **Soft Robotics**: Compliant, adaptive manipulation
- **Bio-inspired Design**: Human hand-like capabilities
- **AI Integration**: Learning-based manipulation strategies
- **Haptic Feedback**: Enhanced tactile sensing and feedback

#### Advanced Applications

- **Collaborative Manipulation**: Human-robot team tasks
- **Multi-object Manipulation**: Handling multiple objects simultaneously
- **Deformable Object Manipulation**: Working with cloth, cables, etc.
- **Tool Use**: Using objects as tools for complex tasks

### Conclusion

Manipulation and grasping in humanoid robots represent a complex integration of mechanical design, control theory, and artificial intelligence. Success requires careful consideration of grasp stability, whole-body coordination, and safety constraints. The field continues to advance with new approaches in learning-based control, soft robotics, and bio-inspired design. As humanoid robots become more capable, their manipulation abilities will increasingly approach human dexterity, enabling them to perform complex tasks in human environments. The integration of advanced sensing, learning algorithms, and sophisticated control strategies will continue to push the boundaries of what humanoid robots can achieve in manipulation tasks, making them increasingly valuable partners in human-centered environments.