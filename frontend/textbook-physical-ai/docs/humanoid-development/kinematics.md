---
sidebar_position: 3
---

# Humanoid Robot Kinematics

## Understanding Movement and Positioning in Bipedal Systems

Humanoid robot kinematics is the study of motion in robots designed with human-like form and movement capabilities. Understanding kinematics is fundamental to controlling humanoid robots, enabling them to perform complex movements while maintaining balance and achieving desired tasks. This section covers both forward and inverse kinematics, which are essential for humanoid robot control.

### Introduction to Robot Kinematics

Robot kinematics is the branch of robotics that studies the motion of robots without considering the forces that cause the motion. For humanoid robots, kinematics becomes particularly complex due to the large number of degrees of freedom and the need to maintain balance while moving.

#### Degrees of Freedom (DOF)

The degrees of freedom represent the number of independent movements a robot can perform:

##### Joint Classification

- **Revolute Joints**: Single rotational degree of freedom
- **Prismatic Joints**: Single translational degree of freedom
- **Spherical Joints**: Three rotational degrees of freedom
- **Planar Joints**: Motion in a plane (two translations + one rotation)

##### Humanoid DOF Requirements

A typical humanoid robot requires:
- **Legs**: 6+ DOF per leg for walking and balance
- **Arms**: 7+ DOF per arm for dexterity
- **Torso**: 3+ DOF for upper body movement
- **Head**: 2-3 DOF for gaze control
- **Total**: 30+ DOF for a fully articulated humanoid

### Forward Kinematics

Forward kinematics calculates the position and orientation of the end-effector given the joint angles.

#### Mathematical Foundation

For a serial chain of n joints, forward kinematics computes:

T = A₁(θ₁) × A₂(θ₂) × ... × Aₙ(θₙ)

Where T is the transformation matrix and Aᵢ(θᵢ) are the joint transformation matrices.

#### Denavit-Hartenberg (DH) Convention

The DH convention provides a systematic way to define coordinate frames on robotic linkages:

##### DH Parameters

For each joint i:
- **aᵢ**: Link length (distance along xᵢ from zᵢ to zᵢ₊₁)
- **αᵢ**: Link twist (angle from zᵢ to zᵢ₊₁ about xᵢ)
- **dᵢ**: Link offset (distance along zᵢ from xᵢ₋₁ to xᵢ)
- **θᵢ**: Joint angle (angle from xᵢ₋₁ to xᵢ about zᵢ)

##### DH Transformation Matrix

```
     | cos(θᵢ)   -sin(θᵢ)cos(αᵢ)   sin(θᵢ)sin(αᵢ)   aᵢcos(θᵢ) |
Aᵢ = | sin(θᵢ)    cos(θᵢ)cos(αᵢ)  -cos(θᵢ)sin(αᵢ)   aᵢsin(θᵢ) |
     | 0          sin(αᵢ)          cos(αᵢ)          dᵢ       |
     | 0          0                0                1        |
```

#### Python Implementation

```python
import numpy as np

def dh_transform(a, alpha, d, theta):
    """Calculate DH transformation matrix"""
    cos_th = np.cos(theta)
    sin_th = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    T = np.array([
        [cos_th, -sin_th*cos_alpha, sin_th*sin_alpha, a*cos_th],
        [sin_th, cos_th*cos_alpha, -cos_th*sin_alpha, a*sin_th],
        [0, sin_alpha, cos_alpha, d],
        [0, 0, 0, 1]
    ])
    return T

def forward_kinematics(joint_angles, dh_params):
    """Calculate forward kinematics for a serial chain"""
    T_total = np.eye(4)  # Identity matrix

    for i, (a, alpha, d, theta_offset) in enumerate(dh_params):
        theta = theta_offset + joint_angles[i]
        T_link = dh_transform(a, alpha, d, theta)
        T_total = np.dot(T_total, T_link)

    return T_total

# Example: 3-DOF planar manipulator
dh_params_3dof = [
    (0.1, 0, 0, 0),    # Joint 1: a=0.1, α=0, d=0, θ_offset=0
    (0.1, 0, 0, 0),    # Joint 2: a=0.1, α=0, d=0, θ_offset=0
    (0.1, 0, 0, 0)     # Joint 3: a=0.1, α=0, d=0, θ_offset=0
]

# Calculate FK for specific joint angles
joint_angles = [np.pi/4, np.pi/6, -np.pi/3]
end_effector_pose = forward_kinematics(joint_angles, dh_params_3dof)
print("End effector pose:\n", end_effector_pose)
```

### Inverse Kinematics

Inverse kinematics calculates the joint angles required to achieve a desired end-effector position and orientation.

#### Analytical vs. Numerical Solutions

##### Analytical Solutions

- **Advantages**: Fast computation, closed-form solution
- **Disadvantages**: Only possible for simple kinematic chains
- **Applicable to**: Planar manipulators, some 6-DOF arms

##### Numerical Solutions

- **Advantages**: Applicable to complex kinematic chains
- **Disadvantages**: Slower computation, may not converge
- **Applicable to**: Humanoid robots, redundant manipulators

#### Jacobian-Based Methods

The Jacobian matrix relates joint velocities to end-effector velocities:

v = J(θ) × θ̇

Where:
- v: End-effector velocity vector
- J(θ): Jacobian matrix
- θ̇: Joint velocity vector

##### Jacobian Calculation

```python
import numpy as np

def calculate_jacobian(robot_chain, joint_angles):
    """Calculate Jacobian matrix for a robot chain"""
    n = len(joint_angles)
    J = np.zeros((6, n))  # 6xN Jacobian (linear + angular velocities)

    # Get current end-effector position
    T_total = forward_kinematics(joint_angles, robot_chain.dh_params)
    end_effector_pos = T_total[:3, 3]

    for i in range(n):
        # Calculate transformation up to joint i
        T_to_joint = np.eye(4)
        for j in range(i + 1):
            T_link = dh_transform(
                robot_chain.dh_params[j][0],  # a
                robot_chain.dh_params[j][1],  # alpha
                robot_chain.dh_params[j][2],  # d
                robot_chain.dh_params[j][3] + joint_angles[j]  # theta
            )
            T_to_joint = np.dot(T_to_joint, T_link)

        # Joint axis in world coordinates
        z_i = T_to_joint[:3, 2]  # z-axis of joint i
        r_i = T_to_joint[:3, 3]  # position of joint i

        # Linear velocity contribution
        J[:3, i] = np.cross(z_i, end_effector_pos - r_i)

        # Angular velocity contribution
        J[3:, i] = z_i

    return J

def inverse_kinematics_jacobian(target_pose, current_angles, robot_chain,
                               max_iterations=100, tolerance=1e-4):
    """Solve inverse kinematics using Jacobian transpose method"""

    current_pose = forward_kinematics(current_angles, robot_chain.dh_params)

    for iteration in range(max_iterations):
        # Calculate error
        pos_error = target_pose[:3, 3] - current_pose[:3, 3]
        rot_error = rotation_error(target_pose, current_pose)
        error = np.concatenate([pos_error, rot_error])

        if np.linalg.norm(error) < tolerance:
            break

        # Calculate Jacobian
        J = calculate_jacobian(robot_chain, current_angles)

        # Update joint angles using Jacobian transpose
        # J# = J^T for Jacobian transpose method
        joint_delta = np.dot(J.T, error)
        current_angles += 0.01 * joint_delta  # Small step size

        # Update current pose
        current_pose = forward_kinematics(current_angles, robot_chain.dh_params)

    return current_angles

def rotation_error(target_pose, current_pose):
    """Calculate rotation error between two poses"""
    R_error = np.dot(target_pose[:3, :3], current_pose[:3, :3].T)
    # Convert rotation matrix to angle-axis representation
    angle_axis = matrix_to_angle_axis(R_error)
    return angle_axis

def matrix_to_angle_axis(R):
    """Convert rotation matrix to angle-axis representation"""
    angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))

    if np.abs(angle) < 1e-6:
        return np.zeros(3)

    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ]) / (2 * np.sin(angle))

    return angle * axis
```

#### Optimization-Based Methods

For complex humanoid robots, optimization-based approaches are often more effective:

```python
from scipy.optimize import minimize
import numpy as np

def inverse_kinematics_optimization(target_pose, initial_angles, robot_chain):
    """Solve inverse kinematics using optimization"""

    def objective_function(joint_angles):
        """Objective function to minimize"""
        current_pose = forward_kinematics(joint_angles, robot_chain.dh_params)

        # Position error
        pos_error = np.sum((target_pose[:3, 3] - current_pose[:3, 3])**2)

        # Orientation error (using Frobenius norm of rotation error)
        rot_error = np.sum((target_pose[:3, :3] - current_pose[:3, :3])**2)

        # Joint limit penalties
        joint_limits = robot_chain.joint_limits
        limit_penalty = 0
        for i, angle in enumerate(joint_angles):
            if angle < joint_limits[i][0] or angle > joint_limits[i][1]:
                limit_penalty += 1000  # Large penalty for violating limits

        return pos_error + 0.1 * rot_error + limit_penalty

    # Optimize to find joint angles
    result = minimize(
        objective_function,
        initial_angles,
        method='L-BFGS-B',
        bounds=robot_chain.joint_limits
    )

    return result.x
```

### Humanoid-Specific Kinematics

#### Leg Kinematics for Bipedal Walking

Humanoid legs require special consideration for stable walking:

##### 6-DOF Leg Structure

A typical humanoid leg has:
- **Hip**: 3 DOF (yaw, pitch, roll)
- **Knee**: 1 DOF (pitch)
- **Ankle**: 2 DOF (pitch, roll)

##### Leg Inverse Kinematics

```python
class HumanoidLegIK:
    def __init__(self, thigh_length=0.4, shin_length=0.4):
        self.thigh_length = thigh_length
        self.shin_length = shin_length

    def solve_leg_ik(self, target_position, leg_type='left'):
        """Solve inverse kinematics for humanoid leg"""
        # Convert global target to leg coordinate system
        x, y, z = target_position

        # Calculate hip yaw (pointing direction)
        hip_yaw = np.arctan2(y, x)

        # Calculate hip roll (lateral movement)
        hip_roll = np.arctan2(-z, np.sqrt(x**2 + y**2))

        # Calculate remaining 4 DOF in sagittal plane
        # Project to sagittal plane
        r_sagittal = np.sqrt(x**2 + y**2 + z**2)

        # Use 2-link planar IK for hip pitch, knee, and ankle pitch
        hip_pitch, knee_angle, ankle_pitch = self.solve_planar_3dof(
            r_sagittal, -z, self.thigh_length, self.shin_length
        )

        # Ankle roll for foot orientation
        ankle_roll = 0  # Usually kept at 0 for flat foot

        if leg_type == 'right':
            # Right leg has opposite signs for some joints
            hip_roll = -hip_roll
            ankle_roll = -ankle_roll

        return np.array([hip_yaw, hip_roll, hip_pitch, knee_angle, ankle_pitch, ankle_roll])

    def solve_planar_3dof(self, x, z, l1, l2):
        """Solve 3-DOF planar leg (hip pitch, knee, ankle pitch)"""
        # Distance from hip to target
        r = np.sqrt(x**2 + z**2)

        # Knee angle using law of cosines
        cos_knee = (l1**2 + l2**2 - r**2) / (2 * l1 * l2)
        cos_knee = np.clip(cos_knee, -1, 1)  # Avoid numerical errors
        knee_angle = np.pi - np.arccos(cos_knee)

        # Hip pitch
        alpha = np.arctan2(z, x)
        beta = np.arccos((l1**2 + r**2 - l2**2) / (2 * l1 * r))
        hip_pitch = alpha + beta

        # Ankle pitch (compensate for knee and hip)
        ankle_pitch = -(hip_pitch + knee_angle)

        return hip_pitch, knee_angle, ankle_pitch
```

#### Arm Kinematics for Manipulation

Humanoid arms require dexterity for manipulation tasks:

##### 7-DOF Arm Structure

A typical humanoid arm has:
- **Shoulder**: 3 DOF (yaw, pitch, roll)
- **Elbow**: 1 DOF (pitch)
- **Wrist**: 3 DOF (yaw, pitch, roll)

##### Redundancy Resolution

With 7 DOF, the arm has one redundant degree of freedom that can be used for:
- Obstacle avoidance
- Posture optimization
- Singularity avoidance

```python
class HumanoidArmIK:
    def __init__(self, upper_arm_length=0.3, forearm_length=0.3):
        self.upper_arm_length = upper_arm_length
        self.forearm_length = forearm_length

    def solve_arm_ik(self, target_pose, elbow_preference='left'):
        """Solve 7-DOF arm inverse kinematics with redundancy resolution"""

        # Extract target position and orientation
        target_pos = target_pose[:3, 3]
        target_rot = target_pose[:3, :3]

        # Calculate shoulder position (elbow up or down configuration)
        elbow_pos = self.calculate_elbow_position(
            target_pos, elbow_preference
        )

        # Solve for shoulder angles
        shoulder_angles = self.solve_shoulder_ik(target_pos, elbow_pos)

        # Solve for elbow and wrist angles
        elbow_angle, wrist_angles = self.solve_wrist_ik(
            elbow_pos, target_pos, target_rot, shoulder_angles
        )

        # Combine all angles
        joint_angles = np.concatenate([shoulder_angles, [elbow_angle], wrist_angles])

        return joint_angles

    def calculate_elbow_position(self, target_pos, preference):
        """Calculate elbow position based on preference"""
        # For now, use simple elbow-up configuration
        # In practice, this would consider obstacles and preferences
        shoulder_to_target = target_pos - np.array([0, 0, 0])  # Assuming shoulder at origin
        arm_length = self.upper_arm_length + self.forearm_length

        if np.linalg.norm(shoulder_to_target) > arm_length:
            # Arm fully extended
            direction = shoulder_to_target / np.linalg.norm(shoulder_to_target)
            elbow_pos = self.upper_arm_length * direction
        else:
            # Use preferred configuration
            elbow_pos = self.calculate_elbow_config(shoulder_to_target, preference)

        return elbow_pos

    def solve_shoulder_ik(self, target_pos, elbow_pos):
        """Solve shoulder joint angles"""
        # Calculate shoulder yaw and pitch based on elbow position
        shoulder_yaw = np.arctan2(elbow_pos[1], elbow_pos[0])
        shoulder_pitch = np.arctan2(elbow_pos[2], np.sqrt(elbow_pos[0]**2 + elbow_pos[1]**2))

        # Shoulder roll to align elbow properly
        shoulder_roll = 0  # Simplified for this example

        return np.array([shoulder_yaw, shoulder_pitch, shoulder_roll])

    def solve_wrist_ik(self, elbow_pos, target_pos, target_rot, shoulder_angles):
        """Solve elbow and wrist joint angles"""
        # Calculate elbow angle
        upper_arm_vec = elbow_pos  # From shoulder to elbow
        forearm_vec = target_pos - elbow_pos  # From elbow to target

        elbow_angle = self.calculate_elbow_angle(upper_arm_vec, forearm_vec)

        # Calculate wrist angles to achieve target orientation
        wrist_angles = self.calculate_wrist_angles(target_rot, shoulder_angles, elbow_angle)

        return elbow_angle, wrist_angles

    def calculate_elbow_angle(self, upper_arm_vec, forearm_vec):
        """Calculate elbow joint angle"""
        # Use dot product to find angle between upper arm and forearm
        cos_angle = np.dot(upper_arm_vec, forearm_vec) / (
            np.linalg.norm(upper_arm_vec) * np.linalg.norm(forearm_vec)
        )
        cos_angle = np.clip(cos_angle, -1, 1)

        # Elbow angle is supplementary to the angle between vectors
        return np.pi - np.arccos(cos_angle)

    def calculate_wrist_angles(self, target_rot, shoulder_angles, elbow_angle):
        """Calculate wrist joint angles"""
        # Simplified wrist calculation
        # In practice, this would involve more complex orientation calculations
        return np.array([0, 0, 0])  # Placeholder
```

### Whole-Body Kinematics

#### Kinematic Chains Integration

Humanoid robots require coordination between multiple kinematic chains:

##### Torso Integration

The torso connects arms and legs, affecting the kinematics of both:

```python
class WholeBodyKinematics:
    def __init__(self):
        self.left_arm_ik = HumanoidArmIK()
        self.right_arm_ik = HumanoidArmIK()
        self.left_leg_ik = HumanoidLegIK()
        self.right_leg_ik = HumanoidLegIK()

    def solve_whole_body_ik(self, targets, current_angles):
        """Solve inverse kinematics for entire humanoid body"""
        # targets: dictionary with desired positions for end-effectors
        # e.g., {'left_hand': [x,y,z], 'right_foot': [x,y,z], ...}

        # Initialize with current angles
        result_angles = current_angles.copy()

        # Solve for each chain with constraints
        if 'left_hand' in targets:
            left_arm_angles = self.left_arm_ik.solve_arm_ik(
                targets['left_hand']
            )
            result_angles[LEFT_ARM_JOINTS] = left_arm_angles

        if 'right_hand' in targets:
            right_arm_angles = self.right_arm_ik.solve_arm_ik(
                targets['right_hand']
            )
            result_angles[RIGHT_ARM_JOINTS] = right_arm_angles

        if 'left_foot' in targets:
            left_leg_angles = self.left_leg_ik.solve_leg_ik(
                targets['left_foot'], 'left'
            )
            result_angles[LEFT_LEG_JOINTS] = left_leg_angles

        if 'right_foot' in targets:
            right_leg_angles = self.right_leg_ik.solve_leg_ik(
                targets['right_foot'], 'right'
            )
            result_angles[RIGHT_LEG_JOINTS] = right_leg_angles

        # Apply whole-body optimization to resolve conflicts
        result_angles = self.resolve_conflicts(result_angles, targets)

        return result_angles

    def resolve_conflicts(self, angles, targets):
        """Resolve conflicts between different kinematic chains"""
        # This would implement whole-body optimization
        # to ensure all constraints are satisfied simultaneously
        # while respecting joint limits and balance requirements

        # Placeholder implementation
        return angles
```

### Kinematic Constraints and Limitations

#### Joint Limits

All joints have physical limitations that must be respected:

##### Soft vs. Hard Limits

- **Hard Limits**: Physical stops that prevent motion
- **Soft Limits**: Software limits for safety and control

##### Singularity Handling

```python
def check_for_singularities(jacobian, threshold=1e-6):
    """Check if the robot is near a singularity"""
    # Calculate condition number of Jacobian
    cond_num = np.linalg.cond(jacobian)

    if cond_num > 1/threshold:
        return True, cond_num
    else:
        return False, cond_num

def damped_least_squares(jacobian, damping_factor=0.01):
    """Calculate damped pseudo-inverse to handle singularities"""
    J = jacobian
    I = np.eye(J.shape[1])

    # Damped least squares: J# = J^T * (J * J^T + λ² * I)^-1
    J_damped = np.dot(
        J.T,
        np.linalg.inv(np.dot(J, J.T) + damping_factor**2 * np.eye(J.shape[0]))
    )

    return J_damped
```

### Implementation Considerations

#### Numerical Stability

Kinematic calculations must be numerically stable:

##### Floating Point Precision

- Use appropriate tolerances for comparisons
- Avoid division by very small numbers
- Clamp inverse trigonometric function inputs

##### Coordinate System Management

```python
class CoordinateSystemManager:
    def __init__(self):
        self.transforms = {}  # Store all coordinate frame transforms

    def add_transform(self, from_frame, to_frame, transform_matrix):
        """Add a transformation between coordinate frames"""
        key = (from_frame, to_frame)
        self.transforms[key] = transform_matrix

    def get_transform(self, from_frame, to_frame):
        """Get transformation between coordinate frames"""
        key = (from_frame, to_frame)
        if key in self.transforms:
            return self.transforms[key]
        else:
            # Calculate if not stored
            return self.calculate_transform(from_frame, to_frame)

    def transform_point(self, point, from_frame, to_frame):
        """Transform a point from one frame to another"""
        T = self.get_transform(from_frame, to_frame)
        point_homogeneous = np.append(point, 1)
        transformed_point = np.dot(T, point_homogeneous)
        return transformed_point[:3]
```

#### Performance Optimization

##### Caching

- Cache transformation matrices when possible
- Pre-compute Jacobians for common configurations
- Use lookup tables for expensive trigonometric calculations

##### Parallel Processing

For real-time applications, consider parallel processing:
- Multi-threaded inverse kinematics
- GPU acceleration for matrix operations
- Vectorized calculations for multiple end-effectors

### Best Practices

#### Design Principles

1. **Modular Implementation**: Separate forward and inverse kinematics
2. **Validation**: Verify solutions with forward kinematics
3. **Error Handling**: Graceful degradation when no solution exists
4. **Documentation**: Clear coordinate frame definitions

#### Testing Strategies

1. **Forward-Backward Testing**: FK of IK solution should match target
2. **Boundary Testing**: Test at joint limits and singularities
3. **Continuity Testing**: Ensure smooth transitions between poses
4. **Real-time Testing**: Verify computational performance requirements

### Future Developments

#### Machine Learning Integration

- **Neural Network IK**: Learning-based inverse kinematics
- **Adaptive Kinematics**: Self-calibrating kinematic models
- **Predictive Kinematics**: Anticipating future kinematic states

#### Advanced Techniques

- **Task-Space Optimization**: Direct optimization in task space
- **Multi-Objective IK**: Balancing multiple competing objectives
- **Learning from Demonstration**: Imitating human movements

### Conclusion

Humanoid robot kinematics is a complex but fundamental aspect of humanoid robotics, requiring understanding of both forward and inverse kinematics for effective control. The large number of degrees of freedom in humanoid robots presents computational and control challenges that require sophisticated mathematical approaches and careful implementation. Modern approaches often combine analytical methods with numerical optimization to achieve robust and efficient kinematic solutions. As humanoid robotics continues to advance, kinematic algorithms will need to become more sophisticated, handling redundancy, avoiding singularities, and enabling the complex movements required for human-like behavior. The integration of machine learning techniques promises to further enhance kinematic capabilities, enabling more adaptive and human-like movement patterns in humanoid robots.