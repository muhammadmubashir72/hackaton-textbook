---
sidebar_position: 5
---

# URDF - Unified Robot Description Format

## Defining Robot Models and Kinematics

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It provides a standardized way to define the physical and kinematic properties of robots, including their geometry, mass, joints, and visual properties. Understanding URDF is essential for simulating robots in Gazebo, visualizing them in RViz, and planning their movements.

### Introduction to URDF

URDF serves as the standard format for robot description in ROS and ROS 2. It allows developers to:

- Define the physical structure of a robot
- Specify joint relationships and kinematics
- Describe visual and collision properties
- Include sensor placements and other attachments
- Enable simulation and visualization in various tools

### URDF Structure and Elements

#### Root Element

Every URDF file has a single `robot` root element:

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- Robot description goes here -->
</robot>
```

#### Links

Links represent rigid bodies in the robot structure:

```xml
<link name="base_link">
    <inertial>
        <mass value="1.0" />
        <origin xyz="0 0 0" rpy="0 0 0" />
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
    </inertial>

    <visual>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
            <box size="1 1 1" />
        </geometry>
        <material name="blue">
            <color rgba="0 0 1 1" />
        </material>
    </visual>

    <collision>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
            <box size="1 1 1" />
        </geometry>
    </collision>
</link>
```

##### Link Components

1. **Inertial**: Defines mass and inertia properties for physics simulation
2. **Visual**: Defines how the link appears in visualization
3. **Collision**: Defines collision boundaries for physics simulation

#### Joints

Joints define the relationship between links:

```xml
<joint name="joint_name" type="joint_type">
    <parent link="parent_link_name" />
    <child link="child_link_name" />
    <origin xyz="0 0 0" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
</joint>
```

##### Joint Types

1. **revolute**: Rotational joint with limited range
2. **continuous**: Rotational joint without limits
3. **prismatic**: Linear sliding joint with limits
4. **fixed**: No movement between links
5. **floating**: 6-DOF movement
6. **planar**: Movement in a plane

### Complete Robot Example

Here's a more comprehensive example of a simple robot arm:

```xml
<?xml version="1.0"?>
<robot name="simple_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Base link -->
    <link name="base_link">
        <inertial>
            <mass value="1.0" />
            <origin xyz="0 0 0.1" />
            <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02" />
        </inertial>

        <visual>
            <origin xyz="0 0 0.1" />
            <geometry>
                <cylinder radius="0.1" length="0.2" />
            </geometry>
            <material name="grey">
                <color rgba="0.5 0.5 0.5 1" />
            </material>
        </visual>

        <collision>
            <origin xyz="0 0 0.1" />
            <geometry>
                <cylinder radius="0.1" length="0.2" />
            </geometry>
        </collision>
    </link>

    <!-- First joint -->
    <joint name="shoulder_joint" type="revolute">
        <parent link="base_link" />
        <child link="shoulder_link" />
        <origin xyz="0 0 0.2" />
        <axis xyz="0 1 0" />
        <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
    </joint>

    <!-- Shoulder link -->
    <link name="shoulder_link">
        <inertial>
            <mass value="0.5" />
            <origin xyz="0 0 0.1" />
            <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.01" />
        </inertial>

        <visual>
            <origin xyz="0 0 0.1" />
            <geometry>
                <cylinder radius="0.05" length="0.2" />
            </geometry>
            <material name="red">
                <color rgba="1 0 0 1" />
            </material>
        </visual>

        <collision>
            <origin xyz="0 0 0.1" />
            <geometry>
                <cylinder radius="0.05" length="0.2" />
            </geometry>
        </collision>
    </link>

    <!-- Second joint -->
    <joint name="elbow_joint" type="revolute">
        <parent link="shoulder_link" />
        <child link="elbow_link" />
        <origin xyz="0 0 0.2" />
        <axis xyz="0 1 0" />
        <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
    </joint>

    <!-- Elbow link -->
    <link name="elbow_link">
        <inertial>
            <mass value="0.3" />
            <origin xyz="0 0 0.1" />
            <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.006" />
        </inertial>

        <visual>
            <origin xyz="0 0 0.1" />
            <geometry>
                <cylinder radius="0.04" length="0.2" />
            </geometry>
            <material name="green">
                <color rgba="0 1 0 1" />
            </material>
        </visual>

        <collision>
            <origin xyz="0 0 0.1" />
            <geometry>
                <cylinder radius="0.04" length="0.2" />
            </geometry>
        </collision>
    </link>

    <!-- Third joint -->
    <joint name="wrist_joint" type="revolute">
        <parent link="elbow_link" />
        <child link="wrist_link" />
        <origin xyz="0 0 0.2" />
        <axis xyz="1 0 0" />
        <limit lower="-1.57" upper="1.57" effort="50" velocity="1" />
    </joint>

    <!-- Wrist link -->
    <link name="wrist_link">
        <inertial>
            <mass value="0.2" />
            <origin xyz="0 0 0.05" />
            <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.004" />
        </inertial>

        <visual>
            <origin xyz="0 0 0.05" />
            <geometry>
                <cylinder radius="0.03" length="0.1" />
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1" />
            </material>
        </visual>

        <collision>
            <origin xyz="0 0 0.05" />
            <geometry>
                <cylinder radius="0.03" length="0.1" />
            </geometry>
        </collision>
    </link>

</robot>
```

### Inertial Properties

The inertial properties are crucial for physics simulation:

#### Mass

The mass of the link in kilograms:

```xml
<mass value="1.0" />
```

#### Origin

The origin of the inertial reference frame relative to the link reference frame:

```xml
<origin xyz="0 0 0" rpy="0 0 0" />
```

#### Inertia Matrix

The 3x3 rotational inertia matrix in the inertia reference frame:

```xml
<inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02" />
```

For simple geometric shapes, you can calculate the inertia values:

- **Box**: `ixx = m/12 * (h² + d²)`, `iyy = m/12 * (w² + d²)`, `izz = m/12 * (w² + h²)`
- **Cylinder**: `ixx = iyy = m/12 * (3*r² + h²)`, `izz = m/2 * r²`
- **Sphere**: `ixx = iyy = izz = 2/5 * m * r²`

### Visual Properties

Visual properties define how the robot appears in visualization tools:

#### Origin

The origin of the visual reference frame:

```xml
<origin xyz="0 0 0" rpy="0 0 0" />
```

#### Geometry

The shape of the visual element:

```xml
<!-- Box -->
<geometry>
    <box size="1 1 1" />
</geometry>

<!-- Cylinder -->
<geometry>
    <cylinder radius="0.1" length="1.0" />
</geometry>

<!-- Sphere -->
<geometry>
    <sphere radius="0.1" />
</geometry>

<!-- Mesh -->
<geometry>
    <mesh filename="package://my_robot/meshes/link1.stl" />
</geometry>
```

#### Material

The color and appearance of the visual element:

```xml
<material name="red">
    <color rgba="1 0 0 1" />
</material>
```

### Collision Properties

Collision properties define the collision boundaries for physics simulation:

```xml
<collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
        <box size="1 1 1" />
    </geometry>
</collision>
```

Collision geometry can be simpler than visual geometry for performance reasons.

### Advanced URDF Features

#### Transmission Elements

For controlling joints with actuators:

```xml
<transmission name="trans_shoulder_joint">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="shoulder_joint">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="shoulder_motor">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>
```

#### Gazebo-Specific Elements

For simulation in Gazebo:

```xml
<gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
</gazebo>
```

#### Sensors

To include sensors in the robot description:

```xml
<gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
        <pose>0 0 0 0 0 0</pose>
        <visualize>true</visualize>
        <update_rate>30.0</update_rate>
        <camera name="head">
            <horizontal_fov>1.3962634</horizontal_fov>
            <image>
                <width>800</width>
                <height>800</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.1</near>
                <far>100</far>
            </clip>
        </camera>
    </sensor>
</gazebo>
```

### Xacro - XML Macros for URDF

Xacro allows you to create more maintainable URDF files using macros, properties, and mathematical expressions:

#### Basic Xacro Structure

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">

    <!-- Properties -->
    <xacro:property name="M_PI" value="3.1415926535897931" />
    <xacro:property name="base_width" value="0.5" />
    <xacro:property name="base_length" value="1.0" />
    <xacro:property name="base_height" value="0.2" />

    <!-- Macros -->
    <xacro:macro name="cylinder_inertial" params="mass radius length">
        <inertial>
            <mass value="${mass}" />
            <origin xyz="0 0 0" />
            <inertia ixx="${mass*(3*radius*radius + length*length)/12}"
                     ixy="0"
                     ixz="0"
                     iyy="${mass*(3*radius*radius + length*length)/12}"
                     iyz="0"
                     izz="${mass*radius*radius/2}" />
        </inertial>
    </xacro:macro>

    <!-- Using the macro -->
    <link name="base_link">
        <xacro:cylinder_inertial mass="1.0" radius="0.1" length="0.2" />

        <visual>
            <origin xyz="0 0 0" />
            <geometry>
                <cylinder radius="${base_width/2}" length="${base_height}" />
            </geometry>
        </visual>

        <collision>
            <origin xyz="0 0 0" />
            <geometry>
                <cylinder radius="${base_width/2}" length="${base_height}" />
            </geometry>
        </collision>
    </link>

</robot>
```

#### Conditional Statements

```xml
<xacro:if value="${has_laser}">
    <link name="laser_link">
        <!-- Laser sensor definition -->
    </link>
</xacro:if>
```

### URDF Validation and Tools

#### Checking URDF Files

ROS provides tools to validate URDF files:

```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# Display robot model
urdf_to_graphiz /path/to/robot.urdf
```

#### Visualization

```bash
# Launch robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat robot.urdf)'

# Visualize in RViz
ros2 run rviz2 rviz2
```

### Common URDF Issues and Solutions

#### Self-Collision Issues

Avoid self-collision by properly defining collision properties:

```xml
<!-- Define self-collision checking -->
<gazebo>
    <self_collide>true</self_collide>
</gazebo>
```

#### Joint Limits

Always define appropriate joint limits:

```xml
<limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
```

#### Mass and Inertia

Ensure all links have proper mass and inertia values:

```xml
<!-- At minimum, provide mass -->
<inertial>
    <mass value="0.001" />  <!-- Don't use 0 mass -->
    <origin xyz="0 0 0" />
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001" />
</inertial>
```

### Integration with ROS 2 Systems

#### Robot State Publisher

The robot state publisher uses URDF to publish TF transforms:

```python
import rclpy
from rclpy.node import Node
from urdf_parser_py.urdf import URDF

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        # The robot state publisher node uses URDF to publish transforms
```

#### Kinematics

URDF is used by kinematics packages for forward and inverse kinematics calculations.

### Best Practices

1. **Start Simple**: Begin with a basic model and add complexity gradually
2. **Use Xacro**: For complex robots, use Xacro to avoid repetition
3. **Proper Inertial Values**: Use realistic mass and inertia values for accurate simulation
4. **Collision vs Visual**: Use simpler geometry for collision than visual when possible
5. **Validate Regularly**: Check your URDF regularly as you build it
6. **Use Standard Units**: Always use SI units (meters, kilograms, seconds)
7. **Meaningful Names**: Use descriptive names for links and joints
8. **Documentation**: Comment complex URDF files to explain the structure

### Conclusion

URDF is a fundamental tool in robotics that enables the description of robot models for simulation, visualization, and control. Mastering URDF allows developers to create accurate robot models that can be used across the ROS ecosystem. Whether you're working with simple wheeled robots or complex humanoid systems, understanding URDF is essential for effective robotics development. The combination of proper kinematic description, accurate physical properties, and appropriate visual representation enables robots to be simulated, visualized, and controlled effectively in ROS-based systems.