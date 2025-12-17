---
sidebar_position: 4
---

# LLM-Based Cognitive Planning

## Using Large Language Models for Complex Task Planning in Robotics

Large Language Models (LLMs) have emerged as powerful tools for cognitive planning in robotics, enabling robots to understand high-level natural language commands and translate them into detailed sequences of executable actions. This section explores how LLMs can be integrated into robotic systems to provide sophisticated task planning and reasoning capabilities.

### Introduction to LLM-Based Planning

LLM-based planning leverages the reasoning and knowledge capabilities of large language models to address complex robotic task planning challenges. Unlike traditional symbolic planning approaches, LLMs can handle ambiguous, high-level commands and incorporate common-sense knowledge about the physical world.

#### Advantages of LLM-Based Planning

- **Natural Language Understanding**: Direct interpretation of human commands
- **Common-Sense Reasoning**: Knowledge about physical world constraints
- **Flexible Planning**: Adaptation to novel situations
- **Multi-step Reasoning**: Complex task decomposition
- **Context Awareness**: Understanding of environmental context

#### Challenges and Considerations

- **Reliability**: Ensuring consistent and safe behavior
- **Grounding**: Connecting abstract plans to physical reality
- **Real-time Performance**: Meeting timing constraints
- **Verification**: Validating plan correctness
- **Safety**: Preventing dangerous or incorrect actions

### LLM Integration Architectures

#### Direct Integration Approach

The direct integration approach uses LLMs as the primary planning component:

```python
import openai
import json
from typing import List, Dict, Any

class DirectLLMPlanner:
    def __init__(self, api_key: str, model: str = "gpt-4"):
        openai.api_key = api_key
        self.model = model
        self.system_prompt = self.create_system_prompt()

    def create_system_prompt(self) -> str:
        """Create system prompt for the LLM planner"""
        return """
        You are a robotic task planner. Your role is to convert high-level natural language commands
        into detailed, executable action sequences for a mobile manipulator robot.

        Robot capabilities:
        - Navigation: Move to locations in the environment
        - Manipulation: Grasp and place objects
        - Perception: Detect and recognize objects
        - Interaction: Open/close doors, press buttons

        Output format: JSON with action sequence
        {
            "actions": [
                {
                    "type": "navigation|manipulation|perception|interaction",
                    "target": "location|object|action",
                    "parameters": {...}
                }
            ],
            "reasoning": "Brief explanation of the plan"
        }

        Constraints:
        - Ensure each action is physically possible
        - Consider robot workspace limitations
        - Account for object properties and affordances
        - Plan for safety and efficiency
        """

    def plan_task(self, command: str, environment_context: Dict[str, Any]) -> Dict[str, Any]:
        """Plan task using LLM"""
        user_prompt = self.create_user_prompt(command, environment_context)

        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.1,  # Low temperature for consistency
            max_tokens=1000
        )

        try:
            # Extract JSON from response
            response_text = response.choices[0].message.content
            plan = self.extract_json_from_response(response_text)
            return plan
        except Exception as e:
            print(f"Error parsing LLM response: {e}")
            return self.create_fallback_plan(command)

    def create_user_prompt(self, command: str, environment_context: Dict[str, Any]) -> str:
        """Create user prompt with environment context"""
        context_str = json.dumps(environment_context, indent=2)

        return f"""
        Command: {command}

        Environment Context:
        {context_str}

        Please generate a detailed action plan to execute this command.
        """

    def extract_json_from_response(self, response_text: str) -> Dict[str, Any]:
        """Extract JSON from LLM response"""
        # Find JSON between curly braces
        start = response_text.find('{')
        end = response_text.rfind('}') + 1

        if start != -1 and end != 0:
            json_str = response_text[start:end]
            return json.loads(json_str)

        # If no JSON found, try to parse the whole response
        return json.loads(response_text)

    def create_fallback_plan(self, command: str) -> Dict[str, Any]:
        """Create fallback plan if LLM fails"""
        return {
            "actions": [
                {
                    "type": "error",
                    "target": "unknown",
                    "parameters": {"error": f"Could not plan command: {command}"}
                }
            ],
            "reasoning": "LLM planning failed, fallback response"
        }
```

#### Hybrid Planning Architecture

A hybrid approach combines LLM reasoning with traditional planning methods:

```python
class HybridPlanningSystem:
    def __init__(self, llm_planner: DirectLLMPlanner, classical_planner):
        self.llm_planner = llm_planner
        self.classical_planner = classical_planner
        self.task_decomposer = TaskDecomposer()

    def plan_complex_task(self, command: str, environment_context: Dict[str, Any]) -> List[Dict]:
        """Plan complex task using hybrid approach"""
        # Use LLM for high-level task decomposition
        high_level_plan = self.llm_planner.plan_task(command, environment_context)

        # Refine with classical planning
        refined_plan = []
        for action in high_level_plan.get('actions', []):
            if action['type'] in ['navigation', 'manipulation']:
                # Use classical planner for detailed motion planning
                detailed_action = self.refine_action_with_classical_planner(action, environment_context)
                refined_plan.append(detailed_action)
            else:
                # Keep high-level action from LLM
                refined_plan.append(action)

        return refined_plan

    def refine_action_with_classical_planner(self, high_level_action: Dict, env_context: Dict) -> Dict:
        """Refine high-level action with classical planning"""
        if high_level_action['type'] == 'navigation':
            # Get detailed path from classical planner
            target_location = high_level_action['target']
            path = self.classical_planner.plan_navigation_path(target_location, env_context)

            return {
                **high_level_action,
                'detailed_plan': path,
                'safety_checks': self.generate_safety_checks(path)
            }

        elif high_level_action['type'] == 'manipulation':
            # Get detailed grasp plan from classical planner
            target_object = high_level_action['target']
            grasp_plan = self.classical_planner.plan_grasp(target_object, env_context)

            return {
                **high_level_action,
                'grasp_plan': grasp_plan,
                'approach_path': self.generate_approach_path(grasp_plan)
            }

        return high_level_action

    def generate_safety_checks(self, path: List) -> List[str]:
        """Generate safety checks for navigation path"""
        return [
            "check_for_dynamic_obstacles",
            "verify_path_clearance",
            "monitor_balance_during_movement"
        ]

    def generate_approach_path(self, grasp_plan: Dict) -> List:
        """Generate approach path for manipulation"""
        # Calculate approach trajectory to grasp pose
        approach_poses = []
        # Implementation would calculate intermediate poses
        return approach_poses
```

### Task Decomposition and Reasoning

#### Hierarchical Task Decomposition

LLMs excel at breaking down complex tasks into manageable subtasks:

```python
class TaskDecomposer:
    def __init__(self):
        self.subtask_templates = self.load_subtask_templates()

    def decompose_task(self, high_level_command: str, context: Dict) -> List[Dict]:
        """Decompose high-level command into subtasks"""
        # Use LLM to decompose task
        decomposition_prompt = f"""
        Decompose the following command into a sequence of subtasks:
        Command: {high_level_command}

        Environment context: {json.dumps(context)}

        Output format:
        {{
            "subtasks": [
                {{
                    "id": "unique_id",
                    "description": "what to do",
                    "type": "navigation|manipulation|perception|interaction",
                    "dependencies": ["other_subtask_ids"],
                    "success_criteria": "how to verify completion"
                }}
            ]
        }}
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a task decomposition expert for robotics."},
                {"role": "user", "content": decomposition_prompt}
            ]
        )

        try:
            result = json.loads(response.choices[0].message.content)
            return result['subtasks']
        except:
            # Fallback decomposition
            return self.fallback_decomposition(high_level_command)

    def fallback_decomposition(self, command: str) -> List[Dict]:
        """Fallback task decomposition"""
        # Simple rule-based decomposition
        if "clean" in command.lower():
            return [
                {"id": "1", "description": "Navigate to cleaning area", "type": "navigation"},
                {"id": "2", "description": "Detect cleaning supplies", "type": "perception"},
                {"id": "3", "description": "Grasp cleaning tool", "type": "manipulation"},
                {"id": "4", "description": "Clean the area", "type": "manipulation"},
                {"id": "5", "description": "Return cleaning tool", "type": "manipulation"}
            ]
        elif "bring" in command.lower() or "get" in command.lower():
            return [
                {"id": "1", "description": "Understand object to retrieve", "type": "perception"},
                {"id": "2", "description": "Navigate to object location", "type": "navigation"},
                {"id": "3", "description": "Grasp the object", "type": "manipulation"},
                {"id": "4", "description": "Navigate to destination", "type": "navigation"},
                {"id": "5", "description": "Place object", "type": "manipulation"}
            ]
        else:
            return [{"id": "1", "description": command, "type": "unknown"}]

    def validate_decomposition(self, subtasks: List[Dict], context: Dict) -> bool:
        """Validate task decomposition for feasibility"""
        # Check if robot has required capabilities
        required_capabilities = set()
        for subtask in subtasks:
            required_capabilities.add(subtask['type'])

        available_capabilities = context.get('robot_capabilities', set())

        if not required_capabilities.issubset(available_capabilities):
            missing = required_capabilities - available_capabilities
            print(f"Missing capabilities: {missing}")
            return False

        # Check for circular dependencies
        if self.has_circular_dependencies(subtasks):
            print("Circular dependencies detected in task decomposition")
            return False

        return True

    def has_circular_dependencies(self, subtasks: List[Dict]) -> bool:
        """Check for circular dependencies in task graph"""
        # Build dependency graph
        graph = {}
        for subtask in subtasks:
            graph[subtask['id']] = subtask.get('dependencies', [])

        # Check for cycles using DFS
        visiting = set()
        visited = set()

        def has_cycle(node):
            if node in visiting:
                return True
            if node in visited:
                return False

            visiting.add(node)
            for dep in graph.get(node, []):
                if has_cycle(dep):
                    return True
            visiting.remove(node)
            visited.add(node)
            return False

        for subtask in subtasks:
            if subtask['id'] not in visited:
                if has_cycle(subtask['id']):
                    return True

        return False
```

#### Context-Aware Planning

LLMs can incorporate environmental context into planning decisions:

```python
class ContextAwarePlanner:
    def __init__(self):
        self.context_encoder = ContextEncoder()

    def plan_with_context(self, command: str, environment_state: Dict, robot_state: Dict) -> Dict:
        """Plan task considering environmental and robot context"""
        # Encode context into planning prompt
        encoded_context = self.context_encoder.encode(
            environment_state, robot_state
        )

        planning_prompt = f"""
        Plan the following task considering the context:

        Command: {command}

        Environment State:
        {encoded_context['environment']}

        Robot State:
        {encoded_context['robot']}

        Generate a detailed plan that:
        1. Respects current environmental constraints
        2. Considers robot's current state and limitations
        3. Accounts for safety requirements
        4. Optimizes for efficiency

        Output JSON with:
        - action_sequence: list of actions with parameters
        - safety_considerations: list of safety checks needed
        - alternative_strategies: alternative approaches if primary fails
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.get_planning_system_prompt()},
                {"role": "user", "content": planning_prompt}
            ]
        )

        return json.loads(response.choices[0].message.content)

    def get_planning_system_prompt(self) -> str:
        """Get system prompt for context-aware planning"""
        return """
        You are an expert robotic task planner that considers environmental and robot state context.
        Always ensure plans are:
        - Safe: No actions that could harm robot or environment
        - Feasible: Robot has required capabilities and current state allows action
        - Efficient: Minimize unnecessary movements and actions
        - Robust: Include safety checks and alternative strategies

        Consider:
        - Current robot pose and battery level
        - Environment layout and object positions
        - Obstacle locations and navigation constraints
        - Object properties and manipulation requirements
        """

    def update_context_during_execution(self, plan: Dict, feedback: Dict) -> Dict:
        """Update plan based on execution feedback"""
        update_prompt = f"""
        Original plan: {json.dumps(plan)}

        Execution feedback: {json.dumps(feedback)}

        Update the plan considering the new information:
        - Adjust for failed actions
        - Account for changed environmental state
        - Modify based on actual robot state
        - Suggest alternative strategies if needed
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.get_planning_system_prompt()},
                {"role": "user", "content": update_prompt}
            ]
        )

        return json.loads(response.choices[0].message.content)

class ContextEncoder:
    def encode(self, environment_state: Dict, robot_state: Dict) -> Dict:
        """Encode environment and robot states for LLM consumption"""
        return {
            'environment': {
                'layout': environment_state.get('layout', {}),
                'objects': environment_state.get('objects', []),
                'obstacles': environment_state.get('obstacles', []),
                'navigation_map': environment_state.get('navigation_map', {}),
                'object_locations': self.extract_object_locations(environment_state)
            },
            'robot': {
                'current_pose': robot_state.get('pose', [0, 0, 0]),
                'battery_level': robot_state.get('battery', 100),
                'current_task': robot_state.get('current_task', None),
                'manipulator_state': robot_state.get('manipulator', {}),
                'navigation_capabilities': robot_state.get('navigation', {}),
                'manipulation_capabilities': robot_state.get('manipulation', {})
            }
        }

    def extract_object_locations(self, env_state: Dict) -> Dict:
        """Extract object location information"""
        locations = {}
        objects = env_state.get('objects', [])

        for obj in objects:
            obj_name = obj.get('name', 'unknown')
            obj_pose = obj.get('pose', [0, 0, 0])
            locations[obj_name] = {
                'position': obj_pose[:2],  # x, y coordinates
                'height': obj_pose[2],
                'reachable': self.is_reachable(obj_pose)
            }

        return locations

    def is_reachable(self, pose: List[float]) -> bool:
        """Check if pose is reachable by robot manipulator"""
        # Simple reachability check based on robot workspace
        x, y, z = pose
        workspace_radius = 1.0  # meters
        workspace_height_range = [0.1, 1.5]  # meters

        distance_from_robot = (x**2 + y**2)**0.5
        in_range = (distance_from_robot <= workspace_radius and
                   workspace_height_range[0] <= z <= workspace_height_range[1])

        return in_range
```

### Safety and Verification

#### Plan Verification and Validation

LLM-generated plans must be verified for safety and correctness:

```python
class PlanVerifier:
    def __init__(self):
        self.safety_rules = self.load_safety_rules()

    def verify_plan(self, plan: Dict, robot_spec: Dict, environment: Dict) -> Dict:
        """Verify plan for safety and feasibility"""
        verification_results = {
            'safe': True,
            'feasible': True,
            'warnings': [],
            'errors': []
        }

        # Check each action in the plan
        for i, action in enumerate(plan.get('actions', [])):
            action_verification = self.verify_action(
                action, robot_spec, environment, plan
            )

            if not action_verification['safe']:
                verification_results['safe'] = False
                verification_results['errors'].extend(action_verification['errors'])

            if not action_verification['feasible']:
                verification_results['feasible'] = False
                verification_results['errors'].extend(action_verification['errors'])

            verification_results['warnings'].extend(action_verification['warnings'])

        return verification_results

    def verify_action(self, action: Dict, robot_spec: Dict, environment: Dict, full_plan: Dict) -> Dict:
        """Verify individual action"""
        verification = {
            'safe': True,
            'feasible': True,
            'warnings': [],
            'errors': []
        }

        action_type = action.get('type', 'unknown')

        if action_type == 'navigation':
            self.verify_navigation_action(action, robot_spec, environment, verification)
        elif action_type == 'manipulation':
            self.verify_manipulation_action(action, robot_spec, environment, verification)
        elif action_type == 'perception':
            self.verify_perception_action(action, robot_spec, environment, verification)
        elif action_type == 'interaction':
            self.verify_interaction_action(action, robot_spec, environment, verification)

        return verification

    def verify_navigation_action(self, action: Dict, robot_spec: Dict, environment: Dict, verification: Dict):
        """Verify navigation action safety and feasibility"""
        target = action.get('target')

        # Check if target location exists in environment
        if not self.location_exists(target, environment):
            verification['feasible'] = False
            verification['errors'].append(f"Target location '{target}' does not exist in environment")

        # Check if path is navigable
        if self.path_has_obstacles(target, environment):
            verification['safe'] = False
            verification['warnings'].append(f"Path to '{target}' may have obstacles")

        # Check robot navigation capabilities
        max_speed = robot_spec.get('navigation', {}).get('max_speed', 1.0)
        if action.get('parameters', {}).get('speed', 0.5) > max_speed:
            verification['safe'] = False
            verification['warnings'].append(f"Navigation speed exceeds robot capability")

    def verify_manipulation_action(self, action: Dict, robot_spec: Dict, environment: Dict, verification: Dict):
        """Verify manipulation action safety and feasibility"""
        target_object = action.get('target')

        # Check if object is manipulable
        if not self.object_is_manipulable(target_object, environment):
            verification['feasible'] = False
            verification['errors'].append(f"Object '{target_object}' is not manipulable")

        # Check robot manipulation capabilities
        max_load = robot_spec.get('manipulation', {}).get('max_load', 5.0)  # kg
        object_weight = self.get_object_weight(target_object, environment)

        if object_weight and object_weight > max_load:
            verification['safe'] = False
            verification['errors'].append(f"Object '{target_object}' is too heavy for robot")

        # Check reachability
        object_pose = self.get_object_pose(target_object, environment)
        if object_pose and not self.is_reachable(object_pose, robot_spec):
            verification['feasible'] = False
            verification['errors'].append(f"Object '{target_object}' is not reachable")

    def verify_perception_action(self, action: Dict, robot_spec: Dict, environment: Dict, verification: Dict):
        """Verify perception action safety and feasibility"""
        target = action.get('target')

        # Check if target is observable
        if not self.target_is_observable(target, robot_spec):
            verification['feasible'] = False
            verification['errors'].append(f"Target '{target}' is not observable with robot sensors")

    def verify_interaction_action(self, action: Dict, robot_spec: Dict, environment: Dict, verification: Dict):
        """Verify interaction action safety and feasibility"""
        target = action.get('target')

        # Check if interaction is safe
        if self.interaction_is_dangerous(target, environment):
            verification['safe'] = False
            verification['errors'].append(f"Interaction with '{target}' may be dangerous")

    def location_exists(self, location: str, environment: Dict) -> bool:
        """Check if location exists in environment"""
        locations = environment.get('navigation_map', {}).get('locations', {})
        return location in locations

    def path_has_obstacles(self, target: str, environment: Dict) -> bool:
        """Check if path to target has obstacles"""
        # Implementation would check navigation map for obstacles
        return False  # Placeholder

    def object_is_manipulable(self, obj_name: str, environment: Dict) -> bool:
        """Check if object can be manipulated"""
        objects = environment.get('objects', [])
        for obj in objects:
            if obj.get('name') == obj_name:
                return obj.get('manipulable', True)
        return False

    def get_object_weight(self, obj_name: str, environment: Dict) -> float:
        """Get object weight from environment"""
        objects = environment.get('objects', [])
        for obj in objects:
            if obj.get('name') == obj_name:
                return obj.get('weight', 0.0)
        return 0.0

    def get_object_pose(self, obj_name: str, environment: Dict) -> List[float]:
        """Get object pose from environment"""
        objects = environment.get('objects', [])
        for obj in objects:
            if obj.get('name') == obj_name:
                return obj.get('pose', [0, 0, 0])
        return [0, 0, 0]

    def is_reachable(self, pose: List[float], robot_spec: Dict) -> bool:
        """Check if pose is reachable by robot"""
        workspace = robot_spec.get('manipulation', {}).get('workspace', {})
        # Implementation would check if pose is within workspace
        return True  # Placeholder

    def target_is_observable(self, target: str, robot_spec: Dict) -> bool:
        """Check if target is observable by robot sensors"""
        sensor_range = robot_spec.get('perception', {}).get('range', 5.0)
        # Implementation would check if target is within sensor range
        return True  # Placeholder

    def interaction_is_dangerous(self, target: str, environment: Dict) -> bool:
        """Check if interaction with target is dangerous"""
        objects = environment.get('objects', [])
        for obj in objects:
            if obj.get('name') == target:
                return obj.get('dangerous', False)
        return False

    def load_safety_rules(self) -> Dict:
        """Load safety rules for plan verification"""
        return {
            'navigation': {
                'min_obstacle_distance': 0.3,  # meters
                'max_speed_indoors': 1.0,     # m/s
                'max_acceleration': 2.0       # m/s²
            },
            'manipulation': {
                'max_load': 5.0,              # kg
                'max_force': 100.0,           # N
                'min_approach_distance': 0.1  # m
            },
            'general': {
                'max_plan_length': 50,        # actions
                'max_execution_time': 300     # seconds
            }
        }
```

### Execution Monitoring and Adaptation

#### Real-time Plan Monitoring

LLM-based plans need continuous monitoring and adaptation:

```python
class PlanMonitor:
    def __init__(self, llm_client):
        self.llm_client = llm_client
        self.current_plan = None
        self.execution_state = {}
        self.failure_recovery_strategies = self.load_recovery_strategies()

    def monitor_execution(self, plan: Dict, robot_feedback: Dict) -> Dict:
        """Monitor plan execution and detect issues"""
        self.current_plan = plan
        self.update_execution_state(robot_feedback)

        # Check for deviations from expected behavior
        deviations = self.detect_deviations(robot_feedback)

        if deviations:
            # Use LLM to suggest corrective actions
            corrective_action = self.suggest_correction(deviations, plan, robot_feedback)
            return {
                'status': 'deviation_detected',
                'deviations': deviations,
                'correction': corrective_action
            }

        # Check if current action is proceeding as expected
        current_action_status = self.evaluate_current_action(robot_feedback)

        if current_action_status['failed']:
            # Attempt recovery
            recovery_action = self.attempt_recovery(current_action_status, plan)
            return {
                'status': 'action_failed',
                'failure': current_action_status['failure'],
                'recovery': recovery_action
            }

        return {'status': 'proceeding_normally'}

    def detect_deviations(self, feedback: Dict) -> List[Dict]:
        """Detect deviations from expected plan execution"""
        deviations = []

        # Check position deviations for navigation
        if 'position_error' in feedback and abs(feedback['position_error']) > 0.2:  # 20cm threshold
            deviations.append({
                'type': 'navigation_deviation',
                'severity': 'warning',
                'details': f"Position error: {feedback['position_error']}m"
            })

        # Check force deviations for manipulation
        if 'force_deviation' in feedback and feedback['force_deviation'] > 50:  # 50N threshold
            deviations.append({
                'type': 'manipulation_deviation',
                'severity': 'critical',
                'details': f"Unexpected force: {feedback['force_deviation']}N"
            })

        # Check timeout conditions
        if feedback.get('action_timeout', False):
            deviations.append({
                'type': 'timeout',
                'severity': 'critical',
                'details': "Action timed out"
            })

        return deviations

    def suggest_correction(self, deviations: List[Dict], plan: Dict, feedback: Dict) -> Dict:
        """Use LLM to suggest correction for deviations"""
        correction_prompt = f"""
        Plan: {json.dumps(plan)}

        Execution feedback: {json.dumps(feedback)}

        Deviations detected: {json.dumps(deviations)}

        Suggest corrective action considering:
        1. The nature of the deviation
        2. The current plan context
        3. Robot capabilities and constraints
        4. Safety requirements

        Provide JSON response with:
        - action: specific corrective action to take
        - reasoning: why this action is appropriate
        - alternatives: other possible actions
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.get_correction_system_prompt()},
                {"role": "user", "content": correction_prompt}
            ]
        )

        try:
            return json.loads(response.choices[0].message.content)
        except:
            return self.get_default_correction(deviations)

    def get_correction_system_prompt(self) -> str:
        """Get system prompt for correction suggestions"""
        return """
        You are an expert in robotic plan correction. When deviations occur, suggest
        appropriate corrective actions that:
        1. Address the specific deviation
        2. Maintain overall task objectives
        3. Ensure robot safety
        4. Are executable by the robot
        5. Consider the current environmental state

        Prioritize safe and feasible corrections over aggressive recovery attempts.
        """

    def attempt_recovery(self, failure_info: Dict, plan: Dict) -> Dict:
        """Attempt to recover from action failure"""
        failure_type = failure_info['type']

        if failure_type in self.failure_recovery_strategies:
            strategy = self.failure_recovery_strategies[failure_type]

            if strategy['approach'] == 'retry':
                return self.retry_action(failure_info, plan)
            elif strategy['approach'] == 'alternative':
                return self.find_alternative_action(failure_info, plan)
            elif strategy['approach'] == 'skip':
                return self.skip_and_continue(failure_info, plan)
            else:
                return self.llm_guided_recovery(failure_info, plan)
        else:
            return self.llm_guided_recovery(failure_info, plan)

    def retry_action(self, failure_info: Dict, plan: Dict) -> Dict:
        """Retry the failed action"""
        return {
            'action': 'retry',
            'target_action': failure_info['action_id'],
            'parameters': failure_info.get('parameters', {}),
            'reasoning': 'Retrying failed action with same parameters'
        }

    def find_alternative_action(self, failure_info: Dict, plan: Dict) -> Dict:
        """Find alternative way to achieve the same goal"""
        alternative_prompt = f"""
        Failed action: {json.dumps(failure_info)}

        Current plan: {json.dumps(plan)}

        Find an alternative action that achieves the same goal considering:
        - Why the original action failed
        - Available alternative approaches
        - Current robot state and environment
        - Safety constraints

        Return JSON with alternative action.
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.get_correction_system_prompt()},
                {"role": "user", "content": alternative_prompt}
            ]
        )

        try:
            return json.loads(response.choices[0].message.content)
        except:
            return {'action': 'abort', 'reason': 'No viable alternative found'}

    def skip_and_continue(self, failure_info: Dict, plan: Dict) -> Dict:
        """Skip the failed action and continue with the plan"""
        return {
            'action': 'skip',
            'reasoning': 'Skipping failed action and continuing with plan',
            'next_action': self.get_next_action(failure_info, plan)
        }

    def llm_guided_recovery(self, failure_info: Dict, plan: Dict) -> Dict:
        """Use LLM for guided recovery from unknown failure types"""
        recovery_prompt = f"""
        Unknown failure occurred: {json.dumps(failure_info)}

        Current plan context: {json.dumps(plan)}

        Suggest appropriate recovery strategy considering:
        - Severity of failure
        - Impact on overall task
        - Available recovery options
        - Safety implications

        Provide recovery action as JSON.
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.get_correction_system_prompt()},
                {"role": "user", "content": recovery_prompt}
            ]
        )

        try:
            return json.loads(response.choices[0].message.content)
        except:
            return {'action': 'abort', 'reason': 'Unable to determine recovery strategy'}

    def get_next_action(self, failure_info: Dict, plan: Dict) -> Dict:
        """Get the next action after skipping current one"""
        current_idx = failure_info.get('action_index', 0)
        actions = plan.get('actions', [])

        if current_idx + 1 < len(actions):
            return actions[current_idx + 1]
        else:
            return {'type': 'complete', 'reason': 'End of plan reached'}

    def load_recovery_strategies(self) -> Dict:
        """Load predefined recovery strategies"""
        return {
            'navigation_failure': {
                'approach': 'alternative',
                'alternatives': ['try_different_path', 'request_assistance', 'abort']
            },
            'grasp_failure': {
                'approach': 'alternative',
                'alternatives': ['try_different_grasp', 'adjust_approach', 'abort']
            },
            'perception_failure': {
                'approach': 'retry',
                'alternatives': ['change_viewpoint', 'adjust_lighting', 'abort']
            },
            'collision_detected': {
                'approach': 'skip',
                'alternatives': ['replan_path', 'request_assistance', 'abort']
            }
        }

    def update_execution_state(self, feedback: Dict):
        """Update internal execution state with feedback"""
        self.execution_state.update(feedback)
        self.execution_state['timestamp'] = time.time()
```

### Integration with Robot Control Systems

#### ROS 2 Integration

LLM-based planning integrates with robotic systems through ROS 2:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus

class LLMPlanningNode(Node):
    def __init__(self):
        super().__init__('llm_planning_node')

        # Initialize LLM components
        self.llm_planner = DirectLLMPlanner(api_key=self.get_llm_api_key())
        self.hybrid_planner = HybridPlanningSystem(self.llm_planner, self.get_classical_planner())
        self.plan_verifier = PlanVerifier()
        self.plan_monitor = PlanMonitor(self.llm_planner)

        # ROS 2 interfaces
        self.command_sub = self.create_subscription(
            String, 'high_level_commands', self.command_callback, 10
        )

        self.status_pub = self.create_publisher(String, 'planning_status', 10)
        self.action_feedback_pub = self.create_publisher(String, 'action_feedback', 10)

        # Action clients for different robot capabilities
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manip_client = ActionClient(self, FollowJointTrajectory, 'manipulator_controller/follow_joint_trajectory')

        # Planning parameters
        self.declare_parameter('max_plan_length', 50)
        self.declare_parameter('planning_timeout', 30.0)
        self.declare_parameter('verification_enabled', True)

    def command_callback(self, msg: String):
        """Handle high-level command from user"""
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Publish planning status
        status_msg = String()
        status_msg.data = f"Planning command: {command}"
        self.status_pub.publish(status_msg)

        # Get current environment and robot state
        env_context = self.get_environment_context()
        robot_state = self.get_robot_state()

        try:
            # Plan the task using LLM
            plan = self.hybrid_planner.plan_complex_task(command, {
                'environment': env_context,
                'robot': robot_state
            })

            # Verify the plan for safety
            if self.get_parameter('verification_enabled').value:
                verification = self.plan_verifier.verify_plan(plan, robot_state, env_context)

                if not verification['safe']:
                    self.get_logger().error(f"Plan failed safety verification: {verification['errors']}")
                    self.publish_error(f"Plan not safe: {verification['errors']}")
                    return

                if not verification['feasible']:
                    self.get_logger().error(f"Plan not feasible: {verification['errors']}")
                    self.publish_error(f"Plan not feasible: {verification['errors']}")
                    return

            # Execute the plan
            execution_result = self.execute_plan(plan)

            # Report results
            result_msg = String()
            result_msg.data = f"Command '{command}' executed: {execution_result}"
            self.status_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f"Planning/execution error: {str(e)}")
            self.publish_error(f"Command failed: {str(e)}")

    def execute_plan(self, plan: Dict) -> str:
        """Execute the planned sequence of actions"""
        for i, action in enumerate(plan.get('actions', [])):
            self.get_logger().info(f"Executing action {i+1}/{len(plan['actions'])}: {action}")

            try:
                # Monitor execution
                feedback = self.execute_single_action(action)

                # Check if action completed successfully
                if not self.action_completed_successfully(feedback):
                    self.get_logger().warn(f"Action {i} failed, attempting recovery")

                    # Try to recover
                    recovery_result = self.attempt_recovery(action, plan, feedback)
                    if not recovery_result:
                        return f"Failed at action {i}, recovery unsuccessful"

                # Publish action feedback
                feedback_msg = String()
                feedback_msg.data = f"Completed action {i}: {action['type']}"
                self.action_feedback_pub.publish(feedback_msg)

            except Exception as e:
                self.get_logger().error(f"Error executing action {i}: {str(e)}")
                return f"Failed at action {i}: {str(e)}"

        return "Plan completed successfully"

    def execute_single_action(self, action: Dict) -> Dict:
        """Execute a single action and return feedback"""
        action_type = action.get('type')

        if action_type == 'navigation':
            return self.execute_navigation_action(action)
        elif action_type == 'manipulation':
            return self.execute_manipulation_action(action)
        elif action_type == 'perception':
            return self.execute_perception_action(action)
        elif action_type == 'interaction':
            return self.execute_interaction_action(action)
        else:
            return {'status': 'error', 'message': f"Unknown action type: {action_type}"}

    def execute_navigation_action(self, action: Dict) -> Dict:
        """Execute navigation action"""
        target_pose = self.get_target_pose(action.get('target'))

        if not target_pose:
            return {'status': 'error', 'message': f"Unknown navigation target: {action.get('target')}"}

        # Send navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        # Wait for action server
        self.nav_client.wait_for_server()

        # Send goal and wait for result
        future = self.nav_client.send_goal_async(goal_msg)

        # Monitor execution
        return self.monitor_navigation_execution(future)

    def execute_manipulation_action(self, action: Dict) -> Dict:
        """Execute manipulation action"""
        # Implementation would send manipulation commands
        # to robot's manipulator controller
        pass

    def execute_perception_action(self, action: Dict) -> Dict:
        """Execute perception action"""
        # Implementation would trigger perception systems
        # and return recognition results
        pass

    def execute_interaction_action(self, action: Dict) -> Dict:
        """Execute interaction action"""
        # Implementation would handle interactions like
        # opening doors, pressing buttons, etc.
        pass

    def monitor_navigation_execution(self, future) -> Dict:
        """Monitor navigation execution and return feedback"""
        # Wait for navigation result with timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() is not None:
            result = future.result().result
            return {'status': 'success', 'result': result}
        else:
            return {'status': 'timeout', 'message': 'Navigation timed out'}

    def action_completed_successfully(self, feedback: Dict) -> bool:
        """Check if action completed successfully"""
        return feedback.get('status') == 'success'

    def attempt_recovery(self, failed_action: Dict, plan: Dict, feedback: Dict) -> bool:
        """Attempt to recover from action failure"""
        # Use plan monitor to suggest recovery
        recovery_plan = self.plan_monitor.attempt_recovery(
            {'action': failed_action, 'feedback': feedback}, plan
        )

        if recovery_plan['action'] == 'retry':
            # Retry the failed action
            retry_feedback = self.execute_single_action(failed_action)
            return self.action_completed_successfully(retry_feedback)

        elif recovery_plan['action'] == 'skip':
            # Continue with plan
            return True

        elif recovery_plan['action'] == 'alternative':
            # Execute alternative action
            alt_feedback = self.execute_single_action(recovery_plan)
            return self.action_completed_successfully(alt_feedback)

        else:
            return False  # Recovery failed

    def get_environment_context(self) -> Dict:
        """Get current environment context"""
        # This would interface with perception and mapping systems
        # to get current environment state
        return {
            'objects': self.get_detected_objects(),
            'obstacles': self.get_map_obstacles(),
            'locations': self.get_known_locations()
        }

    def get_robot_state(self) -> Dict:
        """Get current robot state"""
        # This would interface with robot state publisher
        # to get current robot status
        return {
            'pose': self.get_current_pose(),
            'battery': self.get_battery_level(),
            'manipulator': self.get_manipulator_state()
        }

    def get_detected_objects(self) -> List[Dict]:
        """Get currently detected objects"""
        # Interface with object detection system
        pass

    def get_current_pose(self) -> List[float]:
        """Get robot's current pose"""
        # Interface with localization system
        pass

    def get_battery_level(self) -> float:
        """Get robot's current battery level"""
        # Interface with power system
        pass

    def publish_error(self, error_msg: str):
        """Publish error status"""
        error_status = String()
        error_status.data = f"ERROR: {error_msg}"
        self.status_pub.publish(error_status)

    def get_llm_api_key(self) -> str:
        """Get LLM API key from parameters or environment"""
        # Implementation would retrieve API key securely
        pass

    def get_classical_planner(self):
        """Get classical planning system"""
        # Implementation would return classical planner interface
        pass
```

### Performance Optimization

#### Caching and Efficiency

LLM-based planning can be optimized for performance:

```python
import hashlib
import pickle
from functools import wraps
import time

class OptimizedLLMPlanner:
    def __init__(self, llm_planner, cache_size=1000):
        self.llm_planner = llm_planner
        self.cache = {}
        self.cache_order = []  # For LRU eviction
        self.cache_size = cache_size
        self.stats = {'hits': 0, 'misses': 0, 'total_time': 0}

    def plan_with_cache(self, command: str, context: Dict) -> Dict:
        """Plan task with caching for efficiency"""
        start_time = time.time()

        # Create cache key from command and context
        cache_key = self.create_cache_key(command, context)

        # Check cache first
        if cache_key in self.cache:
            self.stats['hits'] += 1
            result = self.cache[cache_key]
            self.stats['total_time'] += time.time() - start_time
            return result

        # Cache miss - plan normally
        self.stats['misses'] += 1
        result = self.llm_planner.plan_task(command, context)

        # Add to cache
        self.add_to_cache(cache_key, result)

        self.stats['total_time'] += time.time() - start_time
        return result

    def create_cache_key(self, command: str, context: Dict) -> str:
        """Create unique cache key for command and context"""
        combined = f"{command}_{hash(json.dumps(context, sort_keys=True))}"
        return hashlib.md5(combined.encode()).hexdigest()

    def add_to_cache(self, key: str, result: Dict):
        """Add result to cache with LRU eviction"""
        if key in self.cache:
            # Move to end (most recently used)
            self.cache_order.remove(key)
        elif len(self.cache) >= self.cache_size:
            # Evict least recently used
            oldest_key = self.cache_order.pop(0)
            del self.cache[oldest_key]

        self.cache[key] = result
        self.cache_order.append(key)

    def get_cache_stats(self) -> Dict:
        """Get cache performance statistics"""
        total_requests = self.stats['hits'] + self.stats['misses']
        hit_rate = self.stats['hits'] / total_requests if total_requests > 0 else 0

        return {
            'cache_size': len(self.cache),
            'max_size': self.cache_size,
            'hit_rate': hit_rate,
            'total_requests': total_requests,
            'average_time': self.stats['total_time'] / total_requests if total_requests > 0 else 0
        }

    def clear_cache(self):
        """Clear the cache"""
        self.cache.clear()
        self.cache_order.clear()
        self.stats = {'hits': 0, 'misses': 0, 'total_time': 0}
```

### Best Practices

#### Design Principles

1. **Layered Architecture**: Separate LLM reasoning from execution control
2. **Safety First**: Always verify LLM-generated plans before execution
3. **Fallback Mechanisms**: Provide alternatives when LLM fails
4. **Context Awareness**: Include environmental and robot state in prompts
5. **Monitoring**: Continuously monitor execution and adapt as needed

#### Testing Strategies

1. **Unit Testing**: Test individual planning components
2. **Integration Testing**: Test LLM integration with robotic systems
3. **Safety Testing**: Verify safety constraints are maintained
4. **Performance Testing**: Ensure real-time requirements are met
5. **Robustness Testing**: Test with ambiguous or incorrect commands

### Future Developments

#### Emerging Approaches

- **Fine-tuned Models**: Specialized LLMs for robotics planning
- **Multi-modal Integration**: Combining vision, language, and action
- **Learning from Execution**: Improving planning through experience
- **Collaborative Planning**: Multi-robot planning with LLM coordination
- **Explainable AI**: Making planning decisions interpretable to users

### Conclusion

LLM-based cognitive planning represents a significant advancement in robotic task planning, enabling robots to understand and execute complex, high-level commands through natural language interaction. The integration of large language models with traditional robotic systems creates powerful capabilities for task decomposition, reasoning, and execution monitoring. Success in LLM-based planning requires careful attention to safety, verification, and real-time performance constraints. As the technology continues to evolve, these systems will become increasingly sophisticated, enabling more natural and intuitive interaction between humans and robots in complex, real-world environments. The combination of advanced reasoning capabilities with robust safety mechanisms will continue to push the boundaries of what's possible in autonomous robotic systems.