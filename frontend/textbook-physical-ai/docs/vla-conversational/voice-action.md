---
sidebar_position: 3
---

# Voice-to-Action Systems

## Converting Natural Language Commands into Robotic Actions

Voice-to-action systems represent a crucial interface between human natural language and robotic action execution. These systems enable robots to understand spoken commands and translate them into sequences of executable robotic behaviors, forming the foundation for natural human-robot interaction in Physical AI applications.

### Introduction to Voice-to-Action Systems

Voice-to-action systems bridge the gap between human communication and robotic execution, enabling intuitive interaction through natural language. This technology is essential for creating robots that can seamlessly integrate into human environments and respond to human commands without requiring specialized interfaces.

#### System Architecture Overview

A typical voice-to-action system consists of several interconnected components:

1. **Speech Recognition**: Converting spoken language to text
2. **Natural Language Understanding**: Interpreting the meaning of commands
3. **Action Planning**: Translating understood commands into action sequences
4. **Execution Monitoring**: Ensuring actions are performed correctly
5. **Feedback Generation**: Communicating results back to the user

### Speech Recognition and Processing

#### Automatic Speech Recognition (ASR)

Automatic Speech Recognition is the foundation of voice-to-action systems, converting audio signals into textual representations.

##### OpenAI Whisper Implementation

OpenAI Whisper has become a leading model for speech recognition due to its robustness across different accents and environments:

```python
import whisper
import torch
import numpy as np
import librosa

class SpeechRecognizer:
    def __init__(self, model_size="base"):
        """Initialize Whisper model for speech recognition"""
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = whisper.load_model(model_size).to(self.device)

    def transcribe_audio(self, audio_path, language="en"):
        """Transcribe audio file to text"""
        # Load audio file
        audio = whisper.load_audio(audio_path)
        audio = whisper.pad_or_trim(audio)

        # Convert to log mel spectrogram
        mel = whisper.log_mel_spectrogram(audio).to(self.model.device)

        # Decode the audio
        options = whisper.DecodingOptions(
            language=language,
            without_timestamps=True,
            fp16=False if self.device == "cpu" else True
        )
        result = whisper.decode(self.model, mel, options)

        return result.text

    def transcribe_realtime(self, audio_buffer):
        """Transcribe audio buffer in real-time"""
        # Process audio buffer for real-time transcription
        audio = whisper.pad_or_trim(audio_buffer)
        mel = whisper.log_mel_spectrogram(audio).to(self.model.device)

        options = whisper.DecodingOptions(
            language="en",
            without_timestamps=True,
            fp16=False if self.device == "cpu" else True
        )
        result = whisper.decode(self.model, mel, options)

        return result.text

    def batch_transcribe(self, audio_files):
        """Transcribe multiple audio files efficiently"""
        transcriptions = []
        for audio_file in audio_files:
            transcription = self.transcribe_audio(audio_file)
            transcriptions.append(transcription)
        return transcriptions
```

##### Alternative ASR Systems

While Whisper is powerful, other ASR systems may be appropriate depending on requirements:

```python
# Example using SpeechRecognition library with various backends
import speech_recognition as sr

class AlternativeASR:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

    def recognize_google(self, audio_data):
        """Use Google's speech recognition service"""
        try:
            text = self.recognizer.recognize_google(audio_data)
            return text
        except sr.UnknownValueError:
            return None
        except sr.RequestError as e:
            print(f"Google SR error: {e}")
            return None

    def recognize_sphinx(self, audio_data):
        """Use CMU Sphinx (offline)"""
        try:
            text = self.recognizer.recognize_sphinx(audio_data)
            return text
        except sr.UnknownValueError:
            return None
        except sr.RequestError as e:
            print(f"Sphinx SR error: {e}")
            return None
```

#### Audio Preprocessing

Proper audio preprocessing is crucial for robust speech recognition:

```python
import librosa
import numpy as np
from scipy import signal

class AudioPreprocessor:
    def __init__(self):
        self.sample_rate = 16000  # Standard for speech recognition
        self.frame_length = 2048
        self.hop_length = 512

    def preprocess_audio(self, audio_data, sample_rate):
        """Preprocess audio for optimal speech recognition"""
        # Resample to target rate
        if sample_rate != self.sample_rate:
            audio_data = librosa.resample(
                audio_data,
                orig_sr=sample_rate,
                target_sr=self.sample_rate
            )

        # Apply noise reduction
        audio_data = self.reduce_noise(audio_data)

        # Normalize audio
        audio_data = self.normalize_audio(audio_data)

        # Apply voice activity detection (simplified)
        audio_data = self.remove_silence(audio_data)

        return audio_data

    def reduce_noise(self, audio_data):
        """Apply basic noise reduction"""
        # Use spectral gating for noise reduction
        noise_sample = audio_data[:int(0.1 * self.sample_rate)]  # First 100ms as noise profile
        reduced = self.spectral_gate(audio_data, noise_sample)
        return reduced

    def normalize_audio(self, audio_data):
        """Normalize audio to standard level"""
        # Peak normalization
        max_amplitude = np.max(np.abs(audio_data))
        if max_amplitude > 0:
            normalized = audio_data / max_amplitude
            # Scale to reasonable range
            normalized = normalized * 0.8  # Avoid clipping
        else:
            normalized = audio_data
        return normalized

    def remove_silence(self, audio_data, threshold=0.01):
        """Remove silence from audio"""
        # Calculate energy for each frame
        frames = librosa.util.frame(audio_data, frame_length=512, hop_length=256)
        frame_energy = np.mean(frames**2, axis=0)

        # Identify non-silent frames
        non_silent = frame_energy > threshold

        # Reconstruct audio without silent portions
        # This is a simplified version - real implementation would be more complex
        return audio_data  # Placeholder

    def spectral_gate(self, audio, noise_sample):
        """Apply spectral gating for noise reduction"""
        # Calculate noise spectrum
        noise_spectrum = np.abs(librosa.stft(noise_sample))

        # Calculate audio spectrum
        audio_stft = librosa.stft(audio)
        audio_spectrum = np.abs(audio_stft)

        # Create mask based on noise threshold
        mask = audio_spectrum > (noise_spectrum * 1.5)  # 50% above noise level

        # Apply mask to audio spectrum
        cleaned_spectrum = audio_spectrum * mask
        phase = np.angle(audio_stft)

        # Reconstruct audio
        cleaned_stft = cleaned_spectrum * np.exp(1j * phase)
        cleaned_audio = librosa.istft(cleaned_stft)

        return cleaned_audio
```

### Natural Language Understanding (NLU)

#### Command Interpretation

Natural Language Understanding converts recognized text into structured representations that can be processed by robotic systems:

```python
import spacy
import re
from typing import Dict, List, Tuple

class NaturalLanguageUnderstanding:
    def __init__(self):
        # Load spaCy model for English
        try:
            self.nlp = spacy.load("en_core_web_sm")
        except OSError:
            print("Please install spaCy English model: python -m spacy download en_core_web_sm")
            raise

    def parse_command(self, command_text: str) -> Dict:
        """Parse natural language command into structured format"""
        doc = self.nlp(command_text.lower())

        # Extract command components
        action = self.extract_action(doc)
        objects = self.extract_objects(doc)
        locations = self.extract_locations(doc)
        attributes = self.extract_attributes(doc)

        return {
            'action': action,
            'objects': objects,
            'locations': locations,
            'attributes': attributes,
            'original_text': command_text
        }

    def extract_action(self, doc) -> str:
        """Extract the main action from the command"""
        # Look for verbs that represent actions
        for token in doc:
            if token.pos_ == "VERB":
                # Handle common action verbs
                if token.lemma_ in ["move", "go", "navigate", "walk", "drive"]:
                    return "navigate"
                elif token.lemma_ in ["pick", "grasp", "grab", "take", "lift"]:
                    return "grasp"
                elif token.lemma_ in ["place", "put", "set", "drop", "release"]:
                    return "place"
                elif token.lemma_ in ["clean", "wipe", "dust", "sweep"]:
                    return "clean"
                elif token.lemma_ in ["open", "close", "shut"]:
                    return token.lemma_
                else:
                    return token.lemma_

        return "unknown"

    def extract_objects(self, doc) -> List[str]:
        """Extract objects mentioned in the command"""
        objects = []
        for token in doc:
            if token.pos_ in ["NOUN", "PROPN"] and token.dep_ in ["dobj", "pobj", "attr"]:
                # Handle compound nouns
                compound = self.get_compound_noun(token)
                if compound and compound not in objects:
                    objects.append(compound)

        # Also look for objects in prepositional phrases
        for token in doc:
            if token.dep_ == "pobj":  # Object of preposition
                compound = self.get_compound_noun(token)
                if compound and compound not in objects:
                    objects.append(compound)

        return objects

    def extract_locations(self, doc) -> List[str]:
        """Extract location information from the command"""
        locations = []

        # Look for location-related prepositional phrases
        location_keywords = ["to", "at", "in", "on", "near", "by", "beside", "next to"]

        for token in doc:
            if token.text in location_keywords:
                # Get the object of the preposition
                for child in token.children:
                    if child.pos_ in ["NOUN", "PROPN", "ADP"]:
                        location = self.get_compound_noun(child)
                        if location and location not in locations:
                            locations.append(location)

        # Look for named entities that might be locations
        for ent in doc.ents:
            if ent.label_ in ["GPE", "LOC", "FAC"]:  # Geopolitical entity, Location, Facility
                if ent.text not in locations:
                    locations.append(ent.text.lower())

        return locations

    def extract_attributes(self, doc) -> Dict[str, str]:
        """Extract attributes like colors, sizes, etc."""
        attributes = {}

        for token in doc:
            if token.pos_ == "ADJ":  # Adjectives often represent attributes
                # Look for the noun this adjective modifies
                for child in token.children:
                    if child.pos_ == "NOUN":
                        attributes[child.text] = token.text
                # Or look at what this adjective modifies
                if token.head.pos_ == "NOUN":
                    attributes[token.head.text] = token.text

        return attributes

    def get_compound_noun(self, token) -> str:
        """Get compound noun including determiners and adjectives"""
        # Collect all tokens that are part of the noun phrase
        tokens = [token]

        # Add left children (determiners, adjectives)
        for child in token.lefts:
            if child.dep_ in ["det", "amod", "compound"]:
                tokens.insert(0, child)

        # Add right children (post-modifiers)
        for child in token.rights:
            if child.dep_ in ["prep", "compound", "amod"]:
                tokens.append(child)

        # Sort by position in sentence
        tokens.sort(key=lambda x: x.i)

        return " ".join([t.text for t in tokens])
```

#### Semantic Parsing

Advanced semantic parsing converts natural language to formal representations:

```python
from dataclasses import dataclass
from typing import Optional, List

@dataclass
class SemanticAction:
    """Structured representation of an action"""
    verb: str
    direct_object: Optional[str] = None
    indirect_object: Optional[str] = None
    location: Optional[str] = None
    attributes: Dict[str, str] = None

class SemanticParser:
    def __init__(self):
        self.nlu = NaturalLanguageUnderstanding()
        self.action_templates = self.load_action_templates()

    def parse_to_semantic(self, command_text: str) -> SemanticAction:
        """Parse command to semantic action representation"""
        parsed = self.nlu.parse_command(command_text)

        # Map to semantic action based on command structure
        action = SemanticAction(
            verb=parsed['action'],
            direct_object=parsed['objects'][0] if parsed['objects'] else None,
            location=parsed['locations'][0] if parsed['locations'] else None,
            attributes=parsed['attributes']
        )

        # Apply more sophisticated parsing based on context
        action = self.refine_action(action, parsed)

        return action

    def refine_action(self, action: SemanticAction, parsed: Dict) -> SemanticAction:
        """Refine action based on additional context"""
        # Handle complex commands like "pick up the red cup and place it on the table"
        if action.verb == "grasp" and len(parsed['objects']) > 1:
            action.direct_object = parsed['objects'][0]  # First object to grasp
            # Look for subsequent actions in the command
            if "place" in parsed['original_text'] or "put" in parsed['original_text']:
                # This is a compound action - grasp then place
                action.indirect_object = parsed['objects'][1] if len(parsed['objects']) > 1 else None

        return action

    def load_action_templates(self) -> Dict:
        """Load action templates for common command patterns"""
        return {
            "navigation": [
                r"goto (.+)",
                r"go to (.+)",
                r"move to (.+)",
                r"navigate to (.+)"
            ],
            "grasping": [
                r"pick up (.+)",
                r"grasp (.+)",
                r"take (.+)",
                r"lift (.+)"
            ],
            "placement": [
                r"place (.+) on (.+)",
                r"put (.+) on (.+)",
                r"set (.+) on (.+)"
            ]
        }
```

### Action Planning and Execution

#### Task Decomposition

Complex commands need to be decomposed into executable actions:

```python
class TaskDecomposer:
    def __init__(self, robot_capabilities):
        self.capabilities = robot_capabilities
        self.navigation_system = robot_capabilities['navigation']
        self.manipulation_system = robot_capabilities['manipulation']
        self.perception_system = robot_capabilities['perception']

    def decompose_task(self, semantic_action: SemanticAction) -> List[Dict]:
        """Decompose high-level action into executable steps"""
        task_sequence = []

        if semantic_action.verb == "navigate":
            task_sequence.extend(self.plan_navigation(semantic_action))

        elif semantic_action.verb == "grasp":
            task_sequence.extend(self.plan_grasping(semantic_action))

        elif semantic_action.verb == "place":
            task_sequence.extend(self.plan_placement(semantic_action))

        elif semantic_action.verb == "clean":
            task_sequence.extend(self.plan_cleaning(semantic_action))

        else:
            # Handle compound or complex tasks
            task_sequence.extend(self.handle_complex_task(semantic_action))

        return task_sequence

    def plan_navigation(self, action: SemanticAction) -> List[Dict]:
        """Plan navigation to specified location"""
        tasks = []

        # Find location in environment map
        target_pose = self.find_location_pose(action.location)
        if not target_pose:
            raise ValueError(f"Location '{action.location}' not found in environment")

        # Plan path to location
        path = self.navigation_system.plan_path(target_pose)

        tasks.append({
            'type': 'navigation',
            'target_pose': target_pose,
            'path': path,
            'action': action
        })

        return tasks

    def plan_grasping(self, action: SemanticAction) -> List[Dict]:
        """Plan grasping of specified object"""
        tasks = []

        # Find object in environment
        object_info = self.perception_system.find_object(
            action.direct_object,
            attributes=action.attributes
        )
        if not object_info:
            raise ValueError(f"Object '{action.direct_object}' not found")

        # Plan approach to object
        approach_pose = self.calculate_approach_pose(object_info)

        # Plan grasp configuration
        grasp_config = self.calculate_grasp_configuration(
            object_info,
            action.attributes
        )

        tasks.extend([
            {
                'type': 'navigation',
                'target_pose': approach_pose,
                'action': 'approach_object'
            },
            {
                'type': 'manipulation',
                'action': 'grasp',
                'object_info': object_info,
                'grasp_config': grasp_config
            }
        ])

        return tasks

    def plan_placement(self, action: SemanticAction) -> List[Dict]:
        """Plan placement of object at specified location"""
        tasks = []

        # Find placement location
        target_pose = self.find_location_pose(action.location)
        if not target_pose:
            raise ValueError(f"Placement location '{action.location}' not found")

        tasks.extend([
            {
                'type': 'navigation',
                'target_pose': target_pose,
                'action': 'move_to_placement'
            },
            {
                'type': 'manipulation',
                'action': 'place',
                'target_pose': target_pose
            }
        ])

        return tasks

    def handle_complex_task(self, action: SemanticAction) -> List[Dict]:
        """Handle complex or compound tasks"""
        # For complex tasks, we might need to use LLMs for decomposition
        # This is a simplified example
        if "and" in action.direct_object or "then" in action.direct_object:
            # Split compound command
            return self.handle_compound_command(action)

        # Default: treat as unknown action
        return [{
            'type': 'unknown',
            'action': action,
            'error': f"Unknown action: {action.verb}"
        }]

    def handle_compound_command(self, action: SemanticAction) -> List[Dict]:
        """Handle compound commands like 'pick up X and place it on Y'"""
        # This would require more sophisticated parsing
        # For now, return placeholder
        return []

    def find_location_pose(self, location_name: str) -> Optional[List[float]]:
        """Find pose of named location in environment"""
        # Query environment map for location
        # This would interface with the robot's map system
        locations = {
            "kitchen": [1.0, 2.0, 0.0],
            "living room": [3.0, 1.0, 0.0],
            "bedroom": [5.0, 3.0, 0.0],
            "table": [2.0, 2.0, 0.0],
            "counter": [1.5, 1.5, 0.0]
        }
        return locations.get(location_name.lower())

    def calculate_approach_pose(self, object_info: Dict) -> List[float]:
        """Calculate approach pose for object manipulation"""
        # Calculate safe approach position in front of object
        obj_pose = object_info['pose']
        approach_distance = 0.3  # 30cm from object

        # Simple approach: 30cm in front of object
        approach_pose = obj_pose.copy()
        approach_pose[0] -= approach_distance  # Move back from object

        return approach_pose

    def calculate_grasp_configuration(self, object_info: Dict, attributes: Dict) -> Dict:
        """Calculate appropriate grasp configuration"""
        obj_shape = object_info.get('shape', 'unknown')
        obj_size = object_info.get('size', 'medium')

        # Choose grasp based on object properties
        if obj_shape == 'cylindrical':
            grasp_type = 'cylindrical'
        elif obj_shape == 'spherical':
            grasp_type = 'spherical'
        elif obj_size == 'small':
            grasp_type = 'pinch'
        else:
            grasp_type = 'power'

        return {
            'type': grasp_type,
            'width': object_info.get('width', 0.1),
            'orientation': object_info.get('orientation', [0, 0, 0])
        }
```

### Integration with Robotic Systems

#### ROS 2 Integration

Voice-to-action systems integrate with robotic platforms through ROS 2:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import AudioData
from builtin_interfaces.msg import Duration

class VoiceActionNode(Node):
    def __init__(self):
        super().__init__('voice_action_node')

        # Initialize components
        self.speech_recognizer = SpeechRecognizer()
        self.nlu_system = NaturalLanguageUnderstanding()
        self.task_decomposer = TaskDecomposer(self.get_robot_capabilities())

        # Publishers and subscribers
        self.command_sub = self.create_subscription(
            String, 'voice_commands', self.command_callback, 10
        )

        self.audio_sub = self.create_subscription(
            AudioData, 'microphone/audio', self.audio_callback, 10
        )

        self.status_pub = self.create_publisher(String, 'voice_action_status', 10)

        # Action execution feedback
        self.feedback_pub = self.create_publisher(String, 'action_feedback', 10)

    def command_callback(self, msg):
        """Handle text commands"""
        try:
            # Parse the command
            semantic_action = self.nlu_system.parse_to_semantic(msg.data)

            # Decompose into executable tasks
            task_sequence = self.task_decomposer.decompose_task(semantic_action)

            # Execute tasks
            self.execute_task_sequence(task_sequence)

            # Publish success
            status_msg = String()
            status_msg.data = f"Command '{msg.data}' executed successfully"
            self.status_pub.publish(status_msg)

        except Exception as e:
            error_msg = String()
            error_msg.data = f"Error executing command: {str(e)}"
            self.status_pub.publish(error_msg)

    def audio_callback(self, msg):
        """Handle audio commands"""
        try:
            # Convert audio data to numpy array
            audio_array = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

            # Transcribe audio to text
            transcribed_text = self.speech_recognizer.transcribe_realtime(audio_array)

            if transcribed_text:
                # Process as command
                self.get_logger().info(f"Recognized: {transcribed_text}")

                # Parse and execute
                semantic_action = self.nlu_system.parse_to_semantic(transcribed_text)
                task_sequence = self.task_decomposer.decompose_task(semantic_action)
                self.execute_task_sequence(task_sequence)

                # Provide feedback
                feedback_msg = String()
                feedback_msg.data = f"Heard: {transcribed_text}"
                self.feedback_pub.publish(feedback_msg)

        except Exception as e:
            self.get_logger().error(f"Audio processing error: {str(e)}")

    def execute_task_sequence(self, task_sequence):
        """Execute sequence of tasks"""
        for task in task_sequence:
            self.execute_single_task(task)

    def execute_single_task(self, task):
        """Execute a single task"""
        task_type = task['type']

        if task_type == 'navigation':
            self.execute_navigation_task(task)
        elif task_type == 'manipulation':
            self.execute_manipulation_task(task)
        else:
            self.get_logger().warn(f"Unknown task type: {task_type}")

    def execute_navigation_task(self, task):
        """Execute navigation task"""
        # This would interface with navigation stack
        target_pose = task['target_pose']

        # Publish navigation goal
        # nav_goal_pub.publish(target_pose)
        pass

    def execute_manipulation_task(self, task):
        """Execute manipulation task"""
        action = task['action']

        if action == 'grasp':
            # Execute grasping action
            pass
        elif action == 'place':
            # Execute placement action
            pass

    def get_robot_capabilities(self):
        """Get robot capabilities for task decomposition"""
        return {
            'navigation': self.get_navigation_system(),
            'manipulation': self.get_manipulation_system(),
            'perception': self.get_perception_system()
        }

    def get_navigation_system(self):
        """Get navigation system interface"""
        # Return navigation system interface
        pass

    def get_manipulation_system(self):
        """Get manipulation system interface"""
        # Return manipulation system interface
        pass

    def get_perception_system(self):
        """Get perception system interface"""
        # Return perception system interface
        pass
```

### Advanced Voice Processing

#### Multi-modal Integration

Voice-to-action systems benefit from integration with other modalities:

```python
class MultiModalVoiceSystem:
    def __init__(self, speech_recognizer, vision_system, robot_state):
        self.speech_recognizer = speech_recognizer
        self.vision_system = vision_system
        self.robot_state = robot_state

        # Context managers
        self.spatial_context = SpatialContextManager()
        self.task_context = TaskContextManager()

    def process_multimodal_command(self, audio_data, visual_context=None):
        """Process voice command with visual context"""
        # Transcribe audio
        command_text = self.speech_recognizer.transcribe_realtime(audio_data)

        if not command_text:
            return None

        # Get current visual context
        if visual_context is None:
            visual_context = self.vision_system.get_current_scene()

        # Parse command with context
        contextual_command = self.add_context_to_command(
            command_text, visual_context, self.robot_state
        )

        # Parse with enhanced context
        semantic_action = self.parse_contextual_command(contextual_command)

        return semantic_action

    def add_context_to_command(self, command_text, visual_context, robot_state):
        """Add spatial and contextual information to command"""
        # Resolve ambiguous references using context
        # "Pick up that" -> "Pick up the red cup on the table"

        resolved_command = command_text

        # Replace ambiguous references with specific objects
        for obj_ref in self.find_ambiguous_references(command_text):
            specific_obj = self.resolve_reference(obj_ref, visual_context)
            if specific_obj:
                resolved_command = resolved_command.replace(obj_ref, specific_obj)

        return resolved_command

    def find_ambiguous_references(self, command_text):
        """Find ambiguous references like 'that', 'it', 'there'"""
        ambiguous_refs = []

        # Common ambiguous references
        refs = ["that", "it", "there", "this", "them", "the one", "the thing"]

        for ref in refs:
            if ref in command_text.lower():
                ambiguous_refs.append(ref)

        return ambiguous_refs

    def resolve_reference(self, reference, visual_context):
        """Resolve ambiguous reference using visual context"""
        # Use spatial relationships and attention mechanisms
        if reference.lower() == "that":
            # Find the most salient object in the scene
            salient_objects = self.get_salient_objects(visual_context)
            if salient_objects:
                return salient_objects[0]['name']  # Return most salient object name

        elif reference.lower() == "there":
            # Find location based on robot's attention or pointing direction
            attention_direction = self.get_attention_direction()
            nearby_locations = self.find_nearby_locations(
                self.robot_state['position'], attention_direction
            )
            if nearby_locations:
                return nearby_locations[0]['name']

        return None

    def get_salient_objects(self, visual_context):
        """Get most salient objects in visual scene"""
        # Calculate saliency based on size, color, motion, etc.
        objects = visual_context.get('objects', [])

        # Simple saliency calculation (could be more sophisticated)
        for obj in objects:
            size = obj.get('size', 0)
            color_salience = self.calculate_color_salience(obj.get('color'))
            motion_salience = obj.get('motion', 0)

            obj['saliency'] = size * 0.4 + color_salience * 0.3 + motion_salience * 0.3

        # Sort by saliency
        sorted_objects = sorted(objects, key=lambda x: x['saliency'], reverse=True)
        return sorted_objects

    def calculate_color_salience(self, color):
        """Calculate color-based salience"""
        # Red and other bright colors are more salient
        if color == 'red':
            return 1.0
        elif color in ['blue', 'yellow', 'orange']:
            return 0.8
        else:
            return 0.3

    def get_attention_direction(self):
        """Get robot's current attention direction"""
        # This could come from head orientation, gaze tracking, etc.
        return self.robot_state.get('head_orientation', [0, 0, 1])

    def find_nearby_locations(self, robot_pos, direction):
        """Find locations in the direction of attention"""
        # Find locations in the environment map
        # This would interface with the robot's map system
        return []

    def parse_contextual_command(self, contextual_command):
        """Parse command with resolved context"""
        # Use enhanced NLU with context
        doc = self.nlu_system.nlp(contextual_command.lower())

        # Extract action with context
        action = self.nlu_system.extract_action(doc)
        objects = self.nlu_system.extract_objects(doc)
        locations = self.nlu_system.extract_locations(doc)

        return SemanticAction(
            verb=action,
            direct_object=objects[0] if objects else None,
            location=locations[0] if locations else None
        )
```

### Performance Optimization

#### Real-time Processing

Voice-to-action systems require real-time performance:

```python
import asyncio
import threading
from queue import Queue
import time

class RealTimeVoiceProcessor:
    def __init__(self):
        self.audio_queue = Queue(maxsize=10)
        self.command_queue = Queue(maxsize=5)

        self.is_running = False
        self.processing_thread = None
        self.whisper_model = None

    def start_processing(self):
        """Start real-time audio processing"""
        self.is_running = True
        self.processing_thread = threading.Thread(target=self.process_audio_stream)
        self.processing_thread.start()

    def stop_processing(self):
        """Stop real-time audio processing"""
        self.is_running = False
        if self.processing_thread:
            self.processing_thread.join()

    def process_audio_stream(self):
        """Process audio stream in real-time"""
        # Load model once
        self.whisper_model = whisper.load_model("base").to(
            "cuda" if torch.cuda.is_available() else "cpu"
        )

        while self.is_running:
            try:
                # Get audio chunk from queue
                audio_chunk = self.audio_queue.get(timeout=1.0)

                # Process with Whisper
                transcription = self.transcribe_chunk(audio_chunk)

                if transcription and self.is_meaningful_command(transcription):
                    # Add to command queue for NLU processing
                    self.command_queue.put(transcription)

                self.audio_queue.task_done()

            except Exception as e:
                if "Empty" not in str(e):
                    print(f"Audio processing error: {e}")

    def transcribe_chunk(self, audio_chunk):
        """Transcribe audio chunk using Whisper"""
        try:
            # Convert audio chunk to appropriate format
            audio = whisper.pad_or_trim(audio_chunk)
            mel = whisper.log_mel_spectrogram(audio).to(self.whisper_model.device)

            options = whisper.DecodingOptions(
                language="en",
                without_timestamps=True,
                fp16=False if next(self.whisper_model.parameters()).is_cuda else True
            )

            result = whisper.decode(self.whisper_model, mel, options)
            return result.text
        except Exception as e:
            print(f"Transcription error: {e}")
            return None

    def is_meaningful_command(self, transcription):
        """Check if transcription contains a meaningful command"""
        if not transcription or len(transcription.strip()) < 3:
            return False

        # Filter out common non-command phrases
        non_commands = [
            "um", "uh", "you", "the", "a", "an", "and", "or", "but",
            "what", "is", "are", "how", "why", "where", "when"
        ]

        # Simple check: command should contain action words
        action_words = ["go", "move", "pick", "grasp", "take", "put", "place", "clean", "open", "close"]

        words = transcription.lower().split()
        return any(word in action_words for word in words)

    def get_processed_command(self):
        """Get next processed command"""
        try:
            return self.command_queue.get_nowait()
        except:
            return None
```

### Error Handling and Robustness

#### Handling Uncertainty

Voice-to-action systems must handle uncertainty gracefully:

```python
class RobustVoiceActionSystem:
    def __init__(self):
        self.confidence_threshold = 0.7
        self.recovery_strategies = [
            self.ask_for_clarification,
            self.use_default_action,
            self.request_repeat
        ]

    def process_command_with_robustness(self, command_text, confidence_score):
        """Process command with uncertainty handling"""
        if confidence_score < self.confidence_threshold:
            # Handle low confidence
            return self.handle_uncertain_command(command_text)

        try:
            # Process command normally
            semantic_action = self.parse_command(command_text)
            task_sequence = self.decompose_task(semantic_action)
            return self.execute_task_sequence(task_sequence)

        except Exception as e:
            return self.handle_execution_error(command_text, e)

    def handle_uncertain_command(self, command_text):
        """Handle command with low confidence"""
        # Ask for clarification
        clarification = self.ask_for_clarification(command_text)

        if clarification:
            # Process the clarified command
            return self.process_command_with_robustness(clarification, 1.0)
        else:
            # Use default behavior or report failure
            return self.use_default_action(command_text)

    def ask_for_clarification(self, command_text):
        """Ask user for clarification of ambiguous command"""
        # Generate clarification request
        clarification_request = self.generate_clarification_request(command_text)

        # This would involve speech synthesis and waiting for response
        # For now, return None as placeholder
        print(f"Clarification needed: {clarification_request}")
        return None

    def generate_clarification_request(self, command_text):
        """Generate appropriate clarification request"""
        # Analyze command for ambiguity
        parsed = self.nlu_system.parse_command(command_text)

        if not parsed['objects']:
            return f"I didn't understand which object you want me to {parsed['action']}. Could you please specify?"
        elif not parsed['locations']:
            return f"I didn't understand where you want me to {parsed['action']} {parsed['objects'][0]}. Could you please specify the location?"
        else:
            return f"I'm not sure I understood your command '{command_text}'. Could you please repeat it?"

    def handle_execution_error(self, command_text, error):
        """Handle errors during command execution"""
        error_msg = f"Failed to execute command '{command_text}': {str(error)}"

        # Log error
        self.log_error(error_msg)

        # Attempt recovery
        recovery_success = self.attempt_recovery(error)

        if not recovery_success:
            # Report failure to user
            self.report_failure_to_user(command_text, error)

        return recovery_success

    def attempt_recovery(self, error):
        """Attempt to recover from execution error"""
        # Different recovery strategies based on error type
        if "navigation" in str(error).lower():
            return self.recovery_navigation_error(error)
        elif "manipulation" in str(error).lower():
            return self.recovery_manipulation_error(error)
        else:
            return self.general_recovery(error)

    def recovery_navigation_error(self, error):
        """Recover from navigation errors"""
        # Try alternative path
        # Report obstacle to mapping system
        # Ask user for alternative route
        return False  # Placeholder

    def recovery_manipulation_error(self, error):
        """Recover from manipulation errors"""
        # Try alternative grasp
        # Report object properties to perception system
        # Ask user for assistance
        return False  # Placeholder

    def general_recovery(self, error):
        """General error recovery"""
        # Report error and ask for alternative command
        return False  # Placeholder

    def log_error(self, error_msg):
        """Log error for debugging and improvement"""
        # Log to file or database
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        with open("voice_action_errors.log", "a") as f:
            f.write(f"[{timestamp}] {error_msg}\n")

    def report_failure_to_user(self, command_text, error):
        """Report failure to user"""
        failure_msg = f"Sorry, I couldn't execute '{command_text}'. {str(error)}"
        # This would involve speech synthesis
        print(f"Robot says: {failure_msg}")
```

### Best Practices

#### Design Principles

1. **Incremental Processing**: Process audio in chunks for real-time response
2. **Context Awareness**: Use environmental and task context to disambiguate commands
3. **Graceful Degradation**: Handle failures gracefully without stopping the system
4. **User Feedback**: Provide clear feedback about system state and actions
5. **Privacy Considerations**: Handle audio data appropriately with privacy in mind

#### Testing Strategies

1. **Noise Robustness**: Test in various acoustic environments
2. **Command Diversity**: Test with diverse command types and phrasings
3. **Real-time Performance**: Ensure processing meets real-time requirements
4. **Error Recovery**: Test error handling and recovery mechanisms
5. **User Studies**: Conduct studies with real users for practical evaluation

### Future Developments

#### Emerging Technologies

- **Large Language Models**: Integration with advanced LLMs for better understanding
- **Multimodal Learning**: Joint learning of speech, vision, and action
- **Adaptive Systems**: Systems that learn from user interactions
- **Edge Processing**: Efficient processing on robot hardware
- **Contextual Understanding**: Deeper understanding of situational context

### Conclusion

Voice-to-action systems represent a critical component of natural human-robot interaction, enabling robots to understand and execute spoken commands in human environments. The integration of speech recognition, natural language understanding, and action planning creates a powerful interface for controlling robotic systems. Success in voice-to-action systems requires careful attention to real-time processing, error handling, and contextual understanding. As the technology continues to advance, these systems will become increasingly sophisticated, enabling more natural and intuitive interaction between humans and robots in everyday environments. The combination of improved speech recognition, advanced natural language processing, and robust action execution will continue to push the boundaries of what's possible in human-robot collaboration.