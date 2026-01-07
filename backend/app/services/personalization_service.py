"""
Personalization Service - Adapts content based on user's skill level
"""
from typing import Literal
from ..config import settings
import google.generativeai as genai

# Level definitions
Level = Literal['beginner', 'medium', 'advanced']

# Content adaptation prompts for each level - optimized for speed and structure preservation
LEVEL_PROMPTS = {
    'beginner': """You are an expert content simplifier for beginners. Your task is to rewrite technical content using simple, everyday language while preserving ALL HTML structure.

CRITICAL RULES:
1. Keep EVERY HTML tag exactly as it is: <header>, <h1>, <h2>, <p>, <ul>, <li>, <strong>, etc.
2. Keep ALL attributes: class, id, style - everything stays the same
3. ONLY change the text BETWEEN the tags
4. Do NOT add or remove ANY tags
5. Do NOT change tag order or nesting
6. Preserve all whitespace and line breaks within the HTML structure

VOCABULARY TRANSFORMATIONS (apply to ALL text in the content):
- "algorithm/algorithms" → "step-by-step method/methods"
- "implement/implementing" → "create/creating" or "make/making"
- "framework/frameworks" → "toolset/toolsets"
- "architecture/architectural" → "design/structure"
- "optimize/optimizing" → "make better" or "improve"
- "integrate/integration" → "connect" or "combine"
- "sophisticated" → "advanced" or "smart"
- "paradigm/paradigms" → "approach/approaches"
- "methodology" → "way of doing things"
- "utilize" → "use"
- "facilitate" → "help" or "make easier"
- "synchronous" → "one at a time"
- "asynchronous" → "all at once" or "without waiting"

INPUT HTML:
{content}

OUTPUT (EXACT same HTML structure with ALL text simplified):""",

    'medium': """Return the HTML content exactly as provided, with no modifications.

INPUT HTML:
{content}

OUTPUT:""",

    'advanced': """You are an expert technical writer enhancing content for advanced professionals. Your task is to elevate the technical sophistication while preserving ALL HTML structure.

CRITICAL RULES:
1. Keep EVERY HTML tag exactly as it is: <header>, <h1>, <h2>, <p>, <ul>, <li>, <strong>, etc.
2. Keep ALL attributes: class, id, style - everything stays the same
3. ONLY change the text BETWEEN the tags
4. Do NOT add or remove ANY tags
5. Do NOT change tag order or nesting
6. Preserve all whitespace and line breaks within the HTML structure

VOCABULARY TRANSFORMATIONS (apply to ALL text in the content):
- "simple method/methods" → "algorithm/algorithms" or "algorithmic approach"
- "create/creating" → "implement/implementing" or "instantiate"
- "toolset/toolsets" → "framework/frameworks" or "architectural framework"
- "design/structure" → "architectural paradigm" or "architecture"
- "make better/improve" → "optimize" or "refactor"
- "connect/combine" → "integrate" or "interface"
- "smart/advanced" → "sophisticated" or "intelligent"
- "approach/approaches" → "methodology/methodologies" or "paradigm"
- "way of doing things" → "methodology"
- "use" → "utilize" or "leverage"
- "help/make easier" → "facilitate"
- "one at a time" → "synchronous"
- "all at once/without waiting" → "asynchronous" or "concurrent"

INPUT HTML:
{content}

OUTPUT (EXACT same HTML structure with ALL text enhanced):"""
}


class PersonalizationService:
    """Service for adapting content based on user skill level"""

    def __init__(self):
        """Initialize the personalization service"""
        self._initialize_gemini()

    def _initialize_gemini(self):
        """Initialize Gemini AI for content adaptation"""
        try:
            if not hasattr(settings, 'gemini_api_key') or not settings.gemini_api_key:
                print("Warning: Gemini API key not configured - personalization disabled")
                self.gemini_client = None
                return

            genai.configure(api_key=settings.gemini_api_key)
            model_name = getattr(settings, 'gemini_model', 'gemini-1.5-flash')

            # Create model with system instruction to force modifications
            system_instruction = """You are a content adaptation expert. When given HTML content to adapt:
1. You MUST change the text content - never return it unchanged
2. Keep all HTML tags, attributes, and structure exactly the same
3. Only modify the words and sentences between HTML tags
4. Follow the vocabulary transformation guides strictly"""

            self.model = genai.GenerativeModel(
                model_name,
                system_instruction=system_instruction
            )
            self.gemini_client = self.model
            print(f"[OK] Personalization Service initialized with {model_name}")
        except Exception as e:
            print(f"[WARNING] Failed to initialize Gemini for personalization: {str(e)}")
            self.gemini_client = None

    async def adapt_content(self, content: str, level: Level) -> str:
        """
        Adapt content to the specified skill level

        Args:
            content: Original content to adapt
            level: Target skill level ('beginner', 'medium', 'advanced')

        Returns:
            Adapted content at the specified level
        """
        print(f"[PERSONALIZE] adapt_content called - Level: {level}, Content length: {len(content)}")

        # Validate content
        if not content or not content.strip():
            print("Warning: Empty content provided for personalization")
            return content

        # Medium level should return original content without API call
        # (Frontend should handle this, but we provide fallback)
        if level == 'medium':
            print("[PERSONALIZE] Medium level requested - returning original content")
            return content

        # If Gemini is not available, return original content with fallback
        if self.gemini_client is None:
            print("Warning: Gemini API not configured, returning original content")
            return content

        try:
            # Get the appropriate prompt for the level
            prompt = LEVEL_PROMPTS.get(level)
            if not prompt:
                print(f"Warning: Unknown level '{level}', returning original content")
                return content

            # IMPORTANT: Tell Gemini to make actual changes
            enhanced_prompt = f"""IMPORTANT: You MUST modify the text content. Do NOT return it unchanged.
Even if the content seems appropriate for the level, you MUST rewrite it using the vocabulary guidelines provided.

{prompt.format(content=content)}"""

            print(f"[PERSONALIZE] Calling Gemini with prompt length: {len(enhanced_prompt)}")

            # Generate adapted content
            response = await self._generate_content(enhanced_prompt)

            print(f"[PERSONALIZE] Gemini response length: {len(response) if response else 0}")

            if not response or not response.strip():
                print("Warning: Empty response from Gemini, returning original content")
                return content

            # Check if response is different from original
            if response.strip() == content.strip():
                print("⚠️ WARNING: Gemini returned IDENTICAL content - no changes made!")
                print(f"   Original (first 200 chars): {content[:200]}")
                print(f"   Response (first 200 chars): {response[:200]}")

            return response.strip()
        except Exception as e:
            print(f"Error adapting content: {str(e)}")
            # Return original content on error to prevent breaking the UI
            return content  # Return original on error

    async def _generate_content(self, prompt: str) -> str:
        """
        Generate content using Gemini AI

        Args:
            prompt: The prompt to generate content from

        Returns:
            Generated text
        """
        try:
            print(f"[PERSONALIZE] Generating content with prompt preview: {prompt[:200]}...")

            # Configure generation settings for speed and structure preservation
            generation_config = {
                "temperature": 0.5,  # Lower for faster, more predictable output
                "top_p": 0.9,
                "top_k": 40,
                "max_output_tokens": 8192,  # Increased to handle full page content
                "candidate_count": 1,  # Only generate one candidate
            }
            response = self.model.generate_content(
                prompt,
                generation_config=generation_config
            )

            # Check if response was blocked by safety filters
            if not response.candidates:
                print("[PERSONALIZE ERROR] ⚠️ Response blocked by Gemini safety filters")
                print(f"   Prompt feedback: {response.prompt_feedback}")
                raise Exception("Content blocked by safety filters - try simpler content")

            result = response.text.strip()
            print(f"[PERSONALIZE] Generated content preview: {result[:200] if result else 'EMPTY'}...")

            return result
        except Exception as e:
            error_msg = str(e)
            print(f"[PERSONALIZE ERROR] Gemini generation failed: {error_msg}")

            # Check for specific error types
            if "429" in error_msg or "quota" in error_msg.lower():
                print("[PERSONALIZE ERROR] ⚠️ QUOTA EXCEEDED - Please wait for quota to reset")
            elif "404" in error_msg or "not found" in error_msg.lower():
                print("[PERSONALIZE ERROR] ⚠️ MODEL NOT FOUND - Check GEMINI_MODEL in .env")

            raise Exception(f"Gemini generation failed: {error_msg}")

    def get_level_description(self, level: Level) -> str:
        """
        Get a human-readable description of a skill level

        Args:
            level: Skill level

        Returns:
            Description string
        """
        descriptions = {
            'beginner': "Simple explanations with step-by-step examples",
            'medium': "Moderate technical details with balanced content",
            'advanced': "Deep technical content with optimization strategies"
        }
        return descriptions.get(level, "Medium level")


# Singleton instance
personalization_service = PersonalizationService()
