from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
from .config import settings
from .services.retrieval_service import RetrievalService
from .database import create_all_tables
from fastapi.middleware.cors import CORSMiddleware
import re

# Import auth routes
from .auth.routes import router as auth_router

app = FastAPI(
    title="RAG Chatbot API - Physical AI TextBook ",
    description="API for the Integrated RAG Chatbot for Physical AI & Humanoid Robotics Textbook",
    version="1.0.0"
)

# Add CORS middleware to allow requests from frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://localhost:3001",
        "http://127.0.0.1:3000",
        "http://127.0.0.1:3001",
        "https://hackaton-ai-textbook.vercel.app",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include auth routes
app.include_router(auth_router, prefix="/api")

# Initialize services
retrieval_service = RetrievalService()

# Create all database tables on startup
@app.on_event("startup")
def startup_event():
    create_all_tables()
    # Create uploads directory if it doesn't exist
    import os
    os.makedirs("uploads", exist_ok=True)
    os.makedirs("uploads/profile-pictures", exist_ok=True)

# Note: Removed Google Gemini initialization to avoid API key issues

class QueryRequest(BaseModel):
    query: str
    top_k: int = 5

class QueryResponse(BaseModel):
    query: str
    answer: str
    sources: List[str]


class TranslationRequest(BaseModel):
    text: str
    target_language: str = "ur"  # Default to Urdu


class TranslationResponse(BaseModel):
    original_text: str
    translated_text: str
    target_language: str

def clean_text_content(text):
    """
    Clean the retrieved text content to filter out technical documentation
    and format it more like textbook content
    """
    # Remove common technical patterns like bash commands, file paths, task lists, etc.
    lines = text.split('\n')
    filtered_lines = []

    for line in lines:
        # Skip lines that look like bash commands
        if any(skip_pattern in line.lower() for skip_pattern in [
            'bash #', 'grep -r', 'find . -name', 'npm run', 'cd ', 'ls ', 'mkdir ',
            'rm ', 'cp ', 'mv ', 'git ', '.js', '.jsx', '.tsx', '.css', '.scss',
            'include=', 'name="', 'function', 'class ', 'def ', 'import ',
            'from ', 'export ', 'const ', 'let ', 'var ', 'json format',
            'curl -x', 'uvicorn', 'post /', 'http:', 'localhost', 'port ',
            'todo:', '[ ] ', '[x] ', 't\d', 'feature:', 'created:', 'status:', 'dependencies',
            'user stories', 'user story', 'tasks', 'phase', 'goal', 'implementation',
            'sp.', 'specs/', '└──', '├──', '│', '├──', '└──'
        ]):
            continue

        # Skip lines with common technical patterns
        if re.search(r'(bash #|grep -r|find \. -name|\.js\" --include=|\.jsx\" --include=|\.tsx\" --include=|\.css\" --include=|\.scss\" --include=|\[ \]|\[x\]|t\d+)', line.lower()):
            continue

        # Skip lines with technical documentation markers
        if re.search(r'(search for|locate|document which|remove|test that|verify|functionality|requirements|success criteria|functional requirements|branch:|date:|spec:|language/version:|primary dependencies:|testing:|performance goals:|constraints:|scale/scope:)', line.lower()):
            continue

        # Skip file structure representations
        if re.search(r'(\*.md|frontend/|backend/|docs/|src/)', line) and ('├──' in line or '└──' in line):
            continue

        # Skip technical requirement lines
        if re.search(r'(- fr-\d+|fr-\d+:|functional requirements|key entities|system must)', line.lower()):
            continue

        # Skip metadata lines
        if re.search(r'(sidebar_position:|markdown_title:|title:|description:|tags:|keywords:)', line.lower()):
            continue

        # Filter lines to keep educational content
        if line.strip() and len(line.strip()) > 15:  # Ensure it's not a short header
            # Check if the line has characteristics of textbook content
            has_sentence_start = any(cap in line for cap in ['The ', 'This ', 'A ', 'An ', 'These ', 'Those ', 'In ', 'For ', 'With ', 'When ', 'During ', 'Physical ', 'Robotics ', 'AI ', 'Chapter ', 'Section ', 'Introduction ', 'Overview', 'Introduction:', 'Embodied ', 'sensors,', 'robotics', 'Docusaurus', 'NVIDIA Isaac', 'Gazebo', 'Unity', 'Humanoid', 'VLA', 'Conversational'])

            # Allow lines that start with common sentence starters or contain textbook content
            if has_sentence_start or not any(skip_word in line.lower() for skip_word in ['system must', 'goal', 'tasks', 'dependencies', 'implementation', 'phase', 'user story', 'created', 'status', 'feature']):
                filtered_lines.append(line)

    # Join the filtered lines
    cleaned_text = '\n'.join(filtered_lines)

    # Remove excessive newlines
    cleaned_text = re.sub(r'\n\s*\n\s*\n', '\n\n', cleaned_text)

    return cleaned_text.strip()

def format_response(text):
    """
    Format the response to make it cleaner and more readable
    """
    # Remove metadata and format markers
    text = re.sub(r'sidebar_position:\s*\d+', '', text)
    text = re.sub(r'Chapter \d+:[^\n]*', '', text)  # Remove chapter headers
    text = re.sub(r'#.*?\n', '', text)  # Remove markdown headers
    text = re.sub(r'##.*?\n', '', text)  # Remove subheaders
    text = re.sub(r'- \[[^\]]*\]\([^\)]*\)', '', text)  # Remove markdown links
    text = re.sub(r'\[(.*?)\]\(.*?\)', r'\1', text)  # Convert markdown links to plain text
    text = re.sub(r'_([^_]+)_', r'\1', text)  # Remove emphasis
    text = re.sub(r'\*\*([^*]+)\*\*', r'\1', text)  # Remove bold formatting

    # Remove emoji symbols and special markers (like emojis)
    text = re.sub(r'[^\x00-\x7F]', ' ', text)  # Keep only ASCII characters

    # Remove bullet points and lists to make it more conversational
    text = re.sub(r'^\s*[-*+]\s+', '', text, flags=re.MULTILINE)
    text = re.sub(r'^\s*\d+\.\s+', '', text, flags=re.MULTILINE)

    # Clean up extra whitespace
    text = re.sub(r'\n\s*\n', '\n', text)  # Reduce multiple blank lines
    text = re.sub(r'^\s+|\s+$', '', text, flags=re.MULTILINE)  # Trim lines
    text = re.sub(r'\s+', ' ', text)  # Reduce multiple spaces to single space

    # Remove the learning objectives section entirely by finding the "What is Physical AI?" part and keeping that and everything after
    # First, extract the part that starts after "What is Physical AI?" if it exists
    if "What is Physical AI?" in text:
        # Split the text at "What is Physical AI?" and keep that and everything after
        text_parts = text.split("What is Physical AI?", 1)  # Split only on the first occurrence
        if len(text_parts) > 1:
            # Keep "What is Physical AI?" and the content after it
            text = "What is Physical AI?" + text_parts[1]

    # Remove any leftover learning objectives markers
    text = re.sub(r'🎯 Learning Objectives\s*\n?', '', text)
    text = re.sub(r'-\s*Understand the fundamental principles.*\n?', '', text)
    text = re.sub(r'-\s*Identify and classify.*\n?', '', text)
    text = re.sub(r'-\s*Recognize the computational.*\n?', '', text)
    text = re.sub(r'-\s*Appreciate the distinction.*\n?', '', text)
    text = re.sub(r'-\s*Explore the landscape.*\n?', '', text)

    # Clean up any remaining unwanted content
    text = text.replace('🎯 Learning Objectives', '').strip()
    text = text.replace('Learning Objectives', '').strip()

    # Remove empty lines
    lines = [line for line in text.split('\n') if line.strip()]
    text = '\n'.join(lines).strip()

    # If the text is too long, find a good cut-off point - significantly increased for more complete content
    if len(text) > 1500:  # Increased length even more to provide extensive content before truncating
        # Try to find a good sentence boundary
        sentences = text.split('. ')
        result = ""
        for sentence in sentences:
            if sentence.strip():  # Only add non-empty sentences
                if len(result + sentence + ". ") <= 1500:
                    result += sentence + ". "
                else:
                    result = result.rstrip() + "..."
                    break
        if result.strip():  # Check if result is not just empty or whitespace
            text = result.strip()
        else:
            text = text[:1500] + "..."
    # If text is still too long for some reason, truncate
    elif len(text) > 1500:
        text = text[:1500] + "..."

    return text.strip()

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running"}

@app.post("/query", response_model=QueryResponse)
def query_endpoint(request: QueryRequest):
    """Query the RAG system to get answers based on textbook content"""
    try:
        # Retrieve relevant documents
        sources_with_urls = retrieval_service.retrieve(request.query, request.top_k)
        sources = [source[0] for source in sources_with_urls]  # Extract just the text content

        if not sources:
            return QueryResponse(
                query=request.query,
                answer="I couldn't find any relevant information in the textbook to answer your question.",
                sources=[]
            )

        # Clean and find the most relevant source that contains actual textbook content
        filtered_sources = []
        for source in sources:
            cleaned_source = clean_text_content(source)
            if cleaned_source and len(cleaned_source.strip()) > 20:  # Ensure it has content
                filtered_sources.append(cleaned_source)

        if not filtered_sources:
            return QueryResponse(
                query=request.query,
                answer="I found information related to your query, but it appears to be technical documentation rather than textbook content. Please try rephrasing your question.",
                sources=sources
            )

        # Use the first filtered source that contains actual content
        main_content = filtered_sources[0]

        # Format the response to be clean and readable
        formatted_content = format_response(main_content)

        return QueryResponse(
            query=request.query,
            answer=formatted_content,
            sources=sources
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.get("/health")
def health_check():
    """Check if the API is running"""
    return {"status": "healthy", "service": "RAG Chatbot API"}

@app.get("/health/qdrant")
def qdrant_health():
    """Check Qdrant connectivity"""
    try:
        # Try to list collections to verify connectivity
        collections = retrieval_service.qdrant_client.get_collections()
        return {"status": "healthy", "service": "Qdrant"}
    except Exception as e:
        return {"status": "unhealthy", "service": "Qdrant", "error": str(e)}

@app.get("/health/gemini")
def gemini_health():
    """Check Google Gemini connectivity"""
    # Since we removed Gemini API to avoid key issues, this will always show unhealthy
    return {"status": "disabled", "service": "Google Gemini", "info": "Feature disabled to avoid API key issues"}


def is_urdu_text(text: str) -> bool:
    """
    Detect if the text is already in Urdu by checking for Arabic/Urdu Unicode characters
    Urdu characters are in the range U+0600 to U+06FF
    """
    if not text:
        return False

    # Count characters in Arabic/Urdu Unicode range
    arabic_urdu_chars = 0
    total_chars = 0

    for char in text:
        # Check if character is in Arabic/Urdu Unicode block
        if '\u0600' <= char <= '\u06FF':
            arabic_urdu_chars += 1
        # Count only alphabetic characters (excluding spaces and punctuation)
        if char.isalpha() and not ('\u0600' <= char <= '\u06FF'):
            total_chars += 1

    # If there are no Arabic/Urdu characters, it's not Urdu
    if arabic_urdu_chars == 0:
        return False

    # If there are Arabic/Urdu characters and the text is mostly non-English alphabetic chars
    # or if Arabic/Urdu characters make up a significant portion
    total_text_chars = arabic_urdu_chars + total_chars
    if total_text_chars == 0:
        return False

    # If more than 30% of all characters are Arabic/Urdu, consider it Urdu text
    # This handles mixed text better - if there's significant Urdu content, treat as Urdu
    return (arabic_urdu_chars / total_text_chars) > 0.3

def translate_with_google_services(text: str, target_language: str = "ur") -> str:
    """
    Translate text using Google Translation services (Gemini or Translate API)
    """
    # First try Gemini if available
    try:
        # Check if Gemini API is configured
        if hasattr(settings, 'gemini_api_key') and settings.gemini_api_key:
            # Import Google Generative AI (if available)
            import google.generativeai as genai

            # Configure the API
            genai.configure(api_key=settings.gemini_api_key)

            # Select the model from settings or use default
            model_name = getattr(settings, 'gemini_model', 'gemini-1.5-flash')
            model = genai.GenerativeModel(model_name)

            # Create the translation prompt based on source and target languages
            if target_language.lower() == "ur":
                prompt = f"""Translate the following text to Urdu.
IMPORTANT: Return ONLY the Urdu translation, nothing else. No explanations, no English text, no prefixes.

Text to translate:
{text}

Translation:"""
            elif target_language.lower() == "en":
                prompt = f"""Translate the following text to English.
IMPORTANT: Return ONLY the English translation, nothing else. No explanations, no Urdu text, no prefixes.

Text to translate:
{text}

Translation:"""
            else:
                # Default to other languages
                prompt = f"""Translate the following text to {target_language}.
IMPORTANT: Return ONLY the translation, nothing else.

Text to translate:
{text}

Translation:"""

            # Generate content
            response = model.generate_content(prompt)

            # Extract the translated text and clean it
            translated_text = response.text.strip()

            # Remove common prefixes that Gemini might add
            prefixes_to_remove = [
                "Translation:", "ترجمہ:", "Urdu:", "اردو:",
                "Here is the translation:", "یہاں ترجمہ ہے:",
                "The translation is:", "ترجمہ یہ ہے:"
            ]

            for prefix in prefixes_to_remove:
                if translated_text.startswith(prefix):
                    translated_text = translated_text[len(prefix):].strip()

            # Remove quotes if present
            if translated_text.startswith('"') and translated_text.endswith('"'):
                translated_text = translated_text[1:-1]
            if translated_text.startswith("'") and translated_text.endswith("'"):
                translated_text = translated_text[1:-1]

            return translated_text
    except ImportError as e:
        print(f"ImportError: Google Generative AI library not installed: {str(e)}")
    except Exception as e:
        print(f"Error calling Gemini API: {str(e)}")

    # If Gemini fails, try deep-translator as fallback
    try:
        from deep_translator import GoogleTranslator
        target_lang = 'ur' if target_language == 'ur' else 'en'
        translator = GoogleTranslator(source='auto', target=target_lang)
        result = translator.translate(text)
        return result if result else text
    except ImportError:
        print("deep-translator library not installed, using fallback")
        # If deep-translator is not installed, try googletrans as another fallback
        try:
            from googletrans import Translator
            translator = Translator()
            result = translator.translate(text, dest=target_language)
            return result.text
        except ImportError:
            print("googletrans library not installed, using fallback")
        except Exception as e:
            print(f"Error calling Google Translate API: {str(e)}")
    except Exception as e:
        print(f"Error calling deep-translator: {str(e)}")

    # If all services fail, return the original text to avoid placeholder messages
    return text

@app.post("/translate", response_model=TranslationResponse)
def translate_endpoint(request: TranslationRequest):
    """Translate text to the target language with caching and Urdu detection"""
    try:
        # First, check if the text is already in Urdu
        # Only skip translation if target is Urdu AND text is already Urdu
        is_currently_urdu = is_urdu_text(request.text)
        if is_currently_urdu and request.target_language == "ur":
            # If text is already in Urdu and we want Urdu, return as-is
            return TranslationResponse(
                original_text=request.text,
                translated_text=request.text,
                target_language=request.target_language
            )

        # Check Qdrant metadata for existing cached translation
        cached_translation = retrieval_service.check_translation_cache(request.text, request.target_language)

        if cached_translation:
            # Return cached translation if found
            return TranslationResponse(
                original_text=request.text,
                translated_text=cached_translation,
                target_language=request.target_language
            )

        # If not cached, translate using Google services (Gemini or Translate API)
        translated_text = translate_with_google_services(request.text, request.target_language)

        # Store the new translation in Qdrant metadata for future use
        retrieval_service.store_translation_cache(request.text, translated_text, request.target_language)

        # Return the translation response
        return TranslationResponse(
            original_text=request.text,
            translated_text=translated_text,
            target_language=request.target_language
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error translating text: {str(e)}")


# Page Translation Request/Response Models (for full page translation)
class PageTranslationRequest(BaseModel):
    texts: List[str]  # List of text segments to translate
    target_language: str = "ur"  # Default to Urdu


class PageTranslationResponse(BaseModel):
    translated_texts: List[str]
    target_language: str


import asyncio
from concurrent.futures import ThreadPoolExecutor

# Thread pool for parallel translations
_translator_pool = ThreadPoolExecutor(max_workers=10)


def _translate_single(text: str, target_lang: str) -> str:
    """Translate single text"""
    try:
        from deep_translator import GoogleTranslator
        translator = GoogleTranslator(source='auto', target=target_lang)
        result = translator.translate(text)
        return result if result else text
    except:
        return text


async def translate_batch_google_async(texts: List[str], target_language: str = "ur") -> List[str]:
    """
    Translate texts in parallel using thread pool
    """
    target_lang = 'ur' if target_language == 'ur' else 'en'

    loop = asyncio.get_event_loop()
    tasks = [
        loop.run_in_executor(_translator_pool, _translate_single, text, target_lang)
        for text in texts if text and text.strip()
    ]

    results = await asyncio.gather(*tasks)
    return list(results)


@app.post("/translate-page", response_model=PageTranslationResponse)
async def translate_page_endpoint(request: PageTranslationRequest):
    """Translate multiple text segments for full page translation using Google Translate"""
    try:
        # Filter texts that need translation
        texts_to_translate = []
        indices_to_translate = []
        result = list(request.texts)  # Copy original

        for i, text in enumerate(request.texts):
            if not text or not text.strip():
                continue
            # Check if we should skip translation based on target language and current text language
            is_currently_urdu = is_urdu_text(text)
            if request.target_language == "ur" and is_currently_urdu:
                # Skip if target is Urdu and text is already Urdu
                continue
            elif request.target_language == "en" and not is_currently_urdu:
                # Skip if target is English and text is already English
                continue
            texts_to_translate.append(text)
            indices_to_translate.append(i)

        # Parallel translate
        if texts_to_translate:
            translated = await translate_batch_google_async(texts_to_translate, request.target_language)
            for i, idx in enumerate(indices_to_translate):
                if i < len(translated):
                    result[idx] = translated[i]

        return PageTranslationResponse(
            translated_texts=result,
            target_language=request.target_language
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error translating page: {str(e)}")


# Personalization Request/Response Models
class PersonalizationRequest(BaseModel):
    content: str
    level: str  # 'beginner', 'medium', 'advanced'

    class Config:
        # Allow arbitrary Unicode characters and emojis
        str_strip_whitespace = False
        validate_assignment = True


class PersonalizationResponse(BaseModel):
    original_content: str
    personalized_content: str
    level: str

    class Config:
        # Allow arbitrary Unicode characters and emojis
        str_strip_whitespace = False


@app.get("/personalize/health")
async def personalize_health_check():
    """Check if personalization service and Gemini API are working"""
    from .services.personalization_service import personalization_service

    if personalization_service.gemini_client is None:
        return {
            "status": "error",
            "message": "Gemini client not initialized",
            "gemini_configured": False
        }

    # Try a simple test
    try:
        import google.generativeai as genai
        test_response = await personalization_service._generate_content("Say 'OK'")
        return {
            "status": "success",
            "message": "Gemini API is working",
            "gemini_configured": True,
            "test_response": test_response[:50]
        }
    except Exception as e:
        error_msg = str(e)
        if "429" in error_msg or "quota" in error_msg.lower():
            return {
                "status": "quota_exceeded",
                "message": "Gemini API quota exceeded. Please wait for reset.",
                "gemini_configured": True,
                "error": error_msg[:200]
            }
        else:
            return {
                "status": "error",
                "message": "Gemini API error",
                "gemini_configured": True,
                "error": error_msg[:200]
            }


@app.post("/personalize", response_model=PersonalizationResponse)
async def personalize_content(request: PersonalizationRequest):
    """
    Adapt content to user's preferred skill level

    Args:
        content: Original content to adapt
        level: Target skill level ('beginner', 'intermediate', 'advanced')

    Returns:
        Personalized content adapted to specified level
    """
    try:
        from .services.personalization_service import personalization_service

        # Validate level
        valid_levels = ['beginner', 'medium', 'advanced']
        if request.level not in valid_levels:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid level. Must be one of: {', '.join(valid_levels)}"
            )

        # Log the request
        print(f"[PERSONALIZE] Request received - Level: {request.level}, Content length: {len(request.content)}")

        # Adapt content to specified level
        personalized = await personalization_service.adapt_content(
            content=request.content,
            level=request.level
        )

        print(f"[PERSONALIZE] Personalization complete")

        return PersonalizationResponse(
            original_content=request.content,
            personalized_content=personalized,
            level=request.level
        )
    except Exception as e:
        import traceback
        print(f"[PERSONALIZE ERROR] {str(e)}")
        print(traceback.format_exc())
        raise HTTPException(status_code=500, detail=f"Error personalizing content: {str(e)}")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)