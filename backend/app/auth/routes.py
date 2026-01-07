from fastapi import APIRouter, Depends, HTTPException, status, BackgroundTasks, UploadFile, File, Request
from fastapi.responses import RedirectResponse
from fastapi.security import HTTPBearer
from sqlalchemy.orm import Session
from typing import Optional
import os
from uuid import uuid4
import shutil
from ..database import get_db
from ..config import settings
from .models import UserCreate, UserLogin, Token, RefreshTokenRequest, SignoutRequest, UserUpdate, AuthSuccess, UserProfile
from .auth_service import AuthService
from .jwt_service import JWTService


router = APIRouter(prefix="/auth", tags=["Authentication"])
security = HTTPBearer()


def get_current_user(token: str = Depends(security), db: Session = Depends(get_db)):
    """Get current user from token"""
    payload = JWTService.verify_token(token.credentials)
    user_id: str = payload.get("user_id")  # UUID as string from JWT
    email: str = payload.get("email")

    if user_id is None or email is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Import User model properly
    from .database import User
    user = db.query(User).filter(User.id == user_id).first()
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return {"user_id": user_id, "email": email}


@router.post("/signup", response_model=Token)
async def signup(user_data: UserCreate, db: Session = Depends(get_db)):
    """Register a new user"""
    user = AuthService.create_user(db, user_data)
    tokens = AuthService.create_tokens(db, user.id, user.email)
    return tokens


@router.post("/signin", response_model=Token)
async def signin(user_data: UserLogin, db: Session = Depends(get_db)):
    """Authenticate user and return tokens"""
    user = AuthService.authenticate_user(db, user_data.email, user_data.password)
    tokens = AuthService.create_tokens(db, user.id, user.email)
    return tokens


@router.post("/refresh")
async def refresh_token(refresh_request: RefreshTokenRequest, db: Session = Depends(get_db)):
    """Refresh access token using refresh token"""
    tokens = AuthService.refresh_access_token(db, refresh_request.refresh_token)
    return tokens


@router.post("/signout")
async def signout(signout_request: SignoutRequest, current_user: dict = Depends(get_current_user), db: Session = Depends(get_db)):
    """Logout user by invalidating refresh token"""
    AuthService.logout(db, current_user["user_id"], signout_request.refresh_token)
    return {"message": "Successfully logged out"}


@router.post("/signout-all")
async def signout_all(current_user: dict = Depends(get_current_user), db: Session = Depends(get_db)):
    """Logout user from all devices by invalidating all refresh tokens"""
    AuthService.logout(db, current_user["user_id"])
    return {"message": "Successfully logged out from all devices"}


@router.get("/profile")
async def get_profile(current_user: dict = Depends(get_current_user), db: Session = Depends(get_db)):
    """Get current user's profile information"""
    profile = AuthService.get_user_profile(db, current_user["user_id"])
    return profile


@router.post("/profile")
async def update_profile(profile_data: UserUpdate, current_user: dict = Depends(get_current_user), db: Session = Depends(get_db)):
    """Update current user's profile information"""
    updated_profile = AuthService.update_user_profile(db, current_user["user_id"], profile_data)
    return updated_profile


@router.post("/profile/picture")
async def update_profile_picture(file: UploadFile = File(...), current_user: dict = Depends(get_current_user), db: Session = Depends(get_db)):
    """Update user's profile picture"""
    # Validate file type
    allowed_extensions = {".jpg", ".jpeg", ".png", ".gif"}
    file_extension = os.path.splitext(file.filename)[1].lower()

    if file_extension not in allowed_extensions:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid file type. Only JPG, PNG, and GIF files are allowed."
        )

    # Generate unique filename
    unique_filename = f"{current_user['user_id']}_{uuid4()}{file_extension}"
    file_path = os.path.join("uploads", "profile-pictures", unique_filename)

    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(file_path), exist_ok=True)

    # Save file
    with open(file_path, "wb") as buffer:
        shutil.copyfileobj(file.file, buffer)

    # Update user profile with file path
    profile_data = UserUpdate(profile_picture=file_path)
    updated_profile = AuthService.update_user_profile(db, current_user["user_id"], profile_data)

    return updated_profile


import os
import secrets

# Note: Google OAuth imports removed to avoid conflicts with FastAPI
# We use httpx for OAuth requests instead


# Google OAuth endpoints
@router.get("/oauth/google/url")
async def get_google_auth_url():
    """Get Google OAuth URL for frontend"""
    import logging
    import urllib.parse

    logger = logging.getLogger(__name__)

    # Use settings instead of os.getenv for proper .env loading
    client_id = settings.google_client_id
    if not client_id:
        logger.error("GOOGLE_CLIENT_ID not configured")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Google OAuth is not configured. Please set GOOGLE_CLIENT_ID in .env"
        )

    # Use configured redirect URI from settings
    redirect_uri = settings.google_redirect_uri
    if not redirect_uri:
        # Fallback if not set in settings - use the port from settings
        redirect_uri = f"http://localhost:{settings.port}/api/auth/oauth/google/callback"

    logger.info(f"Google OAuth configuration - Client ID: {client_id[:10]}..., Redirect URI: {redirect_uri}")

    # Build OAuth URL properly
    import urllib.parse
    # Only encode scope (spaces become +), not redirect_uri
    scope_encoded = urllib.parse.quote_plus("openid email profile")
    auth_url = (
        f"https://accounts.google.com/o/oauth2/auth?"
        f"client_id={client_id}&"
        f"redirect_uri={redirect_uri}&"
        f"response_type=code&"
        f"scope={scope_encoded}&"
        f"access_type=offline&"
        f"prompt=consent"
    )

    logger.info(f"Generated OAuth URL: {auth_url[:100]}...")
    return {"auth_url": auth_url, "redirect_uri": redirect_uri}


# GET OAuth callback (Server-side flow)
@router.get("/oauth/google/callback")
async def google_oauth_callback_get(code: str, state: Optional[str] = None, error: Optional[str] = None, db: Session = Depends(get_db)):
    """
    Handle Google OAuth callback via GET redirect (Server-side flow).
    Exchanges code for tokens, creates/updates user, and redirects to frontend with tokens.
    """
    import logging
    import httpx
    import urllib.parse
    logger = logging.getLogger(__name__)

    frontend_url = settings.frontend_url or "http://localhost:3000"
    # Use the actual backend URL from settings to ensure consistency
    redirect_uri = settings.google_redirect_uri or f"{settings.backend_url}/api/auth/oauth/google/callback"

    logger.info(f"[OAUTH GET] Callback received - Code: {code[:10] if code else 'None'}")
    logger.info(f"[OAUTH GET] Using redirect_uri: {redirect_uri}")

    if error:
        logger.error(f"[OAUTH GET] Error from Google: {error}")
        return RedirectResponse(f"{frontend_url}/auth-callback?error={error}")

    if not code:
        logger.error("[OAUTH GET] No code provided")
        return RedirectResponse(f"{frontend_url}/auth-callback?error=no_code_provided")

    try:
        client_id = settings.google_client_id
        client_secret = settings.google_client_secret

        if not client_id or not client_secret:
            logger.error("[OAUTH GET] Missing client credentials")
            return RedirectResponse(f"{frontend_url}/auth-callback?error=server_configuration_error")

        # Exchange authorization code for tokens
        token_url = "https://oauth2.googleapis.com/token"
        token_data = {
            "code": code,
            "client_id": client_id,
            "client_secret": client_secret,
            "redirect_uri": redirect_uri,
            "grant_type": "authorization_code"
        }

        logger.info(f"[OAUTH GET] Exchanging code for tokens with redirect_uri: {redirect_uri}")

        try:
            async with httpx.AsyncClient(timeout=30.0) as client:
                token_response = await client.post(token_url, data=token_data)

                if token_response.status_code != 200:
                    error_detail = token_response.text
                    logger.error(f"[OAUTH GET] Token exchange failed: {error_detail}")
                    return RedirectResponse(f"{frontend_url}/auth-callback?error=token_exchange_failed&error_desc={token_response.status_code}")
        except httpx.RequestError as e:
            logger.error(f"[OAUTH GET] Network error during token exchange: {str(e)}")
            return RedirectResponse(f"{frontend_url}/auth-callback?error=network_error")
        except Exception as e:
            logger.error(f"[OAUTH GET] Unexpected error during token exchange: {str(e)}")
            return RedirectResponse(f"{frontend_url}/auth-callback?error=token_exchange_error")

            token_json = token_response.json()
            google_access_token = token_json.get("access_token")

            if not google_access_token:
                logger.error("[OAUTH GET] No access token in response")
                return RedirectResponse(f"{frontend_url}/auth-callback?error=no_access_token")

            # Get user info
            userinfo_url = "https://www.googleapis.com/oauth2/v2/userinfo"
            headers = {"Authorization": f"Bearer {google_access_token}"}

            try:
                userinfo_response = await client.get(userinfo_url, headers=headers)

                if userinfo_response.status_code != 200:
                    logger.error(f"[OAUTH GET] User info failed: {userinfo_response.text}")
                    return RedirectResponse(f"{frontend_url}/auth-callback?error=user_info_failed&error_desc={userinfo_response.status_code}")
            except httpx.RequestError as e:
                logger.error(f"[OAUTH GET] Network error during user info retrieval: {str(e)}")
                return RedirectResponse(f"{frontend_url}/auth-callback?error=network_error")
            except Exception as e:
                logger.error(f"[OAUTH GET] Unexpected error during user info retrieval: {str(e)}")
                return RedirectResponse(f"{frontend_url}/auth-callback?error=user_info_error")

            user_info = userinfo_response.json()
            logger.info(f"[OAUTH GET] User info received: {user_info}")

            email = user_info.get("email")
            name = user_info.get("name", email.split("@")[0] if email else "Unknown")
            picture = user_info.get("picture")

            if not email:
                logger.error("[OAUTH GET] No email in user info")
                return RedirectResponse(f"{frontend_url}/auth-callback?error=no_email_provided")

            # Process User (Create or Update)
            from .database import User
            user = db.query(User).filter(User.email == email).first()

            if user:
                logger.info(f"[OAUTH GET] Existing user found: {user.email}")
                # Update picture if needed
                if picture and user.image != picture:
                    user.image = picture
                    try:
                        db.commit()
                        logger.info("[OAUTH GET] Updated user profile picture")
                    except Exception as e:
                        db.rollback()
                        logger.error(f"[OAUTH GET] DB Error updating picture: {e}")
                        return RedirectResponse(f"{frontend_url}/auth-callback?error=account_update_failed")
            else:
                # Create new user
                logger.info(f"[OAUTH GET] Creating new user for: {email}")
                try:
                    user_data = UserCreate(
                        email=email,
                        name=name,
                        password=secrets.token_urlsafe(32)
                    )
                    user = AuthService.create_user(db, user_data)
                    logger.info(f"[OAUTH GET] New user created with ID: {user.id}")
                    if picture:
                        user.image = picture
                        db.commit()
                        logger.info("[OAUTH GET] Set profile picture for new user")
                except Exception as e:
                    logger.error(f"[OAUTH GET] Error creating user: {e}")
                    return RedirectResponse(f"{frontend_url}/auth-callback?error=account_creation_failed")

            # Create tokens
            try:
                tokens = AuthService.create_tokens(db, user.id, user.email)
                logger.info(f"[OAUTH GET] Tokens created successfully for user: {user.email}")
            except Exception as e:
                logger.error(f"[OAUTH GET] Error creating tokens for user {user.email}: {str(e)}")
                import traceback
                logger.error(traceback.format_exc())
                return RedirectResponse(f"{frontend_url}/auth-callback?error=token_creation_failed")

            # Redirect to frontend with tokens
            target_url = (
                f"{frontend_url}/auth-callback?"
                f"access_token={tokens['access_token']}&"
                f"refresh_token={tokens['refresh_token']}&"
                f"success=true"
            )
            logger.info(f"[OAUTH GET] Success. Redirecting to frontend: {frontend_url}/auth-callback")
            return RedirectResponse(target_url)

    except Exception as e:
        logger.error(f"[OAUTH GET] Unexpected error: {str(e)}")
        import traceback
        logger.error(traceback.format_exc())
        # Use default frontend URL if not defined due to early error
        error_frontend_url = settings.frontend_url or "http://localhost:3000"
        return RedirectResponse(f"{error_frontend_url}/auth-callback?error=server_error")


# Pydantic model for Google OAuth callback (for POST version)
from pydantic import BaseModel as PydanticBaseModel

class GoogleCallbackRequest(PydanticBaseModel):
    code: str
    state: Optional[str] = None

# POST OAuth callback - simplified to avoid Request conflicts

@router.post("/oauth/google/callback", response_model=AuthSuccess)
async def google_oauth_callback_post(oauth_request: GoogleCallbackRequest, db: Session = Depends(get_db)):
    """Handle Google OAuth callback - exchange code for tokens and get user info"""
    import logging
    logger = logging.getLogger(__name__)
    import httpx
    import urllib.parse

    try:
        # Get code from request body (Pydantic model)
        code = oauth_request.code

        if not code or not isinstance(code, str) or len(code) < 10:
            logger.error(f"[OAUTH POST] Invalid code format")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid authorization code format."
            )

        client_id = settings.google_client_id
        client_secret = settings.google_client_secret

        # Use settings for redirect_uri to match what Google expects
        redirect_uri = settings.google_redirect_uri

        logger.info(f"[OAUTH POST] Callback received - Code: {code[:10] if code else 'None'}")
        logger.info(f"[OAUTH POST] Redirect URI used for exchange: {redirect_uri}")

        if not client_id or not client_secret:
            logger.error(f"[OAUTH POST] Missing client credentials")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Google OAuth is not properly configured. Missing client_id or client_secret."
            )

        # Exchange authorization code for tokens
        token_url = "https://oauth2.googleapis.com/token"
        token_data = {
            "code": code,
            "client_id": client_id,
            "client_secret": client_secret,
            "redirect_uri": redirect_uri,
            "grant_type": "authorization_code"
        }

        logger.info(f"[OAUTH POST] Exchanging code for tokens...")

        async with httpx.AsyncClient(timeout=30.0) as client:
            token_response = await client.post(token_url, data=token_data)

            logger.info(f"[OAUTH POST] Token exchange status: {token_response.status_code}")

            if token_response.status_code != 200:
                error_detail = token_response.text[:200] if token_response.text else "Unknown error"
                logger.error(f"[OAUTH POST] Token exchange failed: {error_detail}")

                if "invalid_grant" in error_detail.lower():
                    raise HTTPException(
                        status_code=status.HTTP_400_BAD_REQUEST,
                        detail="Invalid or expired authorization code. Please try signing in again."
                    )
                elif "redirect_uri_mismatch" in error_detail.lower():
                    raise HTTPException(
                        status_code=status.HTTP_400_BAD_REQUEST,
                        detail=f"Redirect URI mismatch. Expected: {redirect_uri}"
                    )
                else:
                    raise HTTPException(
                        status_code=status.HTTP_400_BAD_REQUEST,
                        detail=f"Failed to exchange code for token: {error_detail[:100]}"
                    )

            token_json = token_response.json()
            google_access_token = token_json.get("access_token")

            if not google_access_token:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="No access token received from Google."
                )

            logger.info("[OAUTH POST] Access token received")

            # Get user info from Google
            userinfo_url = "https://www.googleapis.com/oauth2/v2/userinfo"
            headers = {"Authorization": f"Bearer {google_access_token}"}
            userinfo_response = await client.get(userinfo_url, headers=headers)

            logger.info(f"[OAUTH POST] User info status: {userinfo_response.status_code}")

            if userinfo_response.status_code != 200:
                error_detail = userinfo_response.text[:200] if userinfo_response.text else "Unknown error"
                logger.error(f"[OAUTH POST] User info failed: {error_detail}")
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Failed to get user info from Google: {error_detail[:100]}"
                )

            user_info = userinfo_response.json()
            logger.info(f"[OAUTH POST] User info response from Google: {user_info}")

            # Extract user data from Google response
            email = user_info.get("email")
            name = user_info.get("name", email.split("@")[0] if email else "Unknown")
            google_id = user_info.get("id")
            picture = user_info.get("picture")

            logger.info(f"[OAUTH POST] User data extracted - Email: {email}, Name: {name}, ID: {google_id}")

            if not email:
                logger.error("[OAUTH POST] No email provided by Google")
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Email not provided by Google"
                )

            # Import User model
            from .database import User

            # Check if user already exists
            user_to_process = db.query(User).filter(User.email == email).first()

            if user_to_process:
                logger.info(f"[OAUTH POST] Existing user found: {user_to_process.id}")

                # User exists - update profile picture if not set or different
                if picture and (not user_to_process.image or user_to_process.image != picture):
                    user_to_process.image = picture
                    try:
                        db.commit()
                        logger.info("[OAUTH POST] Profile picture updated")
                    except Exception as e:
                        db.rollback()
                        logger.error(f"[OAUTH POST] DB commit error: {str(e)}")
                        raise HTTPException(
                            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                            detail="Error updating user profile."
                        )
                try:
                    tokens = AuthService.create_tokens(db, user_to_process.id, user_to_process.email)
                except Exception as e:
                    logger.error(f"[OAUTH POST] Error creating tokens for existing user: {str(e)}")
                    raise HTTPException(
                        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                        detail="Error generating authentication tokens."
                    )
            else:
                logger.info("[OAUTH POST] Creating new user")

                # Create new user with Google OAuth
                user_data = UserCreate(
                    email=email,
                    name=name,
                    password=secrets.token_urlsafe(32)  # Random password for OAuth users
                )
                try:
                    user_to_process = AuthService.create_user(db, user_data)
                    logger.info(f"[OAUTH POST] New user created: {user_to_process.id}")
                except Exception as e:
                    logger.error(f"[OAUTH POST] Error creating user: {str(e)}")

                    # Check if it's a duplicate email error
                    if "email already registered" in str(e).lower():
                        raise HTTPException(
                            status_code=status.HTTP_400_BAD_REQUEST,
                            detail="Email already registered. Please try signing in instead."
                        )
                    else:
                        raise HTTPException(
                            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                            detail="Error creating user account."
                        )

                # Update user's profile picture
                if picture:
                    user_to_process.image = picture
                    try:
                        db.commit()
                        logger.info("[OAUTH POST] Profile picture set for new user")
                    except Exception as e:
                        db.rollback()
                        logger.error(f"[OAUTH POST] DB commit error: {str(e)}")
                        raise HTTPException(
                            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                            detail="Error saving user profile."
                        )

                try:
                    tokens = AuthService.create_tokens(db, user_to_process.id, user_to_process.email)
                except Exception as e:
                    logger.error(f"[OAUTH POST] Error creating tokens for new user: {str(e)}")
                    raise HTTPException(
                        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                        detail="Error generating authentication tokens."
                    )

            logger.info(f"[OAUTH POST] Returning auth success for: {user_to_process.email}")

            # Create user profile manually to ensure proper field mapping
            user_profile = UserProfile(
                id=user_to_process.id,
                email=user_to_process.email,
                name=user_to_process.name,
                bio=user_to_process.bio,
                profile_picture=user_to_process.image,  # Map image field to profile_picture
                profile_complete=user_to_process.profile_complete,
                created_at=user_to_process.created_at,
                updated_at=user_to_process.updated_at
            )
            return AuthSuccess(user=user_profile, tokens=tokens)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"[OAUTH POST] Unexpected error: {str(e)}")
        import traceback
        logger.error(traceback.format_exc())
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing Google OAuth: {str(e)}"
        )


@router.get("/sessions")
async def get_active_sessions(current_user: dict = Depends(get_current_user), db: Session = Depends(get_db)):
    """Get all active sessions for the user"""
    from .database import TokenSession
    sessions = db.query(TokenSession).filter(
        TokenSession.user_id == current_user["user_id"],
        TokenSession.is_active == True
    ).all()

    return {
        "user_id": current_user["user_id"],
        "active_sessions": len(sessions),
        "sessions": [
            {
                "id": session.id,
                "created_at": session.created_at,
                "expires_at": session.expires_at
            }
            for session in sessions
        ]
    }