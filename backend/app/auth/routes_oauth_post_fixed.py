# POST OAuth callback - simplified to avoid Request conflicts

@router.post("/oauth/google/callback", response_model=AuthSuccess)
async def google_oauth_callback_post(oauth_request: dict, db: Session = Depends(get_db)):
    """Handle Google OAuth callback - exchange code for tokens and get user info"""
    import logging
    logger = logging.getLogger(__name__)
    import httpx
    import urllib.parse

    try:
        # Get code from request body (Pydantic dict-like)
        code = oauth_request.get("code") if oauth_request else None

        if not code or not isinstance(code, str) or len(code) < 10:
            logger.error(f"[OAUTH POST] Invalid code format")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid authorization code format."
            )

        client_id = settings.google_client_id
        client_secret = settings.google_client_secret
        redirect_uri = "http://localhost:3000/auth-callback"

        logger.info(f"[OAUTH POST] Callback received - Code: {code[:10] if code else 'None'}")
        logger.info(f"[OAUTH POST] Redirect URI: {redirect_uri}")

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

            # Extract user data from Google response
            email = user_info.get("email")
            name = user_info.get("name", email.split("@")[0] if email else "Unknown")
            google_id = user_info.get("id")
            picture = user_info.get("picture")

            logger.info(f"[OAUTH POST] User info - Email: {email}, Name: {name}")

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

            # Create UserProfile manually to handle field name differences (image vs profile_picture)
            user_profile = UserProfile(
                id=user_to_process.id,
                email=user_to_process.email,
                name=user_to_process.name,
                bio=user_to_process.bio,
                profile_picture=user_to_process.image,  # Map DB 'image' to profile_picture
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
