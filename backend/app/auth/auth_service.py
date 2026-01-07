from sqlalchemy.orm import Session
from sqlalchemy.exc import IntegrityError
from fastapi import HTTPException, status
from typing import Optional, Union
from .database import User, TokenSession
from .jwt_service import JWTService
from .models import UserCreate, UserLogin, UserUpdate
from datetime import datetime, timedelta
from uuid import UUID
import uuid as uuid_module


class AuthService:
    @staticmethod
    def create_user(db: Session, user_data: UserCreate):
        """Create a new user with hashed password"""
        # Check if user already exists
        existing_user = db.query(User).filter(User.email == user_data.email).first()
        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email already registered"
            )

        # Hash the password
        hashed_password = JWTService.hash_password(user_data.password)

        # Create user (using better-auth compatible column names)
        db_user = User(
            email=user_data.email,
            name=user_data.name,
            password_hash=hashed_password  # better-auth uses password_hash
        )
        db.add(db_user)
        db.commit()
        db.refresh(db_user)

        return db_user

    @staticmethod
    def authenticate_user(db: Session, email: str, password: str):
        """Authenticate user credentials"""
        user = db.query(User).filter(User.email == email).first()

        if not user or not user.password_hash or not JWTService.verify_password(password, user.password_hash):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect email or password",
                headers={"WWW-Authenticate": "Bearer"},
            )

        return user

    @staticmethod
    def create_tokens(db: Session, user_id: Union[UUID, str], email: str):
        """Create access and refresh tokens for a user"""
        # Convert UUID to string for JWT (JWT doesn't support UUID natively)
        user_id_str = str(user_id) if isinstance(user_id, UUID) else user_id
        # Create access token (15 minutes)
        access_token_data = {"user_id": user_id_str, "email": email}
        access_token = JWTService.create_access_token(
            data=access_token_data,
            expires_delta=timedelta(minutes=15)
        )

        # Create refresh token (7 days)
        refresh_token_data = {"user_id": user_id_str, "email": email}
        refresh_token = JWTService.create_refresh_token(
            data=refresh_token_data,
            expires_delta=timedelta(days=7)
        )

        # Store refresh token in database
        # Convert string back to UUID for database storage
        user_id_uuid = UUID(user_id_str) if isinstance(user_id_str, str) else user_id
        token_session = TokenSession(
            user_id=user_id_uuid,
            refresh_token=refresh_token,
            expires_at=datetime.utcnow() + timedelta(days=7),
            is_active=True
        )
        db.add(token_session)
        db.commit()

        return {
            "access_token": access_token,
            "refresh_token": refresh_token,
            "token_type": "bearer"
        }

    @staticmethod
    def refresh_access_token(db: Session, refresh_token: str):
        """Refresh access token using refresh token"""
        # Verify the refresh token exists and is active
        token_session = db.query(TokenSession).filter(
            TokenSession.refresh_token == refresh_token,
            TokenSession.is_active == True,
            TokenSession.expires_at > datetime.utcnow()
        ).first()

        if not token_session:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid or expired refresh token",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Get user info
        user = db.query(User).filter(User.id == token_session.user_id).first()
        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Create new access token
        access_token_data = {"user_id": user.id, "email": user.email}
        access_token = JWTService.create_access_token(
            data=access_token_data,
            expires_delta=timedelta(minutes=15)
        )

        return {
            "access_token": access_token,
            "token_type": "bearer"
        }

    @staticmethod
    def logout(db: Session, user_id: Union[UUID, str], refresh_token: str = None):
        """Logout user by invalidating refresh token"""
        if refresh_token:
            # Find and deactivate the specific refresh token
            token_session = db.query(TokenSession).filter(
                TokenSession.refresh_token == refresh_token,
                TokenSession.user_id == user_id
            ).first()

            if token_session:
                token_session.is_active = False
                db.commit()
        else:
            # Deactivate all refresh tokens for the user
            db.query(TokenSession).filter(
                TokenSession.user_id == user_id
            ).update({"is_active": False})
            db.commit()

    @staticmethod
    def get_user_profile(db: Session, user_id: Union[UUID, str]):
        """Get user profile information"""
        user = db.query(User).filter(User.id == user_id).first()
        if not user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )

        return {
            "id": user.id,
            "email": user.email,
            "name": user.name,
            "bio": user.bio,
            "profile_picture": user.image,  # better-auth uses 'image'
            "profile_complete": user.profile_complete,
            "created_at": user.created_at
        }

    @staticmethod
    def update_user_profile(db: Session, user_id: Union[UUID, str], profile_data: UserUpdate):
        """Update user profile information"""
        user = db.query(User).filter(User.id == user_id).first()
        if not user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )

        # Update user fields directly (better-auth stores all in users table)
        if profile_data.name is not None:
            user.name = profile_data.name

        if profile_data.bio is not None:
            user.bio = profile_data.bio

        if profile_data.profile_picture is not None:
            user.image = profile_data.profile_picture  # better-auth uses 'image'

        if profile_data.profile_complete is not None:
            user.profile_complete = profile_data.profile_complete

        db.commit()
        db.refresh(user)

        return {
            "id": user.id,
            "email": user.email,
            "name": user.name,
            "bio": user.bio,
            "profile_picture": user.image,
            "profile_complete": user.profile_complete,
            "created_at": user.created_at
        }