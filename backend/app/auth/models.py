from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime
from uuid import UUID


class UserBase(BaseModel):
    email: str
    name: Optional[str] = None


class UserCreate(UserBase):
    password: str
    name: str


class UserUpdate(BaseModel):
    name: Optional[str] = None
    bio: Optional[str] = None
    profile_picture: Optional[str] = None
    profile_complete: Optional[bool] = None


class UserLogin(BaseModel):
    email: str
    password: str


class Token(BaseModel):
    access_token: str
    refresh_token: str
    token_type: str = "bearer"


class TokenData(BaseModel):
    user_id: UUID
    email: str


class UserProfile(BaseModel):
    id: UUID
    email: str
    name: Optional[str] = None
    bio: Optional[str] = None
    profile_picture: Optional[str] = Field(None, alias="image") # Add alias for profile picture
    profile_complete: Optional[bool] = None
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True
        populate_by_name = True # Allow population by alias name

# New model for Google OAuth callback success response
class AuthSuccess(BaseModel):
    user: UserProfile
    tokens: Token

class UserResponse(BaseModel):
    id: UUID
    email: str
    name: Optional[str] = None
    bio: Optional[str] = None
    profile_picture: Optional[str] = None
    profile_complete: Optional[bool] = None
    created_at: datetime

    class Config:
        from_attributes = True


class RefreshTokenRequest(BaseModel):
    refresh_token: str


class SignoutRequest(BaseModel):
    refresh_token: str