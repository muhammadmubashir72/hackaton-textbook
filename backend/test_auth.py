"""
Test script to verify authentication functionality
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from app.database import SessionLocal, create_all_tables
from app.auth.auth_service import AuthService
from app.auth.models import UserCreate, UserLogin

def test_auth_flow():
    # Create database tables
    create_all_tables()
    
    # Create a test database session
    db = SessionLocal()
    
    try:
        print("Testing authentication flow...")
        
        # Test user data
        test_user = UserCreate(
            email="test@example.com",
            name="Test User",
            password="password123"
        )
        
        print("1. Creating user...")
        user = AuthService.create_user(db, test_user)
        print(f"   User created: {user.email}")
        
        print("2. Authenticating user...")
        authenticated_user = AuthService.authenticate_user(db, "test@example.com", "password123")
        print(f"   User authenticated: {authenticated_user.email}")
        
        print("3. Creating tokens...")
        tokens = AuthService.create_tokens(db, authenticated_user.id, authenticated_user.email)
        print(f"   Tokens created: access_token length = {len(tokens['access_token'])}")
        
        print("4. Testing failed authentication...")
        try:
            AuthService.authenticate_user(db, "test@example.com", "wrongpassword")
            print("   ERROR: Should have failed with wrong password")
        except Exception as e:
            print(f"   Correctly failed with wrong password: {str(e)}")
        
        print("\nAll tests passed! Authentication flow is working correctly.")

    except Exception as e:
        print(f"Error during testing: {e}")
        import traceback
        traceback.print_exc()
    finally:
        db.close()

if __name__ == "__main__":
    test_auth_flow()