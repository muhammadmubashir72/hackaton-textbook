from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from .config import settings
import urllib.parse
import re

# Create database engine
# Properly handle PostgreSQL/NeonDB connection
SQLALCHEMY_DATABASE_URL = settings.database_url

# For PostgreSQL/NeonDB, we need to handle the URL properly
if SQLALCHEMY_DATABASE_URL.startswith("postgresql"):
    # Remove channel_binding parameter as it's not supported by some drivers
    clean_url = re.sub(r'[&?]channel_binding=[^&]*', '', SQLALCHEMY_DATABASE_URL)

    # Replace postgresql:// with postgresql+psycopg:// for psycopg3 driver
    if clean_url.startswith("postgresql://"):
        clean_url = clean_url.replace("postgresql://", "postgresql+psycopg://", 1)

    # Properly encode the URL to handle special characters in passwords
    engine = create_engine(
        clean_url,
        pool_size=settings.db_pool_min,
        max_overflow=settings.db_pool_max - settings.db_pool_min,
        pool_pre_ping=True,  # Verify connections before use
        pool_recycle=300,    # Recycle connections every 5 minutes
    )
elif SQLALCHEMY_DATABASE_URL.startswith("sqlite"):
    engine = create_engine(
        SQLALCHEMY_DATABASE_URL,
        connect_args={"check_same_thread": False}  # Needed for SQLite
    )
else:
    # Default to SQLite if no recognized protocol
    engine = create_engine("sqlite:///./textbook_auth.db",
                          connect_args={"check_same_thread": False})

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()


def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# Create all tables
def create_all_tables():
    # Import models here to avoid circular imports
    from .auth.database import User, TokenSession
    Base.metadata.create_all(bind=engine)