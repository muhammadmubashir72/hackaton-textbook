"""
Script to add missing profile_complete column to users table
"""
import psycopg
from app.config import settings

def add_profile_complete_column():
    """Add profile_complete column to users table if it doesn't exist"""

    try:
        # Connect to database
        conn_str = settings.database_url
        print(f"Connecting to database...")

        with psycopg.connect(conn_str) as conn:
            with conn.cursor() as cur:
                # Check if column exists
                check_sql = """
                    SELECT column_name
                    FROM information_schema.columns
                    WHERE table_name = 'users'
                    AND column_name = 'profile_complete'
                """
                cur.execute(check_sql)
                result = cur.fetchone()

                if result:
                    print("Column 'profile_complete' already exists in users table")
                    return

                # Add column
                alter_sql = """
                    ALTER TABLE users
                    ADD COLUMN profile_complete BOOLEAN DEFAULT FALSE;
                """
                print("Adding 'profile_complete' column to users table...")
                cur.execute(alter_sql)
                conn.commit()
                print("Column 'profile_complete' added successfully")

    except Exception as e:
        print(f"Error: {e}")
        raise

if __name__ == "__main__":
    add_profile_complete_column()
    print("\nDatabase schema updated successfully!")
