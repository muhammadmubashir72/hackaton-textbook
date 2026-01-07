"""
Check users table columns
"""
import psycopg
from app.config import settings

def check_columns():
    """Check all columns in users table"""

    try:
        conn_str = settings.database_url
        print("Connecting to database...")

        with psycopg.connect(conn_str) as conn:
            with conn.cursor() as cur:
                # Get all columns
                sql = """
                    SELECT column_name, data_type, is_nullable, column_default
                    FROM information_schema.columns
                    WHERE table_name = 'users'
                    ORDER BY ordinal_position
                """
                cur.execute(sql)
                columns = cur.fetchall()

                print(f"\nColumns in 'users' table:")
                print("-" * 80)
                for col in columns:
                    print(f"  {col[0]:25} | {col[1]:20} | Nullable: {col[2]:5} | Default: {col[3]}")
                print("-" * 80)
                print(f"\nTotal columns: {len(columns)}")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    check_columns()
