"""
Clear expired token sessions
"""
import psycopg
from datetime import datetime
from app.config import settings

def clear_expired_tokens():
    """Clear all expired or inactive token sessions"""

    try:
        conn_str = settings.database_url
        print("Connecting to database...")

        with psycopg.connect(conn_str) as conn:
            with conn.cursor() as cur:
                # Count active sessions
                count_sql = "SELECT COUNT(*) FROM token_sessions WHERE is_active = TRUE"
                cur.execute(count_sql)
                active_count = cur.fetchone()[0]
                print(f"Active sessions: {active_count}")

                # Deactivate all expired sessions
                update_sql = """
                    UPDATE token_sessions
                    SET is_active = FALSE
                    WHERE is_active = TRUE
                    AND expires_at < NOW()
                """
                cur.execute(update_sql)
                conn.commit()
                print("Expired sessions deactivated")

                # Optional: Deactivate ALL sessions (for debugging)
                # Uncomment below line if needed
                # cur.execute("UPDATE token_sessions SET is_active = FALSE")
                # conn.commit()
                # print("All sessions deactivated")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    clear_expired_tokens()
    print("\nToken sessions cleaned!")
