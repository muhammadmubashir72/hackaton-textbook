"""
Reset all token sessions for debugging OAuth issues
"""
import psycopg
from app.config import settings

def reset_all_tokens():
    """Deactivate ALL token sessions"""

    try:
        conn_str = settings.database_url
        print("Connecting to database...")

        with psycopg.connect(conn_str) as conn:
            with conn.cursor() as cur:
                # Count all sessions
                count_sql = "SELECT COUNT(*) FROM token_sessions"
                cur.execute(count_sql)
                total_count = cur.fetchone()[0]
                print(f"Total sessions: {total_count}")

                # Count active sessions
                active_sql = "SELECT COUNT(*) FROM token_sessions WHERE is_active = TRUE"
                cur.execute(active_sql)
                active_count = cur.fetchone()[0]
                print(f"Active sessions: {active_count}")

                # Deactivate ALL sessions
                update_sql = "UPDATE token_sessions SET is_active = FALSE"
                cur.execute(update_sql)
                conn.commit()
                print("All sessions deactivated - OAuth should work now!")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    reset_all_tokens()
    print("\nAll tokens reset!")
