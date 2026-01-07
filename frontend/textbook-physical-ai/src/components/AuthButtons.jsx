import React, { useState, useEffect } from 'react';
import { useAuth } from '../context/AuthContext';

const AuthButtons = () => {
  const { user, loading } = useAuth();
  const [showProfileMenu, setShowProfileMenu] = useState(false);

  const toggleProfileMenu = () => {
    setShowProfileMenu(!showProfileMenu);
  };

  if (loading) {
    return (
      <div className="auth-buttons-container">
        <button className="button button--secondary button--sm">Loading...</button>
      </div>
    );
  }

  return (
    <div className="auth-buttons-container">
      {!user ? (
        <div className="auth-buttons">
          <a href="/signin" className="button button--secondary button--sm" style={{marginRight: '0.5rem'}}>Sign In</a>
          <a href="/signup" className="button button--primary button--sm">Sign Up</a>
        </div>
      ) : (
        <div className="user-profile-container">
          <button 
            className="user-profile-button" 
            onClick={toggleProfileMenu}
            style={{
              background: 'none',
              border: 'none',
              cursor: 'pointer',
              display: 'flex',
              alignItems: 'center',
              padding: '0.25rem',
              borderRadius: '50%',
              transition: 'background-color 0.2s',
            }}
          >
            <div 
              style={{
                width: '32px',
                height: '32px',
                borderRadius: '50%',
                backgroundColor: '#4f8afb',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                color: 'white',
                fontWeight: 'bold',
                fontSize: '14px',
              }}
            >
              {user.name ? user.name.charAt(0).toUpperCase() : 'U'}
            </div>
          </button>
          
          {showProfileMenu && (
            <div 
              className="profile-dropdown"
              style={{
                position: 'absolute',
                top: '50px',
                right: '20px',
                backgroundColor: 'var(--ifm-navbar-background-color, #fff)',
                border: '1px solid var(--ifm-color-emphasis-300)',
                borderRadius: '8px',
                boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
                zIndex: 1000,
                minWidth: '160px',
              }}
            >
              <div style={{ padding: '0.75rem 1rem' }}>
                <div style={{ fontWeight: 'bold', marginBottom: '0.25rem' }}>{user.name || user.email}</div>
                <div style={{ fontSize: '0.85rem', color: 'var(--ifm-color-emphasis-700)', marginBottom: '0.75rem' }}>{user.email}</div>
                <hr style={{ margin: '0.5rem 0', borderColor: 'var(--ifm-color-emphasis-300)' }} />
                <a 
                  href="/profile" 
                  className="dropdown-item"
                  style={{ 
                    display: 'block', 
                    padding: '0.5rem 0', 
                    textDecoration: 'none',
                    color: 'inherit',
                  }}
                >
                  Profile
                </a>
                <a 
                  href="/settings" 
                  className="dropdown-item"
                  style={{ 
                    display: 'block', 
                    padding: '0.5rem 0', 
                    textDecoration: 'none',
                    color: 'inherit',
                  }}
                >
                  Settings
                </a>
                <hr style={{ margin: '0.5rem 0', borderColor: 'var(--ifm-color-emphasis-300)' }} />
                <a 
                  href="/api/auth/signout" 
                  className="dropdown-item"
                  style={{ 
                    display: 'block', 
                    padding: '0.5rem 0', 
                    textDecoration: 'none',
                    color: 'inherit',
                  }}
                >
                  Sign Out
                </a>
              </div>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

// Add CSS for dropdown positioning
const style = document.createElement('style');
style.innerHTML = `
  .auth-buttons-container {
    display: flex;
    align-items: center;
    margin-left: 0.5rem;
  }
  
  .auth-buttons {
    display: flex;
    gap: 0.5rem;
  }
  
  .user-profile-container {
    position: relative;
  }
  
  .user-profile-button:hover {
    background-color: var(--ifm-color-emphasis-200);
  }
  
  .profile-dropdown {
    position: absolute;
    top: 100%;
    right: 0;
    background-color: white;
    border-radius: 8px;
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
    z-index: 1000;
    min-width: 160px;
  }
  
  .dropdown-item {
    display: block;
    padding: 0.5rem 1rem;
    text-decoration: none;
    color: inherit;
    transition: background-color 0.2s;
  }
  
  .dropdown-item:hover {
    background-color: var(--ifm-color-emphasis-200);
  }
`;

// Append style to head if not already present
if (!document.querySelector('#auth-buttons-style')) {
  style.id = 'auth-buttons-style';
  document.head.appendChild(style);
}

export default AuthButtons;