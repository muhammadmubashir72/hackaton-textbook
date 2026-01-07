import React, { useState, useEffect, useRef } from 'react';
import { useAuth } from '../../hooks/useAuth';
import SigninModal from '../../components/Auth/SigninModal';
import SignupModal from '../../components/Auth/SignupModal';
import SignoutModal from '../../components/Auth/SignoutModal';

const CustomAuthNavbarItem = (props) => {
  const { user, loading, logout } = useAuth();
  const [showProfileMenu, setShowProfileMenu] = useState(false);
  const [showSigninModal, setShowSigninModal] = useState(false);
  const [showSignupModal, setShowSignupModal] = useState(false);
  const [showSignoutModal, setShowSignoutModal] = useState(false);
  const dropdownRef = useRef(null);

  const toggleProfileMenu = () => {
    setShowProfileMenu((prev) => !prev);
  };

  const handleOpenSigninModal = () => {
    setShowSigninModal(true);
  };

  const handleCloseSigninModal = () => {
    setShowSigninModal(false);
  };

  const handleOpenSignupModal = () => {
    setShowSignupModal(true);
  };

  const handleCloseSignupModal = () => {
    setShowSignupModal(false);
  };

  const handleOpenSignoutModal = () => {
    setShowSignoutModal(true);
    setShowProfileMenu(false); // Close the profile menu when opening signout modal
  };

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
        setShowProfileMenu(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [dropdownRef]);

  if (loading) {
    return (
      <div className="navbar__item">
        <button className="button button--secondary button--sm" disabled>
          Loading...
        </button>
      </div>
    );
  }

  return (
    <>
      <div className="navbar__item" ref={dropdownRef}>
        {!user ? (
          <div className="auth-buttons">
            <button
              className="button button--secondary button--sm"
              onClick={handleOpenSigninModal}
            >
              Sign In
            </button>
            <button
              className="button button--primary button--sm"
              onClick={handleOpenSignupModal}
            >
              Sign Up
            </button>
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
              {user.profilePicture ? (
                <img
                  src={user.profilePicture}
                  alt="Profile"
                  style={{
                    width: '32px',
                    height: '32px',
                    borderRadius: '50%',
                    objectFit: 'cover',
                  }}
                />
              ) : (
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
              )}
            </button>

            {showProfileMenu && (
              <div
                className="profile-dropdown"
                style={{
                  position: 'absolute',
                  top: '50px', // Adjust as needed for proper alignment
                  right: '0',
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
                    href="/dashboard"
                    className="dropdown-item"
                    style={{
                      display: 'block',
                      padding: '0.5rem 0',
                      textDecoration: 'none',
                      color: 'inherit',
                    }}
                  >
                    Dashboard
                  </a>
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
                  <button
                    onClick={handleOpenSignoutModal}
                    className="dropdown-item"
                    style={{
                      display: 'block',
                      padding: '0.5rem 0',
                      textDecoration: 'none',
                      color: 'inherit',
                      background: 'none',
                      border: 'none',
                      width: '100%',
                      textAlign: 'left',
                      cursor: 'pointer',
                    }}
                  >
                    Sign Out
                  </button>
                </div>
              </div>
            )}
          </div>
        )}
      </div>
      <SigninModal isOpen={showSigninModal} onClose={handleCloseSigninModal} />
      <SignupModal isOpen={showSignupModal} onClose={handleCloseSignupModal} />
      <SignoutModal isOpen={showSignoutModal} onClose={() => setShowSignoutModal(false)} />
    </>
  );
};

export default CustomAuthNavbarItem;