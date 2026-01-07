import React, { useState, useEffect, useRef } from 'react';
import NavbarItem from '@theme/NavbarItem';
import { useAuth } from '../context/AuthContext';
import UserAvatar from '../components/Auth/UserAvatar';
import UserDropdown from '../components/Auth/UserDropdown';

const AuthNavbarItem = (props) => {
  const { user, loading } = useAuth();
  const [showProfileMenu, setShowProfileMenu] = useState(false);
  const buttonRef = useRef(null);

  const toggleProfileMenu = (e) => {
    e.preventDefault();
    setShowProfileMenu(!showProfileMenu);
  };

  // Close menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (buttonRef.current && !buttonRef.current.contains(event.target)) {
        setShowProfileMenu(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  if (loading) {
    return (
      <div className="navbar__item auth-buttons-container">
        <button className="button button--secondary button--sm" disabled>
          Loading...
        </button>
      </div>
    );
  }

  return (
    <div className="navbar__item auth-buttons-container" ref={buttonRef}>
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
            ref={buttonRef}
          >
            <UserAvatar user={user} size="32px" />
          </button>

          <UserDropdown
            isOpen={showProfileMenu}
            onClose={() => setShowProfileMenu(false)}
            anchorRef={buttonRef}
          />
        </div>
      )}
    </div>
  );
};

export default AuthNavbarItem;