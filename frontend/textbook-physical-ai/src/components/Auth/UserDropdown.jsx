import React from 'react';
import { useAuth } from '../../context/AuthContext';
import { useHistory } from '@docusaurus/router';

const UserDropdown = ({ isOpen, onClose, anchorRef }) => {
  const { user, logout } = useAuth();
  const history = useHistory();

  if (!isOpen || !user) {
    return null;
  }

  const handleLogout = async () => {
    try {
      await logout();
      onClose();
      history.push('/'); // Redirect to homepage after logout
    } catch (error) {
      console.error('Logout failed:', error);
    }
  };

  // Calculate position relative to the anchor element
  let positionStyle = {};
  if (anchorRef && anchorRef.current) {
    const rect = anchorRef.current.getBoundingClientRect();
    positionStyle = {
      position: 'absolute',
      top: `${rect.bottom + 5}px`,
      right: `${window.innerWidth - rect.right}px`,
      zIndex: 1000,
    };
  }

  return (
    <div
      className="user-dropdown"
      style={{
        ...positionStyle,
        backgroundColor: 'var(--ifm-navbar-background-color, #fff)',
        border: '1px solid var(--ifm-color-emphasis-300)',
        borderRadius: '8px',
        boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
        minWidth: '160px',
        ...(!anchorRef ? {
          position: 'absolute',
          top: 'calc(100% + 5px)',
          right: '0',
        } : {}),
      }}
    >
      <div style={{ padding: '0.75rem 1rem' }}>
        <div style={{ fontWeight: 'bold', marginBottom: '0.25rem' }}>
          {user.name || user.email}
        </div>
        <div
          style={{
            fontSize: '0.85rem',
            color: 'var(--ifm-color-emphasis-700)',
            marginBottom: '0.75rem',
            wordBreak: 'break-all',
          }}
        >
          {user.email}
        </div>
        <hr style={{ margin: '0.5rem 0', borderColor: 'var(--ifm-color-emphasis-300)' }} />

        {/* Profile Link */}
        <a
          href="/profile"
          className="navbar__dropdown-item"
          style={{
            display: 'block',
            padding: '0.5rem 0',
            textDecoration: 'none',
            color: 'inherit',
          }}
          onClick={(e) => {
            onClose();
          }}
        >
          Profile
        </a>

        {/* Dashboard/Progress Link */}
        <a
          href="/dashboard"
          className="navbar__dropdown-item"
          style={{
            display: 'block',
            padding: '0.5rem 0',
            textDecoration: 'none',
            color: 'inherit',
          }}
          onClick={(e) => {
            onClose();
          }}
        >
          Dashboard
        </a>

        {/* Settings Link */}
        <a
          href="/settings"
          className="navbar__dropdown-item"
          style={{
            display: 'block',
            padding: '0.5rem 0',
            textDecoration: 'none',
            color: 'inherit',
          }}
          onClick={(e) => {
            onClose();
          }}
        >
          Settings
        </a>

        <hr style={{ margin: '0.5rem 0', borderColor: 'var(--ifm-color-emphasis-300)' }} />

        {/* Logout Link */}
        <a
          href="#"
          className="navbar__dropdown-item"
          style={{
            display: 'block',
            padding: '0.5rem 0',
            textDecoration: 'none',
            color: 'inherit',
          }}
          onClick={(e) => {
            e.preventDefault();
            handleLogout();
          }}
        >
          Sign Out
        </a>
      </div>
    </div>
  );
};

export default UserDropdown;
