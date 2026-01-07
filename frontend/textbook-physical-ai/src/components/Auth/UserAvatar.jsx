import React from 'react';

const UserAvatar = ({ user, size = '32px', className = '' }) => {
  if (!user) {
    return null;
  }

  const { name, email, profileImage, profilePicture } = user;
  const displayName = name || email;
  const initials = displayName ? displayName.charAt(0).toUpperCase() : 'U';

  // If profilePicture exists (from our backend), use it
  if (profilePicture) {
    return (
      <div
        className={`user-avatar ${className}`}
        style={{
          width: size,
          height: size,
          borderRadius: '50%',
          overflow: 'hidden',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          fontSize: `calc(${size} * 0.4)`,
          fontWeight: 'bold',
          backgroundColor: '#4f8afb',
        }}
      >
        <img
          src={profilePicture}
          alt={displayName}
          style={{
            width: '100%',
            height: '100%',
            objectFit: 'cover',
          }}
          onError={(e) => {
            // Fallback to initials if image fails to load
            e.target.style.display = 'none';
            e.target.nextSibling.style.display = 'flex';
          }}
        />
        <div
          style={{
            display: 'none',
            width: '100%',
            height: '100%',
            alignItems: 'center',
            justifyContent: 'center',
            color: 'white',
          }}
        >
          {initials}
        </div>
      </div>
    );
  }

  // If profileImage exists (e.g., from OAuth), use it
  if (profileImage) {
    return (
      <div
        className={`user-avatar ${className}`}
        style={{
          width: size,
          height: size,
          borderRadius: '50%',
          overflow: 'hidden',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          fontSize: `calc(${size} * 0.4)`,
          fontWeight: 'bold',
          backgroundColor: '#4f8afb',
        }}
      >
        <img
          src={profileImage}
          alt={displayName}
          style={{
            width: '100%',
            height: '100%',
            objectFit: 'cover',
          }}
          onError={(e) => {
            // Fallback to initials if image fails to load
            e.target.style.display = 'none';
            e.target.nextSibling.style.display = 'flex';
          }}
        />
        <div
          style={{
            display: 'none',
            width: '100%',
            height: '100%',
            alignItems: 'center',
            justifyContent: 'center',
            color: 'white',
          }}
        >
          {initials}
        </div>
      </div>
    );
  }

  // Generate a background color based on the user's name/email for consistent colors
  const getColorFromName = (name) => {
    if (!name) return '#4f8afb';

    // Simple hash function to generate consistent colors
    let hash = 0;
    for (let i = 0; i < name.length; i++) {
      hash = name.charCodeAt(i) + ((hash << 5) - hash);
    }

    const colors = [
      '#4f8afb', // blue
      '#6366f1', // indigo
      '#8b5cf6', // violet
      '#ec4899', // pink
      '#f43f5e', // rose
      '#f59e0b', // amber
      '#10b981', // emerald
      '#06b6d4', // cyan
    ];

    return colors[Math.abs(hash) % colors.length];
  };

  const backgroundColor = getColorFromName(displayName);

  return (
    <div
      className={`user-avatar ${className}`}
      style={{
        width: size,
        height: size,
        borderRadius: '50%',
        backgroundColor,
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        color: 'white',
        fontWeight: 'bold',
        fontSize: `calc(${size} * 0.4)`,
      }}
    >
      {initials}
    </div>
  );
};

export default UserAvatar;