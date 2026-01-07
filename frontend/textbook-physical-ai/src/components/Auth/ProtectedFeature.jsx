import React, { useState } from 'react';
import { useProtectedFeature } from '../../hooks/useProtectedFeature';
import SigninModal from './SigninModal';

const ProtectedFeature = ({ children, requireCompleteProfile = false, onUnauthorized }) => {
  const { canAccess, reason } = useProtectedFeature();
  const [signinOpen, setSigninOpen] = useState(false);

  const handleClick = (e) => {
    if (!canAccess) {
      e.preventDefault();
      e.stopPropagation();

      if (reason === 'signin_required') {
        setSigninOpen(true);
      } else if (reason === 'profile_incomplete') {
        alert('Please complete your profile to access this feature. Visit your profile page to add your software and hardware background.');
      }

      if (onUnauthorized) {
        onUnauthorized(reason);
      }
    }
  };

  return (
    <>
      <div onClick={handleClick} style={{ display: 'inline-block' }}>
        {children}
      </div>
      <SigninModal isOpen={signinOpen} onClose={() => setSigninOpen(false)} />
    </>
  );
};

export default ProtectedFeature;
