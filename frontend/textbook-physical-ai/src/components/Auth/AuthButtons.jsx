import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import SignupModal from './SignupModal';
import SigninModal from './SigninModal';
import UserDropdown from './UserDropdown';
import styles from './AuthButtons.module.css';

const AuthButtons = () => {
  const { isAuthenticated, loading } = useAuth();
  const [signupOpen, setSignupOpen] = useState(false);
  const [signinOpen, setSigninOpen] = useState(false);

  if (loading) {
    return <div className={styles.authButtons}>Loading...</div>;
  }

  if (isAuthenticated) {
    return <UserDropdown />;
  }

  return (
    <>
      <div className={styles.authButtons}>
        <button className={`${styles.authButton} ${styles.primary}`} onClick={() => setSignupOpen(true)}>
          Sign Up
        </button>
        <button className={`${styles.authButton} ${styles.secondary}`} onClick={() => setSigninOpen(true)}>
          Sign In
        </button>
      </div>

      <SignupModal isOpen={signupOpen} onClose={() => setSignupOpen(false)} />
      <SigninModal isOpen={signinOpen} onClose={() => setSigninOpen(false)} />
    </>
  );
};

export default AuthButtons;
