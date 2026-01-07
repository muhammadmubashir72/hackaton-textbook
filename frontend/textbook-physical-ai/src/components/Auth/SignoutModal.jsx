import React from 'react';
import { useAuth } from '../../hooks/useAuth';
import Modal from './Modal';
import styles from './AuthForms.module.css';

const SignoutModal = ({ isOpen, onClose }) => {
  const { logout } = useAuth();

  const handleSignout = async () => {
    await logout();
    onClose();
  };

  return (
    <Modal isOpen={isOpen} onClose={onClose} title="Sign Out">
      <div className={styles.authForm}>
        <p className={styles.confirmationText}>
          Are you sure you want to sign out? You will need to sign in again to access your account.
        </p>
        
        <div className={styles.buttonGroup}>
          <button 
            onClick={onClose} 
            className={`${styles.secondaryButton} ${styles.button}`}
          >
            Cancel
          </button>
          <button 
            onClick={handleSignout} 
            className={`${styles.dangerButton} ${styles.button}`}
          >
            Sign Out
          </button>
        </div>
      </div>
    </Modal>
  );
};

export default SignoutModal;