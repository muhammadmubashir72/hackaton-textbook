import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import { validateEmail } from '../../utils/validators';
import Modal from './Modal';
import { getGoogleAuthUrl } from '../../services/authAPI';
import styles from './AuthForms.module.css';

const SigninModal = ({ isOpen, onClose }) => {
  const { signin, loading, error } = useAuth();
  const [formData, setFormData] = useState({ email: '', password: '' });
  const [errors, setErrors] = useState({});
  const [showPassword, setShowPassword] = useState(false); // State for password visibility

  const handleChange = (e) => {
    setFormData({ ...formData, [e.target.name]: e.target.value });
    setErrors({ ...errors, [e.target.name]: '' });
  };

  const handleGoogleSignIn = async () => {
    try {
      const response = await getGoogleAuthUrl();
      if (response.success && response.data) {
        console.log('[SigninModal] Redirecting to Google auth URL:', response.data.authUrl);
        window.location.href = response.data.authUrl;
      } else {
        console.error('Failed to get Google auth URL:', response.error);
        // Set error to display to user
        setErrors({ ...errors, google: response.error || 'Failed to initiate Google sign-in' });
      }
    } catch (err) {
      console.error('Failed to get Google auth URL:', err);
      setErrors({ ...errors, google: err.message || 'Failed to initiate Google sign-in' });
    }
  };

  const validateForm = () => {
    const newErrors = {};
    if (!validateEmail(formData.email)) {
      newErrors.email = 'Invalid email format';
    }
    if (!formData.password) {
      newErrors.password = 'Password is required';
    }
    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!validateForm()) return;

    try {
      await signin(formData.email, formData.password);
      onClose();
      setFormData({ email: '', password: '' });
    } catch (err) {
      // Preserve email, clear password on error
      setFormData({ ...formData, password: '' });
    }
  };

  const togglePasswordVisibility = () => {
    setShowPassword((prev) => !prev);
  };

  return (
    <Modal isOpen={isOpen} onClose={onClose} title="Sign In">
      <form onSubmit={handleSubmit} className={styles.authForm}>
        <div className={styles.formGroup}>
          <label htmlFor="email">Email</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            disabled={loading}
            required
            className={errors.email ? styles.inputError : ''}
          />
          {errors.email && <span className={styles.error}>{errors.email}</span>}
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="password">Password</label>
          <div className={styles.passwordInputContainer}> {/* New container for password field and icon */}
            <input
              type={showPassword ? 'text' : 'password'}
              id="password"
              name="password"
              value={formData.password}
              onChange={handleChange}
              disabled={loading}
              required
              className={errors.password ? styles.inputError : ''}
            />
            <button
              type="button"
              onClick={togglePasswordVisibility}
              className={styles.passwordToggleButton}
              aria-label={showPassword ? 'Hide password' : 'Show password'}
            >
              {showPassword ? (
                // Eye icon (show password)
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  fill="none"
                  viewBox="0 0 24 24"
                  strokeWidth={1.5}
                  stroke="currentColor"
                  className={styles.eyeIcon}
                >
                  <path
                    strokeLinecap="round"
                    strokeLinejoin="round"
                    d="M2.036 12.322a1.012 1.012 0 010-.639C3.423 7.51 7.36 4.5 12 4.5c4.638 0 8.573 3.007 9.963 7.178.07.207.07.431 0 .639C20.577 16.49 16.64 19.5 12 19.5c-4.638 0-8.573-3.007-9.963-7.178z"
                  />
                  <path
                    strokeLinecap="round"
                    strokeLinejoin="round"
                    d="M15 12a3 3 0 11-6 0 3 3 0 016 0z"
                  />
                </svg>
              ) : (
                // Eye-slash icon (hide password)
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  fill="none"
                  viewBox="0 0 24 24"
                  strokeWidth={1.5}
                  stroke="currentColor"
                  className={styles.eyeIcon}
                >
                  <path
                    strokeLinecap="round"
                    strokeLinejoin="round"
                    d="M3.98 8.223A10.477 10.477 0 001.934 12C3.226 16.338 7.244 19.5 12 19.5c.993 0 1.953-.138 2.863-.395M6.228 6.228A10.45 10.45 0 0112 4.5c4.756 0 8.773 3.162 10.065 7.5-.256.91-1.53 3.385-1.787 3.385s-1.53 3.385-1.787 3.385M6.228 6.228L3 3m3.228 3.228l3.65 3.65M12 10.5a1.5 1.5 0 11-3 0 1.5 1.5 0 013 0zm5.405 4.395L21 21m-3.322-3.322l-3.65-3.65"
                  />
                </svg>
              )}
            </button>
          </div>
          {errors.password && <span className={styles.error}>{errors.password}</span>}
        </div>

        {error && <div className={styles.errorBanner}>{error}</div>}
        {errors.google && <div className={styles.errorBanner}>{errors.google}</div>}

        <button type="submit" disabled={loading} className={styles.submitButton}>
          {loading ? (
            <>
              <span className={styles.spinner}></span>
              Signing In...
            </>
          ) : (
            'Sign In'
          )}
        </button>

        <a href="#" className={styles.forgotPassword} disabled>
          Forgot Password? (Coming soon)
        </a>

        <div className={styles.divider}>
          <span>or</span>
        </div>

        <div className={styles.socialLogin}>
          <button
            type="button"
            className={styles.googleButton}
            onClick={handleGoogleSignIn}
          >
            <svg width="18" height="18" viewBox="0 0 24 24" className={styles.googleIcon}>
              <path
                d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92c-.26 1.37-1.04 2.53-2.21 3.31v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.09z"
                fill="#4285F4"
              />
              <path
                d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z"
                fill="#34A853"
              />
              <path
                d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z"
                fill="#FBBC05"
              />
              <path
                d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z"
                fill="#EA4335"
              />
            </svg>
            Sign in with Google
          </button>
        </div>
      </form>
    </Modal>
  );
};

export default SigninModal;
