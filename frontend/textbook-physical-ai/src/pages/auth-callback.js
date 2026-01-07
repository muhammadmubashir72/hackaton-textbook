import React, { useEffect, useState } from 'react';
import { googleAuthCallback } from '../services/authAPI';
import { storeTokens, clearTokens, decodeToken } from '../services/tokenService';
import { useAuth } from '../hooks/useAuth';
import Layout from '@theme/Layout';
import styles from './auth-callback.module.css';

function AuthCallback() {
  const { setUser } = useAuth();
  const [status, setStatus] = useState('loading');
  const [message, setMessage] = useState('');

  useEffect(() => {
    const handleOAuthCallback = async () => {
      const urlParams = new URLSearchParams(window.location.search);
      const code = urlParams.get('code');
      const state = urlParams.get('state');
      const error = urlParams.get('error');

      // Check for error from OAuth
      if (error) {
        setStatus('error');
        setMessage(decodeURIComponent(error));
        clearTokens();
        return;
      }

      // Check for tokens passed via redirect (new GET callback flow)
      const accessToken = urlParams.get('access_token');
      const refreshToken = urlParams.get('refresh_token');
      const success = urlParams.get('success');

      console.log('[AuthCallback] URL parameters:', {
        code: code ? 'YES' : 'NO',
        accessToken: accessToken ? 'YES' : 'NO',
        refreshToken: refreshToken ? 'YES' : 'NO',
        success,
        error,
      });

      // Handle GET redirect flow (tokens passed in URL)
      if (accessToken && refreshToken && success === 'true') {
        try {
          setStatus('loading');
          setMessage('Completing your sign-in...');

          console.log('[AuthCallback] Storing tokens from redirect...');

          // Store tokens
          storeTokens({
            accessToken: accessToken,
            refreshToken: refreshToken,
          });

          // Decode token to get user info
          const user = decodeToken(accessToken);

          if (user) {
            console.log('[AuthCallback] User info from token:', user);
            setUser(user);
            setStatus('success');
            setMessage('Successfully signed in with Google!');

            // Clean URL and redirect to home
            window.history.replaceState({}, '', '/');
            setTimeout(() => {
              window.location.href = '/';
            }, 1500);
          } else {
            throw new Error('Could not decode user from access token');
          }
        } catch (error) {
          console.error('[AuthCallback] Error handling redirect:', error);
          setStatus('error');
          setMessage(
            error.response?.data?.message || error.message || 'Failed to process authentication'
          );
          clearTokens();
        }
        return;
      }

      // Handle POST callback flow (code exchange required)
      if (!code) {
        setStatus('error');
        setMessage('Authorization code not found in URL');
        clearTokens();
        return;
      }

      try {
        setStatus('loading');
        setMessage('Processing your Google sign-in...');

        console.log('[AuthCallback] Calling POST callback with code and state...', { code: code ? code.substring(0, 10) + '...' : null, state });

        const response = await googleAuthCallback(code, state);

        if (response.success && response.data) {
          const { user, tokens } = response.data;

          console.log('[AuthCallback] Received user and tokens from backend:', {
            user: user?.email,
            hasAccessToken: !!tokens?.accessToken,
            hasRefreshToken: !!tokens?.refreshToken
          });

          // Store tokens
          storeTokens({
            accessToken: tokens.accessToken,
            refreshToken: tokens.refreshToken,
          });

          // Set user context
          setUser(user);

          setStatus('success');
          setMessage('Successfully signed in with Google!');

          // Redirect to home page after 2 seconds
          setTimeout(() => {
            window.location.href = '/';
          }, 2000);
        } else {
          console.error('[AuthCallback] Invalid response from server:', response);
          throw new Error(response.error || 'Invalid response from server');
        }
      } catch (error) {
        console.error('[AuthCallback] Google OAuth callback error:', error);
        setStatus('error');
        // Extract the most specific error message
        const errorMessage = error.error ||
                            error.response?.data?.detail ||
                            error.response?.data?.message ||
                            error.message ||
                            'Failed to sign in with Google';
        setMessage(errorMessage);
        clearTokens();
      }
    };

    handleOAuthCallback();
  }, [setUser]);

  return (
    <Layout title="Authentication">
      <div className={styles.container}>
        {status === 'loading' && (
          <div className={styles.messageContainer}>
            <div className={styles.spinner}></div>
            <h2 className={styles.title}>Signing you in...</h2>
            <p className={styles.message}>{message}</p>
          </div>
        )}

        {status === 'success' && (
          <div className={styles.messageContainer}>
            <div className={styles.successIcon}>✓</div>
            <h2 className={styles.title}>Success!</h2>
            <p className={styles.message}>{message}</p>
            <p className={styles.redirect}>Redirecting you to the home page...</p>
          </div>
        )}

        {status === 'error' && (
          <div className={styles.messageContainer}>
            <div className={styles.errorIcon}>✕</div>
            <h2 className={styles.title}>Authentication Failed</h2>
            <p className={styles.message}>{message}</p>
            <button
              className={styles.retryButton}
              onClick={() => (window.location.href = '/')}
            >
              Return to Home
            </button>
          </div>
        )}
      </div>
    </Layout>
  );
}

export default AuthCallback;
