import {
  signup as signupAPI,
  signin as signinAPI,
  refresh as refreshAPI,
  signout as signoutAPI,
  getGoogleAuthUrl,
  googleAuthCallback,
} from './authAPI';
import { storeTokens, getAccessToken, getRefreshToken, clearTokens, decodeToken } from './tokenService';
import { getErrorMessage } from '../utils/errorMessages';

/**
 * Auth Service Interface
 * Provides high-level authentication operations
 */
class AuthService {
  /**
   * Register a new user
   */
  async signup(email, password, name) {
    try {
      const data = await signupAPI(email, password, name);
      storeTokens(data.data.tokens);
      const user = decodeToken(data.data.tokens.accessToken);
      return {
        success: true,
        user,
        tokens: data.data.tokens,
      };
    } catch (error) {
      const message = getErrorMessage(error);
      return {
        success: false,
        error: message,
      };
    }
  }

  /**
   * Sign in existing user
   */
  async signin(email, password) {
    try {
      const data = await signinAPI(email, password);
      storeTokens(data.data.tokens);
      const user = decodeToken(data.data.tokens.accessToken);
      return {
        success: true,
        user,
        tokens: data.data.tokens,
      };
    } catch (error) {
      const message = getErrorMessage(error);
      return {
        success: false,
        error: message,
      };
    }
  }

  /**
   * Sign out current user
   */
  async logout() {
    try {
      const accessToken = getAccessToken();
      const refreshToken = getRefreshToken();
      if (accessToken && refreshToken) {
        await signoutAPI(accessToken, refreshToken);
      }
    } catch (error) {
      console.error('Signout API call failed:', error);
    } finally {
      clearTokens();
      return {
        success: true,
        message: 'Signed out successfully',
      };
    }
  }

  /**
   * Refresh authentication tokens
   */
  async refreshSession(refreshToken) {
    try {
      const data = await refreshAPI(refreshToken);
      const newAccessToken = data.data.accessToken;
      storeTokens({ accessToken: newAccessToken, refreshToken });
      const user = decodeToken(newAccessToken);
      return {
        success: true,
        user,
        tokens: { accessToken: newAccessToken, refreshToken },
      };
    } catch (error) {
      const message = getErrorMessage(error);
      return {
        success: false,
        error: message,
      };
    }
  }

  /**
   * Validate current session
   */
  validateSession(accessToken) {
    try {
      const user = decodeToken(accessToken);
      if (!user) {
        return {
          success: false,
          error: 'Invalid token',
        };
      }

      // Check if token is expired
      const currentTime = Date.now() / 1000;
      if (user.exp < currentTime) {
        return {
          success: false,
          error: 'Token expired',
        };
      }

      return {
        success: true,
        user,
      };
    } catch (error) {
      return {
        success: false,
        error: 'Token validation failed',
      };
    }
  }

  /**
   * Get Google OAuth URL
   */
  async getGoogleAuthUrl() {
    try {
      const data = await getGoogleAuthUrl();
      return {
        success: true,
        url: data.url,
        state: data.state,
      };
    } catch (error) {
      const message = getErrorMessage(error);
      return {
        success: false,
        error: message,
      };
    }
  }

  /**
   * Handle Google OAuth callback
   */
  async handleGoogleCallback(code, state) {
    try {
      const data = await googleAuthCallback(code, state);
      storeTokens(data.data.tokens);
      const user = decodeToken(data.data.tokens.accessToken);
      return {
        success: true,
        user,
        tokens: data.data.tokens,
      };
    } catch (error) {
      const message = getErrorMessage(error);
      return {
        success: false,
        error: message,
      };
    }
  }

  /**
   * Check if user is currently authenticated
   */
  isAuthenticated() {
    const accessToken = getAccessToken();
    if (!accessToken) {
      return false;
    }

    const result = this.validateSession(accessToken);
    return result.success;
  }

  /**
   * Get current user profile
   */
  getCurrentUser() {
    const accessToken = getAccessToken();
    if (!accessToken) {
      return null;
    }

    const user = decodeToken(accessToken);
    return user;
  }
}

export default new AuthService();