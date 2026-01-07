import { jwtDecode } from 'jwt-decode';

const ACCESS_TOKEN_KEY = 'accessToken';
const REFRESH_TOKEN_KEY = 'refreshToken';

/**
 * Store JWT tokens securely
 * Access token in sessionStorage (cleared on tab close)
 * Refresh token in localStorage (persists across sessions)
 */
export const storeTokens = (tokens) => {
  if (typeof window !== 'undefined') { // SSR check
    if (tokens.accessToken) {
      sessionStorage.setItem(ACCESS_TOKEN_KEY, tokens.accessToken);
    }
    if (tokens.refreshToken) {
      localStorage.setItem(REFRESH_TOKEN_KEY, tokens.refreshToken);
    }
  }
};

/**
 * Get access token from sessionStorage
 */
export const getAccessToken = () => {
  if (typeof window !== 'undefined') { // SSR check
    return sessionStorage.getItem(ACCESS_TOKEN_KEY);
  }
  return null;
};

/**
 * Get refresh token from localStorage
 */
export const getRefreshToken = () => {
  if (typeof window !== 'undefined') { // SSR check
    return localStorage.getItem(REFRESH_TOKEN_KEY);
  }
  return null;
};

/**
 * Clear all tokens from storage
 */
export const clearTokens = () => {
  if (typeof window !== 'undefined') { // SSR check
    sessionStorage.removeItem(ACCESS_TOKEN_KEY);
    localStorage.removeItem(REFRESH_TOKEN_KEY);
  }
};

/**
 * Check if token is expired
 */
export const isTokenExpired = (token) => {
  if (!token) return true;
  try {
    const decoded = jwtDecode(token);
    const currentTime = Date.now() / 1000;
    return decoded.exp < currentTime;
  } catch (error) {
    // console.error('Failed to decode token for expiration check:', error); // Log for debugging
    return true; // Assume expired if decoding fails
  }
};

/**
 * Decode JWT token and return payload
 */
export const decodeToken = (token) => {
  if (!token) return null;
  try {
    return jwtDecode(token);
  } catch (error) {
    console.error('Failed to decode token:', error);
    return null;
  }
};

/**
 * Validate token expiration and refresh when needed
 */
export const validateAndRefreshToken = async () => {
  if (typeof window === 'undefined') { // SSR check
    return { isValid: false, user: null };
  }

  const accessToken = getAccessToken();
  const refreshToken = getRefreshToken();

  if (!accessToken || !refreshToken) {
    return { isValid: false, user: null };
  }

  if (isTokenExpired(accessToken)) {
    // Try to refresh the token
    try {
      const authApiUrl = window._env_?.REACT_APP_AUTH_API_URL || 'http://localhost:3001';
      const response = await fetch(`${authApiUrl}/api/auth/refresh`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ refreshToken }),
      });

      if (response.ok) {
        const data = await response.json();
        const newAccessToken = data.data.accessToken;
        storeTokens({ accessToken: newAccessToken, refreshToken });
        const user = decodeToken(newAccessToken);
        return { isValid: true, user };
      } else {
        const errorData = await response.json();
        console.error('Token refresh failed:', errorData);
        // Refresh failed, clear tokens
        clearTokens();
        return { isValid: false, user: null };
      }
    } catch (error) {
      console.error('Token refresh network error:', error);
      clearTokens();
      return { isValid: false, user: null };
    }
  } else {
    // Token is still valid
    const user = decodeToken(accessToken);
    if (!user) {
      console.error('Invalid token format');
      clearTokens();
      return { isValid: false, user: null };
    }
    return { isValid: true, user };
  }
};
