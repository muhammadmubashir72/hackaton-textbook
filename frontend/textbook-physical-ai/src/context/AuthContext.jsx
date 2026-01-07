import React, { createContext, useContext, useReducer, useEffect, useState } from 'react';
import {
  storeTokens,
  getAccessToken,
  getRefreshToken,
  clearTokens,
  isTokenExpired,
  decodeToken,
  validateAndRefreshToken,
} from '../services/tokenService';
import {
  signup as signupAPI,
  signin as signinAPI,
  refresh as refreshAPI,
  signout as signoutAPI,
  updateProfile as updateProfileAPI,
  updateProfilePicture as updateProfilePictureAPI,
  getProfile, // Import getProfile
} from '../services/authAPI';
import { getErrorMessage } from '../utils/errorMessages';
import Toast from '../components/Auth/Toast';

const AuthContext = createContext();

const initialState = {
  isAuthenticated: false,
  user: null,
  loading: true,
  authOperationLoading: false, // For specific auth operations like login/signup
  error: null,
};

const authReducer = (state, action) => {
  switch (action.type) {
    case 'SET_USER':
      return {
        ...state,
        isAuthenticated: true,
        user: action.payload,
        loading: false,
        authOperationLoading: false,
        error: null,
      };
    case 'SET_LOADING':
      return { ...state, loading: action.payload };
    case 'SET_AUTH_OPERATION_LOADING':
      return { ...state, authOperationLoading: action.payload };
    case 'SET_ERROR':
      return { ...state, error: action.payload, loading: false, authOperationLoading: false };
    case 'CLEAR_USER':
      return { ...initialState, loading: false };
    default:
      return state;
  }
};

export const AuthProvider = ({ children }) => {
  const [state, dispatch] = useReducer(authReducer, initialState);
  const [toast, setToast] = useState(null);

  const showToast = (message, type) => {
    setToast({ message, type });
  };

  const hideToast = () => {
    setToast(null);
  };

  // Initialize auth state on mount
  useEffect(() => {
    const initAuth = async () => {
      try {
        const result = await validateAndRefreshToken();
        if (result.isValid && result.user) {
          // Fetch full profile after successful token validation/refresh
          const fullProfile = await getProfile();
          dispatch({ type: 'SET_USER', payload: fullProfile.data.user });
        } else {
          dispatch({ type: 'CLEAR_USER' });
        }
      } catch (error) {
        console.error('Auth initialization failed:', error);
        dispatch({ type: 'CLEAR_USER' });
      }
    };

    initAuth();
  }, []);

  // Storage event listener for multi-tab sync
  useEffect(() => {
    const handleStorageChange = async (e) => { // Added async here
      if (e.key === 'refreshToken' && e.newValue === null) {
        // Refresh token removed (logout in another tab)
        dispatch({ type: 'CLEAR_USER' });
      } else if (e.key === 'accessToken' && e.newValue) {
        // Access token updated (signin in another tab)
        const user = decodeToken(e.newValue);
        if (user) {
          // Fetch full profile after token update from storage
          const fullProfile = await getProfile();
          dispatch({ type: 'SET_USER', payload: fullProfile.data.user });
        }
      }
    };

    if (typeof window !== 'undefined') { // Add this check
      window.addEventListener('storage', handleStorageChange);
      return () => window.removeEventListener('storage', handleStorageChange);
    }
  }, []);

  // Session expiry warning
  useEffect(() => {
    if (!state.isAuthenticated || !state.user) return;

    if (typeof window === 'undefined') return; // Add this check for SSR

    const accessToken = getAccessToken();
    if (!accessToken) return;

    const decoded = decodeToken(accessToken);
    if (!decoded) return;

    const expiryTime = decoded.exp * 1000;
    const currentTime = Date.now();
    const timeUntilExpiry = expiryTime - currentTime;
    const warningTime = timeUntilExpiry - 60000; // 1 minute before

    if (warningTime > 0) {
      const warningTimer = setTimeout(() => {
        showToast('Your session will expire in 1 minute', 'warning');
      }, warningTime);

      return () => clearTimeout(warningTimer);
    }
  }, [state.isAuthenticated, state.user]);

  // Page visibility refresh
  useEffect(() => {
    const handleVisibilityChange = async () => {
      if (typeof document !== 'undefined' && document.visibilityState === 'visible' && state.isAuthenticated) { // Add this check
        const accessToken = getAccessToken();
        const refreshToken = getRefreshToken();

        if (accessToken && isTokenExpired(accessToken) && refreshToken) {
          try {
            const data = await refreshAPI(refreshToken);
            storeTokens({ accessToken: data.data.accessToken, refreshToken });
            // Fetch full profile after token refresh
            const fullProfile = await getProfile();
            dispatch({ type: 'SET_USER', payload: fullProfile.data.user });
          } catch (error) {
            clearTokens();
            dispatch({ type: 'CLEAR_USER' });
          }
        }
      }
    };

    if (typeof document !== 'undefined') { // Add this check
      document.addEventListener('visibilitychange', handleVisibilityChange);
      return () => document.removeEventListener('visibilitychange', handleVisibilityChange);
    }
  }, [state.isAuthenticated]);

  const signup = async (email, password, name) => {
    try {
      dispatch({ type: 'SET_AUTH_OPERATION_LOADING', payload: true });
      const data = await signupAPI(email, password, name);
      storeTokens(data.data.tokens);
      // After signup, fetch the full user profile
      const fullProfile = await getProfile();
      dispatch({ type: 'SET_USER', payload: fullProfile.data.user });
      showToast('Account created successfully!', 'success');
      return fullProfile.data.user; // Return the user data for additional profile updates
    } catch (error) {
      const message = getErrorMessage(error);
      dispatch({ type: 'SET_ERROR', payload: message });
      showToast(message, 'error');
      throw error;
    } finally {
      dispatch({ type: 'SET_AUTH_OPERATION_LOADING', payload: false });
    }
  };


  const signin = async (email, password) => {
    try {
      dispatch({ type: 'SET_AUTH_OPERATION_LOADING', payload: true });
      const data = await signinAPI(email, password);
      storeTokens(data.data.tokens);
      // After signin, fetch the full user profile
      const fullProfile = await getProfile();
      dispatch({ type: 'SET_USER', payload: fullProfile.data.user });
      showToast(`Welcome back, ${fullProfile.data.user.name || fullProfile.data.user.email}!`, 'success');
    } catch (error) {
      const message = getErrorMessage(error);
      dispatch({ type: 'SET_ERROR', payload: message });
      showToast(message, 'error');
      throw error;
    } finally {
      dispatch({ type: 'SET_AUTH_OPERATION_LOADING', payload: false });
    }
  };

  const logout = async () => {
    dispatch({ type: 'SET_AUTH_OPERATION_LOADING', payload: true });
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
      dispatch({ type: 'CLEAR_USER' });
      showToast('Signed out successfully', 'success');
      // Redirect to home page after signout
      if (typeof window !== 'undefined') {
        window.location.href = '/';
      }
      dispatch({ type: 'SET_AUTH_OPERATION_LOADING', payload: false });
    }
  };

  const updateUserProfile = async (profileData) => {
    try {
      dispatch({ type: 'SET_AUTH_OPERATION_LOADING', payload: true });
      const response = await updateProfileAPI(profileData);
      const updatedUser = response.data.user; // Assuming API returns updated user data
      dispatch({ type: 'SET_USER', payload: updatedUser });
      showToast('Profile updated successfully!', 'success');
      return true;
    } catch (error) {
      const message = getErrorMessage(error);
      dispatch({ type: 'SET_ERROR', payload: message });
      showToast(message, 'error');
      throw error;
    } finally {
      dispatch({ type: 'SET_AUTH_OPERATION_LOADING', payload: false });
    }
  };

  const updateProfilePicture = async (file) => {
    try {
      dispatch({ type: 'SET_AUTH_OPERATION_LOADING', payload: true });
      const response = await updateProfilePictureAPI(file);
      const updatedUser = response.data.user;
      dispatch({ type: 'SET_USER', payload: updatedUser });
      showToast('Profile picture updated successfully!', 'success');
      return true;
    } catch (error) {
      const message = getErrorMessage(error);
      dispatch({ type: 'SET_ERROR', payload: message });
      showToast(message, 'error');
      throw error;
    } finally {
      dispatch({ type: 'SET_AUTH_OPERATION_LOADING', payload: false });
    }
  };

  const value = {
    ...state,
    signup,
    signin,
    logout,
    updateUserProfile,
    updateProfilePicture,
    hasCompleteProfile: () => state.user?.profileComplete || false,
    setUser: (user) => dispatch({ type: 'SET_USER', payload: user }),
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
      {toast && <Toast message={toast.message} type={toast.type} onClose={hideToast} />}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export default AuthContext;
