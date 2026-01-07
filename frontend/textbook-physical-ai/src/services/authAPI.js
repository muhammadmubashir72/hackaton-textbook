import axios from 'axios';
import { getAccessToken, getRefreshToken, storeTokens, clearTokens } from './tokenService';

// Helper to get base URL safely for SSR
const getAuthAPIBaseUrl = () => {
  // Always use localhost:8001 for local development
  const baseUrl = 'http://localhost:8001/api/auth';

  // Check for environment override
  if (typeof window !== 'undefined' && window._env_?.REACT_APP_AUTH_API_URL) {
    console.log('[AuthAPI] Using env URL:', window._env_.REACT_APP_AUTH_API_URL);
    return window._env_.REACT_APP_AUTH_API_URL;
  }

  console.log('[AuthAPI] Using default URL:', baseUrl);
  return baseUrl;
};

// Create axios instance with base URL
const authAPI = axios.create({
  baseURL: getAuthAPIBaseUrl(),
  timeout: 10000, // Increased timeout
  headers: {
    'Content-Type': 'application/json',
  },
  withCredentials: false, // Explicitly disable credentials for CORS
});

// Track if refresh is in progress to prevent multiple simultaneous refresh calls
let isRefreshing = false;
let failedQueue = [];

const processQueue = (error, token = null) => {
  failedQueue.forEach((prom) => {
    if (error) {
      prom.reject(error);
    } else {
      prom.resolve(token);
    }
  });
  failedQueue = [];
};

// Request interceptor: Add Authorization header
authAPI.interceptors.request.use(
  (config) => {
    // Skip auth header for public endpoints
    const publicEndpoints = ['/signup', '/signin'];
    const isPublic = publicEndpoints.some((endpoint) => config.url.includes(endpoint));

    if (!isPublic) {
      const accessToken = getAccessToken();
      if (accessToken) {
        config.headers.Authorization = `Bearer ${accessToken}`;
      }
    }

    return config;
  },
  (error) => Promise.reject(error)
);

// Response interceptor: Handle 401 and auto-refresh
authAPI.interceptors.response.use(
  (response) => response,
  async (error) => {
    const originalRequest = error.config;

    // Check if 401 and not already retried
    if (error.response?.status === 401 && !originalRequest._retry) {
      if (isRefreshing) {
        // Another request is already refreshing, wait for it
        return new Promise((resolve, reject) => {
          failedQueue.push({ resolve, reject });
        })
          .then((token) => {
            originalRequest.headers.Authorization = `Bearer ${token}`;
            return authAPI(originalRequest);
          })
          .catch((err) => Promise.reject(err));
      }

      originalRequest._retry = true;
      isRefreshing = true;

      const refreshToken = getRefreshToken();
      if (!refreshToken) {
        clearTokens();
        if (typeof window !== 'undefined') {
          window.location.href = '/';
        }
        return Promise.reject(error);
      }

      try {
        const response = await authAPI.post('/refresh', { refreshToken });
        const newAccessToken = response.data.data.accessToken;
        storeTokens({ accessToken: newAccessToken, refreshToken });

        authAPI.defaults.headers.common.Authorization = `Bearer ${newAccessToken}`;
        originalRequest.headers.Authorization = `Bearer ${newAccessToken}`;

        processQueue(null, newAccessToken);
        isRefreshing = false;

        return authAPI(originalRequest);
      } catch (refreshError) {
        processQueue(refreshError, null);
        isRefreshing = false;
        clearTokens();
        if (typeof window !== 'undefined') {
          window.location.href = '/';
        }
        return Promise.reject(refreshError);
      }
    }

    return Promise.reject(error);
  }
);

// Helper to transform backend response to expected frontend format
const transformAuthResponse = (backendData) => {
  // Backend returns: { access_token, refresh_token, token_type }
  // Frontend expects: { data: { tokens: { accessToken, refreshToken } } }
  return {
    data: {
      tokens: {
        accessToken: backendData.access_token,
        refreshToken: backendData.refresh_token,
      },
    },
  };
};

// API methods
export const signup = async (email, password, name) => {
  const response = await authAPI.post('/signup', { email, password, name });
  return transformAuthResponse(response.data);
};


// Transform user object from snake_case to camelCase
const transformUserObject = (user) => {
  if (!user) return null;
  return {
    ...user,
    profilePicture: user.profile_picture || user.profilePicture,
    profileComplete: user.profile_complete !== undefined ? user.profile_complete : user.profileComplete,
    createdAt: user.created_at || user.createdAt,
    updatedAt: user.updated_at || user.updatedAt,
  };
};

export const getProfile = async () => {
  const response = await authAPI.get('/profile');
  // Backend returns profile directly, frontend expects { data: { user: {...} } }
  return {
    data: {
      user: transformUserObject(response.data),
    },
  };
};

export const updateProfile = async (profileData) => {
  const response = await authAPI.post('/profile', profileData);
  return {
    data: {
      user: transformUserObject(response.data),
    },
  };
};

export const updateProfilePicture = async (file) => {
  const formData = new FormData();
  formData.append('file', file);

  const response = await authAPI.post('/profile/picture', formData, {
    headers: {
      'Content-Type': 'multipart/form-data',
    },
  });
  // Backend returns user profile directly, wrap it in expected format
  return {
    data: {
      user: transformUserObject(response.data),
    },
  };
};

export const signin = async (email, password) => {
  const response = await authAPI.post('/signin', { email, password });
  return transformAuthResponse(response.data);
};

export const refresh = async (refreshToken) => {
  const response = await authAPI.post('/refresh', { refresh_token: refreshToken });
  // Refresh returns { access_token, token_type }
  return {
    data: {
      accessToken: response.data.access_token,
    },
  };
};

export const signout = async (accessToken, refreshToken) => {
  const response = await authAPI.post(
    '/signout',
    { refresh_token: refreshToken },
    {
      headers: { Authorization: `Bearer ${accessToken}` },
    }
  );
  return response.data;
};

// Google OAuth methods
export const getGoogleAuthUrl = async () => {
  try {
    console.log('[AuthAPI] Getting Google auth URL from backend...');
    const response = await authAPI.get('/oauth/google/url');
    console.log('[AuthAPI] Received Google auth URL response:', response.data);

    // Backend returns { auth_url: "..." }, but frontend expects { success: true, data: { authUrl: "..." } }
    return {
      success: true,
      data: {
        authUrl: response.data.auth_url
      }
    };
  } catch (error) {
    console.error('[AuthAPI] Error getting Google auth URL:', error);
    console.error('[AuthAPI] Error response:', error.response?.data);
    console.error('[AuthAPI] Error status:', error.response?.status);

    return {
      success: false,
      error: error.response?.data?.detail || error.message || 'Failed to get Google auth URL'
    };
  }
};

export const googleAuthCallback = async (code, state) => {
  try {
    console.log('[AuthAPI] Sending Google OAuth callback to backend...', { code: code ? 'YES' : 'NO', state });
    const response = await authAPI.post('/oauth/google/callback', { code, state });
    console.log('[AuthAPI] Received Google OAuth callback response:', response.data);

    // Backend now returns { user: {...}, tokens: { accessToken, refreshToken, tokenType } }
    const { user, tokens } = response.data;

    if (user && tokens) {
      return {
        success: true,
        data: {
          user: user,
          tokens: {
            accessToken: tokens.access_token,
            refreshToken: tokens.refresh_token,
          },
        },
      };
    } else {
      // This case should ideally not be hit if backend always returns user and tokens
      throw new Error('Invalid response from Google OAuth callback: missing user or tokens.');
    }
  } catch (error) {
    console.error('[AuthAPI] Google OAuth callback error:', error);
    console.error('[AuthAPI] Error response:', error.response?.data);
    console.error('[AuthAPI] Error status:', error.response?.status);

    return {
      success: false,
      error: error.response?.data?.detail || error.message || 'Failed to complete Google OAuth'
    };
  }
};

export default authAPI;
