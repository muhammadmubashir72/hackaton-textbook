import { jwtDecode } from 'jwt-decode';

/**
 * Decode access token and extract user data
 * Returns structured object with JWT claims
 */
export const decodeAccessToken = (token) => {
  if (!token) return null;
  try {
    const decoded = jwtDecode(token);
    return {
      userId: decoded.sub,
      email: decoded.email,
      name: decoded.name,
      profileComplete: decoded.profileComplete || false,
      roles: decoded.roles || ['user'],
      expiry: decoded.exp,
    };
  } catch (error) {
    console.error('Invalid JWT token:', error);
    return null;
  }
};
