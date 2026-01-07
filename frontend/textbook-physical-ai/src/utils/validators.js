/**
 * Validate email format
 */
export const validateEmail = (email) => {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
};

/**
 * Validate password strength
 * Returns object with valid flag and message
 */
export const validatePassword = (password) => {
  if (!password || password.length < 8) {
    return { valid: false, message: 'Password must be at least 8 characters' };
  }
  return { valid: true, message: '' };
};

/**
 * Validate password confirmation matches
 */
export const validatePasswordMatch = (password, confirmation) => {
  return password === confirmation;
};

/**
 * Validate name is not empty
 */
export const validateName = (name) => {
  if (!name || name.trim().length === 0) {
    return { valid: false, message: 'Name is required' };
  }
  return { valid: true, message: '' };
};
