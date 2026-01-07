/**
 * Map backend error responses to user-friendly messages
 */
export const getErrorMessage = (error) => {
  if (!error.response) {
    return 'Network error. Please check your internet connection.';
  }

  const status = error.response.status;
  const backendMessage = error.response.data?.error;

  const statusMessages = {
    400: 'Please check your input and try again.',
    401: backendMessage || 'Invalid email or password.',
    409: 'Email already registered. Please sign in or use a different email.',
    429: 'Too many attempts. Please try again in 1 hour.',
    500: 'Service temporarily unavailable. Please try again later.',
  };

  return statusMessages[status] || 'An unexpected error occurred. Please try again.';
};
