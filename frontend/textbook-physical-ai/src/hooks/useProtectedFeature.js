import { useAuth } from './useAuth';

/**
 * Hook to check if user can access protected features
 * Returns access status and reason if denied
 */
export const useProtectedFeature = () => {
  const { isAuthenticated, user } = useAuth();

  if (!isAuthenticated) {
    return {
      canAccess: false,
      reason: 'signin_required',
    };
  }

  if (!user?.profileComplete) {
    return {
      canAccess: false,
      reason: 'profile_incomplete',
    };
  }

  return {
    canAccess: true,
    reason: null,
  };
};
