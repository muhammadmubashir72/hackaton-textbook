import React from 'react';
import { useAuth } from '../../context/AuthContext';
import { useHistory } from '@docusaurus/router';

const withAuthProtection = (Component) => {
  const ProtectedRoute = (props) => {
    const { isAuthenticated, loading } = useAuth();
    const history = useHistory();

    if (loading) {
      // You can render a loading spinner or null while auth state is being determined
      return null; 
    }

    if (!isAuthenticated) {
      // Redirect to sign-in page if not authenticated
      history.push('/signin');
      return null;
    }

    return <Component {...props} />;
  };

  return ProtectedRoute;
};

export default withAuthProtection;