import axios from 'axios';

// Helper to get base URL safely for SSR and Docusaurus
const getBackendBaseUrl = () => {
  // Check for runtime environment variable (Vercel/production)
  if (typeof window !== 'undefined' && window._env_?.REACT_APP_BACKEND_URL) {
    return window._env_.REACT_APP_BACKEND_URL;
  }
  // Check for build-time environment variable (Docusaurus/React)
  if (typeof process !== 'undefined' && process.env?.REACT_APP_BACKEND_URL) {
    return process.env.REACT_APP_BACKEND_URL;
  }
  // Fallback to localhost
  return 'http://localhost:8001';
};

const personalizationAPI = axios.create({
  baseURL: getBackendBaseUrl(),
  timeout: 60000, // Longer timeout for AI processing (60 seconds for Gemini)
  headers: {
    'Content-Type': 'application/json',
  },
});

export const personalizeContent = async (content, level) => {
  try {
    console.log('Calling personalization API:', { contentLength: content?.length, level });
    console.log('Backend URL:', getBackendBaseUrl());

    // Ensure content is properly formatted and doesn't have invalid control characters
    const cleanContent = content ? content.trim() : '';

    if (!cleanContent) {
      throw new Error('No content provided for personalization');
    }

    const response = await personalizationAPI.post('/personalize', {
      content: cleanContent,
      level,
    }, {
      headers: {
        'Content-Type': 'application/json; charset=utf-8',
      },
    });

    console.log('Personalization API response:', response.data);
    return response.data;
  } catch (error) {
    console.error('Personalization error details:', error);
    if (error.response) {
      console.error('Response status:', error.response.status);
      console.error('Response data:', error.response.data);
    } else if (error.request) {
      console.error('No response received:', error.request);
    } else {
      console.error('Error message:', error.message);
    }
    throw error;
  }
};

export default personalizationAPI;
