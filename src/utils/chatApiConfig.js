// Set the API base URL as a global variable for the ChatWidget
// Use environment-specific URL or fallback to localhost
const getApiBaseUrl = () => {
  // Check if we're in production (GitHub Pages) environment
  if (typeof window !== 'undefined' && window.location.hostname.includes('github.io')) {
    // For GitHub Pages deployment, you might want to use a proxy or different backend URL
    return process.env.REACT_APP_API_URL || 'https://your-production-backend.com'; // Replace with your actual production URL
  }

  // For development, use environment variable or default to localhost
  return window.chatApiBaseUrl || process.env.REACT_APP_API_URL || 'http://localhost:8000';
};

window.chatApiBaseUrl = getApiBaseUrl();