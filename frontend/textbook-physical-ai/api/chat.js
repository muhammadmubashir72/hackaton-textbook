export default async function handler(req, res) {
  // Only allow POST requests
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { query, top_k } = req.body;

    // Validate inputs
    if (!query) {
      return res.status(400).json({ error: 'Query is required' });
    }

    // Use environment variable for backend URL (set in Vercel dashboard)
    // This will be available server-side in the Vercel function
    const BACKEND_URL = process.env.BACKEND_API_URL || 'https://mubashirsaeedi-ai-book-backend.hf.space';
    
    // Proxy the request to the backend
    const response = await fetch(`${BACKEND_URL}/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query: query,
        top_k: top_k || 3
      })
    });

    if (!response.ok) {
      const errorText = await response.text();
      console.error('Backend API error:', response.status, errorText);
      return res.status(response.status).json({ 
        error: 'Error from backend service', 
        details: errorText 
      });
    }

    const data = await response.json();
    
    return res.status(200).json(data);
  } catch (error) {
    console.error('API route error:', error);
    return res.status(500).json({ 
      error: 'Internal server error', 
      message: error.message 
    });
  }
}

export const config = {
  runtime: 'edge',
};