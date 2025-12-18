// For Vercel serverless functions with Node.js runtime
export default async function handler(request, response) {
  if (request.method !== 'POST') {
    return response.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const body = typeof request.body === 'string' ? JSON.parse(request.body) : request.body;
    const { query, top_k } = body;

    // Validate inputs
    if (!query) {
      return response.status(400).json({ error: 'Query is required' });
    }

    // Use environment variable for backend URL (set in Vercel dashboard)
    const BACKEND_URL = process.env.BACKEND_API_URL || 'https://mubashirsaeedi-ai-book-backend.hf.space';

    // Proxy the request to the backend
    const apiResponse = await fetch(`${BACKEND_URL}/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query: query,
        top_k: top_k || 3
      })
    });

    if (!apiResponse.ok) {
      const errorText = await apiResponse.text();
      console.error('Backend API error:', apiResponse.status, errorText);
      return response.status(apiResponse.status).json({
        error: 'Error from backend service',
        details: errorText
      });
    }

    const data = await apiResponse.json();

    response.status(200).json(data);
  } catch (error) {
    console.error('API route error:', error);
    response.status(500).json({
      error: 'Internal server error',
      message: error.message
    });
  }
}

// Use Node.js runtime instead of Edge
export const config = {
  runtime: 'nodejs',
};