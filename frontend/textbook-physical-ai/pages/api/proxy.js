export default async function handler(req, res) {
  // Set CORS headers
  res.setHeader('Access-Control-Allow-Credentials', true);
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET,POST,OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept');

  // Handle preflight request
  if (req.method === 'OPTIONS') {
    res.status(200).end();
    return;
  }

  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const { targetUrl, query, top_k } = req.body;

    if (!targetUrl || !query) {
      return res.status(400).json({ error: 'targetUrl and query are required' });
    }

    // Make the request to the target backend
    const response = await fetch(targetUrl, {
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
      console.error('Target backend error:', response.status, errorText);
      return res.status(response.status).json({
        error: 'Error from target backend service',
        details: errorText
      });
    }

    const data = await response.json();

    res.status(200).json(data);
  } catch (error) {
    console.error('Proxy error:', error);
    res.status(500).json({
      error: 'Internal server error in proxy',
      message: error.message
    });
  }
}

export const config = {
  runtime: 'nodejs',
};