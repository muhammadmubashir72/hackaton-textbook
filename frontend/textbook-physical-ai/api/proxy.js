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
        const { targetUrl, query, top_k, text, target_language } = req.body;

        // Check if this is a translation request
        if (targetUrl && text !== undefined && target_language !== undefined) {
            // Handle translation request
            const headers = {
                'Content-Type': 'application/json',
            };

            // Add Authorization header if HF_TOKEN is configured in Vercel
            if (process.env.HF_TOKEN) {
                headers['Authorization'] = `Bearer ${process.env.HF_TOKEN}`;
            }

            const response = await fetch(targetUrl, {
                method: 'POST',
                headers: headers,
                body: JSON.stringify({
                    text: text,
                    target_language: target_language
                })
            });

            if (!response.ok) {
                const errorText = await response.text();
                console.error('Translation backend error:', response.status, errorText);

                if (response.status === 403) {
                    return res.status(403).json({
                        error: 'Authentication Required',
                        message: 'Access Forbidden. If your Hugging Face Space is Private, please add HF_TOKEN to your Vercel Environment Variables.',
                        details: errorText
                    });
                }

                return res.status(response.status).json({
                    error: 'Error from translation backend service',
                    details: errorText
                });
            }

            const data = await response.json();
            res.status(200).json(data);
        } else if (targetUrl && query) {
            // Handle query request (existing functionality)
            if (!targetUrl || !query) {
                return res.status(400).json({ error: 'targetUrl and query are required' });
            }

            // Prepare headers
            const headers = {
                'Content-Type': 'application/json',
            };

            // Add Authorization header if HF_TOKEN is configured in Vercel
            if (process.env.HF_TOKEN) {
                headers['Authorization'] = `Bearer ${process.env.HF_TOKEN}`;
            }

            // Make the request to the target backend
            // For local development, we might need to handle authentication differently
            const response = await fetch(targetUrl, {
                method: 'POST',
                headers: headers,
                body: JSON.stringify({
                    query: query,
                    top_k: top_k || 3
                })
            });

            if (!response.ok) {
                const errorText = await response.text();
                console.error('Target backend error:', response.status, errorText);

                if (response.status === 403) {
                    return res.status(403).json({
                        error: 'Authentication Required',
                        message: 'Access Forbidden. If your Hugging Face Space is Private, please add HF_TOKEN to your Vercel Environment Variables.',
                        details: errorText
                    });
                }

                return res.status(response.status).json({
                    error: 'Error from target backend service',
                    details: errorText
                });
            }

            const data = await response.json();

            res.status(200).json(data);
        } else {
            return res.status(400).json({ error: 'Invalid request format. Provide either (targetUrl, query) for search or (targetUrl, text, target_language) for translation.' });
        }
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
