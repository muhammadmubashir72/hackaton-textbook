import React from 'react';
import Layout from '@theme/Layout';

function ChatKitDemo() {
  return (
    <Layout title="ChatKit Demo" description="ChatKit component demo page">
      <div style={{ padding: '20px', maxWidth: '1200px', margin: '0 auto' }}>
        <h1>ChatKit Component Demo</h1>
        <p>This page demonstrates the ChatKit conversational UI component.</p>
        <p>The chat widget is available in the bottom-right corner of the screen.</p>

        <div style={{ marginTop: '20px', padding: '20px', backgroundColor: '#f8f9fa', borderRadius: '8px' }}>
          <h2>Sample Content Area</h2>
          <p>This represents typical page content that the ChatKit component should not interfere with.</p>
          <p>Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nullam auctor, nisl eget ultricies tincidunt, nisl nisl aliquam nisl, eget ultricies nisl nisl eget nisl. Nullam auctor, nisl eget ultricies tincidunt, nisl nisl aliquam nisl, eget ultricies nisl nisl eget nisl.</p>

          <h3>Physical AI Concepts</h3>
          <p>Physical AI combines artificial intelligence with physical systems, enabling robots to interact with the real world intelligently. This includes perception, decision-making, and action in physical environments.</p>

          <h3>Robotics Integration</h3>
          <p>Modern robotics leverages AI to create autonomous systems that can learn from their environment and adapt their behavior accordingly.</p>
        </div>

        <div style={{ marginTop: '20px', padding: '20px', backgroundColor: '#e9ecef', borderRadius: '8px' }}>
          <h2>More Sample Content</h2>
          <p>Additional content to verify that the ChatKit component doesn't interfere with page layout or functionality.</p>
          <ul>
            <li>AI and Robotics fundamentals</li>
            <li>Machine Learning in Physical Systems</li>
            <li>Sensor Integration and Perception</li>
            <li>Control Systems and Actuation</li>
          </ul>
        </div>
      </div>
    </Layout>
  );
}

export default ChatKitDemo;