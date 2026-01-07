import React, { useState, useEffect } from 'react';
import { useAuth } from '../../hooks/useAuth';
import styles from './ContentEditor.module.css';

const ContentEditor = () => {
  const { user, loading: authLoading } = useAuth();
  const [content, setContent] = useState('');
  const [title, setTitle] = useState('');
  const [saving, setSaving] = useState(false);
  const [saveStatus, setSaveStatus] = useState('');

  useEffect(() => {
    if (user) {
      // In a real implementation, we would fetch the user's saved content
      // For now, we'll just initialize with a welcome message
      setContent('# My Document\n\nStart writing your content here...');
      setTitle('My Document');
    }
  }, [user]);

  const handleSave = async () => {
    if (!user) {
      setSaveStatus('Please sign in to save content.');
      return;
    }

    if (!title.trim()) {
      setSaveStatus('Please enter a title for your document.');
      return;
    }

    setSaving(true);
    setSaveStatus('');

    try {
      // In a real implementation, this would be an actual API call
      const response = await fetch('http://localhost:3001/api/content', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          // In a real implementation, we'd include the auth token
          // 'Authorization': `Bearer ${getAccessToken()}`,
        },
        body: JSON.stringify({
          userId: user.id,
          title,
          content,
          createdAt: new Date().toISOString(),
          updatedAt: new Date().toISOString(),
        }),
      });

      if (response.ok) {
        setSaveStatus('Content saved successfully!');
        setTimeout(() => setSaveStatus(''), 3000);
      } else {
        throw new Error('Save failed');
      }
    } catch (error) {
      setSaveStatus(`Save failed: ${error.message}`);
    } finally {
      setSaving(false);
    }
  };

  if (authLoading) {
    return <div className={styles.container}>Loading...</div>;
  }

  if (!user) {
    return (
      <div className={styles.container}>
        <div className={styles.notAuthenticated}>
          <p>Please sign in to edit content.</p>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <h3>Edit Content</h3>
      <div className={styles.editorContainer}>
        <input
          type="text"
          value={title}
          onChange={(e) => setTitle(e.target.value)}
          placeholder="Document Title"
          className={styles.titleInput}
        />
        <textarea
          value={content}
          onChange={(e) => setContent(e.target.value)}
          placeholder="Write your content here using Markdown syntax..."
          className={styles.contentArea}
          rows={20}
        />
        <div className={styles.editorFooter}>
          <button 
            onClick={handleSave} 
            disabled={saving}
            className={`${styles.saveButton} ${saving ? styles.disabled : ''}`}
          >
            {saving ? 'Saving...' : 'Save Content'}
          </button>
          {saveStatus && (
            <div className={`${styles.status} ${saveStatus.includes('failed') ? styles.error : styles.success}`}>
              {saveStatus}
            </div>
          )}
        </div>
      </div>
      <div className={styles.markdownHelp}>
        <h4>Markdown Help</h4>
        <ul>
          <li><code># Heading 1</code></li>
          <li><code>## Heading 2</code></li>
          <li><code>**bold**</code> or <code>__bold__</code></li>
          <li><code>*italic*</code> or <code>_italic_</code></li>
          <li><code>[link](url)</code></li>
          <li><code>![image](url)</code></li>
          <li><code>`inline code`</code></li>
          <li><code>```code block```</code></li>
        </ul>
      </div>
    </div>
  );
};

export default ContentEditor;