import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import styles from './FileUpload.module.css';

const FileUpload = () => {
  const { user, loading: authLoading } = useAuth();
  const [file, setFile] = useState(null);
  const [uploading, setUploading] = useState(false);
  const [uploadStatus, setUploadStatus] = useState('');
  const [progress, setProgress] = useState(0);

  const handleFileChange = (e) => {
    const selectedFile = e.target.files[0];
    if (selectedFile) {
      // Validate file type and size
      const validTypes = ['text/plain', 'text/markdown', 'text/x-markdown', 'application/pdf', 'application/msword', 'application/vnd.openxmlformats-officedocument.wordprocessingml.document'];
      const maxSize = 10 * 1024 * 1024; // 10MB

      if (!validTypes.includes(selectedFile.type) && !selectedFile.name.match(/\.(txt|md|pdf|doc|docx)$/i)) {
        setUploadStatus('Invalid file type. Please upload a text, markdown, PDF, or Word document.');
        return;
      }

      if (selectedFile.size > maxSize) {
        setUploadStatus('File too large. Maximum size is 10MB.');
        return;
      }

      setFile(selectedFile);
      setUploadStatus('');
    }
  };

  const handleUpload = async () => {
    if (!file) {
      setUploadStatus('Please select a file first.');
      return;
    }

    if (!user) {
      setUploadStatus('Please sign in to upload files.');
      return;
    }

    setUploading(true);
    setProgress(0);

    try {
      const formData = new FormData();
      formData.append('file', file);
      formData.append('userId', user.id);

      // Simulate upload progress
      const interval = setInterval(() => {
        setProgress(prev => {
          if (prev >= 90) {
            clearInterval(interval);
            return prev;
          }
          return prev + 10;
        });
      }, 200);

      // In a real implementation, this would be an actual API call
      const response = await fetch('http://localhost:3001/api/upload', {
        method: 'POST',
        body: formData,
        headers: {
          // In a real implementation, we'd include the auth token
          // 'Authorization': `Bearer ${getAccessToken()}`,
        },
      });

      clearInterval(interval);
      setProgress(100);

      if (response.ok) {
        const result = await response.json();
        setUploadStatus('File uploaded successfully!');
        setTimeout(() => {
          setFile(null);
          setUploadStatus('');
          setProgress(0);
        }, 2000);
      } else {
        throw new Error('Upload failed');
      }
    } catch (error) {
      setUploadStatus(`Upload failed: ${error.message}`);
      setProgress(0);
    } finally {
      setUploading(false);
    }
  };

  if (authLoading) {
    return <div className={styles.container}>Loading...</div>;
  }

  if (!user) {
    return (
      <div className={styles.container}>
        <div className={styles.notAuthenticated}>
          <p>Please sign in to upload files.</p>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <h3>Upload Document</h3>
      <div className={styles.uploadArea}>
        <input
          type="file"
          id="fileInput"
          onChange={handleFileChange}
          className={styles.fileInput}
          accept=".txt,.md,.pdf,.doc,.docx"
        />
        <label htmlFor="fileInput" className={styles.fileInputLabel}>
          <div className={styles.fileInputContent}>
            <div className={styles.uploadIcon}>📁</div>
            <p>Click to select a file or drag and drop</p>
            <p className={styles.fileTypes}>Text, Markdown, PDF, DOC, DOCX (Max 10MB)</p>
          </div>
        </label>
        
        {file && (
          <div className={styles.selectedFile}>
            <div className={styles.fileName}>{file.name}</div>
            <div className={styles.fileSize}>{(file.size / 1024).toFixed(2)} KB</div>
          </div>
        )}

        {uploading && (
          <div className={styles.progressBar}>
            <div 
              className={styles.progressFill} 
              style={{ width: `${progress}%` }}
            ></div>
          </div>
        )}

        <button 
          onClick={handleUpload} 
          disabled={uploading || !file}
          className={`${styles.uploadButton} ${uploading || !file ? styles.disabled : ''}`}
        >
          {uploading ? `Uploading... ${progress}%` : 'Upload File'}
        </button>

        {uploadStatus && (
          <div className={`${styles.status} ${uploadStatus.includes('failed') || uploadStatus.includes('Invalid') ? styles.error : styles.success}`}>
            {uploadStatus}
          </div>
        )}
      </div>
    </div>
  );
};

export default FileUpload;