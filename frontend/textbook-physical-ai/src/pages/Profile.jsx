import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../context/AuthContext';
import withAuthProtection from '../components/Auth/withAuthProtection';
import styles from './Profile.module.css';

const Profile = () => {
  const { user, updateUserProfile, updateProfilePicture, authOperationLoading } = useAuth();
  const [isEditing, setIsEditing] = useState(false);
  const [editedName, setEditedName] = useState(user?.name || '');
  const [editedBio, setEditedBio] = useState(user?.bio || '');
  const [selectedFile, setSelectedFile] = useState(null);
  const [previewUrl, setPreviewUrl] = useState(user?.profilePicture || null);

  useEffect(() => {
    setEditedName(user?.name || '');
    setEditedBio(user?.bio || '');
    // Only update previewUrl from user if we're not editing/uploading
    // This prevents overwriting the image preview during upload
    if (!selectedFile) {
      setPreviewUrl(user?.profilePicture || null);
    }
  }, [user, selectedFile]);

  const handleSave = async () => {
    try {
      await updateUserProfile({ name: editedName, bio: editedBio });
      setIsEditing(false);
    } catch (error) {
      // Error handled by AuthContext toast
    }
  };

  const handleCancel = () => {
    setEditedName(user?.name || '');
    setEditedBio(user?.bio || '');
    setSelectedFile(null);
    setIsEditing(false);
  };

  const handleFileChange = (e) => {
    const file = e.target.files[0];
    if (file) {
      // Validate file type
      if (!file.type.startsWith('image/')) {
        alert('Please select an image file (JPG, PNG, GIF, etc.)');
        return;
      }

      // Validate file size (max 5MB)
      if (file.size > 5 * 1024 * 1024) {
        alert('File size exceeds 5MB limit');
        return;
      }

      setSelectedFile(file);

      // Create preview URL
      const preview = URL.createObjectURL(file);
      setPreviewUrl(preview);
    }
  };

  const handleUploadPicture = async () => {
    if (!selectedFile) {
      alert('Please select a profile picture first');
      return;
    }

    try {
      await updateProfilePicture(selectedFile);
      setSelectedFile(null);
    } catch (error) {
      // Error handled by AuthContext toast
    }
  };

  const getInitials = (name, email) => {
    if (name) {
      return name.split(' ').map(n => n[0]).join('').toUpperCase().slice(0, 2);
    }
    return email ? email[0].toUpperCase() : 'U';
  };

  return (
    <Layout
      title="My Profile"
      description="User Profile Page"
    >
      <header className="hero hero--primary">
        <div className="container">
          <h1 className="hero__title">My Profile</h1>
          <p className="hero__subtitle">Manage your identity and personal details</p>
        </div>
      </header>
      <main>
        <section className="container margin-vert--lg">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <div className={styles.profileCard}>
                {/* Avatar Section */}
                <div className={styles.avatarSection}>
                  {previewUrl ? (
                    <img
                      src={previewUrl}
                      alt="Profile"
                      className={styles.profilePicture}
                    />
                  ) : (
                    <div className={styles.avatarPlaceholder}>
                      {getInitials(user?.name, user?.email)}
                    </div>
                  )}
                  <div className={styles.avatarLabel}>Avatar</div>

                  {/* Profile Picture Upload */}
                  <div className={styles.uploadSection}>
                    <input
                      type="file"
                      id="profilePicture"
                      accept="image/*"
                      onChange={handleFileChange}
                      className={styles.fileInput}
                    />
                    <label htmlFor="profilePicture" className="button button--secondary button--sm">
                      Choose Picture
                    </label>
                    {selectedFile && (
                      <button
                        className="button button--primary button--sm margin-left--sm"
                        onClick={handleUploadPicture}
                        disabled={authOperationLoading}
                      >
                        {authOperationLoading ? 'Uploading...' : 'Upload'}
                      </button>
                    )}
                  </div>
                </div>

                {/* Profile Information */}
                <div className={styles.profileContent}>
                  <h2>Your Profile Information</h2>

                  {isEditing ? (
                    <div className={styles.editForm}>
                      <div className={styles.formGroup}>
                        <label className={styles.label}>Full Name</label>
                        <input
                          type="text"
                          value={editedName}
                          onChange={(e) => setEditedName(e.target.value)}
                          className={styles.input}
                          disabled={authOperationLoading}
                          placeholder="Enter your full name"
                        />
                      </div>

                      <div className={styles.formGroup}>
                        <label className={styles.label}>Email</label>
                        <input
                          type="email"
                          value={user?.email || ''}
                          className={styles.input}
                          disabled
                        />
                        <div className={styles.hint}>Email cannot be changed</div>
                      </div>

                      <div className={styles.formGroup}>
                        <label className={styles.label}>Account Type</label>
                        <input
                          type="text"
                          value={user?.authMethod || 'Email'}
                          className={styles.input}
                          disabled
                        />
                      </div>

                      <div className={styles.formGroup}>
                        <label className={styles.label}>Background Info (Optional)</label>
                        <textarea
                          value={editedBio}
                          onChange={(e) => setEditedBio(e.target.value)}
                          className={styles.textarea}
                          disabled={authOperationLoading}
                          placeholder="Tell us a bit about yourself..."
                          rows={4}
                        />
                      </div>

                      <div className={styles.buttonGroup}>
                        <button
                          className="button button--primary margin-right--sm"
                          onClick={handleSave}
                          disabled={authOperationLoading}
                        >
                          {authOperationLoading ? 'Saving...' : 'Save Changes'}
                        </button>
                        <button
                          className="button button--secondary"
                          onClick={handleCancel}
                          disabled={authOperationLoading}
                        >
                          Cancel
                        </button>
                      </div>
                    </div>
                  ) : (
                    <div className={styles.viewMode}>
                      <div className={styles.infoRow}>
                        <span className={styles.infoLabel}>Full Name</span>
                        <span className={styles.infoValue}>{user?.name || 'Not set'}</span>
                      </div>

                      <div className={styles.infoRow}>
                        <span className={styles.infoLabel}>Email</span>
                        <span className={styles.infoValue}>{user?.email}</span>
                      </div>

                      <div className={styles.infoRow}>
                        <span className={styles.infoLabel}>Account Type</span>
                        <span className={styles.infoValue}>
                          <span className={styles.accountTypeBadge}>
                            {user?.authMethod === 'google' ? (
                              <>
                                <svg className={styles.accountIcon} viewBox="0 0 24 24" fill="currentColor">
                                  <path d="M12.545,10.239v3.821h5.445c-0.712,2.315-2.647,3.972-5.445,3.972c-3.332,0-6.033-2.701-6.033-6.032s2.701-6.032,6.033-6.032c1.498,0,2.866,0.549,3.921,1.453l2.814-2.814C17.503,2.988,15.139,2,12.545,2C7.021,2,2.543,6.477,2.543,12s4.478,10,10.002,10c8.396,0,10.249-7.85,9.426-11.748L12.545,10.239z"/>
                                </svg>
                                Google
                              </>
                            ) : (
                              <>
                                <svg className={styles.accountIcon} viewBox="0 0 24 24" fill="currentColor">
                                  <path d="M20 4H4c-1.1 0-1.99.9-1.99 2L2 18c0 1.1.9 2 2 2h16c1.1 0 2-.9 2-2V6c0-1.1-.9-2-2-2zm0 4l-8 5-8-5V6l8 5 8-5v2z"/>
                                </svg>
                                Email
                              </>
                            )}
                          </span>
                        </span>
                      </div>

                      <div className={styles.infoRow}>
                        <span className={styles.infoLabel}>Background Info</span>
                        <span className={styles.infoValue}>{user?.bio || 'Not set'}</span>
                      </div>

                      <button
                        className="button button--primary"
                        onClick={() => setIsEditing(true)}
                      >
                        Edit Profile
                      </button>
                    </div>
                  )}
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
};

export default withAuthProtection(Profile);