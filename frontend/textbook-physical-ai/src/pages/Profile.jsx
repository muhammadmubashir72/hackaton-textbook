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
      <header className="hero hero--primary" style={{ padding: '2rem 0 1.5rem' }}>
        <div className="container">
          <h1 className="hero__title" style={{ marginBottom: '0.25rem', fontSize: '2.5rem' }}>My Profile</h1>
          <p className="hero__subtitle">Manage your identity and personal details</p>
        </div>
      </header>
      <main>
        <section className={styles.profileContainer}>
          <div className={styles.profileGrid}>
            {/* Sidebar - Avatar Card */}
            <div className={styles.sidebar}>
              <div className={styles.profileCard}>
                <div className={styles.cardHeader}>
                  <h3 className={styles.cardTitle}>Profile Picture</h3>
                </div>

                {/* Avatar Section */}
                <div className={styles.avatarSection}>
                  <div className={styles.avatarWrapper}>
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
                  </div>

                  <div className={styles.avatarActions}>
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
                </div>
              </div>

              {/* Account Overview Card */}
              <div className={styles.profileCard}>
                <div className={styles.cardHeader}>
                  <h3 className={styles.cardTitle}>Account Overview</h3>
                </div>
                <div className={styles.accountOverview}>
                  <div className={styles.overviewItem}>
                    <div className={styles.overviewLabel}>Member Since</div>
                    <div className={styles.overviewValue}>
                      {user?.createdAt ? new Date(user.createdAt).toLocaleDateString() : 'Unknown'}
                    </div>
                  </div>
                  <div className={styles.overviewItem}>
                    <div className={styles.overviewLabel}>Account Status</div>
                    <div className={styles.overviewValue}>
                      <span className={styles.statusBadge}>Active</span>
                    </div>
                  </div>
                  <div className={styles.overviewItem}>
                    <div className={styles.overviewLabel}>Account Type</div>
                    <div className={styles.overviewValue}>
                      <span className={styles.accountTypeBadge}>
                        {user?.authMethod === 'google' ? 'Google' : 'Email'}
                      </span>
                    </div>
                  </div>
                </div>
              </div>
            </div>

            {/* Main Content - Profile Info Card */}
            <div className={styles.mainContent}>
              <div className={styles.profileCard}>
                <div className={styles.cardHeader}>
                  <h2 className={styles.cardTitle}>Profile Information</h2>
                  <div className={styles.cardActions}>
                    {!isEditing && (
                      <button
                        className="button button--outline button--sm"
                        onClick={() => setIsEditing(true)}
                      >
                        Edit Profile
                      </button>
                    )}
                  </div>
                </div>

                <div className={styles.cardContent}>
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
                        <label className={styles.label}>Email Address</label>
                        <div className={styles.emailWrapper}>
                          <input
                            type="email"
                            value={user?.email || ''}
                            className={styles.input}
                            disabled
                          />
                          <span className={styles.emailHint}>Email cannot be changed</span>
                        </div>
                      </div>

                      <div className={styles.formGroup}>
                        <label className={styles.label}>Account Type</label>
                        <div className={styles.accountTypeWrapper}>
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
                        </div>
                      </div>

                      <div className={styles.formGroup}>
                        <label className={styles.label}>About Me</label>
                        <textarea
                          value={editedBio}
                          onChange={(e) => setEditedBio(e.target.value)}
                          className={styles.textarea}
                          disabled={authOperationLoading}
                          placeholder="Tell us a bit about yourself..."
                          rows={6}
                        />
                      </div>

                      <div className={styles.formActions}>
                        <button
                          className="button button--primary"
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
                      <div className={styles.infoGrid}>
                        <div className={styles.infoRow}>
                          <span className={styles.infoLabel}>Full Name</span>
                          <span className={styles.infoValue}>{user?.name || 'Not set'}</span>
                        </div>

                        <div className={styles.infoRow}>
                          <span className={styles.infoLabel}>Email Address</span>
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
                          <span className={styles.infoLabel}>About Me</span>
                          <span className={styles.infoValue}>{user?.bio || 'Not set'}</span>
                        </div>
                      </div>

                      <div className={styles.viewActions}>
                        <button
                          className="button button--primary"
                          onClick={() => setIsEditing(true)}
                        >
                          Edit Profile
                        </button>
                      </div>
                    </div>
                  )}
                </div>
              </div>

              {/* Security Card */}
              <div className={styles.profileCard}>
                <div className={styles.cardHeader}>
                  <h3 className={styles.cardTitle}>Security</h3>
                </div>
                <div className={styles.securityOptions}>
                  <div className={styles.securityItem}>
                    <div className={styles.securityInfo}>
                      <div className={styles.securityTitle}>Password</div>
                      <div className={styles.securityDescription}>
                        {user?.authMethod === 'google'
                          ? 'Managed by Google. Visit Google account settings to change.'
                          : 'Last changed recently'}
                      </div>
                    </div>
                    {user?.authMethod !== 'google' && (
                      <button className="button button--outline button--sm">
                        Change Password
                      </button>
                    )}
                  </div>
                  <div className={styles.securityItem}>
                    <div className={styles.securityInfo}>
                      <div className={styles.securityTitle}>Two-Factor Authentication</div>
                      <div className={styles.securityDescription}>
                        Add an extra layer of security to your account
                      </div>
                    </div>
                    <button className="button button--outline button--sm">
                      Set up 2FA
                    </button>
                  </div>
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