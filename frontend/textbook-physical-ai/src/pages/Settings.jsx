import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../context/AuthContext';
import withAuthProtection from '../components/Auth/withAuthProtection';
import styles from './Settings.module.css';

const Settings = () => {
  const { user, logout } = useAuth();
  const [activeSection, setActiveSection] = useState('account');
  const [showChangePassword, setShowChangePassword] = useState(false);
  const [showDeleteConfirm, setShowDeleteConfirm] = useState(false);
  const [showLogoutAllConfirm, setShowLogoutAllConfirm] = useState(false);

  const [passwordForm, setPasswordForm] = useState({
    currentPassword: '',
    newPassword: '',
    confirmPassword: '',
  });

  const handlePasswordChange = (e) => {
    setPasswordForm({
      ...passwordForm,
      [e.target.name]: e.target.value,
    });
  };

  const handlePasswordSubmit = async (e) => {
    e.preventDefault();
    // TODO: Implement password change API call
    console.log('Password change requested:', passwordForm);
    setShowChangePassword(false);
    setPasswordForm({ currentPassword: '', newPassword: '', confirmPassword: '' });
  };

  const handleDeleteAccount = async () => {
    // TODO: Implement delete account API call
    console.log('Delete account requested');
    await logout();
  };

  const handleLogoutAllDevices = async () => {
    // TODO: Implement logout all devices API call
    console.log('Logout all devices requested');
    setShowLogoutAllConfirm(false);
    await logout();
  };

  const renderAccountSection = () => (
    <div className={styles.section}>
      <h3 className={styles.sectionTitle}>Account Security</h3>

      {/* Change Password Section */}
      <div className={styles.settingCard}>
        <div className={styles.settingHeader}>
          <div className={styles.settingInfo}>
            <h4 className={styles.settingTitle}>Change Password</h4>
            <p className={styles.settingDescription}>
              {user?.authMethod === 'google'
                ? 'Password is managed by Google. Visit your Google account settings to change it.'
                : 'Update your password to keep your account secure.'
              }
            </p>
          </div>
          {user?.authMethod !== 'google' && (
            <button
              className="button button--secondary button--sm"
              onClick={() => setShowChangePassword(!showChangePassword)}
            >
              {showChangePassword ? 'Cancel' : 'Change'}
            </button>
          )}
        </div>

        {showChangePassword && user?.authMethod !== 'google' && (
          <form className={styles.passwordForm} onSubmit={handlePasswordSubmit}>
            <div className={styles.formGroup}>
              <label className={styles.label}>Current Password</label>
              <input
                type="password"
                name="currentPassword"
                value={passwordForm.currentPassword}
                onChange={handlePasswordChange}
                className={styles.input}
                placeholder="Enter current password"
                required
              />
            </div>
            <div className={styles.formGroup}>
              <label className={styles.label}>New Password</label>
              <input
                type="password"
                name="newPassword"
                value={passwordForm.newPassword}
                onChange={handlePasswordChange}
                className={styles.input}
                placeholder="Enter new password"
                required
                minLength={8}
              />
            </div>
            <div className={styles.formGroup}>
              <label className={styles.label}>Confirm New Password</label>
              <input
                type="password"
                name="confirmPassword"
                value={passwordForm.confirmPassword}
                onChange={handlePasswordChange}
                className={styles.input}
                placeholder="Confirm new password"
                required
                minLength={8}
              />
            </div>
            <div className={styles.buttonGroup}>
              <button type="submit" className="button button--primary">
                Update Password
              </button>
              <button
                type="button"
                className="button button--secondary"
                onClick={() => {
                  setShowChangePassword(false);
                  setPasswordForm({ currentPassword: '', newPassword: '', confirmPassword: '' });
                }}
              >
                Cancel
              </button>
            </div>
          </form>
        )}
      </div>

      {/* Connected Accounts */}
      <div className={styles.settingCard}>
        <div className={styles.settingHeader}>
          <div className={styles.settingInfo}>
            <h4 className={styles.settingTitle}>Connected Accounts</h4>
            <p className={styles.settingDescription}>
              Manage your connected accounts and OAuth providers.
            </p>
          </div>
        </div>
        <div className={styles.connectedAccounts}>
          <div className={styles.accountItem}>
            <div className={styles.accountIcon}>
              <svg viewBox="0 0 24 24" fill="currentColor">
                <path d="M20 4H4c-1.1 0-1.99.9-1.99 2L2 18c0 1.1.9 2 2 2h16c1.1 0 2-.9 2-2V6c0-1.1-.9-2-2-2zm0 4l-8 5-8-5V6l8 5 8-5v2z"/>
              </svg>
            </div>
            <div className={styles.accountDetails}>
              <div className={styles.accountName}>Email</div>
              <div className={styles.accountEmail}>{user?.email}</div>
            </div>
            <div className={styles.accountStatus}>
              {user?.authMethod === 'email' ? (
                <span className={`${styles.statusBadge} ${styles.statusConnected}`}>Primary</span>
              ) : (
                <span className={`${styles.statusBadge} ${styles.statusConnected}`}>Connected</span>
              )}
            </div>
          </div>
          {user?.authMethod === 'google' && (
            <div className={styles.accountItem}>
              <div className={styles.accountIcon}>
                <svg viewBox="0 0 24 24" fill="currentColor">
                  <path d="M12.545,10.239v3.821h5.445c-0.712,2.315-2.647,3.972-5.445,3.972c-3.332,0-6.033-2.701-6.033-6.032s2.701-6.032,6.033-6.032c1.498,0,2.866,0.549,3.921,1.453l2.814-2.814C17.503,2.988,15.139,2,12.545,2C7.021,2,2.543,6.477,2.543,12s4.478,10,10.002,10c8.396,0,10.249-7.85,9.426-11.748L12.545,10.239z"/>
                </svg>
              </div>
              <div className={styles.accountDetails}>
                <div className={styles.accountName}>Google</div>
                <div className={styles.accountEmail}>{user?.email}</div>
              </div>
              <div className={styles.accountStatus}>
                <span className={`${styles.statusBadge} ${styles.statusConnected}`}>Primary</span>
              </div>
            </div>
          )}
        </div>
      </div>

      {/* Logout All Devices */}
      <div className={styles.settingCard}>
        <div className={styles.settingHeader}>
          <div className={styles.settingInfo}>
            <h4 className={styles.settingTitle}>Sign Out All Devices</h4>
            <p className={styles.settingDescription}>
              Sign out from all devices where you're currently logged in.
            </p>
          </div>
          <button
            className="button button--secondary button--sm"
            onClick={() => setShowLogoutAllConfirm(true)}
          >
            Sign Out All
          </button>
        </div>
      </div>

      {/* Delete Account */}
      <div className={`${styles.settingCard} ${styles.dangerZone}`}>
        <div className={styles.settingHeader}>
          <div className={styles.settingInfo}>
            <h4 className={`${styles.settingTitle} ${styles.dangerTitle}`}>Delete Account</h4>
            <p className={styles.settingDescription}>
              Permanently delete your account and all associated data. This action cannot be undone.
            </p>
          </div>
          <button
            className="button button--danger button--sm"
            onClick={() => setShowDeleteConfirm(true)}
          >
            Delete Account
          </button>
        </div>
      </div>
    </div>
  );

  const renderPreferencesSection = () => (
    <div className={styles.section}>
      <h3 className={styles.sectionTitle}>Preferences</h3>

      <div className={styles.settingCard}>
        <div className={styles.settingHeader}>
          <div className={styles.settingInfo}>
            <h4 className={styles.settingTitle}>Theme</h4>
            <p className={styles.settingDescription}>
              Choose your preferred color theme for the application.
            </p>
          </div>
          <select className={styles.select}>
            <option>Light</option>
            <option>Dark</option>
            <option>System</option>
          </select>
        </div>
      </div>

      <div className={styles.settingCard}>
        <div className={styles.settingHeader}>
          <div className={styles.settingInfo}>
            <h4 className={styles.settingTitle}>Language</h4>
            <p className={styles.settingDescription}>
              Select your preferred language for the interface.
            </p>
          </div>
          <select className={styles.select}>
            <option>English</option>
            <option>Urdu (اردو)</option>
          </select>
        </div>
      </div>

      <div className={styles.settingCard}>
        <div className={styles.settingHeader}>
          <div className={styles.settingInfo}>
            <h4 className={styles.settingTitle}>Email Notifications</h4>
            <p className={styles.settingDescription}>
              Receive email updates about your progress and new content.
            </p>
          </div>
          <label className={styles.toggle}>
            <input type="checkbox" defaultChecked />
            <span className={styles.toggleSlider}></span>
          </label>
        </div>
      </div>
    </div>
  );

  return (
    <Layout
      title="Settings"
      description="User Settings Page"
    >
      <header className="hero hero--primary">
        <div className="container">
          <h1 className="hero__title">Settings</h1>
          <p className="hero__subtitle">Manage your account preferences and security</p>
        </div>
      </header>
      <main>
        <section className="container margin-vert--lg">
          <div className="row">
            <div className="col col--10 col--offset-1">
              <div className={styles.settingsContainer}>
                {/* Sidebar Navigation */}
                <div className={styles.sidebar}>
                  <button
                    className={`${styles.sidebarButton} ${activeSection === 'account' ? styles.active : ''}`}
                    onClick={() => setActiveSection('account')}
                  >
                    <svg className={styles.sidebarIcon} viewBox="0 0 24 24" fill="currentColor">
                      <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z"/>
                    </svg>
                    Account
                  </button>
                  <button
                    className={`${styles.sidebarButton} ${activeSection === 'preferences' ? styles.active : ''}`}
                    onClick={() => setActiveSection('preferences')}
                  >
                    <svg className={styles.sidebarIcon} viewBox="0 0 24 24" fill="currentColor">
                      <path d="M19.14,12.94c0.04-0.3,0.06-0.61,0.06-0.94c0-0.32-0.02-0.64-0.06-0.94l2.03-1.58c0.18-0.14,0.23-0.41,0.12-0.61 l-1.92-3.32c-0.12-0.22-0.37-0.29-0.59-0.22l-2.39,0.96c-0.5-0.38-1.03-0.7-1.62-0.94L14.4,2.81c-0.04-0.24-0.24-0.41-0.48-0.41 h-3.84c-0.24,0-0.43,0.17-0.47,0.41L9.25,5.35C8.66,5.59,8.12,5.92,7.63,6.29L5.24,5.33c-0.22-0.08-0.47,0-0.59,0.22L2.74,8.87 C2.62,9.08,2.66,9.34,2.86,9.48l2.03,1.58C4.84,11.36,4.8,11.69,4.8,12s0.02,0.64,0.06,0.94l-2.03,1.58 c-0.18,0.14-0.23,0.41-0.12,0.61l1.92,3.32c0.12,0.22,0.37,0.29,0.59,0.22l2.39-0.96c0.5,0.38,1.03,0.7,1.62,0.94l0.36,2.54 c0.05,0.24,0.24,0.41,0.48,0.41h3.84c0.24,0,0.44-0.17,0.47-0.41l0.36-2.54c0.59-0.24,1.13-0.56,1.62-0.94l2.39,0.96 c0.22,0.08,0.47,0,0.59-0.22l1.92-3.32c0.12-0.22,0.07-0.47-0.12-0.61L19.14,12.94z M12,15.6c-1.98,0-3.6-1.62-3.6-3.6 s1.62-3.6,3.6-3.6s3.6,1.62,3.6,3.6S13.98,15.6,12,15.6z"/>
                    </svg>
                    Preferences
                  </button>
                </div>

                {/* Content */}
                <div className={styles.content}>
                  {activeSection === 'account' && renderAccountSection()}
                  {activeSection === 'preferences' && renderPreferencesSection()}
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>

      {/* Delete Account Confirmation Modal */}
      {showDeleteConfirm && (
        <div className={styles.modalOverlay}>
          <div className={styles.modal}>
            <div className={styles.modalHeader}>
              <h3 className={styles.modalTitle}>Delete Account</h3>
              <button
                className={styles.modalClose}
                onClick={() => setShowDeleteConfirm(false)}
              >
                ×
              </button>
            </div>
            <div className={styles.modalBody}>
              <p className={styles.modalText}>
                Are you sure you want to delete your account? This action cannot be undone and all your data will be permanently lost.
              </p>
              <div className={styles.modalWarning}>
                <svg className={styles.warningIcon} viewBox="0 0 24 24" fill="currentColor">
                  <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm1 15h-2v-2h2v2zm0-4h-2V7h2v6z"/>
                </svg>
                <span>This action is irreversible</span>
              </div>
            </div>
            <div className={styles.modalFooter}>
              <button
                className="button button--secondary"
                onClick={() => setShowDeleteConfirm(false)}
              >
                Cancel
              </button>
              <button
                className="button button--danger"
                onClick={handleDeleteAccount}
              >
                Delete Account
              </button>
            </div>
          </div>
        </div>
      )}

      {/* Logout All Devices Confirmation Modal */}
      {showLogoutAllConfirm && (
        <div className={styles.modalOverlay}>
          <div className={styles.modal}>
            <div className={styles.modalHeader}>
              <h3 className={styles.modalTitle}>Sign Out All Devices</h3>
              <button
                className={styles.modalClose}
                onClick={() => setShowLogoutAllConfirm(false)}
              >
                ×
              </button>
            </div>
            <div className={styles.modalBody}>
              <p className={styles.modalText}>
                Are you sure you want to sign out from all devices? You'll need to sign in again on each device.
              </p>
            </div>
            <div className={styles.modalFooter}>
              <button
                className="button button--secondary"
                onClick={() => setShowLogoutAllConfirm(false)}
              >
                Cancel
              </button>
              <button
                className="button button--primary"
                onClick={handleLogoutAllDevices}
              >
                Sign Out All
              </button>
            </div>
          </div>
        </div>
      )}
    </Layout>
  );
};

export default withAuthProtection(Settings);
