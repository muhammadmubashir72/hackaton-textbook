import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../context/AuthContext';
import { useHistory } from '@docusaurus/router';
import withAuthProtection from '../components/Auth/withAuthProtection';
import styles from './Dashboard.module.css';

const Dashboard = () => {
  const { user } = useAuth();
  const history = useHistory();
  const [recentActivity, setRecentActivity] = useState([]);
  const [progress, setProgress] = useState({
    chaptersRead: 0,
    totalChapters: 0,
    progressPercentage: 0,
    lastOpenedChapter: null,
  });

  // Mock data - in production, this would come from your backend API
  useEffect(() => {
    // Simulating data fetch
    const mockProgress = {
      chaptersRead: 3,
      totalChapters: 12,
      progressPercentage: 25,
      lastOpenedChapter: {
        id: 'chapter-3',
        title: 'Introduction to Sensors',
        path: '/docs/tutorial-basics/create-a-document',
      },
    };

    const mockActivity = [
      { id: 1, action: 'Completed', item: 'Chapter 2: Robotics Basics', time: '2 hours ago' },
      { id: 2, action: 'Started', item: 'Chapter 3: Introduction to Sensors', time: '5 hours ago' },
      { id: 3, action: 'Viewed', item: 'Video: Robot Kinematics', time: '1 day ago' },
      { id: 4, action: 'Completed', item: 'Chapter 1: Embodied Intelligence', time: '2 days ago' },
    ];

    setProgress(mockProgress);
    setRecentActivity(mockActivity);
  }, []);

  const handleResumeLearning = () => {
    if (progress.lastOpenedChapter) {
      history.push(progress.lastOpenedChapter.path);
    }
  };

  const getGreeting = () => {
    const hour = new Date().getHours();
    if (hour < 12) return 'Good morning';
    if (hour < 18) return 'Good afternoon';
    return 'Good evening';
  };

  return (
    <Layout
      title="My Progress"
      description="User Dashboard Page"
    >
      <header className="hero hero--primary" style={{ padding: '2rem 0 1.5rem' }}>
        <div className="container">
          <h1 className="hero__title" style={{ fontSize: '2.5rem' }}>
            {getGreeting()}, {user?.name || user?.email?.split('@')[0]} 👋
          </h1>
          <p className="hero__subtitle">Track your learning progress and achievements</p>
        </div>
      </header>
      <main>
        <section className="container margin-vert--lg">
          {/* Welcome Section */}
          <div className={styles.welcomeSection}>
            <div className={styles.welcomeCard}>
              <div className={styles.welcomeContent}>
                <div className={styles.welcomeIcon}>
                  <svg viewBox="0 0 24 24" fill="currentColor">
                    <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-2 15l-5-5 1.41-1.41L10 14.17l7.59-7.59L19 8l-9 9z"/>
                  </svg>
                </div>
                <div className={styles.welcomeText}>
                  <h2>Welcome back!</h2>
                  <p>You're making great progress in your robotics journey. Keep up the excellent work!</p>
                </div>
              </div>
            </div>
          </div>

          {/* Stats Grid */}
          <div className={styles.statsGrid}>
            <div className={styles.statCard}>
              <div className={styles.statHeader}>
                <div className={styles.statIcon}>
                  <svg viewBox="0 0 24 24" fill="currentColor">
                    <path d="M19 3H5c-1.1 0-2 .9-2 2v14c0 1.1.9 2 2 2h14c1.1 0 2-.9 2-2V5c0-1.1-.9-2-2-2zm-5 14H7v-2h7v2zm3-4H7v-2h10v2zm0-4H7V7h10v2z"/>
                  </svg>
                </div>
                <div className={styles.statTitle}>Chapters</div>
              </div>
              <div className={styles.statBody}>
                <div className={styles.statValue}>{progress.chaptersRead}</div>
                <div className={styles.statLabel}>Completed</div>
              </div>
              <div className={styles.statFooter}>
                <div className={styles.progressContainer}>
                  <div
                    className={styles.progressBar}
                    style={{ width: `${(progress.chaptersRead / progress.totalChapters) * 100}%` }}
                  ></div>
                </div>
                <div className={styles.progressText}>
                  {progress.chaptersRead} of {progress.totalChapters}
                </div>
              </div>
            </div>

            <div className={styles.statCard}>
              <div className={styles.statHeader}>
                <div className={styles.statIcon}>
                  <svg viewBox="0 0 24 24" fill="currentColor">
                    <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm0 18c-4.41 0-8-3.59-8-8s3.59-8 8-8 8 3.59 8 8-3.59 8-8 8zm.5-13H11v6l5.25 3.15.75-1.23-4.5-2.67z"/>
                  </svg>
                </div>
                <div className={styles.statTitle}>Progress</div>
              </div>
              <div className={styles.statBody}>
                <div className={styles.statValue}>{progress.progressPercentage}%</div>
                <div className={styles.statLabel}>Complete</div>
              </div>
              <div className={styles.statFooter}>
                <div className={styles.progressRing}>
                  <svg viewBox="0 0 36 36" className={styles.progressSvg}>
                    <path
                      d="M18 2.0845 a 15.9155 15.9155 0 0 1 0 31.831 a 15.9155 15.9155 0 0 1 0 -31.831"
                      fill="none"
                      stroke="#e6e6e6"
                      strokeWidth="3"
                    />
                    <path
                      d="M18 2.0845 a 15.9155 15.9155 0 0 1 0 31.831 a 15.9155 15.9155 0 0 1 0 -31.831"
                      fill="none"
                      stroke="#3b82f6"
                      strokeWidth="3"
                      strokeDasharray={`${progress.progressPercentage}, 100`}
                    />
                  </svg>
                  <span className={styles.ringValue}>{progress.progressPercentage}%</span>
                </div>
              </div>
            </div>

            <div className={styles.statCard}>
              <div className={styles.statHeader}>
                <div className={styles.statIcon}>
                  <svg viewBox="0 0 24 24" fill="currentColor">
                    <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-2 15l-5-5 1.41-1.41L10 14.17l7.59-7.59L19 8l-9 9z"/>
                  </svg>
                </div>
                <div className={styles.statTitle}>Total</div>
              </div>
              <div className={styles.statBody}>
                <div className={styles.statValue}>{progress.totalChapters}</div>
                <div className={styles.statLabel}>Chapters</div>
              </div>
              <div className={styles.statFooter}>
                <div className={styles.streakContainer}>
                  <div className={styles.streakIcon}>
                    🔥
                  </div>
                  <div className={styles.streakText}>
                    7 day streak
                  </div>
                </div>
              </div>
            </div>

            <div className={styles.statCard}>
              <div className={styles.statHeader}>
                <div className={styles.statIcon}>
                  <svg viewBox="0 0 24 24" fill="currentColor">
                    <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm3.5 6L12 10.5 8.5 8 12 5.5 15.5 8zM8.5 16L12 13.5 15.5 16 12 18.5 8.5 16z"/>
                  </svg>
                </div>
                <div className={styles.statTitle}>Achievements</div>
              </div>
              <div className={styles.statBody}>
                <div className={styles.statValue}>12</div>
                <div className={styles.statLabel}>Badges earned</div>
              </div>
              <div className={styles.statFooter}>
                <div className={styles.achievementGrid}>
                  <div className={styles.achievementBadge}>🏆</div>
                  <div className={styles.achievementBadge}>⭐</div>
                  <div className={styles.achievementBadge}>🎯</div>
                </div>
              </div>
            </div>
          </div>

          <div className="row margin-vert--lg">
            {/* Continue Learning Card */}
            <div className="col col--12">
              <div className={styles.card}>
                <div className={styles.cardHeader}>
                  <h2 className={styles.cardTitle}>Continue Learning</h2>
                  <div className={styles.cardActions}>
                    <a href="/docs/introduction/intro" className="button button--outline button--sm">
                      View All
                    </a>
                  </div>
                </div>

                {progress.lastOpenedChapter ? (
                  <div className={styles.continueCard}>
                    <div className={styles.chapterInfo}>
                      <div className={styles.chapterIcon}>
                        <svg viewBox="0 0 24 24" fill="currentColor">
                          <path d="M18 2H6c-1.1 0-2 .9-2 2v16c0 1.1.9 2 2 2h12c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zM6 4h5v8l-2.5-1.5L6 12V4z"/>
                        </svg>
                      </div>
                      <div className={styles.chapterDetails}>
                        <div className={styles.chapterTitle}>{progress.lastOpenedChapter.title}</div>
                        <div className={styles.chapterSubtitle}>Last opened today</div>
                      </div>
                    </div>
                    <div className={styles.chapterActions}>
                      <button
                        className={`button button--primary ${styles.resumeButton}`}
                        onClick={handleResumeLearning}
                      >
                        Resume Learning
                      </button>
                      <button className="button button--outline">
                        View Details
                      </button>
                    </div>
                  </div>
                ) : (
                  <div className={styles.emptyState}>
                    <div className={styles.emptyIcon}>
                      <svg viewBox="0 0 24 24" fill="currentColor">
                        <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm1 15h-2v-6h2v6zm0-8h-2V7h2v2z"/>
                      </svg>
                    </div>
                    <p className={styles.emptyText}>You haven't started any chapter yet</p>
                    <a href="/docs/introduction/intro" className="button button--primary">
                      Start Learning
                    </a>
                  </div>
                )}
              </div>
            </div>
          </div>

          <div className="row">
            <div className="col col--8">
              {/* Recent Activity */}
              <div className={styles.card}>
                <div className={styles.cardHeader}>
                  <h2 className={styles.cardTitle}>Recent Activity</h2>
                  <div className={styles.cardActions}>
                    <button className="button button--outline button--sm">
                      View All
                    </button>
                  </div>
                </div>

                {recentActivity.length > 0 ? (
                  <div className={styles.activityList}>
                    {recentActivity.map((activity) => (
                      <div key={activity.id} className={styles.activityItem}>
                        <div className={styles.activityIcon}>
                          <svg viewBox="0 0 24 24" fill="currentColor">
                            <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-2 15l-5-5 1.41-1.41L10 14.17l7.59-7.59L19 8l-9 9z"/>
                          </svg>
                        </div>
                        <div className={styles.activityDetails}>
                          <div className={styles.activityAction}>{activity.action}</div>
                          <div className={styles.activityItem}>{activity.item}</div>
                        </div>
                        <div className={styles.activityTime}>{activity.time}</div>
                      </div>
                    ))}
                  </div>
                ) : (
                  <div className={styles.emptySmall}>
                    <p className={styles.emptyText}>No recent activity</p>
                  </div>
                )}
              </div>
            </div>

            <div className="col col--4">
              {/* Quick Actions */}
              <div className={styles.card}>
                <div className={styles.cardHeader}>
                  <h2 className={styles.cardTitle}>Quick Actions</h2>
                </div>
                <div className={styles.quickActions}>
                  <a href="/docs/introduction/intro" className={`${styles.quickAction} ${styles.actionPrimary}`}>
                    <div className={styles.actionIcon}>📚</div>
                    <div className={styles.actionContent}>
                      <div className={styles.actionTitle}>Start New Chapter</div>
                      <div className={styles.actionDesc}>Begin learning a new topic</div>
                    </div>
                  </a>
                  <a href="/dashboard" className={`${styles.quickAction} ${styles.actionSecondary}`}>
                    <div className={styles.actionIcon}>📊</div>
                    <div className={styles.actionContent}>
                      <div className={styles.actionTitle}>View Analytics</div>
                      <div className={styles.actionDesc}>See your learning insights</div>
                    </div>
                  </a>
                  <a href="/profile" className={`${styles.quickAction} ${styles.actionTertiary}`}>
                    <div className={styles.actionIcon}>👤</div>
                    <div className={styles.actionContent}>
                      <div className={styles.actionTitle}>Update Profile</div>
                      <div className={styles.actionDesc}>Manage your account</div>
                    </div>
                  </a>
                </div>
              </div>

              {/* Upcoming Goals */}
              <div className={styles.card}>
                <div className={styles.cardHeader}>
                  <h2 className={styles.cardTitle}>Upcoming Goals</h2>
                </div>
                <div className={styles.goalsList}>
                  <div className={styles.goalItem}>
                    <div className={styles.goalProgress}>
                      <div className={styles.goalBar}>
                        <div className={styles.goalFill} style={{ width: '65%' }}></div>
                      </div>
                    </div>
                    <div className={styles.goalContent}>
                      <div className={styles.goalTitle}>Complete Week 3</div>
                      <div className={styles.goalDesc}>2 of 3 chapters remaining</div>
                    </div>
                  </div>
                  <div className={styles.goalItem}>
                    <div className={styles.goalProgress}>
                      <div className={styles.goalBar}>
                        <div className={styles.goalFill} style={{ width: '30%' }}></div>
                      </div>
                    </div>
                    <div className={styles.goalContent}>
                      <div className={styles.goalTitle}>Week 4 Challenge</div>
                      <div className={styles.goalDesc}>Due in 5 days</div>
                    </div>
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

export default withAuthProtection(Dashboard);