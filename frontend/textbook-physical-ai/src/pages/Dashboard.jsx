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
      <header className="hero hero--primary">
        <div className="container">
          <h1 className="hero__title">
            {getGreeting()}, {user?.name || user?.email?.split('@')[0]} 👋
          </h1>
          <p className="hero__subtitle">Track your learning progress and resume where you left off</p>
        </div>
      </header>
      <main>
        <section className="container margin-vert--lg">
          <div className="row">
            <div className="col col--12">
              {/* Quick Stats */}
              <div className={styles.statsGrid}>
                <div className={styles.statCard}>
                  <div className={styles.statIcon}>
                    <svg viewBox="0 0 24 24" fill="currentColor">
                      <path d="M19 3H5c-1.1 0-2 .9-2 2v14c0 1.1.9 2 2 2h14c1.1 0 2-.9 2-2V5c0-1.1-.9-2-2-2zm-5 14H7v-2h7v2zm3-4H7v-2h10v2zm0-4H7V7h10v2z"/>
                    </svg>
                  </div>
                  <div className={styles.statContent}>
                    <div className={styles.statValue}>{progress.chaptersRead}</div>
                    <div className={styles.statLabel}>Chapters Completed</div>
                  </div>
                </div>

                <div className={styles.statCard}>
                  <div className={styles.statIcon}>
                    <svg viewBox="0 0 24 24" fill="currentColor">
                      <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm0 18c-4.41 0-8-3.59-8-8s3.59-8 8-8 8 3.59 8 8-3.59 8-8 8zm.5-13H11v6l5.25 3.15.75-1.23-4.5-2.67z"/>
                    </svg>
                  </div>
                  <div className={styles.statContent}>
                    <div className={styles.statValue}>{progress.progressPercentage}%</div>
                    <div className={styles.statLabel}>Overall Progress</div>
                  </div>
                </div>

                <div className={styles.statCard}>
                  <div className={styles.statIcon}>
                    <svg viewBox="0 0 24 24" fill="currentColor">
                      <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-2 15l-5-5 1.41-1.41L10 14.17l7.59-7.59L19 8l-9 9z"/>
                    </svg>
                  </div>
                  <div className={styles.statContent}>
                    <div className={styles.statValue}>{progress.totalChapters}</div>
                    <div className={styles.statLabel}>Total Chapters</div>
                  </div>
                </div>
              </div>
            </div>
          </div>

          <div className="row margin-vert--lg">
            {/* Last Opened Chapter */}
            <div className="col col--8 col--offset-2">
              <div className={styles.card}>
                <h2 className={styles.cardTitle}>Continue Learning</h2>
                {progress.lastOpenedChapter ? (
                  <>
                    <div className={styles.chapterInfo}>
                      <svg className={styles.chapterIcon} viewBox="0 0 24 24" fill="currentColor">
                        <path d="M18 2H6c-1.1 0-2 .9-2 2v16c0 1.1.9 2 2 2h12c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zM6 4h5v8l-2.5-1.5L6 12V4z"/>
                      </svg>
                      <div className={styles.chapterDetails}>
                        <div className={styles.chapterTitle}>{progress.lastOpenedChapter.title}</div>
                        <div className={styles.chapterSubtitle}>Last opened today</div>
                      </div>
                    </div>
                    <button
                      className={`button button--primary ${styles.resumeButton}`}
                      onClick={handleResumeLearning}
                    >
                      Resume Learning
                    </button>
                  </>
                ) : (
                  <div className={styles.emptyState}>
                    <svg className={styles.emptyIcon} viewBox="0 0 24 24" fill="currentColor">
                      <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm1 15h-2v-6h2v6zm0-8h-2V7h2v2z"/>
                    </svg>
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
            {/* Recent Activity */}
            <div className="col col--8 col--offset-2">
              <div className={styles.card}>
                <h2 className={styles.cardTitle}>Recent Activity</h2>
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
                  <p className={styles.emptyText}>No recent activity</p>
                )}
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
};

export default withAuthProtection(Dashboard);