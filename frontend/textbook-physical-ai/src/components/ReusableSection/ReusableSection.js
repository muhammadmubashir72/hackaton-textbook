import React from 'react';
import clsx from 'clsx';
import { motion } from 'framer-motion';
import styles from './ReusableSection.module.css';

const ReusableSection = ({
  title,
  subtitle,
  children,
  className = '',
  backgroundColor = 'light',
  animationDelay = 0,
  ...props
}) => {
  const sectionClasses = clsx(
    styles.reusableSection,
    styles[`background${backgroundColor.charAt(0).toUpperCase() + backgroundColor.slice(1)}`],
    className
  );

  return (
    <motion.section
      className={sectionClasses}
      initial={{ opacity: 0, y: 20 }}
      whileInView={{ opacity: 1, y: 0 }}
      viewport={{ once: true }}
      transition={{ duration: 0.6, delay: animationDelay }}
      {...props}
    >
      <div className="container">
        {(title || subtitle) && (
          <div className={styles.sectionHeader}>
            {title && <h2 className={styles.sectionTitle}>{title}</h2>}
            {subtitle && <p className={styles.sectionSubtitle}>{subtitle}</p>}
          </div>
        )}
        <div className={styles.sectionContent}>
          {children}
        </div>
      </div>
    </motion.section>
  );
};

export default ReusableSection;