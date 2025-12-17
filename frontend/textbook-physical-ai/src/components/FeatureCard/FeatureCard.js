import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { motion } from 'framer-motion';
import styles from './FeatureCard.module.css';

const FeatureCard = ({
  title,
  description,
  icon,
  link,
  linkText = 'Learn More',
  animationDelay = 0,
  className = '',
  variant = 'default', // 'default', 'primary', 'secondary'
  ...props
}) => {
  const cardClasses = clsx(
    styles.featureCard,
    styles[`variant${variant.charAt(0).toUpperCase() + variant.slice(1)}`],
    className
  );

  return (
    <motion.div
      className={cardClasses}
      initial={{ opacity: 0, y: 30 }}
      whileInView={{ opacity: 1, y: 0 }}
      viewport={{ once: true }}
      transition={{ duration: 0.5, delay: animationDelay }}
      whileHover={{ y: -5 }}
      {...props}
    >
      {icon && <div className={styles.featureIcon}>{icon}</div>}
      {title && <h3 className={styles.featureTitle}>{title}</h3>}
      {description && <p className={styles.featureDescription}>{description}</p>}
      {(link && linkText) && (
        <div className={styles.featureAction}>
          <Link className="button button--primary button--sm" to={link}>
            {linkText}
          </Link>
        </div>
      )}
    </motion.div>
  );
};

export default FeatureCard;