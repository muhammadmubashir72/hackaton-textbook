import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { motion } from 'framer-motion';
import styles from './ReusableCard.module.css';

const ReusableCard = ({
  title,
  description,
  icon,
  link,
  linkText,
  animationDelay = 0,
  className = '',
  children,
  ...props
}) => {
  return (
    <motion.div
      className={clsx(styles.reusableCard, className)}
      initial={{ opacity: 0, y: 30 }}
      whileInView={{ opacity: 1, y: 0 }}
      viewport={{ once: true }}
      transition={{ duration: 0.5, delay: animationDelay }}
      whileHover={{ y: -5 }}
      {...props}
    >
      {icon && <div className={styles.cardIcon}>{icon}</div>}
      {title && <h3 className={styles.cardTitle}>{title}</h3>}
      {description && <p className={styles.cardDescription}>{description}</p>}
      {children}
      {(link && linkText) && (
        <div className={styles.cardAction}>
          <Link className="button button--primary button--sm" to={link}>
            {linkText}
          </Link>
        </div>
      )}
    </motion.div>
  );
};

export default ReusableCard;