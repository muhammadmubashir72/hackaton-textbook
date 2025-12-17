import React, { useEffect, useRef, useState, useCallback } from 'react';
import { createParticleSystem } from './utils/particle-system';
import { prefersReducedMotion } from './utils/accessibility';
import styles from './FuturisticRobot.module.css';

const FuturisticRobot = ({
  size = 500,
  speed = 1,
  particleCountInner = 12,
  particleCountOuter = 24,
  colors = {
    body: '#00FFFF', // cyan blue
    core: '#FFA500', // orange
    particles: '#FFA500', // orange for inner particles
    outerParticles: '#00FFFF' // cyan for outer particles
  },
  enableReducedMotion: propReducedMotion = null
}) => {
  const canvasRef = useRef(null);
  const animationRef = useRef(null);
  const particleSystemRef = useRef(null);
  const [isReducedMotion, setIsReducedMotion] = useState(() => {
    // Use prop if provided, otherwise check system preference
    return propReducedMotion !== null ? propReducedMotion : prefersReducedMotion();
  });

  // Initialize particle system
  const initializeParticleSystem = useCallback(() => {
    if (isReducedMotion) return;

    const centerX = size / 2;
    const centerY = size / 2;
    const scale = size / 300; // Scale factor based on size

    // Create configuration for the particle system
    const config = {
      centerX,
      centerY,
      orbits: [
        {
          radius: 60 * scale, // Inner orbit (larger for more visible circular motion)
          count: particleCountInner,
          speed: 0.03 * speed, // Increased speed for better visibility
          color: colors.particles,
          clockwisePattern: 'alternate',
          size: 5
        },
        {
          radius: 120 * scale, // Outer orbit (larger for more visible circular motion)
          count: particleCountOuter,
          speed: 0.02 * speed, // Increased speed for better visibility
          color: colors.outerParticles,
          clockwisePattern: 'reverse-alternate',
          size: 4
        }
      ]
    };

    particleSystemRef.current = createParticleSystem(config);
  }, [size, speed, particleCountInner, particleCountOuter, colors, isReducedMotion]);

  // Animation loop
  useEffect(() => {
    if (isReducedMotion || !particleSystemRef.current) return;

    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');

    // Set canvas size
    canvas.width = size;
    canvas.height = size;

    let lastTime = 0;

    const animate = (currentTime) => {
      // Calculate deltaTime in seconds, with fallback for first frame
      const deltaTime = lastTime ? (currentTime - lastTime) / 1000 : 1/60;
      lastTime = currentTime;

      // Clear canvas with dark space background
      ctx.fillStyle = '#000011'; // Dark space blue
      ctx.fillRect(0, 0, size, size);

      // Draw the robot body
      drawRobot(ctx, size / 2, size / 2, size * 0.1, colors);

      // Update and render particles
      particleSystemRef.current.update(deltaTime);
      particleSystemRef.current.render(ctx);

      animationRef.current = requestAnimationFrame(animate);
    };

    animationRef.current = requestAnimationFrame(animate);

    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
    };
  }, [isReducedMotion, colors, size]);

  // Initialize particle system when dependencies change
  useEffect(() => {
    if (!isReducedMotion) {
      initializeParticleSystem();
    }

    return () => {
      if (particleSystemRef.current) {
        particleSystemRef.current.clear();
      }
    };
  }, [initializeParticleSystem, isReducedMotion]);

  // Handle system reduced motion preference changes
  useEffect(() => {
    if (propReducedMotion !== null) return; // Use prop value if provided

    const handleReducedMotionChange = (reducedMotion) => {
      setIsReducedMotion(reducedMotion);
    };

    // Initial check
    handleReducedMotionChange(prefersReducedMotion());

    // Set up listener for changes
    let removeListener = () => {};
    if (typeof window !== 'undefined' && window.matchMedia) {
      const mediaQuery = window.matchMedia('(prefers-reduced-motion: reduce)');
      const handleChange = (e) => handleReducedMotionChange(e.matches);
      mediaQuery.addEventListener('change', handleChange);
      removeListener = () => mediaQuery.removeEventListener('change', handleChange);
    }

    return removeListener;
  }, [propReducedMotion]);

  const drawRobot = (ctx, x, y, scale, colors) => {
    // Draw orbital path guides to make particle movement more apparent
    ctx.strokeStyle = 'rgba(0, 255, 255, 0.1)'; // Subtle cyan for inner orbit
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(x, y, 60 * (size / 300), 0, Math.PI * 2); // Inner orbit path (updated to match new size)
    ctx.stroke();

    ctx.strokeStyle = 'rgba(255, 165, 0, 0.1)'; // Subtle orange for outer orbit
    ctx.beginPath();
    ctx.arc(x, y, 120 * (size / 300), 0, Math.PI * 2); // Outer orbit path (updated to match new size)
    ctx.stroke();

    // Calculate animation values for more dynamic movement
    const currentTime = Date.now() / 1000; // Convert to seconds
    const handSwingAngle = Math.sin(currentTime * 3) * 0.4; // Faster swing for more noticeable animation
    const headBobAngle = Math.sin(currentTime * 2.5) * 0.1; // Gentle head bobbing
    const bodyGlowPulse = 15 + Math.sin(currentTime * 4) * 5; // Pulsing glow
    const corePulse = 20 + Math.sin(currentTime * 5) * 8; // Pulsing chest core

    // Draw robot head with animated movement and pulsing glow
    ctx.shadowColor = colors.body;
    ctx.shadowBlur = bodyGlowPulse;
    ctx.fillStyle = colors.body;

    // Apply head bobbing motion
    const headOffsetX = x + Math.sin(currentTime * 1.5) * 2;
    const headOffsetY = y - scale * 2 + Math.sin(currentTime * 2) * 1;

    ctx.beginPath();
    ctx.arc(headOffsetX, headOffsetY, scale, 0, Math.PI * 2);
    ctx.fill();

    // Draw robot body with pulsing glow
    ctx.shadowBlur = bodyGlowPulse - 5; // Slightly less than head
    ctx.beginPath();
    ctx.roundRect ? ctx.roundRect(x - scale, y - scale, scale * 2, scale * 2, 10) : ctx.rect(x - scale, y - scale, scale * 2, scale * 2);
    ctx.fill();

    // Draw animated orange glowing chest core with pulsing effect
    ctx.shadowColor = colors.core;
    ctx.shadowBlur = corePulse;
    ctx.fillStyle = colors.core;
    ctx.beginPath();
    ctx.arc(x, y, scale * 0.5, 0, Math.PI * 2);
    ctx.fill();

    // Draw animated orange glowing eyes with enhanced glow and blinking effect
    const eyeBlink = Math.sin(currentTime * 8) > 0.9 ? 0.1 : 0.2; // Occasional blink
    ctx.shadowBlur = 15 + Math.sin(currentTime * 6) * 3; // Pulsing eye glow

    ctx.beginPath();
    ctx.arc(x - scale * 0.3, headOffsetY + 0.2 * scale, scale * eyeBlink, 0, Math.PI * 2);
    ctx.fill();
    ctx.beginPath();
    ctx.arc(x + scale * 0.3, headOffsetY + 0.2 * scale, scale * eyeBlink, 0, Math.PI * 2);
    ctx.fill();

    // Draw animated left arm with more dynamic movement
    ctx.shadowColor = colors.body;
    ctx.shadowBlur = 10;
    ctx.strokeStyle = colors.body;
    ctx.lineWidth = scale * 0.6;
    ctx.lineCap = 'round';

    // Left arm - animated with more distinct circular motion
    const leftArmAngle = -Math.PI/4 + handSwingAngle + currentTime * 1.5; // More pronounced circular motion
    const leftArmEndX = x - scale * 3 * Math.cos(leftArmAngle);
    const leftArmEndY = y - scale + scale * 3 * Math.sin(leftArmAngle); // Extended circular motion for more visible movement

    ctx.beginPath();
    ctx.moveTo(x - scale, y - scale * 0.5); // Start from body
    ctx.lineTo(leftArmEndX, leftArmEndY); // End at hand position
    ctx.stroke();

    // Draw animated left hand with pulsing effect
    const leftHandSize = scale * 0.5 + Math.sin(currentTime * 7) * 0.1;
    ctx.beginPath();
    ctx.arc(leftArmEndX, leftArmEndY, leftHandSize, 0, Math.PI * 2);
    ctx.fillStyle = colors.body;
    ctx.fill();

    // Draw animated right arm with counter-clockwise circular motion
    const rightArmAngle = -Math.PI/4 - handSwingAngle - currentTime * 1.5; // Counter-clockwise circular motion
    const rightArmEndX = x + scale * 3 * Math.cos(rightArmAngle);
    const rightArmEndY = y - scale + scale * 3 * Math.sin(rightArmAngle); // Extended circular motion for more visible movement

    ctx.beginPath();
    ctx.moveTo(x + scale, y - scale * 0.5); // Start from body
    ctx.lineTo(rightArmEndX, rightArmEndY); // End at hand position
    ctx.stroke();

    // Draw animated right hand with pulsing effect
    const rightHandSize = scale * 0.5 + Math.sin(currentTime * 7 + Math.PI) * 0.1;
    ctx.beginPath();
    ctx.arc(rightArmEndX, rightArmEndY, rightHandSize, 0, Math.PI * 2);
    ctx.fill();

    // Draw animated left leg with subtle movement
    ctx.shadowColor = colors.body;
    ctx.shadowBlur = 8;
    ctx.strokeStyle = colors.body;
    ctx.lineWidth = scale * 0.55;
    ctx.lineCap = 'round';

    // Left leg - animated with subtle up/down movement
    const leftLegAngle = Math.PI/6 + Math.sin(currentTime * 2.5) * 0.15; // Subtle leg swing
    const leftLegEndX = x - scale * 0.6 + Math.sin(leftLegAngle) * scale * 1.8;
    const leftLegEndY = y + scale * 2 + Math.cos(leftLegAngle) * scale * 1.8; // Down from body

    ctx.beginPath();
    ctx.moveTo(x - scale * 0.3, y + scale); // Start from lower body
    ctx.lineTo(leftLegEndX, leftLegEndY); // End at foot position
    ctx.stroke();

    // Draw animated left foot
    ctx.beginPath();
    ctx.arc(leftLegEndX, leftLegEndY, scale * 0.6, 0, Math.PI * 2);
    ctx.fillStyle = colors.body;
    ctx.fill();

    // Draw animated right leg with counter movement
    const rightLegAngle = Math.PI/6 - Math.sin(currentTime * 2.5) * 0.15; // Counter swing to left leg
    const rightLegEndX = x + scale * 0.6 + Math.sin(rightLegAngle) * scale * 1.8;
    const rightLegEndY = y + scale * 2 + Math.cos(rightLegAngle) * scale * 1.8; // Down from body

    ctx.beginPath();
    ctx.moveTo(x + scale * 0.3, y + scale); // Start from lower body
    ctx.lineTo(rightLegEndX, rightLegEndY); // End at foot position
    ctx.stroke();

    // Draw animated right foot
    ctx.beginPath();
    ctx.arc(rightLegEndX, rightLegEndY, scale * 0.6, 0, Math.PI * 2);
    ctx.fill();

    // Reset shadow
    ctx.shadowBlur = 0;
  };

  // Static version for reduced motion
  if (isReducedMotion) {
    return (
      <div className={styles.futuristicRobotContainer}>
        <div className={styles.staticRobot} style={{ width: size, height: size, backgroundColor: '#000011' }}>
          <div className={styles.staticRobotBody} style={{ color: colors.body }}>
            <div className={styles.robotHead} style={{ backgroundColor: colors.body }}></div>
            <div className={styles.robotBody} style={{ backgroundColor: colors.body }}></div>
            <div className={styles.robotCore} style={{ backgroundColor: colors.core }}></div>
            <div className={styles.robotLeftArm} style={{ backgroundColor: colors.body }}></div>
            <div className={styles.robotRightArm} style={{ backgroundColor: colors.body }}></div>
            <div className={styles.robotLeftLeg} style={{ backgroundColor: colors.body }}></div>
            <div className={styles.robotRightLeg} style={{ backgroundColor: colors.body }}></div>
            <div className={styles.orbitalRings}>
              {Array.from({ length: 5 }).map((_, i) => (
                <div key={i} className={styles.orbitalRing} style={{
                  width: `${60 + i * 15}%`,
                  height: `${60 + i * 15}%`,
                  borderColor: i % 2 === 0 ? colors.particles : colors.outerParticles
                }}></div>
              ))}
            </div>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.futuristicRobotContainer}>
      <canvas
        ref={canvasRef}
        className={styles.orbitalAnimation}
        style={{ width: size, height: size }}
        aria-label="Futuristic Robot Visualization"
        role="img"
      />
    </div>
  );
};

export default FuturisticRobot;