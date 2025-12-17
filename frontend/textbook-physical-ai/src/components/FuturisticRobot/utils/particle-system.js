/**
 * Particle system utilities for the Futuristic Robot component
 */

/**
 * Create a single particle for the orbital system
 * @param {Object} options - Particle configuration options
 * @param {number} options.centerX - X coordinate of orbit center
 * @param {number} options.centerY - Y coordinate of orbit center
 * @param {number} options.radius - Orbit radius
 * @param {number} options.angle - Initial angle in radians
 * @param {number} options.speed - Movement speed
 * @param {string} options.color - Particle color
 * @param {boolean} options.clockwise - Direction of movement
 * @param {number} options.size - Visual size of particle
 * @returns {Object} Particle object with update and render methods
 */
export const createParticle = ({
  centerX,
  centerY,
  radius,
  angle = 0,
  speed = 0.01,
  color = '#FFA500',
  clockwise = true,
  size = 3
}) => {
  let currentAngle = angle;
  let currentSpeed = speed;
  let currentColor = color;
  let isClockwise = clockwise;
  let particleSize = size;

  return {
    get position() {
      return {
        x: centerX + Math.cos(currentAngle) * radius,
        y: centerY + Math.sin(currentAngle) * radius
      };
    },

    get angle() {
      return currentAngle;
    },

    get color() {
      return currentColor;
    },

    set color(newColor) {
      currentColor = newColor;
    },

    get size() {
      return particleSize;
    },

    set size(newSize) {
      particleSize = newSize;
    },

    update(deltaTime = 1) {
      const direction = isClockwise ? 1 : -1;
      // Increase the speed factor to make animation more visible
      currentAngle += currentSpeed * direction * deltaTime * 60; // Multiply by 60 to normalize with requestAnimationFrame

      // Normalize angle to keep it within 0-2π range
      if (currentAngle >= 2 * Math.PI) {
        currentAngle -= 2 * Math.PI;
      } else if (currentAngle < 0) {
        currentAngle += 2 * Math.PI;
      }
    },

    render(ctx) {
      // Enhanced glow effect for better visibility
      ctx.shadowColor = currentColor;
      ctx.shadowBlur = 20;
      ctx.fillStyle = currentColor;

      ctx.beginPath();
      ctx.arc(this.position.x, this.position.y, particleSize, 0, Math.PI * 2);
      ctx.fill();

      // Add a brighter center for more visibility
      ctx.shadowBlur = 5;
      ctx.fillStyle = '#FFFFFF';
      ctx.beginPath();
      ctx.arc(this.position.x, this.position.y, particleSize * 0.3, 0, Math.PI * 2);
      ctx.fill();

      // Reset shadow
      ctx.shadowBlur = 0;
    },

    setSpeed(newSpeed) {
      currentSpeed = Math.abs(newSpeed);
    },

    setDirection(clockwise) {
      isClockwise = !!clockwise;
    }
  };
};

/**
 * Create a particle system with multiple orbital rings
 * @param {Object} config - System configuration
 * @param {number} config.centerX - X coordinate of system center
 * @param {number} config.centerY - Y coordinate of system center
 * @param {Array} config.orbits - Array of orbit configurations
 * @returns {Object} Particle system object
 */
export const createParticleSystem = (config) => {
  const { centerX, centerY, orbits } = config;
  const particles = [];

  orbits.forEach((orbit, orbitIndex) => {
    const {
      radius,
      count,
      speed,
      color,
      clockwisePattern = 'alternate',
      size = 3
    } = orbit;

    for (let i = 0; i < count; i++) {
      let isClockwise = true;

      switch (clockwisePattern) {
        case 'all-clockwise':
          isClockwise = true;
          break;
        case 'all-counter':
          isClockwise = false;
          break;
        case 'alternate':
          isClockwise = i % 2 === 0;
          break;
        case 'reverse-alternate':
          isClockwise = i % 2 !== 0;
          break;
        default:
          isClockwise = i % 2 === 0;
      }

      const angle = (i / count) * 2 * Math.PI;

      particles.push(createParticle({
        centerX,
        centerY,
        radius,
        angle,
        speed,
        color,
        clockwise: isClockwise,
        size
      }));
    }
  });

  return {
    particles,

    update(deltaTime = 1) {
      particles.forEach(particle => particle.update(deltaTime));
    },

    render(ctx) {
      particles.forEach(particle => particle.render(ctx));
    },

    addParticle(particleConfig) {
      particles.push(createParticle({
        ...particleConfig,
        centerX,
        centerY
      }));
    },

    removeParticle(index) {
      if (index >= 0 && index < particles.length) {
        particles.splice(index, 1);
      }
    },

    getParticle(index) {
      return particles[index];
    },

    getParticleCount() {
      return particles.length;
    },

    clear() {
      particles.length = 0;
    },

    setAllSpeeds(speedMultiplier) {
      particles.forEach(particle => {
        particle.setSpeed(particle.constructor.originalSpeed * speedMultiplier);
      });
    }
  };
};

/**
 * Default configuration for the futuristic robot's particle system
 * @param {number} centerX - Center X coordinate
 * @param {number} centerY - Center Y coordinate
 * @param {number} scale - Scale factor for the system
 * @returns {Object} Default particle system configuration
 */
export const getDefaultParticleSystemConfig = (centerX, centerY, scale = 1) => {
  return {
    centerX,
    centerY,
    orbits: [
      {
        radius: 45 * scale, // Inner orbit (tight orbit around core)
        count: 12,
        speed: 0.02,
        color: '#FFA500', // Orange particles
        clockwisePattern: 'alternate',
        size: 4
      },
      {
        radius: 90 * scale, // Outer orbit (wide orbit around robot)
        count: 24,
        speed: 0.01,
        color: '#00FFFF', // Cyan particles
        clockwisePattern: 'reverse-alternate',
        size: 3
      }
    ]
  };
};