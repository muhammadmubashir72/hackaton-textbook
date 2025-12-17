/**
 * Animation utilities for the Futuristic Robot component
 */

/**
 * Calculate position on a circular orbit
 * @param {number} centerX - X coordinate of orbit center
 * @param {number} centerY - Y coordinate of orbit center
 * @param {number} radius - Radius of the orbit
 * @param {number} angle - Current angle in radians
 * @returns {Object} Position object with x and y coordinates
 */
export const calculateOrbitalPosition = (centerX, centerY, radius, angle) => {
  return {
    x: centerX + Math.cos(angle) * radius,
    y: centerY + Math.sin(angle) * radius
  };
};

/**
 * Normalize angle to be within 0 to 2π range
 * @param {number} angle - Angle in radians
 * @returns {number} Normalized angle
 */
export const normalizeAngle = (angle) => {
  while (angle < 0) {
    angle += 2 * Math.PI;
  }
  while (angle >= 2 * Math.PI) {
    angle -= 2 * Math.PI;
  }
  return angle;
};

/**
 * Calculate distance between two points
 * @param {number} x1 - X coordinate of first point
 * @param {number} y1 - Y coordinate of first point
 * @param {number} x2 - X coordinate of second point
 * @param {number} y2 - Y coordinate of second point
 * @returns {number} Distance between points
 */
export const calculateDistance = (x1, y1, x2, y2) => {
  return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
};

/**
 * Create smooth animation using requestAnimationFrame
 * @param {Function} updateCallback - Function to call on each frame
 * @param {Function} renderCallback - Function to call for rendering
 * @returns {Function} Function to stop the animation
 */
export const createAnimationLoop = (updateCallback, renderCallback) => {
  let animationId;

  const loop = () => {
    updateCallback();
    renderCallback();
    animationId = requestAnimationFrame(loop);
  };

  loop();

  return () => {
    if (animationId) {
      cancelAnimationFrame(animationId);
    }
  };
};

/**
 * Generate orbital path points for visualization
 * @param {number} centerX - X coordinate of orbit center
 * @param {number} centerY - Y coordinate of orbit center
 * @param {number} radius - Radius of the orbit
 * @param {number} numPoints - Number of points to generate
 * @returns {Array} Array of position objects
 */
export const generateOrbitalPath = (centerX, centerY, radius, numPoints = 36) => {
  const points = [];
  for (let i = 0; i < numPoints; i++) {
    const angle = (i / numPoints) * 2 * Math.PI;
    points.push(calculateOrbitalPosition(centerX, centerY, radius, angle));
  }
  return points;
};

/**
 * Calculate orbital speed based on distance from center (simulating Kepler's laws)
 * @param {number} distance - Distance from center
 * @param {number} baseSpeed - Base speed factor
 * @returns {number} Adjusted speed
 */
export const calculateOrbitalSpeed = (distance, baseSpeed = 1) => {
  // Inverse relationship: farther = slower
  return baseSpeed / Math.sqrt(distance);
};