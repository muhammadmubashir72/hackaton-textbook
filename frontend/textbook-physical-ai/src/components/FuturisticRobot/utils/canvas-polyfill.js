/**
 * Canvas Polyfill for basic functionality
 * This file provides fallbacks for older browsers that may not support all canvas features
 */

// Check if canvas is supported
export const isCanvasSupported = () => {
  const canvas = document.createElement('canvas');
  return !!(canvas.getContext && canvas.getContext('2d'));
};

// Basic canvas feature detection
export const getCanvasFeatures = () => {
  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d');

  return {
    canvas: !!canvas,
    canvasText: !!(ctx && ctx.fillText),
    canvasImageData: !!(ctx && ctx.getImageData),
    canvasShadow: !!(ctx && ctx.shadowColor !== undefined),
    canvasToDataURL: !!(canvas.toDataURL),
    canvasToBlob: !!(canvas.toBlob)
  };
};

// Fallback for requestAnimationFrame if needed
export const requestAnimationFrame = (() => {
  return window.requestAnimationFrame ||
    window.webkitRequestAnimationFrame ||
    window.mozRequestAnimationFrame ||
    window.oRequestAnimationFrame ||
    window.msRequestAnimationFrame ||
    function(callback) {
      return window.setTimeout(callback, 1000 / 60);
    };
})();

// Fallback for cancelAnimationFrame if needed
export const cancelAnimationFrame = (() => {
  return window.cancelAnimationFrame ||
    window.webkitCancelAnimationFrame ||
    window.mozCancelAnimationFrame ||
    window.oCancelAnimationFrame ||
    window.msCancelAnimationFrame ||
    function(id) {
      window.clearTimeout(id);
    };
})();

// Export all as default
const CanvasPolyfill = {
  isCanvasSupported,
  getCanvasFeatures,
  requestAnimationFrame,
  cancelAnimationFrame
};

export default CanvasPolyfill;