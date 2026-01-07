/**
 * DOM utilities for handling text translation while preserving elements
 */

/**
 * Get all text nodes within an element, excluding code blocks, pre tags, and links
 * @param {HTMLElement} element - The element to search within
 * @returns {Array} - Array of text nodes that should be translated
 */
export function getTextNodesForTranslation(element) {
  const textNodes = [];
  const walker = document.createTreeWalker(
    element,
    NodeFilter.SHOW_TEXT,
    {
      acceptNode: function(node) {
        // Check if the parent element is one we want to exclude
        const parentElement = node.parentElement;
        if (
          parentElement &&
          (
            parentElement.tagName === 'CODE' ||
            parentElement.tagName === 'PRE' ||
            parentElement.tagName === 'A' ||
            parentElement.tagName === 'CODESPAN' ||
            parentElement.closest('code') ||
            parentElement.closest('pre') ||
            parentElement.closest('a') ||
            parentElement.classList.contains('code-block') ||
            parentElement.classList.contains('codeLine') ||
            parentElement.classList.contains('token') ||
            parentElement.tagName === 'MARK' // might be used for highlighting
          )
        ) {
          return NodeFilter.FILTER_REJECT; // Skip this node and its children
        }
        // Only include text nodes that have actual content
        if (node.nodeValue && node.nodeValue.trim().length > 0) {
          return NodeFilter.FILTER_ACCEPT;
        }
        return NodeFilter.FILTER_REJECT;
      }
    }
  );

  let node;
  while (node = walker.nextNode()) {
    textNodes.push(node);
  }

  return textNodes;
}

/**
 * Get the original text content from an element while preserving structure
 * @param {HTMLElement} element - The element to extract content from
 * @returns {Object} - Object containing original content and structure info
 */
export function extractOriginalContent(element) {
  // Clone the element to preserve original structure
  const clone = element.cloneNode(true);

  // Get all text nodes that would be translated
  const textNodes = getTextNodesForTranslation(clone);

  // Store the original text content
  const originalTexts = textNodes.map(node => node.nodeValue);

  return {
    element: clone,
    textNodes: textNodes,
    originalTexts: originalTexts,
    structure: element.innerHTML // Preserve the original HTML structure
  };
}

/**
 * Replace text content in an element while preserving structure
 * @param {HTMLElement} element - The element to update
 * @param {Array} originalTexts - Array of original text values
 * @param {Array} translatedTexts - Array of translated text values
 */
export function replaceTextContent(element, originalTexts, translatedTexts) {
  if (!element || !originalTexts || !translatedTexts) {
    return;
  }

  // Get the current text nodes that should be translated
  const currentTextNodes = getTextNodesForTranslation(element);

  // Update each text node with the corresponding translated text
  for (let i = 0; i < Math.min(currentTextNodes.length, translatedTexts.length); i++) {
    if (currentTextNodes[i]) {
      currentTextNodes[i].nodeValue = translatedTexts[i];
    }
  }
}

/**
 * Check if an element contains translatable content
 * @param {HTMLElement} element - The element to check
 * @returns {boolean} - True if the element has content that can be translated
 */
export function hasTranslatableContent(element) {
  if (!element) return false;

  const textNodes = getTextNodesForTranslation(element);
  return textNodes.length > 0;
}

/**
 * Preserve code blocks, links, and other non-translatable elements
 * @param {HTMLElement} element - The element to preserve structure for
 * @returns {Array} - Array of preserved elements with their positions
 */
export function preserveNonTranslatableElements(element) {
  const preservedElements = [];

  // Find all elements that should be preserved
  const codeBlocks = element.querySelectorAll('code, pre');
  const links = element.querySelectorAll('a');

  // Store code blocks
  codeBlocks.forEach((el, index) => {
    preservedElements.push({
      type: 'code',
      index,
      position: Array.from(element.children).indexOf(el),
      content: el.innerHTML,
      element: el
    });
  });

  // Store links
  links.forEach((el, index) => {
    preservedElements.push({
      type: 'link',
      index,
      position: Array.from(element.children).indexOf(el),
      content: el.innerHTML,
      href: el.getAttribute('href'),
      element: el
    });
  });

  return preservedElements;
}