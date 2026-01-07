import React, { useState, useEffect, useRef } from 'react';
import { HelpCircle, Languages } from 'lucide-react';
import './TextSelectionButtons.css';

const TextSelectionButtons = ({ onAskCallback }) => {
  const [isVisible, setIsVisible] = useState(false);
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [selectedText, setSelectedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [showResult, setShowResult] = useState(false);
  const [resultText, setResultText] = useState('');
  const [resultType, setResultType] = useState(''); // 'ask' or 'urdu'
  const containerRef = useRef(null);

  // Function to get selected text and its position
  const handleSelection = () => {
    const selection = window.getSelection();
    const text = selection.toString().trim();

    if (text && selection.rangeCount > 0) {
      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();

      // Calculate position near the selected text but ensure it's within viewport
      const x = Math.min(rect.left + rect.width / 2 - 60, window.innerWidth - 120); // Center the button and prevent overflow
      const y = Math.min(rect.top - 10, window.innerHeight - 60); // Position above the selection, prevent overflow

      setPosition({ x, y });
      setSelectedText(text);
      setIsVisible(true);
    } else {
      setIsVisible(false);
      setShowResult(false);
    }
  };

  // Handle document clicks to hide buttons when clicking elsewhere
  const handleClickOutside = (event) => {
    if (containerRef.current && !containerRef.current.contains(event.target)) {
      setIsVisible(false);
      setShowResult(false);
    }
  };

  // Add event listeners
  useEffect(() => {
    const handleMouseUp = () => {
      setTimeout(handleSelection, 0); // Use timeout to ensure selection is complete
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('click', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('click', handleClickOutside);
    };
  }, []);

  const handleAsk = async () => {
    if (!selectedText) return;

    // Dispatch a custom event to notify the ChatKit component
    const event = new CustomEvent('textSelectionAsk', {
      detail: { selectedText }
    });
    window.dispatchEvent(event);

    // Hide the buttons after triggering the event
    setIsVisible(false);
    setShowResult(false);
  };

  const handleUrdu = async () => {
    if (!selectedText) return;

    // Dispatch a custom event for Urdu translation
    const event = new CustomEvent('textSelectionUrdu', {
      detail: { selectedText }
    });
    window.dispatchEvent(event);

    // Hide the buttons after triggering the event
    setIsVisible(false);
    setShowResult(false);
  };

  // Don't render if not visible
  if (!isVisible) return null;

  return (
    <div ref={containerRef} className="text-selection-container">
      <div
        className="text-selection-buttons"
        style={{
          top: position.y,
          left: position.x
        }}
      >
        <button
          className="selection-button ask-button"
          onClick={handleAsk}
          disabled={isLoading}
          aria-label="Ask about selected text"
          title="Ask about selected text"
        >
          <HelpCircle className="button-icon-svg" size={18} strokeWidth={2} />
          <span>Ask AI</span>
        </button>
        <button
          className="selection-button urdu-button"
          onClick={handleUrdu}
          disabled={isLoading}
          aria-label="Translate to Urdu"
          title="Translate to Urdu"
        >
          <Languages className="button-icon-svg" size={18} strokeWidth={2} />
          <span>اردو </span>
        </button>
      </div>
    </div>
  );
};

export default TextSelectionButtons;