import React, { useState, useRef, useEffect, useCallback } from 'react';
import './ChatKit.css';

// Backend wiring function - using Vercel API route as proxy
const connectToBackend = async (message, context) => {
  try {
    // Use Vercel edge function as a CORS proxy
    const response = await fetch('/api/proxy', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        targetUrl: 'https://mubashirsaeedi-ai-book-backend.hf.space/query',
        query: message,
        top_k: 3
      })
    });

    console.log('Response status:', response.status);
    console.log('Response ok?', response.ok);

    if (!response.ok) {
      const errorText = await response.text();
      console.error('Backend API error response:', errorText);
      throw new Error(`Backend API error: ${response.status} - ${errorText}`);
    }

    const data = await response.json();
    console.log('Response data:', data);

    // Check if relevant content was found in the book
    if (!data.answer || data.answer.includes("couldn't find any relevant information") ||
        data.answer.includes("I cannot find this information")) {
      return "I cannot find this information in the Physical AI and Humanoid Robotics textbook. The answer to your question is not available in the book content.";
    }

    // Return the answer from the backend
    return data.answer;
  } catch (error) {
    console.error('Error connecting to backend:', error);
    console.error('Full error details:', {
      message: error.message,
      stack: error.stack,
      name: error.name
    });

    // Check for specific network/CORS errors
    if (error instanceof TypeError && error.message.includes('fetch')) {
      return "I'm having trouble reaching the backend server. This might be due to a network connection issue or CORS policy. Please check your connection and try again.";
    }

    // In case of any error, return a message that clearly states the limitation
    return `I'm having trouble accessing the book content right now. Error: ${error.message}. Please try again in a moment. If the issue persists, the information you're looking for might not be available in the Physical AI and Humanoid Robotics textbook.`;
  }
};

// Create a Vercel API route for proxying requests
// Create the API route file
const createApiRoute = () => {
  // This will be created as a separate file, but for now let's create it
};

// Generate response from Qdrant context
const generateResponseFromContext = (userMessage, results, context = {}) => {
  // Combine all retrieved content sections
  const combinedContext = results.map(r => r.content).join('\n\n');
  const sources = results.map(r => r.source).filter(s => s).slice(0, 3); // Get top 3 sources

  // If user had selected text context, incorporate it
  if (context.selectedText) {
    return `Based on the selected text "${context.selectedText.substring(0, 100)}${context.selectedText.length > 100 ? '...' : ''}", ` +
           `and the book content, here's what I found: ${combinedContext.substring(0, 1000)}${combinedContext.length > 1000 ? '...' : ''}` +
           `${sources.length > 0 ? `\n\nSource: ${sources.join(', ')}` : ''}`;
  }

  // General response from book content
  return `${combinedContext.substring(0, 1000)}${combinedContext.length > 1000 ? '...' : ''}` +
         `${sources.length > 0 ? `\n\nSource: ${sources.join(', ')}` : ''}`;
};

// Mock response generator (kept for fallback)
const generateMockResponse = (userMessage, context = {}) => {
  const lowerMsg = userMessage.toLowerCase();

  // Context-aware responses based on selected text
  if (context.selectedText) {
    return `Regarding "${context.selectedText.substring(0, 60)}${context.selectedText.length > 60 ? '...' : ''}", ` +
           `this is an interesting concept in Physical AI and Robotics. ` +
           `In the context of your selection, this relates to how intelligent systems can interact with the physical world. ` +
           `Would you like me to explain more about this specific topic?`;
  }

  if (lowerMsg.includes('hello') || lowerMsg.includes('hi') || lowerMsg.includes('hey')) {
    return "Hello! I'm your AI assistant for Physical AI and Robotics. How can I help you today?";
  } else if (lowerMsg.includes('robot') || lowerMsg.includes('robotics')) {
    return "Robotics is an interdisciplinary field combining engineering and computer science to design, construct, and operate robots. Physical AI integrates robotic systems with artificial intelligence for autonomous decision-making and learning. Would you like to know more about a specific aspect?";
  } else if (lowerMsg.includes('ai') || lowerMsg.includes('artificial intelligence')) {
    return "Artificial Intelligence in robotics enables machines to perceive, reason, and act autonomously. Physical AI specifically focuses on AI systems that interact with the physical world through robotic bodies. What aspect interests you most?";
  } else if (lowerMsg.includes('learn') || lowerMsg.includes('education') || lowerMsg.includes('study')) {
    return "Great! Learning about AI and Robotics involves understanding machine learning, computer vision, sensor integration, and control systems. I can help explain concepts at any level. What would you like to explore?";
  } else if (lowerMsg.includes('thank')) {
    return "You're welcome! Is there anything else I can help you with regarding AI or Robotics?";
  } else if (lowerMsg.includes('bye') || lowerMsg.includes('goodbye')) {
    return "Goodbye! Feel free to return if you have more questions about AI, Robotics, or Physical AI concepts.";
  } else {
    const responses = [
      "That's an interesting question about AI and Robotics! Physical AI systems are fascinating because they combine artificial intelligence with real-world interaction. Could you elaborate on what specifically you'd like to know?",
      "Physical AI and Robotics is a vast field! Some key areas include perception, learning, control, and interaction. What particular aspect would you like to dive deeper into?",
      "I'm here to help you understand AI and Robotics concepts! From neural networks to robotic kinematics, I can explain various topics. What's on your mind?",
      "The intersection of AI and Robotics is incredibly rich! From autonomous systems to human-robot interaction, there's so much to explore. What captures your interest most?",
      "Excellent question! In the context of Physical AI, this involves understanding how intelligent systems can interact with and learn from the physical world. What specific application are you thinking about?"
    ];
    return responses[Math.floor(Math.random() * responses.length)];
  }
};

const ChatKit = ({ config = {} }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: "Hello! I'm your AI assistant for Physical AI and Robotics. How can I help you today?",
      sender: 'ai',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
    }
  ]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState(null);
  const [isSelectionMode, setIsSelectionMode] = useState(false);
  const [selectionPosition, setSelectionPosition] = useState({ x: 0, y: 0 });

  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);
  const chatContainerRef = useRef(null);

  const defaultConfig = {
    title: 'AI Assistant',
    placeholder: 'Ask about AI, Robotics...',
    ...config
  };

  // Function to handle text selection
  const handleTextSelection = useCallback(() => {
    const selection = window.getSelection();
    const text = selection.toString().trim();

    if (text) {
      // Get selection coordinates for potential floating button
      if (selection.rangeCount > 0) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        setSelectionPosition({ x: rect.right, y: rect.top });
      }

      setSelectedText(text);
      setIsSelectionMode(true);

      // Clear selection after capturing
      setTimeout(() => {
        selection.removeAllRanges();
      }, 100);
    } else {
      setSelectedText(null);
      setIsSelectionMode(false);
    }
  }, []);

  // Add event listeners for text selection
  useEffect(() => {
    const handleMouseUp = () => {
      setTimeout(handleTextSelection, 0);
    };

    const handleClick = (event) => {
      // If user clicks outside the chat container and not on selected text, clear selection
      if (chatContainerRef.current && !chatContainerRef.current.contains(event.target)) {
        const selection = window.getSelection();
        if (selection.toString().trim() === '') {
          setSelectedText(null);
          setIsSelectionMode(false);
        }
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('click', handleClick);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('click', handleClick);
    };
  }, [handleTextSelection]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async () => {
    if (!inputText.trim()) return;

    const userMessage = {
      id: Date.now(),
      text: inputText,
      sender: 'user',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
    };

    setMessages(prev => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);

    try {
      // Prepare context with selected text
      const context = {
        selectedText: selectedText,
        timestamp: Date.now()
      };

      // Use the backend wiring function (currently mocked)
      const aiResponse = await connectToBackend(inputText, context);

      const responseMessage = {
        id: Date.now() + 1,
        text: aiResponse,
        sender: 'ai',
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      };

      setMessages(prev => [...prev, responseMessage]);

      // Clear selection after sending message
      setSelectedText(null);
      setIsSelectionMode(false);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: "I'm having trouble processing your request right now. Please try again.",
        sender: 'ai',
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  };

  const closeChat = () => {
    setIsOpen(false);
    // Clear selection when chat is closed
    setSelectedText(null);
    setIsSelectionMode(false);
  };

  return (
    <>
      {/* Chat Toggle Button */}
      <button
        className={`chatkit-toggle ${isOpen ? 'open' : ''} ${isSelectionMode ? 'selection-mode' : ''}`}
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        {isSelectionMode ? (
          <div className="chatkit-selection-indicator">
            <span className="selection-dot"></span>
          </div>
        ) : isOpen ? (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M18 6L6 18M6 6L18 18" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        ) : (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M8 12H16M12 8V16M21 12C21 16.9706 16.9706 21 12 21C7.02944 21 3 16.9706 3 12C3 7.02944 7.02944 3 12 3C16.9706 3 21 7.02944 21 12Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        )}
      </button>

      {/* Chat Container */}
      {isOpen && (
        <div ref={chatContainerRef} className="chatkit-container">
          <div className="chatkit-header">
            <div className="chatkit-header-content">
              <h3>{defaultConfig.title}</h3>
              <div className="chatkit-header-actions">
                {isSelectionMode && (
                  <span className="chatkit-selection-badge">
                    <span className="selection-dot"></span> Selected text mode
                  </span>
                )}
                <button
                  className="chatkit-close"
                  onClick={closeChat}
                  aria-label="Close chat"
                >
                  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <path d="M18 6L6 18M6 6L18 18" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                  </svg>
                </button>
              </div>
            </div>
          </div>

          <div className="chatkit-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`chatkit-message ${message.sender}`}
              >
                <div className="chatkit-message-bubble">
                  <div className="chatkit-message-text">{message.text}</div>
                  <div className="chatkit-message-time">{message.timestamp}</div>
                </div>
              </div>
            ))}
            {isLoading && (
              <div className="chatkit-message ai">
                <div className="chatkit-message-bubble">
                  <div className="chatkit-typing-indicator">
                    <div className="dot"></div>
                    <div className="dot"></div>
                    <div className="dot"></div>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {isSelectionMode && (
            <div className="chatkit-context-info">
              <div className="context-preview">
                <span className="context-label">Context:</span>
                <span className="context-text">"{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</span>
              </div>
            </div>
          )}

          <div className="chatkit-input-area">
            <textarea
              ref={inputRef}
              value={inputText}
              onChange={(e) => setInputText(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={isSelectionMode ? `${defaultConfig.placeholder} (with selected text context)` : defaultConfig.placeholder}
              className="chatkit-input"
              rows="1"
              aria-label="Type your message"
            />
            <button
              onClick={handleSendMessage}
              disabled={!inputText.trim() || isLoading}
              className="chatkit-send-button"
              aria-label="Send message"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M22 2L11 13M22 2L15 22L11 13M11 13L2 9L22 2" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatKit;