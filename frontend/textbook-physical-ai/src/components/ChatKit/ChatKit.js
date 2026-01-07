import React, { useState, useEffect, useRef, useCallback } from 'react';
import { Send, X, Trash2, Bot, MessageCircle } from 'lucide-react';
import './ChatKit.css';

// Backend wiring function - Connect to backend API
const connectToBackend = async (message, context) => {
  try {
    console.log('Sending request to backend:', message);

    // Use direct backend URL in development, proxy in production
    const isDevelopment = window.location.hostname === 'localhost';
    const backendUrl = isDevelopment
      ? 'http://localhost:8001/query'
      : '/api/proxy';

    const requestBody = isDevelopment
      ? { query: message, top_k: 5 }
      : {
          targetUrl: `${process.env.REACT_APP_BACKEND_URL || 'http://localhost:8001'}/query`,
          query: message,
          top_k: 5
        };

    console.log('Request URL:', backendUrl);
    console.log('Request body:', requestBody);

    const response = await fetch(backendUrl, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody)
    });

    console.log('Response status:', response.status);

    if (!response.ok) {
      const errorText = await response.text();
      console.error('Backend error response:', errorText);
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    const data = await response.json();
    console.log('Backend response:', data);

    if (!data.answer) {
      throw new Error('No answer in response');
    }

    return data.answer;
  } catch (error) {
    console.error('Error connecting to backend:', error);
    throw error; // Re-throw instead of falling back to mock
  }
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
    return "Hello! I'm your AI assistant for **Physical AI and Robotics**. How can I help you today?";
  } else if (lowerMsg.includes('robot') || lowerMsg.includes('robotics')) {
    return "**Robotics** is an interdisciplinary field combining engineering and computer science to design, construct, and operate robots. *Physical AI* integrates robotic systems with artificial intelligence for autonomous decision-making and learning. Would you like to know more about a specific aspect?";
  } else if (lowerMsg.includes('ai') || lowerMsg.includes('artificial intelligence')) {
    return "**Artificial Intelligence** in robotics enables machines to perceive, reason, and act autonomously. *Physical AI* specifically focuses on AI systems that interact with the physical world through robotic bodies. What aspect interests you most?";
  } else if (lowerMsg.includes('learn') || lowerMsg.includes('education') || lowerMsg.includes('study')) {
    return "Great! Learning about AI and Robotics involves understanding `machine learning`, `computer vision`, `sensor integration`, and `control systems`. I can help explain concepts at any level. What would you like to explore?";
  } else if (lowerMsg.includes('thank')) {
    return "You're welcome! Is there anything else I can help you with regarding AI or Robotics?";
  } else if (lowerMsg.includes('bye') || lowerMsg.includes('goodbye')) {
    return "Goodbye! Feel free to return if you have more questions about AI, Robotics, or Physical AI concepts.";
  } else {
    const responses = [
      "That's an interesting question about **AI and Robotics**! Physical AI systems are fascinating because they combine artificial intelligence with real-world interaction. Could you elaborate on what specifically you'd like to know?",
      "**Physical AI and Robotics** is a vast field! Some key areas include *perception*, *learning*, *control*, and *interaction*. What particular aspect would you like to dive deeper into?",
      "I'm here to help you understand **AI and Robotics** concepts! From `neural networks` to `robotic kinematics`, I can explain various topics. What's on your mind?",
      "The intersection of **AI and Robotics** is incredibly rich! From autonomous systems to human-robot interaction, there's so much to explore. What captures your interest most?",
      "Excellent question! In the context of *Physical AI*, this involves understanding how intelligent systems can interact with and learn from the physical world. What specific application are you thinking about?"
    ];
    return responses[Math.floor(Math.random() * responses.length)];
  }
};

// Function to generate quick replies based on context
const generateQuickReplies = (messageText, sender) => {
  if (sender === 'ai') {
    // Generate quick replies for AI responses
    const lowerText = messageText.toLowerCase();

    if (lowerText.includes('robot') || lowerText.includes('robotics')) {
      return ['What are ROS 2 basics?', 'Explain Gazebo simulation', 'Tell me about humanoid robots'];
    } else if (lowerText.includes('ai') || lowerText.includes('artificial intelligence')) {
      return ['How does AI work in robotics?', 'Explain computer vision', 'What is machine learning?'];
    } else if (lowerText.includes('learn') || lowerText.includes('education')) {
      return ['Show me the curriculum', 'What are the prerequisites?', 'How long does it take?'];
    } else if (lowerText.includes('physical ai')) {
      return ['What is embodied intelligence?', 'How do robots learn?', 'Explain sensor integration'];
    } else {
      return ['Tell me more about robotics', 'Explain AI concepts', 'Show learning path'];
    }
  }
  return [];
};

// Function to format message text with basic markdown support
const formatMessageText = (text) => {
  // Convert markdown-style **bold** to <strong>
  let formattedText = text.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');

  // Convert markdown-style *italic* to <em>
  formattedText = formattedText.replace(/\*(.*?)\*/g, '<em>$1</em>');

  // Convert line breaks to <br>
  formattedText = formattedText.replace(/\n/g, '<br>');

  // Convert markdown-style `code` to <code>
  formattedText = formattedText.replace(/`(.*?)`/g, '<code class="chatkit-inline-code">$1</code>');

  return formattedText;
};

const ChatKit = ({ config = {} }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: "👋 **Welcome to AI Robotics Assistant!**\n\nI'm here to help you with:\n- 🤖 **Physical AI & Robotics** concepts\n- 📚 **Textbook content** explanations\n- 🌐 **Urdu translations** of selected text\n\nJust ask me anything or select text and click **Ask** or **Urdu** buttons!",
      sender: 'ai',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
    }
  ]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState(null);
  const [isSelectionMode, setIsSelectionMode] = useState(false);
  const [selectionPosition, setSelectionPosition] = useState({ x: 0, y: 0 });
  const [quickReplies, setQuickReplies] = useState([]);

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

    // Handle the custom event from TextSelectionButtons
    const handleTextSelectionAsk = (event) => {
      const { selectedText } = event.detail;

      // Automatically open the chat when Ask button is clicked
      setIsOpen(true);

      // Set the selected text as context
      setSelectedText(selectedText);
      setIsSelectionMode(true);

      // Add the selected text as a user message
      const userMessage = {
        id: Date.now(),
        text: selectedText,
        sender: 'user',
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      };

      setMessages(prev => [...prev, userMessage]);

      // Process the message with the backend
      setIsLoading(true);

      // Prepare context with selected text
      const context = {
        selectedText: selectedText,
        timestamp: Date.now()
      };

      connectToBackend(selectedText, context)
        .then(aiResponse => {
          const responseMessage = {
            id: Date.now() + 1,
            text: aiResponse,
            sender: 'ai',
            timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
          };

          setMessages(prev => [...prev, responseMessage]);

          // Generate quick replies for the AI response
          const newQuickReplies = generateQuickReplies(aiResponse, 'ai');
          setQuickReplies(newQuickReplies);

          // Clear selection after sending message
          setSelectedText(null);
          setIsSelectionMode(false);
        })
        .catch(error => {
          const errorMessage = {
            id: Date.now() + 1,
            text: "I'm having trouble processing your request right now. Please try again.",
            sender: 'ai',
            timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
          };
          setMessages(prev => [...prev, errorMessage]);
          setQuickReplies([]);
        })
        .finally(() => {
          setIsLoading(false);
        });
    };

    // Handle the custom event for Urdu translation
    const handleTextSelectionUrdu = (event) => {
      const { selectedText } = event.detail;

      // Automatically open the chat when Urdu button is clicked
      setIsOpen(true);

      setSelectedText(selectedText);
      setIsSelectionMode(true);

      // Add the selected text as a user message (show original text)
      const userMessage = {
        id: Date.now(),
        text: selectedText,
        sender: 'user',
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      };

      setMessages(prev => [...prev, userMessage]);

      // Process the translation with the backend
      setIsLoading(true);

      // Use direct backend URL in development, proxy in production
      const isDevelopment = window.location.hostname === 'localhost';
      const translateUrl = isDevelopment
        ? 'http://localhost:8001/translate'
        : '/api/proxy';

      const translateBody = isDevelopment
        ? { text: selectedText, target_language: 'ur' }
        : {
            targetUrl: `${process.env.REACT_APP_BACKEND_URL || 'http://localhost:8001'}/translate`,
            text: selectedText,
            target_language: 'ur'
          };

      console.log('Translation URL:', translateUrl);
      console.log('Translation body:', translateBody);

      // Call translation API
      fetch(translateUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(translateBody)
      })
        .then(response => {
          if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
          }
          return response.json();
        })
        .then(data => {
          const responseMessage = {
            id: Date.now() + 1,
            text: data.translated_text || 'Translation not available',
            sender: 'ai',
            timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
          };

          setMessages(prev => [...prev, responseMessage]);

          // Clear selection after sending message
          setSelectedText(null);
          setIsSelectionMode(false);
        })
        .catch(error => {
          console.error('Translation error:', error);
          const errorMessage = {
            id: Date.now() + 1,
            text: `Translation failed: ${error.message}`,
            sender: 'ai',
            timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
          };
          setMessages(prev => [...prev, errorMessage]);
        })
        .finally(() => {
          setIsLoading(false);
        });
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('click', handleClick);
    window.addEventListener('textSelectionAsk', handleTextSelectionAsk);
    window.addEventListener('textSelectionUrdu', handleTextSelectionUrdu);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('click', handleClick);
      window.removeEventListener('textSelectionAsk', handleTextSelectionAsk);
      window.removeEventListener('textSelectionUrdu', handleTextSelectionUrdu);
    };
  }, [handleTextSelection, isOpen]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleQuickReply = async (replyText) => {
    const userMessage = {
      id: Date.now(),
      text: replyText,
      sender: 'user',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
    };

    setMessages(prev => [...prev, userMessage]);
    setQuickReplies([]); // Clear quick replies after user responds
    setIsLoading(true);

    try {
      // Prepare context with selected text
      const context = {
        selectedText: selectedText,
        timestamp: Date.now()
      };

      // Use the backend wiring function (currently mocked)
      const aiResponse = await connectToBackend(replyText, context);

      const responseMessage = {
        id: Date.now() + 1,
        text: aiResponse,
        sender: 'ai',
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      };

      setMessages(prev => [...prev, responseMessage]);

      // Generate quick replies for the AI response
      const newQuickReplies = generateQuickReplies(aiResponse, 'ai');
      setQuickReplies(newQuickReplies);

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
      setQuickReplies([]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSendMessage = async () => {
    if (!inputText.trim()) return;

    const userMessage = {
      id: Date.now(),
      text: inputText,
      sender: 'user',
      timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
    };

    setMessages(prev => [...prev, userMessage]);
    setQuickReplies([]); // Clear quick replies when user types a message
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

      // Generate quick replies for the AI response
      const newQuickReplies = generateQuickReplies(aiResponse, 'ai');
      setQuickReplies(newQuickReplies);

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
      setQuickReplies([]);
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

  const clearChat = () => {
    // Reset messages to initial state with just the welcome message
    setMessages([
      {
        id: 1,
        text: "👋 **Welcome to AI Robotics Assistant!**\n\nI'm here to help you with:\n- 🤖 **Physical AI & Robotics** concepts\n- 📚 **Textbook content** explanations\n- 🌐 **Urdu translations** of selected text\n\nJust ask me anything or select text and click **Ask** or **Urdu** buttons!",
        sender: 'ai',
        timestamp: new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      }
    ]);
    setQuickReplies([]);
    // Keep any current selection but clear the chat history
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
            <Bot className="selection-dot-icon" size={28} strokeWidth={2} />
          </div>
        ) : isOpen ? (
          <X size={28} strokeWidth={2.5} />
        ) : (
          <MessageCircle size={28} strokeWidth={2} />
        )}
      </button>

      {/* Chat Container */}
      {isOpen && (
        <div ref={chatContainerRef} className="chatkit-container">
          <div className="chatkit-header">
            <div className="chatkit-header-content">
              <div className="chatkit-header-title">
                <Bot className="chatkit-ai-icon" size={28} strokeWidth={2} />
                <h3>{defaultConfig.title}</h3>
              </div>
              <div className="chatkit-header-actions">
                {isSelectionMode && (
                  <span className="chatkit-selection-badge">
                    <span className="selection-dot"></span> Selected text mode
                  </span>
                )}
                <button
                  className="chatkit-clear"
                  onClick={clearChat}
                  aria-label="Clear chat"
                  title="Clear chat history"
                >
                  <Trash2 size={18} strokeWidth={2} />
                </button>
                <button
                  className="chatkit-close"
                  onClick={closeChat}
                  aria-label="Close chat"
                >
                  <X size={20} strokeWidth={2.5} />
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
                  <div
                    className="chatkit-message-text"
                    dangerouslySetInnerHTML={{ __html: formatMessageText(message.text) }}
                  />
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

          {/* Quick Replies */}
          {quickReplies.length > 0 && !isLoading && (
            <div className="chatkit-quick-replies">
              {quickReplies.map((reply, index) => (
                <button
                  key={index}
                  className="chatkit-quick-reply"
                  onClick={() => handleQuickReply(reply)}
                  disabled={isLoading}
                >
                  {reply}
                </button>
              ))}
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
              <Send className="send-icon" size={20} strokeWidth={2} />
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatKit;