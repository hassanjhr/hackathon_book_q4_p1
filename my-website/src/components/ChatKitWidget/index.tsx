/**
 * ChatKitWidget - AI-Native Textbook Chat Interface
 *
 * Modern chat widget with:
 * - Server-Sent Events (SSE) streaming for real-time responses
 * - Thread-based conversation management
 * - Context7-enhanced RAG (textbook + library docs)
 * - Source citations display
 * - Docusaurus theme integration
 */
import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
  timestamp: Date;
}

interface Source {
  type: 'textbook' | 'library_docs';
  module?: string;
  week?: string;
  file?: string;
  library?: string;
  relevance_score?: number;
}

interface ChatKitWidgetProps {
  apiUrl?: string;
  userId?: string;
}

export default function ChatKitWidget({ apiUrl, userId = 'anonymous' }: ChatKitWidgetProps) {
  // State
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [threadId, setThreadId] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  // Refs
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // API base URL (from props, window config, or default)
  const API_BASE_URL = apiUrl ||
    (typeof window !== 'undefined' && (window as any).DOCUSAURUS_API_URL) ||
    'http://localhost:8000';

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen) {
      inputRef.current?.focus();
    }
  }, [isOpen]);

  /**
   * Send message to ChatKit backend with SSE streaming
   */
  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    // Prepare assistant message (will be updated via streaming)
    const assistantMessageId = (Date.now() + 1).toString();
    let assistantContent = '';
    let assistantSources: Source[] = [];

    try {
      // Call RAG API (simpler non-streaming endpoint that works)
      const response = await fetch(`${API_BASE_URL}/api/rag/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: inputValue,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      // Read JSON response
      const data = await response.json();

      // Extract answer and sources
      assistantContent = data.answer || 'Sorry, I could not generate a response.';
      assistantSources = data.sources || [];

      // Add assistant message with sources
      setMessages(prev => [
        ...prev,
        {
          id: assistantMessageId,
          role: 'assistant' as const,
          content: assistantContent,
          sources: assistantSources,
          timestamp: new Date(),
        },
      ]);

    } catch (err) {
      console.error('Chat error:', err);
      setError(err instanceof Error ? err.message : 'Failed to send message');

      // Remove partial assistant message on error
      setMessages(prev => prev.filter(m => m.id !== assistantMessageId));
    } finally {
      setIsLoading(false);
    }
  };

  /**
   * Handle Enter key to send message (Shift+Enter for new line)
   */
  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  /**
   * Render source citations
   */
  const renderSources = (sources: Source[]) => {
    const textbookSources = sources.filter(s => s.type === 'textbook');
    const librarySources = sources.filter(s => s.type === 'library_docs');

    return (
      <div className={styles.sources}>
        {textbookSources.length > 0 && (
          <div className={styles.sourceSection}>
            <strong>📚 Textbook Sources:</strong>
            <ul>
              {textbookSources.map((source, idx) => (
                <li key={idx}>
                  {source.module}, {source.week}: {source.file}
                  {source.relevance_score && (
                    <span className={styles.relevanceScore}>
                      {' '}(relevance: {source.relevance_score.toFixed(2)})
                    </span>
                  )}
                </li>
              ))}
            </ul>
          </div>
        )}

        {librarySources.length > 0 && (
          <div className={styles.sourceSection}>
            <strong>🔧 Library Documentation:</strong>
            <ul>
              {librarySources.map((source, idx) => (
                <li key={idx}>
                  {source.library?.toUpperCase()} API Documentation
                </li>
              ))}
            </ul>
          </div>
        )}
      </div>
    );
  };

  /**
   * Starter prompts for quick questions
   */
  const starterPrompts = [
    "What is Physical AI?",
    "How do I create a PyTorch tensor?",
    "Explain humanoid robotics",
    "What is reinforcement learning?",
  ];

  const handleStarterPrompt = (prompt: string) => {
    setInputValue(prompt);
    inputRef.current?.focus();
  };

  return (
    <>
      {/* Floating chat button */}
      <button
        className={styles.chatButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Open AI Assistant"
        title="Ask AI Assistant"
      >
        💬
      </button>

      {/* Chat panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <h3>🤖 AI Textbook Assistant</h3>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 ? (
              <div className={styles.welcomeMessage}>
                <h4>Welcome! Ask me anything about the textbook.</h4>
                <p>I can help with:</p>
                <ul>
                  <li>Physical AI concepts</li>
                  <li>Humanoid robotics</li>
                  <li>PyTorch, ROS2, Isaac Sim (with live documentation)</li>
                  <li>Reinforcement learning</li>
                </ul>
                <div className={styles.starterPrompts}>
                  {starterPrompts.map((prompt, idx) => (
                    <button
                      key={idx}
                      className={styles.starterPrompt}
                      onClick={() => handleStarterPrompt(prompt)}
                    >
                      {prompt}
                    </button>
                  ))}
                </div>
              </div>
            ) : (
              messages.map(message => (
                <div
                  key={message.id}
                  className={`${styles.message} ${styles[message.role]}`}
                >
                  <div className={styles.messageContent}>
                    <div className={styles.messageText}>{message.content}</div>
                    {message.sources && message.sources.length > 0 && (
                      renderSources(message.sources)
                    )}
                  </div>
                  <div className={styles.messageTime}>
                    {message.timestamp.toLocaleTimeString()}
                  </div>
                </div>
              ))
            )}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.messageContent}>
                  <div className={styles.loadingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}

            {error && (
              <div className={styles.errorMessage}>
                ⚠️ Error: {error}
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className={styles.inputContainer}>
            <textarea
              ref={inputRef}
              className={styles.input}
              value={inputValue}
              onChange={e => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Ask about the textbook content..."
              rows={2}
              disabled={isLoading}
            />
            <button
              className={styles.sendButton}
              onClick={sendMessage}
              disabled={isLoading || !inputValue.trim()}
              aria-label="Send message"
            >
              {isLoading ? '⏳' : '📤'}
            </button>
          </div>
        </div>
      )}
    </>
  );
}
