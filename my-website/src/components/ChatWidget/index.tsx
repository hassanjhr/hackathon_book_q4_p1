import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './styles.module.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  citations?: Array<{
    chunk_id: string;
    excerpt: string;
    chapter_page_ref: string;
    relevance_score: number;
  }>;
}

function ChatWidgetContent(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [mounted, setMounted] = useState(false);

  // Docusaurus-compatible way to get environment variables
  const API_BASE_URL = typeof window !== 'undefined'
    ? (window as any).DOCUSAURUS_API_URL || 'http://localhost:8000'
    : 'http://localhost:8000';

  useEffect(() => {
    setMounted(true);
  }, []);

  if (!mounted) {
    return null;
  }

  const handleSend = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage: Message = { role: 'user', content: input };
    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await fetch(`${API_BASE_URL}/api/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query_text: input,
        }),
      });

      if (!response.ok) {
        throw new Error(`API error: ${response.status}`);
      }

      const data = await response.json();

      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer_text,
        citations: data.citations,
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error querying chatbot:', error);
      const errorMessage: Message = {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please make sure the backend is running at ' + API_BASE_URL,
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <>
      {/* Chat Button */}
      <button
        className={styles.chatButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          <div className={styles.chatHeader}>
            <h3>RAG Assistant</h3>
            <p>Ask questions about the textbook</p>
          </div>

          <div className={styles.chatMessages}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                Welcome! Ask me anything about Physical AI & Humanoid Robotics.
              </div>
            )}

            {messages.map((msg, idx) => (
              <div
                key={idx}
                className={
                  msg.role === 'user' ? styles.userMessage : styles.assistantMessage
                }
              >
                <div className={styles.messageContent}>{msg.content}</div>

                {msg.citations && msg.citations.length > 0 && (
                  <div className={styles.citations}>
                    <strong>Citations:</strong>
                    {msg.citations.map((citation, citIdx) => (
                      <div key={citIdx} className={styles.citation}>
                        <span className={styles.citationRef}>
                          [{citation.chapter_page_ref}]
                        </span>
                        <span className={styles.citationExcerpt}>
                          {citation.excerpt.substring(0, 100)}...
                        </span>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className={styles.loadingMessage}>
                <span className={styles.loadingDots}>Thinking...</span>
              </div>
            )}
          </div>

          <div className={styles.chatInput}>
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question..."
              rows={2}
              disabled={isLoading}
            />
            <button onClick={handleSend} disabled={isLoading || !input.trim()}>
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
}

export default function ChatWidget(): JSX.Element {
  return (
    <BrowserOnly fallback={<div />}>
      {() => <ChatWidgetContent />}
    </BrowserOnly>
  );
}
