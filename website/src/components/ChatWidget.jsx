import React, { useState, useEffect, useRef } from 'react';
import styles from './ChatWidget.module.css';

const API_BASE_URL = process.env.NODE_ENV === 'production' 
    ? 'https://your-production-backend-url.vercel.app' // TODO: Replace with actual production URL
    : 'https://memonkhan-chatbot-hackathon.hf.space';

const ChatWidget = ({ initialInput, isOpen, setIsOpen }) => {
    const [messages, setMessages] = useState([]);
    const [input, setInput] = useState('');
    const [chatSessionId, setChatSessionId] = useState(null);
    const [isLoading, setIsLoading] = useState(false);
    const messagesEndRef = useRef(null);

    useEffect(() => {
        const storedSessionId = localStorage.getItem('chatSessionId');
        if (storedSessionId) {
            setChatSessionId(storedSessionId);
            // TODO: Optionally fetch previous messages for this session
        }
    }, []);

    useEffect(() => {
        if (initialInput && isOpen) {
            setInput(initialInput);
            sendMessage(initialInput); // Send the initial input automatically
        }
    }, [initialInput, isOpen]);

    useEffect(() => {
        messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
    }, [messages]);

    const toggleChat = () => {
        setIsOpen(!isOpen);
    };

    const sendMessage = async (messageText = input) => {
        if (messageText.trim() === '') return;

        const userMessage = { role: 'user', content: messageText };
        setMessages((prevMessages) => [...prevMessages, userMessage]);
        setInput('');
        setIsLoading(true);

        try {
            const response = await fetch(`${API_BASE_URL}/chat`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ query: messageText, chat_session_id: chatSessionId }),
            });

            if (!response.ok) {
                const errorData = await response.json();
                throw new Error(errorData.detail || 'Failed to fetch response from API.');
            }

            const data = await response.json();
            const assistantMessage = { role: 'assistant', content: data.answer, documents: data.documents };
            setMessages((prevMessages) => [...prevMessages, assistantMessage]);
            if (data.chat_session_id && !chatSessionId) {
                setChatSessionId(data.chat_session_id);
                localStorage.setItem('chatSessionId', data.chat_session_id);
            }
        } catch (error) {
            console.error('Error sending message:', error);
            setMessages((prevMessages) => [
                ...prevMessages,
                { role: 'assistant', content: `Error: ${error.message}` },
            ]);
        } finally {
            setIsLoading(false);
        }
    };

    const handleKeyDown = (e) => {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            sendMessage();
        }
    };

    return (
        <div className={styles.chatWidgetContainer}>
            <button className={styles.chatToggleButton} onClick={toggleChat} aria-label="Toggle Chatbot">
                {isOpen ? (
                     /* Close Icon (X) */
                    <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                        <path d="M18 6L6 18" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                        <path d="M6 6L18 18" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                    </svg>
                ) : (
                    /* Message Icon */
                    <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                        <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H7L3 21V5C3 4.46957 3.21071 3.96086 3.58579 3.58579C3.96086 3.21071 4.46957 3 5 3H19C19.5304 3 20.0391 3.21071 20.4142 3.58579C20.7893 3.96086 21 4.46957 21 5V15Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                    </svg>
                )}
            </button>

            {isOpen && (
                <div className={styles.chatWindow}>
                    <div className={styles.chatHeader}>Physical AI Book Chatbot</div>
                    <div className={styles.chatMessages}>
                        {messages.length === 0 && <div className={styles.welcomeMessage}>Ask me anything about the book!</div>}
                        {messages.map((msg, index) => (
                            <div key={index} className={`${styles.message} ${styles[msg.role]}`}>
                                <p>{msg.content}</p>
                                {msg.role === 'assistant' && msg.documents && msg.documents.length > 0 && (
                                    <div className={styles.documents}>
                                        <strong>Sources:</strong>
                                        <ul>
                                            {msg.documents.map((doc, docIndex) => (
                                                <li key={docIndex}>
                                                    <a href={doc.url} target="_blank" rel="noopener noreferrer">{doc.title || doc.url}</a>
                                                </li>
                                            ))}
                                        </ul>
                                    </div>
                                )}
                            </div>
                        ))}
                        {isLoading && (
                            <div className={`${styles.message} ${styles.assistant}`}>
                                <div className={styles.loadingDots}>
                                    <span>.</span><span>.</span><span>.</span>
                                </div>
                            </div>
                        )}
                        <div ref={messagesEndRef} />
                    </div>
                    <div className={styles.chatInputContainer}>
                        <textarea
                            className={styles.chatInput}
                            value={input}
                            onChange={(e) => setInput(e.target.value)}
                            onKeyDown={handleKeyDown}
                            placeholder="Type your message..."
                            rows="1"
                        />
                        <button className={styles.sendButton} onClick={() => sendMessage()} disabled={isLoading}>
                            Send
                        </button>
                    </div>
                </div>
            )}
        </div>
    );
};  

export default ChatWidget; 
