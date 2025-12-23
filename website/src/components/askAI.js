import React, { useState, useEffect, useRef } from 'react';
import styles from './askAI.module.css';

const AskAIButton = ({ onAskAI }) => {
  const [isVisible, setIsVisible] = useState(false);
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const selectedTextRef = useRef(null);

  useEffect(() => {
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      const selectedText = selection.toString().trim();

      if (selectedText.length > 0) {
        selectedTextRef.current = selectedText;
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        
        setIsVisible(true);
        setPosition({
          x: rect.left + window.scrollX + rect.width / 2,
          y: rect.top + window.scrollY - 40, // Position above the text
        });
      } else {
        setIsVisible(false);
        selectedTextRef.current = null;
      }
    };

    document.addEventListener('mouseup', handleSelectionChange);
    document.addEventListener('keyup', handleSelectionChange);

    return () => {
      document.removeEventListener('mouseup', handleSelectionChange);
      document.removeEventListener('keyup', handleSelectionChange);
    };
  }, []);

  const handleClick = () => {
    if (selectedTextRef.current) {
      onAskAI(selectedTextRef.current);
      // Clear selection after asking AI
      if (window.getSelection) {
        window.getSelection().removeAllRanges();
      } else if (document.selection) {
        document.selection.empty();
      }
      setIsVisible(false); // Hide button after click
    }
  };

  if (!isVisible) return null;

  return (
    <button
      className={styles.askAIButton}
      style={{ left: position.x, top: position.y }}
      onClick={handleClick}
    >
      Ask AI
    </button>
  );
};

export default AskAIButton; 
