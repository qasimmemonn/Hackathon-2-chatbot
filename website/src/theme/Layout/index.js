import React, { useState } from 'react';
import Layout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget';
import AskAIButton from '@site/src/components/askAI';

export default function LayoutWrapper(props) {
  const [chatInput, setChatInput] = useState('');
  const [isChatOpen, setIsChatOpen] = useState(false);

  const handleAskAI = (selectedText) => {
    setChatInput(selectedText);
    setIsChatOpen(true);
  };

  return (
    <>
      <Layout {...props} />
      <ChatWidget initialInput={chatInput} isOpen={isChatOpen} setIsOpen={setIsChatOpen} />
      <AskAIButton onAskAI={handleAskAI} />
    </>
  );
}
