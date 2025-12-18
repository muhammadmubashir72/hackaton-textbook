import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatKit from '@site/src/components/ChatKit/ChatKit';

export default function Layout(props) {
  const { children } = props;

  return (
    <>
      <OriginalLayout {...props}>
        {children}
        <ChatKit
          config={{
            title: 'AI Assistant',
            placeholder: 'Ask about AI, Robotics...'
          }}
        />
      </OriginalLayout>
    </>
  );
}