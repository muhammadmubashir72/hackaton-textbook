import React, { useRef, useEffect } from 'react';
import DocItem from '@theme-original/DocItem';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { useLocation } from '@docusaurus/router';

// Import the new combined controls component
const ChapterControls = ExecutionEnvironment.canUseDOM
  ? require('@site/src/components/ChapterControls/ChapterControls').default
  : () => <></>;

export default function DocItemWrapper(props) {
  const location = useLocation();
  const contentRef = useRef(null);

  // Check if we're on a documentation page
  const isDocPage = location.pathname.startsWith('/docs/');

  // Find the content element after render
  useEffect(() => {
    if (isDocPage) {
      // Try to find the main content area
      const findContent = () => {
        const contentElement = document.querySelector('.markdown') || document.querySelector('[class*="markdown"]');
        if (contentElement) {
          contentRef.current = contentElement;
        }
      };

      // Try immediately
      findContent();

      // Also try after a short delay in case DOM isn't ready
      const timeoutId = setTimeout(findContent, 100);

      // Use MutationObserver to detect when content is added
      const observer = new MutationObserver((mutations) => {
        if (!contentRef.current) {
          findContent();
        }
      });

      const docRoot = document.querySelector('main') || document.body;
      if (docRoot) {
        observer.observe(docRoot, { childList: true, subtree: true });
      }

      return () => {
        clearTimeout(timeoutId);
        observer.disconnect();
      };
    }
  }, [isDocPage, location.pathname]);

  // Generate a chapter ID based on the current path
  const getChapterId = () => {
    return location.pathname.replace('/docs/', '').replace(/\//g, '_') || 'default';
  };

  return (
    <>
      {isDocPage && (
        <div style={{
          display: 'flex',
          justifyContent: 'flex-start',
          alignItems: 'center',
          gap: '12px',
          marginBottom: '16px',
          marginTop: '8px',
          position: 'relative',
          zIndex: 1000
        }}>
          <ChapterControls chapterId={getChapterId()} contentElementRef={contentRef} />
        </div>
      )}
      <DocItem {...props} />
    </>
  );
}