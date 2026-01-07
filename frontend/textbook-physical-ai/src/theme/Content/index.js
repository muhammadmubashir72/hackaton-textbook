import React from 'react';
import OriginalContent from '@theme-original/Content';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useLocation from '@docusaurus/useLocation';

// Client-only buttons component
function ContentButtons() {
  const TranslationButton = require('@site/src/components/TranslationButton/TranslationButton').default;
  const PersonalizationSelector = require('@site/src/components/PersonalizationSelector/PersonalizationSelector').default;

  return (
    <div style={{
      display: 'flex',
      justifyContent: 'flex-end',
      gap: '12px',
      marginBottom: '16px',
      position: 'relative',
      zIndex: 1000
    }}>
      <TranslationButton />
      <PersonalizationSelector />
    </div>
  );
}

export default function Content(props) {
  const location = useLocation();

  // Check if we're on a content page
  const isContentPage = location.pathname.startsWith('/docs/') &&
    !location.pathname.endsWith('/docs') &&
    !location.pathname.endsWith('/docs/');

  return (
    <>
      {isContentPage && (
        <BrowserOnly fallback={null}>
          {() => <ContentButtons />}
        </BrowserOnly>
      )}
      <OriginalContent {...props} />
    </>
  );
}
