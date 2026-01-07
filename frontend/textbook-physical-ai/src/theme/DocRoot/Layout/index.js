import React from 'react';
import OriginalDocRootLayout from '@theme-original/DocRoot/Layout';

export default function DocRootLayout(props) {
  // Simply pass through to original - buttons are handled in Content component
  return <OriginalDocRootLayout {...props} />;
}
