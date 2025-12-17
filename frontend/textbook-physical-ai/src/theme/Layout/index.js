import React from 'react';
import OriginalLayout from '@theme-original/Layout';

export default function Layout(props) {
  const { children } = props;

  return (
    <>
      <OriginalLayout {...props}>
        {children}
      </OriginalLayout>
    </>
  );
}