import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';
import UrduComingSoon from '@site/src/components/UrduComingSoon';

export default function Root({children}) {
  return (
    <>
      <UrduComingSoon />
      {children}
      <ChatWidget />
    </>
  );
}
