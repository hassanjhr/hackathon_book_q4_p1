import React from 'react';
import ChatKitWidget from '@site/src/components/ChatKitWidget';

/**
 * Root component wrapper for Docusaurus.
 *
 * Injects the ChatKitWidget globally across all pages.
 * ChatKitWidget provides AI-powered Q&A about textbook content
 * with Context7-enhanced library documentation.
 *
 * Note: Urdu language "coming soon" modal is now handled by
 * the custom LocaleDropdownNavbarItem component.
 */
export default function Root({children}) {
  return (
    <>
      {children}
      <ChatKitWidget />
    </>
  );
}
