import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

export default function UrduComingSoon(): JSX.Element | null {
  const {i18n} = useDocusaurusContext();

  // Only show for Urdu locale
  if (i18n.currentLocale !== 'ur') {
    return null;
  }

  return (
    <div className={styles.banner}>
      <div className={styles.container}>
        <div className={styles.icon}>🚧</div>
        <div className={styles.content}>
          <h3 className={styles.title}>اردو ترجمہ جلد آرہا ہے</h3>
          <p className={styles.message}>
            یہ مواد فی الحال اردو میں ترجمہ کیا جا رہا ہے۔ مکمل تجربے کے لیے، براہ کرم ابھی انگریزی ورژن استعمال کریں۔
          </p>
          <p className={styles.englishMessage}>
            <strong>Urdu Translation Coming Soon</strong><br />
            This content is currently being translated into Urdu. For the complete experience, please use the English version for now.
          </p>
        </div>
      </div>
    </div>
  );
}
