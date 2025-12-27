import React, { useState } from 'react';
import LocaleDropdownNavbarItem from '@theme-original/NavbarItem/LocaleDropdownNavbarItem';
import type { Props } from '@theme/NavbarItem/LocaleDropdownNavbarItem';
import styles from './styles.module.css';

/**
 * Custom LocaleDropdown that intercepts Urdu language selection
 * and shows a "Coming Soon" modal instead of navigating to non-existent pages
 */
export default function LocaleDropdownNavbarItemWrapper(props: Props) {
  const [showModal, setShowModal] = useState(false);

  // Intercept locale dropdown clicks
  const handleLocaleClick = (e: MouseEvent) => {
    const target = e.target as HTMLElement;
    const link = target.closest('a');

    if (link && link.getAttribute('lang') === 'ur') {
      e.preventDefault();
      e.stopPropagation();
      setShowModal(true);
    }
  };

  React.useEffect(() => {
    // Add click listener to intercept Urdu clicks
    document.addEventListener('click', handleLocaleClick, true);
    return () => {
      document.removeEventListener('click', handleLocaleClick, true);
    };
  }, []);

  return (
    <>
      <LocaleDropdownNavbarItem {...props} />

      {showModal && (
        <div className={styles.modalOverlay} onClick={() => setShowModal(false)}>
          <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
            <button
              className={styles.closeButton}
              onClick={() => setShowModal(false)}
              aria-label="Close"
            >
              ✕
            </button>

            <div className={styles.modalBody}>
              <div className={styles.icon}>🚧</div>

              <h2 className={styles.urduTitle}>اردو ترجمہ جلد آرہا ہے</h2>

              <p className={styles.urduText}>
                یہ مواد فی الحال اردو میں ترجمہ کیا جا رہا ہے۔ مکمل تجربے کے لیے، براہ کرم ابھی انگریزی ورژن استعمال کریں۔
              </p>

              <div className={styles.englishSection}>
                <h3>Urdu Translation Coming Soon</h3>
                <p>
                  This content is currently being translated into Urdu.
                  For the complete experience, please use the English version for now.
                </p>
              </div>

              <button
                className={styles.okButton}
                onClick={() => setShowModal(false)}
              >
                OK
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
}
