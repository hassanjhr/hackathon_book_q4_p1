import React from 'react';
import Link from '@docusaurus/Link';
import styles from './UserIconNavbarItem.module.css';

export interface UserIconNavbarItemProps {
  mobile?: boolean;
  authUrl?: string;
}

export default function UserIconNavbarItem({
  mobile = false,
  authUrl = '/auth',
}: UserIconNavbarItemProps): JSX.Element {
  return (
    <Link
      to={authUrl}
      className={`navbar__link ${styles.userIconLink}`}
      aria-label="User menu"
    >
      <svg
        width="20"
        height="20"
        viewBox="0 0 24 24"
        fill="none"
        xmlns="http://www.w3.org/2000/svg"
        className={styles.userIcon}
      >
        <path
          d="M12 12C14.21 12 16 10.21 16 8C16 5.79 14.21 4 12 4C9.79 4 8 5.79 8 8C8 10.21 9.79 12 12 12ZM12 14C9.33 14 4 15.34 4 18V20H20V18C20 15.34 14.67 14 12 14Z"
          fill="currentColor"
        />
      </svg>
      {!mobile && <span className={styles.userIconLabel}>User</span>}
    </Link>
  );
}
