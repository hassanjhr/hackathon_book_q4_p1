import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './auth.module.css';

export default function AuthPage(): JSX.Element {
  return (
    <Layout
      title="Authentication"
      description="Login or sign up to access your account"
    >
      <main className={styles.authPage}>
        <div className="container">
          <section className={styles.authCard}>
            <h1 className={styles.authTitle}>Welcome!</h1>
            <p className={styles.authSubtitle}>Choose an option to continue</p>
            <div className={styles.buttonGroup}>
              <Link
                to="/login"
                className={`button button--primary button--lg ${styles.authButton}`}
              >
                🔐 Sign In
              </Link>
              <Link
                to="/signup"
                className={`button button--secondary button--lg ${styles.authButton}`}
              >
                ✨ Sign Up
              </Link>
            </div>
          </section>
        </div>
      </main>
    </Layout>
  );
}
