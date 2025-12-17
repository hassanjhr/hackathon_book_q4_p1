import React, { useState, FormEvent } from 'react';
import Layout from '@theme/Layout';
import styles from './login.module.css';

interface LoginFormData {
  email: string;
  password: string;
}

export default function LoginPage(): JSX.Element {
  const [formData, setFormData] = useState<LoginFormData>({
    email: '',
    password: '',
  });
  const [errors, setErrors] = useState<Partial<LoginFormData>>({});
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [showMessage, setShowMessage] = useState(false);

  const validateForm = (): boolean => {
    const newErrors: Partial<LoginFormData> = {};

    // Email validation
    if (!formData.email.trim()) {
      newErrors.email = 'Email is required';
    } else if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(formData.email)) {
      newErrors.email = 'Invalid email format';
    }

    // Password validation
    if (!formData.password) {
      newErrors.password = 'Password is required';
    } else if (formData.password.length < 8) {
      newErrors.password = 'Password must be at least 8 characters';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
    // Clear error for this field when user types
    if (errors[name as keyof LoginFormData]) {
      setErrors(prev => ({ ...prev, [name]: undefined }));
    }
  };

  const handleSubmit = (e: FormEvent<HTMLFormElement>) => {
    e.preventDefault();

    if (validateForm()) {
      setIsSubmitting(true);
      setShowMessage(true);
      setTimeout(() => {
        setIsSubmitting(false);
      }, 1000);
    }
  };

  return (
    <Layout
      title="Login"
      description="Sign in to your account"
    >
      <main className={styles.loginPage}>
        <div className="container">
          <div className={styles.loginForm}>
            <h1 className={styles.formTitle}>Sign In</h1>

            <div className={`alert alert--info ${styles.notice}`}>
              <strong>Coming Soon:</strong> Authentication will be enabled in a future update.
            </div>

            <form onSubmit={handleSubmit}>
              <div className={styles.formGroup}>
                <label htmlFor="email" className={styles.label}>
                  Email Address
                </label>
                <input
                  type="email"
                  id="email"
                  name="email"
                  className={styles.input}
                  value={formData.email}
                  onChange={handleChange}
                  placeholder="your@email.com"
                  required
                />
                {errors.email && (
                  <span className={styles.error}>{errors.email}</span>
                )}
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="password" className={styles.label}>
                  Password
                </label>
                <input
                  type="password"
                  id="password"
                  name="password"
                  className={styles.input}
                  value={formData.password}
                  onChange={handleChange}
                  placeholder="••••••••"
                  required
                />
                {errors.password && (
                  <span className={styles.error}>{errors.password}</span>
                )}
              </div>

              <button
                type="submit"
                className={`button button--primary button--lg ${styles.submitButton}`}
                disabled={isSubmitting}
              >
                {isSubmitting ? 'Signing In...' : 'Login'}
              </button>

              {showMessage && (
                <div className={`alert alert--success ${styles.successMessage}`}>
                  Form validated successfully! Authentication will be enabled soon.
                </div>
              )}
            </form>
          </div>
        </div>
      </main>
    </Layout>
  );
}
