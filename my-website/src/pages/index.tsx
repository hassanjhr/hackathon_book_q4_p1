import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

interface ModuleCardProps {
  title: string;
  description: string;
  link: string;
  icon: string;
  color: string;
}

function ModuleCard({title, description, link, icon, color}: ModuleCardProps) {
  return (
    <div className={clsx('col col--6', styles.moduleCard)}>
      <Link to={link} className={styles.moduleCardLink}>
        <div className={styles.moduleCardInner} style={{'--module-color': color} as React.CSSProperties}>
          <div className={styles.moduleIcon}>{icon}</div>
          <Heading as="h3" className={styles.moduleTitle}>
            {title}
          </Heading>
          <p className={styles.moduleDescription}>{description}</p>
          <div className={styles.moduleArrow}>→</div>
        </div>
      </Link>
    </div>
  );
}

function HomepageHeader() {
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <Heading as="h1" className={styles.heroTitle}>
            Welcome to Physical AI & Humanoid Robotics Textbook
          </Heading>
          <p className={styles.heroSubtitle}>
            Master the fundamentals of robotics, simulation, perception, and vision-language-action systems
            with our AI-powered interactive learning platform.
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--primary button--lg', styles.ctaButton)}
              to="/docs/module-1/intro">
              Get Started 🚀
            </Link>
            <Link
              className={clsx('button button--outline button--lg', styles.secondaryButton)}
              to="/docs/module-1/intro">
              Browse Modules 📚
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function ModulesSection() {
  const modules = [
    {
      title: 'Module 1: ROS 2 Fundamentals',
      description: 'Learn the robotic nervous system - understanding middleware, nodes, topics, services, and actions in ROS 2.',
      link: '/docs/module-1/intro',
      icon: '🤖',
      color: '#2e8555',
    },
    {
      title: 'Module 2: Digital Twin (Gazebo & Unity)',
      description: 'Master simulation environments, physics engines, and building digital twins for safe robot testing.',
      link: '/docs/module-2/intro',
      icon: '🌐',
      color: '#0ea5e9',
    },
    {
      title: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
      description: 'Build perception pipelines with Isaac Sim for synthetic data, VSLAM, and autonomous navigation.',
      link: '/docs/module-3/intro',
      icon: '🧠',
      color: '#8b5cf6',
    },
    {
      title: 'Module 4: Vision-Language-Action (VLA)',
      description: 'Integrate voice commands, LLM planning, and complete autonomous humanoid task execution pipelines.',
      link: '/docs/module-4/intro',
      icon: '🎯',
      color: '#f59e0b',
    },
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          Explore Our Modules
        </Heading>
        <p className={styles.sectionSubtitle}>
          Progressive learning path from fundamentals to advanced AI robotics
        </p>
        <div className="row">
          {modules.map((module, idx) => (
            <ModuleCard key={idx} {...module} />
          ))}
        </div>
      </div>
    </section>
  );
}

function FeaturesSection() {
  const features = [
    {
      title: '🤖 AI-Powered Chatbot',
      description: 'Get instant answers with our RAG-powered learning assistant',
    },
    {
      title: '🌍 Bilingual Support',
      description: 'Learn in English or Urdu with seamless translation',
    },
    {
      title: '💡 Interactive Examples',
      description: 'Hands-on exercises and runnable code snippets',
    },
    {
      title: '📊 Progress Tracking',
      description: 'Monitor your learning journey with personalized recommendations',
    },
  ];

  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className="row">
          {features.map((feature, idx) => (
            <div key={idx} className="col col--3">
              <div className={styles.featureCard}>
                <h3>{feature.title}</h3>
                <p>{feature.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="AI-Native Interactive Textbook for Physical AI & Humanoid Robotics - Learn ROS 2, Gazebo, Unity, Isaac, and VLA systems">
      <HomepageHeader />
      <main>
        <ModulesSection />
        <FeaturesSection />
      </main>
    </Layout>
  );
}
