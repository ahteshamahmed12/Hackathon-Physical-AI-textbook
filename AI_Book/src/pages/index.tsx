import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className={clsx("container", styles.heroContent)}>
        <div className={styles.heroTextContent}>
          <Heading as="h1" className="hero__title">
            {siteConfig.title}
          </Heading>
          <h1>
          Colearning ROS2, ISAAC and VLA with Python and TypeScript – Spec Driven Reusable Intelligence
          </h1>
          
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg "
              to="/module-1-intro">
              Start Reading Book
            </Link>
          </div>
        </div>
        <div className={styles.heroImageContainer}>
          <img
            src="/img/image.jpg"
            alt="AI Book Cover"
            className={styles.heroImage}
          />
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <section className={styles.differencesSection}>
          <div className="container">
            <div className="row">
              <div className={clsx('col col--6', styles.differenceColumn)}>
                <Heading as="h2">Vibe Coding</Heading>
                <p>Vibe coding refers to an informal, intuitive, and often unstructured approach to software development. It prioritizes rapid iteration, personal preference, and a "feel-good" factor over strict planning, formal processes, or rigorous architectural design. While it can be fast for small, personal projects, it often leads to:</p>
                <ul>
                  <li>Inconsistent code quality and style.</li>
                  <li>Difficulty in maintenance and debugging.</li>
                  <li>Scalability issues.</li>
                  <li>Poor documentation and knowledge transfer.</li>
                  <li>Increased technical debt over time.</li>
                </ul>
                <p>It's characterized by a lack of upfront design, minimal testing, and a tendency to jump directly into coding based on immediate ideas.</p>
              </div>
              <div className={clsx('col col--6', styles.differenceColumn)}>
                <Heading as="h2">Spec-Driven Development with Real-time Intelligence (SSDRI)</Heading>
                <p>SSDRI is a structured, systematic, and intelligent approach to software development that leverages specifications and real-time insights to guide the entire development lifecycle. It emphasizes clear requirements, detailed planning, continuous feedback, and automated intelligence to ensure high-quality, maintainable, and scalable software. Key aspects include:</p>
                <ul>
                  <li>**Spec-Driven:** Development is guided by explicit, detailed specifications (requirements, architecture, tasks).</li>
                  <li>**Real-time Intelligence:** Utilizes AI/agentic systems for immediate feedback, code analysis, error detection, and architectural decision support.</li>
                  <li>**Structured Planning:** Involves upfront design, breaking down complex features into manageable, testable tasks.</li>
                  <li>**Automated Quality:** Integrates continuous testing, linting, and build processes.</li>
                  <li>**Traceability & Documentation:** Maintains clear links between requirements, code, and architectural decisions (e.g., PHRs, ADRs).</li>
                </ul>
                <p>SSDRI aims to reduce ambiguity, improve collaboration, and deliver robust software efficiently by making informed decisions at every stage.</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
