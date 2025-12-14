import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          Physical AI and Humanoid Robotics
        </Heading>
        <p className="hero__subtitle">
          An exploration of embodied intelligence. Learn to build and control intelligent robots with ROS2, Isaac Sim, and cutting-edge Vision-Language Models.
        </p>
        <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg "
              to="/module-1-intro">
              Start Learning
            </Link>
          </div>
      </div>
      
    </header>

    
  );
}

function WhatYouWillLearn() {
  return (
    <section className={styles.features}>
      <div className="container">

        <div className="row">
          <div className={clsx('col col--12', styles.featuresTitle)}>
            <Heading as="h2">What You'll Learn</Heading>
          </div>
          
          <div className={clsx('col col--3', styles.feature)}>
            <h3>Fundamentals of Robotics and AI</h3>
            <p>Explore the core concepts that underpin modern robotics and artificial intelligence.</p>
          </div>
          <div className={clsx('col col--3', styles.feature)}>
            <h3>Simulating Robots in Gazebo & Isaac Sim</h3>
            <p>Master industry-standard tools for simulating and testing your robotic creations.</p>
          </div>
          <div className={clsx('col col--3', styles.feature)}>
            <h3>Advanced Control with ROS2</h3>
            <p>Learn to develop sophisticated control systems for your robots using ROS2.</p>
          </div>
          <div className={clsx('col col--3', styles.feature)}>
            <h3>Embodied AI with VLMs</h3>
            <p>Integrate cutting-edge Vision-Language Models to create truly intelligent robots.</p>
          </div>
        </div>
      </div>
      
    </section>
  );
}

export default function Home(): React.ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Learn to build and control intelligent robots with ROS2, Isaac Sim, and cutting-edge Vision-Language Models.">
      <HomepageHeader />
      <main>
        <WhatYouWillLearn />
      </main>
    </Layout>
  );
}