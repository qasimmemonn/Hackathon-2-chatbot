import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          Physical AI & Humanoid Robotics
        </Heading>
        <p className="hero__subtitle">A Complete Guide to Building the Robots of Tomorrow</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning
          </Link>
        </div>
      </div>
    </header>
  );
}

function CourseModules() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className={clsx('col col--12')}>
            <Heading as="h2" className={clsx("text--center", styles.courseOverviewHeading)}>📚 Course Modules Overview</Heading>
          </div>
        </div>
        <div className="row" style={{marginTop: '2rem'}}>
          <div className={clsx('col col--6')}>
            <Link className={styles.moduleLink} to="/docs/module-1-ros-2/introduction-to-ros-2">
              <div className={clsx("text--center padding-horiz--md", styles.moduleCard)}>
                <Heading as="h3">Module 1: The Robotic Nervous System (ROS 2)</Heading>
                <p>Master ROS 2 architecture, communication patterns, and robot modeling. Learn to build distributed robotic systems using nodes, topics, services, and actions.</p>
                <Heading as="h4">Learning Outcomes</Heading>
                <ul>
                  <li>Explain the ROS 2 computation graph and its components</li>
                  <li>Create publishers, subscribers, and service clients using rclpy</li>
                  <li>Define robot structure using URDF and visualize it in RViz2</li>
                </ul>
              </div>
            </Link>
          </div>
          <div className={clsx('col col--6')}>
            <Link className={styles.moduleLink} to="/docs/module-2-digital-twin/gazebo-physics-simulation">
              <div className={clsx("text--center padding-horiz--md", styles.moduleCard)}>
                <Heading as="h3">Module 2: Digital Twins — Simulation & Sensors</Heading>
                <p>Build digital twins for robotic systems using Gazebo and Unity. Simulate sensors, physics, and environments for testing before deploying to physical hardware.</p>
                <Heading as="h4">Learning Outcomes</Heading>
                <ul>
                  <li>Create Gazebo simulation environments with physics and sensors</li>
                  <li>Integrate Unity for photorealistic sensor simulation</li>
                  <li>Test navigation and perception algorithms in simulation</li>
                </ul>
              </div>
            </Link>
          </div>
        </div>
        <div className="row" style={{marginTop: '2rem'}}>
          <div className={clsx('col col--6')}>
            <Link className={styles.moduleLink} to="/docs/module-3-nvidia-isaac/introduction-to-isaac-sim">
              <div className={clsx("text--center padding-horiz--md", styles.moduleCard)}>
                <Heading as="h3">Module 3: NVIDIA Isaac — Perception & Navigation</Heading>
                <p>Leverage NVIDIA Isaac Sim for GPU-accelerated robotics. Implement VSLAM, Nav2 navigation stacks, and reinforcement learning for autonomous behaviors.</p>
                <Heading as="h4">Learning Outcomes</Heading>
                <ul>
                  <li>Set up and configure NVIDIA Isaac Sim environments</li>
                  <li>Implement Visual SLAM for robot localization</li>
                  <li>Deploy the Nav2 navigation stack for autonomous navigation</li>
                </ul>
              </div>
            </Link>
          </div>
          <div className={clsx('col col--6')}>
            <Link className={styles.moduleLink} to="/docs/module-4-vla-humanoids/kinematics-dynamics">
              <div className={clsx("text--center padding-horiz--md", styles.moduleCard)}>
                <Heading as="h3">Module 4: VLA & Humanoid Robotics</Heading>
                <p>Integrate Vision-Language-Action models with humanoid robots. Master humanoid kinematics, manipulation, and conversational AI for natural human-robot interaction.</p>
                <Heading as="h4">Learning Outcomes</Heading>
                <ul>
                  <li>Calculate forward and inverse kinematics for humanoid robots</li>
                  <li>Implement manipulation primitives for pick-and-place tasks</li>
                  <li>Integrate conversational AI with robot action planning</li>
                </ul>
              </div>
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="A Complete Guide to Building the Robots of Tomorrow">
      <HomepageHeader />
      <main>
        <CourseModules />
      </main>
    </Layout>
  );
}