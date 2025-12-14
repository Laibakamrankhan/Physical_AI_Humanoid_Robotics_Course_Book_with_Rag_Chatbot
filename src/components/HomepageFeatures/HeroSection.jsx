import React from 'react';
import Link from '@docusaurus/Link';
import { motion } from 'framer-motion'

export default function HeroSection() {
  return (
    <section className="hero-section">
      <div className="hero-overlay">
        <div className="hero-content container flex items-center justify-between">
          
          {/* Text Section */}
          <div className="hero-text max-w-xl">
            <h1>Physical AI & Humanoid Robotics</h1>
            <p className="tagline">
              A Practical Guide to Physical AI and Humanoid Systems
            </p>

            <div className="cta-row">
              <Link to="intro" className="btn btn-primary">
                Start Learning
              </Link>
              <Link to="/signup" className="btn btn-ghost">
                Sign Up
              </Link>
            </div>
          </div>

          {/* Image Section */}
          <div className="hero-image">
          <motion.img
          src="img/robo.png"
          alt="Robot"
          className="w-[380px] h-auto"
          animate={{ y: [0, -20, 0] }}
          transition={{ duration: 2, repeat: Infinity, repeatType: 'mirror', ease: 'easeInOut' }}
          />
          </div>

        </div>
      </div>
    </section>
  );
}
