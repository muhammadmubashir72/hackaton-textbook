import React, { useState, useEffect } from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import { motion, useScroll, useTransform } from "framer-motion";
import { useAuth } from "../hooks/useAuth";
import styles from "./index.module.css";
import FuturisticRobot from "../components/FuturisticRobot/FuturisticRobot";

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  const { isAuthenticated } = useAuth();
  const [scrollY, setScrollY] = useState(0);

  useEffect(() => {
    const handleScroll = () => setScrollY(window.scrollY);
    window.addEventListener('scroll', handleScroll, { passive: true });
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  // Circular animated curriculum section
  const CircularCurriculum = () => {
    const modules = [
      { icon: "🤖", title: "ROS 2", desc: "Robot Operating System" },
      { icon: "🎮", title: "Gazebo", desc: "Physics Simulation" },
      { icon: "🔧", title: "Isaac", desc: "AI Integration" },
      { icon: "🧠", title: "VLA", desc: "Vision-Language-Action" },
      { icon: "🦾", title: "Humanoid", desc: "Design & Control" },
      { icon: "💬", title: "AI", desc: "Conversational" }
    ];

    return (
      <div className={styles.circularContainer}>
        <div className={styles.circularTrack}>
          {modules.map((module, index) => {
            const angle = (index * 60) * (Math.PI / 180);
            const radius = 180;
            const x = radius * Math.cos(angle);
            const y = radius * Math.sin(angle);

            return (
              <motion.div
                key={index}
                className={styles.circularItem}
                style={{
                  position: 'absolute',
                  left: `calc(50% + ${x}px)`,
                  top: `calc(50% + ${y}px)`,
                  transform: 'translate(-50%, -50%)'
                }}
                whileHover={{ scale: 1.2, zIndex: 10 }}
                animate={{
                  rotate: [0, 360],
                }}
                transition={{
                  rotate: {
                    duration: 20,
                    repeat: Infinity,
                    ease: "linear"
                  }
                }}
              >
                <div className={styles.circularItemContent}>
                  <div className={styles.curriculumIcon}>{module.icon}</div>
                  <h4 style={{fontFamily: "'Trebuchet MS', 'Lucida Sans Unicode', 'Lucida Grande', 'Lucida Sans', Arial, sans-serif"}}>{module.title}</h4>
                  <p style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>{module.desc}</p>
                </div>
              </motion.div>
            );
          })}
        </div>
        <div className={styles.centerCircle}>
          <h3 style={{fontFamily: "'Courier New', Courier, monospace"}}>Curriculum</h3>
          <p style={{fontFamily: "'Georgia', 'Times New Roman', serif"}}>Physical AI & Humanoid Robotics</p>
        </div>
      </div>
    );
  };

  // Horizontal scrolling timeline
  const TimelineSection = () => {
    const timelineItems = [
      { week: "Weeks 1-2", topic: "Introduction to Physical AI", desc: "Foundations of Physical AI and embodied intelligence" },
      { week: "Weeks 3-5", topic: "ROS 2 Fundamentals", desc: "ROS 2 architecture and core concepts" },
      { week: "Weeks 6-7", topic: "Robot Simulation", desc: "Gazebo simulation environment setup" },
      { week: "Weeks 8-10", topic: "NVIDIA Isaac Platform", desc: "AI-powered perception and manipulation" },
      { week: "Weeks 11-12", topic: "Humanoid Development", desc: "Humanoid robot kinematics and dynamics" },
      { week: "Week 13", topic: "Conversational Robotics", desc: "Integrating GPT models for conversational AI" }
    ];

    return (
      <div className={styles.timelineContainer}>
        <div className={styles.timelineTrack}>
          {timelineItems.map((item, index) => (
            <motion.div
              key={index}
              className={styles.timelineItem}
              whileHover={{ scale: 1.05 }}
              initial={{ x: -100, opacity: 0 }}
              animate={{ x: 0, opacity: 1 }}
              transition={{ delay: index * 0.2 }}
            >
              <div className={styles.timelineBadge}>
                <span>{item.week}</span>
              </div>
              <h4 style={{fontFamily: "'Trebuchet MS', 'Lucida Sans Unicode', 'Lucida Grande', 'Lucida Sans', Arial, sans-serif"}}>{item.topic}</h4>
              <p style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>{item.desc}</p>
            </motion.div>
          ))}
        </div>
      </div>
    );
  };

  // Animated skills wheel
  const SkillsWheel = () => {
    const skills = [
      { icon: "🎯", skill: "Physical AI", level: 95 },
      { icon: "📡", skill: "ROS 2", level: 90 },
      { icon: "🎮", skill: "Simulation", level: 85 },
      { icon: "🤖", skill: "Isaac Platform", level: 92 },
      { icon: "🦾", skill: "Humanoid Design", level: 88 },
      { icon: "💬", skill: "Conversational AI", level: 87 }
    ];

    return (
      <div className={styles.skillsContainer}>
        {skills.map((skill, index) => (
          <motion.div
            key={index}
            className={styles.skillItem}
            initial={{ scale: 0, rotate: -180 }}
            animate={{ scale: 1, rotate: 0 }}
            transition={{ delay: index * 0.2, type: "spring", stiffness: 100 }}
            whileHover={{ scale: 1.1, rotate: 5 }}
          >
            <div className={styles.skillIcon}>{skill.icon}</div>
            <h4 style={{fontFamily: "'Arial Rounded MT Bold', 'Helvetica Rounded', Arial, sans-serif"}}>{skill.skill}</h4>
            <div className={styles.skillBar}>
              <div
                className={styles.skillBarFill}
                style={{ width: `${skill.level}%` }}
              ></div>
            </div>
            <span className={styles.skillLevel}>{skill.level}%</span>
          </motion.div>
        ))}
      </div>
    );
  };

  return (
    <Layout
      title={`PHYSICAL AI · HUMANOID ROBOTICS`}
      description="A comprehensive, end‑to‑end playbook for ROS2, Isaac Sim, digital twins, and embodied AI agents. Learn the control loops, math, and code that matter."
    >
      <main>
        {/* Hero Section */}
        <motion.section
          className={styles.heroSection}
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ duration: 0.8 }}
        >
          <div className="container">
            <div className={styles.heroContent}>
              <motion.div
                initial={{ y: 20, opacity: 0 }}
                animate={{ y: 0, opacity: 1 }}
                transition={{ delay: 0.2, duration: 0.6 }}
                className={styles.textContent}
              >
                <h1 className={styles.mainTitle} style={{fontFamily: "'Georgia', 'Times New Roman', serif"}}>
                  PHYSICAL AI · HUMANOID ROBOTICS
                </h1>
                <h2 className={styles.subTitle} style={{fontFamily: "'Trebuchet MS', 'Lucida Sans Unicode', 'Lucida Grande', 'Lucida Sans', Arial, sans-serif"}}>
                  Build the robotics OS
                  <br />
                  for your mind.
                </h2>
                <p className={styles.description} style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>
                  A comprehensive, end‑to‑end playbook for ROS2, Isaac Sim,
                  digital twins, and embodied AI agents. Learn the control
                  loops, math, and code that matter.
                </p>
                <div className={styles.buttons}>
                  <Link
                    className="button button--primary button--lg button--futuristic"
                    to="/docs/introduction/intro"
                  >
                    Start the curriculum
                  </Link>
                  {isAuthenticated ? (
                    <Link
                      className="button button--secondary button--lg button--futuristic"
                      to="/dashboard"
                    >
                      Dashboard
                    </Link>
                  ) : (
                    <Link
                      className="button button--secondary button--lg button--futuristic"
                      to="/"
                    >
                      Sign In / Up
                    </Link>
                  )}
                </div>
                <div className={styles.features}>
                  <div className={styles.featureItem}>
                    <span className={styles.featureIcon}>📚</span>
                    <span className={styles.featureText} style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>8+ modules</span>
                  </div>
                  <div className={styles.featureItem}>
                    <span className={styles.featureIcon}>🤖</span>
                    <span className={styles.featureText} style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>Live RAG tutor</span>
                  </div>
                  <div className={styles.featureItem}>
                    <span className={styles.featureIcon}>🚀</span>
                    <span className={styles.featureText} style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>
                      Hands-on projects
                    </span>
                  </div>
                </div>
              </motion.div>

              <motion.div
                initial={{ scale: 0.8, opacity: 0 }}
                animate={{ scale: 1, opacity: 1 }}
                transition={{ delay: 0.4, duration: 0.6 }}
                className={styles.illustration}
              >
                <div
                  style={{
                    width: "500px",
                    height: "500px",
                    display: "flex",
                    alignItems: "center",
                    justifyContent: "center",
                    position: "relative",
                  }}
                >
                  {/* Outer glow effect */}
                  <div
                    style={{
                      position: "absolute",
                      width: "450px",
                      height: "450px",
                      borderRadius: "50%",
                      background:
                        "radial-gradient(circle, rgba(0, 195, 255, 0.15) 0%, transparent 70%)",
                      animation: "glowPulse 4s infinite alternate",
                    }}
                  ></div>

                  <div
                    style={{
                      width: "400px",
                      height: "400px",
                      background:
                        "radial-gradient(circle at center, #1a1a2e 0%, #16213e 40%, #0f0f23 100%)",
                      borderRadius: "50%",
                      boxShadow:
                        "0 0 50px rgba(0, 195, 255, 0.6), inset 0 0 30px rgba(0, 0, 0, 0.8)",
                      position: "relative",
                      overflow: "hidden",
                      border: "1px solid rgba(0, 195, 255, 0.4)",
                    }}
                  >
                    {/* Orbital rings */}
                    <svg
                      width="100%"
                      height="100%"
                      viewBox="0 0 400 400"
                      style={{ position: "absolute", top: 0, left: 0 }}
                    >
                      {/* Inner orbital ring */}
                      <circle
                        cx="200"
                        cy="200"
                        r="60"
                        fill="none"
                        stroke="rgba(0, 195, 255, 0.2)"
                        strokeWidth="1"
                        style={{
                          animation: "rotateCW 15s linear infinite",
                        }}
                      />
                      {/* Middle orbital ring */}
                      <circle
                        cx="200"
                        cy="200"
                        r="90"
                        fill="none"
                        stroke="rgba(0, 195, 255, 0.15)"
                        strokeWidth="0.8"
                        style={{
                          animation: "rotateCCW 20s linear infinite",
                        }}
                      />
                      {/* Outer orbital ring */}
                      <circle
                        cx="200"
                        cy="200"
                        r="120"
                        fill="none"
                        stroke="rgba(0, 195, 255, 0.1)"
                        strokeWidth="0.6"
                        style={{
                          animation: "rotateCW 25s linear infinite",
                        }}
                      />

                      {/* Orbital particles - inner orbit */}
                      {[...Array(8)].map((_, i) => (
                        <circle
                          key={`inner-${i}`}
                          cx={200 + 60 * Math.cos((i * Math.PI) / 4)}
                          cy={200 + 60 * Math.sin((i * Math.PI) / 4)}
                          r="3"
                          fill="#ff8c00"
                          style={{
                            animation: "orbitInner 8s linear infinite",
                            animationDelay: `${i * 0.5}s`,
                          }}
                        />
                      ))}

                      {/* Orbital particles - middle orbit */}
                      {[...Array(12)].map((_, i) => (
                        <circle
                          key={`mid-${i}`}
                          cx={200 + 90 * Math.cos((i * Math.PI) / 6)}
                          cy={200 + 90 * Math.sin((i * Math.PI) / 6)}
                          r="2.5"
                          fill="#00ffff"
                          style={{
                            animation:
                              "orbitMiddle 12s linear infinite reverse",
                            animationDelay: `${i * 0.3}s`,
                          }}
                        />
                      ))}

                      {/* Orbital particles - outer orbit */}
                      {[...Array(16)].map((_, i) => (
                        <circle
                          key={`out-${i}`}
                          cx={200 + 120 * Math.cos((i * Math.PI) / 8)}
                          cy={200 + 120 * Math.sin((i * Math.PI) / 8)}
                          r="2"
                          fill="#ff8c00"
                          style={{
                            animation: "orbitOuter 16s linear infinite",
                            animationDelay: `${i * 0.25}s`,
                          }}
                        />
                      ))}
                    </svg>

                    {/* Animated robot SVG - Updated for Tesla Optimus Gen 2 style realism */}
                    <svg
                      viewBox="0 0 200 300"
                      width="100%"
                      height="100%"
                      style={{
                        position: "absolute",
                        top: 0,
                        left: 0,
                        filter: "drop-shadow(0 0 10px rgba(0, 200, 255, 0.7))",
                      }}
                    >
                      {/* Definitions for gradients and filters */}
                      <defs>
                        <linearGradient id="metallicBody" x1="0%" y1="0%" x2="100%" y2="100%">
                          <stop offset="0%" stopColor="#f0f0f0" />
                          <stop offset="50%" stopColor="#d0d0d0" />
                          <stop offset="100%" stopColor="#b0b0b0" />
                        </linearGradient>
                        <linearGradient id="powerCoreGradient" x1="0%" y1="0%" x2="100%" y2="100%">
                          <stop offset="0%" stopColor="#ff8c00" stopOpacity="1" />
                          <stop offset="100%" stopColor="#ff4500" stopOpacity="0.8" />
                        </linearGradient>
                        <filter id="metallicFilter">
                          <feSpecularLighting result="spec" specularExponent="40" lightingColor="#ffffff">
                            <fePointLight x="-500" y="-500" z="200" />
                          </feSpecularLighting>
                          <feComposite in="spec" in2="SourceAlpha" operator="in" result="spec" />
                          <feComposite in="SourceGraphic" in2="spec" operator="arithmetic" k1="0" k2="1" k3="0.8" k4="0" />
                        </filter>
                        <filter id="coreGlow">
                          <feGaussianBlur stdDeviation="2" result="blur" />
                          <feMerge>
                            <feMergeNode in="blur" />
                            <feMergeNode in="SourceGraphic" />
                          </feMerge>
                        </filter>
                        <filter id="motionBlurFilter">
                          <feGaussianBlur stdDeviation="1 0" />
                        </filter>
                      </defs>

                      {/* Head - Realistic Tesla Optimus style with camera array */}
                      <g filter="url(#metallicFilter)">
                        {/* Main head casing */}
                        <ellipse cx="100" cy="60" rx="15" ry="20" fill="url(#metallicBody)" stroke="#a0a0a0" strokeWidth="0.5" />
                        {/* Face visor */}
                        <rect x="90" y="55" width="20" height="10" rx="2" fill="#202020" opacity="0.8" />
                        {/* Eye cameras */}
                        <circle cx="95" cy="60" r="2" fill="#000000" stroke="#808080" strokeWidth="0.5" />
                        <circle cx="105" cy="60" r="2" fill="#000000" stroke="#808080" strokeWidth="0.5" />
                        <circle cx="95" cy="60" r="0.5" fill="#ffffff" opacity="0.5" />
                        <circle cx="105" cy="60" r="0.5" fill="#ffffff" opacity="0.5" />
                        {/* Sensor array */}
                        <line x1="92" y1="70" x2="108" y2="70" stroke="#808080" strokeWidth="0.5" />
                        <circle cx="100" cy="70" r="1" fill="#ff0000" opacity="0.4" />
                      </g>

                      {/* Neck - Mechanical connector */}
                      <rect x="97" y="80" width="6" height="10" fill="#c0c0c0" stroke="#a0a0a0" strokeWidth="0.5" />
                      <line x1="97" y1="80" x2="103" y2="80" stroke="#808080" strokeWidth="0.5" />
                      <line x1="97" y1="90" x2="103" y2="90" stroke="#808080" strokeWidth="0.5" />

                      {/* Torso - Tesla Optimus style with battery pack and actuators */}
                      <g filter="url(#metallicFilter)">
                        <rect x="85" y="90" width="30" height="80" rx="5" fill="url(#metallicBody)" stroke="#a0a0a0" strokeWidth="0.5" />
                        {/* Battery pack panels */}
                        <rect x="90" y="100" width="20" height="30" rx="2" fill="#e0e0e0" opacity="0.3" stroke="#b0b0b0" strokeWidth="0.5" />
                        <rect x="90" y="140" width="20" height="20" rx="2" fill="#e0e0e0" opacity="0.3" stroke="#b0b0b0" strokeWidth="0.5" />
                        {/* Actuator details */}
                        <line x1="85" y1="110" x2="115" y2="110" stroke="#c0c0c0" strokeWidth="0.5" opacity="0.6" />
                        <line x1="85" y1="150" x2="115" y2="150" stroke="#c0c0c0" strokeWidth="0.5" opacity="0.6" />
                      </g>

                      {/* Pulsing orange chest core */}
                      <g filter="url(#coreGlow)">
                        <circle
                          cx="100"
                          cy="120"
                          r="8"
                          fill="url(#powerCoreGradient)"
                          style={{
                            animation: "pulse 2s infinite alternate",
                          }}
                        />
                        <circle cx="100" cy="120" r="4" fill="#ffffff" opacity="0.7" />
                      </g>

                      {/* Arms - Realistic with elbow actuators */}
                      <g>
                        {/* Left arm - waving */}
                        <g
                          style={{
                            transformOrigin: "85px 95px",
                            animation: "wave 3s infinite alternate",
                            filter: "url(#motionBlurFilter)",
                          }}
                        >
                          {/* Upper arm with actuator */}
                          <rect x="80" y="95" width="10" height="45" rx="2" fill="url(#metallicBody)" />
                          <circle cx="85" cy="140" r="4" fill="#c0c0c0" stroke="#a0a0a0" strokeWidth="0.5" />
                          {/* Forearm */}
                          <rect x="80" y="144" width="10" height="45" rx="2" fill="url(#metallicBody)" />
                          {/* Hand with fingers */}
                          <g transform="translate(80,189)">
                            <rect x="0" y="0" width="10" height="8" rx="1" fill="url(#metallicBody)" />
                            <line x1="1" y1="8" x2="1" y2="16" stroke="gray" strokeWidth="1" />
                            <line x1="3" y1="8" x2="3" y2="18" stroke="gray" strokeWidth="1" />
                            <line x1="5" y1="8" x2="5" y2="17" stroke="gray" strokeWidth="1" />
                            <line x1="7" y1="8" x2="7" y2="18" stroke="gray" strokeWidth="1" />
                            <line x1="9" y1="8" x2="9" y2="16" stroke="gray" strokeWidth="1" />
                          </g>
                        </g>

                        {/* Right arm */}
                        <g
                          style={{
                            filter: "url(#motionBlurFilter)",
                          }}
                        >
                          {/* Upper arm with actuator */}
                          <rect x="110" y="95" width="10" height="45" rx="2" fill="url(#metallicBody)" />
                          <circle cx="115" cy="140" r="4" fill="#c0c0c0" stroke="#a0a0a0" strokeWidth="0.5" />
                          {/* Forearm */}
                          <rect x="110" y="144" width="10" height="45" rx="2" fill="url(#metallicBody)" />
                          {/* Hand with fingers */}
                          <g transform="translate(110,189)">
                            <rect x="0" y="0" width="10" height="8" rx="1" fill="url(#metallicBody)" />
                            <line x1="1" y1="8" x2="1" y2="16" stroke="gray" strokeWidth="1" />
                            <line x1="3" y1="8" x2="3" y2="18" stroke="gray" strokeWidth="1" />
                            <line x1="5" y1="8" x2="5" y2="17" stroke="gray" strokeWidth="1" />
                            <line x1="7" y1="8" x2="7" y2="18" stroke="gray" strokeWidth="1" />
                            <line x1="9" y1="8" x2="9" y2="16" stroke="gray" strokeWidth="1" />
                          </g>
                        </g>
                      </g>

                      {/* Legs - Realistic with knee actuators */}
                      <g>
                        {/* Left leg */}
                        <g>
                          {/* Upper leg with actuator */}
                          <rect x="85" y="170" width="10" height="45" rx="2" fill="url(#metallicBody)" />
                          <circle cx="90" cy="215" r="4" fill="#c0c0c0" stroke="#a0a0a0" strokeWidth="0.5" />
                          {/* Lower leg */}
                          <rect x="85" y="219" width="10" height="45" rx="2" fill="url(#metallicBody)" />
                          {/* Foot */}
                          <rect x="82" y="264" width="16" height="6" rx="2" fill="#a0a0a0" stroke="#808080" strokeWidth="0.5" />
                        </g>

                        {/* Right leg with subtle movement */}
                        <g
                          style={{
                            transformOrigin: "105px 170px",
                            animation: "shift 4s infinite alternate",
                            filter: "url(#motionBlurFilter)",
                          }}
                        >
                          {/* Upper leg with actuator */}
                          <rect x="105" y="170" width="10" height="45" rx="2" fill="url(#metallicBody)" />
                          <circle cx="110" cy="215" r="4" fill="#c0c0c0" stroke="#a0a0a0" strokeWidth="0.5" />
                          {/* Lower leg */}
                          <rect x="105" y="219" width="10" height="45" rx="2" fill="url(#metallicBody)" />
                          {/* Foot */}
                          <rect x="102" y="264" width="16" height="6" rx="2" fill="#a0a0a0" stroke="#808080" strokeWidth="0.5" />
                        </g>
                      </g>

                      {/* Head turning */}
                      <g
                        style={{
                          transformOrigin: "100px 60px",
                          animation: "turn 6s infinite alternate",
                          filter: "url(#motionBlurFilter)",
                        }}
                      >
                        {/* The head is already defined, but this group allows turning */}
                      </g>
                    </svg>

                    {/* Head turning animation effect using CSS */}
                    <div
                      style={{
                        position: "absolute",
                        top: "45px",
                        left: "75px",
                        width: "50px",
                        height: "50px",
                        animation: "turn 6s infinite alternate",
                        zIndex: 10,
                        filter: "url(#motionBlur)",
                      }}
                    ></div>

                    {/* SVG Filters */}
                    <svg style={{ position: "absolute", width: 0, height: 0 }}>
                      <filter id="metallic">
                        <feSpecularLighting
                          result="spec"
                          lightingColor="#ffffff"
                          specularExponent="20"
                        >
                          <feDistantLight azimuth="45" elevation="45" />
                        </feSpecularLighting>
                        <feComposite
                          in="SourceGraphic"
                          in2="spec"
                          operator="arithmetic"
                          k1="0"
                          k2="1"
                          k3="1"
                          k4="0"
                        />
                      </filter>
                      <filter id="motionBlur">
                        <feGaussianBlur stdDeviation="0.5" />
                      </filter>
                    </svg>
                  </div>

                  {/* CSS animations */}
                  <style>{`
                    @keyframes pulse {
                      0% { fill: #ff8c00; r: 8; filter: drop-shadow(0 0 8px #ff8c00); }
                      100% { fill: #ff6600; r: 10; filter: drop-shadow(0 0 12px #ff6600); }
                    }

                    @keyframes wave {
                      0% { transform: rotate(-20deg); }
                      100% { transform: rotate(20deg); }
                    }

                    @keyframes blink {
                      0%, 45%, 55%, 100% { opacity: 1; }
                      48%, 52% { opacity: 0; }
                    }

                    @keyframes shift {
                      0% { transform: translateY(0); }
                      100% { transform: translateY(-5px); }
                    }

                    @keyframes turn {
                      0% { transform: rotate(-10deg); }
                      100% { transform: rotate(10deg); }
                    }

                    @keyframes rotateCW {
                      from { transform: rotate(0deg); }
                      to { transform: rotate(360deg); }
                    }

                    @keyframes rotateCCW {
                      from { transform: rotate(0deg); }
                      to { transform: rotate(-360deg); }
                    }

                    @keyframes orbitInner {
                      from { transform: rotate(0deg) translateX(60px) rotate(0deg); }
                      to { transform: rotate(360deg) translateX(60px) rotate(-360deg); }
                    }

                    @keyframes orbitMiddle {
                      from { transform: rotate(0deg) translateX(90px) rotate(0deg); }
                      to { transform: rotate(-360deg) translateX(90px) rotate(360deg); }
                    }

                    @keyframes orbitOuter {
                      from { transform: rotate(0deg) translateX(120px) rotate(0deg); }
                      to { transform: rotate(360deg) translateX(120px) rotate(-360deg); }
                    }

                    @keyframes glowPulse {
                      0% { opacity: 0.3; }
                      100% { opacity: 0.6; }
                    }

                    @keyframes fingerMove0 {
                      0% { transform: rotate(-15deg); }
                      100% { transform: rotate(-5deg); }
                    }
                    @keyframes fingerMove1 {
                      0% { transform: rotate(5deg); }
                      100% { transform: rotate(15deg); }
                    }
                    @keyframes fingerMove3 {
                      0% { transform: rotate(-15deg); }
                      100% { transform: rotate(-5deg); }
                    }
                    @keyframes fingerMove4 {
                      0% { transform: rotate(15deg); }
                      100% { transform: rotate(5deg); }
                    }
                  `}</style>
                </div>
              </motion.div>
            </div>
          </div>
        </motion.section>


        {/* Circular Curriculum Section */}
        <motion.section
          className={styles.circularSection}
          initial={{ opacity: 0, scale: 0.8 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ delay: 0.8, duration: 1 }}
        >
          <div className="container">
            <div className={styles.sectionHeader}>
              <h2 className={styles.sectionTitle} style={{fontFamily: "'Courier New', Courier, monospace"}}>Interactive Curriculum</h2>
              <p className={styles.sectionSubtitle} style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>
                Explore our comprehensive learning modules in a circular format
              </p>
            </div>
            <div className={styles.circularContent}>
              <CircularCurriculum />
            </div>
          </div>
        </motion.section>

        {/* Skills Wheel Section */}
        <motion.section
          className={styles.skillsSection}
          initial={{ opacity: 0, y: 50 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 1, duration: 0.8 }}
        >
          <div className="container">
            <div className={styles.sectionHeader}>
              <h2 className={styles.sectionTitle} style={{fontFamily: "'Courier New', Courier, monospace"}}>Master These Skills</h2>
              <p className={styles.sectionSubtitle} style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>
                Comprehensive skill development for Physical AI
              </p>
            </div>
            <div className={styles.skillsContent}>
              <SkillsWheel />
            </div>
          </div>
        </motion.section>

        {/* Horizontal Timeline Section */}
        <motion.section
          className={styles.timelineSection}
          initial={{ opacity: 0, x: -100 }}
          animate={{ opacity: 1, x: 0 }}
          transition={{ delay: 1.2, duration: 0.8 }}
        >
          <div className="container">
            <div className={styles.sectionHeader}>
              <h2 className={styles.sectionTitle} style={{fontFamily: "'Courier New', Courier, monospace"}}>Course Timeline</h2>
              <p className={styles.sectionSubtitle} style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>
                13 weeks of intensive learning and development
              </p>
            </div>
            <div className={styles.timelineContent}>
              <TimelineSection />
            </div>
          </div>
        </motion.section>

        {/* Key Features Section */}
        <motion.section
          className={styles.featuresSection}
          initial={{ opacity: 0, y: 50 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 1.4, duration: 0.8 }}
        >
          <div className="container">
            <div className={styles.sectionHeader}>
              <h2 className={styles.sectionTitle} style={{fontFamily: "'Courier New', Courier, monospace"}}>Learning Features</h2>
              <p className={styles.sectionSubtitle} style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>
                Comprehensive approach to embodied AI education
              </p>
            </div>
            <div className={styles.featuresGrid}>
              <motion.div
                className={styles.featureCard}
                whileHover={{ y: -10 }}
                transition={{ duration: 0.3 }}
              >
                <div className={styles.featureIcon}>
                  <FuturisticRobot
                    size={60}
                    speed={1.0}
                    colors={{
                      body: "var(--ifm-color-primary)",
                      core: "#FFA500",
                      particles: "#FFA500",
                      outerParticles: "var(--ifm-color-primary)",
                    }}
                  />
                </div>
                <h3 style={{fontFamily: "'Trebuchet MS', 'Lucida Sans Unicode', 'Lucida Grande', 'Lucida Sans', Arial, sans-serif"}}>Hands-on Projects</h3>
                <p style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>
                  Build real robots and solve complex physical AI challenges
                </p>
              </motion.div>
              <motion.div
                className={styles.featureCard}
                whileHover={{ y: -10 }}
                transition={{ duration: 0.3 }}
              >
                <div className={styles.featureIcon}>
                  <FuturisticRobot
                    size={60}
                    speed={1.0}
                    colors={{
                      body: "var(--ifm-color-secondary)",
                      core: "#FFD700",
                      particles: "#FFD700",
                      outerParticles: "var(--ifm-color-secondary)",
                    }}
                  />
                </div>
                <h3 style={{fontFamily: "'Trebuchet MS', 'Lucida Sans Unicode', 'Lucida Grande', 'Lucida Sans', Arial, sans-serif"}}>AI Integration</h3>
                <p style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>Combine LLMs with robotics for conversational AI agents</p>
              </motion.div>
              <motion.div
                className={styles.featureCard}
                whileHover={{ y: -10 }}
                transition={{ duration: 0.3 }}
              >
                <div className={styles.featureIcon}>
                  <FuturisticRobot
                    size={60}
                    speed={1.0}
                    colors={{
                      body: "var(--ifm-color-success)",
                      core: "#FFA500",
                      particles: "#FFA500",
                      outerParticles: "var(--ifm-color-success)",
                    }}
                  />
                </div>
                <h3 style={{fontFamily: "'Trebuchet MS', 'Lucida Sans Unicode', 'Lucida Grande', 'Lucida Sans', Arial, sans-serif"}}>Industry Ready</h3>
                <p style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>Learn technologies used in real-world robotics companies</p>
              </motion.div>
            </div>
          </div>
        </motion.section>


        {/* Capstone Section */}
        <motion.section
          className={styles.capstoneSection}
          initial={{ opacity: 0, y: 50 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 1.8, duration: 0.8 }}
        >
          <div className="container">
            <div className={styles.capstoneContent}>
              <motion.div
                initial={{ x: -50, opacity: 0 }}
                animate={{ x: 0, opacity: 1 }}
                transition={{ delay: 2.2, duration: 0.8 }}
                className={styles.capstoneText}
              >
                <h2 className={styles.capstoneTitle} style={{fontFamily: "'Courier New', Courier, monospace"}}>Capstone Project</h2>
                <p className={styles.capstoneDescription} style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>
                  The Autonomous Humanoid: A final project where a simulated robot receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it. This project synthesizes all the knowledge gained throughout the program.
                </p>
                <div className={styles.capstoneFeatures}>
                  <div className={styles.capstoneFeature}>
                    <span className={styles.featureIcon}>🎤</span>
                    <span style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>Voice Command Processing</span>
                  </div>
                  <div className={styles.capstoneFeature}>
                    <span className={styles.featureIcon}>🗺️</span>
                    <span style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>Path Planning & Navigation</span>
                  </div>
                  <div className={styles.capstoneFeature}>
                    <span className={styles.featureIcon}>👁️</span>
                    <span style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>Computer Vision & Object Recognition</span>
                  </div>
                  <div className={styles.capstoneFeature}>
                    <span className={styles.featureIcon}>✋</span>
                    <span style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>Manipulation & Control</span>
                  </div>
                </div>
                <Link
                  className="button button--primary button--lg button--futuristic"
                  to="/docs/introduction/intro"
                >
                  Start Building
                </Link>
              </motion.div>
              <motion.div
                initial={{ x: 50, opacity: 0 }}
                animate={{ x: 0, opacity: 1 }}
                transition={{ delay: 2.4, duration: 0.8 }}
                className={styles.capstoneIllustration}
              >
                <FuturisticRobot
                  size={300}
                  speed={2.0}
                  colors={{
                    body: "var(--ifm-color-primary-lighter)",
                    core: "#FFD700",
                    particles: "#FFD700",
                    outerParticles: "var(--ifm-color-primary-lighter)",
                  }}
                />
              </motion.div>
            </div>
          </div>
        </motion.section>

        {/* Call to Action Section */}
        <motion.section
          className={styles.ctaSection}
          initial={{ opacity: 0, y: 50 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 2.4, duration: 0.8 }}
        >
          <div className="container">
            <div className={styles.ctaContent}>
              <motion.div
                initial={{ y: 20, opacity: 0 }}
                animate={{ y: 0, opacity: 1 }}
                transition={{ delay: 2.6, duration: 0.6 }}
                className={styles.ctaText}
              >
                <h2 className={styles.ctaTitle} style={{fontFamily: "'Courier New', Courier, monospace"}}>Join the Future of Robotics Education</h2>
                <p className={styles.ctaDescription} style={{fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif"}}>
                  Be part of the next generation of AI and robotics professionals. Learn from industry experts and build the technologies that will shape our future.
                </p>
                <div className={styles.ctaButtons}>
                  <Link
                    className="button button--primary button--lg button--futuristic"
                    to="/docs/introduction/intro"
                  >
                    Start Learning
                  </Link>
                  <Link
                    className="button button--secondary button--lg button--futuristic"
                    to="https://github.com"
                  >
                    View Repository
                  </Link>
                </div>
              </motion.div>
            </div>
          </div>
        </motion.section>
      </main>
    </Layout>
  );
}
