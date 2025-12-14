import React from 'react';
import Layout from '@theme/Layout';
import HeroSection from '../components/HomepageFeatures/HeroSection';
import ModuleOverview from '../components/HomepageFeatures/ModuleOverview';
import CapstoneHighlight from '../components/HomepageFeatures/CapstoneHighlight';
import PersonalizationPanel from '../components/HomepageFeatures/PersonalizationPanel';
import '../css/custom.css';


export default function Home() {
return (
<Layout title="Physical AI & Humanoid Robotics" description="Book: Physical AI & Humanoid Robotics">
<main className="homepage-root">
<HeroSection />


<section className="container section module-overview-section">
<ModuleOverview />
</section>


<section className="container section capstone-section">
<CapstoneHighlight />
</section>


<section className="container section personalization-section">
<PersonalizationPanel />
</section>
</main>
</Layout>
);
}