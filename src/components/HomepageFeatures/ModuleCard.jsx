ModuleCard.jsx
import React from 'react';
import Link from '@docusaurus/Link';
import { SparklesIcon } from '@heroicons/react/24/outline';


export default function ModuleCard({ title, desc, icon, slug }) {
return (
<div className="module-card">
<div className="module-card-icon">{icon || <SparklesIcon />}</div>
<h3 className="module-card-title">{title}</h3>
<p className="module-card-desc">{desc}</p>
<Link className="module-card-link" to={`/Modules/${slug}`}>
Explore →
</Link>
</div>
);
}