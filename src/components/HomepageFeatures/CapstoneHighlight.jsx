import Link from '@docusaurus/Link';
import React from 'react';


export default function CapstoneHighlight() {
const capstoneVideo = '/media/capstone-demo.mp4'; // replace with actual path


return (
<Link href='Capstone-Project'>
<div className="capstone-highlight">
<div className="capstone-content">
<div className="capstone-text">
<h2>Capstone: Autonomous Humanoid Performing Tasks</h2>
<p>Project Objective: Develop an end-to-end VLA pipeline that allows a simulated humanoid robot to interpret high-level natural language commands, perceive its environment, plan a sequence of actions, and execute those actions to complete a given task in NVIDIA Isaac Sim.</p>
</div>
</div>
</div> </Link>
);
}