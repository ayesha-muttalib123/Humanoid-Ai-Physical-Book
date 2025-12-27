import React from 'react';
import {Redirect} from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function NotFound() {
  const {siteConfig} = useDocusaurusContext();
  const introDoc = '/intro'; // Adjust this path to match your intro document
  
  return <Redirect to={introDoc} />;
}

export default NotFound;