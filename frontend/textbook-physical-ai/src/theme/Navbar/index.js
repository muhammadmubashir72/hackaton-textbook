// This custom Navbar component was causing conflicts with the Docusaurus default navbar.
// The navbar is properly configured in docusaurus.config.js, so we'll use the default implementation.
// This file exists to prevent Docusaurus from throwing an error about a missing override,
// but it simply exports the original Navbar component.

import OriginalNavbar from '@theme-original/Navbar';

export default function Navbar(props) {
  return <OriginalNavbar {...props} />;
}