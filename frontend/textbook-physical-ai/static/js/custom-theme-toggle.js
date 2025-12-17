// Optimized theme toggle script
document.addEventListener('DOMContentLoaded', function() {
  // Initialize theme toggle mount point
  const mountPoint = document.getElementById('custom-theme-toggle-mount');
  if (!mountPoint) return;

  // Create theme toggle button with optimized code
  const themeToggle = document.createElement('button');
  themeToggle.className = 'theme-toggle';
  themeToggle.type = 'button';
  themeToggle.setAttribute('aria-label', 'Toggle theme');
  themeToggle.innerHTML = `
    <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
      <path id="theme-icon-path" d="M12 16C14.2091 16 16 14.2091 16 12C16 9.79086 14.2091 8 12 8C9.79086 8 8 9.79086 8 12C8 14.2091 9.79086 16 12 16Z" stroke="currentColor" stroke-width="2"/>
      <path id="theme-icon-sun-moon" d="M12 2V4M12 20V22M5.63563 5.63563L7.05 7.05M16.95 16.95L18.3644 18.3644M2 12H4M20 12H22M4.94975 19.0503L6.36396 17.636L7.77817 16.2218M16.2218 7.77817L17.636 6.36396L19.0503 4.94975" stroke="currentColor" stroke-width="2"/>
    </svg>
  `;
  
  // Add click handler with performance optimization
  let clicked = false;
  themeToggle.addEventListener('click', function() {
    if (clicked) return; // Debounce clicks
    
    clicked = true;
    setTimeout(() => clicked = false, 300); // Reset debounce
    
    const isDark = document.documentElement.getAttribute('data-theme') === 'dark';
    const newTheme = isDark ? 'light' : 'dark';
    
    // Update theme attribute
    document.documentElement.setAttribute('data-theme', newTheme);
    
    // Update icon
    const pathEl = document.getElementById('theme-icon-path');
    const sunMoonPaths = document.querySelectorAll('#theme-icon-sun-moon');
    
    if (newTheme === 'light') {
      pathEl.setAttribute('d', 'M12 16C14.2091 16 16 14.2091 16 12C16 9.79086 14.2091 8 12 8C9.79086 8 8 9.79086 8 12C8 14.2091 9.79086 16 12 16Z');
    } else {
      pathEl.setAttribute('d', 'M20.354 15.354C19.317 15.771 18.185 16.03 17 16.03C11.778 16.03 7.53 11.782 7.53 6.56C7.53 5.375 7.789 4.243 8.206 3.206C5.89 4.453 4.379 7.088 4.379 10.12C4.379 15.342 8.627 19.59 13.849 19.59C16.881 19.59 19.516 18.079 20.763 15.763L20.354 15.354Z');
    }
    
    // Store preference in localStorage
    localStorage.setItem('theme', newTheme);
  });
  
  mountPoint.appendChild(themeToggle);

  // Check for saved theme preference and apply it
  const savedTheme = localStorage.getItem('theme');
  const prefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
  
  if (savedTheme) {
    document.documentElement.setAttribute('data-theme', savedTheme);
  } else if (prefersDark) {
    document.documentElement.setAttribute('data-theme', 'dark');
  }
});