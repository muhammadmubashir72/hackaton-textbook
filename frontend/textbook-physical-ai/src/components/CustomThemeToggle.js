// Custom theme toggle script to cycle between system preference and opposite mode with 2 clicks
document.addEventListener('DOMContentLoaded', function() {
  // Create the custom theme toggle button
  const themeToggleContainer = document.querySelector('.navbar__item.html');
  if (!themeToggleContainer || !themeToggleContainer.innerHTML.includes('custom-theme-toggle')) {
    // Look for the element with id="custom-theme-toggle"
    const mountPoint = document.getElementById('custom-theme-toggle');
    if (mountPoint) {
      // Create the button element with the custom functionality
      const button = document.createElement('button');
      button.className = 'navbar__item navbar__link theme-toggle-custom';
      button.ariaLabel = 'Toggle between system theme and opposite theme';
      button.title = 'Toggle between system theme and opposite theme';
      button.style.fontSize = '1.2rem';
      button.style.display = 'flex';
      button.style.alignItems = 'center';
      button.style.justifyContent = 'center';
      button.style.width = '40px';
      button.style.height = '40px';
      button.style.border = 'none';
      button.style.backgroundColor = 'transparent';
      button.style.cursor = 'pointer';
      button.style.marginLeft = '0.5rem';

      // Determine initial state based on current theme
      const currentTheme = document.documentElement.getAttribute('data-theme');
      const systemPref = window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';

      // Set initial icon based on current state
      if (currentTheme === systemPref || currentTheme === 'auto' || currentTheme === null || currentTheme === '') {
        // Currently in system mode
        button.innerHTML = systemPref === 'dark' ? '🌙' : '☀️';
        window.currentToggleState = 0; // 0: system, 1: opposite
      } else {
        // Currently in opposite mode
        button.innerHTML = systemPref === 'dark' ? '☀️' : '🌙';
        window.currentToggleState = 1;
      }

      button.onclick = () => {
        const systemPref = window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
        const oppositeMode = systemPref === 'light' ? 'dark' : 'light';

        if (window.currentToggleState === 0) {
          // Switch to opposite mode
          document.documentElement.setAttribute('data-theme', oppositeMode);
          button.innerHTML = oppositeMode === 'dark' ? '🌙' : '☀️';
          window.currentToggleState = 1;
        } else {
          // Switch back to system mode
          document.documentElement.setAttribute('data-theme', systemPref);
          button.innerHTML = systemPref === 'dark' ? '🌙' : '☀️';
          window.currentToggleState = 0;
        }

        // Store preference in localStorage
        localStorage.setItem('theme-preference', window.currentToggleState === 0 ? systemPref : oppositeMode);
      };

      mountPoint.appendChild(button);

      // Also add a global click handler to restore the theme toggle if it gets removed
      document.addEventListener('navToggle', function() {
        if (!document.querySelector('.theme-toggle-custom')) {
          mountPoint.appendChild(button);
        }
      });
    }
  }
});