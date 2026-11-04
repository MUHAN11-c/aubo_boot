/* Page selection changes presentation only. A running ROS operation is never
 * cancelled, resumed or repeated by navigating between tool panels. */
(() => {
  const panels = [...document.querySelectorAll('main > section[id]')];
  const links = [...document.querySelectorAll('.rail nav a[href]')];
  const connection = document.getElementById('connection-panel');
  const toggle = document.getElementById('connection-toggle');
  const route = () => {
    const requested = location.hash.slice(1);
    const active = panels.find(panel => panel.id === requested) ?? panels[0];
    for (const panel of panels) panel.hidden = panel !== active;
    for (const link of links) {
      if (link.hash === `#${active.id}`) link.setAttribute('aria-current', 'page');
      else link.removeAttribute('aria-current');
    }
    document.body.dataset.page = active.id;
    document.getElementById('page-title').textContent =
      links.find(link => link.hash === `#${active.id}`)?.dataset.title ?? '项目验收';
    window.scrollTo(0, 0);
  };
  const setConnectionOpen = open => {
    connection.hidden = !open;
    toggle.setAttribute('aria-expanded', String(open));
  };
  toggle.addEventListener('click', () => setConnectionOpen(connection.hidden));
  document.getElementById('connection-close').addEventListener('click', () => {
    setConnectionOpen(false);
    toggle.focus();
  });
  connection.addEventListener('keydown', event => {
    if (event.key === 'Escape') {
      setConnectionOpen(false);
      toggle.focus();
    }
  });
  document.getElementById('skip-main').addEventListener('click', event => {
    event.preventDefault();
    document.getElementById('main').focus();
  });
  window.addEventListener('hashchange', route);
  route();
})();
