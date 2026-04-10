export function buildLiveEventsUrl(): string {
  const rawBase = (import.meta.env.VITE_API_BASE_URL ?? '').trim().replace(/\/$/, '');
  if (rawBase.startsWith('http://') || rawBase.startsWith('https://')) {
    const url = new URL(rawBase);
    url.protocol = url.protocol === 'https:' ? 'wss:' : 'ws:';
    url.pathname = `${url.pathname.replace(/\/$/, '')}/api/live/events`;
    url.search = '';
    return url.toString();
  }

  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const basePath = rawBase.startsWith('/') ? rawBase : '';
  return `${protocol}//${window.location.host}${basePath}/api/live/events`;
}
