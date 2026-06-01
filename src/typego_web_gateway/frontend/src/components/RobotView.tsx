import { useEffect, useState } from 'react';
import type { CameraTopics } from '../lib/api';
import { api } from '../lib/api';

export default function RobotView() {
  const [offline, setOffline] = useState(false);
  // Bump to force the <img> to re-request the MJPEG stream on error.
  const [epoch, setEpoch] = useState(0);
  const [topics, setTopics] = useState<CameraTopics | null>(null);

  useEffect(() => {
    let alive = true;
    const tick = async () => {
      try {
        const t = await api.cameraTopics();
        if (alive) setTopics(t);
      } catch {
        if (alive) setTopics(null);
      }
    };
    tick();
    const t = setInterval(tick, 5000);
    return () => {
      alive = false;
      clearInterval(t);
    };
  }, []);

  // Retry the stream a few seconds after it errors out.
  useEffect(() => {
    if (!offline) return;
    const t = setTimeout(() => {
      setOffline(false);
      setEpoch(e => e + 1);
    }, 3000);
    return () => clearTimeout(t);
  }, [offline]);

  return (
    <div className="camera-pane">
      <h2>Robot Camera</h2>
      <div className="camera-frame">
        {offline ? (
          <div className="muted camera-offline">camera offline</div>
        ) : (
          <img
            src={`${api.cameraStreamUrl()}?e=${epoch}`}
            alt="robot camera"
            onError={() => setOffline(true)}
          />
        )}
      </div>
      <div className="muted" style={{ marginTop: 6, fontSize: 11 }}>
        {topics
          ? `${topics.topic}${topics.frame_id ? ` · ${topics.frame_id}` : ''}`
          : 'camera topic unknown'}
      </div>
    </div>
  );
}
