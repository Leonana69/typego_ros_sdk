import { useState } from 'react';
import { api } from '../lib/api';

interface Props {
  bagCount: number | null;
}

export default function BagDownload({ bagCount }: Props) {
  const [minutes, setMinutes] = useState('60');
  const href = api.bagDownloadHref(parseInt(minutes, 10) || 60);

  return (
    <div className="panel">
      <h2>Bag Download</h2>
      <div className="row">
        <label>Minutes</label>
        <input type="number" min="1" max="1440" step="1" value={minutes}
               onChange={e => setMinutes(e.target.value)} />
      </div>
      <div className="row">
        <a href={href} download>
          <button>Download last {minutes} min</button>
        </a>
      </div>
      <div className="muted" style={{ marginTop: 4 }}>
        {bagCount === null
          ? 'bag rotator disabled'
          : `${bagCount} chunk(s) on disk`}
      </div>
    </div>
  );
}
