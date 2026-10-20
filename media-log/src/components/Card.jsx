import { STATUS_LABEL, formatProgress, progressFraction } from '../lib/media.js';
import { Thumb } from './Icons.jsx';

export default function Card({ entry, onOpen, onIncrement }) {
  const frac = progressFraction(entry);
  const canBump = entry.status === 'in_progress' && entry.unit !== 'percent' && (!entry.total || entry.progress < entry.total);
  return (
    <article className="card">
      <button className="card-main" onClick={() => onOpen(entry)} aria-label={`Edit ${entry.title}`}>
        <div className="cover">
          {entry.cover_url ? <img src={entry.cover_url} alt="" loading="lazy" referrerPolicy="no-referrer" /> : <span className="cover-fallback" aria-hidden="true">{entry.title.slice(0, 1).toUpperCase()}</span>}
          {entry.rating && <span className={`badge badge-${entry.rating}`} title={`Rated: ${entry.rating}`}><Thumb kind={entry.rating} size={14} /></span>}
        </div>
        <h3 className="card-title">{entry.title}</h3>
        <p className="card-meta">{STATUS_LABEL[entry.status]}{entry.year ? ` · ${entry.year}` : ''}</p>
        <p className="card-progress">{formatProgress(entry)}</p>
        {frac !== null && <div className="bar" role="progressbar" aria-valuenow={Math.round(frac * 100)} aria-valuemin="0" aria-valuemax="100"><span style={{ width: `${frac * 100}%` }} /></div>}
      </button>
      {canBump && (
        <button className="bump" onClick={() => onIncrement(entry)} aria-label={`Add one to ${entry.title}`}>+1</button>
      )}
    </article>
  );
}
