import { useEffect, useId, useRef, useState } from 'react';
import { APPEARANCE_FIELDS, HEX_COLOR } from '../shared/model.ts';
import type { AppearancePatch, BlockAppearance } from '../shared/model.ts';
import { appearanceStyle } from './blockAppearance.ts';

const labels = { background: '背景色', header: '标题栏色', border: '边框色' };
const presets = [
  { name: '灰蓝', background: '#253243', header: '#344760', border: '#6883a3' },
  { name: '海蓝', background: '#182f4d', header: '#264d77', border: '#6498c8' },
  { name: '青绿', background: '#193835', header: '#23594f', border: '#5cb3a2' },
  { name: '紫罗兰', background: '#302540', header: '#4d3867', border: '#a082cd' },
  { name: '暖黄', background: '#3b3222', header: '#5b482b', border: '#bd9855' },
  { name: '砖红', background: '#422a30', header: '#603742', border: '#ca8591' },
];

type Props = {
  count: number;
  label: string;
  initial: BlockAppearance[];
  defaults: Required<BlockAppearance>;
  error: string;
  busy: boolean;
  onClose: () => void;
  onApply: (appearance: AppearancePatch) => void;
};

export default function AppearanceDialog({count,label,initial,defaults,error,busy,onClose,onApply}: Props) {
  const dialog = useRef<HTMLDialogElement>(null);
  const titleId = useId();
  const [draft, setDraft] = useState<Partial<Record<keyof BlockAppearance, string>>>({});
  useEffect(() => {
    const element = dialog.current!;
    element.showModal();
    return () => element.close();
  }, []);

  const preview: Required<BlockAppearance> = { ...defaults };
  for (const field of APPEARANCE_FIELDS) {
    const value = draft[field]?.trim();
    if (HEX_COLOR.test(value || '')) preview[field] = value!;
    else if (value === undefined && initial[0]?.[field]) preview[field] = initial[0][field]!;
  }
  const invalid = Object.values(draft).some(value => value.trim() && !HEX_COLOR.test(value.trim()));
  const hasOverrides = initial.some(value => Object.keys(value).length > 0);

  function apply() {
    const changes: Exclude<AppearancePatch, null> = {};
    for (const field of APPEARANCE_FIELDS) {
      const value = draft[field]?.trim();
      if (value !== undefined) changes[field] = value || null;
    }
    onApply(changes);
  }

  return (
    <dialog ref={dialog} className="appearance-dialog" aria-labelledby={titleId}
      onCancel={event => { event.preventDefault(); if (!busy) onClose(); }}>
      <form onSubmit={event => { event.preventDefault(); if (!invalid && !busy) apply(); }}>
        <div className="appearance-heading"><h2 id={titleId}>外观</h2><span>{count > 1 ? `已选 ${count} 个块` : label}</span></div>
        <div className="appearance-preview" style={appearanceStyle(preview)}>
          <strong>标题栏预览</strong>
          <span>正文颜色随背景调整</span>
        </div>
        <div className="appearance-presets" aria-label="配色预设">
          {presets.map(({name, ...colors}) => (
            <button type="button" className="button" key={name} aria-label={`预设 ${name}`}
              onClick={() => setDraft(colors)} disabled={busy}>
              <i style={{backgroundColor:colors.header,borderColor:colors.border}}/>{name}
            </button>
          ))}
        </div>
        {APPEARANCE_FIELDS.map(field => {
          const original = initial[0]?.[field] || '';
          const mixed = initial.some(value => (value[field] || '') !== original);
          return (
            <div className="appearance-field" key={field}>
              <span>{labels[field]}</span>
              <input type="color" aria-label={`${labels[field]}取色器`} value={preview[field]}
                onChange={event => setDraft({...draft,[field]:event.target.value})} disabled={busy}/>
              <input type="text" aria-label={labels[field]} spellCheck={false}
                value={draft[field] ?? (mixed ? '' : original)} placeholder={mixed ? '混合颜色' : '默认'}
                onChange={event => setDraft({...draft,[field]:event.target.value})} disabled={busy}/>
              <button type="button" className="button" aria-label={`恢复${labels[field]}`}
                disabled={busy} onClick={() => setDraft({...draft,[field]:''})}>默认</button>
            </div>
          );
        })}
        {count > 1 && <p className="appearance-hint">仅应用此处改动的颜色，预览取第一个块。</p>}
        {invalid && <p className="appearance-error" role="alert">请输入 #RRGGBB，如 #23594F。</p>}
        {error && <p className="appearance-error" role="alert">{error}</p>}
        <div className="appearance-actions">
          <button type="button" className="button" onClick={() => onApply(null)} disabled={busy || !hasOverrides}>恢复全部默认</button>
          <button type="button" className="button" onClick={onClose} disabled={busy}>取消</button>
          <button type="submit" className="button primary" disabled={busy || invalid || !Object.keys(draft).length}>应用配色</button>
        </div>
      </form>
    </dialog>
  );
}
