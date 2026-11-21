import type { CSSProperties } from 'react';
import type { BlockAppearance } from '../shared/model.ts';

export function defaultAppearance(kind: string, tone?: string): Required<BlockAppearance> {
  if (kind === 'group') return { background: '#192324', header: '#253535', border: '#64717e' };
  if (kind === 'annotation') {
    if (tone === 'teal') return { background: '#1c3233', header: '#254445', border: '#497b75' };
    if (tone === 'slate') return { background: '#222d39', header: '#2a3b4c', border: '#50687d' };
    return { background: '#2b2b23', header: '#39362a', border: '#7f6f4d' };
  }
  return {
    background: '#222c36',
    header: kind.startsWith('boundary-') ? '#24383c' : '#222c36',
    border: '#526172',
  };
}

function readableText(hex: string) {
  const channels = [1, 3, 5].map(start => {
    const value = Number.parseInt(hex.slice(start, start + 2), 16) / 255;
    return value <= 0.04045 ? value / 12.92 : ((value + 0.055) / 1.055) ** 2.4;
  });
  const luminance = channels[0] * 0.2126 + channels[1] * 0.7152 + channels[2] * 0.0722;
  return luminance > 0.179 ? '#000000' : '#ffffff';
}

export function appearanceStyle(appearance?: BlockAppearance): CSSProperties | undefined {
  if (!appearance) return undefined;
  const { background, header, border } = appearance;
  return {
    '--block-bg': background,
    '--block-fg': background && readableText(background),
    '--block-header': header,
    '--block-header-fg': header && readableText(header),
    '--block-border': border,
  } as CSSProperties;
}

export function appearanceAttributes(appearance?: BlockAppearance) {
  return {
    style: appearanceStyle(appearance),
    'data-custom-background': appearance?.background || undefined,
    'data-custom-header': appearance?.header || undefined,
    'data-custom-border': appearance?.border || undefined,
  };
}
